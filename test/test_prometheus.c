/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_prometheus.c — unit tests for imud-prometheus's two pure pieces
 *
 * src/prom_metrics.c: prom_build_metrics() driven with crafted imud_data_t
 * views — metric names, SI values, flag gauges, HELP/TYPE self-description,
 * truncation handling.
 *
 * src/prom_http.c: the scrape connection state machine, over a socketpair.
 * `now_ms` is an argument to every entry point precisely so these tests can
 * step past a deadline instead of sleeping through one.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>

#include "../include/prom_metrics.h"
#include "../include/prom_http.h"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* Parse "name value\n" out of the page; returns 1 and sets *v on match. */
static int metric(const char *page, const char *name, double *v)
{
    size_t nl = strlen(name);
    for (const char *p = page; (p = strstr(p, name)) != NULL; p++) {
        /* must start a line and be followed by a space (not a longer name) */
        if ((p != page && p[-1] != '\n') || p[nl] != ' ')
            continue;
        *v = strtod(p + nl + 1, NULL);
        return 1;
    }
    return 0;
}

static imud_data_t make_data(void)
{
    imud_data_t d;
    memset(&d, 0, sizeof d);
    d.heading_deg      = 90.0f;
    d.heading_true_deg = 103.2f;
    d.roll             = 0.10f;
    d.pitch            = -0.05f;
    d.yaw              = 1.57f;
    d.rate_of_turn     = 60.0f;      /* deg/min */
    d.heave_m          = 0.42f;
    d.heave_rate       = 0.25f;
    d.wave_height_m    = 1.6f;
    d.wave_period_s    = 6.2f;
    d.roll_period_s    = 4.4f;
    d.roll_amplitude   = 0.12f;
    d.pitch_period_s   = 5.1f;
    d.pitch_amplitude  = 0.06f;
    d.mag_anomaly      = 0.03f;
    d.mag_residual     = 0.02f;
    d.innov_weight     = 0.87f;
    d.innov_reject     = 0.06f;
    d.nis_accel        = 7.25f;
    d.nis_mag          = 1.10f;
    d.accel_quiescence = 0.001f;
    d.gyro_bias[1]     = -0.002f;
    d.temp_c           = 31.4f;
    return d;
}

static void test_no_packet(void)
{
    begin("test_no_packet");
    int fb = g_fail;

    char buf[4096];
    double v;
    int n = prom_build_metrics(buf, sizeof buf, NULL, 5);
    EXPECT(n > 0, "builds without a packet");
    EXPECT(metric(buf, "imud_up", &v) && v == 0.0, "imud_up 0 without a packet");
    EXPECT(metric(buf, "imud_packets_total", &v) && v == 5.0, "packets_total emitted");
    EXPECT(!metric(buf, "imud_heading_degrees", &v), "no data gauges without a packet");
    end(fb);
}

static void test_values_si(void)
{
    begin("test_values_si");
    int fb = g_fail;

    imud_data_t d = make_data();
    d.flags = IMUD_FLAG_DECLINATION_VALID;
    char buf[8192];
    double v;
    int n = prom_build_metrics(buf, sizeof buf, &d, 100);
    EXPECT(n > 0, "builds with a packet");
    EXPECT(metric(buf, "imud_up", &v) && v == 1.0, "imud_up 1");
    EXPECT(metric(buf, "imud_heading_degrees", &v) && fabs(v - 90.0) < 1e-3,
           "heading in degrees");
    EXPECT(metric(buf, "imud_heading_true_degrees", &v) && fabs(v - 103.2) < 1e-3,
           "true heading when declination valid");
    EXPECT(metric(buf, "imud_roll_radians", &v) && fabs(v - 0.10) < 1e-5,
           "roll in radians (SI)");
    /* 60 deg/min = 1 deg/s = 0.017453 rad/s */
    EXPECT(metric(buf, "imud_rate_of_turn_radians_per_second", &v) &&
           fabs(v - 0.0174533) < 1e-5, "rate of turn converted to rad/s");
    EXPECT(metric(buf, "imud_wave_height_meters", &v) && fabs(v - 1.6) < 1e-3,
           "wave height m");
    EXPECT(metric(buf, "imud_roll_amplitude_radians", &v) && fabs(v - 0.12) < 1e-4,
           "roll amplitude rad");
    EXPECT(metric(buf, "imud_pitch_period_seconds", &v) && fabs(v - 5.1) < 1e-2,
           "pitch period s");
    EXPECT(metric(buf, "imud_mag_anomaly_ratio", &v) && fabs(v - 0.03) < 1e-4,
           "mag anomaly");
    EXPECT(metric(buf, "imud_innov_weight_ratio", &v) && fabs(v - 0.87) < 1e-4,
           "innov_weight always on");
    EXPECT(metric(buf, "imud_innov_reject_ratio", &v) && fabs(v - 0.06) < 1e-4,
           "innov_reject always on");
    EXPECT(metric(buf, "imud_nis_accel_ratio", &v) && fabs(v - 7.25) < 1e-4,
           "nis_accel always on");
    EXPECT(metric(buf, "imud_nis_mag_ratio", &v) && fabs(v - 1.10) < 1e-4,
           "nis_mag always on");
    EXPECT(metric(buf, "imud_mag_residual_radians", &v) && fabs(v - 0.02) < 1e-4,
           "mag residual rad");
    EXPECT(metric(buf, "imud_gyro_bias_y_radians_per_second", &v) &&
           fabs(v + 0.002) < 1e-5, "gyro bias y");
    EXPECT(metric(buf, "imud_temperature_celsius", &v) && fabs(v - 31.4) < 1e-2,
           "temperature C");
    end(fb);
}

static void test_flags_and_gating(void)
{
    begin("test_flags_and_gating");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[8192];
    double v;

    /* No declination → true heading omitted entirely. */
    d.flags = 0;
    prom_build_metrics(buf, sizeof buf, &d, 1);
    EXPECT(!metric(buf, "imud_heading_true_degrees", &v),
           "true heading omitted without declination");
    EXPECT(metric(buf, "imud_engine_on", &v) && v == 0.0, "engine_on 0");
    EXPECT(metric(buf, "imud_wave_valid", &v) && v == 0.0, "wave_valid 0");

    d.flags = IMUD_FLAG_ENGINE_ON | IMUD_FLAG_WAVE_VALID | IMUD_FLAG_HEAVE_VALID
            | IMUD_FLAG_MAG_VALID | IMUD_FLAG_FUSION_CONVERGED;
    prom_build_metrics(buf, sizeof buf, &d, 1);
    EXPECT(metric(buf, "imud_engine_on", &v)   && v == 1.0, "engine_on 1");
    EXPECT(metric(buf, "imud_wave_valid", &v)  && v == 1.0, "wave_valid 1");
    EXPECT(metric(buf, "imud_heave_valid", &v) && v == 1.0, "heave_valid 1");
    EXPECT(metric(buf, "imud_mag_valid", &v)   && v == 1.0, "mag_valid 1");
    EXPECT(metric(buf, "imud_converged", &v)   && v == 1.0, "converged 1");
    end(fb);
}

static void test_help_type(void)
{
    begin("test_help_type");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[8192];
    prom_build_metrics(buf, sizeof buf, &d, 1);
    EXPECT(strstr(buf, "# HELP imud_wave_height_meters ") != NULL, "HELP present");
    EXPECT(strstr(buf, "# TYPE imud_wave_height_meters gauge") != NULL, "TYPE gauge");
    EXPECT(strstr(buf, "# TYPE imud_packets_total counter") != NULL, "TYPE counter");
    end(fb);
}

static void test_buffer_too_small(void)
{
    begin("test_buffer_too_small");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[64];
    EXPECT(prom_build_metrics(buf, sizeof buf, &d, 1) == -1,
           "returns -1 on truncation");
    EXPECT(prom_build_metrics(NULL, 0, &d, 1) == -1, "returns -1 on NULL buf");
    end(fb);
}

/* ── src/prom_http.c — the scrape connection ─────────────────────────────── */

/* A connected pair standing in for accept()'s result and the scraper. */
static int pair_open(int *srv, int *cli)
{
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0) return -1;
    *srv = sv[0];
    *cli = sv[1];
    return 0;
}

static bool fd_is_open(int fd) { return fcntl(fd, F_GETFD) >= 0; }

/* Write a whole string; true on success. Length comes from strlen rather
 * than a hand-counted literal — miscounting one is how this test first
 * "failed". */
static bool send_str(int fd, const char *s)
{
    size_t n = strlen(s);
    return write(fd, s, n) == (ssize_t)n;
}

static void test_http_request_shapes(void)
{
    begin("test_http_request_shapes");
    int fb = g_fail;

    prom_conn_t c;
    prom_conn_init(&c);
    EXPECT(!prom_conn_busy(&c), "starts idle");
    EXPECT(prom_conn_service(&c, 0) == 0, "service on an idle conn is a no-op");
    EXPECT(prom_conn_timeout_ms(&c, 0) == -1, "idle imposes no poll timeout");

    /* Whole request in one write. */
    int srv, cli;
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 1000, 2000) == 0, "adopt");
    EXPECT(prom_conn_busy(&c), "busy after adopt");
    EXPECT(send_str(cli, "GET /metrics HTTP/1.1\r\nHost: x\r\n\r\n"),
           "client sends a complete request");
    EXPECT(prom_conn_service(&c, 1000) == 1, "complete request → serve");
    EXPECT(strncmp(c.req, "GET /metrics", 12) == 0, "request text captured");
    prom_conn_close(&c);
    EXPECT(!prom_conn_busy(&c), "idle after close");
    close(cli);

    /* Split across two writes: the first must not look complete. */
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 1000, 2000) == 0, "adopt");
    send_str(cli, "GET /metrics HTTP/1.1\r\n");
    EXPECT(prom_conn_service(&c, 1000) == 0, "partial request → keep reading");
    EXPECT(prom_conn_busy(&c), "still holding the connection");
    send_str(cli, "\r\n");
    EXPECT(prom_conn_service(&c, 1000) == 1, "terminator completes it");
    prom_conn_close(&c);
    close(cli);

    /* A bare LFLF request, which is what `nc` and curl --http0.9 produce. */
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 0, 2000) == 0, "adopt");
    send_str(cli, "GET /metrics\n\n");
    EXPECT(prom_conn_service(&c, 0) == 1, "LFLF also terminates a request");
    prom_conn_close(&c);
    close(cli);

    end(fb);
}

/*
 * The finding itself: a client that connects and sends nothing must not be
 * able to hold the loop. Before this, serve_scrape() did a blocking recv()
 * under a 2 s SO_RCVTIMEO, so the stream reader stopped for 2 s per
 * iteration; the header comment claimed the opposite.
 */
static void test_http_silent_client_is_bounded(void)
{
    begin("test_http_silent_client_is_bounded");
    int fb = g_fail;

    prom_conn_t c;
    prom_conn_init(&c);

    int srv, cli;
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 10000, 2000) == 0, "adopt at t=10000");

    /* Silent client: service returns "still reading" and never blocks. */
    EXPECT(prom_conn_service(&c, 10000) == 0, "silent at t=10000 → keep waiting");
    EXPECT(prom_conn_service(&c, 11999) == 0, "still inside the deadline");

    /* And poll can never be told to wait past the deadline — that bound is
     * what keeps the imud stream drain running on schedule. */
    EXPECT(prom_conn_timeout_ms(&c, 10000) == 2000, "full budget at adopt time");
    EXPECT(prom_conn_timeout_ms(&c, 11500) == 500,  "budget shrinks with time");
    EXPECT(prom_conn_timeout_ms(&c, 12000) == 0,    "0 once due, never negative");
    EXPECT(prom_conn_timeout_ms(&c, 99999) == 0,    "0 well past the deadline");

    EXPECT(prom_conn_service(&c, 12000) == -1, "dropped at the deadline");
    EXPECT(!prom_conn_busy(&c), "connection released");
    EXPECT(!fd_is_open(srv), "and its fd closed");

    close(cli);
    end(fb);
}

static void test_http_drop_paths(void)
{
    begin("test_http_drop_paths");
    int fb = g_fail;

    prom_conn_t c;
    prom_conn_init(&c);

    /* Peer hangs up mid-request. */
    int srv, cli;
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 0, 2000) == 0, "adopt");
    send_str(cli, "GET /metrics HTTP/1.1\r\n");
    close(cli);
    EXPECT(prom_conn_service(&c, 0) == -1, "EOF mid-request → drop");
    EXPECT(!fd_is_open(srv), "fd closed on EOF");

    /* A head larger than PROM_REQ_MAX with no terminator is not a scraper. */
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 0, 60000) == 0, "adopt");
    static char flood[PROM_REQ_MAX * 2];
    memset(flood, 'A', sizeof flood);
    (void)!write(cli, flood, sizeof flood);
    int rc = 0;
    for (int i = 0; i < 8 && rc == 0; i++) rc = prom_conn_service(&c, 0);
    EXPECT(rc == -1, "oversize request head → drop");
    EXPECT(!fd_is_open(srv), "fd closed on oversize");
    close(cli);

    /* Adopting while busy must reject the newcomer and close its fd, not
     * evict the connection already being served. */
    int srv2, cli2;
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(pair_open(&srv2, &cli2) == 0, "second socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 0, 2000) == 0, "first adopt succeeds");
    EXPECT(prom_conn_adopt(&c, srv2, 0, 2000) == -1, "second adopt rejected");
    EXPECT(!fd_is_open(srv2), "rejected fd is closed by adopt, not leaked");
    EXPECT(c.fd == srv, "the incumbent connection is untouched");
    EXPECT(fd_is_open(srv), "and still open");
    prom_conn_close(&c);
    close(cli); close(cli2);

    /* close() is idempotent and adopt(-1) is refused. */
    prom_conn_close(&c);
    EXPECT(!prom_conn_busy(&c), "double close is safe");
    EXPECT(prom_conn_adopt(&c, -1, 0, 2000) == -1, "adopting -1 is refused");

    end(fb);
}

/* The adopted fd must be non-blocking (else service() would stall, which is
 * the whole finding) and close-on-exec, and the page write must be
 * able to put it back to blocking for bridge_write_all. */
static void test_http_fd_flags(void)
{
    begin("test_http_fd_flags");
    int fb = g_fail;

    prom_conn_t c;
    prom_conn_init(&c);

    int srv, cli;
    EXPECT(pair_open(&srv, &cli) == 0, "socketpair");
    EXPECT(prom_conn_adopt(&c, srv, 0, 2000) == 0, "adopt");
    EXPECT((fcntl(srv, F_GETFL, 0) & O_NONBLOCK) != 0, "adopted fd is non-blocking");
    EXPECT((fcntl(srv, F_GETFD) & FD_CLOEXEC) != 0, "adopted fd is close-on-exec");

    EXPECT(prom_conn_ready_to_write(&c) == 0, "ready_to_write succeeds");
    EXPECT((fcntl(srv, F_GETFL, 0) & O_NONBLOCK) == 0,
           "blocking again so the page goes out under SO_SNDTIMEO");
    EXPECT(prom_conn_busy(&c), "still holding the connection to write on");

    prom_conn_close(&c);
    EXPECT(prom_conn_ready_to_write(&c) == -1, "ready_to_write on idle fails");
    close(cli);
    end(fb);
}

int main(void)
{
    puts("=== imud prometheus tests ===");
    test_no_packet();
    test_values_si();
    test_flags_and_gating();
    test_help_type();
    test_buffer_too_small();
    test_http_request_shapes();
    test_http_silent_client_is_bounded();
    test_http_drop_paths();
    test_http_fd_flags();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
