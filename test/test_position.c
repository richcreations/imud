/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_position.c — unit tests for pos_json_double() and gpsd/SignalK parsing
 *
 * Two halves.  The first drives the JSON field extractor and the fix-TTL
 * watchdog with string literals — no sockets, no hardware.  The second runs
 * position_thread for real against a fake gpsd / SignalK server on loopback,
 * covering tcp_connect_host, the line reader, run_gpsd and poll_signalk, which
 * is the code an operator actually depends on.
 *
 * The binary links position.c and wmm.c but stubs imu_ctx_set_declination()
 * and friends, so imu.c and its hardware dependencies are not required — and
 * those stubs double as the assertion surface for the socket tests.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdatomic.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include "../include/config.h"
#include "../include/position.h"
#include "../include/imu.h"   /* for imu_ctx_t forward decl — no implementation */
#include "fdsweep.h"

/* ── Stub ────────────────────────────────────────────────────────────────── */

/*
 * Minimal stub so position.c can be linked without imu.c.
 * Records the last declination value + validity written for assertion.
 */
float g_last_decl  = -999.0f;
bool  g_last_valid = true;

/*
 * Update counter for the socket tests further down: they need to wait for the
 * position thread to deliver a fix, and polling g_last_decl itself would be a
 * genuine data race (a plain float written by one thread and read by another;
 * `volatile` is not atomicity — TSan catches exactly this, and the daemon's
 * own convention is _Atomic for anything crossing threads).  The float values
 * are only read after the thread is joined.
 */
_Atomic int g_decl_updates = 0;

void imu_ctx_set_declination(imu_ctx_t *ctx, float decl_deg, bool valid)
{
    (void)ctx;
    g_last_decl  = decl_deg;
    g_last_valid = valid;
    g_decl_updates++;
}

float g_last_mref_h = -1.0f, g_last_mref_z = -1.0f;
void imu_ctx_set_mag_ref(imu_ctx_t *ctx, float h_gauss, float z_gauss)
{
    (void)ctx;
    g_last_mref_h = h_gauss;
    g_last_mref_z = z_gauss;
}

float g_last_speed = -1.0f;
bool  g_last_speed_valid = true;
void imu_ctx_set_speed(imu_ctx_t *ctx, float speed_mps, bool valid)
{
    (void)ctx;
    g_last_speed       = speed_mps;
    g_last_speed_valid = valid;
}

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR_D(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), (msg))

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── pos_json_double tests ───────────────────────────────────────────────── */

/* Hostile numeric inputs must be rejected (finding 3, 1.5.1): a NaN/Inf or
 * a truncated/garbage number reaching the WMM/atan2 path poisons the MEKF. */
static void test_json_double_nan_inf_rejected(void)
{
    begin("test_json_double_nan_inf_rejected");
    int fb = g_fail;
    double v = 999.0;
    EXPECT(!pos_json_double("{\"lat\":nan}",       "lat", &v), "nan rejected");
    EXPECT(!pos_json_double("{\"lat\":inf}",       "lat", &v), "inf rejected");
    EXPECT(!pos_json_double("{\"lat\":-inf}",      "lat", &v), "-inf rejected");
    EXPECT(!pos_json_double("{\"lat\":infinity}",  "lat", &v), "infinity rejected");
    EXPECT(!pos_json_double("{\"lat\":1e400}",     "lat", &v), "overflow→inf rejected");
    end(fb);
}

/* A number must be followed by a JSON delimiter, not stray text. */
static void test_json_double_delimiter(void)
{
    begin("test_json_double_delimiter");
    int fb = g_fail;
    double v = 0.0;
    EXPECT(!pos_json_double("{\"lat\":12abc}", "lat", &v), "numeric prefix rejected");
    EXPECT(pos_json_double("{\"lat\":12.5,\"lon\":3}", "lat", &v) && v == 12.5,
           "comma-terminated accepted");
    EXPECT(pos_json_double("{\"lat\":12.5}", "lat", &v) && v == 12.5,
           "brace-terminated accepted");
    end(fb);
}

static void test_pos_fix_valid_ranges(void)
{
    begin("test_pos_fix_valid_ranges");
    int fb = g_fail;
    EXPECT(pos_fix_valid(37.8, -122.3, 12.0),   "in-range fix valid");
    EXPECT(pos_fix_valid(-90.0, 180.0, 0.0),    "boundary valid");
    EXPECT(!pos_fix_valid(90.1, 0.0, 0.0),      "lat > 90 rejected");
    EXPECT(!pos_fix_valid(0.0, 180.1, 0.0),     "lon > 180 rejected");
    EXPECT(!pos_fix_valid(-91.0, 0.0, 0.0),     "lat < -90 rejected");
    double inf = 1.0/0.0, nan = 0.0/0.0;
    EXPECT(!pos_fix_valid(inf, 0.0, 0.0),       "inf lat rejected");
    EXPECT(!pos_fix_valid(0.0, nan, 0.0),       "nan lon rejected");
    EXPECT(!pos_fix_valid(37.0, -122.0, inf),   "inf alt rejected");
    end(fb);
}

static void test_json_double_basic(void)
{
    begin("test_json_double_basic");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"lat\":37.87}", "lat", &v), "key found");
    EXPECT_NEAR_D(v, 37.87, 1e-9, "value parsed");
    end(fb);
}

static void test_json_double_negative(void)
{
    begin("test_json_double_negative");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"lon\":-122.315}", "lon", &v), "key found");
    EXPECT_NEAR_D(v, -122.315, 1e-9, "negative value parsed");
    end(fb);
}

static void test_json_double_integer(void)
{
    begin("test_json_double_integer");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"mode\":3}", "mode", &v), "key found");
    EXPECT_NEAR_D(v, 3.0, 1e-9, "integer parsed as double");
    end(fb);
}

static void test_json_double_missing_key(void)
{
    begin("test_json_double_missing_key");
    int fb = g_fail;
    double v = 0;
    EXPECT(!pos_json_double("{\"lat\":37.87}", "lon", &v), "missing key → false");
    end(fb);
}

static void test_json_double_null_value(void)
{
    begin("test_json_double_null_value");
    int fb = g_fail;
    double v = 0;
    /* gpsd emits "lat":null when there is no fix */
    EXPECT(!pos_json_double("{\"lat\":null}", "lat", &v), "null value → false");
    end(fb);
}

static void test_json_double_string_value(void)
{
    begin("test_json_double_string_value");
    int fb = g_fail;
    double v = 0;
    EXPECT(!pos_json_double("{\"lat\":\"N/A\"}", "lat", &v), "string value → false");
    end(fb);
}

static void test_json_double_spaces(void)
{
    begin("test_json_double_spaces");
    int fb = g_fail;
    double v;
    /* gpsd sometimes emits spaces after the colon */
    EXPECT(pos_json_double("{\"lat\": 37.87}", "lat", &v), "space after colon");
    EXPECT_NEAR_D(v, 37.87, 1e-9, "value correct after space");
    end(fb);
}

static void test_json_double_scientific(void)
{
    begin("test_json_double_scientific");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"alt\":3.787e+01}", "alt", &v), "scientific notation parsed");
    EXPECT_NEAR_D(v, 37.87, 1e-6, "scientific value correct");
    end(fb);
}

/* Full gpsd TPV message (representative of actual gpsd output). */
static void test_gpsd_tpv_full(void)
{
    begin("test_gpsd_tpv_full");
    int fb = g_fail;

    const char *tpv =
        "{\"class\":\"TPV\",\"device\":\"/dev/ttyUSB0\","
        "\"mode\":3,\"time\":\"2025-05-01T12:00:00.000Z\","
        "\"lat\":37.8697,\"lon\":-122.3153,"
        "\"alt\":5.0,\"speed\":0.0,\"track\":0.0}";

    double mode = 0, lat = 0, lon = 0;
    EXPECT(pos_json_double(tpv, "mode", &mode), "mode found");
    EXPECT_NEAR_D(mode, 3.0, 1e-9,     "mode = 3");
    EXPECT(pos_json_double(tpv, "lat",  &lat),  "lat found");
    EXPECT_NEAR_D(lat,  37.8697, 1e-9,  "lat = 37.8697");
    EXPECT(pos_json_double(tpv, "lon",  &lon),  "lon found");
    EXPECT_NEAR_D(lon, -122.3153, 1e-9, "lon = -122.3153");
    EXPECT(strstr(tpv, "\"class\":\"TPV\"") != NULL, "class:TPV present");
    end(fb);
}

/* Full SignalK position REST response. */
static void test_signalk_response(void)
{
    begin("test_signalk_response");
    int fb = g_fail;

    const char *body =
        "{\"value\":{\"longitude\":-122.3153,\"latitude\":37.8697},"
        "\"$source\":\"gps.0\","
        "\"timestamp\":\"2025-05-01T12:00:00.000Z\"}";

    double lat = 0, lon = 0;
    EXPECT(pos_json_double(body, "latitude",  &lat), "latitude found");
    EXPECT_NEAR_D(lat,  37.8697, 1e-9,  "latitude = 37.8697");
    EXPECT(pos_json_double(body, "longitude", &lon), "longitude found");
    EXPECT_NEAR_D(lon, -122.3153, 1e-9, "longitude = -122.3153");
    end(fb);
}

/* gpsd TPV with mode < 2 must be treated as no-fix. */
static void test_gpsd_no_fix(void)
{
    begin("test_gpsd_no_fix");
    int fb = g_fail;

    const char *tpv = "{\"class\":\"TPV\",\"mode\":1,\"lat\":0.0,\"lon\":0.0}";
    double mode = 0;
    EXPECT(pos_json_double(tpv, "mode", &mode), "mode parsed");
    EXPECT(mode < 2.0, "mode < 2 → no fix");
    end(fb);
}

/* Multiple keys in one JSON object — each must be found independently. */
static void test_json_multiple_keys(void)
{
    begin("test_json_multiple_keys");
    int fb = g_fail;

    const char *json = "{\"a\":1.0,\"b\":2.5,\"c\":-3.0}";
    double a, b, c;
    EXPECT(pos_json_double(json, "a", &a), "key a");
    EXPECT(pos_json_double(json, "b", &b), "key b");
    EXPECT(pos_json_double(json, "c", &c), "key c");
    EXPECT_NEAR_D(a,  1.0, 1e-9, "a = 1.0");
    EXPECT_NEAR_D(b,  2.5, 1e-9, "b = 2.5");
    EXPECT_NEAR_D(c, -3.0, 1e-9, "c = -3.0");
    end(fb);
}

/*
 * fix_max_age_h TTL: verify the TTL-seconds arithmetic and the boundary
 * conditions that gate the watchdog without mocking the clock.
 *
 * The position_thread watchdog fires when:
 *   fix_max_age_h > 0  AND  last_fix_time > 0
 *   AND  difftime(now, last_fix_time) > fix_max_age_h * 3600
 *
 * All three conditions must be true; any false → no clear.
 */
static void test_fix_ttl_arithmetic(void)
{
    begin("test_fix_ttl_arithmetic");
    int fb = g_fail;

    /* Default 24 h = 86400 s */
    float max_age_h = 24.0f;
    double ttl_s = (double)(max_age_h * 3600.0f);
    EXPECT_NEAR_D(ttl_s, 86400.0, 1.0, "24 h → 86400 s TTL");

    /* 0 = disabled — watchdog must not fire */
    float disabled = 0.0f;
    EXPECT(!(disabled > 0.0f), "fix_max_age_h = 0 suppresses watchdog");

    /* no fix yet (last_fix_time == 0) — watchdog must not fire */
    time_t no_fix = 0;
    EXPECT(!(no_fix > 0), "last_fix_time = 0 suppresses watchdog");

    /* Simulate: fix received, TTL elapsed → watchdog should clear.
     * We don't actually sleep; just verify the comparison logic. */
    time_t fake_fix = 1000;
    time_t fake_now = fake_fix + (time_t)(ttl_s) + 1;   /* 1 second past TTL */
    EXPECT(difftime(fake_now, fake_fix) > ttl_s, "elapsed > TTL triggers clear");

    /* One second short → no clear */
    time_t fake_now_short = fake_fix + (time_t)(ttl_s) - 1;
    EXPECT(!(difftime(fake_now_short, fake_fix) > ttl_s), "elapsed < TTL → no clear");

    end(fb);
}

/*
 * check_fix_ttl() behavior — the real watchdog function, driven directly.
 *
 * Regression test for the anchored-vessel recovery bug: on expiry the
 * watchdog must reset last_lat/last_lon to the force-update sentinel so
 * the next fix at an unchanged position recomputes declination immediately,
 * not after a ≥5 km move.
 */
static void test_check_fix_ttl(void)
{
    begin("test_check_fix_ttl");
    int fb = g_fail;

    imud_config_t cfg;
    memset(&cfg, 0, sizeof cfg);
    pos_ctx_t ctx = { .cfg = &cfg, .imu = NULL, .stop = 0 };

    /* Expired fix (2 h old, 1 h TTL) → clear + sentinel reset. */
    cfg.pos_fix_max_age_h = 1.0f;
    time_t fix  = time(NULL) - 7200;
    double lat  = 37.87, lon = -122.32;
    g_last_decl = -999.0f; g_last_valid = true;
    g_last_speed = -1.0f;  g_last_speed_valid = true;
    check_fix_ttl(&ctx, &fix, &lat, &lon);
    EXPECT_NEAR_D(g_last_decl, 0.0, 1e-9, "expired fix clears declination");
    EXPECT(!g_last_valid,                 "expired fix marks declination invalid");
    EXPECT(!g_last_speed_valid,           "expired fix invalidates speed too");
    EXPECT(fix == 0,                      "expired fix zeroes last_fix_time");
    EXPECT(lat > 900.0 && lon > 900.0,    "expired fix resets position sentinel");

    /* Fresh fix → untouched. */
    fix = time(NULL);
    lat = 37.87; lon = -122.32;
    g_last_decl = -999.0f;
    check_fix_ttl(&ctx, &fix, &lat, &lon);
    EXPECT_NEAR_D(g_last_decl, -999.0, 1e-9, "fresh fix leaves declination alone");
    EXPECT(fix != 0,                          "fresh fix keeps last_fix_time");
    EXPECT_NEAR_D(lat, 37.87, 1e-9,           "fresh fix keeps position");

    /* TTL disabled → stale fix untouched. */
    cfg.pos_fix_max_age_h = 0.0f;
    fix = time(NULL) - 7200;
    g_last_decl = -999.0f;
    check_fix_ttl(&ctx, &fix, &lat, &lon);
    EXPECT_NEAR_D(g_last_decl, -999.0, 1e-9, "disabled TTL never clears");

    /* No fix yet (last_fix_time = 0) → untouched. */
    cfg.pos_fix_max_age_h = 1.0f;
    fix = 0;
    g_last_decl = -999.0f;
    check_fix_ttl(&ctx, &fix, &lat, &lon);
    EXPECT_NEAR_D(g_last_decl, -999.0, 1e-9, "no-fix-yet never clears");

    end(fb);
}

/* ── Live socket tests: the gpsd and SignalK clients end to end ──────────── */

/*
 * Everything above drives the parsers with string literals.  These drive the
 * real client paths — tcp_connect_host, line_reader, read_line's select loop,
 * run_gpsd, poll_signalk and position_thread itself — against a fake server on
 * loopback, because that is the half of position.c an operator actually
 * depends on and none of it was covered.
 *
 * The imu_ctx_* stubs at the top of this file are the assertion surface: if a
 * fix travelled the whole way through, g_last_decl and g_last_speed move.
 */

/*
 * Audit L3 — position.c's client socket must be close-on-exec.
 *
 * It lives inside pos_ctx_t and is closed again the moment position_thread
 * stops, so the only instant it can be observed from outside is between the
 * fake server's accept() returning and the session ending. The sweep
 * therefore runs on the server thread; the main thread reads the verdict
 * after the join. The baseline is taken once the listener exists, so the
 * test's own descriptors are never counted.
 */
static fdsweep_t   g_srv_sweep;
static _Atomic int g_srv_cloexec_leaks = -1;

/* Bind 127.0.0.1:0, listen, and report the kernel-assigned port. */
static int fake_server_open(int *port_out)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);

    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = 0;
    if (bind(fd, (struct sockaddr *)&a, sizeof a) != 0 || listen(fd, 1) != 0) {
        close(fd); return -1;
    }
    socklen_t alen = sizeof a;
    if (getsockname(fd, (struct sockaddr *)&a, &alen) != 0) { close(fd); return -1; }
    *port_out = ntohs(a.sin_port);
    fdsweep_begin(&g_srv_sweep);   /* after the listener exists: it is ours */
    g_srv_cloexec_leaks = -1;
    return fd;
}

/* What the fake server should send once a client connects. */
static const char *g_srv_reply;
static int         g_srv_listen  = -1;
static char        g_srv_request[512];
static _Atomic int g_srv_got_conn;

/* Accept one client, record what it sent us, reply, close. */
static void *fake_server_thread(void *arg)
{
    (void)arg;
    int c = accept(g_srv_listen, NULL, NULL);
    if (c < 0) return NULL;
    fcntl(c, F_SETFD, FD_CLOEXEC);   /* ours, not under test — keep the floor at 0 */
    g_srv_cloexec_leaks = fdsweep_leaks(&g_srv_sweep);
    g_srv_got_conn = 1;

    /* The client speaks first in both protocols: gpsd gets ?WATCH, SignalK
     * gets an HTTP GET.  Capture it so the test can assert on it. */
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    ssize_t n = recv(c, g_srv_request, sizeof g_srv_request - 1, 0);
    if (n > 0) g_srv_request[n] = '\0';

    if (g_srv_reply) {
        ssize_t w = write(c, g_srv_reply, strlen(g_srv_reply));
        (void)w;
    }
    /* Closing ends run_gpsd's read loop / completes the HTTP/1.0 body. */
    close(c);
    return NULL;
}

/* Config shared by the socket tests: WMM on, so a fix produces a declination. */
static void socket_test_cfg(imud_config_t *cfg)
{
    config_defaults(cfg);
    cfg->pos_gpsd_enabled    = false;
    cfg->pos_signalk_enabled = false;
    cfg->pos_fix_max_age_h   = 0.0f;   /* watchdog off: not what we measure */
    snprintf(cfg->pos_gpsd_host,    sizeof cfg->pos_gpsd_host,    "127.0.0.1");
    snprintf(cfg->pos_signalk_host, sizeof cfg->pos_signalk_host, "127.0.0.1");
}

/*
 * Run position_thread until it delivers a declination update, then stop and
 * join.  Waits on the _Atomic counter, never on the float values — those are
 * read by the caller only after this has joined the thread.
 */
static bool run_until_fix(pos_ctx_t *ctx)
{
    int before = g_decl_updates;
    pthread_t tid;
    if (pthread_create(&tid, NULL, position_thread, ctx) != 0) return false;
    bool got = false;
    for (int i = 0; i < 300 && !got; i++) {        /* ≤ 3 s */
        if (g_decl_updates != before) got = true; else usleep(10000);
    }
    ctx->stop = 1;
    pthread_join(tid, NULL);
    return got;
}

static void test_gpsd_live_session(void)
{
    begin("test_gpsd_live_session");
    int fb = g_fail;

    int port = 0;
    g_srv_listen = fake_server_open(&port);
    EXPECT(g_srv_listen >= 0, "fake gpsd listening on loopback");
    if (g_srv_listen < 0) { end(fb); return; }

    g_srv_got_conn = 0;
    g_srv_request[0] = '\0';
    g_srv_reply =
        "{\"class\":\"TPV\",\"mode\":3,\"lat\":37.8697,\"lon\":-122.3153,"
        "\"altMSL\":5.0,\"speed\":3.5}\n";

    pthread_t srv;
    pthread_create(&srv, NULL, fake_server_thread, NULL);

    imud_config_t cfg;
    socket_test_cfg(&cfg);
    cfg.pos_gpsd_enabled = true;
    cfg.pos_gpsd_port    = port;

    pos_ctx_t ctx;
    memset(&ctx, 0, sizeof ctx);
    ctx.cfg = &cfg;
    ctx.imu = (imu_ctx_t *)0x1;          /* only the stubs above touch it */
    snprintf(ctx.wmm_file, sizeof ctx.wmm_file, "data/WMM.COF");

    g_last_decl = -999.0f; g_last_speed = -1.0f;
    bool moved = run_until_fix(&ctx);
    pthread_join(srv, NULL);

    EXPECT(g_srv_got_conn == 1, "position_thread connected to gpsd");
    EXPECT(strstr(g_srv_request, "?WATCH=") != NULL,
           "client subscribed with ?WATCH");
    EXPECT(strstr(g_srv_request, "\"json\":true") != NULL,
           "WATCH requested JSON");
    EXPECT(moved, "a TPV fix reached imu_ctx_set_declination");
    /* San Francisco: declination is easterly and single-digit. */
    EXPECT(g_last_decl > 0.0f && g_last_decl < 25.0f,
           "declination is plausible for the fix location");
    EXPECT_NEAR_D(g_last_speed, 3.5, 1e-4, "speed over ground propagated");
    EXPECT(g_last_mref_h > 0.0f, "magnetic reference H set from the fix");
    EXPECT(g_srv_cloexec_leaks == 0,
           "gpsd client socket is close-on-exec (audit L3)");

    close(g_srv_listen); g_srv_listen = -1;
    end(fb);
}

/* A refused connection must not kill the thread: it retries, and a stop
 * request during the retry backoff still exits promptly. */
static void test_gpsd_connection_refused(void)
{
    begin("test_gpsd_connection_refused");
    int fb = g_fail;

    /* Bind then immediately close: that port is now almost certainly free,
     * which is what we want — connect() should be refused, not hang. */
    int port = 0;
    int probe = fake_server_open(&port);
    EXPECT(probe >= 0, "got a port to abandon");
    if (probe >= 0) close(probe);

    imud_config_t cfg;
    socket_test_cfg(&cfg);
    cfg.pos_gpsd_enabled = true;
    cfg.pos_gpsd_port    = port;

    pos_ctx_t ctx;
    memset(&ctx, 0, sizeof ctx);
    ctx.cfg = &cfg;
    ctx.imu = (imu_ctx_t *)0x1;
    snprintf(ctx.wmm_file, sizeof ctx.wmm_file, "data/WMM.COF");

    pthread_t tid;
    EXPECT(pthread_create(&tid, NULL, position_thread, &ctx) == 0,
           "position_thread starts against a dead port");
    usleep(200000);
    ctx.stop = 1;
    EXPECT(pthread_join(tid, NULL) == 0,
           "thread exits cleanly after a refused connection");
    end(fb);
}

static void test_signalk_live_poll(void)
{
    begin("test_signalk_live_poll");
    int fb = g_fail;

    int port = 0;
    g_srv_listen = fake_server_open(&port);
    EXPECT(g_srv_listen >= 0, "fake SignalK listening on loopback");
    if (g_srv_listen < 0) { end(fb); return; }

    g_srv_got_conn = 0;
    g_srv_request[0] = '\0';
    g_srv_reply =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "\r\n"
        "{\"value\":{\"longitude\":-122.3153,\"latitude\":37.8697},"
        "\"timestamp\":\"2026-05-01T12:00:00.000Z\"}";

    pthread_t srv;
    pthread_create(&srv, NULL, fake_server_thread, NULL);

    imud_config_t cfg;
    socket_test_cfg(&cfg);
    cfg.pos_signalk_enabled = true;
    cfg.pos_signalk_port    = port;

    pos_ctx_t ctx;
    memset(&ctx, 0, sizeof ctx);
    ctx.cfg = &cfg;
    ctx.imu = (imu_ctx_t *)0x1;
    snprintf(ctx.wmm_file, sizeof ctx.wmm_file, "data/WMM.COF");

    g_last_decl = -999.0f;
    bool moved = run_until_fix(&ctx);
    pthread_join(srv, NULL);

    EXPECT(g_srv_got_conn == 1, "position_thread polled SignalK");
    EXPECT(strncmp(g_srv_request, "GET ", 4) == 0, "issued an HTTP GET");
    EXPECT(strstr(g_srv_request, "HTTP/1.0") != NULL,
           "HTTP/1.0 so the server closes to signal end of body");
    EXPECT(moved, "the REST fix reached imu_ctx_set_declination");
    EXPECT(g_last_decl > 0.0f && g_last_decl < 25.0f,
           "declination is plausible for the fix location");
    EXPECT(g_srv_cloexec_leaks == 0,
           "SignalK client socket is close-on-exec (audit L3)");

    close(g_srv_listen); g_srv_listen = -1;
    end(fb);
}

/* Neither source enabled: the thread must return immediately rather than
 * spinning. */
static void test_position_thread_disabled(void)
{
    begin("test_position_thread_disabled");
    int fb = g_fail;

    imud_config_t cfg;
    socket_test_cfg(&cfg);          /* both sources left disabled */

    pos_ctx_t ctx;
    memset(&ctx, 0, sizeof ctx);
    ctx.cfg = &cfg;
    ctx.imu = (imu_ctx_t *)0x1;
    snprintf(ctx.wmm_file, sizeof ctx.wmm_file, "data/WMM.COF");

    pthread_t tid;
    EXPECT(pthread_create(&tid, NULL, position_thread, &ctx) == 0,
           "thread starts");
    EXPECT(pthread_join(tid, NULL) == 0, "and returns without being stopped");
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud position tests ===");

    test_json_double_basic();
    test_json_double_negative();
    test_json_double_integer();
    test_json_double_missing_key();
    test_json_double_null_value();
    test_json_double_string_value();
    test_json_double_spaces();
    test_json_double_scientific();
    test_json_double_nan_inf_rejected();
    test_json_double_delimiter();
    test_pos_fix_valid_ranges();
    test_gpsd_tpv_full();
    test_signalk_response();
    test_gpsd_no_fix();
    test_json_multiple_keys();
    test_fix_ttl_arithmetic();
    test_check_fix_ttl();
    test_position_thread_disabled();
    test_gpsd_live_session();
    test_gpsd_connection_refused();
    test_signalk_live_poll();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
