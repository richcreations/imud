/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_tools_e2e.c — imud-status and imud-mon end to end, main() included
 *
 * status_main.c (64 lines) and mon_main.c (285) were in no test binary.
 * Their pure halves were extracted and covered earlier — status_fmt.c
 * formats the report, mon_parse.c decodes the streams — and what stayed behind
 * is the part that touches the world: connect to a socket and copy it to
 * stdout; bind two UDP ports, select, drain, render.
 *
 * Both real main()s are compiled as <base>_entry by the Makefile's
 * src/%.entry.o rule and called here, against sockets this test binds. stdout
 * is captured through a dup2 of fd 1, because "what the operator sees" is the
 * whole output contract of both tools.
 *
 * mon_main is stopped through its own SIGINT handler rather than a flag — that
 * handler and the g_stop it sets are themselves part of what was never
 * executed. status_main needs no stopping: it returns at EOF.
 *
 * Portable: AF_UNIX, UDP, threads. Every transient file lives in /tmp rather
 * than the build directory — see the note in test_bridge_e2e.c.
 */

#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
#endif

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include "types.h"
#include "packet.h"

int status_main_entry(int argc, char **argv);
int mon_main_entry(int argc, char **argv);

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── stdout capture ──────────────────────────────────────────────────────── */

/* Both streams: what the operator sees is stdout for the report and stderr for
 * the diagnostics, and capturing only one leaves the other spraying into the
 * suite's own output, where it reads like a failure. */
typedef struct { int saved_out, saved_err, tmp; char path[64]; } cap_t;

static void cap_begin(cap_t *c, const char *path)
{
    fflush(stdout);
    fflush(stderr);
    snprintf(c->path, sizeof c->path, "%s", path);
    c->saved_out = dup(STDOUT_FILENO);
    c->saved_err = dup(STDERR_FILENO);
    c->tmp       = open(path, O_RDWR | O_CREAT | O_TRUNC, 0600);
    dup2(c->tmp, STDOUT_FILENO);
    dup2(c->tmp, STDERR_FILENO);
}

static void cap_end(cap_t *c, char *out, size_t outsz)
{
    fflush(stdout);
    fflush(stderr);
    dup2(c->saved_out, STDOUT_FILENO);
    dup2(c->saved_err, STDERR_FILENO);
    close(c->saved_out);
    close(c->saved_err);
    lseek(c->tmp, 0, SEEK_SET);
    ssize_t n = read(c->tmp, out, outsz - 1);
    out[n > 0 ? (size_t)n : 0] = '\0';
    close(c->tmp);
    unlink(c->path);
}

/* ── imud-status ─────────────────────────────────────────────────────────── */

#define ST_SOCK "/tmp/imud_e2e_status.sock"

static const char *ST_REPORT =
    "imud 1.8  up 0d 00:12:34\n"
    "state: converged  heading 271.5°\n";

/* Serve one canned report and hang up — imud's status socket is one-shot. */
static void *status_server(void *arg)
{
    int srv = *(int *)arg;
    int c = accept(srv, NULL, NULL);
    if (c >= 0) {
        size_t len = strlen(ST_REPORT);
        ssize_t w = send(c, ST_REPORT, len, 0);
        (void)w;
        close(c);
    }
    return NULL;
}

static void test_status_prints_the_report(void)
{
    begin("test_status_prints_the_report");
    int fb = g_fail;

    unlink(ST_SOCK);
    int srv = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un a;
    memset(&a, 0, sizeof a);
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", ST_SOCK);
    EXPECT(bind(srv, (struct sockaddr *)&a, sizeof a) == 0, "status socket bound");
    EXPECT(listen(srv, 1) == 0, "listening");

    pthread_t tid;
    pthread_create(&tid, NULL, status_server, &srv);

    char *argv[] = { (char *)"imud-status", (char *)"--socket", (char *)ST_SOCK, NULL };
    cap_t cap;
    cap_begin(&cap, "/tmp/imud_e2e_status.out");
    int rc = status_main_entry(3, argv);
    char out[4096];
    cap_end(&cap, out, sizeof out);

    pthread_join(tid, NULL);
    close(srv);
    unlink(ST_SOCK);

    EXPECT(rc == 0, "exit 0 on a served report");
    EXPECT(strstr(out, "converged") != NULL, "the report reached stdout");
    EXPECT(strstr(out, "271.5") != NULL, "verbatim, not reformatted");
    end(fb);
}

/* No daemon: a distinct non-zero exit, not a crash and not 0. */
static void test_status_no_daemon(void)
{
    begin("test_status_no_daemon");
    int fb = g_fail;

    unlink(ST_SOCK);
    char *argv[] = { (char *)"imud-status", (char *)"--socket", (char *)ST_SOCK, NULL };
    cap_t cap;
    cap_begin(&cap, "/tmp/imud_e2e_status_err.out");
    int rc = status_main_entry(3, argv);
    char out[4096];
    cap_end(&cap, out, sizeof out);

    EXPECT(rc == 3, "connect failure exits 3");
    EXPECT(strstr(out, "connect(") != NULL, "says which socket it could not reach");
    end(fb);
}

/* sun_path is 108 bytes; a longer path must be rejected before connect(),
 * with its own exit code. */
static void test_status_path_too_long(void)
{
    begin("test_status_path_too_long");
    int fb = g_fail;

    char longpath[300];
    memset(longpath, 'x', sizeof longpath - 1);
    longpath[0] = '/';
    longpath[sizeof longpath - 1] = '\0';

    char *argv[] = { (char *)"imud-status", (char *)"--socket", longpath, NULL };
    cap_t cap;
    cap_begin(&cap, "/tmp/imud_e2e_status_long.out");
    int rc = status_main_entry(3, argv);
    char out[4096];
    cap_end(&cap, out, sizeof out);

    EXPECT(rc == 2, "over-long socket path exits 2");
    EXPECT(strstr(out, "too long") != NULL, "diagnoses the length, not errno");
    end(fb);
}

/* ── imud-mon ────────────────────────────────────────────────────────────── */

#define MON_CONF "/tmp/imud_e2e_mon.conf"

static int free_udp_port(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = 0;
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) { close(fd); return -1; }
    socklen_t al = sizeof a;
    getsockname(fd, (struct sockaddr *)&a, &al);
    int port = ntohs(a.sin_port);
    close(fd);
    return port;
}

static void udp_send_to(int port, const void *buf, size_t n)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_port        = htons((uint16_t)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ssize_t w = sendto(fd, buf, n, 0, (struct sockaddr *)&a, sizeof a);
    (void)w;
    close(fd);
}

typedef struct { int rc; pthread_t tid; } mon_run_t;

static void *mon_thread(void *arg)
{
    mon_run_t *m = (mon_run_t *)arg;
    char *argv[] = { (char *)"imud-mon", (char *)"--config", (char *)MON_CONF, NULL };
    m->rc = mon_main_entry(3, argv);
    return NULL;
}

/* Feed both streams, let one render tick land, then stop it the way an
 * operator does. The daemon's own encoder builds the binary packet, so the
 * CRC and magic are real — mon drops anything that fails either. */
static void test_mon_renders_both_streams(void)
{
    begin("test_mon_renders_both_streams");
    int fb = g_fail;

    int nmea_port = free_udp_port();
    int bin_port  = free_udp_port();
    EXPECT(nmea_port > 0 && bin_port > 0, "found two free UDP ports");

    FILE *f = fopen(MON_CONF, "w");
    fprintf(f,
            "[nmea]\nenabled = true\ndest_addr = \"127.0.0.1\"\ndest_port = %d\n"
            "[highrate]\nenabled = true\ndest_addr = \"127.0.0.1\"\ndest_port = %d\n"
            "[logging]\nlevel = \"error\"\n",
            nmea_port, bin_port);
    fclose(f);

    cap_t cap;
    cap_begin(&cap, "/tmp/imud_e2e_mon.out");

    mon_run_t m = { .rc = -999 };
    pthread_create(&m.tid, NULL, mon_thread, &m);

    /* Give it time to bind before sending: a datagram to an unbound port is
     * simply dropped, and the test would then be asserting on nothing. */
    struct timespec settle = { 0, 300 * 1000 * 1000 };
    nanosleep(&settle, NULL);

    const char *hdt = "$IMHDT,271.5,T*2A\r\n";
    fused_state_t s;
    memset(&s, 0, sizeof s);
    s.q[0] = 1.0f;
    s.heading_deg = 271.5f;
    s.imu_seq = 42;
    mag_sample_t mg;
    memset(&mg, 0, sizeof mg);
    imu_sample_t im;
    memset(&im, 0, sizeof im);
    im.temp_c = 31.4f;
    imu_packet_t pkt;
    packet_build(&pkt, &s, &mg, &im, &im, "NED");
    uint8_t wire[IMUD_PACKET_BYTES];
    packet_encode(wire, &pkt);

    for (int i = 0; i < 12; i++) {
        udp_send_to(nmea_port, hdt, strlen(hdt));
        udp_send_to(bin_port, wire, sizeof wire);
        struct timespec t = { 0, 120 * 1000 * 1000 };
        nanosleep(&t, NULL);          /* spans at least one 1 Hz render tick */
    }

    pthread_kill(m.tid, SIGINT);      /* the handler mon installs, doing its job */
    pthread_join(m.tid, NULL);

    char out[65536];
    cap_end(&cap, out, sizeof out);

    EXPECT(m.rc == 0, "mon exits 0 on SIGINT");
    EXPECT(strstr(out, "imud-mon") != NULL, "printed its banner");
    EXPECT(strstr(out, "271.5") != NULL, "rendered the heading it was sent");
    EXPECT(strstr(out, "42") != NULL, "rendered the binary packet's sequence");

    unlink(MON_CONF);
    end(fb);
}

typedef struct { int rc; pthread_t tid; const char *sel; } mon_sel_t;

static void *mon_sel_thread(void *arg)
{
    mon_sel_t *m = (mon_sel_t *)arg;
    char *argv[] = { (char *)"imud-mon", (char *)"--config", (char *)MON_CONF,
                     (char *)m->sel, NULL };
    m->rc = mon_main_entry(4, argv);
    return NULL;
}

/* The positional stream selector: `imud-mon nmea` must bind only the NMEA
 * port, which the banner is the observable proof of. (There is no way to
 * select zero streams — the parser defaults to both — so mon's
 * "no streams available" branch is unreachable from the CLI.) */
static void test_mon_selects_one_stream(void)
{
    begin("test_mon_selects_one_stream");
    int fb = g_fail;

    int nmea_port = free_udp_port();
    int bin_port  = free_udp_port();

    FILE *f = fopen(MON_CONF, "w");
    fprintf(f,
            "[nmea]\nenabled = true\ndest_addr = \"127.0.0.1\"\ndest_port = %d\n"
            "[highrate]\nenabled = true\ndest_addr = \"127.0.0.1\"\ndest_port = %d\n"
            "[logging]\nlevel = \"error\"\n",
            nmea_port, bin_port);
    fclose(f);

    cap_t cap;
    cap_begin(&cap, "/tmp/imud_e2e_mon_sel.out");

    mon_sel_t m = { .rc = -999, .sel = "nmea" };
    pthread_create(&m.tid, NULL, mon_sel_thread, &m);

    struct timespec settle = { 0, 400 * 1000 * 1000 };
    nanosleep(&settle, NULL);
    pthread_kill(m.tid, SIGTERM);            /* the other handler it installs */
    pthread_join(m.tid, NULL);

    char out[16384];
    cap_end(&cap, out, sizeof out);

    char want[64];
    snprintf(want, sizeof want, "NMEA:%d", nmea_port);
    EXPECT(m.rc == 0, "mon exits 0 on SIGTERM");
    EXPECT(strstr(out, want) != NULL, "banner names the NMEA port it bound");
    EXPECT(strstr(out, "Binary:") == NULL, "binary stream not bound when unselected");

    unlink(MON_CONF);
    end(fb);
}

int main(void)
{
    test_status_prints_the_report();
    test_status_no_daemon();
    test_status_path_too_long();
    test_mon_renders_both_streams();
    test_mon_selects_one_stream();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
