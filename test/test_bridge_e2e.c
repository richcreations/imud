/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_bridge_e2e.c — the bridge daemons end to end, main() included
 *
 * Audit L2: five bridge entry points, ~1,300 lines, in no test binary. Every
 * ingredient was already covered — the delta/line/frame/metric encoders, the
 * shared scaffolding in bridge.c, netserv, libimud — and none of the WIRING
 * was: which reload keys are hot and which need a restart, whether a dropped
 * stream reconnects, whether a short frame is rejected rather than emitted.
 * That code exists only inside main(), so nothing could call it.
 *
 * So call it. Each daemon's real main() is compiled as <base>_entry by the
 * Makefile's src/%.entry.o rule and driven here against test/fakestream.h,
 * with the daemon's own output socket bound by the test. Nothing in the
 * shipping sources changes shape to accommodate this.
 *
 * Stopping: pthread_kill(SIGTERM) at the daemon's own thread, which is what
 * production does. An earlier version set bridge_stop from the test thread
 * instead, reasoning that it was the same exit path with no signal delivery to
 * race with. That was wrong twice over: `volatile sig_atomic_t` is the correct
 * idiom for a signal HANDLER (same thread, async-signal-safe) and not for a
 * cross-thread write, so TSan reported it as a data race — correctly — and
 * setting the flag directly skipped bridge_install_signals()'s handler, which
 * is itself part of what was never executed. It is a global shared by all
 * five, so it is still reset between daemons (bridge_reset() below).
 *
 * Portable: AF_UNIX + UDP + threads only, so this suite runs on the macOS dev
 * box as well as in the container.
 *
 * Every transient file lives in /tmp, never the build directory: a suite that
 * litters the repo root is a suite whose leftovers have to be gitignored, and a
 * unix socket cannot be gitignored at all — git will not track one, so a stale
 * one simply sits there invisible to `git status`. test_stream.c set this
 * convention already.
 */

#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
#endif

#include <errno.h>
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
#include <sys/time.h>

#include "bridge.h"
#include "fakebroker.h"
#include "fakestream.h"

/* The real main()s, renamed at compile time (see the Makefile). */
int signalk_main_entry(int argc, char **argv);
int influx_main_entry(int argc, char **argv);
int mavlink_main_entry(int argc, char **argv);
int prom_main_entry(int argc, char **argv);
int mqtt_main_entry(int argc, char **argv);

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Scaffolding ─────────────────────────────────────────────────────────── */

static void write_conf(const char *path, const char *fmt, ...)
{
    FILE *f = fopen(path, "w");
    if (!f) { perror("fopen"); exit(1); }
    va_list ap;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);
    fclose(f);
}

/* Bind 127.0.0.1:0 and report the port the kernel chose: two suites running at
 * once (make -j, or a developer's second shell) must not collide on a fixed
 * port, and a hard-coded one also fails on a machine already running imud. */
static int udp_bind_ephemeral(int *port_out)
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
    if (getsockname(fd, (struct sockaddr *)&a, &al) < 0) { close(fd); return -1; }
    *port_out = ntohs(a.sin_port);

    struct timeval tv = { 2, 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return fd;
}

/* Ask the kernel for a port, then hand it to the daemon to bind. */
static int free_tcp_port(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
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

/* One datagram, or -1 on timeout. */
static ssize_t udp_recv(int fd, char *buf, size_t n)
{
    ssize_t r = recv(fd, buf, n - 1, 0);
    if (r > 0) buf[r] = '\0';
    return r;
}

typedef int (*entry_fn)(int, char **);

typedef struct {
    entry_fn    fn;
    const char *conf;
    int         rc;
    pthread_t   tid;
} bridge_run_t;

static void *bridge_thread(void *arg)
{
    bridge_run_t *r = (bridge_run_t *)arg;
    char *argv[] = { (char *)"imud-bridge-test", (char *)"--config",
                     (char *)r->conf, NULL };
    r->rc = r->fn(3, argv);
    return NULL;
}

static void bridge_reset(void)
{
    bridge_stop   = 0;
    bridge_reload = 0;
}

static void bridge_start(bridge_run_t *r, entry_fn fn, const char *conf)
{
    memset(r, 0, sizeof *r);
    r->fn   = fn;
    r->conf = conf;
    r->rc   = -999;
    bridge_reset();
    if (pthread_create(&r->tid, NULL, bridge_thread, r) != 0) {
        fprintf(stderr, "pthread_create failed\n");
        exit(1);
    }
}

static void bridge_finish(bridge_run_t *r)
{
    /* The real path: the daemon's own SIGTERM handler sets bridge_stop from
     * within the signalled thread. Writing the flag from here instead is a
     * cross-thread write to a non-atomic object — a data race, and TSan says so. */
    pthread_kill(r->tid, SIGTERM);
    pthread_join(r->tid, NULL);
    bridge_reset();
}

/* ── signalk ─────────────────────────────────────────────────────────────── */

#define SK_SOCK "/tmp/imud_e2e_signalk.sock"
#define SK_CONF "/tmp/imud_e2e_signalk.conf"

static void test_signalk_udp_delta(void)
{
    begin("test_signalk_udp_delta");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, SK_SOCK, 200) == 0, "fake stream socket started");

    int port = 0;
    int sink = udp_bind_ephemeral(&port);
    EXPECT(sink >= 0, "udp sink bound");

    write_conf(SK_CONF,
               "[imud-signalk]\n"
               "enabled = true\n"
               "socket = \"%s\"\n"
               "rate_hz = 50\n"
               "source_label = \"e2e\"\n"
               "udp_enabled = true\n"
               "dest_addr = \"127.0.0.1\"\n"
               "dest_port = %d\n"
               "[logging]\nlevel = \"error\"\n",
               SK_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, signalk_main_entry, SK_CONF);

    char buf[2048];
    ssize_t n = udp_recv(sink, buf, sizeof buf);
    EXPECT(n > 0, "a delta datagram arrived");

    /* The wiring under test: the packet the fake daemon served reached the
     * encoder and the encoder's output reached the socket the config named. */
    EXPECT(n > 0 && strstr(buf, "\"updates\"") != NULL, "delta is a Signal K update");
    EXPECT(n > 0 && strstr(buf, "navigation.headingMagnetic") != NULL,
           "carries the heading path");
    EXPECT(n > 0 && strstr(buf, "e2e") != NULL, "carries the configured source label");

    EXPECT(atomic_load(&fs.accepted) >= 1, "the bridge connected to the stream");

    bridge_finish(&r);
    EXPECT(r.rc == 0, "main() returned 0 after bridge_stop");

    close(sink);
    fs_stop(&fs);
    unlink(SK_CONF);
    end(fb);
}

/* A daemon that vanishes and comes back: bridge_stream_ensure()'s reconnect is
 * the property, and it is only reachable through main()'s loop. */
static void test_signalk_reconnects(void)
{
    begin("test_signalk_reconnects");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, SK_SOCK, 200) == 0, "fake stream started");
    atomic_store(&fs.drop_after, 5);          /* hang up after 5 packets */

    int port = 0;
    int sink = udp_bind_ephemeral(&port);

    write_conf(SK_CONF,
               "[imud-signalk]\n"
               "enabled = true\n"
               "socket = \"%s\"\n"
               "rate_hz = 50\n"
               "udp_enabled = true\n"
               "dest_addr = \"127.0.0.1\"\n"
               "dest_port = %d\n"
               "[logging]\nlevel = \"error\"\n",
               SK_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, signalk_main_entry, SK_CONF);

    /* Two accepts means it noticed the drop and came back on its own. */
    bool re = false;
    for (int i = 0; i < 600 && !re; i++) {
        if (atomic_load(&fs.accepted) >= 2) re = true;
        struct timespec t = { 0, 10 * 1000 * 1000 };
        nanosleep(&t, NULL);
    }
    EXPECT(re, "bridge reconnected after the stream dropped");

    char buf[2048];
    EXPECT(udp_recv(sink, buf, sizeof buf) > 0, "still emitting after reconnect");

    bridge_finish(&r);
    close(sink);
    fs_stop(&fs);
    unlink(SK_CONF);
    end(fb);
}

/* enabled = false must exit 0 without touching a socket — the systemd contract
 * (a disabled bridge is not a failed unit). */
static void test_signalk_disabled_exits_clean(void)
{
    begin("test_signalk_disabled_exits_clean");
    int fb = g_fail;

    write_conf(SK_CONF,
               "[imud-signalk]\nenabled = false\n"
               "socket = \"%s\"\n[logging]\nlevel = \"error\"\n", SK_SOCK);

    bridge_run_t r;
    bridge_start(&r, signalk_main_entry, SK_CONF);
    pthread_join(r.tid, NULL);          /* must return on its own, not via stop */
    bridge_reset();

    EXPECT(r.rc == 0, "disabled bridge exits 0");

    unlink(SK_CONF);
    end(fb);
}

/* ── influxdb ────────────────────────────────────────────────────────────── */

#define IX_SOCK "/tmp/imud_e2e_influx.sock"
#define IX_CONF "/tmp/imud_e2e_influx.conf"

static void test_influx_udp_line(void)
{
    begin("test_influx_udp_line");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, IX_SOCK, 200) == 0, "fake stream started");

    int port = 0;
    int sink = udp_bind_ephemeral(&port);

    write_conf(IX_CONF,
               "[imud-influxdb]\n"
               "enabled = true\n"
               "socket = \"%s\"\n"
               "rate_hz = 50\n"
               "measurement = \"e2emeas\"\n"
               "source_label = \"e2esrc\"\n"
               "udp_enabled = true\n"
               "udp_addr = \"127.0.0.1\"\n"
               "udp_port = %d\n"
               "[logging]\nlevel = \"error\"\n",
               IX_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, influx_main_entry, IX_CONF);

    char buf[2048];
    ssize_t n = udp_recv(sink, buf, sizeof buf);
    EXPECT(n > 0, "a line-protocol point arrived");
    EXPECT(n > 0 && strncmp(buf, "e2emeas,", 8) == 0, "configured measurement name");
    EXPECT(n > 0 && strstr(buf, "source=e2esrc") != NULL, "configured source tag");
    EXPECT(n > 0 && strstr(buf, "heading=") != NULL, "carries the heading field");

    bridge_finish(&r);
    EXPECT(r.rc == 0, "main() returned 0");

    close(sink);
    fs_stop(&fs);
    unlink(IX_CONF);
    end(fb);
}

/* The other transport influx_main can be configured for. Its encoder is the
 * same either way; the HTTP request framing around it exists only in main(). */
static void test_influx_http_post(void)
{
    begin("test_influx_http_post");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, IX_SOCK, 200) == 0, "fake stream started");

    /* A listener that answers every request with 204, as InfluxDB does. */
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int on = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = 0;
    EXPECT(bind(srv, (struct sockaddr *)&a, sizeof a) == 0, "http sink bound");
    EXPECT(listen(srv, 4) == 0, "http sink listening");
    socklen_t al = sizeof a;
    getsockname(srv, (struct sockaddr *)&a, &al);
    int port = ntohs(a.sin_port);

    struct timeval tv = { 5, 0 };
    setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    write_conf(IX_CONF,
               "[imud-influxdb]\n"
               "enabled = true\n"
               "socket = \"%s\"\n"
               "rate_hz = 50\n"
               "measurement = \"httpmeas\"\n"
               "udp_enabled = false\n"
               "http_enabled = true\n"
               "http_host = \"127.0.0.1\"\n"
               "http_port = %d\n"
               "http_path = \"/write?db=e2e\"\n"
               "[logging]\nlevel = \"error\"\n",
               IX_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, influx_main_entry, IX_CONF);

    int c = accept(srv, NULL, NULL);
    EXPECT(c >= 0, "the bridge opened an HTTP connection");

    /* Read until the body has arrived, not once: headers and body are two
     * writes and TCP is free to deliver them in separate segments. A single
     * recv() here passed consistently on an idle machine and failed inside
     * dpkg-buildpackage, where the build load changed the timing. */
    char   req[4096] = { 0 };
    size_t got = 0;
    ssize_t n = -1;
    while (c >= 0 && got < sizeof req - 1) {
        ssize_t r = recv(c, req + got, sizeof req - 1 - got, 0);
        if (r <= 0) break;
        got += (size_t)r;
        req[got] = '\0';
        /* Header terminator seen AND something after it = the body landed. */
        char *hdr_end = strstr(req, "\r\n\r\n");
        if (hdr_end && *(hdr_end + 4) != '\0') break;
    }
    if (got > 0) n = (ssize_t)got;

    EXPECT(n > 0, "a request arrived");
    EXPECT(n > 0 && strncmp(req, "POST /write?db=e2e", 18) == 0,
           "POSTs to the configured path");
    EXPECT(n > 0 && strstr(req, "Content-Length:") != NULL, "sends a Content-Length");
    EXPECT(n > 0 && strstr(req, "httpmeas,") != NULL, "body is the line protocol");

    if (c >= 0) {
        const char *resp = "HTTP/1.1 204 No Content\r\nContent-Length: 0\r\n\r\n";
        ssize_t w = send(c, resp, strlen(resp), 0);
        (void)w;
        close(c);
    }

    bridge_finish(&r);
    close(srv);
    fs_stop(&fs);
    unlink(IX_CONF);
    end(fb);
}

/* ── mavlink ─────────────────────────────────────────────────────────────── */

#define MV_SOCK "/tmp/imud_e2e_mavlink.sock"
#define MV_CONF "/tmp/imud_e2e_mavlink.conf"

static void mavlink_conf(int version, const char *sock, int port)
{
    write_conf(MV_CONF,
               "[imud-mavlink]\n"
               "enabled = true\n"
               "socket = \"%s\"\n"
               "version = %d\n"
               "rate_hz = 50\n"
               "send_attitude = true\n"
               "send_attitude_quaternion = true\n"
               "udp_enabled = true\n"
               "udp_addr = \"127.0.0.1\"\n"
               "udp_port = %d\n"
               "[logging]\nlevel = \"error\"\n",
               sock, version, port);
}

/* The v1/v2 start byte is chosen in main()'s config handling, not in the
 * encoder test's golden frames — so which one goes on the wire is exactly the
 * wiring this suite exists to cover. */
static void test_mavlink_udp_frames(void)
{
    begin("test_mavlink_udp_frames");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, MV_SOCK, 200) == 0, "fake stream started");

    int port = 0;
    int sink = udp_bind_ephemeral(&port);
    mavlink_conf(2, MV_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, mavlink_main_entry, MV_CONF);

    char buf[512];
    ssize_t n = udp_recv(sink, buf, sizeof buf);
    EXPECT(n > 0, "a MAVLink frame arrived");
    EXPECT(n > 0 && (unsigned char)buf[0] == 0xFD, "v2 start byte for version = 2");

    bridge_finish(&r);
    close(sink);

    /* Same daemon, version = 1: the other start byte. */
    port = 0;
    sink = udp_bind_ephemeral(&port);
    mavlink_conf(1, MV_SOCK, port);
    bridge_start(&r, mavlink_main_entry, MV_CONF);

    n = udp_recv(sink, buf, sizeof buf);
    EXPECT(n > 0, "a frame arrived under version = 1");
    EXPECT(n > 0 && (unsigned char)buf[0] == 0xFE, "v1 start byte for version = 1");

    bridge_finish(&r);
    close(sink);
    fs_stop(&fs);
    unlink(MV_CONF);
    end(fb);
}

/* mavlink's third sink: a TCP listener GCS clients connect to (QGroundControl
 * tcp:host:5760). netserv is tested on its own; that main() hands it the
 * encoded frames is not. */
static void test_mavlink_tcp_listener(void)
{
    begin("test_mavlink_tcp_listener");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, MV_SOCK, 200) == 0, "fake stream started");

    int port = free_tcp_port();
    write_conf(MV_CONF,
               "[imud-mavlink]\n"
               "enabled = true\n"
               "socket = \"%s\"\n"
               "version = 2\n"
               "rate_hz = 50\n"
               "send_attitude = true\n"
               "udp_enabled = false\n"
               "tcp_enabled = true\n"
               "tcp_bind_addr = \"127.0.0.1\"\n"
               "tcp_port = %d\n"
               "[logging]\nlevel = \"error\"\n",
               MV_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, mavlink_main_entry, MV_CONF);

    /* Connect as a GCS would, retrying while the listener comes up. */
    int c = -1;
    for (int i = 0; i < 300 && c < 0; i++) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in a;
        memset(&a, 0, sizeof a);
        a.sin_family      = AF_INET;
        a.sin_port        = htons((uint16_t)port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) { c = fd; break; }
        close(fd);
        struct timespec t = { 0, 10 * 1000 * 1000 };
        nanosleep(&t, NULL);
    }
    EXPECT(c >= 0, "a client can connect to the TCP listener");

    unsigned char buf[512];
    ssize_t n = -1;
    if (c >= 0) {
        struct timeval tv = { 5, 0 };
        setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        n = recv(c, buf, sizeof buf, 0);
    }
    EXPECT(n > 0, "frames are broadcast to the connected client");
    EXPECT(n > 0 && buf[0] == 0xFD, "v2 frames over TCP too");

    if (c >= 0) close(c);
    bridge_finish(&r);
    fs_stop(&fs);
    unlink(MV_CONF);
    end(fb);
}

/* ── prometheus ──────────────────────────────────────────────────────────── */

#define PR_SOCK "/tmp/imud_e2e_prom.sock"
#define PR_CONF "/tmp/imud_e2e_prom.conf"

/* GET /metrics, with retries while the daemon is still coming up. */
static ssize_t scrape(int port, char *buf, size_t bufsz)
{
    for (int attempt = 0; attempt < 200; attempt++) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) return -1;
        struct sockaddr_in a;
        memset(&a, 0, sizeof a);
        a.sin_family = AF_INET;
        a.sin_port   = htons((uint16_t)port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        struct timeval tv = { 2, 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

        if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) {
            const char *req = "GET /metrics HTTP/1.0\r\n\r\n";
            if (send(fd, req, strlen(req), 0) > 0) {
                size_t got = 0;
                for (;;) {
                    ssize_t r = recv(fd, buf + got, bufsz - 1 - got, 0);
                    if (r <= 0) break;
                    got += (size_t)r;
                    if (got >= bufsz - 1) break;
                }
                buf[got] = '\0';
                close(fd);
                if (got > 0) return (ssize_t)got;
                return 0;
            }
        }
        close(fd);
        struct timespec t = { 0, 10 * 1000 * 1000 };
        nanosleep(&t, NULL);
    }
    return -1;
}

static void test_prometheus_scrape(void)
{
    begin("test_prometheus_scrape");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, PR_SOCK, 200) == 0, "fake stream started");

    int port = free_tcp_port();
    EXPECT(port > 0, "found a free TCP port");

    write_conf(PR_CONF,
               "[imud-prometheus]\n"
               "enabled = true\n"
               "http_enabled = true\n"
               "socket = \"%s\"\n"
               "listen_addr = \"127.0.0.1\"\n"
               "listen_port = %d\n"
               "[logging]\nlevel = \"error\"\n",
               PR_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, prom_main_entry, PR_CONF);

    char buf[16384];
    ssize_t n = scrape(port, buf, sizeof buf);
    EXPECT(n > 0, "the exporter answered a scrape");
    EXPECT(n > 0 && strstr(buf, "200 OK") != NULL, "HTTP 200");
    EXPECT(n > 0 && strstr(buf, "imud_heading_degrees") != NULL, "serves the gauges");
    EXPECT(n > 0 && strstr(buf, "imud_up 1") != NULL, "imud_up 1 with a live stream");

    bridge_finish(&r);
    fs_stop(&fs);
    unlink(PR_CONF);
    end(fb);
}

/* The other half of prom_main's page: no daemon to read. A scraper must still
 * get a valid page saying so, rather than a refused connection. */
static void test_prometheus_up_zero_without_daemon(void)
{
    begin("test_prometheus_up_zero_without_daemon");
    int fb = g_fail;

    unlink(PR_SOCK);                       /* nothing is listening there */
    int port = free_tcp_port();

    write_conf(PR_CONF,
               "[imud-prometheus]\n"
               "enabled = true\n"
               "http_enabled = true\n"
               "socket = \"%s\"\n"
               "listen_addr = \"127.0.0.1\"\n"
               "listen_port = %d\n"
               "[logging]\nlevel = \"error\"\n",
               PR_SOCK, port);

    bridge_run_t r;
    bridge_start(&r, prom_main_entry, PR_CONF);

    char buf[16384];
    ssize_t n = scrape(port, buf, sizeof buf);
    EXPECT(n > 0, "answered a scrape with no daemon present");
    EXPECT(n > 0 && strstr(buf, "imud_up 0") != NULL, "reports imud_up 0");

    bridge_finish(&r);
    unlink(PR_CONF);
    end(fb);
}

/* ── mqtt ────────────────────────────────────────────────────────────────── */

#define MQ_SOCK "/tmp/imud_e2e_mqtt.sock"
#define MQ_CONF "/tmp/imud_e2e_mqtt.conf"

static void test_mqtt_publishes(void)
{
    begin("test_mqtt_publishes");
    int fb = g_fail;

    fakestream_t fs;
    EXPECT(fs_start(&fs, MQ_SOCK, 200) == 0, "fake stream started");

    fakebroker_t br;
    EXPECT(fb_start(&br) == 0, "fake broker listening");

    write_conf(MQ_CONF,
               "[imud-mqtt]\n"
               "enabled = true\n"
               "socket = \"%s\"\n"
               "broker_enabled = true\n"
               "broker_addr = \"127.0.0.1\"\n"
               "broker_port = %d\n"
               "topic_prefix = \"e2etop\"\n"
               "rate_hz = 50\n"
               "ha_discovery = false\n"
               "[logging]\nlevel = \"error\"\n",
               MQ_SOCK, br.port);

    bridge_run_t r;
    bridge_start(&r, mqtt_main_entry, MQ_CONF);

    EXPECT(fb_wait_topic(&br, "e2etop", 5000), "published under the configured prefix");
    EXPECT(atomic_load(&br.connects) >= 1, "completed an MQTT CONNECT");
    EXPECT(fb_wait_topic(&br, "heading", 5000), "published a heading topic");

    bridge_finish(&r);
    fb_stop(&br);
    fs_stop(&fs);
    unlink(MQ_CONF);
    end(fb);
}

int main(void)
{
    test_signalk_udp_delta();
    test_signalk_reconnects();
    test_signalk_disabled_exits_clean();
    test_influx_udp_line();
    test_influx_http_post();
    test_mavlink_udp_frames();
    test_mavlink_tcp_listener();
    test_prometheus_scrape();
    test_prometheus_up_zero_without_daemon();
    test_mqtt_publishes();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
