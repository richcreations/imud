/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_bridge.c — unit tests for the shared bridge scaffolding (src/bridge.c
 * + src/sdnotify.c): the emit-tick timespec math (period/wait/due/advance/
 * earlier — the pieces where an off-by-one would change every bridge's emit
 * cadence), the CLI matrix, config load / reload / disabled flows, signal
 * flag routing, sd_notify datagram delivery over a test-bound NOTIFY_SOCKET,
 * and the stream connect/drop helpers against a local AF_UNIX listener.
 *
 * Portable — builds and runs on the macOS dev box too.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>

#include "bridge.h"
#include "sdnotify.h"
#include "log.h"

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

static const bridge_info_t TBI = {
    .prog         = "imud-test",
    .tag          = "test",
    .section      = "imud-test",
    .default_conf = "/etc/imud/imud-test.conf",
    .usage_desc   = "  Test bridge.\n",
};

/* ── Tick timing math ────────────────────────────────────────────────────── */

static void test_period_ns(void)
{
    begin("test_period_ns");
    int fb = g_fail;

    EXPECT(bridge_period_ns(10, 100000000L) == 100000000L, "10 Hz = 100 ms");
    EXPECT(bridge_period_ns(1000, 0)        == 1000000L,   "1 kHz = 1 ms");
    EXPECT(bridge_period_ns(0, 200000000L)  == 200000000L, "0 → fallback");
    EXPECT(bridge_period_ns(-5, 42L)        == 42L,        "negative → fallback");

    end(fb);
}

static void test_wait_ms(void)
{
    begin("test_wait_ms");
    int fb = g_fail;

    struct timespec now  = { 100, 0 };
    struct timespec next = { 100, 250000000L };
    EXPECT(bridge_wait_ms(&now, &next) == 250, "exact 250 ms");

    next.tv_nsec = 250000001L;                    /* sub-ms remainder */
    EXPECT(bridge_wait_ms(&now, &next) == 251, "rounds up, no busy-spin");

    next.tv_nsec = 1L;
    EXPECT(bridge_wait_ms(&now, &next) == 1, "1 ns still waits 1 ms");

    next = now;
    EXPECT(bridge_wait_ms(&now, &next) == 0, "due now → 0");

    struct timespec past = { 99, 900000000L };
    EXPECT(bridge_wait_ms(&now, &past) == 0, "past deadline clamps to 0");

    /* Cross-second subtraction gives a negative tv_nsec delta. */
    struct timespec a = { 100, 900000000L }, b = { 101, 100000000L };
    EXPECT(bridge_wait_ms(&a, &b) == 200, "cross-second delta");

    end(fb);
}

static void test_due(void)
{
    begin("test_due");
    int fb = g_fail;

    struct timespec next = { 100, 500 };
    struct timespec t;

    t = (struct timespec){ 100, 499 };
    EXPECT(!bridge_due(&t, &next), "just before");
    t = (struct timespec){ 100, 500 };
    EXPECT(bridge_due(&t, &next),  "exactly due (>=)");
    t = (struct timespec){ 100, 501 };
    EXPECT(bridge_due(&t, &next),  "just after");
    t = (struct timespec){ 101, 0 };
    EXPECT(bridge_due(&t, &next),  "next second");
    t = (struct timespec){ 99, 999999999L };
    EXPECT(!bridge_due(&t, &next), "previous second");

    end(fb);
}

static void test_advance(void)
{
    begin("test_advance");
    int fb = g_fail;

    /* Single step, no carry. */
    struct timespec next = { 100, 100000000L };
    struct timespec now  = { 100, 150000000L };
    bridge_advance(&next, &now, 100000000L);
    EXPECT(next.tv_sec == 100 && next.tv_nsec == 200000000L, "single step");

    /* Nanosecond carry into tv_sec. */
    next = (struct timespec){ 100, 950000000L };
    now  = (struct timespec){ 100, 960000000L };
    bridge_advance(&next, &now, 100000000L);
    EXPECT(next.tv_sec == 101 && next.tv_nsec == 50000000L, "nsec carry");

    /* Stall: skip all missed ticks, land strictly after now. */
    next = (struct timespec){ 100, 0 };
    now  = (struct timespec){ 103, 250000000L };
    bridge_advance(&next, &now, 100000000L);
    EXPECT(next.tv_sec == 103 && next.tv_nsec == 300000000L, "skips missed ticks");

    /* Landing exactly on now is not enough (<= keeps advancing). */
    next = (struct timespec){ 100, 0 };
    now  = (struct timespec){ 100, 100000000L };
    bridge_advance(&next, &now, 100000000L);
    EXPECT(next.tv_sec == 100 && next.tv_nsec == 200000000L,
           "exact-boundary tick advances past now");

    end(fb);
}

static void test_earlier(void)
{
    begin("test_earlier");
    int fb = g_fail;

    struct timespec a = { 100, 0 }, b = { 101, 0 };
    EXPECT(bridge_earlier(&a, &b) == &a, "earlier second");
    EXPECT(bridge_earlier(&b, &a) == &a, "order-independent");

    a = (struct timespec){ 100, 100 };
    b = (struct timespec){ 100, 200 };
    EXPECT(bridge_earlier(&a, &b) == &a, "earlier nsec");

    b = a;
    EXPECT(bridge_earlier(&a, &b) == &b, "equal → second argument");

    end(fb);
}

/* ── CLI matrix ──────────────────────────────────────────────────────────── */

static void test_parse_cli(void)
{
    begin("test_parse_cli");
    int fb = g_fail;
    char path[256];

    char *argv0[] = { "imud-test", NULL };
    EXPECT(bridge_parse_cli(1, argv0, &TBI, path, sizeof path) == 0, "no args → run");
    EXPECT(strcmp(path, "/etc/imud/imud-test.conf") == 0, "default path");

    char *argv1[] = { "imud-test", "--config", "/tmp/x.conf", NULL };
    EXPECT(bridge_parse_cli(3, argv1, &TBI, path, sizeof path) == 0, "--config → run");
    EXPECT(strcmp(path, "/tmp/x.conf") == 0, "--config path taken");

    char *argv2[] = { "imud-test", "--version", NULL };
    EXPECT(bridge_parse_cli(2, argv2, &TBI, path, sizeof path) == 1, "--version handled");

    char *argv3[] = { "imud-test", "-h", NULL };
    EXPECT(bridge_parse_cli(2, argv3, &TBI, path, sizeof path) == 1, "-h handled");

    char *argv4[] = { "imud-test", "--bogus", NULL };
    EXPECT(bridge_parse_cli(2, argv4, &TBI, path, sizeof path) == -1, "unknown → error");

    /* --config without a value falls through to unknown-option handling. */
    char *argv5[] = { "imud-test", "--config", NULL };
    EXPECT(bridge_parse_cli(2, argv5, &TBI, path, sizeof path) == -1,
           "--config w/o value → error");

    end(fb);
}

/* ── Config load / reload ────────────────────────────────────────────────── */

static void write_file(const char *path, const char *text)
{
    FILE *f = fopen(path, "w");
    if (f) { fputs(text, f); fclose(f); }
}

static void test_load_config(void)
{
    begin("test_load_config");
    int fb = g_fail;
    char good[128], bad[128];
    snprintf(good, sizeof good, "/tmp/imud_tb_good_%d.conf", (int)getpid());
    snprintf(bad,  sizeof bad,  "/tmp/imud_tb_bad_%d.conf",  (int)getpid());
    write_file(good, "[stream]\nrate_hz = 25\n");
    write_file(bad,  "[imu]\nodr_hz = banana\n");

    imud_config_t cfg;
    EXPECT(bridge_load_config(&TBI, good, &cfg) == 0, "good file loads");
    EXPECT(cfg.stream_rate_hz == 25, "value applied");

    /* Missing file keeps defaults and is not an error (matches the daemon). */
    EXPECT(bridge_load_config(&TBI, "/nonexistent/imud.conf", &cfg) == 0,
           "missing file → defaults, still starts");

    /* Parse error refuses to start (error lines above are expected output). */
    EXPECT(bridge_load_config(&TBI, bad, &cfg) == -1, "parse error refuses");

    /* Reload: nothing pending → 0. */
    bridge_reload = 0;
    imud_config_t nc;
    EXPECT(bridge_reload_begin(&TBI, good, &nc) == 0, "no reload pending");

    /* Pending + good file → 1, flag consumed, values loaded. */
    bridge_reload = 1;
    EXPECT(bridge_reload_begin(&TBI, good, &nc) == 1, "reload succeeds");
    EXPECT(bridge_reload == 0, "reload flag consumed");
    EXPECT(nc.stream_rate_hz == 25, "reload value applied");

    /* Pending + bad file → 0 (keep current), flag still consumed. */
    bridge_reload = 1;
    EXPECT(bridge_reload_begin(&TBI, bad, &nc) == 0, "bad reload keeps current");
    EXPECT(bridge_reload == 0, "flag consumed on failure too");

    unlink(good);
    unlink(bad);
    end(fb);
}

/* ── Signals ─────────────────────────────────────────────────────────────── */

static void test_signals(void)
{
    begin("test_signals");
    int fb = g_fail;

    bridge_stop = 0;
    bridge_reload = 0;
    bridge_install_signals();

    raise(SIGHUP);
    EXPECT(bridge_reload == 1 && bridge_stop == 0, "SIGHUP → reload only");
    bridge_reload = 0;

    raise(SIGTERM);
    EXPECT(bridge_stop == 1 && bridge_reload == 0, "SIGTERM → stop only");
    bridge_stop = 0;

    raise(SIGINT);
    EXPECT(bridge_stop == 1, "SIGINT → stop");
    bridge_stop = 0;

    end(fb);
}

/* ── sd_notify delivery ──────────────────────────────────────────────────── */

static void test_sd_notify(void)
{
    begin("test_sd_notify");
    int fb = g_fail;

    char path[64];
    snprintf(path, sizeof path, "/tmp/imud_tb_sd_%d", (int)getpid());
    unlink(path);

    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
    EXPECT(bind(fd, (struct sockaddr *)&addr, sizeof addr) == 0, "bind notify sock");

    struct timeval tv = { 2, 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    setenv("NOTIFY_SOCKET", path, 1);
    sd_notify_msg("READY=1");

    char buf[64] = {0};
    ssize_t n = recv(fd, buf, sizeof buf - 1, 0);
    EXPECT(n == 7 && strcmp(buf, "READY=1") == 0, "READY=1 delivered");

    /* Without NOTIFY_SOCKET it must be a silent no-op. */
    unsetenv("NOTIFY_SOCKET");
    sd_notify_msg("WATCHDOG=1");
    EXPECT(1, "no-op without NOTIFY_SOCKET");

    close(fd);
    unlink(path);
    end(fb);
}

/* ── Stream connect / drop ───────────────────────────────────────────────── */

static void test_stream_ensure(void)
{
    begin("test_stream_ensure");
    int fb = g_fail;

    char path[64];
    snprintf(path, sizeof path, "/tmp/imud_tb_st_%d", (int)getpid());
    unlink(path);

    /* No listener: 0, stream stays NULL (retry_sleep_s = 0 → no sleep). */
    imud_t *stream = NULL;
    EXPECT(bridge_stream_ensure(&stream, path, "test", 0) == 0, "unavailable → 0");
    EXPECT(stream == NULL, "stream stays NULL");

    /* Local AF_UNIX listener: fresh connect → 2, already connected → 1. */
    int lfd = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
    EXPECT(bind(lfd, (struct sockaddr *)&addr, sizeof addr) == 0 &&
           listen(lfd, 1) == 0, "listener up");

    EXPECT(bridge_stream_ensure(&stream, path, "test", 0) == 2, "fresh connect → 2");
    EXPECT(stream != NULL, "stream handle set");
    EXPECT(bridge_stream_ensure(&stream, path, "test", 0) == 1, "already connected → 1");

    bridge_stream_drop(&stream, "test");
    EXPECT(stream == NULL, "drop NULLs the handle");

    close(lfd);
    unlink(path);
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("test_bridge — shared bridge scaffolding\n");

    test_period_ns();
    test_wait_ms();
    test_due();
    test_advance();
    test_earlier();
    test_parse_cli();
    test_load_config();
    test_signals();
    test_sd_notify();
    test_stream_ensure();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
