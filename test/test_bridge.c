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
#include <fcntl.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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

    /*
     * Stream contract, shared by all five bridges through this one parser:
     * --help is requested output (stdout, rc 1 → exit 0) and an unknown
     * option is a diagnostic (stderr, rc -1 → exit 1).  help2man reads stdout
     * and expects exit 0, so a usage string on the wrong stream produces a man
     * page with an empty OPTIONS section.  See the contract note in cli.c.
     */
    {
        char opath[64], epath[64], obuf[4096], ebuf[4096];
        snprintf(opath, sizeof opath, "/tmp/imud_tb_o_%d.txt", (int)getpid());
        snprintf(epath, sizeof epath, "/tmp/imud_tb_e_%d.txt", (int)getpid());

        for (int help = 1; help >= 0; help--) {
            char *v[] = { "imud-test", help ? (char *)"--help"
                                            : (char *)"--bogus", NULL };
            fflush(stdout); fflush(stderr);
            int so = dup(STDOUT_FILENO), se = dup(STDERR_FILENO);
            int fo = open(opath, O_WRONLY | O_CREAT | O_TRUNC, 0600);
            int fe = open(epath, O_WRONLY | O_CREAT | O_TRUNC, 0600);
            if (fo >= 0) { dup2(fo, STDOUT_FILENO); close(fo); }
            if (fe >= 0) { dup2(fe, STDERR_FILENO); close(fe); }

            int rc = bridge_parse_cli(2, v, &TBI, path, sizeof path);

            fflush(stdout); fflush(stderr);
            dup2(so, STDOUT_FILENO); close(so);
            dup2(se, STDERR_FILENO); close(se);

            obuf[0] = ebuf[0] = '\0';
            FILE *f;
            if ((f = fopen(opath, "r"))) {
                obuf[fread(obuf, 1, sizeof obuf - 1, f)] = '\0'; fclose(f);
            }
            if ((f = fopen(epath, "r"))) {
                ebuf[fread(ebuf, 1, sizeof ebuf - 1, f)] = '\0'; fclose(f);
            }
            unlink(opath); unlink(epath);

            if (help) {
                EXPECT(rc == 1, "bridge --help → rc 1");
                EXPECT(strstr(obuf, "Usage: imud-test") != NULL,
                       "bridge --help writes usage to stdout");
                EXPECT(ebuf[0] == '\0',
                       "bridge --help writes NOTHING to stderr");
            } else {
                EXPECT(rc == -1, "bridge unknown option → rc -1");
                EXPECT(strstr(ebuf, "Usage: imud-test") != NULL,
                       "bridge error writes usage to stderr");
                EXPECT(obuf[0] == '\0',
                       "bridge error writes NOTHING to stdout");
            }
        }
    }

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

    /* Present-but-unreadable is NOT the missing-file case: the bridge configs
     * install 0640 root:imud, so a bridge outside the group would otherwise
     * run on defaults and exit claiming it was disabled in config.
     * Root ignores the mode bits, so this can only be checked unprivileged. */
    if (geteuid() != 0) {
        char noperm[128];
        snprintf(noperm, sizeof noperm, "/tmp/imud_tb_noperm_%d.conf",
                 (int)getpid());
        write_file(noperm, "[stream]\nrate_hz = 25\n");
        if (chmod(noperm, 0) == 0)
            EXPECT(bridge_load_config(&TBI, noperm, &cfg) == -1,
                   "unreadable file refuses to start");
        unlink(noperm);
    } else {
        printf("  (skipped unreadable-config case: running as root)\n");
    }

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

/* ── UDP output, blocking write, and the remaining lifecycle helpers ─────── */

/*
 * bridge_open_udp is the one every bridge sends through, and bridge_write_all
 * is what the HTTP/socket bridges push bytes with; neither had any coverage.
 * Both are exercised against real sockets rather than mocked.
 */
static void test_open_udp(void)
{
    begin("test_open_udp");
    int fb = g_fail;

    /* Bind a receiver first so there is something to send to. */
    int rx = socket(AF_INET, SOCK_DGRAM, 0);
    EXPECT(rx >= 0, "receiver socket");
    struct sockaddr_in ra;
    memset(&ra, 0, sizeof ra);
    ra.sin_family      = AF_INET;
    ra.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ra.sin_port        = 0;
    EXPECT(bind(rx, (struct sockaddr *)&ra, sizeof ra) == 0, "receiver bound");
    socklen_t ralen = sizeof ra;
    getsockname(rx, (struct sockaddr *)&ra, &ralen);
    int port = ntohs(ra.sin_port);
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    struct sockaddr_storage dst;
    socklen_t dlen = 0;
    int fd = bridge_open_udp("127.0.0.1", port, "test", &dst, &dlen);
    EXPECT(fd >= 0, "bridge_open_udp resolves and opens");
    EXPECT(dlen > 0, "destination length filled in");

    if (fd >= 0) {
        const char *msg = "imud-test-datagram";
        ssize_t s = sendto(fd, msg, strlen(msg), 0,
                           (struct sockaddr *)&dst, dlen);
        EXPECT(s == (ssize_t)strlen(msg), "datagram sent to the resolved dest");

        char got[64];
        ssize_t n = recv(rx, got, sizeof got - 1, 0);
        EXPECT(n == (ssize_t)strlen(msg), "receiver got it");
        if (n > 0) { got[n] = '\0'; EXPECT(strcmp(got, msg) == 0, "bytes match"); }
        close(fd);
    }

    /* A name that cannot resolve must fail cleanly, not return a live fd.
     * ".invalid" is reserved by RFC 2606 precisely so it never resolves. */
    struct sockaddr_storage bad_dst;
    socklen_t bad_len = 0;
    EXPECT(bridge_open_udp("no-such-host.invalid", 1234, "test",
                           &bad_dst, &bad_len) < 0,
           "unresolvable host → -1");

    close(rx);
    end(fb);
}

static void test_write_all(void)
{
    begin("test_write_all");
    int fb = g_fail;

    int sv[2];
    EXPECT(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == 0, "socketpair");

    const char *payload = "line one\nline two\n";
    int n = (int)strlen(payload);
    EXPECT(bridge_write_all(sv[0], payload, n) == 0, "full write returns 0");

    char got[64];
    ssize_t r = recv(sv[1], got, sizeof got - 1, 0);
    EXPECT(r == n, "every byte arrived");
    if (r > 0) { got[r] = '\0'; EXPECT(strcmp(got, payload) == 0, "content matches"); }

    /* Peer gone: the write must report failure rather than spin.  SIGPIPE is
     * ignored by bridge_install_signals, which test_signals already ran. */
    close(sv[1]);
    EXPECT(bridge_write_all(sv[0], payload, n) == -1,
           "write to a closed peer → -1");
    close(sv[0]);
    end(fb);
}

/* The sleep must return early when a signal flag is raised, or a bridge would
 * take its full retry interval to notice SIGTERM. */
static void test_sleep_interruptible(void)
{
    begin("test_sleep_interruptible");
    int fb = g_fail;

    bridge_stop = 1;
    time_t t0 = time(NULL);
    bridge_sleep_interruptible(30);
    EXPECT(difftime(time(NULL), t0) < 2.0, "stop flag cuts the sleep short");
    bridge_stop = 0;

    bridge_reload = 1;
    t0 = time(NULL);
    bridge_sleep_interruptible(30);
    EXPECT(difftime(time(NULL), t0) < 2.0, "reload flag cuts the sleep short");
    bridge_reload = 0;

    /* Zero and negative durations must not sleep at all. */
    t0 = time(NULL);
    bridge_sleep_interruptible(0);
    bridge_sleep_interruptible(-1);
    EXPECT(difftime(time(NULL), t0) < 2.0, "0 and negative return immediately");
    end(fb);
}

/*
 * bridge_exit_disabled does NOT exit despite the name — it logs and signals
 * READY=1 so systemd records a clean start instead of restart-looping a
 * deliberately disabled bridge.  The caller does the exiting.  Pinning that
 * here because the name invites someone to "fix" it into an exit() call.
 */
static void test_disabled_and_reload_done(void)
{
    begin("test_disabled_and_reload_done");
    int fb = g_fail;

    unsetenv("NOTIFY_SOCKET");     /* sd_notify_msg no-ops without it */
    bridge_exit_disabled(&TBI);
    EXPECT(1, "bridge_exit_disabled returns to its caller");

    bridge_reload_done(&TBI);
    EXPECT(1, "bridge_reload_done returns");
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
    test_open_udp();
    test_write_all();
    test_sleep_interruptible();
    test_disabled_and_reload_done();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
