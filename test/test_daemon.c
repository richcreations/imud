/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_daemon.c — the daemon itself, start to shutdown (audit L2, L1)
 *
 * src/main.c is 725 lines that no test binary linked: thread lifecycle, the
 * status-socket responder, the SIGHUP reload block, and shutdown ordering. The
 * audit named the reload block and the responder as its highest-value targets,
 * and listed the whole thing under "nothing has run on a Pi".
 *
 * It needs no Pi and no sensor. driver = "sim" skips the GPIO chip entirely
 * (int_gpio = 0), and although imu.c opens i2c_bus unconditionally even in sim
 * mode, /dev/null opens O_RDWR and the sim driver's probe/reset/init ignore the
 * fd. So the real main() runs here, on a thread, with everything else real.
 *
 * SIGNALS: main() blocks SIGTERM/SIGINT/SIGHUP and services them with sigwait
 * on its own thread. Blocking them in THIS thread first is what makes that
 * work — the mask is inherited, so the daemon thread's sigwait is the only
 * consumer and pthread_kill lands there deterministically.
 *
 * PATHS: the Makefile compiles this copy of main.c with PID_FILE and
 * STATUS_SOCK redirected into /tmp. The alternative was a suite that unlinks
 * /run/imud/imud.sock out from under a live daemon when run on the machine it
 * is serving. The tradeoff is that the default path constants are not
 * themselves exercised here.
 *
 * THREAD FAILURE (audit N6): the Makefile links this suite with
 * -Wl,--wrap=pthread_create, so __wrap_pthread_create below can fail one named
 * thread and leave every other one alone. Without that seam main()'s four
 * warn-and-continue output arms and its one fatal one are unreachable from a
 * test — nothing an outside process can do makes pthread_create return EAGAIN.
 *
 * Linux-only, like test_concurrency: it links the daemon objects and -lgpiod.
 */

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include "types.h"
#include "output.h"     /* the three output thread entry points, for the wrap */

int main_entry(int argc, char **argv);

/* ── the pthread_create seam ─────────────────────────────────────────────── */

/* Typed as the entry point rather than void *: casting between function and
 * object pointers is not something C guarantees, and there is no reason to. */
typedef void *(*thread_fn_t)(void *);

/* NULL — the default and what every other case in this suite runs with — means
 * pass every thread straight through. */
static thread_fn_t _Atomic g_fail_fn;

int __real_pthread_create(pthread_t *restrict, const pthread_attr_t *restrict,
                          thread_fn_t, void *restrict);

int __wrap_pthread_create(pthread_t *restrict tid,
                          const pthread_attr_t *restrict attr,
                          thread_fn_t fn, void *restrict arg)
{
    thread_fn_t target = atomic_load(&g_fail_fn);
    if (target != NULL && fn == target)
        return EAGAIN;   /* what the real one returns under resource limits */
    return __real_pthread_create(tid, attr, fn, arg);
}

/* Kept in step with the -D flags in the Makefile's test_daemon rule.
 *
 * /tmp, not the build directory, and not for tidiness: the daemon chmods its
 * sockets to 0660 (bind_unix_mode in include/fileio.h), and chmod() on a socket
 * returns EINVAL on a virtiofs bind mount — which is exactly what the repo is
 * when the suite runs in the dev container. test_stream.c already binds in
 * /tmp for the same reason. */
#define T_PID_FILE    "/tmp/imud_e2e_daemon.pid"
#define T_STATUS_SOCK "/tmp/imud_e2e_daemon.sock"
#define T_STREAM_SOCK "/tmp/imud_e2e_daemon_stream.sock"
#define T_CONF        "/tmp/imud_e2e_daemon.conf"

/* Ports the N6 cases need a listener or a receiver on, in the same 27xxx block
 * as the NMEA dest_port the lifecycle test already uses. */
#define T_NMEA_TCP_PORT 27111
#define T_HIRATE_PORT   27112

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

static void msleep(int ms)
{
    struct timespec t = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&t, NULL);
}

/* The optional outputs only the N6 cases want: each one is a listener or a
 * destination the test can then observe from outside the daemon. */
typedef struct {
    int  nmea_rate_hz;
    int  imu_odr_hz;
    bool nmea_tcp;   /* [nmea] tcp_enabled  → a listener on T_NMEA_TCP_PORT */
    bool hirate;     /* [highrate] enabled  → UDP to 127.0.0.1:T_HIRATE_PORT */
} conf_opt_t;

static void write_conf_opt(conf_opt_t o)
{
    FILE *f = fopen(T_CONF, "w");
    if (!f) { perror("fopen"); exit(1); }
    fprintf(f,
        "[device]\n"
        "i2c_bus = \"/dev/null\"\n"
        "[imu]\n"
        "driver = \"sim\"\n"
        "i2c_addr = 0x00\n"
        "int_gpio = 0\n"
        "odr_hz = %d\n"
        "[mag]\n"
        "driver = \"sim\"\n"
        "i2c_addr = 0x00\n"
        "int_gpio = 0\n"
        "odr_hz = 100\n"
        "set_period_s = 0.0\n"
        "[nmea]\n"
        "enabled = true\n"
        "rate_hz = %d\n"
        "dest_addr = \"127.0.0.1\"\n"
        "dest_port = 27110\n"
        "tcp_enabled = %s\n"
        "tcp_bind_addr = \"127.0.0.1\"\n"
        "tcp_port = %d\n"
        "[highrate]\n"
        "enabled = %s\n"
        "rate_hz = 50\n"
        "dest_addr = \"127.0.0.1\"\n"
        "dest_port = %d\n"
        "[stream]\n"
        "enabled = true\n"
        "socket = \"%s\"\n"
        "rate_hz = 50\n"
        "[logging]\n"
        "level = \"error\"\n",
        o.imu_odr_hz, o.nmea_rate_hz,
        o.nmea_tcp ? "true" : "false", T_NMEA_TCP_PORT,
        o.hirate   ? "true" : "false", T_HIRATE_PORT,
        T_STREAM_SOCK);
    fclose(f);
}

/* The plain form: no optional outputs. nmea rate_hz is a [hot] key and imu
 * odr_hz is a [restart] key; both are printed by the status report, which is
 * what makes the reload contract observable from outside the process. */
static void write_conf(int nmea_rate_hz, int imu_odr_hz)
{
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = nmea_rate_hz,
                                 .imu_odr_hz   = imu_odr_hz });
}

/* Connect to an AF_UNIX path, retrying while the daemon starts up. */
static int connect_unix(const char *path, int timeout_ms)
{
    for (int waited = 0; waited < timeout_ms; waited += 20) {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) return -1;
        struct sockaddr_un a;
        memset(&a, 0, sizeof a);
        a.sun_family = AF_UNIX;
        snprintf(a.sun_path, sizeof a.sun_path, "%s", path);
        if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) return fd;
        close(fd);
        msleep(20);
    }
    return -1;
}

/* One status report, as imud-status would fetch it. */
static bool fetch_status(char *buf, size_t bufsz)
{
    int fd = connect_unix(T_STATUS_SOCK, 5000);
    if (fd < 0) return false;
    size_t got = 0;
    for (;;) {
        ssize_t r = recv(fd, buf + got, bufsz - 1 - got, 0);
        if (r <= 0) break;
        got += (size_t)r;
        if (got >= bufsz - 1) break;
    }
    buf[got] = '\0';
    close(fd);
    return got > 0;
}

/* `done` is how a caller can tell "still running" from "returned" without
 * pthread_kill(tid, 0), which is undefined once the thread has exited — the id
 * may already have been reused.  _Atomic per the project's rule for every
 * cross-thread flag; its release also makes `rc` safe to read after it reads 1. */
typedef struct { int rc; pthread_t tid; _Atomic int done; } daemon_run_t;

static void *daemon_thread(void *arg)
{
    daemon_run_t *d = (daemon_run_t *)arg;
    char *argv[] = { (char *)"imud", (char *)"--config", (char *)T_CONF, NULL };
    d->rc = main_entry(3, argv);
    d->done = 1;
    return NULL;
}

/* The whole lifecycle in one run: start with no hardware, serve the stream and
 * the status socket, honour SIGHUP's hot/restart split, shut down clean. */
static void test_daemon_lifecycle(void)
{
    begin("test_daemon_lifecycle");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    unlink(T_PID_FILE);
    write_conf(10, 833);

    /* Block first, then spawn: the daemon's sigwait must be the only consumer. */
    sigset_t set, old;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, &old);

    daemon_run_t d = { .rc = -999 };
    EXPECT(pthread_create(&d.tid, NULL, daemon_thread, &d) == 0, "daemon thread started");

    /* ── it comes up with no sensor and no GPIO ───────────────────────────── */
    int sfd = connect_unix(T_STREAM_SOCK, 10000);
    EXPECT(sfd >= 0, "stream socket accepts a subscriber");

    /* ── and serves real packets down it ──────────────────────────────────── */
    imu_packet_t pkt;
    size_t got = 0;
    while (sfd >= 0 && got < sizeof pkt) {
        ssize_t r = recv(sfd, (char *)&pkt + got, sizeof pkt - got, 0);
        if (r <= 0) break;
        got += (size_t)r;
    }
    EXPECT(got == sizeof pkt, "a full 276-byte frame arrived");
    EXPECT(got == sizeof pkt && pkt.magic == IMUD_MAGIC, "with the right magic");

    /* ── the status responder answers ─────────────────────────────────────── */
    char rep[8192];
    EXPECT(fetch_status(rep, sizeof rep), "status socket answers");
    EXPECT(strstr(rep, "IMU ODR:") != NULL, "report contains the ODR line");
    EXPECT(strstr(rep, "833") != NULL, "reports the configured 833 Hz");
    EXPECT(strstr(rep, "NMEA out:      10 Hz") != NULL ||
           strstr(rep, "10 Hz") != NULL, "reports the configured NMEA rate");

    /* ── the PID file exists while it runs ────────────────────────────────── */
    struct stat st;
    EXPECT(stat(T_PID_FILE, &st) == 0, "pid file written");

    /* ── SIGHUP: the hot key takes, the restart-only key does not ─────────── */
    write_conf(7, 416);                       /* nmea 10→7 hot, odr 833→416 not */
    EXPECT(pthread_kill(d.tid, SIGHUP) == 0, "SIGHUP delivered to the daemon");
    msleep(1500);                             /* reload is handled by sigwait */

    char rep2[8192];
    EXPECT(fetch_status(rep2, sizeof rep2), "status socket answers after reload");
    EXPECT(strstr(rep2, "7 Hz") != NULL, "hot key applied: NMEA rate is now 7 Hz");
    EXPECT(strstr(rep2, "833") != NULL,
           "restart-only key NOT applied: IMU ODR still 833 Hz");
    EXPECT(strstr(rep2, "416") == NULL, "the new ODR did not take effect live");

    if (sfd >= 0) close(sfd);

    /* ── SIGTERM: clean shutdown, and it cleans up after itself ───────────── */
    EXPECT(pthread_kill(d.tid, SIGTERM) == 0, "SIGTERM delivered");
    pthread_join(d.tid, NULL);
    EXPECT(d.rc == 0, "daemon exits 0");
    EXPECT(stat(T_PID_FILE, &st) != 0, "pid file removed on shutdown");
    EXPECT(stat(T_STATUS_SOCK, &st) != 0, "status socket unlinked on shutdown");

    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    unlink(T_STREAM_SOCK);
    end(fb);
}

/* A config file that does not parse must refuse to start, not start with
 * defaults — the daemon's stated contract for an existing but bad file. */
static void test_daemon_refuses_bad_config(void)
{
    begin("test_daemon_refuses_bad_config");
    int fb = g_fail;

    FILE *f = fopen(T_CONF, "w");
    fprintf(f, "[imu]\nodr_hz = \"not-a-number\"\ndriver = \"sim\"\n"
               "[logging]\nlevel = \"error\"\n");
    fclose(f);

    char *argv[] = { (char *)"imud", (char *)"--config", (char *)T_CONF, NULL };
    int rc = main_entry(3, argv);
    EXPECT(rc != 0, "refuses to start on a bad config");

    unlink(T_CONF);
    end(fb);
}

/*
 * The end-to-end half of the same contract, for an over-long string value.
 *
 * This is the finding stated as a behaviour rather than a return code: a
 * [stream] socket one character too long used to be truncated to a
 * *different, perfectly valid* path, which the daemon then bound and served,
 * while every bridge and libimud client connected to the path written in the
 * config file and found nothing there.  So the assertion that matters is not
 * only that the start is refused — it is that the truncated path is not on
 * disk afterwards.
 */
static void test_daemon_refuses_overlong_socket(void)
{
    begin("test_daemon_refuses_overlong_socket");
    int fb = g_fail;

    /* stream_socket is char[108]: 107 usable, so 108 is one past. */
    char sock[160];
    int n = snprintf(sock, sizeof sock, "/tmp/imud_e2e_long_");
    memset(sock + n, 'x', 108 - (size_t)n);
    sock[108] = '\0';

    FILE *f = fopen(T_CONF, "w");
    if (!f) { perror("fopen"); exit(1); }
    fprintf(f,
        "[device]\ni2c_bus = \"/dev/null\"\n"
        "[imu]\ndriver = \"sim\"\nint_gpio = 0\n"
        "[mag]\ndriver = \"sim\"\nint_gpio = 0\nset_period_s = 0.0\n"
        "[nmea]\nenabled = false\n"
        "[highrate]\nenabled = false\n"
        "[stream]\nenabled = true\nsocket = \"%s\"\n"
        "[logging]\nlevel = \"error\"\n", sock);
    fclose(f);

    char trunc[128];
    snprintf(trunc, sizeof trunc, "%.107s", sock);
    unlink(trunc);

    /*
     * Run it on a thread rather than calling main_entry() straight.  A
     * regression here does not return a bad code — it *succeeds*, binds the
     * truncated path and blocks in the daemon's main loop forever, so a
     * direct call would hang the suite instead of failing it.  Verified by
     * reverting both halves of the fix: the daemon came up and
     * `ls` showed a live socket at the 107-character path while it ran.
     */
    sigset_t set, old;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, &old);

    daemon_run_t d = { .rc = -999 };
    EXPECT(pthread_create(&d.tid, NULL, daemon_thread, &d) == 0,
           "daemon thread started");

    bool exited = false;
    for (int waited = 0; waited < 5000; waited += 50) {
        if (atomic_load(&d.done)) { exited = true; break; }
        if (access(trunc, F_OK) == 0) break;   /* it bound the wrong path */
        msleep(50);
    }

    EXPECT(access(trunc, F_OK) != 0, "did not bind the truncated path");
    EXPECT(exited, "refuses to start on an over-long socket path");

    if (!exited) pthread_kill(d.tid, SIGTERM);  /* don't wedge the suite */
    pthread_join(d.tid, NULL);
    EXPECT(d.rc != 0, "exits non-zero");

    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(trunc);
    unlink(T_CONF);
    end(fb);
}

/* ── audit N6 — a thread that fails to start leaves nothing bound ────────── */

/* One connect attempt, no retry — the opposite of connect_unix, whose timeout
 * is a budget for waiting on a socket that is expected to appear. Here the
 * question is whether something is listening *right now*. */
static int try_connect_unix(const char *path)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un a;
    memset(&a, 0, sizeof a);
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", path);
    if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) return fd;
    close(fd);
    return -1;
}

/* Connect to 127.0.0.1:port. Returns the fd, or -errno on failure, so a caller
 * can tell ECONNREFUSED (what it wants) from ETIMEDOUT (what it does not). */
static int connect_tcp(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -errno;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port   = htons((uint16_t)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) return fd;
    int e = errno;
    close(fd);
    return -e;
}

/* Start the daemon on a thread with SIGTERM blocked here first, so its sigwait
 * is the only consumer. Every N6 case needs the same four lines. */
static void daemon_start(daemon_run_t *d, sigset_t *old)
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, old);
    /* `done` must be cleared BEFORE the thread exists: a caller polling it to
     * decide whether the daemon exited on its own reads a stale non-zero as
     * "already finished" and then blocks forever in pthread_join.  Found the
     * hard way — an uninitialised d.done wedged the suite under a mutation. */
    d->rc = -999;
    atomic_store(&d->done, 0);
    EXPECT(pthread_create(&d->tid, NULL, daemon_thread, d) == 0,
           "daemon thread started");
}

/*
 * The stream output is the one enabled by default and the one every bridge and
 * libimud client reads. Its thread failing is fatal — the alternative is a
 * daemon that systemd reports active while producing nothing at all.
 *
 * The assertion that matters is not the return code on its own: it is that the
 * AF_UNIX node is gone afterwards. Before the fix the daemon warned, kept
 * running, and left the socket bound with nobody calling accept(), so a client
 * connected successfully into the backlog and then waited forever.
 */
static void test_daemon_stream_thread_failure_is_fatal(void)
{
    begin("test_daemon_stream_thread_failure_is_fatal");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    unlink(T_PID_FILE);
    write_conf(10, 833);
    atomic_store(&g_fail_fn, stream_out_thread);

    sigset_t old;
    daemon_run_t d = {0};
    daemon_start(&d, &old);

    /*
     * Poll rather than join outright: a regression here does not return a bad
     * code, it blocks in sigwait forever, so a straight join would hang the
     * suite instead of failing it.
     *
     * Probing the socket only makes sense AFTER that wait, never during it.
     * out_ctx_open binds the listener in step 7 and the threads start in step
     * 10, so on any healthy start there is a window where the socket is bound
     * with nothing accepting yet — a connect() inside the loop succeeds and
     * proves nothing. (Invisible at -O2, wide open under TSan, which is how
     * this was caught.) Once the daemon has failed to exit, that window is
     * long past and a socket still accepting is the finding itself.
     */
    bool exited = false;
    for (int waited = 0; waited < 15000; waited += 50) {
        if (atomic_load(&d.done)) { exited = true; break; }
        msleep(50);
    }
    EXPECT(exited, "refuses to run without its stream thread");

    if (!exited) {
        /* Still up. Say why, rather than leaving a bare timeout: this is the
         * finding stated as a behaviour — a subscriber accepted into a backlog
         * nobody will ever accept() from, which reads to the client as a
         * successful connect followed by permanent silence. */
        int c = try_connect_unix(T_STREAM_SOCK);
        EXPECT(c < 0, "never accepts a subscriber it cannot serve");
        if (c >= 0) close(c);
        pthread_kill(d.tid, SIGTERM);            /* don't wedge the suite */
    }
    pthread_join(d.tid, NULL);
    EXPECT(d.rc == 1, "exits 1, so Restart=on-failure retries it");

    struct stat st;
    EXPECT(stat(T_STREAM_SOCK, &st) != 0, "the stream socket is not left on disk");
    EXPECT(stat(T_PID_FILE, &st) != 0, "pid file removed");
    EXPECT(stat(T_STATUS_SOCK, &st) != 0, "status socket unlinked");

    atomic_store(&g_fail_fn, NULL);
    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    end(fb);
}

/*
 * NMEA is opt-in, so its thread failing stays a warning — but the TCP listener
 * out_ctx_open already bound has to go with it. Before the fix connect()
 * SUCCEEDED and then delivered nothing, which is the worst diagnostic shape
 * available; ECONNREFUSED tells the client the truth immediately.
 */
static void test_daemon_nmea_thread_failure_closes_listener(void)
{
    begin("test_daemon_nmea_thread_failure_closes_listener");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 10, .imu_odr_hz = 833,
                                 .nmea_tcp = true });
    atomic_store(&g_fail_fn, nmea_out_thread);

    sigset_t old;
    daemon_run_t d = {0};
    daemon_start(&d, &old);

    /* Waiting on the stream socket is the synchronisation: step 10 starts nmea
     * BEFORE stream, so a stream that accepts proves the nmea arm has run. */
    int sfd = connect_unix(T_STREAM_SOCK, 10000);
    EXPECT(sfd >= 0, "the daemon still runs — nmea is opt-in, not fatal");
    if (sfd >= 0) close(sfd);

    int rc = connect_tcp(T_NMEA_TCP_PORT);
    EXPECT(rc < 0, "the nmea TCP listener does not accept");
    EXPECT(rc == -ECONNREFUSED, "and refuses rather than hanging a client");
    if (rc >= 0) close(rc);

    /* The flag is cleared too, so imud-status agrees with the socket. */
    char rep[8192];
    EXPECT(fetch_status(rep, sizeof rep), "status socket still answers");

    EXPECT(pthread_kill(d.tid, SIGTERM) == 0, "SIGTERM delivered");
    pthread_join(d.tid, NULL);
    EXPECT(d.rc == 0, "and it still shuts down clean");

    atomic_store(&g_fail_fn, NULL);
    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    unlink(T_STREAM_SOCK);
    end(fb);
}

/*
 * The high-rate output is connectionless, so there is no listener to refuse on
 * and the finding's "silent accept" shape does not apply. What closing its fd
 * does fix is the exit: out_ctx_send_shutdown returns early on hirate_fd < 0,
 * so a stream whose thread never ran no longer announces the end of data that
 * never came. Before the fix exactly one FLAG_SHUTDOWN datagram arrived here.
 */
static void test_daemon_hirate_thread_failure_sends_nothing(void)
{
    begin("test_daemon_hirate_thread_failure_sends_nothing");
    int fb = g_fail;

    /* Bind the receiver before the daemon starts, so nothing can be missed. */
    int rx = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port   = htons(T_HIRATE_PORT);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    EXPECT(rx >= 0 && bind(rx, (struct sockaddr *)&a, sizeof a) == 0,
           "udp receiver bound on the hirate destination");
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 10, .imu_odr_hz = 833,
                                 .hirate = true });
    atomic_store(&g_fail_fn, hirate_out_thread);

    sigset_t old;
    daemon_run_t d = {0};
    daemon_start(&d, &old);

    /* Same ordering trick: hirate is started before stream. */
    int sfd = connect_unix(T_STREAM_SOCK, 10000);
    EXPECT(sfd >= 0, "the daemon still runs — highrate is opt-in, not fatal");
    if (sfd >= 0) close(sfd);

    EXPECT(pthread_kill(d.tid, SIGTERM) == 0, "SIGTERM delivered");
    pthread_join(d.tid, NULL);
    EXPECT(d.rc == 0, "shuts down clean");

    /* Everything the daemon could ever have sent is queued by now. */
    char buf[512];
    ssize_t n = recv(rx, buf, sizeof buf, MSG_DONTWAIT);
    EXPECT(n < 0, "no packet on a stream whose thread never started");
    close(rx);

    atomic_store(&g_fail_fn, NULL);
    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    unlink(T_STREAM_SOCK);
    end(fb);
}

/* --version is handled before any hardware or socket work. */
static void test_daemon_version_flag(void)
{
    begin("test_daemon_version_flag");
    int fb = g_fail;

    char *argv[] = { (char *)"imud", (char *)"--version", NULL };
    EXPECT(main_entry(2, argv) == 0, "--version exits 0");

    char *bad[] = { (char *)"imud", (char *)"--nonsense", NULL };
    EXPECT(main_entry(2, bad) == 1, "an unknown option exits 1");
    end(fb);
}

int main(void)
{
    test_daemon_version_flag();
    test_daemon_refuses_bad_config();
    test_daemon_refuses_overlong_socket();
    test_daemon_lifecycle();
    test_daemon_stream_thread_failure_is_fatal();
    test_daemon_nmea_thread_failure_closes_listener();
    test_daemon_hirate_thread_failure_sends_nothing();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
