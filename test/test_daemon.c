/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_daemon.c — the daemon itself, start to shutdown
 *
 * src/main.c is 725 lines that no test binary linked: thread lifecycle, the
 * status-socket responder, the SIGHUP reload block, and shutdown ordering. The
 * reload block and the responder are the highest-value targets of those, and
 * none of it had ever run outside a Pi.
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
 * THREAD FAILURE: the Makefile compiles this copy of main.c with
 * -Dpthread_create=imud_test_pthread_create, so the stand-in below can fail one
 * named thread and leave every other one alone. Without that seam main()'s four
 * warn-and-continue output arms and its one fatal one are unreachable from a
 * test — nothing an outside process can do makes pthread_create return EAGAIN.
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
#include "capture.h"    /* cap_writer_* — the --replay cases build their own */
#include "drivers.h"    /* sim_synth_imu / sim_synth_mag */

int main_entry(int argc, char **argv);

/* ── the pthread_create seam ─────────────────────────────────────────────── */

/* Typed as the entry point rather than void *: casting between function and
 * object pointers is not something C guarantees, and there is no reason to. */
typedef void *(*thread_fn_t)(void *);

/* NULL — the default and what every other case in this suite runs with — means
 * pass every thread straight through. */
static thread_fn_t _Atomic g_fail_fn;

/* main.c is compiled with -Dpthread_create=imud_test_pthread_create, so its
 * calls land here.  This file is not, so the pass-through below is the real
 * one. */
int imud_test_pthread_create(pthread_t *restrict tid,
                             const pthread_attr_t *restrict attr,
                             thread_fn_t fn, void *restrict arg);

int imud_test_pthread_create(pthread_t *restrict tid,
                             const pthread_attr_t *restrict attr,
                             thread_fn_t fn, void *restrict arg)
{
    thread_fn_t target = atomic_load(&g_fail_fn);
    if (target != NULL && fn == target)
        return EAGAIN;   /* what the real one returns under resource limits */
    return pthread_create(tid, attr, fn, arg);
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
/* The redirected SYS_CONF.  The config-fallback cases need it ABSENT — it is what the daemon
 * reads when no --config is given, and the $HOME fallback only happens when it
 * is missing.  Nothing here ever creates it. */
#define T_SYS_CONF    "/tmp/imud_e2e_sysconf.conf"
/* A $HOME the suite owns, so `getenv("HOME")/.config/imud/imud.conf` resolves
 * somewhere the test wrote rather than to the developer's real config. */
#define T_HOME        "/tmp/imud_e2e_home"
#define T_HOME_CONF   T_HOME "/.config/imud/imud.conf"
/* A path that must not exist, for the case that proves --config gets no
 * fallback. */
#define T_MISSING_CONF "/tmp/imud_e2e_no_such.conf"
#define T_REPLAY_LOG   "/tmp/imud_e2e_daemon_replay.log"

/* Ports the thread-failure cases need a listener or a receiver on, in the same 27xxx block
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

/* The optional outputs only the thread-failure cases want: each one is a listener or a
 * destination the test can then observe from outside the daemon. */
typedef struct {
    int  nmea_rate_hz;
    int  imu_odr_mhz;
    bool nmea_tcp;   /* [nmea] tcp_enabled  → a listener on T_NMEA_TCP_PORT */
    bool hirate;     /* [highrate] enabled  → UDP to 127.0.0.1:T_HIRATE_PORT */
    /* NULL → T_CONF.  The fallback case needs a second config on disk at
     * the $HOME path, identical in every other respect. */
    const char *path;
    /* Omit `rate_hz` from [nmea] entirely rather than writing a value.  This is
     * the deleted-key case: the key is ABSENT, which is different from
     * setting it, and the daemon must fall back to the compiled-in
     * default (10). */
    bool omit_nmea_rate;
    /*
     * Tuned for the --replay cases, and each value earns its place:
     *
     * sim_speed = 0 plays the capture as fast as the reader threads take it.
     * At the default 1.0 a 3 s capture costs 3 s of wall clock, and CI runs
     * this suite three times over (plain, ASan/UBSan, TSan).
     *
     * The settle and align windows default to 5 s EACH, counted in samples at
     * the configured rate — so a short capture would be consumed entirely by
     * startup and the fusion thread would never reach its main loop.  The
     * drain these cases exist to prove would then hold vacuously.
     */
    bool replay;
    /*
     * Replay at sim_speed = 1.0 instead of 0.0, so a 3 s capture takes 3 s of
     * wall clock and a signal sent one second in lands mid-replay.  The signal
     * case needs that: sim_loop would be the obvious way to keep a replay
     * running, but --replay deliberately ignores it (src/main.c), so pacing is
     * the only lever left.  Slower is the safe direction — under a sanitizer
     * the replay takes longer and the margin only grows.
     */
    bool replay_realtime;
    /* Send the daemon's log to this file at level info, rather than dropping
     * everything below error.  For a case that has to tell WHY the daemon
     * stopped: the two shutdown paths are one LOG_I line each. */
    const char *log_file;
} conf_opt_t;

static void write_conf_opt(conf_opt_t o)
{
    FILE *f = fopen(o.path ? o.path : T_CONF, "w");
    if (!f) { perror("fopen"); exit(1); }
    /* Written as a whole line or not at all: the point of the deleted-key case is
     * that the key is absent from the file, not that it holds some value. */
    char nmea_rate[32];
    if (o.omit_nmea_rate) nmea_rate[0] = '\0';
    else snprintf(nmea_rate, sizeof nmea_rate, "rate_hz = %d\n", o.nmea_rate_hz);
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
        "%s"
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
        "level = \"%s\"\n"
        "file = \"%s\"\n",
        o.imu_odr_mhz, nmea_rate,
        o.nmea_tcp ? "true" : "false", T_NMEA_TCP_PORT,
        o.hirate   ? "true" : "false", T_HIRATE_PORT,
        T_STREAM_SOCK,
        o.log_file ? "info" : "error",
        o.log_file ? o.log_file : "");
    if (o.replay)
        fprintf(f,
            "[device]\n"
            "sim_speed = %s\n"
            "[fusion]\n"
            "startup_settle_sec = 0.0\n"
            "align_window_sec = 0.5\n",
            o.replay_realtime ? "1.0" : "0.0");
    fclose(f);
}

/* The plain form: no optional outputs. nmea rate_hz is a [hot] key and imu
 * odr_hz is a [restart] key; both are printed by the status report, which is
 * what makes the reload contract observable from outside the process. */
static void write_conf(int nmea_rate_hz, int imu_odr_mhz)
{
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = nmea_rate_hz,
                                 .imu_odr_mhz   = imu_odr_mhz });
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

/* True when the report line beginning `label` also contains `want`. Scoped to
 * the one line on purpose: "disabled" appears against several outputs, so a
 * bare strstr over the whole report would match the wrong one. */
static bool status_line_has(const char *rep, const char *label, const char *want)
{
    const char *p = strstr(rep, label);
    if (!p) return false;
    const char *eol = strchr(p, '\n');
    const char *hit = strstr(p, want);
    return hit && (!eol || hit < eol);
}

/*
 * Consume a SIGTERM still pending on this process.
 *
 * The daemon under test is supposed to have taken it.  When a regression means
 * it did not, the signal stays pending — blocked, so harmless — until the case
 * restores the caller's mask, and is then delivered to the suite and kills it.
 * That turns a reportable failure into a dead test run with no verdict, which
 * is how the mutation check for this case first presented.
 */
static void drain_pending_sigterm(void)
{
    sigset_t pend;
    if (sigpending(&pend) == 0 && sigismember(&pend, SIGTERM) > 0) {
        sigset_t one;
        sigemptyset(&one);
        sigaddset(&one, SIGTERM);
        int s;
        sigwait(&one, &s);      /* cannot block: it is already pending */
    }
}

/* True when the daemon's log file contains `want`.  A missing file is false,
 * not an error: that is itself a legitimate answer about what was logged. */
static bool log_has(const char *path, const char *want)
{
    FILE *f = fopen(path, "r");
    if (!f) return false;
    char buf[16384];
    size_t n = fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n] = '\0';
    return strstr(buf, want) != NULL;
}

/*
 * Block until the status report says `label`'s line contains `want`, or give
 * up.  Returns true if it happened.
 *
 * Every "wait for the daemon to have done X" in this suite goes through here,
 * because the two obvious alternatives are both wrong: a fixed sleep couples
 * the test to how fast the machine is (a 1.5 s one here is the only
 * thing standing between SIGHUP and the assertion), and connecting to a socket
 * proves nothing, since out_ctx_open binds every listener in step 7 and a bound
 * listener accepts into its backlog with no thread behind it — the whole of
 * the thread-failure finding, which two cases below use as a
 * start signal.
 */
static bool wait_for_status(const char *label, const char *want, int timeout_ms)
{
    char rep[8192];
    for (int waited = 0; waited < timeout_ms; waited += 100) {
        if (fetch_status(rep, sizeof rep) && status_line_has(rep, label, want))
            return true;
        msleep(100);
    }
    return false;
}

/* `done` is how a caller can tell "still running" from "returned" without
 * pthread_kill(tid, 0), which is undefined once the thread has exited — the id
 * may already have been reused.  _Atomic per the project's rule for every
 * cross-thread flag; its release also makes `rc` safe to read after it reads 1. */
typedef struct {
    int rc; pthread_t tid; _Atomic int done;
    /* NULL — most cases — means the usual `--config T_CONF` form.
     * The config-fallback cases need argv the daemon has not seen: no --config at all (the
     * only route left to the $HOME fallback), and --config on a missing path. */
    char **argv; int argc;
} daemon_run_t;

static void *daemon_thread(void *arg)
{
    daemon_run_t *d = (daemon_run_t *)arg;
    char *dflt[] = { (char *)"imud", (char *)"--config", (char *)T_CONF, NULL };
    d->rc = d->argv ? main_entry(d->argc, d->argv) : main_entry(3, dflt);
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
    /* On its own this proves less than it looks — the listener is bound in
     * step 7, so it accepts before any thread exists.  The full frame below is
     * what actually proves the stream thread runs, and since stream is started
     * last of step 10's output arms, that frame also orders everything after
     * it.  Kept as an assertion because a failure here localises the problem. */
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

    /* Poll, don't sleep.  This was msleep(1500), which is a guess about how
     * long sigwait takes to service a reload — fine on an idle box, and the
     * kind of thing that fails on loaded CI or under a sanitizer. */
    EXPECT(wait_for_status("NMEA out:", "7 Hz", 15000),
           "hot key applied: NMEA rate is now 7 Hz");

    char rep2[8192];
    EXPECT(fetch_status(rep2, sizeof rep2), "status socket answers after reload");
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
 * [stream] socket one character too long would be truncated to a
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

/* ── a thread that fails to start leaves nothing bound ───────────────────── */

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
 * is the only consumer. Every thread-failure case needs the same four lines. */
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

/* Stands in for a reader thread: created after the signals are blocked, so it
 * inherits the block exactly as imu.c's readers inherit main.c's. */
static void *raise_from_worker(void *arg)
{
    (void)arg;
    raise(SIGTERM);   /* thread-directed — goes pending on THIS thread ... */
    return NULL;      /* ... and is discarded when it exits, right here. */
}

/*
 * How a worker asks the daemon to shut down.
 *
 * imu.c's reader threads escalate to shutdown after three failed chip resets.
 * raise() will not do: it is thread-directed — POSIX defines it
 * as pthread_kill(pthread_self(), sig) — and main.c blocks SIGTERM before
 * creating any thread, so the signal went pending on the reader, where main's
 * sigwait could not see it, and was thrown away by the break on the next line.
 * The daemon carried on reporting active with a dead reader.
 *
 * Both halves are asserted here because the second only means something given
 * the first: raise() really does go nowhere on this platform, and the
 * process-directed kill() really does reach sigwait.
 *
 * What this does NOT cover: the three-reset-failure path into that escalation.
 * Reaching it needs a driver whose reset() fails repeatedly, and no seam in the
 * tree provides one — the sim driver's reset always succeeds. The mechanism is
 * tested; the trigger is not.
 *
 * Sending a process-directed signal from inside a test binary is only safe
 * because daemon_start() blocks the set in this thread first and every thread
 * here descends from it. A case that spawns a thread outside that mask would
 * take the whole suite down instead.
 */
static void test_daemon_worker_can_signal_shutdown(void)
{
    begin("test_daemon_worker_can_signal_shutdown");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    write_conf(10, 833);

    sigset_t old;
    daemon_run_t d = {0};
    daemon_start(&d, &old);

    EXPECT(wait_for_status("IMU ODR:", "833", 15000), "daemon is up");

    /* ── raise() from a worker reaches nobody ─────────────────────────────── */
    pthread_t w;
    EXPECT(pthread_create(&w, NULL, raise_from_worker, NULL) == 0,
           "worker thread started");
    pthread_join(w, NULL);

    msleep(300);
    EXPECT(!atomic_load(&d.done),
           "raise() from a worker does NOT stop the daemon");

    /* ── kill(getpid()) does ──────────────────────────────────────────────── */
    EXPECT(kill(getpid(), SIGTERM) == 0, "process-directed SIGTERM sent");

    bool exited = false;
    for (int waited = 0; waited < 15000; waited += 50) {
        if (atomic_load(&d.done)) { exited = true; break; }
        msleep(50);
    }
    EXPECT(exited, "a process-directed SIGTERM reaches main's sigwait");
    if (!exited) pthread_kill(d.tid, SIGTERM);   /* don't wedge the suite */
    pthread_join(d.tid, NULL);

    /* Exiting is not enough — it has to leave through §12/§13. */
    EXPECT(d.rc == 0, "exits 0");
    struct stat st;
    EXPECT(stat(T_PID_FILE, &st) != 0, "pid file removed");
    EXPECT(stat(T_STATUS_SOCK, &st) != 0, "status socket unlinked");
    EXPECT(stat(T_STREAM_SOCK, &st) != 0, "stream socket unlinked");

    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    end(fb);
}

/* ── --replay reaches the end of a capture and returns ───────────────────── */

#define T_REPLAY_CAP "/tmp/imud_e2e_daemon_replay.imucap"

/*
 * 3 s at 100 Hz IMU with mag at 10 Hz.
 *
 * The mag rate is the wall-clock budget here, not the file length: the mag ops
 * hand back one sample per call and the mag reader polls every 10 ms, so even
 * at sim_speed = 0 the replay costs ~10 ms per mag record however fast the IMU
 * side runs (it is batched, up to 128 a call).  30 records ≈ 300 ms; a 100 Hz
 * mag capture would make this case slower than the thing it is testing.
 */
static const char *make_replay_capture(void)
{
    cap_writer_t w;
    if (cap_writer_open(&w, T_REPLAY_CAP, 100, 100000, "sim", "sim",
                        "1.9.0", 0, 0) != 0) {
        perror("cap_writer_open");
        exit(1);
    }
    for (int i = 0; i < 300; i++) {
        double t = i / 100.0;
        imu_sample_t s;
        sim_synth_imu(t, &s);
        s.seq     = (uint32_t)i;
        s.chip_ts = (uint32_t)(t * 1e9 / 25000.0);   /* 25 µs ticks */
        cap_writer_imu(&w, &s, (uint64_t)(t * 1e9));
        if (i % 10 == 0) {
            mag_sample_t m;
            sim_synth_mag(t, &m);
            cap_writer_mag(&w, &m, (uint64_t)(t * 1e9));
        }
    }
    cap_writer_close(&w);
    return T_REPLAY_CAP;
}

/*
 * A replay is a one-shot run over a finite file, so it has to end by itself.
 * Without it the daemon logs "[sim] playback finished" and sits in sigwait forever:
 * the sim driver knew, and nothing carried that up to main.
 *
 * Polling rather than joining outright, for the same reason as the
 * thread-failure cases: a regression here does not return a bad code, it
 * blocks, so a straight join would hang the suite instead of failing it.
 */
static void test_daemon_replay_exits_at_end_of_capture(void)
{
    begin("test_daemon_replay_exits_at_end_of_capture");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    unlink(T_PID_FILE);
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 10, .imu_odr_mhz = 100,
                                 .replay = true });
    const char *cap = make_replay_capture();

    char *argv[] = { (char *)"imud", (char *)"--config", (char *)T_CONF,
                     (char *)"--replay", (char *)cap,
                     (char *)"--skip-bias-cal", NULL };
    sigset_t old;
    daemon_run_t d = { .argv = argv, .argc = 6 };
    daemon_start(&d, &old);

    bool exited = false;
    for (int waited = 0; waited < 20000; waited += 50) {
        if (atomic_load(&d.done)) { exited = true; break; }
        msleep(50);
    }
    EXPECT(exited, "--replay returns when the capture runs out");
    if (!exited) pthread_kill(d.tid, SIGTERM);   /* don't wedge the suite */
    pthread_join(d.tid, NULL);

    EXPECT(d.rc == 0, "a finished replay is a success, not a failure");

    /* Exiting is not enough: it has to leave through the normal unwind. */
    struct stat st;
    EXPECT(stat(T_PID_FILE, &st) != 0, "pid file removed");
    EXPECT(stat(T_STATUS_SOCK, &st) != 0, "status socket unlinked");
    EXPECT(stat(T_STREAM_SOCK, &st) != 0, "stream socket unlinked");

    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    unlink(T_REPLAY_CAP);
    end(fb);
}

/*
 * A signal still has to reach a --replay run.
 *
 * That loop is the one place main() cannot sit in sigwait: it must look up
 * periodically to notice the file has run out, so it polls the pending set
 * instead (wait_signal_timed in src/main.c).  Every other case here exercises
 * the sigwait branch, which would stay green while SIGTERM did nothing at all
 * during a replay — a daemon that ignores systemctl stop for as long as the
 * capture lasts.
 *
 * Replayed at real time so the 3 s capture is still playing a second in, which
 * is what makes the signal — and not the end of the file — the thing that
 * stops it.  Process-directed, the form an operator and systemd actually send:
 * sigpending() must report it from the process's pending set, not just the
 * calling thread's.
 */
static void test_daemon_replay_stops_on_signal(void)
{
    begin("test_daemon_replay_stops_on_signal");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    unlink(T_PID_FILE);
    unlink(T_REPLAY_LOG);
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 10, .imu_odr_mhz = 100,
                                 .replay = true, .replay_realtime = true,
                                 .log_file = T_REPLAY_LOG });
    const char *cap = make_replay_capture();

    char *argv[] = { (char *)"imud", (char *)"--config", (char *)T_CONF,
                     (char *)"--replay", (char *)cap,
                     (char *)"--skip-bias-cal", NULL };
    /*
     * [logging] file makes main() dup2() the log over STDERR_FILENO, which is
     * process-wide — and main_entry() runs on a thread of THIS process, so
     * without saving and restoring fd 2 the case would silently swallow every
     * later case's FAIL line along with the daemon's output.
     */
    int saved_stderr = dup(STDERR_FILENO);
    EXPECT(saved_stderr >= 0, "stderr saved");

    sigset_t old;
    daemon_run_t d = { .argv = argv, .argc = 6 };
    daemon_start(&d, &old);

    EXPECT(wait_for_status("IMU ODR:", "100", 15000), "replay is up");

    /* The premise: the replay is still running.  Without this the case would
     * pass on a daemon that ignored the signal and merely reached EOF. */
    msleep(1000);
    EXPECT(!atomic_load(&d.done), "the replay is still playing");

    /*
     * Directed at the thread running main(), where the sibling case above
     * uses kill(getpid()).  The difference is Darwin: its sigpending() reports
     * only the CALLING thread's pending set, so a process-directed signal that
     * every thread blocks is visible to the main thread alone — measured, and
     * the reason this case failed on macOS while the sigwait one passed.
     * In a real run main() IS the process's main thread and sees it, which a
     * bench run confirms (caught signal 1, replay complete 0, exit 0); here
     * main_entry() is on a worker, so the process-directed spelling models a
     * situation that cannot arise.  The code under test is the same either
     * way — wait_signal_timed() scans one pending set and calls sigwait().
     */
    EXPECT(pthread_kill(d.tid, SIGTERM) == 0, "SIGTERM sent to main's thread");

    bool exited = false;
    for (int waited = 0; waited < 15000; waited += 50) {
        if (atomic_load(&d.done)) { exited = true; break; }
        msleep(50);
    }
    EXPECT(exited, "SIGTERM reaches the replay loop's signal poll");
    if (!exited) pthread_kill(d.tid, SIGTERM);   /* don't wedge the suite */
    pthread_join(d.tid, NULL);

    if (saved_stderr >= 0) {                     /* before the first FAIL below */
        dup2(saved_stderr, STDERR_FILENO);
        close(saved_stderr);
    }

    EXPECT(d.rc == 0, "exits 0");

    /*
     * WHY it stopped, not just that it did.  A poll that never noticed the
     * signal would let the replay run on to the end of the file and exit 0
     * there — passing every check above.  The two paths log one line each, so
     * the log is what separates them.
     */
    EXPECT(log_has(T_REPLAY_LOG, "caught signal"),
           "stopped because of the signal");
    EXPECT(!log_has(T_REPLAY_LOG, "replay complete"),
           "and not because the capture ran out");

    /* Exiting is not enough: it has to leave through the normal unwind. */
    struct stat st;
    EXPECT(stat(T_PID_FILE, &st) != 0, "pid file removed");
    EXPECT(stat(T_STATUS_SOCK, &st) != 0, "status socket unlinked");
    EXPECT(stat(T_STREAM_SOCK, &st) != 0, "stream socket unlinked");

    drain_pending_sigterm();
    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    unlink(T_REPLAY_CAP);
    unlink(T_REPLAY_LOG);
    end(fb);
}

/*
 * A capture that cannot be opened is not a completed one.  Both playback
 * streams mark themselves done on a failed open, so anything keying off that
 * alone would report success; `imud --replay /typo` would exit 0 having
 * replayed nothing, rather than idling forever having said so once.
 */
static void test_daemon_replay_missing_capture_exits_1(void)
{
    begin("test_daemon_replay_missing_capture_exits_1");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    unlink(T_PID_FILE);
    unlink(T_REPLAY_CAP);                        /* the point: it is absent */
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 10, .imu_odr_mhz = 100,
                                 .replay = true });

    char *argv[] = { (char *)"imud", (char *)"--config", (char *)T_CONF,
                     (char *)"--replay", (char *)T_REPLAY_CAP,
                     (char *)"--skip-bias-cal", NULL };
    sigset_t old;
    daemon_run_t d = { .argv = argv, .argc = 6 };
    daemon_start(&d, &old);

    bool exited = false;
    for (int waited = 0; waited < 10000; waited += 50) {
        if (atomic_load(&d.done)) { exited = true; break; }
        msleep(50);
    }
    EXPECT(exited, "does not sit idle on a capture it cannot read");
    if (!exited) pthread_kill(d.tid, SIGTERM);
    pthread_join(d.tid, NULL);

    EXPECT(d.rc == 1, "exits 1 — nothing was replayed");

    struct stat st;
    EXPECT(stat(T_PID_FILE, &st) != 0, "pid file removed");
    EXPECT(stat(T_STATUS_SOCK, &st) != 0, "status socket unlinked");

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
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 10, .imu_odr_mhz = 833,
                                 .nmea_tcp = true });
    atomic_store(&g_fail_fn, nmea_out_thread);

    sigset_t old;
    daemon_run_t d = {0};
    daemon_start(&d, &old);

    /*
     * Synchronise on the arm itself, NOT on the stream socket.  An earlier
     * version waited for connect_unix(T_STREAM_SOCK) on the reasoning that
     * step 10 starts nmea before stream — which is true and irrelevant:
     * out_ctx_open binds every listener back in step 7 (main.c:558), and a
     * bound listener accepts into its backlog with no thread behind it.  That
     * is the whole of that finding, so using it as a start signal proves only
     * that step 7 finished, and the window to the nmea arm (main.c:642) is
     * four thread creations wide.  It cost a TSan-only flake to notice.
     *
     * status_fmt prints "disabled" for NMEA only when both cfg flags are
     * false, and clearing both is what the arm does — so that line IS the arm
     * having run.  The status socket is served by health_thread, which starts
     * before the arm, so it answers throughout the window.
     */
    EXPECT(wait_for_status("NMEA out:", "disabled", 15000),
           "the daemon still runs, with nmea disabled — opt-in, not fatal");

    int rc = connect_tcp(T_NMEA_TCP_PORT);
    EXPECT(rc < 0, "the nmea TCP listener does not accept");
    EXPECT(rc == -ECONNREFUSED, "and refuses rather than hanging a client");
    if (rc >= 0) close(rc);

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
 * so a stream whose thread never ran does not announce the end of data that
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
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 10, .imu_odr_mhz = 833,
                                 .hirate = true });
    atomic_store(&g_fail_fn, hirate_out_thread);

    sigset_t old;
    daemon_run_t d = {0};
    daemon_start(&d, &old);

    /* Synchronise on the arm, for the reason spelled out in the nmea case
     * above — a bound stream socket proves only that step 7 ran.  This case is
     * less exposed to it (the real assertion is the recv after join, and by
     * then the daemon has exited), but a start signal that does not signal a
     * start is worth removing wherever it appears. */
    EXPECT(wait_for_status("Hi-rate out:", "disabled", 15000),
           "the daemon still runs, with highrate disabled — opt-in, not fatal");

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

/* ── SIGHUP must resolve config the way startup does ─────────────────────── */

/* mkdir -p for the one directory these cases need. */
static void make_home_dir(void)
{
    mkdir(T_HOME, 0700);
    mkdir(T_HOME "/.config", 0700);
    mkdir(T_HOME "/.config/imud", 0700);
}

/*
 * Reload seeded from the RUNNING config, so a key deleted from the file kept
 * its old value while the daemon logged "config reloaded" — the operator
 * removes rate_hz to get the default back and would otherwise get the running rate,
 * and the next restart then disagrees with the reload that preceded it.
 * Seeding from config_defaults() is what makes reload mean what restart means.
 */
static void test_daemon_reload_reverts_a_deleted_key(void)
{
    begin("test_daemon_reload_reverts_a_deleted_key");
    int fb = g_fail;

    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    /* 7, chosen because the compiled-in default is 10: the assertion below
     * cannot pass by accident on a daemon that never re-read anything. */
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 7, .imu_odr_mhz = 833 });

    sigset_t old;
    daemon_run_t d = {0};
    daemon_start(&d, &old);

    /* Poll the report rather than sleeping a fixed interval: the daemon
     * reaches sigwait only after all of step 10, and connect_unix on the
     * stream socket does not mark that point (see the nmea case). */
    EXPECT(wait_for_status("NMEA out:", "7 Hz", 15000),
           "starts on the configured 7 Hz");

    /* The key is deleted, not changed — the whole point of the case. */
    write_conf_opt((conf_opt_t){ .omit_nmea_rate = true, .imu_odr_mhz = 833 });
    EXPECT(pthread_kill(d.tid, SIGHUP) == 0, "SIGHUP delivered");

    EXPECT(wait_for_status("NMEA out:", "10 Hz", 15000),
           "a deleted [hot] key reverts to its default");

    char rep2[8192];
    EXPECT(fetch_status(rep2, sizeof rep2), "status socket answers after reload");
    EXPECT(!status_line_has(rep2, "NMEA out:", "7 Hz"),
           "and the old value is gone");

    EXPECT(pthread_kill(d.tid, SIGTERM) == 0, "SIGTERM delivered");
    pthread_join(d.tid, NULL);
    EXPECT(d.rc == 0, "shuts down clean");

    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_CONF);
    unlink(T_STREAM_SOCK);
    end(fb);
}

/*
 * Startup falls back to $HOME when the system config
 * is missing, so reload must not re-read args.config_path regardless. A daemon
 * that came up on the fallback therefore answered EVERY SIGHUP with "config
 * reload failed" — hot reload dead, and the message blaming the file it had in
 * fact never opened. It must reload the file it actually read.
 */
static void test_daemon_reload_follows_the_home_fallback(void)
{
    begin("test_daemon_reload_follows_the_home_fallback");
    int fb = g_fail;

    /* The fallback only happens when the system config is missing.  The suite
     * never creates T_SYS_CONF; unlinking is belt and braces. */
    unlink(T_SYS_CONF);
    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    make_home_dir();
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 9, .imu_odr_mhz = 833,
                                 .path = T_HOME_CONF });

    /* Set before the daemon thread exists, so its getenv is ordered behind
     * this store and there is no setenv/getenv race to report. */
    char *saved_home = getenv("HOME");
    char home_copy[512];
    snprintf(home_copy, sizeof home_copy, "%s", saved_home ? saved_home : "");
    setenv("HOME", T_HOME, 1);

    /* No --config at all: with an explicit one the fallback is skipped, so
     * this is the only argv that reaches the branch under test. */
    char *argv[] = { (char *)"imud", NULL };
    sigset_t old;
    daemon_run_t d = { .argv = argv, .argc = 1 };
    daemon_start(&d, &old);

    EXPECT(wait_for_status("NMEA out:", "9 Hz", 15000),
           "startup used the $HOME fallback");

    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 6, .imu_odr_mhz = 833,
                                 .path = T_HOME_CONF });
    EXPECT(pthread_kill(d.tid, SIGHUP) == 0, "SIGHUP delivered");

    EXPECT(wait_for_status("NMEA out:", "6 Hz", 15000),
           "SIGHUP reloads the file startup actually read");

    char rep2[8192];
    EXPECT(fetch_status(rep2, sizeof rep2), "status socket answers after reload");
    EXPECT(!status_line_has(rep2, "NMEA out:", "9 Hz"),
           "not the one it was asked for");

    EXPECT(pthread_kill(d.tid, SIGTERM) == 0, "SIGTERM delivered");
    pthread_join(d.tid, NULL);
    EXPECT(d.rc == 0, "shuts down clean");

    if (saved_home) setenv("HOME", home_copy, 1);
    else            unsetenv("HOME");
    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_HOME_CONF);
    unlink(T_STREAM_SOCK);
    end(fb);
}

/*
 * --config names one file and gets no fallback; falling back to the $HOME
 * fallback whether or not --config was given, so a mistyped path silently
 * started the daemon on someone else's config — while imud.8 had documented
 * --config as replacing the search path since the beginning.
 *
 * The assertion is on the socket, not the exit code, deliberately: a daemon on
 * built-in defaults wants /dev/i2c-1 and an ism330dhcx, so "it exits non-zero"
 * would be an assertion about the machine rather than about the policy.
 * "T_STREAM_SOCK is not on disk" holds whether the defaults daemon fails at
 * hardware init or starts and binds /run/imud/imud-stream.sock instead.
 */
static void test_daemon_explicit_config_does_not_fall_back(void)
{
    begin("test_daemon_explicit_config_does_not_fall_back");
    int fb = g_fail;

    unlink(T_MISSING_CONF);          /* the path must genuinely not exist */
    unlink(T_STATUS_SOCK);
    unlink(T_STREAM_SOCK);
    make_home_dir();
    /* A perfectly valid sim config sitting exactly where the fallback looks:
     * pre-fix, the daemon comes up on this and binds T_STREAM_SOCK. */
    write_conf_opt((conf_opt_t){ .nmea_rate_hz = 9, .imu_odr_mhz = 833,
                                 .path = T_HOME_CONF });

    char *saved_home = getenv("HOME");
    char home_copy[512];
    snprintf(home_copy, sizeof home_copy, "%s", saved_home ? saved_home : "");
    setenv("HOME", T_HOME, 1);

    char *argv[] = { (char *)"imud", (char *)"--config",
                     (char *)T_MISSING_CONF, NULL };
    sigset_t old;
    daemon_run_t d = { .argv = argv, .argc = 3 };
    daemon_start(&d, &old);

    /* Generous: pre-fix the socket appears within a second or two.  Stop early
     * if the defaults daemon has already exited on missing hardware. */
    struct stat st;
    bool bound = false;
    for (int waited = 0; waited < 8000; waited += 100) {
        if (stat(T_STREAM_SOCK, &st) == 0) { bound = true; break; }
        if (atomic_load(&d.done)) break;
        msleep(100);
    }
    EXPECT(!bound, "a missing --config does not silently load the $HOME config");

    if (!atomic_load(&d.done)) pthread_kill(d.tid, SIGTERM);
    pthread_join(d.tid, NULL);

    if (saved_home) setenv("HOME", home_copy, 1);
    else            unsetenv("HOME");
    pthread_sigmask(SIG_SETMASK, &old, NULL);
    unlink(T_HOME_CONF);
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
    test_daemon_worker_can_signal_shutdown();
    test_daemon_replay_exits_at_end_of_capture();
    test_daemon_replay_stops_on_signal();
    test_daemon_replay_missing_capture_exits_1();
    test_daemon_nmea_thread_failure_closes_listener();
    test_daemon_hirate_thread_failure_sends_nothing();
    test_daemon_reload_reverts_a_deleted_key();
    test_daemon_reload_follows_the_home_fallback();
    test_daemon_explicit_config_does_not_fall_back();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
