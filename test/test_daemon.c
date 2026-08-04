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
 * Linux-only, like test_concurrency: it links the daemon objects and -lgpiod.
 */

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include "types.h"

int main_entry(int argc, char **argv);

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

/* nmea rate_hz is a [hot] key and imu odr_hz is a [restart] key; both are
 * printed by the status report, which is what makes the reload contract
 * observable from outside the process. */
static void write_conf(int nmea_rate_hz, int imu_odr_hz)
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
        "[highrate]\n"
        "enabled = false\n"
        "[stream]\n"
        "enabled = true\n"
        "socket = \"%s\"\n"
        "rate_hz = 50\n"
        "[logging]\n"
        "level = \"error\"\n",
        imu_odr_hz, nmea_rate_hz, T_STREAM_SOCK);
    fclose(f);
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

typedef struct { int rc; pthread_t tid; } daemon_run_t;

static void *daemon_thread(void *arg)
{
    daemon_run_t *d = (daemon_run_t *)arg;
    char *argv[] = { (char *)"imud", (char *)"--config", (char *)T_CONF, NULL };
    d->rc = main_entry(3, argv);
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
    test_daemon_lifecycle();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
