/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * main.c — imud daemon entry point (§10)
 *
 * Startup sequence:
 *    1. Parse CLI args; load config
 *    2. Load cal.json (warn if missing; not fatal)
 *    3. Open log file if configured; set log level
 *    4. Clock health check (TAI offset, CLOCK_REALTIME sanity)
 *    5. Ignore SIGPIPE; block SIGTERM/SIGINT/SIGHUP for sigwait()
 *    6. Open I2C + GPIO; probe + init both sensors  (imu_ctx_open)
 *    7. Open UDP output sockets                      (out_ctx_open)
 *    8. Open AF_UNIX status socket for imud-status
 *    9. Write /run/imud.pid; sd_notify("READY=1")
 *   10. Start threads: ism_reader, mag_reader, fusion, health,
 *       nmea_out (if enabled), hirate_out (if enabled), json_out (if enabled),
 *       position (if gpsd or signalk enabled)
 *   11. Wait for SIGTERM / SIGINT; hot-reload on SIGHUP
 *   12. Shutdown: send final packet, stop output threads, stop sensor threads
 *   13. Cleanup: free contexts, remove PID file and status socket
 */

#ifdef __linux__
# define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <sys/select.h>

#ifdef __linux__
# include <sys/timex.h>   /* adjtimex — TAI offset check */
#endif

#include "cal.h"
#include "config.h"
#include "imu.h"
#include "log.h"
#include "output.h"
#include "position.h"
#include "types.h"
#include "wmm.h"

int g_log_level = LOG_INFO;

/* ── Portability ─────────────────────────────────────────────────────────── */

#ifndef SOCK_CLOEXEC
# define SOCK_CLOEXEC 0
#endif
#ifndef SOCK_NONBLOCK
# define SOCK_NONBLOCK 0
#endif

/* ── Constants ───────────────────────────────────────────────────────────── */

#define VERSION_STR   "1.0"
#define PID_FILE      "/run/imud/imud.pid"   /* inside RuntimeDirectory=imud */
#define STATUS_SOCK   "/run/imud/imud.sock"
#define STATS_BUF     512
#define STATUS_BUF    2048

/* ── Global state ────────────────────────────────────────────────────────── */

static time_t g_start_time;

/* ── CLI args ────────────────────────────────────────────────────────────── */

typedef struct {
    char config_path[256];
    int  skip_bias_cal;
    int  no_nmea;
    int  no_hirate;
    int  no_json;
    int  version;
} cli_args_t;

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [OPTIONS]\n"
        "  --config PATH      Config file (default: /etc/imud/imud.conf)\n"
        "  --skip-bias-cal    Skip startup gyro bias estimation\n"
        "  --no-nmea          Disable NMEA output stream\n"
        "  --no-highrate      Disable high-rate binary stream\n"
        "  --no-json          Disable JSON output stream\n"
        "  --foreground       Compatibility no-op (imud always runs in foreground)\n"
        "  --version          Print version and exit\n",
        prog);
}

static int parse_args(int argc, char **argv, cli_args_t *a)
{
    memset(a, 0, sizeof(*a));
    snprintf(a->config_path, sizeof(a->config_path), "/etc/imud/imud.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(a->config_path, sizeof(a->config_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--skip-bias-cal") == 0) {
            a->skip_bias_cal = 1;
        } else if (strcmp(argv[i], "--no-nmea") == 0) {
            a->no_nmea = 1;
        } else if (strcmp(argv[i], "--no-highrate") == 0) {
            a->no_hirate = 1;
        } else if (strcmp(argv[i], "--no-json") == 0) {
            a->no_json = 1;
        } else if (strcmp(argv[i], "--version") == 0) {
            a->version = 1;
        } else if (strcmp(argv[i], "--foreground") == 0) {
            /* always foreground under systemd — accepted, ignored */
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return -1;
        }
    }
    return 0;
}

/* ── PID file ────────────────────────────────────────────────────────────── */

static void pid_write(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) { fprintf(stderr, "[main] cannot write PID file %s: %s\n",
                      path, strerror(errno)); return; }
    fprintf(f, "%d\n", (int)getpid());
    fclose(f);
}

static void pid_remove(const char *path)
{
    unlink(path);
}

/* ── sd_notify ───────────────────────────────────────────────────────────── */

/* Send one sd_notify(3)-style datagram to systemd; no-op outside systemd. */
static void sd_notify_msg(const char *msg)
{
    const char *sock = getenv("NOTIFY_SOCKET");
    if (!sock) return;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (sock[0] == '@') {               /* abstract socket */
        addr.sun_path[0] = '\0';
        strncpy(addr.sun_path + 1, sock + 1, sizeof(addr.sun_path) - 2);
    } else {
        strncpy(addr.sun_path, sock, sizeof(addr.sun_path) - 1);
    }

    int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return;
    sendto(fd, msg, strlen(msg), MSG_NOSIGNAL,
           (struct sockaddr *)&addr, sizeof(addr));
    close(fd);
}

static void sd_notify_ready(void)
{
    sd_notify_msg("READY=1");
}

/* ── Clock health check ──────────────────────────────────────────────────── */

static void clock_health_check(void)
{
    struct timespec rt;
    clock_gettime(CLOCK_REALTIME, &rt);

    /* Sanity: must be after 2024-01-01 */
    if (rt.tv_sec < 1704067200LL) {
        fprintf(stderr, "[clock] WARNING: CLOCK_REALTIME looks unsynchronised "
                "(before 2024-01-01) — timestamps unreliable\n");
    }

#ifdef __linux__
    struct timex tx;
    memset(&tx, 0, sizeof(tx));
    adjtimex(&tx);
    if (tx.tai == 0) {
        fprintf(stderr, "[clock] WARNING: CLOCK_TAI offset is 0 — "
                "chrony has not set tai_offset (leapsectz right/UTC?). "
                "ts_tai_ns will be unreliable.\n");
    } else {
        fprintf(stderr, "[clock] CLOCK_TAI offset: %d s  OK\n", tx.tai);
    }
#endif
}

/* ── Status socket + health thread ──────────────────────────────────────── */

/*
 * Open the AF_UNIX stream socket that imud-status connects to.
 * chmod 0660 so only members of the daemon's group can query it.
 * Returns listening fd, or -1 on error.
 */
static int status_sock_open(const char *path)
{
    unlink(path);   /* remove stale socket from a previous run */

    int fd = socket(AF_UNIX,
                    SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        fprintf(stderr, "[health] socket(%s): %s\n", path, strerror(errno));
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[health] bind(%s): %s\n", path, strerror(errno));
        close(fd); return -1;
    }
    chmod(path, 0660);
    listen(fd, 4);
    return fd;
}

typedef struct {
    const imud_config_t *cfg;
    imu_ctx_t           *imu;
    volatile int        *stop;
    int                  status_fd;   /* listening AF_UNIX fd */
} health_ctx_t;

static void write_status_response(int fd,
                                  const imud_config_t *cfg,
                                  imu_ctx_t *imu)
{
    fused_state_t state;
    mag_sample_t  mag;
    imu_get_state(imu, &state, &mag, NULL, NULL);

    imu_stats_t st;
    imu_get_stats(imu, &st);

    float pitch_deg   = state.pitch * (float)(180.0 / M_PI);
    float roll_deg    = state.roll  * (float)(180.0 / M_PI);
    float cov_trace   = state.cov[0] + state.cov[4] + state.cov[8];
    bool  converged   = (state.flags & FLAG_FUSION_CONVERGED) != 0;

    time_t elapsed = time(NULL) - g_start_time;
    int    hh = (int)(elapsed / 3600);
    int    mm = (int)((elapsed % 3600) / 60);
    int    ss = (int)(elapsed % 60);

    /* Build the response into a stack buffer and write in one call.
     * WS() tracks (wp, wr) so snprintf truncation never advances wp past
     * the buffer end — safe even if a single line exceeds remaining space. */
    char   buf[STATUS_BUF];
    char  *wp  = buf;
    size_t wr  = sizeof(buf);

#define WS(fmt, ...) do { \
        if (wr > 1) { \
            int _r = snprintf(wp, wr, fmt, ##__VA_ARGS__); \
            if (_r > 0 && (size_t)_r < wr) { wp += _r; wr -= (size_t)_r; } \
            else if (_r > 0)                { wp += wr - 1; wr = 1; } \
        } } while (0)

    WS("Chip IDs:       %s 0x%02X   %s 0x%02X\n",
        cfg->imu_driver, cfg->imu_addr,
        cfg->mag_driver, cfg->mag_addr);

    WS("IMU ODR:        %d Hz  (FIFO watermark: %d sample-sets)\n",
        cfg->imu_odr_hz, cfg->imu_fifo_wm);

    WS("Mag ODR:        %d Hz  (SET every %.0f s%s)\n",
        cfg->mag_odr_hz, cfg->mag_set_period_s,
        cfg->mag_set_period_s > 0 ? "" : ", disabled");

    WS("Fusion:         MEKF %s  cov_trace=%.2e rad2\n",
        converged ? "converged" : "converging",
        cov_trace);

    WS("Calibration:    accel %s  gyro %s  mag %s\n",
        (state.flags & FLAG_ACCEL_CAL) ? "yes" : "no",
        (state.flags & FLAG_GYRO_CAL)  ? "yes" : "no",
        (state.flags & FLAG_MAG_CAL)   ? "yes" : "no");

    WS("Attitude:       pitch=%.1f  roll=%.1f  heading=%.1f M\n",
        pitch_deg, roll_deg, state.heading_deg);

    if (cfg->nmea_enabled) {
        WS("NMEA out:       %d Hz  (port %d)\n",
            cfg->nmea_rate_hz, cfg->nmea_dest_port);
    } else {
        WS("NMEA out:       disabled\n");
    }

    if (cfg->highrate_enabled) {
        WS("Hi-rate out:    %d Hz  (port %d, %s)\n",
            cfg->highrate_rate_hz, cfg->highrate_dest_port,
            cfg->highrate_coord_frame);
    } else {
        WS("Hi-rate out:    disabled\n");
    }

    if (cfg->json_enabled) {
        WS("JSON out:       %d Hz  (port %d)\n",
            cfg->json_rate_hz, cfg->json_dest_port);
    } else {
        WS("JSON out:       disabled\n");
    }

    WS("IMU samples:    %llu  overflows: %llu\n",
        (unsigned long long)st.imu_samples,
        (unsigned long long)st.fifo_overflows);

    WS("Uptime:         %02d:%02d:%02d\n", hh, mm, ss);

#undef WS

    /* Write all at once; ignore partial-write on client disconnect. */
    (void)write(fd, buf, (size_t)(wp - buf));
}

static void ts_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

static void *health_thread(void *arg)
{
    health_ctx_t *ctx = arg;

    long stats_period_ns = (ctx->cfg->log_stats_hz > 0)
        ? 1000000000L / ctx->cfg->log_stats_hz
        : 1000000000L;

    struct timespec next_stats;
    clock_gettime(CLOCK_MONOTONIC, &next_stats);
    ts_add_ns(&next_stats, stats_period_ns);

    while (!*ctx->stop) {
        /* systemd watchdog heartbeat — this loop ticks at least once per
         * second, well inside the unit's WatchdogSec window; a hung daemon
         * stops petting the dog and systemd restarts it. No-op when not
         * run under systemd or when WatchdogSec is unset. */
        sd_notify_msg("WATCHDOG=1");

        /* Poll the status socket with 1 s timeout. */
        fd_set rfds;
        FD_ZERO(&rfds);
        if (ctx->status_fd >= 0) FD_SET(ctx->status_fd, &rfds);
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        int nfds = (ctx->status_fd >= 0) ? ctx->status_fd + 1 : 1;

        if (select(nfds, &rfds, NULL, NULL, &tv) > 0 &&
            ctx->status_fd >= 0 &&
            FD_ISSET(ctx->status_fd, &rfds)) {

            int client = accept(ctx->status_fd, NULL, NULL);
            if (client >= 0) {
                /* 1 s receive timeout so a stalled client can't block us. */
                struct timeval rto = { .tv_sec = 1, .tv_usec = 0 };
                setsockopt(client, SOL_SOCKET, SO_RCVTIMEO,
                           &rto, sizeof(rto));
                write_status_response(client, ctx->cfg, ctx->imu);
                close(client);
            }
        }

        /* Periodic stats log — suppressed until fusion is aligned */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if ((now.tv_sec > next_stats.tv_sec) ||
            (now.tv_sec == next_stats.tv_sec &&
             now.tv_nsec >= next_stats.tv_nsec)) {

            /* Advance deadline regardless so we don't burst when settled */
            ts_add_ns(&next_stats, stats_period_ns);

            if (!imu_ctx_is_settled(ctx->imu)) continue;

            imu_stats_t st;
            imu_get_stats(ctx->imu, &st);
            fused_state_t state;
            imu_get_state(ctx->imu, &state, NULL, NULL, NULL);

            LOG_I("[stats] imu=%llu ovf=%llu mag=%llu  "
                "hdg=%.1f pitch=%.1f roll=%.1f  "
                "cov=%.1e %s\n",
                (unsigned long long)st.imu_samples,
                (unsigned long long)st.fifo_overflows,
                (unsigned long long)st.mag_samples,
                state.heading_deg,
                state.pitch * (float)(180.0 / M_PI),
                state.roll  * (float)(180.0 / M_PI),
                state.cov[0] + state.cov[4] + state.cov[8],
                (state.flags & FLAG_FUSION_CONVERGED) ? "converged" : "converging");
        }
    }

    return NULL;
}

/* ── Thread join helper ──────────────────────────────────────────────────── */

static void join_thread(pthread_t tid, const char *name)
{
    if (pthread_join(tid, NULL) != 0)
        fprintf(stderr, "[main] join %s failed\n", name);
}

/* ── WMM declination ─────────────────────────────────────────────────────── */

/*
 * apply_wmm_if_configured — when lat_deg and lon_deg are both non-zero,
 * load WMM.COF, compute declination, and write it into cfg->pos_declination_deg
 * (overriding any static value).  Called at startup and on SIGHUP.
 * Not fatal: logs a warning and leaves pos_declination_deg unchanged on error.
 */
static void apply_wmm_if_configured(imud_config_t *cfg)
{
    if (cfg->pos_lat_deg == 0.0 && cfg->pos_lon_deg == 0.0)
        return;

    wmm_t wmm;
    if (wmm_load(cfg->pos_wmm_file, &wmm) != 0) {
        fprintf(stderr, "[pos] WMM file '%s' failed to load — "
                "declination not computed\n", cfg->pos_wmm_file);
        return;
    }

    double year     = wmm_decimal_year();
    double valid_end = wmm.epoch + 5.0;
    if (year < wmm.epoch || year > valid_end + 0.5) {
        fprintf(stderr, "[pos] WARNING: WMM epoch %.1f, current year %.2f — "
                "coefficients may be stale (update %s)\n",
                wmm.epoch, year, cfg->pos_wmm_file);
    }

    double decl = wmm_declination(cfg->pos_lat_deg, cfg->pos_lon_deg,
                                  0.0, year, &wmm);
    cfg->pos_declination_deg   = (float)decl;
    cfg->pos_declination_valid = true;   /* WMM result is valid even at 0.0° */
    fprintf(stderr, "[pos] WMM declination at (%.4f°N, %.4f°E): %.2f°E\n",
            cfg->pos_lat_deg, cfg->pos_lon_deg, decl);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    g_start_time = time(NULL);

    /* ── 1. Args + config ───────────────────────────────────────────────── */

    cli_args_t args;
    if (parse_args(argc, argv, &args) < 0) return 1;

    if (args.version) {
        printf("imud %s\n", VERSION_STR);
        return 0;
    }

    /* Try --config path, then /etc/imud/imud.conf, then ~/.config/imud/imud.conf */
    imud_config_t cfg;
    config_defaults(&cfg);

    /* A config file that exists but fails to parse is fatal: starting with
     * a half-applied config (everything after the bad line discarded) is
     * worse than not starting at all. A missing file is fine — defaults. */
    int cfg_rc = config_load(args.config_path, &cfg);
    if (cfg_rc == CONFIG_ERR_PARSE) {
        fprintf(stderr, "[main] %s has errors (see above) — refusing to start\n",
                args.config_path);
        return 1;
    }
    if (cfg_rc == CONFIG_ERR_OPEN) {
        /* Fallback: try the other default */
        char alt[256];
        const char *home = getenv("HOME");
        if (home)
            snprintf(alt, sizeof(alt), "%s/.config/imud/imud.conf", home);
        else
            snprintf(alt, sizeof(alt), "/etc/imud/imud.conf");

        if (strcmp(args.config_path, alt) != 0 &&
            config_load(alt, &cfg) == CONFIG_ERR_PARSE) {
            fprintf(stderr, "[main] %s has errors (see above) — refusing to start\n",
                    alt);
            return 1;
        }
        /* Neither file existing is fine — defaults remain. */
    }

    /* Apply CLI overrides */
    if (args.no_nmea)        cfg.nmea_enabled     = false;
    if (args.no_hirate)      cfg.highrate_enabled = false;
    if (args.no_json)        cfg.json_enabled     = false;
    if (args.skip_bias_cal)  cfg.gyro_bias_sec    = 0.0;

    apply_wmm_if_configured(&cfg);

    /* ── 2. Calibration ─────────────────────────────────────────────────── */

    imud_cal_t cal;
    if (cal_load(cfg.cal_file, &cal) < 0) return 1;

    /* ── 3. Log file ────────────────────────────────────────────────────── */

    if (cfg.log_file[0]) {
        int lfd = open(cfg.log_file, O_WRONLY | O_CREAT | O_APPEND, 0640);
        if (lfd < 0)
            fprintf(stderr, "[main] cannot open log %s: %s — using stderr\n",
                    cfg.log_file, strerror(errno));
        else {
            dup2(lfd, STDERR_FILENO);
            close(lfd);
        }
    }

    if      (strcmp(cfg.log_level, "debug") == 0) g_log_level = LOG_DEBUG;
    else if (strcmp(cfg.log_level, "warn")  == 0) g_log_level = LOG_WARN;
    else if (strcmp(cfg.log_level, "error") == 0) g_log_level = LOG_ERROR;
    else                                           g_log_level = LOG_INFO;

    /* ── 4. Clock health ────────────────────────────────────────────────── */

    clock_health_check();

    /* ── 5. Ignore SIGPIPE (broken pipe on status socket writes) ─────────── */

    signal(SIGPIPE, SIG_IGN);

    /* ── 6. Block signals — main thread handles them via sigwait ────────── */

    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGTERM);
    sigaddset(&sigset, SIGINT);
    sigaddset(&sigset, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &sigset, NULL);

    /* ── 7. Open hardware contexts ───────────────────────────────────────── */

    imu_ctx_t *imu = NULL;
    if (imu_ctx_open(&imu, &cfg, &cal) < 0) {
        fprintf(stderr, "[main] fatal: sensor init failed\n");
        return 1;
    }

    out_ctx_t *out = NULL;
    if (out_ctx_open(&out, &cfg, imu) < 0) {
        fprintf(stderr, "[main] fatal: output socket init failed\n");
        imu_ctx_free(imu);
        return 1;
    }

    /* ── 8. Status socket ────────────────────────────────────────────────── */

    int status_fd = status_sock_open(STATUS_SOCK);
    /* Not fatal if this fails — imud-status won't work but the daemon runs. */

    /* ── 9. PID file + systemd notification ─────────────────────────────── */

    pid_write(PID_FILE);
    sd_notify_ready();

    /* ── 10. Start threads ───────────────────────────────────────────────── */

    volatile int stop = 0;

    /* Reader threads first — must be running before fusion blocks on ring. */
    pthread_t ism_tid, mag_tid, fusion_tid, health_tid;
    pthread_t nmea_tid = 0, hirate_tid = 0, json_tid = 0, stream_tid = 0,
              pos_tid = 0;
    bool nmea_started = false, hirate_started = false, json_started = false;
    bool stream_started = false, pos_started = false;

    if (pthread_create(&ism_tid, NULL, ism_reader_thread, imu) != 0) {
        fprintf(stderr, "[main] fatal: cannot create ism_reader thread: %s\n", strerror(errno));
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }
    if (pthread_create(&mag_tid, NULL, mag_reader_thread, imu) != 0) {
        fprintf(stderr, "[main] fatal: cannot create mag_reader thread: %s\n", strerror(errno));
        imu_ctx_stop(imu); join_thread(ism_tid, "ism_reader");
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }
    if (pthread_create(&fusion_tid, NULL, fusion_thread, imu) != 0) {
        fprintf(stderr, "[main] fatal: cannot create fusion thread: %s\n", strerror(errno));
        imu_ctx_stop(imu); join_thread(mag_tid, "mag_reader"); join_thread(ism_tid, "ism_reader");
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }

    health_ctx_t hctx = {
        .cfg       = &cfg,
        .imu       = imu,
        .stop      = &stop,
        .status_fd = status_fd,
    };
    if (pthread_create(&health_tid, NULL, health_thread, &hctx) != 0) {
        fprintf(stderr, "[main] fatal: cannot create health thread: %s\n", strerror(errno));
        imu_ctx_stop(imu);
        join_thread(fusion_tid, "fusion"); join_thread(mag_tid, "mag_reader");
        join_thread(ism_tid, "ism_reader");
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }

    if (cfg.nmea_enabled) {
        if (pthread_create(&nmea_tid, NULL, nmea_out_thread, out) != 0) {
            fprintf(stderr, "[main] warning: cannot create nmea_out thread: %s\n", strerror(errno));
            cfg.nmea_enabled = false;
        } else {
            nmea_started = true;
        }
    }
    if (cfg.highrate_enabled) {
        if (pthread_create(&hirate_tid, NULL, hirate_out_thread, out) != 0) {
            fprintf(stderr, "[main] warning: cannot create hirate_out thread: %s\n", strerror(errno));
            cfg.highrate_enabled = false;
        } else {
            hirate_started = true;
        }
    }
    if (cfg.json_enabled) {
        if (pthread_create(&json_tid, NULL, json_out_thread, out) != 0) {
            fprintf(stderr, "[main] warning: cannot create json_out thread: %s\n", strerror(errno));
            cfg.json_enabled = false;
        } else {
            json_started = true;
        }
    }
    if (cfg.stream_enabled) {
        if (pthread_create(&stream_tid, NULL, stream_out_thread, out) != 0) {
            fprintf(stderr, "[main] warning: cannot create stream_out thread: %s\n", strerror(errno));
            cfg.stream_enabled = false;
        } else {
            stream_started = true;
        }
    }

    /* Position thread — optional; only runs when gpsd or signalk is enabled. */
    static pos_ctx_t pos_ctx;   /* static: lifetime matches daemon */
    pos_ctx.cfg  = &cfg;
    pos_ctx.imu  = imu;
    pos_ctx.stop = 0;
    if (cfg.pos_gpsd_enabled || cfg.pos_signalk_enabled) {
        if (pthread_create(&pos_tid, NULL, position_thread, &pos_ctx) != 0) {
            fprintf(stderr, "[main] warning: cannot create position thread: %s\n",
                    strerror(errno));
        } else {
            pos_started = true;
        }
    }

    fprintf(stderr, "[main] imud %s running (pid %d)\n",
            VERSION_STR, (int)getpid());

    /* ── 11. Signal loop ─────────────────────────────────────────────────── */

    int sig;
    for (;;) {
        sigwait(&sigset, &sig);

        if (sig == SIGTERM || sig == SIGINT) {
            fprintf(stderr, "[main] caught signal %d — shutting down\n", sig);
            break;
        }

        if (sig == SIGHUP) {
            fprintf(stderr, "[main] SIGHUP — reloading config\n");
            imud_config_t new_cfg = cfg;
            if (config_load(args.config_path, &new_cfg) == 0) {
                apply_wmm_if_configured(&new_cfg);
                /* Apply hot-reloadable fields atomically (single assignment). */
                cfg.nmea_rate_hz        = new_cfg.nmea_rate_hz;
                cfg.highrate_rate_hz    = new_cfg.highrate_rate_hz;
                cfg.json_rate_hz        = new_cfg.json_rate_hz;
                cfg.stream_rate_hz      = new_cfg.stream_rate_hz;
                cfg.log_stats_hz        = new_cfg.log_stats_hz;
                /* Fusion noise params + declination — push into running filter */
                cfg.mekf_gyro_noise     = new_cfg.mekf_gyro_noise;
                cfg.mekf_gyro_bias      = new_cfg.mekf_gyro_bias;
                cfg.mekf_accel_noise    = new_cfg.mekf_accel_noise;
                cfg.mekf_mag_noise      = new_cfg.mekf_mag_noise;
                cfg.mag_reject_gauss           = new_cfg.mag_reject_gauss;
                cfg.accel_skip_thresh          = new_cfg.accel_skip_thresh;
                cfg.engine_vibration_g2        = new_cfg.engine_vibration_g2;
                cfg.engine_accel_skip_thresh   = new_cfg.engine_accel_skip_thresh;
                cfg.pos_declination_deg        = new_cfg.pos_declination_deg;
                cfg.pos_declination_valid      = new_cfg.pos_declination_valid;
                imu_ctx_update_config(imu, &cfg);
                fprintf(stderr, "[main] config reloaded\n");
            } else {
                fprintf(stderr, "[main] config reload failed — "
                        "keeping current config\n");
            }
        }
    }

    /* ── 12. Shutdown ────────────────────────────────────────────────────── */

    stop = 1;  /* health thread exits */

    /* Emit shutdown packet before stopping output threads. */
    out_ctx_send_shutdown(out);

    /* Stop position thread first — it's independent of the sensor pipeline. */
    if (pos_started) {
        pos_ctx.stop = 1;
        join_thread(pos_tid, "position");
    }

    out_ctx_stop(out);
    if (nmea_started)   join_thread(nmea_tid,   "nmea_out");
    if (hirate_started) join_thread(hirate_tid, "hirate_out");
    if (json_started)   join_thread(json_tid,   "json_out");
    if (stream_started) join_thread(stream_tid, "stream_out");

    /* Stop sensor and fusion threads. */
    imu_ctx_stop(imu);
    join_thread(health_tid, "health");
    join_thread(fusion_tid, "fusion");
    join_thread(mag_tid,    "mag_reader");
    join_thread(ism_tid,    "ism_reader");

    /* ── 13. Cleanup ─────────────────────────────────────────────────────── */

    out_ctx_free(out);
    imu_ctx_free(imu);

    if (status_fd >= 0) {
        close(status_fd);
        unlink(STATUS_SOCK);
    }
    pid_remove(PID_FILE);

    fprintf(stderr, "[main] exit\n");
    return 0;
}
