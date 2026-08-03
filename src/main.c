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
 *       nmea_out (if enabled), hirate_out (if enabled), stream_out (if enabled),
 *       position (if gpsd or signalk enabled)
 *   11. Wait for SIGTERM / SIGINT; hot-reload on SIGHUP
 *   12. Shutdown: send final packet, stop output threads, stop sensor threads
 *   13. Cleanup: free contexts, remove PID file and status socket
 */

/* The Makefile also passes -D_GNU_SOURCE; guard so a standalone compile
 * still works without redefining it. */
#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
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
#include "cli.h"
#include "config.h"
#include "fileio.h"
#include "imu.h"
#include "imu_math.h"    /* ts_add_ns */
#include "log.h"
#include "output.h"
#include "position.h"
#include "sdnotify.h"
#include "types.h"
#include "wmm.h"

/* ── Portability ─────────────────────────────────────────────────────────── */

#ifndef SOCK_CLOEXEC
# define SOCK_CLOEXEC 0
#endif
#ifndef SOCK_NONBLOCK
# define SOCK_NONBLOCK 0
#endif

/* ── Constants ───────────────────────────────────────────────────────────── */

#include "version.h"                         /* IMUD_VERSION_STR — canonical */
#define VERSION_STR   IMUD_VERSION_STR
#define PID_FILE      "/run/imud/imud.pid"   /* inside RuntimeDirectory=imud */
#define STATUS_SOCK   "/run/imud/imud.sock"
#define STATS_BUF     512
#define STATUS_BUF    4096   /* status text + recent-warnings section */

/* ── Global state ────────────────────────────────────────────────────────── */

static time_t g_start_time;

/* Serializes SIGHUP writes to the shared top-level `cfg` against the health
 * thread's reads (stats period + status-response snapshot). The output
 * threads instead read their own _Atomic rate copies (out_ctx_reload); the
 * fusion/reader threads have their own live_lock snapshot (imu_ctx). */
static pthread_mutex_t g_cfg_lock = PTHREAD_MUTEX_INITIALIZER;

/* ── PID file ────────────────────────────────────────────────────────────── */

static void pid_write(const char *path)
{
    FILE *f = fcreate(path, "w", IMUD_FILE_MODE);
    if (!f) { LOG_E("[main] cannot write PID file %s: %s\n",
                      path, strerror(errno)); return; }
    fprintf(f, "%d\n", (int)getpid());
    fclose(f);
}

static void pid_remove(const char *path)
{
    unlink(path);
}

/* ── sd_notify ───────────────────────────────────────────────────────────── */

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
        LOG_W("[clock] WARNING: CLOCK_REALTIME looks unsynchronised "
                "(before 2024-01-01) — timestamps unreliable\n");
    }

#ifdef __linux__
    /* adjtimex(2) is in systemd's @clock set, hence in @privileged, and
     * ProtectClock= blocks it outright — even this read-only modes = 0 call.
     * imud.service therefore sets no ProtectClock= and re-allows adjtimex
     * after its ~@privileged line.  Test the return anyway: without it a
     * blocked call lands in the tx.tai == 0 branch below and blames chrony
     * for a filter the operator installed. */
    struct timex tx;
    memset(&tx, 0, sizeof(tx));
    if (adjtimex(&tx) < 0) {
        LOG_W("[clock] WARNING: cannot query the TAI offset (%s) — "
                "ts_tai_ns cannot be checked. A unit override dropping "
                "adjtimex from SystemCallFilter= will do this.\n",
                strerror(errno));
    } else if (tx.tai == 0) {
        LOG_W("[clock] WARNING: CLOCK_TAI offset is 0 — "
                "chrony has not set tai_offset (leapsectz right/UTC?). "
                "ts_tai_ns will be unreliable.\n");
    } else {
        LOG_I("[clock] CLOCK_TAI offset: %d s  OK\n", tx.tai);
    }
#endif
}

/* ── Status socket + health thread ──────────────────────────────────────── */

/*
 * Open the AF_UNIX stream socket that imud-status connects to.
 * Mode 0660 so only members of the daemon's group — imud, the primary group
 * of the user in the shipped unit — can query it.  bind_unix_mode() applies
 * that from creation rather than chmod'ing down afterwards, so the socket is
 * never briefly wider than the umask happened to allow.
 * Returns listening fd, or -1 on error.
 */
static int status_sock_open(const char *path)
{
    unlink(path);   /* remove stale socket from a previous run */

    int fd = socket(AF_UNIX,
                    SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        LOG_E("[health] socket(%s): %s\n", path, strerror(errno));
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind_unix_mode(fd, (struct sockaddr *)&addr, sizeof(addr),
                       path, 0660) < 0) {
        LOG_E("[health] bind/chmod(%s): %s\n", path, strerror(errno));
        close(fd); return -1;
    }
    listen(fd, 4);
    return fd;
}

typedef struct {
    const imud_config_t *cfg;
    imu_ctx_t           *imu;
    _Atomic int         *stop;
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

    if (state.flags & FLAG_DECLINATION_VALID) {
        float true_hdg = fmodf(state.heading_deg + state.declination_deg
                               + 360.0f, 360.0f);
        WS("Declination:    %+.2f E  (true heading %.1f T)\n",
            state.declination_deg, true_hdg);
    } else {
        WS("Declination:    unknown  (no true heading output)\n");
    }

    if (cfg->heave_tau_s > 0.0f)
        WS("Heave:          %+.2f m\n", state.heave_m);

    if (cfg->heave_tau_s > 0.0f && cfg->wave_tau_s > 0.0f) {
        if (state.flags & FLAG_WAVE_VALID)
            WS("Sea state:      Hs %.2f m  Tz %.1f s  roll period %.1f s\n",
                state.wave_height_m, state.wave_period_s, state.roll_period_s);
        else
            WS("Sea state:      settling\n");
    }

    if (cfg->capture_enabled) {
        char     cpath[320];
        uint64_t cbytes, cdrops;
        bool     cactive;
        imu_get_capture_status(imu, cpath, sizeof(cpath),
                               &cbytes, &cdrops, &cactive);
        if (cactive)
            WS("Capture:        %s  (%.1f MB, %llu dropped)\n",
                cpath, (double)cbytes / (1024.0 * 1024.0),
                (unsigned long long)cdrops);
        else
            WS("Capture:        stopped (see log)\n");
    }

    if (cfg->nmea_enabled && cfg->nmea_tcp_enabled) {
        WS("NMEA out:       %d Hz  (UDP port %d, TCP port %d)\n",
            cfg->nmea_rate_hz, cfg->nmea_dest_port, cfg->nmea_tcp_port);
    } else if (cfg->nmea_enabled) {
        WS("NMEA out:       %d Hz  (port %d)\n",
            cfg->nmea_rate_hz, cfg->nmea_dest_port);
    } else if (cfg->nmea_tcp_enabled) {
        WS("NMEA out:       %d Hz  (TCP port %d)\n",
            cfg->nmea_rate_hz, cfg->nmea_tcp_port);
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

    WS("IMU samples:    %llu  overflows: %llu\n",
        (unsigned long long)st.imu_samples,
        (unsigned long long)st.fifo_overflows);

    WS("Uptime:         %02d:%02d:%02d\n", hh, mm, ss);

    /* Last few WARN/ERROR lines — what went wrong while unattended. */
    {
        char recent[1200];
        if (log_recent(recent, sizeof recent) > 0)
            WS("Recent warnings:\n%s", recent);
    }

#undef WS

    /* Write all at once; ignore partial-write on client disconnect.  glibc
     * marks write() warn_unused_result and gcc deliberately does not honour a
     * (void) cast for that, so the result has to land somewhere. */
    ssize_t nw = write(fd, buf, (size_t)(wp - buf));
    (void)nw;
}

static void *health_thread(void *arg)
{
    health_ctx_t *ctx = arg;

    pthread_mutex_lock(&g_cfg_lock);
    int log_stats_hz = ctx->cfg->log_stats_hz;
    pthread_mutex_unlock(&g_cfg_lock);
    long stats_period_ns = (log_stats_hz > 0)
        ? 1000000000L / log_stats_hz
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
                imud_config_t cfg_snap;
                pthread_mutex_lock(&g_cfg_lock);
                cfg_snap = *ctx->cfg;
                pthread_mutex_unlock(&g_cfg_lock);
                write_status_response(client, &cfg_snap, ctx->imu);
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
    int rc = pthread_join(tid, NULL);
    if (rc != 0)
        LOG_E("[main] join %s failed: %s\n", name, strerror(rc));
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
        LOG_E("[pos] WMM file '%s' failed to load — "
                "declination not computed\n", cfg->pos_wmm_file);
        return;
    }

    double year     = wmm_decimal_year();
    double valid_end = wmm.epoch + 5.0;
    if (year < wmm.epoch || year > valid_end + 0.5) {
        LOG_W("[pos] WARNING: WMM epoch %.1f, current year %.2f — "
                "coefficients may be stale (update %s)\n",
                wmm.epoch, year, cfg->pos_wmm_file);
    }

    double ned[3];
    wmm_field_ned(cfg->pos_lat_deg, cfg->pos_lon_deg, 0.0, year, &wmm, ned);
    /* Declination + MEKF field invariants (nT → Gauss) from the one vector. */
    double decl;
    wmm_derive_refs(ned, &decl, &cfg->pos_mref_h_gauss, &cfg->pos_mref_z_gauss);
    cfg->pos_declination_deg   = (float)decl;
    cfg->pos_declination_valid = true;   /* WMM result is valid even at 0.0° */
    cfg->pos_mref_valid        = true;

    LOG_I("[pos] WMM at (%.4f°N, %.4f°E): decl %.2f°E  "
            "H %.3f G  Z %.3f G\n",
            cfg->pos_lat_deg, cfg->pos_lon_deg, decl,
            cfg->pos_mref_h_gauss, cfg->pos_mref_z_gauss);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    g_start_time = time(NULL);

    /* The PID file, cal.json and the .imucap captures are created through
     * fcreate() with an explicit 0644 (see include/fileio.h), so their mode
     * no longer depends on how the daemon was launched.  This stays as a
     * backstop for anything created by a library beneath us: a manual start
     * inherits the invoking shell's umask, and umask 0 would leave such a
     * file world-WRITABLE.
     *
     * Tighten-only, never loosen: 022 is a floor, not a setting.  The shipped
     * unit asks for UMask=0027, and overwriting that with 022 would hand back
     * the "other" bits systemd had just taken away. */
    mode_t inherited_umask = umask(022);
    umask(inherited_umask | 022);

    /* ── 1. Args + config ───────────────────────────────────────────────── */

    cli_imud_t args;
    int cli_rc = cli_parse_imud(argc, argv, &args);
    if (cli_rc != 0) return cli_rc < 0 ? 1 : 0;   /* -1 bad usage, 1 --version */

    /* Try --config path, then /etc/imud/imud.conf, then ~/.config/imud/imud.conf */
    imud_config_t cfg;
    config_defaults(&cfg);

    /* A config file that exists but fails to parse is fatal: starting with
     * a half-applied config (everything after the bad line discarded) is
     * worse than not starting at all. A missing file is fine — defaults. */
    int cfg_rc = config_load(args.config_path, &cfg);
    if (cfg_rc == CONFIG_ERR_PARSE) {
        LOG_E("[main] %s has errors (see above) — refusing to start\n",
                args.config_path);
        return 1;
    }
    /* Existing-but-unreadable is fatal for the same reason, and must not fall
     * through to the alt path below: silently running on defaults when the
     * operator has written a config is the worst of both. */
    if (cfg_rc == CONFIG_ERR_PERM) {
        LOG_E("[main] %s exists but cannot be read (see above) — refusing to "
                "start\n", args.config_path);
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

        if (strcmp(args.config_path, alt) != 0) {
            int alt_rc = config_load(alt, &cfg);
            if (alt_rc == CONFIG_ERR_PARSE) {
                LOG_E("[main] %s has errors (see above) — refusing to start\n",
                        alt);
                return 1;
            }
            if (alt_rc == CONFIG_ERR_PERM) {
                LOG_E("[main] %s exists but cannot be read (see above) — "
                        "refusing to start\n", alt);
                return 1;
            }
        }
        /* Neither file existing is fine — defaults remain. */
    }

    /* Apply CLI overrides */
    if (args.no_nmea) {      cfg.nmea_enabled     = false;
                             cfg.nmea_tcp_enabled = false; }
    if (args.no_hirate)      cfg.highrate_enabled = false;
    if (args.skip_bias_cal)  cfg.gyro_bias_sec    = 0.0;
    if (args.replay_path[0]) {
        /* gpsfake-style: force both sensors onto the sim driver playing the
         * capture, polling path (no GPIO lines on replayed data). */
        snprintf(cfg.imu_driver, sizeof(cfg.imu_driver), "sim");
        snprintf(cfg.mag_driver, sizeof(cfg.mag_driver), "sim");
        snprintf(cfg.sim_file,   sizeof(cfg.sim_file),   "%s", args.replay_path);
        cfg.imu_int_gpio = 0;
        cfg.mag_int_gpio = 0;
        LOG_I("[main] replay mode: %s\n", cfg.sim_file);
    }

    apply_wmm_if_configured(&cfg);

    /* ── 2. Calibration ─────────────────────────────────────────────────── */

    imud_cal_t cal;
    if (cal_load(cfg.cal_file, &cal) < 0) return 1;

    /* cal.json's "noise" section (imud-cal characterize) is per-unit sensor
     * characterization for the record ONLY — it never feeds the filter.  The
     * mekf_* values are tuned constants: mekf_gyro_noise/mekf_gyro_bias build
     * the process noise Q (fusion.c), deliberately held above the raw sensor
     * floor so the filter stays responsive and the gyro bias observable.
     * Driving Q from the measured floor makes the filter too stiff (verified:
     * the test_fusion wave benchmark fails, attitude RMS roughly doubles), so
     * there is deliberately no configuration path that does it. */

    /* ── 3. Log destination, style, and level ───────────────────────────── */

    if (cfg.log_file[0]) {
        int lfd = open(cfg.log_file, O_WRONLY | O_CREAT | O_APPEND, 0640);
        if (lfd < 0)
            LOG_E("[main] cannot open log %s: %s — using stderr\n",
                    cfg.log_file, strerror(errno));
        else {
            dup2(lfd, STDERR_FILENO);
            close(lfd);
            log_set_style(LOG_STYLE_FILE);      /* raw file: timestamp lines */
        }
    } else if (getenv("INVOCATION_ID")) {
        /* Under systemd: <N> priority prefixes so journald records real
         * priorities (SyslogLevelPrefix is on by default). */
        log_set_style(LOG_STYLE_JOURNAL);
    }

    log_set_level_str(cfg.log_level);

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
        LOG_E("[main] fatal: sensor init failed\n");
        return 1;
    }

    out_ctx_t *out = NULL;
    if (out_ctx_open(&out, &cfg, imu) < 0) {
        LOG_E("[main] fatal: output socket init failed\n");
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

    _Atomic int stop = 0;

    /* Reader threads first — must be running before fusion blocks on ring. */
    pthread_t ism_tid, mag_tid, fusion_tid, health_tid;
    pthread_t nmea_tid = 0, hirate_tid = 0, stream_tid = 0, pos_tid = 0;
    bool nmea_started = false, hirate_started = false;
    bool stream_started = false, pos_started = false;
    pthread_t capture_tid;
    bool capture_started = false;

    int prc;
    prc = pthread_create(&ism_tid, NULL, ism_reader_thread, imu);
    if (prc != 0) {
        LOG_E("[main] fatal: cannot create ism_reader thread: %s\n", strerror(prc));
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }
    prc = pthread_create(&mag_tid, NULL, mag_reader_thread, imu);
        if (prc != 0) {
        LOG_E("[main] fatal: cannot create mag_reader thread: %s\n", strerror(prc));
        imu_ctx_stop(imu); join_thread(ism_tid, "ism_reader");
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }
    prc = pthread_create(&fusion_tid, NULL, fusion_thread, imu);
        if (prc != 0) {
        LOG_E("[main] fatal: cannot create fusion thread: %s\n", strerror(prc));
        imu_ctx_stop(imu); join_thread(mag_tid, "mag_reader"); join_thread(ism_tid, "ism_reader");
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }

    if (cfg.capture_enabled) {
        prc = pthread_create(&capture_tid, NULL, capture_thread, imu);
        if (prc != 0)
            LOG_E("[main] cannot create capture thread: %s — "
                  "black box disabled\n", strerror(prc));
        else
            capture_started = true;
    }

    health_ctx_t hctx = {
        .cfg       = &cfg,
        .imu       = imu,
        .stop      = &stop,
        .status_fd = status_fd,
    };
    prc = pthread_create(&health_tid, NULL, health_thread, &hctx);
        if (prc != 0) {
        LOG_E("[main] fatal: cannot create health thread: %s\n", strerror(prc));
        imu_ctx_stop(imu);
        join_thread(fusion_tid, "fusion"); join_thread(mag_tid, "mag_reader");
        join_thread(ism_tid, "ism_reader");
        out_ctx_free(out); imu_ctx_free(imu);
        if (status_fd >= 0) { close(status_fd); unlink(STATUS_SOCK); }
        pid_remove(PID_FILE); return 1;
    }

    if (cfg.nmea_enabled || cfg.nmea_tcp_enabled) {
        prc = pthread_create(&nmea_tid, NULL, nmea_out_thread, out);
        if (prc != 0) {
            LOG_W("[main] warning: cannot create nmea_out thread: %s\n", strerror(prc));
            cfg.nmea_enabled     = false;
            cfg.nmea_tcp_enabled = false;
        } else {
            nmea_started = true;
        }
    }
    if (cfg.highrate_enabled) {
        prc = pthread_create(&hirate_tid, NULL, hirate_out_thread, out);
        if (prc != 0) {
            LOG_W("[main] warning: cannot create hirate_out thread: %s\n", strerror(prc));
            cfg.highrate_enabled = false;
        } else {
            hirate_started = true;
        }
    }
    if (cfg.stream_enabled || cfg.stream_tcp_enabled) {
        prc = pthread_create(&stream_tid, NULL, stream_out_thread, out);
        if (prc != 0) {
            LOG_W("[main] warning: cannot create stream_out thread: %s\n", strerror(prc));
            cfg.stream_enabled     = false;
            cfg.stream_tcp_enabled = false;
        } else {
            stream_started = true;
        }
    }

    /* Position thread — optional; only runs when gpsd or signalk is enabled. */
    static pos_ctx_t pos_ctx;   /* static: lifetime matches daemon */
    pos_ctx.cfg  = &cfg;
    pos_ctx.imu  = imu;
    pos_ctx.stop = 0;
    snprintf(pos_ctx.wmm_file, sizeof pos_ctx.wmm_file, "%s", cfg.pos_wmm_file);
    if (cfg.pos_gpsd_enabled || cfg.pos_signalk_enabled) {
        prc = pthread_create(&pos_tid, NULL, position_thread, &pos_ctx);
        if (prc != 0) {
            LOG_W("[main] warning: cannot create position thread: %s\n",
                    strerror(prc));
        } else {
            pos_started = true;
        }
    }

    LOG_I("[main] imud %s running (pid %d)\n",
            VERSION_STR, (int)getpid());

    /* ── 11. Signal loop ─────────────────────────────────────────────────── */

    int sig;
    for (;;) {
        sigwait(&sigset, &sig);

        if (sig == SIGTERM || sig == SIGINT) {
            LOG_I("[main] caught signal %d — shutting down\n", sig);
            break;
        }

        if (sig == SIGHUP) {
            LOG_I("[main] SIGHUP — reloading config\n");
            imud_config_t new_cfg = cfg;
            if (config_load(args.config_path, &new_cfg) == 0) {
                apply_wmm_if_configured(&new_cfg);
                /* Publish the hot fields to the shared cfg under the lock (the
                 * health thread reads cfg concurrently).  The field list is
                 * config_apply_hot(), beside the struct it partitions, so a
                 * new [hot] key is one edit next to its own declaration and
                 * test_config's partition test fails if it is forgotten. */
                pthread_mutex_lock(&g_cfg_lock);
                config_apply_hot(&cfg, &new_cfg);
                pthread_mutex_unlock(&g_cfg_lock);

                /* Effects that go with the copy, not part of it.  Log level is
                 * hot; the log file is reopened so logrotate can move the old
                 * one (postrotate: systemctl reload imud). */
                log_set_level_str(new_cfg.log_level);
                if (cfg.log_file[0]) {
                    int lfd = open(cfg.log_file,
                                   O_WRONLY | O_CREAT | O_APPEND, 0640);
                    if (lfd >= 0) {
                        dup2(lfd, STDERR_FILENO);
                        close(lfd);
                    }
                }
                /* Push to the threads that hold private copies. */
                imu_ctx_update_config(imu, &cfg);
                out_ctx_reload(out, &cfg);
                LOG_I("[main] config reloaded\n");
            } else {
                LOG_W("[main] config reload failed — "
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
    if (stream_started) join_thread(stream_tid, "stream_out");

    /* Stop sensor and fusion threads. */
    imu_ctx_stop(imu);
    join_thread(health_tid, "health");
    join_thread(fusion_tid, "fusion");
    join_thread(mag_tid,    "mag_reader");
    join_thread(ism_tid,    "ism_reader");
    if (capture_started) join_thread(capture_tid, "capture");

    /* ── 13. Cleanup ─────────────────────────────────────────────────────── */

    out_ctx_free(out);
    imu_ctx_free(imu);

    if (status_fd >= 0) {
        close(status_fd);
        unlink(STATUS_SOCK);
    }
    pid_remove(PID_FILE);

    LOG_I("[main] exit\n");
    return 0;
}
