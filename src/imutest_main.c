/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imutest_main.c — CLI for imud-imutest.
 *
 * Everything decided here is presentation: argument parsing, the terminal
 * implementation of imt_ui_t, and where the report file lands.  Every check,
 * threshold and diagnosis lives in src/imutest.c.
 *
 * fprintf(stderr)/printf rather than LOG_*: this is an interactive tool, like
 * imud-cal, imud-mon and imud-status.
 */

#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/un.h>

#include "config.h"
#include "imutest.h"
#include "version.h"

static void usage(const char *prog)
{
    printf(
"Usage: %s [OPTIONS]\n"
"\n"
"Exercises a registered IMU/magnetometer driver against real silicon and\n"
"writes a Markdown report suitable for pasting into a GitHub issue.\n"
"\n"
"Stop the daemon first: imud and imud-imutest both open the same I2C device,\n"
"and both would drain the same FIFO.\n"
"\n"
"  --config PATH        Config file (default: /etc/imud/imud.conf)\n"
"  --report PATH        Report output\n"
"                       (default: ./imud-imutest-<imu>-<YYYYmmdd-HHMMSS>.md)\n"
"\n"
"Device overrides (default: whatever imud.conf says)\n"
"  --imu-driver NAME    [imu] driver\n"
"  --mag-driver NAME    [mag] driver; \"none\" for an IMU-only board\n"
"  --imu-addr HEX       [imu] i2c_addr, e.g. 0x6A\n"
"  --mag-addr HEX       [mag] i2c_addr, e.g. 0x0C\n"
"  --i2c-bus PATH       [device] i2c_bus\n"
"  --gpio-chip NAME     [device] gpio_chip\n"
"  --int-gpio N         [imu] int_gpio; 0 skips the interrupt check\n"
"  --odr HZ             [imu] odr_hz\n"
"  --accel-g N          [imu] accel_g\n"
"  --gyro-dps N         [imu] gyro_dps\n"
"  --fifo-wm N          [imu] fifo_wm\n"
"\n"
"Phases (default: all four)\n"
"  --passive            Automatic checks only; no operator action\n"
"  --faces              Guided six-face accelerometer / axis-sign test\n"
"  --gyro               Guided gyro rotation test\n"
"  --spin               Guided magnetometer spin test\n"
"  --all                All four (the default)\n"
"  --non-interactive    Passive only, never prompt. Implied when stdin is\n"
"                       not a terminal.\n"
"\n"
"Tuning\n"
"  --odr-window S       ODR / seq / chip_ts window, s   (default 5, min 3)\n"
"  --noise-window S     Noise and temperature window, s (default 10)\n"
"  --drdy-window S      Interrupt edge-count window, s  (default 3)\n"
"  --turn-deg N         Commanded angle for --gyro      (default 90)\n"
"  --grav-tol M         Gravity warn tolerance, m/s^2   (default 0.25)\n"
"  --odr-tol PCT        ODR warn tolerance, percent     (default 5)\n"
"  --no-fs-sweep        Skip the full-scale re-init sweep\n"
"  --no-overflow        Do not deliberately overflow the FIFO\n"
"  --no-regdiff         Do not read control registers back\n"
"\n"
"  --force              Run even if imud appears to be running\n"
"  --quiet              Digest only; no live progress lines\n"
"  --version            Print version and exit\n"
"  -h, --help           This message\n"
"\n"
"Exit: 0 all pass  1 usage/config/I-O error  2 a check FAILed\n"
"      3 warnings only  130 aborted\n",
    prog);
}

/* ── Terminal implementation of imt_ui_t ──────────────────────────────────── */

typedef struct {
    bool quiet;
    bool interactive;
    int  last_len;
} term_ui_t;

static volatile sig_atomic_t g_sigint;

static void on_sigint(int sig) { (void)sig; g_sigint = 1; imt_request_abort(); }

/* Clear whatever the progress line left behind. */
static void clear_line(term_ui_t *t)
{
    if (t->last_len > 0) { printf("\r\033[K"); fflush(stdout); t->last_len = 0; }
}

static int term_prompt(void *user, const char *id, const char *title,
                       const char *body)
{
    term_ui_t *t = user;
    (void)id;
    clear_line(t);

    if (!t->interactive) return 1;

    printf("\n  %s\n", title);
    printf("  %s\n", body);
    printf("  Press Enter when ready, 's' then Enter to skip, Ctrl-C to abort: ");
    fflush(stdout);

    int c = getchar();
    if (c == EOF || g_sigint) return -1;
    if (c == 's' || c == 'S') {
        while ((c = getchar()) != '\n' && c != EOF) { }
        printf("  skipped.\n");
        return 1;
    }
    while (c != '\n' && c != EOF) c = getchar();
    return g_sigint ? -1 : 0;
}

/*
 * "Done" is a line on stdin.  Non-blocking, so the collection loop keeps
 * integrating while the operator is still turning the board.
 */
static int term_poll_done(void *user)
{
    term_ui_t *t = user;
    if (g_sigint) return -1;
    if (!t->interactive) return 1;

    struct pollfd p = { .fd = 0, .events = POLLIN };
    if (poll(&p, 1, 0) > 0 && (p.revents & POLLIN)) {
        int c;
        while ((c = getchar()) != '\n' && c != EOF) { }
        return 1;
    }
    return 0;
}

static void term_progress(void *user, const char *id, double frac,
                          const char *detail)
{
    term_ui_t *t = user;
    if (t->quiet) return;

    char line[160];
    int n;
    if (frac >= 0.0)
        n = snprintf(line, sizeof line, "  %-22s %3.0f%%%s%s",
                     id, frac * 100.0, detail ? "  " : "", detail ? detail : "");
    else
        n = snprintf(line, sizeof line, "  %-22s %s", id,
                     detail ? detail : "collecting... (Enter when done)");

    printf("\r%s\033[K", line);
    fflush(stdout);
    t->last_len = n;
}

static void term_coverage(void *user, const int *sectors, int nsec, int cur,
                          int n_samples, double radius_ut)
{
    term_ui_t *t = user;
    if (t->quiet) return;

    /* Same bar as imud-cal's swing display: '#' covered, '.' not yet,
     * 'o' where the board is pointing right now. */
    printf("\r  spin  %5d samples  [", n_samples);
    int filled = 0;
    for (int i = 0; i < nsec; i++) {
        if (sectors[i]) filled++;
        putchar(i == cur ? 'o' : (sectors[i] ? '#' : '.'));
    }
    printf("] %2d/%d  |B|=%.1f uT", filled, nsec, radius_ut);
    if (filled == nsec) printf("  FULL CIRCLE — Enter to finish");
    printf("\033[K");
    fflush(stdout);
    t->last_len = 60;
}

/* ── Daemon-conflict probe ────────────────────────────────────────────────── */

/*
 * Opening /dev/i2c-N succeeds even when another process holds it, so the
 * reliable signal is the daemon's own status socket — the same probe
 * imud-status performs.  This matters more than it looks: both processes would
 * drain the same FIFO, so each sees about half the samples, ODR reads low and
 * seq.gapless fails. That is a false negative that would poison a review.
 */
static bool daemon_running(const imud_config_t *cfg)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    struct sockaddr_un addr;
    size_t plen = strlen(cfg->stream_socket);
    if (plen == 0 || plen >= sizeof addr.sun_path) { close(fd); return false; }

    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, cfg->stream_socket, plen);

    bool up = connect(fd, (struct sockaddr *)&addr, sizeof addr) == 0;
    close(fd);
    return up;
}

/* ── main ─────────────────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    char config_path[256] = "/etc/imud/imud.conf";
    char report_path[512] = "";
    bool have_config_arg = false;
    bool force = false, quiet = false, non_interactive = false;
    unsigned phases = 0;

    /* Device overrides, applied after the config file is loaded. */
    const char *ov_imu = NULL, *ov_mag = NULL, *ov_bus = NULL, *ov_chip = NULL;
    int ov_imu_addr = -1, ov_mag_addr = -1, ov_int_gpio = -1;
    int ov_odr = -1, ov_accel = -1, ov_gyro = -1, ov_wm = -1;
    double ov_odr_win = -1, ov_noise_win = -1, ov_drdy_win = -1;
    double ov_turn = -1, ov_grav_tol = -1, ov_odr_tol = -1;
    bool no_fs = false, no_ovf = false, no_regdiff = false;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if      (strcmp(a, "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof config_path, "%s", argv[++i]);
            have_config_arg = true;
        }
        else if (strcmp(a, "--report") == 0 && i + 1 < argc)
            snprintf(report_path, sizeof report_path, "%s", argv[++i]);
        else if (strcmp(a, "--imu-driver") == 0 && i + 1 < argc) ov_imu = argv[++i];
        else if (strcmp(a, "--mag-driver") == 0 && i + 1 < argc) ov_mag = argv[++i];
        else if (strcmp(a, "--i2c-bus") == 0 && i + 1 < argc)    ov_bus = argv[++i];
        else if (strcmp(a, "--gpio-chip") == 0 && i + 1 < argc)  ov_chip = argv[++i];
        else if (strcmp(a, "--imu-addr") == 0 && i + 1 < argc)
            ov_imu_addr = (int)strtol(argv[++i], NULL, 0);
        else if (strcmp(a, "--mag-addr") == 0 && i + 1 < argc)
            ov_mag_addr = (int)strtol(argv[++i], NULL, 0);
        else if (strcmp(a, "--int-gpio") == 0 && i + 1 < argc)
            ov_int_gpio = atoi(argv[++i]);
        else if (strcmp(a, "--odr") == 0 && i + 1 < argc)       ov_odr = atoi(argv[++i]);
        else if (strcmp(a, "--accel-g") == 0 && i + 1 < argc)   ov_accel = atoi(argv[++i]);
        else if (strcmp(a, "--gyro-dps") == 0 && i + 1 < argc)  ov_gyro = atoi(argv[++i]);
        else if (strcmp(a, "--fifo-wm") == 0 && i + 1 < argc)   ov_wm = atoi(argv[++i]);
        else if (strcmp(a, "--passive") == 0) phases |= IMT_PHASE_PASSIVE;
        else if (strcmp(a, "--faces") == 0)   phases |= IMT_PHASE_FACES;
        else if (strcmp(a, "--gyro") == 0)    phases |= IMT_PHASE_GYRO;
        else if (strcmp(a, "--spin") == 0)    phases |= IMT_PHASE_SPIN;
        else if (strcmp(a, "--all") == 0)     phases  = IMT_PHASE_ALL;
        else if (strcmp(a, "--non-interactive") == 0) non_interactive = true;
        else if (strcmp(a, "--odr-window") == 0 && i + 1 < argc)
            ov_odr_win = atof(argv[++i]);
        else if (strcmp(a, "--noise-window") == 0 && i + 1 < argc)
            ov_noise_win = atof(argv[++i]);
        else if (strcmp(a, "--drdy-window") == 0 && i + 1 < argc)
            ov_drdy_win = atof(argv[++i]);
        else if (strcmp(a, "--turn-deg") == 0 && i + 1 < argc)
            ov_turn = atof(argv[++i]);
        else if (strcmp(a, "--grav-tol") == 0 && i + 1 < argc)
            ov_grav_tol = atof(argv[++i]);
        else if (strcmp(a, "--odr-tol") == 0 && i + 1 < argc)
            ov_odr_tol = atof(argv[++i]) / 100.0;
        else if (strcmp(a, "--no-fs-sweep") == 0) no_fs = true;
        else if (strcmp(a, "--no-overflow") == 0) no_ovf = true;
        else if (strcmp(a, "--no-regdiff") == 0)  no_regdiff = true;
        else if (strcmp(a, "--force") == 0) force = true;
        else if (strcmp(a, "--quiet") == 0) quiet = true;
        else if (strcmp(a, "--version") == 0) {
            printf("imud-imutest %s\n", IMUD_VERSION_STR);
            return 0;
        }
        else if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
        else {
            fprintf(stderr, "unknown option: %s\n", a);
            usage(argv[0]);
            return 1;
        }
    }

    if (phases == 0) phases = IMT_PHASE_ALL;

    bool interactive = isatty(0) && !non_interactive;
    if (!interactive) phases = IMT_PHASE_PASSIVE;

    /* ── Config ──────────────────────────────────────────────────────────── */
    imud_config_t cfg;
    config_defaults(&cfg);
    if (config_load(config_path, &cfg) < 0 && have_config_arg) {
        fprintf(stderr, "imud-imutest: cannot read %s: %s\n",
                config_path, strerror(errno));
        return 1;
    }

    if (ov_imu)  snprintf(cfg.imu_driver, sizeof cfg.imu_driver, "%s", ov_imu);
    if (ov_mag)  snprintf(cfg.mag_driver, sizeof cfg.mag_driver, "%s", ov_mag);
    if (ov_bus)  snprintf(cfg.i2c_bus,    sizeof cfg.i2c_bus,    "%s", ov_bus);
    if (ov_chip) snprintf(cfg.gpio_chip,  sizeof cfg.gpio_chip,  "%s", ov_chip);
    if (ov_imu_addr >= 0) cfg.imu_addr     = ov_imu_addr;
    if (ov_mag_addr >= 0) cfg.mag_addr     = ov_mag_addr;
    if (ov_int_gpio >= 0) cfg.imu_int_gpio = ov_int_gpio;
    if (ov_odr   > 0) cfg.imu_odr_hz   = ov_odr;
    if (ov_accel > 0) cfg.imu_accel_g  = ov_accel;
    if (ov_gyro  > 0) cfg.imu_gyro_dps = ov_gyro;
    if (ov_wm    > 0) cfg.imu_fifo_wm  = ov_wm;

    /* ── Daemon conflict ─────────────────────────────────────────────────── */
    bool daemon_up = daemon_running(&cfg);
    if (daemon_up && !force) {
        fprintf(stderr,
"imud-imutest: imud appears to be running (its socket at %s accepted a\n"
"connection). Both processes open the same I2C device and drain the same\n"
"FIFO, so each would see about half the samples: the measured ODR would read\n"
"low and the seq check would fail for the wrong reason.\n"
"\n"
"  sudo systemctl stop imud\n"
"\n"
"Pass --force to run anyway; the report will be marked accordingly.\n",
                cfg.stream_socket);
        return 1;
    }

    /* ── Options ─────────────────────────────────────────────────────────── */
    term_ui_t term = { .quiet = quiet, .interactive = interactive, .last_len = 0 };

    imt_opts_t opts;
    imt_opts_defaults(&opts);
    opts.phases = phases;
    if (ov_odr_win   > 0) opts.odr_window_s   = ov_odr_win;
    if (ov_noise_win > 0) opts.noise_window_s = ov_noise_win;
    if (ov_drdy_win  > 0) opts.drdy_window_s  = ov_drdy_win;
    if (ov_turn      > 0) opts.turn_deg       = ov_turn;
    if (ov_grav_tol  > 0) opts.grav_tol_warn  = ov_grav_tol;
    if (ov_odr_tol   > 0) opts.odr_tol_warn   = ov_odr_tol;
    if (no_fs)      opts.fs_sweep        = false;
    if (no_ovf)     opts.induce_overflow = false;
    if (no_regdiff) opts.regdiff         = false;

    /* The 20-bit counter on some parts wraps at about 1.05 s; the chip_ts
     * wall-clock check needs several wraps inside the window to mean anything. */
    if (opts.odr_window_s < 3.0) opts.odr_window_s = 3.0;

    opts.ui.prompt    = term_prompt;
    opts.ui.poll_done = term_poll_done;
    opts.ui.progress  = term_progress;
    opts.ui.coverage  = term_coverage;
    opts.ui.user      = &term;

    signal(SIGINT, on_sigint);

    /* ── Run ─────────────────────────────────────────────────────────────── */
    imt_report_t *rep = calloc(1, sizeof *rep);
    if (!rep) { fprintf(stderr, "imud-imutest: out of memory\n"); return 1; }

    snprintf(rep->config_path, sizeof rep->config_path, "%s", config_path);
    rep->daemon_was_running = daemon_up;

    /* Record the exact invocation so the report can be reproduced. */
    size_t off = 0;
    for (int i = 0; i < argc && off < sizeof rep->cmdline - 1; i++)
        off += (size_t)snprintf(rep->cmdline + off, sizeof rep->cmdline - off,
                                "%s%s", i ? " " : "", argv[i]);

    printf("imud-imutest %s — %s", IMUD_VERSION_STR, cfg.imu_driver);
    if (cfg.mag_driver[0] && strcmp(cfg.mag_driver, "none") != 0)
        printf(" + %s", cfg.mag_driver);
    printf(" on %s\n", cfg.i2c_bus);
    if (!interactive)
        printf("  (not a terminal, or --non-interactive: passive checks only)\n");

    char err[256] = "";
    if (imt_run(&cfg, &opts, rep, err, sizeof err) < 0) {
        fprintf(stderr, "imud-imutest: %s\n", err);
        free(rep);
        return 1;
    }

    clear_line(&term);
    imt_print(rep, stdout);

    /* ── Report file ─────────────────────────────────────────────────────── */
    if (report_path[0] == '\0') {
        time_t now = time(NULL);
        struct tm tm;
        localtime_r(&now, &tm);
        char stamp[32];
        strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", &tm);
        snprintf(report_path, sizeof report_path, "imud-imutest-%s-%s.md",
                 cfg.imu_driver, stamp);
    }

    if (imt_write_md(rep, report_path, err, sizeof err) < 0) {
        fprintf(stderr, "imud-imutest: %s: %s\n", err, strerror(errno));
        int rc = imt_exit_code(rep);
        free(rep);
        return rc ? rc : 1;
    }

    printf("\n  Report written to %s\n", report_path);
    if (rep->recommend_clear_experimental)
        printf("  Attach it to an issue at "
               "https://github.com/richcreations/imud/issues to have the\n"
               "  experimental flag cleared for this driver.\n");

    int rc = imt_exit_code(rep);
    free(rep);
    return rc;
}
