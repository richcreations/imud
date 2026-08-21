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

#include "cli.h"
#include "cloexec.h"
#include "config.h"
#include "imutest.h"
#include "version.h"

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
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;
    APPLY_CLOEXEC(fd);

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
    cli_imutest_t args;
    int cli_rc = cli_parse_imutest(argc, argv, &args);
    if (cli_rc != 0) return cli_rc < 0 ? 1 : 0;   /* -1 bad usage, 1 --version/--help */

    /* Names the rest of main() already used; args owns the storage.  Only
     * report_path and phases are written below, so those two stay writable. */
    const char *config_path     = args.config_path;
    char       *report_path     = args.report_path;
    const bool  have_config_arg = args.have_config_arg;
    const bool  force = args.force, quiet = args.quiet;
    unsigned    phases = args.phases;

    const char *ov_imu = args.ov_imu, *ov_mag  = args.ov_mag;
    const char *ov_bus = args.ov_bus, *ov_chip = args.ov_chip;
    const int ov_imu_addr = args.ov_imu_addr, ov_mag_addr = args.ov_mag_addr;
    const int ov_int_gpio = args.ov_int_gpio;
    const int ov_odr = args.ov_odr, ov_accel = args.ov_accel;
    const int ov_gyro = args.ov_gyro, ov_wm = args.ov_wm;
    const double ov_odr_win = args.ov_odr_win, ov_noise_win = args.ov_noise_win;
    const double ov_drdy_win = args.ov_drdy_win, ov_turn = args.ov_turn;
    const double ov_grav_tol = args.ov_grav_tol, ov_odr_tol = args.ov_odr_tol;
    const bool no_fs = args.no_fs, no_ovf = args.no_ovf;
    const bool no_regdiff = args.no_regdiff;

    bool interactive = isatty(0) && !args.non_interactive;
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
    if (ov_odr   > 0) cfg.imu_odr_mhz  = ov_odr;
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
        /* sizeof args.report_path, NOT sizeof report_path: report_path is a
         * char* alias for that char[512] (see the declaration above), so the
         * latter is sizeof(char*) = 8 and the name truncates to "imud-im".
         * gcc catches it twice (-Wsizeof-pointer-memaccess, -Wformat-truncation);
         * clang is silent, which is why it survived on the macOS dev box. */
        snprintf(report_path, sizeof args.report_path, "imud-imutest-%s-%s.md",
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
