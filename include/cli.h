/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cli.h — command-line parsing for the five non-bridge entry points
 *
 * imud, imud-cal, imud-mon, imud-status and imud-imutest each used to parse
 * argv inside their own main(), where no test binary could reach it — while
 * SECURITY.md claimed the CLI parsers were unit-tested.  They live here now,
 * as pure functions over (argc, argv) with no I/O beyond the usage/error text
 * they already printed, so test_cli can drive every branch.  The five main()s
 * keep everything that follows the parse.
 *
 * The bridges are not here: they share bridge_parse_cli() in src/bridge.c,
 * which was already testable and already tested (test_bridge).  This module
 * mirrors its return convention exactly:
 *
 *     0   carry on
 *     1   handled and printed (--version / --help); caller returns 0
 *    -1   bad usage, message printed; caller returns 1
 *
 * Output goes to stderr/stdout directly rather than through LOG_*, matching
 * what each tool already did.  For imud that is byte-identical, not a change:
 * log.c's default LOG_STYLE_PLAIN emits a bare fputs() to stderr, and argv is
 * parsed before anything calls log_set_style().  Staying off log.c also keeps
 * imud-status a dependency-free socket client.
 */

#ifndef IMUD_CLI_H
#define IMUD_CLI_H

#include <stdbool.h>

/* ── imud ────────────────────────────────────────────────────────────────── */

typedef struct {
    char config_path[256];
    /* True only when --config was actually given.  config_path is pre-filled
     * with the system default either way, so the flag is the only way to tell
     * "the operator named this file" from "nobody said" — and imud treats the
     * two differently: an explicit --config disables the $HOME fallback, so a
     * mistyped path cannot silently start the daemon on someone else's config
     * (audit N5). */
    bool config_explicit;
    char replay_path[256];
    int  skip_bias_cal;
    int  no_nmea;
    int  no_hirate;
} cli_imud_t;

int cli_parse_imud(int argc, char **argv, cli_imud_t *a);

/* ── imud-cal ────────────────────────────────────────────────────────────── */

typedef struct {
    char        config_path[256];
    const char *output_path;   /* NULL unless --output; points into argv */
    const char *from_path;     /* NULL unless --from */
    const char *mode;          /* never NULL on a 0 return */
    bool        offline;       /* characterize / fit-temp / fit-ra */
} cli_cal_t;

/*
 * Also validates: the positional mode must be one of the six known ones, and
 * the three offline modes must carry --from.  Both were checks in cal_main's
 * main() after the loop; they are argument validation, so they belong here
 * where they can be asserted on.
 */
int cli_parse_cal(int argc, char **argv, cli_cal_t *a);

/* ── imud-mon ────────────────────────────────────────────────────────────── */

typedef struct {
    char config_path[256];
    bool want_nmea;
    bool want_binary;
} cli_mon_t;

/* With no stream argument both streams are selected, as documented. */
int cli_parse_mon(int argc, char **argv, cli_mon_t *a);

/* ── imud-status ─────────────────────────────────────────────────────────── */

typedef struct {
    const char *sockpath;      /* default DEFAULT_STATUS_SOCK; points into argv */
} cli_status_t;

#define CLI_DEFAULT_STATUS_SOCK "/run/imud/imud.sock"

int cli_parse_status(int argc, char **argv, cli_status_t *a);

/* ── imud-imutest ────────────────────────────────────────────────────────── */

/*
 * The device overrides are applied to imud_config_t after the config file is
 * loaded, so each carries a sentinel meaning "not given": NULL for strings,
 * negative for numbers.  imutest_main keeps that application — this struct is
 * just the parse result.
 */
typedef struct {
    char config_path[256];
    char report_path[512];
    bool have_config_arg;      /* --config given: makes a load failure fatal */
    bool force, quiet, non_interactive;
    unsigned phases;           /* IMT_PHASE_*; never 0 on a 0 return */

    const char *ov_imu, *ov_mag, *ov_bus, *ov_chip;
    int    ov_imu_addr, ov_mag_addr, ov_int_gpio;
    int    ov_odr, ov_accel, ov_gyro, ov_wm;
    double ov_odr_win, ov_noise_win, ov_drdy_win;
    double ov_turn, ov_grav_tol, ov_odr_tol;
    bool   no_fs, no_ovf, no_regdiff;
} cli_imutest_t;

/*
 * phases defaults to IMT_PHASE_ALL when no phase flag is given.  The
 * isatty()/--non-interactive downgrade to passive-only stays in main(): it
 * depends on the environment, not on argv.
 */
int cli_parse_imutest(int argc, char **argv, cli_imutest_t *a);

#endif /* IMUD_CLI_H */
