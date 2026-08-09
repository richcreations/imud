/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cli.c — argv parsing for imud, imud-cal, imud-mon, imud-status, imud-imutest
 *
 * See include/cli.h for why these live outside their main()s and for the
 * 0 / 1 / -1 return convention.
 *
 * STREAM CONTRACT.  Help is REQUESTED output: --help writes to stdout and
 * exits 0, because the operator asked for it and may well be piping it into
 * less or grep.  A bad option is a DIAGNOSTIC: the error line and the usage
 * recap both go to stderr and the tool exits 1, so a script's stdout is not
 * polluted by a failure.  Hence the FILE * every usage_*() takes — the same
 * text, on whichever stream the caller's situation calls for.
 *
 * This is also what help2man requires: it runs `prog --help`, reads STDOUT,
 * and expects exit 0.  The man pages are generated from that output, so a
 * usage string on the wrong stream produces an empty OPTIONS section.
 *
 * The comment this replaces claimed test_cli pinned the streams.  It did not:
 * its capture dup2'd one fd onto both 1 and 2, so the split was invisible to
 * every assertion.  test_cli now captures the two separately and asserts that
 * --help writes nothing to stderr and an unknown option writes nothing to
 * stdout.
 */

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cli.h"
#include "imutest.h"   /* IMT_PHASE_* */
#include "version.h"

/* ── Checked numeric conversion for argv ─────────────────────────────────── */

/*
 * These replace atoi/atof, which carry no error signal at all — atoi("abc") is
 * 0, and either function on a value that cannot be represented is undefined by
 * the letter (C17 7.22.1.2p3).  The practical failures were worse than the UB,
 * and split three ways in imutest_main's override guards:
 *
 *   - --odr, --accel-g, --gyro-dps, --fifo-wm and the six window/tolerance
 *     flags are applied only `if (ov > 0)`, so garbage became 0 and the
 *     override was SILENTLY DISCARDED. The tool ran on the config value and
 *     told the operator nothing.
 *   - --imu-addr and --mag-addr are applied `if (ov >= 0)`, so garbage became
 *     0 and TOOK EFFECT: imud-imutest probed I2C address 0 and reported a
 *     device fault, for a typo.
 *   - --int-gpio is the one that matters most. Also `>= 0`, and 0 is
 *     documented in the help as "skips the interrupt check" — so a mistyped
 *     argument silently skipped the very check the operator asked for, and the
 *     report still said PASS.
 *
 * A bad numeric value is now a usage error, like any other bad argument.
 */
static bool cli_int(const char *s, int *out)
{
    char *end;
    errno = 0;
    long v = strtol(s, &end, 0);          /* base 0: accepts 0x hex */
    if (end == s || *end != '\0') return false;
    if (errno == ERANGE) return false;
#if LONG_MAX > INT_MAX
    if (v < INT_MIN || v > INT_MAX) return false;   /* see config.c parse_int */
#endif
    *out = (int)v;
    return true;
}

/*
 * Positive, finite and not absurd.
 *
 * The upper bound is what lets this reject inf and NaN WITHOUT <math.h>: every
 * comparison against NaN is false, and infinity fails the < test. That matters
 * because cli.c is deliberately libm-free — imud-status links src/cli.o with
 * no -lm, which is what keeps it a two-object link.
 *
 * Requiring > 0 rather than merely finite matches what imutest_main does with
 * these six anyway: it applies each only `if (ov > 0)`. The difference is that
 * a non-positive value is now reported instead of quietly ignored.
 */
static bool cli_pos_dbl(const char *s, double *out)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0') return false;
    if (!(v > 0.0 && v < 1e9)) return false;
    *out = v;
    return true;
}

static int cli_bad_num(const char *prog, const char *flag, const char *val,
                       void (*usage)(FILE *, const char *))
{
    fprintf(stderr, "invalid value for %s: %s\n", flag, val);
    usage(stderr, prog);
    return -1;
}

/* ── imud ────────────────────────────────────────────────────────────────── */

static void usage_imud(FILE *o, const char *prog)
{
    /* CLI help is user-requested output, never level-gated or prefixed. */
    fprintf(o, "Usage: %s [OPTIONS]\n"
        "\n"
        "Options:\n"
        "  --config PATH      Config file (default: /etc/imud/imud.conf)\n"
        "  --skip-bias-cal    Skip startup gyro bias estimation\n"
        "  --replay FILE      Replay an .imucap capture through the sim driver\n"
        "  --no-nmea          Disable NMEA output stream\n"
        "  --no-highrate      Disable high-rate binary stream\n"
        "  --foreground       Compatibility no-op (imud always runs in foreground)\n"
        "  --version          Print version and exit\n"
        "  -h, --help         Print this help and exit\n",
        prog);
}

int cli_parse_imud(int argc, char **argv, cli_imud_t *a)
{
    memset(a, 0, sizeof(*a));
    snprintf(a->config_path, sizeof(a->config_path), "/etc/imud/imud.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(a->config_path, sizeof(a->config_path), "%s", argv[++i]);
            a->config_explicit = true;
        } else if (strcmp(argv[i], "--replay") == 0 && i + 1 < argc) {
            snprintf(a->replay_path, sizeof(a->replay_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--skip-bias-cal") == 0) {
            a->skip_bias_cal = 1;
        } else if (strcmp(argv[i], "--no-nmea") == 0) {
            a->no_nmea = 1;
        } else if (strcmp(argv[i], "--no-highrate") == 0) {
            a->no_hirate = 1;
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud %s\n", IMUD_VERSION_STR);
            return 1;
        } else if (strcmp(argv[i], "--foreground") == 0) {
            /* always foreground under systemd — accepted, ignored */
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage_imud(stdout, argv[0]);
            return 1;
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            usage_imud(stderr, argv[0]);
            return -1;
        }
    }
    return 0;
}

/* ── imud-cal ────────────────────────────────────────────────────────────── */

static void usage_cal(FILE *o, const char *prog)
{
    fprintf(o,
        "Usage: %s [--config PATH] [--output PATH] <mode>\n"
        "\n"
        "Modes:\n"
        "  mag          Magnetometer calibration (vessel swing, daemon stopped)\n"
        "  gyro         Gyroscope bias capture   (hold still, daemon stopped)\n"
        "  accel        Accelerometer 6-position (bench, daemon stopped)\n"
        "  characterize Allan-variance noise analysis of a stationary capture\n"
        "               (requires --from FILE; record with [capture] enabled)\n"
        "  fit-temp     Gyro bias/temperature fit from a warm-up capture\n"
        "               (requires --from FILE)\n"
        "  fit-ra       Check the MEKF accel measurement model against a\n"
        "               rough-water capture (requires --from FILE).\n"
        "               Reports only; writes nothing.\n"
        "\n"
        "Options:\n"
        "  --config PATH   Config file (default: /etc/imud/imud.conf)\n"
        "  --output PATH   Override cal.json output path from config\n"
        "  --from FILE     .imucap capture to analyze (offline modes)\n"
        "  --version       Print version and exit\n"
        "  -h, --help      Print this help and exit\n",
        prog);
}

int cli_parse_cal(int argc, char **argv, cli_cal_t *a)
{
    memset(a, 0, sizeof(*a));
    snprintf(a->config_path, sizeof(a->config_path), "/etc/imud/imud.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(a->config_path, sizeof(a->config_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            a->output_path = argv[++i];
        } else if (strcmp(argv[i], "--from") == 0 && i + 1 < argc) {
            a->from_path = argv[++i];
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud-cal %s\n", IMUD_VERSION_STR);
            return 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage_cal(stdout, argv[0]);
            return 1;
        /* Positional mode. Placed after the flags so --help and --version are
         * never swallowed as one: the guard below only accepts a non-'-'
         * token, but keeping the order explicit stops a future flag from
         * being captured as a mode name. */
        } else if (!a->mode && argv[i][0] != '-') {
            a->mode = argv[i];
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            usage_cal(stderr, argv[0]); return -1;
        }
    }

    if (!a->mode) { usage_cal(stderr, argv[0]); return -1; }

    if (strcmp(a->mode, "mag")          != 0 &&
        strcmp(a->mode, "gyro")         != 0 &&
        strcmp(a->mode, "accel")        != 0 &&
        strcmp(a->mode, "characterize") != 0 &&
        strcmp(a->mode, "fit-temp")     != 0 &&
        strcmp(a->mode, "fit-ra")       != 0) {
        fprintf(stderr, "unknown mode '%s'\n", a->mode);
        usage_cal(stderr, argv[0]); return -1;
    }

    /* The offline analysis modes read an .imucap instead of the sensors, so
     * they need --from, and they only need the config for the cal.json path
     * (which is why cal_main lets a MISSING config fall back to defaults for
     * them, and stays strict for the sensor modes). */
    a->offline = strcmp(a->mode, "characterize") == 0 ||
                 strcmp(a->mode, "fit-temp")     == 0 ||
                 strcmp(a->mode, "fit-ra")       == 0;

    if (a->offline && !a->from_path) {
        fprintf(stderr, "%s requires --from FILE (an .imucap capture)\n",
                a->mode);
        usage_cal(stderr, argv[0]); return -1;
    }
    return 0;
}

/* ── imud-mon ────────────────────────────────────────────────────────────── */

static void usage_mon(FILE *o, const char *prog)
{
    fprintf(o,
        "Usage: %s [--config PATH] [nmea] [binary]\n"
        "\n"
        "Options:\n"
        "  --config PATH  Config file (default: /etc/imud/imud.conf)\n"
        "  --version      Print version and exit\n"
        "  -h, --help     Print this help and exit\n"
        "\n"
        "  nmea    Monitor NMEA 0183 stream (UDP port 10110)\n"
        "  binary  Monitor binary stream    (UDP port 10111)\n"
        "\n"
        "  With no stream arguments both streams are shown.\n"
        "  Port numbers and multicast addresses are read from the config file.\n",
        prog);
}

int cli_parse_mon(int argc, char **argv, cli_mon_t *a)
{
    memset(a, 0, sizeof(*a));
    snprintf(a->config_path, sizeof a->config_path, "/etc/imud/imud.conf");

    bool any_stream = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(a->config_path, sizeof a->config_path, "%s", argv[++i]);
        } else if (strcmp(argv[i], "nmea") == 0) {
            a->want_nmea   = true; any_stream = true;
        } else if (strcmp(argv[i], "binary") == 0) {
            a->want_binary = true; any_stream = true;
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud-mon %s\n", IMUD_VERSION_STR);
            return 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage_mon(stdout, argv[0]); return 1;
        } else {
            fprintf(stderr, "unknown argument: %s\n", argv[i]);
            usage_mon(stderr, argv[0]); return -1;
        }
    }

    if (!any_stream)
        a->want_nmea = a->want_binary = true;

    return 0;
}

/* ── imud-status ─────────────────────────────────────────────────────────── */

static void usage_status(FILE *o, const char *p)
{
    fprintf(o,
        "Usage: %s [--socket PATH]\n"
        "\n"
        "Options:\n"
        /* Concatenated from the constant the parser defaults to, so the help
         * text cannot drift from the actual default. */
        "  --socket PATH  Status socket (default: " CLI_DEFAULT_STATUS_SOCK ")\n"
        "  --version      Print version and exit\n"
        "  -h, --help     Print this help and exit\n", p);
}

int cli_parse_status(int argc, char **argv, cli_status_t *a)
{
    memset(a, 0, sizeof(*a));
    a->sockpath = CLI_DEFAULT_STATUS_SOCK;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--socket") == 0 && i + 1 < argc) {
            a->sockpath = argv[++i];
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud-status %s\n", IMUD_VERSION_STR);
            return 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage_status(stdout, argv[0]);
            return 1;
        } else {
            usage_status(stderr, argv[0]);
            return -1;
        }
    }
    return 0;
}

/* ── imud-imutest ────────────────────────────────────────────────────────── */

static void usage_imutest(FILE *o, const char *prog)
{
    fprintf(o,
"Usage: %s [OPTIONS]\n"
"\n"
"Exercises a registered IMU/magnetometer driver against real silicon and\n"
"writes a Markdown report suitable for pasting into a GitHub issue.\n"
"\n"
"Stop the daemon first: imud and imud-imutest both open the same I2C device,\n"
"and both would drain the same FIFO.\n"
"\n"
"Options:\n"
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

int cli_parse_imutest(int argc, char **argv, cli_imutest_t *a)
{
    memset(a, 0, sizeof(*a));
    snprintf(a->config_path, sizeof a->config_path, "/etc/imud/imud.conf");

    /* Sentinels: "not given" for every override, so imutest_main can tell an
     * explicit value from an absent flag. */
    a->ov_imu_addr = a->ov_mag_addr = a->ov_int_gpio = -1;
    a->ov_odr = a->ov_accel = a->ov_gyro = a->ov_wm = -1;
    a->ov_odr_win = a->ov_noise_win = a->ov_drdy_win = -1;
    a->ov_turn = a->ov_grav_tol = a->ov_odr_tol = -1;

    for (int i = 1; i < argc; i++) {
        const char *s = argv[i];
        if      (strcmp(s, "--config") == 0 && i + 1 < argc) {
            snprintf(a->config_path, sizeof a->config_path, "%s", argv[++i]);
            a->have_config_arg = true;
        }
        else if (strcmp(s, "--report") == 0 && i + 1 < argc)
            snprintf(a->report_path, sizeof a->report_path, "%s", argv[++i]);
        else if (strcmp(s, "--imu-driver") == 0 && i + 1 < argc) a->ov_imu = argv[++i];
        else if (strcmp(s, "--mag-driver") == 0 && i + 1 < argc) a->ov_mag = argv[++i];
        else if (strcmp(s, "--i2c-bus") == 0 && i + 1 < argc)    a->ov_bus = argv[++i];
        else if (strcmp(s, "--gpio-chip") == 0 && i + 1 < argc)  a->ov_chip = argv[++i];
        else if (strcmp(s, "--imu-addr") == 0 && i + 1 < argc) {
            if (!cli_int(argv[++i], &a->ov_imu_addr))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--mag-addr") == 0 && i + 1 < argc) {
            if (!cli_int(argv[++i], &a->ov_mag_addr))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--int-gpio") == 0 && i + 1 < argc) {
            if (!cli_int(argv[++i], &a->ov_int_gpio))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--odr") == 0 && i + 1 < argc) {
            if (!cli_int(argv[++i], &a->ov_odr))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--accel-g") == 0 && i + 1 < argc) {
            if (!cli_int(argv[++i], &a->ov_accel))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--gyro-dps") == 0 && i + 1 < argc) {
            if (!cli_int(argv[++i], &a->ov_gyro))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--fifo-wm") == 0 && i + 1 < argc) {
            if (!cli_int(argv[++i], &a->ov_wm))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--passive") == 0) a->phases |= IMT_PHASE_PASSIVE;
        else if (strcmp(s, "--faces") == 0)   a->phases |= IMT_PHASE_FACES;
        else if (strcmp(s, "--gyro") == 0)    a->phases |= IMT_PHASE_GYRO;
        else if (strcmp(s, "--spin") == 0)    a->phases |= IMT_PHASE_SPIN;
        else if (strcmp(s, "--all") == 0)     a->phases  = IMT_PHASE_ALL;
        else if (strcmp(s, "--non-interactive") == 0) a->non_interactive = true;
        else if (strcmp(s, "--odr-window") == 0 && i + 1 < argc) {
            if (!cli_pos_dbl(argv[++i], &a->ov_odr_win))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--noise-window") == 0 && i + 1 < argc) {
            if (!cli_pos_dbl(argv[++i], &a->ov_noise_win))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--drdy-window") == 0 && i + 1 < argc) {
            if (!cli_pos_dbl(argv[++i], &a->ov_drdy_win))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--turn-deg") == 0 && i + 1 < argc) {
            if (!cli_pos_dbl(argv[++i], &a->ov_turn))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--grav-tol") == 0 && i + 1 < argc) {
            if (!cli_pos_dbl(argv[++i], &a->ov_grav_tol))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
        }
        else if (strcmp(s, "--odr-tol") == 0 && i + 1 < argc) {
            double pct;
            if (!cli_pos_dbl(argv[++i], &pct))
                return cli_bad_num(argv[0], s, argv[i], usage_imutest);
            a->ov_odr_tol = pct / 100.0;   /* the flag is a percentage */
        }
        else if (strcmp(s, "--no-fs-sweep") == 0) a->no_fs = true;
        else if (strcmp(s, "--no-overflow") == 0) a->no_ovf = true;
        else if (strcmp(s, "--no-regdiff") == 0)  a->no_regdiff = true;
        else if (strcmp(s, "--force") == 0) a->force = true;
        else if (strcmp(s, "--quiet") == 0) a->quiet = true;
        else if (strcmp(s, "--version") == 0) {
            printf("imud-imutest %s\n", IMUD_VERSION_STR);
            return 1;
        }
        else if (strcmp(s, "-h") == 0 || strcmp(s, "--help") == 0) {
            usage_imutest(stdout, argv[0]);
            return 1;
        }
        else {
            fprintf(stderr, "unknown option: %s\n", s);
            usage_imutest(stderr, argv[0]);
            return -1;
        }
    }

    if (a->phases == 0) a->phases = IMT_PHASE_ALL;

    return 0;
}
