/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_cli.c — the CLI parsers for imud, imud-cal, imud-mon, imud-status and
 * imud-imutest (src/cli.c).  The bridges' shared parser is covered separately
 * by test_bridge.
 *
 * SECURITY.md declines to fuzz argv on the grounds that it crosses no trust
 * boundary, and points at unit tests instead.  This is that claim: every
 * option of every tool, and — for every flag that consumes the argument after
 * it — the case where there is no argument after it.  That "i + 1 < argc"
 * guard is the one thing in a hand-rolled argv loop that reads off the end if
 * it is ever dropped, so it gets a test per flag rather than one representative
 * case: cli.c has 26 such flags across the five tools.
 *
 * The parsers print usage and error text, so each call runs with stdout and
 * stderr captured to a temp file.  That keeps the run readable and, more
 * usefully, makes the text itself assertable — these tools' output is a
 * documented interface, and it was byte-identical through the extraction from
 * the five main()s.
 *
 * Portable — builds and runs on the macOS dev box.
 */

#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cli.h"
#include "imutest.h"   /* IMT_PHASE_* */
#include "version.h"

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── argv helpers ────────────────────────────────────────────────────────── */

static int argc_of(char **v) { int n = 0; while (v[n]) n++; return n; }

/* ── stdout/stderr capture ───────────────────────────────────────────────── */

static int  g_saved_out = -1, g_saved_err = -1;
static char g_cap_path[128];

static void cap_begin(void)
{
    snprintf(g_cap_path, sizeof g_cap_path, "/tmp/imud_tcli_%d.txt",
             (int)getpid());
    fflush(stdout); fflush(stderr);
    g_saved_out = dup(STDOUT_FILENO);
    g_saved_err = dup(STDERR_FILENO);
    int fd = open(g_cap_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd >= 0) {
        dup2(fd, STDOUT_FILENO);
        dup2(fd, STDERR_FILENO);
        close(fd);
    }
}

static const char *cap_end(void)
{
    static char buf[8192];
    fflush(stdout); fflush(stderr);
    if (g_saved_out >= 0) { dup2(g_saved_out, STDOUT_FILENO); close(g_saved_out); }
    if (g_saved_err >= 0) { dup2(g_saved_err, STDERR_FILENO); close(g_saved_err); }
    g_saved_out = g_saved_err = -1;

    buf[0] = '\0';
    FILE *f = fopen(g_cap_path, "r");
    if (f) {
        size_t n = fread(buf, 1, sizeof buf - 1, f);
        buf[n] = '\0';
        fclose(f);
    }
    unlink(g_cap_path);
    return buf;
}

static bool has(const char *hay, const char *needle)
{
    return strstr(hay, needle) != NULL;
}

/* ── two-stream capture ──────────────────────────────────────────────────────
 *
 * cap_begin() above dup2s ONE fd onto both stdout and stderr, so which stream
 * a message went to is invisible to it.  That is why cli.c could carry a
 * comment claiming "test_cli pins that" about the streams while nothing
 * pinned them at all, and why usage output sat on stderr for years.
 *
 * The stream is part of the contract now — help2man reads stdout and a script
 * redirecting stdout must not collect a usage dump on failure — so it needs
 * assertions, and assertions need the two kept apart.
 *
 * Note the streams are captured to regular files, which makes stdout fully
 * buffered where a terminal would line-buffer it.  Interleaving between the
 * two is therefore NOT preserved; assert on content per stream, never on the
 * order of one relative to the other.
 */
static int  g2_saved_out = -1, g2_saved_err = -1;
static char g2_out_path[128], g2_err_path[128];

static void cap2_begin(void)
{
    snprintf(g2_out_path, sizeof g2_out_path, "/tmp/imud_tcli_o_%d.txt",
             (int)getpid());
    snprintf(g2_err_path, sizeof g2_err_path, "/tmp/imud_tcli_e_%d.txt",
             (int)getpid());
    fflush(stdout); fflush(stderr);
    g2_saved_out = dup(STDOUT_FILENO);
    g2_saved_err = dup(STDERR_FILENO);
    int fo = open(g2_out_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    int fe = open(g2_err_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fo >= 0) { dup2(fo, STDOUT_FILENO); close(fo); }
    if (fe >= 0) { dup2(fe, STDERR_FILENO); close(fe); }
}

static void slurp(const char *path, char *buf, size_t bufsz)
{
    buf[0] = '\0';
    FILE *f = fopen(path, "r");
    if (f) {
        size_t n = fread(buf, 1, bufsz - 1, f);
        buf[n] = '\0';
        fclose(f);
    }
    unlink(path);
}

static void cap2_end(char *out, size_t outsz, char *err, size_t errsz)
{
    fflush(stdout); fflush(stderr);
    if (g2_saved_out >= 0) { dup2(g2_saved_out, STDOUT_FILENO); close(g2_saved_out); }
    if (g2_saved_err >= 0) { dup2(g2_saved_err, STDERR_FILENO); close(g2_saved_err); }
    g2_saved_out = g2_saved_err = -1;
    slurp(g2_out_path, out, outsz);
    slurp(g2_err_path, err, errsz);
}

/* ── imud ────────────────────────────────────────────────────────────────── */

static void test_imud(void)
{
    begin("test_imud");
    int fb = g_fail;
    cli_imud_t a;
    const char *out;
    int rc;

    {   char *v[] = { "imud", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == 0, "no args → run");
        EXPECT(strcmp(a.config_path, "/etc/imud/imud.conf") == 0, "default config");
        /* config_path is pre-filled either way, so this flag is the only thing
         * that distinguishes the default from an operator naming that exact
         * path — and imud gives only the latter a $HOME fallback. */
        EXPECT(!a.config_explicit, "the pre-filled default is not 'explicit'");
        EXPECT(a.replay_path[0] == '\0', "no replay by default");
        EXPECT(!a.skip_bias_cal && !a.no_nmea && !a.no_hirate, "no flags set");
        EXPECT(out[0] == '\0', "silent on a clean parse");
    }

    {   char *v[] = { "imud", "--config", "/tmp/x.conf", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.config_path, "/tmp/x.conf") == 0, "--config");
        EXPECT(a.config_explicit, "--config marks the path explicit");
    }

    /* The awkward one: naming the default path is still explicit, so it too
     * gets no fallback.  A strcmp against the default would get this wrong. */
    {   char *v[] = { "imud", "--config", "/etc/imud/imud.conf", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.config_explicit,
               "--config with the default path is explicit too");
    }

    {   char *v[] = { "imud", "--replay", "/tmp/r.imucap", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.replay_path, "/tmp/r.imucap") == 0, "--replay");
    }

    /* Every boolean at once — they are independent bits, and a copy-paste slip
     * between them would still pass a one-flag-at-a-time test. */
    {   char *v[] = { "imud", "--skip-bias-cal", "--no-nmea", "--no-highrate",
                      NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.skip_bias_cal && a.no_nmea && a.no_hirate,
               "all three booleans");
    }

    /* --foreground is accepted and ignored (compat with pre-systemd units). */
    {   char *v[] = { "imud", "--foreground", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == 0, "--foreground accepted");
        EXPECT(!a.skip_bias_cal && !a.no_nmea && !a.no_hirate,
               "--foreground sets nothing");
        EXPECT(out[0] == '\0', "--foreground is silent");
    }

    {   char *v[] = { "imud", "--version", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == 1, "--version handled");
        EXPECT(has(out, "imud " IMUD_VERSION_STR), "--version prints the version");
    }

    {   char *v[] = { "imud", "--bogus", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == -1, "unknown → error");
        EXPECT(has(out, "unknown option: --bogus"), "names the bad option");
        EXPECT(has(out, "Usage: imud"), "prints usage");
    }

    /* A later option must not be silently eaten as an earlier one's value. */
    {   char *v[] = { "imud", "--config", "/tmp/a.conf", "--no-nmea", NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.no_nmea &&
               strcmp(a.config_path, "/tmp/a.conf") == 0, "value then flag");
    }

    /* An over-long value must truncate into the fixed buffer, not overflow. */
    {   char big[600];
        memset(big, 'x', sizeof big - 1); big[sizeof big - 1] = '\0';
        char *v[] = { "imud", "--config", big, NULL };
        cap_begin(); rc = cli_parse_imud(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strlen(a.config_path) == sizeof a.config_path - 1,
               "over-long --config truncates to the buffer");
    }

    end(fb);
}

/* ── imud-cal ────────────────────────────────────────────────────────────── */

static void test_cal(void)
{
    begin("test_cal");
    int fb = g_fail;
    cli_cal_t a;
    const char *out;
    int rc;

    /* The three sensor modes: no --from needed, offline false. */
    static const char *sensor[] = { "mag", "gyro", "accel" };
    for (unsigned i = 0; i < sizeof sensor / sizeof *sensor; i++) {
        char *v[] = { "imud-cal", (char *)sensor[i], NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.mode, sensor[i]) == 0 && !a.offline,
               "sensor mode accepted, not offline");
    }

    /* The three offline modes: --from required, offline true. */
    static const char *offline[] = { "characterize", "fit-temp", "fit-ra" };
    for (unsigned i = 0; i < sizeof offline / sizeof *offline; i++) {
        char *v[] = { "imud-cal", (char *)offline[i], "--from", "/tmp/c.imucap",
                      NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.offline &&
               strcmp(a.from_path, "/tmp/c.imucap") == 0,
               "offline mode with --from");

        char *w[] = { "imud-cal", (char *)offline[i], NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(w), w, &a); out = cap_end();
        EXPECT(rc == -1, "offline mode without --from → error");
        EXPECT(has(out, "requires --from FILE"), "says --from is required");
    }

    {   char *v[] = { "imud-cal", NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == -1, "no mode → error");
        EXPECT(has(out, "Usage: imud-cal"), "no mode prints usage");
        EXPECT(!has(out, "unknown mode"), "no mode is not 'unknown mode'");
    }

    {   char *v[] = { "imud-cal", "wobble", NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == -1, "unknown mode → error");
        EXPECT(has(out, "unknown mode 'wobble'"), "names the bad mode");
    }

    {   char *v[] = { "imud-cal", "--output", "/tmp/cal.json", "gyro", NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.output_path &&
               strcmp(a.output_path, "/tmp/cal.json") == 0, "--output");
    }

    /* Options may precede or follow the positional mode. */
    {   char *v[] = { "imud-cal", "gyro", "--config", "/tmp/y.conf", NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.config_path, "/tmp/y.conf") == 0,
               "--config after the mode");
    }

    /* Only one positional is allowed; the second falls through to unknown. */
    {   char *v[] = { "imud-cal", "gyro", "mag", NULL };
        cap_begin(); rc = cli_parse_cal(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == -1, "second positional → error");
        EXPECT(has(out, "unknown option: mag"), "names the extra positional");
    }

    end(fb);
}

/* ── imud-mon ────────────────────────────────────────────────────────────── */

static void test_mon(void)
{
    begin("test_mon");
    int fb = g_fail;
    cli_mon_t a;
    const char *out;
    int rc;

    {   char *v[] = { "imud-mon", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.want_nmea && a.want_binary,
               "no stream args → both streams");
        EXPECT(strcmp(a.config_path, "/etc/imud/imud.conf") == 0,
               "default config");
    }

    {   char *v[] = { "imud-mon", "nmea", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.want_nmea && !a.want_binary, "nmea only");
    }

    {   char *v[] = { "imud-mon", "binary", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && !a.want_nmea && a.want_binary, "binary only");
    }

    {   char *v[] = { "imud-mon", "nmea", "binary", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.want_nmea && a.want_binary, "both named");
    }

    /* --config alone still means "both streams" — it is not a stream arg. */
    {   char *v[] = { "imud-mon", "--config", "/tmp/m.conf", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.want_nmea && a.want_binary &&
               strcmp(a.config_path, "/tmp/m.conf") == 0,
               "--config does not count as a stream");
    }

    {   char *v[] = { "imud-mon", "--help", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == 1, "--help handled");
        EXPECT(has(out, "Usage: imud-mon"), "--help prints usage");
    }

    {   char *v[] = { "imud-mon", "-h", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 1, "-h handled");
    }

    {   char *v[] = { "imud-mon", "--bogus", NULL };
        cap_begin(); rc = cli_parse_mon(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == -1, "unknown → error");
        EXPECT(has(out, "unknown argument: --bogus"), "names the bad argument");
    }

    end(fb);
}

/* ── imud-status ─────────────────────────────────────────────────────────── */

static void test_status(void)
{
    begin("test_status");
    int fb = g_fail;
    cli_status_t a;
    const char *out;
    int rc;

    {   char *v[] = { "imud-status", NULL };
        cap_begin(); rc = cli_parse_status(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.sockpath, CLI_DEFAULT_STATUS_SOCK) == 0,
               "default socket path");
    }

    /* The default must agree with what imud actually binds (main.c
     * STATUS_SOCK) — a drift here breaks imud-status against a stock daemon. */
    EXPECT(strcmp(CLI_DEFAULT_STATUS_SOCK, "/run/imud/imud.sock") == 0,
           "default socket is /run/imud/imud.sock");

    {   char *v[] = { "imud-status", "--socket", "/tmp/s.sock", NULL };
        cap_begin(); rc = cli_parse_status(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.sockpath, "/tmp/s.sock") == 0, "--socket");
    }

    {   char *v[] = { "imud-status", "--help", NULL };
        cap_begin(); rc = cli_parse_status(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == 1, "--help handled");
        EXPECT(has(out, "Usage: imud-status"), "--help prints usage");
    }

    {   char *v[] = { "imud-status", "-h", NULL };
        cap_begin(); rc = cli_parse_status(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 1, "-h handled");
    }

    {   char *v[] = { "imud-status", "--bogus", NULL };
        cap_begin(); rc = cli_parse_status(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == -1, "unknown → error");
        EXPECT(has(out, "Usage: imud-status"), "unknown prints usage");
    }

    end(fb);
}

/* ── imud-imutest ────────────────────────────────────────────────────────── */

static void test_imutest(void)
{
    begin("test_imutest");
    int fb = g_fail;
    cli_imutest_t a;
    const char *out;
    int rc;

    {   char *v[] = { "imud-imutest", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0, "no args → run");
        EXPECT(a.phases == IMT_PHASE_ALL, "no phase flag → all four");
        EXPECT(!a.have_config_arg, "have_config_arg false without --config");
        EXPECT(a.report_path[0] == '\0', "no default report path");
        /* Every override must start at its "not given" sentinel: imutest_main
         * distinguishes an explicit value from an absent flag by sign/NULL. */
        EXPECT(!a.ov_imu && !a.ov_mag && !a.ov_bus && !a.ov_chip,
               "string overrides NULL");
        EXPECT(a.ov_imu_addr < 0 && a.ov_mag_addr < 0 && a.ov_int_gpio < 0,
               "address overrides negative");
        EXPECT(a.ov_odr < 0 && a.ov_accel < 0 && a.ov_gyro < 0 && a.ov_wm < 0,
               "int overrides negative");
        EXPECT(a.ov_odr_win < 0 && a.ov_noise_win < 0 && a.ov_drdy_win < 0 &&
               a.ov_turn < 0 && a.ov_grav_tol < 0 && a.ov_odr_tol < 0,
               "double overrides negative");
        EXPECT(!a.force && !a.quiet && !a.non_interactive &&
               !a.no_fs && !a.no_ovf && !a.no_regdiff && !a.degauss,
               "booleans clear");
    }

    {   char *v[] = { "imud-imutest", "--config", "/tmp/i.conf", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.config_path, "/tmp/i.conf") == 0 &&
               a.have_config_arg, "--config also sets have_config_arg");
    }

    {   char *v[] = { "imud-imutest", "--report", "/tmp/r.md", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && strcmp(a.report_path, "/tmp/r.md") == 0, "--report");
    }

    {   char *v[] = { "imud-imutest", "--imu-driver", "ism330dhcx",
                      "--mag-driver", "none", "--i2c-bus", "/dev/i2c-3",
                      "--gpio-chip", "gpiochip4", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && !strcmp(a.ov_imu, "ism330dhcx") &&
               !strcmp(a.ov_mag, "none") && !strcmp(a.ov_bus, "/dev/i2c-3") &&
               !strcmp(a.ov_chip, "gpiochip4"), "string overrides");
    }

    /* I2C addresses are documented as hex; strtol base 0 is what makes 0x
     * work, and a decimal value must still parse. */
    {   char *v[] = { "imud-imutest", "--imu-addr", "0x6A",
                      "--mag-addr", "0x0C", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_imu_addr == 0x6A && a.ov_mag_addr == 0x0C,
               "hex addresses");
    }
    {   char *v[] = { "imud-imutest", "--imu-addr", "106", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_imu_addr == 106, "decimal address");
    }

    /* "--int-gpio 0 skips the interrupt check" per the usage text, so 0 must
     * survive the parse as 0 and not be confused with the -1 sentinel. */
    {   char *v[] = { "imud-imutest", "--int-gpio", "0", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_int_gpio == 0, "--int-gpio 0 is a value, not absent");
    }

    {   char *v[] = { "imud-imutest", "--odr", "208", "--accel-g", "4",
                      "--gyro-dps", "500", "--fifo-wm", "32", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_odr == 208000 && a.ov_accel == 4 &&
               a.ov_gyro == 500 && a.ov_wm == 32, "int overrides");
    }

    /*
     * --odr is written in Hz and kept in milli-Hz, and a fraction is a real
     * rate rather than a typo: 12.5 Hz is a TDK rung, and the ST ladder's own
     * bottom rung is 13.016 Hz.  The flag has to express what [imu] odr_hz
     * expresses, or the file and the command line disagree about the part.
     */
    {   char *v[] = { "imud-imutest", "--odr", "12.5", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_odr == 12500, "--odr 12.5 is 12500 milli-Hz");
    }
    {   char *v[] = { "imud-imutest", "--odr", "13.016", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_odr == 13016, "--odr 13.016 survives to milli-Hz");
    }
    {   char *v[] = { "imud-imutest", "--odr", "0", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc != 0, "--odr 0 is refused, not stored as a rate");
    }

    /* --odr-tol is the only unit conversion in any of the five parsers:
     * the flag is a percentage, the field is a fraction. */
    {   char *v[] = { "imud-imutest", "--odr-tol", "5", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_odr_tol > 0.0499 && a.ov_odr_tol < 0.0501,
               "--odr-tol 5 → 0.05");
    }

    {   char *v[] = { "imud-imutest", "--odr-window", "7.5",
                      "--noise-window", "12", "--drdy-window", "4",
                      "--turn-deg", "45", "--grav-tol", "0.5", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.ov_odr_win == 7.5 && a.ov_noise_win == 12.0 &&
               a.ov_drdy_win == 4.0 && a.ov_turn == 45.0 &&
               a.ov_grav_tol == 0.5, "double overrides");
    }

    /*
     * A bad numeric argument is a usage error, not a silent 0.
     *
     * cli_parse_imutest is the only one of the five parsers that reads
     * numbers, and it used atoi/atof — no error signal at all, and undefined
     * by the letter for an unrepresentable value (C17 7.22.1.2p3). The
     * practical failures were worse than the UB and split three ways:
     *
     *   --odr and friends are applied `if (ov > 0)`, so garbage became 0 and
     *   the override was silently discarded — the tool ran on the config value
     *   and said nothing. --imu-addr is applied `if (ov >= 0)`, so garbage
     *   became 0 and TOOK, probing I2C address 0. And --int-gpio, also `>= 0`,
     *   documents 0 as "skips the interrupt check" — so a typo skipped the
     *   check the operator asked for while the report still said PASS.
     *
     * Every numeric flag, both malformed and out of range, so a regression in
     * any one of them is caught rather than the first.
     */
    static const char *num_flags[] = {
        "--imu-addr", "--mag-addr", "--int-gpio",
        "--odr", "--accel-g", "--gyro-dps", "--fifo-wm",
        "--odr-window", "--noise-window", "--drdy-window",
        "--turn-deg", "--grav-tol", "--odr-tol",
    };
    static const char *bad_vals[] = {
        "abc",          /* not a number at all — the old atoi(…) == 0 case  */
        "12x",          /* trailing garbage: strtol stops, we must not      */
        "",             /* empty                                            */
        "99999999999999999999",  /* beyond long: ERANGE                     */
        "4294977414",   /* fits a long, wraps an int — the config-side bug  */
    };
    for (unsigned f = 0; f < sizeof num_flags / sizeof num_flags[0]; f++) {
        for (unsigned b = 0; b < sizeof bad_vals / sizeof bad_vals[0]; b++) {
            char *v[] = { (char *)"imud-imutest", (char *)num_flags[f],
                          (char *)bad_vals[b], NULL };
            cap_begin();
            rc = cli_parse_imutest(argc_of(v), v, &a);
            const char *out = cap_end();
            EXPECT(rc == -1, "bad numeric argument is a usage error");
            EXPECT(has(out, num_flags[f]),
                   "the error names the offending flag");
        }
    }

    /* The float flags additionally reject non-finite and non-positive values.
     * "nan" and "inf" both parse fine through strtod, and each would have
     * flowed into a measurement window or tolerance. */
    static const char *bad_dbls[] = { "nan", "inf", "-inf", "0", "-1" };
    for (unsigned b = 0; b < sizeof bad_dbls / sizeof bad_dbls[0]; b++) {
        char *v[] = { (char *)"imud-imutest", (char *)"--turn-deg",
                      (char *)bad_dbls[b], NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == -1, "non-finite or non-positive window rejected");
    }

    {   char *v[] = { "imud-imutest", "--passive", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == IMT_PHASE_PASSIVE, "--passive alone");
    }
    {   char *v[] = { "imud-imutest", "--faces", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == IMT_PHASE_FACES, "--faces alone");
    }
    {   char *v[] = { "imud-imutest", "--gyro", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == IMT_PHASE_GYRO, "--gyro alone");
    }
    {   char *v[] = { "imud-imutest", "--spin", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == IMT_PHASE_SPIN, "--spin alone");
    }

    /* Phase flags OR together; --all is an assignment, so it wins whichever
     * side of the other flags it lands on. */
    {   char *v[] = { "imud-imutest", "--passive", "--spin", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == (IMT_PHASE_PASSIVE | IMT_PHASE_SPIN),
               "phase flags OR");
    }
    {   char *v[] = { "imud-imutest", "--all", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == IMT_PHASE_ALL, "--all");
    }
    {   char *v[] = { "imud-imutest", "--passive", "--all", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == IMT_PHASE_ALL, "--all after --passive");
    }
    {   char *v[] = { "imud-imutest", "--all", "--passive", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.phases == IMT_PHASE_ALL, "--passive after --all");
    }

    {   char *v[] = { "imud-imutest", "--no-fs-sweep", "--no-overflow",
                      "--no-regdiff", "--force", "--quiet",
                      "--non-interactive", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.no_fs && a.no_ovf && a.no_regdiff && a.force &&
               a.quiet && a.non_interactive, "all six booleans");
    }

    /* Recovery mode is a plain boolean and does not disturb the phase bits:
     * imutest_main runs it instead of them, so a stray phase flag alongside
     * it must not change what --degauss does. */
    {   char *v[] = { "imud-imutest", "--degauss", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 0 && a.degauss, "--degauss sets the flag");
        EXPECT(a.phases == IMT_PHASE_ALL, "and leaves the phase default alone");
    }

    {   char *v[] = { "imud-imutest", "--version", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == 1, "--version handled");
        EXPECT(has(out, "imud-imutest " IMUD_VERSION_STR), "prints the version");
    }

    {   char *v[] = { "imud-imutest", "--help", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == 1, "--help handled");
        EXPECT(has(out, "Usage: imud-imutest"), "--help prints usage");
    }
    {   char *v[] = { "imud-imutest", "-h", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); cap_end();
        EXPECT(rc == 1, "-h handled");
    }

    {   char *v[] = { "imud-imutest", "--bogus", NULL };
        cap_begin(); rc = cli_parse_imutest(argc_of(v), v, &a); out = cap_end();
        EXPECT(rc == -1, "unknown → error");
        EXPECT(has(out, "unknown option: --bogus"), "names the bad option");
    }

    end(fb);
}

/* ── The i + 1 < argc guard, once per value-taking flag ──────────────────── */

/*
 * This is the test SECURITY.md is pointing at.  Each of these flags reads
 * argv[i + 1]; every one of them is guarded by "i + 1 < argc", and dropping
 * any single guard would read one past the end of argv.  Rather than trust one
 * representative case, every flag is driven as the final argument.
 *
 * A missing guard would not fault here (argv[argc] is the NULL terminator, so
 * the strcmp/snprintf would take a NULL) — but under ASan/UBSan in CI, and
 * against a real execve where nothing promises what follows, it is a genuine
 * read past the array.  The assertion is that the parser rejects rather than
 * dereferences: the flag falls through to the unknown-option branch.
 */

typedef int (*parse_fn)(int, char **, void *);

static int p_imud   (int c, char **v, void *a) { return cli_parse_imud(c, v, a); }
static int p_cal    (int c, char **v, void *a) { return cli_parse_cal(c, v, a); }
static int p_mon    (int c, char **v, void *a) { return cli_parse_mon(c, v, a); }
static int p_status (int c, char **v, void *a) { return cli_parse_status(c, v, a); }
static int p_imutest(int c, char **v, void *a) { return cli_parse_imutest(c, v, a); }

static void test_missing_value(void)
{
    begin("test_missing_value");
    int fb = g_fail;

    static const char *imud_flags[]    = { "--config", "--replay", NULL };
    static const char *cal_flags[]     = { "--config", "--output", "--from", NULL };
    static const char *mon_flags[]     = { "--config", NULL };
    static const char *status_flags[]  = { "--socket", NULL };
    static const char *imutest_flags[] = {
        "--config", "--report", "--imu-driver", "--mag-driver", "--i2c-bus",
        "--gpio-chip", "--imu-addr", "--mag-addr", "--int-gpio", "--odr",
        "--accel-g", "--gyro-dps", "--fifo-wm", "--odr-window",
        "--noise-window", "--drdy-window", "--turn-deg", "--grav-tol",
        "--odr-tol", NULL };

    static const struct {
        const char   *prog;
        parse_fn      fn;
        const char  **flags;
    } tools[] = {
        { "imud",         p_imud,    imud_flags    },
        { "imud-cal",     p_cal,     cal_flags     },
        { "imud-mon",     p_mon,     mon_flags     },
        { "imud-status",  p_status,  status_flags  },
        { "imud-imutest", p_imutest, imutest_flags },
    };

    /* Big enough for the widest of the five arg structs. */
    union {
        cli_imud_t imud; cli_cal_t cal; cli_mon_t mon;
        cli_status_t status; cli_imutest_t imutest;
    } a;

    int checked = 0;
    for (unsigned t = 0; t < sizeof tools / sizeof *tools; t++) {
        for (const char **f = tools[t].flags; *f; f++) {
            char *v[] = { (char *)tools[t].prog, (char *)*f, NULL };
            cap_begin();
            int rc = tools[t].fn(argc_of(v), v, &a);
            const char *out = cap_end();
            EXPECT(rc == -1, "value flag with no value → error");
            EXPECT(out[0] != '\0', "value flag with no value prints something");
            checked++;
        }
    }
    /* Pinned deliberately: `grep -c 'i + 1 < argc' src/cli.c` must agree.
     * Adding a value-taking flag without listing it above fails here rather
     * than leaving the new flag's guard untested. */
    EXPECT(checked == 26, "every value-taking flag was covered");

    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

/*
 * The stream contract, for every parser: --help is requested output and goes
 * to stdout with rc 1 (which main() maps to exit 0); an unknown option is a
 * diagnostic and goes to stderr with rc -1 (exit 1).  Neither may leak onto
 * the other stream.
 *
 * This is the assertion a single-fd capture structurally cannot
 * make.  It matters twice over: help2man builds the man pages from stdout, so
 * usage on stderr yields an empty OPTIONS section; and a script doing
 * `imud-status > report.txt` must not find a usage dump in the file when the
 * command failed.
 */
static void test_help_and_error_streams(void)
{
    begin("test_help_and_error_streams");
    int fb = g_fail;
    static char out[8192], err[8192];

    /* Macros rather than a table: the five parsers take five different
     * argument types, so there is no one function-pointer type to hold them. */
#define HELP_CASE(PROG, TYPE, PARSE)                                          \
    do {                                                                      \
        TYPE a;                                                               \
        char *v[] = { (char *)PROG, (char *)"--help", NULL };                 \
        cap2_begin();                                                         \
        int rc = PARSE(argc_of(v), v, &a);                                    \
        cap2_end(out, sizeof out, err, sizeof err);                           \
        EXPECT(rc == 1, PROG " --help → rc 1 (exit 0)");                      \
        EXPECT(has(out, "Usage: " PROG), PROG " --help writes usage to stdout"); \
        EXPECT(err[0] == '\0', PROG " --help writes NOTHING to stderr");      \
    } while (0)

#define BAD_CASE(PROG, TYPE, PARSE)                                           \
    do {                                                                      \
        TYPE a;                                                               \
        char *v[] = { (char *)PROG, (char *)"--nope", NULL };                 \
        cap2_begin();                                                         \
        int rc = PARSE(argc_of(v), v, &a);                                    \
        cap2_end(out, sizeof out, err, sizeof err);                           \
        EXPECT(rc == -1, PROG " --nope → rc -1 (exit 1)");                    \
        EXPECT(has(err, "Usage: " PROG), PROG " error writes usage to stderr"); \
        EXPECT(out[0] == '\0', PROG " error writes NOTHING to stdout");       \
    } while (0)

    HELP_CASE("imud",          cli_imud_t,    cli_parse_imud);
    HELP_CASE("imud-cal",      cli_cal_t,     cli_parse_cal);
    HELP_CASE("imud-mon",      cli_mon_t,     cli_parse_mon);
    HELP_CASE("imud-status",   cli_status_t,  cli_parse_status);
    HELP_CASE("imud-imutest",  cli_imutest_t, cli_parse_imutest);

    BAD_CASE("imud",           cli_imud_t,    cli_parse_imud);
    BAD_CASE("imud-cal",       cli_cal_t,     cli_parse_cal);
    BAD_CASE("imud-mon",       cli_mon_t,     cli_parse_mon);
    BAD_CASE("imud-status",    cli_status_t,  cli_parse_status);
    BAD_CASE("imud-imutest",   cli_imutest_t, cli_parse_imutest);

#undef HELP_CASE
#undef BAD_CASE

    /* -h is the same door as --help, on every tool that offers it. */
    {   cli_imud_t a;
        char *v[] = { "imud", "-h", NULL };
        cap2_begin();
        int rc = cli_parse_imud(argc_of(v), v, &a);
        cap2_end(out, sizeof out, err, sizeof err);
        EXPECT(rc == 1 && has(out, "Usage: imud") && err[0] == '\0',
               "imud -h behaves as --help");
    }

    end(fb);
}

/*
 * --version on every tool: the string is what help2man puts in the .TH footer,
 * so it must be "<prog> <version>" on stdout with rc 1 and nothing on stderr.
 */
static void test_version_strings(void)
{
    begin("test_version_strings");
    int fb = g_fail;
    static char out[8192], err[8192];

#define VER_CASE(PROG, TYPE, PARSE)                                           \
    do {                                                                      \
        TYPE a;                                                               \
        char *v[] = { (char *)PROG, (char *)"--version", NULL };              \
        cap2_begin();                                                         \
        int rc = PARSE(argc_of(v), v, &a);                                    \
        cap2_end(out, sizeof out, err, sizeof err);                           \
        EXPECT(rc == 1, PROG " --version → rc 1");                            \
        EXPECT(has(out, PROG " " IMUD_VERSION_STR),                           \
               PROG " --version prints '" PROG " " IMUD_VERSION_STR "'");     \
        EXPECT(err[0] == '\0', PROG " --version writes nothing to stderr");   \
    } while (0)

    VER_CASE("imud",          cli_imud_t,    cli_parse_imud);
    VER_CASE("imud-cal",      cli_cal_t,     cli_parse_cal);
    VER_CASE("imud-mon",      cli_mon_t,     cli_parse_mon);
    VER_CASE("imud-status",   cli_status_t,  cli_parse_status);
    VER_CASE("imud-imutest",  cli_imutest_t, cli_parse_imutest);

#undef VER_CASE
    end(fb);
}

int main(void)
{
    printf("test_cli — argv parsing for the five non-bridge entry points\n");

    test_imud();
    test_cal();
    test_mon();
    test_status();
    test_imutest();
    test_missing_value();
    test_help_and_error_streams();
    test_version_strings();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
