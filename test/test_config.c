/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_config.c — unit tests for config_defaults() and config_load()
 *
 * Tests exercise type dispatch (hex int, bool, float, double, quoted string),
 * error paths (missing file, bad type), tilde expansion, unknown-key /
 * unknown-section tolerance, and [position] WMM keys (Step 2).
 * Edge-case tests write small temp files to /tmp/imud_test_NNN.conf and
 * remove them on exit.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "config.h"

/* ── Minimal test framework ─────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR_D(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

#define EXPECT_STR(a, b, msg) \
    EXPECT(strcmp((a), (b)) == 0, msg)

static void begin_test(const char *name)
{
    printf("%-44s", name);
    fflush(stdout);
}

static void end_test(int fail_before)
{
    puts(g_fail == fail_before ? "OK" : "FAIL");
}

/* ── Helpers ────────────────────────────────────────────────────────────── */

/* Write lines to a temp file, return path (static buffer — one at a time). */
static const char *write_tmpconf(int id, const char *content)
{
    static char path[64];
    snprintf(path, sizeof(path), "/tmp/imud_test_%d.conf", id);
    FILE *f = fopen(path, "w");
    if (!f) { perror("fopen tmp"); exit(1); }
    fputs(content, f);
    fclose(f);
    return path;
}

/* ── Tests ──────────────────────────────────────────────────────────────── */

/* config_defaults() spot-checks: types and values from spec §9. */
static void test_defaults_values(void)
{
    begin_test("test_defaults_values");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);

    EXPECT_STR(cfg.i2c_bus,          "/dev/i2c-1",    "i2c_bus default");
    EXPECT_STR(cfg.sim_file,         "",              "sim_file default (synthesis)");
    EXPECT(cfg.sim_loop == false,                     "sim_loop default false");
    EXPECT_NEAR_D(cfg.sim_speed, 1.0, 1e-6,           "sim_speed default 1.0");
    EXPECT_STR(cfg.imu_driver,       "ism330dhcx",    "imu_driver default");
    EXPECT(cfg.imu_addr     == 0x6B,                  "imu_addr default");
    EXPECT(cfg.imu_odr_hz   == 833,                   "imu_odr_hz default");
    EXPECT(cfg.imu_accel_g  == 8,                     "imu_accel_g default");
    EXPECT(cfg.imu_gyro_dps == 2000,                  "imu_gyro_dps default");
    EXPECT(cfg.imu_fifo_wm  == 64,                    "imu_fifo_wm default");
    EXPECT_STR(cfg.mag_driver,       "mmc5983ma",     "mag_driver default");
    EXPECT(cfg.mag_addr     == 0x30,                  "mag_addr default");
    EXPECT(cfg.nmea_enabled  == false,                "nmea_enabled default off (1.6: local stream only)");
    EXPECT(cfg.nmea_rate_hz  == 10,                   "nmea_rate_hz default");
    EXPECT(cfg.nmea_dest_port == 10110,               "nmea_dest_port default");
    EXPECT(cfg.nmea_tcp_enabled == false,             "nmea_tcp_enabled default off");
    EXPECT_STR(cfg.nmea_tcp_bind_addr, "0.0.0.0",     "nmea_tcp_bind_addr default");
    EXPECT(cfg.nmea_tcp_port == 10110,                "nmea_tcp_port default 10110");
    EXPECT(cfg.highrate_enabled == false,             "highrate_enabled default (opt-in)");
    EXPECT(cfg.highrate_rate_hz == 500,               "highrate_rate_hz default");
    EXPECT(cfg.highrate_dest_port == 10111,           "highrate_dest_port default");
    EXPECT_STR(cfg.highrate_coord_frame, "NED",       "coord_frame default");
    EXPECT_NEAR_D(cfg.mekf_gyro_noise,  0.007,  1e-9, "gyro_noise default");
    EXPECT_NEAR_D(cfg.mag_reject_gauss, 0.05,   1e-9, "mag_reject default (0.05 G anomaly)");
    EXPECT_NEAR_D(cfg.accel_skip_thresh,0.05,   1e-9, "accel_skip default");
    EXPECT(cfg.mag_yaw_only == true,                  "mag_yaw_only default true (marine)");
    EXPECT_NEAR_D(cfg.heave_tau_s, 12.0, 1e-5,        "heave_tau_s default 12 s");
    EXPECT_NEAR_D(cfg.mekf_wave_accel, 0.8, 1e-9,     "mekf_wave_accel default 0.8 m/s²");
    EXPECT_NEAR_D(cfg.mekf_wave_accel_tau_s, 0.5, 1e-9,
                  "mekf_wave_accel_tau_s default 0.5 s");
    EXPECT_NEAR_D(cfg.mekf_mag_dip_sigma_deg, 1.0, 1e-9,
                  "mekf_mag_dip_sigma_deg default 1.0 deg");
    EXPECT_NEAR_D(cfg.align_window_sec, 5.0, 1e-9,
                  "align_window_sec default 5 s");
    EXPECT_NEAR_D(cfg.gyro_bias_sec,    2.0,    1e-9, "gyro_bias_sec default");
    EXPECT_STR(cfg.log_level, "warn",                 "log_level default");
    EXPECT(cfg.log_stats_hz == 1,                     "log_stats_hz default");
    EXPECT(cfg.publish_heave == true,                 "publish_heave default true");
    end_test(fb);
}

/* cal_file default is an absolute path to the service data directory. */
static void test_defaults_cal_file(void)
{
    begin_test("test_defaults_cal_file");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);

    EXPECT(cfg.cal_file[0] == '/',  "cal_file default is an absolute path");
    EXPECT_STR(cfg.cal_file, "/etc/imud/cal.json", "cal_file default value");
    end_test(fb);
}

/*
 * Load the real config/imud.conf and check a spread of types:
 *   - hex int  (imu_addr = 0x6B)
 *   - decimal int (highrate_dest_port = 10111)
 *   - bool (stream_enabled = true, nmea_enabled = false — the reference
 *     config enables only the local stream socket)
 *   - float (mag_set_period_s = 5.0)
 *   - double (mekf_gyro_noise = 0.007)
 *   - quoted string (nmea_dest_addr = "255.255.255.255")
 */
static void test_load_real_conf(void)
{
    begin_test("test_load_real_conf");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);

    int rc = config_load("config/imud.conf", &cfg);
    EXPECT(rc == 0, "config_load returns 0");
    if (rc != 0) { end_test(fb); return; }

    EXPECT(cfg.imu_addr == 0x6B,                      "hex int 0x6B");
    EXPECT(cfg.imu_addr == 107,                        "0x6B == 107");
    EXPECT(cfg.imu_odr_hz == 833,                      "decimal int odr_hz");
    EXPECT(cfg.imu_fifo_wm == 64,                      "decimal int fifo_wm");
    EXPECT(cfg.nmea_enabled == false,                  "bool false (stream-only conf)");
    EXPECT(cfg.highrate_enabled == false,              "highrate disabled in conf");
    EXPECT(cfg.stream_enabled == true,                 "bool true (stream socket on)");
    EXPECT(cfg.highrate_dest_port == 10111,            "decimal int port");
    EXPECT_NEAR_D(cfg.mag_set_period_s, 5.0, 1e-5,    "float set_period_s");
    EXPECT_NEAR_D(cfg.mekf_gyro_noise,  0.007, 1e-9,  "double gyro_noise");
    EXPECT_NEAR_D(cfg.mekf_gyro_bias,   0.00015, 1e-9,"double gyro_bias");
    EXPECT_STR(cfg.nmea_dest_addr, "255.255.255.255",  "quoted string");
    EXPECT_STR(cfg.highrate_coord_frame, "NED",        "quoted coord_frame");
    EXPECT_STR(cfg.imu_driver, "ism330dhcx",           "quoted driver string");
    end_test(fb);
}

/* Missing file must return CONFIG_ERR_OPEN — survivable, the daemon runs on
 * defaults.  A file that EXISTS but cannot be read is a different failure and
 * must be distinguishable, or callers cannot refuse to start on it: the bridge
 * configs install 0640, so a service outside the imud group hits exactly this. */
static void test_load_missing_file(void)
{
    begin_test("test_load_missing_file");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load("/tmp/imud_no_such_file_xyz.conf", &cfg);
    EXPECT(rc == CONFIG_ERR_OPEN, "missing file returns CONFIG_ERR_OPEN");

    /* Root ignores the mode bits, so this half only means something as a
     * normal user (the .deb build containers run as root).  Own path rather
     * than write_tmpconf's fixed name: this file is chmod 0 for a moment, and
     * a crash in between must not leave a stale unwritable name behind for the
     * next run to trip over. */
    if (geteuid() != 0) {
        char noperm[64];
        snprintf(noperm, sizeof noperm, "/tmp/imud_test_noperm_%d.conf",
                 (int)getpid());
        FILE *nf = fopen(noperm, "w");
        if (nf) {
            fputs("[stream]\nrate_hz = 25\n", nf);
            fclose(nf);
            if (chmod(noperm, 0) == 0) {
                config_defaults(&cfg);
                EXPECT(config_load(noperm, &cfg) == CONFIG_ERR_PERM,
                       "unreadable file returns CONFIG_ERR_PERM");
            }
            remove(noperm);
        }
    } else {
        printf("  (skipped unreadable-file case: running as root)\n");
    }
    end_test(fb);
}

/* Bad integer value: reported as CONFIG_ERR_PARSE, but the parse continues —
 * the bad key keeps its default and later lines are still applied. */
static void test_load_bad_int(void)
{
    begin_test("test_load_bad_int");
    int fb = g_fail;
    const char *path = write_tmpconf(1,
        "[imu]\n"
        "odr_hz = notanumber\n"
        "gyro_dps = 500\n");
    imud_config_t def;
    config_defaults(&def);
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE,          "bad int returns CONFIG_ERR_PARSE");
    EXPECT(cfg.imu_odr_hz == def.imu_odr_hz, "bad key keeps its default");
    EXPECT(cfg.imu_gyro_dps == 500,          "line after the bad one still applied");
    remove(path);
    end_test(fb);
}

/*
 * The four MEKF noise densities all end up as variances in a denominator, so
 * zero or negative is a broken filter, not a tuning choice: mekf_accel_noise
 * = 0 gives Ra = 0 and hence a Kalman gain of exactly 1, snapping attitude
 * onto every raw accel sample. Must be rejected rather than clamped, so a
 * config typo cannot hide behind plausible-looking output.
 */
static void test_load_noise_density_must_be_positive(void)
{
    begin_test("test_load_noise_density_must_be_positive");
    int fb = g_fail;
    imud_config_t def;
    config_defaults(&def);

    static const char *keys[] = {
        "mekf_gyro_noise", "mekf_gyro_bias",
        "mekf_accel_noise", "mekf_mag_noise",
    };
    for (unsigned i = 0; i < sizeof keys / sizeof keys[0]; i++) {
        for (int neg = 0; neg < 2; neg++) {
            char body[128];
            snprintf(body, sizeof body, "[fusion]\n%s = %s\n",
                     keys[i], neg ? "-0.001" : "0.0");
            const char *path = write_tmpconf(70 + i * 2 + neg, body);
            imud_config_t cfg;
            config_defaults(&cfg);
            int rc = config_load(path, &cfg);
            EXPECT(rc == CONFIG_ERR_PARSE,
                   neg ? "negative noise density rejected"
                       : "zero noise density rejected");
            remove(path);
        }
    }

    /* A positive value on the same key must still load normally. */
    const char *ok = write_tmpconf(79,
        "[fusion]\n"
        "mekf_accel_noise = 0.0044\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load(ok, &cfg) == 0, "positive noise density accepted");
    EXPECT(cfg.mekf_accel_noise == 0.0044,     "positive noise density applied");
    remove(ok);
    end_test(fb);
}

/*
 * Every sample and publish rate must be positive, for the same reason as the
 * noise densities above: a rate divides into a period, and the two sensor
 * rates additionally size the filter's noise variances. [mag] odr_hz = 0 gave
 * Rm = Nm² × 0 = 0 — a magnetometer Kalman gain of exactly 1, so the filter
 * snapped its heading onto every raw mag sample. A negative put a negative
 * variance into the innovation covariance. Rejected at parse time rather than
 * clamped, so the typo cannot hide behind plausible-looking output.
 *
 * The bridge rate_hz keys are covered too: bridge_period_ns() has a <= 0
 * fallback, but reaching it from a config file was always a mistake rather
 * than a way to ask for the default.
 */
static void test_load_rates_must_be_positive(void)
{
    begin_test("test_load_rates_must_be_positive");
    int fb = g_fail;

    static const struct { const char *section, *key; } rates[] = {
        { "imu",            "odr_hz"  },
        { "mag",            "odr_hz"  },
        { "nmea",           "rate_hz" },
        { "highrate",       "rate_hz" },
        { "stream",         "rate_hz" },
        { "imud-signalk",   "rate_hz" },
        { "imud-mqtt",      "rate_hz" },
        { "imud-influxdb",  "rate_hz" },
        { "imud-mavlink",   "rate_hz" },
    };
    for (unsigned i = 0; i < sizeof rates / sizeof rates[0]; i++) {
        for (int neg = 0; neg < 2; neg++) {
            char body[128];
            snprintf(body, sizeof body, "[%s]\n%s = %s\n",
                     rates[i].section, rates[i].key, neg ? "-1" : "0");
            const char *path = write_tmpconf(90 + i * 2 + neg, body);
            imud_config_t cfg;
            config_defaults(&cfg);
            int rc = config_load(path, &cfg);
            EXPECT(rc == CONFIG_ERR_PARSE,
                   neg ? "negative rate rejected" : "zero rate rejected");
            remove(path);
        }
    }

    /* Positive values on the same keys still load, including a rate of 1. */
    const char *ok = write_tmpconf(89,
        "[mag]\n"
        "odr_hz = 200\n"
        "[nmea]\n"
        "rate_hz = 1\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load(ok, &cfg) == 0, "positive rates accepted");
    EXPECT(cfg.mag_odr_hz == 200,      "positive mag odr_hz applied");
    EXPECT(cfg.nmea_rate_hz == 1,      "a rate of 1 Hz is legal");
    remove(ok);
    end_test(fb);
}

/*
 * No float key may be non-finite.  strtod() happily converts "nan", "inf" and
 * "1e999" (the last via ERANGE to HUGE_VAL), and parse_double only ever
 * checked that the whole token converted — so a typo or a corrupted file put
 * a NaN or an infinity straight into the filter's tuning.
 *
 * NEED_POS_DBL did not save us: its only test is `dv > 0.0`, and infinity
 * passes that.  It rejected NaN purely by accident, since every comparison
 * against NaN is false.  So "mekf_gyro_noise = inf" was accepted, and the
 * variance it sizes went to infinity.
 *
 * The stakes are the same as the positivity checks above, one row over: the
 * value parses, is accepted, and then makes the filter degenerate.  A NaN
 * reaching the quaternion is worse than degenerate — it never washes out, so
 * every packet, NMEA sentence and bridge delta for the life of the process
 * carries "nan".  Fatal at parse time, where the operator can still be told
 * which line is wrong.
 */
static void test_load_rejects_non_finite(void)
{
    begin_test("test_load_rejects_non_finite");
    int fb = g_fail;

    /* One key per macro that routes through parse_double, so a regression in
     * any one of the three is caught: NEED_POS_DBL, NEED_DBL, NEED_FLT. */
    static const struct { const char *section, *key; } keys[] = {
        { "fusion", "mekf_gyro_noise"   },  /* NEED_POS_DBL — inf passed > 0 */
        { "fusion", "accel_skip_thresh" },  /* NEED_DBL                      */
        { "fusion", "wave_tau_s"        },  /* NEED_FLT                      */
    };
    /* "1e999" overflows to infinity rather than parsing as one, which is the
     * form a corrupted or hand-edited file is most likely to contain. */
    static const char *vals[] = { "nan", "inf", "-inf", "1e999" };

    int id = 120;
    for (unsigned i = 0; i < sizeof keys / sizeof keys[0]; i++) {
        for (unsigned v = 0; v < sizeof vals / sizeof vals[0]; v++) {
            char body[128];
            snprintf(body, sizeof body, "[%s]\n%s = %s\n",
                     keys[i].section, keys[i].key, vals[v]);
            const char *path = write_tmpconf(id++, body);
            imud_config_t cfg;
            config_defaults(&cfg);
            EXPECT(config_load(path, &cfg) == CONFIG_ERR_PARSE,
                   "non-finite float rejected");
            remove(path);
        }
    }

    /* Ordinary finite values on the same three keys still load and land. */
    const char *ok = write_tmpconf(id,
        "[fusion]\n"
        "mekf_gyro_noise   = 0.007\n"
        "accel_skip_thresh = 0.15\n"
        "wave_tau_s        = 12.0\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load(ok, &cfg) == 0,             "finite floats accepted");
    EXPECT(cfg.mekf_gyro_noise == 0.007,           "finite NEED_POS_DBL applied");
    EXPECT(cfg.accel_skip_thresh == 0.15,          "finite NEED_DBL applied");
    EXPECT_NEAR_D(cfg.wave_tau_s, 12.0, 1e-6,      "finite NEED_FLT applied");
    remove(ok);
    end_test(fb);
}

/* Fill buf with exactly n 'x' characters and NUL-terminate. */
static const char *xstr(char *buf, size_t n)
{
    memset(buf, 'x', n);
    buf[n] = '\0';
    return buf;
}

/*
 * stderr capture, same idiom as test_cli.c's cap_begin/cap_end.  Needed here
 * because for the length failures the *message* is half the fix: several of
 * them are indistinguishable by return code alone — an over-long [mount]
 * preset and a misspelled one are both CONFIG_ERR_PARSE — so an assertion on
 * the code cannot tell whether the right guard fired.
 */
static int  g_saved_err = -1;
static char g_cap_path[128];

static void cap_begin(void)
{
    snprintf(g_cap_path, sizeof g_cap_path, "/tmp/imud_tcfg_%d.txt",
             (int)getpid());
    fflush(stderr);
    g_saved_err = dup(STDERR_FILENO);
    int fd = open(g_cap_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd >= 0) { dup2(fd, STDERR_FILENO); close(fd); }
}

static const char *cap_end(void)
{
    static char buf[4096];
    fflush(stderr);
    if (g_saved_err >= 0) { dup2(g_saved_err, STDERR_FILENO); close(g_saved_err); }
    g_saved_err = -1;

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

/*
 * An over-long string value must be fatal, not a warning.
 *
 * copy_str has always said the caller surfaces truncation — "a silently
 * shortened path (a socket, a cal file) binds or opens something the user
 * never asked for" — and NEED_STR logged LOG_W and carried on anyway.  So a
 * [stream] socket of 130 characters became a *different, perfectly valid*
 * 107-character path, the daemon bound that, and every bridge and libimud
 * client connected to the path written in the config file and found nothing.
 *
 * The rejected field must also keep its default, not the truncated value:
 * imud-mon ignores config_load's return on purpose ("defaults have the right
 * port numbers"), which makes it the one consumer that runs on a config the
 * daemon refused to start on.
 */
static void test_load_rejects_too_long_string(void)
{
    begin_test("test_load_rejects_too_long_string");
    int fb = g_fail;

    imud_config_t def, cfg;
    config_defaults(&def);
    char big[512], body[1024];

    /* stream_socket is char[108], sized to sun_path — 107 usable. */
    snprintf(body, sizeof body, "[stream]\nsocket = \"%s\"\n"
                                "[imu]\ngyro_dps = 500\n",
             xstr(big, sizeof def.stream_socket));
    const char *path = write_tmpconf(140, body);
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == CONFIG_ERR_PARSE,
           "over-long socket path rejected");
    EXPECT_STR(cfg.stream_socket, def.stream_socket,
           "rejected socket path keeps its default");
    EXPECT(cfg.imu_gyro_dps == 500,
           "line after the over-long one still applied");
    remove(path);

    /* A second length class, so the max is read from the field and not
     * hardcoded to one of them: cal_file is char[256]. */
    snprintf(body, sizeof body, "[calibration]\nfile = \"%s\"\n",
             xstr(big, sizeof def.cal_file));
    path = write_tmpconf(141, body);
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == CONFIG_ERR_PARSE,
           "over-long cal file path rejected");
    EXPECT_STR(cfg.cal_file, def.cal_file,
           "rejected cal file path keeps its default");
    remove(path);

    /*
     * The off-by-one pin, and the reason it is not optional: rejecting one
     * character early would break every operator with a long-but-legal socket
     * path, and that failure shows up only at their site.
     */
    snprintf(body, sizeof body, "[stream]\nsocket = \"%s\"\n",
             xstr(big, sizeof def.stream_socket - 1));
    path = write_tmpconf(142, body);
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0, "socket path at exactly the max accepted");
    EXPECT(strlen(cfg.stream_socket) == sizeof def.stream_socket - 1,
           "socket path at exactly the max applied whole");
    remove(path);

    /*
     * A value that fits but whose ~/ expansion does not.  expand_tilde used to
     * decline silently and copy_str could not see it, because it measured the
     * unexpanded value: the field kept a literal "~/..." that is then opened
     * relative to the cwd.  Same class as the truncation above.
     */
    const char *home_env = getenv("HOME");
    char home_save[512];
    snprintf(home_save, sizeof home_save, "%s", home_env ? home_env : "");
    setenv("HOME", xstr(big, sizeof def.cal_file - 6), 1);

    path = write_tmpconf(143, "[calibration]\nfile = \"~/cal.json\"\n");
    config_defaults(&cfg);
    cap_begin();
    int trc = config_load(path, &cfg);
    const char *tmsg = cap_end();
    EXPECT(trc == CONFIG_ERR_PARSE,
           "value rejected when ~/ expansion would overflow");
    EXPECT_STR(cfg.cal_file, def.cal_file,
           "rejected tilde value keeps its default");
    /* The value is 10 characters; reporting it as "too long" would send the
     * operator shortening the wrong string. */
    EXPECT(strstr(tmsg, "$HOME") != NULL,
           "tilde overflow reported as an expansion problem, not a long value");
    remove(path);

    /* Positive control under the SAME long $HOME: only what actually
     * overflows is refused, not every tilde value. */
    path = write_tmpconf(144, "[calibration]\nfile = \"~/c\"\n");
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0, "tilde value that still fits accepted");
    EXPECT(cfg.cal_file[0] != '~',       "tilde value that still fits expanded");
    remove(path);

    if (home_env) setenv("HOME", home_save, 1);
    else          unsetenv("HOME");

    /*
     * [mount] preset does not go through NEED_STR — the name is matched, not
     * stored — but it must still not read the buffer copy_str declined to
     * write.  Fatal, and reported as a length error rather than as an unknown
     * preset, which would send the operator hunting a name spelled right.
     */
    snprintf(body, sizeof body, "[mount]\npreset = \"%s\"\n",
             xstr(big, sizeof def.mount_preset + 8));
    path = write_tmpconf(145, body);
    config_defaults(&cfg);
    cap_begin();
    int prc = config_load(path, &cfg);
    const char *pmsg = cap_end();
    EXPECT(prc == CONFIG_ERR_PARSE, "over-long mount preset rejected");
    EXPECT(!cfg.mount_set,          "rejected preset does not set the mount");
    /*
     * The return code alone cannot tell the guard from its absence: without
     * it the uninitialised buffer matches no known name and the parse fails
     * anyway, as "unknown mount preset". Asserting the message is what makes
     * a regression here visible — verified by removing the guard and watching
     * this line, and only this line, fail.
     */
    EXPECT(strstr(pmsg, "too long") != NULL,
           "over-long preset reported as a length error, not an unknown name");
    remove(path);

    end_test(fb);
}

/* Bad boolean value: same continue-and-report contract as bad int. */
static void test_load_bad_bool(void)
{
    begin_test("test_load_bad_bool");
    int fb = g_fail;
    const char *path = write_tmpconf(2,
        "[nmea]\n"
        "enabled = yes\n"
        "rate_hz = 5\n");
    imud_config_t def;
    config_defaults(&def);
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE,                  "bad bool returns CONFIG_ERR_PARSE");
    EXPECT(cfg.nmea_enabled == def.nmea_enabled,     "bad key keeps its default");
    EXPECT(cfg.nmea_rate_hz == 5,                    "line after the bad one still applied");
    remove(path);
    end_test(fb);
}

/* Unknown section: load must succeed, keys inside it are skipped. */
static void test_load_unknown_section(void)
{
    begin_test("test_load_unknown_section");
    int fb = g_fail;
    const char *path = write_tmpconf(3,
        "[unknown_section]\n"
        "foo = bar\n"
        "[nmea]\n"
        "rate_hz = 5\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "unknown section doesn't abort load");
    EXPECT(cfg.nmea_rate_hz == 5, "subsequent known key still applied");
    remove(path);
    end_test(fb);
}

/* Unknown key in a known section: warning only, load continues. */
static void test_load_unknown_key(void)
{
    begin_test("test_load_unknown_key");
    int fb = g_fail;
    const char *path = write_tmpconf(4,
        "[imu]\n"
        "no_such_key = 99\n"
        "odr_hz = 416\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0,               "unknown key doesn't abort load");
    EXPECT(cfg.imu_odr_hz == 416, "subsequent known key still applied");
    remove(path);
    end_test(fb);
}

/* Inline comment after value is stripped; quoted string with # is preserved. */
static void test_load_inline_comment(void)
{
    begin_test("test_load_inline_comment");
    int fb = g_fail;
    const char *path = write_tmpconf(5,
        "[imu]\n"
        "odr_hz = 208  # rounds to nearest\n"
        "[nmea]\n"
        "dest_addr = \"10.0.0.255\"  # LAN broadcast\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0,                              "inline comment load ok");
    EXPECT(cfg.imu_odr_hz == 208,                "int after inline comment");
    EXPECT_STR(cfg.nmea_dest_addr, "10.0.0.255","quoted string before comment");
    remove(path);
    end_test(fb);
}

/* Tilde in a loaded string value must be expanded to $HOME. */
static void test_load_tilde_expansion(void)
{
    begin_test("test_load_tilde_expansion");
    int fb = g_fail;
    const char *path = write_tmpconf(6,
        "[calibration]\n"
        "file = \"~/.config/imud/cal.json\"\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0,                 "tilde conf loads ok");
    EXPECT(cfg.cal_file[0] != '~',  "tilde expanded in loaded value");
    EXPECT(strstr(cfg.cal_file, "/.config/imud/cal.json") != NULL,
           "expanded path has correct suffix");
    remove(path);
    end_test(fb);
}

/* config_load() must override only the specified keys, leave others at
 * their defaults.  (Verifies the caller must call config_defaults first.) */
static void test_load_partial_override(void)
{
    begin_test("test_load_partial_override");
    int fb = g_fail;
    const char *path = write_tmpconf(7,
        "[imu]\n"
        "odr_hz = 104\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    config_load(path, &cfg);
    EXPECT(cfg.imu_odr_hz  == 104,    "overridden key updated");
    EXPECT(cfg.imu_accel_g == 8,      "untouched key stays at default");
    EXPECT(cfg.nmea_rate_hz == 10,    "different section stays at default");
    remove(path);
    end_test(fb);
}

/* [position] defaults: all zero / disabled, wmm_file = "" (auto-resolve). */
static void test_defaults_position(void)
{
    begin_test("test_defaults_position");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);

    EXPECT_NEAR_D(cfg.pos_declination_deg, 0.0, 1e-9, "pos_declination_deg default 0");
    EXPECT(!cfg.pos_declination_valid,                 "pos_declination_valid default false");
    EXPECT_NEAR_D(cfg.pos_lat_deg,         0.0, 1e-9, "pos_lat_deg default 0");
    EXPECT_NEAR_D(cfg.pos_lon_deg,         0.0, 1e-9, "pos_lon_deg default 0");
    EXPECT_STR(cfg.pos_wmm_file, "", "pos_wmm_file default is the auto sentinel");
    EXPECT_NEAR_D(cfg.pos_fix_max_age_h,  24.0, 1e-5, "pos_fix_max_age_h default 24 h");

    /* config_load resolves the auto sentinel: the /etc override when that
     * file exists on this machine, else the /usr/share package data path. */
    config_load("/tmp/imud_no_such_file_xyz.conf", &cfg);
    EXPECT(strcmp(cfg.pos_wmm_file, "/etc/imud/WMM.COF") == 0 ||
           strcmp(cfg.pos_wmm_file, "/usr/share/imud/WMM.COF") == 0,
           "auto wmm_file resolves to /etc override or /usr/share data");
    end_test(fb);
}

/* [fusion] marine keys: mag_yaw_only and heave_tau_s load and override. */
static void test_fusion_marine_keys(void)
{
    begin_test("test_fusion_marine_keys");
    int fb = g_fail;

    const char *path = write_tmpconf(16,
        "[fusion]\n"
        "mag_yaw_only = false\n"
        "heave_tau_s  = 8.0\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0,        "fusion marine keys load");
    EXPECT(cfg.mag_yaw_only == false,           "mag_yaw_only=false loaded");
    EXPECT_NEAR_D(cfg.heave_tau_s, 8.0, 1e-5,   "heave_tau_s loaded");
    remove(path);
    end_test(fb);
}

/*
 * [fusion] Gauss–Markov wave-state keys (ROADMAP §10.5).
 *
 * These deliberately use NEED_DBL, not NEED_POS_DBL: unlike the noise
 * densities above, 0 is a documented value here — it disables the state and
 * returns the pre-1.7 6-state filter, which is a legitimate thing to ask for.
 * The name check matters too: mekf_wave_accel_tau_s sits one prefix away from
 * the unrelated sea-state wave_tau_s, and a parser that confused them would
 * silently retune the filter when someone set a reporting window.
 */
/*
 * The magnetic dip-reference uncertainty and the alignment window. Both take 0
 * as a meaningful value — 0 dip sigma means "the reference is exact" (a WMM
 * install), and 0 align window falls back to the minimum — so both use
 * NEED_DBL and must stay out of the positivity-fatal set above.
 */
static void test_fusion_dip_and_align_keys(void)
{
    begin_test("test_fusion_dip_and_align_keys");
    int fb = g_fail;

    const char *path = write_tmpconf(19,
        "[fusion]\n"
        "mekf_mag_dip_sigma_deg = 2.5\n"
        "[calibration]\n"
        "align_window_sec = 30.0\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0,                 "dip/align keys load");
    EXPECT_NEAR_D(cfg.mekf_mag_dip_sigma_deg, 2.5, 1e-9, "dip sigma loaded");
    EXPECT_NEAR_D(cfg.align_window_sec, 30.0, 1e-9,      "align window loaded");
    remove(path);

    const char *zero = write_tmpconf(20,
        "[fusion]\n"
        "mekf_mag_dip_sigma_deg = 0\n"
        "[calibration]\n"
        "align_window_sec = 0\n");
    config_defaults(&cfg);
    EXPECT(config_load(zero, &cfg) == 0,          "zero dip/align accepted");
    EXPECT(cfg.mekf_mag_dip_sigma_deg == 0.0,     "zero dip sigma applied");
    EXPECT(cfg.align_window_sec == 0.0,           "zero align window applied");
    remove(zero);

    end_test(fb);
}

static void test_fusion_wave_state_keys(void)
{
    begin_test("test_fusion_wave_state_keys");
    int fb = g_fail;

    const char *path = write_tmpconf(17,
        "[fusion]\n"
        "mekf_wave_accel       = 1.25\n"
        "mekf_wave_accel_tau_s = 0.75\n"
        "wave_tau_s            = 90.0\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0,               "wave-state keys load");
    EXPECT_NEAR_D(cfg.mekf_wave_accel, 1.25, 1e-9,     "mekf_wave_accel loaded");
    EXPECT_NEAR_D(cfg.mekf_wave_accel_tau_s, 0.75, 1e-9,
                  "mekf_wave_accel_tau_s loaded");
    EXPECT_NEAR_D(cfg.wave_tau_s, 90.0, 1e-5,
                  "sea-state wave_tau_s is a different key and is untouched");
    remove(path);

    /* 0 must be ACCEPTED (it means "off"), not rejected as a bad tuning. */
    const char *off = write_tmpconf(18,
        "[fusion]\n"
        "mekf_wave_accel       = 0\n"
        "mekf_wave_accel_tau_s = 0\n");
    config_defaults(&cfg);
    EXPECT(config_load(off, &cfg) == 0,                "zero wave-state accepted");
    EXPECT(cfg.mekf_wave_accel == 0.0,                 "zero sigma applied");
    EXPECT(cfg.mekf_wave_accel_tau_s == 0.0,           "zero tau applied");
    remove(off);

    end_test(fb);
}

/* [stream] defaults and key loading. */
static void test_stream_section(void)
{
    begin_test("test_stream_section");
    int fb = g_fail;

    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(cfg.stream_enabled,                         "stream enabled by default (1.6: the one stock output)");
    EXPECT(cfg.stream_rate_hz == 100,                  "stream rate_hz default 100");
    EXPECT_STR(cfg.stream_socket, "/run/imud/imud-stream.sock",
               "stream socket default path");
    EXPECT(cfg.stream_tcp_enabled == false,            "stream_tcp_enabled default off");
    EXPECT_STR(cfg.stream_tcp_bind_addr, "0.0.0.0",    "stream_tcp_bind_addr default");
    EXPECT(cfg.stream_tcp_port == 10112,               "stream_tcp_port default 10112");

    const char *path = write_tmpconf(14,
        "[stream]\n"
        "enabled = false\n"
        "socket  = \"/tmp/alt-stream.sock\"\n"
        "rate_hz = 50\n");
    EXPECT(config_load(path, &cfg) == 0,               "stream section loads");
    EXPECT(!cfg.stream_enabled,                        "stream enabled=false loaded");
    EXPECT(cfg.stream_rate_hz == 50,                   "stream rate_hz loaded");
    EXPECT_STR(cfg.stream_socket, "/tmp/alt-stream.sock", "stream socket loaded");
    remove(path);
    end_test(fb);
}

/* tcp_* keys ([nmea]/[stream]/[imud-signalk]/[imud-mavlink]): defaults are
 * off/0.0.0.0/per-output port; each section's block loads and overrides. */
static void test_tcp_output_keys(void)
{
    begin_test("test_tcp_output_keys");
    int fb = g_fail;

    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(!cfg.sk_tcp_enabled,                        "sk_tcp_enabled default off");
    EXPECT_STR(cfg.sk_tcp_bind_addr, "0.0.0.0",        "sk_tcp_bind_addr default");
    EXPECT(cfg.sk_tcp_port == 10113,                   "sk_tcp_port default 10113");
    EXPECT(!cfg.mav_tcp_enabled,                       "mav_tcp_enabled default off");
    EXPECT_STR(cfg.mav_tcp_bind_addr, "0.0.0.0",       "mav_tcp_bind_addr default");
    EXPECT(cfg.mav_tcp_port == 5760,                   "mav_tcp_port default 5760");

    const char *path = write_tmpconf(23,
        "[nmea]\n"
        "tcp_enabled   = true\n"
        "tcp_bind_addr = \"127.0.0.1\"\n"
        "tcp_port      = 20110\n"
        "[stream]\n"
        "tcp_enabled   = true\n"
        "tcp_bind_addr = \"192.168.1.5\"\n"
        "tcp_port      = 20112\n"
        "[imud-signalk]\n"
        "tcp_enabled   = true\n"
        "tcp_bind_addr = \"127.0.0.1\"\n"
        "tcp_port      = 20113\n"
        "[imud-mavlink]\n"
        "tcp_enabled   = true\n"
        "tcp_bind_addr = \"0.0.0.0\"\n"
        "tcp_port      = 5761\n");
    EXPECT(config_load(path, &cfg) == 0,               "tcp_* blocks load");
    EXPECT(cfg.nmea_tcp_enabled,                       "nmea tcp_enabled loaded");
    EXPECT_STR(cfg.nmea_tcp_bind_addr, "127.0.0.1",    "nmea tcp_bind_addr loaded");
    EXPECT(cfg.nmea_tcp_port == 20110,                 "nmea tcp_port loaded");
    EXPECT(cfg.stream_tcp_enabled,                     "stream tcp_enabled loaded");
    EXPECT_STR(cfg.stream_tcp_bind_addr, "192.168.1.5","stream tcp_bind_addr loaded");
    EXPECT(cfg.stream_tcp_port == 20112,               "stream tcp_port loaded");
    EXPECT(cfg.sk_tcp_enabled,                         "sk tcp_enabled loaded");
    EXPECT_STR(cfg.sk_tcp_bind_addr, "127.0.0.1",      "sk tcp_bind_addr loaded");
    EXPECT(cfg.sk_tcp_port == 20113,                   "sk tcp_port loaded");
    EXPECT(cfg.mav_tcp_enabled,                        "mav tcp_enabled loaded");
    EXPECT_STR(cfg.mav_tcp_bind_addr, "0.0.0.0",       "mav tcp_bind_addr loaded");
    EXPECT(cfg.mav_tcp_port == 5761,                   "mav tcp_port loaded");
    remove(path);
    end_test(fb);
}

/* [imud-signalk] bridge keys: the own-config-file convention. The bridge reads
 * its own socket + publish_heave from this section (never imud.conf). */
/* [capture] keys: black box disabled by default; all keys load. */
static void test_capture_section(void)
{
    begin_test("test_capture_section");
    int fb = g_fail;

    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(!cfg.capture_enabled,                       "capture disabled by default");
    EXPECT_STR(cfg.capture_dir, "/var/lib/imud",       "capture dir default");
    EXPECT(cfg.capture_max_mb == 256,                  "capture max_mb default 256");
    EXPECT(cfg.capture_max_files == 8,                 "capture max_files default 8");
    EXPECT(cfg.capture_flush_s == 5,                   "capture flush_s default 5");

    const char *path = write_tmpconf(22,
        "[capture]\n"
        "enabled   = true\n"
        "dir       = \"/tmp/blackbox\"\n"
        "max_mb    = 64\n"
        "max_files = 4\n"
        "flush_s   = 1\n");
    EXPECT(config_load(path, &cfg) == 0,               "capture section loads");
    EXPECT(cfg.capture_enabled,                        "capture enabled loaded");
    EXPECT_STR(cfg.capture_dir, "/tmp/blackbox",       "capture dir loaded");
    EXPECT(cfg.capture_max_mb == 64,                   "capture max_mb loaded");
    EXPECT(cfg.capture_max_files == 4,                 "capture max_files loaded");
    EXPECT(cfg.capture_flush_s == 1,                   "capture flush_s loaded");
    remove(path);
    end_test(fb);
}

/* [device] sim playback keys load and override the synthesis defaults. */
static void test_sim_playback_keys(void)
{
    begin_test("test_sim_playback_keys");
    int fb = g_fail;

    imud_config_t cfg;
    config_defaults(&cfg);

    const char *path = write_tmpconf(21,
        "[device]\n"
        "sim_file  = \"/var/lib/imud/imud-20260711-120000.imucap\"\n"
        "sim_loop  = true\n"
        "sim_speed = 0.0\n");
    EXPECT(config_load(path, &cfg) == 0,               "sim keys load");
    EXPECT_STR(cfg.sim_file, "/var/lib/imud/imud-20260711-120000.imucap",
               "sim_file loaded");
    EXPECT(cfg.sim_loop == true,                       "sim_loop loaded");
    EXPECT_NEAR_D(cfg.sim_speed, 0.0, 1e-9,            "sim_speed 0 (fast) loaded");
    remove(path);
    end_test(fb);
}

static void test_signalk_section(void)
{
    begin_test("test_signalk_section");
    int fb = g_fail;

    /* The real bridge config file parses with the expected defaults. */
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load("config/imud-signalk.conf", &cfg) == 0,
           "imud-signalk.conf loads");
    EXPECT(cfg.sk_enabled,                          "sk enabled=true in stock conf");
    EXPECT(!cfg.sk_udp_enabled,                     "sk udp_enabled off in stock conf");
    EXPECT(!cfg.sk_tcp_enabled,                     "sk tcp_enabled off in stock conf");
    EXPECT(cfg.sk_dest_port == 10113,               "sk dest_port 10113");
    EXPECT(cfg.sk_rate_hz == 10,                    "sk rate_hz 10");
    EXPECT_STR(cfg.sk_source_label, "imud",         "sk source_label imud");
    EXPECT(cfg.publish_heave == true,               "publish_heave true in conf");
    EXPECT_STR(cfg.stream_socket, "/run/imud/imud-stream.sock",
               "socket key maps to stream_socket");

    /* socket override + publish_heave=false via the [imud-signalk] section. */
    const char *path = write_tmpconf(17,
        "[imud-signalk]\n"
        "socket        = \"/tmp/sk-stream.sock\"\n"
        "publish_heave = false\n"
        "rate_hz       = 4\n");
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0,            "signalk override loads");
    EXPECT_STR(cfg.stream_socket, "/tmp/sk-stream.sock", "socket override applied");
    EXPECT(cfg.publish_heave == false,              "publish_heave=false loaded");
    EXPECT(cfg.sk_rate_hz == 4,                      "sk rate_hz override loaded");
    remove(path);
    end_test(fb);
}

/* declination_deg sets the validity flag: non-zero → valid, 0.0 → disabled. */
static void test_declination_valid_flag(void)
{
    begin_test("test_declination_valid_flag");
    int fb = g_fail;

    const char *path = write_tmpconf(12,
        "[position]\n"
        "declination_deg = 13.2\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0,               "non-zero declination loads");
    EXPECT(cfg.pos_declination_valid,                  "non-zero declination → valid");
    EXPECT_NEAR_D(cfg.pos_declination_deg, 13.2, 1e-5, "declination value applied");
    remove(path);

    path = write_tmpconf(13,
        "[position]\n"
        "declination_deg = 0.0\n");
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0,   "zero declination loads");
    EXPECT(!cfg.pos_declination_valid,     "explicit 0.0 stays disabled (config semantics)");
    remove(path);

    end_test(fb);
}

/* fix_max_age_h can be loaded and overridden (0 = never expire). */
static void test_fix_max_age_h_load(void)
{
    begin_test("test_fix_max_age_h_load");
    int fb = g_fail;
    const char *path = write_tmpconf(9,
        "[position]\n"
        "fix_max_age_h = 48.0\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0,                                      "fix_max_age_h loads ok");
    EXPECT_NEAR_D(cfg.pos_fix_max_age_h, 48.0, 1e-5,    "fix_max_age_h = 48.0");
    remove(path);
    end_test(fb);
}

/* fix_max_age_h = 0 means never expire. */
static void test_fix_max_age_h_zero(void)
{
    begin_test("test_fix_max_age_h_zero");
    int fb = g_fail;
    const char *path = write_tmpconf(10,
        "[position]\n"
        "fix_max_age_h = 0.0\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0,                                  "fix_max_age_h = 0 loads ok");
    EXPECT_NEAR_D(cfg.pos_fix_max_age_h, 0.0, 1e-9, "fix_max_age_h = 0 (never expire)");
    remove(path);
    end_test(fb);
}

/* [position] keys are parsed: lat/lon as double, wmm_file as string. */
static void test_position_keys_load(void)
{
    begin_test("test_position_keys_load");
    int fb = g_fail;
    const char *path = write_tmpconf(8,
        "[position]\n"
        "lat_deg  = 47.6062\n"
        "lon_deg  = -122.3321\n"
        "wmm_file = \"data/WMM.COF\"\n");
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0,                               "position keys load ok");
    EXPECT_NEAR_D(cfg.pos_lat_deg,  47.6062,  1e-9, "lat_deg parsed as double");
    EXPECT_NEAR_D(cfg.pos_lon_deg, -122.3321, 1e-9, "lon_deg parsed as double");
    EXPECT_STR(cfg.pos_wmm_file, "data/WMM.COF",    "wmm_file string parsed");
    remove(path);
    end_test(fb);
}

/* config/sim.conf must load cleanly and select the sim driver. */
static void test_sim_conf_loads(void)
{
    begin_test("test_sim_conf_loads");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);

    int rc = config_load("config/sim.conf", &cfg);
    EXPECT(rc == 0,                        "sim.conf loads without error");
    EXPECT_STR(cfg.imu_driver, "sim",      "imu driver = sim");
    EXPECT_STR(cfg.mag_driver, "sim",      "mag driver = sim");
    EXPECT(cfg.imu_int_gpio == 0,          "imu int_gpio = 0 (timer fallback)");
    EXPECT(cfg.mag_int_gpio == 0,          "mag int_gpio = 0 (timer fallback)");
    EXPECT_NEAR_D(cfg.gyro_bias_sec, 0.0,  1e-9, "gyro_bias_sec = 0 (skip estimation)");
    EXPECT_STR(cfg.pos_wmm_file, "data/WMM.COF",   "sim wmm_file is dev path");
    EXPECT_NEAR_D(cfg.pos_lat_deg, 0.0,    1e-9, "sim pos_lat_deg = 0 (WMM disabled)");
    EXPECT_NEAR_D(cfg.pos_lon_deg, 0.0,    1e-9, "sim pos_lon_deg = 0 (WMM disabled)");
    end_test(fb);
}

/* Per-output enables (1.6+): every bridge's daemon `enabled` is separate from
 * its output enables, which default off; influxdb keeps a back-compat mapping
 * for the deprecated `transport` selector. */
static void test_bridge_output_enables(void)
{
    begin_test("test_bridge_output_enables");
    int fb = g_fail;

    imud_config_t cfg;
    config_defaults(&cfg);
    EXPECT(!cfg.sk_udp_enabled,        "sk_udp_enabled default off");
    EXPECT(!cfg.mqtt_broker_enabled,   "mqtt_broker_enabled default off");
    EXPECT(!cfg.influx_udp_enabled,    "influx_udp_enabled default off");
    EXPECT(!cfg.influx_http_enabled,   "influx_http_enabled default off");
    EXPECT(!cfg.prom_http_enabled,     "prom_http_enabled default off");
    EXPECT_STR(cfg.influx_transport, "", "influx_transport default empty (unset)");

    const char *path = write_tmpconf(31,
        "[imud-signalk]\n"
        "udp_enabled    = true\n"
        "[imud-mqtt]\n"
        "broker_enabled = true\n"
        "[imud-influxdb]\n"
        "udp_enabled    = true\n"
        "http_enabled   = true\n"
        "[imud-prometheus]\n"
        "http_enabled   = true\n");
    config_defaults(&cfg);
    EXPECT(config_load(path, &cfg) == 0, "output-enable block loads");
    EXPECT(cfg.sk_udp_enabled,        "sk udp_enabled loaded");
    EXPECT(cfg.mqtt_broker_enabled,   "mqtt broker_enabled loaded");
    EXPECT(cfg.influx_udp_enabled,    "influx udp_enabled loaded");
    EXPECT(cfg.influx_http_enabled,   "influx http_enabled loaded (dual output)");
    EXPECT(cfg.prom_http_enabled,     "prom http_enabled loaded");
    remove(path);

    /* Back-compat: legacy transport maps only when no new enable is set. */
    config_defaults(&cfg);
    snprintf(cfg.influx_transport, sizeof cfg.influx_transport, "http");
    EXPECT(config_apply_influx_transport_compat(&cfg), "legacy transport mapped");
    EXPECT(cfg.influx_http_enabled,   "transport=http → http_enabled");
    EXPECT(!cfg.influx_udp_enabled,   "transport=http leaves udp off");

    config_defaults(&cfg);
    snprintf(cfg.influx_transport, sizeof cfg.influx_transport, "udp");
    EXPECT(config_apply_influx_transport_compat(&cfg), "legacy transport=udp mapped");
    EXPECT(cfg.influx_udp_enabled,    "transport=udp → udp_enabled");

    /* An explicit new enable wins; the legacy key is ignored (no remap). */
    config_defaults(&cfg);
    cfg.influx_udp_enabled = true;
    snprintf(cfg.influx_transport, sizeof cfg.influx_transport, "http");
    EXPECT(!config_apply_influx_transport_compat(&cfg), "new enable set → no legacy remap");
    EXPECT(!cfg.influx_http_enabled,  "http stays off when udp explicitly set");

    end_test(fb);
}

/* ── config_apply_hot: the SIGHUP [hot] / [restart] partition ─────────────── */

/*
 * config_apply_hot() is the field list SIGHUP applies to a running daemon.  A
 * field missing from it is invisible in production: the key parses, the reload
 * logs success, and nothing changes.  main.c carries a comment recording that
 * this already happened once, to pos_mref_*.
 *
 * The test is a partition, checked from both sides:
 *
 *   1. Every [hot] field of dst must equal src after the call — catches a
 *      forgotten copy.  The list below is written out by hand precisely so it
 *      is a SECOND, independent statement of the partition; if it and
 *      config.c's list drift apart, step 2 fails.
 *
 *   2. Nothing else may change.  `expect` is the pre-call dst with exactly the
 *      hot fields overwritten from src, and the whole struct is memcmp'd
 *      against it — catches a [restart] field being copied, which would apply
 *      a key to a running daemon that cannot honour it.
 *
 * For step 2 to have teeth, src must differ from dst in EVERY field, so a
 * stray copy of any one of them shows up.  fill_distinct() does that; it was
 * generated from the struct in include/config.h and must gain a line when a
 * field is added (AGENTS.md's add-a-config-key checklist says so).
 */

/*
 * A value that does not fit is a SILENT collision, not a truncation warning:
 * four char[8] fields (highrate_coord_frame, mqtt_units, influx_transport,
 * influx_units) all used to receive "distinct-NN" and all ended up holding
 * "distinc". The whole point of this helper is one distinct value per field —
 * the two-sided memcmp partition test can pass while the code under test has
 * swapped two fields that hold the same bytes. So assert the fit at the write.
 */
#define SET_STR(field, value) do { \
    if (strlen(value) >= sizeof(field)) { \
        fprintf(stderr, "  FAIL  %s:%d  fill_distinct: '%s' truncates in %zu bytes\n", \
                __FILE__, __LINE__, (value), sizeof(field)); \
        g_fail++; \
    } \
    snprintf((field), sizeof(field), "%s", (value)); \
} while (0)

static void fill_distinct(imud_config_t *c)
{
    config_defaults(c);
    SET_STR(c->i2c_bus, "distinct-1");
    SET_STR(c->gpio_chip, "distinct-2");
    SET_STR(c->sim_file, "distinct-3");
    c->sim_loop = !c->sim_loop;
    c->sim_speed = 5.5;
    c->capture_enabled = !c->capture_enabled;
    SET_STR(c->capture_dir, "distinct-7");
    c->capture_max_mb = 15;
    c->capture_max_files = 16;
    c->capture_flush_s = 17;
    SET_STR(c->imu_driver, "distinct-11");
    c->imu_addr = 19;
    c->imu_int_gpio = 20;
    c->imu_odr_hz = 21;
    c->imu_accel_g = 22;
    c->imu_gyro_dps = 23;
    c->imu_fifo_wm = 24;
    SET_STR(c->mag_driver, "distinct-18");
    c->mag_addr = 26;
    c->mag_int_gpio = 27;
    c->mag_odr_hz = 28;
    c->mag_set_period_s = 22.5;
    c->mag_yaw_only = !c->mag_yaw_only;
    c->heave_tau_s = 24.5;
    c->wave_tau_s = 25.5;
    c->mekf_gyro_noise = 26.5;
    c->mekf_gyro_bias = 27.5;
    c->mekf_accel_noise = 28.5;
    c->mekf_mag_noise = 29.5;
    c->mekf_wave_accel = 30.5;
    c->mekf_wave_accel_tau_s = 31.5;
    c->mekf_mag_dip_sigma_deg = 32.5;
    c->mag_reject_gauss = 33.5;
    c->accel_skip_thresh = 34.5;
    c->engine_vibration_g2 = 35.5;
    c->engine_accel_skip_thresh = 36.5;
    SET_STR(c->cal_file, "distinct-37");
    c->startup_settle_sec = 38.5;
    c->gyro_bias_sec = 39.5;
    c->align_window_sec = 40.5;
    c->nmea_enabled = !c->nmea_enabled;
    c->nmea_rate_hz = 49;
    SET_STR(c->nmea_dest_addr, "distinct-43");
    c->nmea_dest_port = 51;
    c->nmea_tcp_enabled = !c->nmea_tcp_enabled;
    SET_STR(c->nmea_tcp_bind_addr, "distinct-46");
    c->nmea_tcp_port = 54;
    c->highrate_enabled = !c->highrate_enabled;
    c->highrate_rate_hz = 56;
    SET_STR(c->highrate_dest_addr, "distinct-50");
    c->highrate_dest_port = 58;
    SET_STR(c->highrate_coord_frame, "d52");
    c->stream_enabled = !c->stream_enabled;
    SET_STR(c->stream_socket, "distinct-54");
    c->stream_rate_hz = 62;
    c->stream_tcp_enabled = !c->stream_tcp_enabled;
    SET_STR(c->stream_tcp_bind_addr, "distinct-57");
    c->stream_tcp_port = 65;
    c->sk_enabled = !c->sk_enabled;
    c->sk_udp_enabled = !c->sk_udp_enabled;
    SET_STR(c->sk_dest_addr, "distinct-61");
    c->sk_dest_port = 69;
    c->sk_rate_hz = 70;
    SET_STR(c->sk_source_label, "distinct-64");
    c->sk_tcp_enabled = !c->sk_tcp_enabled;
    SET_STR(c->sk_tcp_bind_addr, "distinct-66");
    c->sk_tcp_port = 74;
    c->publish_heave = !c->publish_heave;
    c->mqtt_enabled = !c->mqtt_enabled;
    c->mqtt_broker_enabled = !c->mqtt_broker_enabled;
    SET_STR(c->mqtt_broker_addr, "distinct-71");
    c->mqtt_broker_port = 79;
    SET_STR(c->mqtt_client_id, "distinct-73");
    SET_STR(c->mqtt_topic_prefix, "distinct-74");
    c->mqtt_rate_hz = 82;
    c->mqtt_qos = 83;
    c->mqtt_retain = !c->mqtt_retain;
    SET_STR(c->mqtt_units, "d78");
    SET_STR(c->mqtt_username, "distinct-79");
    SET_STR(c->mqtt_password, "distinct-80");
    c->mqtt_tls = !c->mqtt_tls;
    SET_STR(c->mqtt_tls_cafile, "distinct-82");
    c->mqtt_keepalive_s = 90;
    c->mqtt_ha_discovery = !c->mqtt_ha_discovery;
    SET_STR(c->mqtt_ha_prefix, "distinct-85");
    c->influx_enabled = !c->influx_enabled;
    SET_STR(c->influx_transport, "d87");
    c->influx_rate_hz = 95;
    SET_STR(c->influx_measurement, "distinct-89");
    SET_STR(c->influx_source_label, "distinct-90");
    SET_STR(c->influx_units, "d91");
    c->influx_udp_enabled = !c->influx_udp_enabled;
    SET_STR(c->influx_udp_addr, "distinct-93");
    c->influx_udp_port = 101;
    c->influx_http_enabled = !c->influx_http_enabled;
    SET_STR(c->influx_http_host, "distinct-96");
    c->influx_http_port = 104;
    SET_STR(c->influx_http_path, "distinct-98");
    SET_STR(c->influx_http_token, "distinct-99");
    c->prom_enabled = !c->prom_enabled;
    c->prom_http_enabled = !c->prom_http_enabled;
    SET_STR(c->prom_listen_addr, "distinct-102");
    c->prom_listen_port = 110;
    c->mav_enabled = !c->mav_enabled;
    c->mav_version = 112;
    c->mav_system_id = 113;
    c->mav_component_id = 114;
    c->mav_rate_hz = 115;
    c->mav_send_attitude = !c->mav_send_attitude;
    c->mav_send_attitude_quaternion = !c->mav_send_attitude_quaternion;
    c->mav_udp_enabled = !c->mav_udp_enabled;
    SET_STR(c->mav_udp_addr, "distinct-112");
    c->mav_udp_port = 120;
    c->mav_serial_enabled = !c->mav_serial_enabled;
    SET_STR(c->mav_serial_device, "distinct-115");
    c->mav_serial_baud = 123;
    c->mav_tcp_enabled = !c->mav_tcp_enabled;
    SET_STR(c->mav_tcp_bind_addr, "distinct-118");
    c->mav_tcp_port = 126;
    SET_STR(c->log_level, "distinct-120");
    SET_STR(c->log_file, "distinct-121");
    c->log_stats_hz = 129;
    c->pos_declination_deg = 123.5;
    c->pos_declination_valid = !c->pos_declination_valid;
    c->pos_mref_h_gauss = 125.5;
    c->pos_mref_z_gauss = 126.5;
    c->pos_mref_valid = !c->pos_mref_valid;
    c->pos_speed_mps = 128.5;
    c->pos_speed_valid = !c->pos_speed_valid;
    c->pos_lat_deg = 130.5;
    c->pos_lon_deg = 131.5;
    SET_STR(c->pos_wmm_file, "distinct-132");
    c->pos_gpsd_enabled = !c->pos_gpsd_enabled;
    SET_STR(c->pos_gpsd_host, "distinct-134");
    c->pos_gpsd_port = 142;
    c->pos_signalk_enabled = !c->pos_signalk_enabled;
    SET_STR(c->pos_signalk_host, "distinct-137");
    c->pos_signalk_port = 145;
    SET_STR(c->pos_signalk_path, "distinct-139");
    c->pos_fix_max_age_h = 140.5;
    c->mount_set = !c->mount_set;
    for (int i = 0; i < 3; i++) c->mount_euler_deg[i] = 142.5 + i;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) c->mount_rot[i][j] = 143.5 + i * 3 + j;
    SET_STR(c->mount_preset, "distinct-144");
}

/*
 * [mount] preset — the eight quarter-turn shortcuts and their aliases.
 *
 * Untested until audit D1 sent me to look at it: the key is parsed, and it
 * is the one mount key that can be set with a typo. An unrecognised name is
 * deliberately fatal, because starting with a silently wrong mount rotation
 * biases every sample for the life of the run.
 */
static void test_mount_preset(void)
{
    begin_test("test_mount_preset");
    int fb = g_fail;

    static const struct { const char *name; double r, p, y; } cases[] = {
        { "identity",      0,  0,   0 }, { "board_forward", 0,  0,   0 },
        { "yaw_90",        0,  0,  90 }, { "rot_z_90",      0,  0,  90 },
        { "yaw_180",       0,  0, 180 }, { "rot_z_180",     0,  0, 180 },
        { "yaw_270",       0,  0, 270 }, { "rot_z_270",     0,  0, 270 },
        { "roll_90",      90,  0,   0 }, { "rot_x_90",     90,  0,   0 },
        { "roll_270",    270,  0,   0 }, { "rot_x_270",   270,  0,   0 },
        { "pitch_90",      0, 90,   0 }, { "rot_y_90",      0, 90,   0 },
        { "pitch_270",     0,270,   0 }, { "rot_y_270",     0,270,   0 },
    };

    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        char body[128];
        snprintf(body, sizeof body, "[mount]\npreset = \"%s\"\n", cases[i].name);
        const char *path = write_tmpconf(91, body);
        imud_config_t cfg;
        config_defaults(&cfg);
        EXPECT(config_load(path, &cfg) == 0, cases[i].name);
        EXPECT(cfg.mount_set, "preset sets mount_set");
        EXPECT_NEAR_D(cfg.mount_euler_deg[0], cases[i].r, 1e-9, "roll");
        EXPECT_NEAR_D(cfg.mount_euler_deg[1], cases[i].p, 1e-9, "pitch");
        EXPECT_NEAR_D(cfg.mount_euler_deg[2], cases[i].y, 1e-9, "yaw");
        EXPECT(strcmp(cfg.mount_preset, cases[i].name) == 0,
               "the name as written is recorded");
        remove(path);
    }

    /* Case-insensitive, per strcasecmp in apply_kv. */
    {
        const char *path = write_tmpconf(91, "[mount]\npreset = \"YAW_180\"\n");
        imud_config_t cfg;
        config_defaults(&cfg);
        EXPECT(config_load(path, &cfg) == 0, "preset names are case-insensitive");
        EXPECT_NEAR_D(cfg.mount_euler_deg[2], 180.0, 1e-9, "YAW_180 → yaw 180");
        remove(path);
    }

    /* Last mount key wins, in both directions. */
    {
        const char *path = write_tmpconf(91,
            "[mount]\nrotation_euler_deg = [0.0, 0.0, 90.0]\n"
            "preset = \"yaw_180\"\n");
        imud_config_t cfg;
        config_defaults(&cfg);
        EXPECT(config_load(path, &cfg) == 0, "euler then preset loads");
        EXPECT_NEAR_D(cfg.mount_euler_deg[2], 180.0, 1e-9, "preset wins when last");
        remove(path);

        path = write_tmpconf(91,
            "[mount]\npreset = \"yaw_180\"\n"
            "rotation_euler_deg = [0.0, 0.0, 90.0]\n");
        config_defaults(&cfg);
        EXPECT(config_load(path, &cfg) == 0, "preset then euler loads");
        EXPECT_NEAR_D(cfg.mount_euler_deg[2], 90.0, 1e-9, "euler wins when last");
        remove(path);
    }

    /* A typo must stop the daemon, not silently keep the default. */
    {
        const char *path = write_tmpconf(91, "[mount]\npreset = \"yaw_45\"\n");
        imud_config_t cfg;
        config_defaults(&cfg);
        EXPECT(config_load(path, &cfg) != 0, "unknown preset name is fatal");
        remove(path);

        path = write_tmpconf(91, "[mount]\npreset = \"\"\n");
        config_defaults(&cfg);
        EXPECT(config_load(path, &cfg) != 0, "empty preset name is fatal");
        remove(path);
    }

    end_test(fb);
}

/* The hot set, stated independently of src/config.c. */
#define HOT_FIELDS(X) \
    X(nmea_rate_hz) X(highrate_rate_hz) X(stream_rate_hz) X(log_stats_hz) \
    X(log_level) \
    X(mag_yaw_only) X(heave_tau_s) X(wave_tau_s) \
    X(mekf_gyro_noise) X(mekf_gyro_bias) X(mekf_accel_noise) X(mekf_mag_noise) \
    X(mekf_wave_accel) X(mekf_wave_accel_tau_s) X(mekf_mag_dip_sigma_deg) \
    X(mag_reject_gauss) X(accel_skip_thresh) \
    X(engine_vibration_g2) X(engine_accel_skip_thresh) \
    X(pos_wmm_file) X(pos_declination_deg) X(pos_declination_valid) \
    X(pos_mref_h_gauss) X(pos_mref_z_gauss) X(pos_mref_valid)

static void test_apply_hot_partition(void)
{
    begin_test("test_apply_hot_partition");
    int fb = g_fail;

    imud_config_t dst, src, expect;

    config_defaults(&dst);
    fill_distinct(&src);

    /* fill_distinct must actually differ from the defaults, or the whole test
     * passes vacuously. */
    EXPECT(memcmp(&dst, &src, sizeof dst) != 0, "fill_distinct differs from defaults");

    expect = dst;
    config_apply_hot(&dst, &src);

    /* 1. every hot field arrived */
#define CHECK_COPIED(f) \
    EXPECT(memcmp(&dst.f, &src.f, sizeof dst.f) == 0, #f " is [hot], must be copied");
    HOT_FIELDS(CHECK_COPIED)
#undef CHECK_COPIED

    /* 2. and nothing else did */
#define BUILD_EXPECT(f) memcpy(&expect.f, &src.f, sizeof expect.f);
    HOT_FIELDS(BUILD_EXPECT)
#undef BUILD_EXPECT
    EXPECT(memcmp(&dst, &expect, sizeof dst) == 0,
           "config_apply_hot copied exactly the [hot] set and nothing else");

    /* A few [restart] fields named explicitly, so a failure above reads as
     * something more useful than one memcmp.  These are the keys where a
     * silent hot-apply would be worst: the sensor is already open at the old
     * ODR, and the sockets are already bound to the old ports. */
    EXPECT(dst.imu_odr_hz     != src.imu_odr_hz,     "imu_odr_hz stays [restart]");
    EXPECT(dst.mag_odr_hz     != src.mag_odr_hz,     "mag_odr_hz stays [restart]");
    EXPECT(dst.nmea_dest_port != src.nmea_dest_port, "nmea_dest_port stays [restart]");
    EXPECT(dst.stream_enabled != src.stream_enabled, "stream_enabled stays [restart]");
    EXPECT(strcmp(dst.log_file, src.log_file) != 0,  "log_file stays [restart]");
    EXPECT(strcmp(dst.i2c_bus, src.i2c_bus)   != 0,  "i2c_bus stays [restart]");
    EXPECT(dst.capture_enabled != src.capture_enabled,
           "capture_enabled stays [restart]");
    EXPECT(dst.mount_set != src.mount_set, "mount_set stays [restart]");

    /* pos_lat_deg / pos_lon_deg are deliberately NOT in the hot set: main.c
     * runs the WMM recomputation over the freshly parsed config and copies
     * only the derived results, so the live copy has no reader.  Pinned here
     * because it looks like an omission and is not. */
    EXPECT(dst.pos_lat_deg != src.pos_lat_deg,
           "pos_lat_deg not copied (only its WMM result is)");
    EXPECT(dst.pos_lon_deg != src.pos_lon_deg,
           "pos_lon_deg not copied (only its WMM result is)");

    /* Applying the same source twice must land in the same place. */
    imud_config_t once = dst;
    config_apply_hot(&dst, &src);
    EXPECT(memcmp(&dst, &once, sizeof dst) == 0, "config_apply_hot is idempotent");

    end_test(fb);
}

/* ── main ───────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud config tests ===");

    test_defaults_values();
    test_defaults_cal_file();
    test_defaults_position();
    test_fusion_marine_keys();
    test_fusion_wave_state_keys();
    test_fusion_dip_and_align_keys();
    test_stream_section();
    test_tcp_output_keys();
    test_sim_playback_keys();
    test_capture_section();
    test_signalk_section();
    test_bridge_output_enables();
    test_declination_valid_flag();
    test_fix_max_age_h_load();
    test_fix_max_age_h_zero();
    test_load_real_conf();
    test_load_missing_file();
    test_load_bad_int();
    test_load_noise_density_must_be_positive();
    test_load_rates_must_be_positive();
    test_load_rejects_non_finite();
    test_load_rejects_too_long_string();
    test_load_bad_bool();
    test_load_unknown_section();
    test_load_unknown_key();
    test_load_inline_comment();
    test_load_tilde_expansion();
    test_load_partial_override();
    test_position_keys_load();
    test_sim_conf_loads();
    test_mount_preset();
    test_apply_hot_partition();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
