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

/* Missing file must return -1. */
static void test_load_missing_file(void)
{
    begin_test("test_load_missing_file");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load("/tmp/imud_no_such_file_xyz.conf", &cfg);
    EXPECT(rc == -1, "missing file returns -1");
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

/* ── main ───────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud config tests ===");

    test_defaults_values();
    test_defaults_cal_file();
    test_defaults_position();
    test_fusion_marine_keys();
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
    test_load_bad_bool();
    test_load_unknown_section();
    test_load_unknown_key();
    test_load_inline_comment();
    test_load_tilde_expansion();
    test_load_partial_override();
    test_position_keys_load();
    test_sim_conf_loads();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
