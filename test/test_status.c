/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_status.c — the imud-status report text (src/status_fmt.c)
 *
 * Two things are worth testing here, and they are not the same thing:
 *
 *   1. The conditional lines.  Roughly half the report is gated —
 *      declination valid or not, heave and sea state on the [fusion] time
 *      constants, capture running or stopped, and a four-way NMEA branch on
 *      the UDP and TCP enables.  These are the lines an operator reads to
 *      decide whether the daemon is doing what they configured, so a branch
 *      that says "disabled" when it is not is a real defect.
 *
 *   2. The truncation bound.  The report is built with a WS() macro that
 *      tracks remaining space by hand, and it lived in main.c — in no test
 *      binary — since it was written.  Every call here is made at every
 *      buffer size from 0 upward, asserting the result is NUL-terminated,
 *      never longer than sz - 1, and that the returned length matches
 *      strlen().  A caller write()s exactly that many bytes.
 *
 * Portable — builds and runs on the macOS dev box.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "status_fmt.h"

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

static bool has(const char *hay, const char *needle)
{
    return strstr(hay, needle) != NULL;
}

/* A configured, running daemon with nothing exotic enabled. */
static void baseline(imud_config_t *cfg, status_input_t *in)
{
    config_defaults(cfg);
    memset(in, 0, sizeof *in);
    in->cfg      = cfg;
    in->uptime_s = 3661;                    /* 01:01:01 */
    in->state.heading_deg = 123.4f;
    in->stats.imu_samples    = 1000;
    in->stats.fifo_overflows = 2;
}

/* ── The always-present lines ────────────────────────────────────────────── */

static void test_core_lines(void)
{
    begin("test_core_lines");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    baseline(&cfg, &in);
    char buf[4096];
    size_t n = status_format(buf, sizeof buf, &in);

    EXPECT(n == strlen(buf), "return value is strlen");
    EXPECT(has(buf, "Chip IDs:"),    "chip IDs line");
    EXPECT(has(buf, "ism330dhcx 0x6B"), "imu driver and address");
    EXPECT(has(buf, "mmc5983ma 0x30"),  "mag driver and address");
    EXPECT(has(buf, "IMU ODR:        833 Hz"), "imu odr");
    EXPECT(has(buf, "FIFO watermark: 64"),     "fifo watermark");
    EXPECT(has(buf, "Mag ODR:        100 Hz"), "mag odr");
    EXPECT(has(buf, "IMU samples:    1000  overflows: 2"), "counters");
    EXPECT(has(buf, "Uptime:         01:01:01"), "uptime formatting");
    EXPECT(!has(buf, "Recent warnings"), "no warnings section when none");

    /* The last line must be complete: a caller writes exactly n bytes. */
    EXPECT(n > 0 && buf[n - 1] == '\n', "report ends on a newline");

    end(fb);
}

/* ── Fusion + degauss conditionals ───────────────────────────────────────── */

static void test_fusion_lines(void)
{
    begin("test_fusion_lines");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char buf[4096];

    baseline(&cfg, &in);
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "MEKF converging"), "not converged → converging");
    EXPECT(has(buf, "accel no  gyro no  mag no"), "no calibration");

    baseline(&cfg, &in);
    in.state.flags = FLAG_FUSION_CONVERGED | FLAG_ACCEL_CAL | FLAG_GYRO_CAL |
                     FLAG_MAG_CAL;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "MEKF converged"), "converged flag");
    EXPECT(has(buf, "accel yes  gyro yes  mag yes"), "all three calibrations");

    /* mag_set_period_s == 0 means the degauss cycle is off, and the line says
     * so inline rather than being dropped. */
    baseline(&cfg, &in);
    cfg.mag_set_period_s = 0.0f;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "SET every 0 s, disabled"), "degauss disabled is spelled out");

    baseline(&cfg, &in);
    cfg.mag_set_period_s = 5.0f;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "SET every 5 s)"), "degauss period shown when on");

    end(fb);
}

/* ── Declination ─────────────────────────────────────────────────────────── */

static void test_declination(void)
{
    begin("test_declination");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char buf[4096];

    baseline(&cfg, &in);
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Declination:    unknown  (no true heading output)"),
           "invalid → unknown, and says why it matters");
    EXPECT(!has(buf, " T\n"), "no true-heading value without declination");

    /* True heading wraps: 350 M + 15 E = 5 T, not 365. */
    baseline(&cfg, &in);
    in.state.flags |= FLAG_DECLINATION_VALID;
    in.state.heading_deg     = 350.0f;
    in.state.declination_deg = 15.0f;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "+15.00 E"), "declination value and sign");
    EXPECT(has(buf, "true heading 5.0 T"), "true heading wraps past 360");

    /* Westerly declination is negative, and the sum can go below zero. */
    baseline(&cfg, &in);
    in.state.flags |= FLAG_DECLINATION_VALID;
    in.state.heading_deg     = 10.0f;
    in.state.declination_deg = -20.0f;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "-20.00 E"), "westerly declination is signed");
    EXPECT(has(buf, "true heading 350.0 T"), "true heading wraps below zero");

    end(fb);
}

/* ── Heave and sea state ─────────────────────────────────────────────────── */

static void test_heave_and_wave(void)
{
    begin("test_heave_and_wave");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char buf[4096];

    /* Heave off → neither line. */
    baseline(&cfg, &in);
    cfg.heave_tau_s = 0.0f;
    cfg.wave_tau_s  = 120.0f;
    status_format(buf, sizeof buf, &in);
    EXPECT(!has(buf, "Heave:"),     "no heave line when heave_tau_s is 0");
    EXPECT(!has(buf, "Sea state:"), "sea state needs heave, whatever wave_tau_s is");

    /* Heave on, wave off → heave only. */
    baseline(&cfg, &in);
    cfg.heave_tau_s   = 12.0f;
    cfg.wave_tau_s    = 0.0f;
    in.state.heave_m  = -1.25f;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Heave:          -1.25 m"), "heave value, signed");
    EXPECT(!has(buf, "Sea state:"), "no sea state when wave_tau_s is 0");

    /* Both on, not yet valid. */
    baseline(&cfg, &in);
    cfg.heave_tau_s = 12.0f;
    cfg.wave_tau_s  = 120.0f;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Sea state:      settling"), "wave not valid → settling");

    /* Both on and valid. */
    baseline(&cfg, &in);
    cfg.heave_tau_s = 12.0f;
    cfg.wave_tau_s  = 120.0f;
    in.state.flags |= FLAG_WAVE_VALID;
    in.state.wave_height_m = 1.5f;
    in.state.wave_period_s = 6.0f;
    in.state.roll_period_s = 4.0f;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Hs 1.50 m  Tz 6.0 s  roll period 4.0 s"), "sea state values");

    end(fb);
}

/* ── Capture ─────────────────────────────────────────────────────────────── */

static void test_capture(void)
{
    begin("test_capture");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char buf[4096];

    baseline(&cfg, &in);
    cfg.capture_enabled = false;
    in.capture_active   = true;          /* must be ignored */
    in.capture_path     = "/var/lib/imud/x.imucap";
    status_format(buf, sizeof buf, &in);
    EXPECT(!has(buf, "Capture:"), "no capture line when disabled in config");

    baseline(&cfg, &in);
    cfg.capture_enabled = true;
    in.capture_active   = true;
    in.capture_path     = "/var/lib/imud/x.imucap";
    in.capture_bytes    = 3ULL * 1024 * 1024;   /* 3.0 MB */
    in.capture_drops    = 7;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "/var/lib/imud/x.imucap  (3.0 MB, 7 dropped)"),
           "active capture: path, MiB and drops");

    /* Enabled but the writer stopped — this is the line that tells an operator
     * the black box is not recording. */
    baseline(&cfg, &in);
    cfg.capture_enabled = true;
    in.capture_active   = false;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Capture:        stopped (see log)"), "stopped capture");

    /* A NULL path with capture active must not crash or print "(null)". */
    baseline(&cfg, &in);
    cfg.capture_enabled = true;
    in.capture_active   = true;
    in.capture_path     = NULL;
    status_format(buf, sizeof buf, &in);
    EXPECT(!has(buf, "(null)"), "NULL capture path prints empty, not (null)");

    end(fb);
}

/* ── Output configuration ────────────────────────────────────────────────── */

static void test_outputs(void)
{
    begin("test_outputs");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char buf[4096];

    /* All four NMEA combinations — the branch an operator checks after
     * enabling a plotter listener. */
    baseline(&cfg, &in);
    cfg.nmea_enabled = false; cfg.nmea_tcp_enabled = false;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "NMEA out:       disabled"), "nmea both off");

    baseline(&cfg, &in);
    cfg.nmea_enabled = true; cfg.nmea_tcp_enabled = false;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "NMEA out:       10 Hz  (port 10110)"), "nmea udp only");

    baseline(&cfg, &in);
    cfg.nmea_enabled = false; cfg.nmea_tcp_enabled = true;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "NMEA out:       10 Hz  (TCP port 10110)"), "nmea tcp only");

    baseline(&cfg, &in);
    cfg.nmea_enabled = true; cfg.nmea_tcp_enabled = true;
    cfg.nmea_tcp_port = 10119;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "NMEA out:       10 Hz  (UDP port 10110, TCP port 10119)"),
           "nmea both, both ports named");

    baseline(&cfg, &in);
    cfg.highrate_enabled = false;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Hi-rate out:    disabled"), "hi-rate off");

    baseline(&cfg, &in);
    cfg.highrate_enabled = true;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Hi-rate out:    500 Hz  (port 10111, NED)"),
           "hi-rate on, with the coordinate frame");

    end(fb);
}

/* ── Recent warnings ─────────────────────────────────────────────────────── */

static void test_recent(void)
{
    begin("test_recent");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char buf[4096];

    baseline(&cfg, &in);
    in.recent = "";
    status_format(buf, sizeof buf, &in);
    EXPECT(!has(buf, "Recent warnings"), "empty string is not a warnings section");

    baseline(&cfg, &in);
    in.recent = "12:00:00 W [mag] something\n";
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "Recent warnings:\n12:00:00 W [mag] something\n"),
           "warnings appended verbatim");

    end(fb);
}

/* ── Truncation ──────────────────────────────────────────────────────────── */

/*
 * The bound WS() exists to enforce.  Every configuration that lights up the
 * most lines is formatted at every buffer size from 0 to past its full length;
 * at each one the result must be NUL-terminated within the buffer and the
 * returned length must equal strlen().  Guard bytes either side catch a write
 * past the end that a strlen() check alone would miss.
 */
static void test_truncation(void)
{
    begin("test_truncation");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    baseline(&cfg, &in);
    /* Everything on, so the longest possible report is the one being cut. */
    cfg.capture_enabled  = true;
    cfg.heave_tau_s      = 12.0f;
    cfg.wave_tau_s       = 120.0f;
    cfg.nmea_enabled     = true;
    cfg.nmea_tcp_enabled = true;
    cfg.highrate_enabled = true;
    in.capture_active = true;
    in.capture_path   = "/var/lib/imud/imud-20260803-120000.imucap";
    in.state.flags    = FLAG_DECLINATION_VALID | FLAG_WAVE_VALID |
                        FLAG_FUSION_CONVERGED;
    in.recent = "12:00:00 W [mag] one\n12:00:01 E [imu] two\n";

    char full[4096];
    size_t flen = status_format(full, sizeof full, &in);
    EXPECT(flen > 400 && flen < sizeof full, "the everything-on report is sane");

    int bad_term = 0, bad_len = 0, bad_guard = 0, bad_prefix = 0;
    for (size_t sz = 0; sz <= flen + 8; sz++) {
        unsigned char arena[5000];
        memset(arena, 0xAA, sizeof arena);
        char *b = (char *)arena + 64;         /* 64 guard bytes either side */

        size_t n = status_format(b, sz, &in);

        if (sz == 0) {
            if (n != 0) bad_len++;
        } else {
            if (memchr(b, '\0', sz) == NULL)  bad_term++;
            if (n != strlen(b) || n >= sz)    bad_len++;
            /* Truncated output must be a prefix of the untruncated report —
             * a wrapped write pointer would show up here. */
            if (strncmp(b, full, n) != 0)     bad_prefix++;
        }
        for (size_t g = 0; g < 64; g++)
            if (arena[g] != 0xAA) { bad_guard++; break; }
        for (size_t g = 64 + sz; g < sizeof arena; g++)
            if (arena[g] != 0xAA) { bad_guard++; break; }
    }

    EXPECT(bad_term   == 0, "always NUL-terminated inside the buffer");
    EXPECT(bad_len    == 0, "returned length is strlen and is < sz");
    EXPECT(bad_prefix == 0, "truncated output is a prefix of the full report");
    EXPECT(bad_guard  == 0, "never writes outside [buf, buf + sz)");

    /* NULL buffer must be handled, not dereferenced. */
    EXPECT(status_format(NULL, 100, &in) == 0, "NULL buffer returns 0");

    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("test_status — imud-status report text\n");

    test_core_lines();
    test_fusion_lines();
    test_declination();
    test_heave_and_wave();
    test_capture();
    test_outputs();
    test_recent();
    test_truncation();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
