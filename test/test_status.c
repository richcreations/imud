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

#include <math.h>
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

/*
 * Three distinct states, and an operator needs to tell them apart:
 *
 *   MAG_VALID        heading stands on its own (calibrated and healthy)
 *   MAG_UNCAL        fused from an uncalibrated field — bounded and
 *                    repeatable, but offset by the uncorrected hard iron
 *   neither          dead-reckoned; measured on a static bench with the mag
 *                    not fused, heading walked 220 degrees in 24 minutes
 *                    while pitch and roll stayed correct to a tenth of a
 *                    degree, so the number itself gives no hint
 *
 * Both are set only while mekf_update_mag() is actually applying updates, so
 * they are the precise signal for the third.  "Calibration: mag no"
 * appears above but reports whether a FILE exists; these report whether the
 * heading can be believed, which is the question actually being asked.
 */
static void test_heading_dead_reckoned_is_called_out(void)
{
    begin("test_heading_dead_reckoned_is_called_out");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char buf[4096];

    /* Fused and calibrated: the heading stands on its own. */
    baseline(&cfg, &in);
    in.state.flags = FLAG_MAG_VALID | FLAG_MAG_CAL;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "heading=123.4 M"), "heading still reported when fused");
    EXPECT(!has(buf, "DEAD RECKONED"),
           "no warning when the magnetometer is being fused");
    EXPECT(!has(buf, "UNCALIBRATED"),
           "no uncalibrated caveat when a calibration is applied");

    /* Fused but uncalibrated: usable for holding a heading, not for reading
     * one off, and the two are not the same claim.  MAG_VALID stays CLEAR
     * here — it keeps its original "calibrated and healthy" meaning, so a
     * consumer that only tests it is unaffected by uncalibrated fusion. */
    baseline(&cfg, &in);
    in.state.flags = FLAG_MAG_UNCAL;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "heading=123.4 M"), "heading reported when uncalibrated");
    EXPECT(!has(buf, "DEAD RECKONED"),
           "an uncalibrated mag is fused, so it is not dead reckoning");
    EXPECT(has(buf, "UNCALIBRATED"), "uncalibrated fusion is called out");
    EXPECT(has(buf, "imud-cal mag"), "and says what to do about it");

    /* Yaw update not running: the number needs the caveat beside it. */
    baseline(&cfg, &in);
    in.state.flags = 0;
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "heading=123.4 M"), "heading is still reported");
    EXPECT(has(buf, "DEAD RECKONED"), "dead-reckoning is called out");
    EXPECT(has(buf, "imud-cal mag"), "and says what to do about it");

    /*
     * The distinction that matters: a calibration FILE is not the same as a
     * running update.  A mag cal loaded but no update yet performed must still
     * warn, or the caveat disappears exactly when a stalled magnetometer makes
     * it most necessary.
     */
    baseline(&cfg, &in);
    in.state.flags = FLAG_MAG_CAL;          /* cal loaded, update never ran */
    status_format(buf, sizeof buf, &in);
    EXPECT(has(buf, "DEAD RECKONED"),
           "a loaded calibration alone does not clear the warning");

    end(fb);
}

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

/* ── JSON (status_format_json) ────────────────────────────────────────────── */

/*
 * Structurally complete: braces and brackets balanced, string state tracked so
 * a '{' inside a warning does not count, and the whole thing closed off.  Not a
 * parser — a parser would only be able to say the same thing, and this can say
 * it about a buffer that was deliberately cut short.
 */
static bool json_complete(const char *s)
{
    int depth = 0;
    bool instr = false, esc = false, closed = false;
    for (; *s; s++) {
        if (instr) {
            if (esc)              esc = false;
            else if (*s == '\\')  esc = true;
            else if (*s == '"')   instr = false;
            continue;
        }
        switch (*s) {
        case '"': instr = true; break;
        case '{': case '[': depth++; break;
        case '}': case ']':
            if (--depth < 0) return false;
            if (depth == 0)  closed = true;
            break;
        default: break;
        }
    }
    return closed && depth == 0 && !instr;
}

/* Everything on, so every conditional field carries a real value. */
static void everything_on(imud_config_t *cfg, status_input_t *in)
{
    baseline(cfg, in);
    cfg->capture_enabled  = true;
    cfg->heave_tau_s      = 12.0f;
    cfg->wave_tau_s       = 120.0f;
    cfg->nmea_enabled     = true;
    cfg->nmea_tcp_enabled = true;
    cfg->highrate_enabled = true;
    in->capture_active    = true;
    in->capture_path      = "/var/lib/imud/imud-20260803-120000.imucap";
    in->capture_bytes     = 4096;
    in->capture_drops     = 7;
    in->state.flags       = FLAG_DECLINATION_VALID | FLAG_WAVE_VALID |
                            FLAG_FUSION_CONVERGED  | FLAG_MAG_VALID |
                            FLAG_ACCEL_CAL | FLAG_GYRO_CAL | FLAG_MAG_CAL;
    in->state.declination_deg = 13.25f;
    in->state.wave_height_m   = 1.5f;
    in->recent = "  12:00:00 W [mag] one\n  12:00:01 E [imu] two\n";
}

/*
 * The schema is the contract — a script must not have to test for a key's
 * presence before reading it, so every subsystem's object is emitted whether
 * or not it is enabled.  These assertions are what make that a promise.
 */
static void test_json_schema(void)
{
    begin("test_json_schema");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    everything_on(&cfg, &in);

    char j[4096];
    size_t n = status_format_json(j, sizeof j, &in);
    EXPECT(n > 0 && n == strlen(j),  "returns the length it wrote");
    EXPECT(json_complete(j),         "the object is structurally complete");
    EXPECT(j[n - 1] == '\n',         "ends with a newline");

    static const char *keys[] = {
        "\"imud_version\":", "\"uptime_s\":", "\"imu\":", "\"mag\":",
        "\"fusion\":", "\"calibration\":", "\"attitude\":", "\"declination\":",
        "\"heave\":", "\"sea_state\":", "\"capture\":", "\"nmea\":",
        "\"highrate\":", "\"counters\":", "\"warnings\":",
    };
    int missing = 0;
    for (size_t i = 0; i < sizeof keys / sizeof *keys; i++)
        if (!has(j, keys[i])) missing++;
    EXPECT(missing == 0, "every top-level key is present");

    EXPECT(has(j, "\"uptime_s\":3661"),               "uptime is seconds");
    EXPECT(has(j, "\"driver\":\"ism330dhcx\""),       "IMU driver name");
    EXPECT(has(j, "\"odr_mhz\":833000"),              "ODR stays milli-Hz");
    EXPECT(has(j, "\"converged\":true"),              "fusion convergence");
    EXPECT(has(j, "\"accel\":true,\"gyro\":true,\"mag\":true"),
           "calibration flags");
    EXPECT(has(j, "\"heading_deg\":123.4"),           "heading");
    EXPECT(has(j, "\"declination_deg\":13.25"),       "declination");
    EXPECT(has(j, "\"true_heading_deg\":136.65"),     "true heading is summed");
    EXPECT(has(j, "\"bytes\":4096,\"drops\":7"),      "capture counters");
    EXPECT(has(j, "\"imu_samples\":1000,\"fifo_overflows\":2"), "counters");
    EXPECT(has(j, "\"12:00:00 W [mag] one\""),        "a warning, indent gone");
    EXPECT(has(j, "\"12:00:01 E [imu] two\""),        "and the second");

    EXPECT(status_format_json(NULL, 100, &in) == 0, "NULL buffer returns 0");

    end(fb);
}

/* The three heading states, which the text report spends a paragraph on. */
static void test_json_heading_source(void)
{
    begin("test_json_heading_source");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    char j[4096];

    baseline(&cfg, &in);
    in.state.flags = FLAG_MAG_VALID;
    status_format_json(j, sizeof j, &in);
    EXPECT(has(j, "\"heading_source\":\"calibrated\""), "MAG_VALID");

    baseline(&cfg, &in);
    in.state.flags = FLAG_MAG_UNCAL;
    status_format_json(j, sizeof j, &in);
    EXPECT(has(j, "\"heading_source\":\"uncalibrated\""), "MAG_UNCAL");

    baseline(&cfg, &in);
    in.state.flags = 0;
    status_format_json(j, sizeof j, &in);
    EXPECT(has(j, "\"heading_source\":\"dead_reckoned\""), "neither");

    end(fb);
}

/*
 * A field whose subsystem is off is null, never a zero that reads as a
 * measurement: "heave_m": 0 on a daemon with heave disabled is a lie a script
 * cannot detect.
 */
static void test_json_disabled_is_null(void)
{
    begin("test_json_disabled_is_null");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    baseline(&cfg, &in);                    /* declination invalid, all off */
    cfg.heave_tau_s = 0.0f;
    cfg.wave_tau_s  = 0.0f;
    cfg.capture_enabled = false;

    char j[4096];
    EXPECT(status_format_json(j, sizeof j, &in) > 0, "formats");
    EXPECT(json_complete(j), "still a complete object");

    EXPECT(has(j, "\"declination\":{\"valid\":false,\"declination_deg\":null,"
                  "\"true_heading_deg\":null}"), "declination unknown");
    EXPECT(has(j, "\"heave\":{\"enabled\":false,\"heave_m\":null}"),
           "heave off");
    EXPECT(has(j, "\"sea_state\":{\"enabled\":false,\"valid\":false,"
                  "\"wave_height_m\":null"), "sea state off");
    EXPECT(has(j, "\"capture\":{\"enabled\":false,\"active\":false,"
                  "\"path\":null,\"bytes\":null,\"drops\":null}"),
           "capture off");
    EXPECT(has(j, "\"warnings\":[]"), "no warnings is an empty array");

    /* Enabled but not yet settled is a third thing again: on, and no number. */
    baseline(&cfg, &in);
    cfg.heave_tau_s = 12.0f;
    cfg.wave_tau_s  = 120.0f;
    status_format_json(j, sizeof j, &in);
    EXPECT(has(j, "\"sea_state\":{\"enabled\":true,\"valid\":false,"
                  "\"wave_height_m\":null"), "sea state settling");

    end(fb);
}

/*
 * The MEKF can reset itself on a non-finite state (FLAG_STATE_RESET exists for
 * exactly that), and printf writes "nan", which is not JSON.  A diverging
 * filter is the moment a monitoring script most needs a parsable answer.
 */
static void test_json_non_finite_is_null(void)
{
    begin("test_json_non_finite_is_null");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    everything_on(&cfg, &in);
    in.state.heading_deg = NAN;
    in.state.pitch       = (float)INFINITY;
    in.state.roll        = -(float)INFINITY;
    in.state.cov[0]      = NAN;

    char j[4096];
    EXPECT(status_format_json(j, sizeof j, &in) > 0, "still formats");
    EXPECT(json_complete(j), "still a complete object");
    EXPECT(!has(j, "nan") && !has(j, "NaN"), "no nan anywhere");
    EXPECT(!has(j, "inf") && !has(j, "Inf"), "no inf anywhere");
    EXPECT(has(j, "\"heading_deg\":null"),      "heading is null");
    EXPECT(has(j, "\"pitch_deg\":null"),        "pitch is null");
    EXPECT(has(j, "\"roll_deg\":null"),         "roll is null");
    EXPECT(has(j, "\"cov_trace_rad2\":null"),   "cov trace is null");
    /* A non-finite heading must not poison the true heading either. */
    EXPECT(has(j, "\"true_heading_deg\":null"), "true heading is null");

    end(fb);
}

/* Log lines and paths are not JSON-safe by construction. */
static void test_json_escapes(void)
{
    begin("test_json_escapes");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    everything_on(&cfg, &in);
    in.capture_path = "/tmp/a\"b\\c.imucap";
    in.recent       = "  12:00:00 W said \"no\"\n  12:00:01 E a\tb\x01\n";

    char j[4096];
    EXPECT(status_format_json(j, sizeof j, &in) > 0, "formats");
    EXPECT(json_complete(j), "the object survives the quotes");
    EXPECT(has(j, "\"path\":\"/tmp/a\\\"b\\\\c.imucap\""), "path is escaped");
    EXPECT(has(j, "said \\\"no\\\""),  "a quote in a warning is escaped");
    EXPECT(has(j, "a\\tb\\u0001"),     "tab and a control byte are escaped");

    end(fb);
}

/*
 * All or nothing.  Half an object fails a consumer's parser, so at every
 * buffer size the answer is either nothing at all or something complete —
 * the warnings array shedding elements to stay inside the buffer rather than
 * pushing the object over.  Guard bytes catch a write past the end.
 */
static void test_json_all_or_nothing(void)
{
    begin("test_json_all_or_nothing");
    int fb = g_fail;

    imud_config_t cfg; status_input_t in;
    everything_on(&cfg, &in);

    char full[4096];
    size_t flen = status_format_json(full, sizeof full, &in);
    EXPECT(flen > 600 && flen < sizeof full, "the everything-on object is sane");

    int bad_len = 0, bad_form = 0, bad_guard = 0, degraded = 0;
    for (size_t sz = 0; sz <= flen + 8; sz++) {
        unsigned char arena[5000];
        memset(arena, 0xAA, sizeof arena);
        char *b = (char *)arena + 64;         /* 64 guard bytes either side */

        size_t n = status_format_json(b, sz, &in);

        if (n == 0) {
            if (sz > 0 && b[0] != '\0') bad_len++;
        } else {
            if (n != strlen(b) || n >= sz) bad_len++;
            if (!json_complete(b))         bad_form++;
            if (n < flen)                  degraded++;
        }
        for (size_t g = 0; g < 64; g++)
            if (arena[g] != 0xAA) { bad_guard++; break; }
        for (size_t g = 64 + sz; g < sizeof arena; g++)
            if (arena[g] != 0xAA) { bad_guard++; break; }
    }

    EXPECT(bad_len   == 0, "returned length is strlen and is < sz");
    EXPECT(bad_form  == 0, "every non-empty result is a complete object");
    EXPECT(bad_guard == 0, "never writes outside [buf, buf + sz)");
    EXPECT(degraded  >  0, "a short buffer sheds warnings instead of failing");

    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("test_status — imud-status report text\n");

    test_core_lines();
    test_heading_dead_reckoned_is_called_out();
    test_fusion_lines();
    test_declination();
    test_heave_and_wave();
    test_capture();
    test_outputs();
    test_recent();
    test_truncation();
    test_json_schema();
    test_json_heading_source();
    test_json_disabled_is_null();
    test_json_non_finite_is_null();
    test_json_escapes();
    test_json_all_or_nothing();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
