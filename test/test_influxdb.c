/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_influxdb.c — unit tests for the InfluxDB line-protocol encoder
 * (src/influx_line.c). Pure function — no sockets.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "influx_line.h"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* Value after ",key=" (or " key=" for the first field). */
static int field(const char *line, const char *key, double *out)
{
    char needle[40];
    snprintf(needle, sizeof needle, ",%s=", key);
    const char *p = strstr(line, needle);
    if (!p) {
        snprintf(needle, sizeof needle, " %s=", key);
        p = strstr(line, needle);
        if (!p) return 0;
    }
    *out = strtod(p + strlen(needle), NULL);
    return 1;
}

static int count_spaces(const char *s)
{
    int n = 0;
    for (const char *p = s; *p; p++) if (*p == ' ') n++;
    return n;
}

/* heading 90°, RoT 60 dpm, roll 0.10 rad, pitch -0.05, yaw 1.23, heave 0.42,
 * temp 31.4, seq 7, known ts.  The quaternion carries four distinct components
 * so a mis-indexed quat[] shows up as a swapped qw/qx/qy/qz pair, and
 * heading_true_deg starts at libimud's "declination not known" sentinel. */
static imud_data_t make_data(void)
{
    imud_data_t d;
    memset(&d, 0, sizeof d);
    d.ts_wall_ns = 1620307999123000000ULL;
    d.quat[0] = 0.9f; d.quat[1] = 0.3f; d.quat[2] = 0.2f; d.quat[3] = 0.1f;
    d.heading_deg = 90.0f;
    d.heading_true_deg = -1.0f;
    d.rate_of_turn = 60.0f;
    d.roll = 0.10f; d.pitch = -0.05f; d.yaw = 1.23f;
    d.heave_m = 0.42f; d.temp_c = 31.4f; d.imu_seq = 7;
    d.heave_rate = 0.25f;
    d.gyro_bias[0] = 0.001f; d.gyro_bias[1] = -0.002f; d.gyro_bias[2] = 0.003f;
    d.gyro_bias_var[0] = 1e-6f; d.gyro_bias_var[1] = 2e-6f; d.gyro_bias_var[2] = 3e-6f;
    d.accel_quiescence = 0.01f;
    d.wave_height_m = 1.6f; d.wave_period_s = 6.2f; d.roll_period_s = 4.4f;
    d.roll_amplitude = 0.12f; d.pitch_period_s = 5.1f; d.pitch_amplitude = 0.06f;
    d.mag_anomaly = 0.03f; d.mag_residual = 0.02f;
    d.innov_weight = 0.87f; d.innov_reject = 0.06f;
    d.nis_accel = 7.25f;    d.nis_mag = 1.10f;
    return d;
}

static void test_structure_and_deg(void)
{
    begin("test_structure_and_deg");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[1024];
    int n = influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(n > 0, "encode succeeds");
    EXPECT(strncmp(buf, "imud,source=imud ", 17) == 0, "measurement + source tag prefix");
    /* exactly two spaces: tags|fields and fields|timestamp */
    EXPECT(count_spaces(buf) == 2, "line has exactly two spaces (well-formed)");

    double v;
    EXPECT(field(buf, "heading", &v) && fabs(v - 90.0) < 1e-3, "heading 90° in deg mode");
    EXPECT(field(buf, "roll", &v)    && fabs(v - 5.7296) < 1e-2, "roll 0.10 rad → 5.73° (native)");
    EXPECT(field(buf, "rate_of_turn", &v) && fabs(v - 60.0) < 1e-2, "rate_of_turn 60 °/min");
    EXPECT(field(buf, "temp", &v)    && fabs(v - 31.4) < 1e-2, "temp 31.4");
    /* All four components, so an off-by-one into quat[] is visible. */
    EXPECT(field(buf, "qw", &v) && fabs(v - 0.9) < 1e-4, "quaternion qw = quat[0]");
    EXPECT(field(buf, "qx", &v) && fabs(v - 0.3) < 1e-4, "quaternion qx = quat[1]");
    EXPECT(field(buf, "qy", &v) && fabs(v - 0.2) < 1e-4, "quaternion qy = quat[2]");
    EXPECT(field(buf, "qz", &v) && fabs(v - 0.1) < 1e-4, "quaternion qz = quat[3]");
    EXPECT(strstr(buf, ",seq=7i") != NULL, "seq is an integer field (7i)");
    EXPECT(strstr(buf, " 1620307999123000000") != NULL, "nanosecond timestamp appended");
    end(fb);
}

static void test_rad_units(void)
{
    begin("test_rad_units");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[1024];
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, false, INFLUX_DETAIL_HEALTH);

    double v;
    EXPECT(field(buf, "heading", &v) && fabs(v - M_PI/2.0) < 1e-4, "heading 90° → π/2 rad");
    EXPECT(field(buf, "roll", &v)    && fabs(v - 0.10) < 1e-4, "roll 0.10 rad passthrough");
    EXPECT(field(buf, "rate_of_turn", &v) && fabs(v - 60.0*(M_PI/180.0)/60.0) < 1e-6,
           "rate_of_turn 60 dpm → rad/s");
    end(fb);
}

static void test_declination_gated(void)
{
    begin("test_declination_gated");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[1024];
    double v;

    d.flags = 0; d.declination_deg = 13.2f;
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(!field(buf, "heading_true", &v), "heading_true absent without flag");
    EXPECT(!field(buf, "variation", &v),    "variation absent without flag");

    d.flags = IMUD_FLAG_DECLINATION_VALID;
    d.heading_true_deg = 103.2f;               /* 90 mag + 13.2 E, per libimud */
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(field(buf, "variation", &v)    && fabs(v - 13.2) < 1e-2, "variation 13.2°");
    EXPECT(field(buf, "heading_true", &v) && fabs(v - 103.2) < 1e-2, "heading_true = 90 + 13.2");

    /*
     * True heading is libimud's member, not something the encoder recomputes.
     * The two are indistinguishable until the sum wraps: 350 + 20 E is 10°,
     * and an encoder adding the fields itself would write 370° here.
     */
    d.heading_deg      = 350.0f;
    d.declination_deg  = 20.0f;
    d.heading_true_deg = 10.0f;
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(field(buf, "heading_true", &v) && fabs(v - 10.0) < 1e-2,
           "heading_true is the library's wrapped value, not heading + declination");
    end(fb);
}

static void test_heave_gated_and_tags(void)
{
    begin("test_heave_gated_and_tags");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[1024];
    double v;

    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(!field(buf, "heave", &v), "heave absent when emit_heave=false");

    influx_build_line(buf, sizeof buf, &d, "boat", "vessel1", true, true, INFLUX_DETAIL_HEALTH);
    EXPECT(field(buf, "heave", &v) && fabs(v - 0.42) < 1e-3, "heave present when emit_heave=true");
    EXPECT(strncmp(buf, "boat,source=vessel1 ", 20) == 0, "custom measurement + source tag");
    end(fb);
}

static void test_v12_diagnostics(void)
{
    begin("test_v12_diagnostics");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[1024];
    double v;

    /* Gyro-bias / variance / quiescence are always emitted, never unit-converted
     * (raw SI even in deg mode), and independent of the heave gate. */
    d.flags = 0;
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(field(buf, "gbias_x", &v) && fabs(v - 0.001) < 1e-6, "gbias_x raw rad/s");
    EXPECT(field(buf, "gbias_y", &v) && fabs(v + 0.002) < 1e-6, "gbias_y raw rad/s");
    EXPECT(field(buf, "gbias_z", &v) && fabs(v - 0.003) < 1e-6, "gbias_z raw rad/s");
    EXPECT(field(buf, "gbias_var_x", &v) && fabs(v - 1e-6) < 1e-9, "gbias_var_x");
    EXPECT(field(buf, "gbias_var_y", &v) && fabs(v - 2e-6) < 1e-9, "gbias_var_y");
    EXPECT(field(buf, "quiescence", &v) && fabs(v - 0.01) < 1e-5, "quiescence");

    /* heave_rate + heave_valid gate with heave (emit_heave=false → absent). */
    EXPECT(!field(buf, "heave_rate", &v), "heave_rate absent when emit_heave=false");
    EXPECT(strstr(buf, "heave_valid=") == NULL, "heave_valid absent when emit_heave=false");

    /* emit_heave with the valid flag clear → heave still emitted (diagnostics sink),
     * heave_valid=f so the transient can be filtered downstream. */
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", true, true, INFLUX_DETAIL_HEALTH);
    EXPECT(field(buf, "heave_rate", &v) && fabs(v - 0.25) < 1e-3, "heave_rate present + value");
    EXPECT(field(buf, "heave", &v) && fabs(v - 0.42) < 1e-3, "heave emitted from t=0 regardless of validity");
    EXPECT(strstr(buf, "heave_valid=f") != NULL, "heave_valid=f when flag clear");

    /* Valid flag set → heave_valid=t. */
    d.flags = IMUD_FLAG_HEAVE_VALID;
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", true, true, INFLUX_DETAIL_HEALTH);
    EXPECT(strstr(buf, "heave_valid=t") != NULL, "heave_valid=t when flag set");
    end(fb);
}

static void test_v14_seastate(void)
{
    begin("test_v14_seastate");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[1024];
    double v;

    /* Sea state is always emitted (diagnostics sink), raw SI even in deg
     * mode, independent of the heave gate, with wave_valid as the filter. */
    d.flags = 0;
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(field(buf, "wave_height", &v) && fabs(v - 1.6) < 1e-3, "wave_height m, always on");
    EXPECT(field(buf, "wave_period", &v) && fabs(v - 6.2) < 1e-2, "wave_period s");
    EXPECT(field(buf, "roll_period", &v) && fabs(v - 4.4) < 1e-2, "roll_period s");
    EXPECT(field(buf, "roll_amplitude", &v) && fabs(v - 0.12) < 1e-3, "roll_amplitude rad, SI");
    EXPECT(field(buf, "pitch_period", &v) && fabs(v - 5.1) < 1e-2, "pitch_period s");
    EXPECT(field(buf, "pitch_amplitude", &v) && fabs(v - 0.06) < 1e-3, "pitch_amplitude rad, SI");
    EXPECT(field(buf, "mag_anomaly", &v) && fabs(v - 0.03) < 1e-4, "mag_anomaly always on");
    EXPECT(field(buf, "mag_residual", &v) && fabs(v - 0.02) < 1e-4, "mag_residual always on");
    EXPECT(field(buf, "innov_weight", &v) && fabs(v - 0.87) < 1e-4, "innov_weight always on");
    EXPECT(field(buf, "innov_reject", &v) && fabs(v - 0.06) < 1e-4, "innov_reject always on");
    EXPECT(field(buf, "nis_accel", &v) && fabs(v - 7.25) < 1e-4, "nis_accel always on");
    EXPECT(field(buf, "nis_mag", &v) && fabs(v - 1.10) < 1e-4, "nis_mag always on");
    EXPECT(strstr(buf, "wave_valid=f") != NULL, "wave_valid=f when flag clear");

    d.flags = IMUD_FLAG_WAVE_VALID;
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", false, true, INFLUX_DETAIL_HEALTH);
    EXPECT(strstr(buf, "wave_valid=t") != NULL, "wave_valid=t when flag set");
    end(fb);
}

static void test_buffer_too_small(void)
{
    begin("test_buffer_too_small");
    int fb = g_fail;
    imud_data_t d = make_data();
    char small[32];
    EXPECT(influx_build_line(small, sizeof small, &d, "imud", "imud", true, true, INFLUX_DETAIL_HEALTH) == -1,
           "returns -1 when buffer too small");
    end(fb);
}

/*
 * The diagnostics sink keeps emitting heading with no magnetometer, because
 * the drift of an unreferenced heading is a thing you would come here to
 * measure.  Two booleans say what the number means, the way heave_valid does
 * for heave: heading_ref (the mag is being fused) and mag_absent (none is
 * fitted at all).
 */
static void test_heading_always_emitted_with_validity(void)
{
    begin("test_heading_always_emitted_with_validity");
    int fb = g_fail;

    char buf[2048];
    imud_data_t d = make_data();

    d.flags = 0;                                /* dead reckoned */
    d.flags_ext = IMUD_FLAG_EXT_MAG_ABSENT;
    EXPECT(influx_build_line(buf, sizeof buf, &d, "imu", NULL, true, true, INFLUX_DETAIL_HEALTH) > 0,
           "line builds with no magnetometer");
    EXPECT(strstr(buf, ",heading=") != NULL, "heading still emitted");
    EXPECT(strstr(buf, "heading_ref=f") != NULL, "heading_ref=f when unreferenced");
    EXPECT(strstr(buf, "mag_absent=t") != NULL, "mag_absent=t with no mag fitted");

    d.flags = IMUD_FLAG_MAG_VALID;
    d.flags_ext = 0;
    EXPECT(influx_build_line(buf, sizeof buf, &d, "imu", NULL, true, true, INFLUX_DETAIL_HEALTH) > 0,
           "line builds with a fused magnetometer");
    EXPECT(strstr(buf, "heading_ref=t") != NULL, "heading_ref=t when fused");
    EXPECT(strstr(buf, "mag_absent=f") != NULL, "mag_absent=f when one is fitted");

    /* Uncalibrated still counts as referenced — bounded, not drifting. */
    d.flags = IMUD_FLAG_MAG_UNCAL;
    EXPECT(influx_build_line(buf, sizeof buf, &d, "imu", NULL, true, true, INFLUX_DETAIL_HEALTH) > 0,
           "line builds on an uncalibrated fuse");
    EXPECT(strstr(buf, "heading_ref=t") != NULL, "heading_ref=t on an uncal fuse");
    end(fb);
}

/* Count field assignments between the tag set and the trailing timestamp.
 * Line protocol is "<measurement>,<tags> <fields> <ts>", so the field set is
 * bounded by the first space and the last one. */
static int nfields(const char *line)
{
    const char *start = strchr(line, ' ');
    const char *end   = strrchr(line, ' ');
    int n = 0;
    if (!start || !end || end <= start) return 0;
    for (const char *q = start; q < end; q++)
        if (*q == '=') n++;
    return n;
}

/*
 * The five levels are CUMULATIVE, and that is the property worth pinning:
 * every field at level N must still be there at N+1, so a dashboard built at
 * one level keeps working when someone raises it.  Checking a field list per
 * level would pass just as well if the levels were disjoint sets.
 */
static void test_detail_levels_are_cumulative(void)
{
    begin("test_detail_levels_are_cumulative");
    int fb = g_fail;

    imud_data_t d = make_data();
    d.flags |= IMUD_FLAG_DECLINATION_VALID;

    char prev[4096] = "", cur[4096];
    int  prev_n = 0;

    for (int lvl = INFLUX_DETAIL_ATTITUDE; lvl <= INFLUX_DETAIL_FULL; lvl++) {
        int n = influx_build_line(cur, sizeof cur, &d, "imud", "imud", true, true, lvl);
        EXPECT(n > 0, "line builds at every level");

        int nf = nfields(cur);
        EXPECT(nf > prev_n, "each level adds fields");

        /* Every field name the previous level emitted must still appear. */
        if (prev[0]) {
            char work[4096];
            snprintf(work, sizeof work, "%s", prev);
            char *ts = strrchr(work, ' ');
            if (ts) *ts = '\0';           /* drop the trailing timestamp */
            char *sp = strchr(work, ' ');
            EXPECT(sp != NULL, "previous line has a field set");
            if (sp) {
                int missing = 0;
                for (char *tok = strtok(sp + 1, ","); tok; tok = strtok(NULL, ",")) {
                    char *eq = strchr(tok, '=');
                    if (!eq) continue;
                    *eq = '\0';
                    char needle[64];
                    snprintf(needle, sizeof needle, ",%s=", tok);
                    /* the first field follows the space, not a comma */
                    char first[64];
                    snprintf(first, sizeof first, " %s=", tok);
                    if (!strstr(cur, needle) && !strstr(cur, first)) missing++;
                }
                EXPECT(missing == 0, "no field from the level below went missing");
            }
        }
        snprintf(prev, sizeof prev, "%s", cur);
        prev_n = nf;
    }
    end(fb);
}

/* health is the default, and must be byte-identical to what an unset config
 * produced before the levels existed. */
static void test_detail_boundaries(void)
{
    begin("test_detail_boundaries");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[4096];

    influx_build_line(buf, sizeof buf, &d, "imud", "imud", true, true,
                      INFLUX_DETAIL_ATTITUDE);
    EXPECT(strstr(buf, "qw=") && strstr(buf, ",heading=") && strstr(buf, ",seq="),
           "attitude carries quaternion, heading and seq");
    EXPECT(!strstr(buf, "wave_height="), "attitude has no sea state");
    EXPECT(!strstr(buf, "gbias_x="),     "attitude has no filter health");
    EXPECT(!strstr(buf, ",accel_x="),    "attitude has no sensor vectors");

    influx_build_line(buf, sizeof buf, &d, "imud", "imud", true, true,
                      INFLUX_DETAIL_HEALTH);
    EXPECT(strstr(buf, "gbias_x=") && strstr(buf, "nis_accel="),
           "health carries the filter diagnostics");
    EXPECT(!strstr(buf, ",accel_x="), "health stops short of the sensor vectors");

    influx_build_line(buf, sizeof buf, &d, "imud", "imud", true, true,
                      INFLUX_DETAIL_FULL);
    static const char *const full_only[] = {
        ",accel_x=", ",accel_y=", ",accel_z=",
        ",accel_raw_x=", ",accel_raw_y=", ",accel_raw_z=",
        ",gyro_x=", ",gyro_y=", ",gyro_z=",
        ",gyro_raw_x=", ",gyro_raw_y=", ",gyro_raw_z=",
        ",mag_x=", ",mag_y=", ",mag_z=",
        ",mag_raw_x=", ",mag_raw_y=", ",mag_raw_z=",
        ",ts_tai_ns=", ",ts_chip_ticks=", ",anchor_gen=",
    };
    for (size_t i = 0; i < sizeof full_only / sizeof *full_only; i++) {
        char msg[64];
        snprintf(msg, sizeof msg, "full carries %s", full_only[i]);
        EXPECT(strstr(buf, full_only[i]) != NULL, msg);
    }

    /*
     * Field counts per level, declination known and heave on.  This is what
     * catches a field quietly dropped from a level: the cumulative test only
     * proves each level is bigger than the one below, which stays true when
     * three of full's twenty-one go missing.
     */
    static const int want[] = { 0, 12, 18, 25, 38, 59 };
    for (int lvl = INFLUX_DETAIL_ATTITUDE; lvl <= INFLUX_DETAIL_FULL; lvl++) {
        imud_data_t q = make_data();
        q.flags |= IMUD_FLAG_DECLINATION_VALID;
        influx_build_line(buf, sizeof buf, &q, "imud", "imud", true, true, lvl);
        char msg[64];
        snprintf(msg, sizeof msg, "level %d emits %d fields", lvl, want[lvl]);
        EXPECT(nfields(buf) == want[lvl], msg);
    }

    /* An out-of-range level is a config this build does not know: fall back to
     * the default rather than emit an empty field set, which is not valid line
     * protocol and would be rejected point by point. */
    char fallback[4096];
    influx_build_line(fallback, sizeof fallback, &d, "imud", "imud", true, true, 99);
    influx_build_line(buf, sizeof buf, &d, "imud", "imud", true, true,
                      INFLUX_DETAIL_HEALTH);
    EXPECT(strcmp(fallback, buf) == 0, "an unknown level falls back to health");

    /* Name mapping, which is what the config string goes through. */
    EXPECT(influx_detail_from_name("attitude") == INFLUX_DETAIL_ATTITUDE, "name: attitude");
    EXPECT(influx_detail_from_name("full")     == INFLUX_DETAIL_FULL,     "name: full");
    EXPECT(influx_detail_from_name("HEALTH")   == -1,  "names are case-sensitive");
    EXPECT(influx_detail_from_name("")         == -1,  "empty name rejected");
    EXPECT(influx_detail_from_name(NULL)       == -1,  "NULL name rejected");
    EXPECT(strcmp(influx_detail_name(-1), "health") == 0, "unknown level names health");
    end(fb);
}

int main(void)
{
    puts("=== imud influxdb line tests ===");
    test_structure_and_deg();
    test_rad_units();
    test_declination_gated();
    test_heave_gated_and_tags();
    test_v12_diagnostics();
    test_v14_seastate();
    test_heading_always_emitted_with_validity();
    test_detail_levels_are_cumulative();
    test_detail_boundaries();
    test_buffer_too_small();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
