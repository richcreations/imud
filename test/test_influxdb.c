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
 * temp 31.4, seq 7, quat identity, known ts. */
static imud_packet_t make_pkt(void)
{
    imud_packet_t p;
    memset(&p, 0, sizeof p);
    p.magic = IMUD_MAGIC; p.version = IMUD_VERSION;
    p.ts_wall_ns = 1620307999123000000ULL;
    p.quat_w = 1.0f;
    p.heading_deg = 90.0f;
    p.rate_of_turn = 60.0f;
    p.roll = 0.10f; p.pitch = -0.05f; p.yaw = 1.23f;
    p.heave_m = 0.42f; p.temp_c = 31.4f; p.imu_seq = 7;
    p.heave_rate = 0.25f;
    p.gyro_bias_x = 0.001f; p.gyro_bias_y = -0.002f; p.gyro_bias_z = 0.003f;
    p.gyro_bias_var_x = 1e-6f; p.gyro_bias_var_y = 2e-6f; p.gyro_bias_var_z = 3e-6f;
    p.accel_quiescence = 0.01f;
    p.wave_height_m = 1.6f; p.wave_period_s = 6.2f; p.roll_period_s = 4.4f;
    p.roll_amplitude = 0.12f; p.pitch_period_s = 5.1f; p.pitch_amplitude = 0.06f;
    p.mag_anomaly = 0.03f; p.mag_residual = 0.02f;
    p.innov_weight = 0.87f; p.innov_reject = 0.06f;
    p.nis_accel = 7.25f;    p.nis_mag = 1.10f;
    return p;
}

static void test_structure_and_deg(void)
{
    begin("test_structure_and_deg");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[1024];
    int n = influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, true);
    EXPECT(n > 0, "encode succeeds");
    EXPECT(strncmp(buf, "imud,source=imud ", 17) == 0, "measurement + source tag prefix");
    /* exactly two spaces: tags|fields and fields|timestamp */
    EXPECT(count_spaces(buf) == 2, "line has exactly two spaces (well-formed)");

    double v;
    EXPECT(field(buf, "heading", &v) && fabs(v - 90.0) < 1e-3, "heading 90° in deg mode");
    EXPECT(field(buf, "roll", &v)    && fabs(v - 5.7296) < 1e-2, "roll 0.10 rad → 5.73° (native)");
    EXPECT(field(buf, "rate_of_turn", &v) && fabs(v - 60.0) < 1e-2, "rate_of_turn 60 °/min");
    EXPECT(field(buf, "temp", &v)    && fabs(v - 31.4) < 1e-2, "temp 31.4");
    EXPECT(field(buf, "qw", &v)      && fabs(v - 1.0) < 1e-4, "quaternion qw=1");
    EXPECT(strstr(buf, ",seq=7i") != NULL, "seq is an integer field (7i)");
    EXPECT(strstr(buf, " 1620307999123000000") != NULL, "nanosecond timestamp appended");
    end(fb);
}

static void test_rad_units(void)
{
    begin("test_rad_units");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[1024];
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, false);

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

    imud_packet_t p = make_pkt();
    char buf[1024];
    double v;

    p.flags = 0; p.declination_deg = 13.2f;
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, true);
    EXPECT(!field(buf, "heading_true", &v), "heading_true absent without flag");
    EXPECT(!field(buf, "variation", &v),    "variation absent without flag");

    p.flags = IMUD_FLAG_DECLINATION_VALID;
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, true);
    EXPECT(field(buf, "variation", &v)    && fabs(v - 13.2) < 1e-2, "variation 13.2°");
    EXPECT(field(buf, "heading_true", &v) && fabs(v - 103.2) < 1e-2, "heading_true = 90 + 13.2");
    end(fb);
}

static void test_heave_gated_and_tags(void)
{
    begin("test_heave_gated_and_tags");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[1024];
    double v;

    influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, true);
    EXPECT(!field(buf, "heave", &v), "heave absent when emit_heave=false");

    influx_build_line(buf, sizeof buf, &p, "boat", "vessel1", true, true);
    EXPECT(field(buf, "heave", &v) && fabs(v - 0.42) < 1e-3, "heave present when emit_heave=true");
    EXPECT(strncmp(buf, "boat,source=vessel1 ", 20) == 0, "custom measurement + source tag");
    end(fb);
}

static void test_v12_diagnostics(void)
{
    begin("test_v12_diagnostics");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[1024];
    double v;

    /* Gyro-bias / variance / quiescence are always emitted, never unit-converted
     * (raw SI even in deg mode), and independent of the heave gate. */
    p.flags = 0;
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, true);
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
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", true, true);
    EXPECT(field(buf, "heave_rate", &v) && fabs(v - 0.25) < 1e-3, "heave_rate present + value");
    EXPECT(field(buf, "heave", &v) && fabs(v - 0.42) < 1e-3, "heave emitted from t=0 regardless of validity");
    EXPECT(strstr(buf, "heave_valid=f") != NULL, "heave_valid=f when flag clear");

    /* Valid flag set → heave_valid=t. */
    p.flags = IMUD_FLAG_HEAVE_VALID;
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", true, true);
    EXPECT(strstr(buf, "heave_valid=t") != NULL, "heave_valid=t when flag set");
    end(fb);
}

static void test_v14_seastate(void)
{
    begin("test_v14_seastate");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[1024];
    double v;

    /* Sea state is always emitted (diagnostics sink), raw SI even in deg
     * mode, independent of the heave gate, with wave_valid as the filter. */
    p.flags = 0;
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, true);
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

    p.flags = IMUD_FLAG_WAVE_VALID;
    influx_build_line(buf, sizeof buf, &p, "imud", "imud", false, true);
    EXPECT(strstr(buf, "wave_valid=t") != NULL, "wave_valid=t when flag set");
    end(fb);
}

static void test_buffer_too_small(void)
{
    begin("test_buffer_too_small");
    int fb = g_fail;
    imud_packet_t p = make_pkt();
    char small[32];
    EXPECT(influx_build_line(small, sizeof small, &p, "imud", "imud", true, true) == -1,
           "returns -1 when buffer too small");
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
    test_buffer_too_small();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
