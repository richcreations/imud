/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_prometheus.c — unit tests for the Prometheus metrics builder
 * (src/prom_metrics.c). Drives prom_build_metrics() with crafted
 * imud_data_t views and asserts on metric names, SI values, flag gauges,
 * HELP/TYPE self-description, and truncation handling.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../include/prom_metrics.h"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* Parse "name value\n" out of the page; returns 1 and sets *v on match. */
static int metric(const char *page, const char *name, double *v)
{
    size_t nl = strlen(name);
    for (const char *p = page; (p = strstr(p, name)) != NULL; p++) {
        /* must start a line and be followed by a space (not a longer name) */
        if ((p != page && p[-1] != '\n') || p[nl] != ' ')
            continue;
        *v = strtod(p + nl + 1, NULL);
        return 1;
    }
    return 0;
}

static imud_data_t make_data(void)
{
    imud_data_t d;
    memset(&d, 0, sizeof d);
    d.heading_deg      = 90.0f;
    d.heading_true_deg = 103.2f;
    d.roll             = 0.10f;
    d.pitch            = -0.05f;
    d.yaw              = 1.57f;
    d.rate_of_turn     = 60.0f;      /* deg/min */
    d.heave_m          = 0.42f;
    d.heave_rate       = 0.25f;
    d.wave_height_m    = 1.6f;
    d.wave_period_s    = 6.2f;
    d.roll_period_s    = 4.4f;
    d.roll_amplitude   = 0.12f;
    d.pitch_period_s   = 5.1f;
    d.pitch_amplitude  = 0.06f;
    d.mag_anomaly      = 0.03f;
    d.mag_residual     = 0.02f;
    d.innov_weight     = 0.87f;
    d.innov_reject     = 0.06f;
    d.nis_accel        = 7.25f;
    d.nis_mag          = 1.10f;
    d.accel_quiescence = 0.001f;
    d.gyro_bias[1]     = -0.002f;
    d.temp_c           = 31.4f;
    return d;
}

static void test_no_packet(void)
{
    begin("test_no_packet");
    int fb = g_fail;

    char buf[4096];
    double v;
    int n = prom_build_metrics(buf, sizeof buf, NULL, 5);
    EXPECT(n > 0, "builds without a packet");
    EXPECT(metric(buf, "imud_up", &v) && v == 0.0, "imud_up 0 without a packet");
    EXPECT(metric(buf, "imud_packets_total", &v) && v == 5.0, "packets_total emitted");
    EXPECT(!metric(buf, "imud_heading_degrees", &v), "no data gauges without a packet");
    end(fb);
}

static void test_values_si(void)
{
    begin("test_values_si");
    int fb = g_fail;

    imud_data_t d = make_data();
    d.flags = IMUD_FLAG_DECLINATION_VALID;
    char buf[8192];
    double v;
    int n = prom_build_metrics(buf, sizeof buf, &d, 100);
    EXPECT(n > 0, "builds with a packet");
    EXPECT(metric(buf, "imud_up", &v) && v == 1.0, "imud_up 1");
    EXPECT(metric(buf, "imud_heading_degrees", &v) && fabs(v - 90.0) < 1e-3,
           "heading in degrees");
    EXPECT(metric(buf, "imud_heading_true_degrees", &v) && fabs(v - 103.2) < 1e-3,
           "true heading when declination valid");
    EXPECT(metric(buf, "imud_roll_radians", &v) && fabs(v - 0.10) < 1e-5,
           "roll in radians (SI)");
    /* 60 deg/min = 1 deg/s = 0.017453 rad/s */
    EXPECT(metric(buf, "imud_rate_of_turn_radians_per_second", &v) &&
           fabs(v - 0.0174533) < 1e-5, "rate of turn converted to rad/s");
    EXPECT(metric(buf, "imud_wave_height_meters", &v) && fabs(v - 1.6) < 1e-3,
           "wave height m");
    EXPECT(metric(buf, "imud_roll_amplitude_radians", &v) && fabs(v - 0.12) < 1e-4,
           "roll amplitude rad");
    EXPECT(metric(buf, "imud_pitch_period_seconds", &v) && fabs(v - 5.1) < 1e-2,
           "pitch period s");
    EXPECT(metric(buf, "imud_mag_anomaly_ratio", &v) && fabs(v - 0.03) < 1e-4,
           "mag anomaly");
    EXPECT(metric(buf, "imud_innov_weight_ratio", &v) && fabs(v - 0.87) < 1e-4,
           "innov_weight always on");
    EXPECT(metric(buf, "imud_innov_reject_ratio", &v) && fabs(v - 0.06) < 1e-4,
           "innov_reject always on");
    EXPECT(metric(buf, "imud_nis_accel_ratio", &v) && fabs(v - 7.25) < 1e-4,
           "nis_accel always on");
    EXPECT(metric(buf, "imud_nis_mag_ratio", &v) && fabs(v - 1.10) < 1e-4,
           "nis_mag always on");
    EXPECT(metric(buf, "imud_mag_residual_radians", &v) && fabs(v - 0.02) < 1e-4,
           "mag residual rad");
    EXPECT(metric(buf, "imud_gyro_bias_y_radians_per_second", &v) &&
           fabs(v + 0.002) < 1e-5, "gyro bias y");
    EXPECT(metric(buf, "imud_temperature_celsius", &v) && fabs(v - 31.4) < 1e-2,
           "temperature C");
    end(fb);
}

static void test_flags_and_gating(void)
{
    begin("test_flags_and_gating");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[8192];
    double v;

    /* No declination → true heading omitted entirely. */
    d.flags = 0;
    prom_build_metrics(buf, sizeof buf, &d, 1);
    EXPECT(!metric(buf, "imud_heading_true_degrees", &v),
           "true heading omitted without declination");
    EXPECT(metric(buf, "imud_engine_on", &v) && v == 0.0, "engine_on 0");
    EXPECT(metric(buf, "imud_wave_valid", &v) && v == 0.0, "wave_valid 0");

    d.flags = IMUD_FLAG_ENGINE_ON | IMUD_FLAG_WAVE_VALID | IMUD_FLAG_HEAVE_VALID
            | IMUD_FLAG_MAG_VALID | IMUD_FLAG_FUSION_CONVERGED;
    prom_build_metrics(buf, sizeof buf, &d, 1);
    EXPECT(metric(buf, "imud_engine_on", &v)   && v == 1.0, "engine_on 1");
    EXPECT(metric(buf, "imud_wave_valid", &v)  && v == 1.0, "wave_valid 1");
    EXPECT(metric(buf, "imud_heave_valid", &v) && v == 1.0, "heave_valid 1");
    EXPECT(metric(buf, "imud_mag_valid", &v)   && v == 1.0, "mag_valid 1");
    EXPECT(metric(buf, "imud_converged", &v)   && v == 1.0, "converged 1");
    end(fb);
}

static void test_help_type(void)
{
    begin("test_help_type");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[8192];
    prom_build_metrics(buf, sizeof buf, &d, 1);
    EXPECT(strstr(buf, "# HELP imud_wave_height_meters ") != NULL, "HELP present");
    EXPECT(strstr(buf, "# TYPE imud_wave_height_meters gauge") != NULL, "TYPE gauge");
    EXPECT(strstr(buf, "# TYPE imud_packets_total counter") != NULL, "TYPE counter");
    end(fb);
}

static void test_buffer_too_small(void)
{
    begin("test_buffer_too_small");
    int fb = g_fail;

    imud_data_t d = make_data();
    char buf[64];
    EXPECT(prom_build_metrics(buf, sizeof buf, &d, 1) == -1,
           "returns -1 on truncation");
    EXPECT(prom_build_metrics(NULL, 0, &d, 1) == -1, "returns -1 on NULL buf");
    end(fb);
}

int main(void)
{
    puts("=== imud prometheus builder tests ===");
    test_no_packet();
    test_values_si();
    test_flags_and_gating();
    test_help_type();
    test_buffer_too_small();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
