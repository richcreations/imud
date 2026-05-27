/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_cal.c — unit tests for cal_load() and cal_write() (src/cal.c)
 *
 * Tests exercise the JSON round-trip, default initialisation, partial sections,
 * and the full 3×3 soft-iron matrix (regression for the nested-array parse bug
 * where only the first row was recovered).
 *
 * Note: cal_load logs to stderr; that output is expected and harmless.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "cal.h"

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabsf((float)(a) - (float)(b)) < (float)(eps), msg)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static const char *tmppath(int id)
{
    static char path[64];
    snprintf(path, sizeof(path), "/tmp/imud_test_cal_%d.json", id);
    return path;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

/* Missing file: returns 0, all has_* false, safe identity defaults. */
static void test_missing_file(void)
{
    begin("test_missing_file");
    int fb = g_fail;

    imud_cal_t cal;
    int rc = cal_load("/tmp/imud_no_such_cal_file_xyz.json", &cal);
    EXPECT(rc == 0,           "missing file returns 0");
    EXPECT(!cal.has_accel,    "has_accel false");
    EXPECT(!cal.has_gyro,     "has_gyro false");
    EXPECT(!cal.has_mag,      "has_mag false");
    /* Accel scale must default to 1 so apply_accel_cal is a no-op. */
    EXPECT_NEAR(cal.accel_scale[0], 1.0f, 1e-6f, "accel_scale[0] defaults to 1");
    EXPECT_NEAR(cal.accel_scale[1], 1.0f, 1e-6f, "accel_scale[1] defaults to 1");
    EXPECT_NEAR(cal.accel_scale[2], 1.0f, 1e-6f, "accel_scale[2] defaults to 1");
    /* Soft-iron must default to identity so apply_mag_cal is a no-op. */
    EXPECT_NEAR(cal.mag_soft_iron[0][0], 1.0f, 1e-6f, "soft_iron[0][0] = 1");
    EXPECT_NEAR(cal.mag_soft_iron[1][1], 1.0f, 1e-6f, "soft_iron[1][1] = 1");
    EXPECT_NEAR(cal.mag_soft_iron[2][2], 1.0f, 1e-6f, "soft_iron[2][2] = 1");
    EXPECT_NEAR(cal.mag_soft_iron[0][1], 0.0f, 1e-6f, "soft_iron[0][1] = 0");

    end(fb);
}

/* Gyro-only round-trip. */
static void test_round_trip_gyro(void)
{
    begin("test_round_trip_gyro");
    int fb = g_fail;
    const char *path = tmppath(1);

    imud_cal_t w;
    memset(&w, 0, sizeof(w));
    w.gyro_bias[0] =  0.00312f;
    w.gyro_bias[1] = -0.00128f;
    w.gyro_bias[2] =  0.00051f;
    w.has_gyro = true;
    EXPECT(cal_write(path, &w) == 0, "cal_write gyro returns 0");

    imud_cal_t r;
    EXPECT(cal_load(path, &r) == 0, "cal_load gyro returns 0");
    EXPECT(r.has_gyro,  "has_gyro true");
    EXPECT(!r.has_accel, "has_accel false");
    EXPECT(!r.has_mag,   "has_mag false");
    EXPECT_NEAR(r.gyro_bias[0], w.gyro_bias[0], 1e-6f, "bias[0]");
    EXPECT_NEAR(r.gyro_bias[1], w.gyro_bias[1], 1e-6f, "bias[1]");
    EXPECT_NEAR(r.gyro_bias[2], w.gyro_bias[2], 1e-6f, "bias[2]");
    remove(path);
    end(fb);
}

/* Accel-only round-trip. */
static void test_round_trip_accel(void)
{
    begin("test_round_trip_accel");
    int fb = g_fail;
    const char *path = tmppath(2);

    imud_cal_t w;
    memset(&w, 0, sizeof(w));
    w.accel_offset[0] =  0.031f;
    w.accel_offset[1] = -0.012f;
    w.accel_offset[2] =  0.007f;
    w.accel_scale[0]  =  1.003f;
    w.accel_scale[1]  =  0.997f;
    w.accel_scale[2]  =  1.001f;
    w.has_accel = true;
    EXPECT(cal_write(path, &w) == 0, "cal_write accel returns 0");

    imud_cal_t r;
    EXPECT(cal_load(path, &r) == 0, "cal_load accel returns 0");
    EXPECT(r.has_accel,  "has_accel true");
    EXPECT(!r.has_gyro,  "has_gyro false");
    EXPECT(!r.has_mag,   "has_mag false");
    EXPECT_NEAR(r.accel_offset[0], w.accel_offset[0], 1e-5f, "offset[0]");
    EXPECT_NEAR(r.accel_offset[1], w.accel_offset[1], 1e-5f, "offset[1]");
    EXPECT_NEAR(r.accel_offset[2], w.accel_offset[2], 1e-5f, "offset[2]");
    EXPECT_NEAR(r.accel_scale[0],  w.accel_scale[0],  1e-6f, "scale[0]");
    EXPECT_NEAR(r.accel_scale[1],  w.accel_scale[1],  1e-6f, "scale[1]");
    EXPECT_NEAR(r.accel_scale[2],  w.accel_scale[2],  1e-6f, "scale[2]");
    remove(path);
    end(fb);
}

/*
 * Mag round-trip with a non-trivial soft-iron matrix.
 * All 9 elements must survive the write → read cycle, including off-diagonal
 * terms.  This is a regression test for the nested-array parse bug where only
 * the first row (3 values) was ever recovered.
 */
static void test_round_trip_mag_full_soft_iron(void)
{
    begin("test_round_trip_mag_full_soft_iron");
    int fb = g_fail;
    const char *path = tmppath(3);

    imud_cal_t w;
    memset(&w, 0, sizeof(w));
    w.mag_hard_iron[0] =  12.5f;
    w.mag_hard_iron[1] =  -8.3f;
    w.mag_hard_iron[2] =   3.1f;
    /* Non-diagonal soft-iron matrix */
    w.mag_soft_iron[0][0] =  1.05f; w.mag_soft_iron[0][1] =  0.03f; w.mag_soft_iron[0][2] = -0.02f;
    w.mag_soft_iron[1][0] =  0.03f; w.mag_soft_iron[1][1] =  0.97f; w.mag_soft_iron[1][2] =  0.01f;
    w.mag_soft_iron[2][0] = -0.02f; w.mag_soft_iron[2][1] =  0.01f; w.mag_soft_iron[2][2] =  1.03f;
    w.has_mag = true;
    EXPECT(cal_write(path, &w) == 0, "cal_write mag returns 0");

    imud_cal_t r;
    EXPECT(cal_load(path, &r) == 0, "cal_load mag returns 0");
    EXPECT(r.has_mag,   "has_mag true");
    EXPECT(!r.has_gyro,  "has_gyro false");
    EXPECT(!r.has_accel, "has_accel false");

    EXPECT_NEAR(r.mag_hard_iron[0], w.mag_hard_iron[0], 1e-4f, "hard_iron[0]");
    EXPECT_NEAR(r.mag_hard_iron[1], w.mag_hard_iron[1], 1e-4f, "hard_iron[1]");
    EXPECT_NEAR(r.mag_hard_iron[2], w.mag_hard_iron[2], 1e-4f, "hard_iron[2]");

    /* All 9 soft-iron elements — the critical regression check. */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            char msg[64];
            snprintf(msg, sizeof(msg), "soft_iron[%d][%d]", i, j);
            EXPECT_NEAR(r.mag_soft_iron[i][j], w.mag_soft_iron[i][j], 1e-6f, msg);
        }

    remove(path);
    end(fb);
}

/* All three sections together. */
static void test_round_trip_all_sections(void)
{
    begin("test_round_trip_all_sections");
    int fb = g_fail;
    const char *path = tmppath(4);

    imud_cal_t w;
    memset(&w, 0, sizeof(w));

    w.mag_hard_iron[0] = 5.0f; w.mag_hard_iron[1] = -3.0f; w.mag_hard_iron[2] = 1.5f;
    w.mag_soft_iron[0][0] = 1.0f; w.mag_soft_iron[1][1] = 1.0f; w.mag_soft_iron[2][2] = 1.0f;
    w.has_mag = true;

    w.gyro_bias[0] = 0.001f; w.gyro_bias[1] = -0.002f; w.gyro_bias[2] = 0.0005f;
    w.has_gyro = true;

    w.accel_offset[0] = 0.05f; w.accel_offset[1] = -0.03f; w.accel_offset[2] = 0.01f;
    w.accel_scale[0] = 1.002f; w.accel_scale[1] = 0.998f; w.accel_scale[2] = 1.001f;
    w.has_accel = true;

    EXPECT(cal_write(path, &w) == 0, "cal_write all returns 0");

    imud_cal_t r;
    EXPECT(cal_load(path, &r) == 0, "cal_load all returns 0");
    EXPECT(r.has_mag   && r.has_gyro && r.has_accel, "all three sections present");
    EXPECT_NEAR(r.mag_hard_iron[0],  w.mag_hard_iron[0],  1e-4f, "hard_iron[0]");
    EXPECT_NEAR(r.gyro_bias[1],      w.gyro_bias[1],      1e-6f, "gyro_bias[1]");
    EXPECT_NEAR(r.accel_offset[0],   w.accel_offset[0],   1e-4f, "accel_offset[0]");
    EXPECT_NEAR(r.accel_scale[2],    w.accel_scale[2],    1e-6f, "accel_scale[2]");

    remove(path);
    end(fb);
}

/* Write mag only; unset sections must load with safe defaults, not garbage. */
static void test_partial_section_defaults(void)
{
    begin("test_partial_section_defaults");
    int fb = g_fail;
    const char *path = tmppath(5);

    imud_cal_t w;
    memset(&w, 0, sizeof(w));
    w.mag_hard_iron[0] = 10.0f;
    w.mag_soft_iron[0][0] = w.mag_soft_iron[1][1] = w.mag_soft_iron[2][2] = 1.0f;
    w.has_mag = true;
    cal_write(path, &w);

    imud_cal_t r;
    cal_load(path, &r);
    EXPECT(!r.has_gyro,  "has_gyro false when not in file");
    EXPECT(!r.has_accel, "has_accel false when not in file");
    /* Gyro bias defaults to 0 (safe: no correction applied). */
    EXPECT_NEAR(r.gyro_bias[0], 0.0f, 1e-9f, "gyro_bias defaults to 0");
    /* Accel scale defaults to 1 (safe: no correction applied). */
    EXPECT_NEAR(r.accel_scale[0], 1.0f, 1e-6f, "accel_scale defaults to 1");
    EXPECT_NEAR(r.accel_scale[1], 1.0f, 1e-6f, "accel_scale[1] defaults to 1");
    EXPECT_NEAR(r.accel_scale[2], 1.0f, 1e-6f, "accel_scale[2] defaults to 1");

    remove(path);
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud cal tests ===");

    test_missing_file();
    test_round_trip_gyro();
    test_round_trip_accel();
    test_round_trip_mag_full_soft_iron();
    test_round_trip_all_sections();
    test_partial_section_defaults();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
