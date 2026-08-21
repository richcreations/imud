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
#include <unistd.h>
#include "cal.h"
#include "cal_capture.h"
#include "capture.h"

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabsf((float)(a) - (float)(b)) < (float)(eps), msg)

#define EXPECT_NEAR_D(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

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

/* noise + gyro_temp sections round-trip alongside the classic three. */
static void test_round_trip_noise_temp(void)
{
    begin("test_round_trip_noise_temp");
    int fb = g_fail;
    const char *path = tmppath(9);

    imud_cal_t w;
    memset(&w, 0, sizeof(w));
    w.gyro_bias[0] = 0.001f;
    w.has_gyro = true;
    w.gyro_noise_density[0]    = 6.1e-5f;
    w.gyro_noise_density[1]    = 6.5e-5f;
    w.gyro_noise_density[2]    = 7.2e-5f;
    w.gyro_bias_instability[0] = 2.0e-6f;
    w.gyro_bias_instability[1] = 2.2e-6f;
    w.gyro_bias_instability[2] = 1.8e-6f;
    w.accel_noise_density[0]   = 1.9e-3f;
    w.accel_noise_density[1]   = 2.1e-3f;
    w.accel_noise_density[2]   = 2.4e-3f;
    w.has_noise = true;
    w.gyro_temp_coeff[0] =  3.0e-5f;
    w.gyro_temp_coeff[1] = -1.5e-5f;
    w.gyro_temp_coeff[2] =  8.0e-6f;
    w.gyro_temp_ref_c    = 25.0f;
    w.has_gyro_temp = true;
    EXPECT(cal_write(path, &w) == 0, "cal_write returns 0");

    imud_cal_t r;
    EXPECT(cal_load(path, &r) == 0, "cal_load returns 0");
    EXPECT(r.has_gyro && r.has_noise && r.has_gyro_temp,
           "all three flags set");
    EXPECT(!r.has_mag && !r.has_accel, "absent sections stay absent");
    EXPECT_NEAR(r.gyro_noise_density[2],    6.9e-5f, 1e-5f,  "gyro density z (loose)");
    EXPECT_NEAR(r.gyro_noise_density[0],    w.gyro_noise_density[0],    1e-9f, "gyro density x");
    EXPECT_NEAR(r.gyro_bias_instability[1], w.gyro_bias_instability[1], 1e-9f, "instability y");
    EXPECT_NEAR(r.accel_noise_density[2],   w.accel_noise_density[2],   1e-7f, "accel density z");
    EXPECT_NEAR(r.gyro_temp_coeff[1],       w.gyro_temp_coeff[1],       1e-9f, "temp coeff y");
    EXPECT_NEAR(r.gyro_temp_ref_c,          25.0f,                      1e-3f, "ref_c scalar");
    remove(path);
    end(fb);
}

/*
 * Write raw JSON text.  Every other cal.json test round-trips through
 * cal_write, which by construction can only produce well-formed files — so
 * malformed input needs its own helper.
 */
static const char *write_tmpjson(int id, const char *content)
{
    const char *path = tmppath(id);
    FILE *f = fopen(path, "w");
    if (!f) { perror("fopen tmp"); exit(1); }
    fputs(content, f);
    fclose(f);
    return path;
}

/*
 * A cal.json value must be finite.
 *
 * parse_float_array and parse_scalar used strtof and checked only that some
 * digits were consumed, and strtof converts "nan" and "inf" without complaint.
 * cal->gyro_bias goes straight into mekf_init's f->bias, so a single "nan"
 * here made w = gyro - bias non-finite on every predict step.  q_normalize
 * cannot repair that — NaN fails its `n > 1e-10f` test, so the quaternion is
 * left exactly as it arrived — and nothing downstream re-aligns.  Measured:
 * 1000 perfectly level, noiseless samples later, the published roll, pitch,
 * yaw and quaternion were still all NaN.
 *
 * A calibration file is operator-owned, so this is not an attack surface; the
 * realistic trigger is imud's own tooling.  cal_write formats with %.8f, which
 * renders a NaN as the literal "nan", so a degenerate fit wrote a file the
 * daemon then loaded without complaint.  Both halves of that loop are closed:
 * this test covers the read side, test_write_rejects_non_finite the write.
 */
static void test_load_rejects_non_finite(void)
{
    begin("test_load_rejects_non_finite");
    int fb = g_fail;

    /* Hoisted out of the table below: written inline it spans two literals,
     * which -Wstring-concatenation reads as a missing comma. */
    static const char soft_iron_nan[] =
        "{\"mag\": {\"hard_iron\": [0,0,0], "
        "\"soft_iron\": [[1,0,0],[0,nan,0],[0,0,1]]}}\n";

    /* One case per parse path: every array-valued section, plus the one
     * scalar (gyro_temp.ref_c), plus the 9-element nested soft-iron. */
    static const char *bodies[] = {
        "{\"gyro\": {\"bias\": [nan, 0.0, 0.0]}}\n",
        "{\"gyro\": {\"bias\": [0.0, inf, 0.0]}}\n",
        "{\"accel\": {\"offset\": [0.0, 0.0, -inf], \"scale\": [1,1,1]}}\n",
        "{\"accel\": {\"offset\": [0,0,0], \"scale\": [1, nan, 1]}}\n",
        "{\"mag\": {\"hard_iron\": [1e999, 0.0, 0.0]}}\n",
        soft_iron_nan,
        "{\"noise\": {\"gyro_density\": [nan, 1e-5, 1e-5]}}\n",
        "{\"gyro_temp\": {\"coeff\": [0,0,0], \"ref_c\": nan}}\n",
    };

    int id = 20;
    for (unsigned i = 0; i < sizeof bodies / sizeof bodies[0]; i++) {
        const char *path = write_tmpjson(id++, bodies[i]);
        imud_cal_t cal;
        EXPECT(cal_load(path, &cal) < 0, "non-finite cal.json rejected");
        remove(path);
    }

    /* The same shapes with finite values still load. */
    const char *ok = write_tmpjson(id,
        "{\"gyro\": {\"bias\": [0.001, -0.002, 0.003]},\n"
        " \"gyro_temp\": {\"coeff\": [1e-5, 2e-5, 3e-5], \"ref_c\": 25.0}}\n");
    imud_cal_t cal;
    EXPECT(cal_load(ok, &cal) == 0, "finite cal.json accepted");
    EXPECT(cal.has_gyro,            "finite gyro section applied");
    EXPECT_NEAR(cal.gyro_bias[1], -0.002f, 1e-6f, "finite bias value landed");
    EXPECT_NEAR(cal.gyro_temp_ref_c, 25.0f, 1e-6f, "finite scalar landed");
    remove(ok);
    end(fb);
}

/*
 * cal_write must refuse to emit a non-finite value, and must leave any
 * existing file untouched when it does.
 *
 * The ordering is the whole point: fcreate opens "w", which truncates.  A
 * check placed after the open would destroy a good calibration on its way to
 * reporting the failure — the operator would lose the file that was still
 * correct.  So the validation runs first, before anything is opened.
 */
static void test_write_rejects_non_finite(void)
{
    begin("test_write_rejects_non_finite");
    int fb = g_fail;
    const char *path = tmppath(30);

    /* A known-good calibration on disk first. */
    imud_cal_t good;
    memset(&good, 0, sizeof(good));
    good.gyro_bias[0] = 0.00125f;
    good.gyro_bias[1] = 0.00250f;
    good.gyro_bias[2] = 0.00375f;
    good.has_gyro = true;
    EXPECT(cal_write(path, &good) == 0, "good calibration written");

    /* Now a fit that produced a NaN — as allan_characterize can, since its
     * minimum search uses < and every comparison against NaN is false. */
    imud_cal_t bad = good;
    bad.gyro_bias[1] = NAN;
    EXPECT(cal_write(path, &bad) < 0, "cal_write refuses a NaN field");

    imud_cal_t inf_cal = good;
    inf_cal.gyro_bias[2] = INFINITY;
    EXPECT(cal_write(path, &inf_cal) < 0, "cal_write refuses an inf field");

    /* Only fields belonging to a section with has_* set are inspected. */
    imud_cal_t inert = good;
    inert.mag_hard_iron[0] = NAN;   /* has_mag is false */
    EXPECT(cal_write(path, &inert) == 0,
           "a NaN in an unwritten section is not an error");

    /* The file on disk must still be the good one, not truncated. */
    imud_cal_t back;
    EXPECT(cal_load(path, &back) == 0, "existing file still loads");
    EXPECT(back.has_gyro,              "existing file still has its section");
    EXPECT_NEAR(back.gyro_bias[1], 0.00250f, 1e-7f,
                "rejected write left the good value intact");
    remove(path);
    end(fb);
}

/*
 * Finite is not the same as usable.
 *
 * The non-finite guard above stops a NaN or an infinity being *stored*. It
 * says nothing about a finite value so large that applying it overflows, and
 * that gap was reachable with the most ordinary reading a magnetometer can
 * produce. fuzz_cal found it on main, in CI, against v1.8.1:
 * mag.soft_iron[0][0] = 3.33e37 is finite and loaded clean, and the mag
 * correction soft_iron × (sample − hard_iron) for a sample of ZERO is
 * 3.33e37 × −11.4 = −3.8e38, past the 3.4e38 a float holds. The filter got
 * −inf from a calibration cal_load had called good.
 *
 * The guard deliberately does NOT judge whether a value is physically
 * plausible — that is a sensor-domain question, and a bound set too tight
 * rejects a real calibration, which is the worse failure. It asks only
 * whether the arithmetic survives an ordinary sample, so a calibration has to
 * be some thirty-five orders of magnitude out to trip it. Hence the second
 * half of this test: values that are merely large must still load.
 */
static void test_rejects_finite_but_overflowing(void)
{
    begin("test_rejects_finite_but_overflowing");
    int fb = g_fail;

    /* The reproducer from CI, reduced: finite soft iron, ordinary offset.
     * Hoisted out of the table for the same reason as soft_iron_nan above —
     * inline it spans two literals and -Wstring-concatenation reads that as a
     * missing comma. */
    static const char mag_overflow[] =
        "{\"mag\": {\"hard_iron\": [11.42, 0, 0], "
        "\"soft_iron\": [[3.33e37,0,0],[0,1,0],[0,0,1]]}}\n";

    static const char *bodies[] = {
        mag_overflow,
        "{\"accel\": {\"offset\": [1e38, 0, 0], \"scale\": [1e38, 1, 1]}}\n",
        "{\"gyro_temp\": {\"coeff\": [1e38, 0, 0], \"ref_c\": -1e38}}\n",
    };
    int id = 40;
    for (unsigned i = 0; i < sizeof bodies / sizeof bodies[0]; i++) {
        const char *path = write_tmpjson(id++, bodies[i]);
        imud_cal_t cal;
        EXPECT(cal_load(path, &cal) < 0,
               "finite-but-overflowing cal.json rejected");
        remove(path);
    }

    /* Large yet harmless must still load: the guard must not become a range
     * check by the back door. A soft-iron element of 1000 is nonsense for a
     * dimensionless matrix that lives near 1.0, but it does not overflow, and
     * deciding it is nonsense is not this function's job. */
    static const char mag_large[] =
        "{\"mag\": {\"hard_iron\": [500, -500, 500], "
        "\"soft_iron\": [[1000,0,0],[0,1000,0],[0,0,1000]]}}\n";
    const char *ok = write_tmpjson(id++, mag_large);
    imud_cal_t cal;
    EXPECT(cal_load(ok, &cal) == 0, "large but non-overflowing cal accepted");
    EXPECT(cal.has_mag,             "large mag section applied");
    remove(ok);

    /* And cal_write refuses to emit one, closing the loop inside imud's own
     * tooling exactly as the non-finite check does. */
    const char *path = tmppath(45);
    imud_cal_t good;
    memset(&good, 0, sizeof(good));
    good.gyro_bias[0] = 0.001f;
    good.has_gyro = true;
    EXPECT(cal_write(path, &good) == 0, "good calibration written");

    imud_cal_t over = good;
    over.has_mag = true;
    over.mag_hard_iron[0]    = 11.42f;
    over.mag_soft_iron[0][0] = 3.33e37f;
    over.mag_soft_iron[1][1] = 1.0f;
    over.mag_soft_iron[2][2] = 1.0f;
    EXPECT(cal_write(path, &over) < 0, "cal_write refuses an overflowing cal");

    imud_cal_t back;
    EXPECT(cal_load(path, &back) == 0, "existing file still loads");
    EXPECT_NEAR(back.gyro_bias[0], 0.001f, 1e-7f,
                "rejected write left the good value intact");
    remove(path);
    end(fb);
}

/* ── cal_capture: .imucap loading for the offline analysis modes ─────────── */

/*
 * `imud-cal characterize` and `imud-cal fit-temp` both begin here: count the
 * IMU records, measure the span, block-average into per-axis arrays and skip a
 * startup settle window.  It is a file parser with arithmetic in it, and until
 * it moved to src/cal_capture.c it lived in cal_main.c where no test binary
 * could reach it.
 *
 * Fixtures are built with the real cap_writer, exactly as test_capture does —
 * the point is to exercise the reader against files the daemon could actually
 * have produced.
 */

static const char *cappath(int id)
{
    static char path[80];
    snprintf(path, sizeof path, "/tmp/imud_test_calcap_%d_%d.imucap",
             (int)getpid(), id);
    return path;
}

/*
 * n IMU records at `hz`, gyro X = gyro_of(i) on every axis, |accel| = 9.80665
 * straight down, temperature ramping 20 °C + i * temp_step.
 */
static void write_cap(const char *path, int n, double hz,
                      double (*gyro_of)(int), double temp_step)
{
    cap_writer_t w;
    if (cap_writer_open(&w, path, (int)hz, (uint32_t)(hz * 1000), "sim", "sim",
                        "1.9", 0, 0) != 0) {
        fprintf(stderr, "  (could not create %s)\n", path);
        return;
    }
    for (int i = 0; i < n; i++) {
        imu_sample_t s;
        memset(&s, 0, sizeof s);
        double g = gyro_of ? gyro_of(i) : 0.0;
        for (int a = 0; a < 3; a++) s.gyro[a] = (float)g;
        s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = 9.80665f;
        s.temp_c   = (float)(20.0 + i * temp_step);
        cap_writer_imu(&w, &s, (uint64_t)((double)i / hz * 1e9));
    }
    cap_writer_close(&w);
}

static double gyro_zero(int i)  { (void)i; return 0.0; }
static double gyro_alt(int i)   { return (i % 2) ? -1.0 : 1.0; }
static double gyro_ramp(int i)  { return i * 0.01; }

static void test_calcap_basic(void)
{
    begin("test_calcap_basic");
    int fb = g_fail;

    const char *path = cappath(1);
    write_cap(path, 100, 100.0, gyro_zero, 0.0);

    double *gyro[3], *accel[3], *temp;
    cal_capture_stats_t st;
    int cap_rc = 0;
    long n = cal_capture_load(path, 0.0, 0, gyro, accel, &temp, &st, &cap_rc);

    EXPECT(n == 100, "100 records in, 100 samples out");
    EXPECT(st.count == 100, "record count reported");
    EXPECT(st.decim == 1, "no decimation needed");
    EXPECT(st.n_skip == 0, "nothing skipped without a settle window");
    /* fs is (count - 1) / span: 99 intervals over 0.99 s. */
    EXPECT_NEAR_D(st.fs_raw, 100.0, 0.01, "sample rate recovered");
    EXPECT_NEAR_D(st.fs_out, st.fs_raw, 1e-9, "fs_out == fs_raw when decim is 1");
    EXPECT_NEAR_D(st.span_s, 0.99, 0.01, "span measured");
    EXPECT(cap_rc == 0, "no reader error");

    /* The payload actually arrived, not just the count. */
    EXPECT_NEAR_D(accel[2][0], 9.80665, 1e-4, "accel Z carried through");
    EXPECT_NEAR_D(temp[0], 20.0, 1e-4, "temperature carried through");

    cal_capture_free(gyro, accel, temp);
    remove(path);
    end(fb);
}

static void test_calcap_settle(void)
{
    begin("test_calcap_settle");
    int fb = g_fail;

    /* 200 records at 100 Hz = 2 s; skip the first second.  Temperature ramps
     * 0.5 °C per sample, so which records survived is checkable by value. */
    const char *path = cappath(2);
    write_cap(path, 200, 100.0, gyro_zero, 0.5);

    double *gyro[3], *accel[3], *temp;
    cal_capture_stats_t st;
    int cap_rc = 0;
    long n = cal_capture_load(path, 1.0, 0, gyro, accel, &temp, &st, &cap_rc);

    EXPECT(st.n_skip == 100, "one second of records skipped");
    EXPECT(n == 100, "the rest are returned");
    /* Record 100 has temp 20 + 100*0.5 = 70 — the samples kept are the LATER
     * ones, which is the whole point of the settle window. */
    EXPECT_NEAR_D(temp[0], 70.0, 1e-3, "first surviving sample is post-settle");

    cal_capture_free(gyro, accel, temp);

    /* A settle window longer than the record leaves nothing.  n_out is 0 and
     * the arrays are still safe to free. */
    n = cal_capture_load(path, 10.0, 0, gyro, accel, &temp, &st, &cap_rc);
    EXPECT(n == 0, "settle longer than the record yields nothing");
    cal_capture_free(gyro, accel, temp);

    remove(path);
    end(fb);
}

static void test_calcap_decimation(void)
{
    begin("test_calcap_decimation");
    int fb = g_fail;

    /* Gyro alternates +1 / -1, so a block average over an even factor is
     * exactly zero — which is what proves the averaging runs rather than the
     * samples merely being subsampled. */
    const char *path = cappath(3);
    write_cap(path, 100, 100.0, gyro_alt, 0.0);

    double *gyro[3], *accel[3], *temp;
    cal_capture_stats_t st;
    int cap_rc = 0;
    /* max_samples 25 → decim = 100/25 + 1 = 5. */
    long n = cal_capture_load(path, 0.0, 25, gyro, accel, &temp, &st, &cap_rc);

    EXPECT(st.decim == 5, "decimation factor from max_samples");
    EXPECT(n == 20, "count / decim samples out");
    EXPECT_NEAR_D(st.fs_out, st.fs_raw / 5.0, 1e-6, "fs_out scaled by decim");
    /* Blocks of 5 from +1,-1,... average to +1/5. */
    EXPECT_NEAR_D(gyro[0][0], 0.2, 1e-9, "block average, not subsample");
    EXPECT_NEAR_D(accel[2][0], 9.80665, 1e-4, "constant survives averaging");

    cal_capture_free(gyro, accel, temp);

    /* max_samples 0 selects the production cap, which no test file reaches. */
    n = cal_capture_load(path, 0.0, 0, gyro, accel, &temp, &st, &cap_rc);
    EXPECT(n == 100 && st.decim == 1, "max_samples 0 → CAP_ANALYZE_MAX");
    cal_capture_free(gyro, accel, temp);

    remove(path);
    end(fb);
}

static void test_calcap_rejects(void)
{
    begin("test_calcap_rejects");
    int fb = g_fail;

    double *gyro[3], *accel[3], *temp;
    cal_capture_stats_t st;
    int cap_rc = 0;

    /* A file that does not exist. */
    long n = cal_capture_load("/nonexistent/imud_no_such.imucap", 0.0, 0,
                              gyro, accel, &temp, &st, &cap_rc);
    EXPECT(n == -1 && cap_rc != 0, "missing file → error with a reader code");
    cal_capture_free(gyro, accel, temp);   /* must be safe after the error */

    /* Something that is not an .imucap at all. */
    const char *junk = cappath(4);
    FILE *f = fopen(junk, "wb");
    if (f) { fputs("this is not a capture file at all, not even close", f); fclose(f); }
    n = cal_capture_load(junk, 0.0, 0, gyro, accel, &temp, &st, &cap_rc);
    EXPECT(n == -1 && cap_rc == CAP_ERR_FORMAT, "not an .imucap → CAP_ERR_FORMAT");
    cal_capture_free(gyro, accel, temp);
    remove(junk);

    /* Fewer than 16 IMU records is "too little data", not an error: the
     * distinction is what lets cal_main print a useful message. */
    const char *few = cappath(5);
    write_cap(few, 8, 100.0, gyro_zero, 0.0);
    n = cal_capture_load(few, 0.0, 0, gyro, accel, &temp, &st, &cap_rc);
    EXPECT(n == 0 && cap_rc == 0, "under 16 records → 0, not -1");
    EXPECT(st.count == 8, "and the count is reported so the caller can say so");
    cal_capture_free(gyro, accel, temp);
    remove(few);

    /* A valid capture holding only MAG records — the count-is-zero path. */
    const char *magonly = cappath(6);
    cap_writer_t w;
    if (cap_writer_open(&w, magonly, 100, 100000, "sim", "sim", "1.9", 0, 0) == 0) {
        for (int i = 0; i < 50; i++) {
            mag_sample_t m;
            memset(&m, 0, sizeof m);
            cap_writer_mag(&w, &m, (uint64_t)i * 10000000ULL);
        }
        cap_writer_close(&w);
        n = cal_capture_load(magonly, 0.0, 0, gyro, accel, &temp, &st, &cap_rc);
        EXPECT(n == 0 && st.count == 0, "mag-only capture → no IMU data");
        cal_capture_free(gyro, accel, temp);
    }
    remove(magonly);

    end(fb);
}

static void test_calcap_motion_gate(void)
{
    begin("test_calcap_motion_gate");
    int fb = g_fail;

    const char *path = cappath(7);
    double *gyro[3], *accel[3], *temp;
    cal_capture_stats_t st;
    int cap_rc = 0;
    double gpp, astd;

    /* Stationary: gyro flat, |a| constant at 1 g. */
    write_cap(path, 100, 100.0, gyro_zero, 0.0);
    cal_capture_load(path, 0.0, 0, gyro, accel, &temp, &st, &cap_rc);
    EXPECT(cal_capture_motion_ok(gyro, accel, 100, &gpp, &astd), "flat record is stationary");
    EXPECT_NEAR_D(gpp,  0.0, 1e-9, "gyro peak-to-peak is zero");
    EXPECT_NEAR_D(astd, 0.0, 1e-3, "|a| standard deviation is zero");
    cal_capture_free(gyro, accel, temp);
    remove(path);

    /* Moving: the gyro ramps well past the 0.1 rad/s peak-to-peak threshold. */
    write_cap(path, 100, 100.0, gyro_ramp, 0.0);
    cal_capture_load(path, 0.0, 0, gyro, accel, &temp, &st, &cap_rc);
    EXPECT(!cal_capture_motion_ok(gyro, accel, 100, &gpp, &astd), "ramped gyro is not stationary");
    EXPECT_NEAR_D(gpp, 0.99, 1e-6, "peak-to-peak reported for the message");
    cal_capture_free(gyro, accel, temp);
    remove(path);

    /* Right at the threshold: p-p of exactly 0.1 must pass (the test is >). */
    double *g2[3], *a2[3];
    double buf_g[3][2] = { { 0.0, 0.1 }, { 0.0, 0.1 }, { 0.0, 0.1 } };
    double buf_a[3][2] = { { 0, 0 }, { 0, 0 }, { 9.80665, 9.80665 } };
    for (int a = 0; a < 3; a++) { g2[a] = buf_g[a]; a2[a] = buf_a[a]; }
    EXPECT(cal_capture_motion_ok(g2, a2, 2, &gpp, &astd),
           "exactly 0.1 rad/s p-p is still stationary");
    buf_g[0][1] = 0.1001;
    EXPECT(!cal_capture_motion_ok(g2, a2, 2, &gpp, &astd),
           "just over 0.1 rad/s p-p is not");

    /* The peak-to-peak spans all three axes, not just X. */
    buf_g[0][1] = 0.0; buf_g[2][1] = 0.5;
    EXPECT(!cal_capture_motion_ok(g2, a2, 2, &gpp, &astd) && gpp > 0.49,
           "motion on Z alone is detected");

    end(fb);
}

int main(void)
{
    puts("=== imud cal tests ===");

    test_missing_file();
    test_round_trip_gyro();
    test_round_trip_accel();
    test_round_trip_mag_full_soft_iron();
    test_round_trip_all_sections();
    test_partial_section_defaults();
    test_round_trip_noise_temp();
    test_load_rejects_non_finite();
    test_write_rejects_non_finite();
    test_rejects_finite_but_overflowing();

    test_calcap_basic();
    test_calcap_settle();
    test_calcap_decimation();
    test_calcap_rejects();
    test_calcap_motion_gate();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
