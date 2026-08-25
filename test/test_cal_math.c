/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_cal_math.c — unit tests for gauss4() and sphere_fit() (src/cal_math.c)
 *
 * Exercises the numerical routines with analytically known inputs and outputs.
 * All inputs are exact (constructed on a perfect sphere), so tolerances are
 * tight — errors larger than ~1e-6 indicate a real algorithm problem.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "cal_math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR_D(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── gauss4 tests ────────────────────────────────────────────────────────── */

/* Identity system: I·x = b → x should equal b. */
static void test_gauss4_identity(void)
{
    begin("test_gauss4_identity");
    int fb = g_fail;

    double A[4][4] = {
        {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}
    };
    double b[4] = {3.0, -1.5, 0.25, 7.0};
    double x[4];
    EXPECT(gauss4(A, b, x) == 0, "returns 0");
    EXPECT_NEAR_D(x[0], 3.0,   1e-12, "x[0]");
    EXPECT_NEAR_D(x[1], -1.5,  1e-12, "x[1]");
    EXPECT_NEAR_D(x[2], 0.25,  1e-12, "x[2]");
    EXPECT_NEAR_D(x[3], 7.0,   1e-12, "x[3]");
    end(fb);
}

/* Known system with a non-trivial solution. */
static void test_gauss4_known_solution(void)
{
    begin("test_gauss4_known_solution");
    int fb = g_fail;

    /*
     * System: x0 + x1 + x2 + x3 = 10
     *         x0 - x1           = 2
     *              x1 + x2      = 3
     *                   x2 + x3 = 5
     * Solution: x1 = 1.5, x0 = 3.5, x2 = 1.5, x3 = 3.5
     */
    double A[4][4] = {
        {1,  1,  1,  1},
        {1, -1,  0,  0},
        {0,  1,  1,  0},
        {0,  0,  1,  1},
    };
    double b[4] = {10.0, 2.0, 3.0, 5.0};
    double x[4];
    EXPECT(gauss4(A, b, x) == 0, "returns 0");
    EXPECT_NEAR_D(x[0], 3.5, 1e-10, "x[0] = 3.5");
    EXPECT_NEAR_D(x[1], 1.5, 1e-10, "x[1] = 1.5");
    EXPECT_NEAR_D(x[2], 1.5, 1e-10, "x[2] = 1.5");
    EXPECT_NEAR_D(x[3], 3.5, 1e-10, "x[3] = 3.5");
    end(fb);
}

/* Singular matrix must return -1. */
static void test_gauss4_singular(void)
{
    begin("test_gauss4_singular");
    int fb = g_fail;

    double A[4][4] = {
        {1, 2, 3, 4},
        {2, 4, 6, 8},   /* row 1 = 2 × row 0 — singular */
        {0, 1, 0, 0},
        {0, 0, 1, 0},
    };
    double b[4] = {1, 2, 3, 4};
    double x[4];
    EXPECT(gauss4(A, b, x) == -1, "singular matrix returns -1");
    end(fb);
}

/* ── sphere_fit tests ────────────────────────────────────────────────────── */

/*
 * Generate N uniformly distributed points on the sphere with the given center
 * and radius, feed them to sphere_add, then verify sphere_fit recovery.
 *
 * Uses a Fibonacci lattice on the unit sphere, scaled and translated.
 */
static void run_sphere_test(const char *name,
                            double cx, double cy, double cz, double r,
                            int n_points, double tol)
{
    begin(name);
    int fb = g_fail;

    sphere_accum_t acc;
    memset(&acc, 0, sizeof(acc));

    double golden = M_PI * (3.0 - sqrt(5.0));  /* ~2.399 rad */
    for (int i = 0; i < n_points; i++) {
        double y_unit = 1.0 - (2.0 * i) / (double)(n_points - 1);
        double rho    = sqrt(1.0 - y_unit * y_unit);
        double theta  = golden * i;
        float x = (float)(cx + r * rho * cos(theta));
        float y = (float)(cy + r * y_unit);
        float z = (float)(cz + r * rho * sin(theta));
        sphere_add(&acc, x, y, z);
    }

    double center[3], radius;
    EXPECT(sphere_fit(&acc, center, &radius) == 0, "sphere_fit returns 0");
    EXPECT_NEAR_D(center[0], cx, tol, "center x");
    EXPECT_NEAR_D(center[1], cy, tol, "center y");
    EXPECT_NEAR_D(center[2], cz, tol, "center z");
    EXPECT_NEAR_D(radius,    r,  tol, "radius");
    end(fb);
}

/* Sphere at origin, radius 1. */
static void test_sphere_fit_unit_origin(void)
{
    run_sphere_test("test_sphere_fit_unit_origin",
                    0.0, 0.0, 0.0, 1.0, 64, 1e-4);
}

/* Sphere with a realistic Earth-field hard-iron offset and field strength. */
static void test_sphere_fit_hard_iron_offset(void)
{
    /* center ~ typical hard-iron bias in µT; radius ~ typical Earth field */
    run_sphere_test("test_sphere_fit_hard_iron_offset",
                    12.5, -8.3, 3.1, 40.0, 128, 1e-3);
}

/* Large offset, larger radius. */
static void test_sphere_fit_large_offset(void)
{
    run_sphere_test("test_sphere_fit_large_offset",
                    -50.0, 30.0, -15.0, 55.0, 200, 1e-3);
}

/* Fewer than 5 points must return -1. */
static void test_sphere_fit_too_few_points(void)
{
    begin("test_sphere_fit_too_few_points");
    int fb = g_fail;

    sphere_accum_t acc;
    memset(&acc, 0, sizeof(acc));
    sphere_add(&acc, 1, 0, 0);
    sphere_add(&acc, 0, 1, 0);
    sphere_add(&acc, 0, 0, 1);
    sphere_add(&acc, -1, 0, 0);  /* 4 points — one short */
    double center[3], radius;
    EXPECT(sphere_fit(&acc, center, &radius) == -1, "< 5 points returns -1");
    end(fb);
}

/* sphere_add accumulates n correctly. */
static void test_sphere_add_count(void)
{
    begin("test_sphere_add_count");
    int fb = g_fail;

    sphere_accum_t acc;
    memset(&acc, 0, sizeof(acc));
    EXPECT(acc.n == 0, "initially n = 0");
    sphere_add(&acc, 1, 0, 0);
    EXPECT(acc.n == 1, "n = 1 after one add");
    sphere_add(&acc, 0, 1, 0);
    EXPECT(acc.n == 2, "n = 2 after two adds");
    end(fb);
}

/* ── ellipse_fit tests ───────────────────────────────────────────────────── */

/*
 * Generate points on a distorted circle m = D·c, |c| = r, with D a known
 * symmetric 2×2 (soft iron). ellipse_fit must recover S ≈ D⁻¹ so that S·m
 * lies back on the radius-r circle: check S·D ≈ I.
 */
static void gen_ellipse(ellipse_accum_t *acc, const double D[2][2],
                        double r, int n, double noise)
{
    unsigned rng = 7;
    for (int i = 0; i < n; i++) {
        double th = 2.0*M_PI*i/n;
        double cx = r*cos(th), cy = r*sin(th);
        double mx = D[0][0]*cx + D[0][1]*cy;
        double my = D[1][0]*cx + D[1][1]*cy;
        if (noise > 0) {
            rng = rng*1664525u + 1013904223u;
            mx += ((double)(int)(rng>>8)/8388608.0 - 1.0) * noise;
            rng = rng*1664525u + 1013904223u;
            my += ((double)(int)(rng>>8)/8388608.0 - 1.0) * noise;
        }
        ellipse_add(acc, (float)mx, (float)my);
    }
}

/* D = R(30°)·diag(1.2, 0.9)·R(−30°): rotated-axis distortion whose cross
 * term a diagonal fit cannot represent. */
static void make_D(double D[2][2])
{
    double c = cos(30.0*M_PI/180.0), s = sin(30.0*M_PI/180.0);
    double a = 1.2, b = 0.9;
    D[0][0] = c*c*a + s*s*b;
    D[0][1] = c*s*(a - b);
    D[1][0] = D[0][1];
    D[1][1] = s*s*a + c*c*b;
}

static void check_SD_identity(const double S[2][2], const double D[2][2],
                              double tol, const char *tag)
{
    char msg[96];
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            double sd = S[i][0]*D[0][j] + S[i][1]*D[1][j];
            snprintf(msg, sizeof msg, "%s: (S*D)[%d][%d] = I", tag, i, j);
            EXPECT_NEAR_D(sd, (i == j) ? 1.0 : 0.0, tol, msg);
        }
}

static void test_ellipse_fit_rotated(void)
{
    begin("test_ellipse_fit_rotated");
    int fb = g_fail;

    double D[2][2];
    make_D(D);
    ellipse_accum_t acc = {0};
    gen_ellipse(&acc, D, 45.0, 360, 0.0);

    double S[2][2];
    EXPECT(ellipse_fit(&acc, 45.0, S) == 0, "fit succeeds");
    EXPECT(fabs(S[0][1]) > 0.01, "cross term recovered (non-diagonal)");
    check_SD_identity(S, D, 1e-3, "exact");
    end(fb);
}

static void test_ellipse_fit_noise(void)
{
    begin("test_ellipse_fit_noise");
    int fb = g_fail;

    double D[2][2];
    make_D(D);
    ellipse_accum_t acc = {0};
    gen_ellipse(&acc, D, 45.0, 720, 0.25);   /* ±0.25 µT ≈ 0.5% noise */

    double S[2][2];
    EXPECT(ellipse_fit(&acc, 45.0, S) == 0, "noisy fit succeeds");
    check_SD_identity(S, D, 0.02, "noisy");
    end(fb);
}

static void test_ellipse_fit_circle_identity(void)
{
    begin("test_ellipse_fit_circle_identity");
    int fb = g_fail;

    double I2[2][2] = {{1, 0}, {0, 1}};
    ellipse_accum_t acc = {0};
    gen_ellipse(&acc, I2, 30.0, 360, 0.0);

    double S[2][2];
    EXPECT(ellipse_fit(&acc, 30.0, S) == 0, "circle fit succeeds");
    check_SD_identity(S, I2, 1e-3, "circle");
    end(fb);
}

static void test_ellipse_fit_degenerate(void)
{
    begin("test_ellipse_fit_degenerate");
    int fb = g_fail;

    double S[2][2];
    ellipse_accum_t few = {0};
    ellipse_add(&few, 1.0f, 0.0f);
    ellipse_add(&few, 0.0f, 1.0f);
    EXPECT(ellipse_fit(&few, 1.0, S) == -1, "too few points rejected");

    /* Collinear points: conic is degenerate, must be rejected. */
    ellipse_accum_t line = {0};
    for (int i = 0; i < 50; i++)
        ellipse_add(&line, (float)(i - 25), (float)(2.0*(i - 25)));
    EXPECT(ellipse_fit(&line, 10.0, S) == -1, "collinear points rejected");
    end(fb);
}

/* ── Centered extent + diagonal soft iron ────────────────────────────────── */

/*
 * Fill samps[n][3] with a Fibonacci lattice on a sphere of the given center
 * and radius — the same construction run_sphere_test uses, kept separate here
 * because these tests need the raw points, not just the accumulator.
 */
static void make_sphere_samples(double (*samps)[3], int n,
                                const double c[3], double r)
{
    double golden = M_PI * (3.0 - sqrt(5.0));
    for (int i = 0; i < n; i++) {
        double y_unit = 1.0 - (2.0 * i) / (double)(n - 1);
        double rho    = sqrt(1.0 - y_unit * y_unit);
        double theta  = golden * i;
        samps[i][0] = c[0] + r * rho * cos(theta);
        samps[i][1] = c[1] + r * y_unit;
        samps[i][2] = c[2] + r * rho * sin(theta);
    }
}

/* A clean swing: the half-range is the sphere radius on every axis. */
static void test_extent_full_sphere(void)
{
    begin("test_extent_full_sphere");
    int fb = g_fail;

    const double c[3] = { 12.5, -8.3, 3.1 };
    const double r    = 40.0;
    static double samps[256][3];
    make_sphere_samples(samps, 256, c, r);

    extent_accum_t acc = {0};
    for (int i = 0; i < 256; i++)
        extent_add(&acc, samps[i][0] - c[0], samps[i][1] - c[1],
                         samps[i][2] - c[2]);

    double half[3];
    EXPECT(extent_half(&acc, half) == 0, "extent_half returns 0");
    EXPECT(acc.n == 256, "all points counted");
    for (int k = 0; k < 3; k++)
        EXPECT_NEAR_D(half[k], r, 0.5, "half-range equals the radius");

    /* An undistorted sphere needs no soft-iron correction at all. */
    double si[3];
    cal_softiron_diag(half, r, si);
    for (int k = 0; k < 3; k++)
        EXPECT_NEAR_D(si[k], 1.0, 0.02, "soft iron is unity on a true sphere");
    end(fb);
}

/*
 * A diagonal soft-iron scale is only meaningful on an axis the swing actually
 * swept.  The guard used to admit any axis covering 30% of the radius, which
 * permits a 3.3x "correction" -- and no real soft iron is 3.3x.  What that
 * number actually measures is missing coverage.
 *
 * This is not hypothetical: a bench swing with incidental hand-tilt reached
 * 0.48 of the radius in Z and wrote a 2.07x Z scale, which would have doubled
 * the vertical field the moment the sensor left level.  A level swing -- the
 * procedure `imud-cal mag` asks for, and the only one a vessel or a ground
 * robot can perform -- sweeps no Z extent at all, so Z has to come out of it
 * uncorrected rather than confidently wrong.
 */
static void test_softiron_needs_real_coverage(void)
{
    begin("test_softiron_needs_real_coverage");
    int fb = g_fail;

    const double r = 50.0;
    double si[3];

    /* A fully swept axis is believed, and lands near unity on a true sphere. */
    double full[3] = { r, r, r };
    cal_softiron_diag(full, r, si);
    for (int k = 0; k < 3; k++)
        EXPECT_NEAR_D(si[k], 1.0, 1e-9, "a fully swept axis is unity");

    /* Mild real distortion, well covered: believed. */
    double mild[3] = { 0.90 * r, 0.95 * r, r };
    cal_softiron_diag(mild, r, si);
    EXPECT_NEAR_D(si[0], 1.0 / 0.90, 1e-9, "0.90R is covered and corrected");
    EXPECT_NEAR_D(si[1], 1.0 / 0.95, 1e-9, "0.95R is covered and corrected");

    /* The bench case that produced 2.07: refused. */
    double bench[3] = { r, r, 0.482 * r };
    cal_softiron_diag(bench, r, si);
    EXPECT_NEAR_D(si[2], 1.0, 1e-9,
                  "0.48R of Z coverage is refused, not scaled 2.07x");

    /* A level swing sees no Z extent whatever: refused. */
    double level[3] = { r, r, 0.0 };
    cal_softiron_diag(level, r, si);
    EXPECT_NEAR_D(si[2], 1.0, 1e-9, "a level swing leaves Z uncorrected");

    /* Nothing the guard lets through may exceed the plausible range. */
    for (int pct = 0; pct <= 100; pct++) {
        double h[3] = { pct / 100.0 * r, r, r };
        cal_softiron_diag(h, r, si);
        EXPECT(si[0] >= 1.0 && si[0] <= 1.0 / CAL_SI_MIN_SPAN + 1e-9,
               "no emitted scale exceeds 1/CAL_SI_MIN_SPAN");
    }

    end(fb);
}

/*
 * Regression: the half-range must not depend on the hard-iron offset.
 *
 * cal_main.c used to seed mn[]/mx[] from the RAW first sample and then update
 * them with CENTERED ones.  Wherever the raw seed fell outside the centered
 * range it was never displaced, so half[] came out inflated by the hard iron —
 * which under-scaled si[] and over-stated the radius handed to ellipse_fit.
 * This reproduces the old seeding alongside the fixed path and pins both.
 *
 * Center Y is chosen positive so the lattice's first point (the pole, at
 * y = cy + r = 65) lands outside the centered range: that is the swing that
 * used to be mis-scaled by 24% on one axis.
 */
static void test_extent_ignores_hard_iron(void)
{
    begin("test_extent_ignores_hard_iron");
    int fb = g_fail;

    const double c[3] = { 12.5, 25.0, 3.1 };
    const double r    = 40.0;
    static double samps[256][3];
    make_sphere_samples(samps, 256, c, r);

    extent_accum_t acc = {0};
    for (int i = 0; i < 256; i++)
        extent_add(&acc, samps[i][0] - c[0], samps[i][1] - c[1],
                         samps[i][2] - c[2]);

    double half[3];
    EXPECT(extent_half(&acc, half) == 0, "extent_half returns 0");
    for (int k = 0; k < 3; k++)
        EXPECT_NEAR_D(half[k], r, 0.5, "half-range unaffected by hard iron");

    double si[3];
    cal_softiron_diag(half, r, si);
    EXPECT_NEAR_D(si[1], 1.0, 0.02, "Y soft iron stays unity");

    /* The old seeding, verbatim — it has to be visibly wrong, or this test
     * would pass just as happily against the bug it exists to catch. */
    double mn[3] = { samps[0][0], samps[0][1], samps[0][2] };
    double mx[3] = { samps[0][0], samps[0][1], samps[0][2] };
    for (int i = 0; i < 256; i++)
        for (int k = 0; k < 3; k++) {
            double d = samps[i][k] - c[k];
            if (d < mn[k]) mn[k] = d;
            if (d > mx[k]) mx[k] = d;
        }
    double old_half_y = (mx[1] - mn[1]) / 2.0;
    EXPECT(old_half_y > half[1] + 10.0,
           "old seeding inflated the Y half-range (the bug)");
    EXPECT(old_half_y > 50.0, "old seeding kept the raw pole value 65");
    end(fb);
}

/* A genuinely distorted axis still produces the scale that corrects it. */
static void test_softiron_diag_scales_distorted_axis(void)
{
    begin("test_softiron_diag_scales_distorted_axis");
    int fb = g_fail;

    /* X stretched 25% by soft iron; Y and Z clean. */
    double half[3] = { 50.0, 40.0, 40.0 };
    double si[3];
    cal_softiron_diag(half, 40.0, si);
    EXPECT_NEAR_D(si[0], 0.8, 1e-9, "stretched X scaled back to the radius");
    EXPECT_NEAR_D(si[1], 1.0, 1e-9, "clean Y left alone");
    EXPECT_NEAR_D(si[2], 1.0, 1e-9, "clean Z left alone");

    /* Flat swing: no Z extent to fit, so Z must be left at 1.0 rather than
     * amplified by a huge radius/half ratio. */
    double flat[3] = { 40.0, 40.0, 2.0 };
    cal_softiron_diag(flat, 40.0, si);
    EXPECT_NEAR_D(si[2], 1.0, 1e-9, "no Z coverage leaves Z uncorrected");
    EXPECT(si[2] != 40.0 / 2.0, "Z is not scaled 20x off noise");
    end(fb);
}

static void test_extent_no_samples(void)
{
    begin("test_extent_no_samples");
    int fb = g_fail;

    extent_accum_t acc = {0};
    double half[3] = { -1, -1, -1 };
    EXPECT(extent_half(&acc, half) == -1, "empty accumulator returns -1");

    /* A single point has zero extent, but it is a defined answer. */
    extent_add(&acc, 5.0, -2.0, 1.0);
    EXPECT(extent_half(&acc, half) == 0, "one point returns 0");
    for (int k = 0; k < 3; k++)
        EXPECT_NEAR_D(half[k], 0.0, 1e-12, "one point has zero half-range");
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

/* ── Heading-circle coverage (guided swing cal) ─────────────────────────── */

static void test_cov_full_circle(void)
{
    begin("test_cov_full_circle");
    int fb = g_fail;
    int sec[24] = {0};
    /* Full circle around an offset center, iron-distorted (elliptical):
       coverage must still fill every sector. */
    for (int i = 0; i < 360; i++) {
        double a = i * M_PI / 180.0;
        double x = 30.0 + 40.0 * cos(a);       /* hard iron +30 */
        double y = -10.0 + 25.0 * sin(a);      /* squashed ellipse */
        int idx = cal_cov_mark(sec, 24, x, y, 30.0, -10.0);
        EXPECT(idx >= 0 && idx < 24, "sector index in range");
    }
    EXPECT(cal_cov_count(sec, 24) == 24, "full distorted circle fills 24/24");
    end(fb);
}

static void test_cov_half_circle(void)
{
    begin("test_cov_half_circle");
    int fb = g_fail;
    int sec[24] = {0};
    for (int i = 0; i <= 180; i++) {
        double a = i * M_PI / 180.0;
        cal_cov_mark(sec, 24, cos(a), sin(a), 0.0, 0.0);
    }
    int c = cal_cov_count(sec, 24);
    EXPECT(c == 13, "half circle fills 13/24 (inclusive endpoints)");
    end(fb);
}

static void test_cov_wrong_center(void)
{
    begin("test_cov_wrong_center");
    int fb = g_fail;
    /* With the center estimate far outside the data circle, all samples
       land in a narrow angular band — coverage must NOT report full. This
       is why the tool refines the center estimate as it fits. */
    int sec[24] = {0};
    for (int i = 0; i < 360; i++) {
        double a = i * M_PI / 180.0;
        cal_cov_mark(sec, 24, cos(a), sin(a), 100.0, 0.0);
    }
    EXPECT(cal_cov_count(sec, 24) < 6, "distant center collapses coverage");
    end(fb);
}

/* ── Allan deviation tests ───────────────────────────────────────────────── */

/* Deterministic xorshift so results are reproducible across platforms. */
static uint64_t rng_state = 0x9E3779B97F4A7C15ULL;
static double rng_gauss(void)
{
    /* sum of 12 uniforms − 6 ≈ N(0,1); plenty for statistical tests */
    double acc = 0.0;
    for (int i = 0; i < 12; i++) {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 7;
        rng_state ^= rng_state << 17;
        acc += (double)(rng_state >> 11) / 9007199254740992.0;   /* [0,1) */
    }
    return acc - 6.0;
}

/* Constant signal: θ is a perfect line, every second difference is zero. */
static void test_allan_constant_signal(void)
{
    begin("test_allan_constant_signal");
    int fb = g_fail;

    enum { N = 4096 };
    static double x[N];
    for (int i = 0; i < N; i++) x[i] = 0.123;

    avar_pt_t pts[32];
    int np = allan_deviation(x, N, 100.0, pts, 32);
    EXPECT(np >= 8, "octave points computed");
    bool zero = true;
    for (int i = 0; i < np; i++)
        if (pts[i].adev > 1e-12) zero = false;
    EXPECT(zero, "constant signal has zero Allan deviation");
    EXPECT_NEAR_D(pts[0].tau_s, 0.01, 1e-12, "first tau = 1/fs");
    end(fb);
}

/*
 * White noise σ_w per sample at fs: σ(τ) = N/√τ with noise density
 * N = σ_w/√fs.  Checks the −1/2 slope and the extracted density.
 */
static void test_allan_white_noise(void)
{
    begin("test_allan_white_noise");
    int fb = g_fail;

    enum { N = 1 << 17 };                     /* 131072 samples */
    static double x[N];
    const double fs = 100.0, sw = 0.01;
    for (int i = 0; i < N; i++) x[i] = sw * rng_gauss();

    avar_pt_t pts[32];
    int np = allan_deviation(x, N, fs, pts, 32);
    EXPECT(np >= 10, "enough octaves");

    /* slope between the first two octaves ≈ −1/2 */
    double slope = log(pts[2].adev / pts[0].adev)
                 / log(pts[2].tau_s / pts[0].tau_s);
    EXPECT(fabs(slope + 0.5) < 0.05, "white-noise slope is -1/2");

    double nd, bi;
    int imin = allan_characterize(pts, np, &nd, &bi);
    EXPECT(imin >= 0, "characterize succeeds");
    double nd_true = sw / sqrt(fs);
    EXPECT(fabs(nd - nd_true) / nd_true < 0.10,
           "noise density within 10% of truth");
    /* pure white noise keeps descending — min sits at the last octave */
    EXPECT(imin == np - 1, "no bias floor in pure white noise");
    end(fb);
}

/* Random walk (integrated white noise): σ(τ) grows with +1/2 slope. */
static void test_allan_random_walk(void)
{
    begin("test_allan_random_walk");
    int fb = g_fail;

    enum { N = 1 << 16 };
    static double x[N];
    double b = 0.0;
    for (int i = 0; i < N; i++) {
        b += 1e-4 * rng_gauss();
        x[i] = b;
    }

    avar_pt_t pts[32];
    int np = allan_deviation(x, N, 100.0, pts, 32);
    EXPECT(np >= 10, "enough octaves");
    /* mid-curve slope: the last octaves have too few clusters to be stable */
    double slope = log(pts[np - 5].adev / pts[np - 8].adev)
                 / log(pts[np - 5].tau_s / pts[np - 8].tau_s);
    EXPECT(fabs(slope - 0.5) < 0.15, "random-walk slope is +1/2 at long tau");
    end(fb);
}

/* Synthetic curve N/√τ + flat floor: both parameters recovered exactly. */
static void test_allan_characterize_floor(void)
{
    begin("test_allan_characterize_floor");
    int fb = g_fail;

    avar_pt_t pts[16];
    const double N_true = 2e-3, floor_adev = 5e-4;
    int np = 16;   /* white crosses the floor at ~16 s = octave 11; the flat
                    * floor then spans several octaves before the end */
    for (int i = 0; i < np; i++) {
        double tau = 0.01 * (double)(1 << i);
        double white = N_true / sqrt(tau);
        pts[i].tau_s = tau;
        pts[i].adev  = white > floor_adev ? white : floor_adev;
    }

    double nd, bi;
    int imin = allan_characterize(pts, np, &nd, &bi);
    EXPECT(imin >= 0 && imin < np - 1, "floor minimum found before the end");
    EXPECT_NEAR_D(nd, N_true, 1e-9, "noise density exact on synthetic curve");
    EXPECT_NEAR_D(bi, 0.664 * floor_adev, 1e-9, "bias instability 0.664*floor");
    EXPECT(allan_characterize(pts, 2, &nd, &bi) == -1, "npts<3 rejected");
    end(fb);
}

/* ── Gyro temp-fit tests ─────────────────────────────────────────────────── */

static void test_gyro_temp_fit(void)
{
    begin("test_gyro_temp_fit");
    int fb = g_fail;

    enum { N = 2000 };
    static double t[N], y[N];
    const double ref = 25.0, coeff_true = 3e-5, bias_true = -2e-4;
    for (int i = 0; i < N; i++) {
        t[i] = 15.0 + 25.0 * (double)i / N;             /* 15→40 °C ramp */
        y[i] = bias_true + coeff_true * (t[i] - ref)
               + 1e-5 * rng_gauss();                     /* sensor noise */
    }

    double coeff, bias;
    EXPECT(gyro_temp_fit(t, y, N, ref, &coeff, &bias) == 0, "fit succeeds");
    EXPECT(fabs(coeff - coeff_true) / coeff_true < 0.05,
           "coeff within 5% of truth");
    EXPECT(fabs(bias - bias_true) < 5e-6, "bias at ref recovered");

    /* tiny temperature span is rejected */
    for (int i = 0; i < N; i++) t[i] = 25.0 + 0.001 * (i % 2);
    EXPECT(gyro_temp_fit(t, y, N, ref, &coeff, &bias) == -1,
           "sub-degree span rejected");
    EXPECT(gyro_temp_fit(t, y, 1, ref, &coeff, &bias) == -1, "n<2 rejected");
    end(fb);
}

/* ── Six-position accelerometer fit ──────────────────────────────────────── */

/*
 * Build a meas[3][2][3] table by placing the board in each of the six
 * orientations cal_accel_positions[] asks for, in the NED board frame
 * (X fwd, Y stbd, Z down): the axis pointing UP reads +g.
 *
 * `truth_scale`/`truth_offset` let a test inject a real sensor error and
 * check the fit recovers it: raw = ideal / scale + offset.
 */
static void build_meas(float meas[3][2][3],
                       const float truth_scale[3], const float truth_offset[3])
{
    memset(meas, 0, sizeof(float) * 3 * 2 * 3);
    for (int p = 0; p < 6; p++) {
        int axis = cal_accel_positions[p].axis;
        int sign = cal_accel_positions[p].sign;
        int slot = (sign > 0) ? 0 : 1;
        for (int k = 0; k < 3; k++) {
            float ideal = (k == axis) ? (float)sign * CAL_G_MS2 : 0.0f;
            float sc  = truth_scale  ? truth_scale[k]  : 1.0f;
            float off = truth_offset ? truth_offset[k] : 0.0f;
            meas[axis][slot][k] = ideal / sc + off;
        }
    }
}

static void test_accel_fit_ideal(void)
{
    begin("test_accel_fit_ideal");
    int fb = g_fail;

    float meas[3][2][3];
    build_meas(meas, NULL, NULL);

    float off[3], sc[3];
    char err[512] = {0};
    EXPECT(cal_accel_fit(meas, off, sc, err, sizeof err) == 0,
           "a correct six-position run is accepted");
    for (int k = 0; k < 3; k++) {
        EXPECT_NEAR_D(sc[k],  1.0, 1e-5, "scale is unity");
        EXPECT_NEAR_D(off[k], 0.0, 1e-5, "offset is zero");
    }
    end(fb);
}

static void test_accel_fit_recovers_real_error(void)
{
    begin("test_accel_fit_recovers_real_error");
    int fb = g_fail;

    /* A genuine sensor: 3% sensitivity error and a real zero-g offset. */
    const float ts[3]  = { 1.03f, 0.97f, 1.01f };
    const float toff[3] = { 0.20f, -0.15f, 0.05f };
    float meas[3][2][3];
    build_meas(meas, ts, toff);

    float off[3], sc[3];
    char err[512] = {0};
    EXPECT(cal_accel_fit(meas, off, sc, err, sizeof err) == 0, "accepted");
    for (int k = 0; k < 3; k++) {
        EXPECT_NEAR_D(sc[k],  ts[k],   1e-4, "recovers the true scale");
        EXPECT_NEAR_D(off[k], toff[k], 1e-4, "recovers the true offset");
    }
    end(fb);
}

static void test_accel_fit_tolerates_sloppy_placement(void)
{
    begin("test_accel_fit_tolerates_sloppy_placement");
    int fb = g_fail;

    /* 5 degrees off square on every face — a realistic hand placement.
     * The guards must not fire on this. */
    float meas[3][2][3];
    build_meas(meas, NULL, NULL);
    const float c = 0.99619f, s = 0.08716f;   /* cos/sin 5 deg */
    for (int a = 0; a < 3; a++)
        for (int u = 0; u < 2; u++) {
            int other = (a + 1) % 3;
            float on = meas[a][u][a];
            meas[a][u][a]     = on * c;
            meas[a][u][other] = on * s;
        }

    float off[3], sc[3];
    char err[512] = {0};
    EXPECT(cal_accel_fit(meas, off, sc, err, sizeof err) == 0,
           "5 degrees of placement slop is still accepted");
    end(fb);
}

/*
 * The regression test for the imud-cal accel bug.
 *
 * Before the fix, the prompts' parenthetical instructions described the wrong
 * physical position: Z inverted, and X/Y swapped with each other and
 * inverted.  Every axis then produced either a negative or a near-zero
 * half-range, which the old code silently turned into scale = 1.0 with no
 * warning — a calibration that looked clean and did nothing.
 */
static void test_accel_fit_rejects_old_wrong_positions(void)
{
    begin("test_accel_fit_rejects_old_wrong_positions");
    int fb = g_fail;

    const float G = CAL_G_MS2;
    float meas[3][2][3];
    memset(meas, 0, sizeof meas);

    /* What a board actually reads when the operator follows the old
     * instructions, filed into the slots the old code used. */
    /* "+Z up (flat, normal side up)"  -> +Z points DOWN -> Z reads -g */
    meas[2][0][2] = -G;
    /* "+Z down (flat, upside down)"   -> +Z points UP   -> Z reads +g */
    meas[2][1][2] = +G;
    /* "+X up (right edge down)"       -> +Y points DOWN -> Y reads -g */
    meas[0][0][1] = -G;
    /* "+X down (left edge down)"      -> +Y points UP   -> Y reads +g */
    meas[0][1][1] = +G;
    /* "+Y up (front edge down)"       -> +X points DOWN -> X reads -g */
    meas[1][0][0] = -G;
    /* "+Y down (back edge down)"      -> +X points UP   -> X reads +g */
    meas[1][1][0] = +G;

    float off[3] = { 9, 9, 9 }, sc[3] = { 9, 9, 9 };
    char err[512] = {0};
    EXPECT(cal_accel_fit(meas, off, sc, err, sizeof err) == -1,
           "the old wrong positions are rejected, not silently fitted");
    EXPECT(err[0] != '\0', "a diagnosis is produced");
    /* Must not leave a usable-looking result behind. */
    EXPECT(sc[0] == 9 && off[0] == 9,
           "offset/scale are untouched on failure");
    end(fb);
}

static void test_accel_fit_rejects_inverted_pair(void)
{
    begin("test_accel_fit_rejects_inverted_pair");
    int fb = g_fail;

    /* Correct everywhere except that the Z faces were taken the wrong way
     * round — the single most likely operator slip. */
    float meas[3][2][3];
    build_meas(meas, NULL, NULL);
    float tmp = meas[2][0][2];
    meas[2][0][2] = meas[2][1][2];
    meas[2][1][2] = tmp;

    float off[3], sc[3];
    char err[512] = {0};
    EXPECT(cal_accel_fit(meas, off, sc, err, sizeof err) == -1,
           "an inverted Z pair is rejected");
    EXPECT(strstr(err, "inverted") != NULL, "diagnosis says inverted");
    EXPECT(strchr(err, 'Z') != NULL, "diagnosis names the Z axis");
    end(fb);
}

static void test_accel_fit_rejects_wrong_axis(void)
{
    begin("test_accel_fit_rejects_wrong_axis");
    int fb = g_fail;

    /* X and Y positions swapped: gravity lands on the wrong axis. */
    float meas[3][2][3];
    build_meas(meas, NULL, NULL);
    for (int u = 0; u < 2; u++)
        for (int k = 0; k < 3; k++) {
            float t = meas[0][u][k];
            meas[0][u][k] = meas[1][u][k];
            meas[1][u][k] = t;
        }

    float off[3], sc[3];
    char err[512] = {0};
    EXPECT(cal_accel_fit(meas, off, sc, err, sizeof err) == -1,
           "gravity on the wrong axis is rejected");
    EXPECT(strstr(err, "gravity landed on") != NULL,
           "diagnosis names the wrong-axis case");
    end(fb);
}

/* The position table itself must stay consistent with the board frame. */
static void test_accel_positions_table(void)
{
    begin("test_accel_positions_table");
    int fb = g_fail;

    int seen[3][2] = {{0,0},{0,0},{0,0}};
    for (int p = 0; p < 6; p++) {
        const cal_accel_pos_t *q = &cal_accel_positions[p];
        EXPECT(q->label && q->label[0],   "position has a label");
        EXPECT(q->detail && q->detail[0], "position has a detail line");
        EXPECT(q->axis >= 0 && q->axis <= 2, "axis in range");
        EXPECT(q->sign == 1 || q->sign == -1, "sign is +/-1");
        seen[q->axis][q->sign > 0 ? 0 : 1]++;
    }
    for (int a = 0; a < 3; a++)
        for (int u = 0; u < 2; u++)
            EXPECT(seen[a][u] == 1, "each axis/direction appears exactly once");

    /* The one that was wrong: flat and component-side up must be the
     * Z-points-DOWN position, i.e. the one that reads -g. */
    const cal_accel_pos_t *flat = NULL;
    for (int p = 0; p < 6; p++)
        if (strstr(cal_accel_positions[p].label, "component side up"))
            flat = &cal_accel_positions[p];
    EXPECT(flat != NULL, "the component-side-up position exists");
    if (flat) {
        EXPECT(flat->axis == 2, "component-side-up is a Z position");
        EXPECT(flat->sign == -1,
               "component-side-up reads -g (Z is DOWN in the board frame)");
    }
    end(fb);
}

int main(void)
{
    puts("=== imud cal_math tests ===");

    test_gauss4_identity();
    test_gauss4_known_solution();
    test_gauss4_singular();
    test_sphere_add_count();
    test_sphere_fit_too_few_points();
    test_sphere_fit_unit_origin();
    test_sphere_fit_hard_iron_offset();
    test_sphere_fit_large_offset();
    test_ellipse_fit_rotated();
    test_ellipse_fit_noise();
    test_ellipse_fit_circle_identity();
    test_ellipse_fit_degenerate();
    test_extent_full_sphere();
    test_extent_ignores_hard_iron();
    test_softiron_diag_scales_distorted_axis();
    test_softiron_needs_real_coverage();
    test_extent_no_samples();
    test_cov_full_circle();
    test_cov_half_circle();
    test_cov_wrong_center();
    test_allan_constant_signal();
    test_allan_white_noise();
    test_allan_random_walk();
    test_allan_characterize_floor();
    test_gyro_temp_fit();
    test_accel_positions_table();
    test_accel_fit_ideal();
    test_accel_fit_recovers_real_error();
    test_accel_fit_tolerates_sloppy_placement();
    test_accel_fit_rejects_old_wrong_positions();
    test_accel_fit_rejects_inverted_pair();
    test_accel_fit_rejects_wrong_axis();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
