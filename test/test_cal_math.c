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
    test_cov_full_circle();
    test_cov_half_circle();
    test_cov_wrong_center();
    test_allan_constant_signal();
    test_allan_white_noise();
    test_allan_random_walk();
    test_allan_characterize_floor();
    test_gyro_temp_fit();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
