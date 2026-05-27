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

/* ── main ────────────────────────────────────────────────────────────────── */

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

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
