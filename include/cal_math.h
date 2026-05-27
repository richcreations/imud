/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cal_math.h — magnetometer sphere-fitting routines used by imud-cal
 *
 * Usage:
 *   sphere_accum_t acc = {0};
 *   for each sample: sphere_add(&acc, x, y, z);
 *   sphere_fit(&acc, center, &radius);  // at any point with >= 5 samples
 */
#ifndef IMUD_CAL_MATH_H
#define IMUD_CAL_MATH_H

/*
 * Incremental accumulator for the linearised sphere-fit normal equations.
 * Accumulates Σx, Σx², Σxy, Σxr² etc. so sphere_fit can be called at any
 * time without re-processing the raw samples.
 */
typedef struct {
    double sx, sy, sz;
    double sxx, syy, szz, sxy, sxz, syz;
    double sxr, syr, szr;   /* Σx(x²+y²+z²) etc. */
    int    n;
} sphere_accum_t;

/* Add one measurement point to the accumulator. */
void sphere_add(sphere_accum_t *a, float x, float y, float z);

/*
 * Fit a sphere to the accumulated points.
 * Requires n >= 5.  On success fills center[3] and *radius and returns 0.
 * Returns -1 if too few points, if the system is singular, or if the
 * recovered radius² is non-positive.
 */
int sphere_fit(const sphere_accum_t *a, double center[3], double *radius);

/*
 * Solve a 4×4 linear system Ax = b via Gaussian elimination with partial
 * pivoting.  A and b are overwritten.  Returns 0 on success, -1 if singular.
 */
int gauss4(double A[4][4], double b[4], double x[4]);

#endif /* IMUD_CAL_MATH_H */
