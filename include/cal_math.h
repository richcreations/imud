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
 * Heading-circle coverage tracking for the guided swing calibration.
 * The circle is split into nsec angular sectors around the (running)
 * hard-iron center estimate; each raw horizontal sample marks its sector.
 * A full swing fills every sector regardless of iron distortion.
 */

/* Mark the sector containing (x,y) relative to center (cx,cy); returns the
 * sector index [0, nsec). */
int cal_cov_mark(int *sectors, int nsec, double x, double y,
                 double cx, double cy);

/* Number of sectors marked so far. */
int cal_cov_count(const int *sectors, int nsec);

/*
 * Solve a 4×4 linear system Ax = b via Gaussian elimination with partial
 * pivoting.  A and b are overwritten.  Returns 0 on success, -1 if singular.
 */
int gauss4(double A[4][4], double b[4], double x[4]);

/*
 * Incremental accumulator for the horizontal (2D) ellipse fit used for
 * soft-iron correction.  Feed CENTERED samples (hard iron already removed
 * by the sphere fit).
 */
typedef struct {
    double s40, s31, s22, s13, s04;   /* Σx⁴, Σx³y, Σx²y², Σxy³, Σy⁴ */
    double s20, s11, s02;             /* Σx², Σxy, Σy² */
    int    n;
} ellipse_accum_t;

/* Add one centered horizontal measurement (x, y). */
void ellipse_add(ellipse_accum_t *a, float x, float y);

/*
 * Fit the centered conic A·x² + B·xy + C·y² = 1 to the accumulated points
 * and derive the symmetric 2×2 soft-iron correction S = radius·sqrtm(M)
 * (M the conic matrix): applying S maps the ellipse onto a circle of the
 * given radius.  Unlike a per-axis (diagonal) scale, S captures the CROSS
 * term — a soft-iron ellipse whose axes are rotated relative to the sensor
 * axes, which a diagonal correction cannot fix.
 *
 * Requires n >= 8 and a positive-definite conic (a real ellipse).
 * Returns 0 on success (fills S[2][2]), -1 otherwise.
 */
int ellipse_fit(const ellipse_accum_t *a, double radius, double S[2][2]);

#endif /* IMUD_CAL_MATH_H */
