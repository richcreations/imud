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

#include <stddef.h>

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

/* ── Six-position accelerometer calibration ───────────────────────────────── */

/*
 * The board frame is NED-compatible: X forward, Y starboard, Z **down**.  The
 * axis pointing UP therefore reads +g, and the axis pointing DOWN reads -g —
 * so flat and component-side up reads [0, 0, -9.80665].  See manual §11.
 *
 * The position descriptions live here rather than in the tool so that
 * imud-cal and imud-imutest ask for the same six orientations in the same
 * words, and so the fit can be tested without hardware.
 */
typedef struct {
    const char *label;   /* what the operator is asked to do */
    const char *detail;  /* why, in board-frame terms */
    int         axis;    /* 0=X 1=Y 2=Z */
    int         sign;    /* +1 = this axis points up, so it reads +g */
} cal_accel_pos_t;

extern const cal_accel_pos_t cal_accel_positions[6];

/* Standard gravity, m/s² — the reference every position is measured against. */
#define CAL_G_MS2 9.80665f

/*
 * Fit the accelerometer offset/scale model  a_cal = (a_raw - offset) * scale
 * from the six averaged readings.
 *
 *   meas[axis][0] = reading taken with that axis pointing up   (expect +g)
 *   meas[axis][1] = reading taken with that axis pointing down (expect -g)
 *
 * Returns 0 on success.  Returns -1 when a reading is geometrically
 * impossible — gravity on the wrong axis, or an inverted pair — and writes a
 * diagnosis naming the offending position into `err`.  offset[] and scale[]
 * are left untouched on failure: a calibration fitted from a misplaced board
 * is worse than none, so the caller must not save one.
 */
int cal_accel_fit(const float meas[3][2][3], float offset[3], float scale[3],
                  char *err, size_t errsz);

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

/* ── Allan-variance sensor characterization (imud-cal characterize) ─────── */

typedef struct {
    double tau_s;     /* cluster time, s */
    double adev;      /* overlapping Allan deviation at tau (signal units) */
} avar_pt_t;

/*
 * Overlapping Allan deviation of a uniformly sampled signal x[0..n) at
 * octave-spaced cluster times (tau = m/fs, m = 1,2,4,... while 2m < n).
 * Fills pts[0..max_pts); returns the number of points computed (0 when the
 * record is too short or allocation fails).
 */
int allan_deviation(const double *x, size_t n, double fs,
                    avar_pt_t *pts, int max_pts);

/*
 * Extract sensor characteristics from an Allan-deviation curve:
 *   noise_density    N (units/√Hz)  = σ(τ₀)·√τ₀, evaluated at the shortest
 *                    cluster time where white noise dominates
 *   bias_instability B (units)      = 0.664 × the curve minimum; when the
 *                    minimum sits on the LAST point the record was too short
 *                    to see the floor and B is only an upper bound
 * Returns the index of the curve minimum, or -1 when npts < 3.
 */
int allan_characterize(const avar_pt_t *pts, int npts,
                       double *noise_density, double *bias_instability);

/* ── Gyro-bias / temperature linear fit (imud-cal fit-temp) ─────────────── */

/*
 * Least-squares fit  y ≈ bias_ref + coeff·(T − ref_c)  over n paired
 * samples.  Returns 0, or -1 when n < 2 or the temperature span is too
 * small (< 1 °C) to support a fit.
 */
int gyro_temp_fit(const double *temp_c, const double *gyro, size_t n,
                  double ref_c, double *coeff, double *bias_ref);

#endif /* IMUD_CAL_MATH_H */
