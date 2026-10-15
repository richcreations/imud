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

#include <stdbool.h>
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
 * Per-axis extent of the CENTERED samples (hard iron already removed), used
 * for the diagonal soft-iron fallback.
 *
 * Zero-initialising the accumulator is enough: the first point seeds both
 * bounds, so the running min/max can never be contaminated by a value on a
 * different scale.  (It was: seeding from a RAW sample and then comparing
 * centered ones against it, which inflated the half-range by the hard-iron
 * offset.)
 */
typedef struct {
    double mn[3], mx[3];
    int    n;
} extent_accum_t;

/* Add one centered measurement. */
void extent_add(extent_accum_t *a, double x, double y, double z);

/* Half of the per-axis extent.  Returns 0, or -1 when no points were added. */
int extent_half(const extent_accum_t *a, double half[3]);

/*
 * The least coverage an axis may have before its diagonal soft-iron scale is
 * believed, as a fraction of the fitted sphere radius.  A fully swept axis has
 * half[k] ~= radius, so this bound also caps the largest correction that can be
 * emitted at 1/0.8 = 1.25x — the top of the range real soft iron reaches on
 * these parts.  Anything beyond that is missing coverage, not distortion.
 */
#define CAL_SI_MIN_SPAN 0.8

/*
 * Diagonal soft-iron scale: the factor per axis that stretches the measured
 * half-range onto the fitted sphere radius.  An axis covering less than
 * CAL_SI_MIN_SPAN of the radius is left at 1.0.
 *
 * At 0.3 the threshold would admit corrections up to 3.3x.  No real
 * soft iron is 3.3x; that is the fit stretching an axis to cover data that was
 * never taken.  A level swing — the procedure imud-cal mag actually asks for,
 * and the only one a vessel or a ground robot can perform — sweeps no Z extent
 * at all, so Z must come out of it uncorrected rather than confidently wrong.
 * Measured on the bench: a swing with incidental hand-tilt reached 0.48 of the
 * radius in Z and produced a 2.07x scale, which would have doubled the vertical
 * field the moment the sensor left level.
 */
void cal_softiron_diag(const double half[3], double radius, double si[3]);

/*
 * What `imud-cal mag` writes, and what it tells the operator on the way.
 * hard_iron and soft_iron are in BOARD axes, because that is where the daemon
 * applies them — before the mount rotation.  So is centre, for the coverage
 * display.  The rest is in the swing's own axes, where the fit is done.
 */
typedef struct {
    double hard_iron[3];      /* µT, board axes */
    double soft_iron[3][3];   /* board axes */
    double centre[3];         /* centre of the swing's circle, µT, board
                               * axes; off the hard iron along the vertical */
    double radius;            /* fitted field radius, µT; of the HORIZONTAL
                               * field when the swing was level */
    double rms;               /* RMS residual about it, µT */
    double half[3];           /* half-extent per axis after centring, µT */
    double S2[2][2];          /* horizontal soft-iron block */
    double si[3];             /* diagonal soft iron */
    bool   ellipse_ok;        /* S2 is a fitted 2-D block; otherwise si */
    bool   level;             /* a level swing: circle fit, and no vertical
                               * hard iron taken from it */
    double vertical;          /* mean vertical reading, µT, uncorrected */
    double tilt_deg;          /* the swing's vertical from body Z, degrees */
    bool   accel_vertical;    /* that vertical is the accelerometer's; else
                               * the normal of the swing's own plane */
} cal_mag_fit_t;

/*
 * A swing is level when the standard deviation of its vertical reading about
 * the plane it lies in, in its own axes, is under this fraction of the
 * horizontal circle's radius: about ±6° of roll.  A deviation, not the
 * extremes, so that noise and a stray sample barely move it.
 */
#define CAL_LEVEL_MAX_SD 0.05

/*
 * A swing that is not level keeps the sphere only if the sphere found the
 * vertical field.  Noise in the vertical reading pulls the sphere's centre
 * towards the swing's own plane, and the field with it into the hard iron:
 * so the roll, the scatter less the noise seen from one sample to the next,
 * must be CAL_SPHERE_MIN_SNR times the noise, as standard deviations; and the
 * sphere's radius CAL_SPHERE_MIN_GAIN times the circle's, which a slow drift
 * does not give.  Past CAL_SPHERE_FREE_SD of the radius the sphere is taken
 * as it is: a tumble's centre is its own mean height.
 */
#define CAL_SPHERE_MIN_SNR  3.0
#define CAL_SPHERE_MIN_GAIN 1.05
#define CAL_SPHERE_FREE_SD  0.2

/*
 * cal_mag_fit — hard and soft iron from a swing of n raw samples (µT, board
 * axes).  `mount` is the board→body rotation M from [mount], or NULL for a
 * board mounted flat.  `accel` is the accelerometer's mean over the swing
 * (m/s², board axes, corrected as the daemon corrects it), or NULL.
 *
 * The fit is done in the swing's own axes: body axes, turned onto the vertical
 * the swing was made about.  Body axes, because on a board mounted on edge the
 * swing lies in board X–Z or Y–Z, and a fit of board X/Y takes the vertical
 * field for one of the horizontal axes.  Turned, because a vessel heels
 * steadily through its turn, and its swing is then a circle tilted with it.
 * The vertical is gravity's, from `accel`: soft iron that couples a
 * horizontal axis to the vertical tilts the swing's plane just as heel does.
 * Without `accel` it is the normal of that plane.  With R the whole
 * board→swing rotation, the result is mapped back for cal.json,
 * h_board = Rᵀ·h and S_board = Rᵀ·S·R, so that the daemon's
 * M·S_board·(m − h_board) is the swing-axis correction S·(R·m − h), turned
 * back to body axes.
 *
 * A level swing — one that holds its heel, as a vessel's does — gets a
 * circle fit in the horizontal plane, and no vertical hard iron: see
 * CAL_LEVEL_MAX_SD.  One whose heel varies enough to measure the vertical
 * gets the sphere, and its horizontal soft iron is fitted against that sphere.
 *
 * Returns 0, or -1 when the samples do not determine a fit.
 */
int cal_mag_fit(const float (*samps)[3], int n, const double mount[3][3],
                const double accel[3], cal_mag_fit_t *out);

/*
 * A swing of any length in a fixed buffer.  Every sample is kept until the
 * buffer is full; then every other kept one is let go and only one sample in
 * two is kept from there, and so on.  What is kept stays evenly spread over
 * the whole swing, so the swing runs for as long as it takes.
 */
typedef struct {
    float     (*s)[3];  /* cap samples, the caller's */
    int       cap;      /* even, at least 2 */
    int       n;        /* kept */
    long long seen;     /* offered */
    long long stride;   /* one kept in this many */
} cal_mag_keep_t;

void cal_mag_keep_init(cal_mag_keep_t *k, float (*buf)[3], int cap);

/* Offer one sample; true when it was kept. */
bool cal_mag_keep(cal_mag_keep_t *k, const float v[3]);

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
