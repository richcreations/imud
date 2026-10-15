/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cal_math.c — magnetometer sphere-fitting routines for imud-cal
 *
 * Linearises (x-cx)²+(y-cy)²+(z-cz)² = r² as:
 *   x²+y²+z² = 2cx·x + 2cy·y + 2cz·z + (r²-cx²-cy²-cz²)
 * and accumulates the normal equations (AᵀA·p = Aᵀb) incrementally so
 * sphere_fit can be called at any time without storing raw samples.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cal_math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int gauss4(double A[4][4], double b[4], double x[4])
{
    double M[4][5];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) M[i][j] = A[i][j];
        M[i][4] = b[i];
    }

    for (int col = 0; col < 4; col++) {
        int pivot = col;
        for (int row = col + 1; row < 4; row++)
            if (fabs(M[row][col]) > fabs(M[pivot][col])) pivot = row;

        if (pivot != col)
            for (int j = 0; j < 5; j++) {
                double t = M[col][j]; M[col][j] = M[pivot][j]; M[pivot][j] = t;
            }

        if (fabs(M[col][col]) < 1e-12) return -1;

        for (int row = col + 1; row < 4; row++) {
            double f = M[row][col] / M[col][col];
            for (int j = col; j < 5; j++) M[row][j] -= f * M[col][j];
        }
    }

    for (int i = 3; i >= 0; i--) {
        x[i] = M[i][4];
        for (int j = i + 1; j < 4; j++) x[i] -= M[i][j] * x[j];
        x[i] /= M[i][i];
    }
    return 0;
}

void sphere_add(sphere_accum_t *a, float fx, float fy, float fz)
{
    double x = fx, y = fy, z = fz;
    double r2 = x*x + y*y + z*z;
    a->sx  += x;    a->sy  += y;    a->sz  += z;
    a->sxx += x*x;  a->syy += y*y;  a->szz += z*z;
    a->sxy += x*y;  a->sxz += x*z;  a->syz += y*z;
    a->sxr += x*r2; a->syr += y*r2; a->szr += z*r2;
    a->n++;
}

void extent_add(extent_accum_t *a, double x, double y, double z)
{
    const double p[3] = { x, y, z };
    if (a->n == 0) {
        for (int k = 0; k < 3; k++) a->mn[k] = a->mx[k] = p[k];
    } else {
        for (int k = 0; k < 3; k++) {
            if (p[k] < a->mn[k]) a->mn[k] = p[k];
            if (p[k] > a->mx[k]) a->mx[k] = p[k];
        }
    }
    a->n++;
}

int extent_half(const extent_accum_t *a, double half[3])
{
    if (a->n < 1) return -1;
    for (int k = 0; k < 3; k++) half[k] = (a->mx[k] - a->mn[k]) / 2.0;
    return 0;
}

void cal_softiron_diag(const double half[3], double radius, double si[3])
{
    for (int k = 0; k < 3; k++)
        si[k] = (half[k] > CAL_SI_MIN_SPAN * radius) ? (radius / half[k]) : 1.0;
}

int sphere_fit(const sphere_accum_t *a, double center[3], double *radius)
{
    if (a->n < 5) return -1;

    double AtA[4][4] = {
        { 4*a->sxx, 4*a->sxy, 4*a->sxz, 2*a->sx     },
        { 4*a->sxy, 4*a->syy, 4*a->syz, 2*a->sy     },
        { 4*a->sxz, 4*a->syz, 4*a->szz, 2*a->sz     },
        { 2*a->sx,  2*a->sy,  2*a->sz,  (double)a->n },
    };
    double Atb[4] = {
        2*a->sxr, 2*a->syr, 2*a->szr,
        a->sxx + a->syy + a->szz,
    };
    double sol[4];
    if (gauss4(AtA, Atb, sol) < 0) return -1;

    center[0] = sol[0]; center[1] = sol[1]; center[2] = sol[2];
    double r2 = sol[3] + sol[0]*sol[0] + sol[1]*sol[1] + sol[2]*sol[2];
    if (r2 <= 0) return -1;
    *radius = sqrt(r2);
    return 0;
}

/* ── Magnetometer swing fit ───────────────────────────────────────────────── */

static void rot3(const double R[3][3], const double v[3], double out[3])
{
    for (int i = 0; i < 3; i++)
        out[i] = R[i][0]*v[0] + R[i][1]*v[1] + R[i][2]*v[2];
}

/*
 * Algebraic circle through the horizontal components: the 2-D case of the
 * sphere fit above, about the mean for conditioning.  0, or -1 when the
 * points do not determine one.
 */
static int circle_fit(const double (*b)[3], int n, double c[2], double *radius)
{
    if (n < 5) return -1;
    double mx = 0.0, my = 0.0;
    for (int i = 0; i < n; i++) { mx += b[i][0]; my += b[i][1]; }
    mx /= n; my /= n;

    double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxr = 0, syr = 0, sr = 0;
    for (int i = 0; i < n; i++) {
        double x = b[i][0] - mx, y = b[i][1] - my, r2 = x*x + y*y;
        sxx += x*x;  sxy += x*y;  syy += y*y;  sx += x;  sy += y;
        sxr += x*r2; syr += y*r2; sr  += r2;
    }
    /* x² + y² = 2a·x + 2b·y + k, with k = r² − a² − b². */
    double A[4][4] = {
        { 4*sxx, 4*sxy, 2*sx,      0 },
        { 4*sxy, 4*syy, 2*sy,      0 },
        { 2*sx,  2*sy,  (double)n, 0 },
        { 0,     0,     0,         1 },
    };
    double rhs[4] = { 2*sxr, 2*syr, sr, 0 }, p[4];
    if (gauss4(A, rhs, p) < 0) return -1;

    double r2 = p[2] + p[0]*p[0] + p[1]*p[1];
    if (r2 <= 0.0) return -1;
    c[0] = p[0] + mx;
    c[1] = p[1] + my;
    *radius = sqrt(r2);
    return 0;
}

/*
 * The horizontal soft-iron block, fitted against the sphere of centre c and
 * radius R: every sample's horizontal part must satisfy
 * |S2·d_xy|² = R² − (sz·d_z)², however the vessel heeled.  Least squares on
 * P = S2ᵀS2, then S2 = √P.  0, or -1 when P is not positive definite.
 */
static int softiron_on_sphere(const double (*b)[3], int n, const double c[3],
                              double R, double sz, double S2[2][2])
{
    double A[4][4] = { {0} }, rhs[4] = { 0 }, p[4];
    for (int i = 0; i < n; i++) {
        double x = b[i][0] - c[0], y = b[i][1] - c[1], z = sz * (b[i][2] - c[2]);
        double r[3] = { x*x, 2.0*x*y, y*y }, t = R*R - z*z;
        for (int j = 0; j < 3; j++) {
            rhs[j] += r[j] * t;
            for (int k = 0; k < 3; k++) A[j][k] += r[j] * r[k];
        }
    }
    A[3][3] = 1.0;                       /* identity pad row: gauss4 is 4×4 */
    if (gauss4(A, rhs, p) < 0) return -1;

    double det = p[0]*p[2] - p[1]*p[1];
    if (p[0] <= 0.0 || det <= 0.0) return -1;
    /* √P of a 2×2 positive-definite P is (P + √det·I) / √(tr P + 2√det). */
    double sd = sqrt(det), t = sqrt(p[0] + p[2] + 2.0 * sd);
    S2[0][0] = (p[0] + sd) / t;
    S2[0][1] = S2[1][0] = p[1] / t;
    S2[1][1] = (p[2] + sd) / t;
    return 0;
}

/*
 * The plane z = p·x + q·y + d the samples lie nearest, by least squares about
 * their mean m.  p = q = 0 when the samples do not determine one.
 */
static void plane_fit(const double (*b)[3], int n, double m[3],
                      double *p, double *q)
{
    m[0] = m[1] = m[2] = 0.0;
    for (int i = 0; i < n; i++)
        for (int k = 0; k < 3; k++) m[k] += b[i][k];
    for (int k = 0; k < 3; k++) m[k] /= n;

    double sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
    for (int i = 0; i < n; i++) {
        double x = b[i][0] - m[0], y = b[i][1] - m[1], z = b[i][2] - m[2];
        sxx += x*x;  sxy += x*y;  syy += y*y;  sxz += x*z;  syz += y*z;
    }
    double det = sxx*syy - sxy*sxy;
    *p = *q = 0.0;
    if (det > 1e-12 * (sxx + syy) * (sxx + syy)) {
        *p = (sxz*syy - syz*sxy) / det;
        *q = (syz*sxx - sxz*sxy) / det;
    }
}

/* Q = I + [k]× + [k]×²/(1 + n_z), k = Z × n: the least rotation taking body Z
 * onto the unit vector n, which must have n_z > 0. */
static void z_onto(const double n[3], double Q[3][3])
{
    double nx = n[0], ny = n[1], nz = n[2], f = 1.0 / (1.0 + nz);
    Q[0][0] = 1.0 - nx*nx*f;  Q[0][1] = -nx*ny*f;       Q[0][2] = nx;
    Q[1][0] = -nx*ny*f;       Q[1][1] = 1.0 - ny*ny*f;  Q[1][2] = ny;
    Q[2][0] = -nx;            Q[2][1] = -ny;            Q[2][2] = nz;
}

/* Down in body axes, −M·a/|M·a|, from the accelerometer's mean a.  false when
 * there is none, it is not about 1 g, or it is more than 60° from body Z,
 * which is no vertical a swing is made about. */
static bool accel_down(const double M[3][3], const double accel[3],
                       double down[3])
{
    if (!accel) return false;
    double g[3];
    rot3(M, accel, g);
    double norm = sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    if (!(norm > 0.5 * CAL_G_MS2 && norm < 1.5 * CAL_G_MS2)) return false;
    for (int k = 0; k < 3; k++) down[k] = -g[k] / norm;
    return down[2] > 0.5;
}

int cal_mag_fit(const float (*samps)[3], int n, const double mount[3][3],
                const double accel[3], cal_mag_fit_t *out)
{
    static const double I3[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
    const double (*M)[3] = mount ? mount : I3;
    if (n < 5) return -1;

    double (*b)[3] = malloc((size_t)n * sizeof *b);
    if (!b) return -1;
    for (int i = 0; i < n; i++) {
        double m[3] = { samps[i][0], samps[i][1], samps[i][2] };
        rot3(M, m, b[i]);
    }

    /*
     * A vessel heels steadily through its turn, so its swing is a circle
     * tilted with it, about the vertical it turned on.  The fit is done in the
     * swing's own axes, R = Qᵀ·M, whose Z is that vertical: "level",
     * "horizontal" and "vertical" below are all in these axes.  It is
     * gravity's when there is an accelerometer, and otherwise the normal of
     * the plane the samples lie in, which soft iron coupling a horizontal
     * axis to the vertical tilts as well.
     */
    double down[3], m0[3], p, q;
    out->accel_vertical = accel_down(M, accel, down);
    if (!out->accel_vertical) {
        plane_fit((const double (*)[3])b, n, m0, &p, &q);
        down[2] = 1.0 / sqrt(1.0 + p*p + q*q);
        down[0] = -p * down[2];
        down[1] = -q * down[2];
    }
    out->tilt_deg = acos(down[2] < 1.0 ? down[2] : 1.0) * 180.0 / M_PI;

    double Q[3][3], RQ[3][3];
    z_onto(down, Q);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            RQ[i][j] = Q[0][i]*M[0][j] + Q[1][i]*M[1][j] + Q[2][i]*M[2][j];
    const double (*R)[3] = (const double (*)[3])RQ;

    for (int i = 0; i < n; i++) {
        double m[3] = { samps[i][0], samps[i][1], samps[i][2] };
        rot3(R, m, b[i]);
    }

    /* The vertical reading's scatter about the plane the swing lies in.  Any
     * tilt of that plane is soft iron or the accelerometer's own error, and
     * what remains about it is roll and noise.  The noise is what changes
     * from one sample to the next, which roll does not. */
    plane_fit((const double (*)[3])b, n, m0, &p, &q);
    out->vertical = m0[2];
    double vv = 0.0, dd = 0.0, rprev = 0.0;
    for (int i = 0; i < n; i++) {
        double r = b[i][2] - m0[2] - p * (b[i][0] - m0[0]) - q * (b[i][1] - m0[1]);
        vv += r * r;
        if (i > 0) dd += (r - rprev) * (r - rprev);
        rprev = r;
    }
    double sd = sqrt(vv / n), noise = sqrt(dd / (2.0 * (n - 1)));
    double roll2 = sd * sd - noise * noise;

    /*
     * A level swing is a circle in the horizontal plane, and says nothing of
     * the vertical hard iron: every sample has the same vertical reading, so a
     * sphere through them can put its centre anywhere on the axis, and puts it
     * AT the reading — the whole vertical field goes into the "hard iron", and
     * the heading that comes out is wrong by the vertical field times the
     * sine of the heel.  So a level swing gets the circle, and the vertical
     * hard iron is left at zero, which is wrong only by the hard iron itself.
     * A swing that is not level keeps the sphere only if it found the
     * vertical field: see CAL_SPHERE_MIN_SNR.
     */
    double c[3] = { 0.0, 0.0, 0.0 }, cc[2] = { 0.0, 0.0 }, rc = 0.0;
    bool circle = circle_fit((const double (*)[3])b, n, cc, &rc) == 0;
    out->level = circle && sd < CAL_LEVEL_MAX_SD * rc;
    if (!out->level) {
        sphere_accum_t acc = {0};
        for (int i = 0; i < n; i++)
            sphere_add(&acc, (float)b[i][0], (float)b[i][1], (float)b[i][2]);
        double cs[3], rs;
        bool sphere = sphere_fit(&acc, cs, &rs) == 0;
        bool found  = roll2 >= CAL_SPHERE_MIN_SNR * CAL_SPHERE_MIN_SNR * noise * noise
                   && rs >= CAL_SPHERE_MIN_GAIN * rc;
        if (sphere && (!circle || sd >= CAL_SPHERE_FREE_SD * rc || found)) {
            memcpy(c, cs, sizeof c);
            out->radius = rs;
        } else if (circle) {
            out->level = true;
        } else {
            free(b);
            return -1;
        }
    }
    if (out->level) {
        c[0] = cc[0];
        c[1] = cc[1];
        out->radius = rc;
        /* Its vertical, less that plane's tilt: what is left is roll. */
        for (int i = 0; i < n; i++)
            b[i][2] -= p * (b[i][0] - m0[0]) + q * (b[i][1] - m0[1]);
    }

    /* Residual and per-axis extent, about the centre.  For a level swing both
     * are taken in the plane it was swung in. */
    extent_accum_t xacc = {0};
    double ss = 0.0;
    for (int i = 0; i < n; i++) {
        double dx = b[i][0] - c[0], dy = b[i][1] - c[1];
        double dz = b[i][2] - (out->level ? out->vertical : c[2]);
        double res = (out->level ? sqrt(dx*dx + dy*dy)
                                 : sqrt(dx*dx + dy*dy + dz*dz)) - out->radius;
        ss += res * res;
        extent_add(&xacc, dx, dy, dz);
    }
    out->rms = sqrt(ss / n);
    if (extent_half(&xacc, out->half) < 0) { free(b); return -1; }

    /* The diagonal fallback, and the vertical axis either way: only an axis
     * the swing swept is scaled — see CAL_SI_MIN_SPAN. */
    cal_softiron_diag(out->half, out->radius, out->si);

    /*
     * Horizontal soft iron as a 2-D block.  Unlike the per-axis half-range it
     * recovers the CROSS term — a distortion ellipse turned away from the
     * axes, which is the common case and shows as a heading error that varies
     * round the swing.  Heel moves vertical field into the horizontal axes,
     * which an ellipse through them would take for soft iron, so the block is
     * fitted against the sphere.  A level swing gets the sphere its own
     * assumption implies: centre at no vertical hard iron, through the circle
     * and the mean vertical reading.
     */
    double S2[2][2] = { {1.0, 0.0}, {0.0, 1.0} };
    double rs = out->level ? hypot(out->radius, out->vertical) : out->radius;
    out->ellipse_ok = softiron_on_sphere((const double (*)[3])b, n, c, rs,
                                         out->si[2], S2) == 0;
    free(b);
    memcpy(out->S2, S2, sizeof out->S2);

    double S[3][3] = { {out->si[0], 0, 0}, {0, out->si[1], 0}, {0, 0, out->si[2]} };
    if (out->ellipse_ok) {
        S[0][0] = out->S2[0][0];  S[0][1] = out->S2[0][1];
        S[1][0] = out->S2[1][0];  S[1][1] = out->S2[1][1];
    }

    /* Back to board axes: h_board = Rᵀ·h, S_board = Rᵀ·S·R, and the centre
     * of the swing's circle, which sits off the hard iron along the vertical. */
    for (int i = 0; i < 3; i++) {
        out->hard_iron[i] = R[0][i]*c[0] + R[1][i]*c[1] + R[2][i]*c[2];
        out->centre[i]    = R[0][i]*c[0] + R[1][i]*c[1] + R[2][i]*out->vertical;
        for (int j = 0; j < 3; j++) {
            double v = 0.0;
            for (int k = 0; k < 3; k++)
                for (int l = 0; l < 3; l++)
                    v += R[k][i] * S[k][l] * R[l][j];
            out->soft_iron[i][j] = v;
        }
    }
    return 0;
}

void cal_mag_keep_init(cal_mag_keep_t *k, float (*buf)[3], int cap)
{
    k->s      = buf;
    k->cap    = cap;
    k->n      = 0;
    k->seen   = 0;
    k->stride = 1;
}

bool cal_mag_keep(cal_mag_keep_t *k, const float v[3])
{
    long long i = k->seen++;
    if (i % k->stride != 0) return false;
    if (k->n == k->cap) {
        /* Full: every other kept sample goes, and the spacing doubles to
         * match, so the kept ones are still every stride-th offered. */
        for (int j = 0; j < k->cap / 2; j++)
            memcpy(k->s[j], k->s[2 * j], sizeof k->s[0]);
        k->n = k->cap / 2;
        k->stride *= 2;
        if (i % k->stride != 0) return false;
    }
    memcpy(k->s[k->n++], v, sizeof k->s[0]);
    return true;
}

/* ── Six-position accelerometer calibration ───────────────────────────────── */

/*
 * Board frame: X forward, Y starboard, Z DOWN.  So the axis pointing UP reads
 * +g.  The `detail` line states which way the named axis ends up pointing,
 * because that is the part that is easy to get backwards: "flat, component
 * side up" puts board +Z *down*, not up.
 *
 * Same six orientations, in the same words, as imud-imutest's guided
 * six-face phase.
 */
const cal_accel_pos_t cal_accel_positions[6] = {
    { "Flat, component side up",
      "board +Z points down, so Z reads -g",              2, -1 },
    { "Flat, upside down",
      "board +Z points up, so Z reads +g",                2, +1 },
    { "Nose down (bow edge on the bench)",
      "board +X points down, so X reads -g",              0, -1 },
    { "Nose up (stern edge on the bench)",
      "board +X points up, so X reads +g",                0, +1 },
    { "Starboard side down (right edge on the bench)",
      "board +Y points down, so Y reads -g",              1, -1 },
    { "Port side down (left edge on the bench)",
      "board +Y points up, so Y reads +g",                1, +1 },
};

/*
 * Tolerances.  Both are deliberately loose: a board stood on its edge by hand
 * is a few degrees off at best, and this check exists to catch a *wrong
 * position*, not to grade placement.  0.7 g corresponds to about 45° of tilt.
 */
#define ACCEL_MIN_HALF_RANGE (0.70f * CAL_G_MS2)

static int dominant_axis(const float v[3])
{
    int k = 0;
    for (int i = 1; i < 3; i++)
        if (fabsf(v[i]) > fabsf(v[k])) k = i;
    return k;
}

/* Which prompt number (1-based) asked for this axis/sign pair? */
static int position_number(int axis, int sign)
{
    for (int i = 0; i < 6; i++)
        if (cal_accel_positions[i].axis == axis &&
            cal_accel_positions[i].sign == sign)
            return i + 1;
    return 0;
}

int cal_accel_fit(const float meas[3][2][3], float offset[3], float scale[3],
                  char *err, size_t errsz)
{
    static const char axis_name[3] = { 'X', 'Y', 'Z' };
    float off[3], sc[3];

    for (int k = 0; k < 3; k++) {
        const float *up   = meas[k][0];   /* axis k pointing up   -> +g */
        const float *down = meas[k][1];   /* axis k pointing down -> -g */
        int n_up   = position_number(k, +1);
        int n_down = position_number(k, -1);

        /*
         * Gravity has to have landed on the axis we asked for.  If it did
         * not, the board was in the wrong orientation and the pair says
         * nothing about axis k — which is the failure that would
         * pass silently as scale = 1.0.
         */
        int dom_up   = dominant_axis(up);
        int dom_down = dominant_axis(down);
        if (dom_up != k || dom_down != k) {
            int bad     = (dom_up != k) ? dom_up : dom_down;
            int bad_pos = (dom_up != k) ? n_up   : n_down;
            snprintf(err, errsz,
                     "Axis %c: gravity landed on %c, not %c.\n"
                     "    up-face   read [%+.2f, %+.2f, %+.2f]\n"
                     "    down-face read [%+.2f, %+.2f, %+.2f]\n"
                     "  Position %d was not the orientation it asked for — "
                     "re-read the prompt and repeat the run.",
                     axis_name[k], axis_name[bad], axis_name[k],
                     (double)up[0],   (double)up[1],   (double)up[2],
                     (double)down[0], (double)down[1], (double)down[2],
                     bad_pos);
            return -1;
        }

        off[k] = (up[k] + down[k]) / 2.0f;
        float half_range = (up[k] - down[k]) / 2.0f;

        /* Negative half-range means the two faces were taken the wrong way
         * round: the "up" reading is the one that came out negative. */
        if (half_range < 0.0f) {
            snprintf(err, errsz,
                     "Axis %c: the two readings are inverted.\n"
                     "    up-face   read %+.3f (expected about %+.3f)\n"
                     "    down-face read %+.3f (expected about %+.3f)\n"
                     "  Positions %d and %d appear to have been swapped.",
                     axis_name[k],
                     (double)up[k],   (double)CAL_G_MS2,
                     (double)down[k], (double)-CAL_G_MS2,
                     n_up < n_down ? n_up : n_down,
                     n_up < n_down ? n_down : n_up);
            return -1;
        }

        if (half_range < ACCEL_MIN_HALF_RANGE) {
            snprintf(err, errsz,
                     "Axis %c: half-range %.3f m/s^2 is far below gravity.\n"
                     "    up-face   read %+.3f (expected about %+.3f)\n"
                     "    down-face read %+.3f (expected about %+.3f)\n"
                     "  The board was not square with gravity at position "
                     "%d or %d.",
                     axis_name[k], (double)half_range,
                     (double)up[k],   (double)CAL_G_MS2,
                     (double)down[k], (double)-CAL_G_MS2,
                     n_up < n_down ? n_up : n_down,
                     n_up < n_down ? n_down : n_up);
            return -1;
        }

        sc[k] = CAL_G_MS2 / half_range;
    }

    /* Only commit once all three axes have passed. */
    for (int k = 0; k < 3; k++) { offset[k] = off[k]; scale[k] = sc[k]; }
    return 0;
}

/* ── Heading-circle coverage (guided swing cal) ──────────────────────────── */

int cal_cov_mark(int *sectors, int nsec, double x, double y,
                 double cx, double cy)
{
    double angle = atan2(y - cy, x - cx);
    if (angle < 0) angle += 2.0 * M_PI;
    int s = (int)(angle / (2.0 * M_PI / nsec)) % nsec;
    sectors[s] = 1;
    return s;
}

int cal_cov_count(const int *sectors, int nsec)
{
    int n = 0;
    for (int i = 0; i < nsec; i++) n += sectors[i];
    return n;
}

/* ── Allan-variance sensor characterization ──────────────────────────────── */

int allan_deviation(const double *x, size_t n, double fs,
                    avar_pt_t *pts, int max_pts)
{
    if (n < 8 || fs <= 0.0 || max_pts <= 0) return 0;

    /* θ(t): integral of the signal; the overlapping Allan variance is
     *   σ²(τ) = Σ (θ[k+2m] − 2θ[k+m] + θ[k])² / (2 τ² (n − 2m + 1))     */
    double *theta = malloc((n + 1) * sizeof(*theta));
    if (!theta) return 0;

    double dt = 1.0 / fs;
    theta[0] = 0.0;
    for (size_t k = 0; k < n; k++)
        theta[k + 1] = theta[k] + x[k] * dt;

    int np = 0;
    for (size_t m = 1; 2 * m < n && np < max_pts; m *= 2) {
        double tau   = (double)m * dt;
        double sum   = 0.0;
        size_t terms = n - 2 * m + 1;
        for (size_t k = 0; k <= n - 2 * m; k++) {
            double d = theta[k + 2 * m] - 2.0 * theta[k + m] + theta[k];
            sum += d * d;
        }
        pts[np].tau_s = tau;
        pts[np].adev  = sqrt(sum / (2.0 * tau * tau * (double)terms));
        np++;
    }

    free(theta);
    return np;
}

int allan_characterize(const avar_pt_t *pts, int npts,
                       double *noise_density, double *bias_instability)
{
    if (npts < 3) return -1;

    /* White-noise (angle/velocity random walk) region: σ(τ) = N/√τ, so
     * N = σ(τ)·√τ — most accurate at the shortest cluster time, where the
     * white component dominates everything slower. */
    if (noise_density)
        *noise_density = pts[0].adev * sqrt(pts[0].tau_s);

    int imin = 0;
    for (int i = 1; i < npts; i++)
        if (pts[i].adev < pts[imin].adev) imin = i;

    /* Standard IEEE 952 scale factor for the 1/f floor. */
    if (bias_instability)
        *bias_instability = 0.664 * pts[imin].adev;

    return imin;
}

/* ── Gyro-bias / temperature linear fit ──────────────────────────────────── */

int gyro_temp_fit(const double *temp_c, const double *gyro, size_t n,
                  double ref_c, double *coeff, double *bias_ref)
{
    if (n < 2) return -1;

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    double tmin = temp_c[0], tmax = temp_c[0];
    for (size_t i = 0; i < n; i++) {
        double t = temp_c[i] - ref_c;
        sx  += t;
        sy  += gyro[i];
        sxx += t * t;
        sxy += t * gyro[i];
        if (temp_c[i] < tmin) tmin = temp_c[i];
        if (temp_c[i] > tmax) tmax = temp_c[i];
    }
    if (tmax - tmin < 1.0) return -1;   /* span too small for a stable slope */

    double det = (double)n * sxx - sx * sx;
    if (det <= 0.0) return -1;

    double b = ((double)n * sxy - sx * sy) / det;
    double a = (sy - b * sx) / (double)n;
    if (coeff)    *coeff    = b;
    if (bias_ref) *bias_ref = a;
    return 0;
}
