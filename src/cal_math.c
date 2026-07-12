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
#include <stdlib.h>
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

void ellipse_add(ellipse_accum_t *a, float fx, float fy)
{
    double x = fx, y = fy;
    a->s40 += x*x*x*x;  a->s31 += x*x*x*y;  a->s22 += x*x*y*y;
    a->s13 += x*y*y*y;  a->s04 += y*y*y*y;
    a->s20 += x*x;      a->s11 += x*y;      a->s02 += y*y;
    a->n++;
}

int ellipse_fit(const ellipse_accum_t *a, double radius, double S[2][2])
{
    if (a->n < 8 || radius <= 0.0) return -1;

    /*
     * Least squares on A·x² + B·xy + C·y² = 1: normal equations over the
     * regressors (x², xy, y²).  Solved via gauss4 with an identity pad row.
     */
    double M4[4][4] = {
        { a->s40, a->s31, a->s22, 0 },
        { a->s31, a->s22, a->s13, 0 },
        { a->s22, a->s13, a->s04, 0 },
        { 0,      0,      0,      1 },
    };
    double b4[4] = { a->s20, a->s11, a->s02, 0 };
    double p[4];
    if (gauss4(M4, b4, p) < 0) return -1;

    /* Conic matrix M = [[A, B/2], [B/2, C]] must be positive definite. */
    double A = p[0], B = p[1], C = p[2];
    double det = A*C - 0.25*B*B;
    if (A <= 0.0 || det <= 0.0) return -1;

    /* S = radius · sqrtm(M) via 2×2 symmetric eigendecomposition. */
    double half_tr = 0.5*(A + C);
    double disc    = sqrt(0.25*(A - C)*(A - C) + 0.25*B*B);
    double l1 = half_tr + disc, l2 = half_tr - disc;
    if (l2 <= 0.0) return -1;

    /* Eigenvector for l1 (pick the better-conditioned expression). */
    double vx, vy;
    if (fabs(B) > 1e-12) { vx = 0.5*B; vy = l1 - A; }
    else if (A >= C)     { vx = 1.0;   vy = 0.0;    }
    else                 { vx = 0.0;   vy = 1.0;    }
    double vn = sqrt(vx*vx + vy*vy);
    if (vn < 1e-15) return -1;
    vx /= vn; vy /= vn;

    double s1 = radius * sqrt(l1), s2 = radius * sqrt(l2);
    /* S = s1·v·vᵀ + s2·v⊥·v⊥ᵀ with v⊥ = (−vy, vx) */
    S[0][0] = s1*vx*vx + s2*vy*vy;
    S[0][1] = s1*vx*vy - s2*vx*vy;
    S[1][0] = S[0][1];
    S[1][1] = s1*vy*vy + s2*vx*vx;
    return 0;
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
