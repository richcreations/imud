/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fusion.c — MEKF sensor fusion implementation (§6)
 *
 * ── Conventions ────────────────────────────────────────────────────────────
 * q = [w, x, y, z] represents the body→NED rotation: v_NED = R(q) × v_body.
 * Error-state δx = [δθ(3), δb(3)] (Solà §5.4).
 * P is 6×6, top-left 3×3 is attitude error covariance (rad²).
 *
 * Gravity reference:    g_ref = [0, 0, 1]  (unit gravity in NED, Z-down)
 * Accel measurement:    z_a   = −normalize(accel_body)  [gravity direction]
 *   Drivers output specific force in the NED-compatible board frame (Z-down);
 *   at rest, accel_body[Z] ≈ −g.  Negating gives the gravity direction in
 *   the body frame for direct comparison with g_ref = [0, 0, +1].
 *
 * Magnetometer:         z_m   = mag_body  [calibrated µT, NED board frame]
 *   m_ref is the Earth field vector in NED (with magnitude), set at alignment.
 *   Predicted:          h_m   = R^T × m_ref  (expected reading in body frame)
 *
 * Noise model (discrete-time, per predict step):
 *   Q_gyro  = Ng² / fs × I₃   rad²       (angle error noise per step)
 *   Q_bias  = Nb² / fs × I₃   (rad/s)²   (bias random-walk per step)
 *   R_accel = (Na/g)² × fs    (normalised, per sample)
 *   R_mag   = Nm² × fs_mag    Gauss²      (per sample at mag ODR)
 * where Ng, Nb, Na, Nm are the datasheet noise densities from config.
 */

#include <math.h>
#include <string.h>
#include <stddef.h>
#include "fusion.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define G_MS2  9.80665f   /* standard gravity, m/s² */

/* ── Scalar helpers ────────────────────────────────────────────────────────── */

static inline float fsq(float x) { return x * x; }

/* ── 3-vector helpers ──────────────────────────────────────────────────────── */

static inline float v3_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline float v3_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/* ── 3×3 matrix helpers ────────────────────────────────────────────────────── */

/* y = Aᵀ × x */
static void m33_Atx(const float A[3][3], const float x[3], float y[3])
{
    for (int i = 0; i < 3; i++) {
        float s = 0;
        for (int k = 0; k < 3; k++) s += A[k][i] * x[k];
        y[i] = s;
    }
}

/* 3×3 inverse via Cramer's rule; returns -1 if singular */
static int m33_inv(const float A[3][3], float Ai[3][3])
{
    float d = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
            - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
            + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    if (fabsf(d) < 1e-12f) return -1;
    float id = 1.0f / d;
    Ai[0][0] = (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * id;
    Ai[0][1] = (A[0][2]*A[2][1] - A[0][1]*A[2][2]) * id;
    Ai[0][2] = (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * id;
    Ai[1][0] = (A[1][2]*A[2][0] - A[1][0]*A[2][2]) * id;
    Ai[1][1] = (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * id;
    Ai[1][2] = (A[0][2]*A[1][0] - A[0][0]*A[1][2]) * id;
    Ai[2][0] = (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * id;
    Ai[2][1] = (A[0][1]*A[2][0] - A[0][0]*A[2][1]) * id;
    Ai[2][2] = (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * id;
    return 0;
}

/* ── 6×6 matrix helpers ────────────────────────────────────────────────────── */

static void m66_mul(const float A[6][6], const float B[6][6], float C[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float s = 0;
            for (int k = 0; k < 6; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

/* C = A × Bᵀ  (both 6×6) */
static void m66_mulT(const float A[6][6], const float B[6][6], float C[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float s = 0;
            for (int k = 0; k < 6; k++) s += A[i][k] * B[j][k];
            C[i][j] = s;
        }
}

/* ── Quaternion utilities ───────────────────────────────────────────────────── */

/* Hamilton product: c = a ⊗ b  (Solà eq. 16) */
static void q_mul(const float a[4], const float b[4], float c[4])
{
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void q_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-10f) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
}

/*
 * Rotation matrix R from unit quaternion (body→NED).
 * v_NED = R × v_body.  (Solà eq. 129)
 */
static void q_to_R(const float q[4], float R[3][3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z);   R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x);   R[2][2]=1-2*(x*x+y*y);
}

/*
 * Quaternion exponential map: rotation vector v → unit quaternion δq.
 * δq = [cos(|v|/2), sin(|v|/2)/|v| × v]
 * Small-angle approximation for |v| < 1e-7.  (Solà eq. 193)
 */
static void q_from_rotvec(const float v[3], float dq[4])
{
    float th = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (th < 1e-7f) {
        dq[0] = 1.0f;
        dq[1] = v[0] * 0.5f;
        dq[2] = v[1] * 0.5f;
        dq[3] = v[2] * 0.5f;
    } else {
        float s = sinf(th * 0.5f) / th;
        dq[0] = cosf(th * 0.5f);
        dq[1] = v[0] * s;
        dq[2] = v[1] * s;
        dq[3] = v[2] * s;
    }
    q_normalize(dq);
}

/* ── Core MEKF operations ──────────────────────────────────────────────────── */

/*
 * Generic measurement update.  (Solà §4.2)
 *
 * h:         predicted measurement in body frame (3×1)
 * z:         actual measurement in body frame (3×1)
 * R_noise:   isotropic measurement noise variance (scalar, applied to I₃)
 * chi2_gate: innovation gate — skip the update when the normalised
 *            innovation distance d² = innovᵀ·S⁻¹·innov exceeds this
 *            (χ²₃ quantile); 0 disables the gate.
 *
 * H = [-[h×]  |  0₃]  (3×6)  — Jacobian of h w.r.t. error-state δθ
 * (Solà eq. 276, with sign from body-frame observation of a NED reference.)
 *
 * Returns 0 on success, -1 if S is singular, +1 if the innovation gate
 * rejected the measurement (update skipped either way).
 */
static int eskf_update(mekf_t *f,
                       const float h[3],
                       const float z[3],
                       float R_noise,
                       float chi2_gate)
{
    /*
     * H (3×6): H[0:3][0:3] = +[h×], H[0:3][3:6] = 0₃
     *
     * Derivation (right-multiply error convention, q_true = q̂ ⊗ δq):
     *   R(q_true) ≈ R(q̂) × (I + [δθ×])
     *   h_true = R(q_true)ᵀ × g ≈ ĥ − [δθ×]ĥ = ĥ + [ĥ×]δθ
     *   → H_δθ = +[ĥ×]   (Solà §7, right-perturbation)
     *
     * +[h×] = [[  0,  -h2,  h1 ],
     *           [ h2,   0,  -h0 ],
     *           [-h1,  h0,   0  ]]
     */
    const float H[3][6] = {
        {     0, -h[2],  h[1],  0, 0, 0 },
        {  h[2],     0, -h[0],  0, 0, 0 },
        { -h[1],  h[0],     0,  0, 0, 0 },
    };

    /* PHᵀ (6×3):  PHt[i][j] = Σ_k P[i][k] × H[j][k] */
    float PHt[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0;
            for (int k = 0; k < 6; k++) s += f->P[i][k] * H[j][k];
            PHt[i][j] = s;
        }

    /* S = H × PHᵀ + R_noise × I₃  (3×3) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = (i == j) ? R_noise : 0.0f;
            for (int k = 0; k < 6; k++) s += H[i][k] * PHt[k][j];
            S[i][j] = s;
        }

    float Sinv[3][3];
    if (m33_inv(S, Sinv) < 0) return -1;

    /* K = PHᵀ × S⁻¹  (6×3) */
    float K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += PHt[i][k] * Sinv[k][j];
            K[i][j] = s;
        }

    /* innovation = z − h  (3×1) */
    float innov[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };

    /* Innovation handling: d² = innovᵀ·S⁻¹·innov follows χ²(3) when the
     * measurement is consistent with the filter. Wave-orbital acceleration
     * can keep |a| ≈ g while swinging the DIRECTION — the magnitude gate
     * passes those. A hard reject would starve the filter in a steady
     * seaway, so instead the influence is capped Huber-style (scale the
     * innovation so its effective distance equals the gate) and only gross
     * outliers (9× the gate) are rejected outright. */
    if (chi2_gate > 0.0f) {
        /* No convergence gate needed: S = HPHᵀ + R already contains P, so
         * during acquisition (large P) d² stays small and the cap is
         * naturally inactive; it engages only once the filter is confident. */
        float Si[3];
        for (int i = 0; i < 3; i++)
            Si[i] = Sinv[i][0]*innov[0] + Sinv[i][1]*innov[1]
                  + Sinv[i][2]*innov[2];
        float d2 = innov[0]*Si[0] + innov[1]*Si[1] + innov[2]*Si[2];
        if (d2 > 9.0f * chi2_gate) return 1;
        if (d2 > chi2_gate) {
            float w = sqrtf(chi2_gate / d2);
            innov[0] *= w; innov[1] *= w; innov[2] *= w;
        }
    }

    /* δx = K × innovation  (6×1) */
    float dx[6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) dx[i] += K[i][j] * innov[j];

    /* Multiplicative quaternion correction: q ← q ⊗ exp(δθ)  (Solà eq. 280) */
    float dq[4];
    q_from_rotvec(dx, dq);   /* dx[0:3] = δθ */
    float qnew[4];
    q_mul(f->q, dq, qnew);
    memcpy(f->q, qnew, sizeof f->q);
    q_normalize(f->q);

    /* Bias correction: b ← b + δb */
    f->bias[0] += dx[3];
    f->bias[1] += dx[4];
    f->bias[2] += dx[5];

    /*
     * Covariance update, Joseph form:
     *   P ← (I − K·H) · P · (I − K·H)ᵀ + K·R·Kᵀ
     * Guarantees symmetry and positive-definiteness in float arithmetic —
     * the simple (I−KH)·P form slowly loses both at 833 Hz over multi-day
     * runs. R is isotropic (R_noise·I₃), so K·R·Kᵀ = R_noise·K·Kᵀ.
     */
    float KH[6][6] = {{0}};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++) KH[i][j] += K[i][k] * H[k][j];

    float IKH[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

    float Ptmp[6][6], Pnew[6][6];
    m66_mul(IKH, f->P, Ptmp);
    m66_mulT(Ptmp, IKH, Pnew);

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += K[i][k] * K[j][k];
            Pnew[i][j] += R_noise * s;
        }

    /* Enforce exact symmetry (cheap insurance against float round-off). */
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < i; j++) {
            float a = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = Pnew[j][i] = a;
        }

    memcpy(f->P, Pnew, sizeof f->P);

    return 0;
}

/*
 * Scalar yaw measurement update (mag_yaw_only mode).
 *
 * y:       heading innovation, rad, wrapped to ±π
 * R_noise: heading measurement variance, rad²
 *
 * For the right-multiplied error convention, a heading innovation relates
 * to the error state through the NED-down axis expressed in body terms:
 *   y ≈ −(R̂·δθ)_D  →  H = [ −R̂[2][:] | 0₃ ]  (1×6)
 * Same Joseph-form covariance update as the vector path.
 */
/* χ²(1) 99% quantile — yaw innovation gate. */
#define YAW_CHI2_GATE 6.63f

static int eskf_update_yaw(mekf_t *f, float y, float R_noise)
{
    float R[3][3];
    q_to_R(f->q, R);

    float H[6] = { -R[2][0], -R[2][1], -R[2][2], 0, 0, 0 };

    float PHt[6];
    for (int i = 0; i < 6; i++) {
        float s = 0;
        for (int k = 0; k < 6; k++) s += f->P[i][k] * H[k];
        PHt[i] = s;
    }

    float S = R_noise;
    for (int i = 0; i < 6; i++) S += H[i] * PHt[i];
    if (S < 1e-12f) return -1;

    /* Same Huber policy as the vector update: self-regulating via S. */
    float d2 = y * y / S;
    if (d2 > 9.0f * YAW_CHI2_GATE) return 1;
    if (d2 > YAW_CHI2_GATE) y *= sqrtf(YAW_CHI2_GATE / d2);

    float K[6];
    for (int i = 0; i < 6; i++) K[i] = PHt[i] / S;

    float dx[6];
    for (int i = 0; i < 6; i++) dx[i] = K[i] * y;

    float dq[4], qnew[4];
    q_from_rotvec(dx, dq);
    q_mul(f->q, dq, qnew);
    memcpy(f->q, qnew, sizeof f->q);
    q_normalize(f->q);

    f->bias[0] += dx[3];
    f->bias[1] += dx[4];
    f->bias[2] += dx[5];

    /* Joseph form, rank-1: P ← (I−KH)·P·(I−KH)ᵀ + R·K·Kᵀ */
    float IKH[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];

    float Ptmp[6][6], Pnew[6][6];
    m66_mul(IKH, f->P, Ptmp);
    m66_mulT(Ptmp, IKH, Pnew);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            Pnew[i][j] += R_noise * K[i] * K[j];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < i; j++) {
            float a = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = Pnew[j][i] = a;
        }
    memcpy(f->P, Pnew, sizeof f->P);

    return 0;
}

/* ── Public API ────────────────────────────────────────────────────────────── */

void mekf_init(mekf_t *f,
               const imud_config_t *cfg,
               float odr_hz,
               const float gyro_bias_init[3])
{
    memset(f, 0, sizeof *f);

    /* Nominal state: identity attitude, pre-estimated bias */
    f->q[0] = 1.0f;
    if (gyro_bias_init) {
        f->bias[0] = gyro_bias_init[0];
        f->bias[1] = gyro_bias_init[1];
        f->bias[2] = gyro_bias_init[2];
    }

    /*
     * Initial covariance: generous values so the filter accepts early
     * measurements.  Attitude uncertainty ~10° (0.175 rad), bias from startup
     * still window (small).
     */
    float init_att  = fsq(0.175f);    /* ~10° per axis */
    float init_bias = fsq(0.001f);    /* 1 mdps/s residual bias uncertainty */
    for (int i = 0; i < 3; i++) f->P[i][i]     = init_att;
    for (int i = 3; i < 6; i++) f->P[i][i] = init_bias;

    float dt = 1.0f / odr_hz;
    f->dt = dt;

    /*
     * Discrete-time process noise (Solà §4.1):
     *   Qg = Ng² × dt  [rad² per step, where Ng = gyro noise density rad/s/√Hz]
     *   Qb = Nb² × dt  [(rad/s)² per step, random-walk on bias]
     * Using mekf_gyro_bias as the bias noise density (rad/s/√Hz equivalent).
     */
    float Ng = (float)cfg->mekf_gyro_noise;
    float Nb = (float)cfg->mekf_gyro_bias;
    f->Qg = Ng * Ng * dt;
    f->Qb = Nb * Nb * dt;

    /*
     * Measurement noise variances per sample:
     *   Ra: accel noise density Na m/s²/√Hz → normalised to gravity units,
     *       variance at odr_hz = (Na/g)² × odr_hz
     *   Rm: mag noise density Nm Gauss/√Hz → variance at mag_odr_hz
     *       = Nm² × mag_odr_hz  (use 100 Hz default)
     */
    float Na = (float)cfg->mekf_accel_noise;
    float Nm = (float)cfg->mekf_mag_noise;
    f->Ra = (Na / G_MS2) * (Na / G_MS2) * odr_hz;
    f->Rm = Nm * Nm * (float)cfg->mag_odr_hz;

    float sk = (float)cfg->accel_skip_thresh;
    f->accel_skip_lo = 1.0f - sk;
    f->accel_skip_hi = 1.0f + sk;

    float mr = (float)cfg->mag_reject_gauss;
    f->mag_reject_sq = mr * mr;

    f->Ra_scale = 1.0f;   /* raised by fusion_thread during engine vibration */
    f->acc_quiet_ema = 1.0f;   /* start "loud"; must earn quiescence from data */

    f->mag_yaw_only = cfg->mag_yaw_only;

    /* m_ref EMA rate: one accepted sample per 1/mag_odr_hz, τ = 300 s. */
#ifndef MREF_TAU_S
#define MREF_TAU_S 300.0f
#endif
    f->mref_alpha = (cfg->mag_odr_hz > 0 && MREF_TAU_S < 1e9f)
        ? 1.0f / (MREF_TAU_S * (float)cfg->mag_odr_hz)
        : 0.0f;

    f->conv_thresh = 3.0f * fsq(0.5f * (float)(M_PI/180.0)); /* 3 × (0.5°)² */
}

void mekf_set_mref_invariants(mekf_t *f, float h_gauss, float z_gauss)
{
    /*
     * Override the magnetic reference's magnitude and dip with analytically
     * known values (WMM at a known position) while PRESERVING the horizontal
     * direction — the direction is the heading anchor established at
     * alignment and must never jump. With WMM-known invariants, the
     * alignment-tilt error that would otherwise bake into m_ref is removed
     * at the source; the quiescence-gated EMA then only has residual sensor
     * scale error left to track.
     */
    if (!f->m_ref_valid || h_gauss <= 0.0f) return;
    float mh = sqrtf(f->m_ref[0]*f->m_ref[0] + f->m_ref[1]*f->m_ref[1]);
    if (mh < 1e-6f) return;
    float s = h_gauss / mh;
    f->m_ref[0] *= s;
    f->m_ref[1] *= s;
    f->m_ref[2]  = z_gauss;
}

void mekf_align(mekf_t *f, const float accel[3], const float mag[3])
{
    /* ── 1. Tilt from accelerometer ──────────────────────────────────────
     * Gravity direction in body: g_hat = −normalize(accel) (specific force).
     * Roll and pitch in NED aerospace convention (Solà §5.1 example). */
    float an = v3_norm(accel);
    if (an < 0.5f * G_MS2) return;  /* wildly wrong; wait for better sample */

    float gx = -accel[0]/an, gy = -accel[1]/an, gz = -accel[2]/an;

    float roll  = atan2f(gy, gz);
    float pitch = atan2f(-gx, sqrtf(gy*gy + gz*gz));

    /* ── 2. Quaternion from roll + pitch (yaw = 0 initially) ──────────── */
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    /* q = q_pitch ⊗ q_roll (Solà §5.1) */
    float qtilt[4] = { cp*cr, cp*sr, sp*cr, -sp*sr };
    q_normalize(qtilt);

    /* ── 3. Heading from magnetometer ────────────────────────────────────
     * Rotate mag to NED, take atan2 of horizontal components. */
    float R[3][3];
    q_to_R(qtilt, R);

    /* m_lvl = R(qtilt) × mag_body — the field in the tilt-levelled frame,
     * which is the true NED field rotated by −yaw: [H·cosψ, −H·sinψ, Z].
     * Vessel yaw is therefore atan2(−m_y, m_x); the +m_y form returns −ψ
     * (heading mirrored about north — invisible only when aligning while
     * pointing north, which is what every early test happened to do). */
    float mnx = R[0][0]*mag[0] + R[0][1]*mag[1] + R[0][2]*mag[2];
    float mny = R[1][0]*mag[0] + R[1][1]*mag[1] + R[1][2]*mag[2];

    float yaw = atan2f(-mny, mnx);  /* magnetic heading in NED */

    /* ── 4. Full quaternion: q_yaw ⊗ q_tilt ──────────────────────────── */
    float cy = cosf(yaw*0.5f), sy = sinf(yaw*0.5f);
    float q_yaw[4] = { cy, 0, 0, sy };
    float qfull[4];
    q_mul(q_yaw, qtilt, qfull);
    q_normalize(qfull);
    memcpy(f->q, qfull, sizeof f->q);

    /* ── 5. Set magnetometer reference in NED (stored in Gauss) */
    if (!f->m_ref_valid) {
        float Rf[3][3];
        q_to_R(f->q, Rf);
        /* m_ref_NED = R(q) × mag_body, converted µT → Gauss */
        f->m_ref[0] = (Rf[0][0]*mag[0] + Rf[0][1]*mag[1] + Rf[0][2]*mag[2]) * 0.01f;
        f->m_ref[1] = (Rf[1][0]*mag[0] + Rf[1][1]*mag[1] + Rf[1][2]*mag[2]) * 0.01f;
        f->m_ref[2] = (Rf[2][0]*mag[0] + Rf[2][1]*mag[1] + Rf[2][2]*mag[2]) * 0.01f;
        f->m_ref_valid = true;
    }

    f->initialized = true;
}

void mekf_predict(mekf_t *f, const imu_sample_t *s, float dt_s)
{
    if (!f->initialized) return;

    /* Per-sample dt: callers pass the measured inter-sample interval
     * (hardware timestamps) so oscillator tolerance doesn't scale the
     * integrated rotation; f->dt (nominal) is the fallback. */
    float dt = (dt_s > 0.0f) ? dt_s : f->dt;

    /* Bias-corrected angular rate */
    float w[3] = {
        s->gyro[0] - f->bias[0],
        s->gyro[1] - f->bias[1],
        s->gyro[2] - f->bias[2],
    };

    /* ── Propagate quaternion: q ← q ⊗ exp(w × dt)  (Solà eq. 259) ─── */
    float rv[3] = { w[0]*dt, w[1]*dt, w[2]*dt };
    float dq[4];
    q_from_rotvec(rv, dq);
    float qnew[4];
    q_mul(f->q, dq, qnew);
    memcpy(f->q, qnew, sizeof f->q);
    q_normalize(f->q);

    /*
     * ── Covariance propagation: P ← Φ·P·Φᵀ + Q_d  (Solà eq. 268) ────
     *
     * Discrete transition matrix (first-order zero-hold):
     *   Φ = I₆ + Fc·dt
     *   Fc = [[-[w×]  -I₃], [0₃  0₃]]
     *   Φ = [[I₃ - [w×]dt   -I₃·dt],
     *         [0₃             I₃   ]]
     *
     * Diagonal process noise Q_d:
     *   Q_d[0:3][0:3] = Qg·I₃    Q_d[3:6][3:6] = Qb·I₃
     */
    float Phi[6][6] = {{0}};

    /* Top-left: I₃ − [w×]·dt */
    Phi[0][0] = 1.0f;           Phi[0][1] =  w[2]*dt;   Phi[0][2] = -w[1]*dt;
    Phi[1][0] = -w[2]*dt;       Phi[1][1] = 1.0f;        Phi[1][2] =  w[0]*dt;
    Phi[2][0] =  w[1]*dt;       Phi[2][1] = -w[0]*dt;    Phi[2][2] = 1.0f;
    /* Top-right: -I₃·dt */
    Phi[0][3] = -dt;  Phi[1][4] = -dt;  Phi[2][5] = -dt;
    /* Bottom-right: I₃ */
    Phi[3][3] = 1.0f; Phi[4][4] = 1.0f; Phi[5][5] = 1.0f;

    float Ptmp[6][6], Pnew[6][6];
    m66_mul(Phi, f->P, Ptmp);
    m66_mulT(Ptmp, Phi, Pnew);

    /* Add diagonal Q_d, scaled to the actual step length (Qg/Qb are stored
     * per nominal step; noise variance grows linearly with dt). */
    float qscale = dt / f->dt;
    for (int i = 0; i < 3; i++) Pnew[i][i] += f->Qg * qscale;
    for (int i = 3; i < 6; i++) Pnew[i][i] += f->Qb * qscale;

    /* Keep P exactly symmetric (ΦPΦᵀ picks up float round-off). */
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < i; j++) {
            float a = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = Pnew[j][i] = a;
        }

    memcpy(f->P, Pnew, sizeof f->P);

    /* Update convergence flag */
    float trace = f->P[0][0] + f->P[1][1] + f->P[2][2];
    f->converged = (trace < f->conv_thresh);
}

/* χ²(3) 99% quantile — accel innovation gate (overridable for experiments). */
#ifndef ACCEL_CHI2_GATE
#define ACCEL_CHI2_GATE 11.34f
#endif

void mekf_update_accel(mekf_t *f, const imu_sample_t *s)
{
    if (!f->initialized) return;

    /*
     * Speed-aided centripetal correction: in a turn the accelerometer
     * measures gravity plus ω×v, which tilts the apparent gravity exactly
     * when the autopilot most needs good roll. With speed over ground from
     * gpsd (v_body ≈ [v, 0, 0] along the bow — leeway neglected):
     *   ω×v = [0, ω_z·v, −ω_y·v]
     * Subtracting it turns coordinated-turn samples from χ²-gate rejects
     * into usable gravity references. speed_mps = 0 (no live GPS) is a
     * no-op.
     */
    float acc[3] = { s->accel[0], s->accel[1], s->accel[2] };
    if (f->speed_mps > 0.1f) {
        float wy = s->gyro[1] - f->bias[1];
        float wz = s->gyro[2] - f->bias[2];
        acc[1] -= wz * f->speed_mps;
        acc[2] += wy * f->speed_mps;
    }

    float an = v3_norm(acc);
    float ag = an / G_MS2;

    /* Quiescence tracker (τ ≈ 2 s): EMA of squared |a|-deviation from 1 g,
     * updated on EVERY sample including gated ones. The m_ref EMA learns
     * only while this says the platform is calm — in a seaway the wave
     * phase correlates with which samples pass the gates, and reference
     * learning from that biased subset drifts m_ref (measured: +10%
     * horizontal over 3 min of synthetic waves). */
    {
        float dev = ag - 1.0f;
        f->acc_quiet_ema += (dev * dev - f->acc_quiet_ema) * (f->dt / 2.0f);
    }

    /* Skip during linear acceleration (|a|/g outside [1±thresh]) */
    if (ag < f->accel_skip_lo || ag > f->accel_skip_hi) return;

    /* Gravity direction in body: z = −normalize(specific_force) */
    float z[3] = { -acc[0]/an, -acc[1]/an, -acc[2]/an };

    /* Predicted gravity in body: h = Rᵀ × [0, 0, 1] = third row of R */
    float R[3][3];
    q_to_R(f->q, R);
    float h[3] = { R[2][0], R[2][1], R[2][2] };

    /*
     * Ra_scale inflates the accel noise ×4 while engine vibration is
     * detected (set by fusion_thread) — vibration is high-frequency and
     * near-zero-mean, so deweighting (not gating) is the right response.
     * Wave-driven DIRECTION disturbance is handled by the χ² innovation
     * cap in eskf_update instead: benchmarking showed that deweighting by
     * |a|-magnitude deviation correlates with wave phase and rectifies the
     * direction error into an attitude/bias offset, so it is deliberately
     * NOT done here.
     */
    float R_eff = f->Ra * f->Ra_scale;

    /* Stash the accel-measured gravity direction (body frame) — the mag
     * m_ref EMA uses it as an attitude-independent dip reference. */
    f->g_body[0] = z[0];
    f->g_body[1] = z[1];
    f->g_body[2] = z[2];
    f->g_body_valid = true;

    eskf_update(f, h, z, R_eff, ACCEL_CHI2_GATE);
}

void mekf_update_mag(mekf_t *f, const mag_sample_t *m)
{
    if (!f->initialized || !f->m_ref_valid || !m->valid) return;

    /* Convert µT → Gauss for comparison with m_ref (stored in Gauss) */
    float mx = m->field[0] * 0.01f;
    float my = m->field[1] * 0.01f;
    float mz = m->field[2] * 0.01f;

    /* Predicted mag in body: h_raw = Rᵀ × m_ref */
    float R[3][3];
    q_to_R(f->q, R);
    float h_raw[3];
    m33_Atx(R, f->m_ref, h_raw);

    float h_mag = v3_norm(h_raw);
    if (h_mag < 1e-6f) return;

    /*
     * Normalise both predicted and measured vectors before eskf_update.
     *
     * Attitude depends only on the direction of the Earth's field, not its
     * magnitude.  More critically, when P_att is small (after accel
     * convergence), the S matrix in Gauss² units has det ≈ Rm³ ≈ 4e-15,
     * below the mat33_inv singularity threshold.  Normalising makes S
     * dimensionless (elements ≈ P_att ~ 0.03), det ≈ 1e-9, always
     * invertible.  Rm is rescaled by 1/|h|² to preserve optimal gain.
     */
    float h[3] = { h_raw[0]/h_mag, h_raw[1]/h_mag, h_raw[2]/h_mag };
    float z_mag = sqrtf(mx*mx + my*my + mz*mz);
    if (z_mag < 1e-6f) return;
    float z[3] = { mx/z_mag, my/z_mag, mz/z_mag };
    float Rm_n = f->Rm / (h_mag * h_mag);

    /* Residual in normalised units */
    float innov[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };
    float res_sq = v3_dot(innov, innov);

    /*
     * Compass health diagnostics — updated BEFORE the rejection gates below:
     * the gated-out samples are precisely the anomalies these metrics exist
     * to surface (averaging only accepted innovations would hide a hard
     * fault behind a small residual). Fixed EMA, τ ≈ 30 s at the 100 Hz mag
     * ODR (τ scales with a nonstandard mag rate; diagnostics only).
     *
     * mag_anom_ema:  fractional field-magnitude deviation — attitude-
     *                independent (|h_raw| = |m_ref|), catches interference
     *                and gross iron-cal drift.
     * mag_resid_ema: |heading innovation| in rad, mode-independent (computed
     *                here even under 3D fusion) — "the compass disagrees
     *                with the filter by X rad" = calibration health.
     */
    {
        const float alpha = 1.0f / 3000.0f;
        float anom = fabsf(z_mag - h_mag) / h_mag;
        f->mag_anom_ema += alpha * (anom - f->mag_anom_ema);

        float m_ned[3];
        for (int i = 0; i < 3; i++)
            m_ned[i] = R[i][0]*mx + R[i][1]*my + R[i][2]*mz;
        float mh_ref = sqrtf(f->m_ref[0]*f->m_ref[0] + f->m_ref[1]*f->m_ref[1]);
        float mh_mea = sqrtf(m_ned[0]*m_ned[0] + m_ned[1]*m_ned[1]);
        if (mh_ref > 0.2f * h_mag && mh_mea > 0.2f * h_mag) {
            float y = atan2f(m_ned[1], m_ned[0])
                    - atan2f(f->m_ref[1], f->m_ref[0]);
            while (y >  (float)M_PI) y -= 2.0f*(float)M_PI;
            while (y < -(float)M_PI) y += 2.0f*(float)M_PI;
            f->mag_resid_ema += alpha * (fabsf(y) - f->mag_resid_ema);
        }
    }

    /*
     * Magnitude gate (always): if the measured field magnitude differs from
     * the expected Earth-field magnitude by more than 50 %, it is a sensor
     * fault or a strong nearby magnet — never pass to the filter.
     * This catches scenarios that normalisation would otherwise hide
     * (e.g. a 10-Gauss anomaly pointing in a plausible direction).
     */
    float mag_ratio = z_mag / h_mag;
    if (mag_ratio < 0.5f || mag_ratio > 2.0f) return;

    /* Direction gate (always): quaternion correction > 90° is nonsensical. */
    if (res_sq > 4.0f) return;

    /*
     * Slow m_ref re-estimation (τ ≈ 5 min): adapt only the field MAGNITUDE
     * and DIP, never the horizontal direction. The horizontal direction IS
     * the heading reference — updating it from the filter's own attitude
     * would feed the heading estimate back into its own reference and let
     * both random-walk away together (gauge drift). Magnitude and dip carry
     * the alignment-tilt error baked in at startup and the slow change of
     * the Earth field along a voyage; neither feeds back into heading.
     *
     * Both targets are ATTITUDE-INDEPENDENT body-frame invariants: the
     * total field magnitude, and the dip from the angle between the mag
     * vector and the accel-measured gravity direction. The filter's
     * attitude never enters, so a wrong attitude can neither hide the
     * reference error nor be fed by the healing (no standoff, no gauge
     * feedback). Runs whenever measurement and prediction roughly agree
     * (direction within ~18°) — deliberately NOT gated on the tight
     * converged-only anomaly threshold below, which would reject exactly
     * the persistent reference error this exists to heal.
     */
    if (f->mref_alpha > 0.0f && f->g_body_valid && res_sq < 0.1f &&
        f->acc_quiet_ema < 2e-4f /* platform calm: |a| within ~1.4% of g */) {
        float sin_dip = (mx*f->g_body[0] + my*f->g_body[1] + mz*f->g_body[2])
                        / z_mag;
        if (sin_dip >  1.0f) sin_dip =  1.0f;
        if (sin_dip < -1.0f) sin_dip = -1.0f;
        float cos_dip = sqrtf(1.0f - sin_dip*sin_dip);
        float tgt_v = z_mag * sin_dip;   /* target vertical component (down +) */
        float tgt_h = z_mag * cos_dip;   /* target horizontal magnitude */

        float mh_ref = sqrtf(f->m_ref[0]*f->m_ref[0] + f->m_ref[1]*f->m_ref[1]);
        if (mh_ref > 1e-6f) {
            float scale = 1.0f + f->mref_alpha * (tgt_h / mh_ref - 1.0f);
            f->m_ref[0] *= scale;
            f->m_ref[1] *= scale;
        }
        f->m_ref[2] += f->mref_alpha * (tgt_v - f->m_ref[2]);
    }

    /* Tight anomaly threshold (converged only), scaled to normalised units. */
    if (f->converged && res_sq > f->mag_reject_sq / (h_mag * h_mag)) return;

    int rc;
    if (f->mag_yaw_only) {
        /*
         * Heading-only fusion (marine default): the swing-circle mag cal is
         * structurally 2D, so the field's dip component is the least
         * calibrated channel — it must not pull on roll/pitch. Rotate the
         * measurement into NED with the current attitude and innovate on
         * the horizontal field direction only.
         */
        float m_ned[3];
        for (int i = 0; i < 3; i++)
            m_ned[i] = R[i][0]*mx + R[i][1]*my + R[i][2]*mz;

        float mh_ref = sqrtf(f->m_ref[0]*f->m_ref[0] + f->m_ref[1]*f->m_ref[1]);
        float mh_mea = sqrtf(m_ned[0]*m_ned[0] + m_ned[1]*m_ned[1]);
        if (mh_ref < 0.2f * h_mag || mh_mea < 0.2f * h_mag)
            return;   /* nearly vertical field (magnetic poles) — no heading info */

        float y = atan2f(m_ned[1], m_ned[0])
                - atan2f(f->m_ref[1], f->m_ref[0]);
        while (y >  (float)M_PI) y -= 2.0f*(float)M_PI;
        while (y < -(float)M_PI) y += 2.0f*(float)M_PI;

        /* Heading noise: per-axis field noise over the horizontal magnitude */
        float R_psi = f->Rm / (mh_ref * mh_ref);
        rc = eskf_update_yaw(f, y, R_psi);
    } else {
        rc = eskf_update(f, h, z, Rm_n, ACCEL_CHI2_GATE);
    }
    (void)rc;
}

/* ── Heave estimator ───────────────────────────────────────────────────────── */

/*
 * Leaky double integration of NED vertical linear acceleration, followed by
 * a true first-order high-pass.
 *
 * a_down = (R(q)·accel_body)_D + g   — zero at rest, wave heave otherwise.
 * The two leaky integration stages bound drift, but they still have a DC
 * gain of τ² (≈144 m per m/s² at the default τ) — any slow residual (accel
 * bias, small tilt error, start-up transients) would appear metres-scale.
 * The output high-pass has an exact zero at DC, so those residuals are
 * killed structurally rather than estimated. Passband covers ordinary wave
 * periods: amplitude error ≈ 2 % at 8 s, ≈ 4 % at 12 s for τ = 12 s.
 * Output is heave in metres, positive UP (PASHR convention).
 */
/* Heave settles after ~10 time constants (≈2 min at the default τ = 12 s). */
#define HEAVE_SETTLE_FACTOR 10.0f

void heave_init(heave_t *h, float tau_s, float dt)
{
    memset(h, 0, sizeof *h);
    h->tau     = tau_s;
    h->dt      = dt;
    h->enabled = (tau_s > 0.0f && dt > 0.0f);
}

float heave_update(heave_t *h, const float q[4], const float accel[3])
{
    if (!h->enabled) return 0.0f;

    h->elapsed += h->dt;
    if (!h->settled && h->elapsed >= HEAVE_SETTLE_FACTOR * h->tau)
        h->settled = true;

    float R[3][3];
    q_to_R(q, R);
    float a_down = R[2][0]*accel[0] + R[2][1]*accel[1] + R[2][2]*accel[2]
                 + G_MS2;

    float leak = h->dt / h->tau;
    h->vel  += a_down * h->dt;
    h->vel  -= h->vel * leak;
    h->disp += h->vel * h->dt;
    h->disp -= h->disp * leak;

    /* First-order high-pass: y = α·(y + x − x_prev), exact zero at DC. */
    float alpha = h->tau / (h->tau + h->dt);
    h->hp_y = alpha * (h->hp_y + h->disp - h->disp_prev);
    h->disp_prev = h->disp;

    return -h->hp_y;   /* NED down → heave positive up */
}

/* ── Sea-state estimator ───────────────────────────────────────────────────── */

/*
 * Exponentially weighted mean/variance pairs over heave, heave rate, roll,
 * and roll rate; spectral moments give the sea-state outputs directly:
 *
 *   m0 = var(x), m2 = var(ẋ)  ⇒  Tz = 2π·√(m0/m2)  (exact for a pure sine:
 *   var = A²/2, var-rate = A²ω²/2 ⇒ ratio = 1/ω²), and Hs = 4·√m0 — the
 *   standard significant-height estimate for a Gaussian sea.
 *
 * The EW window (τ = wave_tau_s) trades responsiveness against stability:
 * oceanographic practice is 10–20 min records, but a live display wants
 * minutes, so the default is 120 s and the key is hot-reloadable. Stats are
 * fed only while the heave estimator is settled — its settling ramp is a
 * huge low-frequency transient that would dominate both variances.
 */
#define SEASTATE_SETTLE_FACTOR 2.0f
/* Below these oscillation floors the period ratio times only sensor noise. */
#define SEASTATE_MIN_HEAVE_SIG 0.02f              /* σ(heave) ≥ 2 cm  */
#define SEASTATE_MIN_ANGLE_SIG (0.3f*(float)M_PI/180.0f) /* σ(angle) ≥ 0.3° */

void seastate_init(seastate_t *w, float tau_s, float dt)
{
    memset(w, 0, sizeof *w);
    w->tau     = tau_s;
    w->dt      = dt;
    w->enabled = (tau_s > 0.0f && dt > 0.0f);
}

/* One EW mean/variance step: var converges to the variance about the EW mean. */
static void ew_stat(float *mean, float *var, float x, float alpha)
{
    float d = x - *mean;
    *mean += alpha * d;
    *var  += alpha * ((1.0f - alpha) * d * d - *var);
}

void seastate_update(seastate_t *w, float heave_m, float heave_rate,
                     float roll, float roll_rate,
                     float pitch, float pitch_rate)
{
    if (!w->enabled) return;

    float alpha = w->dt / w->tau;
    ew_stat(&w->h_mean,  &w->h_var,  heave_m,    alpha);
    ew_stat(&w->hr_mean, &w->hr_var, heave_rate, alpha);
    ew_stat(&w->r_mean,  &w->r_var,  roll,       alpha);
    ew_stat(&w->rr_mean, &w->rr_var, roll_rate,  alpha);
    ew_stat(&w->p_mean,  &w->p_var,  pitch,      alpha);
    ew_stat(&w->pr_mean, &w->pr_var, pitch_rate, alpha);

    w->elapsed += w->dt;
    if (!w->settled && w->elapsed >= SEASTATE_SETTLE_FACTOR * w->tau)
        w->settled = true;
}

float seastate_wave_height(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    return 4.0f * sqrtf(w->h_var);
}

float seastate_wave_period(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    if (sqrtf(w->h_var) < SEASTATE_MIN_HEAVE_SIG || w->hr_var <= 0.0f)
        return 0.0f;
    return 2.0f * (float)M_PI * sqrtf(w->h_var / w->hr_var);
}

float seastate_roll_period(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    if (sqrtf(w->r_var) < SEASTATE_MIN_ANGLE_SIG || w->rr_var <= 0.0f)
        return 0.0f;
    return 2.0f * (float)M_PI * sqrtf(w->r_var / w->rr_var);
}

float seastate_roll_amplitude(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    return 2.0f * sqrtf(w->r_var);
}

float seastate_pitch_period(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    if (sqrtf(w->p_var) < SEASTATE_MIN_ANGLE_SIG || w->pr_var <= 0.0f)
        return 0.0f;
    return 2.0f * (float)M_PI * sqrtf(w->p_var / w->pr_var);
}

float seastate_pitch_amplitude(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    return 2.0f * sqrtf(w->p_var);
}

void mekf_reconfigure(mekf_t *f, const imud_config_t *cfg)
{
    float dt     = f->dt;
    float odr_hz = 1.0f / dt;

    float Ng = (float)cfg->mekf_gyro_noise;
    float Nb = (float)cfg->mekf_gyro_bias;
    f->Qg = Ng * Ng * dt;
    f->Qb = Nb * Nb * dt;

    float Na = (float)cfg->mekf_accel_noise;
    float Nm = (float)cfg->mekf_mag_noise;
    f->Ra = (Na / G_MS2) * (Na / G_MS2) * odr_hz;
    f->Rm = Nm * Nm * (float)cfg->mag_odr_hz;

    float sk = (float)cfg->accel_skip_thresh;
    f->accel_skip_lo = 1.0f - sk;
    f->accel_skip_hi = 1.0f + sk;

    float mr = (float)cfg->mag_reject_gauss;
    f->mag_reject_sq = mr * mr;

    f->mag_yaw_only = cfg->mag_yaw_only;
}

void mekf_get_state(const mekf_t *f, fused_state_t *out, uint16_t flags_in)
{
    memcpy(out->q,         f->q,    sizeof f->q);
    memcpy(out->bias_gyro, f->bias, sizeof f->bias);

    /* Top-left 3×3 of P is the attitude error covariance (rad²) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out->cov[i*3+j] = f->P[i][j];

    /* Bottom-right 3×3 diagonal of P is the gyro-bias variance ((rad/s)²) */
    out->bias_gyro_var[0] = f->P[3][3];
    out->bias_gyro_var[1] = f->P[4][4];
    out->bias_gyro_var[2] = f->P[5][5];

    /* Platform-quiescence metric (EMA of (|a|/g − 1)²) */
    out->quiescence = f->acc_quiet_ema;

    /* Compass health diagnostics (see mekf_update_mag) */
    out->mag_anomaly  = f->mag_anom_ema;
    out->mag_residual = f->mag_resid_ema;

    /* Euler angles from rotation matrix R (NED, 3-2-1 aerospace convention) */
    float R[3][3];
    q_to_R(f->q, R);

    out->pitch = asinf(-R[2][0]);
    out->roll  = atan2f(R[2][1], R[2][2]);
    out->yaw   = atan2f(R[1][0], R[0][0]);

    /* Magnetic heading: yaw in [0, 360°) */
    float h_deg = out->yaw * (float)(180.0 / M_PI);
    if (h_deg < 0) h_deg += 360.0f;
    out->heading_deg = h_deg;

    /* Rate of turn (deg/min) from bias-corrected yaw rate, NED Z-axis */
    out->rate_of_turn = 0.0f;   /* set by fusion_thread from bias-corrected gyro[2] */

    uint16_t flags = flags_in;
    if (f->converged)    flags |= FLAG_FUSION_CONVERGED;
    if (!f->initialized) flags |= FLAG_STARTUP;
    out->flags = flags;
}
