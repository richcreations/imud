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
 * h:       predicted measurement in body frame (3×1)
 * z:       actual measurement in body frame (3×1)
 * R_noise: isotropic measurement noise variance (scalar, applied to I₃)
 *
 * H = [-[h×]  |  0₃]  (3×6)  — Jacobian of h w.r.t. error-state δθ
 * (Solà eq. 276, with sign from body-frame observation of a NED reference.)
 *
 * Returns 0 on success, -1 if S is singular (update skipped).
 */
static int eskf_update(mekf_t *f,
                       const float h[3],
                       const float z[3],
                       float R_noise)
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
     * Covariance update: P ← (I − K·H) × P  (simple form)
     * Numerically adequate for this system; switch to Joseph form if
     * symmetry loss becomes an issue after long operation.
     */
    float KH[6][6] = {{0}};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++) KH[i][j] += K[i][k] * H[k][j];

    float IKH[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

    float Pnew[6][6];
    m66_mul(IKH, f->P, Pnew);
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

    f->conv_thresh = 3.0f * fsq(0.5f * (float)(M_PI/180.0)); /* 3 × (0.5°)² */
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

    /* m_NED = R × mag_body */
    float mnx = R[0][0]*mag[0] + R[0][1]*mag[1] + R[0][2]*mag[2];
    float mny = R[1][0]*mag[0] + R[1][1]*mag[1] + R[1][2]*mag[2];

    float yaw = atan2f(mny, mnx);  /* magnetic heading in NED */

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

void mekf_predict(mekf_t *f, const imu_sample_t *s)
{
    if (!f->initialized) return;

    /* Bias-corrected angular rate */
    float w[3] = {
        s->gyro[0] - f->bias[0],
        s->gyro[1] - f->bias[1],
        s->gyro[2] - f->bias[2],
    };

    /* ── Propagate quaternion: q ← q ⊗ exp(w × dt)  (Solà eq. 259) ─── */
    float rv[3] = { w[0]*f->dt, w[1]*f->dt, w[2]*f->dt };
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
    float dt = f->dt;
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

    /* Add diagonal Q_d */
    for (int i = 0; i < 3; i++) Pnew[i][i]   += f->Qg;
    for (int i = 3; i < 6; i++) Pnew[i][i] += f->Qb;

    memcpy(f->P, Pnew, sizeof f->P);

    /* Update convergence flag */
    float trace = f->P[0][0] + f->P[1][1] + f->P[2][2];
    f->converged = (trace < f->conv_thresh);
}

void mekf_update_accel(mekf_t *f, const imu_sample_t *s)
{
    if (!f->initialized) return;

    float an = v3_norm(s->accel);

    /* Skip during linear acceleration (|a|/g outside [1±thresh]) */
    float ag = an / G_MS2;
    if (ag < f->accel_skip_lo || ag > f->accel_skip_hi) return;

    /* Gravity direction in body: z = −normalize(specific_force) */
    float z[3] = { -s->accel[0]/an, -s->accel[1]/an, -s->accel[2]/an };

    /* Predicted gravity in body: h = Rᵀ × [0, 0, 1] = third row of R */
    float R[3][3];
    q_to_R(f->q, R);
    float h[3] = { R[2][0], R[2][1], R[2][2] };

    eskf_update(f, h, z, f->Ra);
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

    /* Tight anomaly threshold (converged only), scaled to normalised units. */
    if (f->converged && res_sq > f->mag_reject_sq / (h_mag * h_mag)) return;

    eskf_update(f, h, z, Rm_n);
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
}

void mekf_get_state(const mekf_t *f, fused_state_t *out, uint16_t flags_in)
{
    memcpy(out->q,         f->q,    sizeof f->q);
    memcpy(out->bias_gyro, f->bias, sizeof f->bias);

    /* Top-left 3×3 of P is the attitude error covariance (rad²) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out->cov[i*3+j] = f->P[i][j];

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
