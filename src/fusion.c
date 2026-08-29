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
 * Error-state δx = [δθ(3), δb(3), δa_w(3)] (Solà §5.4, plus the Gauss–Markov
 * wave-acceleration state of docs/math.md §4.1.1).
 * P is 9×9, top-left 3×3 is attitude error covariance (rad²).
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

/*
 * 3×3 inverse via Cramer's rule; returns -1 if singular.
 *
 * The singularity test is RELATIVE to the matrix scale, not absolute. It has
 * to be: the only caller inverts S = HPHᵀ + R·I, which carries physical units,
 * and for the gravity update those units are tiny. With Ra ≈ 4.2e-5 and the
 * attitude block converged to ~0.4°, det(S) ≈ Ra·λ² ≈ 6e-13 — a perfectly
 * well-conditioned matrix (condition number ≈ 3) whose determinant simply
 * lives near 1e-13.
 *
 * The previous absolute |det| < 1e-12 therefore declared the converged filter
 * singular and made eskf_update return −1, silently dropping the accel
 * correction. That produced a limit cycle rather than a divergence — updates
 * were refused until gyro drift grew P back over the threshold, a few got
 * through, and the correction switched off again — so attitude uncertainty sat
 * pinned at whatever value made det(S) ≈ 1e-12 (0.41° at the default tuning)
 * instead of converging. It looked like a noise-model floor; it was arithmetic.
 *
 * Comparing against scale³ (scale = largest |entry|) tests what "singular"
 * actually means — rank deficiency — and is unit-free. 1e-6 is well above
 * float's ~1.2e-7 epsilon, so a determinant lost to round-off is still caught.
 */
static int m33_inv(const float A[3][3], float Ai[3][3])
{
    float d = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
            - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
            + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    float scale = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float a = fabsf(A[i][j]);
            if (a > scale) scale = a;
        }
    if (!(fabsf(d) > 1e-6f * scale * scale * scale)) return -1;
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

/*
 * Error-state reset Jacobian G = I − ½[δθ]× (Solà eq. 285).
 *
 * Injecting δθ into the nominal quaternion moves the linearisation point, so
 * the covariance of the (now zeroed) error state must be rotated into the new
 * frame. Only the attitude block resets — the gyro-bias and wave-acceleration
 * errors are additive and carry over unchanged — so callers apply
 * diag(G, I₃, I₃).
 */
static void m33_reset_jac(const float dth[3], float G[3][3])
{
    G[0][0] =  1.0f;         G[0][1] =  0.5f*dth[2]; G[0][2] = -0.5f*dth[1];
    G[1][0] = -0.5f*dth[2];  G[1][1] =  1.0f;        G[1][2] =  0.5f*dth[0];
    G[2][0] =  0.5f*dth[1];  G[2][1] = -0.5f*dth[0]; G[2][2] =  1.0f;
}

/*
 * M ← diag(G, I₃, I₃) × M, in place, for a MEKF_N×ncols row-major M.
 * Rows 3+ are left alone, so only the first three rows are touched.
 */
static void g_premul_rows3(const float G[3][3], float *M, int ncols)
{
    for (int j = 0; j < ncols; j++) {
        float m0 = M[0*ncols + j], m1 = M[1*ncols + j], m2 = M[2*ncols + j];
        M[0*ncols + j] = G[0][0]*m0 + G[0][1]*m1 + G[0][2]*m2;
        M[1*ncols + j] = G[1][0]*m0 + G[1][1]*m1 + G[1][2]*m2;
        M[2*ncols + j] = G[2][0]*m0 + G[2][1]*m1 + G[2][2]*m2;
    }
}

/*
 * Record how an eskf update fared against the innovation gate.
 *   w:   Huber weight actually applied (1.0 when uncapped, 0.0 when rejected)
 *   rej: 1.0 if the gross-outlier gate threw the measurement out, else 0.0
 * Rejected updates must still be counted — they are the strongest evidence
 * the filter and its measurements disagree, and hiding them would defeat the
 * purpose of the metric.
 */
static void gate_health(mekf_t *f, float alpha, float w, float rej)
{
    f->innov_weight_ema += alpha * (w   - f->innov_weight_ema);
    f->innov_reject_ema += alpha * (rej - f->innov_reject_ema);
}

/*
 * Record the normalised innovation squared for one update.
 *   ema:   channel accumulator (nis_accel_ema or nis_mag_ema)
 *   alpha: that channel's EMA gain
 *   d2:    νᵀS⁻¹ν, BEFORE any Huber capping or rejection
 *   dof:   effective degrees of freedom — see below
 *
 * Normalising by dof puts "consistent" at 1.0 for every channel, so the two
 * wire fields are directly comparable and a single threshold works for both.
 *
 * The vector updates carry TWO dof, not three, because the measurement is a
 * unit vector: normalising z removes the radial component, so the noise lives
 * only in the tangent plane. Writing the true innovation covariance as
 * E[ννᵀ] = HPHᵀ + R(I − hhᵀ) while S = HPHᵀ + R·I,
 *
 *   E[d²] = tr(S⁻¹·E[ννᵀ]) = tr(S⁻¹(S − R·hhᵀ)) = 3 − R·hᵀS⁻¹h
 *
 * and since H = [h×] gives Hᵀh = −[h×]h = 0, h is an eigenvector of S with
 * eigenvalue R, so S⁻¹h = h/R and the correction term is exactly 1:
 *
 *   E[d²] = 2   for both the gravity and 3-D mag updates.
 *
 * Dividing by 3 instead would peg a perfectly consistent filter at 0.67 and
 * make the wire field disagree with its own documentation.
 *
 * Where gate_health records the filter's RESPONSE (how much it had to
 * discount), this records the EVIDENCE (how badly the measurement and the
 * covariance disagree) — which is why it must be fed the uncapped d², and
 * fed on rejected updates too. Clamped because a transient singular-ish S
 * can produce an enormous d² that would otherwise dominate the average for
 * minutes; 100 is already ~100× inconsistent, far past any useful reading.
 */
static void nis_record(float *ema, float alpha, float d2, float dof)
{
    float v = d2 / dof;
    if (!(v >= 0.0f)) return;          /* NaN guard */
    if (v > 100.0f) v = 100.0f;
    *ema += alpha * (v - *ema);
}

/* ── Error-state (MEKF_N × MEKF_N) matrix helpers ──────────────────────────── */

static void mNN_mul(const float A[MEKF_N][MEKF_N], const float B[MEKF_N][MEKF_N],
                    float C[MEKF_N][MEKF_N])
{
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < MEKF_N; j++) {
            float s = 0;
            for (int k = 0; k < MEKF_N; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

/* C = A × Bᵀ  (both MEKF_N×MEKF_N) */
static void mNN_mulT(const float A[MEKF_N][MEKF_N], const float B[MEKF_N][MEKF_N],
                     float C[MEKF_N][MEKF_N])
{
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < MEKF_N; j++) {
            float s = 0;
            for (int k = 0; k < MEKF_N; k++) s += A[i][k] * B[j][k];
            C[i][j] = s;
        }
}

/* Enforce exact symmetry (cheap insurance against float round-off). */
static void mNN_symmetrize(float P[MEKF_N][MEKF_N])
{
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < i; j++) {
            float a = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = a;
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

/*
 * The isfinite test is not redundant with the epsilon.  `n > 1e-10f` is a
 * guard against dividing by ZERO; a NaN fails it too, and the quaternion is
 * then passed through completely untouched — which is the opposite of what
 * the epsilon's presence suggests.  That is the mechanism by which a single
 * non-finite value became permanent: nothing downstream re-normalises, so the
 * NaN survived every subsequent step.
 *
 * Identity is the right fallback here rather than "leave it alone": the
 * caller's alternative is to keep propagating a value that is not a rotation
 * at all.  Callers holding a mekf_t get the louder treatment — see
 * mekf_sanitize, which resets the whole state and raises FLAG_STATE_RESET.
 * This function also serves three local-quaternion sites that have no filter
 * to flag.
 */
static void q_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (isfinite(n) && n > 1e-10f) {
        q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
    } else if (!isfinite(n)) {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    }
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
 * Gross-outlier reject threshold, as a multiple of the χ² cap γ.
 *
 * The Huber cap already bounds any single update's influence smoothly; this
 * gate exists only to discard measurements that cannot be sensor noise at all
 * — a fault, a slam, a corrupted read — so it must sit far enough out that
 * ordinary seaway motion never reaches it.  Too close and it rejects routine
 * wave-orbital motion, starving the filter of the corrections it needs; the
 * resulting drift then produces larger innovations, which trips the gate more
 * often still.
 *
 * 25γ is where the benefit saturates: far enough out that a steady seaway
 * never trips it, close enough in to catch a genuine fault.  In
 * innovation-distance terms it rejects beyond 5x the cap radius.
 *
 * What this does NOT do is make Ra right.  NIS stays around 20-25 with the
 * gate wide, so the measurement model really is over-confident, as
 * docs/math.md §4.7 sets out — widening the gate removes the damage the gate
 * itself was doing, and no scalar Ra can fix the rest.
 *
 * Overridable for the sweep in test_fusion.c; see docs/math.md §4.7.
 */
/*
 * Huber covariance-consistency variant (docs/math.md §4.5), for measurement only.
 *   0 = shipped: attenuate ν, leave K at full confidence
 *   1 = K ← w·K   (exact Joseph for the suboptimal gain)
 *   2 = R → R/w   (one IRLS re-solve, uncapped ν)
 *   3 = R → R/w²
 * Rebuild with -DHUBER_VARIANT=n; see the measured table in eskf_update.
 */
#ifndef HUBER_VARIANT
#define HUBER_VARIANT 0
#endif

#ifndef GROSS_REJECT_MULT
#define GROSS_REJECT_MULT 25.0f
#endif

/*
 * Measurement Jacobian for a body-frame direction observation (3×MEKF_N).
 *
 * H = [ +[h×] | 0₃ | 0₃ ] — Jacobian of h w.r.t. the error state.
 *
 * Derivation (right-multiply error convention, q_true = q̂ ⊗ δq):
 *   R(q_true) ≈ R(q̂) × (I + [δθ×])
 *   h_true = R(q_true)ᵀ × g ≈ ĥ − [δθ×]ĥ = ĥ + [ĥ×]δθ
 *   → H_δθ = +[ĥ×]   (Solà eq. 276 / §7, right-perturbation)
 *
 * +[h×] = [[  0,  -h2,  h1 ],
 *          [ h2,   0,  -h0 ],
 *          [-h1,  h0,   0  ]]
 *
 * Shared by eskf_update and mekf_accel_probe so the daemon's gain solve and
 * the offline report can never disagree about what H is.
 */
static void dir_jacobian(const float h[3], float H[3][MEKF_N])
{
    memset(H, 0, 3 * MEKF_N * sizeof(float));
    H[0][1] = -h[2];  H[0][2] =  h[1];
    H[1][0] =  h[2];  H[1][2] = -h[0];
    H[2][0] = -h[1];  H[2][1] =  h[0];
}

/*
 * Measurement Jacobian and predicted direction for the gravity update with the
 * Gauss–Markov wave state active (docs/math.md §4.1.1).
 *
 * The measured direction is z = −normalize(specific force).  Writing the
 * specific force in normalised gravity units, that is
 *
 *   z = normalize(h − a_w)          v ≡ h − â_w,   ẑ ≡ v/|v|
 *
 * where h is the predicted gravity direction and a_w the wave acceleration in
 * the same units.  Perturbing both (h_true = ĥ + [ĥ×]δθ, a_true = â_w + δa_w):
 *
 *   δv = [ĥ×]δθ − δa_w
 *   δẑ = P_ẑ/|v| · δv        with the tangent projector P_ẑ = I − ẑẑᵀ
 *
 * giving the two blocks below.  The projector is NOT optional book-keeping:
 * it is what makes ẑᵀH = 0 hold EXACTLY, hence Sẑ = R·ẑ, hence the NIS dof of
 * 2 derived in docs/math.md §8.2 (a unit-vector measurement carries no radial
 * noise).  Dropping it — using [ĥ×] and −I directly — leaves ẑᵀH = O(|â_w|),
 * ẑ stops being an eigenvector of S, and the NIS the wire reports quietly
 * stops meaning "1.0 = consistent".
 *
 * At â_w = 0 this reduces algebraically to dir_jacobian: v = h is already a
 * unit vector, and (I − hhᵀ)[h×] = [h×] because hᵀ[h×] = 0.
 *
 * Returns false if |v| is too small to normalise — an acceleration comparable
 * to gravity itself, where the linearisation has no meaning; the caller then
 * falls back to the plain direction update.
 */
static bool wave_jacobian(const float h[3], const float aw[3],
                          float zhat[3], float H[3][MEKF_N])
{
    float v[3] = { h[0]-aw[0], h[1]-aw[1], h[2]-aw[2] };
    float vn = v3_norm(v);
    if (vn < 0.5f) return false;
    float inv = 1.0f / vn;

    for (int i = 0; i < 3; i++) zhat[i] = v[i] * inv;

    /* P_ẑ = I − ẑẑᵀ */
    float Pz[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Pz[i][j] = (i == j ? 1.0f : 0.0f) - zhat[i]*zhat[j];

    /* +[h×], as in dir_jacobian */
    const float hx[3][3] = {
        {     0, -h[2],  h[1] },
        {  h[2],     0, -h[0] },
        { -h[1],  h[0],     0 },
    };

    memset(H, 0, 3 * MEKF_N * sizeof(float));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += Pz[i][k] * hx[k][j];
            H[i][j]     =  s * inv;              /* H_δθ  = P_ẑ·[h×]/|v| */
            H[i][6 + j] = -Pz[i][j] * inv;       /* H_δa_w = −P_ẑ/|v|    */
        }
    return true;
}

/* PHᵀ (MEKF_N×3):  PHt[i][j] = Σ_k P[i][k] × H[j][k] */
static void dir_PHt(const mekf_t *f, const float H[3][MEKF_N],
                    float PHt[MEKF_N][3])
{
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0;
            for (int k = 0; k < MEKF_N; k++) s += f->P[i][k] * H[j][k];
            PHt[i][j] = s;
        }
}

/*
 * Innovation covariance S = H·PHᵀ + R.
 *
 * R is isotropic R_noise·I₃ unless R33 is non-NULL, in which case it is that
 * matrix. Only the 3-D magnetometer update needs the general form (see
 * mekf_update_mag); everything else passes NULL and takes the scalar path
 * literally.
 */
static void dir_S(const float H[3][MEKF_N], const float PHt[MEKF_N][3],
                  float R_noise, const float (*R33)[3], float S[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = R33 ? R33[i][j] : ((i == j) ? R_noise : 0.0f);
            for (int k = 0; k < MEKF_N; k++) s += H[i][k] * PHt[k][j];
            S[i][j] = s;
        }
}

/*
 * Generic measurement update.  (Solà §4.2)
 *
 * h:         predicted measurement in body frame (3×1)
 * z:         actual measurement in body frame (3×1)
 * R_noise:   isotropic measurement noise variance (scalar, applied to I₃)
 * chi2_gate: innovation gate — skip the update when the normalised
 *            innovation distance d² = innovᵀ·S⁻¹·innov exceeds this
 *            (χ²₃ quantile); 0 disables the gate.
 * nis_ema:   channel NIS accumulator to feed (NULL to skip)
 * nis_alpha: EMA gain for that accumulator
 * aw:        wave-acceleration estimate to fold into the prediction, or NULL
 *            for a plain direction measurement.  NULL takes literally the
 *            pre-§10.5 code path — no projection arithmetic runs at all — so
 *            the mag channels and a disabled wave state are untouched.
 * R33:       full 3×3 measurement noise, or NULL for isotropic R_noise·I₃.
 *            Same contract as aw: NULL is the scalar path, unchanged.
 *
 * Returns 0 on success, -1 if S is singular, +1 if the innovation gate
 * rejected the measurement (update skipped either way).
 */
static int eskf_update(mekf_t *f,
                       const float h[3],
                       const float z[3],
                       float R_noise,
                       float chi2_gate,
                       float *nis_ema,
                       float nis_alpha,
                       const float *aw,
                       const float (*R33)[3])
{
    /* Local, so the Huber R-inflating variants can repoint it at a scaled
     * copy without touching the caller's matrix. */
    const float (*Rmat)[3] = R33;
    float H[3][MEKF_N];
    float zhat[3];
    if (!(aw && wave_jacobian(h, aw, zhat, H))) {
        dir_jacobian(h, H);
        zhat[0] = h[0]; zhat[1] = h[1]; zhat[2] = h[2];
    }

    float PHt[MEKF_N][3];
    dir_PHt(f, H, PHt);

    /*
     * Gain solve: S = H·PHᵀ + R·I₃, then K = PHᵀ·S⁻¹.
     * Factored into a macro rather than written once because the R-inflating
     * Huber variants (HUBER_VARIANT 2/3, below) have to re-run it with a
     * different R. PHt does not depend on R, so it stays hoisted out.
     */
    float S[3][3], Sinv[3][3], K[MEKF_N][3];
#if HUBER_VARIANT == 2 || HUBER_VARIANT == 3
    float R33_scaled[3][3];
#endif
    #define ESKF_SOLVE_GAIN(Rv)                                            \
        do {                                                               \
            dir_S(H, PHt, (Rv), Rmat, S);                                  \
            if (m33_inv(S, Sinv) < 0) return -1;                           \
            for (int i = 0; i < MEKF_N; i++)                               \
                for (int j = 0; j < 3; j++) {                              \
                    float s = 0;                                           \
                    for (int k = 0; k < 3; k++) s += PHt[i][k]*Sinv[k][j]; \
                    K[i][j] = s;                                           \
                }                                                          \
        } while (0)

    ESKF_SOLVE_GAIN(R_noise);

    /* innovation = z − ẑ  (3×1); ẑ = h when no wave state is in play */
    float innov[3] = { z[0]-zhat[0], z[1]-zhat[1], z[2]-zhat[2] };

    /* Innovation handling: d² = innovᵀ·S⁻¹·innov follows χ²(3) when the
     * measurement is consistent with the filter. Wave-orbital acceleration
     * can keep |a| ≈ g while swinging the DIRECTION — the magnitude gate
     * passes those. A hard reject would starve the filter in a steady
     * seaway, so instead the influence is capped Huber-style (scale the
     * innovation so its effective distance equals the gate) and only gross
     * outliers (GROSS_REJECT_MULT × the gate) are rejected outright.
     *
     * The cap is deliberately inconsistent with the covariance: shrinking ν
     * alone leaves K — and hence the Joseph update below — at full
     * confidence, so P contracts as if the measurement had been fully
     * trusted. Do not "fix" that. Contracting P by the attenuated gain
     * instead leaves a larger P, hence a larger gain on the FOLLOWING
     * samples, which in a seaway are contaminated by the same wave; the
     * inconsistency holds the gain down exactly when the measurements are
     * least trustworthy. All three consistent forms (HUBER_VARIANT 1/2/3
     * above) measure both less accurate and less self-consistent across the
     * wave benchmark, and retuning Ra does not rescue them. The cap is a
     * robustness device, not a statistical one. See docs/math.md §4.5.
     *
     * innov_weight_ema / innov_reject_ema below expose how hard the cap is
     * being leaned on; nis_accel_ema / nis_mag_ema expose whether the noise
     * model underneath it is right. */
    if (chi2_gate > 0.0f || nis_ema) {
        /* No convergence gate needed: S = HPHᵀ + R already contains P, so
         * during acquisition (large P) d² stays small and the cap is
         * naturally inactive; it engages only once the filter is confident. */
        float Si[3];
        for (int i = 0; i < 3; i++)
            Si[i] = Sinv[i][0]*innov[0] + Sinv[i][1]*innov[1]
                  + Sinv[i][2]*innov[2];
        float d2 = innov[0]*Si[0] + innov[1]*Si[1] + innov[2]*Si[2];

        /* Consistency first, while d² is still uncensored — see nis_record.
         * dof = 2: unit-vector measurement, radial component carries no noise.
         * Holds for the wave-aware Jacobian too — wave_jacobian projects H onto
         * ẑ's tangent plane, so ẑᵀH = 0 exactly and S⁻¹ẑ = ẑ/R just as before. */
        if (nis_ema) nis_record(nis_ema, nis_alpha, d2, 2.0f);

        if (chi2_gate > 0.0f) {
            if (d2 > GROSS_REJECT_MULT * chi2_gate) {
                gate_health(f, nis_alpha, 0.0f, 1.0f);
                return 1;
            }
            float w = 1.0f;
            if (d2 > chi2_gate) {
                w = sqrtf(chi2_gate / d2);
#if HUBER_VARIANT == 0
                /* Shipped: attenuate the innovation, leave K alone. */
                innov[0] *= w; innov[1] *= w; innov[2] *= w;
#elif HUBER_VARIANT == 1
                /* K ← w·K: identical state correction to variant 0, but the
                 * Joseph update below then contracts P by the gain actually
                 * used — the "exact Joseph for a suboptimal gain" form. */
                for (int i = 0; i < MEKF_N; i++)
                    for (int j = 0; j < 3; j++) K[i][j] *= w;
#elif HUBER_VARIANT == 2 || HUBER_VARIANT == 3
                /* IRLS: re-solve the gain with R inflated, and apply the
                 * UNCAPPED innovation through it. With a full R the whole
                 * matrix scales, not just the scalar. */
                {
#if HUBER_VARIANT == 2
                    float wr = w;
#else
                    float wr = w*w;
#endif
                    if (Rmat) {
                        for (int a = 0; a < 3; a++)
                            for (int b = 0; b < 3; b++)
                                R33_scaled[a][b] = Rmat[a][b] / wr;
                        Rmat = (const float (*)[3])R33_scaled;
                    }
                    ESKF_SOLVE_GAIN(R_noise / wr);
                    R_noise /= wr;
                }
#else
#error "HUBER_VARIANT must be 0, 1, 2 or 3"
#endif
            }
            gate_health(f, nis_alpha, w, 0.0f);
        }
    }
    #undef ESKF_SOLVE_GAIN

    /* δx = K × innovation  (MEKF_N×1) */
    float dx[MEKF_N] = {0};
    for (int i = 0; i < MEKF_N; i++)
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

    /* Wave-acceleration correction: a_w ← a_w + δa_w. Unconditional: when the
     * state is disabled P's wave rows are zero, so K's are too and this adds
     * exactly 0.0f. */
    f->wave_acc[0] += dx[6];
    f->wave_acc[1] += dx[7];
    f->wave_acc[2] += dx[8];

    /*
     * Covariance update, Joseph form, with the error-state reset folded in:
     *   P ← G·[(I−K·H)·P·(I−K·H)ᵀ + K·R·Kᵀ]·Gᵀ
     *     = [G(I−KH)]·P·[G(I−KH)]ᵀ + (G·K)·R·(G·K)ᵀ
     * so the reset is applied by premultiplying K and (I−KH) by G_full.
     * Joseph form guarantees symmetry and positive-definiteness in float
     * arithmetic — the simple (I−KH)·P form slowly loses both at 833 Hz over
     * multi-day runs — and holds for any gain, including the Huber-scaled one.
     * R is isotropic (R_noise·I₃), so K·R·Kᵀ = R_noise·K·Kᵀ.
     */
    float KH[MEKF_N][MEKF_N] = {{0}};
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < MEKF_N; j++)
            for (int k = 0; k < 3; k++) KH[i][j] += K[i][k] * H[k][j];

    float IKH[MEKF_N][MEKF_N];
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < MEKF_N; j++)
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

    /* Error-state reset: the correction just injected into q has moved the
     * linearisation point, so P must be rotated into the new error frame.
     * G_full = diag(G, I₃, I₃) — neither the bias nor the wave-acceleration
     * error has a reset — hence only rows 0–2 of K and IKH are touched. */
    {
        float G[3][3];
        m33_reset_jac(dx, G);
        g_premul_rows3(G, &K[0][0],   3);
        g_premul_rows3(G, &IKH[0][0], MEKF_N);
    }

    float Ptmp[MEKF_N][MEKF_N], Pnew[MEKF_N][MEKF_N];
    mNN_mul(IKH, f->P, Ptmp);
    mNN_mulT(Ptmp, IKH, Pnew);

    /* K·R·Kᵀ. The isotropic shortcut R·K Kᵀ is only valid for scalar R, so
     * the general form is spelled out when a full matrix was supplied. */
    if (Rmat) {
        for (int i = 0; i < MEKF_N; i++)
            for (int j = 0; j < MEKF_N; j++) {
                float s = 0;
                for (int k = 0; k < 3; k++)
                    for (int l = 0; l < 3; l++)
                        s += K[i][k] * Rmat[k][l] * K[j][l];
                Pnew[i][j] += s;
            }
    } else {
        for (int i = 0; i < MEKF_N; i++)
            for (int j = 0; j < MEKF_N; j++) {
                float s = 0;
                for (int k = 0; k < 3; k++) s += K[i][k] * K[j][k];
                Pnew[i][j] += R_noise * s;
            }
    }

    mNN_symmetrize(Pnew);
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
 *   y ≈ −(R̂·δθ)_D  →  H = [ −R̂[2][:] | 0₃ | 0₃ ]  (1×MEKF_N)
 * Same Joseph-form covariance update as the vector path.
 */
/* χ²(1) 99% quantile — yaw innovation gate. */
#define YAW_CHI2_GATE 6.63f

static int eskf_update_yaw(mekf_t *f, float y, float R_noise)
{
    float R[3][3];
    q_to_R(f->q, R);

    float H[MEKF_N] = { -R[2][0], -R[2][1], -R[2][2] };

    float PHt[MEKF_N];
    for (int i = 0; i < MEKF_N; i++) {
        float s = 0;
        for (int k = 0; k < MEKF_N; k++) s += f->P[i][k] * H[k];
        PHt[i] = s;
    }

    float S = R_noise;
    for (int i = 0; i < MEKF_N; i++) S += H[i] * PHt[i];
    if (S < 1e-12f) return -1;

    /* Same Huber policy as the vector update, including the same deliberate
     * covariance inconsistency documented there. */
    float d2 = y * y / S;

    /* Scalar measurement: dof = 1. Recorded pre-cap, as in eskf_update. */
    nis_record(&f->nis_mag_ema, f->nis_mag_alpha, d2, 1.0f);

    if (d2 > GROSS_REJECT_MULT * YAW_CHI2_GATE) {
        gate_health(f, f->nis_mag_alpha, 0.0f, 1.0f);
        return 1;
    }
    float w = 1.0f;
    if (d2 > YAW_CHI2_GATE) { w = sqrtf(YAW_CHI2_GATE / d2); y *= w; }
    gate_health(f, f->nis_mag_alpha, w, 0.0f);

    float K[MEKF_N];
    for (int i = 0; i < MEKF_N; i++) K[i] = PHt[i] / S;

    float dx[MEKF_N];
    for (int i = 0; i < MEKF_N; i++) dx[i] = K[i] * y;

    float dq[4], qnew[4];
    q_from_rotvec(dx, dq);
    q_mul(f->q, dq, qnew);
    memcpy(f->q, qnew, sizeof f->q);
    q_normalize(f->q);

    f->bias[0] += dx[3];
    f->bias[1] += dx[4];
    f->bias[2] += dx[5];

    /* The heading measurement says nothing directly about wave acceleration
     * (H's wave block is zero), but the cross-covariance built up by the accel
     * path means K's is not — so the correction is real Kalman book-keeping,
     * and structurally zero while the state is disabled. */
    f->wave_acc[0] += dx[6];
    f->wave_acc[1] += dx[7];
    f->wave_acc[2] += dx[8];

    /* Joseph form, rank-1, with the error-state reset folded in as in
     * eskf_update: P ← [G(I−KH)]·P·[G(I−KH)]ᵀ + (G·K)·R·(G·K)ᵀ */
    float IKH[MEKF_N][MEKF_N];
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < MEKF_N; j++)
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];

    {
        float G[3][3];
        m33_reset_jac(dx, G);
        g_premul_rows3(G, K,          1);
        g_premul_rows3(G, &IKH[0][0], MEKF_N);
    }

    float Ptmp[MEKF_N][MEKF_N], Pnew[MEKF_N][MEKF_N];
    mNN_mul(IKH, f->P, Ptmp);
    mNN_mulT(Ptmp, IKH, Pnew);
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 0; j < MEKF_N; j++)
            Pnew[i][j] += R_noise * K[i] * K[j];
    mNN_symmetrize(Pnew);
    memcpy(f->P, Pnew, sizeof f->P);

    return 0;
}

/* ── Public API ────────────────────────────────────────────────────────────── */

/* m_ref EMA rate: one accepted sample per 1/mag_odr_hz, τ = 300 s. */
#ifndef MREF_TAU_S
#define MREF_TAU_S 300.0f
#endif

/* Update-gate health EMA time constant, s — matches the mag diagnostics. */
#define GATE_TAU_S 30.0f

/*
 * Derive every tuning value that follows from config.  Called by BOTH
 * mekf_init and mekf_reconfigure so the two can never drift apart — the
 * stale-mref_alpha-after-reconfigure bug existed precisely because these
 * derivations were duplicated and only one copy was kept up to date.
 *
 * Touches tuning only: filter state (q, P, bias, dt) and runtime scalars
 * (Ra_scale, acc_quiet_ema, the gate-health and NIS EMAs) are deliberately
 * left alone, so this is safe to call on a running filter. Note the EMA
 * GAINS (gate_alpha, nis_mag_alpha) are derived here — only the accumulated
 * values are preserved, so a SIGHUP retunes the averaging without discarding
 * the history, which is exactly what an A/B of mekf_accel_noise needs.
 */
static void mekf_derive_tuning(mekf_t *f, const imud_config_t *cfg)
{
    /*
     * Both rates come from the filter, not from cfg: f->dt and f->mag_odr_hz
     * hold the rates the drivers actually programmed, which is not necessarily
     * what cfg->{imu,mag}_odr_hz asked for. Reading cfg->mag_odr_hz here is
     * which otherwise sizes Rm for a rate the magnetometer is not running at.
     * Both odr_hz keys are [restart], so neither can go stale on SIGHUP.
     */
    float dt         = f->dt;
    float odr_hz     = 1.0f / dt;
    float mag_odr_hz = f->mag_odr_hz;

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
     *       = Nm² × mag_odr_hz  (100 Hz by default)
     */
    float Na = (float)cfg->mekf_accel_noise;
    float Nm = (float)cfg->mekf_mag_noise;
    f->Ra = (Na / G_MS2) * (Na / G_MS2) * odr_hz;
    f->Rm = Nm * Nm * mag_odr_hz;

    /*
     * Gauss–Markov wave-acceleration state (docs/math.md §4.1.1).
     *
     * The seaway gravity residual is wave-orbital: correlated over ~0.3–1 s,
     * not white. Feeding 833 such samples per second to a white-noise R tells
     * the filter it has 833 independent gravity measurements when it has
     * closer to one per wave; P collapses and the filter diverges into its own
     * over-confidence. Modelling the disturbance as a first-order GM process
     * with steady-state σ and correlation time τ lets the correlated part live
     * in the state, where repeated samples correctly stop adding information.
     *
     * σ is configured in m/s² (what an accelerometer spec sheet and fit-ra
     * both speak) and stored as a variance in normalised gravity units, the
     * units the measurement z = −normalize(a) actually uses.
     *
     * Either knob at 0 disables the state: the block stays identically zero
     * and the filter is bit-for-bit the 6-state filter.
     */
    float wsig = (float)cfg->mekf_wave_accel / G_MS2;
    f->wave_tau     = (float)cfg->mekf_wave_accel_tau_s;
    f->wave_sig2    = wsig * wsig;
    f->wave_enabled = (cfg->mekf_wave_accel > 0.0 &&
                       cfg->mekf_wave_accel_tau_s > 0.0);

    /* Dip-reference uncertainty: configured in degrees, used as a variance in
     * the same normalised direction units as Rm_n (a dip error of δ radians
     * displaces the predicted unit direction by δ). See mekf_update_mag. */
    float dsig = (float)cfg->mekf_mag_dip_sigma_deg * (float)(M_PI/180.0);
    f->dip_sig2 = (dsig > 0.0f) ? dsig * dsig : 0.0f;

    float sk = (float)cfg->accel_skip_thresh;
    f->accel_skip_lo = 1.0f - sk;
    f->accel_skip_hi = 1.0f + sk;

    float mr = (float)cfg->mag_reject_gauss;
    f->mag_reject_sq = mr * mr;

    f->mag_yaw_only = cfg->mag_yaw_only;
    f->mag_fuse_uncal   = cfg->mag_fuse_uncal;
    f->mag_uncal_reject = (float)cfg->mag_uncal_reject_frac;

    f->mref_alpha = (mag_odr_hz > 0.0f && MREF_TAU_S < 1e9f)
        ? 1.0f / (MREF_TAU_S * mag_odr_hz)
        : 0.0f;

    /* Gate health is fed by every eskf update; accel at odr_hz dominates.
     * nis_accel_ema shares this gain — it too is fed at the accel rate. */
    f->gate_alpha = 1.0f / (GATE_TAU_S * odr_hz);

    /* The mag NIS channel is fed at the MAG rate, so it needs its own gain:
     * reusing gate_alpha (sized for 833 Hz) at ~104 Hz would stretch its time
     * constant from 30 s to about four minutes. */
    f->nis_mag_alpha = (mag_odr_hz > 0.0f)
        ? 1.0f / (GATE_TAU_S * mag_odr_hz)
        : f->gate_alpha;

    /*
     * Convergence threshold on tr(P[0:3]), i.e. "attitude is acquired".
     *
     * Historically a flat 3 × (0.5°)². That value was set against a filter
     * whose steady-state attitude variance was not believable — it settled at
     * 0.14°/axis while its actual error was degrees (NEES 8–18). With the wave
     * state modelling the seaway disturbance, P tells the truth, and the truth
     * has a floor: a correlated acceleration of σ_g is indistinguishable from a
     * tilt of asin(σ_g) within one correlation time, and only averages down
     * slowly after that. At the shipped tuning that floor is ≈0.9°/axis, so a
     * 0.5° threshold would simply never be reached and FLAG_FUSION_CONVERGED
     * would never set.
     *
     * The threshold therefore tracks the floor it has to clear. 0.30× the
     * equivalent tilt puts it at 1.40°/axis for the default σ = 0.8 m/s²,
     * about 1.6× the measured floor — reached in ~4 s of level running, and
     * still far tighter than any heading requirement it gates.
     */
    float conv_deg = 0.5f;
    if (f->wave_enabled) {
        float sg = sqrtf(f->wave_sig2);
        if (sg > 0.5f) sg = 0.5f;                      /* asin domain guard */
        float floor_deg = 0.30f * asinf(sg) * (float)(180.0/M_PI);
        if (floor_deg > conv_deg) conv_deg = floor_deg;
    }
    f->conv_thresh = 3.0f * fsq(conv_deg * (float)(M_PI/180.0));
}

/*
 * Nominal state and covariance back to their starting values, leaving every
 * tuning field alone.
 *
 * Shared by mekf_init and mekf_sanitize so the P0 constants exist once.  It
 * must run AFTER mekf_derive_tuning, because the wave block of P is sized
 * from wave_sig2/wave_enabled, which that function sets.  Nothing here is
 * read by mekf_derive_tuning in return — it looks only at cfg — so the
 * ordering is a one-way dependency rather than a cycle.
 */
static void mekf_reset_state(mekf_t *f)
{
    /* Nominal state: identity attitude, zero bias and wave acceleration. */
    f->q[0] = 1.0f;
    f->q[1] = f->q[2] = f->q[3] = 0.0f;
    f->bias[0] = f->bias[1] = f->bias[2] = 0.0f;
    f->wave_acc[0] = f->wave_acc[1] = f->wave_acc[2] = 0.0f;

    /*
     * Initial covariance: generous values so the filter accepts early
     * measurements.  Attitude uncertainty ~10° (0.175 rad), bias from startup
     * still window (small).
     */
    float init_att  = fsq(0.175f);    /* ~10° per axis */
    float init_bias = fsq(0.001f);    /* 1 mdps/s residual bias uncertainty */
    memset(f->P, 0, sizeof f->P);
    for (int i = 0; i < 3; i++) f->P[i][i] = init_att;
    for (int i = 3; i < 6; i++) f->P[i][i] = init_bias;

    /* Wave block starts at its steady state — the GM process is stationary,
     * so there is no acquisition transient to model. Stays 0 when disabled. */
    if (f->wave_enabled)
        for (int i = 6; i < 9; i++) f->P[i][i] = f->wave_sig2;
}

void mekf_init(mekf_t *f,
               const imud_config_t *cfg,
               float odr_hz,
               float mag_odr_hz,
               const float gyro_bias_init[3])
{
    memset(f, 0, sizeof *f);

    /* Both sample rates, stored before mekf_derive_tuning reads them back. */
    f->dt         = 1.0f / odr_hz;
    f->mag_odr_hz = mag_odr_hz;

    mekf_derive_tuning(f, cfg);

    /* After derive_tuning: the wave block of P is sized from wave_sig2. */
    mekf_reset_state(f);

    /* Pre-estimated bias from the startup still window, if the caller has one.
     * Applied after the reset, which zeroes it. */
    if (gyro_bias_init) {
        f->bias[0] = gyro_bias_init[0];
        f->bias[1] = gyro_bias_init[1];
        f->bias[2] = gyro_bias_init[2];
    }

    f->Ra_scale = 1.0f;   /* raised by fusion_thread during engine vibration */
    f->acc_quiet_ema = 1.0f;   /* start "loud"; must earn quiescence from data */

    /* Health EMAs start optimistic: no capping, no rejection seen yet. */
    f->innov_weight_ema = 1.0f;
    f->innov_reject_ema = 0.0f;

    /* NIS starts at the consistent value rather than 0, so a fresh filter
     * does not spend its first τ reporting an implausibly perfect model. */
    f->nis_accel_ema = 1.0f;
    f->nis_mag_ema   = 1.0f;
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
     *   Φ = I₉ + Fc·dt   for the attitude/bias blocks
     *   Fc = [[-[w×]  -I₃], [0₃  0₃]]
     *   Φ = [[I₃ - [w×]dt   -I₃·dt     0₃ ],
     *         [0₃             I₃       0₃ ],
     *         [0₃             0₃      φ·I₃]]
     * where the wave block is the EXACT Gauss–Markov transition φ = e^(−dt/τ),
     * not a first-order expansion (see below); φ = 1 when the state is
     * disabled, leaving that block an identity that never touches anything.
     *
     * Diagonal process noise Q_d:
     *   Q_d[0:3][0:3] = Qg·I₃   Q_d[3:6][3:6] = Qb·I₃   Q_d[6:9][6:9] = Qw·I₃
     */
    float Phi[MEKF_N][MEKF_N] = {{0}};

    /* Top-left: I₃ − [w×]·dt */
    Phi[0][0] = 1.0f;           Phi[0][1] =  w[2]*dt;   Phi[0][2] = -w[1]*dt;
    Phi[1][0] = -w[2]*dt;       Phi[1][1] = 1.0f;        Phi[1][2] =  w[0]*dt;
    Phi[2][0] =  w[1]*dt;       Phi[2][1] = -w[0]*dt;    Phi[2][2] = 1.0f;
    /* Top-right: -I₃·dt */
    Phi[0][3] = -dt;  Phi[1][4] = -dt;  Phi[2][5] = -dt;
    /* Bias block: I₃ */
    Phi[3][3] = 1.0f; Phi[4][4] = 1.0f; Phi[5][5] = 1.0f;

    /*
     * Wave block: the EXACT first-order Gauss–Markov transition
     *   φ  = e^(−dt/τ)          Q_w = σ²·(1 − e^(−2dt/τ))
     * rather than the linear-in-dt approximation used for Qg/Qb below.
     *
     * Not a refinement for its own sake: τ is of order 0.5 s while the |a|
     * skip band routinely leaves gaps of the same order, so dt/τ is NOT small
     * on the steps that matter and a first-order expansion would both
     * over-decay the state and mis-size the noise exactly when it is being
     * asked to bridge a gap. The exact form is stationary for any dt — feed it
     * a step of 10τ and it returns the state to zero with variance σ², which
     * is the correct answer — so the filter cannot be destabilised by a
     * scheduling hiccup.
     */
    float phi_w = 1.0f, qw = 0.0f;
    if (f->wave_enabled) {
        phi_w = expf(-dt / f->wave_tau);
        /* expm1f, not (1 - phi*phi): for small dt/tau both factors are within
         * an ULP of 1 and their difference is a cancellation.  Measured
         * against the exact value, the naive form loses 0.047 % at 8 kHz and
         * 0.053 % at 32 kHz with tau = 2 s, against 0.000 % at the 833 Hz
         * default — small, but it is the process noise on the state that
         * bridges the |a| skip band, and the error grows with every rate
         * increase.  This form is exact and drops a multiply. */
        qw    = -f->wave_sig2 * expm1f(-2.0f * dt / f->wave_tau);
    }
    Phi[6][6] = phi_w; Phi[7][7] = phi_w; Phi[8][8] = phi_w;
#ifdef WAVE_TRANSPORT
    /*
     * Optional transport term. Wave-orbital acceleration is a property of the
     * seaway, so it is arguably NED-stationary; carrying it as a body-frame
     * vector then needs Φ_w = e^(−dt/τ)(I − [ω dt]×) to account for the frame
     * rotating underneath it. Measured A/B in docs/math.md §4.7.
     */
    if (f->wave_enabled) {
        Phi[6][7] =  w[2]*dt*phi_w;  Phi[6][8] = -w[1]*dt*phi_w;
        Phi[7][6] = -w[2]*dt*phi_w;  Phi[7][8] =  w[0]*dt*phi_w;
        Phi[8][6] =  w[1]*dt*phi_w;  Phi[8][7] = -w[0]*dt*phi_w;
    }
#endif

    float Ptmp[MEKF_N][MEKF_N], Pnew[MEKF_N][MEKF_N];
    mNN_mul(Phi, f->P, Ptmp);
    mNN_mulT(Ptmp, Phi, Pnew);

    /* Add diagonal Q_d, scaled to the actual step length (Qg/Qb are stored
     * per nominal step; noise variance grows linearly with dt). The wave
     * block's Q is already exact for this dt and must not be rescaled. */
    float qscale = dt / f->dt;
    for (int i = 0; i < 3; i++) Pnew[i][i] += f->Qg * qscale;
    for (int i = 3; i < 6; i++) Pnew[i][i] += f->Qb * qscale;
    if (f->wave_enabled)
        for (int i = 6; i < 9; i++) Pnew[i][i] += qw;

    /* Nominal wave acceleration follows the same transition (zero-mean). */
    if (f->wave_enabled) {
#ifdef WAVE_TRANSPORT
        float a[3] = { f->wave_acc[0], f->wave_acc[1], f->wave_acc[2] };
        f->wave_acc[0] = phi_w * (a[0] + (w[2]*a[1] - w[1]*a[2]) * dt);
        f->wave_acc[1] = phi_w * (a[1] + (w[0]*a[2] - w[2]*a[0]) * dt);
        f->wave_acc[2] = phi_w * (a[2] + (w[1]*a[0] - w[0]*a[1]) * dt);
#else
        for (int i = 0; i < 3; i++) f->wave_acc[i] *= phi_w;
#endif
    }

    mNN_symmetrize(Pnew);
    memcpy(f->P, Pnew, sizeof f->P);

    /* Update convergence flag */
    float trace = f->P[0][0] + f->P[1][1] + f->P[2][2];
    f->converged = (trace < f->conv_thresh);
}

/*
 * χ²(3) 99% quantile — accel innovation cap (overridable for experiments).
 *
 * Re-measured after GROSS_REJECT_MULT moved 9γ → 25γ, since widening the
 * reject gate changes how much work the cap is left doing. Over the 12-seed
 * benchmark (3-D / yaw-only attitude RMS, accel NIS):
 *
 *    6.25   5.55° / 2.06°   reject .046 / .000
 *   11.34   5.65° / 2.31°   reject .007 / .000   ← kept
 *   20.00   5.78° / 2.90°   reject .000 / .000
 *
 * 6.25 buys ~2% of 3-D and ~11% of yaw-only attitude RMS, but it caps often
 * enough to push the gross-reject rate back up to 4.6% on the 3-D path —
 * re-entering, in miniature, the starvation regime that widening
 * GROSS_REJECT_MULT just fixed. 20 gives that up for worse tracking. 11.34
 * sits at the knee, and it is the principled value (the χ²(3) 99% quantile)
 * rather than a fitted one — which matters because the benchmark's noise is
 * uniform (×1.732), so its tails are not Gaussian and must not be used to fit
 * gate quantiles. See docs/math.md §4.7.
 */
#ifndef ACCEL_CHI2_GATE
#define ACCEL_CHI2_GATE 11.34f
#endif

/*
 * Pre-update geometry for the accelerometer path, shared by
 * mekf_update_accel and mekf_accel_probe:
 *
 *   - speed-aided centripetal correction: in a turn the accelerometer
 *     measures gravity plus ω×v, which tilts the apparent gravity exactly
 *     when the autopilot most needs good roll. With speed over ground from
 *     gpsd (v_body ≈ [v, 0, 0] along the bow — leeway neglected):
 *       ω×v = [0, ω_z·v, −ω_y·v]
 *     Subtracting it turns coordinated-turn samples from χ²-gate rejects
 *     into usable gravity references. speed_mps = 0 (no live GPS) is a no-op.
 *   - the |a|/g skip band (linear acceleration)
 *   - z = −normalize(specific force) — the measured gravity direction, body
 *   - h = Rᵀ·[0,0,1] = third row of R — the predicted gravity direction, body
 *
 * ag_out (optional) receives |a|/g whether or not the band accepts, because
 * the caller's quiescence tracker must see every sample.
 * Returns false when the sample must be skipped; z and h are then untouched.
 */
static bool accel_geometry(const mekf_t *f, const imu_sample_t *s,
                           float z[3], float h[3], float *ag_out)
{
    float acc[3] = { s->accel[0], s->accel[1], s->accel[2] };
    if (f->speed_mps > 0.1f) {
        float wy = s->gyro[1] - f->bias[1];
        float wz = s->gyro[2] - f->bias[2];
        acc[1] -= wz * f->speed_mps;
        acc[2] += wy * f->speed_mps;
    }

    float an = v3_norm(acc);
    float ag = an / G_MS2;
    if (ag_out) *ag_out = ag;

    if (an < 1e-6f) return false;
    if (ag < f->accel_skip_lo || ag > f->accel_skip_hi) return false;

    z[0] = -acc[0]/an;  z[1] = -acc[1]/an;  z[2] = -acc[2]/an;

    float R[3][3];
    q_to_R(f->q, R);
    h[0] = R[2][0];  h[1] = R[2][1];  h[2] = R[2][2];
    return true;
}

bool mekf_accel_probe(const mekf_t *f, const imu_sample_t *s,
                      float innov[3], float *d2_out)
{
    if (!f->initialized) return false;

    float z[3], h[3];
    if (!accel_geometry(f, s, z, h, NULL)) return false;

    /* Exactly the choice mekf_update_accel makes — the innovation the daemon
     * would form, not an approximation of it. */
    float H[3][MEKF_N], zhat[3];
    if (!(f->wave_enabled && wave_jacobian(h, f->wave_acc, zhat, H))) {
        dir_jacobian(h, H);
        zhat[0] = h[0]; zhat[1] = h[1]; zhat[2] = h[2];
    }

    for (int i = 0; i < 3; i++) innov[i] = z[i] - zhat[i];

    float PHt[MEKF_N][3], S[3][3], Sinv[3][3];
    dir_PHt(f, H, PHt);
    dir_S(H, PHt, f->Ra * f->Ra_scale, NULL, S);
    if (m33_inv(S, Sinv) < 0) return false;

    /* d² in double: the report averages millions of these and the offline
     * tool has no reason to inherit the filter's float budget. */
    double d2 = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            d2 += (double)innov[i] * Sinv[i][j] * innov[j];
    *d2_out = (float)d2;
    return true;
}

/*
 * Elapsed-time EMA gain, 1 − e^(−Δt/τ).  Header carries the rationale; the
 * short version is that `1.0f - expf(-x)` is a cancellation for small x and
 * loses 2.7 % at 32 kHz against a 30 s constant.  expm1f is exact.
 */
float mekf_ema_alpha(float dt_s, float tau_s)
{
    if (!(dt_s > 0.0f) || !(tau_s > 0.0f)) return 0.0f;
    float a = -expm1f(-dt_s / tau_s);
    return (a > 1.0f) ? 1.0f : a;   /* Δt ≫ τ: take the whole step, never more */
}

void mekf_update_accel(mekf_t *f, const imu_sample_t *s)
{
    if (!f->initialized) return;

    float z[3], h[3], ag = 0.0f;
    bool usable = accel_geometry(f, s, z, h, &ag);

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

    /*
     * Advance the health clock on EVERY sample, accepted or not, so the gain
     * below can be derived from elapsed TIME rather than from a count of
     * updates. This matters more than it looks: in a seaway the |a| band
     * discards 85–95% of samples, so a fixed per-update gain sized from the
     * IMU ODR stretches the intended 30 s time constant to ten minutes or
     * more — long enough that innov_weight/innov_reject/nis_accel report
     * their own initial values instead of the data.
     */
    f->health_dt_accum += f->dt;

    /* Skipped during linear acceleration (|a|/g outside [1±thresh]) */
    if (!usable) return;

    /* Elapsed-time EMA gain — see mekf_ema_alpha for why it is not spelled
     * `1.0f - expf(-x)` here. */
    float alpha_a = mekf_ema_alpha(f->health_dt_accum, GATE_TAU_S);
    f->health_dt_accum = 0.0f;

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

    eskf_update(f, h, z, R_eff, ACCEL_CHI2_GATE,
                &f->nis_accel_ema, alpha_a,
                f->wave_enabled ? f->wave_acc : NULL, NULL);
}

void mekf_update_mag(mekf_t *f, const mag_sample_t *m)
{
    if (!f->initialized || !f->m_ref_valid || !m->valid) return;

    /*
     * An uncalibrated sample carries an uncorrected hard/soft-iron error.  It
     * is still fused, because the heading it yields is wrong by a bounded,
     * repeatable, heading-dependent deviation while dead reckoning is wrong
     * without bound.  Two constraints come with it, both enforced below:
     * the update is forced heading-only so the uncorrected dip cannot reach
     * roll or pitch, and it is withdrawn once the field-magnitude evidence
     * says the hard iron may exceed the horizontal field.
     */
    bool uncal = !m->calibrated;
    if (uncal) {
        if (!f->mag_fuse_uncal) return;
        f->mag_uncal_seen = true;
    }

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
    /*
     * Skipped for an uncalibrated field: both targets are invariants only
     * because a calibrated magnitude does not depend on heading.  Under
     * uncorrected hard iron it does, so the EMA would chase the measurement
     * around a turn and walk the reference with it.
     */
    if (!uncal && f->mref_alpha > 0.0f && f->g_body_valid && res_sq < 0.1f &&
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

    /*
     * Tight anomaly threshold (converged only), scaled to normalised units.
     *
     * res_sq is the full 3-vector residual, so it answers a question the
     * heading-only path never asks: an uncalibrated field has a permanently
     * wrong magnitude and dip, and res_sq stays large once yaw has converged
     * onto the offset heading.  Applying this gate there would reject every
     * sample from the moment the filter converged and revert to dead
     * reckoning without saying so.  YAW_CHI2_GATE inside eskf_update_yaw is
     * the right test for that path and still runs.
     */
    if (!uncal && f->converged && res_sq > f->mag_reject_sq / (h_mag * h_mag))
        return;

    int rc;
    if (f->mag_yaw_only || uncal) {
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
        /*
         * Withdrawal backstop for an uncalibrated field.  Indicated heading
         * stops being monotonic in true heading once the horizontal hard iron
         * exceeds the horizontal field: two true headings then read the same,
         * and the sense of the error inverts over half the rose, so a
         * heading-hold consumer diverges rather than degrades.  That boundary
         * is |b_h| = |H|; mag_anom_ema measures |b|/|B| against the TOTAL
         * field, so the equivalent threshold is the reference ratio
         * |H_ref|/|B_ref| = mh_ref/h_mag, scaled by a margin.
         *
         * Two limits are deliberate.  The EMA is a mean, not a peak — over a
         * constant-rate turn it under-reads the peak by about 2/π — and a
         * mostly-vertical offset inflates it without harming heading here.
         * So this is a backstop against gross iron, not a precise boundary,
         * and the margin is set well below 1.  Hysteresis at 0.8 keeps a
         * marginal installation from chattering.
         *
         * The transition is only recorded here; imu.c logs it.  fusion.c
         * links against libm alone, and two test suites depend on that.
         */
        if (uncal && f->mag_uncal_reject > 0.0f) {
            float limit = f->mag_uncal_reject * (mh_ref / h_mag);
            f->mag_uncal_limit = limit;
            if (!f->mag_uncal_withdrawn && f->mag_anom_ema > limit)
                f->mag_uncal_withdrawn = true;
            else if (f->mag_uncal_withdrawn && f->mag_anom_ema < 0.8f * limit)
                f->mag_uncal_withdrawn = false;
            if (f->mag_uncal_withdrawn) return;
        }

        float R_psi = f->Rm / (mh_ref * mh_ref);
        rc = eskf_update_yaw(f, y, R_psi);
        if (rc == 0) f->mag_accepted++;
    } else {
        /*
         * 3-D vector fusion. Here the field's DIP constrains roll and pitch,
         * which is the mode's whole value — and also its exposure, because the
         * dip of m_ref is the one part of the reference the filter cannot get
         * right on its own.
         *
         * m_ref's dip is fixed at alignment from the tilt estimate available
         * at that moment. In a seaway that tilt is wrong by around a degree
         * even after averaging the alignment window, and the error is not
         * recoverable afterwards: the in-run m_ref EMA can only learn while
         * the platform is quiescent (acc_quiet_ema), because the samples that
         * clear the |a| band in a seaway are wave-phase correlated and
         * learning from them walks the reference away rather than toward the
         * truth. So a dip error simply persists, and in this mode it shows up
         * as a CONSTANT roll/pitch bias that P has no term for.
         *
         * The honest response is to tell P how much that channel is worth. A
         * dip error dδ rotates m_ref about â_NED = ĥ_hor × ê_D, so in body
         * frame it perturbs the normalised prediction along the single unit
         * tangent direction u = (Rᵀâ_NED) × ĥ. The uncertainty is therefore
         * exactly rank-1:
         *
         *     R = Rm_n·I₃ + σ_dip²·u·uᵀ
         *
         * Inflating Rm isotropically would move NEES(strict) just as well, but
         * it deweights the heading-carrying components too and collapses
         * nis_mag to ~0.01 — destroying the wire's magnetometer-health
         * instrument to fix a roll/pitch problem. The rank-1 form leaves them
         * alone.
         *
         * u is tangent (uᵀĥ = 0), so R·ĥ = Rm_n·ĥ and ĥ stays an eigenvector
         * of S with eigenvalue Rm_n — which is what keeps the NIS dof at 2
         * (docs/math.md §8.2). An anisotropy with any radial component would
         * silently break that.
         */
        float R33[3][3];
        const float (*Rp)[3] = NULL;
        float mh_ref = sqrtf(f->m_ref[0]*f->m_ref[0] + f->m_ref[1]*f->m_ref[1]);
        if (f->dip_sig2 > 0.0f && mh_ref > 0.2f * h_mag) {
            /* â_NED = ĥ_hor × ê_D = (ĥ_hor_y, −ĥ_hor_x, 0) */
            float a_ned[3] = { f->m_ref[1]/mh_ref, -f->m_ref[0]/mh_ref, 0.0f };
            float a_body[3];
            m33_Atx(R, a_ned, a_body);
            float u[3] = {
                a_body[1]*h[2] - a_body[2]*h[1],
                a_body[2]*h[0] - a_body[0]*h[2],
                a_body[0]*h[1] - a_body[1]*h[0],
            };
            float un = v3_norm(u);
            if (un > 1e-6f) {
                for (int i = 0; i < 3; i++) u[i] /= un;
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        R33[i][j] = (i == j ? Rm_n : 0.0f)
                                  + f->dip_sig2 * u[i] * u[j];
                Rp = (const float (*)[3])R33;
            }
        }
        /* aw NULL: the magnetic field is not contaminated by acceleration. */
        rc = eskf_update(f, h, z, Rm_n, ACCEL_CHI2_GATE,
                         &f->nis_mag_ema, f->nis_mag_alpha, NULL, Rp);
        if (rc == 0) f->mag_accepted++;
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

/*
 * settle_samples — how many samples of settling a factor·tau ramp is worth.
 *
 * Computed once in double, then counted in uint64_t.  The obvious
 * alternative, summing dt into a float until it reaches factor·tau, does not
 * work at the top of the rate ladder: `x += dt` with a CONSTANT dt stops
 * advancing entirely once dt falls below half an ULP of x, which at 32 kHz
 * happens at 1024 s (2048 s at 16 kHz, 4096 s at 8 kHz).  A window whose ramp
 * runs past that point would leave `settled` false forever — silently, since
 * the only symptom is a valid flag that never sets.
 *
 * Rounded up so a threshold is never met early, and floored at 1 so an
 * absurdly short tau still costs one sample rather than settling instantly.
 */
static uint64_t settle_samples(float factor, float tau_s, float dt)
{
    double n = ceil((double)factor * (double)tau_s / (double)dt);
    if (!(n >= 1.0)) return 1;                 /* also catches NaN */
    if (n > 1e18) return (uint64_t)1e18;       /* far past any real run */
    return (uint64_t)n;
}

void heave_init(heave_t *h, float tau_s, float dt)
{
    memset(h, 0, sizeof *h);
    h->tau      = tau_s;
    h->dt       = dt;
    h->enabled  = (tau_s > 0.0f && dt > 0.0f);
    h->settle_n = h->enabled
        ? settle_samples(HEAVE_SETTLE_FACTOR, tau_s, dt) : 0;
}

float heave_update(heave_t *h, const float q[4], const float accel[3])
{
    if (!h->enabled) return 0.0f;

    h->n++;
    if (!h->settled && h->n >= h->settle_n)
        h->settled = true;

    float R[3][3];
    q_to_R(q, R);
    /*
     * Gravity removal in double: R·accel is ~9.8 and G_MS2 is ~9.8, so this
     * is a cancellation down to a wave signal three or four orders smaller.
     * The rotation stays float — it is the accel sample's own precision, and
     * widening it would flatter the result without changing the hardware.
     */
    const double a_down = (double)R[2][0]*accel[0] + (double)R[2][1]*accel[1]
                        + (double)R[2][2]*accel[2] + (double)G_MS2;

    const double dt   = (double)h->dt;
    const double tau  = (double)h->tau;
    const double leak = dt / tau;
    h->vel  += a_down * dt;
    h->vel  -= h->vel * leak;
    h->disp += h->vel * dt;
    h->disp -= h->disp * leak;

    /* First-order high-pass: y = α·(y + x − x_prev), exact zero at DC.
     * α in double for the same reason as leak: in float it rounds to exactly
     * 1.0 once dt/tau goes under an ULP, and the DC zero disappears. */
    const double alpha = tau / (tau + dt);
    h->hp_y = alpha * (h->hp_y + h->disp - h->disp_prev);
    h->disp_prev = h->disp;

    return (float)(-h->hp_y);   /* NED down → heave positive up */
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
    w->tau      = tau_s;
    w->dt       = dt;
    w->enabled  = (tau_s > 0.0f && dt > 0.0f);
    w->settle_n = w->enabled
        ? settle_samples(SEASTATE_SETTLE_FACTOR, tau_s, dt) : 0;
}

/*
 * One EW mean/variance step: var converges to the variance about the EW mean.
 *
 * Double, deliberately — see the seastate_t declaration for the measured
 * reason.  The incremental form is already the numerically good one (not
 * E[x²] − mean²), so the width is about the GAIN, not the formula: alpha is
 * dt/tau and goes under float32 epsilon at a high rate with a long window.
 */
static void ew_stat(double *mean, double *var, double x, double alpha)
{
    double d = x - *mean;
    *mean += alpha * d;
    *var  += alpha * ((1.0 - alpha) * d * d - *var);
}

void seastate_update(seastate_t *w, float heave_m, float heave_rate,
                     float roll, float roll_rate,
                     float pitch, float pitch_rate)
{
    if (!w->enabled) return;

    /* alpha in double as well: at 32 kHz with a 1200 s window it is 2.6e-8,
     * which float32 still represents fine — but rounding it there before
     * multiplying would give back some of what the double accumulators buy. */
    double alpha = (double)w->dt / (double)w->tau;
    ew_stat(&w->h_mean,  &w->h_var,  (double)heave_m,    alpha);
    ew_stat(&w->hr_mean, &w->hr_var, (double)heave_rate, alpha);
    ew_stat(&w->r_mean,  &w->r_var,  (double)roll,       alpha);
    ew_stat(&w->rr_mean, &w->rr_var, (double)roll_rate,  alpha);
    ew_stat(&w->p_mean,  &w->p_var,  (double)pitch,      alpha);
    ew_stat(&w->pr_mean, &w->pr_var, (double)pitch_rate, alpha);

    w->n++;
    if (!w->settled && w->n >= w->settle_n)
        w->settled = true;
}

float seastate_wave_height(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    return 4.0f * (float)sqrt(w->h_var);
}

float seastate_wave_period(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    if (sqrt(w->h_var) < (double)SEASTATE_MIN_HEAVE_SIG || w->hr_var <= 0.0)
        return 0.0f;
    return (float)(2.0 * M_PI * sqrt(w->h_var / w->hr_var));
}

float seastate_roll_period(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    if (sqrt(w->r_var) < (double)SEASTATE_MIN_ANGLE_SIG || w->rr_var <= 0.0)
        return 0.0f;
    return (float)(2.0 * M_PI * sqrt(w->r_var / w->rr_var));
}

float seastate_roll_amplitude(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    return 2.0f * (float)sqrt(w->r_var);
}

float seastate_pitch_period(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    if (sqrt(w->p_var) < (double)SEASTATE_MIN_ANGLE_SIG || w->pr_var <= 0.0)
        return 0.0f;
    return (float)(2.0 * M_PI * sqrt(w->p_var / w->pr_var));
}

float seastate_pitch_amplitude(const seastate_t *w)
{
    if (!w->enabled || !w->settled) return 0.0f;
    return 2.0f * (float)sqrt(w->p_var);
}

void mekf_reconfigure(mekf_t *f, const imud_config_t *cfg)
{
    /*
     * Shared with mekf_init so no derived value can be updated in one and
     * forgotten in the other. Note this resets accel_skip_lo/hi to the
     * non-engine threshold; the fusion thread re-asserts the engine-mode
     * window on the next sample (imu.c), so the two cannot be left in the
     * half-engine state a SIGHUP would otherwise leave behind.
     */
    bool was_enabled = f->wave_enabled;
    mekf_derive_tuning(f, cfg);

    /*
     * The wave block is the one piece of P a reconfigure cannot leave alone,
     * because enabling and disabling changes which states exist:
     *
     *   off → on:  seed the steady-state variance, else the state is pinned at
     *              its (zero) initial estimate with zero uncertainty and can
     *              never learn anything.
     *   on → off:  clear the block and the nominal estimate, else a stale a_w
     *              would keep biasing the gravity prediction with no dynamics
     *              left to decay it — and the cross-covariances would keep
     *              feeding a state nothing updates.
     *
     * A σ or τ change with the state left enabled needs neither: the GM
     * process re-equilibrates through Q_w within a few τ, and discarding the
     * learned cross-covariance would cost more than the transient.
     */
    if (f->wave_enabled && !was_enabled) {
        for (int i = 6; i < MEKF_N; i++) f->P[i][i] = f->wave_sig2;
    } else if (!f->wave_enabled && was_enabled) {
        for (int i = 0; i < MEKF_N; i++)
            for (int j = 6; j < MEKF_N; j++) f->P[i][j] = f->P[j][i] = 0.0f;
        f->wave_acc[0] = f->wave_acc[1] = f->wave_acc[2] = 0.0f;
    }
}

bool mekf_sanitize(mekf_t *f)
{
    bool ok = true;
    for (int i = 0; i < 4 && ok; i++) if (!isfinite(f->q[i]))        ok = false;
    for (int i = 0; i < 3 && ok; i++) if (!isfinite(f->bias[i]))     ok = false;
    for (int i = 0; i < 3 && ok; i++) if (!isfinite(f->wave_acc[i])) ok = false;
    /* The diagonal is enough: P is symmetric and every off-diagonal term is
     * built from products that pass through it, so a non-finite covariance
     * reaches the diagonal within one propagation step. */
    for (int i = 0; i < MEKF_N && ok; i++) if (!isfinite(f->P[i][i])) ok = false;

    if (ok) return false;

    /* Counted, not logged: fusion.c is deliberately free of every dependency
     * but libm, which is what lets test_fusion link it alone.  The caller owns
     * the message and the rate limiting; the count is here because it belongs
     * to the filter's own state. */
    if (f->reset_count < UINT32_MAX) f->reset_count++;

    mekf_reset_state(f);

    /* Re-align from the next good accel/mag pair rather than carrying on from
     * an identity attitude that means nothing. */
    f->initialized  = false;
    f->converged    = false;
    f->m_ref_valid  = false;
    f->state_reset  = true;
    /* mag_uncal_withdrawn deliberately survives: it is an assessment of the
     * magnetometer's hard iron, which a filter reset does not change, and the
     * mag_anom_ema it was made from survives too. */
    return true;
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

    /* Update-gate health (see gate_health / eskf_update) */
    out->innov_weight = f->innov_weight_ema;
    out->innov_reject = f->innov_reject_ema;

    /* Measurement-model consistency (see nis_record) */
    out->nis_accel = f->nis_accel_ema;
    out->nis_mag   = f->nis_mag_ema;

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
    /* Latched, not a pulse: the bit stays up from the reset until the filter
     * has re-converged, so a consumer sampling far below the output rate can
     * still see that it happened.  Reading it off `converged` rather than
     * clearing the field keeps mekf_get_state const. */
    if (f->state_reset && !f->converged) flags |= FLAG_STATE_RESET;
    out->flags = flags;
}
