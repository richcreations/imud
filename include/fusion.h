/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fusion.h — MEKF (Multiplicative Extended Kalman Filter) for 9-DOF AHRS
 *
 * Reference: Joan Solà, "Quaternion Kinematics for the Error-State Kalman
 * Filter," arXiv:1711.02508.  Notation follows that paper directly.
 *
 * Error-state: δx = [δθ(3), δb(3)] — rotation error + gyro bias error.
 * Nominal state: q (unit quaternion, body→NED), b (gyro bias rad/s).
 * Covariance: P (6×6, symmetric positive-definite).
 *
 * Coordinate convention:
 *   q represents the body→NED rotation:  v_NED = R(q) × v_body
 *   g_ref = [0, 0, 1] — unit gravity in NED (Z-down)
 *   Accel measurement passed to mekf_update_accel() must be the GRAVITY
 *   DIRECTION in body frame: z = −normalize(specific_force_body).
 *   ISM330DHCX reads +g when Z points up, so: z = −normalize(accel_body).
 */
#ifndef IMUD_FUSION_H
#define IMUD_FUSION_H

#include <stdbool.h>
#include "types.h"
#include "config.h"

typedef struct {
    /* ── Nominal state ────────────────────────────────────────────────── */
    float q[4];      /* unit quaternion [w, x, y, z], body→NED */
    float bias[3];   /* gyro bias estimate, rad/s */

    /* ── Error-state covariance P (6×6) ───────────────────────────────── */
    float P[6][6];   /* [δθ(3) | δb(3)], row-major */

    /* ── Magnetometer reference ───────────────────────────────────────── */
    float m_ref[3];     /* Earth field direction in NED (with magnitude), Gauss */
    bool  m_ref_valid;  /* true once set from first good mag reading */

    /* ── Filter state ─────────────────────────────────────────────────── */
    bool  initialized;   /* true once tilt-init from accel has run */
    bool  converged;     /* true when trace(P[0:3][0:3]) < conv_thresh */

    /* ── Tuning (set by mekf_init, not changed at runtime) ────────────── */
    float dt;            /* predict step period = 1/imu_odr_hz, s */
    float Qg;            /* gyro angle noise var per step, rad² */
    float Qb;            /* gyro bias noise var per step, (rad/s)² */
    float Ra;            /* accel meas noise var (normalised units) */
    float Rm;            /* mag meas noise var per axis, Gauss² */
    float accel_skip_lo; /* skip accel update if |a|/g < this */
    float accel_skip_hi; /* skip accel update if |a|/g > this */
    float Ra_scale;      /* extra accel-noise inflation (engine vibration ×4) */
    float mag_reject_sq; /* skip mag update if |residual|² > this */
    float conv_thresh;   /* trace threshold for converged flag */
    bool  mag_yaw_only;  /* heading-only mag fusion (marine default) */
    float mref_alpha;    /* per-sample EMA gain for m_ref re-estimation */
    float g_body[3];     /* latest accel-measured gravity direction, body */
    bool  g_body_valid;  /* true once an accel update has stashed g_body */
    float acc_quiet_ema; /* EMA of (|a|/g−1)², τ≈2 s — platform quiescence */
    float speed_mps;     /* speed over ground for centripetal correction; 0 = off */
} mekf_t;

/*
 * mekf_init — initialise filter state from config.
 *   gyro_bias_init: pre-computed bias from startup still window (may be zero).
 *   odr_hz:         IMU output data rate (determines dt).
 */
void mekf_init(mekf_t *f,
               const imud_config_t *cfg,
               float odr_hz,
               const float gyro_bias_init[3]);

/*
 * mekf_align — call with the first ~N static accel+mag readings to
 * initialise attitude (tilt from accel, heading from mag).
 * Safe to call multiple times; uses the most recent values.
 */
void mekf_align(mekf_t *f, const float accel[3], const float mag[3]);

/*
 * mekf_set_mref_invariants — override the magnetic reference magnitude
 * (horizontal component, Gauss) and dip (vertical component, Gauss, down +)
 * with analytically known values (WMM at a known position). Preserves the
 * horizontal DIRECTION (the heading anchor). No-op before alignment.
 */
void mekf_set_mref_invariants(mekf_t *f, float h_gauss, float z_gauss);

/*
 * mekf_predict — gyro integration + covariance propagation.
 * Call for every IMU sample at the configured ODR.
 * dt_s: measured interval since the previous sample (from hardware
 * timestamps); pass 0 or f->dt to use the nominal 1/ODR period.
 */
void mekf_predict(mekf_t *f, const imu_sample_t *s, float dt_s);

/*
 * mekf_update_accel — accelerometer tilt correction.
 * Skipped if |a|/g outside [1±accel_skip_thresh] (linear acceleration).
 * Expects accel in body frame as calibrated m/s² (specific force convention).
 */
void mekf_update_accel(mekf_t *f, const imu_sample_t *s);

/*
 * mekf_update_mag — magnetometer heading correction.
 * Skipped if m_ref not yet valid, or residual exceeds mag_reject_gauss.
 * Expects mag in calibrated body frame, µT (Z sign already flipped by driver).
 */
void mekf_update_mag(mekf_t *f, const mag_sample_t *m);

/*
 * mekf_get_state — extract nominal state into fused_state_t for output.
 * flags_in: FLAG_* bits set by the caller before this call (e.g. cal flags).
 * The function adds FLAG_FUSION_CONVERGED and FLAG_STARTUP as appropriate.
 */
void mekf_get_state(const mekf_t *f, fused_state_t *out, uint16_t flags_in);

/* ── Heave estimator ──────────────────────────────────────────────────────── */

/*
 * Leaky double integration of NED vertical linear acceleration.
 * Self-contained; driven by fusion_thread after each predict/update cycle.
 * τ = 0 disables (heave_update returns 0.0).
 */
typedef struct {
    float tau;       /* leak / high-pass time constant, s */
    float dt;        /* sample period, s */
    float vel;       /* leaked vertical velocity, m/s (down +) */
    float disp;      /* leaked vertical displacement, m (down +) */
    float disp_prev; /* previous displacement (high-pass state) */
    float hp_y;      /* high-passed displacement (down +) */
    float elapsed;   /* seconds since enable (settling ramp) */
    bool  settled;   /* true once elapsed ≥ ~10·tau — heave trustworthy */
    bool  enabled;
} heave_t;

void  heave_init(heave_t *h, float tau_s, float dt);

/* Returns heave in metres, positive up. q = current attitude (body→NED),
 * accel = calibrated specific force in body frame (m/s²). */
float heave_update(heave_t *h, const float q[4], const float accel[3]);

/*
 * mekf_reconfigure — update filter noise parameters from a new config.
 * Safe to call from the fusion thread at any time after mekf_init.
 * Only the hot-reloadable scalar fields are updated; dt, q, P, and bias
 * are left untouched so the filter continues running without interruption.
 */
void mekf_reconfigure(mekf_t *f, const imud_config_t *cfg);

#endif /* IMUD_FUSION_H */
