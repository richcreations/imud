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
 * Error-state: δx = [δθ(3), δb(3), δa_w(3)] — rotation error, gyro bias error,
 * and wave-acceleration error (a first-order Gauss–Markov process modelling the
 * time-correlated seaway disturbance on the gravity measurement; see §4.1 of
 * docs/math.md).  The wave block is APPENDED so the attitude and bias blocks
 * keep their indices — mekf_get_state and the wire format are untouched.
 * Nominal state: q (unit quaternion, body→NED), b (gyro bias rad/s),
 * a_w (wave acceleration in body frame, normalised gravity units).
 * Covariance: P (9×9, symmetric positive-definite).
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

/*
 * Error-state dimension.  [δθ(3) | δb(3) | δa_w(3)].
 *
 * The wave block is inert (P rows/columns 6–8 identically zero, Φ block = I,
 * H block = 0) unless BOTH mekf_wave_accel and mekf_wave_accel_tau_s are
 * configured positive, in which case the filter is bit-for-bit the 6-state
 * filter it was before.  There is no compile-time or runtime dimension switch.
 */
#define MEKF_N 9

typedef struct {
    /* ── Nominal state ────────────────────────────────────────────────── */
    float q[4];        /* unit quaternion [w, x, y, z], body→NED */
    float bias[3];     /* gyro bias estimate, rad/s */
    float wave_acc[3]; /* wave-acceleration estimate, body frame, g units */

    /* ── Error-state covariance P (9×9) ───────────────────────────────── */
    float P[MEKF_N][MEKF_N];   /* [δθ(3) | δb(3) | δa_w(3)], row-major */

    /* ── Magnetometer reference ───────────────────────────────────────── */
    float m_ref[3];     /* Earth field direction in NED (with magnitude), Gauss */
    bool  m_ref_valid;  /* true once set from first good mag reading */

    /* ── Filter state ─────────────────────────────────────────────────── */
    bool  initialized;   /* true once tilt-init from accel has run */
    bool  converged;     /* true when trace(P[0:3][0:3]) < conv_thresh */
    bool  state_reset;   /* mekf_sanitize reset the state; latched until the
                          * filter next converges (see FLAG_STATE_RESET) */
    uint32_t reset_count;/* resets since startup, for the rate-limited log */

    /* ── Tuning (set by mekf_init, not changed at runtime) ────────────── */
    float dt;            /* predict step period = 1/imu_odr_hz, s */
    float mag_odr_hz;    /* magnetometer rate actually being sampled, Hz.
                          * Stored rather than re-read from cfg because cfg
                          * holds the operator's request, which the driver may
                          * not be able to program — dt carries the IMU's real
                          * rate the same way. */
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

    /* ── Gauss–Markov wave-acceleration state (docs/math.md §4.1.1) ────────
     * Derived in mekf_derive_tuning from mekf_wave_accel / _tau_s. */
    float wave_tau;      /* GM correlation time, s */
    float wave_sig2;     /* GM steady-state variance per axis, g² */
    bool  wave_enabled;  /* both knobs positive — otherwise the block is inert */

    /* ── Magnetic dip-reference uncertainty ───────────────────────────────
     * Variance (rad²) of the DIP angle of m_ref — the one part of the
     * magnetic reference that alignment cannot get right in a seaway and
     * that no amount of in-run learning can heal (see mekf_update_mag).
     * Feeds a rank-1 anisotropic term into the 3-D mag update's R so the
     * error it causes is admitted into P instead of biasing roll/pitch
     * invisibly. 0 = the dip reference is exact (WMM). Unused in yaw-only
     * mode, which never reads the dip channel. */
    float dip_sig2;

    /* ── Compass health diagnostics (τ ≈ 30 s at 100 Hz mag ODR) ──────────
     * Fed BEFORE the rejection gates in mekf_update_mag — rejected samples
     * are evidence of anomaly, not noise to hide. Exported on the wire. */
    float mag_anom_ema;  /* EMA of ||B|−|B_ref||/|B_ref| — interference / iron-cal drift */
    float mag_resid_ema; /* EMA of |heading innovation|, rad — compass-vs-filter disagreement */

    /* ── Update-gate health (τ ≈ 30 s) ────────────────────────────────────
     * Fed by every eskf update (accel, 3-D mag, yaw), so at the 833 Hz accel
     * rate these are effectively an accel-path metric. Together they say how
     * much the filter is having to distrust its own measurements. */
    float innov_weight_ema; /* EMA of the Huber weight √(γ/d²); 1.0 = never capped,
                             * → 0.33 = sustained capping at the reject boundary */
    float innov_reject_ema; /* EMA of the reject indicator; fraction of updates
                             * discarded by the gross-outlier gate */
    float gate_alpha;       /* nominal per-update EMA gain (odr-derived) */
    /* Seconds elapsed since the last accel-path health/NIS update. The |a|
     * skip band throws away most samples in a seaway, so a gain sized from
     * the IMU ODR would stretch the intended 30 s time constant by an order
     * of magnitude; the accel path converts this elapsed time into a gain
     * instead, which is rate-independent. */
    float health_dt_accum;

    /* ── Measurement-model consistency: rolling NIS (τ ≈ 30 s) ────────────
     * EMA of the normalised innovation squared d²/dof, where
     * d² = νᵀS⁻¹ν and S = HPHᵀ + R. Reads 1.0 when the filter's own
     * covariance correctly predicts the spread of its innovations, > 1
     * when it is over-confident (measurements disagree with P more than P
     * claims they should). This is the field instrument for docs/math.md §4.7:
     * unlike innov_weight/innov_reject — which say how hard the Huber cap
     * is being LEANED ON — NIS says how wrong the model is, and it keeps
     * rising after the cap saturates.
     *
     * Accumulated PRE-cap and including gross-outlier-rejected updates: the cap
     * censors d² at the gate, so a post-cap average would be bounded by
     * construction and could never report the inconsistency it exists to
     * measure.
     *
     * Split by channel because the two run at very different rates — at
     * 833 Hz accel vs ~104 Hz mag a combined EMA would be ~8:1 accel and
     * the mag signal would be invisible — and because they have different
     * dof (3 for accel and 3-D mag, 1 for yaw-only). */
    float nis_accel_ema;    /* accel gravity update, d²/2 */
    float nis_mag_ema;      /* mag update: d²/2 (3-D) or d²/1 (yaw-only) */
    float nis_mag_alpha;    /* per-update EMA gain for nis_mag_ema (mag ODR) */
} mekf_t;

/*
 * mekf_init — initialise filter state from config.
 *   gyro_bias_init: pre-computed bias from startup still window (may be zero).
 *   odr_hz:         IMU output data rate (determines dt).
 *   mag_odr_hz:     magnetometer output data rate.
 *
 * Both rates must be the rates the drivers actually programmed
 * (imu_ctx_open resolves them with odr_actual_imu / odr_actual_mag), not the
 * cfg->*_odr_hz requests — every noise variance and EMA gain derived from
 * them would otherwise be sized for a rate the hardware is not running at.
 * Both must be > 0; config rejects anything else at parse time.
 */
void mekf_init(mekf_t *f,
               const imud_config_t *cfg,
               float odr_hz,
               float mag_odr_hz,
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
 * mekf_accel_probe — the pre-update half of mekf_update_accel, exposed for
 * offline analysis (imud-cal fit-ra).  Reproduces the centripetal correction,
 * the |a| skip band, the measured and predicted gravity directions, and the
 * normalised innovation distance d² = νᵀS⁻¹ν, all from the filter's own state
 * and WITHOUT modifying it.
 *
 * Exported rather than reimplemented so the offline tool and the daemon can
 * never drift apart — the two copies did, silently, before this existed.
 *
 * Returns false when the sample would be skipped (|a| outside the band, or a
 * singular S); innov and d2_out are then untouched.
 */
bool mekf_accel_probe(const mekf_t *f, const imu_sample_t *s,
                      float innov[3], float *d2_out);

/*
 * mekf_update_mag — magnetometer heading correction.
 * Skipped if m_ref not yet valid, or residual exceeds mag_reject_gauss.
 * Expects mag in calibrated body frame, µT (Z sign already flipped by driver).
 */
void mekf_update_mag(mekf_t *f, const mag_sample_t *m);

/*
 * mekf_sanitize — reject a non-finite filter state before it can be published.
 *
 * Checks q, the gyro bias, the wave acceleration and P's diagonal.  If any is
 * non-finite the nominal state and covariance go back to their initial values,
 * `initialized` clears so the filter re-aligns from the next good accel/mag
 * pair, and FLAG_STATE_RESET is raised until it re-converges.  Tuning is left
 * alone — it comes from a validated config and cannot be the source.
 *
 * Returns true if it acted, false if the state was already finite.
 *
 * Call once per sample AFTER the predict and update steps and BEFORE
 * mekf_get_state: that ordering is what guarantees no packet can carry a
 * non-finite attitude, since all three producers have run by then.
 *
 * This is a backstop for arithmetic, not for input.  Non-finite values from
 * cal.json and imud.conf are rejected by their loaders, which is where an
 * operator can still be told which line is wrong.
 */
bool mekf_sanitize(mekf_t *f);

/*
 * mekf_get_state — extract nominal state into fused_state_t for output.
 * flags_in: FLAG_* bits set by the caller before this call (e.g. cal flags).
 * The function adds FLAG_FUSION_CONVERGED and FLAG_STARTUP as appropriate.
 */
void mekf_get_state(const mekf_t *f, fused_state_t *out, uint16_t flags_in);

/*
 * mekf_ema_alpha — per-update EMA gain for an elapsed interval: 1 − e^(−Δt/τ).
 *
 * Exact for any feed rate, and equal to Δt/τ in the limit of frequent
 * updates, which is what makes an EMA rate-independent when its input is
 * delivered irregularly (the accel path's |a| skip band being the case that
 * needs it).
 *
 * Named rather than inlined at its one call site because the implementation
 * has a trap in it: Δt/τ is small, so `1.0f - expf(-x)` subtracts two numbers
 * differing by less than an ULP of 1.0, quantising the result to multiples of
 * 5.96e-8 — 2.7 % relative error at 32 kHz against a 30 s constant, and only
 * 17 distinct representable values. expm1f is exact. Anything else needing an
 * elapsed-time gain should call this and not rediscover that.
 */
float mekf_ema_alpha(float dt_s, float tau_s);

/* ── Heave estimator ──────────────────────────────────────────────────────── */

/*
 * Leaky double integration of NED vertical linear acceleration.
 * Self-contained; driven by fusion_thread after each predict/update cycle.
 * τ = 0 disables (heave_update returns 0.0).
 */
typedef struct {
    float tau;       /* leak / high-pass time constant, s */
    float dt;        /* sample period, s */
    /*
     * Double, for a sharper version of the reason seastate_t gives.  Both
     * stages are leaky — vel -= vel*(dt/tau) — and the leak is what bounds
     * the drift of a double integration of accelerometer noise.  In float32
     * that leak becomes a no-op once dt/tau falls below half an ULP, and the
     * filter silently turns into a PLAIN double integrator.  The output
     * high-pass fails the same way: alpha = tau/(tau+dt) rounds to exactly
     * 1.0 and the exact zero at DC is gone.
     *
     * Measured against a double reference of the same arithmetic
     * (BENCH_SWEEP_PRECISION), recovered amplitude error:
     *
     *     32 kHz / tau  12 s   0.225 %      <- shipped default
     *     32 kHz / tau  60 s   1.225 %
     *     32 kHz / tau 300 s   7.263 %
     *     32 kHz / tau 900 s   6543974 %    <- 253 m of "heave"
     *
     * That last row is not a degraded reading, it is an unbounded integrator
     * with FLAG_HEAVE_VALID set.  Double moves the cliff to dt/tau ~ 1e-16,
     * which nothing reaches.  heave_update still returns float.
     */
    double vel;       /* leaked vertical velocity, m/s (down +) */
    double disp;      /* leaked vertical displacement, m (down +) */
    double disp_prev; /* previous displacement (high-pass state) */
    double hp_y;      /* high-passed displacement (down +) */
    /*
     * Settling is counted in SAMPLES, not summed in seconds.  A float
     * accumulating a constant dt stops advancing once dt drops below half an
     * ULP of the running total — at 32 kHz that is 1024 s, and a heave_tau_s
     * above ~102 s would then never reach 10·tau, leaving `settled` false
     * forever with no error anywhere.  A counter is exact at every rate for
     * any run length, and says what the code means.
     */
    uint64_t n;         /* samples fed since enable */
    uint64_t settle_n;  /* n needed for `settled`; ~10·tau worth of samples */
    bool  settled;   /* true once n ≥ settle_n — heave trustworthy */
    bool  enabled;
} heave_t;

void  heave_init(heave_t *h, float tau_s, float dt);

/* Returns heave in metres, positive up. q = current attitude (body→NED),
 * accel = calibrated specific force in body frame (m/s²). */
float heave_update(heave_t *h, const float q[4], const float accel[3]);

/* ── Sea-state estimator ──────────────────────────────────────────────────── */

/*
 * Windowed statistics over the heave, roll, and pitch oscillations, via
 * exponentially weighted mean/variance pairs (time constant wave_tau_s).
 * Spectral-moment identities give the outputs without any FFT or storage:
 *   Hs      = 4·σ(heave)                    — significant wave height, m
 *   Tz      = 2π·√(var(heave)/var(ḣeave))   — mean zero-crossing period, s
 *   T_roll  = 2π·√(var(roll)/var(ṙoll))     — natural roll period, s
 *   A_roll  = 2·σ(roll)                     — significant single amplitude, rad
 *   (pitch: same pair as roll)
 * Feed only while the heave estimator is settled; τ = 0 disables.
 */
typedef struct {
    float tau;       /* averaging time constant, s */
    float dt;        /* sample period, s */
    /*
     * The accumulators are DOUBLE, and this is the one place in the filter
     * where that is load-bearing rather than tidy.  The EMA gain is dt/tau,
     * so it shrinks with the sample rate AND with the window; at 32 kHz with
     * a 1200 s window it is 2.6e-8, well under float32's 1.19e-7 epsilon, and
     * `mean += alpha*d` stops making progress.  Measured against a
     * double-precision reference of the same arithmetic (BENCH_SWEEP_PRECISION):
     *
     *     dt/tau        Hs error   Tz error
     *     1.0e-5  (833 Hz/120 s)      0.000 %    0.000 %   <- shipped default
     *     2.1e-7  (8 kHz/600 s)       0.128 %    0.491 %
     *     5.2e-8  (32 kHz/600 s)      1.823 %    0.982 %
     *     2.6e-8  (32 kHz/1200 s)    17.995 %    6.865 %
     *
     * — and that last row is a valid-flagged 18 % under-report of significant
     * wave height, not a missing reading.  Both long windows are documented
     * practice (10–20 min records), so the combination is reachable.  Double
     * moves the same cliff out past dt/tau ~ 1e-16, which no rate reaches.
     * The accessors still return float; only the accumulation is widened.
     */
    double h_mean,  h_var;   /* heave, m / m² */
    double hr_mean, hr_var;  /* heave rate, m/s / (m/s)² */
    double r_mean,  r_var;   /* roll, rad / rad² (mean = steady heel) */
    double rr_mean, rr_var;  /* roll rate, rad/s / (rad/s)² */
    double p_mean,  p_var;   /* pitch, rad / rad² (mean = steady trim) */
    double pr_mean, pr_var;  /* pitch rate, rad/s / (rad/s)² */
    /* Samples, not summed seconds — see heave_t for why.  This one is the
     * reachable case: `settled` needs 2·wave_tau_s, and wave_tau_s has no
     * upper bound while the window this estimator is built for is minutes. */
    uint64_t n;         /* samples of (heave-valid) input accumulated */
    uint64_t settle_n;  /* n needed for `settled`; ~2·tau worth of samples */
    bool  settled;   /* true once n ≥ settle_n — stats trustworthy */
    bool  enabled;
} seastate_t;

void seastate_init(seastate_t *w, float tau_s, float dt);

/* Accumulate one sample. Call ONLY while heave is settled (its output is a
 * ramp during settling and would poison the variances). Angles/rates in rad,
 * rad/s (Euler rates, not body ω). */
void seastate_update(seastate_t *w, float heave_m, float heave_rate,
                     float roll, float roll_rate,
                     float pitch, float pitch_rate);

/* Output getters; all return 0.0 when disabled or not yet settled. Periods
 * additionally return 0.0 when the respective oscillation is too small to
 * time (becalmed / not rolling). Amplitudes are significant SINGLE
 * amplitudes (2σ, the seakeeping convention); wave height is the significant
 * DOUBLE amplitude (4σ, crest-to-trough). */
float seastate_wave_height(const seastate_t *w);     /* Hs, m */
float seastate_wave_period(const seastate_t *w);     /* Tz, s */
float seastate_roll_period(const seastate_t *w);     /* s */
float seastate_roll_amplitude(const seastate_t *w);  /* 2σ(roll), rad */
float seastate_pitch_period(const seastate_t *w);    /* s */
float seastate_pitch_amplitude(const seastate_t *w); /* 2σ(pitch), rad */

/*
 * mekf_reconfigure — update filter noise parameters from a new config.
 * Safe to call from the fusion thread at any time after mekf_init.
 * Only the hot-reloadable scalar fields are updated; dt, q, P, and bias
 * are left untouched so the filter continues running without interruption.
 */
void mekf_reconfigure(mekf_t *f, const imud_config_t *cfg);

#endif /* IMUD_FUSION_H */
