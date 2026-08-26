/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fit_ra.h — offline analysis of the MEKF accelerometer measurement model
 * against a real .imucap capture (docs/math.md §4.7).
 *
 * `imud-cal characterize` cannot answer this question: it assumes a
 * stationary platform, gates out anything that moves, and block-averages the
 * samples. What matters here is precisely the moving case — how large the
 * gravity-direction residual really is in a seaway, and whether the filter's
 * covariance predicts it.
 *
 * The core is separated from the CLI so a test can drive it against a
 * synthesised capture (test/test_fit_ra.c).
 */
#ifndef IMUD_FIT_RA_H
#define IMUD_FIT_RA_H

#include <stdbool.h>
#include <stdint.h>

#include "cal.h"
#include "config.h"

typedef struct {
    /* Coverage */
    double   duration_s;       /* replayed span after the settle window */
    uint64_t n_imu;            /* IMU records replayed */
    uint64_t n_mag;            /* mag records replayed */
    uint64_t n_accel_upd;      /* accel updates that passed the |a| skip band */
    uint64_t n_skipped;        /* accel samples skipped by the |a| band */

    /* Residual statistics, in normalised (unit-vector) gravity-direction
     * units — the same units as mekf_t.Ra. */
    double resid_mean[3];      /* per-axis mean of (z − h) */
    double resid_var[3];       /* per-axis variance of (z − h) */
    double resid_var_total;    /* mean of the three per-axis variances */

    /* Innovation consistency at the capture's configured tuning. */
    double nis_mean;           /* mean d²/2; 1.0 = covariance consistent */
    double d2_frac_over_cap;   /* fraction with d² > ACCEL_CHI2_GATE */
    double d2_frac_over_rej;   /* fraction with d² > GROSS_REJECT_MULT × gate */
    double d2_frac_over_9g;    /* fraction with d² > 9 × gate (pre-1.7 gate) */

    /* Residual autocorrelation time, s — how far from white the residual is.
     * A large value is the signature of wave-orbital contamination. 0 when not
     * estimable, and near 0 when the wave state has absorbed the correlation,
     * which is what a correctly tuned filter looks like. Computed from the gap
     * between consecutive ACCEPTED samples, not the IMU period: the |a| band
     * discards most samples in a seaway, so using the raw period understates τ
     * by whatever the acceptance ratio happens to be (typically 3–10×). */
    double resid_tau_s;

    /* Recommendations, both as mekf_accel_noise values (m/s²/√Hz). */
    double na_raw;             /* from the raw residual variance */
    double na_consistent;      /* bisected so mean NIS ≈ 1 */
    bool   na_consistent_ok;   /* false when the bisection did not converge */
    double na_configured;      /* what the config passed in, for comparison */

    /*
     * Gauss–Markov wave state (docs/math.md §4.1.1).
     *
     * The suggestions come from a SEPARATE replay with the wave state forced
     * off, because the disturbance has to be measured before it is modelled:
     * with a well-tuned state enabled, the residual is white and small by
     * construction and would suggest turning the state off. So `wave_*_suggest`
     * describes the seaway; nis_mean and resid_* above describe how the
     * configured filter is coping with it.
     */
    bool   wave_configured;    /* the replayed config had the state enabled */
    double wave_sigma_cfg;     /* mekf_wave_accel as configured, m/s² */
    double wave_tau_cfg;       /* mekf_wave_accel_tau_s as configured, s */
    double wave_sigma_suggest; /* GM σ implied by the unmodelled residual, m/s² */
    double wave_tau_suggest;   /* GM τ implied by the unmodelled residual, s */
    double resid_var_unmodelled;  /* residual variance with the state off */
    double nis_unmodelled;        /* mean NIS with the state off */
} fitra_report_t;

/*
 * Replay `capture_path` through the MEKF and fill `*out`.
 *
 * cfg/cal are applied exactly as the daemon would (mount rotation, then
 * calibration, then gyro temperature compensation), because captures are
 * recorded pre-mount and pre-cal.
 *
 * settle_s discards the filter's acquisition transient, during which P is
 * large and the innovations say nothing about the noise model.
 *
 * Returns 0 on success, -1 on I/O or format error (message via `errbuf`),
 * or -2 when the capture is too short to say anything useful.
 */
int fitra_run(const char *capture_path,
              const imud_config_t *cfg,
              const imud_cal_t *cal,
              double settle_s,
              fitra_report_t *out,
              char *errbuf, size_t errbufsz);

/* Print a human-readable report to stdout. */
void fitra_print(const fitra_report_t *r, const char *capture_path);

#endif /* IMUD_FIT_RA_H */
