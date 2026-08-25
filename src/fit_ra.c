/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fit_ra.c — measure the accelerometer measurement model against a real
 * capture (see fit_ra.h and ROADMAP §10.1).
 *
 * Method: replay the .imucap through a live MEKF exactly as the daemon
 * would, and at each accel update compute the innovation and its normalised
 * distance d² = νᵀS⁻¹ν from the filter's own state, BEFORE the update is
 * applied. Those are the same quantities the daemon's nis_accel field
 * averages, so the offline report and the live wire metric are directly
 * comparable — that is the point of computing them here rather than reading
 * them out of mekf_t afterwards.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fit_ra.h"
#include "capture.h"
#include "fusion.h"
#include "imu_math.h"

#define G_MS2  9.80665f

/* Must match fusion.c; kept local rather than exported because these are the
 * filter's private tuning, and fit-ra only REPORTS against them — the
 * innovation and its d² come from mekf_accel_probe, which is the filter's own
 * code, so the two cannot drift on anything that affects a number. */
#define FITRA_CHI2_GATE   11.34f
#define FITRA_REJECT_MULT 25.0f
#define FITRA_OLD_MULT     9.0f

/* Accumulators for one replay pass. */
typedef struct {
    double   sum[3], sumsq[3];
    double   sum_d2;
    uint64_t n;
    uint64_t n_over_cap, n_over_rej, n_over_9g;
    uint64_t n_mag_used;
    /* lag-1 autocorrelation of the residual magnitude, for resid_tau_s */
    double   ac_num, ac_den, prev_dev[3];
    bool     have_prev;
    double   dt_mean;
    /*
     * Mean interval between consecutive ACCEPTED samples — the lag the
     * autocorrelation above is actually computed at.
     *
     * This is not the IMU period. The |a| skip band throws away most samples
     * in a seaway (66% on the synthetic capture, 85–95% in a real one), so
     * consecutive accepted samples are 3–10 periods apart. Dividing by the IMU
     * period, as this did before, therefore reported τ that many times too
     * SMALL — which is why the recorded field figure of "τ ≈ 0.3–0.9 s" is a
     * lower bound on the real correlation time, and why it must not be used to
     * set mekf_wave_accel_tau_s without this correction.
     */
    double   gap_sum, prev_t;
    uint64_t gap_n;
} fitra_acc_t;

/*
 * Lag-1 autocorrelation → exponential correlation time τ = −gap/ln(ρ), where
 * gap is the mean interval between the ACCEPTED samples the autocorrelation
 * was computed over (see fitra_acc_t). White noise gives ρ ≈ 0 and τ ≈ 0;
 * wave contamination gives ρ → 1. Returns 0 when not estimable.
 */
static double fitra_tau(const fitra_acc_t *a)
{
    if (a->ac_den <= 0.0 || a->gap_n == 0) return 0.0;
    double rho = a->ac_num / a->ac_den;
    if (!(rho > 1e-6 && rho < 0.999999)) return 0.0;
    double gap = a->gap_sum / (double)a->gap_n;
    if (!(gap > 0.0)) gap = a->dt_mean;
    return -gap / log(rho);
}

/*
 * One full replay. `na` overrides cfg->mekf_accel_noise (the bisection
 * re-runs this); pass 0 to use the configured value.
 */
static int replay(const char *path, const imud_config_t *cfg_in,
                  const imud_cal_t *cal, double settle_s, double na,
                  fitra_acc_t *a, double *dur_out,
                  uint64_t *n_imu, uint64_t *n_mag, uint64_t *n_skip,
                  char *errbuf, size_t errbufsz)
{
    cap_reader_t r;
    int rc = cap_reader_open(&r, path);
    if (rc != 0) {
        snprintf(errbuf, errbufsz, "%s: %s", path,
                 rc == CAP_ERR_FORMAT ? "not a valid .imucap file"
                                      : "cannot read");
        return -1;
    }

    imud_config_t cfg = *cfg_in;
    if (na > 0.0) cfg.mekf_accel_noise = na;

    /*
     * Prefer the capture's exact milli-Hz rate, then its whole-Hz field (a
     * file written before imu_odr_mhz existed), then the config.  Compared as
     * double so the ?: does not silently change signedness.
     */
    float odr = (float)(r.hdr.imu_odr_mhz
                            ? (double)r.hdr.imu_odr_mhz * 1e-3
                            : r.hdr.imu_odr_hz ? (double)r.hdr.imu_odr_hz
                                               : (double)cfg.imu_odr_mhz * 1e-3);
    if (odr <= 0.0f) odr = 833.0f;

    mekf_t f;
    float bias0[3] = {0, 0, 0};
    mekf_init(&f, &cfg, odr, (float)cfg.mag_odr_mhz * 1e-3f, bias0);

    memset(a, 0, sizeof *a);
    *n_imu = *n_mag = *n_skip = 0;

    cap_record_t rec;
    bool     aligned = false, have_imu = false;
    imu_sample_t last_imu;
    memset(&last_imu, 0, sizeof last_imu);
    uint32_t prev_ticks = 0;
    double   t0 = -1.0, t_now = 0.0, t_meas0 = -1.0;
    double   sum_dt = 0.0; uint64_t n_dt = 0;

    while (cap_reader_next(&r, &rec) == 1) {
        double t = (double)rec.mono_ns * 1e-9;
        if (t0 < 0.0) t0 = t;
        t_now = t - t0;

        if (rec.type == CAP_REC_IMU) {
            imu_sample_t s = rec.imu;
            /* Captures are pre-mount and pre-cal; imu_finalise_sample() is
             * the same code the daemon runs, so the two cannot drift. */
            imu_finalise_sample(&cfg, cal, &s);

            (*n_imu)++;
            last_imu = s;
            have_imu = true;
            if (!aligned) { prev_ticks = s.chip_ts; continue; }

            float dts = (float)((uint32_t)(s.chip_ts - prev_ticks) * 25e-6);
            prev_ticks = s.chip_ts;
            if (!(dts > 0.0f) || dts > 1.0f) dts = f.dt;
            sum_dt += dts; n_dt++;

            mekf_predict(&f, &s, dts);

            if (t_now >= settle_s) {
                if (t_meas0 < 0.0) t_meas0 = t_now;
                float innov[3], d2;
                if (mekf_accel_probe(&f, &s, innov, &d2)) {
                    for (int i = 0; i < 3; i++) {
                        a->sum[i]   += innov[i];
                        a->sumsq[i] += (double)innov[i] * innov[i];
                    }
                    a->sum_d2 += d2 / 2.0;   /* dof = 2; see fusion.c */
                    a->n++;
                    if (d2 > FITRA_CHI2_GATE)                     a->n_over_cap++;
                    if (d2 > FITRA_REJECT_MULT * FITRA_CHI2_GATE) a->n_over_rej++;
                    if (d2 > FITRA_OLD_MULT    * FITRA_CHI2_GATE) a->n_over_9g++;

                    if (a->have_prev) {
                        for (int i = 0; i < 3; i++) {
                            a->ac_num += (double)innov[i] * a->prev_dev[i];
                            a->ac_den += (double)innov[i] * innov[i];
                        }
                        a->gap_sum += t_now - a->prev_t;
                        a->gap_n++;
                    }
                    for (int i = 0; i < 3; i++) a->prev_dev[i] = innov[i];
                    a->prev_t    = t_now;
                    a->have_prev = true;
                } else {
                    (*n_skip)++;
                }
            }
            mekf_update_accel(&f, &s);

        } else if (rec.type == CAP_REC_MAG) {
            mag_sample_t m = rec.mag;
            mag_finalise_sample(&cfg, cal, &m);
            m.valid = cal->has_mag;
            (*n_mag)++;

            if (!aligned && have_imu) {
                mekf_align(&f, last_imu.accel, m.field);
                aligned = true;
                continue;
            }
            if (aligned && m.valid) { mekf_update_mag(&f, &m); a->n_mag_used++; }
        }
    }
    cap_reader_close(&r);

    a->dt_mean = n_dt ? sum_dt / (double)n_dt : 1.0 / odr;
    *dur_out = (t_meas0 >= 0.0) ? (t_now - t_meas0) : 0.0;

    if (!aligned) {
        snprintf(errbuf, errbufsz,
                 "capture never aligned — needs at least one mag record "
                 "after the first IMU record");
        return -2;
    }
    /*
     * Without heading updates the yaw error state is unobservable, P grows
     * without bound, and every innovation statistic below becomes a
     * measurement of that divergence rather than of the accel noise model.
     * The daemon marks mag samples invalid when cal.json has no mag section,
     * and this replay mirrors that faithfully — so an uncalibrated capture
     * has to be refused rather than silently reported.
     */
    if (a->n_mag_used == 0) {
        snprintf(errbuf, errbufsz,
                 "no magnetometer updates were accepted — cal.json has no mag "
                 "calibration, so heading is unconstrained and the covariance "
                 "diverges. Run `imud-cal mag` first.");
        return -2;
    }
    if (a->n < 1000) {
        snprintf(errbuf, errbufsz,
                 "only %llu usable accel updates after the %.0f s settle "
                 "window — capture is too short",
                 (unsigned long long)a->n, settle_s);
        return -2;
    }
    return 0;
}

int fitra_run(const char *capture_path,
              const imud_config_t *cfg,
              const imud_cal_t *cal,
              double settle_s,
              fitra_report_t *out,
              char *errbuf, size_t errbufsz)
{
    memset(out, 0, sizeof *out);
    out->na_configured = cfg->mekf_accel_noise;

    fitra_acc_t a;
    double dur; uint64_t n_imu, n_mag, n_skip;
    int rc = replay(capture_path, cfg, cal, settle_s, 0.0,
                    &a, &dur, &n_imu, &n_mag, &n_skip, errbuf, errbufsz);
    if (rc != 0) return rc;

    out->duration_s  = dur;
    out->n_imu       = n_imu;
    out->n_mag       = n_mag;
    out->n_accel_upd = a.n;
    out->n_skipped   = n_skip;

    double vtot = 0.0;
    for (int i = 0; i < 3; i++) {
        double mean = a.sum[i] / (double)a.n;
        double var  = a.sumsq[i] / (double)a.n - mean * mean;
        if (var < 0.0) var = 0.0;
        out->resid_mean[i] = mean;
        out->resid_var[i]  = var;
        vtot += var;
    }
    out->resid_var_total  = vtot / 3.0;
    out->nis_mean         = a.sum_d2 / (double)a.n;
    out->d2_frac_over_cap = (double)a.n_over_cap / (double)a.n;
    out->d2_frac_over_rej = (double)a.n_over_rej / (double)a.n;
    out->d2_frac_over_9g  = (double)a.n_over_9g  / (double)a.n;

    out->resid_tau_s = fitra_tau(&a);

    /*
     * na_raw: the noise density whose per-sample variance equals the measured
     * residual variance. Ra = (Na/g)²·odr, so Na = g·√(Ra/odr).
     */
    double odr = (a.dt_mean > 0.0) ? 1.0 / a.dt_mean : 833.0;
    out->na_raw = G_MS2 * sqrt(out->resid_var_total / odr);

    /*
     * Gauss–Markov suggestions, from a replay with the wave state forced OFF.
     *
     * It has to be a separate pass. Once the state is enabled and roughly
     * right it absorbs the correlated part of the residual — that is the whole
     * point — so the residual measured through the configured filter is white
     * and small, and reading σ off it would recommend switching the state off.
     * What the operator needs is the disturbance the filter is up against,
     * which is what the unmodelled residual measures.
     */
    out->wave_sigma_cfg  = cfg->mekf_wave_accel;
    out->wave_tau_cfg    = cfg->mekf_wave_accel_tau_s;
    out->wave_configured = (cfg->mekf_wave_accel > 0.0 &&
                            cfg->mekf_wave_accel_tau_s > 0.0);
    {
        imud_config_t nowave = *cfg;
        nowave.mekf_wave_accel       = 0.0;
        nowave.mekf_wave_accel_tau_s = 0.0;

        fitra_acc_t u; double ud; uint64_t ui, um, us; char ue[128];
        if (replay(capture_path, &nowave, cal, settle_s, 0.0,
                   &u, &ud, &ui, &um, &us, ue, sizeof ue) == 0 && u.n > 0) {
            double uv = 0.0;
            for (int i = 0; i < 3; i++) {
                double mean = u.sum[i] / (double)u.n;
                double var  = u.sumsq[i] / (double)u.n - mean * mean;
                if (var > 0.0) uv += var;
            }
            uv /= 3.0;
            out->resid_var_unmodelled = uv;
            out->nis_unmodelled       = u.sum_d2 / (double)u.n;
            out->wave_tau_suggest     = fitra_tau(&u);

            /*
             * σ from the part of the residual that white sensor noise cannot
             * explain. The residual is a unit-vector difference, so its
             * variance is already in gravity units: subtract the per-sample
             * measurement variance Ra = (Na/g)²·odr and what is left is the
             * disturbance. Multiplying by g puts it back in m/s².
             */
            double Ra = (out->na_configured / G_MS2) *
                        (out->na_configured / G_MS2) * odr;
            double excess = uv - Ra;
            if (excess > 0.0)
                out->wave_sigma_suggest = G_MS2 * sqrt(excess);
        }
    }

    /*
     * na_consistent: bisect Na so the mean NIS lands at 1. NIS falls
     * monotonically as Na rises (bigger R, smaller d²), so plain bisection on
     * log Na is well behaved. Each probe is a full replay, so the bracket is
     * kept deliberately small.
     */
    double lo = out->na_configured * 0.05, hi = out->na_configured * 50.0;
    if (lo < 1e-5) lo = 1e-5;
    fitra_acc_t pa; double pd; uint64_t pi, pm, ps; char pe[128];
    double best = 0.0; bool ok = false;
    for (int it = 0; it < 18; it++) {
        double mid = sqrt(lo * hi);          /* geometric midpoint */
        if (replay(capture_path, cfg, cal, settle_s, mid,
                   &pa, &pd, &pi, &pm, &ps, pe, sizeof pe) != 0 || pa.n == 0)
            break;
        double nis = pa.sum_d2 / (double)pa.n;
        best = mid;
        if (fabs(nis - 1.0) < 0.05) { ok = true; break; }
        if (nis > 1.0) lo = mid; else hi = mid;
    }
    out->na_consistent    = best;
    out->na_consistent_ok = ok;
    return 0;
}

void fitra_print(const fitra_report_t *r, const char *capture_path)
{
    printf("fit-ra: %s\n", capture_path);
    printf("  replayed        %.1f s after settle "
           "(%llu IMU, %llu mag records)\n",
           r->duration_s, (unsigned long long)r->n_imu,
           (unsigned long long)r->n_mag);
    printf("  accel updates   %llu used, %llu skipped by the |a| band (%.1f%%)\n",
           (unsigned long long)r->n_accel_upd,
           (unsigned long long)r->n_skipped,
           100.0 * (double)r->n_skipped /
               (double)(r->n_accel_upd + r->n_skipped ? r->n_accel_upd + r->n_skipped : 1));

    printf("\n  Gravity-direction residual (normalised units)\n");
    printf("    mean      % .5f  % .5f  % .5f\n",
           r->resid_mean[0], r->resid_mean[1], r->resid_mean[2]);
    printf("    std       % .5f  % .5f  % .5f   (≈ %.3f° tilt)\n",
           sqrt(r->resid_var[0]), sqrt(r->resid_var[1]), sqrt(r->resid_var[2]),
           sqrt(r->resid_var_total) * 180.0 / M_PI);
    printf("    corr time %.3f s", r->resid_tau_s);
    if (r->resid_tau_s > 0.05)
        printf("   <- strongly time-correlated; no white R can be fully correct");
    else if (r->wave_configured)
        printf("   <- white: the wave state has absorbed the correlation");
    printf("\n");

    printf("\n  Innovation consistency at mekf_accel_noise = %g\n",
           r->na_configured);
    printf("    mean NIS            %.2f   (1.0 = covariance consistent)\n",
           r->nis_mean);
    printf("    d² over the cap     %.2f%%\n",  100.0 * r->d2_frac_over_cap);
    printf("    d² over reject 25γ  %.3f%%\n",  100.0 * r->d2_frac_over_rej);
    printf("    d² over old 9γ      %.3f%%   (the pre-1.7 gate, for reference)\n",
           100.0 * r->d2_frac_over_9g);

    printf("\n  Wave-acceleration state (Gauss-Markov)\n");
    if (r->wave_configured)
        printf("    configured          mekf_wave_accel = %.2f m/s^2, "
               "tau = %.2f s\n", r->wave_sigma_cfg, r->wave_tau_cfg);
    else
        printf("    configured          DISABLED\n");
    printf("    seaway measured     sigma ~ %.2f m/s^2, tau ~ %.2f s\n"
           "                        (from a replay with the state off: NIS %.1f,\n"
           "                         residual std %.5f)\n",
           r->wave_sigma_suggest, r->wave_tau_suggest, r->nis_unmodelled,
           sqrt(r->resid_var_unmodelled));
    if (r->wave_sigma_suggest > 0.0)
        printf("    suggested           mekf_wave_accel = %.1f, "
               "mekf_wave_accel_tau_s = %.1f\n"
               "                        (round both UP; too small a sigma is the\n"
               "                         failure mode that hurts, and tau is a\n"
               "                         lower bound from a lag-1 estimator)\n",
               r->wave_sigma_suggest, r->wave_tau_suggest);

    printf("\n  Suggested mekf_accel_noise\n");
    printf("    from raw residual   %.4f\n", r->na_raw);
    if (r->na_consistent_ok)
        printf("    for mean NIS = 1    %.4f\n", r->na_consistent);
    else
        printf("    for mean NIS = 1    did not converge (nearest %.4f)\n",
               r->na_consistent);
    printf("    currently configured %.4f\n", r->na_configured);
    printf("      NOTE: this bisection is conditional on the wave state above.\n"
           "      With it enabled, mekf_accel_noise should describe SENSOR noise\n"
           "      only - the seaway is the wave state's job - so a suggestion far\n"
           "      above the datasheet value means the wave state is under-sized,\n"
           "      not that this one is.\n");

    printf("\n  These are diagnostics, not a calibration: nothing is written.\n");
    if (!r->wave_configured) {
        printf("  A mean NIS of roughly 10-30 is NORMAL for this capture because\n"
               "  the wave state is DISABLED. The gravity residual in a seaway is\n"
               "  wave-orbital and time-correlated, so no single white noise value\n"
               "  can describe it: raising mekf_accel_noise until NIS reaches 1\n"
               "  does make the innovations look consistent, but it weakens the\n"
               "  gravity correction and measurably degrades attitude accuracy.\n"
               "  Enable mekf_wave_accel instead - see ROADMAP 10.5.\n");
    } else {
        printf("  With the wave state enabled, mean NIS SHOULD be near 1: the\n"
               "  correlated part of the residual is modelled rather than left\n"
               "  for a white R to absorb. Readings well above 1 mean the seaway\n"
               "  is rougher than mekf_wave_accel allows for; well below 1 means\n"
               "  calmer. See ROADMAP 10.5 and man 5 imud.conf.\n");
    }

    if (r->wave_configured) {
        if (r->nis_mean > 5.0)
            printf("\n  NOTE: mean NIS is high with the wave state on. Either this\n"
                   "  seaway exceeds the configured sigma (%.2f, measured ~%.2f\n"
                   "  above), or the mount rotation / accel calibration is wrong -\n"
                   "  check for a large residual MEAN, which no noise value fixes.\n",
                   r->wave_sigma_cfg, r->wave_sigma_suggest);
        else if (r->nis_mean < 0.3)
            printf("\n  NOTE: mean NIS is well below 1, so the filter is carrying\n"
                   "  more wave budget than this capture needs. That is safe but\n"
                   "  conservative; it is only worth lowering mekf_wave_accel if\n"
                   "  this capture is representative of your worst conditions.\n");
    } else if (r->nis_mean > 60.0) {
        printf("\n  NOTE: mean NIS is high even for a seaway. Check the mount\n"
               "  rotation and accel calibration first - a wrong mount shows up\n"
               "  here as a large residual MEAN, which no noise value can fix.\n");
    }
}
