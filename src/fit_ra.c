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
 * filter's private tuning, and fit-ra reports against whatever the filter
 * actually uses. */
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
} fitra_acc_t;

/* 3×3 inverse (Cramer); false if singular. */
static bool m33_inverse(const float M[3][3], float Mi[3][3])
{
    float c00 =  M[1][1]*M[2][2] - M[1][2]*M[2][1];
    float c01 = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]);
    float c02 =  M[1][0]*M[2][1] - M[1][1]*M[2][0];
    float det =  M[0][0]*c00 + M[0][1]*c01 + M[0][2]*c02;
    if (fabsf(det) < 1e-20f) return false;
    float id = 1.0f / det;
    Mi[0][0] = c00*id; Mi[1][0] = c01*id; Mi[2][0] = c02*id;
    Mi[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) * id;
    Mi[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * id;
    Mi[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) * id;
    Mi[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * id;
    Mi[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) * id;
    Mi[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * id;
    return true;
}

/*
 * Reproduce the pre-update half of mekf_update_accel: the |a| skip band, the
 * normalised gravity direction z, the predicted direction h, and d².
 * Returns false when the sample would be skipped.
 */
static bool accel_probe(const mekf_t *f, const imu_sample_t *s,
                        float innov[3], float *d2_out)
{
    float acc[3] = { s->accel[0], s->accel[1], s->accel[2] };
    if (f->speed_mps > 0.1f) {
        float wy = s->gyro[1] - f->bias[1];
        float wz = s->gyro[2] - f->bias[2];
        acc[1] -= wz * f->speed_mps;
        acc[2] += wy * f->speed_mps;
    }
    float an = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (an < 1e-6f) return false;
    float ag = an / G_MS2;
    if (ag < f->accel_skip_lo || ag > f->accel_skip_hi) return false;

    float z[3] = { -acc[0]/an, -acc[1]/an, -acc[2]/an };

    /* h = third row of R(q) — mirror of fusion.c q_to_R. */
    float w=f->q[0], x=f->q[1], y=f->q[2], zq=f->q[3];
    float h[3] = { 2*(x*zq - w*y), 2*(y*zq + w*x), 1 - 2*(x*x + y*y) };

    for (int i = 0; i < 3; i++) innov[i] = z[i] - h[i];

    /* S = H·P·Hᵀ + R_eff·I with H = [h×]. */
    const float H[3][6] = {
        {     0, -h[2],  h[1], 0, 0, 0 },
        {  h[2],     0, -h[0], 0, 0, 0 },
        { -h[1],  h[0],     0, 0, 0, 0 },
    };
    float PHt[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float acc2 = 0;
            for (int k = 0; k < 6; k++) acc2 += f->P[i][k] * H[j][k];
            PHt[i][j] = acc2;
        }
    float R_eff = f->Ra * f->Ra_scale;
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float acc2 = (i == j) ? R_eff : 0.0f;
            for (int k = 0; k < 6; k++) acc2 += H[i][k] * PHt[k][j];
            S[i][j] = acc2;
        }
    float Si[3][3];
    if (!m33_inverse(S, Si)) return false;

    double d2 = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            d2 += (double)innov[i] * Si[i][j] * innov[j];
    *d2_out = (float)d2;
    return true;
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

    float odr = (float)(r.hdr.imu_odr_hz ? r.hdr.imu_odr_hz : cfg.imu_odr_hz);
    if (odr <= 0.0f) odr = 833.0f;

    mekf_t f;
    float bias0[3] = {0, 0, 0};
    mekf_init(&f, &cfg, odr, bias0);

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
            /* Captures are pre-mount and pre-cal: reproduce imu.c exactly. */
            apply_mount_rot_if_set(&cfg, s.accel);
            apply_mount_rot_if_set(&cfg, s.gyro);
            memcpy(s.accel_raw, s.accel, sizeof s.accel_raw);
            apply_imu_cal(cal, &s);

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
                if (accel_probe(&f, &s, innov, &d2)) {
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
                    }
                    for (int i = 0; i < 3; i++) a->prev_dev[i] = innov[i];
                    a->have_prev = true;
                } else {
                    (*n_skip)++;
                }
            }
            mekf_update_accel(&f, &s);

        } else if (rec.type == CAP_REC_MAG) {
            mag_sample_t m = rec.mag;
            apply_mount_rot_if_set(&cfg, m.field);
            memcpy(m.field_raw, m.field, sizeof m.field_raw);
            apply_mag_cal(cal, &m);
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

    /* Lag-1 autocorrelation → exponential correlation time τ = −dt/ln(ρ).
     * White noise gives ρ ≈ 0 and τ ≈ 0; wave contamination gives ρ → 1. */
    if (a.ac_den > 0.0) {
        double rho = a.ac_num / a.ac_den;
        if (rho > 1e-6 && rho < 0.999999)
            out->resid_tau_s = -a.dt_mean / log(rho);
    }

    /*
     * na_raw: the noise density whose per-sample variance equals the measured
     * residual variance. Ra = (Na/g)²·odr, so Na = g·√(Ra/odr).
     */
    double odr = (a.dt_mean > 0.0) ? 1.0 / a.dt_mean : 833.0;
    out->na_raw = G_MS2 * sqrt(out->resid_var_total / odr);

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
        printf("   ← strongly time-correlated; no white R can be fully correct");
    printf("\n");

    printf("\n  Innovation consistency at mekf_accel_noise = %g\n",
           r->na_configured);
    printf("    mean NIS            %.2f   (1.0 = covariance consistent)\n",
           r->nis_mean);
    printf("    d² over the cap     %.2f%%\n",  100.0 * r->d2_frac_over_cap);
    printf("    d² over reject 25γ  %.3f%%\n",  100.0 * r->d2_frac_over_rej);
    printf("    d² over old 9γ      %.3f%%   (the pre-1.7 gate, for reference)\n",
           100.0 * r->d2_frac_over_9g);

    printf("\n  Suggested mekf_accel_noise\n");
    printf("    from raw residual   %.4f\n", r->na_raw);
    if (r->na_consistent_ok)
        printf("    for mean NIS = 1    %.4f\n", r->na_consistent);
    else
        printf("    for mean NIS = 1    did not converge (nearest %.4f)\n",
               r->na_consistent);
    printf("    currently configured %.4f\n", r->na_configured);
    printf("\n  These are diagnostics, not a calibration: nothing is written.\n");
    printf("  A mean NIS of roughly 10-30 is NORMAL in a seaway and is not by\n"
           "  itself a reason to change anything. The gravity residual there is\n"
           "  wave-orbital and time-correlated, so no single white noise value\n"
           "  can describe it: raising mekf_accel_noise until NIS reaches 1\n"
           "  does make the innovations look consistent, but it weakens the\n"
           "  gravity correction and measurably degrades attitude accuracy.\n"
           "  See ROADMAP 10.1 and man 5 imud.conf before changing the value.\n");
    if (r->nis_mean < 3.0)
        printf("\n  NOTE: mean NIS is low, so this capture is calmer than the\n"
               "  tuning assumes; it is not evidence for lowering the setting.\n");
    else if (r->nis_mean > 60.0)
        printf("\n  NOTE: mean NIS is high even for a seaway. Check the mount\n"
               "  rotation and accel calibration first - a wrong mount shows up\n"
               "  here as a large residual MEAN, which no noise value can fix.\n");
}
