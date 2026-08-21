/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_fit_ra.c — cross-validation of `imud-cal fit-ra` against the wave
 * scenario the fusion benchmark uses.
 *
 * The point is to tie the FIELD instrument to the BENCH instrument: fit-ra
 * replays a real capture and reports mean NIS, while test_fusion runs the
 * same physics in-process and reports the daemon's own nis_accel EMA. If the
 * two disagree, one of them is lying, and an operator would be tuning against
 * a broken meter. So this test synthesises a capture from the same closed
 * form the benchmark uses and asserts that fit-ra's numbers land where the
 * benchmark says they should.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "capture.h"
#include "fit_ra.h"
#include "cal.h"
#include "config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define G 9.80665f

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *n) { printf("%-52s", n); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Scenario synthesis (mirrors test_fusion.c run_wave_scenario) ─────────── */

static uint32_t rng = 0x1234ABCDu;
static float frand(void)
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return ((float)(int32_t)rng) * (1.0f / 2147483648.0f);
}
static float fnoise(float sigma) { return frand() * sigma * 1.732f; }

static void euler_to_q(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);
    q[0] = cy*cp*cr + sy*sp*sr;
    q[1] = cy*cp*sr - sy*sp*cr;
    q[2] = cy*sp*cr + sy*cp*sr;
    q[3] = sy*cp*cr - cy*sp*sr;
}

static void q_to_R(const float q[4], float R[3][3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z);   R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x);   R[2][2]=1-2*(x*x+y*y);
}

/*
 * Write a rough-water capture: the benchmark's wave scenario, at the SAME
 * 833 Hz / ~104 Hz rates the fusion benchmark uses.
 *
 * The rate is not a free parameter here. Ra = (Na/g)²·odr, so dropping the
 * IMU rate shrinks Ra while simultaneously cutting the number of gravity
 * corrections — a 200 Hz version of this scenario drives the filter into the
 * starvation regime and diverges, which would make the cross-check compare
 * two different filters rather than two instruments.
 */
static void write_wave_capture(const char *path, int odr, int dur_s)
{
    const float dt  = 1.0f / (float)odr;
    const float d2r = (float)(M_PI / 180.0);
    const float Ar = 15.0f*d2r, wr = 2.0f*(float)M_PI*0.20f;
    const float Ap =  8.0f*d2r, wp = 2.0f*(float)M_PI*0.14f;
    const float yaw_true = 60.0f * d2r;
    const float m_ned[3] = { 0.2206f, 0.0f, 0.4149f };   /* Gauss */
    const float bias_true[3] = { 0.0020f, -0.0010f, 0.0015f };

    cap_writer_t w;
    if (cap_writer_open(&w, path, (uint32_t)odr, (uint32_t)(odr * 1000), "sim", "sim", "1.7", 0, 0) != 0) {
        fprintf(stderr, "cannot write %s\n", path);
        exit(1);
    }

    const int mag_div = 8;          /* 833/8 ≈ 104 Hz, as in the benchmark */
    rng = 0x1234ABCDu;

    for (int i = 0; i < odr * dur_s; i++) {
        float t = (float)i * dt;
        float roll  = Ar * sinf(wr * t);
        float pitch = Ap * sinf(wp * t + 1.0f);
        float rolld = Ar * wr * cosf(wr * t);
        float pitchd= Ap * wp * cosf(wp * t + 1.0f);

        float q_true[4]; euler_to_q(roll, pitch, yaw_true, q_true);
        float Rt[3][3];  q_to_R(q_true, Rt);

        float w_body[3] = { rolld, pitchd*cosf(roll), -pitchd*sinf(roll) };

        /* Wave-orbital linear acceleration in NED (m/s²) */
        float a_lin[3] = {
            1.2f * sinf(wr * t + 0.5f),
            1.2f * cosf(0.8f * wr * t),
            1.0f * sinf(wr * t),
        };
        float f_ned[3] = { a_lin[0], a_lin[1], a_lin[2] - G };

        imu_sample_t s;
        memset(&s, 0, sizeof s);
        for (int k = 0; k < 3; k++) {
            s.accel[k] = Rt[0][k]*f_ned[0] + Rt[1][k]*f_ned[1] + Rt[2][k]*f_ned[2]
                       + fnoise(0.03f);
            s.gyro[k]  = w_body[k] + bias_true[k] + fnoise(0.002f);
        }
        s.temp_c  = 25.0f;
        s.seq     = (uint32_t)i;
        /* chip_ts advances at the ISM330's 25 µs/tick, which is what
         * fit_ra.c assumes when it reconstructs dt. */
        s.chip_ts = (uint32_t)llround((double)t / 25e-6);
        cap_writer_imu(&w, &s, (uint64_t)((double)t * 1e9));

        if (i % mag_div == 0) {
            mag_sample_t m;
            memset(&m, 0, sizeof m);
            for (int k = 0; k < 3; k++)
                m.field[k] = (Rt[0][k]*m_ned[0] + Rt[1][k]*m_ned[1]
                              + Rt[2][k]*m_ned[2]) * 100.0f + fnoise(0.3f);
            m.valid   = true;
            m.wall_ns = (uint64_t)((double)t * 1e9);
            cap_writer_mag(&w, &m, (uint64_t)((double)t * 1e9));
        }
    }
    cap_writer_close(&w);
}

/*
 * A pass-through mag calibration. Not cosmetic: the replay mirrors the
 * daemon, which marks mag samples invalid when cal.json has no mag section,
 * and without heading updates yaw is unobservable and P diverges.
 */
static imud_cal_t identity_cal(void)
{
    imud_cal_t c;
    memset(&c, 0, sizeof c);
    for (int i = 0; i < 3; i++) c.mag_soft_iron[i][i] = 1.0f;
    c.has_mag = true;
    return c;
}

static imud_config_t bench_cfg(void)
{
    imud_config_t c;
    memset(&c, 0, sizeof c);
    c.mekf_gyro_noise   = 0.007;
    c.mekf_gyro_bias    = 0.00015;
    c.mekf_accel_noise  = 0.0022;
    c.mekf_mag_noise    = 0.0004;
    c.accel_skip_thresh = 0.05;
    c.mag_reject_gauss  = 0.05;
    c.mag_odr_mhz        = 104;
    c.imu_odr_mhz        = 833;
    c.mag_yaw_only      = true;
    c.startup_settle_sec = 30.0;
    /* Shipped Gauss–Markov wave-state defaults (config.c). fit-ra reports on
     * the filter the daemon actually runs, so the replay has to be configured
     * like the daemon — with these at 0 it measures a filter nobody ships. */
    c.mekf_wave_accel       = 0.8;
    c.mekf_wave_accel_tau_s = 0.5;
    return c;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static char g_cap[256];

/*
 * The headline cross-validation: fit-ra replays the same seaway the fusion
 * benchmark drives, over a file round-trip, and must reach the same verdict
 * about the measurement model.
 *
 * This cross-check earns its keep: it is what exposed the health-EMA gain
 * bug. The daemon's EMA used a per-update gain sized from the IMU ODR, but
 * the |a| skip band discards most samples in a seaway, so the metric was
 * converging ~20× too slowly and reporting its own seed value of 1.0 rather
 * than the data. fit-ra's uniform mean over the window disagreed, and the
 * disagreement was the bug.
 *
 * The verdict itself changed with ROADMAP §10.5. It used to be "NIS ≈ 25, the
 * model is over-confident"; with the Gauss–Markov wave state carrying the
 * correlated part of the residual it is "NIS ≈ 1 — and here is the ≈70 it
 * would have been without the state", which fit-ra now measures in a second
 * pass so the two can be asserted against each other.
 */
static void test_fitra_matches_bench_nis(void)
{
    begin("test_fitra_matches_bench_nis");
    int fb = g_fail;

    imud_config_t cfg = bench_cfg();
    imud_cal_t cal = identity_cal();

    fitra_report_t rep;
    char err[256] = {0};
    int rc = fitra_run(g_cap, &cfg, &cal, 30.0, &rep, err, sizeof err);
    EXPECT(rc == 0, err[0] ? err : "fitra_run succeeds");
    if (rc != 0) { end(fb); return; }

    EXPECT(rep.n_accel_upd > 1000, "enough accel updates were analysed");
    EXPECT(rep.duration_s > 30.0,  "measured window spans the capture");

    /*
     * With the shipped wave state the model is no longer over-confident. A
     * one-sided bound only: this synthetic capture is calmer than the shipped
     * σ budgets for (it reads ≈0.14), and being conservative is not a fault —
     * but reading HIGH would mean the state is not doing its job.
     */
    EXPECT(rep.nis_mean < 3.0,
           "fit-ra: with the wave state on, the model is not over-confident");

    /*
     * ...and the same capture through the same code with the state off is the
     * control. This is the measurement that says the wave state — not a lucky
     * capture — is what brought NIS down.
     */
    EXPECT(rep.nis_unmodelled > 10.0,
           "fit-ra: without the wave state the same seaway is wildly inconsistent");
    EXPECT(rep.nis_unmodelled > 20.0 * rep.nis_mean,
           "fit-ra: the wave state accounts for the difference");

    /* With the 25γ gate almost nothing should be rejected — the property the
     * gate widening was made for. The old 9γ gate must show visibly more. */
    EXPECT(rep.d2_frac_over_rej < 0.02,
           "almost nothing exceeds the shipped 25-gamma reject gate");
    EXPECT(rep.d2_frac_over_9g >= rep.d2_frac_over_rej,
           "the old 9-gamma gate was at least as aggressive");

    end(fb);
}

/*
 * fit-ra's σ/τ suggestions are what an operator tunes from, so they have to
 * recover a disturbance whose truth is known. The synthesised capture drives
 * the same 1.2/1.2/1.0 m/s² wave-orbital tone the fusion benchmark does —
 * per-axis RMS 0.80 m/s² — and fit-ra sees it only through a file, a replayed
 * filter, and a 67%-decimating skip band.
 */
static void test_fitra_suggests_wave_knobs(void)
{
    begin("test_fitra_suggests_wave_knobs");
    int fb = g_fail;

    imud_config_t cfg = bench_cfg();
    imud_cal_t cal = identity_cal();
    fitra_report_t rep; char err[256] = {0};

    if (fitra_run(g_cap, &cfg, &cal, 30.0, &rep, err, sizeof err) != 0) {
        EXPECT(0, err); end(fb); return;
    }

    EXPECT(rep.wave_configured, "the replayed config had the wave state on");
    EXPECT(rep.wave_sigma_suggest > 0.4 && rep.wave_sigma_suggest < 1.4,
           "suggested sigma recovers the capture's true 0.80 m/s^2");
    EXPECT(rep.wave_tau_suggest > 0.2 && rep.wave_tau_suggest < 6.0,
           "suggested tau is a plausible seaway correlation time");

    /*
     * The τ estimator divides by the mean gap between ACCEPTED samples, not by
     * the IMU period. With 67% of samples discarded by the |a| band those
     * differ by ~3×, and using the period reported τ that much too small — the
     * bias behind the "τ ≈ 0.3–0.9 s" figure in the older notes. A τ below the
     * IMU period would mean the correction was dropped.
     */
    EXPECT(rep.wave_tau_suggest > 0.05,
           "tau is corrected for the skip band, not the raw IMU period");

    end(fb);
}

/*
 * The seaway residual is wave-driven and therefore strongly time-correlated.
 * That measurement is what justified NOT trying to fix covariance consistency
 * with a bigger scalar R (ROADMAP §10.1) and what motivated the Gauss–Markov
 * state (§10.5) — and it is now ALSO the pass/fail criterion for that state,
 * from the two ends of the same replay:
 *
 *   with the state off  → the residual is correlated (that is the problem)
 *   with the state on   → the residual is white     (that is the fix)
 *
 * A whitened residual is the strongest available statement that the state is
 * modelling the disturbance rather than merely inflating the covariance until
 * the innovations stop complaining.
 */
static void test_fitra_detects_time_correlation(void)
{
    begin("test_fitra_detects_time_correlation");
    int fb = g_fail;

    imud_config_t cfg = bench_cfg();
    imud_cal_t cal = identity_cal();
    fitra_report_t rep; char err[256] = {0};

    if (fitra_run(g_cap, &cfg, &cal, 30.0, &rep, err, sizeof err) != 0) {
        EXPECT(0, err); end(fb); return;
    }

    EXPECT(rep.wave_tau_suggest > 0.01,
           "unmodelled residual is measurably time-correlated, not white");
    EXPECT(rep.resid_tau_s < 0.5 * rep.wave_tau_suggest,
           "the wave state whitens the residual it is given");
    EXPECT(rep.resid_var_unmodelled > 4.0 * rep.resid_var_total,
           "and shrinks it: the disturbance is explained, not absorbed into tilt");

    /* Raw-residual Na is a white-noise reading of the UNMODELLED residual, so
     * it must still exceed the datasheet value — the seaway is really there. */
    double na_raw_unmodelled = 9.80665 * sqrt(rep.resid_var_unmodelled /
                                              (double)cfg.imu_odr_mhz);
    EXPECT(na_raw_unmodelled > rep.na_configured,
           "unmodelled residual still exceeds the datasheet-derived default");
    end(fb);
}

/* Error paths: a missing file and a non-capture must both be reported, not
 * silently produce a plausible-looking report. */
static void test_fitra_rejects_bad_input(void)
{
    begin("test_fitra_rejects_bad_input");
    int fb = g_fail;

    imud_config_t cfg = bench_cfg();
    imud_cal_t cal = identity_cal();
    fitra_report_t rep; char err[256];

    err[0] = 0;
    EXPECT(fitra_run("/tmp/imud_no_such_capture_xyz.imucap", &cfg, &cal,
                     5.0, &rep, err, sizeof err) == -1,
           "missing capture reports an error");
    EXPECT(err[0] != 0, "missing capture sets an error message");

    char junk[256];
    snprintf(junk, sizeof junk, "/tmp/imud_fitra_junk_%d.bin", (int)getpid());
    FILE *f = fopen(junk, "w");
    if (f) { fputs("this is not a capture file at all", f); fclose(f); }
    err[0] = 0;
    EXPECT(fitra_run(junk, &cfg, &cal, 5.0, &rep, err, sizeof err) == -1,
           "non-capture file reports an error");
    remove(junk);

    end(fb);
}

/* ── The report itself ───────────────────────────────────────────────────── */

/*
 * fitra_print is the entire user-visible output of `imud-cal fit-ra`, and its
 * advice branches on the report: whether the wave state was configured,
 * whether the residual is time-correlated, whether the NIS bisection
 * converged.  Picking the wrong arm tells an operator to tune the wrong knob,
 * so both arms of each branch are exercised here against a captured stdout.
 */
static char *capture_print(const fitra_report_t *r, const char *path)
{
    static char buf[16384];
    char tmp[128];
    snprintf(tmp, sizeof tmp, "/tmp/imud_fitra_out_%d.txt", (int)getpid());

    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    FILE *f = freopen(tmp, "w", stdout);
    if (f) fitra_print(r, path);
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);

    buf[0] = '\0';
    FILE *in = fopen(tmp, "r");
    if (in) {
        size_t n = fread(buf, 1, sizeof buf - 1, in);
        buf[n] = '\0';
        fclose(in);
    }
    remove(tmp);
    return buf;
}

static void test_fitra_print_wave_enabled(void)
{
    begin("test_fitra_print_wave_enabled");
    int fb = g_fail;

    fitra_report_t r;
    memset(&r, 0, sizeof r);
    r.duration_s = 90.0; r.n_imu = 74970; r.n_mag = 9000;
    r.n_accel_upd = 60000; r.n_skipped = 15000;
    r.resid_var[0] = r.resid_var[1] = r.resid_var[2] = 1e-4;
    r.resid_var_total = 1e-4;
    r.nis_mean = 1.05;
    r.resid_tau_s = 0.01;              /* white → the "absorbed" arm */
    r.wave_configured = true;
    r.wave_sigma_cfg = 0.8; r.wave_tau_cfg = 0.5;
    r.wave_sigma_suggest = 0.9; r.wave_tau_suggest = 0.6;
    r.na_raw = 0.0022; r.na_consistent = 0.0025; r.na_consistent_ok = true;
    r.na_configured = 0.0022;

    const char *out = capture_print(&r, "/tmp/session.imucap");
    EXPECT(strstr(out, "/tmp/session.imucap") != NULL, "names the capture");
    EXPECT(strstr(out, "wave state has absorbed") != NULL,
           "white residual → the wave state absorbed it");
    EXPECT(strstr(out, "mekf_wave_accel = 0.80") != NULL ||
           strstr(out, "mekf_wave_accel = 0.8") != NULL,
           "reports the configured wave sigma");
    EXPECT(strstr(out, "suggested") != NULL, "offers a suggestion");
    EXPECT(strstr(out, "SHOULD be near 1") != NULL,
           "wave-on guidance, not the wave-off guidance");
    EXPECT(strstr(out, "did not converge") == NULL,
           "no bisection warning when it converged");
    EXPECT(strstr(out, "nothing is written") != NULL,
           "states that it writes no calibration");
    end(fb);
}

static void test_fitra_print_wave_disabled(void)
{
    begin("test_fitra_print_wave_disabled");
    int fb = g_fail;

    fitra_report_t r;
    memset(&r, 0, sizeof r);
    r.duration_s = 90.0;
    r.n_accel_upd = 0; r.n_skipped = 0;      /* exercises the /0 guard */
    r.resid_var_total = 4e-3;
    r.nis_mean = 19.3;
    r.resid_tau_s = 0.4;                     /* correlated → the warning arm */
    r.wave_configured = false;
    r.wave_sigma_suggest = 0.0;              /* no suggestion available */
    r.na_consistent_ok = false;              /* bisection failed arm */
    r.na_consistent = 0.031;

    const char *out = capture_print(&r, "rough.imucap");
    EXPECT(strstr(out, "strongly time-correlated") != NULL,
           "correlated residual is called out");
    EXPECT(strstr(out, "DISABLED") != NULL, "reports the wave state as disabled");
    EXPECT(strstr(out, "did not converge") != NULL,
           "reports a failed NIS bisection rather than a bogus number");
    EXPECT(strstr(out, "10-30 is NORMAL") != NULL,
           "wave-off guidance explains the high NIS");
    EXPECT(strstr(out, "ROADMAP 10.5") != NULL, "points at the roadmap item");
    EXPECT(strstr(out, "suggested           mekf_wave_accel") == NULL,
           "no suggestion when sigma_suggest is 0");
    /* The n_accel_upd + n_skipped == 0 guard must not produce nan/inf. */
    EXPECT(strstr(out, "nan") == NULL && strstr(out, "inf") == NULL,
           "zero-update capture prints no nan/inf");
    end(fb);
}

int main(void)
{
    printf("=== imud fit-ra tests ===\n");

    snprintf(g_cap, sizeof g_cap, "/tmp/imud_fitra_%d.imucap", (int)getpid());
    write_wave_capture(g_cap, 833, 90);

    test_fitra_matches_bench_nis();
    test_fitra_suggests_wave_knobs();
    test_fitra_detects_time_correlation();
    test_fitra_rejects_bad_input();
    test_fitra_print_wave_enabled();
    test_fitra_print_wave_disabled();

    remove(g_cap);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
