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
    if (cap_writer_open(&w, path, (uint32_t)odr, "sim", "sim", "1.7", 0, 0) != 0) {
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
    c.mag_odr_hz        = 104;
    c.mag_yaw_only      = true;
    c.startup_settle_sec = 30.0;
    return c;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static char g_cap[256];

/*
 * The headline cross-validation. test_fusion measures the daemon's own
 * nis_accel EMA at 25.2 for this scenario in yaw-only mode; fit-ra computes
 * the same statistic by an independent path (explicit S and d² recomputed
 * outside the filter, over a file round-trip). They must agree.
 *
 * This cross-check earns its keep: it is what exposed the health-EMA gain
 * bug. The daemon's EMA used a per-update gain sized from the IMU ODR, but
 * the |a| skip band discards most samples in a seaway, so the metric was
 * converging ~20× too slowly and reporting its own seed value of 1.0 rather
 * than the data. fit-ra's uniform mean over the window disagreed, and the
 * disagreement was the bug.
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

    /* The daemon's benchmark reads 25.2 for this scenario. Allow a wide band
     * — the capture starts from a zero bias estimate where the benchmark
     * seeds a near-truth one, and the settle windows differ — but the two
     * must not disagree by an order of magnitude. */
    EXPECT(rep.nis_mean > 5.0 && rep.nis_mean < 90.0,
           "fit-ra mean NIS agrees with the fusion benchmark (~25)");
    EXPECT(rep.nis_mean > 3.0,
           "fit-ra confirms the measurement model is over-confident (ROADMAP 10.1)");

    /* With the 25γ gate almost nothing should be rejected — the property the
     * gate widening was made for. The old 9γ gate must show visibly more. */
    EXPECT(rep.d2_frac_over_rej < 0.02,
           "almost nothing exceeds the shipped 25-gamma reject gate");
    EXPECT(rep.d2_frac_over_9g >= rep.d2_frac_over_rej,
           "the old 9-gamma gate was at least as aggressive");

    end(fb);
}

/*
 * The residual in a seaway is wave-driven and therefore strongly
 * time-correlated. This is the measurement that justifies NOT trying to fix
 * covariance consistency with a bigger scalar R (ROADMAP §10.1/§10.5): if
 * the residual were white, a scalar R would be the right tool.
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

    EXPECT(rep.resid_tau_s > 0.01,
           "residual is measurably time-correlated, not white");
    EXPECT(rep.na_raw > rep.na_configured,
           "raw-residual estimate exceeds the datasheet-derived default");
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

int main(void)
{
    printf("=== imud fit-ra tests ===\n");

    snprintf(g_cap, sizeof g_cap, "/tmp/imud_fitra_%d.imucap", (int)getpid());
    write_wave_capture(g_cap, 833, 90);

    test_fitra_matches_bench_nis();
    test_fitra_detects_time_correlation();
    test_fitra_rejects_bad_input();

    remove(g_cap);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
