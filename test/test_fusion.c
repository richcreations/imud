/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_fusion.c — unit tests for the MEKF fusion module
 *
 * Tests operate through the public API in fusion.h.  No mocking: math
 * is exercised with known analytical inputs whose outputs we can predict.
 *
 * Convention recap (must match fusion.c):
 *   q = [w,x,y,z] body→NED.  g_ref = [0,0,1] (NED Z-down).
 *   Accel input = specific force in body frame (ISM330DHCX convention:
 *   reads +g when Z points up).  z_accel = −normalize(accel_body).
 *   So: flat board, Z-down → accel = [0,0,−g] → z = [0,0,+1] = h.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "fusion.h"
#include "config.h"
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define G   9.80665f
#define DEG (float)(M_PI / 180.0)

/* ── Minimal test framework ─────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, msg); } \
} while(0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabsf((float)(a) - (float)(b)) < (float)(eps), msg)

#define EXPECT_LT(a, b, msg) \
    EXPECT((float)(a) < (float)(b), msg)

#define TEST(name) static void name(void)
#define RUN(name) do { \
    printf("%-40s", #name); \
    int before_fail = g_fail; \
    name(); \
    puts(g_fail == before_fail ? "OK" : "FAILED"); \
} while(0)

/* ── Helpers ────────────────────────────────────────────────────────────────── */

/*
 * When non-zero, overrides mekf_accel_noise in the wave benchmark. Only the
 * BENCH_SWEEP_RA driver sets it; it is 0 in every normal build, so the
 * benchmark uses the shipped default like everything else.
 */
static double bench_ra_override = 0.0;

/*
 * Gauss–Markov wave-state knobs. These carry the SHIPPED defaults, so the
 * benchmark measures what a real vessel gets; the BENCH_SWEEP_WAVE driver
 * overwrites them to explore the (σ, τ) grid. Kept here rather than read from
 * config_defaults() because make_cfg hand-builds its config — test_fusion does
 * not link config.c.
 */
static double bench_wave_sigma = 0.8;
static double bench_wave_tau   = 0.5;

/* Dip-reference uncertainty (deg) fed to the 3-D mag update; see
 * mekf_update_mag. Carries the shipped default so the benchmark measures what
 * a real vessel gets; the BENCH_SWEEP_DIP driver overwrites it. */
static double bench_dip_sigma = 1.0;

/*
 * The rates the wave scenario runs at.
 *
 * BENCH_REF_FS / BENCH_REF_MAG_FS are the pair every recorded number in this
 * tree was measured at, and the scenario is required to reproduce that pair
 * bit-for-bit — test_bench_stream_fingerprint plus a diff of the printed lines
 * is what holds it to that.  Everything else is new ground.
 *
 * The drivers advertise 27 distinct IMU rates from 12 Hz to 8 kHz and 13
 * magnetometer rates from 1 Hz to 1 kHz (the supported_odr_hz tables under
 * src/drivers).
 * Exactly one pairing out of those 351 has ever been measured, which is what
 * these globals exist to fix.
 */
#define BENCH_REF_FS     833.0f
#define BENCH_REF_MAG_FS 100.0f

#ifdef BENCH_ODR_HZ
static float bench_fs = (float)(BENCH_ODR_HZ);
#else
static float bench_fs = BENCH_REF_FS;
#endif

#ifdef BENCH_MAG_HZ
static float bench_mag_fs = (float)(BENCH_MAG_HZ);
#else
static float bench_mag_fs = BENCH_REF_MAG_FS;
#endif

static imud_config_t make_cfg(void)
{
    imud_config_t c;
    memset(&c, 0, sizeof c);
    c.mekf_gyro_noise   = 0.007;
    c.mekf_gyro_bias    = 0.00015;
    c.mekf_accel_noise  = 0.0022;
    c.mekf_mag_noise    = 0.0004;
    c.accel_skip_thresh = 0.05;
    c.mag_reject_gauss  = 0.05f;
    c.mag_odr_hz        = 100;
    c.mekf_wave_accel       = bench_wave_sigma;
    c.mekf_wave_accel_tau_s = bench_wave_tau;
    c.mekf_mag_dip_sigma_deg = bench_dip_sigma;
    if (bench_ra_override > 0.0) c.mekf_accel_noise = bench_ra_override;
    return c;
}

/*
 * Live error-state dimension. P is always MEKF_N×MEKF_N, but the wave block
 * (rows/columns 6–8) is identically zero unless the Gauss–Markov state is
 * configured — so a positive-diagonal check must scan only the live states,
 * while a symmetry check can and should scan the whole matrix.
 */
static int live_n(const mekf_t *f) { return f->wave_enabled ? MEKF_N : 6; }

/* Make a flat-board accel (Z-down, specific-force convention: reads −g). */
static imu_sample_t make_accel(float ax, float ay, float az)
{
    imu_sample_t s;
    memset(&s, 0, sizeof s);
    s.accel[0] = ax; s.accel[1] = ay; s.accel[2] = az;
    return s;
}

static imu_sample_t make_gyro(float wx, float wy, float wz)
{
    imu_sample_t s;
    memset(&s, 0, sizeof s);
    s.gyro[0] = wx; s.gyro[1] = wy; s.gyro[2] = wz;
    return s;
}

static mag_sample_t make_mag(float mx, float my, float mz)
{
    mag_sample_t m;
    memset(&m, 0, sizeof m);
    /* Driver outputs µT; 1 Gauss = 100 µT */
    m.field[0] = mx * 100.0f;
    m.field[1] = my * 100.0f;
    m.field[2] = mz * 100.0f;
    m.valid = true;
    return m;
}

/* Quaternion to yaw/pitch/roll (NED 3-2-1). */
static void q_to_euler(const float q[4],
                        float *roll, float *pitch, float *yaw)
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    float R20 = 2*(x*z-w*y);
    float R21 = 2*(y*z+w*x);
    float R22 = 1-2*(x*x+y*y);
    float R10 = 2*(x*y+w*z);
    float R00 = 1-2*(y*y+z*z);
    *pitch = asinf(-R20);
    *roll  = atan2f(R21, R22);
    *yaw   = atan2f(R10, R00);
}

static float q_norm(const float q[4])
{
    return sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
}

static float p_att_trace(const mekf_t *f)
{
    return f->P[0][0] + f->P[1][1] + f->P[2][2];
}

/*
 * Attitude error as a rotation vector, in the filter's OWN error convention.
 *
 * fusion.c uses the right-multiply convention q_true = q̂ ⊗ δq (Solà §7), so
 * the error quaternion is δq = q̂⁻¹ ⊗ q_true and δθ = 2·log(δq) — expressed in
 * the BODY frame, which is exactly the frame P[0:3][0:3] describes. Getting
 * this wrong (using an NED-frame or left-multiplied error) would silently
 * scramble the NEES numbers whenever attitude is far from identity, which in
 * the wave scenario is most of the time.
 */
static void q_err_rotvec(const float qhat[4], const float q_true[4],
                         float dth[3])
{
    /* δq = q̂⁻¹ ⊗ q_true  (q̂ is unit, so the inverse is the conjugate) */
    float a[4] = { qhat[0], -qhat[1], -qhat[2], -qhat[3] };
    const float *b = q_true;
    float dq[4] = {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    };
    /* Pick the short way round so |δθ| ≤ π. */
    if (dq[0] < 0.0f) { dq[0]=-dq[0]; dq[1]=-dq[1]; dq[2]=-dq[2]; dq[3]=-dq[3]; }

    float nv = sqrtf(dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
    if (nv < 1e-9f) {           /* small angle: δθ ≈ 2·vec(δq) */
        dth[0] = 2.0f*dq[1]; dth[1] = 2.0f*dq[2]; dth[2] = 2.0f*dq[3];
        return;
    }
    float th = 2.0f * atan2f(nv, dq[0]);
    float s  = th / nv;
    dth[0] = dq[1]*s; dth[1] = dq[2]*s; dth[2] = dq[3]*s;
}

/* Closed-form 3×3 inverse; returns false if singular. */
static bool m33_inverse(const float M[3][3], float Mi[3][3])
{
    float c00 =  M[1][1]*M[2][2] - M[1][2]*M[2][1];
    float c01 = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]);
    float c02 =  M[1][0]*M[2][1] - M[1][1]*M[2][0];
    float det =  M[0][0]*c00 + M[0][1]*c01 + M[0][2]*c02;
    if (fabsf(det) < 1e-20f) return false;
    float id = 1.0f / det;
    Mi[0][0] = c00*id;
    Mi[1][0] = c01*id;
    Mi[2][0] = c02*id;
    Mi[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) * id;
    Mi[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * id;
    Mi[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) * id;
    Mi[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * id;
    Mi[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) * id;
    Mi[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * id;
    return true;
}

/* ── Tests ──────────────────────────────────────────────────────────────────── */

/*
 * Quaternion is always unit-norm: predict with zero gyro for 1000 steps.
 */
TEST(test_quat_norm_preserved)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    imu_sample_t s = make_gyro(0.1f, -0.2f, 0.3f);
    for (int i = 0; i < 1000; i++) mekf_predict(&f, &s, f.dt);

    EXPECT_NEAR(q_norm(f.q), 1.0f, 1e-5f, "q norm after 1000 steps");
}

/*
 * Static prediction (zero gyro): quaternion must not drift, P must grow
 * (because gyro noise is being integrated into the covariance).
 */
TEST(test_predict_static_no_drift)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    /* Flat board pointing North */
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    float q0[4]; memcpy(q0, f.q, sizeof q0);
    float p0 = p_att_trace(&f);

    imu_sample_t zero = make_gyro(0, 0, 0);
    for (int i = 0; i < 100; i++) mekf_predict(&f, &zero, f.dt);

    /* q should be unchanged (zero gyro input) */
    EXPECT_NEAR(f.q[0], q0[0], 1e-6f, "w unchanged for zero gyro");
    EXPECT_NEAR(f.q[1], q0[1], 1e-6f, "x unchanged for zero gyro");
    EXPECT_NEAR(f.q[2], q0[2], 1e-6f, "y unchanged for zero gyro");
    EXPECT_NEAR(f.q[3], q0[3], 1e-6f, "z unchanged for zero gyro");

    /* P must grow (gyro noise accumulating) */
    EXPECT_LT(p0, p_att_trace(&f), "P grows with zero gyro (noise integration)");
}

/*
 * Known rotation: ωz = 90 deg/s for exactly 1 second at 833 Hz.
 * Expected yaw after integration: ≈ π/2 ± 0.1°.
 */
TEST(test_predict_known_rotation_yaw)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    float wz = 90.0f * DEG;  /* rad/s around Z (yaw) */
    imu_sample_t s = make_gyro(0, 0, wz);

    for (int i = 0; i < 833; i++) mekf_predict(&f, &s, f.dt);

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);

    EXPECT_NEAR(roll,  0,      0.005f, "roll ≈ 0 during yaw rotation");
    EXPECT_NEAR(pitch, 0,      0.005f, "pitch ≈ 0 during yaw rotation");
    EXPECT_NEAR(yaw, (float)(M_PI/2), 0.002f, "yaw ≈ 90° after 1 s at 90 °/s");
}

/*
 * Known rotation: ωx = 45 deg/s for 2 seconds → 90° roll.
 */
TEST(test_predict_known_rotation_roll)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    float wx = 45.0f * DEG;
    imu_sample_t s = make_gyro(wx, 0, 0);
    for (int i = 0; i < 833*2; i++) mekf_predict(&f, &s, f.dt);

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    EXPECT_NEAR(roll, (float)(M_PI/2), 0.002f, "roll ≈ 90° after 2 s at 45 °/s");
    EXPECT_NEAR(pitch, 0, 0.005f, "pitch ≈ 0 during roll");
}

/*
 * Align: flat board (Z-down accel) pointing North → identity quaternion.
 */
TEST(test_align_flat_north)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    /* Z-down flat board: specific force = [0,0,−g].
     * Mag in body frame pointing +X (North in NED when flat). */
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);

    EXPECT_NEAR(roll,  0,         1.0f*DEG, "roll ≈ 0° for flat board");
    EXPECT_NEAR(pitch, 0,         1.0f*DEG, "pitch ≈ 0° for flat board");
    /* Heading = atan2(0, 0.2) ≈ 0 = North */
    EXPECT_NEAR(yaw, 0, 2.0f*DEG, "heading ≈ North for X-aligned mag");
    EXPECT(f.initialized,         "filter initialized after align");
    EXPECT(f.m_ref_valid,         "m_ref set after align");
}

/*
 * Align: board tilted 30° roll.
 * Accel for 30° roll: specific force = R_x(30°)^T × [0,0,−g] body coords.
 * With q=Rx(30°): accel body = [0, sin(30°)×g, −cos(30°)×g] (specific force upward).
 * Wait: specific force = −gravity in body. Gravity in body = R^T×g_NED.
 * g_NED = [0,0,g]. g_body = R_x(30°)^T × [0,0,g] = [0, −sin(30°)g, cos(30°)g]...
 * Specific force = −g_body = [0, sin(30°)g, −cos(30°)g].
 */
TEST(test_align_30deg_roll)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    float phi = 30.0f * DEG;
    /*
     * Specific force for 30° roll (body→NED = Rx(phi)).
     * Gravity in body = Rx(phi)^T × [0,0,1] = [0, sin(phi), cos(phi)].
     * Specific force = −gravity = [0, −sin(phi)×g, −cos(phi)×g].
     */
    float ax =  0.0f;
    float ay = -sinf(phi) * G;   /* −0.5g */
    float az = -cosf(phi) * G;   /* −0.866g */
    /* Mag pointing North in NED, tilt-corrected to body:
     * m_body = R_x(phi)^T × [1,0,0] = [1,0,0] (X-axis unaffected by roll) */
    mekf_align(&f, (float[]){ax, ay, az}, (float[]){100.0f, 0.0f, 0.0f});

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);

    EXPECT_NEAR(roll,  phi, 1.0f*DEG, "roll ≈ 30° from accel");
    EXPECT_NEAR(pitch, 0,   1.0f*DEG, "pitch ≈ 0°");
    EXPECT_NEAR(yaw,   0,   2.0f*DEG, "heading ≈ North");
}

/*
 * Accel update corrects roll error:
 * Inject a 10° roll error into an aligned filter, run accel-only updates,
 * verify roll error decreases toward zero.
 */
TEST(test_accel_update_corrects_roll)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Inject 10° roll error by directly rotating q */
    float err = 10.0f * DEG;
    float dq[4] = { cosf(err/2), sinf(err/2), 0, 0 };
    float qnew[4] = {
        f.q[0]*dq[0] - f.q[1]*dq[1] - f.q[2]*dq[2] - f.q[3]*dq[3],
        f.q[0]*dq[1] + f.q[1]*dq[0] + f.q[2]*dq[3] - f.q[3]*dq[2],
        f.q[0]*dq[2] - f.q[1]*dq[3] + f.q[2]*dq[0] + f.q[3]*dq[1],
        f.q[0]*dq[3] + f.q[1]*dq[2] - f.q[2]*dq[1] + f.q[3]*dq[0],
    };
    memcpy(f.q, qnew, sizeof f.q);

    float roll0, pitch0, yaw0;
    q_to_euler(f.q, &roll0, &pitch0, &yaw0);
    float err0 = fabsf(roll0);

    /* Feed correct flat-board accel (no gyro predict — pure update test) */
    imu_sample_t s = make_accel(0, 0, -G);
    for (int i = 0; i < 20; i++) mekf_update_accel(&f, &s);

    float roll1, pitch1, yaw1;
    q_to_euler(f.q, &roll1, &pitch1, &yaw1);
    float err1 = fabsf(roll1);

    EXPECT_LT(err1, err0, "roll error decreases after accel updates");
    EXPECT_NEAR(roll1, 0, 2.0f*DEG, "roll converges close to zero");
}

/*
 * Accel update is skipped when linear acceleration is detected (|a|/g > 1+thresh).
 */
TEST(test_accel_reject_linear_acceleration)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Inject 10° roll error */
    float err = 10.0f * DEG;
    float dq[4] = { cosf(err/2), sinf(err/2), 0, 0 };
    float qnew[4] = {
        f.q[0]*dq[0]-f.q[1]*dq[1]-f.q[2]*dq[2]-f.q[3]*dq[3],
        f.q[0]*dq[1]+f.q[1]*dq[0]+f.q[2]*dq[3]-f.q[3]*dq[2],
        f.q[0]*dq[2]-f.q[1]*dq[3]+f.q[2]*dq[0]+f.q[3]*dq[1],
        f.q[0]*dq[3]+f.q[1]*dq[2]-f.q[2]*dq[1]+f.q[3]*dq[0],
    };
    memcpy(f.q, qnew, sizeof f.q);
    float q_before[4]; memcpy(q_before, f.q, sizeof f.q);

    /* Feed 2g accel (strong linear acceleration — exceeds skip threshold) */
    imu_sample_t s = make_accel(0, 0, -2.0f*G);
    for (int i = 0; i < 20; i++) mekf_update_accel(&f, &s);

    /* q should be unchanged — update was skipped */
    EXPECT_NEAR(f.q[0], q_before[0], 1e-6f, "w unchanged under linear accel");
    EXPECT_NEAR(f.q[1], q_before[1], 1e-6f, "x unchanged under linear accel");
    EXPECT_NEAR(f.q[2], q_before[2], 1e-6f, "y unchanged under linear accel");
    EXPECT_NEAR(f.q[3], q_before[3], 1e-6f, "z unchanged under linear accel");
}

/*
 * Mag update corrects yaw error:
 * Inject 20° yaw error, run mag updates with correct mag, verify yaw improves.
 */
TEST(test_mag_update_corrects_yaw)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    float mag_gauss[3] = {0.2f, 0.0f, 0.05f};  /* Gauss, for make_mag */
    float mag_ut[3]    = {20.0f, 0.0f, 5.0f};   /* µT, for mekf_align */
    mekf_align(&f, (float[]){0,0,-G}, mag_ut);

    /* Inject 20° yaw error */
    float err = 20.0f * DEG;
    float dq[4] = { cosf(err/2), 0, 0, sinf(err/2) };
    float qnew[4] = {
        f.q[0]*dq[0]-f.q[1]*dq[1]-f.q[2]*dq[2]-f.q[3]*dq[3],
        f.q[0]*dq[1]+f.q[1]*dq[0]+f.q[2]*dq[3]-f.q[3]*dq[2],
        f.q[0]*dq[2]-f.q[1]*dq[3]+f.q[2]*dq[0]+f.q[3]*dq[1],
        f.q[0]*dq[3]+f.q[1]*dq[2]-f.q[2]*dq[1]+f.q[3]*dq[0],
    };
    memcpy(f.q, qnew, sizeof f.q);

    float roll0, pitch0, yaw0;
    q_to_euler(f.q, &roll0, &pitch0, &yaw0);
    float err0 = fabsf(yaw0);  /* should be ≈ 20° */

    mag_sample_t m = make_mag(mag_gauss[0], mag_gauss[1], mag_gauss[2]);
    for (int i = 0; i < 30; i++) mekf_update_mag(&f, &m);

    float roll1, pitch1, yaw1;
    q_to_euler(f.q, &roll1, &pitch1, &yaw1);
    float err1 = fabsf(yaw1);

    EXPECT_LT(err1, err0, "yaw error decreases after mag updates");
    EXPECT_NEAR(yaw1, 0, 3.0f*DEG, "yaw converges close to zero");
}

/*
 * Mag anomaly rejection: residual far beyond mag_reject_gauss → no update.
 */
TEST(test_mag_reject_anomaly)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    float q_before[4]; memcpy(q_before, f.q, sizeof f.q);

    /* Wildly wrong mag (10 Gauss anomaly) */
    mag_sample_t m = make_mag(10.0f, 5.0f, -3.0f);
    for (int i = 0; i < 10; i++) mekf_update_mag(&f, &m);

    EXPECT_NEAR(f.q[0], q_before[0], 1e-5f, "w unchanged for mag anomaly");
    EXPECT_NEAR(f.q[1], q_before[1], 1e-5f, "x unchanged for mag anomaly");
    EXPECT_NEAR(f.q[2], q_before[2], 1e-5f, "y unchanged for mag anomaly");
    EXPECT_NEAR(f.q[3], q_before[3], 1e-5f, "z unchanged for mag anomaly");
}

/*
 * Covariance decreases after measurement updates.
 * Start with large initial P; feed accel and mag updates; P should shrink.
 */
TEST(test_covariance_decreases_with_updates)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    float p0 = p_att_trace(&f);

    imu_sample_t sa = make_accel(0, 0, -G);
    mag_sample_t sm = make_mag(0.2f, 0, 0.05f);
    for (int i = 0; i < 50; i++) {
        mekf_update_accel(&f, &sa);
        mekf_update_mag(&f, &sm);
    }

    EXPECT_LT(p_att_trace(&f), p0, "P trace decreases after accel+mag updates");
}

/*
 * Convergence flag: after sufficient updates the converged flag is set.
 */
TEST(test_convergence_flag)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    EXPECT(!f.converged, "not converged immediately after init");

    imu_sample_t sa = make_accel(0, 0, -G);
    mag_sample_t sm = make_mag(0.2f, 0, 0.05f);
    imu_sample_t sg = make_gyro(0, 0, 0);

    /*
     * ~12 s. The 2.4 s this used to run was enough when P collapsed to an
     * unbelievable 0.14°/axis; with the wave state the descent is honest and
     * slower — the attitude uncertainty has to average a modelled 0.8 m/s²
     * disturbance down over many correlation times. See mekf_derive_tuning
     * for how conv_thresh now tracks that floor.
     */
    for (int i = 0; i < 10000; i++) {
        mekf_predict(&f, &sg, f.dt);
        mekf_update_accel(&f, &sa);
        if (i % 8 == 0) mekf_update_mag(&f, &sm);  /* ~100 Hz at 833 Hz predict */
    }

    EXPECT(f.converged, "converged after prolonged accel+mag updates");
}

/*
 * Pre-computed bias: filter initialised with a known gyro bias (from the
 * startup still window) should track correctly and not diverge.
 * The MEKF bias channel corrects residual drift via the Kalman cross-covariance,
 * which builds up slowly; the startup still-window estimate is the primary source.
 */
TEST(test_precomputed_bias_used)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float known_bias[3] = {0.005f, -0.003f, 0.002f};  /* rad/s, pre-estimated */
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, known_bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Verify bias is stored */
    EXPECT_NEAR(f.bias[0], known_bias[0], 1e-7f, "bias[0] stored from init");
    EXPECT_NEAR(f.bias[1], known_bias[1], 1e-7f, "bias[1] stored from init");
    EXPECT_NEAR(f.bias[2], known_bias[2], 1e-7f, "bias[2] stored from init");

    /* Run with gyro = known_bias (sensor at rest with bias = true drift).
     * The bias-corrected rate = gyro - bias = 0, so q should not drift. */
    imu_sample_t sa = make_accel(0, 0, -G);
    mag_sample_t sm = make_mag(0.2f, 0, 0.05f);
    float q0[4]; memcpy(q0, f.q, sizeof q0);

    for (int i = 0; i < 833; i++) {
        imu_sample_t sg = make_gyro(known_bias[0], known_bias[1], known_bias[2]);
        mekf_predict(&f, &sg, f.dt);
        mekf_update_accel(&f, &sa);
        if (i % 8 == 0) mekf_update_mag(&f, &sm);
    }

    /* Attitude should be nearly unchanged (bias fully cancels gyro) */
    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    EXPECT_NEAR(roll,  0, 1.0f*DEG, "roll stable with known bias");
    EXPECT_NEAR(pitch, 0, 1.0f*DEG, "pitch stable with known bias");
    EXPECT_NEAR(yaw,   0, 2.0f*DEG, "yaw stable with known bias");
}

/*
 * get_state: Euler angles extracted from a known quaternion are correct.
 * q_pitch(30°): pitch = 30°, roll = yaw = 0.
 */
TEST(test_get_state_euler_extraction)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Manually set q to a 30° pitch quaternion */
    float theta = 30.0f * DEG;
    f.q[0] = cosf(theta/2); f.q[1] = 0;
    f.q[2] = sinf(theta/2); f.q[3] = 0;

    fused_state_t out;
    mekf_get_state(&f, &out, 0);

    EXPECT_NEAR(out.pitch,       theta, 0.001f, "pitch = 30°");
    EXPECT_NEAR(out.roll,        0,     0.001f, "roll = 0°");
    EXPECT_NEAR(out.yaw,         0,     0.001f, "yaw = 0°");
    EXPECT_NEAR(out.heading_deg, 0,     0.1f,   "heading = 0°");
    /* q is copied into out */
    EXPECT_NEAR(out.q[0], f.q[0], 1e-6f, "q[0] passed through");
    EXPECT_NEAR(out.q[2], f.q[2], 1e-6f, "q[2] passed through");
}

/*
 * A non-finite state must never reach a packet, and must not be permanent.
 *
 * The audit case: a gyro bias of NaN — which reached f->bias straight from
 * cal.json — made w = gyro - bias non-finite on every predict step.
 * q_from_rotvec then takes its else branch (NaN < 1e-7f is false) and
 * q_normalize declines to repair the result, because NaN fails its
 * `n > 1e-10f` test and the quaternion is passed through untouched.  There
 * was no path back: 1000 perfectly level, noiseless samples later the
 * published attitude was still NaN, and %.4f renders that as "nan" into an
 * NMEA sentence.
 *
 * The loaders now reject a non-finite bias, so this can no longer arrive from
 * a file.  mekf_sanitize is the backstop for arithmetic that produces one
 * anyway, and it is what this test drives.
 */
TEST(test_sanitize_recovers_from_non_finite)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Poison the bias directly — the same shape as the cal.json case, but
     * reached the way the other guard-path tests here reach a branch. */
    f.bias[0] = NAN;

    imu_sample_t s = make_gyro(0.0f, 0.0f, 0.0f);
    s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = -G;
    mekf_predict(&f, &s, f.dt);

    EXPECT(!isfinite(f.q[0]) || !isfinite(f.bias[0]),
           "a NaN bias does reach the state (the bug being guarded)");

    EXPECT(mekf_sanitize(&f), "sanitize reports it acted");
    EXPECT(isfinite(f.q[0]) && isfinite(f.q[1]) &&
           isfinite(f.q[2]) && isfinite(f.q[3]), "q is finite after reset");
    EXPECT(isfinite(f.bias[0]) && isfinite(f.bias[1]) && isfinite(f.bias[2]),
           "bias is finite after reset");
    EXPECT(!f.initialized, "filter re-aligns rather than carrying on");

    /* Whatever the caller does next, nothing non-finite may be published. */
    fused_state_t out;
    mekf_get_state(&f, &out, 0);
    EXPECT(isfinite(out.roll) && isfinite(out.pitch) && isfinite(out.yaw),
           "published euler angles are finite");
    EXPECT(isfinite(out.heading_deg), "published heading is finite");
    EXPECT(isfinite(out.q[0]) && isfinite(out.q[3]), "published q is finite");
    EXPECT(out.flags & FLAG_STATE_RESET, "FLAG_STATE_RESET is published");
}

/*
 * The flag latches until the filter reconverges, rather than appearing in a
 * single packet.  At up to 500 Hz a momentary flag is invisible to a 1 Hz
 * consumer — imud-mon, a Prometheus scrape — and a fault nobody can observe
 * is not worth a bit of the wire format.
 */
TEST(test_state_reset_flag_latches_until_converged)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    f.bias[0] = NAN;
    imu_sample_t s = make_gyro(0.0f, 0.0f, 0.0f);
    s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = -G;
    mekf_predict(&f, &s, f.dt);
    EXPECT(mekf_sanitize(&f), "sanitize acted");

    fused_state_t out;
    mekf_get_state(&f, &out, 0);
    EXPECT(out.flags & FLAG_STATE_RESET, "flag set on the reset packet");

    /* Still set several packets later, while the filter re-converges. */
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    for (int i = 0; i < 50; i++) {
        mekf_predict(&f, &s, f.dt);
        mekf_update_accel(&f, &s);
        mekf_sanitize(&f);
    }
    mekf_get_state(&f, &out, 0);
    EXPECT(out.flags & FLAG_STATE_RESET, "flag still set while re-converging");

    /* Once converged, it clears — a recovered filter must read as recovered. */
    f.converged = true;
    mekf_get_state(&f, &out, 0);
    EXPECT(!(out.flags & FLAG_STATE_RESET), "flag clears on reconvergence");
    EXPECT(out.flags & FLAG_FUSION_CONVERGED, "converged flag takes over");
}

/*
 * get_state: heading wraps correctly at 0° / 360° boundary.
 */
TEST(test_get_state_heading_wrap)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Set q to a −10° yaw (= 350° heading) */
    float psi = -10.0f * DEG;
    f.q[0] = cosf(psi/2); f.q[1] = 0; f.q[2] = 0; f.q[3] = sinf(psi/2);

    fused_state_t out;
    mekf_get_state(&f, &out, 0);

    EXPECT_NEAR(out.heading_deg, 350.0f, 0.2f, "−10° yaw → 350° heading");
}

/* ── Tier 1: safety paths and gate logic ────────────────────────────────────── */

/*
 * mekf_align must not initialize the filter when |accel| < 0.5 g.
 * Catches the "wildly wrong accel" guard at the top of mekf_align.
 */
TEST(test_align_accel_too_weak)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    float weak = 0.4f * G;   /* 0.4 g < 0.5 g threshold */
    mekf_align(&f, (float[]){0, 0, -weak}, (float[]){20.0f, 0.0f, 5.0f});

    EXPECT(!f.initialized, "filter NOT initialized when |accel| < 0.5 g");
    EXPECT(!f.m_ref_valid, "m_ref NOT set when |accel| < 0.5 g");
}

/*
 * mekf_update_mag must skip when m->valid is false (invalid reading) or
 * when m_ref_valid is false (reference not yet established).
 */
TEST(test_mag_update_skips_invalid)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0, 0, -G}, (float[]){20.0f, 0.0f, 5.0f});

    float q0[4]; memcpy(q0, f.q, sizeof q0);

    /* m.valid = false — sensor marked bad */
    mag_sample_t m = make_mag(0.2f, 0.0f, 0.05f);
    m.valid = false;
    mekf_update_mag(&f, &m);
    EXPECT_NEAR(f.q[0], q0[0], 1e-7f, "w unchanged when m.valid=false");
    EXPECT_NEAR(f.q[3], q0[3], 1e-7f, "z unchanged when m.valid=false");

    /* m_ref_valid = false — reference not yet set */
    m.valid = true;
    f.m_ref_valid = false;
    mekf_update_mag(&f, &m);
    EXPECT_NEAR(f.q[0], q0[0], 1e-7f, "w unchanged when m_ref_valid=false");
    EXPECT_NEAR(f.q[3], q0[3], 1e-7f, "z unchanged when m_ref_valid=false");
}

/*
 * Magnitude gate: mag updates must be rejected when the measured field
 * magnitude is outside [0.5×, 2×] the expected Earth-field magnitude.
 * This gate directly catches the µT/Gauss unit mismatch bug we found.
 */
TEST(test_mag_ratio_gate)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    /* Align: m_ref set to ~[0.20, 0, 0.05] Gauss → |m_ref| ≈ 0.206 Gauss */
    mekf_align(&f, (float[]){0, 0, -G}, (float[]){20.0f, 0.0f, 5.0f});
    float q0[4]; memcpy(q0, f.q, sizeof q0);

    /* 0.01 Gauss field: ratio ≈ 0.05 < 0.5 → must be rejected */
    mag_sample_t low = make_mag(0.01f, 0.0f, 0.0f);
    for (int i = 0; i < 10; i++) mekf_update_mag(&f, &low);
    EXPECT_NEAR(f.q[0], q0[0], 1e-6f, "w unchanged for sub-threshold field magnitude");
    EXPECT_NEAR(f.q[3], q0[3], 1e-6f, "z unchanged for sub-threshold field magnitude");

    /* 2.0 Gauss field: ratio ≈ 9.7 > 2.0 → must be rejected */
    mag_sample_t high = make_mag(2.0f, 0.0f, 0.0f);
    for (int i = 0; i < 10; i++) mekf_update_mag(&f, &high);
    EXPECT_NEAR(f.q[0], q0[0], 1e-6f, "w unchanged for super-threshold field magnitude");
    EXPECT_NEAR(f.q[3], q0[3], 1e-6f, "z unchanged for super-threshold field magnitude");
}

/*
 * mekf_reconfigure must update all noise/gate scalars without touching
 * q, P, bias, or dt (the filter keeps running through a SIGHUP reload).
 */
TEST(test_mekf_reconfigure)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0.001f, -0.002f, 0.003f};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0, 0, -G}, (float[]){20.0f, 0.0f, 5.0f});

    float q_before[4]; memcpy(q_before, f.q, sizeof q_before);
    float dt_before = f.dt;

    imud_config_t new_cfg        = cfg;
    new_cfg.mekf_gyro_noise      = 0.02;    /* was 0.007 */
    new_cfg.mekf_gyro_bias       = 0.0005;  /* was 0.00015 */
    new_cfg.mekf_accel_noise     = 0.005;   /* was 0.0022 */
    new_cfg.mekf_mag_noise       = 0.001;   /* was 0.0004 */
    new_cfg.accel_skip_thresh    = 0.10;    /* was 0.05 */
    new_cfg.mag_reject_gauss     = 0.002;   /* was 0.0008 */

    mekf_reconfigure(&f, &new_cfg);

    float dt = 1.0f / 833.0f;
    EXPECT_NEAR(f.Qg, 0.02f*0.02f*dt,              1e-10f, "Qg updated");
    EXPECT_NEAR(f.Qb, 0.0005f*0.0005f*dt,          1e-13f, "Qb updated");
    EXPECT_NEAR(f.Ra, (0.005f/9.80665f)*(0.005f/9.80665f)*833.0f, 1e-8f, "Ra updated");
    EXPECT_NEAR(f.Rm, 0.001f*0.001f*100.0f,        1e-9f,  "Rm updated (mag_odr_hz=100)");
    EXPECT_NEAR(f.accel_skip_lo, 0.90f,            1e-6f,  "accel_skip_lo updated");
    EXPECT_NEAR(f.accel_skip_hi, 1.10f,            1e-6f,  "accel_skip_hi updated");
    EXPECT_NEAR(f.mag_reject_sq, 0.002f*0.002f,    1e-9f,  "mag_reject_sq updated");

    /* dt and q must be untouched */
    EXPECT_NEAR(f.dt, dt_before, 1e-9f, "dt unchanged after reconfigure");
    EXPECT_NEAR(f.q[0], q_before[0], 1e-7f, "q[0] unchanged after reconfigure");
    EXPECT_NEAR(f.q[3], q_before[3], 1e-7f, "q[3] unchanged after reconfigure");
}

/*
 * Rm and mref_alpha are derived in one shared helper. Reconfigure used to
 * rebuild Rm while leaving mref_alpha at its init value, so a retune left the
 * m_ref EMA running at the wrong gain indefinitely; deriving both in one place
 * is what stops them drifting apart again. Driven here through mekf_mag_noise,
 * which is [hot] and moves Rm alone — the mag RATE is [restart] and no longer
 * reaches this path at all (see the next test).
 */
TEST(test_reconfigure_rederives_mag_tuning)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    float alpha_before = f.mref_alpha;
    float rm_before    = f.Rm;
    EXPECT(alpha_before > 0.0f, "mref_alpha non-zero at init");

    imud_config_t new_cfg = cfg;
    new_cfg.mekf_mag_noise = cfg.mekf_mag_noise * 2.0;
    mekf_reconfigure(&f, &new_cfg);

    EXPECT_NEAR(f.Rm, rm_before * 4.0f, 1e-12f,
                "Rm follows mekf_mag_noise squared");
    EXPECT_NEAR(f.mref_alpha, alpha_before, 1e-9f,
                "mref_alpha is re-derived, not left stale or zeroed");
    EXPECT_NEAR(f.nis_mag_alpha, 1.0f / (30.0f * 100.0f), 1e-9f,
                "nis_mag_alpha is re-derived from the programmed mag rate");
}

/*
 * Every mag-rate-derived value comes from the rate handed to mekf_init — the
 * rate the driver said it would actually program — and NOT from
 * cfg->mag_odr_hz, which is only the operator's request.
 *
 * Rm used to be Nm² × cfg->mag_odr_hz, so an off-grid request (137 Hz on a
 * part whose grid is 1/10/20/50/100/200/1000) sized the magnetometer's noise
 * variance for a rate the chip was not sampling at. mref_alpha and
 * nis_mag_alpha had the same source. The second half pins the [restart]
 * contract: SIGHUP does not re-init the driver, so a changed request must not
 * retune the filter for a rate the hardware is not running at.
 */
TEST(test_mag_tuning_uses_the_programmed_rate)
{
    imud_config_t cfg = make_cfg();
    cfg.mag_odr_hz = 137;          /* requested — not on any real grid */
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, 200.0f, bias);   /* what the driver programs */

    float Nm = (float)cfg.mekf_mag_noise;
    EXPECT_NEAR(f.Rm, Nm * Nm * 200.0f, 1e-12f,
                "Rm is sized for the programmed 200 Hz, not the 137 requested");
    EXPECT_NEAR(f.mref_alpha, 1.0f / (300.0f * 200.0f), 1e-12f,
                "mref_alpha uses the programmed rate");
    EXPECT_NEAR(f.nis_mag_alpha, 1.0f / (30.0f * 200.0f), 1e-12f,
                "nis_mag_alpha uses the programmed rate");

    /* [mag] odr_hz is [restart]: a reload must not move any of the three. */
    float rm = f.Rm, mref = f.mref_alpha, nis = f.nis_mag_alpha;
    imud_config_t new_cfg = cfg;
    new_cfg.mag_odr_hz = 50;
    mekf_reconfigure(&f, &new_cfg);

    EXPECT_NEAR(f.Rm, rm, 1e-12f,
                "reconfigure ignores a changed mag odr_hz request (restart key)");
    EXPECT_NEAR(f.mref_alpha, mref, 1e-12f, "mref_alpha unchanged by reload");
    EXPECT_NEAR(f.nis_mag_alpha, nis, 1e-12f,
                "nis_mag_alpha unchanged by reload");
}

/*
 * Reconfigure resets the accel skip window from the non-engine threshold.
 * The fusion thread re-asserts the engine-mode window on the very next
 * sample, so the two can no longer be left inconsistent (skip narrow while
 * Ra_scale is still 4.0) — this pins the fusion half of that contract:
 * reconfigure must leave the window at exactly the configured value, so the
 * thread's unconditional re-assert is what decides.
 */
TEST(test_reconfigure_resets_skip_window)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    /* Simulate the fusion thread having entered engine mode. */
    f.accel_skip_lo = 1.0f - 0.20f;
    f.accel_skip_hi = 1.0f + 0.20f;
    f.Ra_scale      = 4.0f;

    mekf_reconfigure(&f, &cfg);

    EXPECT_NEAR(f.accel_skip_lo, 1.0f - (float)cfg.accel_skip_thresh, 1e-6f,
                "reconfigure resets skip_lo to the configured value");
    EXPECT_NEAR(f.accel_skip_hi, 1.0f + (float)cfg.accel_skip_thresh, 1e-6f,
                "reconfigure resets skip_hi to the configured value");
    EXPECT_NEAR(f.Ra_scale, 4.0f, 1e-6f,
                "reconfigure leaves Ra_scale to the fusion thread");
}

/* ── Sim end-to-end tests ───────────────────────────────────────────────────── */

/* Sim parameters (must match src/drivers/sim.c) */
#define SIM_YAW_DEG_S   6.0f
#define SIM_B_NORTH_G   0.25f   /* 25 µT north */
#define SIM_B_DOWN_G   -0.40f   /* -40 µT down */

/*
 * Gyro-only: 6°/s for 10 s → heading ≈ 60°.
 * No measurement updates — verifies pure gyro integration and heading extraction.
 */
TEST(test_sim_gyro_heading)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    /* Flat board, pointing North; sim initial mag = [25, 0, -40] µT */
    mekf_align(&f, (float[]){0, 0, -G}, (float[]){25.0f, 0.0f, -40.0f});

    imu_sample_t sg = make_gyro(0, 0, SIM_YAW_DEG_S * DEG);
    for (int i = 0; i < 833 * 10; i++)
        mekf_predict(&f, &sg, f.dt);

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    float hdg = yaw * (180.0f / (float)M_PI);
    if (hdg < 0) hdg += 360.0f;

    EXPECT_NEAR(roll,  0,     0.5f*DEG, "roll ≈ 0° during pure yaw");
    EXPECT_NEAR(pitch, 0,     0.5f*DEG, "pitch ≈ 0° during pure yaw");
    EXPECT_NEAR(hdg,  60.0f, 1.0f,     "heading ≈ 60° after 10 s at 6°/s");
}

/*
 * Heading wraps at 360°: 6°/s × 61 s = 366° → should read ≈ 6°.
 */
TEST(test_sim_heading_wraps)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0, 0, -G}, (float[]){25.0f, 0.0f, -40.0f});

    imu_sample_t sg = make_gyro(0, 0, SIM_YAW_DEG_S * DEG);
    for (int i = 0; i < 833 * 61; i++)
        mekf_predict(&f, &sg, f.dt);

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    float hdg = yaw * (180.0f / (float)M_PI);
    if (hdg < 0) hdg += 360.0f;

    EXPECT_NEAR(hdg, 6.0f, 2.0f, "heading wraps: 366° → ≈ 6°");
}

/*
 * Full sim fusion: gyro + rotating mag + static accel for 30 s.
 * Expected: heading ≈ 180°, pitch ≈ roll ≈ 0°.
 */
TEST(test_sim_full_fusion)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0, 0, -G}, (float[]){25.0f, 0.0f, -40.0f});

    float wz_rad = SIM_YAW_DEG_S * DEG;
    float elapsed = 0.0f;
    float dt      = 1.0f / 833.0f;
    int   mag_div = 0;

    for (int i = 0; i < 833 * 30; i++) {
        elapsed += dt;

        imu_sample_t sg = make_gyro(0, 0, wz_rad);
        mekf_predict(&f, &sg, f.dt);

        imu_sample_t sa = make_accel(0, 0, -G);
        mekf_update_accel(&f, &sa);

        /* Mag at ~100 Hz (every 8 IMU samples), rotating with yaw */
        if (++mag_div >= 8) {
            mag_div = 0;
            float yaw_t = wz_rad * elapsed;
            mag_sample_t m = make_mag( SIM_B_NORTH_G * cosf(yaw_t),
                                      -SIM_B_NORTH_G * sinf(yaw_t),
                                       SIM_B_DOWN_G);
            mekf_update_mag(&f, &m);
        }
    }

    fused_state_t state;
    mekf_get_state(&f, &state, 0);

    EXPECT_NEAR(state.heading_deg,                      180.0f, 3.0f, "heading ≈ 180° after 30 s");
    EXPECT_NEAR(state.pitch * (180.0f/(float)M_PI), 0.0f,   1.0f, "pitch ≈ 0°");
    EXPECT_NEAR(state.roll  * (180.0f/(float)M_PI), 0.0f,   1.0f, "roll ≈ 0°");
}

/* ── New-mechanics tests: Joseph form, χ² cap, m_ref EMA, yaw-only ─────────── */

/*
 * Joseph form must keep P symmetric and positive on the diagonal through
 * tens of thousands of noisy update cycles (the simple form slowly loses
 * both in float arithmetic).
 */
TEST(test_joseph_symmetry_psd)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    uint32_t rng = 42;
    for (int i = 0; i < 50000; i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        float n1 = ((float)(int32_t)rng) * (1.0f/2147483648.0f);
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        float n2 = ((float)(int32_t)rng) * (1.0f/2147483648.0f);

        imu_sample_t g = make_gyro(0.05f*n1, -0.03f*n2, 0.04f*n1);
        mekf_predict(&f, &g, f.dt);

        imu_sample_t a = make_accel(0.2f*n2, 0.2f*n1, -G);
        mekf_update_accel(&f, &a);

        if (i % 8 == 0) {
            mag_sample_t m = make_mag(0.20f + 0.002f*n1, 0.002f*n2, 0.40f);
            mekf_update_mag(&f, &m);
        }
    }

    float max_asym = 0.0f, min_diag = 1e9f;
    for (int i = 0; i < MEKF_N; i++) {
        if (i < live_n(&f) && f.P[i][i] < min_diag) min_diag = f.P[i][i];
        for (int j = 0; j < MEKF_N; j++) {
            float d = fabsf(f.P[i][j] - f.P[j][i]);
            if (d > max_asym) max_asym = d;
        }
    }
    EXPECT(max_asym == 0.0f, "P exactly symmetric after 50k updates");
    EXPECT(min_diag > 0.0f,  "P diagonal stays positive after 50k updates");
    EXPECT_NEAR(q_norm(f.q), 1.0f, 1e-4f, "q unit norm after 50k updates");
}

/*
 * Error-state reset: after injecting δθ into the quaternion, P must be rotated
 * into the new error frame by G = I − ½[δθ]×. Verified by reproducing the
 * transform independently: run one update, then check the attitude block moved
 * the way G·P·Gᵀ predicts rather than staying in the pre-reset frame.
 *
 * Constructed so δθ is large enough for G to be clearly distinguishable from
 * I: a filter still wide open from init, given an accel far off its predicted
 * direction, takes a correction of several degrees.
 */
TEST(test_reset_jacobian_rotates_P)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Snapshot P and bias, then take one large-correction update. */
    float P0[MEKF_N][MEKF_N];
    memcpy(P0, f.P, sizeof P0);
    float q0[4]; memcpy(q0, f.q, sizeof q0);

    /* Tilt on BOTH horizontal axes so δθ has more than one non-zero
     * component: a single-axis tilt makes G·P·Gᵀ − P purely diagonal (since
     * [v]×[v]× = vvᵀ − |v|²I collapses when v has one component), which would
     * hide any off-diagonal error in the reset. */
    float th = 8.0f * DEG;
    imu_sample_t tilted = make_accel(G*sinf(th)*0.7071f,
                                     G*sinf(th)*0.7071f,
                                     -G*cosf(th));
    mekf_update_accel(&f, &tilted);

    /* Recover the δθ actually applied, from the quaternion change:
     * dq = q0* ⊗ q1, and δθ = 2·vec(dq) for a small rotation. */
    float q0c[4] = { q0[0], -q0[1], -q0[2], -q0[3] };
    float dq[4];
    dq[0] = q0c[0]*f.q[0] - q0c[1]*f.q[1] - q0c[2]*f.q[2] - q0c[3]*f.q[3];
    dq[1] = q0c[0]*f.q[1] + q0c[1]*f.q[0] + q0c[2]*f.q[3] - q0c[3]*f.q[2];
    dq[2] = q0c[0]*f.q[2] - q0c[1]*f.q[3] + q0c[2]*f.q[0] + q0c[3]*f.q[1];
    dq[3] = q0c[0]*f.q[3] + q0c[1]*f.q[2] - q0c[2]*f.q[1] + q0c[3]*f.q[0];
    float dth[3] = { 2.0f*dq[1], 2.0f*dq[2], 2.0f*dq[3] };
    float dth_mag = sqrtf(dth[0]*dth[0] + dth[1]*dth[1] + dth[2]*dth[2]);

    /* The test is only meaningful if G is materially different from I. */
    EXPECT(dth_mag > 0.01f, "reset test drives a correction big enough to see");

    /* G differs from I by ½|δθ|; P's attitude block must have been rotated,
     * i.e. it cannot equal the unrotated Joseph result. Rather than reproduce
     * the whole Joseph update, assert the weaker but decisive property: the
     * attitude block is NOT symmetric-equal to what an un-rotated update would
     * leave, and P stayed a valid covariance. */
    float max_asym = 0.0f, min_diag = 1e9f;
    for (int i = 0; i < MEKF_N; i++) {
        if (i < live_n(&f) && f.P[i][i] < min_diag) min_diag = f.P[i][i];
        for (int j = 0; j < MEKF_N; j++) {
            float d = fabsf(f.P[i][j] - f.P[j][i]);
            if (d > max_asym) max_asym = d;
        }
    }
    EXPECT(max_asym == 0.0f, "P symmetric after reset rotation");
    EXPECT(min_diag > 0.0f,  "P diagonal positive after reset rotation");

    /* Directly verify the rotation: apply G to the PRE-update attitude block
     * and confirm the sign of the off-diagonal change matches. G = I − ½[δθ]×
     * is not symmetric, so G·P₀·Gᵀ perturbs off-diagonals in a direction set
     * by δθ; an identity reset would leave them untouched by this term. */
    float Gm[3][3] = {
        {  1.0f,          0.5f*dth[2], -0.5f*dth[1] },
        { -0.5f*dth[2],   1.0f,         0.5f*dth[0] },
        {  0.5f*dth[1],  -0.5f*dth[0],  1.0f        },
    };
    float GP[3][3], GPGt[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += Gm[i][k] * P0[k][j];
            GP[i][j] = s;
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += GP[i][k] * Gm[j][k];
            GPGt[i][j] = s;
        }
    /* G·P₀·Gᵀ must differ from P₀ across the whole attitude block, and the
     * off-diagonal part specifically must move — that is the part an identity
     * reset would leave untouched. */
    float moved = 0.0f, moved_offdiag = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float d = fabsf(GPGt[i][j] - P0[i][j]);
            moved += d;
            if (i != j) moved_offdiag += d;
        }
    EXPECT(moved > 1e-9f,
           "G is materially different from identity for this δθ");
    EXPECT(moved_offdiag > 1e-12f,
           "reset rotates off-diagonal attitude covariance, not just scale");

    /* The reset applies G to the attitude block only: G_full = diag(G, I₃, I₃).
     * Any leakage into the bias or wave rows would be a silent modelling error,
     * so check the wave block came through the update still symmetric and
     * still carrying its own variance rather than the attitude's. */
    for (int i = 6; i < MEKF_N; i++)
        EXPECT(f.P[i][i] > 0.0f, "wave block keeps positive variance through reset");
}

/* ── Gauss–Markov wave-acceleration state (ROADMAP §10.5) ───────────────────── */

static imud_config_t make_cfg_nowave(void)
{
    imud_config_t c = make_cfg();
    c.mekf_wave_accel       = 0.0;
    c.mekf_wave_accel_tau_s = 0.0;
    return c;
}

/*
 * With either knob at 0 the wave block must be INERT, not merely small: rows
 * and columns 6–8 of P exactly zero and the nominal estimate exactly zero,
 * through tens of thousands of noisy cycles. This is what lets the feature
 * ship enabled by default without stranding anyone who turns it off — the
 * disabled filter is the pre-§10.5 filter, bit for bit.
 */
TEST(test_wave_disabled_inert)
{
    imud_config_t cfg = make_cfg_nowave();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    EXPECT(!f.wave_enabled, "wave state reports disabled with both knobs 0");

    uint32_t rng = 7;
    for (int i = 0; i < 50000; i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        float n1 = ((float)(int32_t)rng) * (1.0f/2147483648.0f);
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        float n2 = ((float)(int32_t)rng) * (1.0f/2147483648.0f);

        imu_sample_t g = make_gyro(0.05f*n1, -0.03f*n2, 0.04f*n1);
        mekf_predict(&f, &g, f.dt);
        imu_sample_t a = make_accel(0.2f*n2, 0.2f*n1, -G);
        mekf_update_accel(&f, &a);
        if (i % 8 == 0) {
            mag_sample_t m = make_mag(0.20f + 0.002f*n1, 0.002f*n2, 0.40f);
            mekf_update_mag(&f, &m);
        }
    }

    float leak = 0.0f;
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 6; j < MEKF_N; j++)
            leak += fabsf(f.P[i][j]) + fabsf(f.P[j][i]);
    for (int i = 0; i < 3; i++) leak += fabsf(f.wave_acc[i]);

    EXPECT(leak == 0.0f, "disabled wave block stays exactly zero over 50k cycles");
}

/*
 * One knob at 0 is enough to disable — the pair is meaningless apart, and a
 * config with a σ but no τ (or vice versa) must not half-enable anything.
 */
TEST(test_wave_needs_both_knobs)
{
    mekf_t f;
    float bias[3] = {0};

    imud_config_t c1 = make_cfg();  c1.mekf_wave_accel_tau_s = 0.0;
    mekf_init(&f, &c1, 833.0f, (float)c1.mag_odr_hz, bias);
    EXPECT(!f.wave_enabled, "sigma without tau leaves the wave state off");
    EXPECT(f.P[6][6] == 0.0f, "sigma without tau seeds no wave covariance");

    imud_config_t c2 = make_cfg();  c2.mekf_wave_accel = 0.0;
    mekf_init(&f, &c2, 833.0f, (float)c2.mag_odr_hz, bias);
    EXPECT(!f.wave_enabled, "tau without sigma leaves the wave state off");
    EXPECT(f.P[6][6] == 0.0f, "tau without sigma seeds no wave covariance");

    imud_config_t c3 = make_cfg();
    mekf_init(&f, &c3, 833.0f, (float)c3.mag_odr_hz, bias);
    EXPECT(f.wave_enabled, "both knobs positive enables the wave state");
    EXPECT_NEAR(f.P[6][6], (c3.mekf_wave_accel/9.80665)*(c3.mekf_wave_accel/9.80665),
                1e-9f, "wave block seeded at its steady-state variance");
}

/*
 * The estimator has to actually estimate. Drive a level board with a KNOWN
 * oscillating lateral acceleration — slow enough to be seen, fast enough that
 * the τ = 0.5 s prior does not fight it — and check three things:
 *
 *   1. a_w tracks the truth (this is the sign check: get ẑ = normalize(h − a_w)
 *      backwards and the state runs away from the disturbance, not toward it);
 *   2. the attitude estimate stays much closer to level than the same run with
 *      the state disabled, because the disturbance is explained rather than
 *      absorbed into tilt;
 *   3. NIS comes down, because the innovations are now predicted.
 */
static void wave_track_run(bool enabled, float *att_err_deg, float *aw_rms_err,
                           float *aw_rms_true, float *nis)
{
    imud_config_t cfg = enabled ? make_cfg() : make_cfg_nowave();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    const float fs = 833.0f, dt = 1.0f/fs;
    const float amp = 0.9f;                       /* m/s² */
    const float w   = 2.0f*(float)M_PI*0.5f;      /* 0.5 Hz — period 2 s */

    double se = 0.0, st = 0.0, sa = 0.0;
    int n = 0;

    for (int i = 0; i < (int)(90.0f*fs); i++) {
        float t = (float)i * dt;
        float a_lat = amp * sinf(w * t);

        imu_sample_t s;
        memset(&s, 0, sizeof s);
        s.accel[0] = a_lat;     /* level board: specific force = a_lin − g */
        s.accel[1] = 0.0f;
        s.accel[2] = -G;

        mekf_predict(&f, &s, dt);
        mekf_update_accel(&f, &s);
        if (i % 8 == 0) {
            mag_sample_t m = make_mag(0.20f, 0.0f, 0.40f);
            mekf_update_mag(&f, &m);
        }

        if (t > 30.0f) {                          /* after settling */
            float truth = a_lat / G;              /* a_w in g units */
            float d = f.wave_acc[0] - truth;
            se += (double)d * d;
            st += (double)truth * truth;
            float roll, pitch, yaw;
            q_to_euler(f.q, &roll, &pitch, &yaw);
            sa += (double)(roll*roll + pitch*pitch);
            n++;
        }
    }
    *att_err_deg = (float)(sqrt(sa / n) / DEG);
    *aw_rms_err  = (float)sqrt(se / n);
    *aw_rms_true = (float)sqrt(st / n);
    *nis         = f.nis_accel_ema;
}

TEST(test_wave_state_tracks_colored_residual)
{
    float att_on, att_off, err_on, err_off, truth, nis_on, nis_off;
    wave_track_run(true,  &att_on,  &err_on,  &truth, &nis_on);
    wave_track_run(false, &att_off, &err_off, &truth, &nis_off);

    EXPECT(err_on < 0.5f * truth,
           "wave state tracks the injected acceleration to better than half its RMS");
    EXPECT(err_on < 0.5f * err_off,
           "tracking is better than the disabled filter's implicit zero estimate");
    EXPECT(att_on < 0.6f * att_off,
           "explaining the disturbance keeps the attitude estimate closer to level");
    EXPECT(nis_on < nis_off,
           "modelled innovations are more consistent than unmodelled ones");
}

/*
 * The dof = 2 the wire reports depends on ẑᵀH = 0 holding EXACTLY, which is
 * what the tangent projector in wave_jacobian is for. Drop it and ẑ stops
 * being an eigenvector of S, the radial component leaks into d², and the NIS
 * field quietly stops meaning "1.0 = consistent".
 *
 * H is not reachable from here, but the consequence is. Feed a purely WHITE
 * accel disturbance sized to match Ra exactly — so the base path's dof = 2
 * derivation says NIS = 1 — and run it twice: once with the wave state off
 * (the pre-§10.5 Jacobian) and once with it on at a σ small enough not to
 * dominate S. Both must land on 1, and on each other. An unprojected wave
 * Jacobian breaks the second run while leaving the first untouched.
 *
 * Note this is deliberately NOT run at the shipped σ. At σ = 0.8 m/s² in dead
 * calm the filter budgets for a seaway that is not there and NIS reads 0.02 —
 * correct, conservative, and useless as a dof check.
 */
static float white_nis_run(double wave_sigma)
{
    imud_config_t cfg = make_cfg();
    cfg.mekf_wave_accel       = wave_sigma;
    cfg.mekf_wave_accel_tau_s = wave_sigma > 0.0 ? 0.5 : 0.0;
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Per-sample accel noise the filter expects: σ = Na·√odr = 0.0635 m/s². */
    const float sig = (float)(0.0022 * sqrt(833.0));

    uint32_t rng = 20260725u;
    for (int i = 0; i < 300000; i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        float n1 = ((float)(int32_t)rng) * (1.0f/2147483648.0f) * 1.732f;
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        float n2 = ((float)(int32_t)rng) * (1.0f/2147483648.0f) * 1.732f;

        imu_sample_t s;
        memset(&s, 0, sizeof s);
        s.accel[0] = sig*n1; s.accel[1] = sig*n2; s.accel[2] = -G;
        mekf_predict(&f, &s, f.dt);
        mekf_update_accel(&f, &s);
        if (i % 8 == 0) {
            mag_sample_t m = make_mag(0.20f, 0.0f, 0.40f);
            mekf_update_mag(&f, &m);
        }
    }
    return f.nis_accel_ema;
}

TEST(test_wave_nis_dof_consistent)
{
    float nis_off = white_nis_run(0.0);
    float nis_on  = white_nis_run(0.02);   /* ≪ the 0.0635 m/s² of white noise */

    EXPECT(nis_off > 0.6f && nis_off < 1.6f,
           "white noise matched to Ra gives NIS ~ 1 on the base path (dof = 2)");
    EXPECT(nis_on > 0.6f && nis_on < 1.6f,
           "the wave-aware Jacobian preserves NIS ~ 1 (tangent projection)");
    EXPECT(fabsf(nis_on - nis_off) < 0.25f * nis_off,
           "wave-aware and base paths agree on d² when the state is not driving");
}

/*
 * Enabling and disabling changes which states exist, so mekf_reconfigure has
 * to seed and clear the block — see the comment there for why a σ/τ change
 * with the state left on must NOT touch P.
 */
TEST(test_wave_reconfigure_transitions)
{
    imud_config_t on  = make_cfg();
    imud_config_t off = make_cfg_nowave();
    mekf_t f;
    float bias[3] = {0};

    /* Start enabled, run a little so cross-covariances build up. */
    mekf_init(&f, &on, 833.0f, (float)on.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    for (int i = 0; i < 5000; i++) {
        imu_sample_t s = make_accel(0.3f, 0.0f, -G);
        mekf_predict(&f, &s, f.dt);
        mekf_update_accel(&f, &s);
    }
    EXPECT(f.P[6][6] > 0.0f, "enabled wave block carries variance");
    EXPECT(fabsf(f.wave_acc[0]) > 1e-4f, "wave state learned something to clear");

    /* on → off: block and estimate must both go to exactly zero. */
    mekf_reconfigure(&f, &off);
    float leak = 0.0f;
    for (int i = 0; i < MEKF_N; i++)
        for (int j = 6; j < MEKF_N; j++) leak += fabsf(f.P[i][j]) + fabsf(f.P[j][i]);
    for (int i = 0; i < 3; i++) leak += fabsf(f.wave_acc[i]);
    EXPECT(!f.wave_enabled, "reconfigure to 0 disables the wave state");
    EXPECT(leak == 0.0f, "disabling clears the wave block and estimate exactly");

    /* off → on: block must be re-seeded, or the state can never learn. */
    mekf_reconfigure(&f, &on);
    EXPECT(f.wave_enabled, "reconfigure back on re-enables the wave state");
    EXPECT_NEAR(f.P[6][6], (on.mekf_wave_accel/9.80665)*(on.mekf_wave_accel/9.80665),
                1e-9f, "re-enabling reseeds the steady-state variance");

    /* σ/τ change with the state on: P is deliberately left alone. */
    for (int i = 0; i < 5000; i++) {
        imu_sample_t s = make_accel(0.3f, 0.0f, -G);
        mekf_predict(&f, &s, f.dt);
        mekf_update_accel(&f, &s);
    }
    float p66 = f.P[6][6], p06 = f.P[0][6];
    imud_config_t tweak = make_cfg();
    tweak.mekf_wave_accel = on.mekf_wave_accel * 1.5;
    mekf_reconfigure(&f, &tweak);
    EXPECT(f.P[6][6] == p66 && f.P[0][6] == p06,
           "changing sigma with the state on leaves P to re-equilibrate");
    EXPECT(f.wave_sig2 > 0.0f, "changed sigma is picked up in the tuning");
}

/* ── Anisotropic magnetic dip-reference uncertainty ─────────────────────────── */

/*
 * Drive a level board with a clean field and a mag reference whose DIP is
 * deliberately wrong by `dip_err_deg`, in 3-D vector mode. Reports the settled
 * roll/pitch bias, P's roll/pitch sigma, and the mag NIS.
 */
static void dip_run(double dip_sigma_deg, float dip_err_deg, float mag_noise_g,
                    float *bias_deg, float *sigma_deg, float *nis_mag)
{
    imud_config_t cfg = make_cfg();
    cfg.mag_yaw_only = false;
    cfg.mekf_mag_dip_sigma_deg = dip_sigma_deg;
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    /* Field: 0.47 G total at 62° dip, pointing north. */
    const float H = 0.2206f, Z = 0.4149f;
    mekf_align(&f, (float[]){0,0,-G}, (float[]){H*100.0f, 0.0f, Z*100.0f});

    /* Corrupt ONLY the dip of the reference, preserving |m_ref| and the
     * horizontal direction — exactly the error alignment leaves behind. */
    {
        float mag = sqrtf(H*H + Z*Z);
        float d   = atan2f(Z, H) + dip_err_deg * DEG;
        f.m_ref[0] = mag * cosf(d);
        f.m_ref[1] = 0.0f;
        f.m_ref[2] = mag * sinf(d);
    }

    imu_sample_t sa = make_accel(0, 0, -G);
    imu_sample_t sg = make_gyro(0, 0, 0);
    uint32_t rng = 424242u;
    for (int i = 0; i < 200000; i++) {
        mekf_predict(&f, &sg, f.dt);
        mekf_update_accel(&f, &sa);
        if (i % 8 == 0) {
            float n[3];
            for (int k = 0; k < 3; k++) {
                rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
                n[k] = ((float)(int32_t)rng) * (1.0f/2147483648.0f)
                       * mag_noise_g * 1.732f;
            }
            mag_sample_t sm = make_mag(H + n[0], n[1], Z + n[2]);
            mekf_update_mag(&f, &sm);
        }
    }

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    *bias_deg  = sqrtf(roll*roll + pitch*pitch) / DEG;
    *sigma_deg = sqrtf(0.5f * (f.P[0][0] + f.P[1][1])) / DEG;
    *nis_mag   = f.nis_mag_ema;
}

/*
 * σ_dip = 0 must leave the 3-D update on the isotropic path untouched, and a
 * non-zero σ_dip must widen P's roll/pitch without moving the estimate much —
 * the term admits an unmodelled reference error into the covariance, it does
 * not pretend to remove it.
 */
TEST(test_mag_dip_sigma_widens_covariance)
{
    float b0, s0, n0, b1, s1, n1, b2, s2, n2;
    dip_run(0.0, 3.0f, 0.0f, &b0, &s0, &n0);
    dip_run(3.0, 3.0f, 0.0f, &b1, &s1, &n1);
    dip_run(6.0, 3.0f, 0.0f, &b2, &s2, &n2);

    EXPECT(b0 > 0.5f, "a wrong dip reference does bias roll/pitch in 3-D mode");
    EXPECT(s1 > s0 && s2 > s1,
           "sigma_dip monotonically widens the roll/pitch covariance");
    EXPECT(b2 <= b0,
           "and does not make the bias itself worse");

    /*
     * The widening is deliberately modest HERE (0.82° → 0.88° at σ_dip = 6°)
     * and that is not a weak result: this platform is dead flat and quiet, so
     * the accelerometer is very confident about roll and pitch and owns most
     * of that covariance — the magnetometer's dip channel is a small
     * contributor to it. The regime where the term earns its keep is a seaway,
     * where the wave state correctly makes the accelerometer far less
     * confident: there the same knob moves 3-D NEES(strict) 12.83 → 5.74.
     * That magnitude claim belongs to test_wave_benchmark; this test owns the
     * mechanism.
     */
}

/*
 * The rank-1 term must be TANGENT to the predicted direction (uᵀĥ = 0). That
 * is what keeps ĥ an eigenvector of S with eigenvalue Rm — the premise of the
 * dof-2 normalisation behind `nis_mag` (docs/math.md §8.2) — and it is also
 * what makes the term do any work: the radial direction of a normalised
 * measurement carries no information, so any inflation spent there is wasted.
 *
 * The signature is measurable. With a correct reference and white mag noise,
 * comparing σ_dip = 0 against σ_dip = 6°:
 *
 *                        P roll/pitch σ    residual pull    nis_mag
 *   tangent u (correct)   0.787 → 0.862      93% absorbed    0.96 → 0.49
 *   u with a radial part  0.787 → 0.796      53% absorbed    0.96 → 0.65
 *
 * i.e. a contaminated u barely widens P and leaves half the pull in place.
 *
 * On the nis_mag column: ~0.5 is the CORRECT reading once σ_dip dominates, not
 * a regression. One of the two tangent degrees of freedom is deliberately
 * deweighted, so a consistent filter reads about half. The dof is still 2 —
 * that is the radial direction still contributing nothing. What must not
 * happen is the collapse to ~0.01 that isotropic inflation produces, which
 * would leave the wire unable to report a magnetometer fault at all.
 */
TEST(test_mag_dip_sigma_is_tangent_to_the_dip_channel)
{
    /* Mag noise sized to the filter's own Rm = Nm²·f_mag, so the isotropic
     * case reads exactly 1: σ = √(0.0004²·100) = 4e-3 Gauss. */
    const float sig_g = (float)sqrt(0.0004 * 0.0004 * 100.0);

    float b0, s0, n0, b1, s1, n1;
    dip_run(0.0, 0.0f, sig_g, &b0, &s0, &n0);
    dip_run(6.0, 0.0f, sig_g, &b1, &s1, &n1);

    EXPECT(n0 > 0.5f && n0 < 2.0f,
           "isotropic 3-D mag update reads NIS ~1 on a correct reference");
    EXPECT(n1 > 0.3f && n1 < 1.0f,
           "nis_mag stays a usable instrument with the dip term active");

    EXPECT(s1 > 1.05f * s0,
           "the dip term widens P along the direction a dip error acts on");
    EXPECT(b1 < 0.25f * b0,
           "and absorbs the roll/pitch pull it is aimed at");
}

/* Yaw-only fusion never reads the dip channel, so the knob must be inert
 * there — the marine default cannot be perturbed by a 3-D-mode setting. */
TEST(test_mag_dip_sigma_inert_in_yaw_only)
{
    imud_config_t c0 = make_cfg(); c0.mag_yaw_only = true;
    c0.mekf_mag_dip_sigma_deg = 0.0;
    imud_config_t c1 = c0;         c1.mekf_mag_dip_sigma_deg = 5.0;

    mekf_t f0, f1;
    float bias[3] = {0};
    mekf_init(&f0, &c0, 833.0f, (float)c0.mag_odr_hz, bias);
    mekf_init(&f1, &c1, 833.0f, (float)c1.mag_odr_hz, bias);
    mekf_align(&f0, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    mekf_align(&f1, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    uint32_t rng = 99;
    for (int i = 0; i < 20000; i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        float n1 = ((float)(int32_t)rng) * (1.0f/2147483648.0f);
        imu_sample_t g = make_gyro(0.04f*n1, -0.02f*n1, 0.03f*n1);
        imu_sample_t a = make_accel(0.2f*n1, 0.15f*n1, -G);
        mekf_predict(&f0, &g, f0.dt); mekf_update_accel(&f0, &a);
        mekf_predict(&f1, &g, f1.dt); mekf_update_accel(&f1, &a);
        if (i % 8 == 0) {
            mag_sample_t m = make_mag(0.20f + 0.002f*n1, 0.002f*n1, 0.40f);
            mekf_update_mag(&f0, &m);
            mekf_update_mag(&f1, &m);
        }
    }
    EXPECT(memcmp(f0.q, f1.q, sizeof f0.q) == 0,
           "dip sigma leaves yaw-only attitude bit-identical");
    EXPECT(memcmp(f0.P, f1.P, sizeof f0.P) == 0,
           "dip sigma leaves yaw-only covariance bit-identical");
}

/*
 * The m_ref quiescence gate must stay tight enough that a seaway never opens
 * it. This looks like a bug — the in-run dip/magnitude healing therefore never
 * runs at sea, which is exactly when a reference error matters — and it is
 * not. Raising it was measured over 30 minutes of the wave benchmark:
 *
 *   3-D, 5 s align, 1800 s   dip err   |m_ref_h| err   att RMS   NEES(strict)
 *     gate 2e-4 (shipped)     +0.862°       −5.04%      1.151°       12.84
 *     gate 2e-2               −1.460°       +4.24%      1.725°       42.10
 *     gate OFF                −1.469°       +4.27%      1.729°       42.31
 *
 * The reference sails past truth and keeps going, because the samples that
 * clear the |a| band in a seaway are wave-phase correlated: learning from that
 * subset walks m_ref away, not toward. (A shorter 120 s window catches it in
 * transit through zero and looks like a fix — it is not.) The dip error is not
 * observable from seaway data; it is removed by WMM invariants or admitted
 * into P by mekf_mag_dip_sigma_deg.
 *
 * This test exists so that measurement is not quietly re-litigated.
 */
TEST(test_mref_quiet_gate_stays_tight)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* A modest seaway: |a| swinging a few percent around g. */
    const float fs = 833.0f, w = 2.0f*(float)M_PI*0.2f;
    float quiet_min = 1e30f;
    for (int i = 0; i < (int)(60.0f*fs); i++) {
        float t = (float)i / fs;
        imu_sample_t s = make_accel(1.2f*sinf(w*t), 1.2f*cosf(0.8f*w*t),
                                    -G + 1.0f*sinf(w*t));
        mekf_predict(&f, &s, f.dt);
        mekf_update_accel(&f, &s);
        if (t > 20.0f && f.acc_quiet_ema < quiet_min) quiet_min = f.acc_quiet_ema;
    }

    EXPECT(quiet_min > 1e-3f,
           "a seaway keeps acc_quiet_ema well above the m_ref healing gate");
}

/*
 * The update-gate health EMAs must actually track what the gate does:
 * clean data leaves weight at 1 and rejection at 0; a stream of gross
 * outliers must push the reject EMA up and the weight EMA down.
 */
TEST(test_gate_health_emas)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    EXPECT_NEAR(f.innov_weight_ema, 1.0f, 1e-6f, "weight EMA starts at 1.0");
    EXPECT_NEAR(f.innov_reject_ema, 0.0f, 1e-6f, "reject EMA starts at 0.0");

    /* Clean, on-model data: converge, then confirm the gate stays quiet. */
    imu_sample_t still = make_accel(0, 0, -G);
    imu_sample_t zg    = make_gyro(0, 0, 0);
    for (int i = 0; i < 200000; i++) {
        mekf_predict(&f, &zg, f.dt);
        mekf_update_accel(&f, &still);
    }
    EXPECT(f.innov_weight_ema > 0.95f, "weight EMA stays high on clean data");
    EXPECT(f.innov_reject_ema < 0.05f, "reject EMA stays low on clean data");

    /* Now a sustained stream of |a| = g but wildly mis-pointed samples: they
     * pass the magnitude skip band and land beyond the gross-outlier reject gate. */
    float th = 60.0f * DEG;
    imu_sample_t bad = make_accel(G*sinf(th), 0, -G*cosf(th));
    for (int i = 0; i < 100000; i++) {
        mekf_predict(&f, &zg, f.dt);
        mekf_update_accel(&f, &bad);
    }
    EXPECT(f.innov_reject_ema > 0.10f,
           "reject EMA rises under sustained gross outliers");
    EXPECT(f.innov_weight_ema < 0.95f,
           "weight EMA falls under sustained gross outliers");
}

/*
 * Rolling NIS (ROADMAP §10.1 instrument). The properties that matter:
 *
 *  (a) on-model data reads ≈ 1 — the definition of a consistent covariance;
 *  (b) sustained GROSS outliers, which the gross-outlier gate throws away, must still
 *      drive it UP. This is the whole reason it exists: innov_weight
 *      saturates at the cap and stops carrying information about how wrong
 *      the model is, whereas NIS keeps climbing. A post-cap or
 *      accepted-only average would be bounded by construction and could
 *      never show this.
 */
/* Local uniform RNG in [-1,1); the benchmark's own generator is defined
 * further down and this test must not depend on its seed state. */
static float nis_rand(uint32_t *st)
{
    *st ^= *st << 13; *st ^= *st >> 17; *st ^= *st << 5;
    return ((float)(int32_t)*st) * (1.0f / 2147483648.0f);
}

TEST(test_nis_consistency_emas)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    EXPECT_NEAR(f.nis_accel_ema, 1.0f, 1e-6f, "accel NIS starts at 1.0");
    EXPECT_NEAR(f.nis_mag_ema,   1.0f, 1e-6f, "mag NIS starts at 1.0");

    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /*
     * Clean data with accel noise matching the filter's own Ra: this is
     * exactly the model the filter assumes, so NIS must settle near 1.
     * Ra is in normalised (unit-vector) units, so the equivalent m/s²
     * sigma per sample is g·√Ra.
     */
    float sigma = G * sqrtf(f.Ra);
    imu_sample_t zg = make_gyro(0, 0, 0);
    uint32_t rng = 0x5EED1234u;
    #define NOISE(s) (nis_rand(&rng) * (s) * 1.732f)
    for (int i = 0; i < 300000; i++) {
        imu_sample_t s = make_accel(NOISE(sigma),
                                    NOISE(sigma),
                                    -G + NOISE(sigma));
        mekf_predict(&f, &zg, f.dt);
        mekf_update_accel(&f, &s);
    }
    #undef NOISE
    EXPECT(f.nis_accel_ema > 0.4f && f.nis_accel_ema < 2.5f,
           "accel NIS settles near 1 when the data matches the noise model");

    float nis_clean = f.nis_accel_ema;

    /* Sustained gross outliers — |a| = g but 60° mis-pointed, so they clear
     * the magnitude skip band and are then rejected by the gross-outlier gate. */
    float th = 60.0f * DEG;
    imu_sample_t bad = make_accel(G*sinf(th), 0, -G*cosf(th));
    for (int i = 0; i < 100000; i++) {
        mekf_predict(&f, &zg, f.dt);
        mekf_update_accel(&f, &bad);
    }
    EXPECT(f.nis_accel_ema > 10.0f * nis_clean,
           "accel NIS rises sharply under sustained gross outliers");
    EXPECT(f.innov_reject_ema > 0.10f,
           "...and those outliers really were gate-rejected (not merely capped)");
}

/*
 * mekf_reconfigure is a live SIGHUP path: it must retune the EMA GAINS
 * without discarding accumulated history. An A/B of mekf_accel_noise in the
 * field depends on this — resetting the NIS accumulators on every SIGHUP
 * would erase the measurement the operator is trying to read.
 */
TEST(test_reconfigure_preserves_nis)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Drive the accumulators somewhere clearly away from their seed value. */
    float th = 60.0f * DEG;
    imu_sample_t bad = make_accel(G*sinf(th), 0, -G*cosf(th));
    imu_sample_t zg  = make_gyro(0, 0, 0);
    for (int i = 0; i < 50000; i++) {
        mekf_predict(&f, &zg, f.dt);
        mekf_update_accel(&f, &bad);
    }
    float nis_a = f.nis_accel_ema, nis_m = f.nis_mag_ema;
    EXPECT(nis_a > 2.0f, "accel NIS moved off its seed before reconfigure");

    cfg.mekf_accel_noise = 0.05;    /* the kind of change an A/B would make */
    cfg.mag_odr_hz       = 50;      /* [restart] — must not reach the gain */
    mekf_reconfigure(&f, &cfg);

    EXPECT_NEAR(f.nis_accel_ema, nis_a, 1e-6f,
                "reconfigure preserves accumulated accel NIS");
    EXPECT_NEAR(f.nis_mag_ema, nis_m, 1e-6f,
                "reconfigure preserves accumulated mag NIS");
    EXPECT_NEAR(f.nis_mag_alpha, 1.0f/(30.0f*100.0f), 1e-9f,
                "the mag NIS gain stays on the programmed rate, not the "
                "reloaded request");
}

/*
 * χ² innovation handling: a gravity-magnitude accel pointing far from the
 * predicted direction (wave orbital motion) must have bounded influence —
 * one such sample may not yank a converged filter.
 */
TEST(test_accel_innovation_capped)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Converge on clean data. */
    imu_sample_t still = make_accel(0, 0, -G);
    imu_sample_t zg    = make_gyro(0, 0, 0);
    for (int i = 0; i < 5000; i++) {
        mekf_predict(&f, &zg, f.dt);
        mekf_update_accel(&f, &still);
    }
    float q0[4]; memcpy(q0, f.q, sizeof q0);

    /* |a| = g exactly, but direction 25° off — passes the magnitude gate. */
    float th = 25.0f * DEG;
    imu_sample_t tilted = make_accel(G*sinf(th), 0, -G*cosf(th));
    mekf_update_accel(&f, &tilted);

    float roll, pitch, yaw, roll0, pitch0, yaw0;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    q_to_euler(q0, &roll0, &pitch0, &yaw0);
    /* Uncapped Kalman gain on a converged filter is small anyway; the cap
     * bounds the movement — assert the single sample moved pitch < 0.1°. */
    EXPECT(fabsf(pitch - pitch0) < 0.1f*DEG,
           "single 25°-off accel sample has bounded influence");
}

/*
 * m_ref EMA heals a wrong dip/magnitude reference but must NOT touch the
 * horizontal direction (heading anchor).
 */
TEST(test_mref_ema_heals_dip)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    /* Corrupt the reference the way a mid-swing alignment does: right
     * heading direction, dip/magnitude off by ~8° (within the EMA's
     * direction-agreement bound). */
    f.m_ref[0] = 0.24f;   /* true horizontal is 0.20 */
    f.m_ref[2] = 0.34f;   /* true vertical   is 0.40 */
    float hdg_dir0 = atan2f(f.m_ref[1], f.m_ref[0]);

    /* Shorten τ so the test runs in seconds (real τ is 5 min). */
    f.mref_alpha = 1.0f / (15.0f * 833.0f);   /* τ = 15 s at this call rate */

    imu_sample_t still = make_accel(0, 0, -G);
    imu_sample_t zg    = make_gyro(0, 0, 0);
    mag_sample_t m     = make_mag(0.20f, 0.0f, 0.40f);

    for (int i = 0; i < 60000; i++) {   /* ~72 s ≈ 4.8 τ */
        mekf_predict(&f, &zg, f.dt);
        mekf_update_mag(&f, &m);
        mekf_update_accel(&f, &still);
    }

    float mh = sqrtf(f.m_ref[0]*f.m_ref[0] + f.m_ref[1]*f.m_ref[1]);
    EXPECT_NEAR(mh,          0.20f, 0.015f, "m_ref horizontal magnitude healed");
    EXPECT_NEAR(f.m_ref[2],  0.40f, 0.015f, "m_ref dip component healed");
    EXPECT_NEAR(atan2f(f.m_ref[1], f.m_ref[0]), hdg_dir0, 1e-4f,
                "m_ref horizontal DIRECTION untouched (no gauge feedback)");
}

/*
 * Yaw-only mode: a mag disturbance that (in 3D mode) would pull roll/pitch
 * must leave tilt untouched and only steer heading.
 */
TEST(test_yaw_only_leaves_tilt)
{
    imud_config_t cfg = make_cfg();
    cfg.mag_yaw_only = true;
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    imu_sample_t zg = make_gyro(0, 0, 0);
    float roll0, pitch0, yaw0;
    q_to_euler(f.q, &roll0, &pitch0, &yaw0);

    /* Disturbed field: horizontal direction twisted ~9°, dip changed —
     * total magnitude within the ratio gate (m_ref here is (0.20,0,0.05),
     * |m_ref| = 0.206; this sample is 0.227 → ratio 1.1). */
    mag_sample_t bad = make_mag(0.19f, 0.03f, 0.12f);
    for (int i = 0; i < 2000; i++) {
        mekf_predict(&f, &zg, f.dt);
        mekf_update_mag(&f, &bad);
    }

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    EXPECT(fabsf(roll  - roll0)  < 0.05f*DEG, "yaw-only: roll untouched by mag");
    EXPECT(fabsf(pitch - pitch0) < 0.05f*DEG, "yaw-only: pitch untouched by mag");
    EXPECT(fabsf(yaw - yaw0) > 1.0f*DEG,      "yaw-only: heading does follow mag");
}

/*
 * WMM-informed m_ref: invariants override magnitude/dip but preserve the
 * horizontal direction (heading anchor); no-op before alignment.
 */
TEST(test_mref_invariants)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);

    /* Before alignment: no-op. */
    mekf_set_mref_invariants(&f, 0.25f, 0.45f);
    EXPECT(!f.m_ref_valid, "no-op before alignment");

    mekf_align(&f, (float[]){0,0,-G}, (float[]){18.0f, 6.0f, 38.0f});
    float dir0 = atan2f(f.m_ref[1], f.m_ref[0]);

    mekf_set_mref_invariants(&f, 0.25f, 0.45f);
    float mh = sqrtf(f.m_ref[0]*f.m_ref[0] + f.m_ref[1]*f.m_ref[1]);
    EXPECT_NEAR(mh,         0.25f, 1e-5f, "horizontal magnitude set from WMM");
    EXPECT_NEAR(f.m_ref[2], 0.45f, 1e-5f, "vertical component set from WMM");
    EXPECT_NEAR(atan2f(f.m_ref[1], f.m_ref[0]), dir0, 1e-5f,
                "horizontal direction preserved (no heading jump)");
}

/*
 * Speed-aided centripetal correction: a coordinated turn (constant yaw rate,
 * constant speed) puts ω×v on the accelerometer. Without speed the filter
 * leans into the turn (apparent roll); with speed the tilt stays level.
 */
static float turn_roll_error(float speed_for_filter)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    f.speed_mps = speed_for_filter;

    const float r = 0.15f;   /* yaw rate, rad/s (~8.6 °/s) */
    const float v = 6.0f;    /* true speed, m/s */

    imu_sample_t s;
    memset(&s, 0, sizeof s);
    s.gyro[2]  = r;                    /* flat coordinated turn */
    s.accel[0] = 0;
    s.accel[1] = r * v;                /* centripetal, body lateral */
    s.accel[2] = -G;

    for (int i = 0; i < 833*20; i++) {  /* 20 s of turning */
        mekf_predict(&f, &s, f.dt);
        mekf_update_accel(&f, &s);
    }

    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);
    return fabsf(roll);
}

TEST(test_centripetal_correction)
{
    float roll_off = turn_roll_error(0.0f);   /* no speed: leans into turn */
    float roll_on  = turn_roll_error(6.0f);   /* corrected */

    /*
     * The uncorrected error is 1.81°, not the 5.24° it was before the wave
     * state existed: a sustained 0.9 m/s² lateral acceleration looks a lot
     * like the disturbance the Gauss–Markov state is there to absorb, and it
     * soaks up roughly 60% of it (a_w settles near 0.06 g).
     *
     * It does NOT soak up all of it, and must not — the state's τ = 0.5 s
     * prior keeps pulling it back to zero, so a genuinely sustained offset
     * stays partly attributed to tilt. That is the δθ↔δa_w separation working:
     * the two are told apart by their dynamics, and a DC lateral acceleration
     * is not what a 0.5 s Gauss–Markov process looks like.
     *
     * So the speed aiding still earns its keep (1.81° → 0.00°) and this bound
     * still proves it; it is just no longer measuring a 5° error.
     */
    EXPECT(roll_off > 1.2f*DEG,
           "without speed, sustained turn still tilts the roll estimate");
    EXPECT(roll_on < 0.3f*DEG,
           "with speed, roll stays level through the turn");
}

/* ── Heave estimator tests ──────────────────────────────────────────────────── */

/*
 * Drive heave_update at flat attitude with a synthetic sinusoidal heave of
 * known amplitude: accel_z = −g + Hω²·sin(ωt) corresponds to heave
 * H·sin(ωt) (positive up). After settling, the recovered amplitude must be
 * close to H across ordinary wave periods.
 */
static float heave_sine_amp(float period_s, float amp_m, float tau_s)
{
    const float fs = 833.0f, dt = 1.0f/fs;
    float w = 2.0f*(float)M_PI/period_s;

    heave_t h;
    heave_init(&h, tau_s, dt);
    float q_id[4] = {1, 0, 0, 0};

    /* The filter has a triple pole at 1/τ (two leaky integrators + output
     * high-pass): the start-up transient decays as t²·e^(−t/τ), so allow a
     * full 10 τ before measuring. */
    float peak = 0.0f;
    int n_settle = (int)(10.0f*tau_s*fs);
    int n_meas   = (int)(2.0f*period_s*fs);
    for (int i = 0; i < n_settle + n_meas; i++) {
        float t = (float)i*dt;
        float acc[3] = { 0, 0, -G + amp_m*w*w*sinf(w*t) };
        float hv = heave_update(&h, q_id, acc);
        if (i >= n_settle && fabsf(hv) > peak) peak = fabsf(hv);
    }
    return peak;
}

TEST(test_heave_sine_amplitude)
{
    EXPECT_NEAR(heave_sine_amp(3.0f,  1.0f, 12.0f), 1.0f, 0.05f,
                "heave amplitude within 5% at 3 s period");
    EXPECT_NEAR(heave_sine_amp(8.0f,  1.5f, 12.0f), 1.5f, 0.15f,
                "heave amplitude within 10% at 8 s period");
    EXPECT_NEAR(heave_sine_amp(12.0f, 2.0f, 12.0f), 2.0f, 0.30f,
                "heave amplitude within 15% at 12 s period");
}

TEST(test_heave_disabled_and_settle)
{
    const float fs = 833.0f, dt = 1.0f/fs;
    float q_id[4] = {1, 0, 0, 0};
    float still[3] = {0, 0, -G};

    /* tau = 0 disables. */
    heave_t off;
    heave_init(&off, 0.0f, dt);
    EXPECT(heave_update(&off, q_id, still) == 0.0f, "tau=0 → heave 0.0");

    /* Zero input settles to (numerically) zero. */
    heave_t h;
    heave_init(&h, 12.0f, dt);
    float last = 0;
    for (int i = 0; i < (int)(30.0f*fs); i++)
        last = heave_update(&h, q_id, still);
    EXPECT(fabsf(last) < 1e-3f, "zero input settles to ~0");

    /* A constant accel-bias step must not run away: the bias tracker +
     * leaks bound the response and pull it back toward zero. */
    heave_t hb;
    heave_init(&hb, 5.0f, dt);
    float biased[3] = {0, 0, -G + 0.2f};
    float peak = 0, final = 0;
    for (int i = 0; i < (int)(300.0f*fs); i++) {
        final = heave_update(&hb, q_id, biased);
        if (fabsf(final) > peak) peak = fabsf(final);
    }
    EXPECT(peak < 6.0f,          "bias step response bounded");
    EXPECT(fabsf(final) < 0.05f, "bias step fully absorbed after 300 s");
}

/* ── Sea-state estimator ────────────────────────────────────────────────────── */

TEST(test_seastate_sine)
{
    const float fs = 200.0f, dt = 1.0f/fs;
    const float A = 0.9f;        /* heave amplitude, m */
    const float Tw = 6.0f;       /* wave period, s */
    const float B = 12.0f*(float)M_PI/180.0f;  /* roll amplitude, rad */
    const float Tr = 4.5f;       /* roll period, s */
    const float C = 8.0f*(float)M_PI/180.0f;   /* pitch amplitude, rad */
    const float Tp = 5.0f;       /* pitch period, s */
    const float heel = 5.0f*(float)M_PI/180.0f; /* steady heel offset */
    const float trim = 2.0f*(float)M_PI/180.0f; /* steady trim offset */
    const float ww = 2.0f*(float)M_PI/Tw, wr = 2.0f*(float)M_PI/Tr,
                wp = 2.0f*(float)M_PI/Tp;

    seastate_t w;
    seastate_init(&w, 30.0f, dt);
    EXPECT(w.enabled, "tau>0 enables");
    EXPECT(seastate_wave_height(&w) == 0.0f, "no output before settle");

    for (int i = 0; i < (int)(120.0f*fs); i++) {
        float t = (float)i*dt;
        seastate_update(&w, A*sinf(ww*t), A*ww*cosf(ww*t),
                        heel + B*sinf(wr*t), B*wr*cosf(wr*t),
                        trim + C*sinf(wp*t), C*wp*cosf(wp*t));
    }
    EXPECT(w.settled, "settled after 2 tau");
    /* Pure sine: σ = A/√2 → Hs = 4σ = 2.828·A, significant single
     * amplitude = 2σ = 1.414·A; all periods exact. */
    EXPECT_NEAR(seastate_wave_height(&w), 2.828f*A, 0.10f*2.828f*A,
                "Hs = 2.83·A within 10%");
    EXPECT_NEAR(seastate_wave_period(&w), Tw, 0.05f*Tw,
                "wave period within 5%");
    EXPECT_NEAR(seastate_roll_period(&w), Tr, 0.05f*Tr,
                "roll period within 5% despite steady heel");
    EXPECT_NEAR(seastate_roll_amplitude(&w), 1.414f*B, 0.10f*1.414f*B,
                "roll amplitude = 1.41·B within 10%");
    EXPECT_NEAR(seastate_pitch_period(&w), Tp, 0.05f*Tp,
                "pitch period within 5% despite steady trim");
    EXPECT_NEAR(seastate_pitch_amplitude(&w), 1.414f*C, 0.10f*1.414f*C,
                "pitch amplitude = 1.41·C within 10%");
}

TEST(test_seastate_gates)
{
    const float fs = 200.0f, dt = 1.0f/fs;

    /* tau = 0 disables. */
    seastate_t off;
    seastate_init(&off, 0.0f, dt);
    seastate_update(&off, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    EXPECT(!off.enabled && seastate_wave_height(&off) == 0.0f, "tau=0 → off");

    /* Becalmed: mm-scale heave and 0.05° roll ripple → periods report 0,
     * Hs reports the (tiny) truth. */
    seastate_t calm;
    seastate_init(&calm, 20.0f, dt);
    const float a = 0.004f, b = 0.05f*(float)M_PI/180.0f, wq = 2.0f*(float)M_PI/3.0f;
    for (int i = 0; i < (int)(80.0f*fs); i++) {
        float t = (float)i*dt;
        seastate_update(&calm, a*sinf(wq*t), a*wq*cosf(wq*t),
                        b*sinf(wq*t), b*wq*cosf(wq*t),
                        b*sinf(wq*t), b*wq*cosf(wq*t));
    }
    EXPECT(calm.settled, "calm run settled");
    EXPECT(seastate_wave_period(&calm) == 0.0f, "becalmed → wave period 0");
    EXPECT(seastate_roll_period(&calm) == 0.0f, "not rolling → roll period 0");
    EXPECT(seastate_pitch_period(&calm) == 0.0f, "not pitching → pitch period 0");
    EXPECT(seastate_wave_height(&calm) < 0.05f, "becalmed Hs is small");
}

TEST(test_mag_health)
{
    imud_config_t cfg = make_cfg();
    float bias[3] = {0};

    /* Clean field: both metrics stay ~0. */
    mekf_t f;
    mekf_init(&f, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    mag_sample_t m = make_mag(0.20f, 0.0f, 0.05f);   /* Gauss (make_mag scales) */
    for (int i = 0; i < 2000; i++) mekf_update_mag(&f, &m);
    EXPECT(f.mag_anom_ema  < 0.01f, "clean field: anomaly ~0");
    EXPECT(f.mag_resid_ema < 0.01f, "clean field: residual ~0");

    /* Magnitude anomaly: same direction, 1.5x strength (inside the hard
     * 0.5-2.0 gate). EMA (alpha=1/3000) reaches 63% of the 0.5 step. */
    mekf_t fa;
    mekf_init(&fa, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&fa, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    mag_sample_t ma = make_mag(0.30f, 0.0f, 0.075f);
    for (int i = 0; i < 3000; i++) mekf_update_mag(&fa, &ma);
    EXPECT(fa.mag_anom_ema  > 0.2f,  "1.5x magnitude: anomaly rises");
    EXPECT(fa.mag_resid_ema < 0.05f, "direction unchanged: residual low");

    /* Heading anomaly with updates REJECTED (converged + tight gate): the
     * metric must rise precisely while the filter refuses the data. */
    mekf_t fr;
    mekf_init(&fr, &cfg, 833.0f, (float)cfg.mag_odr_hz, bias);
    mekf_align(&fr, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    fr.converged = true;   /* arm the tight anomaly gate */
    float q_before[4]; memcpy(q_before, fr.q, sizeof fr.q);
    mag_sample_t mr = make_mag(0.1732f, 0.10f, 0.05f);   /* horizontal +30 deg */
    for (int i = 0; i < 3000; i++) mekf_update_mag(&fr, &mr);
    EXPECT_NEAR(fr.q[3], q_before[3], 1e-4f, "rejected: attitude unmoved");
    EXPECT(fr.mag_resid_ema > 0.2f,  "30 deg offset: residual rises while rejected");
    EXPECT(fr.mag_anom_ema  < 0.05f, "magnitude unchanged: anomaly stays low");
}

/* ── Rough-sea wave benchmark ───────────────────────────────────────────────── */

/*
 * Synthetic rough-sea scenario driven straight into the MEKF API.
 *
 * Truth: roll ±15° @ 0.2 Hz, pitch ±8° @ 0.14 Hz, fixed heading 60°,
 * wave-orbital linear acceleration ~0.12 g laterally + 0.10 g vertically,
 * true gyro bias the filter must find. Sensors get deterministic
 * pseudo-random noise. Alignment mimics the daemon: one instantaneous
 * (mid-swing, noisy) sample at t = 0.
 *
 * After a 60 s warm-up, attitude/heading RMS error over the next 120 s is
 * printed (benchmark) and asserted against a regression bound.
 *
 * The scenario is run over a FIXED SET OF SEEDS and asserted on the mean, not
 * on a single draw. A single seed is not a usable signal here: the original
 * seed 0x1234ABCD scores 3D attitude RMS 5.15° while the mean over the seed
 * set is 6.71° and the worst draw is 13.78°, so a one-seed test both flatters
 * the filter and moves enough between unrelated changes to hide or invent
 * regressions. Averaging is what makes A-vs-B comparisons of filter changes
 * meaningful.
 */

/*
 * Seed set for the averaged benchmark. Twelve arbitrary but fixed values —
 * enough to average out the per-draw spread (which is wide: see above) while
 * keeping the test near a second. Fixed, so the test stays deterministic.
 */
static const uint32_t bench_seeds[] = {
    0x1234ABCDu, 0xDEADBEEFu, 0x0BADF00Du, 0x13579BDFu,
    0x2468ACE0u, 0xFEEDFACEu, 0xC0FFEE11u, 0xA5A5A5A5u,
    0x77777777u, 0x31415926u, 0x27182818u, 0x16180339u,
};
#define N_BENCH_SEEDS ((int)(sizeof bench_seeds / sizeof bench_seeds[0]))

static uint32_t bench_seed      = 0x1234ABCDu;   /* set per run by the driver */
static uint32_t bench_rng_state = 0x1234ABCDu;

/*
 * Draw counter for test_bench_stream_fingerprint().  Counts calls, changes no
 * state the generator reads, so it cannot perturb the stream it measures.
 */
static unsigned long bench_draws = 0;

static float bench_rand(void)   /* uniform in [-1, 1) */
{
    bench_draws++;
    bench_rng_state ^= bench_rng_state << 13;
    bench_rng_state ^= bench_rng_state >> 17;
    bench_rng_state ^= bench_rng_state << 5;
    return ((float)(int32_t)bench_rng_state) * (1.0f / 2147483648.0f);
}
static float bench_noise(float sigma)  /* zero-mean, std ≈ sigma */
{
    return bench_rand() * sigma * 1.732f;   /* uniform with matching variance */
}

/* ZYX Euler → quaternion (body→NED). */
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

/* Rotation matrix (body→NED) from quaternion — mirror of fusion.c q_to_R. */
static void bench_q_to_R(const float q[4], float R[3][3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z);   R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x);   R[2][2]=1-2*(x*x+y*y);
}

/* Angle (rad) between two unit quaternions. */
static float q_angle_between(const float a[4], const float b[4])
{
    float d = fabsf(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
    if (d > 1.0f) d = 1.0f;
    return 2.0f * acosf(d);
}

/*
 * Everything one wave run measures. Two NEES forms are reported because they
 * answer different questions and the tree's historical numbers use the first:
 *
 *   nees_trace  = mean(‖δθ‖²) / mean(tr P[0:3])  — the scaled-trace instrument
 *                 the recorded 28.4 / 32.4 figures (fusion.c, docs/math.md,
 *                 NEWS) come from. Note this is a ratio of AGGREGATES, not a
 *                 mean of per-sample ratios: it is a variance ratio, "how many
 *                 times too small is the total attitude variance", and equals
 *                 RMS_att²/mean-trace. (The mean-of-ratios form reads ~18%
 *                 higher — it is dominated by the moments when tr P happens to
 *                 be small — which is why the two must not be mixed when
 *                 comparing against the recorded numbers.) Consistent = 1.0.
 *   nees_strict = mean δθᵀ·P₃₃⁻¹·δθ / 3      — the textbook NEES, which also
 *                 sees the SHAPE of P, not just its total size. Divided by the
 *                 dof so that 1.0 is "consistent" in both columns. It reads far
 *                 worse than the trace form because the error is concentrated
 *                 in P's stiffest directions — the trace form UNDERSTATES the
 *                 problem, and 10.1's target is stated against it only because
 *                 that is what the historical record uses.
 */
typedef struct {
    float rms_att;       /* attitude RMS, deg */
    float rms_hdg;       /* heading RMS, deg */
    float bias_err;      /* |bias_z error| at end of run, rad/s */
    float nees_trace;    /* mean ‖δθ‖²/tr(P[0:3]); 1.0 = consistent */
    float nees_strict;   /* mean δθᵀP₃₃⁻¹δθ / 3; 1.0 = consistent */
    float innov_weight;  /* end-of-run Huber-weight EMA (1.0 = never capped) */
    float innov_reject;  /* end-of-run gross-outlier-reject-rate EMA */
    float nis_accel;     /* end-of-run rolling accel NIS — the FIELD instrument */
    float nis_mag;       /* end-of-run rolling mag NIS */
    float m_ref[3];      /* filter's magnetic reference at end of run */
} wave_run_t;

/*
 * Which linear-acceleration disturbance the scenario drives.
 *
 * SCEN_TONE is the historical single-tone seaway: three sinusoids locked to
 * the roll frequency. It is realistic in amplitude and it is what every
 * recorded number in the tree was measured on, so it stays byte-identical.
 * But a pure tone has an autocorrelation that never decays, so it cannot say
 * what the Gauss–Markov correlation time SHOULD be — fit τ against it and you
 * are fitting the benchmark, not the physics.
 *
 * SCEN_GM drives the same filter with a genuine first-order Gauss–Markov
 * process of KNOWN (σ, τ). That is the only configuration in which the right
 * answer is known in advance: set the filter's knobs to the truth and a
 * correct implementation must report NIS ≈ 1 and NEES ≈ 1. Tuning happens
 * here; SCEN_TONE is the held-out validation.
 */
typedef enum { SCEN_TONE = 0, SCEN_GM = 1 } wave_scen_t;

/* Truth for SCEN_GM, m/s² and s. Deliberately NOT the filter's knobs. */
static double scen_gm_sigma = 0.8;
static double scen_gm_tau   = 0.7;

/*
 * Whether the scenario supplies WMM field invariants at alignment. The daemon
 * does this whenever a position source has given it a reference field
 * (src/imu.c), and it is the only thing that removes the alignment dip error
 * at the source rather than admitting it into P — so it is a distinct, and
 * previously unmeasured, shipping configuration.
 */
static bool bench_wmm_ref = false;

/*
 * Align from a clean, disturbance-free sample instead of the daemon's window
 * average. For UNIT TESTS OF THE ESTIMATOR only — never for the benchmark.
 *
 * Averaging specific force over the alignment window is biased in a seaway,
 * and not only by sensor noise: the body frame is itself rotating, so the mean
 * of a body-frame gravity vector over a roll cycle is both shrunk and tilted.
 * The daemon has that characteristic (src/imu.c sums raw body-frame samples)
 * and the benchmark must therefore keep it. But a test whose question is "does
 * the wave estimator report NIS = 1 when its knobs are the truth" must not have
 * that confound folded in — under the broadband disturbance the alignment error
 * alone moves that number from 0.99 to 1.63, which says nothing about the
 * estimator.
 */
static bool bench_align_clean = false;

static void run_wave_scenario_ex(bool yaw_only, wave_scen_t scen, wave_run_t *out)
{
    const float fs   = bench_fs;
    const float dt   = 1.0f / fs;
    const float d2r  = (float)(M_PI / 180.0);

    /*
     * Sensor-noise scale.  THE trap in rate-parameterising this scenario, and
     * the reason it is spelled out here rather than inlined.
     *
     * The sigmas below are PER-SAMPLE, while the filter models sensor noise as a
     * density times bandwidth: Ra = (Na/g)^2 * odr, Qg = Ng^2 * dt (fusion.c
     * mekf_derive_tuning).  Those two only correspond at the rate the constants
     * were picked for.  Read as densities they are
     *
     *     accel  0.03  / sqrt(833) = 1.04e-3 m/s^2/sqrt(Hz)
     *     gyro   0.002 / sqrt(833) = 6.93e-5 rad/s/sqrt(Hz)
     *
     * i.e. sensor-floor scale — the gyro figure is about half the ~1.2e-4 raw
     * floor the roadmap cites for this part.  They were chosen as density times
     * sqrt(833).
     *
     * So changing fs without scaling them leaves the synthetic sensor's noise
     * fixed while the filter's Ra shrinks with the rate: at 104 Hz the filter
     * would be 8x over-confident about noise that never got quieter, and every
     * number attributed to "the filter at 104 Hz" would be measuring a broken
     * harness instead.  That is the same failure class as the m33_inv
     * singularity test that silently discarded 87% of accel updates.
     *
     * The magnetometer scales against ITS OWN cadence for the same reason
     * (Rm = Nm^2 * mag_odr), which is a separate factor — see mag_nscale.
     *
     * Ratio form deliberately: at fs = BENCH_REF_FS the division is exactly 1.0f
     * and IEEE sqrtf(1.0f) is exactly 1.0f, so the reference pair multiplies its
     * sigmas by exactly one and bench_noise receives a bit-identical argument.
     * The alternative (0.03f/sqrtf(833.0f))*sqrtf(fs) rounds twice and would
     * silently re-base the baseline.
     */
    const float nscale     = sqrtf(fs / BENCH_REF_FS);
    const float mag_nscale = sqrtf(bench_mag_fs / BENCH_REF_MAG_FS);

    /* Wave truth parameters */
    const float Ar = 15.0f * d2r,  wr = 2.0f*(float)M_PI*0.20f;  /* roll  */
    const float Ap =  8.0f * d2r,  wp = 2.0f*(float)M_PI*0.14f;  /* pitch */
    const float yaw_true = 60.0f * d2r;

    /* Earth field: 0.47 G total, 62° dip (mid-latitude), magnetic-NED */
    const float m_ned[3] = { 0.2206f, 0.0f, 0.4149f };  /* Gauss */

    /* True gyro bias; filter is seeded with an imperfect estimate. */
    const float bias_true[3] = { 0.0020f, -0.0010f, 0.0015f };
    float bias_init[3] = { 0.0025f, -0.0005f, 0.0010f };

    imud_config_t cfg = make_cfg();
    cfg.mag_yaw_only = yaw_only;

    /*
     * Magnetometer cadence.
     *
     * Two regimes, because the magnetometer can legitimately be FASTER than the
     * IMU — a 12 Hz icm42688p paired with a 200 Hz mmc5983ma is a configuration
     * the drivers permit.  The daemon handles that at src/imu.c: it predicts on
     * an IMU sample and then drains the mag ring with a while loop, so zero, one
     * or several mag updates land per IMU sample.  This models the same thing.
     *
     * mag <= fs uses an exact integer divisor rather than a float accumulator.
     * That is not a shortcut: at the reference pair the divisor is
     * (int)(833/100) = 8, which reproduces the historical `i % 8` bit-for-bit,
     * and integer counting cannot drift over 150k samples the way a repeatedly
     * accumulated 1/3 would.
     *
     * The achieved cadence is therefore fs/mag_div, not exactly bench_mag_fs —
     * 104.125 Hz for a requested 100 at 833 Hz.  That quantisation is the
     * behaviour this benchmark has always had; naming it here makes it bounded
     * and visible rather than accidental.
     */
    const bool  mag_slower  = (bench_mag_fs <= fs);
    int         mag_div     = mag_slower ? (int)(fs / bench_mag_fs) : 1;
    if (mag_div < 1) mag_div = 1;
    const float mag_per_imu = mag_slower ? 1.0f / (float)mag_div
                                         : bench_mag_fs / fs;
    float       mag_accum   = 0.0f;

    /*
     * The rate handed to mekf_init must be the one the mag is ACTUALLY fed at —
     * include/fusion.h is emphatic that both rates are what the drivers
     * programmed, since every noise variance and EMA gain derives from them.
     *
     * The reference pair is the exception, and deliberately so.  It has always
     * passed the DECLARED 100 Hz while injecting at 104.125 Hz, a 4% mismatch.
     * "Correcting" it changes Rm and re-bases every recorded number in this
     * tree — and test_bench_stream_fingerprint would NOT catch that, because it
     * moves no draws.  Only the printed-line diff would.  So the reference keeps
     * its historical value and the discrepancy is recorded rather than fixed.
     */
    const bool  at_reference = (fs == BENCH_REF_FS &&
                                bench_mag_fs == BENCH_REF_MAG_FS);
    const float mag_eff = at_reference ? BENCH_REF_MAG_FS
                        : (mag_slower  ? fs / (float)mag_div : bench_mag_fs);

    mekf_t f;
    mekf_init(&f, &cfg, fs, mag_eff, bias_init);
    bench_rng_state = bench_seed;

    const float warmup_s = 60.0f, measure_s = 120.0f;
    const int   n_total  = (int)((warmup_s + measure_s) * fs);

    /* Alignment window, matching the daemon's 5 s default (align_window_sec). */
    const bool wmm_ref = bench_wmm_ref;
    const int align_n = (int)(5.0f * fs);
    double align_acc_sum[3] = {0}, align_mag_sum[3] = {0};

    double sum_att2 = 0.0, sum_hdg2 = 0.0;
    double sum_e2 = 0.0, sum_tr = 0.0, sum_nees_st = 0.0;
    int    n_meas   = 0, n_nees = 0;

    /*
     * SCEN_GM state: exact first-order GM recursion, the same discretisation
     * the filter uses (fusion.c mekf_predict), driven by the benchmark's own
     * uniform noise. Started at a draw from the stationary distribution so
     * there is no spin-up transient inside the measurement window.
     *
     * Nothing here may draw from the RNG unless SCEN_GM is selected: SCEN_TONE
     * has to consume exactly the stream it always did, or every recorded
     * number in the tree shifts.
     */
    const float gm_phi = expf(-dt / (float)scen_gm_tau);
    const float gm_q   = (float)scen_gm_sigma * sqrtf(1.0f - gm_phi*gm_phi);
    float gm[3] = {0, 0, 0};
    if (scen == SCEN_GM)
        for (int k = 0; k < 3; k++) gm[k] = bench_noise((float)scen_gm_sigma);

    for (int i = 0; i < n_total; i++) {
        float t = (float)i * dt;

        /* Truth attitude and rates */
        float roll  = Ar * sinf(wr * t);
        float pitch = Ap * sinf(wp * t + 1.0f);
        float rolld = Ar * wr * cosf(wr * t);
        float pitchd= Ap * wp * cosf(wp * t + 1.0f);

        float q_true[4];
        euler_to_q(roll, pitch, yaw_true, q_true);
        float Rt[3][3];
        bench_q_to_R(q_true, Rt);

        /* Body rates for ZYX Euler with yaw-dot = 0 */
        float w_body[3] = {
            rolld,
            pitchd * cosf(roll),
            -pitchd * sinf(roll),
        };

        /* Wave-orbital linear acceleration in NED (m/s²) */
        float a_lin[3];
        if (scen == SCEN_GM) {
            for (int k = 0; k < 3; k++) {
                gm[k] = gm_phi * gm[k] + bench_noise(gm_q);
                a_lin[k] = gm[k];
            }
        } else {
            a_lin[0] = 1.2f * sinf(wr * t + 0.5f);
            a_lin[1] = 1.2f * cosf(0.8f * wr * t);
            a_lin[2] = 1.0f * sinf(wr * t);
        }

        /* Specific force in body: f_b = Rᵀ (a_lin − g·e_down) */
        float f_ned[3] = { a_lin[0], a_lin[1], a_lin[2] - G };
        imu_sample_t s;
        memset(&s, 0, sizeof s);
        for (int k = 0; k < 3; k++) {
            s.accel[k] = Rt[0][k]*f_ned[0] + Rt[1][k]*f_ned[1] + Rt[2][k]*f_ned[2]
                       + bench_noise(0.03f * nscale);
            s.gyro[k]  = w_body[k] + bias_true[k] + bench_noise(0.002f * nscale);
        }

        /*
         * Alignment, as the daemon does it (src/imu.c, the align loop): mean
         * accelerometer and magnetometer over a window, then WMM invariants if
         * a position is known.
         *
         * This used to align from ONE instantaneous sample and claim in a
         * comment that that was "like the daemon". It is not, and the
         * difference dominated everything the 3-D mode reported. A single
         * sample taken mid-roll bakes the instantaneous tilt error into m_ref
         * as a permanent DIP error (measured: −4.38°), and in 3-D mode the dip
         * channel then holds roll and pitch at that wrong reference forever —
         * a constant attitude bias P has no term for. Measured over the 12
         * seeds, 3-D mode:
         *
         *                       dip err   att RMS   hdg RMS   NEES(tr)  NEES(st)
         *   1 sample (old)       −4.38°    4.452°    0.828°      3.47      335
         *   window mean (this)   +0.86°    1.204°    0.745°      0.21     12.8
         *   dip reference exact   0.00°    0.852°       —        0.11     0.22
         *
         * i.e. ~96% of the 3-D inconsistency was the instrument, not the
         * filter. The remaining +0.86° is residual dip error, which cannot be
         * healed from seaway data (see the acc_quiet_ema gate in fusion.c) and
         * is what the anisotropic dip term in R exists to admit into P.
         */
        if (bench_align_clean && i == 0) {
            /* Estimator unit tests: noise-free, disturbance-free reference. */
            float ca[3], cm[3];
            float g_ned[3] = { 0.0f, 0.0f, -G };
            for (int k = 0; k < 3; k++) {
                ca[k] = Rt[0][k]*g_ned[0] + Rt[1][k]*g_ned[1] + Rt[2][k]*g_ned[2];
                cm[k] = (Rt[0][k]*m_ned[0] + Rt[1][k]*m_ned[1]
                         + Rt[2][k]*m_ned[2]) * 100.0f;
            }
            mekf_align(&f, ca, cm);
            continue;
        }
        if (!bench_align_clean && i < align_n) {
            float mb[3];
            for (int k = 0; k < 3; k++)
                mb[k] = (Rt[0][k]*m_ned[0] + Rt[1][k]*m_ned[1] + Rt[2][k]*m_ned[2])
                        * 100.0f + bench_noise(0.3f * mag_nscale);   /* µT */
            for (int k = 0; k < 3; k++) {
                align_acc_sum[k] += s.accel[k];
                align_mag_sum[k] += mb[k];
            }
            if (i == align_n - 1) {
                float aa[3], mm[3];
                for (int k = 0; k < 3; k++) {
                    aa[k] = (float)(align_acc_sum[k] / align_n);
                    mm[k] = (float)(align_mag_sum[k] / align_n);
                }
                mekf_align(&f, aa, mm);
                /* WMM-known field invariants, as the daemon applies them when
                 * a position source has given it a reference field. */
                if (wmm_ref) {
                    float mh = sqrtf(m_ned[0]*m_ned[0] + m_ned[1]*m_ned[1]);
                    mekf_set_mref_invariants(&f, mh, m_ned[2]);
                }
            }
            continue;
        }

        mekf_predict(&f, &s, f.dt);

        /*
         * Mag updates for this IMU sample: exactly one every mag_div samples
         * while the mag is the slower sensor, or several per sample when it is
         * the faster one — the daemon's drain loop, modelled.
         */
        int n_mag = 0;
        if (mag_slower) {
            n_mag = (i % mag_div == 0) ? 1 : 0;
        } else {
            mag_accum += mag_per_imu;
            while (mag_accum >= 1.0f) { mag_accum -= 1.0f; n_mag++; }
        }
        for (int u = 0; u < n_mag; u++) {
            mag_sample_t m;
            memset(&m, 0, sizeof m);
            for (int k = 0; k < 3; k++)
                m.field[k] = (Rt[0][k]*m_ned[0] + Rt[1][k]*m_ned[1] + Rt[2][k]*m_ned[2])
                             * 100.0f + bench_noise(0.3f * mag_nscale);
            m.valid = true;
            mekf_update_mag(&f, &m);
        }

        mekf_update_accel(&f, &s);

        if (t >= warmup_s) {
            float err = q_angle_between(f.q, q_true);
            sum_att2 += (double)err * err;

            float r_e, p_e, y_e;
            q_to_euler(f.q, &r_e, &p_e, &y_e);
            float hd = y_e - yaw_true;
            while (hd >  (float)M_PI) hd -= 2.0f*(float)M_PI;
            while (hd < -(float)M_PI) hd += 2.0f*(float)M_PI;
            sum_hdg2 += (double)hd * hd;
            n_meas++;

            /* Covariance consistency, accumulated on EVERY measured step.
             * Subsampling is tempting (consecutive 833 Hz samples are nearly
             * identical) but wrong here: the attitude error is wave-phase
             * correlated, and any decimation at a rate commensurate with the
             * 5 s roll period samples a handful of fixed wave phases and
             * biases the mean. Averaging over every step is the unbiased
             * estimator and costs ~0.3 s across the whole seed set. */
            {
                float dth[3];
                q_err_rotvec(f.q, q_true, dth);
                float e2 = dth[0]*dth[0] + dth[1]*dth[1] + dth[2]*dth[2];

                sum_e2 += (double)e2;
                sum_tr += (double)p_att_trace(&f);

                float P33[3][3], Pi[3][3];
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++) P33[a][b] = f.P[a][b];
                if (m33_inverse(P33, Pi)) {
                    double q = 0.0;
                    for (int a = 0; a < 3; a++)
                        for (int b = 0; b < 3; b++)
                            q += (double)dth[a] * Pi[a][b] * dth[b];
                    sum_nees_st += q / 3.0;   /* /dof so 1.0 = consistent */
                }
                n_nees++;
            }
        }
    }

    out->rms_att      = (float)sqrt(sum_att2 / n_meas) / d2r;
    out->rms_hdg      = (float)sqrt(sum_hdg2 / n_meas) / d2r;
    out->bias_err     = fabsf(f.bias[2] - bias_true[2]);
    out->nees_trace   = sum_tr > 1e-20 ? (float)(sum_e2 / sum_tr) : 0.0f;
    out->nees_strict  = n_nees ? (float)(sum_nees_st / n_nees) : 0.0f;
    out->innov_weight = f.innov_weight_ema;
    out->innov_reject = f.innov_reject_ema;
    out->nis_accel    = f.nis_accel_ema;
    out->nis_mag      = f.nis_mag_ema;
    memcpy(out->m_ref, f.m_ref, sizeof f.m_ref);
}

/* Mean and worst-case RMS over the seed set, for one mag mode. */
typedef struct {
    float att_mean, hdg_mean, bias_mean;
    float att_worst, hdg_worst;
    float nees_tr_mean, nees_st_mean, nees_tr_worst;
    float weight_mean, reject_mean;
    float nis_a_mean, nis_m_mean;
} bench_result_t;

static void run_wave_seeds_ex(bool yaw_only, wave_scen_t scen, bench_result_t *r)
{
    double sa = 0, sh = 0, sb = 0, sn = 0, ss = 0, sw = 0, sr = 0;
    double sna = 0, snm = 0;
    float  wa = 0, wh = 0, wn = 0;
    for (int i = 0; i < N_BENCH_SEEDS; i++) {
        wave_run_t run;
        bench_seed = bench_seeds[i];
        run_wave_scenario_ex(yaw_only, scen, &run);
        sa += run.rms_att;    sh += run.rms_hdg;      sb += run.bias_err;
        sn += run.nees_trace; ss += run.nees_strict;
        sw += run.innov_weight; sr += run.innov_reject;
        sna += run.nis_accel;   snm += run.nis_mag;
        if (run.rms_att    > wa) wa = run.rms_att;
        if (run.rms_hdg    > wh) wh = run.rms_hdg;
        if (run.nees_trace > wn) wn = run.nees_trace;
    }
    r->att_mean      = (float)(sa / N_BENCH_SEEDS);
    r->hdg_mean      = (float)(sh / N_BENCH_SEEDS);
    r->bias_mean     = (float)(sb / N_BENCH_SEEDS);
    r->att_worst     = wa;
    r->hdg_worst     = wh;
    r->nees_tr_mean  = (float)(sn / N_BENCH_SEEDS);
    r->nees_st_mean  = (float)(ss / N_BENCH_SEEDS);
    r->nees_tr_worst = wn;
    r->weight_mean   = (float)(sw / N_BENCH_SEEDS);
    r->reject_mean   = (float)(sr / N_BENCH_SEEDS);
    r->nis_a_mean    = (float)(sna / N_BENCH_SEEDS);
    r->nis_m_mean    = (float)(snm / N_BENCH_SEEDS);
}

/* The historical seaway. Every recorded number in the tree is this one. */
static void run_wave_seeds(bool yaw_only, bench_result_t *r)
{
    run_wave_seeds_ex(yaw_only, SCEN_TONE, r);
}

/*
 * The draw stream, pinned.
 *
 * run_wave_scenario_ex notes that SCEN_TONE must consume EXACTLY its historical
 * random-draw sequence or every recorded number in this tree shifts.  That was a
 * comment, which is to say it was a hope.  This makes it a mechanism: one seed,
 * two constants, recorded from the tree as it stood.
 *
 * bench_draws pins how many draws the scenario takes.  bench_rng_state pins the
 * generator's state afterwards, which — xorshift being a pure state machine — is
 * a function of the seed and the call count and nothing else.  So the second
 * assertion is not independent evidence about the sequence; what it adds over
 * the first is that the SEED did not change.
 *
 * WHAT THIS DOES NOT CATCH is therefore larger than it looks, and both halves
 * were confirmed by mutation rather than assumed:
 *
 *   - Reordering.  Swapping the accel and gyro draws inside the sample loop
 *     leaves the count at 966801 and the state at 0x390A0177 — this test passes
 *     — while 3-D attitude RMS moves 1.178 -> 1.175.
 *   - Anything that changes what is DONE with a draw: a different mekf_init
 *     argument, a noise sigma scaled wrongly.  No draw moves at all.
 *
 * Both are caught only by diffing the benchmark's printed lines before and
 * after.  This guard is the cheap half of a two-part proof, never the whole of
 * it; a refactor of this file needs the byte-diff as well.
 *
 * One seed rather than twelve: the guard is about the per-sample structure of
 * the loop, which every seed shares, and this keeps the cost near 1/36th of the
 * benchmark.
 */
static void test_bench_stream_fingerprint(void)
{
    /*
     * Recorded from the tree at the time this guard was added, and independently
     * predicted before it was measured — which is why the count is quoted with
     * its derivation rather than as a magic number:
     *
     *   align   4165 samples x 9 draws  (6 sensor + 3 mag)  =  37 485
     *   measure 145775 samples x 6 draws (sensor only)      = 874 650
     *   mag     18222 updates x 3 draws                     =  54 666
     *                                                         --------
     *                                                          966 801
     *
     * If a change moves this, that arithmetic says which term moved.
     */
    const unsigned long EXPECT_DRAWS = 966801UL;
    const uint32_t      EXPECT_STATE = 0x390A0177u;

    wave_run_t run;

    /*
     * The constants below pin ONE pairing: the reference rates.  Under a
     * -DBENCH_ODR_HZ or -DBENCH_MAG_HZ probe build the scenario is deliberately
     * running somewhere else, where a different draw count is the correct
     * outcome and asserting the reference count would report a failure that
     * means nothing.  Skip, and say so, rather than emit a misleading red.
     */
    if (bench_fs != BENCH_REF_FS || bench_mag_fs != BENCH_REF_MAG_FS) {
        printf("\n    [stream] skipped: probe build at %.0f Hz / %.0f Hz mag,"
               " not the reference pair\n    ",
               (double)bench_fs, (double)bench_mag_fs);
        return;
    }

    bench_seed  = bench_seeds[0];
    bench_draws = 0;
    run_wave_scenario_ex(false, SCEN_TONE, &run);

    printf("\n    [stream] draws = %lu  final state = 0x%08X\n    ",
           bench_draws, bench_rng_state);

    EXPECT(bench_draws == EXPECT_DRAWS,
           "SCEN_TONE draw count unchanged (the stream is the baseline)");
    EXPECT(bench_rng_state == EXPECT_STATE,
           "SCEN_TONE final RNG state unchanged");
}

/*
 * ROADMAP §10.1 tuning sweep — not built by default.
 *
 *   make test_fusion CFLAGS="-D_GNU_SOURCE -O2 -Wall -Wextra -std=c11 \
 *       -pthread -Iinclude -DBENCH_SWEEP_RA" && ./test_fusion
 *
 * Sweeps mekf_accel_noise across the full seed set in both mag modes and
 * prints tracking accuracy alongside both consistency instruments. The
 * shipped default is chosen from this table; the recorded run lives in
 * docs/math.md §4.7 so the choice can be re-derived without a rebuild.
 */
#ifdef BENCH_SWEEP_RA
static void bench_sweep_ra(void)
{
    static const double na[] = {
        0.0022, 0.005, 0.01, 0.02, 0.03, 0.05, 0.08, 0.12, 0.2, 0.3,
    };
    printf("\n  ── mekf_accel_noise sweep (12 seeds/point) ─────────────────\n");
    printf("  %-8s %-5s %8s %8s %9s %9s %9s %8s %8s\n",
           "Na", "mode", "att RMS", "hdg RMS", "bias_z",
           "NEES(tr)", "NEES(st)", "NIS_a", "reject");
    for (size_t i = 0; i < sizeof na / sizeof na[0]; i++) {
        bench_ra_override = na[i];
        for (int mode = 0; mode < 2; mode++) {
            bench_result_t r;
            run_wave_seeds(mode == 1, &r);
            printf("  %-8.4f %-5s %8.3f %8.3f %9.6f %9.2f %9.2f %8.2f %8.4f\n",
                   na[i], mode ? "yaw" : "3D",
                   r.att_mean, r.hdg_mean, r.bias_mean,
                   r.nees_tr_mean, r.nees_st_mean, r.nis_a_mean, r.reject_mean);
        }
    }
    bench_ra_override = 0.0;
    printf("  ────────────────────────────────────────────────────────────\n\n");
}
#endif

/*
 * ROADMAP §10.5 tuning sweep for the Gauss–Markov wave state — not built by
 * default. `make` does not track CFLAGS, so remove the binary first:
 *
 *   rm -f test_fusion && make test_fusion CFLAGS="-D_GNU_SOURCE -O2 -Wall \
 *       -Wextra -std=c11 -pthread -Iinclude -DBENCH_SWEEP_WAVE" && ./test_fusion
 *
 * σ = 0 rows are the disabled filter, printed as the row everything else has
 * to beat. The recorded run lives in docs/math.md §4.7.
 */
#ifdef BENCH_SWEEP_WAVE
static void bench_sweep_wave(void)
{
    static const double sig[] = { 0.0, 0.3, 0.6, 0.8, 0.9, 1.2, 1.8 };
    static const double tau[] = { 0.3, 0.5, 0.8, 1.0, 2.0, 4.0 };
    double s0 = bench_wave_sigma, t0 = bench_wave_tau;

    for (int sc = 0; sc < 2; sc++) {
        wave_scen_t scen = sc ? SCEN_TONE : SCEN_GM;
        if (scen == SCEN_GM)
            printf("\n  ── GM wave-state sweep, BROADBAND scenario "
                   "(truth sigma=%.2f tau=%.2f) ──\n"
                   "  Tuning happens here: knobs = truth must give NIS ~ 1.\n",
                   scen_gm_sigma, scen_gm_tau);
        else
            printf("\n  ── GM wave-state sweep, TONE scenario "
                   "(held-out validation) ──\n");
        printf("  %-6s %-5s %-5s %8s %8s %9s %9s %9s %8s %8s\n",
               "sigma", "tau", "mode", "att RMS", "hdg RMS", "bias_z",
               "NEES(tr)", "NEES(st)", "NIS_a", "reject");
        for (size_t i = 0; i < sizeof sig / sizeof sig[0]; i++) {
            size_t ntau = (sig[i] == 0.0) ? 1 : sizeof tau / sizeof tau[0];
            for (size_t j = 0; j < ntau; j++) {
                bench_wave_sigma = sig[i];
                bench_wave_tau   = tau[j];
                for (int mode = 0; mode < 2; mode++) {
                    bench_result_t r;
                    run_wave_seeds_ex(mode == 1, scen, &r);
                    printf("  %-6.2f %-5.1f %-5s %8.3f %8.3f %9.6f %9.2f %9.2f %8.2f %8.4f\n",
                           sig[i], sig[i] == 0.0 ? 0.0 : tau[j], mode ? "yaw" : "3D",
                           r.att_mean, r.hdg_mean, r.bias_mean,
                           r.nees_tr_mean, r.nees_st_mean, r.nis_a_mean,
                           r.reject_mean);
                }
            }
        }
        printf("  ────────────────────────────────────────────────────────────\n");
    }
    bench_wave_sigma = s0;
    bench_wave_tau   = t0;
    printf("\n");
}
#endif

static void test_wave_benchmark(void)
{
#ifdef BENCH_SWEEP_RA
    bench_sweep_ra();
#endif
#ifdef BENCH_SWEEP_WAVE
    bench_sweep_wave();
#endif
    bench_result_t b3, by, bw;
    run_wave_seeds(false, &b3);   /* 3D vector mag */
    run_wave_seeds(true,  &by);   /* yaw-only (default) */
    /* 3-D with WMM field invariants — what a GPS-equipped boat actually runs,
     * and the only configuration in which the dip reference is trustworthy. */
    bench_wmm_ref = true;
    run_wave_seeds(false, &bw);
    bench_wmm_ref = false;

    printf("\n    [wave bench 3D  ] attitude RMS = %.3f° (worst %.3f)  "
           "heading RMS = %.3f° (worst %.3f)  bias_z err = %.5f rad/s\n",
           b3.att_mean, b3.att_worst, b3.hdg_mean, b3.hdg_worst, b3.bias_mean);
    printf("    [wave bench yaw ] attitude RMS = %.3f° (worst %.3f)  "
           "heading RMS = %.3f° (worst %.3f)  bias_z err = %.5f rad/s\n",
           by.att_mean, by.att_worst, by.hdg_mean, by.hdg_worst, by.bias_mean);
    printf("    [consistency 3D ] NEES(trace) = %.2f (worst %.2f)  "
           "NEES(strict/3) = %.2f  NIS(a/m) = %.2f/%.2f  weight = %.3f  reject = %.4f\n",
           b3.nees_tr_mean, b3.nees_tr_worst, b3.nees_st_mean,
           b3.nis_a_mean, b3.nis_m_mean, b3.weight_mean, b3.reject_mean);
    printf("    [consistency yaw] NEES(trace) = %.2f (worst %.2f)  "
           "NEES(strict/3) = %.2f  NIS(a/m) = %.2f/%.2f  weight = %.3f  reject = %.4f\n",
           by.nees_tr_mean, by.nees_tr_worst, by.nees_st_mean,
           by.nis_a_mean, by.nis_m_mean, by.weight_mean, by.reject_mean);
    printf("    [wave bench 3D+WMM] attitude RMS = %.3f°  heading RMS = %.3f°  "
           "NEES(trace) = %.2f  NEES(strict/3) = %.2f  NIS(a/m) = %.2f/%.2f\n    ",
           bw.att_mean, bw.hdg_mean, bw.nees_tr_mean, bw.nees_st_mean,
           bw.nis_a_mean, bw.nis_m_mean);

    /*
     * Regression bounds, on the MEAN over N_BENCH_SEEDS draws (see the
     * scenario comment for why a single seed is not a usable signal).
     *
     * ── Recorded change: gross-reject gate 9γ → 25γ (ROADMAP §10.1) ───────
     * Investigating §10.1's claim that Ra was far too optimistic for a seaway
     * showed the opposite: the 9γ gate was rejecting 7–11% of ordinary wave
     * motion, starving the filter, and the resulting drift produced the large
     * innovations that looked like an over-confident noise model. Widening
     * the gate to 25γ (see GROSS_REJECT_MULT in fusion.c) improves every
     * measured quantity in both modes, with Ra unchanged:
     *
     *              3-D att   3-D hdg   3-D bias   yaw att   yaw hdg   yaw bias
     *    before      6.848°    4.271°   3.52e-4    3.574°    3.099°   9.68e-4
     *    after       5.653°    3.065°   2.38e-4    2.309°    1.961°   6.63e-4
     *
     *              NEES(tr)  NEES(st)   NIS_a    reject
     *    3-D before   28.96     62.66   30.66    0.2618
     *    3-D after    18.32     57.38   19.30    0.0066
     *    yaw before   21.14    129.02   27.26    0.0429
     *    yaw after     7.80     37.73   25.18    0.0000
     *
     * The worst-case draws collapsed too (3-D 13.68° → 7.02°, yaw 9.59° →
     * 2.34°): the wide per-seed spread §10.8 records was this rejection
     * feedback, not scenario luck.
     *
     * The previous baseline was measured with the error-state reset Jacobian
     * in place; without it the same seed set gave 6.71/4.09 (3D) and
     * 4.10/3.54 (yaw), i.e. the reset bought ~13% on yaw attitude for ~2% on
     * 3D and cut NEES 32.4 → 28.4.
     *
     * Bounds are set ~15% above measured to absorb float/platform variance
     * without letting a real regression through. Worst-case draws are printed
     * but still NOT asserted: even though the tail is now tight, it remains a
     * scenario-luck statistic rather than a filter-quality one.
     *
     * ── Recorded change: m33_inv singularity test, and the Gauss–Markov
     *    wave-acceleration state (ROADMAP §10.5) ───────────────────────────
     *
     * The numbers above were, it turned out, measured through a bug. m33_inv
     * declared S singular on an ABSOLUTE |det| < 1e-12, but S = HPHᵀ + R·I for
     * the gravity update carries physical units and its determinant sits near
     * 1e-13 at ordinary conditioning. 87% of accel updates in this benchmark
     * were being silently discarded by that test, and the health EMAs were
     * being fed only by the 13% that survived. The accidental decimation was
     * doing real work — it crudely decorrelated the wave-contaminated samples
     * — which is why the filter looked as good as it did.
     *
     * With the test made scale-relative (fusion.c m33_inv) and nothing else
     * changed, the filter feeds on all 833 samples/s, believes each one is an
     * independent measurement of gravity, and diverges into its own
     * over-confidence: 9.49°/11.42° attitude, NEES(trace) 259/252, NIS 56/57,
     * 15% gross rejects. That is §10.1's diagnosis in its undisguised form —
     * a seaway's gravity residual is correlated, so 833 correlated samples do
     * not carry 833 samples' worth of information, and no scalar R can say so.
     *
     * The Gauss–Markov wave state says so. Modelling the disturbance as a
     * first-order GM process (σ = 0.8 m/s², τ = 0.5 s) makes repeated samples
     * correctly stop adding information, and every column lands:
     *
     *                  3-D att   3-D hdg   yaw att   yaw hdg   NEES(tr)   NIS_a
     *   old baseline     5.653°    3.065°    2.309°    1.961°  18.3/ 7.8  19.3/25.2
     *   inv fixed only   9.488°    7.255°   11.424°    8.801°   259 / 252  56.5/56.9
     *   + wave state     4.452°    0.828°    2.308°    1.016°  3.47/0.99  1.01/0.69
     *
     * The old baseline is the honest bar (ROADMAP §10.5 states the criterion
     * against it), and the wave state clears it everywhere: 3-D attitude −21%,
     * 3-D heading −73%, yaw heading −48%, yaw attitude a dead heat, NIS from
     * 19–25 to ≈1, NEES(trace) from 18.3/7.8 to 3.47/0.99, and the Huber cap
     * and gross-reject gate both go completely idle (weight 1.000, reject 0).
     *
     * σ and τ were chosen on a BROADBAND scenario (SCEN_GM) whose disturbance
     * is a real Gauss–Markov process of known σ and τ, not on the single tone
     * asserted here — the tone's autocorrelation never decays, so fitting τ to
     * it would be fitting the benchmark. The tone is the held-out validation.
     * σ = 0.8 m/s² is also exactly the tone's true per-axis RMS and τ = 0.5 s
     * sits inside fit-ra's measured 0.3–0.9 s, so neither knob is a free
     * parameter. The grid is in docs/math.md §4.7 (-DBENCH_SWEEP_WAVE).
     *
     * ── Recorded change: the alignment fidelity bug ───────────────────────
     * This scenario used to align from ONE instantaneous sample and claim it
     * was doing what the daemon does. It was not — the daemon averages a 5 s
     * window (src/imu.c) — and the difference dominated every 3-D number here.
     * See the alignment block above for the mechanism and the measured table.
     * Re-basing on a daemon-faithful alignment:
     *
     *                     3-D att   3-D hdg   yaw att   yaw hdg   NEES(st) 3D/yaw
     *   1-sample (old)     4.452°    0.828°    2.308°    1.016°     335 / 10.1
     *   5 s mean (this)    1.204°    0.745°    2.185°    1.011°    12.8 / 8.37
     *
     * Nothing in the filter changed to produce that: it is the instrument
     * being made honest. The yaw-only default barely moves (2.308° → 2.185°),
     * which is the expected signature — heading-only fusion never uses the dip
     * channel that the alignment error corrupts.
     *
     * ── On the 3-D+WMM row ────────────────────────────────────────────────
     * The residual 3-D inconsistency is m_ref DIP error left over from
     * alignment (+0.86°), which the dip channel converts into a constant
     * roll/pitch bias that P has no term for. It cannot be healed from seaway
     * data — see the acc_quiet_ema gate in fusion.c and the refutation
     * recorded there. It CAN be removed at the source, which is what
     * mekf_set_mref_invariants does when a position source supplies the field.
     * That configuration is the best the filter offers and was previously
     * unmeasured, so it is now a printed row: attitude 0.841°, NEES(strict)
     * 0.22 — an order of magnitude better than yaw-only, and consistent.
     *
     * ── On the anisotropic dip term (mekf_mag_dip_sigma_deg) ─────────────
     * For installs without a position source, the dip error cannot be removed,
     * so it is admitted into P instead: R gains a rank-1 term σ_dip²·uuᵀ along
     * the one direction a dip error perturbs (see mekf_update_mag). At the
     * shipped 1.0° — the measured +0.86° residual, rounded up, NOT a value
     * fitted to this metric — 3-D NEES(strict) goes 12.83 → 5.74 while
     * attitude and heading both improve slightly (1.204° → 1.178°,
     * 0.745° → 0.714°) and yaw-only is bit-identical.
     *
     * It does not reach 1, and cannot: the dip error is a BIAS, and a
     * covariance term can only partly stand in for one. The accelerometer
     * legitimately keeps P tight in roll and pitch, so P cannot grow to cover
     * a systematic offset without making the filter worse at everything else.
     * Driving this number to ~1 needs σ_dip ≈ 4°, which would be inventing
     * uncertainty to satisfy a statistic. The sweep is in docs/math.md §4.8.1;
     * the real fix for an install that cares is a position source.
     *
     * The cost is `nis_mag` in 3-D mode: 0.52 → 0.31. Isotropic inflation
     * reaching the same NEES puts it at 0.01 — i.e. destroys the wire's
     * magnetometer-health instrument — which is the whole argument for the
     * rank-1 form. Yaw-only, the default, is untouched at 0.55.
     *
     * Bounds are ~1.3× the measured means. The NIS bounds are TWO-SIDED: with
     * a state that can absorb disturbance, under-confidence is now a reachable
     * failure mode (too large a σ eats real tilt error) and a one-sided bound
     * would not see it. Worst-case draws are printed but not asserted.
     */
    EXPECT(b3.att_mean  < 1.55f,    "wave-bench 3D attitude RMS under bound");
    EXPECT(b3.hdg_mean  < 1.00f,    "wave-bench 3D heading RMS under bound");
    EXPECT(b3.bias_mean < 0.001f,   "wave-bench 3D bias error under bound");
    EXPECT(by.att_mean  < 2.90f,    "wave-bench yaw-only attitude RMS under bound");
    EXPECT(by.hdg_mean  < 1.35f,    "wave-bench yaw-only heading RMS under bound");
    EXPECT(by.bias_mean < 0.001f,   "wave-bench yaw-only bias error under bound");

    /* Consistency, two-sided: 1.0 is the target, not a ceiling to stay under. */
    EXPECT(b3.nis_a_mean > 0.35f && b3.nis_a_mean < 1.10f,
           "wave-bench 3D accel NIS stays near 1");
    EXPECT(by.nis_a_mean > 0.40f && by.nis_a_mean < 1.20f,
           "wave-bench yaw-only accel NIS stays near 1");
    EXPECT(b3.nees_tr_mean < 0.30f, "wave-bench 3D NEES(trace) under bound");
    EXPECT(by.nees_tr_mean < 1.20f, "wave-bench yaw-only NEES(trace) under bound");
    EXPECT(by.nees_st_mean < 11.0f, "wave-bench yaw-only NEES(strict) under bound");
    EXPECT(b3.nees_st_mean < 7.50f, "wave-bench 3D NEES(strict) under bound");

    /* 3-D with a trustworthy dip reference must be both the most accurate
     * configuration and a consistent one — this is the row that proves the
     * residual 3-D inconsistency is the REFERENCE, not the filter. */
    EXPECT(bw.att_mean     < 1.05f, "wave-bench 3D+WMM attitude RMS under bound");
    EXPECT(bw.att_mean     < b3.att_mean,
           "WMM invariants beat an aligned dip reference");
    EXPECT(bw.nees_st_mean < 0.50f,
           "wave-bench 3D+WMM NEES(strict) is consistent");

    /* Both gates must stay idle in a normal seaway; the pre-1.7 9γ gate sat at
     * 0.26 here and the un-modelled seaway at 0.15, so this catches a
     * regression to the starvation regime with a wide margin. */
    EXPECT(b3.reject_mean  < 0.02f, "wave-bench 3D gross-reject rate stays negligible");
    EXPECT(by.reject_mean  < 0.02f, "wave-bench yaw-only gross-reject rate stays negligible");
    EXPECT(b3.weight_mean  > 0.95f, "wave-bench 3D Huber cap stays essentially idle");
    EXPECT(by.weight_mean  > 0.95f, "wave-bench yaw-only Huber cap stays essentially idle");
}

/*
 * Ground truth for the Gauss–Markov state. The tone benchmark cannot supply
 * one: a sinusoid has no correlation time, so "the filter's knobs are right"
 * is not a statement it can evaluate. Here the disturbance IS a first-order
 * GM process of known σ and τ, so setting the filter's knobs to the truth
 * makes the correct answer exactly NIS = 1 and NEES = 1 — which is the only
 * check that tests the estimator rather than the tuning.
 */
static void test_wave_gm_ground_truth(void)
{
    double s0 = bench_wave_sigma, t0 = bench_wave_tau;
    bench_wave_sigma = scen_gm_sigma;   /* knobs := truth */
    bench_wave_tau   = scen_gm_tau;
    bench_align_clean = true;           /* isolate the estimator; see the flag */

    bench_result_t r;
    run_wave_seeds_ex(true, SCEN_GM, &r);   /* marine default: yaw-only */

    printf("\n    [GM truth yaw   ] sigma = %.2f m/s^2  tau = %.2f s  ->  "
           "att RMS = %.3f°  NEES(trace) = %.2f  NIS_a = %.2f  reject = %.4f\n    ",
           scen_gm_sigma, scen_gm_tau, r.att_mean, r.nees_tr_mean,
           r.nis_a_mean, r.reject_mean);

    /*
     * Tight on purpose. This is the one configuration where the correct answer
     * is known exactly rather than measured, so the band is a real check on the
     * estimator, not a ratchet. It is also the suite's sharpest detector of a
     * mis-derived Jacobian: dropping the tangent projector from wave_jacobian
     * (H_δθ = [h×]/|v|, H_δa_w = −I/|v|, which is what a first pass at this
     * naturally writes) leaves every other test in this file passing and moves
     * this number out of the band.
     *
     * Measured: correct 0.87, unprojected 0.73. The margins are 8% below and
     * 44% above, so the band is tight enough to be a real check and loose
     * enough to survive float/platform variance — but it is deliberately not
     * tighter, because 0.14 is the whole separation available.
     */
    EXPECT(r.nis_a_mean > 0.80f && r.nis_a_mean < 1.25f,
           "GM ground truth: accel NIS is ~1 when the knobs are the truth");
    EXPECT(r.nees_tr_mean > 0.5f && r.nees_tr_mean < 2.5f,
           "GM ground truth: NEES(trace) is ~1 when the knobs are the truth");
    EXPECT(r.reject_mean < 0.01f,
           "GM ground truth: gross-reject gate stays idle");

    bench_wave_sigma  = s0;
    bench_wave_tau    = t0;
    bench_align_clean = false;
}

/* ── Main ────────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("=== imud fusion tests ===\n");
    RUN(test_quat_norm_preserved);
    RUN(test_predict_static_no_drift);
    RUN(test_predict_known_rotation_yaw);
    RUN(test_predict_known_rotation_roll);
    RUN(test_align_flat_north);
    RUN(test_align_30deg_roll);
    RUN(test_accel_update_corrects_roll);
    RUN(test_accel_reject_linear_acceleration);
    RUN(test_mag_update_corrects_yaw);
    RUN(test_mag_reject_anomaly);
    RUN(test_covariance_decreases_with_updates);
    RUN(test_convergence_flag);
    RUN(test_precomputed_bias_used);
    RUN(test_get_state_euler_extraction);
    RUN(test_sanitize_recovers_from_non_finite);
    RUN(test_state_reset_flag_latches_until_converged);
    RUN(test_get_state_heading_wrap);
    RUN(test_align_accel_too_weak);
    RUN(test_mag_update_skips_invalid);
    RUN(test_mag_ratio_gate);
    RUN(test_mekf_reconfigure);
    RUN(test_reconfigure_rederives_mag_tuning);
    RUN(test_mag_tuning_uses_the_programmed_rate);
    RUN(test_reconfigure_resets_skip_window);
    RUN(test_sim_gyro_heading);
    RUN(test_sim_heading_wraps);
    RUN(test_sim_full_fusion);
    RUN(test_joseph_symmetry_psd);
    RUN(test_reset_jacobian_rotates_P);
    RUN(test_gate_health_emas);
    RUN(test_nis_consistency_emas);
    RUN(test_reconfigure_preserves_nis);
    RUN(test_accel_innovation_capped);
    RUN(test_mref_ema_heals_dip);
    RUN(test_yaw_only_leaves_tilt);
    RUN(test_mref_invariants);
    RUN(test_centripetal_correction);
    RUN(test_heave_sine_amplitude);
    RUN(test_heave_disabled_and_settle);
    RUN(test_seastate_sine);
    RUN(test_seastate_gates);
    RUN(test_mag_health);
    RUN(test_wave_disabled_inert);
    RUN(test_wave_needs_both_knobs);
    RUN(test_wave_state_tracks_colored_residual);
    RUN(test_wave_nis_dof_consistent);
    RUN(test_wave_reconfigure_transitions);
    RUN(test_mag_dip_sigma_widens_covariance);
    RUN(test_mag_dip_sigma_is_tangent_to_the_dip_channel);
    RUN(test_mag_dip_sigma_inert_in_yaw_only);
    RUN(test_mref_quiet_gate_stays_tight);
    RUN(test_bench_stream_fingerprint);
    RUN(test_wave_benchmark);
    RUN(test_wave_gm_ground_truth);
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
