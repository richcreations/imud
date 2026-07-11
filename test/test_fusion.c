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
    return c;
}

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

/* ── Tests ──────────────────────────────────────────────────────────────────── */

/*
 * Quaternion is always unit-norm: predict with zero gyro for 1000 steps.
 */
TEST(test_quat_norm_preserved)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);

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
    mekf_init(&f, &cfg, 833.0f, bias);

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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);

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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});

    EXPECT(!f.converged, "not converged immediately after init");

    imu_sample_t sa = make_accel(0, 0, -G);
    mag_sample_t sm = make_mag(0.2f, 0, 0.05f);
    imu_sample_t sg = make_gyro(0, 0, 0);

    for (int i = 0; i < 2000; i++) {
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
    mekf_init(&f, &cfg, 833.0f, known_bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
 * get_state: heading wraps correctly at 0° / 360° boundary.
 */
TEST(test_get_state_heading_wrap)
{
    imud_config_t cfg = make_cfg();
    mekf_t f;
    float bias[3] = {0};
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);

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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    for (int i = 0; i < 6; i++) {
        if (f.P[i][i] < min_diag) min_diag = f.P[i][i];
        for (int j = 0; j < 6; j++) {
            float d = fabsf(f.P[i][j] - f.P[j][i]);
            if (d > max_asym) max_asym = d;
        }
    }
    EXPECT(max_asym == 0.0f, "P exactly symmetric after 50k updates");
    EXPECT(min_diag > 0.0f,  "P diagonal stays positive after 50k updates");
    EXPECT_NEAR(q_norm(f.q), 1.0f, 1e-4f, "q unit norm after 50k updates");
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);
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
    mekf_init(&f, &cfg, 833.0f, bias);

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
    mekf_init(&f, &cfg, 833.0f, bias);
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

    EXPECT(roll_off > 2.0f*DEG,
           "without speed, sustained turn tilts the roll estimate");
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
    mekf_init(&f, &cfg, 833.0f, bias);
    mekf_align(&f, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    mag_sample_t m = make_mag(0.20f, 0.0f, 0.05f);   /* Gauss (make_mag scales) */
    for (int i = 0; i < 2000; i++) mekf_update_mag(&f, &m);
    EXPECT(f.mag_anom_ema  < 0.01f, "clean field: anomaly ~0");
    EXPECT(f.mag_resid_ema < 0.01f, "clean field: residual ~0");

    /* Magnitude anomaly: same direction, 1.5x strength (inside the hard
     * 0.5-2.0 gate). EMA (alpha=1/3000) reaches 63% of the 0.5 step. */
    mekf_t fa;
    mekf_init(&fa, &cfg, 833.0f, bias);
    mekf_align(&fa, (float[]){0,0,-G}, (float[]){20.0f,0,5.0f});
    mag_sample_t ma = make_mag(0.30f, 0.0f, 0.075f);
    for (int i = 0; i < 3000; i++) mekf_update_mag(&fa, &ma);
    EXPECT(fa.mag_anom_ema  > 0.2f,  "1.5x magnitude: anomaly rises");
    EXPECT(fa.mag_resid_ema < 0.05f, "direction unchanged: residual low");

    /* Heading anomaly with updates REJECTED (converged + tight gate): the
     * metric must rise precisely while the filter refuses the data. */
    mekf_t fr;
    mekf_init(&fr, &cfg, 833.0f, bias);
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
 */

static uint32_t bench_rng_state = 0x1234ABCDu;
static float bench_rand(void)   /* uniform in [-1, 1) */
{
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

static void run_wave_scenario(bool yaw_only,
                              float *rms_att_out, float *rms_hdg_out,
                              float *bias_err_out, float m_ref_out[3])
{
    const float fs   = 833.0f;
    const float dt   = 1.0f / fs;
    const float d2r  = (float)(M_PI / 180.0);

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
    mekf_t f;
    mekf_init(&f, &cfg, fs, bias_init);
    bench_rng_state = 0x1234ABCDu;

    const float warmup_s = 60.0f, measure_s = 120.0f;
    const int   n_total  = (int)((warmup_s + measure_s) * fs);

    double sum_att2 = 0.0, sum_hdg2 = 0.0;
    int    n_meas   = 0;

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
        float a_lin[3] = {
            1.2f * sinf(wr * t + 0.5f),
            1.2f * cosf(0.8f * wr * t),
            1.0f * sinf(wr * t),
        };

        /* Specific force in body: f_b = Rᵀ (a_lin − g·e_down) */
        float f_ned[3] = { a_lin[0], a_lin[1], a_lin[2] - G };
        imu_sample_t s;
        memset(&s, 0, sizeof s);
        for (int k = 0; k < 3; k++) {
            s.accel[k] = Rt[0][k]*f_ned[0] + Rt[1][k]*f_ned[1] + Rt[2][k]*f_ned[2]
                       + bench_noise(0.03f);
            s.gyro[k]  = w_body[k] + bias_true[k] + bench_noise(0.002f);
        }

        /* Alignment: one instantaneous noisy sample, like the daemon */
        if (i == 0) {
            float mb[3];
            for (int k = 0; k < 3; k++)
                mb[k] = (Rt[0][k]*m_ned[0] + Rt[1][k]*m_ned[1] + Rt[2][k]*m_ned[2])
                        * 100.0f + bench_noise(0.3f);   /* µT */
            mekf_align(&f, s.accel, mb);
            continue;
        }

        mekf_predict(&f, &s, f.dt);

        /* Mag update at ~104 Hz */
        if (i % 8 == 0) {
            mag_sample_t m;
            memset(&m, 0, sizeof m);
            for (int k = 0; k < 3; k++)
                m.field[k] = (Rt[0][k]*m_ned[0] + Rt[1][k]*m_ned[1] + Rt[2][k]*m_ned[2])
                             * 100.0f + bench_noise(0.3f);
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
        }
    }

    *rms_att_out  = (float)sqrt(sum_att2 / n_meas) / d2r;
    *rms_hdg_out  = (float)sqrt(sum_hdg2 / n_meas) / d2r;
    *bias_err_out = fabsf(f.bias[2] - bias_true[2]);
    if (m_ref_out) memcpy(m_ref_out, f.m_ref, sizeof f.m_ref);
}

static void test_wave_benchmark(void)
{
    float att3, hdg3, be3, atty, hdgy, bey;
    float mr3[3], mry[3];
    run_wave_scenario(false, &att3, &hdg3, &be3, mr3);  /* 3D vector mag */
    run_wave_scenario(true,  &atty, &hdgy, &bey, mry);  /* yaw-only (default) */

    printf("\n    [wave bench 3D  ] attitude RMS = %.3f°  heading RMS = %.3f°  "
           "bias_z err = %.5f rad/s  m_ref=[%.3f %.3f %.3f]\n",
           att3, hdg3, be3, mr3[0], mr3[1], mr3[2]);
    printf("    [wave bench yaw ] attitude RMS = %.3f°  heading RMS = %.3f°  "
           "bias_z err = %.5f rad/s  m_ref=[%.3f %.3f %.3f]\n    ",
           atty, hdgy, bey, mry[0], mry[1], mry[2]);

    /*
     * Regression bounds. Pre-improvement baseline (simple covariance form,
     * hard magnitude gate only, static m_ref, single-sample values):
     * attitude RMS 7.10°, heading RMS 4.88°. The improved filter measured
     * ~5.1° / ~2.6°; bounds leave headroom for platform float variance.
     */
    EXPECT(att3 < 6.0f,      "wave-bench 3D attitude RMS under bound");
    EXPECT(hdg3 < 3.5f,      "wave-bench 3D heading RMS under bound");
    EXPECT(be3  < 0.001f,    "wave-bench 3D bias error under bound");
    EXPECT(atty < 6.0f,      "wave-bench yaw-only attitude RMS under bound");
    EXPECT(hdgy < 3.5f,      "wave-bench yaw-only heading RMS under bound");
    EXPECT(bey  < 0.001f,    "wave-bench yaw-only bias error under bound");
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
    RUN(test_get_state_heading_wrap);
    RUN(test_align_accel_too_weak);
    RUN(test_mag_update_skips_invalid);
    RUN(test_mag_ratio_gate);
    RUN(test_mekf_reconfigure);
    RUN(test_sim_gyro_heading);
    RUN(test_sim_heading_wraps);
    RUN(test_sim_full_fusion);
    RUN(test_joseph_symmetry_psd);
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
    RUN(test_wave_benchmark);
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
