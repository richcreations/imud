/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * sim.c — synthetic IMU + magnetometer driver
 *
 * Simulates a small boat under way: constant yaw sweep overlaid with
 * sinusoidal roll and pitch from wave action and heave acceleration.
 *
 * Motion model:
 *   Yaw    ψ(t) = 60° + SIM_YAW_RATE_RPS × t    (6°/s sweep from 60° start;
 *                                                non-zero start catches
 *                                                heading-mirror bugs)
 *   Roll   φ(t) = SIM_ROLL_AMP  × sin(2π f_r t)        (±4°, 6 s period)
 *   Pitch  θ(t) = SIM_PITCH_AMP × sin(2π f_p t + φ_p)  (±2°, 8 s period)
 *   Heave  h(t) = SIM_HEAVE_AMP × sin(2π f_h t)         (±0.3 m/s², 5 s period)
 *
 * IMU outputs (physically correct NED Z-down aerospace convention):
 *   gyro[xyz]  — angular velocity in body frame (full kinematic formula)
 *   accel[xyz] — specific force = body acceleration − gravity, in body frame
 *
 * Magnetometer output:
 *   field[xyz] — NED Earth field rotated into body frame by full attitude R
 *
 * Activate: driver = "sim" in both [imu] and [mag], int_gpio = 0 for both.
 */

#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "drivers.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Physical constants ─────────────────────────────────────────────────── */

#define G_MS2              9.80665f

/* Yaw: one full sweep every 60 s.
 * The sweep starts at 60°, NOT north: heading-convention bugs (sign flips,
 * mirrors) have a fixed point at 0°, so a north-started sim hides them —
 * that is exactly how the mekf_align heading-mirror bug survived every
 * end-to-end test. Never change this back to 0. */
#define SIM_YAW_START_RAD  (60.0 * (M_PI / 180.0))
#define SIM_YAW_RATE_RPS   (6.0 * (M_PI / 180.0))

/* NED Earth field: mid-latitude, ~47 µT total.
 * Down component is positive in NED (Z-down) convention — field points
 * into the earth in the northern hemisphere. */
#define SIM_MAG_NORTH_UT   25.0f
#define SIM_MAG_DOWN_UT    40.0f

/* Roll: ±4°, 6-second period */
#define SIM_ROLL_AMP       (4.0 * (M_PI / 180.0))
#define SIM_ROLL_HZ        (1.0 / 6.0)

/* Pitch: ±2°, 8-second period, phase-offset from roll */
#define SIM_PITCH_AMP      (2.0 * (M_PI / 180.0))
#define SIM_PITCH_HZ       (1.0 / 8.0)
#define SIM_PITCH_PHASE    (M_PI / 3.0)   /* ~60° out of phase with roll */

/* Heave: ±0.3 m/s² vertical wave acceleration, 5-second period */
#define SIM_HEAVE_AMP_MS2  0.3f
#define SIM_HEAVE_HZ       (1.0 / 5.0)

/* ── Time helper ────────────────────────────────────────────────────────── */

static uint64_t mono_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ULL + (uint64_t)t.tv_nsec;
}

/*
 * Shared reference time — set when either sensor inits, used by both.
 * Ensures the attitude is consistent across IMU and mag reads.
 */
static uint64_t sim_t0_ns;
static bool     sim_t0_set;

static double elapsed_s(void)
{
    return (double)(mono_ns() - sim_t0_ns) * 1e-9;
}

/* ── Attitude helpers ───────────────────────────────────────────────────── */

/*
 * Attitude at time t using the 3-2-1 Euler sequence (yaw→pitch→roll):
 *   R = Rz(ψ) · Ry(θ) · Rx(φ)   (body → NED)
 *
 * Angles are in radians.
 */
static void sim_attitude(double t,
                         double *psi, double *theta, double *phi)
{
    *psi   = SIM_YAW_START_RAD + SIM_YAW_RATE_RPS * t;
    *phi   = SIM_ROLL_AMP  * sin(2.0 * M_PI * SIM_ROLL_HZ  * t);
    *theta = SIM_PITCH_AMP * sin(2.0 * M_PI * SIM_PITCH_HZ * t + SIM_PITCH_PHASE);
}

/*
 * Rotate a NED vector [nx, ny, nz] into body frame:
 *   v_body = R^T · v_NED = Rx(-φ) · Ry(-θ) · Rz(-ψ) · v_NED
 */
static void ned_to_body(double psi, double theta, double phi,
                        double nx, double ny, double nz,
                        float *bx, float *by, float *bz)
{
    /* Step 1: Rz(-ψ) */
    double cp = cos(psi), sp = sin(psi);
    double ax =  cp * nx + sp * ny;
    double ay = -sp * nx + cp * ny;
    double az = nz;

    /* Step 2: Ry(-θ) */
    double ct = cos(theta), st = sin(theta);
    double bx_ =  ct * ax + st * az;
    double by_  =  ay;
    double bz_  = -st * ax + ct * az;

    /* Step 3: Rx(-φ) */
    double cr = cos(phi), sr = sin(phi);
    *bx = (float)bx_;
    *by = (float)(cr * by_ + sr * bz_);
    *bz = (float)(-sr * by_ + cr * bz_);
}

/* ── IMU sim state ──────────────────────────────────────────────────────── */

static struct {
    uint64_t t_last_ns;   /* end of the most-recently-delivered sample batch */
    uint32_t seq;
    int      odr_hz;
} imu_s;

static int sim_imu_probe(int fd, uint8_t addr) { (void)fd; (void)addr; return 0; }
static int sim_imu_reset(int fd, uint8_t addr) { (void)fd; (void)addr; return 0; }

static int sim_imu_init(int fd, uint8_t addr, const imu_cfg_t *cfg)
{
    (void)fd; (void)addr;
    imu_s.t_last_ns = mono_ns();
    imu_s.seq       = 0;
    imu_s.odr_hz    = cfg->odr_hz > 0 ? cfg->odr_hz : 100;
    if (!sim_t0_set) { sim_t0_ns = imu_s.t_last_ns; sim_t0_set = true; }
    return 0;
}

static int sim_imu_read(int fd, uint8_t addr,
                        imu_sample_t *buf, int max, int *n_out)
{
    (void)fd; (void)addr;

    uint64_t now_ns    = mono_ns();
    uint64_t period_ns = 1000000000ULL / (uint64_t)imu_s.odr_hz;
    int      n         = (int)((now_ns - imu_s.t_last_ns) / period_ns);

    if (n <= 0) { *n_out = 0; return 0; }
    if (n > max)  n = max;

    /* t_start: time of the first (oldest) sample in this batch */
    uint64_t t_start_ns = imu_s.t_last_ns;
    imu_s.t_last_ns += (uint64_t)n * period_ns;

    /* ISM330DHCX chip timer: 40000 ticks/s (25 µs/tick) */
    uint32_t ticks_per = 40000u / (uint32_t)imu_s.odr_hz;

    for (int i = 0; i < n; i++) {
        double t = (double)(t_start_ns + (uint64_t)i * period_ns - sim_t0_ns) * 1e-9;

        /* ── Attitude at this sample ───────────────────────────────── */
        double psi, theta, phi;
        sim_attitude(t, &psi, &theta, &phi);

        /* ── Euler angle rates ─────────────────────────────────────── */
        double dpsi   = SIM_YAW_RATE_RPS;
        double dphi   = SIM_ROLL_AMP  * 2.0 * M_PI * SIM_ROLL_HZ
                        * cos(2.0 * M_PI * SIM_ROLL_HZ  * t);
        double dtheta = SIM_PITCH_AMP * 2.0 * M_PI * SIM_PITCH_HZ
                        * cos(2.0 * M_PI * SIM_PITCH_HZ * t + SIM_PITCH_PHASE);

        /*
         * ── Body-frame angular velocity (full kinematic formula) ────
         *
         * For aerospace 3-2-1 Euler (ψ, θ, φ):
         *   ω_x = φ̇ − ψ̇ sin θ
         *   ω_y = θ̇ cos φ + ψ̇ cos θ sin φ
         *   ω_z = −θ̇ sin φ + ψ̇ cos θ cos φ
         */
        buf[i].gyro[0] = (float)(dphi   - dpsi * sin(theta));
        buf[i].gyro[1] = (float)(dtheta * cos(phi) + dpsi * cos(theta) * sin(phi));
        buf[i].gyro[2] = (float)(-dtheta * sin(phi) + dpsi * cos(theta) * cos(phi));

        /*
         * ── Specific force in body frame ─────────────────────────────
         *
         * The accelerometer measures: f = a_platform − g_field
         * where a_platform is the vessel's inertial acceleration and
         * g_field = [0, 0, G] in NED (positive downward).
         *
         * Wave heave gives vertical NED acceleration h(t).
         * Net NED force per unit mass: [0, 0, h(t) − G]
         * Rotate into body frame with R^T:
         */
        double heave = SIM_HEAVE_AMP_MS2
                       * sin(2.0 * M_PI * SIM_HEAVE_HZ * t);
        ned_to_body(psi, theta, phi,
                    0.0, 0.0, heave - (double)G_MS2,
                    &buf[i].accel[0], &buf[i].accel[1], &buf[i].accel[2]);

        buf[i].temp_c  = 25.0f;
        buf[i].seq     = imu_s.seq;
        buf[i].chip_ts = imu_s.seq * ticks_per;   /* wraps at ~29.8 h */
        imu_s.seq++;
    }

    *n_out = n;
    return 0;
}

/* ── IMU driver descriptor ──────────────────────────────────────────────── */

const imu_ops_t sim_imu_ops = {
    .name               = "sim",
    .experimental       = false,
    .probe              = sim_imu_probe,
    .reset              = sim_imu_reset,
    .init               = sim_imu_init,
    .read               = sim_imu_read,
    .has_fifo           = true,
    .has_hw_timestamp   = true,
    .supported_odr_hz   = { 12, 26, 52, 104, 208, 416, 833, 1660, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};

/* ── Mag sim state ──────────────────────────────────────────────────────── */

static int sim_mag_probe(int fd, uint8_t addr) { (void)fd; (void)addr; return 0; }
static int sim_mag_reset(int fd, uint8_t addr) { (void)fd; (void)addr; return 0; }

static int sim_mag_init(int fd, uint8_t addr, const mag_cfg_t *cfg)
{
    (void)fd; (void)addr; (void)cfg;
    if (!sim_t0_set) { sim_t0_ns = mono_ns(); sim_t0_set = true; }
    return 0;
}

static int sim_mag_read(int fd, uint8_t addr, mag_sample_t *out)
{
    (void)fd; (void)addr;

    double t = elapsed_s();

    double psi, theta, phi;
    sim_attitude(t, &psi, &theta, &phi);

    /*
     * Earth's field in NED: [NORTH, 0, DOWN].
     * Rotate into body frame using the full attitude matrix.
     */
    ned_to_body(psi, theta, phi,
                (double)SIM_MAG_NORTH_UT, 0.0, (double)SIM_MAG_DOWN_UT,
                &out->field[0], &out->field[1], &out->field[2]);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    out->valid   = true;

    return 0;
}

/* ── Mag driver descriptor ──────────────────────────────────────────────── */

const mag_ops_t sim_mag_ops = {
    .name             = "sim",
    .experimental     = false,
    .probe            = sim_mag_probe,
    .reset            = sim_mag_reset,
    .init             = sim_mag_init,
    .read             = sim_mag_read,
    .set_reset        = NULL,
    .has_interrupt    = false,
    .has_set_reset    = false,
    .supported_odr_hz = { 1, 10, 20, 50, 100, 200, 1000, 0 },
};
