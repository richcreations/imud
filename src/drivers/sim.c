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
 *
 * PLAYBACK MODE: when [device] sim_file names an .imucap capture (or the
 * daemon was started with --replay FILE), both sim ops replay the recorded
 * raw samples instead of synthesizing — same pacing model, timing anchored
 * to the capture header's t0_mono_ns so IMU/mag relative timing is
 * preserved.  sim_loop repeats the file with seq/chip_ts/time rebased to
 * stay monotonic; sim_speed scales pacing (0 = as fast as possible).
 */

#include <math.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include <time.h>

#include "capture.h"
#include "drivers.h"
#include "log.h"

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

    /* Step 2: Ry(-θ) — row signs [c 0 -s; 0 1 0; s 0 c].  (A +θ rotation
     * here mirrored the pitch axis against the gyro's Euler-rate model —
     * accel/mag said −θ while gyro said +θ̇ — caught by the capture-replay
     * end-to-end test in 1.5.) */
    double ct = cos(theta), st = sin(theta);
    double bx_ =  ct * ax - st * az;
    double by_  =  ay;
    double bz_  =  st * ax + ct * az;

    /* Step 3: Rx(-φ) */
    double cr = cos(phi), sr = sin(phi);
    *bx = (float)bx_;
    *by = (float)(cr * by_ + sr * bz_);
    *bz = (float)(-sr * by_ + cr * bz_);
}

/* ── Exported synthesis hooks (drivers.h) ───────────────────────────────── */

/*
 * Evaluate the motion model at scenario time t.  Exposed so tests and
 * capture-scenario generation can sample the model directly at any t
 * instead of waiting on the driver's real-time pacing.
 */
void sim_synth_imu(double t, imu_sample_t *out)
{
    /* ── Attitude at this sample ───────────────────────────────────── */
    double psi, theta, phi;
    sim_attitude(t, &psi, &theta, &phi);

    /* ── Euler angle rates ─────────────────────────────────────────── */
    double dpsi   = SIM_YAW_RATE_RPS;
    double dphi   = SIM_ROLL_AMP  * 2.0 * M_PI * SIM_ROLL_HZ
                    * cos(2.0 * M_PI * SIM_ROLL_HZ  * t);
    double dtheta = SIM_PITCH_AMP * 2.0 * M_PI * SIM_PITCH_HZ
                    * cos(2.0 * M_PI * SIM_PITCH_HZ * t + SIM_PITCH_PHASE);

    /*
     * ── Body-frame angular velocity (full kinematic formula) ────────
     *
     * For aerospace 3-2-1 Euler (ψ, θ, φ):
     *   ω_x = φ̇ − ψ̇ sin θ
     *   ω_y = θ̇ cos φ + ψ̇ cos θ sin φ
     *   ω_z = −θ̇ sin φ + ψ̇ cos θ cos φ
     */
    out->gyro[0] = (float)(dphi   - dpsi * sin(theta));
    out->gyro[1] = (float)(dtheta * cos(phi) + dpsi * cos(theta) * sin(phi));
    out->gyro[2] = (float)(-dtheta * sin(phi) + dpsi * cos(theta) * cos(phi));

    /*
     * ── Specific force in body frame ─────────────────────────────────
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
                &out->accel[0], &out->accel[1], &out->accel[2]);

    out->temp_c = 25.0f;
}

void sim_synth_mag(double t, mag_sample_t *out)
{
    double psi, theta, phi;
    sim_attitude(t, &psi, &theta, &phi);

    /*
     * Earth's field in NED: [NORTH, 0, DOWN].
     * Rotate into body frame using the full attitude matrix.
     */
    ned_to_body(psi, theta, phi,
                (double)SIM_MAG_NORTH_UT, 0.0, (double)SIM_MAG_DOWN_UT,
                &out->field[0], &out->field[1], &out->field[2]);
    out->valid = true;
}

/* ── Playback mode ([device] sim_file) ──────────────────────────────────── */

/*
 * Each sensor thread replays from its OWN reader session over the same
 * file (no cross-thread stream sharing); a session filters out the other
 * sensor's records.  Pacing uses a shared start time and the capture
 * header's t0_mono_ns base, so the two streams stay mutually consistent.
 * Loop rebasing tracks the WHOLE file's record extent (both sessions see
 * the same records), so per-pass offsets are identical in both streams.
 */
typedef struct {
    cap_reader_t rdr;
    bool         open;
    bool         done;         /* EOF with loop disabled, or error */
    bool         eof_logged;
    cap_record_t pend;
    bool         have_pend;
    uint64_t     last_any_mono;/* newest mono_ns seen this pass, any type */
    uint64_t     n_pass;       /* records of our type seen this pass */
    uint64_t     mono_off;     /* accumulated loop offset, ns */
    uint32_t     seq_off;      /* accumulated imu seq offset */
    uint32_t     first_seq, last_seq;
} pb_stream_t;

static struct {
    bool            enabled;
    bool            loop;
    float           speed;     /* 1.0 realtime; 0 = as fast as possible */
    char            file[256];
    bool            started;
    uint64_t        start_mono;
    uint64_t        start_wall;
    pthread_mutex_t mtx;       /* guards `started` (both threads race once) */
    pb_stream_t     imu;
    pb_stream_t     mag;
} pb = { .mtx = PTHREAD_MUTEX_INITIALIZER };

void sim_set_playback(const char *file, bool loop, float speed)
{
    if (pb.imu.open) cap_reader_close(&pb.imu.rdr);
    if (pb.mag.open) cap_reader_close(&pb.mag.rdr);
    pb.enabled = (file != NULL && file[0] != '\0');
    if (pb.enabled)
        snprintf(pb.file, sizeof(pb.file), "%s", file);
    else
        pb.file[0] = '\0';
    pb.loop    = loop;
    pb.speed   = speed;
    pb.started = false;
    memset(&pb.imu, 0, sizeof(pb.imu));
    memset(&pb.mag, 0, sizeof(pb.mag));
}

static void pb_start_once(void)
{
    pthread_mutex_lock(&pb.mtx);
    if (!pb.started) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        pb.start_wall = (uint64_t)ts.tv_sec * 1000000000ULL
                        + (uint64_t)ts.tv_nsec;
        pb.start_mono = mono_ns();
        pb.started    = true;
        LOG_I("[sim] playback: %s (loop=%s, speed=%.2g)\n", pb.file,
              pb.loop ? "yes" : "no", (double)pb.speed);
    }
    pthread_mutex_unlock(&pb.mtx);
}

/* Record time relative to capture start (clamped), plus loop offset. */
static uint64_t pb_rel(const pb_stream_t *st, uint64_t rec_mono)
{
    uint64_t base = st->rdr.hdr.t0_mono_ns;
    uint64_t rel  = rec_mono >= base ? rec_mono - base : 0;
    return rel + st->mono_off;
}

/*
 * Ensure st->pend holds the next record of `type`.
 * Returns 1 when pending, 0 at end of stream, -1 on error.
 */
static int pb_fetch(pb_stream_t *st, uint8_t type)
{
    if (st->have_pend) return 1;
    if (st->done)      return 0;

    if (!st->open) {
        int rc = cap_reader_open(&st->rdr, pb.file);
        if (rc != 0) {
            LOG_E("[sim] cannot open capture '%s' (%s)\n", pb.file,
                  rc == CAP_ERR_FORMAT ? "bad format/version" : "I/O error");
            st->done = true;
            return -1;
        }
        st->open = true;
    }

    cap_record_t rec;
    for (;;) {
        int rc = cap_reader_next(&st->rdr, &rec);
        if (rc == 1) {
            st->last_any_mono = rec.mono_ns;
            if (rec.type != type) continue;
            if (st->n_pass == 0) st->first_seq = rec.imu.seq;
            st->last_seq = rec.imu.seq;
            st->n_pass++;
            st->pend      = rec;
            st->have_pend = true;
            return 1;
        }
        if (rc != 0) {            /* read error */
            st->done = true;
            return -1;
        }
        /* EOF */
        if (!pb.loop || st->last_any_mono == 0) {
            st->done = true;
            if (!st->eof_logged) {
                st->eof_logged = true;
                LOG_I("[sim] playback finished: %s\n", pb.file);
            }
            return 0;
        }
        /*
         * Loop: rebase so the next pass continues seamlessly.  The offset
         * derives from the WHOLE file's extent + one nominal IMU period,
         * so both sensor streams advance identically.
         */
        uint32_t odr = st->rdr.hdr.imu_odr_hz ? st->rdr.hdr.imu_odr_hz : 100;
        uint64_t period = 1000000000ULL / odr;
        uint64_t base   = st->rdr.hdr.t0_mono_ns;
        uint64_t span   = st->last_any_mono >= base
                          ? st->last_any_mono - base : 0;
        st->mono_off += span + period;
        if (type == CAP_REC_IMU && st->n_pass > 0)
            st->seq_off += st->last_seq - st->first_seq + 1;
        st->n_pass = 0;
        if (cap_reader_rewind(&st->rdr) != 0) {
            st->done = true;
            return -1;
        }
    }
}

/* Playback pacing budget: how much capture time may be delivered by now. */
static bool pb_due(uint64_t rel)
{
    if (pb.speed <= 0.0f) return true;   /* as fast as possible */
    uint64_t elapsed = mono_ns() - pb.start_mono;
    return rel <= (uint64_t)((double)elapsed * (double)pb.speed);
}

static int pb_imu_read(imu_sample_t *buf, int max, int *n_out)
{
    pb_start_once();

    int n = 0;
    while (n < max) {
        if (pb_fetch(&pb.imu, CAP_REC_IMU) <= 0) break;
        uint64_t rel = pb_rel(&pb.imu, pb.imu.pend.mono_ns);
        if (!pb_due(rel)) break;
        buf[n] = pb.imu.pend.imu;
        buf[n].seq += pb.imu.seq_off;
        /* chip ticks are 25 µs — keep them consistent with the loop offset
         * so the daemon's chip→wall anchor never sees time run backwards */
        buf[n].chip_ts += (uint32_t)(pb.imu.mono_off / 25000u);
        n++;
        pb.imu.have_pend = false;
    }
    *n_out = n;
    return 0;
}

static int pb_mag_read(mag_sample_t *out)
{
    pb_start_once();

    if (pb_fetch(&pb.mag, CAP_REC_MAG) <= 0) return 1;   /* no data yet */
    uint64_t rel = pb_rel(&pb.mag, pb.mag.pend.mono_ns);
    if (!pb_due(rel)) return 1;

    *out = pb.mag.pend.mag;
    /* Remap the mag timestamp into replay time (stretched by 1/speed so
     * downstream sees self-consistent spacing; fast mode keeps file time). */
    uint64_t rel_out = (pb.speed > 0.0f && pb.speed != 1.0f)
                       ? (uint64_t)((double)rel / (double)pb.speed) : rel;
    out->wall_ns = pb.start_wall + rel_out;
    pb.mag.have_pend = false;
    return 0;
}

/* ── IMU sim state ──────────────────────────────────────────────────────── */

static struct {
    uint64_t t_last_ns;   /* end of the most-recently-delivered sample batch */
    uint32_t seq;
    int      odr_hz;
} imu_s;

static int sim_imu_probe(const imud_bus_t *bus) { (void)bus; return 0; }
static int sim_imu_reset(const imud_bus_t *bus) { (void)bus; return 0; }

static int sim_imu_init(const imud_bus_t *bus, const imu_cfg_t *cfg)
{
    (void)bus;
    imu_s.t_last_ns = mono_ns();
    imu_s.seq       = 0;
    imu_s.odr_hz    = cfg->odr_hz > 0 ? cfg->odr_hz : 100;
    if (!sim_t0_set) { sim_t0_ns = imu_s.t_last_ns; sim_t0_set = true; }
    return 0;
}

static int sim_imu_read(const imud_bus_t *bus,
                        imu_sample_t *buf, int max, int *n_out)
{
    (void)bus;

    if (pb.enabled) return pb_imu_read(buf, max, n_out);

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

        sim_synth_imu(t, &buf[i]);
        buf[i].seq     = imu_s.seq;
        buf[i].chip_ts = imu_s.seq * ticks_per;   /* wraps at ~29.8 h */
        imu_s.seq++;
    }

    *n_out = n;
    return 0;
}

/* ── IMU driver descriptor ──────────────────────────────────────────────── */

/*
 * imu_ops_t / mag_ops_t ::actual_odr_hz. The sim paces itself at exactly the
 * rate it is handed (sim_imu_init above) rather than snapping to the table it
 * advertises, so identity is the honest answer — anything else would make the
 * filter tune for a rate the sim is not producing.
 */
static int sim_actual_odr_hz(int requested) { return requested; }

const imu_ops_t sim_imu_ops = {
    .name               = "sim",
    .experimental       = false,
    /* The sim driver never touches the handle, so it imposes no transport
     * constraint of its own. spi_dev must still name a real spidev node:
     * bus_open configures mode and speed on it before any driver runs. */
    .bus_caps           = { .spi_capable = true, .spi_mode = 3,
                            .spi_max_hz = 10000000 },
    .probe              = sim_imu_probe,
    .reset              = sim_imu_reset,
    .init               = sim_imu_init,
    .read               = sim_imu_read,
    .has_fifo           = true,
    .has_hw_timestamp   = true,
    .ts_tick_ns         = 25000,  /* mimics the ISM330DHCX 25 µs timer */
    .supported_odr_hz   = { 12, 26, 52, 104, 208, 416, 833, 1660, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
    .actual_odr_hz      = sim_actual_odr_hz,
};

/* ── Mag sim state ──────────────────────────────────────────────────────── */

static int sim_mag_probe(const imud_bus_t *bus) { (void)bus; return 0; }
static int sim_mag_reset(const imud_bus_t *bus) { (void)bus; return 0; }

static int sim_mag_init(const imud_bus_t *bus, const mag_cfg_t *cfg)
{
    (void)bus; (void)cfg;
    if (!sim_t0_set) { sim_t0_ns = mono_ns(); sim_t0_set = true; }
    return 0;
}

static int sim_mag_read(const imud_bus_t *bus, mag_sample_t *out)
{
    (void)bus;

    if (pb.enabled) return pb_mag_read(out);

    sim_synth_mag(elapsed_s(), out);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

    return 0;
}

/* ── Mag driver descriptor ──────────────────────────────────────────────── */

const mag_ops_t sim_mag_ops = {
    .name             = "sim",
    .experimental     = false,
    .bus_caps         = { .spi_capable = true, .spi_mode = 3,
                          .spi_max_hz = 10000000 },   /* see sim_imu_ops */
    .probe            = sim_mag_probe,
    .reset            = sim_mag_reset,
    .init             = sim_mag_init,
    .read             = sim_mag_read,
    .set_reset        = NULL,
    .has_interrupt    = false,
    .has_set_reset    = false,
    .supported_odr_hz = { 1, 10, 20, 50, 100, 200, 1000, 0 },
    .actual_odr_hz    = sim_actual_odr_hz,
};
