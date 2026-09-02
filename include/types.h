/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * types.h — core data structures shared across all imud threads
 *
 * Wire packet is little-endian, 276 bytes. All angle units are radians
 * unless the field name ends in _deg. Magnetic field in µT. Accel in m/s².
 * Gyro in rad/s.
 */
#ifndef IMUD_TYPES_H
#define IMUD_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>

/* The wire packet below is a HOST-ORDER struct.  It reaches a socket only
 * through packet_encode()/packet_decode() (src/packet.c), which convert to and
 * from the little-endian wire layout with shifts, so the host may be either
 * endianness and the bytes on the wire are the same on both. */

/* ── Packet constants ──────────────────────────────────────────────────────── */

#define IMUD_MAGIC    0x494D5544u   /* "IMUD" */
#define IMUD_PACKET_BYTES 276u      /* encoded wire size, fixed */
/* Wire-layout revision, NOT the release version (that is IMUD_VERSION_STR in
 * version.h).  Encoded as major*10 + minor of the release that last CHANGED the
 * packet layout — 17 = the layout introduced in 1.7 (update-gate health fields);
 * 14 was the 1.4 layout, which 1.5 and 1.6 shipped unchanged.
 * Bump only when the layout changes; see docs/RELEASING.md. */
#define IMUD_VERSION  17

/* ── Packet flags (§8) — bitmask in imu_packet_t.flags and fused_state_t.flags */

#define FLAG_MAG_VALID        (1u <<  0)  /* mag calibrated and sensor healthy */
#define FLAG_MAG_SET_RESET    (1u <<  1)  /* SET pulse within last 1 ms */
#define FLAG_FUSION_CONVERGED (1u <<  2)  /* MEKF cov trace < threshold */
#define FLAG_ACCEL_CAL        (1u <<  3)  /* accel calibration applied */
#define FLAG_GYRO_CAL         (1u <<  4)  /* gyro bias applied */
#define FLAG_MAG_CAL          (1u <<  5)  /* mag hard/soft-iron cal applied */
/* Bit 6 is RETIRED, not pending.  It was reserved for a "platform is moving"
 * indicator and never set.  The packet already carries that information at
 * higher fidelity — accel_quiescence as a continuous float, plus
 * FLAG_ENGINE_ON (bit 13) — so a boolean restatement would only lose
 * resolution.  The define stays so existing consumers still compile; the bit
 * will never be set.  Do not reuse it for something else: a stale consumer
 * would read the new meaning through the old name. */
#define FLAG_MOTION           (1u <<  6)  /* retired — never set, never will be */
#define FLAG_FIFO_OVERFLOW    (1u <<  7)  /* ISM330 FIFO overflowed (gap!) */
#define FLAG_STARTUP          (1u <<  8)  /* gyro bias estimation in progress */
#define FLAG_SHUTDOWN         (1u <<  9)  /* final packet before clean exit */
#define FLAG_DECLINATION_VALID (1u << 10) /* WMM/static declination known; true_heading valid */
#define FLAG_HEAVE_VALID       (1u << 11) /* heave estimator has settled (heave_m/heave_rate valid) */
#define FLAG_WAVE_VALID        (1u << 12) /* sea-state stats settled (wave/roll/pitch fields valid) */
#define FLAG_ENGINE_ON         (1u << 13) /* engine-vibration detector currently asserting */
/* The MEKF found a non-finite value in its own state and reset itself.
 * Latched from the reset until the filter next converges, NOT a single-packet
 * pulse: at up to 500 Hz a momentary bit is invisible to a 1 Hz consumer, and
 * a fault nobody can observe is not worth a bit of the wire format.  While it
 * is set the filter is re-aligning, so the attitude is valid but unconverged —
 * FLAG_FUSION_CONVERGED is clear for the same span. */
#define FLAG_STATE_RESET       (1u << 14) /* MEKF reset itself after non-finite state */
/* Heading is being corrected by a magnetometer that has NO calibration, so
 * the uncorrected hard iron offsets it by a deviation that varies with
 * heading — bounded by asin(|b|/|H|) and repeatable, which a heading-hold
 * consumer can use, but not a number to navigate on.  Mutually exclusive with
 * FLAG_MAG_VALID, which keeps its original meaning and stays clear whenever
 * there is no calibration; both clear means heading is dead-reckoned from the
 * gyro and drifts without bound.  A consumer that only ever tested
 * FLAG_MAG_VALID therefore behaves exactly as before. */
#define FLAG_MAG_UNCAL         (1u << 15) /* fused from an uncalibrated field */

/* ── IMU sample — one calibrated sample from the configured IMU ────────────── */

typedef struct {
    float    accel[3];       /* m/s², calibrated, body frame XYZ */
    float    accel_raw[3];   /* m/s², pre-calibration (after mount rotation) */
    float    gyro[3];        /* rad/s, bias-corrected, body frame XYZ */
    float    temp_c;         /* °C, die temperature (256 LSB/°C, 0 = 25 °C) */
    uint32_t chip_ts;        /* ISM330DHCX TIMESTAMP counter (25 µs/tick, 32-bit) */
    uint32_t seq;            /* monotonic counter across all FIFO bursts */
    /*
     * CLOCK_REALTIME at which the I2C burst carrying this sample COMPLETED.
     * Daemon-internal: it is not on the wire and not in a capture record
     * (cap_imu_rec_t is built field-by-field, so the .imucap format is
     * untouched).  It exists to split sample latency into its two terms —
     * this minus the sample instant is FIFO residence, and the fusion
     * thread's clock minus this is the daemon's own pipeline.  Only their
     * sum was ever discussed, and only the second is under imud's control.
     */
    uint64_t read_done_ns;
    /*
     * The same instant on THIS host's CLOCK_REALTIME, which is a different
     * clock from read_done_ns only during replay: there read_done_ns is
     * rebuilt on the recording host's clock so the FIFO term reproduces what
     * the live daemon measured, and subtracting it from this machine's clock
     * would measure the age of the recording instead (a capture replayed days
     * later reported 416010576.8 ms).  The pipeline term is imud's own cost
     * on the machine actually running, so it needs this one.  Live, the two
     * are set from the same value.
     */
    uint64_t host_done_ns;
} imu_sample_t;

/* ── Magnetometer sample — one MMC5983MA reading ──────────────────────────── */

typedef struct {
    float    field[3];       /* µT, calibrated; field[2] has Z sign flipped */
    float    field_raw[3];   /* µT, pre-calibration (after mount rotation) */
    uint64_t wall_ns;        /* CLOCK_REALTIME at read time (ns) */
    /*
     * Two independent statements, deliberately not merged.  valid is the
     * DRIVER's: the reading came back and is usable.  calibrated is the
     * CALIBRATION's: hard/soft-iron correction was applied to field[].  An
     * uncalibrated sample is still fused (heading-only); only an invalid one
     * is discarded.
     */
    bool     valid;          /* sensor reading is good */
    bool     calibrated;     /* hard/soft-iron cal applied to field[] */
} mag_sample_t;

/* ── MEKF fused state — written by fusion thread, read by output threads ───── */

typedef struct {
    float    q[4];           /* unit quaternion [w, x, y, z] */
    float    bias_gyro[3];   /* estimated gyro bias, rad/s */
    float    cov[9];         /* 3×3 attitude error covariance, row-major (rad²) */
    float    pitch;          /* rad, NED (+bow up) */
    float    roll;           /* rad, NED (+starboard up) */
    float    yaw;            /* rad, NED magnetic */
    float    heading_deg;    /* 0–360° magnetic */
    float    declination_deg; /* °E+; valid only when FLAG_DECLINATION_VALID set */
    float    rate_of_turn;   /* deg/min, derived from yaw rate */
    float    heave_m;        /* vertical displacement, m, + up; 0 when disabled */
    float    heave_rate;     /* vertical velocity, m/s, + up; 0 when disabled */
    float    bias_gyro_var[3]; /* gyro-bias variance, (rad/s)², MEKF P diagonal */
    float    quiescence;     /* accel-quiescence EMA (|a|/g−1)²; disturbance metric */
    float    wave_height_m;  /* significant wave height Hs, m; 0 when not settled */
    float    wave_period_s;  /* mean zero-crossing wave period Tz, s; 0 = n/a */
    float    roll_period_s;  /* vessel roll period, s; 0 = not rolling / n/a */
    float    roll_amplitude; /* significant single amplitude 2σ(roll), rad */
    float    pitch_period_s; /* vessel pitch period, s; 0 = not pitching / n/a */
    float    pitch_amplitude;/* significant single amplitude 2σ(pitch), rad */
    float    mag_anomaly;    /* EMA of ||B|−|B_ref||/|B_ref|; interference metric */
    float    mag_residual;   /* EMA of |heading innovation|, rad; compass cal health */
    float    innov_weight;   /* EMA of Huber weight √(γ/d²); 1 = no capping */
    float    innov_reject;   /* EMA of gate-reject indicator; 0 = nothing rejected */
    float    nis_accel;      /* EMA of accel d²/2; 1 = covariance consistent, >1 over-confident */
    float    nis_mag;        /* EMA of mag d²/dof; 1 = covariance consistent */
    uint16_t flags;          /* FLAG_* bitmask */
    uint32_t imu_seq;        /* IMU sample counter of last prediction step */
    /*
     * WHEN THE SAMPLE WAS TAKEN — not when this state was computed or sent.
     *
     * Do not word this "CLOCK_REALTIME of last prediction step": that reads as
     * emit-side time and hid what these are for.  src/imu.c sets them from
     * chip_to_wall() on the sample's own chip counter, so they carry the
     * instant the SENSOR sampled, reconstructed against an anchor that also
     * corrects the chip oscillator's measured period.
     *
     * That makes them the correlation timestamp: a consumer can align an imud
     * packet with a camera frame, and `now() - ts_wall_ns` at receipt is the
     * end-to-end age of the estimate, FIFO residence included.  (Over a network
     * that subtraction is only as good as the two clocks' agreement.)
     *
     * On a driver with no hardware timestamp counter (has_hw_timestamp = false:
     * icm20948, mpu925x) chip_ts is always 0 and the anchor is instead refreshed
     * every burst from the read midpoint, so these degrade to the time of the
     * I2C READ that delivered the sample — burst-granular, and it cannot see
     * how long the sample sat in the FIFO first.
     */
    uint64_t ts_wall_ns;     /* CLOCK_REALTIME the sample was taken (ns) */
    uint64_t ts_tai_ns;      /* CLOCK_TAI, same instant (ns) */
    uint32_t ts_chip_ticks;  /* the chip counter that instant came from */
    uint32_t anchor_gen;     /* increments each time wall-clock anchor is reset */
} fused_state_t;

/* ── Wire packet — §8, 276 bytes fixed, little-endian ─────────────────────── */

typedef struct __attribute__((packed)) {
    /* Header — 32 bytes */
    uint32_t magic;          /* IMUD_MAGIC */
    uint16_t version;        /* IMUD_VERSION */
    uint16_t flags;          /* FLAG_* bitmask */
    /* Sample-taken instant, not emit time — see fused_state_t above for the
     * full contract and the has_hw_timestamp caveat. */
    uint64_t ts_wall_ns;     /* CLOCK_REALTIME the sample was taken, ns */
    uint64_t ts_tai_ns;      /* CLOCK_TAI, same instant, ns */
    uint32_t ts_chip_ticks;  /* chip counter that instant came from */
    uint32_t anchor_gen;
    /* Accelerometer — calibrated then raw, m/s² */
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    accel_raw_x;    /* pre-calibration (after mount rotation) */
    float    accel_raw_y;
    float    accel_raw_z;
    /* Gyroscope — bias-corrected then raw, rad/s */
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    float    gyro_raw_x;     /* before bias correction */
    float    gyro_raw_y;
    float    gyro_raw_z;
    /* Magnetometer — calibrated then raw, µT */
    float    mag_x;
    float    mag_y;
    float    mag_z;
    float    mag_raw_x;      /* pre-calibration (after mount rotation) */
    float    mag_raw_y;
    float    mag_raw_z;
    /* Fused attitude */
    float    quat_w;
    float    quat_x;
    float    quat_y;
    float    quat_z;
    float    pitch;          /* rad, NED */
    float    roll;           /* rad, NED */
    float    yaw;            /* rad, NED magnetic */
    float    heading_deg;    /* 0–360° */
    float    rate_of_turn;  /* deg/min, + = turning right (clockwise from above) */
    float    temp_c;         /* °C */
    float    cov[9];         /* 3×3 row-major, rad² */
    uint32_t imu_seq;        /* monotonic ISM330 sample counter */
    float    declination_deg; /* °E+; 0.0 when FLAG_DECLINATION_VALID not set */
    float    heave_m;        /* vertical displacement, m, + up (v11); 0.0 when
                              * the heave estimator is disabled */
    /* v12 additions — body-frame / frame-neutral (NOT coord_frame-converted) */
    float    gyro_bias_x;    /* estimated gyro bias, rad/s, IMU body frame */
    float    gyro_bias_y;
    float    gyro_bias_z;
    float    gyro_bias_var_x; /* gyro-bias variance, (rad/s)², MEKF P diagonal */
    float    gyro_bias_var_y;
    float    gyro_bias_var_z;
    float    heave_rate;     /* vertical velocity, m/s, + up (v12); 0.0 when disabled */
    float    accel_quiescence; /* EMA of (|a|/g − 1)²; platform-disturbance metric */
    /* v14 additions — sea state + compass health, all frame-neutral scalars */
    float    wave_height_m;  /* significant wave height Hs = 4σ(heave), m;
                              * 0.0 until FLAG_WAVE_VALID */
    float    wave_period_s;  /* mean zero-crossing wave period Tz, s;
                              * 0.0 when becalmed or not settled */
    float    roll_period_s;  /* vessel roll period, s; 0.0 when not rolling */
    float    roll_amplitude; /* significant single amplitude 2σ(roll), rad */
    float    pitch_period_s; /* vessel pitch period, s; 0.0 when not pitching */
    float    pitch_amplitude;/* significant single amplitude 2σ(pitch), rad */
    float    mag_anomaly;    /* EMA of ||B|−|B_ref||/|B_ref| (unitless) */
    float    mag_residual;   /* EMA of |heading innovation|, rad */
    /* v17 additions — MEKF update-gate health */
    float    innov_weight;   /* EMA of the Huber weight √(γ/d²) applied to accepted
                              * updates: 1.0 = never capped, → 0.33 = sustained
                              * capping at the reject boundary */
    float    innov_reject;   /* EMA of the reject indicator: fraction of updates
                              * discarded by the gross-outlier innovation gate */
    /* v17 additions — MEKF measurement-model consistency (rolling NIS).
     * Normalised innovation squared d²/dof, EMA over τ ≈ 30 s, accumulated
     * BEFORE the Huber cap and including gate-rejected updates. 1.0 = the
     * filter's covariance correctly predicts its own innovation spread;
     * > 1 = over-confident. Where innov_weight/innov_reject report how hard
     * the robustness machinery is working, these report whether the noise
     * model itself is right. See docs/math.md §4.7. */
    float    nis_accel;      /* accel gravity update, d²/2 */
    float    nis_mag;        /* mag update, d²/2 (3-D) or d²/1 (yaw-only) */
    uint32_t crc32;          /* IEEE 802.3 CRC32 of bytes 0–271 */
} imu_packet_t;

_Static_assert(sizeof(imu_packet_t) == 276,
               "imu_packet_t must be exactly 276 bytes");
_Static_assert(sizeof(imu_packet_t) == IMUD_PACKET_BYTES,
               "IMUD_PACKET_BYTES must match the struct the encoder walks");
_Static_assert(offsetof(imu_packet_t, crc32) == 272,
               "crc32 must be at offset 272");

/* ── IMU ring buffer — ism_reader → fusion ─────────────────────────────────── */

#define IMU_RING_LEN  256   /* power of 2; ~0.3 s at 833 Hz */

typedef struct {
    imu_sample_t    buf[IMU_RING_LEN];
    unsigned        head;   /* next write index */
    unsigned        tail;   /* next read index */
    unsigned        count;  /* samples available */
    pthread_mutex_t lock;
    pthread_cond_t  ready;
} imu_ring_t;

/* ── Mag ring buffer — mag_reader → fusion ─────────────────────────────────── */

#define MAG_RING_LEN  32    /* ~0.3 s at 100 Hz */

typedef struct {
    mag_sample_t    buf[MAG_RING_LEN];
    unsigned        head;
    unsigned        tail;
    unsigned        count;
    pthread_mutex_t lock;
    pthread_cond_t  ready;
} mag_ring_t;

/* ── Shared fused state — fusion → output threads ──────────────────────────── */

typedef struct {
    fused_state_t   state;
    mag_sample_t    latest_mag;  /* always the most recent mag reading */
    pthread_mutex_t lock;
} shared_state_t;

#endif /* IMUD_TYPES_H */
