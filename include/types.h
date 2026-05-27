/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * types.h — core data structures shared across all imud threads
 *
 * Wire packet is little-endian, 192 bytes. All angle units are radians
 * unless the field name ends in _deg. Magnetic field in µT. Accel in m/s².
 * Gyro in rad/s.
 */
#ifndef IMUD_TYPES_H
#define IMUD_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>

/* ── Packet constants ──────────────────────────────────────────────────────── */

#define IMUD_MAGIC    0x494D5544u   /* "IMUD" */
#define IMUD_VERSION  10   /* 1.0 — encoded as decimal: major*10 + minor */

/* ── Packet flags (§8) — bitmask in imu_packet_t.flags and fused_state_t.flags */

#define FLAG_MAG_VALID        (1u <<  0)  /* mag calibrated and sensor healthy */
#define FLAG_MAG_SET_RESET    (1u <<  1)  /* SET pulse within last 1 ms */
#define FLAG_FUSION_CONVERGED (1u <<  2)  /* MEKF cov trace < threshold */
#define FLAG_ACCEL_CAL        (1u <<  3)  /* accel calibration applied */
#define FLAG_GYRO_CAL         (1u <<  4)  /* gyro bias applied */
#define FLAG_MAG_CAL          (1u <<  5)  /* mag hard/soft-iron cal applied */
#define FLAG_MOTION           (1u <<  6)  /* significant motion detected */
#define FLAG_FIFO_OVERFLOW    (1u <<  7)  /* ISM330 FIFO overflowed (gap!) */
#define FLAG_STARTUP          (1u <<  8)  /* gyro bias estimation in progress */
#define FLAG_SHUTDOWN         (1u <<  9)  /* final packet before clean exit */
#define FLAG_DECLINATION_VALID (1u << 10) /* WMM/static declination known; true_heading valid */

/* ── IMU sample — one calibrated sample from the configured IMU ────────────── */

typedef struct {
    float    accel[3];       /* m/s², calibrated, body frame XYZ */
    float    accel_raw[3];   /* m/s², pre-calibration (after mount rotation) */
    float    gyro[3];        /* rad/s, bias-corrected, body frame XYZ */
    float    temp_c;         /* °C, die temperature (256 LSB/°C, 0 = 25 °C) */
    uint32_t chip_ts;        /* ISM330DHCX TIMESTAMP counter (25 µs/tick, 32-bit) */
    uint32_t seq;            /* monotonic counter across all FIFO bursts */
} imu_sample_t;

/* ── Magnetometer sample — one MMC5983MA reading ──────────────────────────── */

typedef struct {
    float    field[3];       /* µT, calibrated; field[2] has Z sign flipped */
    float    field_raw[3];   /* µT, pre-calibration (after mount rotation) */
    uint64_t wall_ns;        /* CLOCK_REALTIME at read time (ns) */
    bool     valid;          /* false: uncalibrated or sensor fault */
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
    uint16_t flags;          /* FLAG_* bitmask */
    uint32_t imu_seq;        /* ISM330 sample counter of last prediction step */
    uint64_t ts_wall_ns;     /* CLOCK_REALTIME of last prediction step (ns) */
    uint64_t ts_tai_ns;      /* CLOCK_TAI of last prediction step (ns) */
    uint32_t ts_chip_ticks;  /* ISM330 counter of last prediction step */
    uint32_t anchor_gen;     /* increments each time wall-clock anchor is reset */
} fused_state_t;

/* ── Wire packet — §8, 192 bytes fixed, little-endian ─────────────────────── */

typedef struct __attribute__((packed)) {
    /* Header — 32 bytes */
    uint32_t magic;          /* IMUD_MAGIC */
    uint16_t version;        /* IMUD_VERSION */
    uint16_t flags;          /* FLAG_* bitmask */
    uint64_t ts_wall_ns;     /* CLOCK_REALTIME, ns */
    uint64_t ts_tai_ns;      /* CLOCK_TAI, ns */
    uint32_t ts_chip_ticks;  /* ISM330 25 µs counter */
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
    uint32_t crc32;          /* IEEE 802.3 CRC32 of bytes 0–187 */
} imu_packet_t;

_Static_assert(sizeof(imu_packet_t) == 192,
               "imu_packet_t must be exactly 192 bytes");
_Static_assert(offsetof(imu_packet_t, crc32) == 188,
               "crc32 must be at offset 188");

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
