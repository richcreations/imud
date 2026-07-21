/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu_math.h — pure helpers factored out of imu.c (§3) so they can be unit
 * tested without the reader/fusion threads, GPIO, or I2C.
 *
 * Calibration application, chip-timer → wall-clock reconstruction, ODR
 * rounding, and mount rotation are self-contained transforms: they touch only
 * their arguments (chip_to_wall/anchor_update also the ts_anchor_t mutex).
 * imu.c #includes this and calls through; test/test_imu_math.c links
 * src/imu_math.c directly.
 */
#ifndef IMUD_IMU_MATH_H
#define IMUD_IMU_MATH_H

#include <stdint.h>
#include <time.h>
#include <pthread.h>

#include "types.h"
#include "cal.h"
#include "config.h"

/* ── Timestamp anchor ───────────────────────────────────────────────────── */

typedef struct {
    uint64_t        wall_ns;    /* CLOCK_REALTIME at anchor point, ns */
    uint64_t        tai_ns;     /* CLOCK_TAI at anchor point, ns */
    uint32_t        chip_ticks; /* IMU hardware counter at anchor point */
    uint32_t        gen;        /* incremented on each update */
    pthread_mutex_t mtx;
} ts_anchor_t;

/* ── Calibration helpers ─────────────────────────────────────────────────── */

/* Apply accel offset/scale and gyro temperature compensation from cal.json on
 * top of the driver's chip-level scaling. Gyro bias is NOT subtracted here —
 * the MEKF subtracts it during predict. */
void apply_imu_cal(const imud_cal_t *cal, imu_sample_t *s);

/* Apply hard/soft-iron correction: m_cal = soft_iron × (m_raw − hard_iron). */
void apply_mag_cal(const imud_cal_t *cal, mag_sample_t *s);

/* ── Timestamp helpers ───────────────────────────────────────────────────── */

uint64_t ts_ns(const struct timespec *t);

/* Advance an absolute timespec by ns nanoseconds (carries into tv_sec).
 * static inline so consumers compiled without imu_math.c (test_stream) need
 * no link change. */
static inline void ts_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

void anchor_update(ts_anchor_t *a, uint32_t chip_ts,
                   uint64_t wall_ns, uint64_t tai_ns);

/* Convert a chip counter value to wall + TAI timestamps using 32-bit wrapping
 * arithmetic (see the definition for the wrap bounds). */
void chip_to_wall(ts_anchor_t *a, uint32_t chip_ts, uint32_t tick_ns,
                  uint64_t *wall_out, uint64_t *tai_out, uint32_t *gen_out);

/* ── Utilities ───────────────────────────────────────────────────────────── */

/* Nearest value in a 0-terminated ascending table to `requested`. */
int nearest_odr(const int supported[], int requested);

/* Apply mount rotation (board -> body) if configured. In-place on v. */
void apply_mount_rot_if_set(const imud_config_t *cfg, float v[3]);

#endif /* IMUD_IMU_MATH_H */
