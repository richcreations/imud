/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cal.h — calibration data type, loader, and writer
 *
 * imud_cal_t holds all three calibration sections (mag, gyro, accel).
 * cal_load  reads cal.json; a missing file is not an error (daemon runs
 *           uncalibrated with identity scale/soft-iron defaults).
 * cal_write writes cal.json; only sections whose has_* flag is set are
 *           emitted, so a partial calibration never overwrites unrelated data.
 */
#ifndef IMUD_CAL_H
#define IMUD_CAL_H

#include <stdbool.h>

/* ── Calibration data ────────────────────────────────────────────────────── */

typedef struct {
    float accel_offset[3];     /* from cal.accel.offset, m/s² */
    float accel_scale[3];      /* from cal.accel.scale, diagonal, dimensionless */
    float gyro_bias[3];        /* from cal.gyro.bias, rad/s */
    float mag_hard_iron[3];    /* from cal.mag.hard_iron, µT */
    float mag_soft_iron[3][3]; /* from cal.mag.soft_iron, 3×3 */
    bool  has_accel;
    bool  has_gyro;
    bool  has_mag;
} imud_cal_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * Load calibration from path.  Initialises *cal (memset + identity defaults)
 * before parsing, so callers need not zero it first.
 * Missing file: returns 0 with has_* = false (runs uncalibrated).
 * Returns -1 only on I/O error on a file that exists.
 */
int cal_load(const char *path, imud_cal_t *cal);

/*
 * Write calibration to path.
 * Returns 0 on success, -1 on error (logged to stderr).
 */
int cal_write(const char *path, const imud_cal_t *cal);

#endif /* IMUD_CAL_H */
