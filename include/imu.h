/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu.h — reader threads, ring buffers, and fusion thread for imud (§3)
 *
 * Callers (main.c, output threads) work through an opaque imu_ctx_t:
 *   imu_ctx_open   — open I2C + GPIO, probe + init both sensors
 *   ism_reader_thread / mag_reader_thread / fusion_thread — pass to pthread_create
 *   imu_get_state  — snapshot latest fused output (thread-safe)
 *   imu_ctx_stop   — signal threads to exit
 *   imu_ctx_free   — release all resources after threads have joined
 */

#ifndef IMUD_IMU_H
#define IMUD_IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "types.h"
#include "config.h"

/* ── Calibration data (defined in cal.h) ───────────────────────────────── */

#include "cal.h"   /* imud_cal_t */

/* ── Opaque context (defined in imu.c) ─────────────────────────────────── */

typedef struct imu_ctx imu_ctx_t;

/* ── Thread statistics ──────────────────────────────────────────────────── */

typedef struct {
    uint64_t imu_samples;      /* total IMU samples pushed to ring */
    uint64_t mag_samples;      /* total mag samples pushed to ring */
    uint64_t fifo_overflows;   /* IMU ring drop events (software) + hardware overflows */
    uint64_t imu_errors;       /* consecutive-error-reset events from ism_reader */
    uint64_t mag_errors;       /* consecutive-error-reset events from mag_reader */
} imu_stats_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/*
 * Open /dev/i2cN, probe and initialise both sensors, request GPIO lines.
 * cal may be NULL (no calibration applied, all flags = 0).
 * Returns 0 on success, -1 on any fatal error (logged to stderr).
 * Must complete before the reader threads are started.
 */
int imu_ctx_open(imu_ctx_t **ctx_out,
                 const imud_config_t *cfg,
                 const imud_cal_t *cal);

/* Signal all threads to stop (safe to call from signal handler). */
void imu_ctx_stop(imu_ctx_t *ctx);

/*
 * Push updated hot-reloadable fusion config into a running context.
 * Copies the MEKF noise and threshold fields from new_cfg into the
 * context's internal config copy and signals fusion_thread to recompute
 * its derived filter scalars on the next predict step.
 * Safe to call from the main thread while fusion_thread is running.
 */
void imu_ctx_update_config(imu_ctx_t *ctx, const imud_config_t *new_cfg);

/*
 * Update the magnetic declination live, without a full config reload.
 * Called by the position thread whenever a GPS fix triggers a WMM recompute.
 * The fusion thread reads pos_declination_deg on every predict step, so the
 * new value takes effect within one IMU sample period.
 * Safe to call from any thread while fusion_thread is running.
 */
void imu_ctx_set_declination(imu_ctx_t *ctx, float decl_deg);

/* Release I2C fd, GPIO lines, ring buffers, and the context itself.
 * Call only after all three threads have been joined. */
void imu_ctx_free(imu_ctx_t *ctx);

/*
 * Thread entry points — pass the same imu_ctx_t * as the void * argument
 * to each pthread_create call.
 */
void *ism_reader_thread(void *arg);
void *mag_reader_thread(void *arg);
void *fusion_thread(void *arg);

/*
 * Snapshot the latest fused state (thread-safe).
 * mag_out:     most recent mag reading (valid flag set only when cal present).
 * imu_out:     most recent IMU sample with bias-corrected gyro.
 * raw_imu_out: pre-calibration snapshot: accel_raw[] before cal correction,
 *              gyro[] before bias subtraction.
 * Any pointer may be NULL if that field is not needed.
 */
void imu_get_state(imu_ctx_t *ctx,
                   fused_state_t *state_out,
                   mag_sample_t  *mag_out,
                   imu_sample_t  *imu_out,
                   imu_sample_t  *raw_imu_out);

void imu_get_stats(imu_ctx_t *ctx, imu_stats_t *out);

/*
 * Returns true once fusion_thread has completed settle + gyro bias +
 * alignment and entered the main prediction loop.  Output threads and
 * the stats ticker should suppress output until this returns true.
 */
bool imu_ctx_is_settled(imu_ctx_t *ctx);

#endif /* IMUD_IMU_H */
