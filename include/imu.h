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
#include <stddef.h>
#include <stdint.h>
#include "types.h"
#include "config.h"

/* ── Calibration data (defined in cal.h) ───────────────────────────────── */

#include "cal.h"       /* imud_cal_t */
#include "imu_math.h"  /* lat_hist_t */

/* ── Opaque context (defined in imu.c) ─────────────────────────────────── */

typedef struct imu_ctx imu_ctx_t;

/* ── Thread statistics ──────────────────────────────────────────────────── */

typedef struct {
    uint64_t imu_samples;      /* total IMU samples pushed to ring */
    uint64_t mag_samples;      /* total mag samples pushed to ring */
    uint64_t fifo_overflows;   /* IMU ring drop events (software) + hardware overflows */
    uint64_t imu_errors;       /* consecutive-error-reset events from ism_reader */
    uint64_t mag_errors;       /* consecutive-error-reset events from mag_reader */

    /*
     * Sample latency, ns.  p50/p99 describe the most recent ~1 s window;
     * max is since start, because the worst excursion is what diagnoses a
     * stall and a per-window reset would hide it.
     *
     * fifo_*  sample taken -> I2C read complete.  Dominated by fifo_wm/odr,
     *         so it is the operator's knob, not the daemon's.
     * pipe_*  read complete -> state fused.  This is imud's own cost, and
     *         the one spec.md §14's 1.5/3 ms budgets can fairly be read
     *         against.
     *
     * Zero on a driver with no hardware timestamp counter: without one there
     * is no sample instant to subtract, so the split is unmeasurable and
     * reporting the pipeline alone would invite reading it as the total.
     */
    uint64_t lat_fifo_p50_ns, lat_fifo_p99_ns, lat_fifo_max_ns;
    uint64_t lat_pipe_p50_ns, lat_pipe_p99_ns, lat_pipe_max_ns;
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
 * Samples read but not yet consumed by the fusion thread.
 *
 * For shutting down a finished replay without discarding its tail:
 * imu_ctx_stop() makes imu_ring_pop() return at once, so whatever is still
 * queued is dropped.  Waiting for this to reach zero first means every
 * recorded sample got through the filter.
 */
int imu_ctx_ring_backlog(imu_ctx_t *ctx);

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
 * Called by the position thread whenever a GPS fix triggers a WMM recompute
 * (valid = true) or the fix-TTL watchdog expires (valid = false).
 * `valid` drives FLAG_DECLINATION_VALID explicitly, so a genuine 0.0°
 * declination on the agonic line is still reported as valid.
 * The fusion thread reads these fields on every predict step, so the
 * new value takes effect within one IMU sample period.
 * Safe to call from any thread while fusion_thread is running.
 */
void imu_ctx_set_declination(imu_ctx_t *ctx, float decl_deg, bool valid);

/*
 * Push WMM-derived magnetic-field invariants (horizontal magnitude and
 * vertical component, Gauss) into the running filter — the fusion thread
 * applies them direction-preservingly via mekf_set_mref_invariants().
 * Called by the position thread whenever WMM is recomputed.
 */
void imu_ctx_set_mag_ref(imu_ctx_t *ctx, float h_gauss, float z_gauss);

/*
 * Update live speed over ground (m/s) for the centripetal correction.
 * valid = false disables the correction (GPS fix lost/expired).
 * Same lockless single-word pattern as imu_ctx_set_declination().
 */
void imu_ctx_set_speed(imu_ctx_t *ctx, float speed_mps, bool valid);

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

/* Black-box capture writer ([capture] enabled) — drains the tap ring into
 * rotating .imucap files.  Start only when cfg.capture_enabled. */
void *capture_thread(void *arg);

/*
 * Snapshot of the capture writer for status reporting.  active=false means
 * capture is disabled or the writer stopped (e.g. disk error); path may be
 * "" before the first file opens.
 */
void imu_get_capture_status(imu_ctx_t *ctx, char *path, size_t path_sz,
                            uint64_t *bytes, uint64_t *drops, bool *active);

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
