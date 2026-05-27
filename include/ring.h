
/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ring.h — lock-based circular ring buffers for imu_sample_t and mag_sample_t
 *
 * IMU ring (256 slots): ism_reader → fusion.  Blocking pop; oldest dropped
 *                       on overflow so the reader is never stalled.
 * Mag ring  (32 slots): mag_reader → fusion.  Non-blocking try-pop; same
 *                       overflow policy.
 */

#ifndef IMUD_RING_H
#define IMUD_RING_H

#include "types.h"

/* ── Init ────────────────────────────────────────────────────────────────── */

void imu_ring_init(imu_ring_t *r);
void mag_ring_init(mag_ring_t *r);

/* ── IMU ring ────────────────────────────────────────────────────────────── */

/*
 * Push n samples.  If the ring is full, oldest samples are silently dropped
 * (continuous mode — reader is never blocked).
 * Returns the number of samples dropped due to overflow.
 */
int imu_ring_push(imu_ring_t *r, const imu_sample_t *s, int n);

/*
 * Block until a sample is available or 100 ms elapses.
 * Wakes immediately when *stop becomes non-zero (set stop before broadcasting
 * on the cond var to unblock cleanly).
 * Returns 0 on success, -1 on timeout or stop.
 */
int imu_ring_pop(imu_ring_t *r, imu_sample_t *out, volatile int *stop);

/* ── Mag ring ────────────────────────────────────────────────────────────── */

/* Push one sample; drops oldest if full. */
void mag_ring_push(mag_ring_t *r, const mag_sample_t *s);

/* Non-blocking.  Returns 0 on success, -1 if empty. */
int mag_ring_try_pop(mag_ring_t *r, mag_sample_t *out);

#endif /* IMUD_RING_H */
