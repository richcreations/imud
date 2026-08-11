/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * chip_ts.h — keep a back-calculated FIFO burst's timestamps monotonic.
 *
 * THE PROBLEM.  Three drivers here (ism330dhcx, lsm6dso, icm42688p) time a
 * burst by reading the chip's live timestamp counter AFTER draining the FIFO,
 * calling that the newest sample's time, and stepping back one sample period
 * per older sample.  But that register reads *now*, not when the newest sample
 * was taken, and the gap between the two varies with bus timing and scheduler
 * jitter.  So a low-lag drain following a high-lag one can compute a first
 * sample that lands at or before the previous burst's last sample, and chip_ts
 * goes backwards across the seam.
 *
 * Measured on a Raspberry Pi 5 against the reference ISM330DHCX: 2-4 reversals
 * per 5 s window at 833 Hz, reproducible, with no counter wraps involved.
 * `imud-imutest` grades it FAIL under imu.chipts.monotonic, and rightly —
 * drivers.h makes chip_ts a monotonic sample clock.
 *
 * WHY IT IS NOT A FUSION BUG.  imu.c derives dt from consecutive timestamps
 * only when `wall > prev_wall_ns`, and then clamps to [0.5x, 2x] nominal, so a
 * reversal falls through to the nominal period and never reaches mekf_predict
 * as a non-positive dt.  The guard exists for FIFO-overflow gaps and anchor
 * resets and catches this too.  The defect is the contract, not the filter.
 *
 * THE REAL FIX, AND WHY IT IS NOT THIS.  These parts can batch timestamps into
 * the FIFO itself (ST: DEC_TS_BATCH + tag 0x04; ICM: bytes [14:15] of each
 * packet), which times samples where they are produced instead of inferring it
 * afterwards.  That is the right answer and it is on the ROADMAP.  It is not
 * done here because DS13012 never states whether the timestamp word precedes
 * or follows the sample set it describes, because batching changes FIFO word
 * traffic and therefore the watermark arithmetic — moving the very sample
 * latency figures the bench is about to measure — and because none of it can
 * be checked without the hardware.  Enforcing the contract is separable from
 * improving the estimate, so it goes first.
 *
 * Header-only and driver-private, like bus_io.h: adding a caller needs no
 * Makefile change, and each driver keeps its own state.
 */
#ifndef IMUD_DRIVERS_CHIP_TS_H
#define IMUD_DRIVERS_CHIP_TS_H

#include <stdbool.h>
#include <stdint.h>

/* Per-driver state.  Zero-initialised is "no burst seen yet". */
typedef struct {
    uint32_t last;      /* chip_ts of the newest sample of the previous burst */
    bool     have;      /* false until the first burst has been stamped */
} chip_ts_guard_t;

/*
 * Forget the previous burst.  Call from init(), so a reconfigure does not
 * drag a stale anchor across an ODR change.
 */
static inline void chip_ts_guard_reset(chip_ts_guard_t *g)
{
    g->last = 0;
    g->have = false;
}

/*
 * Given the timestamp this burst would assign its OLDEST sample, return the
 * offset to add to every sample in the burst so the run stays strictly
 * increasing.  Returns 0 when the burst already starts after the previous one,
 * which is the ordinary case.
 *
 * `step` is the per-sample tick count: the shifted burst starts exactly one
 * sample period after the previous burst ended, which is the smallest
 * correction that satisfies the contract and keeps the *within-burst* spacing
 * — which is real information from the chip — untouched.
 *
 * Comparison is on a SIGNED difference so a genuine 32-bit counter wrap reads
 * as a large forward jump rather than a huge backward one.
 *
 * A backward jump too large to be jitter is NOT shifted: the counter has been
 * reset or lost sync, and dragging every future timestamp up to meet a stale
 * anchor would corrupt the clock indefinitely to preserve one invariant.  The
 * guard re-seeds instead, and imu.c's anchor absorbs the discontinuity.
 * `max_jitter` bounds "plausibly jitter" — a second of ticks is generous next
 * to the millisecond-scale lag this corrects.
 */
static inline uint32_t chip_ts_guard_shift(chip_ts_guard_t *g,
                                           uint32_t first, uint32_t step,
                                           uint32_t max_jitter)
{
    if (!g->have) return 0;
    int32_t delta = (int32_t)(first - g->last);   /* <= 0 means overlap */
    if (delta > 0) return 0;
    if ((uint32_t)(-delta) > max_jitter) return 0;   /* reset, not jitter */
    return (uint32_t)(-delta) + step;
}

/* Record the newest sample of the burst just emitted. */
static inline void chip_ts_guard_note(chip_ts_guard_t *g, uint32_t newest)
{
    g->last = newest;
    g->have = true;
}

#endif /* IMUD_DRIVERS_CHIP_TS_H */
