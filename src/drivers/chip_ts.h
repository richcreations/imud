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
    /*
     * Consecutive refusals.  A guard that refuses read after read is not
     * protecting the clock from the part; it is holding an anchor the part has
     * moved away from, and every further refusal drags the error along.
     * chip_ts_guard_refused() counts, chip_ts_guard_accepted() clears, and
     * past CHIP_TS_MAX_REFUSALS the caller re-seeds instead.
     */
    unsigned refusals;
} chip_ts_guard_t;

/*
 * How many consecutive refusals before the guard is the thing that is wrong.
 *
 * A genuine bad counter read is isolated: the next one is fine.  A stale anchor
 * refuses forever, because every correct reading looks equally implausible
 * against it.  Measured on the reference ISM330DHCX: one read landed 65,706
 * ticks (1.58 s) ahead, was accepted because the forward bound was a flat
 * 9.6 s, and the eleven correct reads after it were then refused one after
 * another while the stamps walked further from real time.  Three is enough to
 * tell one bad read from a guard that has lost the plot.
 */
#define CHIP_TS_MAX_REFUSALS 3u

/*
 * Forget the previous burst.  Call from init(), so a reconfigure does not
 * drag a stale anchor across an ODR change.
 */
static inline void chip_ts_guard_reset(chip_ts_guard_t *g)
{
    g->last     = 0;
    g->have     = false;
    g->refusals = 0;
}

/*
 * The caller refused a reading.  Returns true when the guard has now refused
 * too many in a row to be believed itself, in which case the caller must
 * re-seed from the reading rather than extrapolate again.
 */
static inline bool chip_ts_guard_refused(chip_ts_guard_t *g)
{
    return ++g->refusals > CHIP_TS_MAX_REFUSALS;
}

/* The caller used a reading: the guard is tracking the part again. */
static inline void chip_ts_guard_accepted(chip_ts_guard_t *g)
{
    g->refusals = 0;
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
 * A backward jump too large to be jitter is NOT shifted here, and returning 0
 * says only "not my correction to make" — it does not mean the stamp is fit to
 * use.  Callers must ask chip_ts_guard_backward_ok() BEFORE this and refuse the
 * read outright when it says no; letting a large backward jump through on the
 * theory that the counter had been reset is what emitted nine samples with
 * timestamps near 2^32 on the bench.  `max_jitter` bounds "plausibly jitter" —
 * a second of ticks is generous next to the millisecond-scale lag this
 * corrects.
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

/*
 * Is `first` credible as the next burst's OLDEST stamp, or is the counter read
 * behind it garbage?
 *
 * The backward guard above only corrects overlaps. A post-drain counter read
 * that comes back wrong in the FORWARD direction is not an overlap, so nothing
 * caught it — and the fallback path treats that read as the newest sample's
 * time, so one bad read stamps an entire burst far in the future. Measured on
 * the reference ISM330DHCX: one burst of 9 samples in 94,539 landed 2,163,509
 * ticks (54 s) ahead, with `seq` continuous across it. The burst after was
 * correct, because the backward jump back to real time exceeded max_jitter and
 * re-seeded — so the guard recovered, one burst too late, having emitted nine
 * samples with wire timestamps 54 s wrong.
 *
 * The irony worth remembering: st_fifo_ts.h ALREADY rejects the batched anchor
 * when `now_ts` fails its cross-check, and the fallback then uses that very
 * `now_ts` as ground truth. This is the check the fallback was missing.
 *
 * `max_forward` bounds a believable gap between consecutive bursts, and the
 * bound is TIGHT rather than generous, which is the opposite of what it used to
 * be.  The physical fact is that this burst's OLDEST sample follows the previous
 * burst's NEWEST by one sample period: the FIFO loses nothing in between, so
 * however long the reader was away, the samples it missed are still queued and
 * come back in order.  Starvation makes bursts BIGGER, not later.  The only
 * thing that breaks the chain is an overflow, and an overflow is reported --
 * callers reset the guard on it rather than stretching this bound to cover it.
 *
 * So callers pass a small multiple of ticks_per_sample.  The flat 9.6 s this
 * replaced was chosen on the theory that "a real gap of seconds means the reader
 * was starved for seconds", which is exactly the case the FIFO already handles.
 * What the slack actually admitted was a bad read: on the reference ISM330DHCX
 * at 104 Hz one landed 65,706 ticks (1.58 s) ahead of 384 expected, sailed under
 * the bound, and poisoned the anchor for the eleven bursts that followed.
 */
static inline bool chip_ts_guard_forward_ok(const chip_ts_guard_t *g,
                                            uint32_t first,
                                            uint32_t max_forward)
{
    if (!g->have) return true;             /* nothing to judge it against */
    int32_t delta = (int32_t)(first - g->last);
    if (delta <= 0) return true;           /* backward: the other guard's job */
    return (uint32_t)delta <= max_forward;
}

/*
 * Is `first` credible as the next burst's OLDEST stamp in the BACKWARD
 * direction, or is the counter read behind it garbage?
 *
 * chip_ts_guard_shift() above deliberately does NOT correct a backward jump
 * bigger than jitter, on the reading that the counter must have been reset and
 * that dragging every later timestamp up to meet a stale anchor would corrupt
 * the clock indefinitely.  The premise is what fails: within a run the counter
 * cannot legitimately reset.  It resets on SW_RESET and on power-up, both of
 * which reach init(), and init() calls chip_ts_guard_reset() — so a guard that
 * still has history has not seen a reset, and a large backward jump is a bad
 * read.  A genuine 32-bit wrap is not one either: the signed difference makes a
 * wrap read as a small FORWARD step.
 *
 * Measured on the reference ISM330DHCX at 52 Hz, 2026-08-20: one burst of nine
 * samples in 6,494 was stamped from a counter read of about zero, so the burst
 * stepped back below zero and went out as chip_ts near 2^32 — 0.5 s behind,
 * with `seq` continuous across it.  The guard then latched that value and
 * extrapolated at exactly ticks_per_sample until the next read was far enough
 * ahead to be believed, nine samples later.  Same nine-sample signature as the
 * forward case above, and the same cost: wire timestamps that go backwards.
 *
 * `max_backward` is the same bound that separates jitter from nonsense, so a
 * real overlap still gets shifted and only nonsense is refused.
 */
static inline bool chip_ts_guard_backward_ok(const chip_ts_guard_t *g,
                                             uint32_t first,
                                             uint32_t max_backward)
{
    if (!g->have) return true;             /* nothing to judge it against */
    int32_t delta = (int32_t)(first - g->last);
    if (delta >= 0) return true;           /* forward: the other guard's job */
    return (uint32_t)(-delta) <= max_backward;
}

/*
 * The stamp the next sample should carry when the counter read cannot be
 * trusted: one sample period after the previous burst ended. Only meaningful
 * once the guard has seen a burst, which chip_ts_guard_forward_ok() implies
 * whenever it returns false.
 */
static inline uint32_t chip_ts_guard_next(const chip_ts_guard_t *g, uint32_t step)
{
    return g->last + step;
}

/* Record the newest sample of the burst just emitted. */
static inline void chip_ts_guard_note(chip_ts_guard_t *g, uint32_t newest)
{
    g->last = newest;
    g->have = true;
}

#endif /* IMUD_DRIVERS_CHIP_TS_H */
