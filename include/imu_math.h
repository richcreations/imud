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

#include <stddef.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

#include "types.h"
#include "cal.h"
#include "mhz.h"
#include "config.h"
#include "drivers.h"

/* ── Timestamp anchor ───────────────────────────────────────────────────── */

/*
 * The chip counter's declared period is a nominal figure, and the oscillator
 * driving it is not that accurate: a measured ISM330DHCX ran 4.1% fast, so
 * 40000 ticks took 0.96 s of wall clock rather than 1.00 s.  Multiplying ticks
 * by the declared ts_tick_ns therefore over-states elapsed time, and with a
 * 60 s anchor interval that compounds: sample timestamps run ~2.4 s ahead by
 * the end of an epoch and then jump backwards when the anchor is refreshed,
 * and every per-sample dt handed to the filter is 4% long, which scales
 * integrated rotation one-for-one.
 *
 * So the anchor measures the period instead of trusting it.  Two consecutive
 * anchors give a tick count and the monotonic time it took; the quotient is
 * the real period, and chip_to_wall() uses it in place of the nominal one.
 * CLOCK_MONOTONIC rather than the stored CLOCK_REALTIME because an NTP step
 * inside the window would otherwise be absorbed into the estimate.
 */
typedef struct {
    uint64_t        wall_ns;    /* CLOCK_REALTIME at anchor point, ns */
    uint64_t        tai_ns;     /* CLOCK_TAI at anchor point, ns */
    uint64_t        mono_ns;    /* CLOCK_MONOTONIC at anchor point, ns */
    uint32_t        chip_ticks; /* IMU hardware counter at anchor point */
    uint32_t        gen;        /* incremented on each update */
    /* Measured tick period in ns, 16-bit fraction.  0 until two anchors far
     * enough apart have been seen; chip_to_wall() falls back to the caller's
     * nominal value until then.  Fixed point, not float, to keep the timestamp
     * path integer-exact. */
    uint64_t        tick_q16;
    bool            have_mono;  /* a previous anchor exists to measure against */
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

/*
 * Re-anchor chip time to the host clocks, and refine the measured tick period
 * from the interval since the last anchor.  `mono_ns` must come from
 * CLOCK_MONOTONIC; `nominal_tick_ns` is imu_ops_t.ts_tick_ns, used both as the
 * fallback and as the plausibility centre for the measurement.  Pass 0 for
 * `nominal_tick_ns` on a part with no chip timer, which disables measurement.
 */
void anchor_update(ts_anchor_t *a, uint32_t chip_ts,
                   uint64_t wall_ns, uint64_t tai_ns,
                   uint64_t mono_ns, uint32_t nominal_tick_ns);

/*
 * Which host instant to pair with the newest sample's chip_ts, given the
 * clock readings either side of the burst read.
 *
 * This is the anchor's whole accuracy in one decision, and it turns on what
 * the driver's chip_ts MEANS.
 *
 * `ts_is_sample_instant` — the part has a chip timer and the driver reports
 * when each sample was actually taken.  The newest sample was already sitting
 * in the FIFO when the read began, so `t_before_ns` is the right pairing: it
 * over-states that sample's age by somewhere in [0, one sample period), and
 * crucially that error does NOT grow with burst depth or bus speed.
 *
 * Otherwise — no chip timer, chip_ts is always 0, and the anchor IS the
 * per-sample timestamp, refreshed every burst.  Nothing is known about where
 * inside the read any individual sample was taken, so the midpoint is the
 * best estimate for the burst as a whole.
 *
 * WHY NOT THE MIDPOINT FOR BOTH.  It used to be, and it was right when a
 * driver's chip_ts for the newest sample came from reading the counter AFTER
 * the drain — that value is "now at t_after", and pairing it with the midpoint
 * split the difference.  Batching the FIFO's own timestamp (1.9.0) changed the
 * meaning underneath that choice: chip_ts became the true sample instant, which
 * is EARLIER than t_before, so pairing it with the midpoint placed every
 * reconstructed timestamp late by about half the read duration — 2-3 ms at
 * 833 Hz over I2C, more at deeper watermarks or slower buses.  That is a
 * constant offset, so it cancels out of dt and never reached the filter, but
 * ts_wall_ns is documented as the instant the sensor sampled, and it was not.
 * imu.chipts.wall could not catch it either: that check grades the chip/wall
 * RATIO, which a constant offset leaves untouched.
 */
uint64_t anchor_wall_ns(uint64_t t_before_ns, uint64_t t_after_ns,
                        bool ts_is_sample_instant);

/* The measured tick period in ns, or 0.0 while still using the nominal one.
 * For logging and for `imud-status`; not needed to convert a timestamp. */
double anchor_measured_tick_ns(ts_anchor_t *a);

/*
 * Convert a chip counter value to wall + TAI timestamps using 32-bit wrapping
 * arithmetic (see the definition for the wrap bounds).  `tick_ns` is the
 * driver's declared period, used only until the anchor has measured the real
 * one.
 */
void chip_to_wall(ts_anchor_t *a, uint32_t chip_ts, uint32_t tick_ns,
                  uint64_t *wall_out, uint64_t *tai_out, uint32_t *gen_out);

/* ── Sample latency ──────────────────────────────────────────────────────── */

/*
 * spec.md §14 budgets FIFO read jitter, fusion latency and end-to-end latency.
 * Nothing measured any of them, so all three were aspirations — and the
 * end-to-end row is the one that looks wrong: `fifo_wm = 64` at 833 Hz is 77 ms
 * of buffering on its own, against a 3 ms budget.
 *
 * This is the instrument.  Two terms, kept apart because they answer different
 * questions and only their sum was ever discussed:
 *
 *   FIFO residence   sample taken -> I2C read completed   (fifo_wm / odr)
 *   pipeline         read completed -> state published    (the daemon's own)
 *
 * A log-spaced histogram rather than a mean, because the budget is stated as a
 * p99 and a mean cannot answer it; and fixed-bucket rather than a reservoir
 * because this runs per sample on the hot path and must not allocate.
 *
 * Bucket k holds [2^k, 2^(k+1)) microseconds, so the 20 buckets span 1 µs to
 * ~1.05 s, with anything larger clamped into the top bucket.  Percentiles are
 * therefore bucket-resolution: lat_percentile returns the UPPER edge of the
 * bucket the percentile falls in, which is a conservative read — never a
 * flattering one.  For an exact worst case use max_ns, which is not bucketed.
 */
#define LAT_BUCKETS 20

typedef struct {
    uint32_t bucket[LAT_BUCKETS];
    uint64_t count;         /* samples in the current window */
    uint64_t max_ns;        /* exact window maximum, not bucketed */
    uint64_t max_ever_ns;   /* exact maximum since start; survives a reset */
} lat_hist_t;

/* Record one interval.  Saturates into the top bucket rather than wrapping. */
void lat_record(lat_hist_t *h, uint64_t ns);

/*
 * Upper edge, in ns, of the bucket containing the p-th percentile (p in
 * [0, 1]).  Returns 0 for an empty histogram.  Because it reports a bucket
 * edge, the true value is at most a factor of two below what is returned.
 */
uint64_t lat_percentile(const lat_hist_t *h, double p);

/* Clear the window (buckets, count, max_ns).  max_ever_ns is deliberately
 * kept: the worst excursion since start is what diagnoses a stall, and a
 * per-window reset would hide it from anyone not watching that window. */
void lat_reset_window(lat_hist_t *h);

/* What one histogram publishes when its window fills.  `valid` is false when
 * the window was not full on that call and the other three are untouched. */
typedef struct {
    uint64_t p50, p99, max_ever;
    bool     valid;
} lat_pub_t;

/*
 * One sample's worth of latency accounting: record both terms where each is
 * measurable, then publish and roll whichever window has reached `need`.
 *
 * The two histograms gate INDEPENDENTLY, and that is the whole point of this
 * function rather than two lines at the call site.  They do not fill at the
 * same rate: `pipe` records on essentially every sample, while `fifo` records
 * only while `wall_ns` still trails `read_done_ns`.  That condition really can
 * fail for a whole window — before ts_anchor_t has measured the chip's tick
 * period, a part a percent or two off nominal makes the extrapolated `wall_ns`
 * outrun the read stamp, and `fifo` simply stops filling until the next anchor.
 * Sharing one gate meant `pipe` — the term the daemon controls and can be held
 * to a budget — was withheld for as long as a DIFFERENT histogram stayed short.
 * On the 2026-08-11 bench that was every 40 s run in the matrix.
 *
 * Both differences are clamped rather than allowed to wrap: an anchor refresh
 * can momentarily put `wall_ns` past the read stamp, and an unsigned underflow
 * there would land in the top bucket and poison p99 for the window.
 *
 * Caller keeps the histograms; this does no locking and no allocation.
 */
void lat_step(lat_hist_t *fifo, lat_hist_t *pipe, uint64_t need,
              uint64_t wall_ns, uint64_t read_done_ns, uint64_t now_ns,
              lat_pub_t *out_fifo, lat_pub_t *out_pipe);

/* ── Utilities ───────────────────────────────────────────────────────────── */

/*
 * Nearest value in a 0-terminated ascending table to `requested`.
 *
 * This answers "which advertised rate did the operator probably mean", which
 * is what imutest reports as advice. It is NOT what the hardware does — use
 * odr_actual_imu() / odr_actual_mag() for anything that has to agree with the
 * rate actually programmed.
 */
int nearest_odr(const int supported[], int requested);

/*
 * Lowest value in a 0-terminated ascending table that is >= `requested`,
 * clamped to the last entry. This is the rounding every register-table
 * driver's odr_encode() chain performs.
 */
int snap_odr_up(const int supported[], int requested);

/*
 * The rate the driver will really program for `req_mhz`: its actual_odr_mhz
 * hook when it has one, else snap_odr_up() over its supported_odr_mhz table.
 * MILLI-HERTZ in and out, like the rest of the driver interface — see the
 * unit note at the top of drivers.h for why whole Hz was not enough.
 *
 * The single source of truth for the sample rate. imu.c passes the result to
 * both the driver and the filter, and imutest measures against it, so the
 * three cannot disagree.
 */
/*
 * How long to wait on a data-ready edge before reading anyway, in ms, for a
 * sensor running at `odr_mhz`.  This is the missed-interrupt recovery, and it
 * has to exist: a LATCHED data-ready asserts on conversion-complete and is
 * re-armed only by the acknowledge the read performs, so exactly one rising
 * edge is produced per acknowledge and a single missed edge leaves the line
 * high for ever with no way back.  Measured on an MMC5983MA: after one
 * acknowledge and then a silent bus, 0 further edges in 3 s at 20 Hz and 1 at
 * 100 Hz, at every rate on its ladder.
 *
 * HALF a sample period, so a missed edge costs less than one sample.  It has
 * to be a fraction rather than a multiple: on this part the acknowledge the
 * fallback performs is also what re-arms the line, so waiting longer produces
 * FEWER edges, not merely later ones.  Measured at 20 Hz over 3 s: a 20 ms
 * fallback yields 64 edges and a 95 ms one yields 36.
 *
 * A fixed timeout is wrong at both ends of a ladder spanning 1 Hz to 6664:
 * 20 ms is fifty reads per conversion at 1 Hz and twenty-four conversions of
 * latency at 1204.
 *
 * Bounded at 2 ms so the fastest rates cannot spin, and at 250 ms so a 1 Hz
 * part still notices a stalled line within a quarter second.
 */
long imu_int_fallback_ms(int odr_mhz, int depth, int grace_samples);

/*
 * How long the IMU reader waits on the FIFO watermark before draining anyway.
 *
 * A LEVEL watermark needs no recovery -- it stays asserted until the FIFO
 * drops below the threshold -- so unlike imu_int_fallback_ms() above this is
 * not about missed edges. It is the drain CADENCE, and it is what actually
 * bounds FIFO residence: the reader takes whichever of the watermark and this
 * timeout comes first, so `fifo_wm` has an effect only while wm/odr is under
 * it. ROADMAP §3.1 and spec.md §14 both quote that rule, and the four-cell
 * sweep behind it, against this number.
 *
 * Shared so imud-imutest paces its drains the way the daemon does rather than
 * on a timer of its own choosing.
 */
/*
 * Fallback used only where there is nothing to predict -- no interrupt line
 * configured, so no expected arrival to be late against.  Where a watermark
 * IS configured the wait comes from imu_wm_fallback_ms() instead.
 */
#define IMU_DRAIN_WAIT_MS 10



int odr_actual_imu(const imu_ops_t *ops, int req_mhz);
int odr_actual_mag(const mag_ops_t *ops, int req_mhz);

/* Apply mount rotation (board -> body) if configured. In-place on v. */
void apply_mount_rot_if_set(const imud_config_t *cfg, float v[3]);

#endif /* IMUD_IMU_MATH_H */
