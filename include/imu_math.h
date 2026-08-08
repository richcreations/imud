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
 * The rate the driver will really program for `requested`: its actual_odr_hz
 * hook when it has one, else snap_odr_up() over its supported_odr_hz table.
 *
 * The single source of truth for the sample rate. imu.c passes the result to
 * both the driver and the filter, and imutest measures against it, so the
 * three cannot disagree.
 */
int odr_actual_imu(const imu_ops_t *ops, int requested);
int odr_actual_mag(const mag_ops_t *ops, int requested);

/* Apply mount rotation (board -> body) if configured. In-place on v. */
void apply_mount_rot_if_set(const imud_config_t *cfg, float v[3]);

#endif /* IMUD_IMU_MATH_H */
