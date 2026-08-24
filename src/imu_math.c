/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu_math.c — pure helpers factored out of imu.c (see imu_math.h).
 *
 * Behaviour is identical to the former static definitions in imu.c; the move
 * only makes them externally linkable so test/test_imu_math.c can exercise
 * them directly.
 */

#include <stdio.h>    /* snprintf */
#include <stdlib.h>   /* abs */

#include "imu_math.h"

/* ── Calibration helpers ─────────────────────────────────────────────────── */

/*
 * Apply accel cal from cal.json on top of the driver's chip-level scaling.
 * Gyro bias is NOT subtracted here — the MEKF subtracts it during predict.
 */
void apply_imu_cal(const imud_cal_t *cal, imu_sample_t *s)
{
    /* Gyro bias/temperature compensation (imud-cal fit-temp): remove the
     * temperature-tracking bias component here so the MEKF's random-walk
     * estimator only has to follow the residual. */
    if (cal->has_gyro_temp) {
        float dT = s->temp_c - cal->gyro_temp_ref_c;
        s->gyro[0] -= cal->gyro_temp_coeff[0] * dT;
        s->gyro[1] -= cal->gyro_temp_coeff[1] * dT;
        s->gyro[2] -= cal->gyro_temp_coeff[2] * dT;
    }

    if (!cal->has_accel) return;
    for (int i = 0; i < 3; i++)
        s->accel[i] = (s->accel[i] - cal->accel_offset[i]) * cal->accel_scale[i];
}

/* Apply hard/soft-iron correction: m_cal = soft_iron × (m_raw − hard_iron). */
void apply_mag_cal(const imud_cal_t *cal, mag_sample_t *s)
{
    if (!cal->has_mag) return;
    float tmp[3];
    for (int i = 0; i < 3; i++)
        tmp[i] = s->field[i] - cal->mag_hard_iron[i];
    for (int i = 0; i < 3; i++)
        s->field[i] = cal->mag_soft_iron[i][0] * tmp[0]
                    + cal->mag_soft_iron[i][1] * tmp[1]
                    + cal->mag_soft_iron[i][2] * tmp[2];
}

/* ── Timestamp helpers ───────────────────────────────────────────────────── */

uint64_t ts_ns(const struct timespec *t)
{
    return (uint64_t)t->tv_sec * 1000000000ULL + (uint64_t)t->tv_nsec;
}

/*
 * Guards on the measurement.  The interval has to be long enough that the
 * jitter in where inside the I2C read the anchor lands is negligible against
 * it: the reader anchors every 60 s, a burst read is tens of milliseconds, and
 * 20 s puts that jitter under 0.2% — well below the several percent of
 * oscillator error this exists to catch.
 */
#define ANCHOR_MIN_INTERVAL_NS  20000000000ULL   /* 20 s */
#define ANCHOR_MIN_TICKS        1000u

/* Reject anything further than this from the declared period.  A measurement
 * outside it is not a slow oscillator: it is a counter that was reset under
 * us, a wrap the driver lost, or a reader that stalled for minutes. */
#define ANCHOR_TOL_PCT          10

void anchor_update(ts_anchor_t *a, uint32_t chip_ts,
                   uint64_t wall_ns, uint64_t tai_ns,
                   uint64_t mono_ns, uint32_t nominal_tick_ns)
{
    pthread_mutex_lock(&a->mtx);

    /*
     * have_mono, because the first anchor has nothing to measure against and
     * a zeroed struct would otherwise be treated as an anchor at time zero —
     * the daemon's first anchor lands at system uptime against a counter that
     * has been running since power-on, and dividing one by the other would
     * produce a confident nonsense.  nominal_tick_ns, because the
     * plausibility band below is built from it and a zero band means nothing;
     * on a part with no chip timer the tick delta is also always zero, so this
     * is the clearer of two guards rather than the only one.
     */
    if (a->have_mono && nominal_tick_ns != 0 && mono_ns > a->mono_ns) {
        uint32_t dticks = chip_ts - a->chip_ticks;      /* 32-bit wrapping */
        uint64_t dmono  = mono_ns - a->mono_ns;

        if (dticks >= ANCHOR_MIN_TICKS && dmono >= ANCHOR_MIN_INTERVAL_NS) {
            /* ns per tick, 16-bit fraction.  dmono is at most hours, so the
             * shift cannot overflow: 1 h << 16 is about 2.4e17. */
            uint64_t meas = (dmono << 16) / dticks;
            uint64_t nom  = (uint64_t)nominal_tick_ns << 16;
            uint64_t lo   = nom * (100 - ANCHOR_TOL_PCT) / 100;
            uint64_t hi   = nom * (100 + ANCHOR_TOL_PCT) / 100;

            if (meas >= lo && meas <= hi) {
                if (a->tick_q16 == 0) {
                    /* Take the first good measurement whole rather than
                     * filtering up to it: the error it corrects is systematic
                     * and present from the first sample, so converging over
                     * several minutes would leave it in place for no reason. */
                    a->tick_q16 = meas;
                } else {
                    /* Then filter, so one noisy interval cannot swing dt. */
                    int64_t err = (int64_t)meas - (int64_t)a->tick_q16;
                    a->tick_q16 = (uint64_t)((int64_t)a->tick_q16 + err / 4);
                }
            }
        }
    }

    a->chip_ticks = chip_ts;
    a->wall_ns    = wall_ns;
    a->tai_ns     = tai_ns;
    a->mono_ns    = mono_ns;
    a->have_mono  = true;
    a->gen++;
    pthread_mutex_unlock(&a->mtx);
}

uint64_t anchor_wall_ns(uint64_t t_before_ns, uint64_t t_after_ns,
                        bool ts_is_sample_instant)
{
    /* See imu_math.h for why these are different, and what it cost when they
     * were not.  Averaging as (a>>1)+(b>>1) rather than (a+b)/2 keeps the
     * midpoint correct for realtime nanosecond values near the top of the
     * range; the low bit it drops is a nanosecond. */
    if (ts_is_sample_instant) return t_before_ns;
    return (t_before_ns >> 1) + (t_after_ns >> 1);
}

double anchor_measured_tick_ns(ts_anchor_t *a)
{
    pthread_mutex_lock(&a->mtx);
    double v = a->tick_q16 ? (double)a->tick_q16 / 65536.0 : 0.0;
    pthread_mutex_unlock(&a->mtx);
    return v;
}

/*
 * Convert a chip counter value to wall + TAI timestamps.
 *
 * The delta is taken as SIGNED, because samples on both sides of the anchor
 * are ordinary.  The reader anchors on the NEWEST sample of a burst and then
 * pushes the whole burst, so every other sample in that burst is older than
 * the anchor by up to a watermark's worth of ticks.  Reading the delta as
 * unsigned turned each of those into a ~2^32-tick jump FORWARD — about 29.8
 * hours at 25 µs/tick — and imu.c published it as ts_wall_ns.  The filter
 * never saw it (the dt clamp and the anchor-generation check both reject the
 * discontinuity, and the latency histogram drops a sample whose wall is after
 * its own read), so it reached the wire and nothing else.
 *
 * 32-bit wrapping arithmetic still handles the counter's own wrap; taking the
 * result signed halves the representable span to 2^31 ticks — ~14.9 h at the
 * ST parts' 25 µs/tick, ~35.8 min at the ICM-42688-P's 1.067 µs/tick, which
 * unwraps its 20-bit counter into 32.  The 60 s anchor refresh keeps deltas
 * three orders of magnitude below either bound.
 *
 * The magnitude is computed unsigned and the sign applied afterwards: a
 * right-shift of a negative signed value is implementation-defined, and the
 * q16 fixed-point path needs that shift.
 *
 * The period used is the measured one once the anchor has it, falling back to
 * the caller's `tick_ns` — imu_ops_t.ts_tick_ns — until then.  That fallback
 * also covers !has_hw_timestamp, where tick_ns is 0, chip_ts is always 0, and
 * the offset degenerates to 0 as intended.
 */
void chip_to_wall(ts_anchor_t *a, uint32_t chip_ts, uint32_t tick_ns,
                  uint64_t *wall_out, uint64_t *tai_out,
                  uint32_t *gen_out)
{
    pthread_mutex_lock(&a->mtx);
    int32_t  dticks = (int32_t)(chip_ts - a->chip_ticks);
    bool     before = dticks < 0;
    /* Widen before negating so INT32_MIN has somewhere to go. */
    uint64_t mag    = before ? (uint64_t)(-(int64_t)dticks) : (uint64_t)dticks;
    uint64_t offset = a->tick_q16
                    ? (mag * a->tick_q16) >> 16
                    : mag * tick_ns;
    if (wall_out) *wall_out = before ? a->wall_ns - offset : a->wall_ns + offset;
    if (tai_out)  *tai_out  = before ? a->tai_ns  - offset : a->tai_ns  + offset;
    if (gen_out)  *gen_out  = a->gen;
    pthread_mutex_unlock(&a->mtx);
}

/* ── Utilities ───────────────────────────────────────────────────────────── */

/* ── Sample latency ──────────────────────────────────────────────────────── */

/*
 * Bucket index for an interval: floor(log2(microseconds)), clamped.
 *
 * Sub-microsecond lands in bucket 0 rather than being dropped — a zero-length
 * interval is a real measurement (both stamps inside the same clock tick), and
 * silently discarding it would bias the count that percentiles divide by.
 *
 * Written as a shift loop rather than __builtin_clzll: the builtin is undefined
 * for zero, which is exactly the input that arrives when the two stamps land in
 * the same tick, and guarding it costs more than the loop saves at this rate.
 */
static int lat_bucket(uint64_t ns)
{
    uint64_t us = ns / 1000u;
    int k = 0;
    while (us > 1u && k < LAT_BUCKETS - 1) { us >>= 1; k++; }
    return k;
}

void lat_record(lat_hist_t *h, uint64_t ns)
{
    h->bucket[lat_bucket(ns)]++;
    h->count++;
    if (ns > h->max_ns)      h->max_ns      = ns;
    if (ns > h->max_ever_ns) h->max_ever_ns = ns;
}

uint64_t lat_percentile(const lat_hist_t *h, double p)
{
    if (h->count == 0) return 0;
    if (p < 0.0) p = 0.0;
    if (p > 1.0) p = 1.0;

    /*
     * Rank via ceil so p=1.0 selects the last sample rather than one past it,
     * and any p>0 selects at least the first.
     */
    uint64_t want = (uint64_t)((double)h->count * p + 0.999999);
    if (want == 0) want = 1;

    uint64_t seen = 0;
    for (int k = 0; k < LAT_BUCKETS; k++) {
        seen += h->bucket[k];
        if (seen >= want) {
            /* Upper edge of bucket k = 2^(k+1) microseconds, in ns. */
            return (uint64_t)1000u << (k + 1);
        }
    }
    return (uint64_t)1000u << LAT_BUCKETS;   /* unreachable while count > 0 */
}

void lat_reset_window(lat_hist_t *h)
{
    for (int k = 0; k < LAT_BUCKETS; k++) h->bucket[k] = 0;
    h->count  = 0;
    h->max_ns = 0;
    /* max_ever_ns intentionally survives — see imu_math.h. */
}

/* Publish and roll one window if it is full; otherwise say nothing happened. */
static void lat_pub_if_ready(lat_hist_t *h, uint64_t need, lat_pub_t *out)
{
    out->valid = (need > 0 && h->count >= need);
    if (!out->valid) return;

    out->p50      = lat_percentile(h, 0.50);
    out->p99      = lat_percentile(h, 0.99);
    out->max_ever = h->max_ever_ns;
    lat_reset_window(h);
}

void lat_step(lat_hist_t *fifo, lat_hist_t *pipe, uint64_t need,
              uint64_t wall_ns, uint64_t read_done_ns, uint64_t now_ns,
              lat_pub_t *out_fifo, lat_pub_t *out_pipe)
{
    if (read_done_ns > wall_ns)      lat_record(fifo, read_done_ns - wall_ns);
    if (now_ns       > read_done_ns) lat_record(pipe, now_ns - read_done_ns);

    /* Two calls, never one condition covering both — see imu_math.h. */
    lat_pub_if_ready(fifo, need, out_fifo);
    lat_pub_if_ready(pipe, need, out_pipe);
}

/* ── Utilities ───────────────────────────────────────────────────────────── */

int nearest_odr(const int supported[], int requested)
{
    int best = supported[0], best_diff = abs(supported[0] - requested);
    for (int i = 1; supported[i] != 0; i++) {
        int d = abs(supported[i] - requested);
        if (d < best_diff) { best = supported[i]; best_diff = d; }
    }
    return best;
}

int snap_odr_up(const int supported[], int requested)
{
    int last = supported[0];
    for (int i = 0; supported[i] != 0; i++) {
        if (supported[i] >= requested) return supported[i];
        last = supported[i];
    }
    return last;   /* above the top of the table — clamp to the highest */
}

/*
 * When should an interrupt-driven reader give up waiting and read anyway?
 *
 * Only when the interrupt is LATE -- never merely because it has not happened
 * yet. So the wait is the expected arrival plus a grace, and BOTH are measured
 * in samples, because a grace in milliseconds means something different at
 * every rate: 2 ms is thirteen samples at 6664 Hz and three hundredths of one
 * at 13 Hz.
 *
 *     wait = (depth + grace) sample periods
 *
 * `depth` is what the line is waiting FOR: the IMU's fifo_wm sample-sets for a
 * FIFO watermark, or 1 for a per-sample data-ready. These parts' magnetometers
 * have no FIFO, so depth is 1 there.
 *
 * This matters more than a tidier constant. A LEVEL watermark asserts once the
 * FIFO holds fifo_wm sets and deasserts as soon as a drain empties it, so a
 * fallback shorter than the watermark period drains first, holds the FIFO
 * permanently below the threshold, and the line never asserts AT ALL. The
 * interrupt is not merely unused -- it is suppressed by the thing meant to
 * back it up.
 *
 * Measured: with the old flat 10 ms and the default wm = 64, the ISM330DHCX's
 * watermark was reachable at exactly ONE of its ten rates (6664 Hz, where it
 * lands at 9.6 ms). At the other nine the timer always won, and the daemon
 * reported drains=0/1261 e/t on a line that was working perfectly. The
 * magnetometer had the mirror-image bug: a half-sample-period fallback that
 * expired BEFORE its own data-ready could arrive.
 *
 * The cost is that a genuinely dead line now stalls for depth + grace samples,
 * and `depth` is whatever batching the operator asked for. That is inherent --
 * configuring a 5-second batch asks for 5-second latency -- and noticing a
 * dead line is the health path's job, not the drain timer's.
 */
long imu_int_fallback_ms(int odr_mhz, int depth, int grace_samples)
{
    if (depth < 1)         depth = 1;
    if (grace_samples < 1) grace_samples = 1;
    if (odr_mhz <= 0)      return 20;        /* unknown rate: the old constant */

    /* odr is milli-Hz, so one sample period in ms is 1e6 / odr_mhz. */
    double ms = 1000000.0 * (double)(depth + grace_samples) / (double)odr_mhz;
    if (ms > 600000.0) ms = 600000.0;        /* guard the arithmetic, 10 min */
    long out = (long)(ms + 0.5);
    return out < 1 ? 1 : out;
}

int odr_actual_imu(const imu_ops_t *ops, int req_mhz)
{
    if (ops->actual_odr_mhz) return ops->actual_odr_mhz(req_mhz);
    return snap_odr_up(ops->supported_odr_mhz, req_mhz);
}

int odr_actual_mag(const mag_ops_t *ops, int req_mhz)
{
    if (ops->actual_odr_mhz) return ops->actual_odr_mhz(req_mhz);
    return snap_odr_up(ops->supported_odr_mhz, req_mhz);
}

/* Apply mount rotation (board -> body) if configured. In-place on v. */
void apply_mount_rot_if_set(const imud_config_t *cfg, float v[3])
{
    if (!cfg->mount_set) return;
    double out0 = cfg->mount_rot[0][0] * v[0]
                + cfg->mount_rot[0][1] * v[1]
                + cfg->mount_rot[0][2] * v[2];
    double out1 = cfg->mount_rot[1][0] * v[0]
                + cfg->mount_rot[1][1] * v[1]
                + cfg->mount_rot[1][2] * v[2];
    double out2 = cfg->mount_rot[2][0] * v[0]
                + cfg->mount_rot[2][1] * v[1]
                + cfg->mount_rot[2][2] * v[2];
    v[0] = (float)out0; v[1] = (float)out1; v[2] = (float)out2;
}
