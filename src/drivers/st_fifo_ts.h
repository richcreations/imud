/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * st_fifo_ts.h — anchor an ST 6-axis FIFO burst on the chip's own timestamp.
 *
 * THE PROBLEM.  ism330dhcx.c and lsm6dso.c time a burst by reading TIMESTAMP0
 * AFTER draining the FIFO, calling that the newest sample's time, and stepping
 * back one sample period per older sample.  But that register reads *now*, and
 * the gap between "now" and when the newest sample was actually taken moves
 * with bus timing and scheduler jitter.  So a low-lag drain following a
 * high-lag one computes a first sample at or before the previous burst's last
 * sample, and chip_ts goes backwards across the seam — measured on a Pi 5 at
 * 2-4 reversals per 5 s.  chip_ts.h enforces the contract afterwards; this
 * fixes the estimate at source.
 *
 * THE FIX.  The part can put its timestamp counter INTO the FIFO, as tag 0x04
 * words interleaved with the samples (FIFO_CTRL4's DEC_TS_BATCH, DS13012
 * Table 31).  A word that arrived in the stream was written when the samples
 * around it were written, so it dates the burst with no post-drain read and no
 * lag to vary.  Every sample is then placed relative to that one known point.
 *
 * WHY DECIMATED, NOT PER-SAMPLE.  DEC_TS_BATCH can write a timestamp for every
 * sample-set (01), but that is a third word alongside the accel and gyro pair:
 * +50% FIFO word traffic, so a 128-word watermark holds 42 sample-sets instead
 * of 64.  fifo_wm would quietly stop meaning what docs/manual.md says it means,
 * and the sample-latency figures in spec.md §14 would move under the
 * measurement.  At 32 (11) it is one extra word per 32 sets —
 * +1.6%, so the same watermark holds 63 sets rather than 64 — and one anchor
 * inside a burst is all the arithmetic needs.  Anchoring is the goal; a stamp
 * per sample is not.
 *
 * WHAT IS NOT IN THE DATASHEET, AND HOW THAT IS HANDLED.  DS13012 documents
 * the FIFO output registers only as generic X/Y/Z 16-bit words (section 9.61),
 * and never says how a tag-0x04 payload is laid out inside them, nor whether
 * the word precedes or follows the samples it describes.  So:
 *
 *   - The 32-bit little-endian-in-X/Y layout below is the convention ST's own
 *     drivers and the Linux st_lsm6dsx driver use.  It is an assumption, and
 *     st_fifo_ts_apply() checks it against the post-drain register read rather
 *     than trusting it: a burst that disagrees is refused whole and falls back
 *     to the old path, so a wrong guess degrades instead of corrupting time.
 *   - Ordering does not have to be answered.  The tag byte carries TAG_CNT, "a
 *     2-bit counter which identifies sensor time slot" (Table 158), so a
 *     timestamp word can be matched to its own slot rather than to a position.
 *
 *     MEASURED, 2026-08-25, ism330dhcx over SPI at 833 Hz: TAG_CNT does NOT
 *     participate for tag 0x04.  Across 2000 timestamp words it read 0 every
 *     single time — cnt0=2000, cnt1=cnt2=cnt3=0 — and never once equalled the
 *     TAG_CNT of the set just completed.  So the comparison below always takes
 *     its else branch and the word dates the set still being assembled.
 *
 *     That is the degradation this was written to tolerate, and the bound
 *     stands: off by one sample, 1.2 ms at 833 Hz, constant across the burst,
 *     so it shifts sample age slightly and leaves every dt untouched.  The
 *     comparison is kept rather than deleted because it costs nothing and is
 *     correct for any part whose timestamp words do carry a slot counter; it is
 *     simply inert on this one.
 *
 * Header-only and driver-private, like bus_io.h and chip_ts.h.
 */
#ifndef IMUD_DRIVERS_ST_FIFO_TS_H
#define IMUD_DRIVERS_ST_FIFO_TS_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers.h"

/* FIFO tag for a batched timestamp word (DS13012 Table 159). */
#define ST_TAG_TIMESTAMP  0x04

/* Tag byte: TAG_SENSOR[4:0] | TAG_CNT[1:0] | TAG_PARITY (Table 157). */
#define ST_TAG_SENSOR(b)  (((b) >> 3) & 0x1F)
#define ST_TAG_CNT(b)     (((b) >> 1) & 0x03)

/*
 * The newest sample must not post-date the register read that follows the
 * drain, and must not predate it implausibly.  One second of 25 µs ticks is
 * far longer than any real drain (the shipped 64-set watermark at 833 Hz is
 * ~25 ms over I²C) and far shorter than the garbage a misread payload
 * produces, which is the discrimination this needs to make.
 */
#define ST_TS_MAX_LAG_TICKS  40000u

/* Per-drain scratch.  st_fifo_ts_begin() at the top of every read(). */
typedef struct {
    bool     have_word;   /* a tag-0x04 word was seen this drain */
    int      idx;         /* sample-set index it dates */
    uint32_t ts;          /* its 32-bit counter value */
    bool     have_set;    /* at least one sample-set has been completed */
    uint8_t  set_cnt;     /* TAG_CNT of the most recently completed set */
} st_fifo_ts_t;

static inline void st_fifo_ts_begin(st_fifo_ts_t *s)
{
    s->have_word = false;
    s->have_set  = false;
    s->idx       = 0;
    s->ts        = 0;
    s->set_cnt   = 0;
}

/*
 * Pick DEC_TS_BATCH for a watermark, as the raw 2-bit field for FIFO_CTRL4.
 *
 * The decimation has to be fine enough that a watermark-depth drain usually
 * contains a word — a drain with none simply falls back, which is correct but
 * buys nothing.  Below 8 sample-sets nothing is batched at all: those
 * watermarks are chosen for latency, and the only decimation that would land a
 * word in them is the +50% one this deliberately avoids.
 */
static inline uint8_t st_fifo_ts_dec_batch(int fifo_wm)
{
    if (fifo_wm >= 32) return 0x3;   /* every 32 sample-sets, +1.6% words */
    if (fifo_wm >=  8) return 0x2;   /* every 8,                +6.3% words */
    return 0x0;                      /* not batched */
}

/* Call when a sample-set is completed, with the tag byte of either of its
 * words — accel and gyro from one slot carry the same TAG_CNT. */
static inline void st_fifo_ts_note_set(st_fifo_ts_t *s, uint8_t tag_byte)
{
    s->set_cnt  = ST_TAG_CNT(tag_byte);
    s->have_set = true;
}

/*
 * Call on a tag-0x04 word, with the 7-byte FIFO word and the number of
 * sample-sets completed so far.
 *
 * The word dates the slot its TAG_CNT names: the set just completed if the
 * counters match, otherwise the set still being assembled.  That is what makes
 * the undocumented ordering irrelevant — position in the stream is never used.
 * Later words overwrite earlier ones, so the anchor is always the newest in the
 * burst and the extrapolation either side of it is as short as it can be.
 */
static inline void st_fifo_ts_note_word(st_fifo_ts_t *s, const uint8_t *word,
                                        int produced)
{
    int idx = (s->have_set && ST_TAG_CNT(word[0]) == s->set_cnt)
            ? produced - 1     /* the set that just completed */
            : produced;        /* the next one to complete */
    if (idx < 0) return;

    s->ts        = (uint32_t)word[1]
                 | ((uint32_t)word[2] <<  8)
                 | ((uint32_t)word[3] << 16)
                 | ((uint32_t)word[4] << 24);
    s->idx       = idx;
    s->have_word = true;
}

/*
 * Timestamp the whole burst from the anchor.  Returns false if there was no
 * usable word or the result did not survive its checks, in which case buf is
 * untouched and the caller must fall back to the post-drain read.
 *
 * `now_ts` is TIMESTAMP0 read after the drain — used only to sanity-check, not
 * to place anything.  All arithmetic is 32-bit modular, which is what makes it
 * correct across the counter's wrap.
 */
static inline bool st_fifo_ts_apply(const st_fifo_ts_t *s, imu_sample_t *buf,
                                    int produced, uint32_t ticks_per_sample,
                                    uint32_t now_ts)
{
    if (!s->have_word || produced <= 0) return false;
    /* The word may name a set that never completed — truncated at `max`, or
     * left half-assembled at the end of the FIFO. */
    if (s->idx >= produced) return false;

    uint32_t newest = s->ts + (uint32_t)(produced - 1 - s->idx) * ticks_per_sample;

    /* The newest sample was taken before the register was read, and not long
     * before.  A misread payload lands outside this by a wide margin. */
    uint32_t lag = now_ts - newest;              /* modular; wrap-correct */
    if ((int32_t)lag < 0 || lag > ST_TS_MAX_LAG_TICKS) return false;

    for (int i = 0; i < produced; i++)
        buf[i].chip_ts = s->ts + (uint32_t)(i - s->idx) * ticks_per_sample;

    return true;
}

#endif /* IMUD_DRIVERS_ST_FIFO_TS_H */
