/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_imu_math.c — unit tests for the pure helpers factored out of imu.c
 * into src/imu_math.c: nearest_odr, the chip-timer → wall-clock reconstruction
 * (anchor_update / chip_to_wall, including 32-bit wraparound), the mount
 * rotation, and the calibration application.
 *
 * These transforms carry the daemon's timestamp and calibration correctness
 * but are otherwise compiled only into the TSan concurrency test, which
 * never asserts their numeric output.  Pure and portable — builds and runs on
 * the macOS dev box too.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>

#include "imu_math.h"

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── nearest_odr ─────────────────────────────────────────────────────────── */

/*
 * The missed-interrupt recovery interval.
 *
 * A latched data-ready yields exactly one rising edge per acknowledge, so one
 * missed edge leaves the line asserted with nothing to clear it -- seen on
 * an MMC5983MA as 0 further edges in 3 s after a single acknowledge. The
 * fallback read is the only way back, which makes this interval load-bearing
 * rather than a safety net, and a flat constant wrong at both ends of a ladder
 * spanning 1 Hz to 6664.
 */
/*
 * A magnetometer that stops delivering otherwise produces no log line --
 * read() returning "no new measurement" is normal, so the reader looped on it
 * for ever.  Measured: 800 s with zero mag samples and one line in the log,
 * from the fusion aligner at t+5 s.  This is the policy behind the warning
 * that now fires; the warning itself is a log line, so the threshold is what
 * gets pinned.
 */
/*
 * Calibration must be applied in the frame it was measured in.
 *
 * imud-cal reads the driver directly and never applies the mount rotation, so
 * every calibration it writes is in SENSOR axes.  Rotating
 * board->body FIRST and calibrate second, which subtracts a sensor-frame offset
 * from body-frame data.  With [mount] identity the two frames coincide and
 * nothing is visibly wrong, which is why it survived: the defect only appears
 * on a non-identity mount, and mounting a sensor on edge is exactly when you
 * need one.
 *
 * Rotating the calibration instead is not representable — accel_scale[] is a
 * diagonal and R*diag*R^T is a full matrix — so the fix is the order.
 *
 * Uses a 90 degree rotation about X, R = [[1,0,0],[0,0,-1],[0,1,0]], and unit
 * scales so the arithmetic is checkable by hand:
 *
 *   correct   R * (m - h)   =  [9, -27, 18]
 *   inverted  (R * m) - h   =  [9, -32, 17]
 */
static void test_cal_is_applied_before_mount_rotation(void)
{
    begin("test_cal_is_applied_before_mount_rotation");
    int fb = g_fail;

    imud_config_t cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.mount_set = true;
    cfg.mount_rot[0][0] = 1; cfg.mount_rot[0][1] =  0; cfg.mount_rot[0][2] = 0;
    cfg.mount_rot[1][0] = 0; cfg.mount_rot[1][1] =  0; cfg.mount_rot[1][2] = -1;
    cfg.mount_rot[2][0] = 0; cfg.mount_rot[2][1] =  1; cfg.mount_rot[2][2] = 0;

    imud_cal_t cal;
    memset(&cal, 0, sizeof cal);
    cal.has_accel = true;
    for (int k = 0; k < 3; k++) {
        cal.accel_offset[k] = (float)(k + 1);   /* 1, 2, 3 */
        cal.accel_scale[k]  = 1.0f;
    }
    cal.has_mag = true;
    for (int k = 0; k < 3; k++) {
        cal.mag_hard_iron[k] = (float)(k + 1);
        for (int j = 0; j < 3; j++) cal.mag_soft_iron[k][j] = (k == j) ? 1.0f : 0.0f;
    }

    imu_sample_t s;
    memset(&s, 0, sizeof s);
    s.accel[0] = 10; s.accel[1] = 20; s.accel[2] = 30;
    imu_finalise_sample(&cfg, &cal, &s);
    EXPECT_NEAR(s.accel[0],   9.0f, 1e-4, "accel X: calibrated then rotated");
    EXPECT_NEAR(s.accel[1], -27.0f, 1e-4, "accel Y: calibrated then rotated");
    EXPECT_NEAR(s.accel[2],  18.0f, 1e-4, "accel Z: calibrated then rotated");
    EXPECT(fabsf(s.accel[1] - (-32.0f)) > 1e-3,
           "not the inverted order, which would give -32 on Y");

    /* accel_raw keeps the meaning the wire gives it: pre-calibration, AFTER
     * the mount rotation.  R * [10,20,30] = [10,-30,20]. */
    EXPECT_NEAR(s.accel_raw[0],  10.0f, 1e-4, "raw X is uncalibrated, rotated");
    EXPECT_NEAR(s.accel_raw[1], -30.0f, 1e-4, "raw Y is uncalibrated, rotated");
    EXPECT_NEAR(s.accel_raw[2],  20.0f, 1e-4, "raw Z is uncalibrated, rotated");

    mag_sample_t m;
    memset(&m, 0, sizeof m);
    m.field[0] = 10; m.field[1] = 20; m.field[2] = 30;
    mag_finalise_sample(&cfg, &cal, &m);
    EXPECT_NEAR(m.field[0],   9.0f, 1e-4, "mag X: calibrated then rotated");
    EXPECT_NEAR(m.field[1], -27.0f, 1e-4, "mag Y: calibrated then rotated");
    EXPECT_NEAR(m.field[2],  18.0f, 1e-4, "mag Z: calibrated then rotated");
    EXPECT_NEAR(m.field_raw[1], -30.0f, 1e-4, "mag raw is uncalibrated, rotated");

    /* Identity mount: the frames coincide, so order cannot matter — this is
     * the configuration under which the defect was invisible. */
    imud_config_t idc;
    memset(&idc, 0, sizeof idc);
    idc.mount_set = false;
    imu_sample_t t;
    memset(&t, 0, sizeof t);
    t.accel[0] = 10; t.accel[1] = 20; t.accel[2] = 30;
    imu_finalise_sample(&idc, &cal, &t);
    EXPECT_NEAR(t.accel[0],  9.0f, 1e-4, "identity mount: plain offset removal");
    EXPECT_NEAR(t.accel[1], 18.0f, 1e-4, "identity mount: plain offset removal");
    EXPECT_NEAR(t.accel[2], 27.0f, 1e-4, "identity mount: plain offset removal");
    EXPECT_NEAR(t.accel_raw[0], 10.0f, 1e-4, "identity mount: raw untouched");

    end(fb);
}

static void test_mag_stall_ms(void)
{
    begin("test_mag_stall_ms");
    int fb = g_fail;

    /* Ten sample periods once that exceeds the floor. 1 Hz -> 10 s. */
    EXPECT(imu_mag_stall_ms(1000) == 10000, "1 Hz waits ten periods");
    EXPECT(imu_mag_stall_ms(2000) == 5000,  "2 Hz waits ten periods");

    /* Below the floor the fast rates would warn on a hiccup: 100 Hz is 94 ms. */
    EXPECT(imu_mag_stall_ms(100000)  == 2000, "100 Hz floors at 2 s");
    EXPECT(imu_mag_stall_ms(1000000) == 2000, "1000 Hz floors at 2 s");
    EXPECT(imu_mag_stall_ms(5000)    == 2000, "5 Hz floors at 2 s");

    /* The floor boundary: 10 periods == 2 s exactly at 5 Hz. */
    EXPECT(imu_mag_stall_ms(4000) == 2500, "4 Hz is above the floor");

    /* An unknown rate must still bound the wait rather than never warning. */
    EXPECT(imu_mag_stall_ms(0)  == 2000, "an unknown rate uses the floor");
    EXPECT(imu_mag_stall_ms(-1) == 2000, "a negative rate uses the floor");

    end(fb);
}

static void test_int_fallback_ms(void)
{
    begin("test_int_fallback_ms");
    int fb = g_fail;

    /* Half a period: the fallback read re-arms the line, so waiting
     * longer yields fewer edges rather than later ones. */
    /*
     * The wait is (depth + grace) SAMPLE PERIODS, not a time. A grace in
     * milliseconds means a different thing at every rate -- 2 ms is thirteen
     * samples at 6664 Hz and three hundredths of one at 13 Hz -- so the
     * fallback fires when the line is late by a fixed number of SAMPLES.
     *
     * depth is what the line waits for: fifo_wm sample-sets for a FIFO
     * watermark, 1 for a per-sample data-ready.
     */
    /* 100 Hz, per-sample DRDY, 2 samples of grace: 3 periods = 30 ms. */
    EXPECT(imu_int_fallback_ms(100000, 1, 2) == 30, "100 Hz, depth 1, grace 2");
    /* Must exceed one period: half a period expires before the very edge it
     * is waiting for can arrive. */
    EXPECT(imu_int_fallback_ms(100000, 1, 2) > 10, "longer than one period");
    /* 833 Hz with the default wm=64: 66 periods = 79 ms, against the flat
     * 10 ms that would suppress the watermark entirely. */
    EXPECT(imu_int_fallback_ms(833000, 64, 2) == 79, "833 Hz, wm 64, grace 2");
    EXPECT(imu_int_fallback_ms(6664000, 64, 2) == 10, "6664 Hz, wm 64, grace 2");
    EXPECT(imu_int_fallback_ms(20000, 1, 2) == 150, "20 Hz, depth 1, grace 2");
    /* Degenerate inputs must not produce a zero or negative wait. */
    EXPECT(imu_int_fallback_ms(0, 64, 2) == 20, "unknown rate falls back");
    EXPECT(imu_int_fallback_ms(833000, 0, 2) > 0, "depth 0 is treated as 1");
    EXPECT(imu_int_fallback_ms(833000, 64, 0) > 0, "grace 0 is treated as 1");
    EXPECT(imu_int_fallback_ms(-1, 1, 1) == 20, "negative rate falls back");

    /* The fast end must not spin down to a zero wait. */
    EXPECT(imu_int_fallback_ms(6664000, 1, 1) >= 1, "6664 Hz still waits >= 1 ms");

    /*
     * Monotonic: a faster rate never waits longer than a slower one.  Seeded
     * at LONG_MAX so the first entry is always accepted -- a helper
     * clamped at 250 ms, so a literal seed worked; unclamped, 1 Hz with one
     * sample of depth and two of grace is 3000 ms.
     */
    long prev = LONG_MAX;
    static const int ladder[] = { 1, 10, 20, 50, 100, 200, 833, 1000, 6664 };
    int mono = 1;
    for (size_t i = 0; i < sizeof ladder / sizeof ladder[0]; i++) {
        long v = imu_int_fallback_ms(ladder[i] * 1000, 1, 2);
        if (v > prev) mono = 0;
        prev = v;
    }
    EXPECT(mono, "the interval never grows as the rate rises");

    /* An unknown rate falls back to the fixed constant. */
    EXPECT(imu_int_fallback_ms(0, 1, 2) == 20, "an unknown rate keeps the old 20 ms");
    EXPECT(imu_int_fallback_ms(-5, 1, 2) == 20, "and so does a nonsense one");

    end(fb);
}

static void test_nearest_odr(void)
{
    begin("test_nearest_odr");
    int fb = g_fail;

    /* The ISM330DHCX table, as registered. */
    static const int ism[] = { 13, 26, 52, 104, 208, 416, 833, 1666, 0 };
    EXPECT(nearest_odr(ism, 208) == 208, "exact match");
    EXPECT(nearest_odr(ism, 60)  == 52,  "round to nearer (52 vs 104)");
    EXPECT(nearest_odr(ism, 900) == 833, "round to nearer (833 vs 1666)");
    EXPECT(nearest_odr(ism, 1)   == 13,  "below min clamps to first");
    EXPECT(nearest_odr(ism, 5000) == 1666, "above max clamps to last");

    /* Exact ties resolve to the first (lower) entry: the update is strict
     * `d < best_diff`, so an equal distance does not displace the incumbent. */
    static const int tie[] = { 10, 20, 0 };
    EXPECT(nearest_odr(tie, 15) == 10, "tie resolves to lower");

    static const int one[] = { 100, 0 };
    EXPECT(nearest_odr(one, 5)    == 100, "single entry, low request");
    EXPECT(nearest_odr(one, 9999) == 100, "single entry, high request");

    end(fb);
}

/* ── snap_odr_up ─────────────────────────────────────────────────────────── */

/*
 * The rounding every register-table driver's odr_encode() chain performs, and
 * therefore the rate the chip really runs at. It differs from nearest_odr for
 * every request that falls in the lower half of a gap — which is the whole
 * reason this function exists: tuning the filter with nearest_odr while the
 * driver programs the snap-up rate makes the two disagree.
 */
static void test_snap_odr_up(void)
{
    begin("test_snap_odr_up");
    int fb = g_fail;

    static const int ism[] = { 13, 26, 52, 104, 208, 416, 833, 1666, 0 };
    EXPECT(snap_odr_up(ism, 208) == 208, "exact match");
    EXPECT(snap_odr_up(ism, 60)  == 104, "between entries rounds UP");
    EXPECT(snap_odr_up(ism, 1)   == 13,  "below min gives the first entry");
    EXPECT(snap_odr_up(ism, 5000) == 1666, "above max clamps to last");

    /* The regression this whole change is about, on the MMC5983MA grid:
     * the driver programs 200 Hz while nearest_odr() said 100. */
    static const int mmc[] = { 1, 10, 20, 50, 100, 200, 1000, 0 };
    EXPECT(snap_odr_up(mmc, 137) == 200, "137 -> 200 (driver), not 100");
    EXPECT(nearest_odr(mmc, 137) == 100, "nearest_odr still answers 100");

    /* Same disagreement on the IMU side, which the daemon had too:
     * imu_odr_mhz = 900 tuned the filter for 833 and ran the chip at 1666. */
    EXPECT(snap_odr_up(ism, 900) == 1666, "900 -> 1666 (driver), not 833");
    EXPECT(nearest_odr(ism, 900) == 833,  "nearest_odr still answers 833");

    static const int one[] = { 100, 0 };
    EXPECT(snap_odr_up(one, 5)    == 100, "single entry, low request");
    EXPECT(snap_odr_up(one, 9999) == 100, "single entry, high request");

    end(fb);
}

/* ── odr_actual_imu / odr_actual_mag ─────────────────────────────────────── */

static int stub_hook_calls;
static int stub_hook(int requested) { stub_hook_calls++; return requested * 2; }

/*
 * The resolver picks the driver's hook when it has one and falls back to
 * snap_odr_up otherwise. Divider-based parts (mpu925x, icm20948) rely on the
 * hook winning; every register-table driver relies on the NULL default.
 */
static void test_odr_actual_resolver(void)
{
    begin("test_odr_actual_resolver");
    int fb = g_fail;

    imu_ops_t imu_no_hook = {
        .supported_odr_mhz = { 12, 26, 52, 104, 0 },
        .actual_odr_mhz    = NULL,
    };
    EXPECT(odr_actual_imu(&imu_no_hook, 60) == 104,
           "NULL hook falls back to snap_odr_up");

    imu_ops_t imu_hook = imu_no_hook;
    imu_hook.actual_odr_mhz = stub_hook;
    stub_hook_calls = 0;
    EXPECT(odr_actual_imu(&imu_hook, 60) == 120,
           "hook wins over the table");
    EXPECT(stub_hook_calls == 1, "hook called exactly once");

    mag_ops_t mag_no_hook = {
        .supported_odr_mhz = { 1, 10, 20, 50, 100, 200, 1000, 0 },
        .actual_odr_mhz    = NULL,
    };
    EXPECT(odr_actual_mag(&mag_no_hook, 137) == 200,
           "mag NULL hook falls back to snap_odr_up");

    mag_ops_t mag_hook = mag_no_hook;
    mag_hook.actual_odr_mhz = stub_hook;
    stub_hook_calls = 0;
    EXPECT(odr_actual_mag(&mag_hook, 137) == 274, "mag hook wins");
    EXPECT(stub_hook_calls == 1, "mag hook called exactly once");

    end(fb);
}

/* ── timestamp reconstruction ────────────────────────────────────────────── */

static void anchor_init(ts_anchor_t *a)
{
    memset(a, 0, sizeof *a);
    pthread_mutex_init(&a->mtx, NULL);
}

static void test_timestamp_basic(void)
{
    begin("test_timestamp_basic");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);

    anchor_update(&a, /*chip*/ 1000, /*wall*/ 5000, /*tai*/ 6000,
                  /*mono*/ 0, /*nominal tick*/ 25000);

    uint64_t wall = 0, tai = 0;
    uint32_t gen = 0;

    /* At the anchor itself the offset is zero. */
    chip_to_wall(&a, 1000, 25000, &wall, &tai, &gen);
    EXPECT(wall == 5000, "wall at anchor");
    EXPECT(tai  == 6000, "tai at anchor");
    EXPECT(gen  == 1,    "gen after one update");

    /* 40 ticks later at 25 µs/tick = 1 ms = 1_000_000 ns. */
    chip_to_wall(&a, 1040, 25000, &wall, &tai, &gen);
    EXPECT(wall == 5000 + 1000000ULL, "wall advances by tick offset");
    EXPECT(tai  == 6000 + 1000000ULL, "tai advances by tick offset");

    /* NULL out-params must be tolerated. */
    chip_to_wall(&a, 1040, 25000, NULL, NULL, NULL);
    EXPECT(1, "NULL outputs do not crash");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

static void test_timestamp_wraparound(void)
{
    begin("test_timestamp_wraparound");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);

    /* Anchor near the top of the 32-bit counter; a later sample has wrapped
     * past 2^32.  The unsigned subtraction must recover the true 32-tick
     * delta rather than a ~4-billion-tick backward jump. */
    anchor_update(&a, 0xFFFFFFF0u, /*wall*/ 100000, /*tai*/ 200000,
                  /*mono*/ 0, /*nominal tick*/ 25000);

    uint64_t wall = 0, tai = 0;
    chip_to_wall(&a, 0x00000010u, 25000, &wall, &tai, NULL);

    /* (0x10 - 0xFFFFFFF0) mod 2^32 = 32 ticks. */
    EXPECT(wall == 100000 + 32ULL * 25000, "wall handles counter wrap");
    EXPECT(tai  == 200000 + 32ULL * 25000, "tai handles counter wrap");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

static void test_timestamp_no_hw_timer(void)
{
    begin("test_timestamp_no_hw_timer");
    int fb = g_fail;

    /* tick_ns == 0 is the !has_hw_timestamp case: the offset must degenerate
     * to zero regardless of the chip_ts passed, so wall/tai == the anchor. */
    ts_anchor_t a;
    anchor_init(&a);
    anchor_update(&a, 0, /*wall*/ 777, /*tai*/ 888,
                  /*mono*/ 0, /*nominal tick*/ 0);

    uint64_t wall = 1, tai = 1;
    chip_to_wall(&a, 123456, /*tick_ns*/ 0, &wall, &tai, NULL);
    EXPECT(wall == 777, "wall == anchor when tick_ns==0");
    EXPECT(tai  == 888, "tai == anchor when tick_ns==0");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

static void test_anchor_gen_increments(void)
{
    begin("test_anchor_gen_increments");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);

    uint32_t gen = 0;
    anchor_update(&a, 10, 1, 1, 0, 25000);
    chip_to_wall(&a, 10, 25000, NULL, NULL, &gen);
    EXPECT(gen == 1, "gen == 1 after first anchor");

    anchor_update(&a, 20, 2, 2, 0, 25000);
    chip_to_wall(&a, 20, 25000, NULL, NULL, &gen);
    EXPECT(gen == 2, "gen == 2 after re-anchor");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

/* ── Measured tick period ────────────────────────────────────────────────── */

/*
 * A 60 s anchor interval on a counter running 4.1% fast, which is what a real
 * ISM330DHCX measured: 40000 nominal ticks/s means 2.4e6 ticks in 60 s, but a
 * fast counter delivers more of them in the same wall time.
 */
#define FAST_PCT       4.08
#define NOM_TICK_NS    25000u
#define ANCHOR_GAP_NS  60000000000ULL   /* 60 s */

/* Ticks the chip emits over `wall_ns` when its period is `err_pct` short. */
static uint32_t ticks_over(uint64_t wall_ns, double err_pct)
{
    double real_tick = (double)NOM_TICK_NS / (1.0 + err_pct / 100.0);
    return (uint32_t)((double)wall_ns / real_tick + 0.5);
}

/*
 * Which host instant the anchor pairs the newest sample's chip_ts with.
 *
 * A chip_ts read AFTER the drain means "now at t_after", so the midpoint of
 * the burst read is the right pairing for it.  A chip_ts batched from the
 * FIFO's own timestamp is the instant the sample was taken, which is EARLIER
 * than t_before; pairing that with the midpoint places every reconstructed
 * timestamp late by about half the read
 * duration.
 *
 * Nothing caught it: dt is a difference, so the offset cancelled before it
 * reached the filter, and imud-imutest's imu.chipts.wall grades the chip/wall
 * RATIO, which a constant offset leaves at 1.0000.  It surfaced on the
 * the bench only as a sample-latency term that reads low.
 */
static void test_anchor_pairs_the_sample_instant(void)
{
    begin("test_anchor_pairs_the_sample_instant");
    int fb = g_fail;

    /*
     * One drain, modelled with real proportions: 833 Hz, a 64-sample burst,
     * and a read that takes 24 ms — twenty sample periods, which is what makes
     * the midpoint error visible rather than academic.
     */
    const uint64_t period   = 1200000ULL;                 /* 1.2 ms @ 833 Hz */
    const uint64_t t_newest = 1000000000000ULL;           /* newest sample */
    const uint64_t t_before = t_newest + period / 2;      /* status read */
    const uint64_t read_dur = 24000000ULL;                /* 24 ms drain */
    const uint64_t t_after  = t_before + read_dur;

    EXPECT(anchor_wall_ns(t_before, t_after, true) == t_before,
           "a driver reporting sample instants pairs with the pre-read stamp");
    EXPECT(anchor_wall_ns(t_before, t_after, false) == t_before + read_dur / 2,
           "a driver with no chip timer still pairs with the read midpoint");

    /* End to end: anchor on this burst, then reconstruct the newest sample. */
    ts_anchor_t a;
    anchor_init(&a);
    const uint32_t chip_newest = 500000;
    anchor_update(&a, chip_newest,
                  anchor_wall_ns(t_before, t_after, true),
                  /*tai*/ anchor_wall_ns(t_before, t_after, true),
                  /*mono*/ 0, NOM_TICK_NS);

    uint64_t wall = 0, tai = 0;
    uint32_t gen = 0;
    chip_to_wall(&a, chip_newest, NOM_TICK_NS, &wall, &tai, &gen);

    /* Within one sample period of when it was really taken... */
    EXPECT(wall >= t_newest && wall - t_newest <= period,
           "the newest sample reconstructs within a sample period of truth");
    /* ...and specifically NOT half a read late, which is what the midpoint
     * gives.  Pin the direction: an error that grows with the read is the
     * defect, an error bounded by the sample period is the residual. */
    EXPECT(wall - t_newest < read_dur / 2,
           "and is not late by half the read duration");

    /*
     * A constant offset must not reach the filter.  Two samples one period
     * apart must be one period apart after reconstruction, whichever pairing
     * the anchor used — this is why the defect never moved attitude.
     */
    uint64_t w_prev = 0;
    chip_to_wall(&a, chip_newest - (uint32_t)(period / NOM_TICK_NS),
                 NOM_TICK_NS, &w_prev, &tai, &gen);
    EXPECT(wall - w_prev == period, "dt between samples is unchanged by it");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

/*
 * A sample OLDER than the anchor must reconstruct behind it, not 29.8 hours
 * ahead of it.
 *
 * This is the ordinary case, not a corner one: the reader anchors on the
 * NEWEST sample of a burst and then pushes the whole burst, so every other
 * sample in that burst is older than the anchor by up to a watermark's worth
 * of ticks.  chip_to_wall took the delta as UNSIGNED, so each of those became
 * a ~2^32-tick jump forward, and imu.c published it as ts_wall_ns.
 *
 * Nothing downstream noticed: the dt clamp and the anchor-generation check
 * both reject the discontinuity before it reaches the filter, and the latency
 * histogram drops a sample whose wall lands after its own read stamp.  So it
 * went out on the wire and nowhere else — which is exactly why it needs a
 * test rather than a downstream assertion.
 */
static void test_samples_older_than_the_anchor(void)
{
    begin("test_samples_older_than_the_anchor");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);

    const uint32_t chip_anchor = 1000000;
    const uint64_t wall_anchor = 5000000000ULL;
    anchor_update(&a, chip_anchor, wall_anchor, wall_anchor + 37000000000ULL,
                  /*mono*/ 0, NOM_TICK_NS);

    uint64_t wall = 0, tai = 0;
    uint32_t gen = 0;

    /* One tick before the anchor: 25 µs earlier, not 29.8 hours later. */
    chip_to_wall(&a, chip_anchor - 1, NOM_TICK_NS, &wall, &tai, &gen);
    EXPECT(wall == wall_anchor - 25000ULL, "one tick before the anchor");
    EXPECT(tai  == wall_anchor + 37000000000ULL - 25000ULL, "tai follows");

    /* A whole watermark back — 64 sample-sets at 833 Hz is 3072 ticks. */
    chip_to_wall(&a, chip_anchor - 3072, NOM_TICK_NS, &wall, &tai, &gen);
    EXPECT(wall == wall_anchor - 3072ULL * 25000ULL,
           "a full burst before the anchor stays behind it");
    EXPECT(wall < wall_anchor, "and is emphatically not in the future");

    /* The counter's own 32-bit wrap still resolves the short way round: an
     * anchor just past the wrap, a sample just before it. */
    ts_anchor_t w;
    anchor_init(&w);
    anchor_update(&w, 0x00000010u, wall_anchor, wall_anchor, 0, NOM_TICK_NS);
    chip_to_wall(&w, 0xFFFFFFF0u, NOM_TICK_NS, &wall, NULL, NULL);
    EXPECT(wall == wall_anchor - 32ULL * 25000ULL,
           "a pre-wrap sample is 32 ticks back, not 2^32 forward");
    pthread_mutex_destroy(&w.mtx);

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

static void test_tick_measured_from_anchors(void)
{
    begin("test_tick_measured_from_anchors");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);

    /* First anchor: nothing to measure against yet, so the nominal stands. */
    anchor_update(&a, 0, /*wall*/ 0, /*tai*/ 0, /*mono*/ 0, NOM_TICK_NS);
    EXPECT(anchor_measured_tick_ns(&a) == 0.0, "no estimate from one anchor");

    uint32_t t1 = ticks_over(ANCHOR_GAP_NS, FAST_PCT);
    anchor_update(&a, t1, ANCHOR_GAP_NS, ANCHOR_GAP_NS, ANCHOR_GAP_NS,
                  NOM_TICK_NS);

    double meas = anchor_measured_tick_ns(&a);
    double want = (double)NOM_TICK_NS / (1.0 + FAST_PCT / 100.0);
    EXPECT(meas > 0.0, "an estimate exists after two anchors");
    EXPECT(fabs(meas - want) < 1.0, "measured period matches the real one");
    EXPECT(meas < NOM_TICK_NS, "a fast counter measures a shorter period");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

/*
 * The defect this exists for.  Scaling ticks by the declared period made
 * reported time run ahead inside an anchor epoch and jump backwards when the
 * anchor refreshed — about 2.4 s of sawtooth per 60 s on a 4% counter, in the
 * timestamps that reach the wire.
 */
static void test_no_sawtooth_within_epoch(void)
{
    begin("test_no_sawtooth_within_epoch");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);
    anchor_update(&a, 0, 0, 0, 0, NOM_TICK_NS);

    uint32_t t1 = ticks_over(ANCHOR_GAP_NS, FAST_PCT);
    anchor_update(&a, t1, ANCHOR_GAP_NS, ANCHOR_GAP_NS, ANCHOR_GAP_NS,
                  NOM_TICK_NS);

    /* Walk to the end of the next epoch and compare reconstructed elapsed time
     * against the wall clock that will be there when the anchor refreshes. */
    uint32_t t2 = t1 + ticks_over(ANCHOR_GAP_NS, FAST_PCT);
    uint64_t wall = 0;
    chip_to_wall(&a, t2, NOM_TICK_NS, &wall, NULL, NULL);

    double err_s = ((double)wall - (double)(ANCHOR_GAP_NS * 2)) * 1e-9;
    EXPECT(fabs(err_s) < 0.05,
           "reconstructed time tracks the wall clock across a whole epoch");

    /* And the size of what was being corrected, so the test says why it is
     * here: the nominal period would have run ~2.4 s ahead over the same span. */
    uint64_t naive = (uint64_t)(t2 - t1) * NOM_TICK_NS + ANCHOR_GAP_NS;
    double naive_err_s = ((double)naive - (double)(ANCHOR_GAP_NS * 2)) * 1e-9;
    EXPECT(naive_err_s > 2.0,
           "the declared period would have drifted seconds over one epoch");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

/*
 * dt is the interval between two consecutive samples, which is what the filter
 * integrates.  A 4% long dt scales all integrated rotation by 4%.
 */
static void test_dt_between_samples_is_true(void)
{
    begin("test_dt_between_samples_is_true");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);
    anchor_update(&a, 0, 0, 0, 0, NOM_TICK_NS);
    uint32_t t1 = ticks_over(ANCHOR_GAP_NS, FAST_PCT);
    anchor_update(&a, t1, ANCHOR_GAP_NS, ANCHOR_GAP_NS, ANCHOR_GAP_NS,
                  NOM_TICK_NS);

    /* 48 ticks per sample is an 833 Hz ISM330DHCX. */
    uint64_t w0 = 0, w1 = 0;
    chip_to_wall(&a, t1 + 4800, NOM_TICK_NS, &w0, NULL, NULL);
    chip_to_wall(&a, t1 + 4848, NOM_TICK_NS, &w1, NULL, NULL);

    double dt      = (double)(w1 - w0) * 1e-9;
    double true_dt = 48.0 * ((double)NOM_TICK_NS / (1.0 + FAST_PCT / 100.0)) * 1e-9;
    EXPECT(fabs(dt - true_dt) / true_dt < 0.002, "dt uses the measured period");
    EXPECT(dt < 48.0 * NOM_TICK_NS * 1e-9,
           "dt is shorter than the declared period would give");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

static void test_tick_measurement_rejects_nonsense(void)
{
    begin("test_tick_measurement_rejects_nonsense");
    int fb = g_fail;

    /* A counter reset under us: chip_ts restarts, so the tick delta is a huge
     * wrapped value and the implied period is absurd.  It must be thrown away,
     * not filtered in. */
    ts_anchor_t a;
    anchor_init(&a);
    anchor_update(&a, 2400000, 0, 0, 0, NOM_TICK_NS);
    anchor_update(&a, 5000, ANCHOR_GAP_NS, ANCHOR_GAP_NS, ANCHOR_GAP_NS,
                  NOM_TICK_NS);
    EXPECT(anchor_measured_tick_ns(&a) == 0.0, "a wrapped delta is rejected");

    /* Too short an interval to measure anything: the reader anchors every
     * burst when there is no hardware timer, and those must not be used. */
    ts_anchor_t b;
    anchor_init(&b);
    anchor_update(&b, 0, 0, 0, 0, NOM_TICK_NS);
    anchor_update(&b, 4000, 100000000ULL, 0, 100000000ULL, NOM_TICK_NS);
    EXPECT(anchor_measured_tick_ns(&b) == 0.0, "a 0.1 s interval is rejected");

    /* No hardware timer at all: nominal 0 disables measurement outright. */
    ts_anchor_t c;
    anchor_init(&c);
    anchor_update(&c, 0, 0, 0, 0, 0);
    anchor_update(&c, 0, ANCHOR_GAP_NS, 0, ANCHOR_GAP_NS, 0);
    EXPECT(anchor_measured_tick_ns(&c) == 0.0, "no timer means no estimate");

    /*
     * The very first anchor has nothing to measure against.  The daemon's
     * lands at whatever CLOCK_MONOTONIC reads at startup — system uptime, not
     * zero — against a counter that has been running since the board powered
     * on, and differencing that against a zeroed struct divides uptime by
     * total ticks.
     *
     * The numbers below are chosen so that quotient lands *inside* the
     * plausibility band, which is the case that matters: when the daemon
     * starts near boot the two epochs nearly coincide and the bogus figure
     * looks entirely reasonable.  It stops looking reasonable after a daemon
     * restart, or a driver-level chip reset, and by then it is in use.
     */
    ts_anchor_t e;
    anchor_init(&e);
    anchor_update(&e, 160000000u, 0, 0,
                  160000000ULL * NOM_TICK_NS /* uptime that divides to 25000 */,
                  NOM_TICK_NS);
    EXPECT(anchor_measured_tick_ns(&e) == 0.0,
           "the first anchor measures nothing, however plausible the quotient");

    /* A believable error is still accepted, so the rejection band is not so
     * tight that it throws away the thing it exists to measure. */
    ts_anchor_t d;
    anchor_init(&d);
    anchor_update(&d, 0, 0, 0, 0, NOM_TICK_NS);
    anchor_update(&d, ticks_over(ANCHOR_GAP_NS, 8.0), ANCHOR_GAP_NS, 0,
                  ANCHOR_GAP_NS, NOM_TICK_NS);
    EXPECT(anchor_measured_tick_ns(&d) > 0.0, "an 8% error is still measured");

    pthread_mutex_destroy(&a.mtx);
    pthread_mutex_destroy(&b.mtx);
    pthread_mutex_destroy(&c.mtx);
    pthread_mutex_destroy(&d.mtx);
    pthread_mutex_destroy(&e.mtx);
    end(fb);
}

static void test_tick_estimate_is_filtered(void)
{
    begin("test_tick_estimate_is_filtered");
    int fb = g_fail;

    ts_anchor_t a;
    anchor_init(&a);
    uint64_t mono = 0;
    uint32_t chip = 0;

    anchor_update(&a, chip, 0, 0, mono, NOM_TICK_NS);
    mono += ANCHOR_GAP_NS; chip += ticks_over(ANCHOR_GAP_NS, FAST_PCT);
    anchor_update(&a, chip, mono, 0, mono, NOM_TICK_NS);
    double settled = anchor_measured_tick_ns(&a);

    /* One interval at a different rate must move the estimate part of the way,
     * not all of it — otherwise a single noisy anchor swings every dt. */
    mono += ANCHOR_GAP_NS; chip += ticks_over(ANCHOR_GAP_NS, -4.0);
    anchor_update(&a, chip, mono, 0, mono, NOM_TICK_NS);
    double after = anchor_measured_tick_ns(&a);

    /* Where an unfiltered estimate would have landed: the new measurement in
     * full.  The filtered one must be a fraction of the way there, so bound it
     * well short of halfway rather than just short of the endpoint — an
     * unfiltered value lands on the endpoint and a loose bound lets rounding
     * hide it. */
    double whole = NOM_TICK_NS / (1.0 - 0.04);
    EXPECT(after > settled, "the estimate moved toward the new measurement");
    EXPECT(after < settled + 0.5 * (whole - settled),
           "it moved a fraction of the way, not the whole way");
    EXPECT(fabs((after - settled) - 0.25 * (whole - settled))
               < 0.05 * (whole - settled),
           "it moved about a quarter of the way");

    pthread_mutex_destroy(&a.mtx);
    end(fb);
}

/* ── Sample-latency histogram ────────────────────────────────────────────── */

static void test_lat_buckets_and_percentiles(void)
{
    begin("test_lat_buckets_and_percentiles");
    int fb = g_fail;
    lat_hist_t h;
    memset(&h, 0, sizeof h);

    EXPECT(lat_percentile(&h, 0.99) == 0, "empty histogram reports 0");

    /* 1000 samples all at 5 ms. Bucket for 5000 µs is floor(log2 5000) = 12,
     * whose upper edge is 2^13 µs = 8192 µs. Everything lands there, so every
     * percentile reads that edge — bucket resolution, conservative by design. */
    for (int i = 0; i < 1000; i++) lat_record(&h, 5000000ull);
    EXPECT(h.count == 1000, "count tracks records");
    EXPECT(lat_percentile(&h, 0.50) == 8192000ull, "p50 = bucket upper edge");
    EXPECT(lat_percentile(&h, 0.99) == 8192000ull, "p99 = same bucket");
    EXPECT(h.max_ns == 5000000ull, "max_ns is exact, not bucketed");

    /*
     * The property that matters for a p99 budget: one late sample in a hundred
     * must move p99 and must NOT move p50. 99 fast, 1 slow.
     */
    memset(&h, 0, sizeof h);
    for (int i = 0; i < 99; i++) lat_record(&h, 100000ull);   /* 100 µs */
    lat_record(&h, 900000000ull);                              /* 900 ms  */
    EXPECT(lat_percentile(&h, 0.50) < 1000000ull, "p50 unmoved by one outlier");
    EXPECT(lat_percentile(&h, 0.99) < 1000000ull, "p99 at exactly 99/100 is fast");
    EXPECT(lat_percentile(&h, 1.00) > 500000000ull, "p100 catches the outlier");
    EXPECT(h.max_ns == 900000000ull, "max_ns caught it exactly");

    /* Monotonicity across the whole range, which a bucketing bug breaks. */
    memset(&h, 0, sizeof h);
    for (uint64_t ns = 1000; ns < 2000000000ull; ns *= 2) lat_record(&h, ns);
    uint64_t prev = 0;
    bool mono = true;
    for (double p = 0.1; p <= 1.0; p += 0.1) {
        uint64_t v = lat_percentile(&h, p);
        if (v < prev) mono = false;
        prev = v;
    }
    EXPECT(mono, "percentiles are non-decreasing in p");

    /* Saturation: nothing wraps or lands out of range. */
    memset(&h, 0, sizeof h);
    lat_record(&h, 60ull * 1000000000ull);      /* a minute */
    EXPECT(h.count == 1 && h.bucket[LAT_BUCKETS-1] == 1,
           "an absurd interval saturates into the top bucket");
    EXPECT(h.max_ns == 60ull*1000000000ull, "max_ns still exact when saturated");

    /* Sub-microsecond is a real measurement, not something to drop. */
    memset(&h, 0, sizeof h);
    lat_record(&h, 0);
    lat_record(&h, 999);
    EXPECT(h.count == 2 && h.bucket[0] == 2, "sub-µs lands in bucket 0");

    end(fb);
}

static void test_lat_reset_keeps_all_time_max(void)
{
    begin("test_lat_reset_keeps_all_time_max");
    int fb = g_fail;
    lat_hist_t h;
    memset(&h, 0, sizeof h);

    lat_record(&h, 500000000ull);               /* 500 ms spike */
    lat_record(&h, 100000ull);
    EXPECT(h.max_ever_ns == 500000000ull, "all-time max recorded");

    lat_reset_window(&h);
    EXPECT(h.count == 0, "window count cleared");
    EXPECT(h.max_ns == 0, "window max cleared");
    EXPECT(lat_percentile(&h, 0.99) == 0, "buckets cleared");
    EXPECT(h.max_ever_ns == 500000000ull,
           "all-time max SURVIVES the reset — a stall must not be erasable "
           "by the next window rolling over");

    lat_record(&h, 1000ull);
    EXPECT(h.max_ns == 1000ull, "window max restarts from the new data");
    EXPECT(h.max_ever_ns == 500000000ull, "all-time max still held");

    end(fb);
}

/*
 * lat_step: the two windows publish independently.
 *
 * The two must not share one gate keyed off the FIFO
 * histogram's count.  `pipe` records on essentially every sample, but `fifo`
 * records only while the reconstructed sample time still trails the read
 * stamp — and before ts_anchor_t has measured the chip's tick period, a part a
 * few percent fast makes the extrapolation outrun the read stamp and `fifo`
 * stops filling entirely.  With a shared gate that withheld `pipe` too, which
 * is why a Pi 5 latency matrix produces no numbers at all in six
 * 40 s runs while the 30-minute soak, which outlived the first re-anchor,
 * captured them fine.
 */
static void test_lat_step_windows_are_independent(void)
{
    begin("test_lat_step_windows_are_independent");
    int fb = g_fail;

    lat_hist_t fifo, pipe;
    lat_pub_t  pf, pp;
    const uint64_t need = 100;

    memset(&fifo, 0, sizeof fifo);
    memset(&pipe, 0, sizeof pipe);

    /*
     * wall AHEAD of the read stamp on every sample, which is exactly the state
     * an unmeasured tick period produces.  fifo can never record; pipe must
     * still publish on schedule.
     */
    int fifo_pubs = 0, pipe_pubs = 0;
    for (uint64_t i = 0; i < 250; i++) {
        uint64_t read_done = 1000000000ull + i * 1000000ull;
        lat_step(&fifo, &pipe, need,
                 /*wall*/ read_done + 40000000ull,   /* 40 ms ahead */
                 read_done, read_done,
                 /*now*/  read_done + 300000ull,     /* 0.3 ms pipeline */
                 &pf, &pp);
        if (pf.valid) fifo_pubs++;
        if (pp.valid) pipe_pubs++;
    }
    EXPECT(fifo.count == 0, "fifo records nothing while wall runs ahead");
    EXPECT(fifo_pubs == 0,  "and therefore never publishes");
    EXPECT(pipe_pubs == 2,  "pipe publishes twice in 250 samples at need=100 "
                            "— NOT held back by the empty fifo window");
    EXPECT(pp.p99 > 0, "the published pipeline percentile is real");

    /* And the reverse: a full fifo window does not drag pipe along with it. */
    memset(&fifo, 0, sizeof fifo);
    memset(&pipe, 0, sizeof pipe);
    for (uint64_t i = 0; i < need; i++) {
        uint64_t read_done = 1000000000ull + i * 1000000ull;
        /* now == read_done, so the pipe term is not recordable this pass. */
        lat_step(&fifo, &pipe, need, read_done - 30000000ull, read_done,
                 read_done, read_done, &pf, &pp);
    }
    EXPECT(pf.valid,  "fifo publishes on its own count");
    EXPECT(!pp.valid, "pipe stays silent on an empty window rather than "
                      "publishing a zero that reads as a measurement");
    EXPECT(fifo.count == 0, "the published window was rolled");

    end(fb);
}

/*
 * lat_step clamps both differences instead of subtracting into a wrap.  An
 * anchor refresh can momentarily put the reconstructed time past the read
 * stamp; an unsigned underflow there lands in the top bucket and poisons p99
 * for the whole window.
 */
/*
 * Replay puts the two ends on two different clocks: the FIFO term is rebuilt
 * on the RECORDING host's clock so it reproduces what the live daemon
 * measured, while the pipeline term stays on the machine actually running.
 * Feeding one stamp to both is what reported a 416010576.8 ms FIFO latency --
 * the age of the recording, not a latency.
 */
static void test_lat_step_ends_are_on_independent_clocks(void)
{
    begin("test_lat_step_ends_are_on_independent_clocks");
    int fb = g_fail;

    lat_hist_t fifo, pipe;
    lat_pub_t  pf, pp;
    memset(&fifo, 0, sizeof fifo);
    memset(&pipe, 0, sizeof pipe);

    /* Capture clock: sample at t=1e9, delivered 5 ms later. */
    const uint64_t cap_wall  = 1000000000ull;
    const uint64_t cap_done  = cap_wall + 5000000ull;
    /* This host, replaying it a long time afterwards. */
    const uint64_t host_done = 9000000000000ull;
    const uint64_t host_now  = host_done + 2000000ull;   /* 2 ms pipeline */

    lat_step(&fifo, &pipe, 1, cap_wall, cap_done, host_done, host_now,
             &pf, &pp);

    /* need=1, so each window publishes and resets on this one call; the
     * all-time max is what survives (see imu_math.h). */
    EXPECT(pf.valid && pp.valid, "both terms recorded and published");
    EXPECT(pf.max_ever <= 8000000ull && pf.max_ever >= 4000000ull,
           "FIFO term is the 5 ms the recording saw, not the gap between "
           "the recording and this replay");
    EXPECT(pp.max_ever <= 3000000ull,
           "pipeline term is this machine's 2 ms, not the capture's");

    end(fb);
}

/* ── ovf_latch_step (FLAG_FIFO_OVERFLOW) ─────────────────────────────────── */

/* Step the latch n times at `dt` with a static counter; return the last
 * verdict, and how many of those steps were latched. */
static bool ovf_coast(ovf_latch_t *l, uint64_t count, float dt, int n,
                      int *n_latched)
{
    bool up = false;
    if (n_latched) *n_latched = 0;
    for (int i = 0; i < n; i++) {
        up = ovf_latch_step(l, count, dt);
        if (up && n_latched) (*n_latched)++;
    }
    return up;
}

static void test_ovf_latch(void)
{
    begin("test_ovf_latch");
    int fb = g_fail;

    const float dt = 1.0f / 833.0f;   /* the default IMU rate */

    /* A sensor that never gaps never raises the flag, however long it runs. */
    ovf_latch_t quiet = OVF_LATCH_INIT;
    int nl;
    ovf_coast(&quiet, 0, dt, 10000, &nl);
    EXPECT(nl == 0, "a counter that never advances never raises the flag");

    /* First gap raises it, and it holds for the whole window. */
    ovf_latch_t l = OVF_LATCH_INIT;
    EXPECT(ovf_latch_step(&l, 1, dt), "a new gap raises the flag");

    /* Just under the window: still up.  ovf_latch_step consumed no dt on the
     * raising step, so the window has the full OVF_LATCH_S left to run. */
    int steps_in = (int)(OVF_LATCH_S / dt) - 2;
    EXPECT(ovf_coast(&l, 1, dt, steps_in, &nl) && nl == steps_in,
           "the flag holds for every step inside the window");

    /* Past it: down, and it stays down. */
    ovf_coast(&l, 1, dt, 4, NULL);
    EXPECT(!ovf_coast(&l, 1, dt, 100, &nl) && nl == 0,
           "the flag clears once the window elapses, and stays clear");

    /* A second gap mid-window re-arms the FULL window rather than riding out
     * the first one — otherwise a sensor gapping steadily would flicker. */
    ovf_latch_t r = OVF_LATCH_INIT;
    ovf_latch_step(&r, 1, dt);
    ovf_coast(&r, 1, dt, (int)(OVF_LATCH_S / dt) - 2, NULL);  /* nearly expired */
    EXPECT(ovf_latch_step(&r, 2, dt), "a second gap raises it again");
    EXPECT(ovf_coast(&r, 2, dt, steps_in, &nl) && nl == steps_in,
           "the second gap re-arms the full window, not the remainder");

    /* A burst of drops arrives as one big jump in the counter, and must latch
     * once — the flag says "there was a gap", it is not a count. */
    ovf_latch_t b = OVF_LATCH_INIT;
    EXPECT(ovf_latch_step(&b, 4096, dt), "a large jump latches");
    EXPECT(ovf_coast(&b, 4096, dt, (int)(OVF_LATCH_S / dt) + 8, &nl)
           == false && nl > 0,
           "a large jump latches once, for one window, not once per sample");

    /* The window is real time, not a step count: at a much slower rate the
     * same number of SECONDS elapses in far fewer steps. */
    ovf_latch_t s = OVF_LATCH_INIT;
    ovf_latch_step(&s, 1, 0.1f);                      /* 10 Hz */
    EXPECT(ovf_coast(&s, 1, 0.1f, 19, &nl) && nl == 19,
           "at 10 Hz the window is still 2 s, so 19 more steps stay latched");
    EXPECT(!ovf_coast(&s, 1, 0.1f, 2, NULL),
           "and the 21st clears it");

    /* A counter already non-zero at the first step really did gap: adopting it
     * silently would hide an overflow that happened during alignment. */
    ovf_latch_t warm = OVF_LATCH_INIT;
    EXPECT(ovf_latch_step(&warm, 7, dt),
           "a counter already non-zero on the first step raises the flag");

    end(fb);
}

static void test_lat_step_clamps_rather_than_wraps(void)
{
    begin("test_lat_step_clamps_rather_than_wraps");
    int fb = g_fail;

    lat_hist_t fifo, pipe;
    lat_pub_t  pf, pp;
    memset(&fifo, 0, sizeof fifo);
    memset(&pipe, 0, sizeof pipe);

    /* wall one ns past the read stamp, and now one ns before it. */
    lat_step(&fifo, &pipe, 1, /*wall*/ 1000001, /*read_done*/ 1000000,
             /*pipe_start*/ 1000000, /*now*/ 999999, &pf, &pp);

    EXPECT(fifo.count == 0, "wall past the read stamp records nothing");
    EXPECT(pipe.count == 0, "now before the read stamp records nothing");
    EXPECT(fifo.max_ever_ns == 0 && pipe.max_ever_ns == 0,
           "no 18-exasecond outlier reached either histogram");

    end(fb);
}

/*
 * The re-anchor interval has to clear ANCHOR_MIN_INTERVAL_NS or the tick period
 * is never measured at all.  imu.c re-anchors at 25 s until it has one, then
 * settles to 60 s; this pins the claim that 25 s is enough and that the guard
 * still rejects a shorter gap, so neither number can drift into the other.
 */
static void test_anchor_interval_bounds(void)
{
    begin("test_anchor_interval_bounds");
    int fb = g_fail;

    /* 25 s: what the reader uses before the tick is known. */
    {
        ts_anchor_t a;
        anchor_init(&a);
        const uint64_t gap = 25000000000ULL;
        anchor_update(&a, 0, 0, 0, 0, NOM_TICK_NS);
        anchor_update(&a, ticks_over(gap, FAST_PCT), gap, gap, gap,
                      NOM_TICK_NS);
        double meas = anchor_measured_tick_ns(&a);
        double want = (double)NOM_TICK_NS / (1.0 + FAST_PCT / 100.0);
        EXPECT(meas > 0.0, "a 25 s gap measures the tick period");
        EXPECT(fabs(meas - want) < 1.0, "and measures it correctly");
        pthread_mutex_destroy(&a.mtx);
    }

    /* 19 s: still inside the guard, so deliberately no measurement.  The guard
     * exists to keep read-position jitter under 0.2% of the interval. */
    {
        ts_anchor_t a;
        anchor_init(&a);
        const uint64_t gap = 19000000000ULL;
        anchor_update(&a, 0, 0, 0, 0, NOM_TICK_NS);
        anchor_update(&a, ticks_over(gap, FAST_PCT), gap, gap, gap,
                      NOM_TICK_NS);
        EXPECT(anchor_measured_tick_ns(&a) == 0.0,
               "a 19 s gap is rejected — under ANCHOR_MIN_INTERVAL_NS");
        pthread_mutex_destroy(&a.mtx);
    }

    end(fb);
}

static void test_ts_ns(void)
{
    begin("test_ts_ns");
    int fb = g_fail;

    struct timespec t = { .tv_sec = 3, .tv_nsec = 500 };
    EXPECT(ts_ns(&t) == 3000000500ULL, "sec*1e9 + nsec");

    struct timespec z = { .tv_sec = 0, .tv_nsec = 0 };
    EXPECT(ts_ns(&z) == 0, "zero");

    end(fb);
}

/* ── mount rotation ──────────────────────────────────────────────────────── */

static void test_mount_rotation(void)
{
    begin("test_mount_rotation");
    int fb = g_fail;

    imud_config_t cfg;
    memset(&cfg, 0, sizeof cfg);

    /* Not configured → identity (no change). */
    cfg.mount_set = false;
    float v0[3] = { 1.0f, 2.0f, 3.0f };
    apply_mount_rot_if_set(&cfg, v0);
    EXPECT_NEAR(v0[0], 1.0f, 1e-6, "unset: x unchanged");
    EXPECT_NEAR(v0[1], 2.0f, 1e-6, "unset: y unchanged");
    EXPECT_NEAR(v0[2], 3.0f, 1e-6, "unset: z unchanged");

    /* Identity matrix, mount_set true → unchanged. */
    cfg.mount_set = true;
    double I[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} };
    memcpy(cfg.mount_rot, I, sizeof I);
    float v1[3] = { 4.0f, 5.0f, 6.0f };
    apply_mount_rot_if_set(&cfg, v1);
    EXPECT_NEAR(v1[0], 4.0f, 1e-6, "identity: x");
    EXPECT_NEAR(v1[1], 5.0f, 1e-6, "identity: y");
    EXPECT_NEAR(v1[2], 6.0f, 1e-6, "identity: z");

    /* Swap X/Y and flip Z — checks full matrix application with a sign. */
    double R[3][3] = { {0,1,0}, {1,0,0}, {0,0,-1} };
    memcpy(cfg.mount_rot, R, sizeof R);
    float v2[3] = { 1.0f, 2.0f, 3.0f };
    apply_mount_rot_if_set(&cfg, v2);
    EXPECT_NEAR(v2[0], 2.0f,  1e-6, "swap: x<-y");
    EXPECT_NEAR(v2[1], 1.0f,  1e-6, "swap: y<-x");
    EXPECT_NEAR(v2[2], -3.0f, 1e-6, "flip: z<--z");

    end(fb);
}

/* ── calibration application ─────────────────────────────────────────────── */

static void test_apply_imu_cal(void)
{
    begin("test_apply_imu_cal");
    int fb = g_fail;

    imud_cal_t cal;
    memset(&cal, 0, sizeof cal);

    /* No cal flags → sample untouched. */
    imu_sample_t s0 = {0};
    s0.accel[0] = 10; s0.accel[1] = 20; s0.accel[2] = 30;
    s0.gyro[0]  = 1;  s0.gyro[1]  = 1;  s0.gyro[2]  = 1;
    s0.temp_c   = 40;
    apply_imu_cal(&cal, &s0);
    EXPECT_NEAR(s0.accel[0], 10, 1e-6, "no cal: accel unchanged");
    EXPECT_NEAR(s0.gyro[1],  1,  1e-6, "no cal: gyro unchanged");

    /* Accel offset + scale: (a - offset) * scale. */
    cal.has_accel = true;
    cal.accel_offset[0] = 1; cal.accel_offset[1] = 2; cal.accel_offset[2] = 3;
    cal.accel_scale[0]  = 2; cal.accel_scale[1]  = 2; cal.accel_scale[2]  = 2;
    imu_sample_t s1 = {0};
    s1.accel[0] = 10; s1.accel[1] = 20; s1.accel[2] = 30;
    apply_imu_cal(&cal, &s1);
    EXPECT_NEAR(s1.accel[0], (10 - 1) * 2, 1e-5, "accel x = (a-off)*scale");
    EXPECT_NEAR(s1.accel[1], (20 - 2) * 2, 1e-5, "accel y = (a-off)*scale");
    EXPECT_NEAR(s1.accel[2], (30 - 3) * 2, 1e-5, "accel z = (a-off)*scale");

    /* Gyro temperature compensation: gyro -= coeff * (T - Tref). */
    memset(&cal, 0, sizeof cal);
    cal.has_gyro_temp = true;
    cal.gyro_temp_ref_c   = 25;
    cal.gyro_temp_coeff[0] = 0.1f;
    cal.gyro_temp_coeff[1] = 0.2f;
    cal.gyro_temp_coeff[2] = 0.3f;
    imu_sample_t s2 = {0};
    s2.gyro[0] = 1; s2.gyro[1] = 1; s2.gyro[2] = 1;
    s2.temp_c  = 35;                 /* dT = 10 */
    apply_imu_cal(&cal, &s2);
    EXPECT_NEAR(s2.gyro[0], 1 - 0.1 * 10, 1e-5, "gyro x temp-compensated");
    EXPECT_NEAR(s2.gyro[1], 1 - 0.2 * 10, 1e-5, "gyro y temp-compensated");
    EXPECT_NEAR(s2.gyro[2], 1 - 0.3 * 10, 1e-5, "gyro z temp-compensated");
    /* accel path is guarded off (has_accel false) so accel stays zero. */
    EXPECT_NEAR(s2.accel[0], 0, 1e-6, "gyro-temp only leaves accel");

    end(fb);
}

static void test_apply_mag_cal(void)
{
    begin("test_apply_mag_cal");
    int fb = g_fail;

    imud_cal_t cal;
    memset(&cal, 0, sizeof cal);

    /* No mag cal → field untouched. */
    mag_sample_t m0 = {0};
    m0.field[0] = 10; m0.field[1] = 20; m0.field[2] = 30;
    apply_mag_cal(&cal, &m0);
    EXPECT_NEAR(m0.field[0], 10, 1e-6, "no cal: field unchanged");

    /* Hard-iron subtract then diagonal soft-iron scale. */
    cal.has_mag = true;
    cal.mag_hard_iron[0] = 1; cal.mag_hard_iron[1] = 2; cal.mag_hard_iron[2] = 3;
    cal.mag_soft_iron[0][0] = 2;
    cal.mag_soft_iron[1][1] = 2;
    cal.mag_soft_iron[2][2] = 2;
    mag_sample_t m1 = {0};
    m1.field[0] = 10; m1.field[1] = 20; m1.field[2] = 30;
    apply_mag_cal(&cal, &m1);
    EXPECT_NEAR(m1.field[0], (10 - 1) * 2, 1e-5, "mag x = soft*(raw-hard)");
    EXPECT_NEAR(m1.field[1], (20 - 2) * 2, 1e-5, "mag y = soft*(raw-hard)");
    EXPECT_NEAR(m1.field[2], (30 - 3) * 2, 1e-5, "mag z = soft*(raw-hard)");

    /* Off-diagonal soft-iron cross terms mix axes: row0 = [1,1,0]·tmp. */
    memset(&cal, 0, sizeof cal);
    cal.has_mag = true;
    cal.mag_soft_iron[0][0] = 1; cal.mag_soft_iron[0][1] = 1;
    cal.mag_soft_iron[1][1] = 1;
    cal.mag_soft_iron[2][2] = 1;
    mag_sample_t m2 = {0};
    m2.field[0] = 3; m2.field[1] = 4; m2.field[2] = 5;
    apply_mag_cal(&cal, &m2);
    EXPECT_NEAR(m2.field[0], 3 + 4, 1e-5, "cross term mixes x and y");
    EXPECT_NEAR(m2.field[1], 4,     1e-5, "row1 identity");
    EXPECT_NEAR(m2.field[2], 5,     1e-5, "row2 identity");

    end(fb);
}

int main(void)
{
    puts("=== imud imu_math tests ===");

    test_int_fallback_ms();
    test_mag_stall_ms();
    test_cal_is_applied_before_mount_rotation();
    test_nearest_odr();
    test_snap_odr_up();
    test_odr_actual_resolver();
    test_timestamp_basic();
    test_timestamp_wraparound();
    test_timestamp_no_hw_timer();
    test_anchor_gen_increments();
    test_anchor_pairs_the_sample_instant();
    test_samples_older_than_the_anchor();
    test_tick_measured_from_anchors();
    test_no_sawtooth_within_epoch();
    test_dt_between_samples_is_true();
    test_tick_measurement_rejects_nonsense();
    test_tick_estimate_is_filtered();
    test_lat_buckets_and_percentiles();
    test_lat_reset_keeps_all_time_max();
    test_lat_step_windows_are_independent();
    test_lat_step_clamps_rather_than_wraps();
    test_lat_step_ends_are_on_independent_clocks();
    test_ovf_latch();
    test_anchor_interval_bounds();
    test_ts_ns();
    test_mount_rotation();
    test_apply_imu_cal();
    test_apply_mag_cal();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
