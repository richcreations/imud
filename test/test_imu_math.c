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
 * but were previously only compiled into the TSan concurrency test, which
 * never asserts their numeric output.  Pure and portable — builds and runs on
 * the macOS dev box too.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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

static void test_nearest_odr(void)
{
    begin("test_nearest_odr");
    int fb = g_fail;

    /* The ISM330DHCX table, as registered. */
    static const int ism[] = { 12, 26, 52, 104, 208, 416, 833, 1660, 0 };
    EXPECT(nearest_odr(ism, 208) == 208, "exact match");
    EXPECT(nearest_odr(ism, 60)  == 52,  "round to nearer (52 vs 104)");
    EXPECT(nearest_odr(ism, 900) == 833, "round to nearer (833 vs 1660)");
    EXPECT(nearest_odr(ism, 1)   == 12,  "below min clamps to first");
    EXPECT(nearest_odr(ism, 5000) == 1660, "above max clamps to last");

    /* Exact ties resolve to the first (lower) entry: the update is strict
     * `d < best_diff`, so an equal distance does not displace the incumbent. */
    static const int tie[] = { 10, 20, 0 };
    EXPECT(nearest_odr(tie, 15) == 10, "tie resolves to lower");

    static const int one[] = { 100, 0 };
    EXPECT(nearest_odr(one, 5)    == 100, "single entry, low request");
    EXPECT(nearest_odr(one, 9999) == 100, "single entry, high request");

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

    test_nearest_odr();
    test_timestamp_basic();
    test_timestamp_wraparound();
    test_timestamp_no_hw_timer();
    test_anchor_gen_increments();
    test_tick_measured_from_anchors();
    test_no_sawtooth_within_epoch();
    test_dt_between_samples_is_true();
    test_tick_measurement_rejects_nonsense();
    test_tick_estimate_is_filtered();
    test_ts_ns();
    test_mount_rotation();
    test_apply_imu_cal();
    test_apply_mag_cal();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
