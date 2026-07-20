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

    anchor_update(&a, /*chip*/ 1000, /*wall*/ 5000, /*tai*/ 6000);

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
    anchor_update(&a, 0xFFFFFFF0u, /*wall*/ 100000, /*tai*/ 200000);

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
    anchor_update(&a, 0, /*wall*/ 777, /*tai*/ 888);

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
    anchor_update(&a, 10, 1, 1);
    chip_to_wall(&a, 10, 25000, NULL, NULL, &gen);
    EXPECT(gen == 1, "gen == 1 after first anchor");

    anchor_update(&a, 20, 2, 2);
    chip_to_wall(&a, 20, 25000, NULL, NULL, &gen);
    EXPECT(gen == 2, "gen == 2 after re-anchor");

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
    test_ts_ns();
    test_mount_rotation();
    test_apply_imu_cal();
    test_apply_mag_cal();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
