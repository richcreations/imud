/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_ring.c — unit tests for imu_ring_t and mag_ring_t (src/ring.c)
 *
 * Tests are single-threaded (no concurrent push/pop) so they exercise
 * the data-structure logic without needing real hardware or timing.
 * The blocking imu_ring_pop is tested by pre-seeding data (non-blocking
 * path) and by setting stop=1 (immediate return path).
 *
 * Properties verified:
 *   IMU ring
 *     - FIFO ordering over a simple push/pop sequence
 *     - Count stays accurate
 *     - Overflow: oldest dropped, newest retained, drop count returned
 *     - Overflow at exactly capacity boundary
 *     - Circular wraparound (head/tail cross the array boundary)
 *     - pop from empty (stop=1) returns -1 immediately
 *     - pop returns -1 when stop is set even if data arrives
 *
 *   Mag ring
 *     - FIFO ordering
 *     - try_pop from empty returns -1
 *     - Overflow: oldest dropped, newest retained
 *     - Push exactly to capacity then drain
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "ring.h"
#include "types.h"

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/* Build an imu_sample_t whose identity is encoded in its seq field. */
static imu_sample_t make_imu(uint32_t seq)
{
    imu_sample_t s;
    memset(&s, 0, sizeof(s));
    s.seq = seq;
    return s;
}

/* Build a mag_sample_t whose identity is encoded in field[0]. */
static mag_sample_t make_mag(float id)
{
    mag_sample_t m;
    memset(&m, 0, sizeof(m));
    m.field[0] = id;
    m.valid = true;
    return m;
}

/* ── IMU ring tests ──────────────────────────────────────────────────────── */

static void test_imu_ring_fifo_order(void)
{
    begin("test_imu_ring_fifo_order");
    int fb = g_fail;

    imu_ring_t r;
    imu_ring_init(&r);

    imu_sample_t samples[3] = { make_imu(10), make_imu(20), make_imu(30) };
    imu_ring_push(&r, samples, 3);

    _Atomic int stop = 0;
    imu_sample_t out;
    EXPECT(imu_ring_pop(&r, &out, &stop) == 0 && out.seq == 10, "pop 1 → seq 10");
    EXPECT(imu_ring_pop(&r, &out, &stop) == 0 && out.seq == 20, "pop 2 → seq 20");
    EXPECT(imu_ring_pop(&r, &out, &stop) == 0 && out.seq == 30, "pop 3 → seq 30");
    end(fb);
}

static void test_imu_ring_count_accurate(void)
{
    begin("test_imu_ring_count_accurate");
    int fb = g_fail;

    imu_ring_t r;
    imu_ring_init(&r);
    EXPECT(r.count == 0, "initially empty");

    imu_sample_t s = make_imu(1);
    imu_ring_push(&r, &s, 1);
    EXPECT(r.count == 1, "count == 1 after one push");

    s = make_imu(2);
    imu_ring_push(&r, &s, 1);
    EXPECT(r.count == 2, "count == 2 after second push");

    _Atomic int stop = 0;
    imu_sample_t out;
    imu_ring_pop(&r, &out, &stop);
    EXPECT(r.count == 1, "count == 1 after one pop");

    imu_ring_pop(&r, &out, &stop);
    EXPECT(r.count == 0, "count == 0 after draining");
    end(fb);
}

static void test_imu_ring_pop_empty_with_stop(void)
{
    begin("test_imu_ring_pop_empty_with_stop");
    int fb = g_fail;

    imu_ring_t r;
    imu_ring_init(&r);

    _Atomic int stop = 1;  /* already stopped — should return immediately */
    imu_sample_t out;
    EXPECT(imu_ring_pop(&r, &out, &stop) == -1, "pop from empty with stop=1 returns -1");
    end(fb);
}

static void test_imu_ring_overflow_drops_oldest(void)
{
    begin("test_imu_ring_overflow_drops_oldest");
    int fb = g_fail;

    imu_ring_t r;
    imu_ring_init(&r);

    /* Fill to capacity */
    for (uint32_t i = 0; i < IMU_RING_LEN; i++) {
        imu_sample_t s = make_imu(i);
        imu_ring_push(&r, &s, 1);
    }
    EXPECT(r.count == IMU_RING_LEN, "ring full at IMU_RING_LEN");

    /* Push 3 more — drops seq 0, 1, 2; newest are IMU_RING_LEN..IMU_RING_LEN+2 */
    imu_sample_t extras[3] = {
        make_imu(IMU_RING_LEN),
        make_imu(IMU_RING_LEN + 1),
        make_imu(IMU_RING_LEN + 2),
    };
    int dropped = imu_ring_push(&r, extras, 3);
    EXPECT(dropped == 3, "3 samples dropped on overflow");
    EXPECT(r.count == IMU_RING_LEN, "count stays at IMU_RING_LEN after overflow");

    /* Oldest remaining should be seq 3 */
    _Atomic int stop = 0;
    imu_sample_t out;
    imu_ring_pop(&r, &out, &stop);
    EXPECT(out.seq == 3, "oldest remaining is seq 3 (0,1,2 were dropped)");
    end(fb);
}

static void test_imu_ring_overflow_returns_drop_count(void)
{
    begin("test_imu_ring_overflow_returns_drop_count");
    int fb = g_fail;

    imu_ring_t r;
    imu_ring_init(&r);

    /* Fill to capacity */
    for (uint32_t i = 0; i < IMU_RING_LEN; i++) {
        imu_sample_t s = make_imu(i);
        imu_ring_push(&r, &s, 1);
    }

    imu_sample_t s = make_imu(9999);
    int dropped = imu_ring_push(&r, &s, 1);
    EXPECT(dropped == 1, "exactly 1 drop when ring is full and 1 pushed");

    /* Push 0 samples — no drop */
    dropped = imu_ring_push(&r, &s, 0);
    EXPECT(dropped == 0, "0 drops when pushing 0 samples");
    end(fb);
}

static void test_imu_ring_exactly_full_no_overflow(void)
{
    begin("test_imu_ring_exactly_full_no_overflow");
    int fb = g_fail;

    imu_ring_t r;
    imu_ring_init(&r);

    /* Push exactly IMU_RING_LEN - 1, then one more — should not overflow yet */
    for (uint32_t i = 0; i < (uint32_t)(IMU_RING_LEN - 1); i++) {
        imu_sample_t s = make_imu(i);
        imu_ring_push(&r, &s, 1);
    }
    imu_sample_t last = make_imu(IMU_RING_LEN - 1);
    int dropped = imu_ring_push(&r, &last, 1);
    EXPECT(dropped == 0, "no drop when filling to exactly IMU_RING_LEN");
    EXPECT(r.count == IMU_RING_LEN, "count == IMU_RING_LEN when exactly full");
    end(fb);
}

/*
 * Verify FIFO order is maintained when head and tail wrap past the array end.
 * Strategy: fill half the ring, drain it (tail advances), fill again past the
 * wrap point, drain and check order.
 */
static void test_imu_ring_circular_wraparound(void)
{
    begin("test_imu_ring_circular_wraparound");
    int fb = g_fail;

    imu_ring_t r;
    imu_ring_init(&r);
    _Atomic int stop = 0;
    imu_sample_t out;

    /* Advance head and tail into the middle of the array */
    for (uint32_t i = 0; i < IMU_RING_LEN / 2; i++) {
        imu_sample_t s = make_imu(i);
        imu_ring_push(&r, &s, 1);
    }
    for (unsigned i = 0; i < IMU_RING_LEN / 2; i++)
        imu_ring_pop(&r, &out, &stop);

    EXPECT(r.count == 0, "ring empty after draining first half");
    /* head and tail are now at IMU_RING_LEN/2 */

    /* Push enough to wrap head past the array end */
    uint32_t base = 1000;
    for (uint32_t i = 0; i < (uint32_t)(IMU_RING_LEN * 3 / 4); i++) {
        imu_sample_t s = make_imu(base + i);
        imu_ring_push(&r, &s, 1);
    }

    /* Drain and verify FIFO order */
    bool order_ok = true;
    for (uint32_t i = 0; i < (uint32_t)(IMU_RING_LEN * 3 / 4); i++) {
        if (imu_ring_pop(&r, &out, &stop) != 0 || out.seq != base + i)
            order_ok = false;
    }
    EXPECT(order_ok, "FIFO order preserved across array wraparound");
    EXPECT(r.count == 0, "ring empty after wraparound drain");
    end(fb);
}

/* ── Mag ring tests ──────────────────────────────────────────────────────── */

static void test_mag_ring_fifo_order(void)
{
    begin("test_mag_ring_fifo_order");
    int fb = g_fail;

    mag_ring_t r;
    mag_ring_init(&r);

    for (int i = 0; i < 5; i++) {
        mag_sample_t m = make_mag((float)i);
        mag_ring_push(&r, &m);
    }

    mag_sample_t out;
    for (int i = 0; i < 5; i++) {
        EXPECT(mag_ring_try_pop(&r, &out) == 0, "try_pop succeeds");
        EXPECT(out.field[0] == (float)i, "mag ring FIFO order");
    }
    end(fb);
}

static void test_mag_ring_try_pop_empty(void)
{
    begin("test_mag_ring_try_pop_empty");
    int fb = g_fail;

    mag_ring_t r;
    mag_ring_init(&r);

    mag_sample_t out;
    EXPECT(mag_ring_try_pop(&r, &out) == -1, "try_pop from empty returns -1");
    end(fb);
}

static void test_mag_ring_overflow_drops_oldest(void)
{
    begin("test_mag_ring_overflow_drops_oldest");
    int fb = g_fail;

    mag_ring_t r;
    mag_ring_init(&r);

    /* Fill to capacity */
    for (int i = 0; i < MAG_RING_LEN; i++) {
        mag_sample_t m = make_mag((float)i);
        mag_ring_push(&r, &m);
    }
    EXPECT(r.count == MAG_RING_LEN, "mag ring full at MAG_RING_LEN");

    /* Push 2 more — oldest (0, 1) dropped; newest are MAG_RING_LEN and +1 */
    mag_sample_t extra0 = make_mag((float)MAG_RING_LEN);
    mag_sample_t extra1 = make_mag((float)(MAG_RING_LEN + 1));
    mag_ring_push(&r, &extra0);
    mag_ring_push(&r, &extra1);

    EXPECT(r.count == MAG_RING_LEN, "count stays at MAG_RING_LEN after overflow");

    /* Oldest remaining should be id=2 */
    mag_sample_t out;
    mag_ring_try_pop(&r, &out);
    EXPECT(out.field[0] == 2.0f, "oldest remaining is id=2 (0,1 were dropped)");
    end(fb);
}

static void test_mag_ring_fill_and_drain(void)
{
    begin("test_mag_ring_fill_and_drain");
    int fb = g_fail;

    mag_ring_t r;
    mag_ring_init(&r);

    for (int i = 0; i < MAG_RING_LEN; i++) {
        mag_sample_t m = make_mag((float)i);
        mag_ring_push(&r, &m);
    }
    EXPECT(r.count == MAG_RING_LEN, "exactly MAG_RING_LEN samples present");

    mag_sample_t out;
    bool all_ok = true;
    for (int i = 0; i < MAG_RING_LEN; i++) {
        if (mag_ring_try_pop(&r, &out) != 0 || out.field[0] != (float)i)
            all_ok = false;
    }
    EXPECT(all_ok, "all MAG_RING_LEN samples drain in order");
    EXPECT(mag_ring_try_pop(&r, &out) == -1, "ring empty after drain");
    end(fb);
}

static void test_mag_ring_circular_wraparound(void)
{
    begin("test_mag_ring_circular_wraparound");
    int fb = g_fail;

    mag_ring_t r;
    mag_ring_init(&r);
    mag_sample_t out;

    /* Advance head/tail into the middle */
    for (int i = 0; i < MAG_RING_LEN / 2; i++) {
        mag_sample_t m = make_mag((float)i);
        mag_ring_push(&r, &m);
    }
    for (int i = 0; i < MAG_RING_LEN / 2; i++)
        mag_ring_try_pop(&r, &out);

    /* Push past the wrap point */
    float base = 100.0f;
    for (int i = 0; i < MAG_RING_LEN - 2; i++) {
        mag_sample_t m = make_mag(base + i);
        mag_ring_push(&r, &m);
    }

    bool order_ok = true;
    for (int i = 0; i < MAG_RING_LEN - 2; i++) {
        if (mag_ring_try_pop(&r, &out) != 0 || out.field[0] != base + i)
            order_ok = false;
    }
    EXPECT(order_ok, "mag ring FIFO order preserved across wraparound");
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud ring buffer tests ===");

    test_imu_ring_fifo_order();
    test_imu_ring_count_accurate();
    test_imu_ring_pop_empty_with_stop();
    test_imu_ring_overflow_drops_oldest();
    test_imu_ring_overflow_returns_drop_count();
    test_imu_ring_exactly_full_no_overflow();
    test_imu_ring_circular_wraparound();

    test_mag_ring_fifo_order();
    test_mag_ring_try_pop_empty();
    test_mag_ring_overflow_drops_oldest();
    test_mag_ring_fill_and_drain();
    test_mag_ring_circular_wraparound();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
