/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_host_time.c — the conformance suite for include/host_time.h.
 *
 * It asserts the CONTRACT, never one rung's implementation, so it holds for
 * any backend.  That is what lets a port check its own before it has anything
 * else running:
 *
 *     make test_host_time HOST_TIME_TEST_SRC=src/host_time_myos.c
 *
 * The default rung is src/host_time_fallback.c rather than whichever one the
 * tree links, on the same terms as test_bus_null: the fallback is the one with
 * substitutes in it — a looping nanosleep and a default-clock condvar — so it
 * is the rung whose contract is worth pinning, and pinning it here runs it on
 * every host, including the Linux ones where it is not what imud links.
 *
 * Two properties carry the suite, and both are ways a rung fails silently:
 *
 *   A tick must never fire EARLY.  The output threads pace on
 *   host_sleep_until(), and a sleep that returns on the first signal emits a
 *   packet per signal.  That is what the src/output.c stub this seam replaced
 *   did, and nothing downstream can see it — the rate is simply wrong.
 *
 *   The cond's clock and the deadline's clock must AGREE.  A rung that reads
 *   one and waits on the other is not slightly off: CLOCK_MONOTONIC counts
 *   from boot and CLOCK_REALTIME from 1970, so the wait either returns at once
 *   or blocks for five decades.  Both mistakes look like a hang or a spin, in
 *   a thread nobody is watching.
 *
 * Timing assertions here are one-sided wherever the machine can make them so.
 * "Not before the deadline" is a real invariant; "within N ms after it" is a
 * statement about scheduler luck, so the upper bounds are loose enough to
 * survive a loaded box and are there to catch a sleep that is wrong by a
 * factor, not one that is late by a slice.
 */

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "host_time.h"

static int g_fail;
static int g_checks;

#define EXPECT(c, msg) do {                                       \
        g_checks++;                                               \
        if (!(c)) { printf("  FAIL: %s\n", (msg)); g_fail++; }     \
    } while (0)

/* Elapsed ms between two timespecs. */
static double ms_between(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) * 1000.0
         + (double)(b->tv_nsec - a->tv_nsec) / 1000000.0;
}

static void ts_add_ms(struct timespec *t, long ms)
{
    t->tv_sec  += ms / 1000L;
    t->tv_nsec += (ms % 1000L) * 1000000L;
    if (t->tv_nsec >= 1000000000L) { t->tv_sec++; t->tv_nsec -= 1000000000L; }
}

/* ── Signal machinery ────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_sigs;

static void on_sig(int s) { (void)s; g_sigs++; }

/* Hammer `target` with SIGUSR1 every 5 ms until told to stop.  A thread rather
 * than setitimer so the interruptions keep coming while the main thread is
 * inside the sleep under test, which is the whole point. */
static pthread_t     g_hammer;
/* _Atomic, not volatile: a plain cross-thread flag is a C11 data race, and the
 * TSan job says so — volatile orders nothing between threads. Tree-wide rule. */
static _Atomic int   g_hammer_stop;
static pthread_t     g_hammer_target;

static void *hammer(void *arg)
{
    (void)arg;
    while (!g_hammer_stop) {
        pthread_kill(g_hammer_target, SIGUSR1);
        struct timespec d = { 0, 5 * 1000000L };
        nanosleep(&d, NULL);
    }
    return NULL;
}

static void hammer_start(void)
{
    g_sigs = 0;
    g_hammer_stop = 0;
    g_hammer_target = pthread_self();
    pthread_create(&g_hammer, NULL, hammer, NULL);
}

static void hammer_stop(void)
{
    g_hammer_stop = 1;
    pthread_join(g_hammer, NULL);
}

/* ── host_sleep_until ────────────────────────────────────────────────────── */

static void test_sleep_reaches_the_deadline(void)
{
    printf("test_sleep_reaches_the_deadline\n");

    struct timespec start, deadline, end;
    host_monotonic_now(&start);
    deadline = start;
    ts_add_ms(&deadline, 50);

    EXPECT(host_sleep_until(&deadline) == 0, "sleep to a future deadline returns 0");
    host_monotonic_now(&end);

    double el = ms_between(&start, &end);
    /* The invariant. A tick that fires early is a packet emitted early, and
     * the rate is silently wrong. 1 ms of slack for clock granularity. */
    EXPECT(el >= 49.0, "does not return before the deadline");
    EXPECT(el < 250.0,  "and returns near it, not a multiple of it");
}

static void test_past_deadline_returns_at_once(void)
{
    printf("test_past_deadline_returns_at_once\n");

    /* A tick that overran leaves the next deadline behind `now`. That is a
     * normal return, not an error, and it must not sleep — a rung that
     * mishandled the sign here would sleep for decades. */
    struct timespec start, deadline, end;
    host_monotonic_now(&start);
    deadline = start;
    deadline.tv_sec -= 5;

    EXPECT(host_sleep_until(&deadline) == 0, "past deadline returns 0");
    host_monotonic_now(&end);
    EXPECT(ms_between(&start, &end) < 50.0, "and does not sleep");
}

/*
 * The assertion the old src/output.c stub fails.
 *
 * That stub converted the deadline to a duration and called nanosleep once,
 * so the first signal ended it with most of the interval still to run. Here
 * a signal lands every 5 ms across a 200 ms sleep — around 40 of them — and
 * the sleep must still return only at the deadline.
 */
static void test_sleep_absorbs_signals(void)
{
    printf("test_sleep_absorbs_signals\n");

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sig;      /* no SA_RESTART: nanosleep must see EINTR */
    sigaction(SIGUSR1, &sa, NULL);

    struct timespec start, deadline, end;
    host_monotonic_now(&start);
    deadline = start;
    ts_add_ms(&deadline, 200);

    hammer_start();
    int rc = host_sleep_until(&deadline);
    hammer_stop();

    host_monotonic_now(&end);
    double el = ms_between(&start, &end);

    EXPECT(rc == 0, "returns 0 despite the signals");
    EXPECT(g_sigs > 5, "the test actually delivered signals");
    EXPECT(el >= 199.0, "does not return early on EINTR");
    EXPECT(el < 400.0,  "and does not overshoot wildly");
}

/*
 * Drift, which is the property a per-signal early return destroys quietly.
 *
 * 40 ticks of 5 ms, each deadline advanced from the PREVIOUS deadline rather
 * than from the wake-up — exactly what the output threads do. The schedule is
 * therefore fixed in advance, and total elapsed time must match it whatever
 * the per-tick jitter. Signals run throughout.
 */
static void test_ticks_do_not_drift(void)
{
    printf("test_ticks_do_not_drift\n");

    struct timespec start, next, end;
    host_monotonic_now(&start);
    next = start;

    hammer_start();
    int early = 0;
    for (int i = 0; i < 40; i++) {
        ts_add_ms(&next, 5);
        host_sleep_until(&next);

        struct timespec now;
        host_monotonic_now(&now);
        if (ms_between(&next, &now) < -1.0) early++;
    }
    hammer_stop();

    host_monotonic_now(&end);
    double el = ms_between(&start, &end);

    EXPECT(early == 0, "no tick fires before its own deadline");
    EXPECT(el >= 199.0, "40 x 5 ms takes at least its 200 ms");
    EXPECT(el < 400.0,  "and does not accumulate a tick's worth of drift");
}

/* ── The cond pair ───────────────────────────────────────────────────────── */

/*
 * The clock host_cond_deadline() reads and the clock the cond times out on
 * must be the same one. Nothing exposes either, so this measures: a 100 ms
 * deadline that nobody signals must time out in about 100 ms.
 *
 * Mixing the two clocks does not produce a slightly wrong wait. A monotonic
 * deadline against a realtime cond is decades in the past, so the wait returns
 * instantly; the other way round it is decades ahead. Both are caught here.
 */
static void test_cond_deadline_matches_cond_clock(void)
{
    printf("test_cond_deadline_matches_cond_clock\n");

    pthread_cond_t  c;
    pthread_mutex_t m;
    EXPECT(host_cond_init_monotonic(&c) == 0, "cond init succeeds");
    pthread_mutex_init(&m, NULL);

    struct timespec deadline, start, end;
    host_monotonic_now(&start);
    host_cond_deadline(&deadline, 100);

    /* Looped, not called once: pthread_cond_timedwait may wake spuriously, and
     * a bare `rc == ETIMEDOUT` would then fail on a correct rung. Nothing
     * signals this cond, so every 0 is spurious and the loop re-waits against
     * the SAME deadline — rebuilding it per iteration would restart the
     * timeout and hang. */
    int rc;
    pthread_mutex_lock(&m);
    do {
        rc = host_cond_timedwait(&c, &m, &deadline);
    } while (rc == 0);
    pthread_mutex_unlock(&m);

    host_monotonic_now(&end);
    double el = ms_between(&start, &end);

    EXPECT(rc == ETIMEDOUT, "an unsignalled wait times out");
    /* Returning at once is the monotonic-deadline-on-a-realtime-cond bug. */
    EXPECT(el >= 95.0,  "and waits, rather than expiring immediately");
    /* Not returning at all is the other direction; the suite would hang, so
     * this bound is what a merely-late rung trips instead. */
    EXPECT(el < 500.0,  "and expires near the deadline");

    pthread_cond_destroy(&c);
    pthread_mutex_destroy(&m);
}

/*
 * Signal `c` under `m`, having set the predicate `ready` first.
 *
 * All three parts matter. Signalling without the mutex held lets the signal
 * land BEFORE the waiter reaches its wait, where it is simply lost and the
 * waiter then sits until the deadline — 20 ms of head start is plenty on an
 * idle box and nothing at all on a loaded CI runner. The predicate is what
 * makes a signal that arrives early still count.
 */
struct sig_arg {
    pthread_cond_t  *c;
    pthread_mutex_t *m;
    int             *ready;
};

static void *signaller(void *arg)
{
    struct sig_arg *a = arg;
    struct timespec d = { 0, 20 * 1000000L };
    nanosleep(&d, NULL);

    pthread_mutex_lock(a->m);
    *a->ready = 1;
    pthread_cond_signal(a->c);
    pthread_mutex_unlock(a->m);
    return NULL;
}

static void test_cond_wakes_on_signal(void)
{
    printf("test_cond_wakes_on_signal\n");

    pthread_cond_t  c;
    pthread_mutex_t m;
    host_cond_init_monotonic(&c);
    pthread_mutex_init(&m, NULL);

    struct timespec deadline, start, end;
    host_monotonic_now(&start);
    host_cond_deadline(&deadline, 2000);

    int ready = 0;
    struct sig_arg a = { &c, &m, &ready };
    pthread_t th;
    pthread_create(&th, NULL, signaller, &a);

    /* Wait on the predicate, not on one return: a spurious wake would
     * otherwise read as the signal arriving, and a signal that beat us to the
     * wait would read as a timeout. */
    int rc = 0;
    pthread_mutex_lock(&m);
    while (!ready && rc == 0)
        rc = host_cond_timedwait(&c, &m, &deadline);
    pthread_mutex_unlock(&m);
    pthread_join(th, NULL);

    host_monotonic_now(&end);

    /* Signalled, not timed out, well inside a 2 s deadline. A rung whose
     * deadline landed in the past would report ETIMEDOUT here instead — which
     * test_cond_deadline_matches_cond_clock also catches; this one pins that a
     * real signal still gets through. */
    EXPECT(ready, "the signaller ran");
    EXPECT(rc == 0, "a signalled wait returns 0, not ETIMEDOUT");
    EXPECT(ms_between(&start, &end) < 1000.0, "and returns before the deadline");

    pthread_cond_destroy(&c);
    pthread_mutex_destroy(&m);
}

/* ── TAI ─────────────────────────────────────────────────────────────────── */

/*
 * Both TAI calls are allowed to fail, and a rung that cannot answer must fail
 * with ENOSYS specifically — src/main.c prints a different warning for it than
 * for a query that failed, because they are different faults: one is the port,
 * the other is the operator's clock daemon.
 *
 * host_clock_tai() must fill *ts either way. src/imu.c calls it on the hot
 * path and uses the result unconditionally, so a rung that left it untouched
 * would put an uninitialised timestamp on every sample.
 */
static void test_tai_contract(void)
{
    printf("test_tai_contract\n");

    int secs = -12345;
    errno = 0;
    if (host_tai_offset(&secs) < 0) {
        EXPECT(errno != 0, "a failed offset query leaves errno set");
    } else {
        EXPECT(secs >= 0 && secs < 1000, "a reported offset is plausible");
    }

    struct timespec ts;
    memset(&ts, 0, sizeof(ts));
    errno = 0;
    int rc = host_clock_tai(&ts);
    if (rc < 0) EXPECT(errno != 0, "a failed TAI read leaves errno set");

    /* Filled whether or not it succeeded — after 2024-01-01, which is the same
     * sanity bound src/main.c's clock health check uses on CLOCK_REALTIME. */
    EXPECT(ts.tv_sec > 1704067200LL, "*ts is filled even when TAI is absent");
}

int main(void)
{
    printf("=== test_host_time ===\n");

    test_sleep_reaches_the_deadline();
    test_past_deadline_returns_at_once();
    test_sleep_absorbs_signals();
    test_ticks_do_not_drift();
    test_cond_deadline_matches_cond_clock();
    test_cond_wakes_on_signal();
    test_tai_contract();

    printf("\n%d checks, %d failed\n", g_checks, g_fail);
    if (g_fail == 0) printf("ALL PASS (%d checks passed, 0 failed)\n", g_checks);
    return g_fail ? 1 : 0;
}
