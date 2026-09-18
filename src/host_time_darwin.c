/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * host_time_darwin.c — include/host_time.h on macOS.
 *
 * Not a rung below the other three: macOS has no _POSIX_CLOCK_SELECTION and a
 * TAI offset, where src/host_time_posix.c has the option and no offset.  It
 * answers both of the header's pairs with mach, and the offset with
 * ntp_gettime(2) -- the same David Mills interface Linux reads through
 * adjtimex(2), and the reason this file exists.
 *
 * What it buys over src/host_time_fallback.c is mechanism, not jitter.
 * Measured on a 2019 Intel Air against a 20 ms schedule, 100 ticks: this rung
 * lands +3.0 to +4.2 ms mean and never early, and the fallback's nanosleep
 * loop lands in the same place.  The ~10 ms ceiling both hit is macOS timer
 * coalescing, which QOS_CLASS_USER_INTERACTIVE does not move either.  What
 * changes is the three things below.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <mach/mach_time.h>
#include <sys/timex.h>

#include "host_time.h"

#define NS_PER_S 1000000000ULL

/* A mach tick is numer/denom nanoseconds: 1/1 on Intel, 125/3 on Apple silicon
 * (a 24 MHz counter).  Read once; it cannot change while the machine runs. */
static mach_timebase_info_data_t g_tb;
static pthread_once_t            g_tb_once = PTHREAD_ONCE_INIT;

static void tb_init(void)
{
    if (mach_timebase_info(&g_tb) != KERN_SUCCESS
        || g_tb.numer == 0 || g_tb.denom == 0) {
        g_tb.numer = 1;
        g_tb.denom = 1;
    }
}

static uint64_t ts_to_ns(const struct timespec *ts)
{
    return (uint64_t)ts->tv_sec * NS_PER_S + (uint64_t)ts->tv_nsec;
}

void host_monotonic_now(struct timespec *out)
{
    /* CLOCK_UPTIME_RAW is mach_absolute_time() scaled by the timebase, so it is
     * the clock host_sleep_until() below waits against -- both ends of the pair
     * from one source.  NOT CLOCK_MONOTONIC, which Darwin derives from the wall
     * clock minus boot time and which keeps counting while the machine is
     * asleep. */
    clock_gettime(CLOCK_UPTIME_RAW, out);
}

int host_sleep_until(const struct timespec *deadline)
{
    pthread_once(&g_tb_once, tb_init);

    /* Nanoseconds to ticks, rounded UP.  Rounding down would put the deadline
     * up to one tick early, and firing early is the one thing this contract
     * does not allow.  A deadline already past returns at once. */
    const uint64_t ns    = ts_to_ns(deadline);
    const uint64_t ticks = (ns * g_tb.denom + g_tb.numer - 1) / g_tb.numer;

    for (;;) {
        kern_return_t kr = mach_wait_until(ticks);
        if (kr == KERN_SUCCESS) return 0;

        /* KERN_ABORTED is this rung's EINTR -- thread_abort ended the wait with
         * the deadline still ahead -- so re-arm on the same absolute deadline.
         * No other mach code has an errno; EINVAL is the report, not a
         * translation. */
        if (kr != KERN_ABORTED) { errno = EINVAL; return -1; }
    }
}

int host_cond_init_monotonic(pthread_cond_t *c)
{
    /* No condattr clock to set, and none needed: host_cond_timedwait() below
     * makes the timeout relative, so the cond's own clock never applies. */
    int rc = pthread_cond_init(c, NULL);
    if (rc != 0) { errno = rc; return -1; }
    return 0;
}

void host_cond_deadline(struct timespec *out, long ms)
{
    host_monotonic_now(out);
    out->tv_sec  += ms / 1000L;
    out->tv_nsec += (ms % 1000L) * 1000000L;
    if (out->tv_nsec >= 1000000000L) { out->tv_sec++; out->tv_nsec -= 1000000000L; }
}

int host_cond_timedwait(pthread_cond_t *c, pthread_mutex_t *m,
                        const struct timespec *deadline)
{
    /* pthread_cond_timedwait() would read this deadline on CLOCK_REALTIME, so a
     * wall-clock step would stretch or cut the wait.  The _np variant takes a
     * remainder instead, recomputed here each time round the caller's predicate
     * loop, which a step cannot move. */
    struct timespec now, rel;
    host_monotonic_now(&now);

    rel.tv_sec  = deadline->tv_sec  - now.tv_sec;
    rel.tv_nsec = deadline->tv_nsec - now.tv_nsec;
    if (rel.tv_nsec < 0) { rel.tv_sec--; rel.tv_nsec += (long)NS_PER_S; }

    /* Past already: a zero remainder returns ETIMEDOUT, where a negative one
     * would be EINVAL. */
    if (rel.tv_sec < 0) { rel.tv_sec = 0; rel.tv_nsec = 0; }

    return pthread_cond_timedwait_relative_np(c, m, &rel);
}

int host_tai_offset(int *secs)
{
    struct ntptimeval ntv;
    memset(&ntv, 0, sizeof(ntv));

    /*
     * xnu returns ntv.time_state as the syscall's error code (bsd/kern/
     * kern_ntptime.c), so libc reports a clock that is merely unsynchronised as
     * -1 with errno TIME_ERROR.  The struct is already filled by then: EFAULT
     * is the only failure that leaves it untouched, and that cannot happen for
     * a stack copy.  So the offset is good whenever the pointer was.
     */
    errno = 0;
    if (ntp_gettime(&ntv) < 0 && errno == EFAULT) return -1;

    *secs = (int)ntv.tai;
    return 0;
}

int host_clock_tai(struct timespec *ts)
{
    /* Filled first, per the contract: the caller gets UTC whatever follows. */
    clock_gettime(CLOCK_REALTIME, ts);

    int secs = 0;
    if (host_tai_offset(&secs) < 0) return -1;

    /* Asked, and nothing has set it -- Apple's timed never calls MOD_TAI, so
     * this is the usual answer on macOS.  TAI-UTC has only ever grown, so a
     * negative offset is no answer either. */
    if (secs <= 0) { errno = ENODATA; return -1; }

    ts->tv_sec += secs;
    return 0;
}
