/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * host_time_fallback.c — include/host_time.h with no clock selection.
 *
 * The bottom rung, and today that means macOS: no clock_nanosleep(2) and no
 * pthread_condattr_setclock(3), because both are _POSIX_CLOCK_SELECTION and
 * macOS does not implement the option.  This is what lets imud link there at
 * all -- src/ring.c had no other way to create its condition variable.
 *
 * Both substitutes are worse than the real thing, in ways worth knowing:
 *
 *   The sleep is a loop, not a single call.  nanosleep(2) takes a DURATION, so
 *   the deadline has to be turned into one against the clock as it is now --
 *   and a signal ends that sleep early with the deadline still ahead.  Sleeping
 *   once and returning is what the old src/output.c stub did, and it is why an
 *   output thread there emitted a tick per signal.  Recomputing the remainder
 *   and sleeping again converges instead: the error per iteration does not
 *   accumulate into the schedule, because every iteration measures against the
 *   deadline rather than against the last wake-up.
 *
 *   The cond runs on the default clock, which is CLOCK_REALTIME.  So a step of
 *   the wall clock -- ntpd, an operator, a VM resuming -- can stretch or cut
 *   one imu_ring_pop() timeout.  That timeout is 100 ms and its expiry is a
 *   normal no-data return, so the cost is one late or early poll, not a
 *   correctness failure.  host_cond_deadline() below uses the same clock, and
 *   that agreement is the part that must hold.
 */

#include <errno.h>

#include "host_time.h"

void host_monotonic_now(struct timespec *out)
{
    /* A macOS rung takes this from mach_absolute_time() or CLOCK_UPTIME_RAW,
     * to match mach_wait_until()'s timebase.  Plain CLOCK_MONOTONIC here
     * because nanosleep() below has no timebase of its own to match. */
    clock_gettime(CLOCK_MONOTONIC, out);
}

int host_sleep_until(const struct timespec *deadline)
{
    for (;;) {
        struct timespec now, d;
        host_monotonic_now(&now);

        d.tv_sec  = deadline->tv_sec  - now.tv_sec;
        d.tv_nsec = deadline->tv_nsec - now.tv_nsec;
        if (d.tv_nsec < 0) { d.tv_sec--; d.tv_nsec += 1000000000L; }

        /* Reached, or already past -- the caller's deadline may be behind it
         * when a tick overran, which is not an error. */
        if (d.tv_sec < 0) return 0;

        /* Re-check the deadline rather than trusting this return: that is what
         * makes the loop converge on it from either side. */
        if (nanosleep(&d, NULL) < 0 && errno != EINTR) return -1;
    }
}

int host_cond_init_monotonic(pthread_cond_t *c)
{
    /* No clock selection, so the cond keeps libc's default clock and
     * host_cond_deadline() below matches it. */
    int rc = pthread_cond_init(c, NULL);
    if (rc != 0) { errno = rc; return -1; }
    return 0;
}

void host_cond_deadline(struct timespec *out, long ms)
{
    clock_gettime(CLOCK_REALTIME, out);
    out->tv_sec  += ms / 1000L;
    out->tv_nsec += (ms % 1000L) * 1000000L;
    if (out->tv_nsec >= 1000000000L) { out->tv_sec++; out->tv_nsec -= 1000000000L; }
}

int host_cond_timedwait(pthread_cond_t *c, pthread_mutex_t *m,
                        const struct timespec *deadline)
{
    /* Absolute, on the default clock host_cond_deadline() used.  A macOS rung
     * converts the deadline to a remainder here and calls
     * pthread_cond_timedwait_relative_np(), which a clock step cannot move. */
    return pthread_cond_timedwait(c, m, deadline);
}

int host_tai_offset(int *secs)
{
    (void)secs;
    errno = ENOSYS;
    return -1;
}

int host_clock_tai(struct timespec *ts)
{
    /* Fill it anyway, per the contract: UTC, and a -1 saying it is not TAI. */
    clock_gettime(CLOCK_REALTIME, ts);
    errno = ENOSYS;
    return -1;
}
