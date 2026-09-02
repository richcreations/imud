/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * host_time_posix.c — include/host_time.h with clock selection but no TAI.
 *
 * The middle rung: FreeBSD, NetBSD, OpenBSD, illumos.  Every one has
 * _POSIX_CLOCK_SELECTION, so the sleep and the cond are word for word
 * src/host_time_linux.c's; none has CLOCK_TAI or adjtimex(2).
 *
 * The duplicated sleep and cond are the price of keeping a backend free of
 * #ifdefs, and the same price src/bus_null.c pays to repeat open() and
 * close() from src/bus_linux.c.  A shared file with two conditionals in it
 * would be shorter and would put the host question back above the seam.
 *
 * FreeBSD does have ntp_adjtime(2) and a struct timex, and NetBSD has both
 * too, so host_tai_offset() is implementable there -- it is ENOSYS here
 * because it is unwritten and untested, not because it is impossible.  A port
 * that wants it should split this rung rather than add an #ifdef.
 */

#include <errno.h>

#include "host_time.h"

void host_monotonic_now(struct timespec *out)
{
    clock_gettime(CLOCK_MONOTONIC, out);
}

int host_sleep_until(const struct timespec *deadline)
{
    /* Absolute deadline, so a signal costs a retry and not a tick; EINTR is
     * the only return worth looping on. */
    for (;;) {
        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline, NULL);
        if (rc == 0) return 0;
        if (rc != EINTR) { errno = rc; return -1; }
    }
}

int host_cond_init_monotonic(pthread_cond_t *c)
{
    pthread_condattr_t attr;
    int rc = pthread_condattr_init(&attr);
    if (rc != 0) { errno = rc; return -1; }

    rc = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    if (rc == 0) rc = pthread_cond_init(c, &attr);
    pthread_condattr_destroy(&attr);

    if (rc != 0) { errno = rc; return -1; }
    return 0;
}

void host_cond_deadline(struct timespec *out, long ms)
{
    clock_gettime(CLOCK_MONOTONIC, out);
    out->tv_sec  += ms / 1000L;
    out->tv_nsec += (ms % 1000L) * 1000000L;
    if (out->tv_nsec >= 1000000000L) { out->tv_sec++; out->tv_nsec -= 1000000000L; }
}

int host_cond_timedwait(pthread_cond_t *c, pthread_mutex_t *m,
                        const struct timespec *deadline)
{
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
    /* Fill it anyway, per the contract: the caller gets UTC and a -1 saying
     * so, which is what src/imu.c carried as a bare #define before this seam
     * existed -- silently, which was the defect. */
    clock_gettime(CLOCK_REALTIME, ts);
    errno = ENOSYS;
    return -1;
}
