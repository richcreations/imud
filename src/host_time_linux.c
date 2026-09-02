/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * host_time_linux.c — include/host_time.h on Linux.
 *
 * The top rung: clock selection plus a TAI clock.  src/host_time_posix.c is
 * this file without the last two functions, and src/host_time_fallback.c is
 * that without clock selection either.
 */

/* clock_nanosleep and CLOCK_TAI are behind _GNU_SOURCE on glibc.  The Makefile
 * passes it too; guard so a standalone compile still works. */
#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <errno.h>
#include <string.h>
#include <sys/timex.h>

#include "host_time.h"

void host_monotonic_now(struct timespec *out)
{
    clock_gettime(CLOCK_MONOTONIC, out);
}

int host_sleep_until(const struct timespec *deadline)
{
    /* TIMER_ABSTIME is what makes this drift-free: the kernel compares against
     * the deadline itself, so a signal costs a retry and not a tick.  EINTR is
     * the only return worth looping on -- every other error is permanent. */
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
    /*
     * modes = 0 is a read, but adjtimex(2) is in systemd's @clock set and so in
     * @privileged, and ProtectClock= blocks even this.  imud.service therefore
     * sets no ProtectClock= and re-allows adjtimex after its ~@privileged line.
     *
     * Note what this compiles to, because it is not one syscall: glibc issues
     * adjtimex on x86-64, clock_adjtime on arm64 (which has no adjtimex at
     * all), and clock_adjtime64 on armhf.  A SystemCallFilter= naming only
     * "adjtimex" therefore allows nothing on ARM and the process dies of
     * SIGSYS here — which is what shipped in the 1.9.0 RC.  See the comment on
     * SystemCallFilter= in etc/imud.service.in.
     */
    struct timex tx;
    memset(&tx, 0, sizeof(tx));
    if (adjtimex(&tx) < 0) return -1;
    *secs = tx.tai;
    return 0;
}

int host_clock_tai(struct timespec *ts)
{
    return clock_gettime(CLOCK_TAI, ts);
}
