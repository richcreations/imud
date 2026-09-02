/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ring.c — circular ring buffer implementation for imu_sample_t / mag_sample_t
 *
 * Both rings use a mutex + condition variable.  The IMU ring has a blocking
 * pop (with 100 ms timeout + stop flag) for the fusion thread.  The mag ring
 * uses a non-blocking try-pop because the fusion thread drains it between
 * predict steps.
 *
 * Overflow policy: on full ring, push overwrites the oldest entry (head
 * advances past tail).  The reader is never blocked or signalled — continuous
 * mode is assumed.
 */

#include <time.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include "host_time.h"
#include "ring.h"

/* ── IMU ring ────────────────────────────────────────────────────────────── */

void imu_ring_init(imu_ring_t *r)
{
    memset(r, 0, sizeof(*r));
    pthread_mutex_init(&r->lock, NULL);
    /* Monotonic where the host has clock selection, libc's default where it
     * does not; imu_ring_pop's deadline comes from the matching call.  See
     * include/host_time.h on why the two are a pair. */
    host_cond_init_monotonic(&r->ready);
}

int imu_ring_push(imu_ring_t *r, const imu_sample_t *s, int n)
{
    int dropped = 0;
    pthread_mutex_lock(&r->lock);
    for (int i = 0; i < n; i++) {
        if (r->count == IMU_RING_LEN) {
            /* Full — overwrite oldest entry. */
            r->tail = (r->tail + 1) & (IMU_RING_LEN - 1);
            dropped++;
        } else {
            r->count++;
        }
        r->buf[r->head] = s[i];
        r->head = (r->head + 1) & (IMU_RING_LEN - 1);
    }
    pthread_mutex_unlock(&r->lock);
    if (n > 0) pthread_cond_signal(&r->ready);
    return dropped;
}

int imu_ring_pop(imu_ring_t *r, imu_sample_t *out, _Atomic int *stop)
{
    struct timespec abs;
    host_cond_deadline(&abs, 100);

    pthread_mutex_lock(&r->lock);
    while (r->count == 0 && !*stop) {
        if (host_cond_timedwait(&r->ready, &r->lock, &abs) == ETIMEDOUT) {
            pthread_mutex_unlock(&r->lock);
            return -1;
        }
    }
    if (r->count == 0) { pthread_mutex_unlock(&r->lock); return -1; }
    *out = r->buf[r->tail];
    r->tail = (r->tail + 1) & (IMU_RING_LEN - 1);
    r->count--;
    pthread_mutex_unlock(&r->lock);
    return 0;
}

int imu_ring_count(imu_ring_t *r)
{
    pthread_mutex_lock(&r->lock);
    int n = (int)r->count;
    pthread_mutex_unlock(&r->lock);
    return n;
}

/* ── Mag ring ────────────────────────────────────────────────────────────── */

void mag_ring_init(mag_ring_t *r)
{
    memset(r, 0, sizeof(*r));
    pthread_mutex_init(&r->lock, NULL);
    pthread_cond_init(&r->ready, NULL);
}

void mag_ring_push(mag_ring_t *r, const mag_sample_t *s)
{
    pthread_mutex_lock(&r->lock);
    if (r->count == MAG_RING_LEN) {
        r->tail = (r->tail + 1) & (MAG_RING_LEN - 1);
    } else {
        r->count++;
    }
    r->buf[r->head] = *s;
    r->head = (r->head + 1) & (MAG_RING_LEN - 1);
    pthread_mutex_unlock(&r->lock);
}

int mag_ring_try_pop(mag_ring_t *r, mag_sample_t *out)
{
    pthread_mutex_lock(&r->lock);
    if (r->count == 0) { pthread_mutex_unlock(&r->lock); return -1; }
    *out = r->buf[r->tail];
    r->tail = (r->tail + 1) & (MAG_RING_LEN - 1);
    r->count--;
    pthread_mutex_unlock(&r->lock);
    return 0;
}
