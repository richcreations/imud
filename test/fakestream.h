/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fakestream.h — stand in for imud's [stream] socket, from a thread
 *
 * The bridge daemons are stream consumers: they connect to an AF_UNIX socket,
 * read 276-byte packets, and emit. Everything they emit is already unit-tested
 * (sk_delta, influx_line, mavlink_encode, prom_metrics, mqtt_publish), and
 * everything they use to get there is too (bridge.c, netserv.c, libimud) — what
 * was never tested is the wiring, because it only exists inside main().
 *
 * So: serve the real wire format, built by the daemon's own packet_build(), and
 * let a test call the real main() against it. test_libimud.c already listens on
 * a local AF_UNIX socket inline; this generalises that into something several
 * suites can share, the same way test/fdsweep.h is shared.
 *
 * Header-only and static: include it, link src/packet.c, done. Portable — no
 * Linux-only calls, so the bridge e2e suites still build on the macOS dev box.
 *
 * The knobs exist because the interesting bridge behaviour is failure
 * behaviour: drop_after makes the daemon look like it restarted (reconnect
 * policy), truncate_after sends a short frame (validation, not emission).
 */

#ifndef IMUD_TEST_FAKESTREAM_H
#define IMUD_TEST_FAKESTREAM_H

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "types.h"
#include "packet.h"

typedef struct {
    char        path[104];
    int         listen_fd;
    pthread_t   tid;
    bool        running;

    int         period_us;      /* pacing between packets */

    atomic_int  stop;
    atomic_int  served;         /* packets written, all connections */
    atomic_int  accepted;       /* connections accepted */
    atomic_int  drop_after;     /* close the client after N packets; -1 = never */
    atomic_int  truncate_after; /* send a short frame as packet N; -1 = never */
    atomic_uint seq;            /* imu_seq of the next packet */
} fakestream_t;

/* One packet with known, checkable values, from the daemon's real encoder.
 * heading_deg is the field the tests key on: it survives every bridge's
 * encoding, so "what came out" can be matched back to "what went in". */
static inline imu_packet_t fs_packet(uint32_t seq, float heading_deg)
{
    fused_state_t s;
    memset(&s, 0, sizeof s);
    s.q[0]            = 1.0f;
    s.heading_deg     = heading_deg;
    s.declination_deg = 13.2f;
    s.heave_m         = 0.42f;
    s.heave_rate      = 0.25f;
    s.quiescence      = 0.01f;
    /* FLAG_MAG_VALID is part of the baseline because the bridges withhold
     * their heading outputs without it, and the e2e cases key on the heading
     * path to prove the packet reached the encoder and the encoder reached
     * the socket.  Withholding is covered by each bridge's own unit tests. */
    s.flags           = FLAG_MAG_VALID | FLAG_DECLINATION_VALID | FLAG_HEAVE_VALID;
    s.imu_seq         = seq;
    s.ts_wall_ns      = 1620307999123000000ULL;

    mag_sample_t m;
    memset(&m, 0, sizeof m);
    imu_sample_t i;
    memset(&i, 0, sizeof i);
    i.temp_c   = 31.4f;
    i.accel[2] = -9.81f;

    imu_packet_t pkt;
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    return pkt;
}

static inline void *fs_thread(void *arg)
{
    fakestream_t *fs = (fakestream_t *)arg;

    while (!atomic_load(&fs->stop)) {
        /* Non-blocking accept loop rather than a blocking accept(): stop must
         * be observed even when no client ever connects, or fs_stop() hangs. */
        int c = accept(fs->listen_fd, NULL, NULL);
        if (c < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                struct timespec t = { 0, 2 * 1000 * 1000 };
                nanosleep(&t, NULL);
                continue;
            }
            break;
        }
        atomic_fetch_add(&fs->accepted, 1);

        int on_this_conn = 0;
        while (!atomic_load(&fs->stop)) {
            int drop = atomic_load(&fs->drop_after);
            if (drop >= 0 && on_this_conn >= drop) break;   /* look like a restart */

            imu_packet_t pkt = fs_packet(atomic_fetch_add(&fs->seq, 1), 90.0f);
            uint8_t wire[IMUD_PACKET_BYTES];
            packet_encode(wire, &pkt);

            size_t n = sizeof wire;
            int trunc = atomic_load(&fs->truncate_after);
            if (trunc >= 0 && on_this_conn == trunc) n = sizeof wire / 2;

            ssize_t w = send(c, wire, n, 0);
            if (w < 0) break;                                /* peer went away */
            on_this_conn++;
            atomic_fetch_add(&fs->served, 1);

            struct timespec t = { 0, (long)fs->period_us * 1000L };
            nanosleep(&t, NULL);
        }
        close(c);
    }
    return NULL;
}

/* Bind `path`, start serving. 0 on success. rate_hz paces the packets. */
static inline int fs_start(fakestream_t *fs, const char *path, int rate_hz)
{
    memset(fs, 0, sizeof *fs);
    snprintf(fs->path, sizeof fs->path, "%s", path);
    fs->period_us = rate_hz > 0 ? 1000000 / rate_hz : 10000;
    atomic_init(&fs->stop, 0);
    atomic_init(&fs->served, 0);
    atomic_init(&fs->accepted, 0);
    atomic_init(&fs->drop_after, -1);
    atomic_init(&fs->truncate_after, -1);
    atomic_init(&fs->seq, 1);

    unlink(path);
    fs->listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fs->listen_fd < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path);
    if (bind(fs->listen_fd, (struct sockaddr *)&addr, sizeof addr) < 0 ||
        listen(fs->listen_fd, 4) < 0) {
        close(fs->listen_fd);
        fs->listen_fd = -1;
        return -1;
    }

    /* O_NONBLOCK on the listener is what makes the stop flag reachable. */
    int fl = fcntl(fs->listen_fd, F_GETFL, 0);
    fcntl(fs->listen_fd, F_SETFL, fl | O_NONBLOCK);

    if (pthread_create(&fs->tid, NULL, fs_thread, fs) != 0) {
        close(fs->listen_fd);
        fs->listen_fd = -1;
        return -1;
    }
    fs->running = true;
    return 0;
}

static inline void fs_stop(fakestream_t *fs)
{
    if (!fs->running) return;
    atomic_store(&fs->stop, 1);
    pthread_join(fs->tid, NULL);
    if (fs->listen_fd >= 0) close(fs->listen_fd);
    unlink(fs->path);
    fs->running = false;
}

/* Block until at least `n` packets have been written, or `timeout_ms` passes.
 * Returns true if the count was reached — never sleeps a fixed duration, so a
 * slow machine does not turn into a flaky test. */
static inline bool fs_wait_served(fakestream_t *fs, int n, int timeout_ms)
{
    for (int waited = 0; waited < timeout_ms; waited += 5) {
        if (atomic_load(&fs->served) >= n) return true;
        struct timespec t = { 0, 5 * 1000 * 1000 };
        nanosleep(&t, NULL);
    }
    return atomic_load(&fs->served) >= n;
}

#endif /* IMUD_TEST_FAKESTREAM_H */
