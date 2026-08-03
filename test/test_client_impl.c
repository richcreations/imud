/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_client_impl.c — thin wrappers over lib/imud_client.h for test_client.c
 *
 * Deliberately its own translation unit: imud_client.h is compiled here
 * exactly as a third-party consumer would use it — standalone, without the
 * daemon's types.h in scope. test_client.c builds packets with the daemon's
 * packet_build() and hands the raw bytes across this boundary, so any drift
 * between the two packet definitions (size, field offsets, CRC coverage,
 * magic/version) fails the test instead of shipping silently.
 */

#define IMUD_CLIENT_IMPLEMENTATION
#include <sys/time.h>
#include "../lib/imud_client.h"

bool client_packet_valid(const void *buf, size_t len)
{
    return imud_packet_valid(buf, len);
}

/* Field accessors read through the CLIENT struct definition, so they verify
 * the client's field offsets against bytes produced by the daemon. */

static imud_packet_t client_copy(const void *buf)
{
    imud_packet_t p;
    memcpy(&p, buf, sizeof p);
    return p;
}

uint16_t client_flags(const void *buf)       { return client_copy(buf).flags; }
uint64_t client_ts_wall_ns(const void *buf)  { return client_copy(buf).ts_wall_ns; }
float    client_heading(const void *buf)     { return client_copy(buf).heading_deg; }
float    client_declination(const void *buf) { return client_copy(buf).declination_deg; }
float    client_quat_w(const void *buf)      { return client_copy(buf).quat_w; }
float    client_mag_x(const void *buf)       { return client_copy(buf).mag_x; }
float    client_heave(const void *buf)       { return client_copy(buf).heave_m; }
float    client_wave_height(const void *buf) { return client_copy(buf).wave_height_m; }
float    client_roll_period(const void *buf) { return client_copy(buf).roll_period_s; }
float    client_pitch_period(const void *buf) { return client_copy(buf).pitch_period_s; }
float    client_mag_residual(const void *buf) { return client_copy(buf).mag_residual; }
float    client_innov_weight(const void *buf) { return client_copy(buf).innov_weight; }

float client_true_heading(const void *buf)
{
    imud_packet_t p = client_copy(buf);
    return imud_true_heading(&p);
}

/* ── Socket helpers ──────────────────────────────────────────────────────── */

/*
 * imud_open/imud_recv/imud_close are the half of the single header a vendoring
 * consumer actually runs, and they had no coverage.  Wrapped here for the same
 * reason as everything above: test_client.c drives them without the client's
 * struct definition in scope.
 */

int client_open(int port, const char *dest_addr)
{
    return imud_open(port, dest_addr);
}

/* Receive one valid packet into `out`, which must have room for a whole one. */
int client_recv_raw(int fd, void *out)
{
    imud_packet_t p;
    int rc = imud_recv(fd, &p);
    if (rc == 0) memcpy(out, &p, sizeof p);
    return rc;
}

/*
 * Bound the blocking recv so a regression in imud_packet_valid cannot hang
 * CI: imud_recv loops until a datagram validates, so without a timeout a
 * rejected-but-valid packet would wedge the test forever rather than fail it.
 * Also gives the test a way to exercise imud_recv's error return.
 */
int client_set_rcvtimeo(int fd, int secs)
{
    struct timeval tv = { .tv_sec = secs, .tv_usec = 0 };
    return setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
}

void client_close(int fd)
{
    imud_close(fd);
}
