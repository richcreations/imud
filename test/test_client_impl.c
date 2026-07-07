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

float client_true_heading(const void *buf)
{
    imud_packet_t p = client_copy(buf);
    return imud_true_heading(&p);
}
