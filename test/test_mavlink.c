/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_mavlink.c — unit tests for the MAVLink encoder (src/mavlink_encode.c)
 *
 * Compares encoder output against golden frames captured from a pymavlink
 * cross-check (the reference implementation decoded these exact bytes as valid
 * HEARTBEAT / ATTITUDE / ATTITUDE_QUATERNION in both v1 and v2). Self-contained,
 * so CI validates correctness without pymavlink. Inputs match the cross-check
 * harness: sysid=1, compid=1, seq 0/1/2, time_boot_ms=1234, roll=0.1,
 * pitch=-0.05, yaw=1.23, rates 0.01/0.02/0.03, quaternion (1,0,0,0).
 */

#include <stdio.h>
#include <string.h>
#include "mavlink_encode.h"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", __FILE__, __LINE__, (msg)); } \
} while (0)

static void tohex(const uint8_t *b, int n, char *out)
{
    for (int i = 0; i < n; i++) sprintf(out + 2 * i, "%02x", b[i]);
    out[2 * n] = '\0';
}

static void check_golden(const char *name, int n, const uint8_t *b, const char *golden)
{
    char hex[160];
    tohex(b, n, hex);
    int ok = (strcmp(hex, golden) == 0);
    EXPECT(ok, name);
    if (!ok) fprintf(stderr, "    got:  %s\n    want: %s\n", hex, golden);
}

int main(void)
{
    puts("=== imud mavlink encoder tests ===");
    uint8_t b[MAV_MAX_FRAME];
    int n;

    /* ── v2 golden frames ────────────────────────────────────────────────── */
    n = mav_pack_heartbeat(b, 2, 1, 1, 0);
    check_golden("v2 HEARTBEAT", n, b,
        "fd090000000101000000000000000008000403f098");
    n = mav_pack_attitude(b, 2, 1, 1, 1, 1234, 0.1f, -0.05f, 1.23f, 0.01f, 0.02f, 0.03f);
    check_golden("v2 ATTITUDE", n, b,
        "fd1c00000101011e0000d2040000cdcccc3dcdcc4cbda4709d3f0ad7233c0ad7a33c8fc2f53ce73d");
    n = mav_pack_attitude_quaternion(b, 2, 1, 1, 2, 1234, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.02f, 0.03f);
    check_golden("v2 ATTITUDE_QUATERNION", n, b,
        "fd2000000201011f0000d20400000000803f0000000000000000000000000ad7233c0ad7a33c8fc2f53c9438");

    /* ── v1 golden frames ────────────────────────────────────────────────── */
    n = mav_pack_heartbeat(b, 1, 1, 1, 0);
    check_golden("v1 HEARTBEAT", n, b,
        "fe09000101000000000000080004036a5b");
    n = mav_pack_attitude(b, 1, 1, 1, 1, 1234, 0.1f, -0.05f, 1.23f, 0.01f, 0.02f, 0.03f);
    check_golden("v1 ATTITUDE", n, b,
        "fe1c0101011ed2040000cdcccc3dcdcc4cbda4709d3f0ad7233c0ad7a33c8fc2f53c3ded");
    n = mav_pack_attitude_quaternion(b, 1, 1, 1, 2, 1234, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.02f, 0.03f);
    check_golden("v1 ATTITUDE_QUATERNION", n, b,
        "fe200201011fd20400000000803f0000000000000000000000000ad7233c0ad7a33c8fc2f53c7c97");

    /* ── structural spot-checks ──────────────────────────────────────────── */
    n = mav_pack_heartbeat(b, 2, 1, 1, 0);
    EXPECT(b[0] == MAV_STX_V2,           "v2 STX = 0xFD");
    EXPECT(b[1] == 9,                    "v2 HEARTBEAT length = 9");
    EXPECT(b[2] == 0 && b[3] == 0,       "v2 incompat/compat flags = 0");
    EXPECT(b[7] == MAVMSG_HEARTBEAT,     "v2 msgid low byte = 0");
    n = mav_pack_heartbeat(b, 1, 1, 1, 0);
    EXPECT(b[0] == MAV_STX_V1,           "v1 STX = 0xFE");
    EXPECT(b[1] == 9,                    "v1 HEARTBEAT length = 9");
    EXPECT(b[5] == MAVMSG_HEARTBEAT,     "v1 msgid = 0");
    n = mav_pack_attitude(b, 2, 7, 42, 5, 0, 0, 0, 0, 0, 0, 0);
    EXPECT(b[5] == 7 && b[6] == 42,      "v2 sysid/compid placed correctly");
    EXPECT(b[4] == 5,                    "v2 seq placed correctly");

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
