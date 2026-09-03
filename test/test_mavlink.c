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

/* ── SYS_STATUS (#1) ──────────────────────────────────────────────────────
 *
 * Golden frames, cross-checked against pymavlink 2.4.49 the same way the
 * messages above were: sysid=1, compid=1, seq=0, a magnetometer fitted
 * (3D_MAG in present/enabled) but not being fused (3D_MAG clear in health).
 * The structural assertions below are kept alongside them because a golden
 * frame says only "these bytes differ", not which field moved.
 */

static void crc_acc(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xff);
    tmp ^= (uint8_t)(tmp << 4);
    *crc = (uint16_t)((*crc >> 8) ^ ((uint16_t)tmp << 8)
                      ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

static void crc_acc_str(const char *s, uint16_t *crc)
{
    while (*s) crc_acc((uint8_t)*s++, crc);
}

/*
 * Recompute MAVLink's CRC_EXTRA from the message definition, the way mavgen
 * does: the message name then each field's type and name, in WIRE order.  A
 * wrong CRC_EXTRA yields frames every receiver silently drops, and it is a
 * bare constant in the encoder with nothing else to check it against.  Two
 * independent derivations agreeing on 124 is the check.
 */
static uint8_t sys_status_crc_extra(void)
{
    static const char *fields[][2] = {
        { "uint32_t", "onboard_control_sensors_present" },
        { "uint32_t", "onboard_control_sensors_enabled" },
        { "uint32_t", "onboard_control_sensors_health"  },
        { "uint16_t", "load"                            },
        { "uint16_t", "voltage_battery"                 },
        { "int16_t",  "current_battery"                 },
        { "uint16_t", "drop_rate_comm"                  },
        { "uint16_t", "errors_comm"                     },
        { "uint16_t", "errors_count1"                   },
        { "uint16_t", "errors_count2"                   },
        { "uint16_t", "errors_count3"                   },
        { "uint16_t", "errors_count4"                   },
        { "int8_t",   "battery_remaining"               },
    };
    uint16_t crc = 0xFFFF;
    crc_acc_str("SYS_STATUS ", &crc);
    for (size_t i = 0; i < sizeof fields / sizeof *fields; i++) {
        crc_acc_str(fields[i][0], &crc); crc_acc(' ', &crc);
        crc_acc_str(fields[i][1], &crc); crc_acc(' ', &crc);
    }
    return (uint8_t)((crc & 0xFF) ^ (crc >> 8));
}

static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void test_sys_status(void)
{
    EXPECT(sys_status_crc_extra() == 124,
           "SYS_STATUS CRC_EXTRA derives to 124, the value the encoder uses");

    const uint32_t present = MAV_SENSOR_3D_GYRO | MAV_SENSOR_3D_ACCEL |
                             MAV_SENSOR_AHRS | MAV_SENSOR_3D_MAG;
    const uint32_t health  = MAV_SENSOR_3D_GYRO | MAV_SENSOR_3D_ACCEL |
                             MAV_SENSOR_AHRS;   /* mag fitted but not fused */

    uint8_t b[MAV_MAX_FRAME];
    int n = mav_pack_sys_status(b, 1, 1, 1, 0, present, present, health);
    check_golden("v1 SYS_STATUS matches pymavlink", n, b,
                 "fe1f000101010700200007002000030020000000ffffffff"
                 "000000000000000000000000fffe76");
    EXPECT(n == 6 + 31 + 2, "v1 SYS_STATUS frame is 39 bytes");
    EXPECT(b[0] == MAV_STX_V1, "v1 STX");
    EXPECT(b[1] == 31, "payload length 31");
    EXPECT(b[5] == MAVMSG_SYS_STATUS, "msgid 1");

    const uint8_t *pl = b + 6;
    EXPECT(rd_u32le(pl + 0) == present, "present bitmask at payload offset 0");
    EXPECT(rd_u32le(pl + 4) == present, "enabled bitmask at offset 4");
    EXPECT(rd_u32le(pl + 8) == health,  "health bitmask at offset 8");
    EXPECT((pl[8] & 0x04) == 0,
           "3D_MAG clear in health when the magnetometer is not fused");
    EXPECT((pl[0] & 0x04) != 0,
           "3D_MAG set in present when a magnetometer is fitted");
    /* Battery/current are "unknown" sentinels, not zeros a receiver would
     * display as a flat battery. */
    EXPECT(pl[12] == 0 && pl[13] == 0, "load = 0 at offset 12");
    EXPECT(pl[14] == 0xFF && pl[15] == 0xFF, "voltage_battery = UINT16_MAX at 14");
    EXPECT(pl[30] == 0xFF, "battery_remaining = -1");

    n = mav_pack_sys_status(b, 2, 1, 1, 0, present, present, health);
    check_golden("v2 SYS_STATUS matches pymavlink", n, b,
                 "fd1f00000001010100000700200007002000030020000000ffffffff"
                 "000000000000000000000000ff2edb");
    EXPECT(n == 10 + 31 + 2, "v2 SYS_STATUS frame is 43 bytes");
    EXPECT(b[0] == MAV_STX_V2, "v2 STX");
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

    test_sys_status();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
