/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mavlink_encode.c — minimal MAVLink v1/v2 frame encoder (see mavlink_encode.h)
 */

#include "mavlink_encode.h"
#include <string.h>

/* CRC-16/MCRF4XX (the MAVLink checksum), one byte at a time. */
static void crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xff);
    tmp ^= (uint8_t)(tmp << 4);
    *crc = (uint16_t)((*crc >> 8) ^ ((uint16_t)tmp << 8)
                      ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

/* Little-endian payload writers. */
static void put_u8 (uint8_t **p, uint8_t v)  { *(*p)++ = v; }
static void put_u32(uint8_t **p, uint32_t v)
{
    (*p)[0] = (uint8_t)v;         (*p)[1] = (uint8_t)(v >> 8);
    (*p)[2] = (uint8_t)(v >> 16); (*p)[3] = (uint8_t)(v >> 24);
    *p += 4;
}
static void put_f(uint8_t **p, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    put_u32(p, u);
}

int mav_frame(uint8_t *out, int ver, uint8_t sysid, uint8_t compid, uint8_t seq,
              uint32_t msgid, const uint8_t *payload, uint8_t len, uint8_t crc_extra)
{
    int i = 0;
    uint16_t crc = 0xFFFF;

    if (ver == 1) {
        out[i++] = MAV_STX_V1;
        uint8_t hdr[5] = { len, seq, sysid, compid, (uint8_t)(msgid & 0xFF) };
        for (int h = 0; h < 5; h++) { out[i++] = hdr[h]; crc_accumulate(hdr[h], &crc); }
    } else {
        out[i++] = MAV_STX_V2;
        uint8_t hdr[9] = { len, 0x00, 0x00, seq, sysid, compid,
                           (uint8_t)(msgid & 0xFF),
                           (uint8_t)((msgid >> 8) & 0xFF),
                           (uint8_t)((msgid >> 16) & 0xFF) };
        for (int h = 0; h < 9; h++) { out[i++] = hdr[h]; crc_accumulate(hdr[h], &crc); }
    }

    for (uint8_t k = 0; k < len; k++) { out[i++] = payload[k]; crc_accumulate(payload[k], &crc); }

    crc_accumulate(crc_extra, &crc);
    out[i++] = (uint8_t)(crc & 0xFF);
    out[i++] = (uint8_t)(crc >> 8);
    return i;
}

int mav_pack_heartbeat(uint8_t *out, int ver, uint8_t sysid, uint8_t compid, uint8_t seq)
{
    uint8_t pl[9], *p = pl;
    put_u32(&p, 0);   /* custom_mode                       */
    put_u8(&p, 0);    /* type = MAV_TYPE_GENERIC           */
    put_u8(&p, 8);    /* autopilot = MAV_AUTOPILOT_INVALID */
    put_u8(&p, 0);    /* base_mode                         */
    put_u8(&p, 4);    /* system_status = MAV_STATE_ACTIVE  */
    put_u8(&p, 3);    /* mavlink_version                   */
    return mav_frame(out, ver, sysid, compid, seq, MAVMSG_HEARTBEAT, pl, 9, 50);
}

int mav_pack_attitude(uint8_t *out, int ver, uint8_t sysid, uint8_t compid, uint8_t seq,
                      uint32_t time_boot_ms, float roll, float pitch, float yaw,
                      float rollspeed, float pitchspeed, float yawspeed)
{
    uint8_t pl[28], *p = pl;
    put_u32(&p, time_boot_ms);
    put_f(&p, roll);  put_f(&p, pitch); put_f(&p, yaw);
    put_f(&p, rollspeed); put_f(&p, pitchspeed); put_f(&p, yawspeed);
    return mav_frame(out, ver, sysid, compid, seq, MAVMSG_ATTITUDE, pl, 28, 39);
}

int mav_pack_attitude_quaternion(uint8_t *out, int ver, uint8_t sysid, uint8_t compid,
                                 uint8_t seq, uint32_t time_boot_ms,
                                 float q1, float q2, float q3, float q4,
                                 float rollspeed, float pitchspeed, float yawspeed)
{
    uint8_t pl[32], *p = pl;
    put_u32(&p, time_boot_ms);
    put_f(&p, q1); put_f(&p, q2); put_f(&p, q3); put_f(&p, q4);
    put_f(&p, rollspeed); put_f(&p, pitchspeed); put_f(&p, yawspeed);
    return mav_frame(out, ver, sysid, compid, seq, MAVMSG_ATTITUDE_QUATERNION, pl, 32, 246);
}
