/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mavlink_encode.h — minimal, hand-rolled MAVLink v1/v2 frame encoder
 *
 * Encodes just the messages the imud-mavlink bridge sends (HEARTBEAT, ATTITUDE,
 * ATTITUDE_QUATERNION) for MAVLink protocol version 1 (0xFE) or 2 (0xFD), with
 * the CRC-16/MCRF4XX + per-message CRC_EXTRA. No signing (v2 incompat/compat
 * flags = 0), no v2 payload truncation (full-length payloads — always valid).
 *
 * Pure and self-contained: takes primitive values (not imud packets), so it has
 * no dependency on the client header and is unit-tested directly. All angles are
 * SI radians / rad/s, in MAVLink's FRD/NED body frame (which matches imud).
 *
 * Each pack function writes a complete frame to `out` (size a MAV_MAX_FRAME
 * buffer) and returns the frame length in bytes. `ver` is 1 or 2.
 */
#ifndef IMUD_MAVLINK_ENCODE_H
#define IMUD_MAVLINK_ENCODE_H

#include <stdint.h>

#define MAV_STX_V1     0xFE
#define MAV_STX_V2     0xFD
#define MAV_MAX_FRAME  64          /* v2 header 10 + max payload 32 + crc 2 = 44 */

/* Message ids and their CRC_EXTRA (from the common dialect). */
#define MAVMSG_HEARTBEAT             0
#define MAVMSG_ATTITUDE              30
#define MAVMSG_ATTITUDE_QUATERNION   31

/* Low-level framer (exposed for testing): frames one message. */
int mav_frame(uint8_t *out, int ver, uint8_t sysid, uint8_t compid, uint8_t seq,
              uint32_t msgid, const uint8_t *payload, uint8_t len, uint8_t crc_extra);

/* HEARTBEAT (#0): type=GENERIC, autopilot=INVALID, status=ACTIVE. */
int mav_pack_heartbeat(uint8_t *out, int ver, uint8_t sysid, uint8_t compid, uint8_t seq);

/* ATTITUDE (#30): angles rad, rates rad/s. */
int mav_pack_attitude(uint8_t *out, int ver, uint8_t sysid, uint8_t compid, uint8_t seq,
                      uint32_t time_boot_ms, float roll, float pitch, float yaw,
                      float rollspeed, float pitchspeed, float yawspeed);

/* ATTITUDE_QUATERNION (#31): q = w,x,y,z; rates rad/s (base message, len 32). */
int mav_pack_attitude_quaternion(uint8_t *out, int ver, uint8_t sysid, uint8_t compid,
                                 uint8_t seq, uint32_t time_boot_ms,
                                 float q1, float q2, float q3, float q4,
                                 float rollspeed, float pitchspeed, float yawspeed);

#endif /* IMUD_MAVLINK_ENCODE_H */
