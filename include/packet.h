/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/* packet.h — 276-byte binary packet encoder for imud Stream B (§8) */

#ifndef IMUD_PACKET_H
#define IMUD_PACKET_H

#include "types.h"

/*
 * Build one imu_packet_t from the current fused state, latest mag,
 * bias-corrected IMU sample, and raw (pre-calibration) IMU sample.
 *
 * coord_frame: "NED" (default) or "ENU".  If "ENU", vectors and quaternion
 *              are transformed from NED to ENU before packing.
 *              All other fields (Euler angles, heading, etc.) are frame-neutral.
 *
 * Leaves .crc32 zero: the CRC covers the little-endian WIRE bytes, which only
 * packet_encode() has.  Nothing may transmit this struct directly.
 */
void packet_build(imu_packet_t       *pkt,
                  const fused_state_t *state,
                  const mag_sample_t  *mag,
                  const imu_sample_t  *imu,
                  const imu_sample_t  *raw_imu,
                  const char          *coord_frame);

/*
 * Serialise pkt into the 276 little-endian wire bytes, computing the CRC over
 * bytes 0–271 and writing it at 272.  Returns that CRC.  The struct's own
 * .crc32 is ignored.
 */
uint32_t packet_encode(uint8_t out[IMUD_PACKET_BYTES], const imu_packet_t *pkt);

/*
 * Deserialise 276 wire bytes into a host-order struct, .crc32 included.  Does
 * not validate magic, version or CRC — the caller checks those on the bytes.
 */
void packet_decode(imu_packet_t *pkt, const uint8_t in[IMUD_PACKET_BYTES]);

#endif /* IMUD_PACKET_H */
