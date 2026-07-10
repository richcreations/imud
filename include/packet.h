/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/* packet.h — 228-byte binary packet encoder for imud Stream B (§8) */

#ifndef IMUD_PACKET_H
#define IMUD_PACKET_H

#include "types.h"

/*
 * Build one 228-byte imu_packet_t from the current fused state, latest mag,
 * bias-corrected IMU sample, and raw (pre-calibration) IMU sample.
 *
 * coord_frame: "NED" (default) or "ENU".  If "ENU", vectors and quaternion
 *              are transformed from NED to ENU before packing.
 *              All other fields (Euler angles, heading, etc.) are frame-neutral.
 */
void packet_build(imu_packet_t       *pkt,
                  const fused_state_t *state,
                  const mag_sample_t  *mag,
                  const imu_sample_t  *imu,
                  const imu_sample_t  *raw_imu,
                  const char          *coord_frame);

#endif /* IMUD_PACKET_H */
