/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * packet.c — 196-byte binary packet encoder for imud Stream B (§8)
 *
 * Wire layout: little-endian, IEEE 802.3 CRC32 over bytes 0–187.
 * Coordinate frame: NED by default; "ENU" rotates vectors and quaternion.
 *
 * NED → ENU transform:
 *   vectors:    (x,y,z)_ENU = (y,x,-z)_NED
 *   quaternion: q_ENU = [0, 1/√2, 1/√2, 0] ⊗ q_NED  (180° around [1,1,0]/√2)
 */

#include <stddef.h>
#include <string.h>
#include <math.h>
#include "packet.h"

/* ── CRC32 (IEEE 802.3 / Ethernet polynomial 0xEDB88320) ────────────────── */

static uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── NED → ENU ───────────────────────────────────────────────────────────── */

#define INV_SQRT2  0.70710678118654752f   /* 1/√2 */

/*
 * Rotate a 3-vector from NED to ENU in-place.
 * ENU: x=East, y=North, z=Up  ←→  NED: x=North, y=East, z=Down
 *   x_ENU = y_NED
 *   y_ENU = x_NED
 *   z_ENU = -z_NED
 */
static void vec3_ned_to_enu(float v[3])
{
    float tmp = v[0];
    v[0] =  v[1];
    v[1] =  tmp;
    v[2] = -v[2];
}

/*
 * Rotate quaternion from body→NED to body→ENU.
 * q_ENU = p ⊗ q_NED  where p = [0, 1/√2, 1/√2, 0]  (Hamilton product).
 *
 * Expanded (p.w=0, p.x=1/√2, p.y=1/√2, p.z=0):
 *   rw = -(qx + qy) / √2
 *   rx =  (qw + qz) / √2
 *   ry =  (qw - qz) / √2
 *   rz =  (qy - qx) / √2
 */
static void quat_ned_to_enu(float q[4])
{
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    q[0] = INV_SQRT2 * (-qx - qy);
    q[1] = INV_SQRT2 * ( qw + qz);
    q[2] = INV_SQRT2 * ( qw - qz);
    q[3] = INV_SQRT2 * ( qy - qx);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void packet_build(imu_packet_t       *pkt,
                  const fused_state_t *state,
                  const mag_sample_t  *mag,
                  const imu_sample_t  *imu,
                  const imu_sample_t  *raw_imu,
                  const char          *coord_frame)
{
    memset(pkt, 0, sizeof(*pkt));

    pkt->magic   = IMUD_MAGIC;
    pkt->version = IMUD_VERSION;
    pkt->flags   = state->flags;

    /* Timestamps */
    pkt->ts_wall_ns    = state->ts_wall_ns;
    pkt->ts_tai_ns     = state->ts_tai_ns;
    pkt->ts_chip_ticks = state->ts_chip_ticks;
    pkt->anchor_gen    = state->anchor_gen;

    /* Sensor fields — start from NED values, convert if ENU requested */
    float accel[3]      = { imu->accel[0],         imu->accel[1],         imu->accel[2]         };
    float accel_raw[3]  = { raw_imu->accel_raw[0],  raw_imu->accel_raw[1],  raw_imu->accel_raw[2]  };
    float gyro[3]       = { imu->gyro[0],           imu->gyro[1],           imu->gyro[2]           };
    float gyro_raw[3]   = { raw_imu->gyro[0],       raw_imu->gyro[1],       raw_imu->gyro[2]       };
    float mag_f[3]      = { mag->field[0],           mag->field[1],          mag->field[2]          };
    float mag_raw[3]    = { mag->field_raw[0],       mag->field_raw[1],      mag->field_raw[2]      };
    float q[4]          = { state->q[0],             state->q[1],            state->q[2], state->q[3] };

    if (coord_frame && coord_frame[0] == 'E') {  /* "ENU" */
        vec3_ned_to_enu(accel);
        vec3_ned_to_enu(accel_raw);
        vec3_ned_to_enu(gyro);
        vec3_ned_to_enu(gyro_raw);
        vec3_ned_to_enu(mag_f);
        vec3_ned_to_enu(mag_raw);
        quat_ned_to_enu(q);
    }

    pkt->accel_x = accel[0];
    pkt->accel_y = accel[1];
    pkt->accel_z = accel[2];

    pkt->accel_raw_x = accel_raw[0];
    pkt->accel_raw_y = accel_raw[1];
    pkt->accel_raw_z = accel_raw[2];

    pkt->gyro_x = gyro[0];
    pkt->gyro_y = gyro[1];
    pkt->gyro_z = gyro[2];

    pkt->gyro_raw_x = gyro_raw[0];
    pkt->gyro_raw_y = gyro_raw[1];
    pkt->gyro_raw_z = gyro_raw[2];

    pkt->mag_x = mag_f[0];
    pkt->mag_y = mag_f[1];
    pkt->mag_z = mag_f[2];

    pkt->mag_raw_x = mag_raw[0];
    pkt->mag_raw_y = mag_raw[1];
    pkt->mag_raw_z = mag_raw[2];

    pkt->quat_w = q[0];
    pkt->quat_x = q[1];
    pkt->quat_y = q[2];
    pkt->quat_z = q[3];

    /* Euler angles and heading are frame-neutral scalars */
    pkt->pitch       = state->pitch;
    pkt->roll        = state->roll;
    pkt->yaw         = state->yaw;
    pkt->heading_deg  = state->heading_deg;
    pkt->rate_of_turn = state->rate_of_turn;

    pkt->temp_c = imu->temp_c;

    /* Attitude error covariance from MEKF P matrix (rad², row-major 3×3) */
    memcpy(pkt->cov, state->cov, sizeof(pkt->cov));

    pkt->imu_seq = state->imu_seq;

    pkt->declination_deg = (state->flags & FLAG_DECLINATION_VALID)
                           ? state->declination_deg : 0.0f;
    pkt->heave_m = state->heave_m;

    /* v12 diagnostics — body-frame / frame-neutral (no coord_frame conversion) */
    pkt->gyro_bias_x      = state->bias_gyro[0];
    pkt->gyro_bias_y      = state->bias_gyro[1];
    pkt->gyro_bias_z      = state->bias_gyro[2];
    pkt->gyro_bias_var_x  = state->bias_gyro_var[0];
    pkt->gyro_bias_var_y  = state->bias_gyro_var[1];
    pkt->gyro_bias_var_z  = state->bias_gyro_var[2];
    pkt->heave_rate       = state->heave_rate;
    pkt->accel_quiescence = state->quiescence;

    /* CRC32 covers all bytes before the crc32 field */
    pkt->crc32 = crc32_ieee((const uint8_t *)pkt, offsetof(imu_packet_t, crc32));
}
