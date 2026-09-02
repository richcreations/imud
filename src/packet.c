/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * packet.c — 276-byte binary packet encoder for imud Stream B (§8)
 *
 * Wire layout: little-endian, IEEE 802.3 CRC32 over bytes 0–271.  packet_build
 * fills a host-order struct; packet_encode/packet_decode convert it to and from
 * the wire bytes with shifts, so both endiannesses run one code path.
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
#include "crc32.h"
#include "wire.h"

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

    /* v14 sea state + compass health — frame-neutral scalars */
    pkt->wave_height_m   = state->wave_height_m;
    pkt->wave_period_s   = state->wave_period_s;
    pkt->roll_period_s   = state->roll_period_s;
    pkt->roll_amplitude  = state->roll_amplitude;
    pkt->pitch_period_s  = state->pitch_period_s;
    pkt->pitch_amplitude = state->pitch_amplitude;
    pkt->mag_anomaly     = state->mag_anomaly;
    pkt->mag_residual    = state->mag_residual;
    pkt->innov_weight    = state->innov_weight;
    pkt->innov_reject    = state->innov_reject;
    pkt->nis_accel       = state->nis_accel;
    pkt->nis_mag         = state->nis_mag;

    /* The CRC belongs to the wire bytes, which packet_encode builds. */
    pkt->crc32 = 0;
}

/* ── Wire serialisation ──────────────────────────────────────────────────── */

/*
 * Both directions walk the fields in wire order, spelled out one per line, so
 * the list reads against spec.md §8's offset table.  A field added to the
 * struct and forgotten here survives compilation, so test_packet's whole-struct
 * round-trip over a distinct fill is what catches it.
 */

uint32_t packet_encode(uint8_t out[IMUD_PACKET_BYTES], const imu_packet_t *pkt)
{
    uint8_t *p = out;

    wire_put_u32(p, pkt->magic);            p += 4;
    wire_put_u16(p, pkt->version);          p += 2;
    wire_put_u16(p, pkt->flags);            p += 2;
    wire_put_u64(p, pkt->ts_wall_ns);       p += 8;
    wire_put_u64(p, pkt->ts_tai_ns);        p += 8;
    wire_put_u32(p, pkt->ts_chip_ticks);    p += 4;
    wire_put_u32(p, pkt->anchor_gen);       p += 4;

    wire_put_f32(p, pkt->accel_x);          p += 4;
    wire_put_f32(p, pkt->accel_y);          p += 4;
    wire_put_f32(p, pkt->accel_z);          p += 4;
    wire_put_f32(p, pkt->accel_raw_x);      p += 4;
    wire_put_f32(p, pkt->accel_raw_y);      p += 4;
    wire_put_f32(p, pkt->accel_raw_z);      p += 4;

    wire_put_f32(p, pkt->gyro_x);           p += 4;
    wire_put_f32(p, pkt->gyro_y);           p += 4;
    wire_put_f32(p, pkt->gyro_z);           p += 4;
    wire_put_f32(p, pkt->gyro_raw_x);       p += 4;
    wire_put_f32(p, pkt->gyro_raw_y);       p += 4;
    wire_put_f32(p, pkt->gyro_raw_z);       p += 4;

    wire_put_f32(p, pkt->mag_x);            p += 4;
    wire_put_f32(p, pkt->mag_y);            p += 4;
    wire_put_f32(p, pkt->mag_z);            p += 4;
    wire_put_f32(p, pkt->mag_raw_x);        p += 4;
    wire_put_f32(p, pkt->mag_raw_y);        p += 4;
    wire_put_f32(p, pkt->mag_raw_z);        p += 4;

    wire_put_f32(p, pkt->quat_w);           p += 4;
    wire_put_f32(p, pkt->quat_x);           p += 4;
    wire_put_f32(p, pkt->quat_y);           p += 4;
    wire_put_f32(p, pkt->quat_z);           p += 4;

    wire_put_f32(p, pkt->pitch);            p += 4;
    wire_put_f32(p, pkt->roll);             p += 4;
    wire_put_f32(p, pkt->yaw);              p += 4;
    wire_put_f32(p, pkt->heading_deg);      p += 4;
    wire_put_f32(p, pkt->rate_of_turn);     p += 4;
    wire_put_f32(p, pkt->temp_c);           p += 4;

    for (int i = 0; i < 9; i++) { wire_put_f32(p, pkt->cov[i]); p += 4; }

    wire_put_u32(p, pkt->imu_seq);          p += 4;
    wire_put_f32(p, pkt->declination_deg);  p += 4;
    wire_put_f32(p, pkt->heave_m);          p += 4;

    wire_put_f32(p, pkt->gyro_bias_x);      p += 4;
    wire_put_f32(p, pkt->gyro_bias_y);      p += 4;
    wire_put_f32(p, pkt->gyro_bias_z);      p += 4;
    wire_put_f32(p, pkt->gyro_bias_var_x);  p += 4;
    wire_put_f32(p, pkt->gyro_bias_var_y);  p += 4;
    wire_put_f32(p, pkt->gyro_bias_var_z);  p += 4;
    wire_put_f32(p, pkt->heave_rate);       p += 4;
    wire_put_f32(p, pkt->accel_quiescence); p += 4;

    wire_put_f32(p, pkt->wave_height_m);    p += 4;
    wire_put_f32(p, pkt->wave_period_s);    p += 4;
    wire_put_f32(p, pkt->roll_period_s);    p += 4;
    wire_put_f32(p, pkt->roll_amplitude);   p += 4;
    wire_put_f32(p, pkt->pitch_period_s);   p += 4;
    wire_put_f32(p, pkt->pitch_amplitude);  p += 4;
    wire_put_f32(p, pkt->mag_anomaly);      p += 4;
    wire_put_f32(p, pkt->mag_residual);     p += 4;

    wire_put_f32(p, pkt->innov_weight);     p += 4;
    wire_put_f32(p, pkt->innov_reject);     p += 4;
    wire_put_f32(p, pkt->nis_accel);        p += 4;
    wire_put_f32(p, pkt->nis_mag);          p += 4;

    uint32_t crc = crc32_ieee(out, offsetof(imu_packet_t, crc32));
    wire_put_u32(p, crc);

    return crc;
}

void packet_decode(imu_packet_t *pkt, const uint8_t in[IMUD_PACKET_BYTES])
{
    const uint8_t *p = in;

    pkt->magic            = wire_get_u32(p); p += 4;
    pkt->version          = wire_get_u16(p); p += 2;
    pkt->flags            = wire_get_u16(p); p += 2;
    pkt->ts_wall_ns       = wire_get_u64(p); p += 8;
    pkt->ts_tai_ns        = wire_get_u64(p); p += 8;
    pkt->ts_chip_ticks    = wire_get_u32(p); p += 4;
    pkt->anchor_gen       = wire_get_u32(p); p += 4;

    pkt->accel_x          = wire_get_f32(p); p += 4;
    pkt->accel_y          = wire_get_f32(p); p += 4;
    pkt->accel_z          = wire_get_f32(p); p += 4;
    pkt->accel_raw_x      = wire_get_f32(p); p += 4;
    pkt->accel_raw_y      = wire_get_f32(p); p += 4;
    pkt->accel_raw_z      = wire_get_f32(p); p += 4;

    pkt->gyro_x           = wire_get_f32(p); p += 4;
    pkt->gyro_y           = wire_get_f32(p); p += 4;
    pkt->gyro_z           = wire_get_f32(p); p += 4;
    pkt->gyro_raw_x       = wire_get_f32(p); p += 4;
    pkt->gyro_raw_y       = wire_get_f32(p); p += 4;
    pkt->gyro_raw_z       = wire_get_f32(p); p += 4;

    pkt->mag_x            = wire_get_f32(p); p += 4;
    pkt->mag_y            = wire_get_f32(p); p += 4;
    pkt->mag_z            = wire_get_f32(p); p += 4;
    pkt->mag_raw_x        = wire_get_f32(p); p += 4;
    pkt->mag_raw_y        = wire_get_f32(p); p += 4;
    pkt->mag_raw_z        = wire_get_f32(p); p += 4;

    pkt->quat_w           = wire_get_f32(p); p += 4;
    pkt->quat_x           = wire_get_f32(p); p += 4;
    pkt->quat_y           = wire_get_f32(p); p += 4;
    pkt->quat_z           = wire_get_f32(p); p += 4;

    pkt->pitch            = wire_get_f32(p); p += 4;
    pkt->roll             = wire_get_f32(p); p += 4;
    pkt->yaw              = wire_get_f32(p); p += 4;
    pkt->heading_deg      = wire_get_f32(p); p += 4;
    pkt->rate_of_turn     = wire_get_f32(p); p += 4;
    pkt->temp_c           = wire_get_f32(p); p += 4;

    for (int i = 0; i < 9; i++) { pkt->cov[i] = wire_get_f32(p); p += 4; }

    pkt->imu_seq          = wire_get_u32(p); p += 4;
    pkt->declination_deg  = wire_get_f32(p); p += 4;
    pkt->heave_m          = wire_get_f32(p); p += 4;

    pkt->gyro_bias_x      = wire_get_f32(p); p += 4;
    pkt->gyro_bias_y      = wire_get_f32(p); p += 4;
    pkt->gyro_bias_z      = wire_get_f32(p); p += 4;
    pkt->gyro_bias_var_x  = wire_get_f32(p); p += 4;
    pkt->gyro_bias_var_y  = wire_get_f32(p); p += 4;
    pkt->gyro_bias_var_z  = wire_get_f32(p); p += 4;
    pkt->heave_rate       = wire_get_f32(p); p += 4;
    pkt->accel_quiescence = wire_get_f32(p); p += 4;

    pkt->wave_height_m    = wire_get_f32(p); p += 4;
    pkt->wave_period_s    = wire_get_f32(p); p += 4;
    pkt->roll_period_s    = wire_get_f32(p); p += 4;
    pkt->roll_amplitude   = wire_get_f32(p); p += 4;
    pkt->pitch_period_s   = wire_get_f32(p); p += 4;
    pkt->pitch_amplitude  = wire_get_f32(p); p += 4;
    pkt->mag_anomaly      = wire_get_f32(p); p += 4;
    pkt->mag_residual     = wire_get_f32(p); p += 4;

    pkt->innov_weight     = wire_get_f32(p); p += 4;
    pkt->innov_reject     = wire_get_f32(p); p += 4;
    pkt->nis_accel        = wire_get_f32(p); p += 4;
    pkt->nis_mag          = wire_get_f32(p); p += 4;

    pkt->crc32            = wire_get_u32(p);
}
