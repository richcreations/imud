/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_packet.c — unit tests for the binary packet encoder (src/packet.c)
 *
 * Key correctness properties verified:
 *   - CRC32 implementation matches the IEEE 802.3 test vector
 *   - packet_build() populates the CRC field correctly
 *   - Flipping any payload byte changes the CRC (corruption detected)
 *   - NED mode: sensor vectors are copied unchanged
 *   - ENU mode: vectors are permuted (x,y,z)→(y,x,-z)
 *   - ENU mode: quaternion norm is preserved (still unit quaternion)
 *   - ENU mode: identity NED quaternion maps to the NED→ENU rotation quaternion
 *   - Magic, version, flags, timestamps, seq all copied correctly
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "packet.h"
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabsf((float)(a) - (float)(b)) < (float)(eps), msg)

static void begin(const char *name) { printf("%-48s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Reference CRC32 (IEEE 802.3 polynomial 0xEDB88320) ─────────────────── */

/* ── Reference little-endian readers ─────────────────────────────────────── */

/* Independent of include/wire.h on purpose: the encoder is checked against
 * these, not against itself. */
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint64_t rd64(const uint8_t *p)
{
    return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32);
}

static uint32_t ref_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── Input constructors ──────────────────────────────────────────────────── */

static fused_state_t make_state(void)
{
    fused_state_t s;
    memset(&s, 0, sizeof(s));
    s.q[0] = 1.0f;                     /* identity quaternion */
    s.pitch       =  0.05f;
    s.roll        = -0.10f;
    s.yaw         =  1.50f;
    s.heading_deg =  85.9f;
    s.rate_of_turn  = 2.3f;
    s.ts_wall_ns    = 1715000000000000000ULL;
    s.ts_tai_ns     = 1715000000037000000ULL;
    s.ts_chip_ticks = 40000u;
    s.anchor_gen    = 3u;
    s.imu_seq       = 12345u;
    s.flags = FLAG_FUSION_CONVERGED | FLAG_ACCEL_CAL | FLAG_GYRO_CAL;
    s.cov[0] = 1e-4f; s.cov[4] = 2e-4f; s.cov[8] = 3e-4f;
    return s;
}

static mag_sample_t make_mag(void)
{
    mag_sample_t m;
    memset(&m, 0, sizeof(m));
    m.field[0] = 20.0f;
    m.field[1] =  5.0f;
    m.field[2] = -42.0f;
    m.field_raw[0] = 21.0f;   /* slightly different from calibrated */
    m.field_raw[1] =  6.0f;
    m.field_raw[2] = -43.0f;
    m.wall_ns   = 1715000000000000000ULL - 1000u;
    m.valid     = true;
    return m;
}

static imu_sample_t make_imu(void)
{
    imu_sample_t s;
    memset(&s, 0, sizeof(s));
    s.accel[0] = 0.12f;     s.accel[1] = 0.34f;     s.accel[2] = 9.80f;
    s.accel_raw[0] = 0.13f; s.accel_raw[1] = 0.35f; s.accel_raw[2] = 9.81f;
    s.gyro[0]  = 0.01f; s.gyro[1]  = 0.02f; s.gyro[2]  = 0.005f;
    s.temp_c   = 25.3f;
    s.seq      = 1000u;
    s.chip_ts  = 40000u;
    return s;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

/* IEEE 802.3 CRC32 test vector: CRC32("123456789") == 0xCBF43926 */
static void test_crc32_ieee_vector(void)
{
    begin("test_crc32_ieee_vector");
    int fb = g_fail;
    uint32_t crc = ref_crc32((const uint8_t *)"123456789", 9);
    EXPECT(crc == 0xCBF43926u, "CRC32(\"123456789\") == 0xCBF43926");
    end(fb);
}

static void test_packet_size(void)
{
    begin("test_packet_size");
    int fb = g_fail;
    EXPECT(sizeof(imu_packet_t) == 288, "imu_packet_t is exactly 288 bytes");
    EXPECT(offsetof(imu_packet_t, crc32) == 284, "crc32 field at offset 284");
    /* v18: the two new fields sit between nis_mag and the CRC, so every
     * offset asserted below is unmoved — that is the point of appending. */
    EXPECT(offsetof(imu_packet_t, flags_ext)        == 272, "flags_ext at offset 272");
    EXPECT(offsetof(imu_packet_t, reserved)         == 276, "reserved at offset 276");
    EXPECT(sizeof(((imu_packet_t *)0)->reserved)    == 8,   "reserved is 8 bytes");
    EXPECT(offsetof(imu_packet_t, gyro_bias_x)      == 192, "gyro_bias_x at offset 192");
    EXPECT(offsetof(imu_packet_t, gyro_bias_var_x)  == 204, "gyro_bias_var_x at offset 204");
    EXPECT(offsetof(imu_packet_t, heave_rate)       == 216, "heave_rate at offset 216");
    EXPECT(offsetof(imu_packet_t, accel_quiescence) == 220, "accel_quiescence at offset 220");
    EXPECT(offsetof(imu_packet_t, wave_height_m)    == 224, "wave_height_m at offset 224");
    EXPECT(offsetof(imu_packet_t, wave_period_s)    == 228, "wave_period_s at offset 228");
    EXPECT(offsetof(imu_packet_t, roll_period_s)    == 232, "roll_period_s at offset 232");
    EXPECT(offsetof(imu_packet_t, roll_amplitude)   == 236, "roll_amplitude at offset 236");
    EXPECT(offsetof(imu_packet_t, pitch_period_s)   == 240, "pitch_period_s at offset 240");
    EXPECT(offsetof(imu_packet_t, pitch_amplitude)  == 244, "pitch_amplitude at offset 244");
    EXPECT(offsetof(imu_packet_t, mag_anomaly)      == 248, "mag_anomaly at offset 248");
    EXPECT(offsetof(imu_packet_t, mag_residual)     == 252, "mag_residual at offset 252");
    EXPECT(offsetof(imu_packet_t, innov_weight)     == 256, "innov_weight at offset 256");
    EXPECT(offsetof(imu_packet_t, innov_reject)     == 260, "innov_reject at offset 260");
    EXPECT(offsetof(imu_packet_t, nis_accel)        == 264, "nis_accel at offset 264");
    EXPECT(offsetof(imu_packet_t, nis_mag)          == 268, "nis_mag at offset 268");
    end(fb);
}

static void test_magic_version(void)
{
    begin("test_magic_version");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    EXPECT(pkt.magic   == IMUD_MAGIC,   "magic == 0x494D5544");
    EXPECT(pkt.version == IMUD_VERSION, "version == IMUD_VERSION (17 = v1.7)");
    end(fb);
}

static void test_crc_correct(void)
{
    begin("test_crc_correct");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    uint8_t wire[IMUD_PACKET_BYTES];
    uint32_t crc = packet_encode(wire, &pkt);

    uint32_t expected = ref_crc32(wire, offsetof(imu_packet_t, crc32));
    EXPECT(crc == expected, "CRC32 over wire bytes 0-271 matches the return value");
    EXPECT(rd32(wire + offsetof(imu_packet_t, crc32)) == expected,
           "the same CRC is written at offset 272");
    EXPECT(crc != 0u, "CRC32 is non-zero");
    EXPECT(pkt.crc32 == 0u, "packet_build leaves .crc32 zero — the CRC is the wire's");
    end(fb);
}

static void test_crc_detects_corruption(void)
{
    begin("test_crc_detects_corruption");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");

    /* Flip one bit in byte 10 (well inside the CRC-covered range). */
    ((uint8_t *)&pkt)[10] ^= 0x01u;
    uint32_t recomputed = ref_crc32((const uint8_t *)&pkt, offsetof(imu_packet_t, crc32));
    EXPECT(recomputed != pkt.crc32, "corrupted payload fails CRC check");
    end(fb);
}

static void test_flags_copied(void)
{
    begin("test_flags_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    EXPECT(pkt.flags == s.flags, "flags copied from fused_state_t");
    end(fb);
}

static void test_timestamps_copied(void)
{
    begin("test_timestamps_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    EXPECT(pkt.ts_wall_ns    == s.ts_wall_ns,    "ts_wall_ns");
    EXPECT(pkt.ts_tai_ns     == s.ts_tai_ns,     "ts_tai_ns");
    EXPECT(pkt.ts_chip_ticks == s.ts_chip_ticks, "ts_chip_ticks");
    EXPECT(pkt.anchor_gen    == s.anchor_gen,     "anchor_gen");
    EXPECT(pkt.imu_seq       == s.imu_seq,        "imu_seq");
    end(fb);
}

static void test_v12_fields_copied(void)
{
    begin("test_v12_fields_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    s.bias_gyro[0] = 0.001f;  s.bias_gyro[1] = -0.002f;  s.bias_gyro[2] = 0.003f;
    s.bias_gyro_var[0] = 1e-6f; s.bias_gyro_var[1] = 2e-6f; s.bias_gyro_var[2] = 3e-6f;
    s.heave_rate = 0.25f;
    s.quiescence = 0.01f;
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    EXPECT_NEAR(pkt.gyro_bias_x,      s.bias_gyro[0],     1e-9f,  "gyro_bias_x copied");
    EXPECT_NEAR(pkt.gyro_bias_y,      s.bias_gyro[1],     1e-9f,  "gyro_bias_y copied");
    EXPECT_NEAR(pkt.gyro_bias_z,      s.bias_gyro[2],     1e-9f,  "gyro_bias_z copied");
    EXPECT_NEAR(pkt.gyro_bias_var_x,  s.bias_gyro_var[0], 1e-12f, "gyro_bias_var_x copied");
    EXPECT_NEAR(pkt.heave_rate,       s.heave_rate,       1e-9f,  "heave_rate copied");
    EXPECT_NEAR(pkt.accel_quiescence, s.quiescence,       1e-9f,  "accel_quiescence copied");
    /* v12 diagnostics are body-frame — identical in ENU (NOT coord_frame-rotated) */
    imu_packet_t pkt_enu;
    packet_build(&pkt_enu, &s, &m, &i, &i, "ENU");
    EXPECT_NEAR(pkt_enu.gyro_bias_x, s.bias_gyro[0], 1e-9f, "gyro_bias not ENU-rotated");
    EXPECT_NEAR(pkt_enu.heave_rate,  s.heave_rate,   1e-9f, "heave_rate not ENU-rotated");
    end(fb);
}

static void test_v14_fields_copied(void)
{
    begin("test_v14_fields_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    s.wave_height_m = 1.8f;
    s.wave_period_s = 6.5f;
    s.roll_period_s = 4.2f;
    s.roll_amplitude = 0.15f;
    s.pitch_period_s = 5.3f;
    s.pitch_amplitude = 0.07f;
    s.mag_anomaly = 0.04f;
    s.mag_residual = 0.025f;
    s.innov_weight = 0.87f;
    s.innov_reject = 0.06f;
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    EXPECT_NEAR(pkt.wave_height_m, s.wave_height_m, 1e-9f, "wave_height_m copied");
    EXPECT_NEAR(pkt.wave_period_s, s.wave_period_s, 1e-9f, "wave_period_s copied");
    EXPECT_NEAR(pkt.roll_period_s, s.roll_period_s, 1e-9f, "roll_period_s copied");
    EXPECT_NEAR(pkt.roll_amplitude,  s.roll_amplitude,  1e-9f, "roll_amplitude copied");
    EXPECT_NEAR(pkt.pitch_period_s,  s.pitch_period_s,  1e-9f, "pitch_period_s copied");
    EXPECT_NEAR(pkt.pitch_amplitude, s.pitch_amplitude, 1e-9f, "pitch_amplitude copied");
    EXPECT_NEAR(pkt.mag_anomaly,     s.mag_anomaly,     1e-9f, "mag_anomaly copied");
    EXPECT_NEAR(pkt.mag_residual,    s.mag_residual,    1e-9f, "mag_residual copied");
    EXPECT_NEAR(pkt.innov_weight,    s.innov_weight,    1e-9f, "innov_weight copied");
    EXPECT_NEAR(pkt.innov_reject,    s.innov_reject,    1e-9f, "innov_reject copied");
    /* frame-neutral scalars — identical in ENU */
    imu_packet_t pkt_enu;
    packet_build(&pkt_enu, &s, &m, &i, &i, "ENU");
    EXPECT_NEAR(pkt_enu.wave_height_m, s.wave_height_m, 1e-9f, "wave_height not ENU-rotated");
    end(fb);
}

static void test_ned_vectors_unchanged(void)
{
    begin("test_ned_vectors_unchanged");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");

    EXPECT_NEAR(pkt.accel_x, i.accel[0], 1e-6f, "NED accel_x unchanged");
    EXPECT_NEAR(pkt.accel_y, i.accel[1], 1e-6f, "NED accel_y unchanged");
    EXPECT_NEAR(pkt.accel_z, i.accel[2], 1e-6f, "NED accel_z unchanged");
    EXPECT_NEAR(pkt.gyro_x,  i.gyro[0],  1e-6f, "NED gyro_x unchanged");
    EXPECT_NEAR(pkt.gyro_y,  i.gyro[1],  1e-6f, "NED gyro_y unchanged");
    EXPECT_NEAR(pkt.gyro_z,  i.gyro[2],  1e-6f, "NED gyro_z unchanged");
    EXPECT_NEAR(pkt.mag_x,   m.field[0], 1e-6f, "NED mag_x unchanged");
    EXPECT_NEAR(pkt.mag_y,   m.field[1], 1e-6f, "NED mag_y unchanged");
    EXPECT_NEAR(pkt.mag_z,   m.field[2], 1e-6f, "NED mag_z unchanged");
    end(fb);
}

/*
 * NED→ENU vector: (x, y, z)_ENU = (y_NED, x_NED, -z_NED)
 * accel NED = [0.12, 0.34, 9.80] → ENU = [0.34, 0.12, -9.80]
 */
static void test_enu_vectors_permuted(void)
{
    begin("test_enu_vectors_permuted");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "ENU");

    EXPECT_NEAR(pkt.accel_x,  i.accel[1],  1e-5f, "ENU accel_x = NED accel_y");
    EXPECT_NEAR(pkt.accel_y,  i.accel[0],  1e-5f, "ENU accel_y = NED accel_x");
    EXPECT_NEAR(pkt.accel_z, -i.accel[2],  1e-5f, "ENU accel_z = -NED accel_z");
    EXPECT_NEAR(pkt.gyro_x,   i.gyro[1],   1e-5f, "ENU gyro_x = NED gyro_y");
    EXPECT_NEAR(pkt.gyro_y,   i.gyro[0],   1e-5f, "ENU gyro_y = NED gyro_x");
    EXPECT_NEAR(pkt.gyro_z,  -i.gyro[2],   1e-5f, "ENU gyro_z = -NED gyro_z");
    EXPECT_NEAR(pkt.mag_x,    m.field[1],  1e-5f, "ENU mag_x = NED mag_y");
    EXPECT_NEAR(pkt.mag_y,    m.field[0],  1e-5f, "ENU mag_y = NED mag_x");
    EXPECT_NEAR(pkt.mag_z,   -m.field[2],  1e-5f, "ENU mag_z = -NED mag_z");
    end(fb);
}

/* ENU quaternion must still be a unit quaternion. */
static void test_enu_quat_norm_preserved(void)
{
    begin("test_enu_quat_norm_preserved");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "ENU");

    float norm2 = pkt.quat_w * pkt.quat_w + pkt.quat_x * pkt.quat_x
                + pkt.quat_y * pkt.quat_y + pkt.quat_z * pkt.quat_z;
    EXPECT_NEAR(norm2, 1.0f, 1e-5f, "ENU quaternion has unit norm");
    end(fb);
}

/*
 * Identity quaternion in NED ([1,0,0,0]) converted to ENU should give
 * [0, 1/√2, 1/√2, 0]: the rotation that maps NED basis vectors to ENU.
 * Derivation: q_ENU = [0, 1/√2, 1/√2, 0] ⊗ [1,0,0,0] = [0, 1/√2, 1/√2, 0]
 */
static void test_enu_quat_identity_ned(void)
{
    begin("test_enu_quat_identity_ned");
    int fb = g_fail;

    imu_packet_t pkt;
    fused_state_t s = make_state();  /* q = [1,0,0,0] by construction */
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "ENU");

#define INV_SQRT2_D 0.7071067811865476
    EXPECT_NEAR(pkt.quat_w, 0.0f,        1e-5f, "ENU identity: qw=0");
    EXPECT_NEAR(pkt.quat_x, INV_SQRT2_D, 1e-5f, "ENU identity: qx=1/√2");
    EXPECT_NEAR(pkt.quat_y, INV_SQRT2_D, 1e-5f, "ENU identity: qy=1/√2");
    EXPECT_NEAR(pkt.quat_z, 0.0f,        1e-5f, "ENU identity: qz=0");
#undef INV_SQRT2_D
    end(fb);
}

/* ENU mode CRC must still be valid (computed after transformation). */
static void test_enu_crc_valid(void)
{
    begin("test_enu_crc_valid");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "ENU");
    uint8_t wire[IMUD_PACKET_BYTES];
    uint32_t crc = packet_encode(wire, &pkt);
    uint32_t expected = ref_crc32(wire, offsetof(imu_packet_t, crc32));
    EXPECT(crc == expected, "ENU packet CRC32 is valid");
    end(fb);
}

/* NED and ENU packets should have different CRCs (sensor fields differ). */
static void test_ned_enu_crc_differ(void)
{
    begin("test_ned_enu_crc_differ");
    int fb = g_fail;
    imu_packet_t ned_pkt, enu_pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&ned_pkt, &s, &m, &i, &i, "NED");
    packet_build(&enu_pkt, &s, &m, &i, &i, "ENU");
    uint8_t ned_wire[IMUD_PACKET_BYTES], enu_wire[IMUD_PACKET_BYTES];
    EXPECT(packet_encode(ned_wire, &ned_pkt) != packet_encode(enu_wire, &enu_pkt),
           "NED and ENU packets have different CRCs");
    end(fb);
}

/*
 * Raw gyro fields must match the raw_imu sample passed to packet_build,
 * independently of bias correction applied to the fused gyro fields.
 */
static void test_raw_gyro_copied(void)
{
    begin("test_raw_gyro_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  imu_fused = make_imu();
    imu_sample_t  raw       = make_imu();
    /* Give raw_imu different gyro values so we can distinguish them */
    raw.gyro[0] = 0.11f; raw.gyro[1] = 0.22f; raw.gyro[2] = 0.33f;

    packet_build(&pkt, &s, &m, &imu_fused, &raw, "NED");

    /* Fused (bias-corrected) gyro comes from imu_fused */
    EXPECT_NEAR(pkt.gyro_x, imu_fused.gyro[0], 1e-6f, "gyro_x from fused sample");
    EXPECT_NEAR(pkt.gyro_y, imu_fused.gyro[1], 1e-6f, "gyro_y from fused sample");
    EXPECT_NEAR(pkt.gyro_z, imu_fused.gyro[2], 1e-6f, "gyro_z from fused sample");

    /* Raw gyro comes from raw_imu */
    EXPECT_NEAR(pkt.gyro_raw_x, 0.11f, 1e-6f, "gyro_raw_x from raw sample");
    EXPECT_NEAR(pkt.gyro_raw_y, 0.22f, 1e-6f, "gyro_raw_y from raw sample");
    EXPECT_NEAR(pkt.gyro_raw_z, 0.33f, 1e-6f, "gyro_raw_z from raw sample");
    end(fb);
}

static void test_raw_accel_copied(void)
{
    begin("test_raw_accel_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  raw = make_imu();
    /* Give raw distinct accel_raw to distinguish from calibrated */
    raw.accel_raw[0] = 1.1f; raw.accel_raw[1] = 2.2f; raw.accel_raw[2] = 3.3f;

    packet_build(&pkt, &s, &m, &raw, &raw, "NED");

    EXPECT_NEAR(pkt.accel_raw_x, 1.1f, 1e-6f, "accel_raw_x from raw sample");
    EXPECT_NEAR(pkt.accel_raw_y, 2.2f, 1e-6f, "accel_raw_y from raw sample");
    EXPECT_NEAR(pkt.accel_raw_z, 3.3f, 1e-6f, "accel_raw_z from raw sample");
    end(fb);
}

static void test_raw_mag_copied(void)
{
    begin("test_raw_mag_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();

    packet_build(&pkt, &s, &m, &i, &i, "NED");

    EXPECT_NEAR(pkt.mag_raw_x, m.field_raw[0], 1e-6f, "mag_raw_x from field_raw");
    EXPECT_NEAR(pkt.mag_raw_y, m.field_raw[1], 1e-6f, "mag_raw_y from field_raw");
    EXPECT_NEAR(pkt.mag_raw_z, m.field_raw[2], 1e-6f, "mag_raw_z from field_raw");
    end(fb);
}

static void test_covariance_copied(void)
{
    begin("test_covariance_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    for (int j = 0; j < 9; j++)
        EXPECT_NEAR(pkt.cov[j], s.cov[j], 1e-9f, "covariance element copied");
    end(fb);
}

static void test_temp_copied(void)
{
    begin("test_temp_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    EXPECT_NEAR(pkt.temp_c, i.temp_c, 1e-5f, "temp_c copied from imu sample");
    end(fb);
}

static void test_rate_of_turn_copied(void)
{
    begin("test_rate_of_turn_copied");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    EXPECT_NEAR(pkt.rate_of_turn, s.rate_of_turn, 1e-5f, "rate_of_turn from fused state");
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

/*
 * The packet fuzz target only reaches its decode path through packet_ok(),
 * which checks size, magic and version first. A seed left over from an older
 * wire revision is therefore inert — the fuzzer starts from nothing and
 * nobody notices, which is exactly what happened to the v14/260-byte seed
 * across the v17 bump. This asserts the committed seed still matches the
 * wire the code implements; regenerate with `make fuzz-seeds` when it fails.
 */
static void test_fuzz_seed_matches_wire(void)
{
    begin("test_fuzz_seed_matches_wire");
    int fb = g_fail;

    char path[128];
    snprintf(path, sizeof path,
             "test/fuzz/corpus/packet/valid_v%u.bin", (unsigned)IMUD_VERSION);

    FILE *f = fopen(path, "rb");
    if (!f) {
        /* Also try one level up, so the test works from a build subdir. */
        char alt[160];
        snprintf(alt, sizeof alt, "../%s", path);
        f = fopen(alt, "rb");
    }
    EXPECT(f != NULL,
           "fuzz seed for the current wire version exists "
           "(run `make fuzz-seeds` after a wire change)");
    if (!f) { end(fb); return; }

    unsigned char buf[sizeof(imu_packet_t) + 8];
    size_t n = fread(buf, 1, sizeof buf, f);
    fclose(f);

    EXPECT(n == sizeof(imu_packet_t), "fuzz seed is exactly one packet long");
    if (n != sizeof(imu_packet_t)) { end(fb); return; }

    imu_packet_t pkt;
    packet_decode(&pkt, buf);
    EXPECT(pkt.magic   == IMUD_MAGIC,   "fuzz seed carries the current magic");
    EXPECT(pkt.version == IMUD_VERSION, "fuzz seed carries the current version");
    EXPECT(ref_crc32(buf, offsetof(imu_packet_t, crc32)) == pkt.crc32,
           "fuzz seed CRC is valid for the current layout");
    end(fb);
}

/* ── Wire byte order ─────────────────────────────────────────────────────── */

/*
 * The encoder must emit little-endian regardless of the host's byte order, so
 * these read the buffer with the reference readers above rather than casting.
 */
static void test_wire_is_little_endian(void)
{
    begin("test_wire_is_little_endian");
    int fb = g_fail;
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "NED");

    /* Pin values whose encodings are known by hand. */
    pkt.ts_wall_ns = 0x0102030405060708ull;
    pkt.imu_seq    = 0xDEADBEEFu;
    pkt.accel_x    = 1.0f;                 /* IEEE-754: 0x3F800000 */

    uint8_t w[IMUD_PACKET_BYTES];
    packet_encode(w, &pkt);

    const uint8_t magic_le[4] = { 0x44, 0x55, 0x4D, 0x49 };   /* "IMUD" LSB-first */
    EXPECT(memcmp(w, magic_le, 4) == 0, "magic 0x494D5544 emits 44 55 4D 49");
    EXPECT(w[4] == (IMUD_VERSION & 0xFF) && w[5] == 0,
           "version is LSB-first at offset 4");

    const uint8_t ts_le[8] = { 8, 7, 6, 5, 4, 3, 2, 1 };
    EXPECT(memcmp(w + 8, ts_le, 8) == 0, "uint64 timestamp is LSB-first");

    const uint8_t one_le[4] = { 0x00, 0x00, 0x80, 0x3F };
    EXPECT(memcmp(w + 32, one_le, 4) == 0, "float 1.0f emits 00 00 80 3F");

    EXPECT(rd32(w + 180) == 0xDEADBEEFu, "uint32 imu_seq reads back at offset 180");
    EXPECT(rd16(w + 6) == pkt.flags,     "flags read back at offset 6");
    EXPECT(rd64(w + 8) == pkt.ts_wall_ns, "timestamp reads back at offset 8");
    end(fb);
}

/*
 * Every field must survive wire → struct → wire.  A field missing from
 * packet_encode leaves the 0xAA pre-fill; one missing from packet_decode
 * encodes back as zero.  Either way this fails, which is what keeps the two
 * hand-written lists honest as the packet grows.
 */
static void test_wire_roundtrip_covers_every_field(void)
{
    begin("test_wire_roundtrip_covers_every_field");
    int fb = g_fail;

    /* Distinct byte per position.  Every field from offset 32 on is a 4-byte
     * scalar, so index%4==3 is its high byte on the wire — pinned to 0x3E so
     * no float decodes to NaN or Inf and bit-exact comparison is meaningful. */
    uint8_t in[IMUD_PACKET_BYTES];
    for (size_t k = 0; k < sizeof in; k++)
        in[k] = (k >= 32 && k % 4 == 3) ? 0x3Eu : (uint8_t)(k * 7u + 1u);

    imu_packet_t pkt;
    memset(&pkt, 0, sizeof pkt);
    packet_decode(&pkt, in);

    uint8_t out[IMUD_PACKET_BYTES];
    memset(out, 0xAA, sizeof out);
    uint32_t crc = packet_encode(out, &pkt);

    /* Bytes 0-271 must come back identical; 272-275 are the recomputed CRC. */
    EXPECT(memcmp(in, out, offsetof(imu_packet_t, crc32)) == 0,
           "every wire byte before the CRC survives decode then encode");
    EXPECT(crc == ref_crc32(in, offsetof(imu_packet_t, crc32)),
           "the re-encoded CRC matches the reference over the same bytes");
    EXPECT(pkt.crc32 == rd32(in + offsetof(imu_packet_t, crc32)),
           "packet_decode reads the CRC field too");
    end(fb);
}

/*
 * On a little-endian host the encoded bytes are still the packed struct's own
 * image, so this change is byte-compatible with every deployed consumer and
 * needs no wire-version bump.  Vacuous elsewhere, which is the point: there is
 * nothing to compare against on a big-endian host.
 */
static void test_wire_matches_struct_image_on_le(void)
{
    begin("test_wire_matches_struct_image_on_le");
    int fb = g_fail;
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    imu_packet_t pkt;
    fused_state_t s = make_state();
    mag_sample_t  m = make_mag();
    imu_sample_t  i = make_imu();
    packet_build(&pkt, &s, &m, &i, &i, "ENU");

    uint8_t w[IMUD_PACKET_BYTES];
    uint32_t crc = packet_encode(w, &pkt);
    pkt.crc32 = crc;                        /* the one field build leaves zero */
    EXPECT(memcmp(w, &pkt, sizeof pkt) == 0,
           "wire bytes are the struct image on a little-endian host");
#else
    EXPECT(1, "little-endian only");
#endif
    end(fb);
}

int main(void)
{
    puts("=== imud packet tests ===");
    test_wire_is_little_endian();
    test_wire_roundtrip_covers_every_field();
    test_wire_matches_struct_image_on_le();

    test_crc32_ieee_vector();
    test_packet_size();
    test_fuzz_seed_matches_wire();
    test_magic_version();
    test_crc_correct();
    test_crc_detects_corruption();
    test_flags_copied();
    test_timestamps_copied();
    test_v12_fields_copied();
    test_v14_fields_copied();
    test_ned_vectors_unchanged();
    test_enu_vectors_permuted();
    test_enu_quat_norm_preserved();
    test_enu_quat_identity_ned();
    test_enu_crc_valid();
    test_ned_enu_crc_differ();
    test_raw_gyro_copied();
    test_raw_accel_copied();
    test_raw_mag_copied();
    test_covariance_copied();
    test_temp_copied();
    test_rate_of_turn_copied();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
