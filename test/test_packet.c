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
    EXPECT(sizeof(imu_packet_t) == 192, "imu_packet_t is exactly 192 bytes");
    EXPECT(offsetof(imu_packet_t, crc32) == 188, "crc32 field at offset 188");
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
    EXPECT(pkt.version == IMUD_VERSION, "version == 5");
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

    uint32_t expected = ref_crc32((const uint8_t *)&pkt, offsetof(imu_packet_t, crc32));
    EXPECT(pkt.crc32 == expected, "CRC32 over bytes 0-179 matches packet.crc32");
    EXPECT(pkt.crc32 != 0u,      "CRC32 is non-zero");
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
    uint32_t expected = ref_crc32((const uint8_t *)&pkt, offsetof(imu_packet_t, crc32));
    EXPECT(pkt.crc32 == expected, "ENU packet CRC32 is valid");
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
    EXPECT(ned_pkt.crc32 != enu_pkt.crc32, "NED and ENU packets have different CRCs");
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

int main(void)
{
    puts("=== imud packet tests ===");

    test_crc32_ieee_vector();
    test_packet_size();
    test_magic_version();
    test_crc_correct();
    test_crc_detects_corruption();
    test_flags_copied();
    test_timestamps_copied();
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
