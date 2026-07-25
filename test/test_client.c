/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_client.c — wire-format compatibility test: daemon vs client library
 *
 * Builds packets with the daemon's packet_build() (src/packet.c + types.h)
 * and validates/reads the raw bytes through lib/imud_client.h, compiled in a
 * separate translation unit (test_client_impl.c). Guards against the packet
 * definitions drifting apart — size, magic, version, CRC coverage, and field
 * offsets are all exercised end-to-end.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "../include/types.h"
#include "../include/packet.h"

/* Wrappers implemented in test_client_impl.c against lib/imud_client.h. */
bool     client_packet_valid(const void *buf, size_t len);
uint16_t client_flags(const void *buf);
uint64_t client_ts_wall_ns(const void *buf);
float    client_heading(const void *buf);
float    client_declination(const void *buf);
float    client_quat_w(const void *buf);
float    client_mag_x(const void *buf);
float    client_heave(const void *buf);
float    client_wave_height(const void *buf);
float    client_roll_period(const void *buf);
float    client_pitch_period(const void *buf);
float    client_mag_residual(const void *buf);
float    client_innov_weight(const void *buf);
float    client_true_heading(const void *buf);

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabsf((float)(a) - (float)(b)) < (float)(eps), (msg))

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Fixture ─────────────────────────────────────────────────────────────── */

static void make_inputs(fused_state_t *st, mag_sample_t *mag,
                        imu_sample_t *imu, imu_sample_t *raw)
{
    memset(st,  0, sizeof *st);
    memset(mag, 0, sizeof *mag);
    memset(imu, 0, sizeof *imu);
    memset(raw, 0, sizeof *raw);

    st->q[0] = 0.998f; st->q[1] = 0.001f; st->q[2] = -0.054f; st->q[3] = 0.031f;
    st->heading_deg     = 214.7f;
    st->declination_deg = 13.2f;
    st->rate_of_turn    = -6.2f;
    st->flags           = FLAG_MAG_VALID | FLAG_FUSION_CONVERGED |
                          FLAG_DECLINATION_VALID;
    st->ts_wall_ns      = 0x0123456789ABCDEFULL;
    st->heave_m         = 0.87f;
    st->wave_height_m   = 1.75f;
    st->roll_period_s   = 5.6f;
    st->pitch_period_s  = 4.8f;
    st->mag_residual    = 0.02f;
    st->innov_weight    = 0.87f;
    st->innov_reject    = 0.06f;
    st->ts_tai_ns       = st->ts_wall_ns + 37000000000ULL;

    mag->field[0] = 21.5f; mag->field[1] = -3.2f; mag->field[2] = 44.1f;
    imu->accel[2] = -9.81f;
    raw->accel[2] = -9.79f;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_client_accepts_daemon_packet(void)
{
    begin("test_client_accepts_daemon_packet");
    int fb = g_fail;

    fused_state_t st; mag_sample_t mag; imu_sample_t imu, raw;
    make_inputs(&st, &mag, &imu, &raw);
    imu_packet_t pkt;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");

    EXPECT(sizeof(pkt) == 276, "daemon packet is 276 bytes (v17)");
    EXPECT(client_packet_valid(&pkt, sizeof pkt),
           "client accepts daemon-built packet (magic+version+CRC)");
    EXPECT(!client_packet_valid(&pkt, sizeof pkt - 1),
           "client rejects short packet");
    end(fb);
}

static void test_client_rejects_corruption(void)
{
    begin("test_client_rejects_corruption");
    int fb = g_fail;

    fused_state_t st; mag_sample_t mag; imu_sample_t imu, raw;
    make_inputs(&st, &mag, &imu, &raw);
    imu_packet_t pkt;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");

    unsigned char bytes[276];
    memcpy(bytes, &pkt, sizeof bytes);
    bytes[100] ^= 0x01;   /* flip one payload bit */
    EXPECT(!client_packet_valid(bytes, sizeof bytes),
           "client rejects corrupted payload (CRC)");

    memcpy(bytes, &pkt, sizeof bytes);
    bytes[4] ^= 0xFF;     /* mangle version */
    EXPECT(!client_packet_valid(bytes, sizeof bytes),
           "client rejects wrong version");
    end(fb);
}

static void test_client_field_offsets(void)
{
    begin("test_client_field_offsets");
    int fb = g_fail;

    fused_state_t st; mag_sample_t mag; imu_sample_t imu, raw;
    make_inputs(&st, &mag, &imu, &raw);
    imu_packet_t pkt;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");

    EXPECT(client_flags(&pkt) == st.flags,            "flags roundtrip");
    EXPECT(client_ts_wall_ns(&pkt) == st.ts_wall_ns,  "ts_wall_ns roundtrip");
    EXPECT_NEAR(client_heading(&pkt), 214.7f, 1e-4f,  "heading_deg roundtrip");
    EXPECT_NEAR(client_declination(&pkt), 13.2f, 1e-4f, "declination_deg roundtrip");
    EXPECT_NEAR(client_quat_w(&pkt), 0.998f, 1e-4f,   "quat_w roundtrip");
    EXPECT_NEAR(client_mag_x(&pkt), 21.5f, 1e-4f,     "mag_x roundtrip");
    EXPECT_NEAR(client_heave(&pkt), 0.87f, 1e-4f,     "heave_m roundtrip (v1.1)");
    EXPECT_NEAR(client_wave_height(&pkt), 1.75f, 1e-4f, "wave_height_m roundtrip (v14)");
    EXPECT_NEAR(client_roll_period(&pkt), 5.6f, 1e-4f, "roll_period_s roundtrip (v14)");
    EXPECT_NEAR(client_pitch_period(&pkt), 4.8f, 1e-4f, "pitch_period_s roundtrip (v14)");
    EXPECT_NEAR(client_mag_residual(&pkt), 0.02f, 1e-5f, "mag_residual roundtrip (v14)");
    EXPECT_NEAR(client_innov_weight(&pkt), 0.87f, 1e-5f, "innov_weight roundtrip (v17)");
    end(fb);
}

static void test_client_true_heading_helper(void)
{
    begin("test_client_true_heading_helper");
    int fb = g_fail;

    fused_state_t st; mag_sample_t mag; imu_sample_t imu, raw;
    make_inputs(&st, &mag, &imu, &raw);
    imu_packet_t pkt;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");

    /* 214.7 mag + 13.2 E = 227.9 true */
    EXPECT_NEAR(client_true_heading(&pkt), 227.9f, 0.01f,
                "imud_true_heading = mag + declination");

    /* Without the flag the helper must return the -1.0 sentinel. */
    st.flags = FLAG_MAG_VALID;
    st.declination_deg = 0.0f;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");
    EXPECT_NEAR(client_true_heading(&pkt), -1.0f, 1e-6f,
                "helper returns -1.0 without FLAG_DECLINATION_VALID");

    /* Hostile wire values must not hang or leak garbage (a CRC-valid
     * packet can carry any float — the fuzzer hung the old while-loop
     * normalization with Inf/1e38 for 20 minutes). */
    make_inputs(&st, &mag, &imu, &raw);
    st.heading_deg = 3.4e38f;                     /* huge finite */
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");
    EXPECT_NEAR(client_true_heading(&pkt), -1.0f, 1e-6f,
                "huge heading returns the sentinel, promptly");

    st.heading_deg = 1.0f / 0.0f;                 /* +Inf */
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");
    EXPECT_NEAR(client_true_heading(&pkt), -1.0f, 1e-6f,
                "Inf heading returns the sentinel");

    st.heading_deg = 0.0f / 0.0f;                 /* NaN */
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");
    EXPECT_NEAR(client_true_heading(&pkt), -1.0f, 1e-6f,
                "NaN heading returns the sentinel");

    /* wrap case the loop exists for: 359 mag + 5 E = 4 true */
    make_inputs(&st, &mag, &imu, &raw);
    st.heading_deg = 359.0f;
    st.declination_deg = 5.0f;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");
    EXPECT_NEAR(client_true_heading(&pkt), 4.0f, 0.01f,
                "wraparound 359+5 -> 4 still works");
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud client library tests ===");

    test_client_accepts_daemon_packet();
    test_client_rejects_corruption();
    test_client_field_offsets();
    test_client_true_heading_helper();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
