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
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
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
    uint8_t wire[IMUD_PACKET_BYTES];
    packet_encode(wire, &pkt);

    EXPECT(sizeof(pkt) == 288, "daemon packet is 288 bytes (v18)");
    EXPECT(client_packet_valid(wire, sizeof wire),
           "client accepts daemon-encoded packet (magic+version+CRC)");
    EXPECT(!client_packet_valid(wire, sizeof wire - 1),
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

    unsigned char bytes[288];
    packet_encode(bytes, &pkt);
    bytes[100] ^= 0x01;   /* flip one payload bit */
    EXPECT(!client_packet_valid(bytes, sizeof bytes),
           "client rejects corrupted payload (CRC)");

    packet_encode(bytes, &pkt);
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
     * packet can carry any float — the fuzzer hangs a naive while-loop
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

/* ── Socket path: imud_open / imud_recv / imud_close ─────────────────────── */

int  client_open(int port, const char *dest_addr);
int  client_recv_raw(int fd, void *out);
int  client_set_rcvtimeo(int fd, int secs);
void client_close(int fd);

/* Grab a free UDP port by binding :0 and releasing it. */
static int free_udp_port(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = 0;
    if (bind(fd, (struct sockaddr *)&a, sizeof a) != 0) { close(fd); return -1; }
    socklen_t alen = sizeof a;
    getsockname(fd, (struct sockaddr *)&a, &alen);
    int port = ntohs(a.sin_port);
    close(fd);
    return port;
}

static void send_to_port(int port, const void *buf, size_t len)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = htons((uint16_t)port);
    sendto(fd, buf, len, 0, (struct sockaddr *)&a, sizeof a);
    close(fd);
}

/*
 * The receive path a vendoring consumer runs: open a socket, get exactly the
 * valid packets back, and have the invalid ones dropped silently rather than
 * surfaced.  Datagrams are queued before recv is called, so this cannot race.
 */
static void test_client_socket_roundtrip(void)
{
    begin("test_client_socket_roundtrip");
    int fb = g_fail;

    int port = free_udp_port();
    EXPECT(port > 0, "got a free UDP port");

    int fd = client_open(port, NULL);
    EXPECT(fd >= 0, "imud_open binds the port");
    if (fd < 0) { end(fb); return; }
    client_set_rcvtimeo(fd, 3);

    fused_state_t st; mag_sample_t mag; imu_sample_t imu, raw;
    make_inputs(&st, &mag, &imu, &raw);
    imu_packet_t pkt;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");
    uint8_t wire[IMUD_PACKET_BYTES];
    pkt.crc32 = packet_encode(wire, &pkt);   /* what a decode will read back */

    /* Three datagrams the client must discard, then the real one.  Each
     * exercises a different rejection: wrong size, bad magic, bad CRC. */
    send_to_port(port, "short", 5);

    uint8_t bad_magic[IMUD_PACKET_BYTES];
    memcpy(bad_magic, wire, sizeof bad_magic);
    bad_magic[0] ^= 0xFF;
    send_to_port(port, bad_magic, sizeof bad_magic);

    uint8_t bad_crc[IMUD_PACKET_BYTES];
    memcpy(bad_crc, wire, sizeof bad_crc);
    bad_crc[offsetof(imu_packet_t, crc32)] ^= 0xFFu;
    send_to_port(port, bad_crc, sizeof bad_crc);

    send_to_port(port, wire, sizeof wire);

    imu_packet_t got;
    memset(&got, 0, sizeof got);
    EXPECT(client_recv_raw(fd, &got) == 0, "imud_recv returns the valid packet");
    EXPECT(memcmp(&got, &pkt, sizeof pkt) == 0,
           "and it decodes field-for-field to the one sent");
    EXPECT(client_recv_raw(fd, &got) == -1,
           "imud_recv reports failure once the queue is empty (timeout)");

    client_close(fd);
    end(fb);
}

/* dest_addr only triggers a multicast join for 224.0.0.0/4; anything else is
 * ignored rather than treated as an error. */
static void test_client_open_dest_addr(void)
{
    begin("test_client_open_dest_addr");
    int fb = g_fail;

    int port = free_udp_port();
    int fd = client_open(port, "127.0.0.1");     /* unicast → no join */
    EXPECT(fd >= 0, "unicast dest_addr opens normally");
    if (fd >= 0) client_close(fd);

    port = free_udp_port();
    fd = client_open(port, "not-an-ip");         /* inet_pton fails → no join */
    EXPECT(fd >= 0, "unparseable dest_addr is ignored, not fatal");
    if (fd >= 0) client_close(fd);

    port = free_udp_port();
    fd = client_open(port, "");                  /* empty string → no join */
    EXPECT(fd >= 0, "empty dest_addr opens normally");
    if (fd >= 0) client_close(fd);

    /*
     * No privileged-port check here.  It would assert on the platform rather
     * than on imud: Linux refuses a non-root bind below 1024, macOS allows it
     * for UDP, so the "must fail" assertion passes or fails depending on the
     * host.  imud_open's bind-failure path is one line and not worth an
     * unportable test.
     */
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
    test_client_socket_roundtrip();
    test_client_open_dest_addr();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
