/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_libimud.c — unit tests for the libimud public API (lib/libimud.c)
 *
 * End-to-end over a local AF_UNIX listener and a UDP loopback socket, with
 * packets built by the daemon's real encoder (src/packet.c). Also enforces
 * the imud_data_t append-only ABI contract with offsetof static asserts:
 * if any of those fire, an existing member moved — an ABI break.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "../lib/imud_client.h"   /* wire struct (enables imud_wire decl) */
#include "../lib/imud.h"
#include "../include/packet.h"
#include "../include/types.h"
#include "../include/version.h"   /* IMUD_VERSION_STR — expected lib version */

/* glibc marks write() warn_unused_result; these test writes are asserted. */
static void put(int fd, const void *buf, size_t len)
{
    ssize_t n = write(fd, buf, len);
    assert(n == (ssize_t)len);
}

/* ── ABI guard: existing imud_data_t members must never move ────────────── */

#define OFF(m) offsetof(imud_data_t, m)
_Static_assert(OFF(ts_wall_ns)       ==   0, "ABI: ts_wall_ns moved");
_Static_assert(OFF(ts_tai_ns)        ==   8, "ABI: ts_tai_ns moved");
_Static_assert(OFF(ts_chip_ticks)    ==  16, "ABI: ts_chip_ticks moved");
_Static_assert(OFF(anchor_gen)       ==  20, "ABI: anchor_gen moved");
_Static_assert(OFF(flags)            ==  24, "ABI: flags moved");
_Static_assert(OFF(imu_seq)          ==  28, "ABI: imu_seq moved");
_Static_assert(OFF(accel)            ==  32, "ABI: accel moved");
_Static_assert(OFF(accel_raw)        ==  44, "ABI: accel_raw moved");
_Static_assert(OFF(gyro)             ==  56, "ABI: gyro moved");
_Static_assert(OFF(gyro_raw)         ==  68, "ABI: gyro_raw moved");
_Static_assert(OFF(mag)              ==  80, "ABI: mag moved");
_Static_assert(OFF(mag_raw)          ==  92, "ABI: mag_raw moved");
_Static_assert(OFF(quat)             == 104, "ABI: quat moved");
_Static_assert(OFF(pitch)            == 120, "ABI: pitch moved");
_Static_assert(OFF(roll)             == 124, "ABI: roll moved");
_Static_assert(OFF(yaw)              == 128, "ABI: yaw moved");
_Static_assert(OFF(heading_deg)      == 132, "ABI: heading_deg moved");
_Static_assert(OFF(heading_true_deg) == 136, "ABI: heading_true_deg moved");
_Static_assert(OFF(rate_of_turn)     == 140, "ABI: rate_of_turn moved");
_Static_assert(OFF(temp_c)           == 144, "ABI: temp_c moved");
_Static_assert(OFF(cov)              == 148, "ABI: cov moved");
_Static_assert(OFF(declination_deg)  == 184, "ABI: declination_deg moved");
_Static_assert(OFF(heave_m)          == 188, "ABI: heave_m moved");
_Static_assert(OFF(heave_rate)       == 192, "ABI: heave_rate moved");
_Static_assert(OFF(gyro_bias)        == 196, "ABI: gyro_bias moved");
_Static_assert(OFF(gyro_bias_var)    == 208, "ABI: gyro_bias_var moved");
_Static_assert(OFF(accel_quiescence) == 220, "ABI: accel_quiescence moved");
/* New members append AFTER accel_quiescence; update the assert list when
 * they do (their offsets then become part of the contract too). */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

#define SOCK_PATH "./test_libimud.sock"
#define UDP_PORT  15555

/* A packet with known values, built by the daemon's real encoder. */
static imu_packet_t make_pkt(void)
{
    fused_state_t s; memset(&s, 0, sizeof s);
    s.q[0] = 1.0f;
    s.heading_deg = 90.0f;
    s.declination_deg = 13.2f;
    s.heave_m = 0.42f;
    s.heave_rate = 0.25f;
    s.bias_gyro[0] = 0.001f; s.bias_gyro[1] = -0.002f; s.bias_gyro[2] = 0.003f;
    s.bias_gyro_var[0] = 1e-6f; s.bias_gyro_var[1] = 2e-6f; s.bias_gyro_var[2] = 3e-6f;
    s.quiescence = 0.01f;
    s.flags = FLAG_DECLINATION_VALID | FLAG_HEAVE_VALID;
    s.imu_seq = 7;
    s.ts_wall_ns = 1620307999123000000ULL;

    mag_sample_t m; memset(&m, 0, sizeof m);
    imu_sample_t i; memset(&i, 0, sizeof i);
    i.temp_c = 31.4f; i.accel[2] = -9.81f;

    imu_packet_t pkt;
    packet_build(&pkt, &s, &m, &i, &i, "NED");
    return pkt;
}

/* Local AF_UNIX listener the library connects to. */
static int listen_unix(const char *path)
{
    unlink(path);
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    assert(fd >= 0);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
    assert(bind(fd, (struct sockaddr *)&addr, sizeof addr) == 0);
    assert(listen(fd, 4) == 0);
    return fd;
}

static void test_stream_roundtrip(void)
{
    begin("test_stream_roundtrip");
    int fb = g_fail;

    int srv = listen_unix(SOCK_PATH);
    imud_t *h = imud_connect_stream(SOCK_PATH);
    EXPECT(h != NULL, "connect_stream succeeds");
    int conn = accept(srv, NULL, NULL);
    EXPECT(conn >= 0, "server accepted the client");

    EXPECT(imud_fd(h) >= 0, "imud_fd exposes a valid descriptor");
    EXPECT(imud_wire_version(h) == 0, "wire_version 0 before first packet");

    imu_packet_t pkt = make_pkt();
    put(conn, &pkt, sizeof pkt);
    EXPECT(imud_read(h, 1000) == 0, "read returns 0 on a full frame");

    const imud_data_t *d = imud_data(h);
    EXPECT(d != NULL, "imud_data non-NULL");
    EXPECT(d->ts_wall_ns == 1620307999123000000ULL, "ts_wall_ns copied");
    EXPECT(fabsf(d->quat[0] - 1.0f) < 1e-6f, "quat[0] copied");
    EXPECT(fabsf(d->heading_deg - 90.0f) < 1e-3f, "heading copied");
    EXPECT(fabsf(d->heading_true_deg - 103.2f) < 1e-2f, "heading_true derived (90+13.2)");
    EXPECT(fabsf(d->heave_m - 0.42f) < 1e-3f, "heave copied");
    EXPECT(fabsf(d->heave_rate - 0.25f) < 1e-3f, "heave_rate copied");
    EXPECT(fabsf(d->gyro_bias[1] + 0.002f) < 1e-6f, "gyro_bias[1] copied");
    EXPECT(fabsf(d->gyro_bias_var[2] - 3e-6f) < 1e-9f, "gyro_bias_var[2] copied");
    EXPECT(fabsf(d->accel_quiescence - 0.01f) < 1e-5f, "quiescence copied");
    EXPECT(fabsf(d->temp_c - 31.4f) < 1e-2f, "temp copied");
    EXPECT(d->imu_seq == 7, "imu_seq copied");
    EXPECT((d->flags & IMUD_FLAG_HEAVE_VALID) != 0, "flags carried over");
    EXPECT(imud_wire_version(h) == IMUD_VERSION, "wire_version matches after read");

    const imud_packet_t *w = imud_wire(h);
    EXPECT(w && memcmp(w, &pkt, sizeof pkt) == 0, "imud_wire is the byte-exact packet");

    /* Partial frame: half now, timeout, rest later — reassembled correctly. */
    put(conn, &pkt, 100);
    EXPECT(imud_read(h, 50) == 1, "mid-frame wait returns timeout");
    put(conn, (const unsigned char *)&pkt + 100, sizeof pkt - 100);
    EXPECT(imud_read(h, 1000) == 0, "split frame reassembled");

    /* Corrupt frame: dropped silently, then timeout. */
    imu_packet_t bad = pkt;
    bad.heading_deg = 180.0f;               /* CRC now wrong */
    put(conn, &bad, sizeof bad);
    EXPECT(imud_read(h, 100) == 1, "corrupt frame discarded (timeout follows)");

    /* EOF → -1; reconnect to the still-listening server works. */
    close(conn);
    EXPECT(imud_read(h, 1000) == -1, "EOF reported as connection lost");
    EXPECT(imud_reconnect(h) == 0, "reconnect succeeds");
    conn = accept(srv, NULL, NULL);
    put(conn, &pkt, sizeof pkt);
    EXPECT(imud_read(h, 1000) == 0, "read works after reconnect");

    close(conn);
    close(srv);
    unlink(SOCK_PATH);
    imud_free(h);
    end(fb);
}

static void test_udp_roundtrip(void)
{
    begin("test_udp_roundtrip");
    int fb = g_fail;

    imud_t *h = imud_connect_udp(UDP_PORT, NULL);
    EXPECT(h != NULL, "connect_udp succeeds");

    int tx = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in to;
    memset(&to, 0, sizeof to);
    to.sin_family = AF_INET;
    to.sin_port   = htons(UDP_PORT);
    inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);

    /* Garbage and short datagrams are discarded (timeout). */
    sendto(tx, "junk", 4, 0, (struct sockaddr *)&to, sizeof to);
    EXPECT(imud_read(h, 100) == 1, "short datagram discarded");

    imu_packet_t pkt = make_pkt();
    sendto(tx, &pkt, sizeof pkt, 0, (struct sockaddr *)&to, sizeof to);
    EXPECT(imud_read(h, 1000) == 0, "valid datagram received");
    EXPECT(fabsf(imud_data(h)->heading_deg - 90.0f) < 1e-3f, "payload decoded");

    EXPECT(imud_read(h, 50) == 1, "no data → timeout");

    close(tx);
    imud_free(h);
    end(fb);
}

static void test_misc(void)
{
    begin("test_misc");
    int fb = g_fail;

    EXPECT(strcmp(imud_lib_version(), IMUD_VERSION_STR) == 0,
           "lib version matches include/version.h");
    EXPECT(imud_data(NULL) == NULL, "imud_data(NULL) is NULL");
    EXPECT(imud_fd(NULL) == -1, "imud_fd(NULL) is -1");
    EXPECT(imud_read(NULL, 0) == -1, "imud_read(NULL) is -1");
    imud_free(NULL);                        /* must be a no-op */
    EXPECT(imud_connect_stream("/nonexistent/imud.sock") == NULL,
           "connect to missing socket fails cleanly");
    end(fb);
}

int main(void)
{
    puts("=== imud libimud API tests ===");
    test_stream_roundtrip();
    test_udp_roundtrip();
    test_misc();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
