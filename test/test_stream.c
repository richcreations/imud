/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_stream.c — end-to-end test of the AF_UNIX subscription stream
 *
 * Links the real output.c (plus nmea.c/packet.c/config.c) with stubbed
 * imu_get_state()/imu_ctx_is_settled(), starts a real stream_out_thread,
 * connects as a subscriber over the Unix socket, and verifies:
 *   - the listener socket is created and accepts connections
 *   - packets arrive as exact 192-byte frames with correct magic/version
 *   - packet content reflects the fused state (heading roundtrip)
 *   - a disconnected subscriber is pruned and a new one can join
 *   - clean thread stop and socket unlink
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <math.h>
#include "../include/types.h"
#include "../include/config.h"
#include "../include/output.h"

#define TEST_SOCK "/tmp/imud_test_stream.sock"

/* ── Stubs so output.c links without imu.c ───────────────────────────────── */

bool imu_ctx_is_settled(imu_ctx_t *ctx)
{
    (void)ctx;
    return true;
}

void imu_get_state(imu_ctx_t *ctx, fused_state_t *state_out,
                   mag_sample_t *mag_out, imu_sample_t *imu_out,
                   imu_sample_t *raw_imu_out)
{
    (void)ctx;
    if (state_out) {
        memset(state_out, 0, sizeof *state_out);
        state_out->q[0]        = 1.0f;
        state_out->heading_deg = 123.4f;
        state_out->flags       = FLAG_FUSION_CONVERGED;
        state_out->ts_wall_ns  = 1751800000000000000ULL;
    }
    if (mag_out)     memset(mag_out,     0, sizeof *mag_out);
    if (imu_out)     memset(imu_out,     0, sizeof *imu_out);
    if (raw_imu_out) memset(raw_imu_out, 0, sizeof *raw_imu_out);
}

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static int connect_subscriber(void)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, TEST_SOCK, sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/* Read exactly one 192-byte frame; returns 0 on success. */
static int read_frame(int fd, unsigned char *frame)
{
    size_t got = 0;
    while (got < 192) {
        ssize_t n = read(fd, frame + got, 192 - got);
        if (n <= 0) return -1;   /* timeout, EOF, or error */
        got += (size_t)n;
    }
    return 0;
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud stream socket tests ===");
    int fb = g_fail;
    printf("%-52s", "test_stream_end_to_end");
    fflush(stdout);

    imud_config_t cfg;
    config_defaults(&cfg);
    cfg.nmea_enabled     = false;
    cfg.highrate_enabled = false;
    cfg.json_enabled     = false;
    cfg.stream_enabled   = true;
    cfg.stream_rate_hz   = 200;   /* fast so the test finishes quickly */
    snprintf(cfg.stream_socket, sizeof cfg.stream_socket, TEST_SOCK);

    unlink(TEST_SOCK);
    out_ctx_t *out = NULL;
    EXPECT(out_ctx_open(&out, &cfg, (imu_ctx_t *)0x1) == 0,
           "out_ctx_open creates the stream listener");

    pthread_t tid;
    EXPECT(pthread_create(&tid, NULL, stream_out_thread, out) == 0,
           "stream_out_thread starts");

    /* Subscribe and read three consecutive frames. */
    int sub = connect_subscriber();
    EXPECT(sub >= 0, "subscriber connects to the Unix socket");

    unsigned char frame[192];
    int frames_ok = 0;
    for (int i = 0; i < 3; i++) {
        if (read_frame(sub, frame) != 0) break;
        uint32_t magic;   memcpy(&magic,   frame,     4);
        uint16_t version; memcpy(&version, frame + 4, 2);
        float heading;
        memcpy(&heading, frame + offsetof(imu_packet_t, heading_deg), 4);
        if (magic == IMUD_MAGIC && version == IMUD_VERSION &&
            fabsf(heading - 123.4f) < 1e-4f)
            frames_ok++;
    }
    EXPECT(frames_ok == 3, "three exact 192-byte frames with correct "
                           "magic/version/heading");

    /* Drop the subscriber; the thread must prune it and accept a new one. */
    close(sub);
    usleep(100 * 1000);
    int sub2 = connect_subscriber();
    EXPECT(sub2 >= 0, "second subscriber connects after first disconnects");
    EXPECT(read_frame(sub2, frame) == 0, "second subscriber receives a frame");
    close(sub2);

    /* Clean stop. */
    out_ctx_stop(out);
    EXPECT(pthread_join(tid, NULL) == 0, "stream_out_thread joins cleanly");
    out_ctx_free(out);
    EXPECT(access(TEST_SOCK, F_OK) != 0, "socket path unlinked on free");

    puts(g_fail == fb ? "OK" : "FAIL");
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
