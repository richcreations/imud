/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * output.c — UDP output threads for imud (§7, §8)
 *
 * Security posture:
 *   - inet_pton() for all address parsing — no getaddrinfo(), no DNS
 *   - SOCK_DGRAM | SOCK_CLOEXEC on every socket
 *   - SO_BROADCAST set (harmless for unicast; required for .255 destinations)
 *   - No bind() on output sockets — kernel assigns ephemeral source port
 *   - No recv() / recvfrom() on output sockets
 *   - sendto() failures are logged and counted but never abort the daemon
 *   - Fixed stack buffers only; no malloc in the hot path
 */

/* Linux enables SOCK_CLOEXEC and clock_nanosleep via _GNU_SOURCE. */
#ifdef __linux__
# define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "output.h"
#include "nmea.h"
#include "packet.h"

/* ── Portability stubs (Linux-only features used at runtime on Pi) ───────── */

#ifndef SOCK_CLOEXEC
/* macOS: set close-on-exec via fcntl after socket creation. */
# define SOCK_CLOEXEC 0
# include <fcntl.h>
# define APPLY_CLOEXEC(fd) fcntl((fd), F_SETFD, FD_CLOEXEC)
#else
# define APPLY_CLOEXEC(fd) (0)
#endif

#ifndef TIMER_ABSTIME
# define TIMER_ABSTIME 1
static int clock_nanosleep(clockid_t clk, int flags,
                           const struct timespec *req, struct timespec *rem)
{
    (void)clk; (void)flags; (void)rem;
    return nanosleep(req, NULL); /* best-effort on non-Linux */
}
#endif

/* ── Context ─────────────────────────────────────────────────────────────── */

struct out_ctx {
    const imud_config_t *cfg;
    imu_ctx_t           *imu;

    int                  nmea_fd;
    int                  hirate_fd;
    int                  json_fd;
    struct sockaddr_in   nmea_dest;
    struct sockaddr_in   hirate_dest;
    struct sockaddr_in   json_dest;

    /* Send-error counts (stats only; best-effort, no lock). */
    uint64_t             nmea_errors;
    uint64_t             hirate_errors;
    uint64_t             json_errors;

    volatile int         stop;
};

/* ── Socket helpers ──────────────────────────────────────────────────────── */

/*
 * Open a UDP output socket aimed at dest_ip:port.
 * dest_ip MUST be a numeric IPv4 address (validated via inet_pton).
 *
 * Multicast addresses (224.0.0.0/4): sets IP_MULTICAST_TTL=1 (link-local,
 * never traverses a router) and IP_MULTICAST_LOOP=1 (same-host consumers
 * receive the packets alongside LAN consumers).  No SO_BROADCAST needed.
 *
 * Broadcast/unicast addresses: sets SO_BROADCAST (harmless for unicast).
 *
 * Returns fd on success, -1 on error.
 */
static int open_udp_out(const char *dest_ip, int port,
                        struct sockaddr_in *dest_out)
{
    struct in_addr addr;
    if (inet_pton(AF_INET, dest_ip, &addr) <= 0) {
        fprintf(stderr, "[output] invalid destination address '%s' "
                "(must be a numeric IPv4 address, not a hostname)\n", dest_ip);
        return -1;
    }

    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        fprintf(stderr, "[output] socket(): %s\n", strerror(errno));
        return -1;
    }
    APPLY_CLOEXEC(fd);

    if (IN_MULTICAST(ntohl(addr.s_addr))) {
        /* Multicast: link-local TTL, loopback on so same-host consumers work. */
        uint8_t ttl = 1;
        if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL,
                       &ttl, sizeof(ttl)) < 0) {
            fprintf(stderr, "[output] IP_MULTICAST_TTL: %s\n", strerror(errno));
            close(fd); return -1;
        }
        uint8_t loop = 1;
        if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP,
                       &loop, sizeof(loop)) < 0) {
            fprintf(stderr, "[output] IP_MULTICAST_LOOP: %s\n", strerror(errno));
            close(fd); return -1;
        }
    } else {
        /* Broadcast or unicast. */
        int yes = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) < 0) {
            fprintf(stderr, "[output] SO_BROADCAST: %s\n", strerror(errno));
            close(fd); return -1;
        }
    }

    memset(dest_out, 0, sizeof(*dest_out));
    dest_out->sin_family      = AF_INET;
    dest_out->sin_port        = htons((uint16_t)port);
    dest_out->sin_addr        = addr;

    return fd;
}

/* ── Timing helper ───────────────────────────────────────────────────────── */

/* Advance abs timespec by period_ns nanoseconds. */
static void ts_add_ns(struct timespec *ts, long period_ns)
{
    ts->tv_nsec += period_ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

/* ── nmea_out_thread ─────────────────────────────────────────────────────── */

void *nmea_out_thread(void *arg)
{
    out_ctx_t *ctx = arg;
    char        buf[NMEA_BUF_MIN];
    fused_state_t state;

    long period_ns = (ctx->cfg->nmea_rate_hz > 0)
        ? 1000000000L / ctx->cfg->nmea_rate_hz
        : 100000000L;  /* 10 Hz fallback if misconfigured */

    /* Align first deadline to the next whole period boundary. */
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = (next.tv_nsec / period_ns + 1) * period_ns;
    while (next.tv_nsec >= 1000000000L) {
        next.tv_sec++;
        next.tv_nsec -= 1000000000L;
    }

    while (!ctx->stop) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        ts_add_ns(&next, period_ns);

        if (!imu_ctx_is_settled(ctx->imu)) continue;

        imu_get_state(ctx->imu, &state, NULL, NULL, NULL);

        int n = nmea_encode(buf, sizeof(buf), &state);
        if (n <= 0) continue;

        ssize_t sent = sendto(ctx->nmea_fd, buf, (size_t)n, 0,
                              (struct sockaddr *)&ctx->nmea_dest,
                              sizeof(ctx->nmea_dest));
        if (sent < 0) {
            if (ctx->nmea_errors++ == 0)   /* log first occurrence only */
                fprintf(stderr, "[nmea_out] sendto: %s\n", strerror(errno));
        }
    }

    return NULL;
}

/* ── hirate_out_thread ───────────────────────────────────────────────────── */

void *hirate_out_thread(void *arg)
{
    out_ctx_t *ctx = arg;
    imu_packet_t pkt;
    fused_state_t state;
    mag_sample_t  mag;
    imu_sample_t  imu;

    long period_ns = (ctx->cfg->highrate_rate_hz > 0)
        ? 1000000000L / ctx->cfg->highrate_rate_hz
        : 2000000L;   /* 500 Hz fallback if misconfigured */

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = (next.tv_nsec / period_ns + 1) * period_ns;
    while (next.tv_nsec >= 1000000000L) {
        next.tv_sec++;
        next.tv_nsec -= 1000000000L;
    }

    while (!ctx->stop) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        ts_add_ns(&next, period_ns);

        if (!imu_ctx_is_settled(ctx->imu)) continue;

        imu_sample_t raw_imu;
        imu_get_state(ctx->imu, &state, &mag, &imu, &raw_imu);

        packet_build(&pkt, &state, &mag, &imu, &raw_imu,
                     ctx->cfg->highrate_coord_frame);

        ssize_t sent = sendto(ctx->hirate_fd, &pkt, sizeof(pkt), 0,
                              (struct sockaddr *)&ctx->hirate_dest,
                              sizeof(ctx->hirate_dest));
        if (sent < 0) {
            if (ctx->hirate_errors++ == 0)
                fprintf(stderr, "[hirate_out] sendto: %s\n", strerror(errno));
        }
    }

    return NULL;
}

/* ── json_out_thread ─────────────────────────────────────────────────────── */

/*
 * Emits one NDJSON object per datagram at the configured rate.
 * Format: compact single-line JSON, newline-terminated.
 * All angles in degrees; quaternion [w, x, y, z]; timestamps as UNIX seconds.
 */
void *json_out_thread(void *arg)
{
    out_ctx_t    *ctx = arg;
    fused_state_t state;
    char          buf[512];

    long period_ns = (ctx->cfg->json_rate_hz > 0)
        ? 1000000000L / ctx->cfg->json_rate_hz
        : 10000000L;   /* 100 Hz fallback if misconfigured */

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = (next.tv_nsec / period_ns + 1) * period_ns;
    while (next.tv_nsec >= 1000000000L) {
        next.tv_sec++;
        next.tv_nsec -= 1000000000L;
    }

    while (!ctx->stop) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        ts_add_ns(&next, period_ns);

        if (!imu_ctx_is_settled(ctx->imu)) continue;

        imu_get_state(ctx->imu, &state, NULL, NULL, NULL);

        double ts_s      = (double)state.ts_wall_ns * 1e-9;
        float  pitch_deg = state.pitch * (float)(180.0 / M_PI);
        float  roll_deg  = state.roll  * (float)(180.0 / M_PI);
        float  cov_trace = state.cov[0] + state.cov[4] + state.cov[8];

        int n = snprintf(buf, sizeof(buf),
            "{"
            "\"ts\":%.6f,"
            "\"heading_deg\":%.2f,"
            "\"pitch_deg\":%.3f,"
            "\"roll_deg\":%.3f,"
            "\"rot_dpm\":%.2f,"
            "\"quat\":[%.6f,%.6f,%.6f,%.6f],"
            "\"gyro_bias\":[%.6f,%.6f,%.6f],"
            "\"cov_trace\":%.4e,"
            "\"flags\":%u",
            ts_s,
            state.heading_deg, pitch_deg, roll_deg, state.rate_of_turn,
            state.q[0], state.q[1], state.q[2], state.q[3],
            state.bias_gyro[0], state.bias_gyro[1], state.bias_gyro[2],
            cov_trace,
            (unsigned)state.flags);

        if (n <= 0 || n >= (int)sizeof(buf)) continue;

        /* Append optional true heading fields when declination is known. */
        if (state.flags & FLAG_DECLINATION_VALID) {
            float true_hdg = fmodf(state.heading_deg + state.declination_deg + 360.0f,
                                   360.0f);
            int m = snprintf(buf + n, sizeof(buf) - (size_t)n,
                ",\"declination_deg\":%.2f,\"true_heading_deg\":%.2f",
                state.declination_deg, true_hdg);
            if (m > 0 && n + m < (int)sizeof(buf))
                n += m;
        }

        /* Close JSON object. */
        if (n + 3 >= (int)sizeof(buf)) continue;
        buf[n++] = '}';
        buf[n++] = '\n';
        buf[n]   = '\0';

        ssize_t sent = sendto(ctx->json_fd, buf, (size_t)n, 0,
                              (struct sockaddr *)&ctx->json_dest,
                              sizeof(ctx->json_dest));
        if (sent < 0) {
            if (ctx->json_errors++ == 0)
                fprintf(stderr, "[json_out] sendto: %s\n", strerror(errno));
        }
    }

    return NULL;
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

int out_ctx_open(out_ctx_t **ctx_out,
                 const imud_config_t *cfg,
                 imu_ctx_t           *imu)
{
    out_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) { perror("calloc"); return -1; }

    ctx->cfg  = cfg;
    ctx->imu  = imu;
    ctx->nmea_fd   = -1;
    ctx->hirate_fd = -1;
    ctx->json_fd   = -1;

    if (cfg->nmea_enabled) {
        ctx->nmea_fd = open_udp_out(cfg->nmea_dest_addr, cfg->nmea_dest_port,
                                    &ctx->nmea_dest);
        if (ctx->nmea_fd < 0) goto fail;
        fprintf(stderr, "[output] NMEA UDP → %s:%d (%.0f Hz)\n",
                cfg->nmea_dest_addr, cfg->nmea_dest_port,
                (double)cfg->nmea_rate_hz);
    }

    if (cfg->highrate_enabled) {
        ctx->hirate_fd = open_udp_out(cfg->highrate_dest_addr,
                                      cfg->highrate_dest_port,
                                      &ctx->hirate_dest);
        if (ctx->hirate_fd < 0) goto fail;
        fprintf(stderr, "[output] hi-rate UDP → %s:%d (%.0f Hz, %s, %s)\n",
                cfg->highrate_dest_addr, cfg->highrate_dest_port,
                (double)cfg->highrate_rate_hz, cfg->highrate_coord_frame,
                IN_MULTICAST(ntohl(ctx->hirate_dest.sin_addr.s_addr))
                    ? "multicast TTL=1" : "broadcast/unicast");
    }

    if (cfg->json_enabled) {
        ctx->json_fd = open_udp_out(cfg->json_dest_addr, cfg->json_dest_port,
                                    &ctx->json_dest);
        if (ctx->json_fd < 0) goto fail;
        fprintf(stderr, "[output] JSON UDP → %s:%d (%d Hz)\n",
                cfg->json_dest_addr, cfg->json_dest_port, cfg->json_rate_hz);
    }

    *ctx_out = ctx;
    return 0;

fail:
    if (ctx->nmea_fd   >= 0) close(ctx->nmea_fd);
    if (ctx->hirate_fd >= 0) close(ctx->hirate_fd);
    if (ctx->json_fd   >= 0) close(ctx->json_fd);
    free(ctx);
    return -1;
}

void out_ctx_send_shutdown(out_ctx_t *ctx)
{
    if (!ctx || ctx->hirate_fd < 0) return;

    fused_state_t state;
    mag_sample_t  mag;
    imu_sample_t  imu_s;
    imu_sample_t raw_imu_s;
    imu_get_state(ctx->imu, &state, &mag, &imu_s, &raw_imu_s);
    state.flags |= FLAG_SHUTDOWN;

    imu_packet_t pkt;
    packet_build(&pkt, &state, &mag, &imu_s, &raw_imu_s, ctx->cfg->highrate_coord_frame);
    sendto(ctx->hirate_fd, &pkt, sizeof(pkt), 0,
           (struct sockaddr *)&ctx->hirate_dest,
           sizeof(ctx->hirate_dest));
}

void out_ctx_stop(out_ctx_t *ctx)
{
    if (ctx) ctx->stop = 1;
}

void out_ctx_free(out_ctx_t *ctx)
{
    if (!ctx) return;
    if (ctx->nmea_fd   >= 0) close(ctx->nmea_fd);
    if (ctx->hirate_fd >= 0) close(ctx->hirate_fd);
    if (ctx->json_fd   >= 0) close(ctx->json_fd);
    free(ctx);
}
