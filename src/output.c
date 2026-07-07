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
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
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
/* Best-effort on non-Linux dev machines (the Pi uses the real syscall):
 * TIMER_ABSTIME deadlines must be converted to a relative sleep, otherwise
 * nanosleep() would treat the absolute timestamp as a duration. */
static int clock_nanosleep(clockid_t clk, int flags,
                           const struct timespec *req, struct timespec *rem)
{
    (void)rem;
    if (flags & TIMER_ABSTIME) {
        struct timespec now, d;
        clock_gettime(clk, &now);
        d.tv_sec  = req->tv_sec  - now.tv_sec;
        d.tv_nsec = req->tv_nsec - now.tv_nsec;
        if (d.tv_nsec < 0) { d.tv_sec--; d.tv_nsec += 1000000000L; }
        if (d.tv_sec < 0) return 0;       /* deadline already passed */
        return nanosleep(&d, NULL);
    }
    return nanosleep(req, NULL);
}
#endif

/* ── Context ─────────────────────────────────────────────────────────────── */

/* Max simultaneous AF_UNIX stream subscribers. */
#define STREAM_MAX_CLIENTS 8

struct out_ctx {
    const imud_config_t *cfg;
    imu_ctx_t           *imu;

    int                  nmea_fd;
    int                  hirate_fd;
    int                  json_fd;
    struct sockaddr_in   nmea_dest;
    struct sockaddr_in   hirate_dest;
    struct sockaddr_in   json_dest;

    /* AF_UNIX subscription stream — listen fd plus connected subscribers.
     * Only stream_out_thread touches clients[]/nclients after startup. */
    int                  stream_fd;
    int                  stream_clients[STREAM_MAX_CLIENTS];
    int                  stream_nclients;

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
        /* Re-derive each tick so a SIGHUP rate_hz change takes effect
         * immediately ([hot] in imud.conf(5)); single-word load, same
         * lockless pattern as the fusion params. */
        period_ns = (ctx->cfg->nmea_rate_hz > 0)
            ? 1000000000L / ctx->cfg->nmea_rate_hz
            : 100000000L;
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
        /* Re-derive each tick — SIGHUP rate_hz is [hot]. */
        period_ns = (ctx->cfg->highrate_rate_hz > 0)
            ? 1000000000L / ctx->cfg->highrate_rate_hz
            : 2000000L;
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
        /* Re-derive each tick — SIGHUP rate_hz is [hot]. */
        period_ns = (ctx->cfg->json_rate_hz > 0)
            ? 1000000000L / ctx->cfg->json_rate_hz
            : 10000000L;
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
            "\"heave_m\":%.2f,"
            "\"quat\":[%.6f,%.6f,%.6f,%.6f],"
            "\"gyro_bias\":[%.6f,%.6f,%.6f],"
            "\"cov_trace\":%.4e,"
            "\"flags\":%u",
            ts_s,
            state.heading_deg, pitch_deg, roll_deg, state.rate_of_turn,
            state.heave_m,
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

/* ── stream_out_thread ───────────────────────────────────────────────────── */

/*
 * Local AF_UNIX subscription stream (Stream D) — the "third tier".
 *
 * Same 192-byte binary packets as the UDP high-rate stream, but over a
 * SOCK_STREAM socket: no datagram loss for same-host consumers, and clients
 * subscribe by connecting instead of listening on a port.  The fixed packet
 * size plus magic/CRC make the stream self-framing — read 192 bytes at a
 * time and validate exactly as with UDP (lib/imud_client.h works unchanged
 * on chunks read from this socket).
 *
 * Slow-consumer policy: sends are non-blocking. A full kernel buffer
 * (EAGAIN) drops that packet for that subscriber; a PARTIAL write would
 * corrupt framing, so it disconnects the subscriber instead. Consumers are
 * expected to drain at rate_hz — anything that can't keep up gets packet
 * gaps (detectable via imu_seq), never a stalled daemon.
 */
void *stream_out_thread(void *arg)
{
    out_ctx_t *ctx = arg;
    imu_packet_t  pkt;
    fused_state_t state;
    mag_sample_t  mag;
    imu_sample_t  imu;

    long period_ns = (ctx->cfg->stream_rate_hz > 0)
        ? 1000000000L / ctx->cfg->stream_rate_hz
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
        /* Re-derive each tick — SIGHUP rate_hz is [hot]. */
        period_ns = (ctx->cfg->stream_rate_hz > 0)
            ? 1000000000L / ctx->cfg->stream_rate_hz
            : 10000000L;
        ts_add_ns(&next, period_ns);

        /* Accept pending subscribers (listen fd is non-blocking). */
        for (;;) {
            int c = accept(ctx->stream_fd, NULL, NULL);
            if (c < 0) break;
            APPLY_CLOEXEC(c);
            fcntl(c, F_SETFL, O_NONBLOCK);
            if (ctx->stream_nclients >= STREAM_MAX_CLIENTS) {
                fprintf(stderr, "[stream_out] subscriber limit (%d) reached "
                        "— rejecting\n", STREAM_MAX_CLIENTS);
                close(c);
            } else {
                ctx->stream_clients[ctx->stream_nclients++] = c;
                fprintf(stderr, "[stream_out] subscriber connected (%d/%d)\n",
                        ctx->stream_nclients, STREAM_MAX_CLIENTS);
            }
        }

        if (!imu_ctx_is_settled(ctx->imu)) continue;
        if (ctx->stream_nclients == 0)     continue;

        imu_sample_t raw_imu;
        imu_get_state(ctx->imu, &state, &mag, &imu, &raw_imu);
        packet_build(&pkt, &state, &mag, &imu, &raw_imu,
                     ctx->cfg->highrate_coord_frame);

        for (int i = 0; i < ctx->stream_nclients; ) {
            ssize_t s = send(ctx->stream_clients[i], &pkt, sizeof(pkt),
                             MSG_NOSIGNAL);
            if (s == (ssize_t)sizeof(pkt)) { i++; continue; }
            if (s < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                i++;            /* buffer full: drop this packet, keep client */
                continue;
            }
            /* Error, hangup, or partial write (would corrupt framing). */
            close(ctx->stream_clients[i]);
            ctx->stream_clients[i] =
                ctx->stream_clients[--ctx->stream_nclients];
            fprintf(stderr, "[stream_out] subscriber disconnected (%d/%d)\n",
                    ctx->stream_nclients, STREAM_MAX_CLIENTS);
        }
    }

    for (int i = 0; i < ctx->stream_nclients; i++)
        close(ctx->stream_clients[i]);
    ctx->stream_nclients = 0;
    return NULL;
}

/*
 * Open the AF_UNIX listen socket for the subscription stream.
 * Unlinks a stale socket first; mode 0660 to match the status socket.
 */
static int open_stream_listener(const char *path)
{
    struct sockaddr_un addr;
    if (strlen(path) >= sizeof(addr.sun_path)) {
        fprintf(stderr, "[output] stream socket path too long: %s\n", path);
        return -1;
    }

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        fprintf(stderr, "[output] stream socket(): %s\n", strerror(errno));
        return -1;
    }
    APPLY_CLOEXEC(fd);

    unlink(path);   /* remove stale socket from a previous run */
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[output] stream bind(%s): %s\n", path, strerror(errno));
        close(fd); return -1;
    }
    chmod(path, 0660);
    if (listen(fd, 4) < 0) {
        fprintf(stderr, "[output] stream listen(): %s\n", strerror(errno));
        close(fd); unlink(path); return -1;
    }
    /* Non-blocking so stream_out_thread can poll accept() each tick. */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    return fd;
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
    ctx->stream_fd = -1;

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

    if (cfg->stream_enabled) {
        ctx->stream_fd = open_stream_listener(cfg->stream_socket);
        if (ctx->stream_fd < 0) goto fail;
        fprintf(stderr, "[output] stream AF_UNIX → %s (%d Hz, max %d subscribers)\n",
                cfg->stream_socket, cfg->stream_rate_hz, STREAM_MAX_CLIENTS);
    }

    *ctx_out = ctx;
    return 0;

fail:
    if (ctx->nmea_fd   >= 0) close(ctx->nmea_fd);
    if (ctx->hirate_fd >= 0) close(ctx->hirate_fd);
    if (ctx->json_fd   >= 0) close(ctx->json_fd);
    if (ctx->stream_fd >= 0) { close(ctx->stream_fd); unlink(cfg->stream_socket); }
    free(ctx);
    return -1;
}

void out_ctx_send_shutdown(out_ctx_t *ctx)
{
    if (!ctx) return;
    if (ctx->hirate_fd < 0 && ctx->stream_nclients == 0) return;

    fused_state_t state;
    mag_sample_t  mag;
    imu_sample_t  imu_s;
    imu_sample_t raw_imu_s;
    imu_get_state(ctx->imu, &state, &mag, &imu_s, &raw_imu_s);
    state.flags |= FLAG_SHUTDOWN;

    imu_packet_t pkt;
    packet_build(&pkt, &state, &mag, &imu_s, &raw_imu_s, ctx->cfg->highrate_coord_frame);
    if (ctx->hirate_fd >= 0)
        sendto(ctx->hirate_fd, &pkt, sizeof(pkt), 0,
               (struct sockaddr *)&ctx->hirate_dest,
               sizeof(ctx->hirate_dest));

    /* Best-effort final packet to stream subscribers; stream_out_thread is
     * still parked in clock_nanosleep at this point, so the client list is
     * stable (it's joined after out_ctx_stop). */
    for (int i = 0; i < ctx->stream_nclients; i++)
        send(ctx->stream_clients[i], &pkt, sizeof(pkt), MSG_NOSIGNAL);
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
    if (ctx->stream_fd >= 0) {
        close(ctx->stream_fd);
        unlink(ctx->cfg->stream_socket);
    }
    free(ctx);
}
