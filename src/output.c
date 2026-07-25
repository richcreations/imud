/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * output.c — output threads for imud (§7, §8)
 *
 * Security posture:
 *   - inet_pton() for all address parsing — no getaddrinfo(), no DNS
 *   - SOCK_CLOEXEC on every socket
 *   - SO_BROADCAST set (harmless for unicast; required for .255 destinations)
 *   - No bind() on UDP output sockets — kernel assigns ephemeral source port
 *   - No recv() / recvfrom() anywhere: the optional TCP listeners
 *     (netserv.c) are broadcast-only, clients are never read from
 *   - send failures are logged and counted but never abort the daemon
 *   - Fixed stack buffers only; no malloc in the hot path
 */

/* Linux enables SOCK_CLOEXEC and clock_nanosleep via _GNU_SOURCE.
 * The Makefile also passes -D_GNU_SOURCE; guard so a standalone compile
 * still works without redefining it. */
#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
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
#include "imu_math.h"    /* ts_add_ns */
#include "nmea.h"
#include "packet.h"
#include "netserv.h"
#include "log.h"

/* ── Portability stubs (Linux-only features used at runtime on Pi) ───────── */

#ifndef SOCK_CLOEXEC
/* macOS: set close-on-exec via fcntl after socket creation. */
# define SOCK_CLOEXEC 0
# include <fcntl.h>
# define APPLY_CLOEXEC(fd) fcntl((fd), F_SETFD, FD_CLOEXEC)
#else
/* Linux: SOCK_CLOEXEC is passed to socket() directly, so this is a no-op.
 * ((void)0) rather than (0) so the call sites aren't bare no-effect statements. */
# define APPLY_CLOEXEC(fd) ((void)0)
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
    struct sockaddr_in   nmea_dest;
    struct sockaddr_in   hirate_dest;

    /* AF_UNIX subscription stream — listen fd plus connected subscribers.
     * Only stream_out_thread touches clients[]/nclients after startup. */
    int                  stream_fd;
    int                  stream_clients[STREAM_MAX_CLIENTS];
    int                  stream_nclients;

    /* Optional TCP listeners (1.6). Single-owner: after the threads start,
     * nmea_tcp is touched only by nmea_out_thread and bin_tcp only by
     * stream_out_thread — out_ctx_free's netserv_close runs post-join. */
    netserv_t            nmea_tcp;   /* [nmea] tcp_* — sentence bursts */
    netserv_t            bin_tcp;    /* [stream] tcp_* — framed packets */

    /* Send-error counts (stats only; best-effort, no lock). */
    uint64_t             nmea_errors;
    uint64_t             hirate_errors;

    /* [hot] output rates: main mutates the shared config on SIGHUP, so the
     * output threads read their own _Atomic copies (via out_ctx_reload)
     * instead of cfg->*_rate_hz — a plain shared int read/write is a C11
     * data race. Immutable fields (addresses, coord_frame) stay on ->cfg. */
    _Atomic int          nmea_rate_hz;
    _Atomic int          highrate_rate_hz;
    _Atomic int          stream_rate_hz;

    _Atomic int          stop;   /* C11-clean cross-thread stop (TSan) */
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
        LOG_E("[output] invalid destination address '%s' "
                "(must be a numeric IPv4 address, not a hostname)\n", dest_ip);
        return -1;
    }

    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        LOG_E("[output] socket(): %s\n", strerror(errno));
        return -1;
    }
    APPLY_CLOEXEC(fd);

    if (IN_MULTICAST(ntohl(addr.s_addr))) {
        /* Multicast: link-local TTL, loopback on so same-host consumers work. */
        uint8_t ttl = 1;
        if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL,
                       &ttl, sizeof(ttl)) < 0) {
            LOG_E("[output] IP_MULTICAST_TTL: %s\n", strerror(errno));
            close(fd); return -1;
        }
        uint8_t loop = 1;
        if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP,
                       &loop, sizeof(loop)) < 0) {
            LOG_E("[output] IP_MULTICAST_LOOP: %s\n", strerror(errno));
            close(fd); return -1;
        }
    } else {
        /* Broadcast or unicast. */
        int yes = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) < 0) {
            LOG_E("[output] SO_BROADCAST: %s\n", strerror(errno));
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

/* Align the first deadline to the next whole period boundary.
 * (ts_add_ns comes from imu_math.h.) */
static void align_next(struct timespec *next, long period_ns)
{
    clock_gettime(CLOCK_MONOTONIC, next);
    next->tv_nsec = (next->tv_nsec / period_ns + 1) * period_ns;
    while (next->tv_nsec >= 1000000000L) {
        next->tv_sec++;
        next->tv_nsec -= 1000000000L;
    }
}

/* ── nmea_out_thread ─────────────────────────────────────────────────────── */

void *nmea_out_thread(void *arg)
{
    out_ctx_t *ctx = arg;
    char        buf[NMEA_BUF_MIN];
    fused_state_t state;

    long period_ns = (ctx->nmea_rate_hz > 0)
        ? 1000000000L / ctx->nmea_rate_hz
        : 100000000L;  /* 10 Hz fallback if misconfigured */

    struct timespec next;
    align_next(&next, period_ns);

    while (!ctx->stop) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        /* Re-derive each tick so a SIGHUP rate_hz change takes effect
         * immediately ([hot] in imud.conf(5)); single-word load, same
         * lockless pattern as the fusion params. */
        period_ns = (ctx->nmea_rate_hz > 0)
            ? 1000000000L / ctx->nmea_rate_hz
            : 100000000L;
        ts_add_ns(&next, period_ns);

        /* Accept TCP subscribers even while settling (no-op if disabled). */
        netserv_accept(&ctx->nmea_tcp);

        if (!imu_ctx_is_settled(ctx->imu)) continue;

        imu_get_state(ctx->imu, &state, NULL, NULL, NULL);

        int n = nmea_encode(buf, sizeof(buf), &state);
        if (n <= 0) continue;

        if (ctx->nmea_fd >= 0) {
            ssize_t sent = sendto(ctx->nmea_fd, buf, (size_t)n, 0,
                                  (struct sockaddr *)&ctx->nmea_dest,
                                  sizeof(ctx->nmea_dest));
            if (sent < 0) {
                ctx->nmea_errors++;
                /* log.c's repeat suppression handles flooding */
                LOG_W("[nmea_out] sendto: %s\n", strerror(errno));
            }
        }

        /* Same sentence burst to connected TCP clients (plotters). */
        netserv_broadcast(&ctx->nmea_tcp, buf, (size_t)n);
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

    long period_ns = (ctx->highrate_rate_hz > 0)
        ? 1000000000L / ctx->highrate_rate_hz
        : 2000000L;   /* 500 Hz fallback if misconfigured */

    struct timespec next;
    align_next(&next, period_ns);

    while (!ctx->stop) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        /* Re-derive each tick — SIGHUP rate_hz is [hot]. */
        period_ns = (ctx->highrate_rate_hz > 0)
            ? 1000000000L / ctx->highrate_rate_hz
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
            ctx->hirate_errors++;
            LOG_W("[hirate_out] sendto: %s\n", strerror(errno));
        }
    }

    return NULL;
}

/* ── stream_out_thread ───────────────────────────────────────────────────── */

/*
 * Local AF_UNIX subscription stream (Stream D) — the "third tier".
 *
 * Same 276-byte binary packets as the UDP high-rate stream, but over a
 * SOCK_STREAM socket: no datagram loss for same-host consumers, and clients
 * subscribe by connecting instead of listening on a port.  The fixed packet
 * size plus magic/CRC make the stream self-framing — read 276 bytes at a
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

    long period_ns = (ctx->stream_rate_hz > 0)
        ? 1000000000L / ctx->stream_rate_hz
        : 10000000L;   /* 100 Hz fallback if misconfigured */

    struct timespec next;
    align_next(&next, period_ns);

    while (!ctx->stop) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        /* Re-derive each tick — SIGHUP rate_hz is [hot]. */
        period_ns = (ctx->stream_rate_hz > 0)
            ? 1000000000L / ctx->stream_rate_hz
            : 10000000L;
        ts_add_ns(&next, period_ns);

        /* Accept pending subscribers (listen fds are non-blocking).
         * stream_fd is -1 when only the TCP listener is enabled. */
        if (ctx->stream_fd >= 0) for (;;) {
            int c = accept(ctx->stream_fd, NULL, NULL);
            if (c < 0) break;
            APPLY_CLOEXEC(c);
            fcntl(c, F_SETFL, O_NONBLOCK);
            if (ctx->stream_nclients >= STREAM_MAX_CLIENTS) {
                LOG_E("[stream_out] subscriber limit (%d) reached "
                        "— rejecting\n", STREAM_MAX_CLIENTS);
                close(c);
            } else {
                ctx->stream_clients[ctx->stream_nclients++] = c;
                LOG_I("[stream_out] subscriber connected (%d/%d)\n",
                        ctx->stream_nclients, STREAM_MAX_CLIENTS);
            }
        }
        netserv_accept(&ctx->bin_tcp);

        if (!imu_ctx_is_settled(ctx->imu)) continue;
        if (ctx->stream_nclients == 0 &&
            netserv_nclients(&ctx->bin_tcp) == 0) continue;

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
            LOG_I("[stream_out] subscriber disconnected (%d/%d)\n",
                    ctx->stream_nclients, STREAM_MAX_CLIENTS);
        }

        /* Same framed packet to TCP subscribers (same slow-client policy). */
        netserv_broadcast(&ctx->bin_tcp, &pkt, sizeof(pkt));
    }

    /* Final shutdown packet to subscribers is sent HERE, by the thread that
     * owns the client arrays (AF_UNIX and TCP both) — never from
     * out_ctx_send_shutdown on the main thread, which would traverse
     * stream_clients[]/bin_tcp concurrently with this thread's
     * accept/disconnect mutations. On loop exit the daemon is stopping
     * (fusion is still alive; it is joined after the output threads), so
     * imu_get_state is valid. */
    if (ctx->stream_nclients > 0 || netserv_nclients(&ctx->bin_tcp) > 0) {
        fused_state_t state;
        mag_sample_t  mag;
        imu_sample_t  imu_s, raw_imu_s;
        imu_get_state(ctx->imu, &state, &mag, &imu_s, &raw_imu_s);
        state.flags |= FLAG_SHUTDOWN;
        imu_packet_t pkt;
        packet_build(&pkt, &state, &mag, &imu_s, &raw_imu_s,
                     ctx->cfg->highrate_coord_frame);
        for (int i = 0; i < ctx->stream_nclients; i++)
            send(ctx->stream_clients[i], &pkt, sizeof(pkt), MSG_NOSIGNAL);
        netserv_broadcast(&ctx->bin_tcp, &pkt, sizeof(pkt));
    }

    for (int i = 0; i < ctx->stream_nclients; i++)
        close(ctx->stream_clients[i]);
    ctx->stream_nclients = 0;
    netserv_close(&ctx->bin_tcp);
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
        LOG_E("[output] stream socket path too long: %s\n", path);
        return -1;
    }

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        LOG_E("[output] stream socket(): %s\n", strerror(errno));
        return -1;
    }
    APPLY_CLOEXEC(fd);

    /* Fail loudly rather than silently binding a truncated path. */
    size_t plen = strlen(path);
    if (plen >= sizeof(addr.sun_path)) {
        LOG_E("[output] stream socket path too long (%zu bytes, max %zu): %s\n",
              plen, sizeof(addr.sun_path) - 1, path);
        close(fd); return -1;
    }

    unlink(path);   /* remove stale socket from a previous run */
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, path, plen);   /* addr is zeroed → NUL-terminated */

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_E("[output] stream bind(%s): %s\n", path, strerror(errno));
        close(fd); return -1;
    }
    chmod(path, 0660);
    if (listen(fd, 4) < 0) {
        LOG_E("[output] stream listen(): %s\n", strerror(errno));
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
    if (!ctx) { LOG_E("[output] calloc: %s\n", strerror(errno)); return -1; }

    ctx->cfg  = cfg;
    ctx->imu  = imu;
    ctx->nmea_rate_hz     = cfg->nmea_rate_hz;
    ctx->highrate_rate_hz = cfg->highrate_rate_hz;
    ctx->stream_rate_hz   = cfg->stream_rate_hz;
    ctx->nmea_fd   = -1;
    ctx->hirate_fd = -1;
    ctx->stream_fd = -1;
    netserv_init(&ctx->nmea_tcp);
    netserv_init(&ctx->bin_tcp);

    if (cfg->nmea_enabled) {
        ctx->nmea_fd = open_udp_out(cfg->nmea_dest_addr, cfg->nmea_dest_port,
                                    &ctx->nmea_dest);
        if (ctx->nmea_fd < 0) goto fail;
        LOG_I("[output] NMEA UDP → %s:%d (%.0f Hz)\n",
                cfg->nmea_dest_addr, cfg->nmea_dest_port,
                (double)cfg->nmea_rate_hz);
    }

    if (cfg->nmea_tcp_enabled) {
        if (netserv_open(&ctx->nmea_tcp, cfg->nmea_tcp_bind_addr,
                         cfg->nmea_tcp_port, "nmea_tcp") < 0) goto fail;
        LOG_I("[output] NMEA TCP listener %s:%d (%.0f Hz, max %d clients)\n",
                cfg->nmea_tcp_bind_addr, ctx->nmea_tcp.port,
                (double)cfg->nmea_rate_hz, NETSRV_MAX_CLIENTS);
    }

    if (cfg->highrate_enabled) {
        ctx->hirate_fd = open_udp_out(cfg->highrate_dest_addr,
                                      cfg->highrate_dest_port,
                                      &ctx->hirate_dest);
        if (ctx->hirate_fd < 0) goto fail;
        LOG_I("[output] hi-rate UDP → %s:%d (%.0f Hz, %s, %s)\n",
                cfg->highrate_dest_addr, cfg->highrate_dest_port,
                (double)cfg->highrate_rate_hz, cfg->highrate_coord_frame,
                IN_MULTICAST(ntohl(ctx->hirate_dest.sin_addr.s_addr))
                    ? "multicast TTL=1" : "broadcast/unicast");
    }

    if (cfg->stream_enabled) {
        ctx->stream_fd = open_stream_listener(cfg->stream_socket);
        if (ctx->stream_fd < 0) goto fail;
        LOG_I("[output] stream AF_UNIX → %s (%d Hz, max %d subscribers)\n",
                cfg->stream_socket, cfg->stream_rate_hz, STREAM_MAX_CLIENTS);
    }

    if (cfg->stream_tcp_enabled) {
        if (netserv_open(&ctx->bin_tcp, cfg->stream_tcp_bind_addr,
                         cfg->stream_tcp_port, "stream_tcp") < 0) goto fail;
        LOG_I("[output] stream TCP listener %s:%d (%d Hz, max %d clients)\n",
                cfg->stream_tcp_bind_addr, ctx->bin_tcp.port,
                cfg->stream_rate_hz, NETSRV_MAX_CLIENTS);
    }

    *ctx_out = ctx;
    return 0;

fail:
    if (ctx->nmea_fd   >= 0) close(ctx->nmea_fd);
    if (ctx->hirate_fd >= 0) close(ctx->hirate_fd);
    if (ctx->stream_fd >= 0) { close(ctx->stream_fd); unlink(cfg->stream_socket); }
    netserv_close(&ctx->nmea_tcp);
    netserv_close(&ctx->bin_tcp);
    free(ctx);
    return -1;
}

/*
 * Emit the final FLAG_SHUTDOWN packet on the connectionless hirate UDP
 * output only. The stream subscribers' copy is sent by stream_out_thread
 * itself on exit (single-owner of the client array) — see stream_out_thread.
 * Called from main before out_ctx_stop, while hirate_out_thread may also be
 * sending; both are plain sendto() on the same fd (kernel-serialized, no
 * shared user state), so that is safe.
 */
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

/* Publish hot-reloaded output rates (SIGHUP). The threads read these
 * atomics each tick; the immutable fields stay on the shared ->cfg. */
void out_ctx_reload(out_ctx_t *ctx, const imud_config_t *cfg)
{
    if (!ctx) return;
    ctx->nmea_rate_hz     = cfg->nmea_rate_hz;
    ctx->highrate_rate_hz = cfg->highrate_rate_hz;
    ctx->stream_rate_hz   = cfg->stream_rate_hz;
}

void out_ctx_free(out_ctx_t *ctx)
{
    if (!ctx) return;
    if (ctx->nmea_fd   >= 0) close(ctx->nmea_fd);
    if (ctx->hirate_fd >= 0) close(ctx->hirate_fd);
    if (ctx->stream_fd >= 0) {
        close(ctx->stream_fd);
        unlink(ctx->cfg->stream_socket);
    }
    /* Runs after the output threads have joined; bin_tcp is usually already
     * closed by stream_out_thread on exit (netserv_close is idempotent).
     * This also covers listeners whose thread never started. */
    netserv_close(&ctx->nmea_tcp);
    netserv_close(&ctx->bin_tcp);
    free(ctx);
}
