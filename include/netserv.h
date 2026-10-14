/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * netserv.h — payload-agnostic multi-client TCP broadcast server
 *
 * The TCP-listener pattern shared by the daemon outputs ([nmea]/[stream])
 * and the bridges (imud-signalk, imud-mavlink): a non-blocking listen
 * socket polled with a bounded accept loop each tick, and a slow-client
 * send policy lifted from stream_out_thread (src/output.c):
 *
 *   - full send            → keep the client
 *   - EAGAIN/EWOULDBLOCK   → drop THIS payload for that client, keep it
 *                            (consumers see a gap, the daemon never stalls),
 *                            unless it has taken nothing for NETSRV_STALL_NS
 *   - error / hangup /
 *     partial write        → close + swap-remove (a partial write would
 *                            corrupt framing for stream payloads)
 *
 * Single-owner discipline (the 1.5.1 concurrency rule): after netserv_open,
 * exactly one thread calls netserv_accept/netserv_broadcast/netserv_close.
 * There is no locking here on purpose — do not share a netserv_t across
 * threads.
 *
 * All functions tolerate a never-opened or already-closed server (listen_fd
 * == -1): accept and broadcast become no-ops, close is idempotent. Zero out
 * the struct (or use netserv_init) before first use.
 */
#ifndef IMUD_NETSERV_H
#define IMUD_NETSERV_H

#include <stddef.h>
#include <stdint.h>

/* Max simultaneous TCP subscribers per listener (mirrors the AF_UNIX
 * stream's STREAM_MAX_CLIENTS). */
#define NETSRV_MAX_CLIENTS 8

/* A client that has taken no whole payload for this long has stopped reading
 * and is only holding a slot, so it is closed.  The tests build a short one. */
#ifndef NETSRV_STALL_NS
#define NETSRV_STALL_NS 30000000000ULL
#endif

typedef struct {
    int         listen_fd;                   /* -1 when not open */
    int         clients[NETSRV_MAX_CLIENTS];
    uint64_t    taken_ns[NETSRV_MAX_CLIENTS]; /* last whole payload, monotonic */
    int         nclients;
    int         port;                        /* actual bound port (after open) */
    const char *tag;                         /* log prefix, e.g. "nmea_tcp" */
} netserv_t;

/* Mark a netserv_t as not-open so accept/broadcast/close are safe no-ops. */
void netserv_init(netserv_t *s);

/*
 * Open a TCP listener on bind_addr:port. bind_addr MUST be a numeric IPv4
 * address (inet_pton — no DNS, matching the project socket posture); port 0
 * binds an ephemeral port, readable back from s->port. The listen fd is
 * non-blocking + CLOEXEC. Returns 0 on success, -1 on error (logged).
 */
int  netserv_open(netserv_t *s, const char *bind_addr, int port,
                  const char *tag);

/* Accept all pending connections (non-blocking); reject beyond
 * NETSRV_MAX_CLIENTS. Call once per output tick. */
void netserv_accept(netserv_t *s);

/* Send buf to every connected client with the slow-client policy above. */
void netserv_broadcast(netserv_t *s, const void *buf, size_t len);

/*
 * The policy itself, over any array of client fds: sends buf to the first n,
 * closes and swap-removes the ones it drops, keeps taken_ns[] in step, and
 * returns the new count.  The AF_UNIX stream in output.c uses it directly;
 * max and tag only label the log line.
 */
int  netserv_send_each(int *fds, uint64_t *taken_ns, int n, int max,
                       const void *buf, size_t len, const char *tag);

/* The clock taken_ns[] runs on, for a caller stamping a client it accepted. */
uint64_t netserv_now_ns(void);

/* Number of currently connected clients (0 when not open). */
int  netserv_nclients(const netserv_t *s);

/* Close all clients and the listener; idempotent. */
void netserv_close(netserv_t *s);

#endif /* IMUD_NETSERV_H */
