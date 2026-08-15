/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * prom_http.h — one in-flight scrape connection, driven by the caller's poll()
 *
 * imud-prometheus is a single poll() loop over the imud stream fd and the
 * HTTP listener. Reading a scrape request with a blocking recv() — even one
 * bounded by SO_RCVTIMEO — stalls that loop, and therefore the stream
 * reader, for as long as the client stays silent.
 *
 * So the accepted fd becomes non-blocking and joins the poll set. This holds
 * exactly one connection at a time: Prometheus scrapes are serial, the page
 * is served from a cache, and a second concurrent scraper is a
 * misconfiguration rather than a load pattern worth queueing for. While one
 * is in flight the caller drops the listener from its poll set, so no accept
 * happens with nowhere to put the result.
 *
 * `now_ms` is a parameter rather than a clock_gettime() inside, so a test can
 * step past a deadline without sleeping.
 */

#ifndef IMUD_PROM_HTTP_H
#define IMUD_PROM_HTTP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Request headers we are willing to buffer. Prometheus sends a GET line plus
 * a handful of headers; anything past this is not a scraper. */
#define PROM_REQ_MAX 1024

typedef struct {
    int      fd;               /* -1 when idle */
    uint64_t deadline_ms;      /* drop the connection at this point */
    size_t   nreq;
    char     req[PROM_REQ_MAX];
} prom_conn_t;

/* Idle state. Does not close anything — call on a zeroed or fresh struct. */
void prom_conn_init(prom_conn_t *c);

static inline bool prom_conn_busy(const prom_conn_t *c) { return c->fd >= 0; }

/*
 * Take ownership of an accepted fd: O_NONBLOCK, close-on-exec, deadline set
 * to now_ms + timeout_ms. Returns 0, or -1 if a connection is already in
 * flight or the fd could not be configured — in both cases `fd` is closed,
 * so the caller never has to clean up after a rejection.
 */
int prom_conn_adopt(prom_conn_t *c, int fd, uint64_t now_ms, int timeout_ms);

/*
 * Read whatever is available.
 *   0  still reading — nothing to do yet
 *   1  request complete: the caller serves the page, then prom_conn_close()
 *  -1  dropped (deadline, EOF, error, or a request past PROM_REQ_MAX); the
 *      fd is already closed and the struct is idle again
 * Safe to call when idle (returns 0), so it can sit unguarded in the loop.
 */
int prom_conn_service(prom_conn_t *c, uint64_t now_ms);

/* Close and return to idle. Idempotent. */
void prom_conn_close(prom_conn_t *c);

/*
 * Clear O_NONBLOCK so the page can go out through the ordinary write-all
 * loop, bounded by the caller's SO_SNDTIMEO. Only the *read* was the stall
 * this file removes — a bounded write was always the design, and a partial
 * write on a non-blocking fd would just be reported as a failed scrape.
 * Returns 0, or -1 (connection closed) if the fd cannot be reconfigured.
 */
int prom_conn_ready_to_write(prom_conn_t *c);

/*
 * Timeout to hand poll(): -1 when idle (the caller's own cadence governs),
 * otherwise the milliseconds left before the deadline, floored at 0. Never
 * returns a value that would let poll() block past the deadline — that is
 * the property that keeps a silent scraper from delaying the stream drain.
 */
int prom_conn_timeout_ms(const prom_conn_t *c, uint64_t now_ms);

/* CLOCK_MONOTONIC in milliseconds; the loop's source for now_ms. */
uint64_t prom_now_ms(void);

#endif /* IMUD_PROM_HTTP_H */
