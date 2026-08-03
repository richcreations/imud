/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * prom_http.c — the scrape connection state machine; see include/prom_http.h
 *
 * Split out of prom_main.c so it can be driven directly over a socketpair:
 * the failure this replaces (a silent client stalling the stream reader) is
 * a timing property, and timing properties are only testable when the clock
 * is an argument.
 */

/* The Makefile also passes -D_GNU_SOURCE; guard so a standalone compile
 * still works without redefining it. */
#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
#endif

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>

#include "prom_http.h"
#include "cloexec.h"

uint64_t prom_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

void prom_conn_init(prom_conn_t *c)
{
    c->fd          = -1;
    c->deadline_ms = 0;
    c->nreq        = 0;
    c->req[0]      = '\0';
}

void prom_conn_close(prom_conn_t *c)
{
    if (c->fd >= 0) close(c->fd);
    prom_conn_init(c);
}

int prom_conn_adopt(prom_conn_t *c, int fd, uint64_t now_ms, int timeout_ms)
{
    if (fd < 0) return -1;

    /* One at a time. Closing the newcomer rather than evicting the incumbent
     * keeps a scraper that opens connections faster than it finishes them
     * from starving the one already being served. */
    if (c->fd >= 0) { close(fd); return -1; }

    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0 || fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0) {
        close(fd);
        return -1;
    }
    APPLY_CLOEXEC(fd);

    c->fd          = fd;
    c->deadline_ms = now_ms + (timeout_ms > 0 ? (uint64_t)timeout_ms : 0u);
    c->nreq        = 0;
    c->req[0]      = '\0';
    return 0;
}

int prom_conn_ready_to_write(prom_conn_t *c)
{
    if (c->fd < 0) return -1;
    int fl = fcntl(c->fd, F_GETFL, 0);
    if (fl < 0 || fcntl(c->fd, F_SETFL, fl & ~O_NONBLOCK) < 0) {
        prom_conn_close(c);
        return -1;
    }
    return 0;
}

/* End of an HTTP request head: CRLFCRLF, or LFLF from a hand-typed client. */
static bool request_complete(const char *buf, size_t n)
{
    if (n >= 4 && memcmp(buf + n - 4, "\r\n\r\n", 4) == 0) return true;
    if (n >= 2 && memcmp(buf + n - 2, "\n\n", 2) == 0)     return true;
    return false;
}

int prom_conn_service(prom_conn_t *c, uint64_t now_ms)
{
    if (c->fd < 0) return 0;

    for (;;) {
        size_t room = sizeof c->req - 1 - c->nreq;
        if (room == 0) {
            /* Head larger than any scraper sends: not a client we serve. */
            prom_conn_close(c);
            return -1;
        }

        ssize_t r = recv(c->fd, c->req + c->nreq, room, 0);
        if (r > 0) {
            c->nreq += (size_t)r;
            c->req[c->nreq] = '\0';
            if (request_complete(c->req, c->nreq)) return 1;
            continue;                       /* more may be buffered */
        }
        if (r == 0) {                       /* peer hung up mid-request */
            prom_conn_close(c);
            return -1;
        }
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        prom_conn_close(c);                 /* real error */
        return -1;
    }

    /* Nothing more readable right now — the deadline is the only other way
     * out, and checking it here is what bounds a silent client. */
    if (now_ms >= c->deadline_ms) {
        prom_conn_close(c);
        return -1;
    }
    return 0;
}

int prom_conn_timeout_ms(const prom_conn_t *c, uint64_t now_ms)
{
    if (c->fd < 0) return -1;
    if (now_ms >= c->deadline_ms) return 0;
    uint64_t left = c->deadline_ms - now_ms;
    return left > (uint64_t)INT32_MAX ? INT32_MAX : (int)left;
}
