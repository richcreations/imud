/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * netserv.c — payload-agnostic multi-client TCP broadcast server (netserv.h)
 *
 * Factored from stream_out_thread / open_stream_listener (src/output.c) so
 * the daemon outputs and the bridges share one accept loop and one
 * slow-client policy. Security posture matches output.c: inet_pton only
 * (no DNS), CLOEXEC on every fd, non-blocking sends, no reads from clients.
 */

#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
#endif

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "netserv.h"
#include "log.h"
#include "cloexec.h"

#ifndef MSG_NOSIGNAL
# define MSG_NOSIGNAL 0    /* macOS: callers ignore SIGPIPE instead */
#endif

void netserv_init(netserv_t *s)
{
    memset(s, 0, sizeof *s);
    s->listen_fd = -1;
}

int netserv_open(netserv_t *s, const char *bind_addr, int port,
                 const char *tag)
{
    netserv_init(s);
    s->tag = tag;

    struct in_addr addr;
    if (inet_pton(AF_INET, bind_addr, &addr) <= 0) {
        LOG_E("[%s] invalid bind address '%s' "
                "(must be a numeric IPv4 address, not a hostname)\n",
                tag, bind_addr);
        return -1;
    }

    int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        LOG_E("[%s] socket(): %s\n", tag, strerror(errno));
        return -1;
    }
    APPLY_CLOEXEC(fd);

    int yes = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes) < 0) {
        LOG_E("[%s] SO_REUSEADDR: %s\n", tag, strerror(errno));
        close(fd); return -1;
    }

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_port   = htons((uint16_t)port);
    sa.sin_addr   = addr;
    if (bind(fd, (struct sockaddr *)&sa, sizeof sa) < 0) {
        LOG_E("[%s] bind(%s:%d): %s\n", tag, bind_addr, port,
                strerror(errno));
        close(fd); return -1;
    }
    if (listen(fd, 4) < 0) {
        LOG_E("[%s] listen(): %s\n", tag, strerror(errno));
        close(fd); return -1;
    }
    /* Non-blocking so the owning thread can poll accept() each tick. */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    /* Read back the bound port (meaningful when port 0 was requested). */
    socklen_t slen = sizeof sa;
    if (getsockname(fd, (struct sockaddr *)&sa, &slen) == 0)
        s->port = (int)ntohs(sa.sin_port);
    else
        s->port = port;

    s->listen_fd = fd;
    return 0;
}

void netserv_accept(netserv_t *s)
{
    if (s->listen_fd < 0) return;
    for (;;) {
        int c = ACCEPT_CLOEXEC(s->listen_fd);
        if (c < 0) break;
        fcntl(c, F_SETFL, O_NONBLOCK);
        if (s->nclients >= NETSRV_MAX_CLIENTS) {
            LOG_E("[%s] client limit (%d) reached — rejecting\n",
                    s->tag, NETSRV_MAX_CLIENTS);
            close(c);
        } else {
            s->clients[s->nclients++] = c;
            LOG_I("[%s] client connected (%d/%d)\n",
                    s->tag, s->nclients, NETSRV_MAX_CLIENTS);
        }
    }
}

void netserv_broadcast(netserv_t *s, const void *buf, size_t len)
{
    if (s->listen_fd < 0 || len == 0) return;
    for (int i = 0; i < s->nclients; ) {
        ssize_t n = send(s->clients[i], buf, len, MSG_NOSIGNAL);
        if (n == (ssize_t)len) { i++; continue; }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            i++;            /* buffer full: drop this payload, keep client */
            continue;
        }
        /* Error, hangup, or partial write (would corrupt framing). */
        close(s->clients[i]);
        s->clients[i] = s->clients[--s->nclients];
        LOG_I("[%s] client disconnected (%d/%d)\n",
                s->tag, s->nclients, NETSRV_MAX_CLIENTS);
    }
}

int netserv_nclients(const netserv_t *s)
{
    return s->listen_fd < 0 ? 0 : s->nclients;
}

void netserv_close(netserv_t *s)
{
    if (s->listen_fd < 0) return;
    for (int i = 0; i < s->nclients; i++)
        close(s->clients[i]);
    s->nclients = 0;
    close(s->listen_fd);
    s->listen_fd = -1;
}
