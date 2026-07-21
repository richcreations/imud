/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * prom_main.c — imud-prometheus: Prometheus exporter bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket)
 * via libimud and serves the latest fused state as Prometheus text-format
 * gauges over a minimal embedded HTTP responder (GET anything → /metrics
 * page, Connection: close). Pure C, no external dependencies.
 *
 * Prometheus is pull-based, so unlike the push bridges there is no emit
 * rate: a single poll() loop watches the imud fd (refreshing the cached
 * data view on every frame) and the HTTP listener (answering scrapes from
 * the cache). A slow scraper cannot wedge the stream reader — responses
 * are written once with a short send timeout and the connection closed.
 *
 * This is the first bridge built purely on libimud's ABI-stable imud_data_t
 * (no imud_client.h, no wire pinning): it needs no rebuild across wire
 * revisions, only across imud_data_t appends it wants to surface.
 */

/* The Makefile also passes -D_GNU_SOURCE; guard so a standalone compile
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
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include "prom_metrics.h"
#include "../lib/imud.h"        /* libimud: stream connect/read/validate */
#include "bridge.h"             /* shared bridge scaffolding */
#include "sdnotify.h"
#include "config.h"
#include "log.h"

#define METRICS_BUF 8192             /* whole exposition page */

static const bridge_info_t BI = {
    .prog         = "imud-prometheus",
    .tag          = "prom",
    .section      = "imud-prometheus",
    .default_conf = "/etc/imud/imud-prometheus.conf",
    .usage_desc   =
        "  Prometheus exporter: reads imud's stream socket and serves the\n"
        "  latest fused state as text-format gauges on GET /metrics.\n"
        "  Configured by [imud-prometheus] in its own file.\n",
};

/* ── HTTP listener ───────────────────────────────────────────────────────── */

static int open_listener(const char *addr, int port)
{
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags    = AI_PASSIVE;
    if (getaddrinfo(addr, portstr, &hints, &res) != 0 || !res) {
        LOG_E("[prom] cannot resolve listen address '%s'\n", addr);
        return -1;
    }
    int fd = socket(res->ai_family, res->ai_socktype | SOCK_CLOEXEC, 0);
    if (fd < 0) { freeaddrinfo(res); return -1; }
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    if (bind(fd, res->ai_addr, res->ai_addrlen) != 0 || listen(fd, 8) != 0) {
        LOG_E("[prom] bind/listen %s:%d: %s\n", addr, port, strerror(errno));
        freeaddrinfo(res);
        close(fd);
        return -1;
    }
    freeaddrinfo(res);
    return fd;
}

/*
 * Serve one scrape: read (and discard) the request line, write the metrics
 * page, close. Short socket timeouts bound a stalled scraper; the loop's
 * cached view means this never touches the stream socket.
 */
static void serve_scrape(int cfd, const imud_data_t *d, uint64_t packets)
{
    struct timeval tv = { 2, 0 };
    setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);

    char req[1024];
    ssize_t r = recv(cfd, req, sizeof req - 1, 0);
    if (r <= 0) { close(cfd); return; }
    req[r] = '\0';

    static char body[METRICS_BUF];
    int blen = prom_build_metrics(body, sizeof body, d, packets);
    if (blen < 0) { close(cfd); return; }

    char hdr[256];
    int hn = snprintf(hdr, sizeof hdr,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/plain; version=0.0.4; charset=utf-8\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n", blen);
    if (hn > 0 && hn < (int)sizeof hdr &&
        bridge_write_all(cfd, hdr, hn) == 0)
        (void)bridge_write_all(cfd, body, blen);
    close(cfd);
}

int main(int argc, char **argv)
{
    char config_path[256];
    int rc = bridge_parse_cli(argc, argv, &BI, config_path, sizeof config_path);
    if (rc != 0) return rc < 0 ? 1 : 0;

    imud_config_t cfg;
    if (bridge_load_config(&BI, config_path, &cfg) < 0) return 1;

    if (!cfg.prom_enabled) {
        bridge_exit_disabled(&BI);
        return 0;
    }

    bridge_install_signals();

    /* /metrics HTTP listener ([restart]). */
    int lfd = -1;
    if (cfg.prom_http_enabled) {
        lfd = open_listener(cfg.prom_listen_addr, cfg.prom_listen_port);
        if (lfd < 0) return 1;
        LOG_I("[prom] exporter → http://%s:%d/metrics, reading %s\n",
              cfg.prom_listen_addr, cfg.prom_listen_port, cfg.stream_socket);
    } else {
        LOG_W("[prom] no output enabled (http_enabled = false) — /metrics not served\n");
    }
    sd_notify_msg("READY=1");

    imud_t      *stream   = NULL;
    imud_data_t  latest;
    bool         have_pkt = false;
    uint64_t     packets  = 0;

    while (!bridge_stop) {
        sd_notify_msg("WATCHDOG=1");

        imud_config_t nc;
        if (bridge_reload_begin(&BI, config_path, &nc)) {
            if (nc.prom_http_enabled != cfg.prom_http_enabled)
                LOG_W("[prom] http_enabled change needs a restart to apply\n");
            if (strcmp(nc.prom_listen_addr, cfg.prom_listen_addr) != 0 ||
                nc.prom_listen_port != cfg.prom_listen_port)
                LOG_W("[prom] listen address change needs a restart to apply\n");
            cfg = nc;
            bridge_reload_done(&BI);
        }

        if (bridge_stream_ensure(&stream, cfg.stream_socket, BI.tag, 0) == 0) {
            have_pkt = false;
            /* Keep answering scrapes (imud_up 0) while imud is away. */
            if (lfd >= 0) {
                struct pollfd pw = { .fd = lfd, .events = POLLIN };
                if (poll(&pw, 1, 2000) == 1 && (pw.revents & POLLIN)) {
                    int cfd = accept(lfd, NULL, NULL);
                    if (cfd >= 0) serve_scrape(cfd, NULL, packets);
                }
            } else {
                bridge_sleep_interruptible(2);
            }
            continue;
        }

        struct pollfd pfds[2] = {
            { .fd = imud_fd(stream), .events = POLLIN },
            { .fd = lfd,             .events = POLLIN },
        };
        nfds_t nfds = (lfd >= 0) ? 2 : 1;   /* omit the listener when off */
        int pr = poll(pfds, nfds, 1000);
        if (pr < 0) {
            if (errno == EINTR) continue;
            LOG_E("[prom] poll: %s\n", strerror(errno));
            break;
        }

        if (pfds[0].revents & (POLLIN | POLLHUP | POLLERR)) {
            /* Drain everything pending; keep only the newest frame. */
            int r;
            while ((r = imud_read(stream, 0)) == 0) {
                latest   = *imud_data(stream);
                have_pkt = true;
                packets++;
            }
            if (r < 0) {
                bridge_stream_drop(&stream, BI.tag);
                have_pkt = false;
                bridge_sleep_interruptible(1);
                continue;
            }
        }

        if (lfd >= 0 && (pfds[1].revents & POLLIN)) {
            int cfd = accept(lfd, NULL, NULL);
            if (cfd >= 0)
                serve_scrape(cfd, have_pkt ? &latest : NULL, packets);
        }
    }

    LOG_I("[prom] shutting down\n");
    imud_free(stream);
    if (lfd >= 0) close(lfd);
    return 0;
}
