/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * influx_main.c — imud-influxdb: InfluxDB bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket),
 * reads the 288-byte packets, and writes one InfluxDB line-protocol point per
 * tick — over UDP (default) or HTTP. Pure C, no external dependencies.
 *
 *   UDP : fire-and-forget datagram to InfluxDB 1.x's UDP listener or Telegraf's
 *         socket_listener (the usual path into InfluxDB 2.x/3.x).
 *   HTTP: a plaintext POST per point to /write (1.x) or /api/v2/write (2.x),
 *         with an optional `Authorization: Token` header. Connection: close, so
 *         a slow/absent server never wedges the stream reader (bounded connect).
 *
 * Reuses the imud-signalk skeleton for the stream side and reads its own config
 * file (/etc/imud/imud-influxdb.conf).
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
#include <time.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>

#include "influx_line.h"        /* pulls in ../lib/imud_client.h (types only) */
#include "../lib/imud.h"        /* libimud: stream connect/read/validate */
#include "bridge.h"             /* shared bridge scaffolding */
#include "sdnotify.h"
#include "config.h"
#include "log.h"

static const bridge_info_t BI = {
    .prog         = "imud-influxdb",
    .tag          = "influx",
    .section      = "imud-influxdb",
    .default_conf = "/etc/imud/imud-influxdb.conf",
    .usage_desc   =
        "  InfluxDB bridge: reads imud's stream socket and writes InfluxDB\n"
        "  line-protocol points over UDP (default) or HTTP.\n"
        "  Configured by [imud-influxdb] in its own file.\n",
};

/* ── HTTP transport ──────────────────────────────────────────────────────── */

/* Non-blocking connect with a bounded wait. Returns a connected fd or -1. */
static int tcp_connect(const char *host, int port, int timeout_ms)
{
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, portstr, &hints, &res) != 0) return -1;

    int fd = -1;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype | SOCK_CLOEXEC, 0);
        if (fd < 0) continue;
        int fl = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, fl | O_NONBLOCK);
        int r = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (r == 0) { fcntl(fd, F_SETFL, fl); break; }
        if (errno == EINPROGRESS) {
            fd_set w; FD_ZERO(&w); FD_SET(fd, &w);
            struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
            if (select(fd + 1, NULL, &w, NULL, &tv) == 1) {
                int err = 0; socklen_t el = sizeof err;
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el);
                if (err == 0) { fcntl(fd, F_SETFL, fl); break; }
            }
        }
        close(fd); fd = -1;
    }
    freeaddrinfo(res);
    if (fd >= 0) {
        struct timeval tv = { 2, 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    }
    return fd;
}

/* POST one line-protocol point. Returns the HTTP status, or -1 on transport
 * failure. Connection: close — stateless, never wedges the reader. */
static int http_post(const imud_config_t *cfg, const char *line, int linelen)
{
    int fd = tcp_connect(cfg->influx_http_host, cfg->influx_http_port, 1000);
    if (fd < 0) return -1;

    char hdr[640];
    int hn;
    if (cfg->influx_http_token[0])
        hn = snprintf(hdr, sizeof hdr,
            "POST %s HTTP/1.1\r\nHost: %s:%d\r\nAuthorization: Token %s\r\n"
            "Content-Type: text/plain; charset=utf-8\r\nContent-Length: %d\r\n"
            "Connection: close\r\n\r\n",
            cfg->influx_http_path, cfg->influx_http_host, cfg->influx_http_port,
            cfg->influx_http_token, linelen);
    else
        hn = snprintf(hdr, sizeof hdr,
            "POST %s HTTP/1.1\r\nHost: %s:%d\r\n"
            "Content-Type: text/plain; charset=utf-8\r\nContent-Length: %d\r\n"
            "Connection: close\r\n\r\n",
            cfg->influx_http_path, cfg->influx_http_host, cfg->influx_http_port,
            linelen);

    int status = -1;
    if (hn > 0 && hn < (int)sizeof hdr &&
        bridge_write_all(fd, hdr, hn) == 0 && bridge_write_all(fd, line, linelen) == 0) {
        char resp[1024];
        ssize_t r = recv(fd, resp, sizeof resp - 1, 0);
        if (r > 0) {
            resp[r] = '\0';
            if (sscanf(resp, "HTTP/1.%*d %d", &status) != 1) status = 0;
            char sink[1024];                       /* drain the rest to EOF */
            while (recv(fd, sink, sizeof sink, 0) > 0) { }
        }
    }
    close(fd);
    return status;
}

int main(int argc, char **argv)
{
    char config_path[256];
    int rc = bridge_parse_cli(argc, argv, &BI, config_path, sizeof config_path);
    if (rc != 0) return rc < 0 ? 1 : 0;

    imud_config_t cfg;
    if (bridge_load_config(&BI, config_path, &cfg) < 0) return 1;

    if (!cfg.influx_enabled) {
        bridge_exit_disabled(&BI);
        return 0;
    }

    /* Back-compat: honour the deprecated `transport` key when no per-output
     * enable is set (mirrors what the reload path does below). */
    if (config_apply_influx_transport_compat(&cfg))
        LOG_W("[influx] `transport` is deprecated — use udp_enabled / http_enabled "
              "(mapped transport=\"%s\" for now)\n", cfg.influx_transport);

    bridge_install_signals();

    bool deg        = (strcmp(cfg.influx_units, "rad") != 0);
    int  detail     = influx_detail_from_name(cfg.influx_detail);
    if (detail < 0) {
        LOG_W("[influxdb] unknown detail \"%s\"; using \"%s\"\n",
              cfg.influx_detail, influx_detail_name(INFLUX_DETAIL_HEALTH));
        detail = INFLUX_DETAIL_HEALTH;
    }
    bool emit_heave = cfg.publish_heave;
    long period_ns  = bridge_period_ns(cfg.influx_rate_hz, 100000000L);

    /* UDP transport ([restart]). */
    int udp_fd = -1;
    struct sockaddr_storage udest;
    socklen_t udestlen = 0;
    if (cfg.influx_udp_enabled) {
        udp_fd = bridge_open_udp(cfg.influx_udp_addr, cfg.influx_udp_port,
                                 BI.tag, &udest, &udestlen);
        if (udp_fd < 0) return 1;
    }

    if (!cfg.influx_udp_enabled && !cfg.influx_http_enabled)
        LOG_W("[influx] no output enabled (udp_enabled/http_enabled both false) "
              "— nothing will be sent\n");

    if (cfg.influx_http_enabled)
        LOG_I("[influx] bridge → http://%s:%d%s @ %d Hz (%s), reading %s\n",
              cfg.influx_http_host, cfg.influx_http_port, cfg.influx_http_path,
              cfg.influx_rate_hz, deg ? "deg" : "rad", cfg.stream_socket);
    if (cfg.influx_udp_enabled)
        LOG_I("[influx] bridge → udp %s:%d @ %d Hz (%s), reading %s\n",
              cfg.influx_udp_addr, cfg.influx_udp_port, cfg.influx_rate_hz,
              deg ? "deg" : "rad", cfg.stream_socket);
    sd_notify_msg("READY=1");

    imud_t       *stream = NULL;
    imud_packet_t latest;
    bool have_pkt = false;
    bool http_ok  = true;   /* rate-limits the HTTP failure log */

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!bridge_stop) {
        sd_notify_msg("WATCHDOG=1");

        imud_config_t nc;
        if (bridge_reload_begin(&BI, config_path, &nc)) {
            deg        = (strcmp(nc.influx_units, "rad") != 0);
            {   /* keep the running level on an unparseable reload */
                int d = influx_detail_from_name(nc.influx_detail);
                if (d < 0) LOG_W("[influxdb] unknown detail \"%s\"; keeping "
                                 "\"%s\"\n", nc.influx_detail,
                                 influx_detail_name(detail));
                else detail = d;
            }
            emit_heave = nc.publish_heave;
            period_ns  = bridge_period_ns(nc.influx_rate_hz, 100000000L);
            config_apply_influx_transport_compat(&nc);
            if (cfg.influx_udp_enabled &&
                (strcmp(nc.influx_udp_addr, cfg.influx_udp_addr) != 0 ||
                 nc.influx_udp_port != cfg.influx_udp_port)) {
                close(udp_fd);
                udp_fd = bridge_open_udp(nc.influx_udp_addr, nc.influx_udp_port,
                                         BI.tag, &udest, &udestlen);
                if (udp_fd < 0) { LOG_E("[influx] reload: bad UDP dest — exiting\n"); break; }
            }
            /*
             * The output enables are [restart] keys, in the registry and in
             * the man page, and this warning says so.  Carry the RUNNING
             * values across the copy so that is actually true.
             *
             * Copying them made the warning a lie in a way that bites: a
             * reload turning UDP on left udp_fd at -1, because the reopen
             * above is guarded on the enable that was in force when the
             * socket was opened.  The publish path then called sendto() on
             * -1 at the emit rate for the life of the process, logging
             * EBADF every time.  Turning UDP off had the mirror problem —
             * it stopped the output immediately, which is the change the
             * warning promises will not take effect until a restart.
             */
            bool udp_on  = cfg.influx_udp_enabled;
            bool http_on = cfg.influx_http_enabled;
            if (nc.influx_udp_enabled != udp_on ||
                nc.influx_http_enabled != http_on)
                LOG_W("[influx] output enable change needs a restart to apply\n");
            cfg = nc;
            cfg.influx_udp_enabled  = udp_on;
            cfg.influx_http_enabled = http_on;
            bridge_reload_done(&BI);
        }

        int cs = bridge_stream_ensure(&stream, cfg.stream_socket, BI.tag, 2);
        if (cs == 0) { have_pkt = false; continue; }
        if (cs == 2) clock_gettime(CLOCK_MONOTONIC, &next);

        /* Wait until the next emit tick, draining frames as they arrive. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int r = imud_read(stream, bridge_wait_ms(&now, &next));
        if (r < 0) {
            bridge_stream_drop(&stream, BI.tag);
            have_pkt = false;
            continue;
        }
        if (r == 0) {
            latest   = *imud_wire(stream);
            have_pkt = true;
        }
        /* r == 1: tick deadline (or a signal) — fall through to the emitter. */

        clock_gettime(CLOCK_MONOTONIC, &now);
        if (bridge_due(&now, &next)) {
            if (have_pkt) {
                /* Full field set incl. v17 gate diagnostics. Worst case
                 * measured at 711 bytes (every field negative-and-long, both
                 * name fields at their 31-char maximum), so 1024 keeps real
                 * headroom — an overflow here returns n <= 0 and the sample is
                 * dropped silently, which is a bad way to find out. */
                char line[1024];
                int n = influx_build_line(line, sizeof line, &latest,
                                          cfg.influx_measurement,
                                          cfg.influx_source_label, emit_heave,
                                          deg, detail);
                if (n > 0) {
                    if (cfg.influx_http_enabled) {
                        int st = http_post(&cfg, line, n);
                        if (st < 0 || st / 100 != 2) {
                            if (http_ok)
                                LOG_W("[influx] HTTP write failed (%s) — suppressing until it recovers\n",
                                      st < 0 ? "no connection" : "non-2xx");
                            http_ok = false;
                        } else if (!http_ok) {
                            LOG_I("[influx] HTTP write recovered\n");
                            http_ok = true;
                        }
                    }
                    /* udp_fd is open whenever the enable is set — startup
                     * refuses to run otherwise, and the enable is now
                     * restart-scoped — but that correlation is established a
                     * hundred lines up and survives a struct copy, so it is
                     * invisible here to a reader and to the analyzer. Say it
                     * where the descriptor is used. */
                    if (cfg.influx_udp_enabled && udp_fd >= 0) {
                        if (sendto(udp_fd, line, (size_t)n, 0,
                                   (struct sockaddr *)&udest, udestlen) < 0)
                            LOG_W("[influx] sendto: %s\n", strerror(errno));
                    }
                }
            }
            bridge_advance(&next, &now, period_ns);
        }
    }

    LOG_I("[influx] shutting down\n");
    imud_free(stream);
    if (udp_fd >= 0)    close(udp_fd);
    return 0;
}
