/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * signalk_main.c — imud-signalk: Signal K bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket),
 * reads the 276-byte packets, and emits a Signal K delta (JSON) over UDP at
 * the configured rate for every imud field that has a standard Signal K path.
 * An optional TCP listener (tcp_enabled) serves the same deltas newline-
 * framed to connected clients — the Signal K server consumes it as a TCP
 * data connection.  See sk_delta.c for the field/unit mapping.
 *
 * It is an ordinary stream consumer: it holds no hardware, reuses the public
 * client header (lib/imud_client.h) for packet validation, and reconnects
 * automatically whenever imud restarts.  Requires [stream] enabled = true.
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
#include <sys/socket.h>
#include <netinet/in.h>

#include "sk_delta.h"          /* pulls in ../lib/imud_client.h (types only) */
#include "../lib/imud.h"       /* libimud: stream connect/read/validate */
#include "bridge.h"            /* shared bridge scaffolding */
#include "sdnotify.h"
#include "config.h"
#include "netserv.h"
#include "log.h"

static const bridge_info_t BI = {
    .prog         = "imud-signalk",
    .tag          = "signalk",
    .section      = "imud-signalk",
    .default_conf = "/etc/imud/imud-signalk.conf",
    .usage_desc   =
        "  Signal K bridge: reads imud's stream socket and emits Signal K\n"
        "  delta JSON over UDP. Configured by [imud-signalk] in its own file.\n",
};

int main(int argc, char **argv)
{
    char config_path[256];
    int rc = bridge_parse_cli(argc, argv, &BI, config_path, sizeof config_path);
    if (rc != 0) return rc < 0 ? 1 : 0;

    imud_config_t cfg;
    if (bridge_load_config(&BI, config_path, &cfg) < 0) return 1;

    if (!cfg.sk_enabled) {
        bridge_exit_disabled(&BI);
        return 0;
    }

    bridge_install_signals();

    /* UDP delta output ([restart]).  bridge_open_udp() resolves with
     * getaddrinfo(AF_UNSPEC), so dest_addr accepts a hostname as well as a
     * numeric address — matching every other bridge.  This one used
     * inet_pton() until 1.7 and rejected names the others accepted. */
    struct sockaddr_storage dest;
    socklen_t dest_len = 0;
    int udp_fd = -1;
    if (cfg.sk_udp_enabled) {
        udp_fd = bridge_open_udp(cfg.sk_dest_addr, cfg.sk_dest_port,
                                 BI.tag, &dest, &dest_len);
        if (udp_fd < 0) return 1;
    }

    /* Optional TCP listener — same deltas, newline-framed ([restart]). */
    netserv_t tcp;
    netserv_init(&tcp);
    if (cfg.sk_tcp_enabled) {
        if (netserv_open(&tcp, cfg.sk_tcp_bind_addr, cfg.sk_tcp_port,
                         "signalk_tcp") < 0) { if (udp_fd >= 0) close(udp_fd); return 1; }
        LOG_I("[signalk] TCP listener %s:%d (max %d clients)\n",
              cfg.sk_tcp_bind_addr, tcp.port, NETSRV_MAX_CLIENTS);
    }

    if (!cfg.sk_udp_enabled && !cfg.sk_tcp_enabled)
        LOG_W("[signalk] no output enabled (udp_enabled/tcp_enabled both false) "
              "— nothing will be sent\n");

    LOG_I("[signalk] bridge → %s:%d @ %d Hz (source '%s')%s%s, reading %s\n",
          cfg.sk_dest_addr, cfg.sk_dest_port, cfg.sk_rate_hz, cfg.sk_source_label,
          cfg.sk_udp_enabled ? " +udp" : "", cfg.sk_tcp_enabled ? " +tcp" : "",
          cfg.stream_socket);
    sd_notify_msg("READY=1");

    long period_ns = bridge_period_ns(cfg.sk_rate_hz, 100000000L);
    bool emit_heave = cfg.publish_heave;

    imud_t       *stream = NULL;
    imud_packet_t latest;
    bool have_pkt = false;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!bridge_stop) {
        sd_notify_msg("WATCHDOG=1");

        /* Accept pending TCP clients (no-op when tcp_enabled = false). */
        netserv_accept(&tcp);

        imud_config_t nc;
        if (bridge_reload_begin(&BI, config_path, &nc)) {
            emit_heave = nc.publish_heave;
            period_ns  = bridge_period_ns(nc.sk_rate_hz, 100000000L);
            if (cfg.sk_udp_enabled &&
                (strcmp(nc.sk_dest_addr, cfg.sk_dest_addr) != 0 ||
                 nc.sk_dest_port != cfg.sk_dest_port)) {
                close(udp_fd);
                udp_fd = bridge_open_udp(nc.sk_dest_addr, nc.sk_dest_port,
                                         BI.tag, &dest, &dest_len);
                if (udp_fd < 0) { LOG_E("[signalk] reload: bad dest — exiting\n"); break; }
            }
            if (nc.sk_udp_enabled != cfg.sk_udp_enabled)
                LOG_W("[signalk] udp_enabled change needs a restart\n");
            if (nc.sk_tcp_enabled != cfg.sk_tcp_enabled ||
                nc.sk_tcp_port != cfg.sk_tcp_port ||
                strcmp(nc.sk_tcp_bind_addr, cfg.sk_tcp_bind_addr) != 0)
                LOG_W("[signalk] tcp_* change needs a restart\n");
            cfg = nc;
            bridge_reload_done(&BI);
        }

        int cs = bridge_stream_ensure(&stream, cfg.stream_socket, BI.tag, 2);
        if (cs == 0) { have_pkt = false; continue; }
        if (cs == 2) clock_gettime(CLOCK_MONOTONIC, &next);

        /* Wait until the next emit tick, draining frames as they arrive so the
         * packet we send is always the most recent. */
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
                char delta[512];
                int n = sk_build_delta(delta, sizeof delta, &latest,
                                       cfg.sk_source_label, emit_heave);
                if (n > 0) {
                    if (udp_fd >= 0 &&
                        sendto(udp_fd, delta, (size_t)n, 0,
                               (struct sockaddr *)&dest, dest_len) < 0)
                        LOG_W("[signalk] sendto: %s\n", strerror(errno));
                    /* Same delta to TCP clients, newline-framed (one JSON
                     * line per delta — the SK data-connection format). */
                    if ((size_t)n + 1 < sizeof delta) {
                        delta[n] = '\n';
                        netserv_broadcast(&tcp, delta, (size_t)n + 1);
                    }
                }
            }
            bridge_advance(&next, &now, period_ns);
        }
    }

    LOG_I("[signalk] shutting down\n");
    imud_free(stream);
    netserv_close(&tcp);
    if (udp_fd >= 0) close(udp_fd);
    return 0;
}
