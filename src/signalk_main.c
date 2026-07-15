/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * signalk_main.c — imud-signalk: Signal K bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket),
 * reads the 260-byte packets, and emits a Signal K delta (JSON) over UDP at
 * the configured rate for every imud field that has a standard Signal K path.
 * See sk_delta.c for the field/unit mapping.
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
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sk_delta.h"          /* pulls in ../lib/imud_client.h (types only) */
#include "../lib/imud.h"       /* libimud: stream connect/read/validate */
#include "config.h"
#include "log.h"

#ifndef SOCK_CLOEXEC
# define SOCK_CLOEXEC 0
#endif

#include "version.h"                 /* IMUD_VERSION_STR — canonical version */
#define VERSION_STR IMUD_VERSION_STR

static volatile sig_atomic_t g_stop;
static volatile sig_atomic_t g_reload;

static void on_signal(int sig)
{
    if (sig == SIGHUP) g_reload = 1;
    else               g_stop   = 1;
}

/* ── sd_notify (mirrors src/main.c) ──────────────────────────────────────── */

static void sd_notify_msg(const char *msg)
{
    const char *sock = getenv("NOTIFY_SOCKET");
    if (!sock) return;
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    if (sock[0] == '@') {
        addr.sun_path[0] = '\0';
        strncpy(addr.sun_path + 1, sock + 1, sizeof(addr.sun_path) - 2);
    } else {
        strncpy(addr.sun_path, sock, sizeof(addr.sun_path) - 1);
    }
    int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return;
    sendto(fd, msg, strlen(msg), MSG_NOSIGNAL, (struct sockaddr *)&addr, sizeof addr);
    close(fd);
}

/* ── UDP destination socket ──────────────────────────────────────────────── */

static int open_udp_dest(const char *addr_s, int port, struct sockaddr_in *dest)
{
    struct in_addr addr;
    if (inet_pton(AF_INET, addr_s, &addr) <= 0) {
        LOG_E("[signalk] invalid dest_addr '%s' (numeric IPv4 only)\n", addr_s);
        return -1;
    }
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) { LOG_E("[signalk] socket(): %s\n", strerror(errno)); return -1; }

    /* Harmless for unicast; needed if the SK host is a broadcast address. */
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof yes);

    memset(dest, 0, sizeof *dest);
    dest->sin_family = AF_INET;
    dest->sin_port   = htons((uint16_t)port);
    dest->sin_addr   = addr;
    return fd;
}

/* Sleep `secs` in 1 s steps, waking on stop/reload. */
static void interruptible_sleep(int secs)
{
    for (int i = 0; i < secs && !g_stop && !g_reload; i++) sleep(1);
}

/* ── CLI + config ────────────────────────────────────────────────────────── */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--config PATH]\n"
        "\n"
        "  Signal K bridge: reads imud's stream socket and emits Signal K\n"
        "  delta JSON over UDP. Configured by [imud-signalk] in its own file.\n"
        "\n"
        "  --config PATH   Config file (default: /etc/imud/imud-signalk.conf)\n"
        "  --version       Print version and exit\n",
        prog);
}

int main(int argc, char **argv)
{
    char config_path[256];
    snprintf(config_path, sizeof config_path, "/etc/imud/imud-signalk.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof config_path, "%s", argv[++i]);
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud-signalk %s\n", VERSION_STR);
            return 0;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            LOG_E("unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(config_path, &cfg);
    if (rc == CONFIG_ERR_PARSE) {
        LOG_E("[signalk] %s has errors (see above) — refusing to start\n", config_path);
        return 1;
    }

    /* Match the daemon's logging behaviour. */
    if (getenv("INVOCATION_ID")) log_set_style(LOG_STYLE_JOURNAL);
    log_set_level_str(cfg.log_level);

    if (!cfg.sk_enabled) {
        LOG_I("[signalk] disabled in config ([imud-signalk] enabled = false) — exiting\n");
        return 0;
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGHUP,  &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    struct sockaddr_in dest;
    int udp_fd = open_udp_dest(cfg.sk_dest_addr, cfg.sk_dest_port, &dest);
    if (udp_fd < 0) return 1;

    LOG_I("[signalk] bridge → %s:%d @ %d Hz (source '%s'), reading %s\n",
          cfg.sk_dest_addr, cfg.sk_dest_port, cfg.sk_rate_hz,
          cfg.sk_source_label, cfg.stream_socket);
    sd_notify_msg("READY=1");

    long period_ns = (cfg.sk_rate_hz > 0) ? 1000000000L / cfg.sk_rate_hz
                                          : 100000000L;
    bool emit_heave = cfg.publish_heave;

    imud_t       *stream = NULL;
    imud_packet_t latest;
    bool have_pkt = false;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!g_stop) {
        sd_notify_msg("WATCHDOG=1");

        if (g_reload) {
            g_reload = 0;
            imud_config_t nc;
            config_defaults(&nc);
            if (config_load(config_path, &nc) != CONFIG_ERR_PARSE) {
                log_set_level_str(nc.log_level);
                emit_heave = nc.publish_heave;
                period_ns  = (nc.sk_rate_hz > 0) ? 1000000000L / nc.sk_rate_hz
                                                 : 100000000L;
                if (strcmp(nc.sk_dest_addr, cfg.sk_dest_addr) != 0 ||
                    nc.sk_dest_port != cfg.sk_dest_port) {
                    close(udp_fd);
                    udp_fd = open_udp_dest(nc.sk_dest_addr, nc.sk_dest_port, &dest);
                    if (udp_fd < 0) { LOG_E("[signalk] reload: bad dest — exiting\n"); break; }
                }
                cfg = nc;
                LOG_I("[signalk] config reloaded\n");
            } else {
                LOG_W("[signalk] reload failed — keeping current config\n");
            }
        }

        if (!stream) {
            stream = imud_connect_stream(cfg.stream_socket);
            if (!stream) {
                LOG_W("[signalk] stream socket %s unavailable: %s — retrying\n",
                      cfg.stream_socket, strerror(errno));
                have_pkt = false;
                interruptible_sleep(2);
                continue;
            }
            LOG_I("[signalk] connected to %s\n", cfg.stream_socket);
            clock_gettime(CLOCK_MONOTONIC, &next);
        }

        /* Wait until the next emit tick, draining frames as they arrive so the
         * packet we send is always the most recent. Round the wait up so a
         * sub-millisecond remainder can't busy-spin. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long wait_ns = (next.tv_sec - now.tv_sec) * 1000000000L
                     + (next.tv_nsec - now.tv_nsec);
        if (wait_ns < 0) wait_ns = 0;

        int r = imud_read(stream, (int)((wait_ns + 999999L) / 1000000L));
        if (r < 0) {
            LOG_I("[signalk] stream disconnected\n");
            imud_free(stream); stream = NULL; have_pkt = false;
            continue;
        }
        if (r == 0) {
            latest   = *imud_wire(stream);
            have_pkt = true;
        }
        /* r == 1: tick deadline (or a signal) — fall through to the emitter. */

        clock_gettime(CLOCK_MONOTONIC, &now);
        if ((now.tv_sec > next.tv_sec) ||
            (now.tv_sec == next.tv_sec && now.tv_nsec >= next.tv_nsec)) {
            if (have_pkt) {
                char delta[512];
                int n = sk_build_delta(delta, sizeof delta, &latest,
                                       cfg.sk_source_label, emit_heave);
                if (n > 0) {
                    if (sendto(udp_fd, delta, (size_t)n, 0,
                               (struct sockaddr *)&dest, sizeof dest) < 0)
                        LOG_W("[signalk] sendto: %s\n", strerror(errno));
                }
            }
            /* Advance the deadline, skipping missed ticks after a stall. */
            do {
                next.tv_nsec += period_ns;
                while (next.tv_nsec >= 1000000000L) {
                    next.tv_sec++; next.tv_nsec -= 1000000000L;
                }
            } while ((next.tv_sec < now.tv_sec) ||
                     (next.tv_sec == now.tv_sec && next.tv_nsec <= now.tv_nsec));
        }
    }

    LOG_I("[signalk] shutting down\n");
    imud_free(stream);
    close(udp_fd);
    return 0;
}
