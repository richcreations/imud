/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * influx_main.c — imud-influxdb: InfluxDB bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket),
 * reads the 228-byte packets, and writes one InfluxDB line-protocol point per
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

#ifdef __linux__
# define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netdb.h>
#include <netinet/in.h>

#define IMUD_CLIENT_IMPLEMENTATION
#include "influx_line.h"        /* pulls in ../lib/imud_client.h (with impl) */
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

/* ── sd_notify (mirrors src/signalk_main.c) ──────────────────────────────── */

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

/* ── AF_UNIX stream connect + framed read (mirrors src/signalk_main.c) ────── */

static int connect_stream(const char *path)
{
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr *)&addr, sizeof addr) < 0) { close(fd); return -1; }
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return fd;
}

static int read_frame(int fd, unsigned char *frame)
{
    size_t got = 0;
    while (got < IMUD_PACKET_SIZE) {
        ssize_t n = read(fd, frame + got, IMUD_PACKET_SIZE - got);
        if (n <= 0) return -1;
        got += (size_t)n;
    }
    return 0;
}

static void interruptible_sleep(int secs)
{
    for (int i = 0; i < secs && !g_stop && !g_reload; i++) sleep(1);
}

/* ── UDP transport ───────────────────────────────────────────────────────── */

/* Resolve host:port and open a datagram socket; store the destination. */
static int open_udp(const char *host, int port,
                    struct sockaddr_storage *dst, socklen_t *dlen)
{
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) {
        LOG_E("[influx] cannot resolve UDP host '%s'\n", host);
        return -1;
    }
    int fd = socket(res->ai_family, res->ai_socktype | SOCK_CLOEXEC, 0);
    if (fd < 0) { freeaddrinfo(res); return -1; }
    memcpy(dst, res->ai_addr, res->ai_addrlen);
    *dlen = (socklen_t)res->ai_addrlen;
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof yes);  /* harmless for unicast */
    freeaddrinfo(res);
    return fd;
}

/* ── HTTP transport ──────────────────────────────────────────────────────── */

static int write_all(int fd, const char *b, int n)
{
    int off = 0;
    while (off < n) {
        ssize_t w = write(fd, b + off, n - off);
        if (w <= 0) { if (w < 0 && errno == EINTR) continue; return -1; }
        off += (int)w;
    }
    return 0;
}

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
        write_all(fd, hdr, hn) == 0 && write_all(fd, line, linelen) == 0) {
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

/* ── CLI ─────────────────────────────────────────────────────────────────── */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--config PATH]\n"
        "\n"
        "  InfluxDB bridge: reads imud's stream socket and writes InfluxDB\n"
        "  line-protocol points over UDP (default) or HTTP.\n"
        "  Configured by [imud-influxdb] in its own file.\n"
        "\n"
        "  --config PATH   Config file (default: /etc/imud/imud-influxdb.conf)\n"
        "  --version       Print version and exit\n",
        prog);
}

int main(int argc, char **argv)
{
    char config_path[256];
    snprintf(config_path, sizeof config_path, "/etc/imud/imud-influxdb.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof config_path, "%s", argv[++i]);
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud-influxdb %s\n", VERSION_STR);
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
        LOG_E("[influx] %s has errors (see above) — refusing to start\n", config_path);
        return 1;
    }

    if (getenv("INVOCATION_ID")) log_set_style(LOG_STYLE_JOURNAL);
    log_set_level_str(cfg.log_level);

    if (!cfg.influx_enabled) {
        LOG_I("[influx] disabled in config ([imud-influxdb] enabled = false) — exiting\n");
        return 0;
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGHUP,  &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    bool is_http    = (strcmp(cfg.influx_transport, "http") == 0);
    bool deg        = (strcmp(cfg.influx_units, "rad") != 0);
    bool emit_heave = cfg.publish_heave;
    long period_ns  = (cfg.influx_rate_hz > 0) ? 1000000000L / cfg.influx_rate_hz
                                               : 100000000L;

    int udp_fd = -1;
    struct sockaddr_storage udest;
    socklen_t udestlen = 0;
    if (!is_http) {
        udp_fd = open_udp(cfg.influx_udp_addr, cfg.influx_udp_port, &udest, &udestlen);
        if (udp_fd < 0) return 1;
    }

    if (is_http)
        LOG_I("[influx] bridge → http://%s:%d%s @ %d Hz (%s), reading %s\n",
              cfg.influx_http_host, cfg.influx_http_port, cfg.influx_http_path,
              cfg.influx_rate_hz, deg ? "deg" : "rad", cfg.stream_socket);
    else
        LOG_I("[influx] bridge → udp %s:%d @ %d Hz (%s), reading %s\n",
              cfg.influx_udp_addr, cfg.influx_udp_port, cfg.influx_rate_hz,
              deg ? "deg" : "rad", cfg.stream_socket);
    sd_notify_msg("READY=1");

    int  stream_fd = -1;
    unsigned char frame[IMUD_PACKET_SIZE];
    imud_packet_t latest;
    bool have_pkt = false;
    bool http_ok  = true;   /* rate-limits the HTTP failure log */

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
                deg        = (strcmp(nc.influx_units, "rad") != 0);
                emit_heave = nc.publish_heave;
                period_ns  = (nc.influx_rate_hz > 0) ? 1000000000L / nc.influx_rate_hz
                                                     : 100000000L;
                if (!is_http &&
                    (strcmp(nc.influx_udp_addr, cfg.influx_udp_addr) != 0 ||
                     nc.influx_udp_port != cfg.influx_udp_port)) {
                    close(udp_fd);
                    udp_fd = open_udp(nc.influx_udp_addr, nc.influx_udp_port, &udest, &udestlen);
                    if (udp_fd < 0) { LOG_E("[influx] reload: bad UDP dest — exiting\n"); break; }
                }
                if (strcmp(nc.influx_transport, cfg.influx_transport) != 0)
                    LOG_W("[influx] transport change needs a restart to apply\n");
                cfg = nc;
                LOG_I("[influx] config reloaded\n");
            } else {
                LOG_W("[influx] reload failed — keeping current config\n");
            }
        }

        if (stream_fd < 0) {
            stream_fd = connect_stream(cfg.stream_socket);
            if (stream_fd < 0) {
                LOG_W("[influx] stream socket %s unavailable: %s — retrying\n",
                      cfg.stream_socket, strerror(errno));
                have_pkt = false;
                interruptible_sleep(2);
                continue;
            }
            LOG_I("[influx] connected to %s\n", cfg.stream_socket);
            clock_gettime(CLOCK_MONOTONIC, &next);
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long wait_ns = (next.tv_sec - now.tv_sec) * 1000000000L
                     + (next.tv_nsec - now.tv_nsec);
        if (wait_ns < 0) wait_ns = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(stream_fd, &rfds);
        struct timeval tv = { .tv_sec  = wait_ns / 1000000000L,
                              .tv_usec = (wait_ns % 1000000000L) / 1000 };
        int s = select(stream_fd + 1, &rfds, NULL, NULL, &tv);
        if (s < 0) {
            if (errno == EINTR) continue;
            LOG_W("[influx] select: %s\n", strerror(errno));
            close(stream_fd); stream_fd = -1; have_pkt = false;
            continue;
        }
        if (s > 0 && FD_ISSET(stream_fd, &rfds)) {
            if (read_frame(stream_fd, frame) != 0) {
                LOG_I("[influx] stream disconnected\n");
                close(stream_fd); stream_fd = -1; have_pkt = false;
                continue;
            }
            if (imud_packet_valid(frame, sizeof frame)) {
                memcpy(&latest, frame, sizeof latest);
                have_pkt = true;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        if ((now.tv_sec > next.tv_sec) ||
            (now.tv_sec == next.tv_sec && now.tv_nsec >= next.tv_nsec)) {
            if (have_pkt) {
                char line[768];   /* full field set incl. v12 diagnostics */
                int n = influx_build_line(line, sizeof line, &latest,
                                          cfg.influx_measurement,
                                          cfg.influx_source_label, emit_heave, deg);
                if (n > 0) {
                    if (is_http) {
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
                    } else {
                        if (sendto(udp_fd, line, (size_t)n, 0,
                                   (struct sockaddr *)&udest, udestlen) < 0)
                            LOG_W("[influx] sendto: %s\n", strerror(errno));
                    }
                }
            }
            do {
                next.tv_nsec += period_ns;
                while (next.tv_nsec >= 1000000000L) {
                    next.tv_sec++; next.tv_nsec -= 1000000000L;
                }
            } while ((next.tv_sec < now.tv_sec) ||
                     (next.tv_sec == now.tv_sec && next.tv_nsec <= now.tv_nsec));
        }
    }

    LOG_I("[influx] shutting down\n");
    if (stream_fd >= 0) close(stream_fd);
    if (udp_fd >= 0)    close(udp_fd);
    return 0;
}
