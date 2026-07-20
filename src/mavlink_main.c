/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mavlink_main.c — imud-mavlink: MAVLink bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket),
 * reads the 260-byte packets, and emits MAVLink (v1 or v2) HEARTBEAT (1 Hz) +
 * ATTITUDE/ATTITUDE_QUATERNION (rate_hz) to UDP, serial, and/or a TCP
 * listener (GCS clients connect, e.g. QGroundControl tcp:host:5760)
 * simultaneously.
 * Pure C, no external dependencies (hand-rolled encoder in mavlink_encode.c).
 *
 * MAVLink's body frame is FRD/NED, the same as imud, so roll/pitch/yaw, the gyro
 * body rates, and the quaternion pass straight through (SI radians / rad/s).
 * Reuses the imud-signalk/influx skeleton for the stream side; reads its own
 * config file (/etc/imud/imud-mavlink.conf).
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
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netdb.h>
#include <netinet/in.h>

#include "../lib/imud_client.h"
#include "../lib/imud.h"        /* libimud: stream connect/read/validate */
#include "mavlink_encode.h"
#include "config.h"
#include "netserv.h"
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

/* One output transport: UDP destination, serial device, or a TCP listener
 * (srv != NULL) whose connected GCS clients all receive the same frames. */
typedef struct {
    int      fd;       /* -1 for the TCP-listener sink */
    bool     is_udp;
    netserv_t *srv;    /* TCP-listener sink; NULL for udp/serial */
    struct sockaddr_storage dst;
    socklen_t dlen;
    uint8_t  seq;      /* per-link sequence (MAVLink loss detection is per link) */
} sink_t;

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

static void interruptible_sleep(int secs)
{
    for (int i = 0; i < secs && !g_stop && !g_reload; i++) sleep(1);
}

/* ── Transports ──────────────────────────────────────────────────────────── */

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
        LOG_E("[mavlink] cannot resolve UDP host '%s'\n", host);
        return -1;
    }
    int fd = socket(res->ai_family, res->ai_socktype | SOCK_CLOEXEC, 0);
    if (fd < 0) { freeaddrinfo(res); return -1; }
    memcpy(dst, res->ai_addr, res->ai_addrlen);
    *dlen = (socklen_t)res->ai_addrlen;
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof yes);
    freeaddrinfo(res);
    return fd;
}

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default:     return 0;
    }
}

static int open_serial(const char *dev, int baud)
{
    speed_t sp = baud_to_speed(baud);
    if (sp == 0) { LOG_E("[mavlink] unsupported serial baud %d\n", baud); return -1; }
    int fd = open(dev, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) { LOG_E("[mavlink] open %s: %s\n", dev, strerror(errno)); return -1; }
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        LOG_E("[mavlink] tcgetattr %s: %s\n", dev, strerror(errno));
        close(fd); return -1;
    }
    cfmakeraw(&t);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~(unsigned)CSTOPB;             /* 1 stop bit */
    t.c_cflag &= ~(unsigned)PARENB;             /* no parity  */
    t.c_cflag &= ~(unsigned)CSIZE;
    t.c_cflag |= CS8;                            /* 8 data bits */
#ifdef CRTSCTS
    t.c_cflag &= ~(unsigned)CRTSCTS;             /* no hardware flow control */
#endif
    cfsetispeed(&t, sp);
    cfsetospeed(&t, sp);
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        LOG_E("[mavlink] tcsetattr %s: %s\n", dev, strerror(errno));
        close(fd); return -1;
    }
    return fd;
}

static void sink_write(sink_t *s, const uint8_t *buf, int n)
{
    if (s->srv)
        netserv_broadcast(s->srv, buf, (size_t)n);
    else if (s->is_udp)
        sendto(s->fd, buf, (size_t)n, 0, (struct sockaddr *)&s->dst, s->dlen);
    else {
        ssize_t w = write(s->fd, buf, (size_t)n);
        (void)w;   /* best-effort; a stalled radio must not wedge the reader */
    }
}

static uint32_t boot_ms(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long ms = (now.tv_sec - start->tv_sec) * 1000L
            + (now.tv_nsec - start->tv_nsec) / 1000000L;
    return (uint32_t)ms;
}

/* ── CLI ─────────────────────────────────────────────────────────────────── */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--config PATH]\n"
        "\n"
        "  MAVLink bridge: reads imud's stream socket and emits MAVLink\n"
        "  HEARTBEAT/ATTITUDE/ATTITUDE_QUATERNION over UDP and/or serial.\n"
        "  Configured by [imud-mavlink] in its own file.\n"
        "\n"
        "  --config PATH   Config file (default: /etc/imud/imud-mavlink.conf)\n"
        "  --version       Print version and exit\n",
        prog);
}

int main(int argc, char **argv)
{
    char config_path[256];
    snprintf(config_path, sizeof config_path, "/etc/imud/imud-mavlink.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof config_path, "%s", argv[++i]);
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud-mavlink %s\n", VERSION_STR);
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
        LOG_E("[mavlink] %s has errors (see above) — refusing to start\n", config_path);
        return 1;
    }

    if (getenv("INVOCATION_ID")) log_set_style(LOG_STYLE_JOURNAL);
    log_set_level_str(cfg.log_level);

    if (!cfg.mav_enabled) {
        LOG_I("[mavlink] disabled in config ([imud-mavlink] enabled = false) — exiting\n");
        sd_notify_msg("READY=1");   /* signal a clean start so systemd stops us, no restart loop */
        return 0;
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGHUP,  &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    int  ver       = (cfg.mav_version == 1) ? 1 : 2;
    long period_ns = (cfg.mav_rate_hz > 0) ? 1000000000L / cfg.mav_rate_hz : 100000000L;

    /* Build the sink list. */
    sink_t sinks[3];
    netserv_t tcp_srv;
    netserv_init(&tcp_srv);
    int nsinks = 0;
    int udp_idx = -1;
    if (cfg.mav_udp_enabled) {
        sink_t *s = &sinks[nsinks];
        memset(s, 0, sizeof *s);
        s->is_udp = true;
        s->fd = open_udp(cfg.mav_udp_addr, cfg.mav_udp_port, &s->dst, &s->dlen);
        if (s->fd < 0) return 1;
        udp_idx = nsinks++;
    }
    if (cfg.mav_serial_enabled) {
        sink_t *s = &sinks[nsinks];
        memset(s, 0, sizeof *s);
        s->is_udp = false;
        s->fd = open_serial(cfg.mav_serial_device, cfg.mav_serial_baud);
        if (s->fd < 0) return 1;
        nsinks++;
    }
    if (cfg.mav_tcp_enabled) {
        if (netserv_open(&tcp_srv, cfg.mav_tcp_bind_addr, cfg.mav_tcp_port,
                         "mavlink_tcp") < 0) {
            for (int k = 0; k < nsinks; k++) close(sinks[k].fd);
            return 1;
        }
        sink_t *s = &sinks[nsinks];
        memset(s, 0, sizeof *s);
        s->fd  = -1;
        s->srv = &tcp_srv;
        nsinks++;
        LOG_I("[mavlink] TCP listener %s:%d (max %d clients)\n",
              cfg.mav_tcp_bind_addr, tcp_srv.port, NETSRV_MAX_CLIENTS);
    }
    if (nsinks == 0)
        LOG_W("[mavlink] no transport enabled (udp_enabled/serial_enabled/"
              "tcp_enabled all false) — nothing will be sent\n");

    LOG_I("[mavlink] bridge v%d sys=%d comp=%d @ %d Hz%s%s%s, reading %s\n",
          ver, cfg.mav_system_id, cfg.mav_component_id, cfg.mav_rate_hz,
          cfg.mav_udp_enabled ? " +udp" : "", cfg.mav_serial_enabled ? " +serial" : "",
          cfg.mav_tcp_enabled ? " +tcp" : "",
          cfg.stream_socket);
    sd_notify_msg("READY=1");

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    imud_t       *stream = NULL;
    imud_packet_t latest;
    bool have_pkt = false;

    struct timespec now, next_hb, next_data;
    clock_gettime(CLOCK_MONOTONIC, &now);
    next_hb = next_data = now;

    while (!g_stop) {
        sd_notify_msg("WATCHDOG=1");

        if (g_reload) {
            g_reload = 0;
            imud_config_t nc;
            config_defaults(&nc);
            if (config_load(config_path, &nc) != CONFIG_ERR_PARSE) {
                log_set_level_str(nc.log_level);
                ver       = (nc.mav_version == 1) ? 1 : 2;
                period_ns = (nc.mav_rate_hz > 0) ? 1000000000L / nc.mav_rate_hz : 100000000L;
                if (udp_idx >= 0 &&
                    (strcmp(nc.mav_udp_addr, cfg.mav_udp_addr) != 0 ||
                     nc.mav_udp_port != cfg.mav_udp_port)) {
                    close(sinks[udp_idx].fd);
                    uint8_t keep_seq = sinks[udp_idx].seq;
                    sinks[udp_idx].fd = open_udp(nc.mav_udp_addr, nc.mav_udp_port,
                                                 &sinks[udp_idx].dst, &sinks[udp_idx].dlen);
                    sinks[udp_idx].seq = keep_seq;
                    if (sinks[udp_idx].fd < 0) { LOG_E("[mavlink] reload: bad UDP dest — exiting\n"); break; }
                }
                if (nc.mav_system_id != cfg.mav_system_id ||
                    nc.mav_component_id != cfg.mav_component_id ||
                    nc.mav_udp_enabled != cfg.mav_udp_enabled ||
                    nc.mav_serial_enabled != cfg.mav_serial_enabled ||
                    nc.mav_tcp_enabled != cfg.mav_tcp_enabled ||
                    nc.mav_tcp_port != cfg.mav_tcp_port ||
                    strcmp(nc.mav_tcp_bind_addr, cfg.mav_tcp_bind_addr) != 0)
                    LOG_W("[mavlink] sysid/compid/transport change needs a restart\n");
                cfg = nc;
                LOG_I("[mavlink] config reloaded\n");
            } else {
                LOG_W("[mavlink] reload failed — keeping current config\n");
            }
        }

        /* Accept pending TCP clients (no-op when tcp_enabled = false). */
        netserv_accept(&tcp_srv);

        if (!stream) {
            stream = imud_connect_stream(cfg.stream_socket);
            if (!stream) {
                LOG_W("[mavlink] stream socket %s unavailable: %s — retrying\n",
                      cfg.stream_socket, strerror(errno));
                have_pkt = false;
                interruptible_sleep(2);
                continue;
            }
            LOG_I("[mavlink] connected to %s\n", cfg.stream_socket);
        }

        /* Wait until the nearer of the two deadlines, draining stream frames.
         * Round up so a sub-millisecond remainder can't busy-spin. */
        clock_gettime(CLOCK_MONOTONIC, &now);
        const struct timespec *earliest =
            ((next_hb.tv_sec < next_data.tv_sec) ||
             (next_hb.tv_sec == next_data.tv_sec && next_hb.tv_nsec < next_data.tv_nsec))
            ? &next_hb : &next_data;
        long wait_ns = (earliest->tv_sec - now.tv_sec) * 1000000000L
                     + (earliest->tv_nsec - now.tv_nsec);
        if (wait_ns < 0) wait_ns = 0;

        int r = imud_read(stream, (int)((wait_ns + 999999L) / 1000000L));
        if (r < 0) {
            LOG_I("[mavlink] stream disconnected\n");
            imud_free(stream); stream = NULL; have_pkt = false;
            continue;
        }
        if (r == 0) {
            latest   = *imud_wire(stream);
            have_pkt = true;
        }
        /* r == 1: deadline reached (or a signal) — fall through to senders. */

        uint8_t sid = (uint8_t)cfg.mav_system_id;
        uint8_t cid = (uint8_t)cfg.mav_component_id;
        uint8_t buf[MAV_MAX_FRAME];
        int n;

        clock_gettime(CLOCK_MONOTONIC, &now);

        /* HEARTBEAT at 1 Hz (independent of data, and of having a packet). */
        if ((now.tv_sec > next_hb.tv_sec) ||
            (now.tv_sec == next_hb.tv_sec && now.tv_nsec >= next_hb.tv_nsec)) {
            for (int k = 0; k < nsinks; k++) {
                n = mav_pack_heartbeat(buf, ver, sid, cid, sinks[k].seq++);
                sink_write(&sinks[k], buf, n);
            }
            do { next_hb.tv_sec += 1; } while (next_hb.tv_sec <= now.tv_sec);
        }

        /* ATTITUDE / ATTITUDE_QUATERNION at rate_hz. */
        if ((now.tv_sec > next_data.tv_sec) ||
            (now.tv_sec == next_data.tv_sec && now.tv_nsec >= next_data.tv_nsec)) {
            if (have_pkt) {
                uint32_t t = boot_ms(&start);
                for (int k = 0; k < nsinks; k++) {
                    if (cfg.mav_send_attitude) {
                        n = mav_pack_attitude(buf, ver, sid, cid, sinks[k].seq++, t,
                                              latest.roll, latest.pitch, latest.yaw,
                                              latest.gyro_x, latest.gyro_y, latest.gyro_z);
                        sink_write(&sinks[k], buf, n);
                    }
                    if (cfg.mav_send_attitude_quaternion) {
                        n = mav_pack_attitude_quaternion(buf, ver, sid, cid, sinks[k].seq++, t,
                                              latest.quat_w, latest.quat_x, latest.quat_y,
                                              latest.quat_z,
                                              latest.gyro_x, latest.gyro_y, latest.gyro_z);
                        sink_write(&sinks[k], buf, n);
                    }
                }
            }
            do {
                next_data.tv_nsec += period_ns;
                while (next_data.tv_nsec >= 1000000000L) {
                    next_data.tv_sec++; next_data.tv_nsec -= 1000000000L;
                }
            } while ((next_data.tv_sec < now.tv_sec) ||
                     (next_data.tv_sec == now.tv_sec && next_data.tv_nsec <= now.tv_nsec));
        }
    }

    LOG_I("[mavlink] shutting down\n");
    imud_free(stream);
    for (int k = 0; k < nsinks; k++) {
        if (sinks[k].srv) netserv_close(sinks[k].srv);
        else              close(sinks[k].fd);
    }
    return 0;
}
