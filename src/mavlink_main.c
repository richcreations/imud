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
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>

#include "../lib/imud_client.h"
#include "../lib/imud.h"        /* libimud: stream connect/read/validate */
#include "mavlink_encode.h"
#include "bridge.h"             /* shared bridge scaffolding */
#include "sdnotify.h"
#include "config.h"
#include "netserv.h"
#include "log.h"

static const bridge_info_t BI = {
    .prog         = "imud-mavlink",
    .tag          = "mavlink",
    .section      = "imud-mavlink",
    .default_conf = "/etc/imud/imud-mavlink.conf",
    .usage_desc   =
        "  MAVLink bridge: reads imud's stream socket and emits MAVLink\n"
        "  HEARTBEAT/ATTITUDE/ATTITUDE_QUATERNION over UDP and/or serial.\n"
        "  Configured by [imud-mavlink] in its own file.\n",
};

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

/* ── Transports ──────────────────────────────────────────────────────────── */

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

int main(int argc, char **argv)
{
    char config_path[256];
    int rc = bridge_parse_cli(argc, argv, &BI, config_path, sizeof config_path);
    if (rc != 0) return rc < 0 ? 1 : 0;

    imud_config_t cfg;
    if (bridge_load_config(&BI, config_path, &cfg) < 0) return 1;

    if (!cfg.mav_enabled) {
        bridge_exit_disabled(&BI);
        return 0;
    }

    bridge_install_signals();

    int  ver       = (cfg.mav_version == 1) ? 1 : 2;
    long period_ns = bridge_period_ns(cfg.mav_rate_hz, 100000000L);

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
        s->fd = bridge_open_udp(cfg.mav_udp_addr, cfg.mav_udp_port, BI.tag,
                                &s->dst, &s->dlen);
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

    while (!bridge_stop) {
        sd_notify_msg("WATCHDOG=1");

        imud_config_t nc;
        if (bridge_reload_begin(&BI, config_path, &nc)) {
            ver       = (nc.mav_version == 1) ? 1 : 2;
            period_ns = bridge_period_ns(nc.mav_rate_hz, 100000000L);
            if (udp_idx >= 0 &&
                (strcmp(nc.mav_udp_addr, cfg.mav_udp_addr) != 0 ||
                 nc.mav_udp_port != cfg.mav_udp_port)) {
                close(sinks[udp_idx].fd);
                uint8_t keep_seq = sinks[udp_idx].seq;
                sinks[udp_idx].fd = bridge_open_udp(nc.mav_udp_addr, nc.mav_udp_port,
                                                    BI.tag, &sinks[udp_idx].dst,
                                                    &sinks[udp_idx].dlen);
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
            bridge_reload_done(&BI);
        }

        /* Accept pending TCP clients (no-op when tcp_enabled = false). */
        netserv_accept(&tcp_srv);

        if (bridge_stream_ensure(&stream, cfg.stream_socket, BI.tag, 2) == 0) {
            have_pkt = false;
            continue;
        }

        /* Wait until the nearer of the two deadlines, draining stream frames. */
        clock_gettime(CLOCK_MONOTONIC, &now);
        const struct timespec *earliest = bridge_earlier(&next_hb, &next_data);

        int r = imud_read(stream, bridge_wait_ms(&now, earliest));
        if (r < 0) {
            bridge_stream_drop(&stream, BI.tag);
            have_pkt = false;
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
        if (bridge_due(&now, &next_hb)) {
            for (int k = 0; k < nsinks; k++) {
                n = mav_pack_heartbeat(buf, ver, sid, cid, sinks[k].seq++);
                sink_write(&sinks[k], buf, n);
            }
            do { next_hb.tv_sec += 1; } while (next_hb.tv_sec <= now.tv_sec);
        }

        /* ATTITUDE / ATTITUDE_QUATERNION at rate_hz. */
        if (bridge_due(&now, &next_data)) {
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
            bridge_advance(&next_data, &now, period_ns);
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
