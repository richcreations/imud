/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mon_main.c — imud-mon: live display of imud UDP output streams
 *
 * Connects to the NMEA and/or binary UDP streams exactly as any other
 * consumer would.  Prints a one-line summary per stream once per second.
 *
 * Both streams show the first packet received in each 1-second window, so
 * NMEA and binary headings will agree at normal yaw rates.
 *
 * Usage: imud-mon [--config PATH] [nmea] [binary]
 *   With no stream arguments both streams are shown.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>

#include "cli.h"
#include "cloexec.h"
#include "config.h"
#include "mon_parse.h"
#include "packet.h"
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }

/* ── UDP receive socket ──────────────────────────────────────────────────── */

/*
 * Open a non-blocking UDP socket bound to INADDR_ANY:port.
 * If dest_addr is a multicast address (224.0.0.0/4) the socket joins the group
 * so it receives multicast packets; broadcast and unicast work with no extra
 * setup beyond the bind.
 */
static int open_recv_sock(int port, const char *dest_addr)
{
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    APPLY_CLOEXEC(fd);

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
#ifdef SO_REUSEPORT
    setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &one, sizeof one);
#endif

    struct sockaddr_in sa = {
        .sin_family      = AF_INET,
        .sin_port        = htons((uint16_t)port),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(fd, (struct sockaddr *)&sa, sizeof sa) < 0) {
        close(fd);
        return -1;
    }

    /* Join multicast group if the destination is in 224.0.0.0/4 */
    if (dest_addr && dest_addr[0]) {
        struct in_addr ia;
        if (inet_aton(dest_addr, &ia) &&
            (ntohl(ia.s_addr) >> 28) == 0xEu) {
            struct ip_mreq mreq = {
                .imr_multiaddr        = ia,
                .imr_interface.s_addr = INADDR_ANY,
            };
            setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       &mreq, sizeof mreq);
        }
    }

    /* Non-blocking: we drain with MSG_DONTWAIT after select() unblocks */
    fcntl(fd, F_SETFL, O_NONBLOCK);
    return fd;
}

/* ── Display ─────────────────────────────────────────────────────────────── */

static void print_snapshot(const mon_state_t *st,
                            bool want_nmea, bool want_binary)
{
    time_t now = time(NULL);
    struct tm tmbuf;

    /* localtime_r, not localtime(): the latter returns a pointer into a
       shared static buffer and NULL on an out-of-range time_t, which the
       old code dereferenced unchecked. */
    if (!localtime_r(&now, &tmbuf))
        memset(&tmbuf, 0, sizeof tmbuf);

    printf("─── %02d:%02d:%02d ────────────────────────────────────────────────\n",
           tmbuf.tm_hour, tmbuf.tm_min, tmbuf.tm_sec);

    if (want_nmea) {
        if (st->have_nmea) {
            printf("NMEA    hdg=%6.1f°  pitch=%+6.1f°  roll=%+6.1f°"
                   "  rot=%+7.1f dpm\n",
                   st->nmea_hdg, st->nmea_pitch, st->nmea_roll, st->nmea_rot);
            if (st->nmea_has_true_hdg)
                printf("        true_hdg=%6.1f°  ($HCHDT)\n", st->nmea_true_hdg);
        } else {
            printf("NMEA    (no data)\n");
        }
    }

    if (want_binary) {
        if (st->have_binary) {
            const imu_packet_t *p = &st->bin_pkt;
            float pitch_d   = p->pitch * (float)(180.0 / M_PI);
            float roll_d    = p->roll  * (float)(180.0 / M_PI);
            float cov_trace = p->cov[0] + p->cov[4] + p->cov[8];
            char  fstr[32];
            mon_flag_str(p->flags, fstr, sizeof fstr);
            /* Line 1: fused attitude */
            printf("Binary  hdg=%6.1f°  pitch=%+6.1f°  roll=%+6.1f°"
                   "  rot=%+7.1f dpm  seq=%-8u  cov=%.1e  %s\n",
                   p->heading_deg, pitch_d, roll_d, p->rate_of_turn,
                   p->imu_seq, (double)cov_trace, fstr);
            if (p->flags & FLAG_DECLINATION_VALID) {
                float true_hdg = fmodf(p->heading_deg + p->declination_deg + 360.0f,
                                       360.0f);
                printf("        true_hdg=%6.1f°  decl=%+.1f°\n",
                       true_hdg, p->declination_deg);
            }
            /* Line 2: raw vs fused gyro; Line 3: raw accel/mag */
            printf("        gyro_raw=[%+7.4f %+7.4f %+7.4f] rad/s"
                   "  fused=[%+7.4f %+7.4f %+7.4f]  temp=%.1f°C\n",
                   p->gyro_raw_x, p->gyro_raw_y, p->gyro_raw_z,
                   p->gyro_x,     p->gyro_y,     p->gyro_z,
                   p->temp_c);
            printf("        accel_raw=[%+6.3f %+6.3f %+6.3f] m/s²"
                   "  mag_raw=[%+6.1f %+6.1f %+6.1f] µT\n",
                   p->accel_raw_x, p->accel_raw_y, p->accel_raw_z,
                   p->mag_raw_x,   p->mag_raw_y,   p->mag_raw_z);
        } else {
            printf("Binary  (no data)\n");
        }
    }

    fflush(stdout);
}

/* ── Entry point ─────────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    cli_mon_t args;
    int cli_rc = cli_parse_mon(argc, argv, &args);
    if (cli_rc != 0) return cli_rc < 0 ? 1 : 0;   /* -1 bad usage, 1 --help */

    const bool want_nmea   = args.want_nmea;
    const bool want_binary = args.want_binary;

    /* Load config — ignore failure, defaults have the right port numbers */
    imud_config_t cfg;
    config_defaults(&cfg);
    config_load(args.config_path, &cfg);

    /* Open receive sockets */
    int nmea_fd = -1, bin_fd = -1;

    if (want_nmea) {
        nmea_fd = open_recv_sock(cfg.nmea_dest_port, cfg.nmea_dest_addr);
        if (nmea_fd < 0)
            fprintf(stderr, "mon: NMEA socket (port %d): %s\n",
                    cfg.nmea_dest_port, strerror(errno));
    }
    if (want_binary) {
        bin_fd = open_recv_sock(cfg.highrate_dest_port, cfg.highrate_dest_addr);
        if (bin_fd < 0)
            fprintf(stderr, "mon: binary socket (port %d): %s\n",
                    cfg.highrate_dest_port, strerror(errno));
    }

    /* At least one socket must be open */
    if (nmea_fd < 0 && bin_fd < 0) {
        fprintf(stderr, "mon: no streams available — exiting\n");
        return 1;
    }

    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    /* Print header */
    printf("imud-mon — listening on:");
    if (nmea_fd   >= 0) printf("  NMEA:%d",   cfg.nmea_dest_port);
    if (bin_fd    >= 0) printf("  Binary:%d", cfg.highrate_dest_port);
    printf("  (Ctrl-C to stop)\n\n");

    mon_state_t st = {0};

    /* Align first tick to the next whole second */
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_sec++;
    next.tv_nsec = 0;

    char dgram[8192];

    while (!g_stop) {
        /* ── Build fd_set ─────────────────────────────────────────────── */
        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = -1;
        if (nmea_fd >= 0) { FD_SET(nmea_fd, &rfds); if (nmea_fd > maxfd) maxfd = nmea_fd; }
        if (bin_fd  >= 0) { FD_SET(bin_fd,  &rfds); if (bin_fd  > maxfd) maxfd = bin_fd;  }

        /* ── Compute timeout to next 1-second tick ────────────────────── */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long ns = (next.tv_sec  - now.tv_sec ) * 1000000000L
                + (next.tv_nsec - now.tv_nsec);
        if (ns < 0) ns = 0;

        struct timeval tv = {
            .tv_sec  = ns / 1000000000L,
            .tv_usec = (ns % 1000000000L) / 1000,
        };

        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        /* ── Drain all pending datagrams, keeping only the latest ─────── */
        ssize_t n;

        if (nmea_fd >= 0 && FD_ISSET(nmea_fd, &rfds)) {
            while ((n = recv(nmea_fd, dgram, sizeof dgram - 1, MSG_DONTWAIT)) > 0)
                mon_parse_nmea(&st, dgram, (size_t)n);
        }
        if (bin_fd >= 0 && FD_ISSET(bin_fd, &rfds)) {
            while ((n = recv(bin_fd, dgram, sizeof dgram, MSG_DONTWAIT)) > 0) {
                if (!st.have_binary && (size_t)n == IMUD_PACKET_BYTES) {
                    imu_packet_t p;
                    packet_decode(&p, (const uint8_t *)dgram);
                    if (p.magic == IMUD_MAGIC &&
                        mon_crc32_ieee((const uint8_t *)dgram,
                                       offsetof(imu_packet_t, crc32)) == p.crc32) {
                        st.bin_pkt     = p;
                        st.have_binary = true;
                    }
                }
            }
        }

        /* ── Print snapshot at each 1-second tick ─────────────────────── */
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > next.tv_sec ||
            (now.tv_sec == next.tv_sec && now.tv_nsec >= next.tv_nsec)) {
            print_snapshot(&st, want_nmea, want_binary);
            /* Clear flags: if nothing arrives next second we show "no data" */
            st.have_nmea   = false;
            st.have_binary = false;
            next.tv_sec++;
        }
    }

    if (nmea_fd >= 0) close(nmea_fd);
    if (bin_fd  >= 0) close(bin_fd);

    printf("\nimud-mon: stopped\n");
    return 0;
}
