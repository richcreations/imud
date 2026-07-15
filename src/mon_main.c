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

#include "config.h"
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
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

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

/* ── CRC32 (IEEE 802.3) ──────────────────────────────────────────────────── */

static uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── Snapshot state ──────────────────────────────────────────────────────── */

#define NMEA_MAX 512

typedef struct {
    /* NMEA — most recent burst, parsed fields */
    char  nmea_raw[NMEA_MAX];
    float nmea_hdg, nmea_roll, nmea_pitch, nmea_rot;
    float nmea_true_hdg;
    bool  have_nmea;
    bool  nmea_has_true_hdg;

    /* Binary — most recent valid packet */
    imu_packet_t bin_pkt;
    bool         have_binary;
} mon_state_t;

/* ── NMEA helpers ────────────────────────────────────────────────────────── */

/* Extract a float from field N (0-based) of a single NMEA sentence. */
static bool nmea_get_field(const char *sentence, int field, float *out)
{
    const char *p = strchr(sentence, ',');
    for (int i = 0; i < field; i++) {
        if (!p) return false;
        p = strchr(p + 1, ',');
    }
    if (!p) return false;
    p++;
    if (!*p || *p == ',' || *p == '*') return false;
    *out = strtof(p, NULL);
    return true;
}

/*
 * Parse a received NMEA buffer that may contain up to 6 sentences.
 * Extracts heading/roll/pitch from $PASHR and rate-of-turn from $TIROT.
 *
 * PASHR layout: $PASHR,hdg,M,roll,pitch,...
 *   field 0 = heading, field 2 = roll, field 3 = pitch
 */
static void parse_nmea(mon_state_t *st, const char *buf, size_t len)
{
    if (len >= NMEA_MAX) len = NMEA_MAX - 1;
    memcpy(st->nmea_raw, buf, len);
    st->nmea_raw[len] = '\0';
    st->have_nmea = true;
    st->nmea_has_true_hdg = false;   /* cleared each burst; set below if $HCHDT present */

    const char *pas = strstr(st->nmea_raw, "$PASHR,");
    if (pas) {
        float hdg = 0, roll = 0, pitch = 0;
        if (sscanf(pas + 7, "%f,M,%f,%f", &hdg, &roll, &pitch) == 3) {
            st->nmea_hdg   = hdg;
            st->nmea_roll  = roll;
            st->nmea_pitch = pitch;
        }
    }

    const char *rot = strstr(st->nmea_raw, "$TIROT,");
    if (rot) {
        float r = 0;
        if (nmea_get_field(rot, 0, &r))
            st->nmea_rot = r;
    }

    const char *hdt = strstr(st->nmea_raw, "$HCHDT,");
    if (hdt) {
        float h = 0;
        if (nmea_get_field(hdt, 0, &h)) {
            st->nmea_true_hdg    = h;
            st->nmea_has_true_hdg = true;
        }
    }
}

/* ── Display ─────────────────────────────────────────────────────────────── */

static void flag_str(uint16_t flags, char *buf, size_t sz)
{
    /* Compact flag summary: C=converged V=mag-valid A=accel-cal G=gyro-cal M=mag-cal */
    int n = snprintf(buf, sz, "0x%04X [", flags);
    if (n < 0 || (size_t)n >= sz) return;
    char *p = buf + n; size_t r = sz - (size_t)n;
    if (flags & FLAG_FUSION_CONVERGED)  { snprintf(p, r, "C");  p++; r--; }
    if (flags & FLAG_MAG_VALID)         { snprintf(p, r, "V");  p++; r--; }
    if (flags & FLAG_ACCEL_CAL)         { snprintf(p, r, "A");  p++; r--; }
    if (flags & FLAG_GYRO_CAL)          { snprintf(p, r, "G");  p++; r--; }
    if (flags & FLAG_MAG_CAL)           { snprintf(p, r, "M");  p++; r--; }
    if (flags & FLAG_DECLINATION_VALID) { snprintf(p, r, "D");  p++; r--; }
    if (flags & FLAG_STARTUP)           { snprintf(p, r, "S");  p++; r--; }
    if (flags & FLAG_FIFO_OVERFLOW)     { snprintf(p, r, "!"); }
    strncat(buf, "]", sz - strlen(buf) - 1);
}

static void print_snapshot(const mon_state_t *st,
                            bool want_nmea, bool want_binary)
{
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    printf("─── %02d:%02d:%02d ────────────────────────────────────────────────\n",
           t->tm_hour, t->tm_min, t->tm_sec);

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
            flag_str(p->flags, fstr, sizeof fstr);
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

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--config PATH] [nmea] [binary]\n"
        "\n"
        "  --config PATH  Config file (default: /etc/imud/imud.conf)\n"
        "\n"
        "  nmea    Monitor NMEA 0183 stream (UDP port 10110)\n"
        "  binary  Monitor binary stream    (UDP port 10111)\n"
        "\n"
        "  With no stream arguments both streams are shown.\n"
        "  Port numbers and multicast addresses are read from the config file.\n",
        prog);
}

int main(int argc, char **argv)
{
    char config_path[256];
    snprintf(config_path, sizeof config_path, "/etc/imud/imud.conf");

    bool want_nmea = false, want_binary = false;
    bool any_stream = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof config_path, "%s", argv[++i]);
        } else if (strcmp(argv[i], "nmea") == 0) {
            want_nmea   = true; any_stream = true;
        } else if (strcmp(argv[i], "binary") == 0) {
            want_binary = true; any_stream = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]); return 0;
        } else {
            fprintf(stderr, "unknown argument: %s\n", argv[i]);
            usage(argv[0]); return 1;
        }
    }

    if (!any_stream)
        want_nmea = want_binary = true;

    /* Load config — ignore failure, defaults have the right port numbers */
    imud_config_t cfg;
    config_defaults(&cfg);
    config_load(config_path, &cfg);

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
                parse_nmea(&st, dgram, (size_t)n);
        }
        if (bin_fd >= 0 && FD_ISSET(bin_fd, &rfds)) {
            while ((n = recv(bin_fd, dgram, sizeof dgram, MSG_DONTWAIT)) > 0) {
                if (!st.have_binary && (size_t)n == sizeof(imu_packet_t)) {
                    imu_packet_t *p = (imu_packet_t *)(void *)dgram;
                    if (p->magic == IMUD_MAGIC &&
                        crc32_ieee((const uint8_t *)p, offsetof(imu_packet_t, crc32)) == p->crc32) {
                        st.bin_pkt     = *p;
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
