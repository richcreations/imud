/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * libimud.c — implementation of the libimud shared library (see imud.h)
 *
 * Wire knowledge (packed struct, constants) comes from imud_client.h,
 * included WITHOUT IMUD_CLIENT_IMPLEMENTATION so none of the single-header's
 * public symbols (imud_open/imud_recv/...) are exported from the library;
 * validation and CRC are private copies below. Timeouts use poll(2) rather
 * than SO_RCVTIMEO so a partial stream frame can span imud_read() calls.
 */

#include "imud_client.h"   /* wire struct + constants; must precede imud.h */
#include "imud.h"
#include "../include/version.h"
#include "../include/crc32.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifndef SOCK_CLOEXEC
# define SOCK_CLOEXEC 0    /* macOS: no atomic CLOEXEC; dev/test builds only */
#endif

#define IMUD_DEFAULT_STREAM_SOCK "/run/imud/imud-stream.sock"
#define IMUD_DEFAULT_UDP_PORT    10111
#define IMUD_DEFAULT_TCP_PORT    10112
#define IMUD_DEFAULT_TCP_HOST    "127.0.0.1"

/* ── Handle ──────────────────────────────────────────────────────────────── */

enum imud_kind { IMUD_KIND_STREAM, IMUD_KIND_UDP, IMUD_KIND_TCP };

struct imud {
    enum imud_kind kind;
    int            fd;                     /* -1 when disconnected */
    /* endpoint (kept for imud_reconnect) */
    char           path[108];              /* AF_UNIX path or TCP host
                                            * (sun_path-sized) */
    int            port;
    char           group[64];              /* multicast group or "" */
    /* stream reassembly: a frame may span multiple reads */
    uint8_t        rx[IMUD_PACKET_SIZE];
    size_t         rx_got;
    /* latest packet */
    imud_packet_t  wire;
    imud_data_t    data;
    unsigned       wire_version;
    int            have;
};

/* ── Packet validation ───────────────────────────────────────────────────── */

static int packet_ok(const uint8_t *buf, size_t len)
{
    if (len != IMUD_PACKET_SIZE)
        return 0;
    uint32_t magic;
    memcpy(&magic, buf, 4);
    if (magic != IMUD_MAGIC)
        return 0;
    uint16_t version;
    memcpy(&version, buf + 4, 2);
    if (version != IMUD_VERSION)
        return 0;
    uint32_t crc;
    memcpy(&crc, buf + offsetof(imud_packet_t, crc32), 4);
    return crc32_ieee(buf, offsetof(imud_packet_t, crc32)) == crc;
}

/* ── Wire → data view ────────────────────────────────────────────────────── */

static void fill_data(imud_data_t *d, const imud_packet_t *p)
{
    d->ts_wall_ns    = p->ts_wall_ns;
    d->ts_tai_ns     = p->ts_tai_ns;
    d->ts_chip_ticks = p->ts_chip_ticks;
    d->anchor_gen    = p->anchor_gen;
    d->flags         = p->flags;
    d->imu_seq       = p->imu_seq;

    d->accel[0] = p->accel_x;         d->accel[1] = p->accel_y;         d->accel[2] = p->accel_z;
    d->accel_raw[0] = p->accel_raw_x; d->accel_raw[1] = p->accel_raw_y; d->accel_raw[2] = p->accel_raw_z;
    d->gyro[0]  = p->gyro_x;          d->gyro[1]  = p->gyro_y;          d->gyro[2]  = p->gyro_z;
    d->gyro_raw[0] = p->gyro_raw_x;   d->gyro_raw[1] = p->gyro_raw_y;   d->gyro_raw[2] = p->gyro_raw_z;
    d->mag[0]   = p->mag_x;           d->mag[1]   = p->mag_y;           d->mag[2]   = p->mag_z;
    d->mag_raw[0] = p->mag_raw_x;     d->mag_raw[1] = p->mag_raw_y;     d->mag_raw[2] = p->mag_raw_z;

    d->quat[0] = p->quat_w; d->quat[1] = p->quat_x;
    d->quat[2] = p->quat_y; d->quat[3] = p->quat_z;
    d->pitch = p->pitch;
    d->roll  = p->roll;
    d->yaw   = p->yaw;
    d->heading_deg      = p->heading_deg;
    d->heading_true_deg = imud_true_heading(p);   /* -1.0 until declination valid */
    d->rate_of_turn     = p->rate_of_turn;
    d->temp_c           = p->temp_c;
    memcpy(d->cov, p->cov, sizeof d->cov);
    d->declination_deg  = p->declination_deg;
    d->heave_m          = p->heave_m;
    d->heave_rate       = p->heave_rate;
    d->gyro_bias[0]     = p->gyro_bias_x;
    d->gyro_bias[1]     = p->gyro_bias_y;
    d->gyro_bias[2]     = p->gyro_bias_z;
    d->gyro_bias_var[0] = p->gyro_bias_var_x;
    d->gyro_bias_var[1] = p->gyro_bias_var_y;
    d->gyro_bias_var[2] = p->gyro_bias_var_z;
    d->accel_quiescence = p->accel_quiescence;
    d->wave_height_m    = p->wave_height_m;
    d->wave_period_s    = p->wave_period_s;
    d->roll_period_s    = p->roll_period_s;
    d->roll_amplitude   = p->roll_amplitude;
    d->pitch_period_s   = p->pitch_period_s;
    d->pitch_amplitude  = p->pitch_amplitude;
    d->mag_anomaly      = p->mag_anomaly;
    d->mag_residual     = p->mag_residual;
    d->innov_weight     = p->innov_weight;
    d->innov_reject     = p->innov_reject;
    d->nis_accel        = p->nis_accel;
    d->nis_mag          = p->nis_mag;
}

/* ── Transport dialing ───────────────────────────────────────────────────── */

static int dial_stream(const char *path)
{
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0)
        return -1;
    struct sockaddr_un addr;
    size_t plen = strlen(path);
    if (plen >= sizeof addr.sun_path) {   /* imud_connect_stream already bounds
                                           * h->path; belt-and-braces */
        close(fd);
        errno = ENAMETOOLONG;
        return -1;
    }
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, path, plen);   /* addr is zeroed → NUL-terminated */
    if (connect(fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        int e = errno;
        close(fd);
        errno = e;
        return -1;
    }
    return fd;
}

static int dial_udp(int port, const char *group)
{
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0)
        return -1;

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof bind_addr);
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons((uint16_t)port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&bind_addr, sizeof bind_addr) < 0) {
        int e = errno;
        close(fd);
        errno = e;
        return -1;
    }

    /* Join the multicast group when the address is in 224.0.0.0/4. */
    if (group && group[0] != '\0') {
        struct in_addr grp;
        if (inet_pton(AF_INET, group, &grp) == 1 &&
            (ntohl(grp.s_addr) >> 28) == 0xEu) {
            uint32_t mreq[2] = { grp.s_addr, htonl(INADDR_ANY) };
            if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                           mreq, sizeof mreq) < 0) {
                int e = errno;
                close(fd);
                errno = e;
                return -1;
            }
        }
    }
    return fd;
}

static int dial_tcp(const char *host, int port)
{
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_INET;      /* the daemon's listener is IPv4 */
    hints.ai_socktype = SOCK_STREAM;
    int gai = getaddrinfo(host, portstr, &hints, &res);
    if (gai != 0 || !res) {
        errno = (gai == EAI_SYSTEM) ? errno : EHOSTUNREACH;
        return -1;
    }
    int fd = -1;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype | SOCK_CLOEXEC, 0);
        if (fd < 0)
            continue;
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0)
            break;
        int e = errno;
        close(fd);
        fd = -1;
        errno = e;
    }
    freeaddrinfo(res);
    return fd;
}

static int dial(imud_t *h)
{
    h->rx_got = 0;
    switch (h->kind) {
    case IMUD_KIND_STREAM: h->fd = dial_stream(h->path);         break;
    case IMUD_KIND_TCP:    h->fd = dial_tcp(h->path, h->port);   break;
    default:               h->fd = dial_udp(h->port, h->group);  break;
    }
    return h->fd < 0 ? -1 : 0;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

imud_t *imud_connect_stream(const char *path)
{
    imud_t *h = calloc(1, sizeof *h);
    if (!h)
        return NULL;
    h->kind = IMUD_KIND_STREAM;
    h->data.heading_true_deg = -1.0f;
    /* Reject an over-long path rather than silently truncating it and then
     * failing to connect to a path the caller never asked for.  h->path is
     * sun_path-sized, so anything that doesn't fit here can't be bound either. */
    int pn = snprintf(h->path, sizeof h->path, "%s",
                      (path && path[0]) ? path : IMUD_DEFAULT_STREAM_SOCK);
    if (pn < 0 || (size_t)pn >= sizeof h->path) {
        free(h);
        errno = ENAMETOOLONG;
        return NULL;
    }
    if (dial(h) < 0) {
        int e = errno;
        free(h);
        errno = e;
        return NULL;
    }
    return h;
}

imud_t *imud_connect_tcp(const char *host, int port)
{
    imud_t *h = calloc(1, sizeof *h);
    if (!h)
        return NULL;
    h->kind = IMUD_KIND_TCP;
    h->data.heading_true_deg = -1.0f;
    h->port = port > 0 ? port : IMUD_DEFAULT_TCP_PORT;
    /* Reject an over-long host rather than silently truncating it and then
     * connecting somewhere the caller never asked for. */
    int hn = snprintf(h->path, sizeof h->path, "%s",
                      (host && host[0]) ? host : IMUD_DEFAULT_TCP_HOST);
    if (hn < 0 || (size_t)hn >= sizeof h->path) {
        free(h);
        errno = ENAMETOOLONG;
        return NULL;
    }
    if (dial(h) < 0) {
        int e = errno;
        free(h);
        errno = e;
        return NULL;
    }
    return h;
}

imud_t *imud_connect_udp(int port, const char *group)
{
    imud_t *h = calloc(1, sizeof *h);
    if (!h)
        return NULL;
    h->kind = IMUD_KIND_UDP;
    h->data.heading_true_deg = -1.0f;
    h->port = port > 0 ? port : IMUD_DEFAULT_UDP_PORT;
    snprintf(h->group, sizeof h->group, "%s", group ? group : "");
    if (dial(h) < 0) {
        int e = errno;
        free(h);
        errno = e;
        return NULL;
    }
    return h;
}

/* Milliseconds until `deadline`, clamped at 0; -1 when no deadline. */
static int ms_left(const struct timespec *deadline)
{
    if (!deadline)
        return -1;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long long ms = (deadline->tv_sec - now.tv_sec) * 1000LL +
                   (deadline->tv_nsec - now.tv_nsec) / 1000000LL;
    return ms > 0 ? (int)ms : 0;
}

int imud_read(imud_t *h, int timeout_ms)
{
    if (!h || h->fd < 0)
        return -1;

    struct timespec deadline, *dl = NULL;
    if (timeout_ms >= 0) {
        clock_gettime(CLOCK_MONOTONIC, &deadline);
        deadline.tv_sec  += timeout_ms / 1000;
        deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec++;
            deadline.tv_nsec -= 1000000000L;
        }
        dl = &deadline;
    }

    for (;;) {
        struct pollfd pfd = { .fd = h->fd, .events = POLLIN };
        int pr = poll(&pfd, 1, ms_left(dl));
        if (pr == 0)
            return 1;                       /* timeout */
        if (pr < 0) {
            if (errno == EINTR)
                return 1;                   /* let the caller check its flags */
            return -1;
        }

        if (h->kind == IMUD_KIND_UDP) {
            uint8_t buf[IMUD_PACKET_SIZE + 1];  /* +1 detects oversized datagrams */
            ssize_t n = recv(h->fd, buf, sizeof buf, 0);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN)
                    continue;
                return -1;
            }
            if (!packet_ok(buf, (size_t)n))
                continue;                   /* silently discard */
            memcpy(&h->wire, buf, IMUD_PACKET_SIZE);
        } else {
            ssize_t n = read(h->fd, h->rx + h->rx_got,
                             IMUD_PACKET_SIZE - h->rx_got);
            if (n == 0)
                return -1;                  /* daemon closed the stream */
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN)
                    continue;
                return -1;
            }
            h->rx_got += (size_t)n;
            if (h->rx_got < IMUD_PACKET_SIZE)
                continue;                   /* frame not complete yet */
            h->rx_got = 0;
            if (!packet_ok(h->rx, IMUD_PACKET_SIZE))
                continue;                   /* corrupt frame: drop, stay framed */
            memcpy(&h->wire, h->rx, IMUD_PACKET_SIZE);
        }

        fill_data(&h->data, &h->wire);
        h->wire_version = h->wire.version;
        h->have = 1;
        return 0;
    }
}

int imud_reconnect(imud_t *h)
{
    if (!h)
        return -1;
    if (h->fd >= 0) {
        close(h->fd);
        h->fd = -1;
    }
    return dial(h);
}

const imud_data_t *imud_data(const imud_t *h)
{
    return h ? &h->data : NULL;
}

const imud_packet_t *imud_wire(const imud_t *h)
{
    return h ? &h->wire : NULL;
}

int imud_fd(const imud_t *h)
{
    return h ? h->fd : -1;
}

void imud_free(imud_t *h)
{
    if (!h)
        return;
    if (h->fd >= 0)
        close(h->fd);
    free(h);
}

const char *imud_lib_version(void)
{
    return IMUD_VERSION_STR;
}

unsigned imud_wire_version(const imud_t *h)
{
    return h ? h->wire_version : 0;
}
