/*
 * imud_client.h — single-header C client library for imud Stream B
 *
 * Drop this file into any C project.  No other dependencies beyond POSIX.
 *
 * QUICK START
 *
 *   #define IMUD_CLIENT_IMPLEMENTATION   // in exactly ONE .c file
 *   #include "imud_client.h"
 *
 *   int fd = imud_open(10111, NULL);     // NULL = bind to 0.0.0.0
 *   imud_packet_t pkt;
 *   while (imud_recv(fd, &pkt) == 0)
 *       printf("hdg=%.1f  rot=%.1f dpm\n", pkt.heading_deg, pkt.rate_of_turn);
 *   imud_close(fd);
 *
 * For multicast (default hi-rate address 239.255.0.1):
 *
 *   int fd = imud_open(10111, "239.255.0.1");
 *
 * In all other translation units just #include without the #define.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IMUD_CLIENT_H
#define IMUD_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Protocol constants ──────────────────────────────────────────────────── */

#define IMUD_MAGIC        0x494D5544u   /* "IMUD" */
#define IMUD_VERSION      10   /* 1.0 — encode as decimal: major*10 + minor */
#define IMUD_PACKET_SIZE  192           /* bytes, fixed */

/* ── Packet flags (bitmask in imud_packet_t.flags) ──────────────────────── */

#define IMUD_FLAG_MAG_VALID        (1u << 0)  /* mag healthy and calibrated */
#define IMUD_FLAG_MAG_SET_RESET    (1u << 1)  /* SET pulse within last read */
#define IMUD_FLAG_FUSION_CONVERGED (1u << 2)  /* MEKF covariance settled */
#define IMUD_FLAG_ACCEL_CAL        (1u << 3)  /* accel calibration applied */
#define IMUD_FLAG_GYRO_CAL         (1u << 4)  /* gyro bias applied */
#define IMUD_FLAG_MAG_CAL          (1u << 5)  /* mag hard/soft-iron cal applied */
#define IMUD_FLAG_MOTION           (1u << 6)  /* significant motion detected */
#define IMUD_FLAG_FIFO_OVERFLOW    (1u << 7)  /* sample gap (ISM330 FIFO overflow) */
#define IMUD_FLAG_STARTUP              (1u << 8)  /* gyro bias estimation in progress */
#define IMUD_FLAG_SHUTDOWN             (1u << 9)  /* final packet before clean exit */
#define IMUD_FLAG_DECLINATION_VALID    (1u << 10) /* declination known; true_heading valid */

/* ── Wire packet — 192 bytes, little-endian ─────────────────────────────── */

#if defined(_MSC_VER)
#  pragma pack(push, 1)
#  define IMUD__PACKED
#elif defined(__GNUC__) || defined(__clang__)
#  define IMUD__PACKED __attribute__((packed))
#else
#  define IMUD__PACKED
#endif

typedef struct IMUD__PACKED {
    /* Header */
    uint32_t magic;           /* IMUD_MAGIC */
    uint16_t version;         /* IMUD_VERSION */
    uint16_t flags;           /* IMUD_FLAG_* bitmask */
    uint64_t ts_wall_ns;      /* CLOCK_REALTIME, nanoseconds */
    uint64_t ts_tai_ns;       /* CLOCK_TAI, nanoseconds */
    uint32_t ts_chip_ticks;   /* IMU hardware counter (25 µs/tick for ISM330) */
    uint32_t anchor_gen;      /* increments on wall-clock re-anchor */
    /* Accelerometer — m/s² */
    float accel_x;            /* calibrated (cal-corrected) */
    float accel_y;
    float accel_z;
    float accel_raw_x;        /* pre-calibration, after mount rotation */
    float accel_raw_y;
    float accel_raw_z;
    /* Gyroscope — rad/s */
    float gyro_x;             /* bias-corrected (fused) */
    float gyro_y;
    float gyro_z;
    float gyro_raw_x;         /* before bias correction */
    float gyro_raw_y;
    float gyro_raw_z;
    /* Magnetometer — µT */
    float mag_x;              /* calibrated (hard/soft-iron corrected) */
    float mag_y;
    float mag_z;
    float mag_raw_x;          /* pre-calibration, after mount rotation */
    float mag_raw_y;
    float mag_raw_z;
    /* Fused attitude */
    float quat_w;             /* unit quaternion [w, x, y, z], body→NED */
    float quat_x;
    float quat_y;
    float quat_z;
    float pitch;              /* rad, NED (+bow up) */
    float roll;               /* rad, NED (+starboard up) */
    float yaw;                /* rad, NED magnetic */
    float heading_deg;        /* 0–360° magnetic */
    float rate_of_turn;       /* deg/min, + = turning right */
    float temp_c;             /* IMU die temperature, °C */
    float cov[9];             /* 3×3 attitude error covariance, row-major (rad²) */
    uint32_t imu_seq;         /* monotonic sample counter */
    float    declination_deg; /* °E+; 0.0 when IMUD_FLAG_DECLINATION_VALID not set */
    uint32_t crc32;           /* IEEE 802.3 CRC32 of bytes 0–187 */
} imud_packet_t;

#if defined(_MSC_VER)
#  pragma pack(pop)
#endif

/* Compile-time size guard — will error if the struct is mis-packed. */
typedef char imud__size_check[sizeof(imud_packet_t) == IMUD_PACKET_SIZE ? 1 : -1];

/* ── Inline helpers ──────────────────────────────────────────────────────── */

/*
 * imud_true_heading — compute true (geographic) heading from a packet.
 *
 * Returns the true heading in [0, 360) when IMUD_FLAG_DECLINATION_VALID is
 * set, or -1.0f if the declination is not yet known.
 *
 * No <math.h> required.
 */
static inline float imud_true_heading(const imud_packet_t *pkt)
{
    if (!(pkt->flags & IMUD_FLAG_DECLINATION_VALID))
        return -1.0f;
    /* heading_deg ∈ [0,360) + declination ∈ (-360,360) + 360 → h > 0 always;
     * the while handles the h ≥ 360 case (e.g. heading=359° + decl=+5°). */
    float h = pkt->heading_deg + pkt->declination_deg + 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * imud_open — open a UDP receive socket on the given port.
 *
 * dest_addr: if NULL or empty, binds to INADDR_ANY (receives unicast and
 *            broadcast). If it is a multicast address (224.0.0.0/4) the
 *            socket joins that group and receives multicast packets.
 *
 * Returns a file descriptor on success, -1 on error (errno set).
 */
int imud_open(int port, const char *dest_addr);

/*
 * imud_recv — receive one valid imud binary packet (blocking).
 *
 * Discards any datagram that is the wrong size, has the wrong magic, or
 * fails the CRC32 check.  Blocks until a valid packet arrives.
 *
 * Returns 0 on success, -1 on socket error.
 */
int imud_recv(int fd, imud_packet_t *pkt);

/*
 * imud_packet_valid — validate magic, size, version, and CRC32.
 *
 * Use this if you already have a raw buffer (e.g. from your own recvfrom).
 * buf must point to exactly IMUD_PACKET_SIZE bytes.
 *
 * Returns true if the packet is well-formed and the CRC matches.
 */
bool imud_packet_valid(const void *buf, size_t len);

/*
 * imud_close — close the receive socket.
 */
void imud_close(int fd);

#endif /* IMUD_CLIENT_H */


/* ══════════════════════════════════════════════════════════════════════════
 * IMPLEMENTATION — define IMUD_CLIENT_IMPLEMENTATION before including
 * this header in exactly one translation unit.
 * ══════════════════════════════════════════════════════════════════════════ */

#ifdef IMUD_CLIENT_IMPLEMENTATION

/* Expose ip_mreq and POSIX socket extensions. */
#ifndef _DEFAULT_SOURCE
#  define _DEFAULT_SOURCE
#endif

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ── CRC32 (IEEE 802.3 polynomial 0xEDB88320) ────────────────────────────── */

static uint32_t imud__crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── Validation ──────────────────────────────────────────────────────────── */

bool imud_packet_valid(const void *buf, size_t len)
{
    if (len != IMUD_PACKET_SIZE)
        return false;

    const imud_packet_t *p = (const imud_packet_t *)buf;

    /* Check magic and version before touching the rest of the packet. */
    uint32_t magic;
    memcpy(&magic, buf, 4);
    /* On big-endian hosts, byte-swap; on little-endian (Pi, x86) it's native. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    magic = __builtin_bswap32(magic);
#endif
    if (magic != IMUD_MAGIC)
        return false;

    uint16_t version;
    memcpy(&version, (const uint8_t *)buf + 4, 2);
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    version = __builtin_bswap16(version);
#endif
    if (version != IMUD_VERSION)
        return false;

    /* CRC covers bytes 0..(IMUD_PACKET_SIZE - 5), i.e. everything before crc32. */
    uint32_t computed = imud__crc32((const uint8_t *)buf,
                                     offsetof(imud_packet_t, crc32));
    return computed == p->crc32;
}

/* ── Socket helpers ──────────────────────────────────────────────────────── */

int imud_open(int port, const char *dest_addr)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        return -1;

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons((uint16_t)port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        close(fd);
        return -1;
    }

    /* Join multicast group if address is in 224.0.0.0/4.
     * ip_mreq is two uint32_t in network order (mcast addr, iface addr);
     * avoid struct ip_mreq to sidestep feature-test-macro ordering issues. */
    if (dest_addr && dest_addr[0] != '\0') {
        struct in_addr grp;
        if (inet_pton(AF_INET, dest_addr, &grp) == 1 &&
            (ntohl(grp.s_addr) >> 28) == 0xEu) {
            uint32_t mreq[2];
            mreq[0] = grp.s_addr;
            mreq[1] = htonl(INADDR_ANY);
            if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                           mreq, sizeof(mreq)) < 0) {
                close(fd);
                return -1;
            }
        }
    }

    return fd;
}

int imud_recv(int fd, imud_packet_t *pkt)
{
    uint8_t buf[IMUD_PACKET_SIZE + 1];  /* +1 to detect oversized datagrams */
    for (;;) {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n < 0)
            return -1;
        if (imud_packet_valid(buf, (size_t)n)) {
            memcpy(pkt, buf, IMUD_PACKET_SIZE);
            return 0;
        }
        /* Invalid packet: silently discard and wait for the next one. */
    }
}

void imud_close(int fd)
{
    close(fd);
}

#endif /* IMUD_CLIENT_IMPLEMENTATION */
