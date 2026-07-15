/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imud.h — public API of libimud, the imud client shared library
 *
 * Link with -limud (pkg-config: libimud). Unlike the single-header
 * imud_client.h (which pins the wire format and needs a recompile per wire
 * revision), this API is ABI-STABLE: applications compiled against it keep
 * working across imud updates without recompilation. Upgrade rule: libimud is
 * built and shipped with the daemon; keep the two installed together.
 *
 * QUICK START
 *
 *   #include <imud.h>
 *
 *   imud_t *h = imud_connect_stream(NULL);      // local socket (recommended)
 *   while (imud_read(h, 1000) >= 0) {           // 0 = data, 1 = timeout
 *       const imud_data_t *d = imud_data(h);
 *       printf("hdg=%.1f  heave=%.2f\n", d->heading_deg, d->heave_m);
 *   }
 *   imud_free(h);
 *
 * ── ABI CONTRACT ──────────────────────────────────────────────────────────
 * imud_data_t is library-owned and APPEND-ONLY:
 *   - new quantities are only ever added after the existing members;
 *     existing members never move, change type, or disappear;
 *   - always access it through the pointer imud_data() returns — do NOT
 *     copy the struct by value or bake sizeof(imud_data_t) into anything
 *     that must survive a library upgrade.
 * Under those rules a binary built against an older imud.h reads its fields
 * correctly from any newer libimud.so.0.
 */
#ifndef IMUD_H
#define IMUD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Packet flags (bitmask in imud_data_t.flags) ──────────────────────────
 * Values are fixed by the wire protocol. Guarded so this header coexists
 * with imud_client.h, which defines the same constants. */
#ifndef IMUD_FLAG_MAG_VALID
#define IMUD_FLAG_MAG_VALID        (1u << 0)  /* mag healthy and calibrated */
#define IMUD_FLAG_MAG_SET_RESET    (1u << 1)  /* SET pulse within last read */
#define IMUD_FLAG_FUSION_CONVERGED (1u << 2)  /* MEKF covariance settled */
#define IMUD_FLAG_ACCEL_CAL        (1u << 3)  /* accel calibration applied */
#define IMUD_FLAG_GYRO_CAL         (1u << 4)  /* gyro bias applied */
#define IMUD_FLAG_MAG_CAL          (1u << 5)  /* mag hard/soft-iron cal applied */
#define IMUD_FLAG_MOTION           (1u << 6)  /* reserved — never set as of wire v14 */
#define IMUD_FLAG_FIFO_OVERFLOW    (1u << 7)  /* sample gap (FIFO overflow) */
#define IMUD_FLAG_STARTUP              (1u << 8)  /* gyro bias estimation in progress */
#define IMUD_FLAG_SHUTDOWN             (1u << 9)  /* final packet before clean exit */
#define IMUD_FLAG_DECLINATION_VALID    (1u << 10) /* declination known */
#define IMUD_FLAG_HEAVE_VALID          (1u << 11) /* heave estimator settled */
#endif
#ifndef IMUD_FLAG_WAVE_VALID
#define IMUD_FLAG_WAVE_VALID           (1u << 12) /* sea-state stats settled */
#endif
#ifndef IMUD_FLAG_ENGINE_ON
#define IMUD_FLAG_ENGINE_ON            (1u << 13) /* engine-vibration detector asserting */
#endif

/* ── The data view — APPEND-ONLY (see ABI contract above) ────────────────── */

typedef struct imud_data {
    /* Timestamps */
    uint64_t ts_wall_ns;       /* CLOCK_REALTIME, nanoseconds */
    uint64_t ts_tai_ns;        /* CLOCK_TAI, nanoseconds */
    uint32_t ts_chip_ticks;    /* IMU hardware counter */
    uint32_t anchor_gen;       /* increments on wall-clock re-anchor */
    uint32_t flags;            /* IMUD_FLAG_* bitmask */
    uint32_t imu_seq;          /* monotonic sample counter */
    /* Sensors — body frame XYZ */
    float accel[3];            /* m/s², calibrated */
    float accel_raw[3];        /* m/s², pre-calibration */
    float gyro[3];             /* rad/s, bias-corrected */
    float gyro_raw[3];         /* rad/s, before bias correction */
    float mag[3];              /* µT, calibrated */
    float mag_raw[3];          /* µT, pre-calibration */
    /* Fused attitude */
    float quat[4];             /* unit quaternion [w, x, y, z], body→NED */
    float pitch;               /* rad, NED (+bow up) */
    float roll;                /* rad, NED (+starboard up) */
    float yaw;                 /* rad, NED magnetic */
    float heading_deg;         /* 0–360° magnetic */
    float heading_true_deg;    /* 0–360° true; -1.0 until declination known */
    float rate_of_turn;        /* deg/min, + = turning right */
    float temp_c;              /* IMU die temperature, °C */
    float cov[9];              /* 3×3 attitude error covariance, row-major (rad²) */
    float declination_deg;     /* °E+; valid with IMUD_FLAG_DECLINATION_VALID */
    /* Heave (0.0 when the daemon's estimator is disabled) */
    float heave_m;             /* vertical displacement, m, + up */
    float heave_rate;          /* vertical velocity, m/s, + up */
    /* Fusion diagnostics — IMU body frame / frame-neutral */
    float gyro_bias[3];        /* estimated gyro bias, rad/s */
    float gyro_bias_var[3];    /* gyro-bias variance, (rad/s)² */
    float accel_quiescence;    /* EMA of (|a|/g − 1)²; disturbance metric */
    /* Sea state (wire v14) — 0.0 until IMUD_FLAG_WAVE_VALID */
    float wave_height_m;       /* significant wave height Hs, m */
    float wave_period_s;       /* mean zero-crossing wave period Tz, s; 0 = n/a */
    float roll_period_s;       /* vessel roll period, s; 0 = not rolling */
    float roll_amplitude;      /* significant single amplitude 2σ(roll), rad */
    float pitch_period_s;      /* vessel pitch period, s; 0 = not pitching */
    float pitch_amplitude;     /* significant single amplitude 2σ(pitch), rad */
    /* Compass health (wire v14) — diagnostics, 0.0 until mag updates flow */
    float mag_anomaly;         /* EMA of ||B|−|B_ref||/|B_ref| (unitless) */
    float mag_residual;        /* EMA of |heading innovation|, rad */
    /* ── new members are appended here; never reorder the above ── */
} imud_data_t;

/* ── Connection API ──────────────────────────────────────────────────────── */

typedef struct imud imud_t;   /* opaque connection handle */

/*
 * imud_connect_stream — connect to the daemon's local AF_UNIX stream socket
 * (the recommended, lossless same-host path). path NULL or "" uses the
 * default /run/imud/imud-stream.sock. Returns NULL on error (errno set).
 */
imud_t *imud_connect_stream(const char *path);

/*
 * imud_connect_udp — receive the high-rate binary UDP stream. port 0 uses
 * the default 10111. group, when a multicast address (224.0.0.0/4), is
 * joined; NULL/"" receives unicast/broadcast. Returns NULL on error.
 */
imud_t *imud_connect_udp(int port, const char *group);

/*
 * imud_read — wait up to timeout_ms for the next valid packet.
 * timeout_ms < 0 blocks indefinitely.
 *
 * Returns 0 when new data is available via imud_data(), 1 on timeout,
 * -1 when the connection is lost (stream EOF / socket error) — after -1,
 * call imud_reconnect() or imud_free(). Invalid packets (bad size, magic,
 * version, or CRC) are discarded silently.
 */
int imud_read(imud_t *h, int timeout_ms);

/*
 * imud_reconnect — tear down and re-dial the same endpoint. Returns 0 on
 * success, -1 on failure (errno set; safe to retry with backoff).
 */
int imud_reconnect(imud_t *h);

/*
 * imud_data — the latest decoded packet as the append-only view struct.
 * The pointer is owned by the handle, stays valid until imud_free(), and is
 * updated in place by each successful imud_read(). Contents are meaningful
 * only after the first imud_read() == 0.
 */
const imud_data_t *imud_data(const imud_t *h);

/*
 * imud_fd — the underlying file descriptor, for integrating with an
 * external select/poll/epoll loop. When it is readable, call
 * imud_read(h, 0). Returns -1 if the handle is disconnected.
 */
int imud_fd(const imud_t *h);

/*
 * imud_free — close the connection and release the handle. NULL is a no-op.
 */
void imud_free(imud_t *h);

/* ── Introspection ───────────────────────────────────────────────────────── */

/* Library release version string, e.g. "1.5". */
const char *imud_lib_version(void);

/* Wire-protocol version of the most recently received packet (0 before the
 * first packet). The library accepts exactly the wire version it was built
 * with — it is upgraded in lockstep with the daemon. */
unsigned imud_wire_version(const imud_t *h);

/*
 * imud_wire — the latest raw wire packet.
 *
 * FOR CO-VERSIONED TOOLS ONLY (imud's own bridges): the layout of
 * imud_packet_t is pinned to the wire version, so this accessor is NOT
 * ABI-stable across imud releases. It is declared only when the wire header
 * (imud_client.h) is included before this one — including it is the explicit
 * opt-in to a wire-version-pinned build.
 */
#ifdef IMUD_CLIENT_H
const imud_packet_t *imud_wire(const imud_t *h);
#endif

#ifdef __cplusplus
}
#endif

#endif /* IMUD_H */
