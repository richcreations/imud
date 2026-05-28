/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * drivers.h — IMU and magnetometer driver abstraction layer (§4)
 *
 * Drivers implement imu_ops_t or mag_ops_t and register themselves in
 * drivers.c. The rest of the daemon calls through these structs and is
 * independent of chip specifics.
 */
#ifndef IMUD_DRIVERS_H
#define IMUD_DRIVERS_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

/* ── IMU driver configuration ──────────────────────────────────────────────── */

typedef struct {
    int odr_hz;     /* requested; driver rounds to nearest in supported_odr_hz */
    int accel_g;    /* full-scale: 2 | 4 | 8 | 16 */
    int gyro_dps;   /* full-scale: 125 | 250 | 500 | 1000 | 2000 | 4000 */
    int fifo_wm;    /* watermark in sample-sets; ignored if !has_fifo */
} imu_cfg_t;

/* ── Magnetometer driver configuration ────────────────────────────────────── */

typedef struct {
    int   odr_hz;        /* requested ODR */
    float set_period_s;  /* degauss pulse interval in seconds; 0 = disable */
} mag_cfg_t;

/* ── IMU driver operations ─────────────────────────────────────────────────── */

typedef struct {
    const char *name;   /* must match config [imu] driver = "..." */
    bool experimental;  /* true → print warning at startup; not validated on hardware */

    /* Return 0 on success, -1 on failure. */
    int (*probe)  (int fd, uint8_t addr);
    int (*reset)  (int fd, uint8_t addr);
    int (*init)   (int fd, uint8_t addr, const imu_cfg_t *cfg);

    /*
     * Read pending samples from FIFO (or single DRDY register if !has_fifo).
     * Samples are written to buf[] in SI units (m/s², rad/s) using datasheet
     * sensitivity — user calibration (offsets, soft-iron) is applied later.
     * *n is set to the number of samples written (0..max).
     * Returns 0 on success, -1 on I2C error.
     */
    int (*read)   (int fd, uint8_t addr,
                   imu_sample_t *buf, int max, int *n);

    bool has_fifo;           /* false → single-sample DRDY read per wakeup */
    bool has_hw_timestamp;   /* chip has internal sample timer */
    int  supported_odr_hz[16];   /* ascending, 0-terminated */
    int  supported_accel_g[8];   /* ascending, 0-terminated */
    int  supported_gyro_dps[8];  /* ascending, 0-terminated */
} imu_ops_t;

/* ── Magnetometer driver operations ───────────────────────────────────────── */

typedef struct {
    const char *name;   /* must match config [mag] driver = "..." */
    bool experimental;  /* true → print warning at startup; not validated on hardware */

    int (*probe)    (int fd, uint8_t addr);
    int (*reset)    (int fd, uint8_t addr);
    int (*init)     (int fd, uint8_t addr, const mag_cfg_t *cfg);
    int (*read)     (int fd, uint8_t addr, mag_sample_t *out);

    /* Issue a SET (degaussing) pulse. NULL if chip has no coil. */
    int (*set_reset)(int fd, uint8_t addr);

    bool has_interrupt;
    bool has_set_reset;
    int  supported_odr_hz[16];  /* ascending, 0-terminated */
} mag_ops_t;

/* ── Registry lookup — implemented in drivers.c ────────────────────────────── */

const imu_ops_t *imu_driver_find(const char *name);
const mag_ops_t *mag_driver_find(const char *name);

#endif /* IMUD_DRIVERS_H */
