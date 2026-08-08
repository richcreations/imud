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
#include "bus.h"
#include "types.h"

/* ── IMU driver configuration ──────────────────────────────────────────────── */

/*
 * odr_hz here is already resolved: imu.c passes the rate the driver said it
 * would program (see actual_odr_hz below), not the operator's raw request, so
 * the driver's own rounding is a no-op and the filter, the driver and imutest
 * cannot disagree about the sample rate.
 */
typedef struct {
    int odr_hz;     /* resolved sample rate in Hz */
    int accel_g;    /* full-scale: 2 | 4 | 8 | 16 */
    int gyro_dps;   /* full-scale: 125 | 250 | 500 | 1000 | 2000 | 4000 */
    int fifo_wm;    /* watermark in sample-sets; ignored if !has_fifo */
} imu_cfg_t;

/* ── Magnetometer driver configuration ────────────────────────────────────── */

typedef struct {
    int   odr_hz;        /* resolved ODR in Hz, as for imu_cfg_t */
    float set_period_s;  /* degauss pulse interval in seconds; 0 = disable */
} mag_cfg_t;

/*
 * ── The actual_odr_hz hook, shared by both ops structs ─────────────────────
 *
 * Both imu_ops_t and mag_ops_t carry:
 *
 *     int (*actual_odr_hz)(int requested);
 *
 * the rate this driver will really program for `requested`, in Hz.
 *
 * NULL means the default rule — the lowest entry in supported_odr_hz that is
 * >= requested, clamped to the highest — which is what every register-table
 * driver's odr_encode() chain does. Divider-based parts (mpu925x, icm20948)
 * derive the rate from a base clock and reach values that are not in their
 * advertised table at all, so they must implement this; so must any driver
 * that honours the request verbatim (sim).
 *
 * Resolve through odr_actual_imu() / odr_actual_mag() in imu_math.h rather
 * than calling the hook directly, so the NULL default lives in one place.
 */

/* ── IMU driver operations ─────────────────────────────────────────────────── */

typedef struct {
    const char *name;   /* must match config [imu] driver = "..." */
    bool experimental;  /* true → print warning at startup; not validated on hardware */

    /* Return 0 on success, -1 on failure. */
    int (*probe)  (const imud_bus_t *bus);
    int (*reset)  (const imud_bus_t *bus);
    int (*init)   (const imud_bus_t *bus, const imu_cfg_t *cfg);

    /*
     * Read pending samples from FIFO (or single DRDY register if !has_fifo).
     * Samples are written to buf[] in SI units (m/s², rad/s) using datasheet
     * sensitivity — user calibration (offsets, soft-iron) is applied later.
     * *n is set to the number of samples written (0..max).
     * Returns 0 on success, -1 on bus error.
     */
    int (*read)   (const imud_bus_t *bus,
                   imu_sample_t *buf, int max, int *n);

    bool has_fifo;           /* false → single-sample DRDY read per wakeup */
    bool has_hw_timestamp;   /* chip has internal sample timer */
    uint32_t ts_tick_ns;     /* ns per chip-timer tick (e.g. 25000 for the ST
                              * 25 µs counter, 1000 for the ICM-42688-P);
                              * required when has_hw_timestamp */
    int  supported_odr_hz[16];   /* ascending, 0-terminated */
    int  supported_accel_g[8];   /* ascending, 0-terminated */
    int  supported_gyro_dps[8];  /* ascending, 0-terminated */

    int (*actual_odr_hz)(int requested);  /* NULL → snap up the table; see above */
} imu_ops_t;

/* ── Magnetometer driver operations ───────────────────────────────────────── */

typedef struct {
    const char *name;   /* must match config [mag] driver = "..." */
    bool experimental;  /* true → print warning at startup; not validated on hardware */

    int (*probe)    (const imud_bus_t *bus);
    int (*reset)    (const imud_bus_t *bus);
    int (*init)     (const imud_bus_t *bus, const mag_cfg_t *cfg);
    int (*read)     (const imud_bus_t *bus, mag_sample_t *out);

    /* Issue a SET (degaussing) pulse. NULL if chip has no coil. */
    int (*set_reset)(const imud_bus_t *bus);

    bool has_interrupt;
    bool has_set_reset;
    int  supported_odr_hz[16];  /* ascending, 0-terminated */

    int (*actual_odr_hz)(int requested);  /* NULL → snap up the table; see above */
} mag_ops_t;

/* ── Registry lookup — implemented in drivers.c ────────────────────────────── */

const imu_ops_t *imu_driver_find(const char *name);
const mag_ops_t *mag_driver_find(const char *name);

/* ── Sim driver synthesis hooks (src/drivers/sim.c) ────────────────────────── */

/*
 * The sim driver's closed-form motion model at scenario time t (seconds),
 * exposed so tests and capture-scenario generation can evaluate it directly
 * at any t instead of waiting on the driver's real-time pacing.
 * sim_synth_imu fills accel/gyro/temp_c (not chip_ts/seq);
 * sim_synth_mag fills field/valid (not wall_ns).
 */
void sim_synth_imu(double t, imu_sample_t *out);
void sim_synth_mag(double t, mag_sample_t *out);

/*
 * Switch both sim ops into .imucap playback mode (file != NULL/"" enables;
 * NULL/"" returns to built-in synthesis).  Call before imu_ctx_open —
 * driven by [device] sim_file / sim_loop / sim_speed or `imud --replay`.
 */
void sim_set_playback(const char *file, bool loop, float speed);

#endif /* IMUD_DRIVERS_H */
