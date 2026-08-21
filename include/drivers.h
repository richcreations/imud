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
 * RATES ARE MILLI-HERTZ THROUGHOUT THIS INTERFACE.
 *
 * Whole Hz could not express the rates real parts actually run.  The ST 6-axis
 * ladder is one binary divider chain, 6664/2^n, so its bottom rung is 13.016 Hz
 * — the datasheet prints "12.5 Hz" and the driver then rounded that to 12,
 * leaving the advertised rate 7.8% below what the part produced and
 * ticks_per_sample scaled for a rate nothing was running at.  The TDK parts
 * have an exact 12.5 Hz rung that no integer represents at all.  Rounding
 * either way was wrong in a way that showed up on the bench as a driver fault.
 *
 * So: milli-Hz, everywhere the interface carries a rate.  13016 is the ST
 * bottom rung, 12500 the TDK one, and neither needs a round trip through a
 * whole number.  `[imu] odr_hz` still reads in Hz and accepts a decimal —
 * config.c converts once, at the edge.
 *
 * odr_mhz here is already resolved: imu.c passes the rate the driver said it
 * would program (see actual_odr_mhz below), not the operator's raw request, so
 * the driver's own rounding is a no-op and the filter, the driver and imutest
 * cannot disagree about the sample rate.
 */
typedef struct {
    int odr_mhz;    /* resolved sample rate in milli-Hz */
    int accel_g;    /* full-scale: 2 | 4 | 8 | 16 */
    int gyro_dps;   /* full-scale: 125 | 250 | 500 | 1000 | 2000 | 4000 */
    int fifo_wm;    /* watermark in sample-sets; ignored if !has_fifo */
} imu_cfg_t;

/* ── Magnetometer driver configuration ────────────────────────────────────── */

/*
 * int_driven says how the caller waits, which on some parts decides what read()
 * is even able to check.
 *
 * A part whose DRDY is a *latched* interrupt has to be acknowledged by a
 * register write, and if that write also clears the status bit read() gates on,
 * then the gate and the interrupt are mutually exclusive: acknowledging to
 * re-arm the edge is what destroys the evidence the gate wants.  The
 * MMC5983MA is such a part — measured, see mmc5983ma.c — and there the edge
 * itself is the data-ready signal, so read() trusts it and skips the gate.
 *
 * Most parts are not like this: an unlatched DRDY pin that mirrors a status bit
 * (LIS3MDL, LIS2MDL) or clears when the data registers are read (RM3100) has no
 * acknowledge write at all, so it has nothing to destroy.  Check the datasheet
 * before setting this for a new driver; the driver guide in docs/manual.md §11
 * has the comparison.
 *
 * Callers that poll leave it false and keep the gate — which is right, because
 * a poller has no edge to trust.
 */
typedef struct {
    int   odr_mhz;       /* resolved ODR in milli-Hz, as for imu_cfg_t */
    float set_period_s;  /* degauss pulse interval in seconds; 0 = disable */
    bool  int_driven;    /* caller blocks on the DRDY edge (see above) */
} mag_cfg_t;

/*
 * ── The actual_odr_mhz hook, shared by both ops structs ────────────────────
 *
 * Both imu_ops_t and mag_ops_t carry:
 *
 *     int (*actual_odr_mhz)(int requested_mhz);
 *
 * the rate this driver will really program for `requested_mhz`, in milli-Hz.
 *
 * NULL means the default rule — the lowest entry in supported_odr_mhz that is
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

    /*
     * What this part's silicon can do on SPI — datasheet facts, not operator
     * policy. Left zeroed means spi_capable = false, so `bus = "spi"` is
     * refused by name rather than tried and mis-framed. See include/bus.h.
     */
    bus_caps_t bus_caps;

    /* Return 0 on success, -1 on failure. */
    int (*probe)  (const imud_bus_t *bus);
    int (*reset)  (const imud_bus_t *bus);
    int (*init)   (const imud_bus_t *bus, const imu_cfg_t *cfg);

    /*
     * Read pending samples from FIFO (or single DRDY register if !has_fifo).
     * Samples are written to buf[] in SI units (m/s², rad/s) using datasheet
     * sensitivity — user calibration (offsets, soft-iron) is applied later.
     * *n is set to the number of samples written (0..max).
     *
     * Returns 0 on success, 1 when there is no data yet — a DRDY that has not
     * asserted, or a FIFO overflow, neither of which is an error — and -1 on a
     * bus error only.  The `1` is load-bearing: ism330dhcx.c really returns it
     * and imu.c depends on the distinction to avoid counting a quiet sensor
     * toward the error-reset threshold.  The mag read() below is the same.
     */
    int (*read)   (const imud_bus_t *bus,
                   imu_sample_t *buf, int max, int *n);

    bool has_fifo;           /* false → single-sample DRDY read per wakeup */
    bool has_hw_timestamp;   /* chip has internal sample timer */
    uint32_t ts_tick_ns;     /* ns per chip-timer tick (e.g. 25000 for the ST
                              * 25 µs counter, 1067 for the ICM-42688-P);
                              * required when has_hw_timestamp.  This is the
                              * TYPICAL period from the datasheet — see
                              * ts_tick_ns_actual for the per-part value */

    /*
     * Optional: ask THIS part what its timer period really is.  Returns ns per
     * tick, or 0 to keep ts_tick_ns as declared.  Called once after init(),
     * with the bus already open, so it may read registers.
     *
     * Why this exists.  ts_tick_ns is a datasheet typical, and the tolerance on
     * it is not small: the reference ISM330DHCX on the bench runs 4.05% fast,
     * and the part says so — the ST 6-axis family carries INTERNAL_FREQ_FINE
     * (0x63), a factory-trim count of 0.15% steps (DS13012 §9.41, Table 139).
     * A const field cannot express that, since it is silicon-specific rather
     * than part-number-specific.
     *
     * imu.c's ts_anchor_t does measure the real period at runtime, but only
     * once it holds two anchors 20 s apart.  Until then chip_to_wall() falls
     * back to this number, and on a 4%-fast part the extrapolated sample time
     * gains ~40 ms per second of elapsed time — enough to make the sample
     * latency histogram stop recording entirely.  Getting the declared value
     * right is what makes the first minute behave like the rest of the run.
     *
     * Implementations must be defensive: a wrong period is worse than the
     * typical one, because every per-sample dt is scaled by it.  Return 0 on
     * any bus error, and satisfy yourself that whatever the register can hold
     * cannot produce an absurd period — bound it if it can.
     */
    uint32_t (*ts_tick_ns_actual)(const imud_bus_t *bus);

    int  supported_odr_mhz[16];  /* milli-Hz, ascending, 0-terminated */
    int  supported_accel_g[8];   /* ascending, 0-terminated */
    int  supported_gyro_dps[8];  /* ascending, 0-terminated */

    int (*actual_odr_mhz)(int req_mhz);  /* NULL → snap up the table; see above */
} imu_ops_t;

/* ── Magnetometer driver operations ───────────────────────────────────────── */

/*
 * Which way to magnetise an AMR bridge.  SET and RESET drive the film in
 * opposite directions, so the field-dependent term of a reading flips sign
 * between them while the bridge's own offset does not — which is what lets a
 * caller separate the two.  See degauss() below.
 */
typedef enum {
    MAG_DEGAUSS_SET = 0,
    MAG_DEGAUSS_RESET
} mag_degauss_t;

typedef struct {
    const char *name;   /* must match config [mag] driver = "..." */
    bool experimental;  /* true → print warning at startup; not validated on hardware */

    bus_caps_t bus_caps;   /* as for imu_ops_t above */

    /* Return 0 on success, -1 on failure — as for imu_ops_t above. */
    int (*probe)    (const imud_bus_t *bus);
    int (*reset)    (const imud_bus_t *bus);
    int (*init)     (const imud_bus_t *bus, const mag_cfg_t *cfg);

    /*
     * Read one completed measurement into *out.
     *
     * Returns 0 on success, 1 when there is no new measurement yet, and -1 on a
     * bus error only.  Same three-way contract as the IMU read() above, and the
     * `1` is load-bearing for the same reason: imu.c must not count a quiet
     * sensor toward the error-reset threshold.
     *
     * What "no new measurement yet" is decided FROM depends on how the caller
     * waits — see mag_cfg_t.int_driven.  A driver whose data-ready is a latched
     * interrupt cleared by the same write that clears its status bit cannot use
     * that bit when the caller blocks on the edge; mmc5983ma.c carries the
     * measurement and the reasoning.
     */
    int (*read)     (const imud_bus_t *bus, mag_sample_t *out);

    /* Issue a SET (degaussing) pulse. NULL if chip has no coil. */
    int (*set_reset)(const imud_bus_t *bus);

    /*
     * Optional: one degauss pulse in a chosen direction.  NULL on a part that
     * cannot drive the RESET half, and on every part with no coil at all.
     *
     * DIAGNOSTIC ONLY.  The daemon never calls this — set_reset() above is the
     * production path and its behaviour is unchanged.  imud-imutest uses the
     * pair to split one reading into the true field and the bridge offset:
     *
     *     vS = +S*B + offset      (after MAG_DEGAUSS_SET)
     *     vR = -S*B + offset      (after MAG_DEGAUSS_RESET)
     *       field  = (vS - vR) / 2
     *       offset = (vS + vR) / 2
     *
     * which is the only way to tell a genuine external field from a bridge
     * offset without a second transport or a known reference field.  A driver
     * implementing this must leave the part in the SET state when the caller is
     * done; imutest asks for that explicitly rather than assuming it.
     */
    int (*degauss)(const imud_bus_t *bus, mag_degauss_t dir);

    bool has_interrupt;
    bool has_set_reset;
    int  supported_odr_mhz[16]; /* milli-Hz, ascending, 0-terminated */

    int (*actual_odr_mhz)(int req_mhz);  /* NULL → snap up the table; see above */
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

/* How far a .imucap playback has got.  Values are published per stream and
 * combined by sim_playback_state(). */
typedef enum {
    SIM_PB_OFF = 0,   /* not replaying a file (synthesis, or no sim driver) */
    SIM_PB_RUNNING,   /* armed, and not finished */
    SIM_PB_EOF,       /* every stream reached end of file cleanly */
    SIM_PB_ERROR      /* the capture could not be opened or read */
} sim_pb_state_t;

/*
 * Playback progress, safe to call from any thread.
 *
 * SIM_PB_EOF is reported only after every stream has RETURNED empty, not
 * merely reached end of file internally — so once this says EOF, every sample
 * the file contained has already been handed to its reader.  Callers can act
 * on it without racing the last burst.
 *
 * Stays SIM_PB_RUNNING forever when looping is on, because a looping playback
 * never ends.
 */
sim_pb_state_t sim_playback_state(void);

#endif /* IMUD_DRIVERS_H */
