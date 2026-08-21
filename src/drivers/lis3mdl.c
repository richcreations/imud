/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * lis3mdl.c — LIS3MDL magnetometer driver (ST)
 *
 * Runs in continuous measurement mode, ultra-high performance (UHP) for both
 * XY (CTRL_REG1) and Z (CTRL_REG4).  Full-scale fixed at ±4 Gauss for best
 * sensitivity.  Interrupt pin available (has_interrupt = true).
 *
 * Multi-byte reads require the MSB of the sub-address register to be set
 * (0x80 | reg) to enable address auto-increment; single-byte reads do not.
 *
 * Axis remapping assumption: Adafruit/SparkFun breakout with chip X=port,
 * Y=bow, Z=up (component-side up, connector toward stern).
 * Remap: NED_X=+Y (bow), NED_Y=−X (starboard), NED_Z=−Z (down).
 * Use [mount] rotation_euler_deg to correct for your specific installation.
 *
 * Register references: LIS3MDL datasheet DS9463 Rev 7.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "log.h"

/* ── Register addresses (DS9463 §7) ───────────────────────────────────────── */

#define REG_WHO_AM_I    0x0F   /* fixed 0x3D */
#define REG_CTRL_REG1   0x20   /* TEMP_EN|OM[1:0]|DO[2:0]|FAST_ODR|ST */
#define REG_CTRL_REG2   0x21   /* FS[1:0]|REBOOT|SOFT_RST */
#define REG_CTRL_REG3   0x22   /* LP|SIM|MD[1:0]: 00=continuous */
#define REG_CTRL_REG4   0x23   /* OMZ[1:0] at bits [3:2] */
#define REG_CTRL_REG5   0x24   /* FAST_READ|BDU */
#define REG_STATUS_REG  0x27   /* bit3 = ZYXDA */
#define REG_OUT_X_L     0x28   /* burst: X_L,X_H,Y_L,Y_H,Z_L,Z_H */

#define WHO_AM_I_VALUE  0x3D

/* ── Sensitivity ───────────────────────────────────────────────────────────── */

/* ±4 Gauss: 6842 LSB/G, 1 G = 100 µT → 100/6842 µT/LSB */
#define LIS3MDL_SCALE   (100.0f / 6842.0f)

/* ── ODR encoding ──────────────────────────────────────────────────────────── */

/*
 * Returns CTRL_REG1 value for the requested ODR.
 * All normal-rate modes use OM=11 (ultra-high performance XY).
 * Rates above 80 Hz use FAST_ODR=1 (UHP → 155 Hz).
 *
 * DO[2:0] normal-rate table (Table 21):
 *   000=0.625 Hz (unused), 001=1.25, 010=2.5, 011=5, 100=10, 101=20, 110=40, 111=80 Hz
 */
static uint8_t odr_to_ctrl1(int mhz)
{
    if (mhz > 80000)
        return (uint8_t)((0x3 << 5) | 0x02);   /* OM=11, DO=000, FAST_ODR=1 → 155 Hz */

    uint8_t do_bits = (mhz <=  1250) ? 1
                    : (mhz <=  2500) ? 2
                    : (mhz <=  5000) ? 3
                    : (mhz <= 10000) ? 4
                    : (mhz <= 20000) ? 5
                    : (mhz <= 40000) ? 6
                    :                  7;  /* 80 Hz */
    return (uint8_t)((0x3 << 5) | (do_bits << 2));   /* OM=11, FAST_ODR=0 */
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int lis_probe(const imud_bus_t *bus)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
        LOG_E("lis3mdl: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_VALUE) {
        LOG_E("lis3mdl: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_VALUE);
        return -1;
    }
    return 0;
}

static int lis_reset(const imud_bus_t *bus)
{
    /* SOFT_RST is bit 2 of CTRL_REG2; self-clears. */
    if (bus_reg_write(bus, REG_CTRL_REG2, 0x04) < 0) return -1;
    usleep(10000);  /* 10 ms power-on time */
    return 0;
}

static int lis_init(const imud_bus_t *bus, const mag_cfg_t *cfg)
{
    /* Enable block data update (no partial reads). */
    if (bus_reg_write(bus, REG_CTRL_REG5, 0x40) < 0) return -1;  /* BDU=1 */
    /* Z-axis operative mode = ultra-high performance. */
    if (bus_reg_write(bus, REG_CTRL_REG4, 0x0C) < 0) return -1;  /* OMZ=11 */
    /* XY mode + ODR from CTRL_REG1. */
    if (bus_reg_write(bus, REG_CTRL_REG1, odr_to_ctrl1(cfg->odr_mhz)) < 0) return -1;
    /* ±4 Gauss full scale; clear REBOOT and SOFT_RST. */
    if (bus_reg_write(bus, REG_CTRL_REG2, 0x00) < 0) return -1;
    /* Continuous measurement mode: MD[1:0] = 00. */
    if (bus_reg_write(bus, REG_CTRL_REG3, 0x00) < 0) return -1;
    return 0;
}

/*
 * lis_read — read one magnetometer sample.
 *
 * Returns:
 *   0  — sample written to *out
 *   1  — data not ready (ZYXDA not set); caller should wait for next interrupt
 *  -1  — bus error
 */
static int lis_read(const imud_bus_t *bus, mag_sample_t *out)
{
    uint8_t status;
    if (bus_reg_read(bus, REG_STATUS_REG, &status) < 0) return -1;
    if (!(status & 0x08)) return 1;  /* ZYXDA not set */

    /*
     * Burst read 6 bytes.  This part puts the auto-increment bit in a
     * different place on each transport:
     *
     *   I2C  the sub-address MSB, 0x80|reg              (DS9463 Rev 7 §5.1)
     *   SPI  MS at 0x40; 0x80 is the read bit there     (§5.2)
     *
     * bus_io.h applies the SPI half from spi_inc_mask, so only the I2C half is
     * spelled here.  Being explicit about it is a readability choice, not a
     * fix: ORing 0x80 unconditionally also works today, because the SPI
     * command is reg|0x80|0x40 and the I2C bit lands on the very bit SPI uses
     * for read — 0xA8 and 0x28 both encode as 0xE8, address 0x28.  That is a
     * coincidence of bit positions, and a mutation confirms no test can see
     * the difference.  Naming the transport keeps the coincidence from
     * becoming load-bearing.
     */
    uint8_t raw[6];
    uint8_t out_reg = REG_OUT_X_L;
    if (bus->kind == BUS_I2C) out_reg |= 0x80;
    if (bus_burst_read(bus, out_reg, raw, 6) < 0) return -1;

    int16_t rx = reg_s16le(&raw[0]);
    int16_t ry = reg_s16le(&raw[2]);
    int16_t rz = reg_s16le(&raw[4]);

    /*
     * Remap chip frame (X=port, Y=bow, Z=up) → NED board frame.
     * NED: X=bow=+chipY, Y=starboard=−chipX, Z=down=−chipZ.
     */
    out->field[0] =  (float)ry * LIS3MDL_SCALE;
    out->field[1] = -(float)rx * LIS3MDL_SCALE;
    out->field[2] = -(float)rz * LIS3MDL_SCALE;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    out->valid   = true;

    return 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const mag_ops_t lis3mdl_ops = {
    .name             = "lis3mdl",
    .experimental     = true,
    /* DS9463 Rev 7 §5.2 (protocol, 4-wire, SPC idle high and captured on the
     * rising edge — mode 3) and §2.4.1 Table 5 (10 MHz).  The only part in the
     * tree that needs an explicit auto-increment bit: MS at 0x40, because its
     * address field is six bits.  Every register here is <= 0x33, so bit 6 is
     * free to carry it.  init() writes CTRL_REG3 = 0x00, which selects 4-wire
     * (SIM = 0) — nothing extra to do for SPI. */
    .bus_caps         = { .spi_capable = true, .spi_mode = 3,
                          .spi_max_hz = 10000000, .spi_inc_mask = 0x40 },
    .probe            = lis_probe,
    .reset            = lis_reset,
    .init             = lis_init,
    .read             = lis_read,
    .set_reset        = NULL,
    .has_interrupt    = true,
    .has_set_reset    = false,
    /*
     * Deliberately narrower than the silicon, at both ends (DS9463 Rev 7
     * Tables 19 and 21).
     *
     * TOP.  Above 80 Hz the rate comes from FAST_ODR, and what it selects
     * depends on the operating mode: UHP 155, HP 300, MP 560, LP 1000 Hz.
     * This driver fixes OM = ultra-high-performance for noise performance
     * (CTRL_REG1 OM = 11, CTRL_REG4 OMZ = 11), so 155 Hz is the ceiling by
     * choice, not by limit.  Reaching 1000 Hz would mean low-power mode on a
     * part whose only job here is to feed the MEKF a heading reference — a
     * bad trade, and 155 Hz is already far more than that update needs.
     *
     * BOTTOM.  DO = 000 is 0.625 Hz, which this int table cannot express; it
     * would have to round onto 1.25's slot and lie about it.
     */
    /* 1.25 and 2.5 Hz are exact here; the whole-Hz ladder rounded them. */
    .supported_odr_mhz = { 1250, 2500, 5000, 10000, 20000, 40000, 80000,
                           155000, 0 },
};
