/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * lis2mdl.c — LIS2MDL magnetometer driver (ST)
 *
 * LIS3MDL successor with a simpler register interface.  Fixed I2C address
 * 0x1E (no configurable SA pin).  Fixed ±50 Gauss full scale; fixed
 * sensitivity 1.5 mgauss/LSB = 0.15 µT/LSB.  Four ODR choices (10–100 Hz).
 * Native I2C auto-increment: no 0x80|reg needed for burst reads.
 *
 * Axis remapping assumption: Adafruit 4413 / breakouts with chip X=port,
 * Y=bow, Z=up (component-side up).
 * Remap: NED_X=+Y (bow), NED_Y=−X (starboard), NED_Z=−Z (down).
 * Use [mount] rotation_euler_deg to correct for your installation if needed.
 *
 * Register references: LIS2MDL datasheet DS12144 Rev 6.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "log.h"

/* ── Register addresses (DS12144 §8) ──────────────────────────────────────── */

#define REG_WHO_AM_I    0x4F   /* fixed 0x40 */
#define REG_CFG_REG_A   0x60   /* COMP_TEMP_EN|REBOOT|SOFT_RST|LP|ODR1|ODR0|MD1|MD0 */
#define REG_CFG_REG_B   0x61   /* LPF|OFF_CANC|… */
#define REG_CFG_REG_C   0x62   /* INT_on_PIN|I2C_DIS|BDU|BLE|…|INT_MAG_PIN|DRDY_on_PIN */
#define REG_STATUS_REG  0x67   /* bit3 = ZYXDA */
#define REG_OUTX_L      0x68   /* auto-increment through OUTZ_H (0x6D) */

#define WHO_AM_I_VALUE  0x40

/* Fixed sensitivity: 1.5 mgauss/LSB = 0.15 µT/LSB */
#define LIS2MDL_SCALE   0.15f

/* ── ODR encoding (CFG_REG_A bits [3:2]) ──────────────────────────────────── */

static uint8_t odr_encode(int hz)
{
    if (hz <= 10) return (0x0 << 2);
    if (hz <= 20) return (0x1 << 2);
    if (hz <= 50) return (0x2 << 2);
    return         (0x3 << 2);   /* 100 Hz */
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int li2_probe(const imud_bus_t *bus)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
        LOG_E("lis2mdl: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_VALUE) {
        LOG_E("lis2mdl: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_VALUE);
        return -1;
    }
    return 0;
}

static int li2_reset(const imud_bus_t *bus)
{
    /* SOFT_RST is bit 5 of CFG_REG_A; self-clears after ~5 ms. */
    if (bus_reg_write(bus, REG_CFG_REG_A, 0x20) < 0) return -1;
    usleep(5000);
    return 0;
}

static int li2_init(const imud_bus_t *bus, const mag_cfg_t *cfg)
{
    /* Enable block data update + DRDY interrupt pin. */
    if (bus_reg_write(bus, REG_CFG_REG_C, 0x11) < 0) return -1;  /* BDU|DRDY_on_PIN */
    /* Enable offset cancellation. */
    if (bus_reg_write(bus, REG_CFG_REG_B, 0x02) < 0) return -1;  /* OFF_CANC */
    /* Set ODR and enable continuous mode (MD[1:0] = 00). */
    if (bus_reg_write(bus, REG_CFG_REG_A, odr_encode(cfg->odr_hz)) < 0) return -1;
    return 0;
}

/*
 * li2_read — read one magnetometer sample.
 *
 * Returns:
 *   0  — sample written to *out
 *   1  — data not ready (ZYXDA not set); caller should wait for next interrupt
 *  -1  — I2C error
 */
static int li2_read(const imud_bus_t *bus, mag_sample_t *out)
{
    uint8_t status;
    if (bus_reg_read(bus, REG_STATUS_REG, &status) < 0) return -1;
    if (!(status & 0x08)) return 1;  /* ZYXDA not set */

    uint8_t raw[6];
    if (bus_burst_read(bus, REG_OUTX_L, raw, 6) < 0) return -1;

    int16_t rx = reg_s16le(&raw[0]);
    int16_t ry = reg_s16le(&raw[2]);
    int16_t rz = reg_s16le(&raw[4]);

    /*
     * Remap chip frame (X=port, Y=bow, Z=up) → NED board frame.
     * NED: X=bow=+chipY, Y=starboard=−chipX, Z=down=−chipZ.
     */
    out->field[0] =  (float)ry * LIS2MDL_SCALE;
    out->field[1] = -(float)rx * LIS2MDL_SCALE;
    out->field[2] = -(float)rz * LIS2MDL_SCALE;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    out->valid   = true;

    return 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const mag_ops_t lis2mdl_ops = {
    .name             = "lis2mdl",
    .experimental     = true,
    /*
     * NO SPI, and not merely "not yet" — this part's SPI is incompatible with
     * how imud reads it.
     *
     * DS12144 Rev 6 §6.2: the LIS2MDL's SPI defaults to THREE wires (CS, SPC,
     * a shared SDI/O), and 4-wire mode has to be switched on by writing bit 2
     * of CFG_REG_C — which, in the datasheet's own words, "disables the
     * interrupt and data-ready signaling capability of the device".
     *
     * That is exactly what this driver depends on: init() writes CFG_REG_C =
     * 0x11 for BDU | DRDY_on_PIN, has_interrupt is true below, and imu.c
     * requests a GPIO line and blocks the mag reader on its edges.  So 4-wire
     * SPI costs the interrupt, and keeping the interrupt means half-duplex
     * 3-wire, which the bus layer does not do (spi_burst_read is a
     * full-duplex command-then-data pair; 3-wire needs SPI_3WIRE).
     *
     * Enabling either would be a change to how the part is READ, not just how
     * it is addressed, so it belongs in a commit of its own with hardware to
     * check it against.  See docs/ROADMAP.md.
     */
    .bus_caps         = { .spi_capable = false },
    .probe            = li2_probe,
    .reset            = li2_reset,
    .init             = li2_init,
    .read             = li2_read,
    .set_reset        = NULL,
    .has_interrupt    = true,
    .has_set_reset    = false,
    .supported_odr_hz = { 10, 20, 50, 100, 0 },
};
