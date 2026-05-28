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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "drivers.h"

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

/* ── I2C helpers ───────────────────────────────────────────────────────────── */

static int burst_read(int fd, uint8_t addr, uint8_t reg,
                      uint8_t *buf, uint16_t len)
{
    struct i2c_msg msgs[2] = {
        { .addr = addr, .flags = 0,        .len = 1,   .buf = &reg },
        { .addr = addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static int reg_write(int fd, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = addr, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static int reg_read(int fd, uint8_t addr, uint8_t reg, uint8_t *val)
{
    return burst_read(fd, addr, reg, val, 1);
}

/* ── ODR encoding (CFG_REG_A bits [3:2]) ──────────────────────────────────── */

static uint8_t odr_encode(int hz)
{
    if (hz <= 10) return (0x0 << 2);
    if (hz <= 20) return (0x1 << 2);
    if (hz <= 50) return (0x2 << 2);
    return         (0x3 << 2);   /* 100 Hz */
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int li2_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (reg_read(fd, addr, REG_WHO_AM_I, &who) < 0) {
        fprintf(stderr, "lis2mdl: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_VALUE) {
        fprintf(stderr, "lis2mdl: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_VALUE);
        return -1;
    }
    return 0;
}

static int li2_reset(int fd, uint8_t addr)
{
    /* SOFT_RST is bit 5 of CFG_REG_A; self-clears after ~5 ms. */
    if (reg_write(fd, addr, REG_CFG_REG_A, 0x20) < 0) return -1;
    usleep(5000);
    return 0;
}

static int li2_init(int fd, uint8_t addr, const mag_cfg_t *cfg)
{
    /* Enable block data update + DRDY interrupt pin. */
    if (reg_write(fd, addr, REG_CFG_REG_C, 0x11) < 0) return -1;  /* BDU|DRDY_on_PIN */
    /* Enable offset cancellation. */
    if (reg_write(fd, addr, REG_CFG_REG_B, 0x02) < 0) return -1;  /* OFF_CANC */
    /* Set ODR and enable continuous mode (MD[1:0] = 00). */
    if (reg_write(fd, addr, REG_CFG_REG_A, odr_encode(cfg->odr_hz)) < 0) return -1;
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
static int li2_read(int fd, uint8_t addr, mag_sample_t *out)
{
    uint8_t status;
    if (reg_read(fd, addr, REG_STATUS_REG, &status) < 0) return -1;
    if (!(status & 0x08)) return 1;  /* ZYXDA not set */

    uint8_t raw[6];
    if (burst_read(fd, addr, REG_OUTX_L, raw, 6) < 0) return -1;

    int16_t rx = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

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
    .probe            = li2_probe,
    .reset            = li2_reset,
    .init             = li2_init,
    .read             = li2_read,
    .set_reset        = NULL,
    .has_interrupt    = true,
    .has_set_reset    = false,
    .supported_odr_hz = { 10, 20, 50, 100, 0 },
};
