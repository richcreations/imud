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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "drivers.h"

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

/* ── I2C helpers ───────────────────────────────────────────────────────────── */

/*
 * burst_read_auto: multi-byte read using 0x80|reg for address auto-increment.
 * Required for LIS3MDL burst reads (§6.1.1 of the datasheet).
 */
static int burst_read_auto(int fd, uint8_t addr, uint8_t reg,
                           uint8_t *buf, uint16_t len)
{
    uint8_t sub = (uint8_t)(reg | 0x80);
    struct i2c_msg msgs[2] = {
        { .addr = addr, .flags = 0,        .len = 1,   .buf = &sub },
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
    /* Single-byte: no auto-increment needed, use plain sub-address. */
    struct i2c_msg msgs[2] = {
        { .addr = addr, .flags = 0,        .len = 1, .buf = &reg },
        { .addr = addr, .flags = I2C_M_RD, .len = 1, .buf = val  },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

/* ── ODR encoding ──────────────────────────────────────────────────────────── */

/*
 * Returns CTRL_REG1 value for the requested ODR.
 * All normal-rate modes use OM=11 (ultra-high performance XY).
 * Rates above 80 Hz use FAST_ODR=1 (UHP → 155 Hz).
 *
 * DO[2:0] normal-rate table (Table 21):
 *   000=0.625 Hz (unused), 001=1.25, 010=2.5, 011=5, 100=10, 101=20, 110=40, 111=80 Hz
 */
static uint8_t odr_to_ctrl1(int hz)
{
    if (hz > 80)
        return (uint8_t)((0x3 << 5) | 0x02);   /* OM=11, DO=000, FAST_ODR=1 → 155 Hz */

    uint8_t do_bits = (hz <=  1) ? 1   /* 1.25 Hz */
                    : (hz <=  2) ? 2   /* 2.5 Hz */
                    : (hz <=  5) ? 3   /* 5 Hz */
                    : (hz <= 10) ? 4   /* 10 Hz */
                    : (hz <= 20) ? 5   /* 20 Hz */
                    : (hz <= 40) ? 6   /* 40 Hz */
                    :              7;  /* 80 Hz */
    return (uint8_t)((0x3 << 5) | (do_bits << 2));   /* OM=11, FAST_ODR=0 */
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int lis_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (reg_read(fd, addr, REG_WHO_AM_I, &who) < 0) {
        fprintf(stderr, "lis3mdl: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_VALUE) {
        fprintf(stderr, "lis3mdl: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_VALUE);
        return -1;
    }
    return 0;
}

static int lis_reset(int fd, uint8_t addr)
{
    /* SOFT_RST is bit 2 of CTRL_REG2; self-clears. */
    if (reg_write(fd, addr, REG_CTRL_REG2, 0x04) < 0) return -1;
    usleep(10000);  /* 10 ms power-on time */
    return 0;
}

static int lis_init(int fd, uint8_t addr, const mag_cfg_t *cfg)
{
    /* Enable block data update (no partial reads). */
    if (reg_write(fd, addr, REG_CTRL_REG5, 0x40) < 0) return -1;  /* BDU=1 */
    /* Z-axis operative mode = ultra-high performance. */
    if (reg_write(fd, addr, REG_CTRL_REG4, 0x0C) < 0) return -1;  /* OMZ=11 */
    /* XY mode + ODR from CTRL_REG1. */
    if (reg_write(fd, addr, REG_CTRL_REG1, odr_to_ctrl1(cfg->odr_hz)) < 0) return -1;
    /* ±4 Gauss full scale; clear REBOOT and SOFT_RST. */
    if (reg_write(fd, addr, REG_CTRL_REG2, 0x00) < 0) return -1;
    /* Continuous measurement mode: MD[1:0] = 00. */
    if (reg_write(fd, addr, REG_CTRL_REG3, 0x00) < 0) return -1;
    return 0;
}

/*
 * lis_read — read one magnetometer sample.
 *
 * Returns:
 *   0  — sample written to *out
 *   1  — data not ready (ZYXDA not set); caller should wait for next interrupt
 *  -1  — I2C error
 */
static int lis_read(int fd, uint8_t addr, mag_sample_t *out)
{
    uint8_t status;
    if (reg_read(fd, addr, REG_STATUS_REG, &status) < 0) return -1;
    if (!(status & 0x08)) return 1;  /* ZYXDA not set */

    /* Burst read 6 bytes; address auto-increment requires 0x80|reg. */
    uint8_t raw[6];
    if (burst_read_auto(fd, addr, REG_OUT_X_L, raw, 6) < 0) return -1;

    int16_t rx = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

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
    .probe            = lis_probe,
    .reset            = lis_reset,
    .init             = lis_init,
    .read             = lis_read,
    .set_reset        = NULL,
    .has_interrupt    = true,
    .has_set_reset    = false,
    .supported_odr_hz = { 1, 2, 5, 10, 20, 40, 80, 155, 0 },
};
