/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ak09916.c — AK09916 magnetometer driver (ICM-20948 internal compass)
 *
 * Requires I2C bypass mode (BYPASS_EN=1 in INT_PIN_CFG, set by icm20948_ops.init),
 * which exposes the AK09916 on the host I2C bus at address 0x0C.
 *
 * There is no external interrupt pin in bypass mode; the mag_reader_thread
 * uses a 10 ms nanosleep loop and checks DRDY in read().  has_interrupt=false
 * tells imu.c to skip GPIO line setup for the mag.
 *
 * AK09916 has no degauss coil — set_reset is NULL.
 *
 * Key differences from MMC5983MA:
 *   - 16-bit data (vs 18-bit)       — 0.15 µT/LSB sensitivity
 *   - Range ±4912 µT (vs ±800 µT)
 *   - No SET/RESET coil
 *   - DRDY polled via ST1 register; ST2 must be read to complete each transaction
 *
 * Register references: ICM-20948 Datasheet DS-000189 Rev 1.6, §12–§13.
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

/* ── Register addresses (AK09916, accessible in bypass mode) ──────────────── */

#define REG_WIA2   0x01   /* Device ID — reads 0x09 */
#define REG_ST1    0x10   /* Status 1: DRDY (bit0), DOR (bit1) */
#define REG_HXL    0x11   /* Measurement data: HXL…HZH (6 bytes) */
#define REG_ST2    0x18   /* Status 2: HOFL (bit3); must read to end transaction */
#define REG_CNTL2  0x31   /* Control 2: operation mode */
#define REG_CNTL3  0x32   /* Control 3: SRST (soft reset, bit0) */

#define WIA2_VALUE 0x09

/* CNTL2 mode codes */
#define MODE_POWER_DOWN  0x00
#define MODE_CONT_1      0x02   /* 10 Hz  */
#define MODE_CONT_2      0x04   /* 20 Hz  */
#define MODE_CONT_3      0x06   /* 50 Hz  */
#define MODE_CONT_4      0x08   /* 100 Hz */

/* Sensitivity: 0.15 µT per LSB (fixed, no sensitivity adjustment registers) */
#define AK09916_SCALE    0.15f

/* ── I2C helpers ─────────────────────────────────────────────────────────── */

static int burst_read(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
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

/* Select the continuous mode code for the requested ODR. */
static uint8_t odr_to_mode(int odr_hz)
{
    if (odr_hz <=  10) return MODE_CONT_1;
    if (odr_hz <=  20) return MODE_CONT_2;
    if (odr_hz <=  50) return MODE_CONT_3;
    return MODE_CONT_4;   /* 100 Hz max */
}

/* ── Driver operations ───────────────────────────────────────────────────── */

static int ak_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (reg_read(fd, addr, REG_WIA2, &who) < 0) {
        fprintf(stderr, "ak09916: WIA2 read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WIA2_VALUE) {
        fprintf(stderr, "ak09916: WIA2 = 0x%02X, expected 0x%02X\n",
                who, WIA2_VALUE);
        return -1;
    }
    return 0;
}

static int ak_reset(int fd, uint8_t addr)
{
    /* Power-down first, then issue soft reset. */
    if (reg_write(fd, addr, REG_CNTL2, MODE_POWER_DOWN) < 0) return -1;
    usleep(1000);

    if (reg_write(fd, addr, REG_CNTL3, 0x01) < 0) return -1;  /* SRST=1 */

    for (int i = 0; i < 20; i++) {
        usleep(1000);
        uint8_t val;
        if (reg_read(fd, addr, REG_CNTL3, &val) < 0) return -1;
        if (!(val & 0x01)) return 0;   /* bit self-clears after reset */
    }
    fprintf(stderr, "ak09916: SRST did not clear after 20 ms\n");
    return -1;
}

static int ak_init(int fd, uint8_t addr, const mag_cfg_t *cfg)
{
    /* Transition through power-down before setting continuous mode
     * (required by AK09916 datasheet). */
    if (reg_write(fd, addr, REG_CNTL2, MODE_POWER_DOWN) < 0) return -1;
    usleep(1000);
    if (reg_write(fd, addr, REG_CNTL2, odr_to_mode(cfg->odr_hz)) < 0) return -1;
    return 0;
}

/*
 * ak_read — read one magnetometer sample.
 *
 * Polls ST1 DRDY up to 15 ms (the longest inter-measurement gap at 100 Hz
 * is 10 ms).  If DRDY does not assert within that window, returns -1.
 *
 * ST2 is always read at the end of each burst to complete the AK09916
 * measurement transaction and allow the next measurement to proceed.
 */
static int ak_read(int fd, uint8_t addr, mag_sample_t *out)
{
    /* Poll DRDY (ST1 bit 0) — at 100 Hz this settles within ~10 ms. */
    uint8_t st1 = 0;
    for (int i = 0; i < 15; i++) {
        if (reg_read(fd, addr, REG_ST1, &st1) < 0) return -1;
        if (st1 & 0x01) break;   /* DRDY asserted */
        usleep(1000);
    }
    if (!(st1 & 0x01)) {
        /* DRDY never asserted — sensor may be in power-down or not ready. */
        return -1;
    }

    /* Burst read HXL through HZH (6 bytes, little-endian 16-bit signed). */
    uint8_t raw[6];
    if (burst_read(fd, addr, REG_HXL, raw, 6) < 0) return -1;

    /* Read ST2 to complete the transaction.  Check HOFL (overflow). */
    uint8_t st2;
    if (reg_read(fd, addr, REG_ST2, &st2) < 0) return -1;

    if (st2 & 0x08) {
        /* HOFL: measurement data is not reliable during magnetic overflow. */
        return -1;
    }

    int16_t hx = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    int16_t hy = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    int16_t hz = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    /*
     * Remap to NED-compatible board frame (X=bow, Y=starboard, Z=down),
     * matching the ICM-20948 driver convention.
     *
     * Per ICM-20948 datasheet: AK09916 X = ICM X, Y = ICM Y, Z = −ICM Z.
     *   X: same as ICM — no flip.
     *   Y: same as ICM chip-native Y (port) — flip to starboard.
     *   Z: raw AK Z is already in the down direction (−ICM chip Z = −up = down);
     *      ICM driver flips its own Z from up→down, so raw AK Z matches — no
     *      additional sign change needed.
     */
    out->field[0] =  (float)hx * AK09916_SCALE;   /* µT */
    out->field[1] = -(float)hy * AK09916_SCALE;
    out->field[2] =  (float)hz * AK09916_SCALE;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    out->valid   = true;

    return 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const mag_ops_t ak09916_ops = {
    .name            = "ak09916",
    .probe           = ak_probe,
    .reset           = ak_reset,
    .init            = ak_init,
    .read            = ak_read,
    .set_reset       = NULL,     /* no degauss coil */
    .has_interrupt   = false,    /* no external INT pin in bypass mode */
    .has_set_reset   = false,
    .supported_odr_hz = { 10, 20, 50, 100, 0 },
};
