/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * lsm6dso.c — LSM6DSO / LSM6DSOX IMU driver
 *
 * Near-identical register layout to ISM330DHCX (same FIFO scheme, same
 * timestamp peripheral, same sensitivity table).  Three differences:
 *   1. WHO_AM_I is 0x6C (LSM6DSO) or 0x6D (LSM6DSOX) instead of 0x6B.
 *   2. DEVICE_CONF (REG_CTRL9_XL) write is not required.
 *   3. ODR table extends to 3332 Hz (code 0x9) and 6664 Hz (code 0xA).
 *
 * Two ops structs are exported (lsm6dso_ops / lsm6dsox_ops) so both chip
 * name strings work from config; they share all function pointers.
 *
 * Register references: LSM6DSO datasheet DS12140 Rev 3.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "drivers.h"
#include "log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Register addresses (same layout as ISM330DHCX, DS12140 §8) ───────────── */

#define REG_FIFO_CTRL1        0x07
#define REG_FIFO_CTRL2        0x08
#define REG_FIFO_CTRL3        0x09
#define REG_FIFO_CTRL4        0x0A
#define REG_INT1_CTRL         0x0D
#define REG_WHO_AM_I          0x0F
#define REG_CTRL1_XL          0x10
#define REG_CTRL2_G           0x11
#define REG_CTRL3_C           0x12
#define REG_CTRL10_C          0x19
#define REG_OUT_TEMP_L        0x20  /* temperature output L; H at 0x21 (256 LSB/°C, 0=25°C) */
#define REG_FIFO_STATUS1      0x3A
#define REG_FIFO_STATUS2      0x3B
#define REG_TIMESTAMP0        0x40
#define REG_FIFO_DATA_OUT_TAG 0x78

/* ── FIFO tag codes (same as ISM330DHCX, Table 159) ───────────────────────── */

#define TAG_GYRO_NC   0x01
#define TAG_ACCEL_NC  0x02
#define TAG_TEMP      0x03

/* ── Chip identity ─────────────────────────────────────────────────────────── */

#define WHO_AM_I_LSM6DSO   0x6C
#define WHO_AM_I_LSM6DSOX  0x6D

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float    accel_scale;
    float    gyro_scale;
    float    last_temp;         /* °C; persists across drains (temp batched
                                 * slower than the FIFO is drained) */
    uint32_t seq;
    uint32_t ticks_per_sample;
} ls;

/* ── Low-level I2C helpers ─────────────────────────────────────────────────── */

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

/* ── ODR and full-scale encoding helpers ───────────────────────────────────── */

static uint8_t odr_encode(int hz)
{
    if (hz <=   12) return 0x1;
    if (hz <=   26) return 0x2;
    if (hz <=   52) return 0x3;
    if (hz <=  104) return 0x4;
    if (hz <=  208) return 0x5;
    if (hz <=  416) return 0x6;
    if (hz <=  833) return 0x7;
    if (hz <= 1660) return 0x8;
    if (hz <= 3332) return 0x9;
    return 0xA;  /* 6664 Hz */
}

static int odr_actual(int hz)
{
    static const int steps[] = { 12, 26, 52, 104, 208, 416, 833, 1660, 3332, 6664 };
    for (int i = 0; i < 9; i++)
        if (hz <= steps[i]) return steps[i];
    return 6664;
}

static uint8_t xl_fs_encode(int g, float *scale)
{
    switch (g) {
    case  2: *scale = 0.061e-3f * 9.80665f; return 0x00;
    case  4: *scale = 0.122e-3f * 9.80665f; return 0x08;
    case 16: *scale = 0.488e-3f * 9.80665f; return 0x04;
    default:
    case  8: *scale = 0.244e-3f * 9.80665f; return 0x0C;
    }
}

static uint8_t gy_fs_encode(int dps, float *scale)
{
    const float d2r = (float)(M_PI / 180.0 / 1000.0);
    switch (dps) {
    case  125: *scale =   4.375f * d2r; return 0x02;
    case  250: *scale =   8.75f  * d2r; return 0x00;
    case  500: *scale =  17.5f   * d2r; return 0x04;
    case 1000: *scale =  35.0f   * d2r; return 0x08;
    case 4000: *scale = 140.0f   * d2r; return 0x01;
    default:
    case 2000: *scale =  70.0f   * d2r; return 0x0C;
    }
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int lsm_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (reg_read(fd, addr, REG_WHO_AM_I, &who) < 0) {
        LOG_E("lsm6dso: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_LSM6DSO && who != WHO_AM_I_LSM6DSOX) {
        LOG_E("lsm6dso: WHO_AM_I = 0x%02X, expected 0x%02X or 0x%02X\n",
                who, WHO_AM_I_LSM6DSO, WHO_AM_I_LSM6DSOX);
        return -1;
    }
    return 0;
}

static int lsm_reset(int fd, uint8_t addr)
{
    if (reg_write(fd, addr, REG_CTRL3_C, 0x01) < 0) return -1;

    for (int i = 0; i < 50; i++) {
        usleep(1000);
        uint8_t val;
        if (reg_read(fd, addr, REG_CTRL3_C, &val) < 0) return -1;
        if (!(val & 0x01)) goto reset_done;
    }
    LOG_W("lsm6dso: SW_RESET did not clear after 50 ms\n");
    return -1;

reset_done:
    usleep(35000);
    return 0;
}

static int lsm_init(int fd, uint8_t addr, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_hz);
    uint8_t xlfs = xl_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t gyfs = gy_fs_encode(cfg->gyro_dps, &gyro_scale);

    if (reg_write(fd, addr, REG_CTRL1_XL, (uint8_t)((odr << 4) | xlfs | 0x02)) < 0) return -1;
    if (reg_write(fd, addr, REG_CTRL2_G,  (uint8_t)((odr << 4) | gyfs))         < 0) return -1;
    if (reg_write(fd, addr, REG_CTRL3_C,  0x44)                                  < 0) return -1;
    /* No DEVICE_CONF write — not present on LSM6DSO */
    if (reg_write(fd, addr, REG_CTRL10_C, 0x20)                                  < 0) return -1;

    int wm = cfg->fifo_wm * 2;
    if (wm > 511) wm = 511;
    if (reg_write(fd, addr, REG_FIFO_CTRL1, (uint8_t)(wm & 0xFF))        < 0) return -1;
    if (reg_write(fd, addr, REG_FIFO_CTRL2, (uint8_t)((wm >> 8) & 0x01)) < 0) return -1;
    if (reg_write(fd, addr, REG_FIFO_CTRL3, (uint8_t)((odr << 4) | odr)) < 0) return -1;
    /* Continuous mode + temperature batched at 12.5 Hz (ODR_T_BATCH = 10),
     * matching ism330dhcx.c — temp feeds thermal comp and imud-cal fit-temp. */
    if (reg_write(fd, addr, REG_FIFO_CTRL4, 0x26)                        < 0) return -1;
    if (reg_write(fd, addr, REG_INT1_CTRL,  0x08)                        < 0) return -1;

    /* Seed last_temp from the live temperature register (see ism330dhcx.c). */
    {
        uint8_t tb[2];
        if (burst_read(fd, addr, REG_OUT_TEMP_L, tb, 2) == 0) {
            int16_t rt = (int16_t)(((uint16_t)tb[1] << 8) | tb[0]);
            ls.last_temp = (float)rt / 256.0f + 25.0f;
        } else {
            ls.last_temp = 25.0f;
        }
    }
    ls.accel_scale      = accel_scale;
    ls.gyro_scale       = gyro_scale;
    ls.seq              = 0;
    ls.ticks_per_sample = (uint32_t)(40000u / (unsigned)odr_actual(cfg->odr_hz));

    return 0;
}

static int lsm_read(int fd, uint8_t addr,
                    imu_sample_t *buf, int max, int *n_out)
{
    uint8_t st[2];
    if (burst_read(fd, addr, REG_FIFO_STATUS1, st, 2) < 0) return -1;

    int n_words  = (((int)(st[1] & 0x03)) << 8) | st[0];
    int overflow = (st[1] & 0x40) != 0;

    if (n_words == 0) {
        *n_out = 0;
        return overflow ? 1 : 0;
    }

    float p_accel[3] = {0.0f, 0.0f, 0.0f};
    float p_gyro[3]  = {0.0f, 0.0f, 0.0f};
    int   have_accel = 0;
    int   have_gyro  = 0;
    int   produced   = 0;

    for (int i = 0; i < n_words && produced < max; i++) {
        uint8_t word[7];
        if (burst_read(fd, addr, REG_FIFO_DATA_OUT_TAG, word, 7) < 0) return -1;

        uint8_t tag   = (word[0] >> 3) & 0x1F;
        int16_t raw_x = (int16_t)(((uint16_t)word[2] << 8) | word[1]);
        int16_t raw_y = (int16_t)(((uint16_t)word[4] << 8) | word[3]);
        int16_t raw_z = (int16_t)(((uint16_t)word[6] << 8) | word[5]);

        switch (tag) {
        case TAG_ACCEL_NC:
            p_accel[0] = raw_x * ls.accel_scale;
            p_accel[1] = raw_y * ls.accel_scale;
            p_accel[2] = raw_z * ls.accel_scale;
            have_accel = 1;
            break;

        case TAG_GYRO_NC:
            p_gyro[0] = raw_x * ls.gyro_scale;
            p_gyro[1] = raw_y * ls.gyro_scale;
            p_gyro[2] = raw_z * ls.gyro_scale;
            have_gyro = 1;
            break;

        case TAG_TEMP:
            ls.last_temp = (float)raw_x / 256.0f + 25.0f;   /* persist across drains */
            break;

        default:
            break;
        }

        if (have_accel && have_gyro) {
            buf[produced].accel[0] =  p_accel[0];
            buf[produced].accel[1] = -p_accel[1];
            buf[produced].accel[2] = -p_accel[2];
            buf[produced].gyro[0]  =  p_gyro[0];
            buf[produced].gyro[1]  = -p_gyro[1];
            buf[produced].gyro[2]  = -p_gyro[2];
            buf[produced].temp_c   = ls.last_temp;
            buf[produced].seq      = ls.seq++;
            buf[produced].chip_ts  = 0;
            produced++;
            have_accel = have_gyro = 0;
        }
    }

    if (produced > 0) {
        uint8_t ts[4];
        if (burst_read(fd, addr, REG_TIMESTAMP0, ts, 4) == 0) {
            uint32_t burst_ts = (uint32_t)ts[0]
                              | ((uint32_t)ts[1] <<  8)
                              | ((uint32_t)ts[2] << 16)
                              | ((uint32_t)ts[3] << 24);
            for (int i = 0; i < produced; i++) {
                uint32_t age = (uint32_t)(produced - 1 - i) * ls.ticks_per_sample;
                buf[i].chip_ts = burst_ts - age;
            }
        }
    }

    *n_out = produced;
    return overflow ? 1 : 0;
}

/* ── Driver descriptors ────────────────────────────────────────────────────── */

const imu_ops_t lsm6dso_ops = {
    .name             = "lsm6dso",
    .experimental     = true,
    .probe            = lsm_probe,
    .reset            = lsm_reset,
    .init             = lsm_init,
    .read             = lsm_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .ts_tick_ns       = 25000,   /* 32-bit counter, 25 µs/tick */
    .supported_odr_hz   = { 12, 26, 52, 104, 208, 416, 833, 1660, 3332, 6664, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};

const imu_ops_t lsm6dsox_ops = {
    .name             = "lsm6dsox",
    .experimental     = true,
    .probe            = lsm_probe,
    .reset            = lsm_reset,
    .init             = lsm_init,
    .read             = lsm_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .ts_tick_ns       = 25000,   /* 32-bit counter, 25 µs/tick */
    .supported_odr_hz   = { 12, 26, 52, 104, 208, 416, 833, 1660, 3332, 6664, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};
