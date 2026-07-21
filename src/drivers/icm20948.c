/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * icm20948.c — ICM-20948 driver
 *
 * Bank 0: operational registers (data, FIFO, interrupts).
 * Bank 2: ODR / full-scale / filter configuration.
 *
 * Magnetometer (AK09916) is accessed via I2C bypass mode (BYPASS_EN=1 in
 * INT_PIN_CFG).  Configure separately with the ak09916 mag driver.
 *
 * Hardware timestamp: NOT available (chip_ts is always 0).
 * FIFO: 512 bytes, stream mode.  Each sample = 6 B accel + 6 B gyro = 12 B.
 * ODR: 1125 Hz / (1 + divider) — nearest supported values are in supported_odr_hz.
 *
 * Register references: ICM-20948 Datasheet DS-000189 Rev 1.6.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "drivers.h"
#include "i2c_io.h"
#include "log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Register addresses ────────────────────────────────────────────────────── */

/* Bank 0 */
#define B0_WHO_AM_I       0x00   /* fixed 0xEA */
#define B0_USER_CTRL      0x03   /* FIFO_EN | I2C_MST_EN | ... */
#define B0_PWR_MGMT_1     0x06   /* DEVICE_RESET | SLEEP | LP_EN | CLKSEL */
#define B0_PWR_MGMT_2     0x07   /* DISABLE_ACCEL[2:0] | DISABLE_GYRO[2:0] */
#define B0_INT_PIN_CFG    0x0F   /* INT1_ACTL | BYPASS_EN | ... */
#define B0_INT_ENABLE_1   0x11   /* RAW_DATA_0_RDY_EN */
#define B0_INT_STATUS_1   0x1A   /* RAW_DATA_0_RDY_INT */
#define B0_ACCEL_XOUT_H   0x2D
#define B0_TEMP_OUT_H     0x39
#define B0_FIFO_EN_2      0x67   /* ACCEL | GYRO_Z/Y/X | TEMP */
#define B0_FIFO_RST       0x68
#define B0_FIFO_MODE      0x69   /* 0=stream, 1=snapshot */
#define B0_FIFO_COUNTH    0x70
#define B0_FIFO_COUNTL    0x71
#define B0_FIFO_R_W       0x72
#define REG_BANK_SEL      0x7F

/* Bank 2 */
#define B2_GYRO_SMPLRT_DIV  0x00
#define B2_GYRO_CONFIG_1    0x01   /* DLPFCFG[2:0] | FS_SEL[1:0] | FCHOICE */
#define B2_ACCEL_SMPLRT_1   0x10   /* MSB[11:8] of 12-bit divider */
#define B2_ACCEL_SMPLRT_2   0x11   /* LSB[7:0] */
#define B2_ACCEL_CONFIG     0x14   /* DLPFCFG[2:0] | FS_SEL[1:0] | FCHOICE */

#define WHO_AM_I_VALUE      0xEA

/* USER_CTRL bits */
#define USER_CTRL_FIFO_EN   0x40

/* FIFO_EN_2 bits: accel (bit4) + gyro Z/Y/X (bits 3-1) */
#define FIFO_EN2_ACCEL_GYRO 0x1E

/* INT_PIN_CFG: BYPASS_EN (bit 1) — exposes AK09916 on host I2C bus */
#define INT_PIN_BYPASS_EN   0x02

/* PWR_MGMT_1: auto-select best clock */
#define CLKSEL_AUTO         0x01

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float    accel_scale;   /* m/s²/LSB */
    float    gyro_scale;    /* rad/s/LSB */
    uint32_t seq;
} s;

/* ── I2C helpers ────────────────────────────────────────────────────────────── */

/* Select register bank (0–3). Must be called before accessing any bank register. */
static int bank_sel(int fd, uint8_t addr, uint8_t bank)
{
    return i2c_reg_write(fd, addr, REG_BANK_SEL, (uint8_t)(bank << 4));
}

/* ── Full-scale encoding ────────────────────────────────────────────────────── */

static uint8_t gyro_fs_encode(int dps, float *scale)
{
    const float d2r = (float)(M_PI / 180.0);
    switch (dps) {
    case  250: *scale =  1.0f / 131.0f * d2r; return 0x00;
    case  500: *scale =  1.0f / 65.5f  * d2r; return 0x02;
    case 1000: *scale =  1.0f / 32.8f  * d2r; return 0x04;
    default:
    case 2000: *scale =  1.0f / 16.4f  * d2r; return 0x06;
    }
}

static uint8_t accel_fs_encode(int g, float *scale)
{
    switch (g) {
    case  2: *scale = 9.80665f / 16384.0f; return 0x00;
    case  4: *scale = 9.80665f /  8192.0f; return 0x02;
    case  8: *scale = 9.80665f /  4096.0f; return 0x04;
    default:
    case 16: *scale = 9.80665f /  2048.0f; return 0x06;
    }
}

/* ODR = 1125 Hz / (1 + divider). Compute divider for nearest ODR. */
static uint8_t gyro_smplrt_div(int odr_hz)
{
    if (odr_hz <= 0) return 0;
    int div = (1125 + odr_hz / 2) / odr_hz - 1;
    if (div < 0) div = 0;
    if (div > 255) div = 255;
    return (uint8_t)div;
}

/* Accel ODR divider is 12-bit; use same formula. */
static uint16_t accel_smplrt_div(int odr_hz)
{
    if (odr_hz <= 0) return 0;
    int div = (1125 + odr_hz / 2) / odr_hz - 1;
    if (div < 0) div = 0;
    if (div > 4095) div = 4095;
    return (uint16_t)div;
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int icm_probe(int fd, uint8_t addr)
{
    if (bank_sel(fd, addr, 0) < 0) return -1;
    uint8_t who;
    if (i2c_reg_read(fd, addr, B0_WHO_AM_I, &who) < 0) {
        LOG_E("icm20948: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_VALUE) {
        LOG_E("icm20948: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_VALUE);
        return -1;
    }
    return 0;
}

static int icm_reset(int fd, uint8_t addr)
{
    if (bank_sel(fd, addr, 0) < 0) return -1;

    /* Trigger device reset (bit 7); self-clears. */
    if (i2c_reg_write(fd, addr, B0_PWR_MGMT_1, 0x80) < 0) return -1;

    for (int i = 0; i < 100; i++) {
        usleep(1000);
        uint8_t val;
        if (i2c_reg_read(fd, addr, B0_PWR_MGMT_1, &val) < 0) return -1;
        if (!(val & 0x80)) goto reset_done;
    }
    LOG_W("icm20948: DEVICE_RESET did not clear after 100 ms\n");
    return -1;

reset_done:
    usleep(20000);  /* wait for PLL and registers to stabilise */
    return 0;
}

static int icm_init(int fd, uint8_t addr, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t  gfs  = gyro_fs_encode(cfg->gyro_dps, &gyro_scale);
    uint8_t  afs  = accel_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t  gdiv = gyro_smplrt_div(cfg->odr_hz);
    uint16_t adiv = accel_smplrt_div(cfg->odr_hz);

    /* ── Bank 0: wake up, enable sensors ─────────────────────────────────── */
    if (bank_sel(fd, addr, 0) < 0) return -1;

    /* Auto-select best clock source (PLL when gyro is on, relaxation osc otherwise). */
    if (i2c_reg_write(fd, addr, B0_PWR_MGMT_1, CLKSEL_AUTO) < 0) return -1;
    usleep(5000);

    /* Enable accel and gyro (clear disable bits). */
    if (i2c_reg_write(fd, addr, B0_PWR_MGMT_2, 0x00) < 0) return -1;

    /* ── Bank 2: sample rate + full-scale + DLPF ─────────────────────────── */
    if (bank_sel(fd, addr, 2) < 0) return -1;

    /* Gyro: sample rate divider, then config (FCHOICE=1, DLPFCFG=3 ~51Hz NBW). */
    if (i2c_reg_write(fd, addr, B2_GYRO_SMPLRT_DIV, gdiv) < 0) return -1;
    if (i2c_reg_write(fd, addr, B2_GYRO_CONFIG_1,
                  (uint8_t)(0x01 | (3 << 3) | gfs)) < 0) return -1;

    /* Accel: 12-bit sample rate divider, then config (FCHOICE=1, DLPFCFG=3 ~50Hz NBW). */
    if (i2c_reg_write(fd, addr, B2_ACCEL_SMPLRT_1, (uint8_t)(adiv >> 8)) < 0) return -1;
    if (i2c_reg_write(fd, addr, B2_ACCEL_SMPLRT_2, (uint8_t)(adiv & 0xFF)) < 0) return -1;
    if (i2c_reg_write(fd, addr, B2_ACCEL_CONFIG,
                  (uint8_t)(0x01 | (3 << 3) | afs)) < 0) return -1;

    /* ── Bank 0: FIFO + interrupts + bypass ──────────────────────────────── */
    if (bank_sel(fd, addr, 0) < 0) return -1;

    /* Enable FIFO in USER_CTRL (I2C master off). */
    if (i2c_reg_write(fd, addr, B0_USER_CTRL, USER_CTRL_FIFO_EN) < 0) return -1;

    /* Reset FIFO: assert then deassert. */
    if (i2c_reg_write(fd, addr, B0_FIFO_RST, 0x1F) < 0) return -1;
    usleep(100);
    if (i2c_reg_write(fd, addr, B0_FIFO_RST, 0x00) < 0) return -1;

    /* Stream mode: overwrite oldest when full. */
    if (i2c_reg_write(fd, addr, B0_FIFO_MODE, 0x00) < 0) return -1;

    /* Batch accel + gyro XYZ (12 bytes/sample); temp not needed in FIFO. */
    if (i2c_reg_write(fd, addr, B0_FIFO_EN_2, FIFO_EN2_ACCEL_GYRO) < 0) return -1;

    /* Assert INT1 on raw data ready so a GPIO line can wake the reader.
     * If no GPIO is wired, the reader uses a 10 ms timer fallback instead. */
    if (i2c_reg_write(fd, addr, B0_INT_ENABLE_1, 0x01) < 0) return -1;

    /* Enable I2C bypass so the AK09916 magnetometer is visible to the host. */
    if (i2c_reg_write(fd, addr, B0_INT_PIN_CFG, INT_PIN_BYPASS_EN) < 0) return -1;

    s.accel_scale = accel_scale;
    s.gyro_scale  = gyro_scale;
    s.seq         = 0;

    return 0;
}

/*
 * icm_read — drain FIFO and return calibrated sample-pairs.
 *
 * Each 12-byte FIFO word: ACCEL_X H/L, Y H/L, Z H/L, GYRO_X H/L, Y H/L, Z H/L.
 * Returns 0 on success, 1 on FIFO overflow detected, -1 on I2C error.
 * *n_out is set to number of samples written to buf[].
 *
 * Note: chip_ts is always 0 — no hardware timestamp on ICM-20948.
 */
static int icm_read(int fd, uint8_t addr,
                    imu_sample_t *buf, int max, int *n_out)
{
    *n_out = 0;

    /* Must be in Bank 0 (set during init; reader never changes banks). */

    /* ── 1. FIFO byte count (13-bit) ─────────────────────────────────────── */
    uint8_t cnt[2];
    if (i2c_burst_read(fd, addr, B0_FIFO_COUNTH, cnt, 2) < 0) return -1;
    int n_bytes   = (((int)(cnt[0] & 0x1F)) << 8) | cnt[1];
    int n_samples = n_bytes / 12;

    if (n_samples == 0) return 0;

    int overflow = (n_bytes >= 504);   /* ≥504 of 512 bytes: near/at capacity */

    if (n_samples > max) n_samples = max;

    /* ── 2. Burst read from FIFO_R_W ─────────────────────────────────────── */
    uint8_t raw[128 * 12];   /* max caller buf is 128 samples = 1536 bytes */
    int to_read = n_samples * 12;
    if (i2c_burst_read(fd, addr, B0_FIFO_R_W, raw, (uint16_t)to_read) < 0) return -1;

    /* ── 3. Parse and scale ───────────────────────────────────────────────── */
    for (int i = 0; i < n_samples; i++) {
        const uint8_t *p = raw + i * 12;

        int16_t ax = i2c_s16be(&p[0]);
        int16_t ay = i2c_s16be(&p[2]);
        int16_t az = i2c_s16be(&p[4]);
        int16_t gx = i2c_s16be(&p[6]);
        int16_t gy = i2c_s16be(&p[8]);
        int16_t gz = i2c_s16be(&p[10]);

        /*
         * Remap chip frame → NED-compatible board frame.
         * ICM-20948 chip-native axes (component-side up, X toward bow):
         * X=bow, Y=port, Z=up.  Flip Y and Z to match ISM330DHCX convention:
         * X=bow, Y=starboard, Z=down.  If your board has a different Y
         * orientation, add a yaw or roll correction in rotation_euler_deg.
         */
        buf[i].accel[0] =  ax * s.accel_scale;
        buf[i].accel[1] = -ay * s.accel_scale;
        buf[i].accel[2] = -az * s.accel_scale;
        buf[i].gyro[0]  =  gx * s.gyro_scale;
        buf[i].gyro[1]  = -gy * s.gyro_scale;
        buf[i].gyro[2]  = -gz * s.gyro_scale;
        buf[i].temp_c   = 0.0f;  /* read separately if needed */
        buf[i].chip_ts  = 0;     /* no hardware timestamp on ICM-20948 */
        buf[i].seq      = s.seq++;
    }

    /* Read temperature from live register (single read for the whole burst). */
    uint8_t tmp[2];
    if (i2c_burst_read(fd, addr, B0_TEMP_OUT_H, tmp, 2) == 0) {
        int16_t raw_temp = i2c_s16be(tmp);
        float temp_c = (float)raw_temp / 333.87f + 21.0f;
        for (int i = 0; i < n_samples; i++)
            buf[i].temp_c = temp_c;
    }

    *n_out = n_samples;
    return overflow ? 1 : 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const imu_ops_t icm20948_ops = {
    .name             = "icm20948",
    .experimental     = true,
    .probe            = icm_probe,
    .reset            = icm_reset,
    .init             = icm_init,
    .read             = icm_read,
    .has_fifo         = true,
    .has_hw_timestamp = false,   /* no chip timer — timestamps are wall-clock only */
    .supported_odr_hz   = { 225, 281, 375, 562, 1125, 0 },  /* 1125/(1+div) */
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 250, 500, 1000, 2000, 0 },
};
