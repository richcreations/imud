/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ism330dhcx.c — ISM330DHCX driver
 *
 * read() to drain the pending sample-pairs.
 *
 * Register references: ISM330DHCX datasheet DS13012 Rev 7.
 * Sensitivity values: datasheet Table 2 (Mechanical characteristics).
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

/* ── Register addresses (§8 Register mapping, DS13012 Rev 7) ──────────────── */

#define REG_FIFO_CTRL1        0x07  /* watermark [7:0] */
#define REG_FIFO_CTRL2        0x08  /* watermark [8], compression, ODR change */
#define REG_FIFO_CTRL3        0x09  /* BDR_GY[3:0] | BDR_XL[3:0] */
#define REG_FIFO_CTRL4        0x0A  /* DEC_TS_BATCH | ODR_T_BATCH | FIFO_MODE */
#define REG_INT1_CTRL         0x0D  /* INT1 source routing */
#define REG_WHO_AM_I          0x0F  /* fixed 0x6B */
#define REG_CTRL1_XL          0x10  /* accel: ODR[3:0] | FS[1:0] | LPF2 | 0 */
#define REG_CTRL2_G           0x11  /* gyro:  ODR[3:0] | FS[1:0] | FS_125 | FS_4000 */
#define REG_CTRL3_C           0x12  /* BOOT | BDU | H_LACTIVE | PP_OD | SIM | IF_INC | 0 | SW_RESET */
#define REG_CTRL9_XL          0x18  /* DEN_* | DEVICE_CONF | 0 */
#define REG_CTRL10_C          0x19  /* 0[7:6] | TIMESTAMP_EN | 0[4:0] */
#define REG_FIFO_STATUS1      0x3A  /* DIFF_FIFO[7:0] */
#define REG_FIFO_STATUS2      0x3B  /* FIFO_WTM_IA | FIFO_OVR_IA | FIFO_FULL_IA | ... | DIFF_FIFO[9:8] */
#define REG_TIMESTAMP0        0x40  /* 32-bit chip counter [7:0],  25 µs/tick */
#define REG_FIFO_DATA_OUT_TAG 0x78  /* tag byte; 0x79–0x7E follow: X_L/H Y_L/H Z_L/H */

/* ── FIFO tag sensor codes (datasheet Table 159) ───────────────────────────── */

#define TAG_GYRO_NC   0x01   /* gyroscope, non-compressed */
#define TAG_ACCEL_NC  0x02   /* accelerometer, non-compressed */
#define TAG_TEMP      0x03   /* temperature */

/* ── Chip identity ─────────────────────────────────────────────────────────── */

#define WHO_AM_I_VALUE  0x6B

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float    accel_scale;       /* LSB → m/s² */
    float    gyro_scale;        /* LSB → rad/s */
    uint32_t seq;               /* monotonic sample counter across all bursts */
    uint32_t ticks_per_sample;  /* chip timer ticks between adjacent samples */
} s;

/* ── Low-level I2C helpers ─────────────────────────────────────────────────── */

/*
 * Combined write-then-read in a single I2C transaction (no repeated-start
 * overhead). Saves ~40 µs vs separate transactions at 400 kHz.
 */
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

/*
 * Returns the 4-bit ODR code for CTRL1_XL / CTRL2_G (Table 43 / Table 46).
 * Rounds up to the nearest supported rate.
 */
static uint8_t odr_encode(int hz)
{
    if (hz <=   12) return 0x1;
    if (hz <=   26) return 0x2;
    if (hz <=   52) return 0x3;
    if (hz <=  104) return 0x4;
    if (hz <=  208) return 0x5;
    if (hz <=  416) return 0x6;
    if (hz <=  833) return 0x7;
    return 0x8;  /* 1660 Hz */
}

/* Nearest supported ODR for ticks_per_sample calculation. */
static int odr_actual(int hz)
{
    static const int steps[] = { 12, 26, 52, 104, 208, 416, 833, 1660 };
    for (int i = 0; i < 7; i++)
        if (hz <= steps[i]) return steps[i];
    return 1660;
}

/*
 * Returns the lower 4 bits of CTRL1_XL for the requested accel full-scale.
 * Writes the corresponding m/s²/LSB sensitivity to *scale.
 * Sensitivity from datasheet Table 2, col LA_So (mg/LSB × 9.80665 → m/s²/LSB).
 */
static uint8_t xl_fs_encode(int g, float *scale)
{
    switch (g) {
    case  2: *scale = 0.061e-3f * 9.80665f; return 0x00;  /* FS1=0,FS0=0 */
    case  4: *scale = 0.122e-3f * 9.80665f; return 0x08;  /* FS1=1,FS0=0 */
    case 16: *scale = 0.488e-3f * 9.80665f; return 0x04;  /* FS1=0,FS0=1 */
    default:                                               /* ±8g default */
    case  8: *scale = 0.244e-3f * 9.80665f; return 0x0C;  /* FS1=1,FS0=1 */
    }
}

/*
 * Returns the lower 4 bits of CTRL2_G for the requested gyro full-scale.
 * Writes the corresponding rad/s/LSB sensitivity to *scale.
 * Sensitivity from datasheet Table 2, col G_So (mdps/LSB × π/180/1000).
 */
static uint8_t gy_fs_encode(int dps, float *scale)
{
    const float d2r = (float)(M_PI / 180.0 / 1000.0);  /* mdps → rad/s */
    switch (dps) {
    case  125: *scale =   4.375f * d2r; return 0x02;  /* FS_125=1 */
    case  250: *scale =   8.75f  * d2r; return 0x00;  /* FS[1:0]=00 */
    case  500: *scale =  17.5f   * d2r; return 0x04;  /* FS[1:0]=01 */
    case 1000: *scale =  35.0f   * d2r; return 0x08;  /* FS[1:0]=10 */
    case 4000: *scale = 140.0f   * d2r; return 0x01;  /* FS_4000=1 */
    default:                                           /* ±2000 dps default */
    case 2000: *scale =  70.0f   * d2r; return 0x0C;  /* FS[1:0]=11 */
    }
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int ism_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (reg_read(fd, addr, REG_WHO_AM_I, &who) < 0) {
        LOG_E("ism330dhcx: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_VALUE) {
        LOG_E("ism330dhcx: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_VALUE);
        return -1;
    }
    return 0;
}

static int ism_reset(int fd, uint8_t addr)
{
    /* Trigger software reset (bit 0 of CTRL3_C); self-clears after ~50 µs. */
    if (reg_write(fd, addr, REG_CTRL3_C, 0x01) < 0) return -1;

    for (int i = 0; i < 50; i++) {
        usleep(1000);
        uint8_t val;
        if (reg_read(fd, addr, REG_CTRL3_C, &val) < 0) return -1;
        if (!(val & 0x01)) goto reset_done;
    }
    LOG_W("ism330dhcx: SW_RESET did not clear after 50 ms\n");
    return -1;

reset_done:
    usleep(35000);  /* Ton = 35 ms from electrical characteristics table */
    return 0;
}

static int ism_init(int fd, uint8_t addr, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_hz);
    uint8_t xlfs = xl_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t gyfs = gy_fs_encode(cfg->gyro_dps, &gyro_scale);

    /* Accel: ODR | FS | LPF2_XL_EN=1 | bit0=0 (must be 0) */
    if (reg_write(fd, addr, REG_CTRL1_XL, (uint8_t)((odr << 4) | xlfs | 0x02)) < 0) return -1;
    /* Gyro: ODR | FS */
    if (reg_write(fd, addr, REG_CTRL2_G,  (uint8_t)((odr << 4) | gyfs))         < 0) return -1;
    /* BDU=1 (no partial reads), IF_INC=1 (burst-read support) */
    if (reg_write(fd, addr, REG_CTRL3_C,  0x44)                                  < 0) return -1;
    /* DEVICE_CONF=1 — recommended by datasheet for all configurations */
    if (reg_write(fd, addr, REG_CTRL9_XL, 0x02)                                  < 0) return -1;
    /* Enable 32-bit hardware timestamp counter (25 µs/tick, regs 0x40–0x43) */
    if (reg_write(fd, addr, REG_CTRL10_C, 0x20)                                  < 0) return -1;

    /*
     * FIFO watermark: config value is in sample-sets (accel+gyro pairs).
     * Each sample-set = 2 FIFO words → multiply by 2 for the 9-bit WTM field.
     * WTM[8:0] spans FIFO_CTRL1 (bits [7:0]) and FIFO_CTRL2 (bit 0).
     */
    int wm = cfg->fifo_wm * 2;
    if (wm > 511) wm = 511;
    if (reg_write(fd, addr, REG_FIFO_CTRL1, (uint8_t)(wm & 0xFF))        < 0) return -1;
    if (reg_write(fd, addr, REG_FIFO_CTRL2, (uint8_t)((wm >> 8) & 0x01)) < 0) return -1;
    /* Batch accel and gyro at the same rate as ODR (BDR code == ODR code) */
    if (reg_write(fd, addr, REG_FIFO_CTRL3, (uint8_t)((odr << 4) | odr)) < 0) return -1;
    /* Continuous mode; no timestamp or temperature batching in FIFO */
    if (reg_write(fd, addr, REG_FIFO_CTRL4, 0x06)                        < 0) return -1;
    /* Assert INT1 on FIFO watermark threshold (INT1_FIFO_TH = bit 3) */
    if (reg_write(fd, addr, REG_INT1_CTRL,  0x08)                        < 0) return -1;

    s.accel_scale      = accel_scale;
    s.gyro_scale       = gyro_scale;
    s.seq              = 0;
    /* Chip timer ticks between samples: 1 s / (25 µs/tick) / ODR_hz */
    s.ticks_per_sample = (uint32_t)(40000u / (unsigned)odr_actual(cfg->odr_hz));

    return 0;
}

/*
 * ism_read — drain FIFO and return calibrated sample-pairs.
 *
 * Each FIFO word is 7 bytes: 1 tag byte + 6 data bytes (X_L/H, Y_L/H, Z_L/H).
 * Accel (tag 0x02) and gyro (tag 0x01) entries arrive interleaved at the same
 * BDR; we pair them into imu_sample_t structs.
 *
 * Returns:
 *   0  — success
 *   1  — success, but FIFO overflow was detected (data gap; set FLAG_FIFO_OVERFLOW)
 *  -1  — I2C error
 */
static int ism_read(int fd, uint8_t addr,
                    imu_sample_t *buf, int max, int *n_out)
{
    /* ── 1. Read FIFO status ─────────────────────────────────────────────── */
    uint8_t st[2];
    if (burst_read(fd, addr, REG_FIFO_STATUS1, st, 2) < 0) return -1;

    int  n_words  = (((int)(st[1] & 0x03)) << 8) | st[0]; /* DIFF_FIFO[9:0] */
    int  overflow = (st[1] & 0x40) != 0;                   /* FIFO_OVR_IA */

    if (n_words == 0) {
        *n_out = 0;
        return overflow ? 1 : 0;
    }

    /* ── 2. Drain FIFO words, pair accel+gyro into samples ──────────────── */

    float    p_accel[3] = {0.0f, 0.0f, 0.0f};
    float    p_gyro[3]  = {0.0f, 0.0f, 0.0f};
    float    p_temp     = 25.0f;
    int      have_accel = 0;
    int      have_gyro  = 0;
    int      produced   = 0;

    for (int i = 0; i < n_words && produced < max; i++) {
        uint8_t word[7];
        if (burst_read(fd, addr, REG_FIFO_DATA_OUT_TAG, word, 7) < 0) return -1;

        uint8_t tag   = (word[0] >> 3) & 0x1F;
        int16_t raw_x = (int16_t)(((uint16_t)word[2] << 8) | word[1]);
        int16_t raw_y = (int16_t)(((uint16_t)word[4] << 8) | word[3]);
        int16_t raw_z = (int16_t)(((uint16_t)word[6] << 8) | word[5]);

        switch (tag) {
        case TAG_ACCEL_NC:
            p_accel[0] = raw_x * s.accel_scale;
            p_accel[1] = raw_y * s.accel_scale;
            p_accel[2] = raw_z * s.accel_scale;
            have_accel = 1;
            break;

        case TAG_GYRO_NC:
            p_gyro[0] = raw_x * s.gyro_scale;
            p_gyro[1] = raw_y * s.gyro_scale;
            p_gyro[2] = raw_z * s.gyro_scale;
            have_gyro = 1;
            break;

        case TAG_TEMP:
            /* 16-bit signed, 256 LSB/°C, 0 LSB = 25 °C (Table 4) */
            p_temp = (float)raw_x / 256.0f + 25.0f;
            break;

        default:
            break;
        }

        if (have_accel && have_gyro) {
            /*
             * Remap chip frame → NED-compatible board frame.
             * ISM330DHCX (SparkFun 9DoF SEN-19895) chip-native axes when
             * component-side up, X toward bow:  X=bow, Y=port, Z=up.
             * Flip Y and Z so output is: X=bow, Y=starboard, Z=down.
             * With this convention, rotation_euler_deg = [0,0,0] is correct
             * for a component-up installation; only yaw is needed for heading
             * alignment.
             */
            buf[produced].accel[0] =  p_accel[0];
            buf[produced].accel[1] = -p_accel[1];
            buf[produced].accel[2] = -p_accel[2];
            buf[produced].gyro[0]  =  p_gyro[0];
            buf[produced].gyro[1]  = -p_gyro[1];
            buf[produced].gyro[2]  = -p_gyro[2];
            buf[produced].temp_c   = p_temp;
            buf[produced].seq      = s.seq++;
            buf[produced].chip_ts  = 0;  /* filled below */
            produced++;
            have_accel = have_gyro = 0;
        }
    }

    /* ── 3. Timestamp: read chip counter once, back-calculate per sample ── */

    if (produced > 0) {
        uint8_t ts[4];
        if (burst_read(fd, addr, REG_TIMESTAMP0, ts, 4) == 0) {
            /*
             * burst_ts ≈ timestamp of the most-recent (newest) sample.
             * Sample 0 is the oldest in the burst; sample (produced-1) newest.
             * Step back by ticks_per_sample for each older sample.
             */
            uint32_t burst_ts = (uint32_t)ts[0]
                              | ((uint32_t)ts[1] <<  8)
                              | ((uint32_t)ts[2] << 16)
                              | ((uint32_t)ts[3] << 24);
            for (int i = 0; i < produced; i++) {
                uint32_t age = (uint32_t)(produced - 1 - i) * s.ticks_per_sample;
                buf[i].chip_ts = burst_ts - age;
            }
        }
        /* If the timestamp read fails, chip_ts stays 0; anchor detects this. */
    }

    *n_out = produced;
    return overflow ? 1 : 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const imu_ops_t ism330dhcx_ops = {
    .name             = "ism330dhcx",
    .experimental     = false,
    .probe            = ism_probe,
    .reset            = ism_reset,
    .init             = ism_init,
    .read             = ism_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .supported_odr_hz   = { 12, 26, 52, 104, 208, 416, 833, 1660, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};
