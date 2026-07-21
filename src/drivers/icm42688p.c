/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * icm42688p.c — ICM-42688-P IMU driver (TDK/InvenSense)
 *
 * FIFO mode using 16-byte Packet 3 format (accel + gyro + temp + timestamp).
 * Hardware timestamp via Bank 1 TMSTVAL registers (20-bit, 1 µs/tick).
 *
 * Chip axis convention (SparkFun DEV-21301, Adafruit 4754): X=bow, Y=port, Z=up.
 * Remap to NED board frame: Y flipped (port→starboard), Z flipped (up→down).
 * Physical orientation varies by breakout PCB; use [mount] rotation_euler_deg
 * to correct for your installation if needed.
 *
 * Register references: ICM-42688-P datasheet DS-000347 Rev 1.2.
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

/* ── Register addresses — Bank 0 ────────────────────────────────────────────── */

#define REG_DEVICE_CONFIG   0x11   /* bit0 = SOFT_RESET */
#define REG_INT_CONFIG      0x14
#define REG_FIFO_CONFIG     0x16   /* bits[7:6]: 00=bypass, 01=stream */
#define REG_FIFO_COUNTH     0x2E   /* high byte; burst-read latches count */
#define REG_FIFO_COUNTL     0x2F   /* low byte */
#define REG_FIFO_DATA       0x30   /* 16-byte packets */
#define REG_SIGNAL_PATH_RESET 0x4B /* bit1=FIFO_FLUSH */
#define REG_INTF_CONFIG1    0x4D   /* bit1=CLKSEL; default 0x91 (PLL) */
#define REG_PWR_MGMT0       0x4E   /* [3:2]=gyro, [1:0]=accel; 0x0F=both LN */
#define REG_GYRO_CONFIG0    0x4F   /* [7:5]=FS_SEL, [3:0]=ODR */
#define REG_ACCEL_CONFIG0   0x50   /* [7:5]=FS_SEL, [3:0]=ODR */
#define REG_TMST_CONFIG     0x54   /* bit4=TMST_TO_REGS_EN, bit0=TMST_EN */
#define REG_FIFO_CONFIG1    0x5F   /* bit3=TMST_FSYNC_EN, bit1=GYRO_EN, bit0=ACCEL_EN */
#define REG_FIFO_CONFIG2    0x60   /* watermark low 8 bits */
#define REG_FIFO_CONFIG3    0x61   /* watermark high 4 bits */
#define REG_INT_CONFIG1     0x64   /* bit4=INT_ASYNC_RESET (must write 0 after reset) */
#define REG_INT_SOURCE0     0x65   /* bit2=FIFO_THS_INT1_EN */
#define REG_WHO_AM_I        0x75   /* reads 0x47 */
#define REG_BANK_SEL        0x76   /* bank select: 0–4 */

/* ── Register addresses — Bank 1 ────────────────────────────────────────────── */

#define REG_TMSTVAL0        0x62   /* timestamp bits [7:0]  */
#define REG_TMSTVAL1        0x63   /* timestamp bits [15:8] */
#define REG_TMSTVAL2        0x64   /* timestamp bits [19:16] in [3:0] */

/* ── Chip identity ─────────────────────────────────────────────────────────── */

#define WHO_AM_I_VALUE  0x47

/* ── FIFO packet header bits ───────────────────────────────────────────────── */

#define FIFO_HDR_MSG    0x80   /* empty / invalid packet */
#define FIFO_HDR_ACCEL  0x40   /* accel data valid */
#define FIFO_HDR_GYRO   0x20   /* gyro data valid */

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float    accel_scale;       /* LSB → m/s² */
    float    gyro_scale;        /* LSB → rad/s */
    uint32_t seq;
    uint32_t ticks_per_sample;  /* µs between adjacent samples */
    uint32_t ts_last_raw;       /* last raw 20-bit TMSTVAL read */
    uint32_t ts_hi;             /* accumulated wrap carry (multiples of 2^20) */
} ic;

/* ── ODR and full-scale encoding ───────────────────────────────────────────── */

/* GYRO_CONFIG0 / ACCEL_CONFIG0 bits [3:0] — Table 58 / Table 64 */
static uint8_t odr_encode(int hz)
{
    if (hz <=   12) return 0x0B;  /* 12.5 Hz */
    if (hz <=   25) return 0x0A;
    if (hz <=   50) return 0x09;
    if (hz <=  100) return 0x08;
    if (hz <=  200) return 0x07;
    if (hz <= 1000) return 0x06;
    if (hz <= 2000) return 0x05;
    if (hz <= 4000) return 0x04;
    return 0x03;  /* 8000 Hz */
}

static int odr_actual(int hz)
{
    static const int steps[] = { 12, 25, 50, 100, 200, 1000, 2000, 4000, 8000 };
    for (int i = 0; i < 8; i++)
        if (hz <= steps[i]) return steps[i];
    return 8000;
}

/* GYRO_CONFIG0 bits [7:5]: FS_SEL.  Sensitivity: datasheet Table 4. */
static uint8_t gyro_fs_encode(int dps, float *scale)
{
    const float d2r = (float)(M_PI / 180.0);
    switch (dps) {
    case  125: *scale =  125.0f * d2r / 32768.0f; return (0x4 << 5);
    case  250: *scale =  250.0f * d2r / 32768.0f; return (0x3 << 5);
    case  500: *scale =  500.0f * d2r / 32768.0f; return (0x2 << 5);
    case 1000: *scale = 1000.0f * d2r / 32768.0f; return (0x1 << 5);
    default:
    case 2000: *scale = 2000.0f * d2r / 32768.0f; return (0x0 << 5);
    }
}

/* ACCEL_CONFIG0 bits [7:5]: FS_SEL.  Sensitivity: datasheet Table 4. */
static uint8_t accel_fs_encode(int g, float *scale)
{
    switch (g) {
    case  2: *scale =  2.0f * 9.80665f / 32768.0f; return (0x3 << 5);
    case  4: *scale =  4.0f * 9.80665f / 32768.0f; return (0x2 << 5);
    case 16: *scale = 16.0f * 9.80665f / 32768.0f; return (0x0 << 5);
    default:
    case  8: *scale =  8.0f * 9.80665f / 32768.0f; return (0x1 << 5);
    }
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int icm_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (i2c_reg_read(fd, addr, REG_WHO_AM_I, &who) < 0) {
        LOG_E("icm42688p: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_VALUE) {
        LOG_E("icm42688p: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_VALUE);
        return -1;
    }
    return 0;
}

static int icm_reset(int fd, uint8_t addr)
{
    /* Ensure we are in Bank 0 before reset. */
    if (i2c_reg_write(fd, addr, REG_BANK_SEL, 0) < 0) return -1;

    if (i2c_reg_write(fd, addr, REG_DEVICE_CONFIG, 0x01) < 0) return -1;  /* SOFT_RESET */

    for (int i = 0; i < 100; i++) {
        usleep(1000);
        uint8_t val;
        if (i2c_reg_read(fd, addr, REG_DEVICE_CONFIG, &val) < 0) return -1;
        if (!(val & 0x01)) goto reset_done;
    }
    LOG_W("icm42688p: SOFT_RESET did not clear after 100 ms\n");
    return -1;

reset_done:
    usleep(1000);
    return 0;
}

static int icm_init(int fd, uint8_t addr, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_hz);
    uint8_t gfs  = gyro_fs_encode(cfg->gyro_dps,  &gyro_scale);
    uint8_t afs  = accel_fs_encode(cfg->accel_g, &accel_scale);

    /* Must clear INT_ASYNC_RESET after reset (datasheet §12.6). */
    if (i2c_reg_write(fd, addr, REG_INT_CONFIG1, 0x00) < 0) return -1;

    /* Enable hardware timestamp latch to Bank 1 registers. */
    if (i2c_reg_write(fd, addr, REG_TMST_CONFIG, 0x11) < 0) return -1; /* TO_REGS_EN|EN */

    /* Enable accel + gyro in low-noise mode; wait for power-up. */
    if (i2c_reg_write(fd, addr, REG_PWR_MGMT0, 0x0F) < 0) return -1;
    usleep(200);

    /* Configure ODR and full-scale. */
    if (i2c_reg_write(fd, addr, REG_GYRO_CONFIG0,  (uint8_t)(gfs | odr)) < 0) return -1;
    if (i2c_reg_write(fd, addr, REG_ACCEL_CONFIG0, (uint8_t)(afs | odr)) < 0) return -1;

    /* Watermark in packets (one 16-byte packet per sample-pair). */
    int wm = cfg->fifo_wm;
    if (wm > 511) wm = 511;
    if (i2c_reg_write(fd, addr, REG_FIFO_CONFIG2, (uint8_t)(wm & 0xFF))        < 0) return -1;
    if (i2c_reg_write(fd, addr, REG_FIFO_CONFIG3, (uint8_t)((wm >> 8) & 0x0F)) < 0) return -1;

    /* Route FIFO threshold to INT1. */
    if (i2c_reg_write(fd, addr, REG_INT_SOURCE0, 0x04) < 0) return -1; /* FIFO_THS_INT1_EN */

    /* Enable accel, gyro, temp, and timestamp in FIFO packets. */
    if (i2c_reg_write(fd, addr, REG_FIFO_CONFIG1, 0x0B) < 0) return -1; /* TMST|GYRO|ACCEL */

    /* Flush FIFO before enabling stream mode. */
    if (i2c_reg_write(fd, addr, REG_SIGNAL_PATH_RESET, 0x02) < 0) return -1; /* FIFO_FLUSH */
    usleep(1000);

    /* Start stream mode: FIFO_CONFIG bits[7:6] = 01. */
    if (i2c_reg_write(fd, addr, REG_FIFO_CONFIG, 0x40) < 0) return -1;

    ic.accel_scale      = accel_scale;
    ic.gyro_scale       = gyro_scale;
    ic.seq              = 0;
    /* 1 µs/tick timestamp; ticks between adjacent samples at the actual ODR. */
    ic.ticks_per_sample = (uint32_t)(1000000u / (unsigned)odr_actual(cfg->odr_hz));
    ic.ts_last_raw      = 0;    /* counter restarts with the chip */
    ic.ts_hi            = 0;

    return 0;
}

/*
 * icm_read — drain FIFO and return calibrated sample-pairs.
 *
 * FIFO Packet 3 layout (16 bytes, accel+gyro+temp+timestamp enabled):
 *   [0]    header:  bit7=MSG(skip), bit6=ACCEL_valid, bit5=GYRO_valid
 *   [1–2]  accel X  (MSB first, big endian)
 *   [3–4]  accel Y
 *   [5–6]  accel Z
 *   [7–8]  gyro  X
 *   [9–10] gyro  Y
 *   [11–12] gyro Z
 *   [13]   temperature (int8_t, 1°C/LSB, 0 = 25°C)
 *   [14–15] FIFO timestamp (unused; absolute TS read from Bank 1 after drain)
 */
static int icm_read(int fd, uint8_t addr,
                    imu_sample_t *buf, int max, int *n_out)
{
    /* ── 1. Read FIFO byte count (big endian; COUNTH read latches both) ─── */
    uint8_t cnt[2];
    if (i2c_burst_read(fd, addr, REG_FIFO_COUNTH, cnt, 2) < 0) return -1;

    int n_bytes = (((int)(cnt[0] & 0x3F)) << 8) | cnt[1];
    int n_pkts  = n_bytes / 16;

    if (n_pkts == 0) {
        *n_out = 0;
        return 0;
    }

    /* ── 2. Drain FIFO packets ──────────────────────────────────────────── */
    int produced = 0;

    for (int i = 0; i < n_pkts && produced < max; i++) {
        uint8_t pkt[16];
        if (i2c_burst_read(fd, addr, REG_FIFO_DATA, pkt, 16) < 0) return -1;

        uint8_t hdr = pkt[0];
        if (hdr & FIFO_HDR_MSG) continue;                    /* empty/padding */
        if (!(hdr & FIFO_HDR_ACCEL) || !(hdr & FIFO_HDR_GYRO)) continue;

        int16_t ax = i2c_s16be(&pkt[1]);
        int16_t ay = i2c_s16be(&pkt[3]);
        int16_t az = i2c_s16be(&pkt[5]);
        int16_t gx = i2c_s16be(&pkt[7]);
        int16_t gy = i2c_s16be(&pkt[9]);
        int16_t gz = i2c_s16be(&pkt[11]);

        /*
         * Chip native: X=bow, Y=port, Z=up.
         * Flip Y (port→starboard) and Z (up→down) for NED board frame.
         */
        buf[produced].accel[0] =  ax * ic.accel_scale;
        buf[produced].accel[1] = -ay * ic.accel_scale;
        buf[produced].accel[2] = -az * ic.accel_scale;
        buf[produced].gyro[0]  =  gx * ic.gyro_scale;
        buf[produced].gyro[1]  = -gy * ic.gyro_scale;
        buf[produced].gyro[2]  = -gz * ic.gyro_scale;
        buf[produced].temp_c   = (float)(int8_t)pkt[13] + 25.0f;
        buf[produced].seq      = ic.seq++;
        buf[produced].chip_ts  = 0;
        produced++;
    }

    /* ── 3. Timestamp: read Bank 1 TMSTVAL, back-calculate per sample ───── */
    if (produced > 0) {
        if (i2c_reg_write(fd, addr, REG_BANK_SEL, 1) == 0) {
            uint8_t ts[3];
            if (i2c_burst_read(fd, addr, REG_TMSTVAL0, ts, 3) == 0) {
                /* TMSTVAL: 20-bit, 1 µs/tick.  ts[2][3:0] = bits [19:16]. */
                uint32_t raw = ((uint32_t)(ts[2] & 0x0F) << 16)
                             | ((uint32_t)ts[1] << 8)
                             | ts[0];
                /* Unwrap the 20-bit counter into 32 bits.  It wraps every
                 * 2^20 µs ≈ 1.05 s; the FIFO watermark makes read() run many
                 * times per second at every supported ODR, so at most one
                 * wrap occurs between drains.  (A >1 s stall would lose
                 * wraps; the dt clamp in imu.c bounds the damage.) */
                if (raw < ic.ts_last_raw)
                    ic.ts_hi += 1u << 20;
                ic.ts_last_raw = raw;
                uint32_t burst_ts = ic.ts_hi + raw;
                for (int i = 0; i < produced; i++) {
                    uint32_t age = (uint32_t)(produced - 1 - i) * ic.ticks_per_sample;
                    buf[i].chip_ts = burst_ts - age;
                }
            }
            i2c_reg_write(fd, addr, REG_BANK_SEL, 0);
        }
    }

    *n_out = produced;
    return 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const imu_ops_t icm42688p_ops = {
    .name             = "icm42688p",
    .experimental     = true,
    .probe            = icm_probe,
    .reset            = icm_reset,
    .init             = icm_init,
    .read             = icm_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .ts_tick_ns       = 1000,    /* TMSTVAL: 1 µs/tick (unwrapped to 32 bits) */
    .supported_odr_hz   = { 12, 25, 50, 100, 200, 1000, 2000, 4000, 8000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 0 },
};
