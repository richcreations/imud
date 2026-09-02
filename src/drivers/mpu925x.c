/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mpu925x.c — InvenSense/TDK MPU-9250 and MPU-9255 driver
 *
 * One driver, two part numbers: the register maps are identical and only
 * WHO_AM_I differs (0x71 vs 0x73), so mpu9250_ops and mpu9255_ops share every
 * function.  Same arrangement as lsm6dso / lsm6dsox.
 *
 * The package contains an MPU-6500 (gyro + accel) and an AK8963 magnetometer
 * on the MPU's auxiliary bus.  The mag is reached via I2C bypass mode
 * (BYPASS_EN=1 in INT_PIN_CFG, I2C_MST_EN clear in USER_CTRL), which exposes
 * the AK8963 to the host bus at 0x0C.  Configure it separately with the
 * ak8963 mag driver — imu_ctx_open() always brings the IMU up first, which is
 * what opens the bypass.
 *
 * Hardware timestamp: NOT available (chip_ts is always 0).
 * FIFO: 512 bytes, overwrite-oldest.  Each sample = 6 B accel + 6 B gyro,
 *       written in register order (accel first), so 12 B per sample-set and
 *       about 42 sets of depth.  Temperature is read from the live register
 *       once per burst rather than batched into the FIFO: this is the
 *       smallest FIFO in the tree and batching temperature would cost a
 *       quarter of its depth.
 * ODR:  1000 Hz / (1 + SMPLRT_DIV) with the DLPF enabled.
 *
 * Counterfeit parts: the MPU-9250 breakout market is full of relabelled
 * MPU-6500s, which have no magnetometer at all.  probe() rejects WHO_AM_I
 * 0x70 by name and then verifies the AK8963 answers through the bypass, so
 * the most common failure mode becomes a self-explaining startup message
 * rather than a bare I2C error from the mag driver.
 *
 * Register references: MPU-9250 Register Map RM-MPU-9250A-00 Rev 1.4 and
 * MPU-9255 Register Map RM-000008 Rev 1.0 (identical register tables).
 * Sensitivity values: MPU-9250 Product Specification PS-MPU-9250A-01 Rev 1.0
 * Tables 1 and 2; temperature scaling from Table 4.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Register addresses (§3 Register map, RM-MPU-9250A-00 Rev 1.4) ────────── */

#define REG_SMPLRT_DIV     0x19   /* ODR = internal rate / (1 + SMPLRT_DIV)   */
#define REG_CONFIG         0x1A   /* FIFO_MODE | EXT_SYNC_SET | DLPF_CFG      */
#define REG_GYRO_CONFIG    0x1B   /* self-test | GYRO_FS_SEL | FCHOICE_B      */
#define REG_ACCEL_CONFIG   0x1C   /* self-test | ACCEL_FS_SEL                 */
#define REG_ACCEL_CONFIG2  0x1D   /* accel_fchoice_b | A_DLPF_CFG             */
#define REG_FIFO_EN        0x23   /* TEMP | GYRO Z/Y/X | ACCEL | SLV2/1/0     */
#define REG_INT_PIN_CFG    0x37   /* ACTL | OPEN | LATCH | ... | BYPASS_EN    */
#define REG_INT_ENABLE     0x38   /* FIFO_OVERFLOW_EN | RAW_RDY_EN            */
#define REG_INT_STATUS     0x3A   /* FIFO_OFLOW_INT | RAW_DATA_RDY_INT        */
#define REG_TEMP_OUT_H     0x41
#define REG_USER_CTRL      0x6A   /* FIFO_EN | I2C_MST_EN | FIFO_RST          */
#define REG_PWR_MGMT_1     0x6B   /* H_RESET | SLEEP | CYCLE | CLKSEL         */
#define REG_PWR_MGMT_2     0x6C   /* DIS_XA..DIS_ZG; 0x00 = everything on     */
#define REG_FIFO_COUNTH    0x72   /* [4:0] = FIFO_CNT[12:8]                   */
#define REG_FIFO_R_W       0x74
#define REG_WHO_AM_I       0x75

#define WHO_AM_I_MPU9250   0x71
#define WHO_AM_I_MPU9255   0x73
#define WHO_AM_I_MPU6500   0x70   /* the usual relabelled counterfeit         */

/* CONFIG: bit 6 clear = overwrite oldest when full (stream behaviour). */
#define CONFIG_FIFO_STREAM 0x00

/* FIFO_EN: accel (bit 3) + gyro Z/Y/X (bits 6-4). Temp (bit 7) deliberately
 * off — see the FIFO note in the file header. */
#define FIFO_EN_ACCEL_GYRO 0x78

/* USER_CTRL bits. I2C_MST_EN (0x20) stays clear so bypass can work. */
#define USER_CTRL_FIFO_EN  0x40
#define USER_CTRL_FIFO_RST 0x04

/* INT_PIN_CFG: BYPASS_EN (bit 1) — exposes the AK8963 on the host bus. */
#define INT_PIN_BYPASS_EN  0x02

/* INT_ENABLE: RAW_RDY_EN (bit 0) — data-ready on the INT pin. */
#define INT_ENABLE_RAW_RDY 0x01

/* PWR_MGMT_1 */
#define PWR1_H_RESET       0x80
#define PWR1_CLKSEL_AUTO   0x01   /* PLL when ready, else internal oscillator */

/* AK8963, seen through the bypass — probe() only. The full driver is
 * src/drivers/ak8963.c. */
#define AK8963_I2C_ADDR    0x0C
#define AK8963_REG_WIA     0x00
#define AK8963_WIA_VALUE   0x48

/* Chip constants */
#define FIFO_BYTES         512    /* §3.1; 12 B per sample-set                */
#define FIFO_SAMPLE_BYTES  12
#define FIFO_MAX_SETS      (FIFO_BYTES / FIFO_SAMPLE_BYTES)   /* 42 */
#define TEMP_SENSITIVITY   333.87f  /* LSB/°C, untrimmed (PS Table 4)         */
#define TEMP_OFFSET_C      21.0f    /* TEMP_degC = TEMP_OUT/Sens + 21         */

/* Gyro start-up, 35 ms typical from sleep (PS Table 1 — the 30 ms in reset()
 * is Table 2's ACCELEROMETER figure).  Doubled, as Linux's inv_mpu6050 does
 * for this family: the datasheet gives a typical, and FIFO_EN (Register 35)
 * says the FIFO buffers gyro data "even though that data path is not enabled",
 * so a FIFO started early batches samples nothing downstream can tell are
 * bad. */
#define GYRO_STARTUP_US    70000

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float    accel_scale;   /* m/s² per LSB */
    float    gyro_scale;    /* rad/s per LSB */
    uint32_t seq;
} s;

/* ── Encoding helpers ──────────────────────────────────────────────────────── */

/*
 * ODR = 1000 / (1 + SMPLRT_DIV) with the DLPF enabled (CONFIG.DLPF_CFG 1–6
 * gives an 1 kHz internal rate).  Returns the divider for the nearest
 * reachable rate; the caller reports what it actually got.
 */
static uint8_t smplrt_div_encode(int odr_mhz)
{
    if (odr_mhz <= 0) return 0;
    /* 1000 Hz / (1 + div), in milli-Hz: 1000000 / (1 + div). */
    int div = (1000000 + odr_mhz / 2) / odr_mhz - 1;
    if (div < 0)   div = 0;
    if (div > 255) div = 255;
    return (uint8_t)div;
}

static int odr_actual(uint8_t div)
{
    return 1000000 / (1 + (int)div);
}

/*
 * imu_ops_t::actual_odr_mhz. This part is divider-based, so it reaches rates
 * that are not in supported_odr_mhz at all and the shared snap-up default in
 * odr_actual_imu() would be wrong for it.
 */
static int mpu_actual_odr_mhz(int req_mhz)
{
    return odr_actual(smplrt_div_encode(req_mhz));
}

static uint8_t gyro_fs_encode(int dps, float *scale)
{
    /* PS Table 1: 131 / 65.5 / 32.8 / 16.4 LSB per °/s. */
    const float d2r = (float)(M_PI / 180.0);
    switch (dps) {
    case  250: *scale = 1.0f / 131.0f * d2r; return 0x00;
    case  500: *scale = 1.0f /  65.5f * d2r; return 0x08;
    case 1000: *scale = 1.0f /  32.8f * d2r; return 0x10;
    default:
    case 2000: *scale = 1.0f /  16.4f * d2r; return 0x18;
    }
}

static uint8_t accel_fs_encode(int g, float *scale)
{
    /* PS Table 2: 16384 / 8192 / 4096 / 2048 LSB per g. */
    switch (g) {
    case  2: *scale = 9.80665f / 16384.0f; return 0x00;
    case  4: *scale = 9.80665f /  8192.0f; return 0x08;
    case  8: *scale = 9.80665f /  4096.0f; return 0x10;
    default:
    case 16: *scale = 9.80665f /  2048.0f; return 0x18;
    }
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

/*
 * mpu_probe — identify the part, then prove the magnetometer is really there.
 *
 * `expect` is the caller's WHO_AM_I; probe_common accepts it and diagnoses the
 * two interesting wrong answers by name.  Enabling bypass here is safe and
 * idempotent: init() sets it again as its last step.
 */
static int probe_common(const imud_bus_t *bus, uint8_t expect, const char *part)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
        LOG_E("%s: WHO_AM_I read failed: %s\n", part, strerror(errno));
        return -1;
    }
    if (who != expect) {
        if (who == WHO_AM_I_MPU6500)
            LOG_E("%s: WHO_AM_I = 0x%02X — this is an MPU-6500, not an %s. "
                  "It has no magnetometer; boards sold as MPU-9250 are often "
                  "relabelled MPU-6500s.\n", part, who, part);
        else if (who == WHO_AM_I_MPU9250 || who == WHO_AM_I_MPU9255)
            LOG_E("%s: WHO_AM_I = 0x%02X, expected 0x%02X — this is the other "
                  "part; select driver '%s' instead.\n", part, who, expect,
                  who == WHO_AM_I_MPU9250 ? "mpu9250" : "mpu9255");
        else
            LOG_E("%s: WHO_AM_I = 0x%02X, expected 0x%02X\n", part, who, expect);
        return -1;
    }

    /*
     * Open the bypass and check the AK8963 answers.  A genuine part always
     * does; a clone with no magnetometer does not, and catching it here turns
     * the most common failure mode into one clear message instead of an
     * unexplained I2C error later, from the mag driver, about a different
     * address.
     */
    if (bus_reg_write(bus, REG_USER_CTRL, 0x00) < 0) return -1;
    if (bus_reg_write(bus, REG_INT_PIN_CFG, INT_PIN_BYPASS_EN) < 0) return -1;
    usleep(1000);

    /* The compass answers at its own address on the same wires, so borrow the
     * handle and swap only the address.  This is I2C-only by construction:
     * the bypass exists precisely to put the AKM die on the host I2C bus. */
    imud_bus_t akm = *bus;
    akm.i2c_addr = AK8963_I2C_ADDR;

    uint8_t wia;
    if (bus_reg_read(&akm, AK8963_REG_WIA, &wia) < 0 ||
        wia != AK8963_WIA_VALUE) {
        LOG_E("%s: no AK8963 magnetometer found at 0x%02X through the I2C "
              "bypass — this is probably a relabelled MPU-6500 (6-axis only). "
              "WHO_AM_I said 0x%02X but the compass did not answer.\n",
              part, AK8963_I2C_ADDR, who);
        return -1;
    }
    return 0;
}

static int mpu9250_probe(const imud_bus_t *bus)
{
    return probe_common(bus, WHO_AM_I_MPU9250, "mpu9250");
}

static int mpu9255_probe(const imud_bus_t *bus)
{
    return probe_common(bus, WHO_AM_I_MPU9255, "mpu9255");
}

/*
 * fifo_restart — empty the FIFO and resume writing on a packet boundary.
 *
 * Four writes, in the order the vendor sequence and Linux's inv_mpu6050 driver
 * use: stop the sensor writes at FIFO_EN (0x23), pulse FIFO_RST with
 * USER_CTRL's own FIFO_EN bit CLEAR, select the sensors again, and only then
 * reopen the port.  Pulsing the reset with the port already open leaves it
 * racing a sample-set that is part-way written, and this FIFO carries no
 * per-word tag to resync from afterwards.
 *
 * FIFO_RST self-clears after one clock cycle on silicon; USER_CTRL is written
 * back explicitly anyway so init() leaves a deterministic image
 * (`imud-imutest` compares two consecutive inits byte for byte).  That image is
 * USER_CTRL 0x40 and FIFO_EN 0x78.  I2C_MST_EN stays clear throughout — bypass
 * needs the master disabled.
 */
static int fifo_restart(const imud_bus_t *bus)
{
    if (bus_reg_write(bus, REG_FIFO_EN, 0x00) < 0) return -1;
    if (bus_reg_write(bus, REG_USER_CTRL, USER_CTRL_FIFO_RST) < 0) return -1;
    usleep(1000);
    if (bus_reg_write(bus, REG_FIFO_EN, FIFO_EN_ACCEL_GYRO) < 0) return -1;
    if (bus_reg_write(bus, REG_USER_CTRL, USER_CTRL_FIFO_EN) < 0) return -1;
    return 0;
}

static int mpu_reset(const imud_bus_t *bus)
{
    /* H_RESET (bit 7) restores defaults and self-clears. */
    if (bus_reg_write(bus, REG_PWR_MGMT_1, PWR1_H_RESET) < 0) return -1;

    for (int i = 0; i < 100; i++) {
        usleep(1000);
        uint8_t val;
        if (bus_reg_read(bus, REG_PWR_MGMT_1, &val) < 0) return -1;
        if (!(val & PWR1_H_RESET)) goto reset_done;
    }
    LOG_W("mpu925x: H_RESET did not clear after 100 ms\n");
    return -1;

reset_done:
    usleep(30000);   /* PS Table 2: 30 ms accelerometer start-up from cold */
    return 0;
}

static int mpu_init(const imud_bus_t *bus, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t gfs = gyro_fs_encode(cfg->gyro_dps,  &gyro_scale);
    uint8_t afs = accel_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t div = smplrt_div_encode(cfg->odr_mhz);

    /*
     * Two config values this chip cannot honour, adjusted here and logged once
     * so an operator who reads the daemon's status is not left wondering why
     * it is not running at the rate they asked for.
     */
    int actual_mhz = odr_actual(div);
    if (actual_mhz != cfg->odr_mhz)
        LOG_I("mpu925x: ODR %d.%03d Hz is not on the 1000/(1+SMPLRT_DIV) grid; "
              "using %d.%03d Hz (SMPLRT_DIV=%u)\n",
              cfg->odr_mhz / 1000, cfg->odr_mhz % 1000,
              actual_mhz / 1000, actual_mhz % 1000, div);

    int wm = cfg->fifo_wm;
    if (wm > FIFO_MAX_SETS) {
        LOG_I("mpu925x: fifo_wm %d exceeds this chip's %d sample-sets "
              "(%d-byte FIFO / %d B per set); the reader will drain whatever "
              "is pending instead\n",
              wm, FIFO_MAX_SETS, FIFO_BYTES, FIFO_SAMPLE_BYTES);
    }

    /* ── Wake up, pick a clock, enable both sensors ──────────────────────── */
    if (bus_reg_write(bus, REG_PWR_MGMT_1, PWR1_CLKSEL_AUTO) < 0) return -1;
    usleep(5000);
    if (bus_reg_write(bus, REG_PWR_MGMT_2, 0x00) < 0) return -1;

    /* ── Rate and filters ────────────────────────────────────────────────── */

    /* Gyro/temp DLPF: DLPF_CFG=3 is 41 Hz bandwidth at an 1 kHz internal rate,
     * which is what makes the 1000/(1+SMPLRT_DIV) grid available at all.
     * FIFO_MODE=0 so a full FIFO overwrites its oldest data. */
    if (bus_reg_write(bus, REG_CONFIG, CONFIG_FIFO_STREAM | 0x03) < 0) return -1;
    if (bus_reg_write(bus, REG_SMPLRT_DIV, div) < 0) return -1;

    /* Gyro full scale, FCHOICE_B=00 so the DLPF above is in circuit. */
    if (bus_reg_write(bus, REG_GYRO_CONFIG, gfs) < 0) return -1;

    /* Accel full scale, then its own filter: accel_fchoice_b=0 (DLPF enabled)
     * with A_DLPF_CFG=3, again 41 Hz at 1 kHz. */
    if (bus_reg_write(bus, REG_ACCEL_CONFIG,  afs)  < 0) return -1;
    if (bus_reg_write(bus, REG_ACCEL_CONFIG2, 0x03) < 0) return -1;

    /* PWR_MGMT_1/2 above woke the part and took the gyro out of standby; hold
     * the FIFO off until its sense path has come up. */
    usleep(GYRO_STARTUP_US);

    /* ── FIFO ────────────────────────────────────────────────────────────── */

    /* Start the FIFO clean and on a packet boundary. */
    if (fifo_restart(bus) < 0) return -1;

    /* Data-ready on INT so a GPIO line can wake the reader.  With no GPIO
     * wired the reader polls at the batch period, same as every other driver
     * here. */
    if (bus_reg_write(bus, REG_INT_ENABLE, INT_ENABLE_RAW_RDY) < 0) return -1;

    /* Last: open the bypass so the AK8963 is visible to the host bus. */
    if (bus_reg_write(bus, REG_INT_PIN_CFG, INT_PIN_BYPASS_EN) < 0) return -1;

    s.accel_scale = accel_scale;
    s.gyro_scale  = gyro_scale;
    s.seq         = 0;

    return 0;
}

/*
 * mpu_read — drain the FIFO and return calibrated sample-pairs.
 *
 * Each 12-byte word is ACCEL X/Y/Z then GYRO X/Y/Z, big-endian: the FIFO is
 * filled in register-number order and ACCEL_XOUT_H (0x3B) precedes
 * GYRO_XOUT_H (0x43).
 *
 * Returns 0 on success, 1 when a FIFO overflow was detected, -1 on I2C error.
 * chip_ts is always 0 — this part has no sample timer.
 */
static int mpu_read(const imud_bus_t *bus,
                    imu_sample_t *buf, int max, int *n_out)
{
    *n_out = 0;

    /* ── 1. Pending byte count (13-bit; reading COUNTH latches both) ─────── */
    uint8_t cnt[2];
    if (bus_burst_read(bus, REG_FIFO_COUNTH, cnt, 2) < 0) return -1;
    int n_bytes   = (((int)(cnt[0] & 0x1F)) << 8) | cnt[1];
    int n_samples = n_bytes / FIFO_SAMPLE_BYTES;

    /*
     * Overflow is latched in INT_STATUS, which is read-to-clear, so it must be
     * sampled every pass rather than only when the FIFO looks full: by the
     * time a burst has been drained the depth no longer shows what happened.
     */
    uint8_t istat = 0;
    if (bus_reg_read(bus, REG_INT_STATUS, &istat) < 0) return -1;
    int overflow = (istat & 0x10) ? 1 : 0;

    /*
     * An overflow destroys packet framing permanently, so the FIFO has to be
     * restarted rather than drained.  The buffer is 512 bytes and a sample-set
     * is 12: 512 % 12 = 8, so when the part drops the oldest data to make room
     * (CONFIG bit[6] FIFO_MODE = 0, which init() selects) the bytes it drops
     * are not a whole number of sample-sets.  Every later read is then parsed
     * one field late, and nothing in the stream reveals it — unlike the ST
     * parts, this FIFO carries no per-word tag to resync from.
     *
     * Draining anyway scrambles accel and gyro across each other: gravity
     * lands in the accel X slot, the gyro slots carry the *next* sample's
     * accel X/Y, and |a| still reads ~9.8 whenever one real axis happens to be
     * vertical — so a magnitude-only check cannot see it either.  Discarding
     * the buffered samples costs one drain; keeping them costs every drain
     * after it.
     */
    if (overflow) {
        if (fifo_restart(bus) < 0) return -1;
        return 1;
    }

    if (n_samples == 0) return 0;

    if (n_samples > max) n_samples = max;

    /* ── 2. Burst read from the FIFO port ────────────────────────────────── */
    enum { MPU_MAX_SETS = 128 };
    uint8_t raw[MPU_MAX_SETS * FIFO_SAMPLE_BYTES];
    /*
     * Clamp to what THIS buffer holds, not to what the caller offered.  The
     * caller's limit rose from 128 to IMU_DRAIN_MAX so one read could empty
     * the ST FIFO; sizing a driver's private buffer to the caller's old cap
     * and trusting it would have written 3072 bytes into 1536 here.  A driver
     * bounds its own storage.
     */
    if (n_samples > MPU_MAX_SETS) n_samples = MPU_MAX_SETS;
    int to_read = n_samples * FIFO_SAMPLE_BYTES;
    /*
     * n_samples is at least 1 here, so to_read is at least one whole
     * sample-set.  Stated rather than left implied: the static analyser
     * otherwise explores a to_read == 1 path into spi_burst_read's
     * single-word branch, which writes only buf[0], and then reports the
     * parse below reading p[1] as a garbage value.  Unreachable in fact --
     * to_read is always a multiple of the set size -- but the analyser cannot
     * see that, and a suppression would hide the same finding if it ever
     * became real.
     */
    if (to_read < FIFO_SAMPLE_BYTES) return -1;
    if (bus_burst_read(bus, REG_FIFO_R_W, raw, (uint16_t)to_read) < 0)
        return -1;

    /* ── 3. Parse and scale ──────────────────────────────────────────────── */
    for (int i = 0; i < n_samples; i++) {
        const uint8_t *p = raw + i * FIFO_SAMPLE_BYTES;

        int16_t ax = reg_s16be(&p[0]);
        int16_t ay = reg_s16be(&p[2]);
        int16_t az = reg_s16be(&p[4]);
        int16_t gx = reg_s16be(&p[6]);
        int16_t gy = reg_s16be(&p[8]);
        int16_t gz = reg_s16be(&p[10]);

        /*
         * Remap chip frame → NED-compatible board frame.
         * MPU-925x chip-native axes (component-side up, X toward bow):
         * X=bow, Y=port, Z=up.  Flip Y and Z to match the ISM330DHCX
         * convention: X=bow, Y=starboard, Z=down.  If your board has a
         * different orientation, correct it in rotation_euler_deg rather
         * than here.
         */
        buf[i].accel[0] =  ax * s.accel_scale;
        buf[i].accel[1] = -ay * s.accel_scale;
        buf[i].accel[2] = -az * s.accel_scale;
        buf[i].gyro[0]  =  gx * s.gyro_scale;
        buf[i].gyro[1]  = -gy * s.gyro_scale;
        buf[i].gyro[2]  = -gz * s.gyro_scale;
        buf[i].temp_c   = 0.0f;   /* filled in below */
        buf[i].chip_ts  = 0;      /* no hardware timestamp on MPU-925x */
        buf[i].seq      = s.seq++;
    }

    /* Temperature from the live register — one read for the whole burst. */
    uint8_t tmp[2];
    if (bus_burst_read(bus, REG_TEMP_OUT_H, tmp, 2) == 0) {
        int16_t raw_temp = reg_s16be(tmp);
        float temp_c = (float)raw_temp / TEMP_SENSITIVITY + TEMP_OFFSET_C;
        for (int i = 0; i < n_samples; i++)
            buf[i].temp_c = temp_c;
    }

    *n_out = n_samples;
    return overflow;
}

/* ── Driver descriptors ────────────────────────────────────────────────────── */

/*
 * supported_odr_mhz lists representative 1000/(1+SMPLRT_DIV) values, but the
 * divider reaches many rates that are not in it, so actual_odr_mhz is what
 * reports the rate the chip will really produce — the table is advice for the
 * operator, not the grid.  imud's shipped default of 833 Hz is not on the
 * divider grid either and lands on 1000 Hz (SMPLRT_DIV = 0).
 */
const imu_ops_t mpu9250_ops = {
    .name             = "mpu9250",
    .experimental     = true,
    .probe            = mpu9250_probe,
    .reset            = mpu_reset,
    .init             = mpu_init,
    .read             = mpu_read,
    .has_fifo         = true,
    .has_hw_timestamp = false,   /* no chip timer — wall-clock timestamps only */
    .supported_odr_mhz  = { 100000, 125000, 200000, 250000, 333333,
                            500000, 1000000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 250, 500, 1000, 2000, 0 },
    .actual_odr_mhz      = mpu_actual_odr_mhz,
};

const imu_ops_t mpu9255_ops = {
    .name             = "mpu9255",
    .experimental     = true,
    .probe            = mpu9255_probe,
    .reset            = mpu_reset,
    .init             = mpu_init,
    .read             = mpu_read,
    .has_fifo         = true,
    .has_hw_timestamp = false,
    .supported_odr_mhz  = { 100000, 125000, 200000, 250000, 333333,
                            500000, 1000000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 250, 500, 1000, 2000, 0 },
    .actual_odr_mhz      = mpu_actual_odr_mhz,
};
