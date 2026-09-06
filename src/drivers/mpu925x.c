/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mpu925x.c — InvenSense/TDK MPU-9250, MPU-9255 and MPU-6500 driver
 *
 * One driver, three part numbers.  The MPU-9250 and MPU-9255 differ only in
 * WHO_AM_I (0x71 vs 0x73); the MPU-6500 (0x70) is the gyro+accel die those
 * two package with an AK8963, so it answers the same register map with the
 * compass simply absent.  All three ops structs therefore share reset(),
 * init() and read(), and differ only in probe().  Same arrangement as
 * lsm6dso / lsm6dsox, one part number further.
 *
 * The 9250/9255 package contains an MPU-6500 (gyro + accel) and an AK8963
 * magnetometer on the MPU's auxiliary bus.  The mag is reached via I2C bypass mode
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
 * MPU-6500s, which have no magnetometer at all.  The 925x probes reject
 * WHO_AM_I 0x70 by name and then verify the AK8963 answers through the
 * bypass, so the most common failure mode becomes a self-explaining startup
 * message rather than a bare I2C error from the mag driver — and it now names
 * the mpu6500 driver, which runs such a board as the six-axis part it is.
 * That is also why mpu6500 is a registered driver rather than a debugging
 * aid: the MPU-9250 is discontinued, so a board bought as one today is more
 * likely to be a 6500 than not.
 *
 * Register references: MPU-9250 Register Map RM-MPU-9250A-00 Rev 1.4 and
 * MPU-9255 Register Map RM-000008 Rev 1.0 (identical register tables).
 * Sensitivity values: MPU-9250 Product Specification PS-MPU-9250A-01 Rev 1.0
 * Tables 1 and 2; temperature scaling from Table 4.
 */

#include <errno.h>
#include <math.h>
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
#define REG_SIGPATH_RESET  0x68   /* GYRO_RST | ACCEL_RST | TEMP_RST          */
#define REG_USER_CTRL      0x6A   /* FIFO_EN | I2C_MST_EN | FIFO_RST          */
#define REG_PWR_MGMT_1     0x6B   /* H_RESET | SLEEP | CYCLE | CLKSEL         */
#define REG_PWR_MGMT_2     0x6C   /* DIS_XA..DIS_ZG; 0x00 = everything on     */
#define REG_FIFO_COUNTH    0x72   /* [4:0] = FIFO_CNT[12:8]                   */
#define REG_FIFO_R_W       0x74
#define REG_WHO_AM_I       0x75

#define WHO_AM_I_MPU9250   0x71
#define WHO_AM_I_MPU9255   0x73
#define WHO_AM_I_MPU6500   0x70   /* six-axis die; also the usual counterfeit */

/* CONFIG: bit 6 clear = overwrite oldest when full (stream behaviour). */
#define CONFIG_FIFO_STREAM 0x00

/* FIFO_EN: accel (bit 3) + gyro Z/Y/X (bits 6-4). Temp (bit 7) deliberately
 * off — see the FIFO note in the file header. */
#define FIFO_EN_ACCEL_GYRO 0x78

/* USER_CTRL bits. I2C_MST_EN stays clear so bypass can work. */
#define USER_CTRL_FIFO_EN    0x40
#define USER_CTRL_I2C_MST_EN 0x20
#define USER_CTRL_FIFO_RST   0x04

/* INT_PIN_CFG: BYPASS_EN (bit 1) — exposes the AK8963 on the host bus. */
#define INT_PIN_BYPASS_EN  0x02

/* INT_ENABLE: RAW_RDY_EN (bit 0) — data-ready on the INT pin. */
#define INT_ENABLE_RAW_RDY 0x01

/* SIGNAL_PATH_RESET (Register 104): gyro, accel and temp digital signal
 * paths.  Sensor registers are not cleared — that is SIG_COND_RST's job. */
#define SIGPATH_RST_ALL    0x07

/* Self-test actuation bits: GYRO_CONFIG[7:5] XGYRO_Cten/YGYRO_Cten/ZGYRO_Cten
 * (register map §4.6) and ACCEL_CONFIG[7:5] ax_st_en/ay_st_en/az_st_en
 * (§4.7).  Both sit above the full-scale field, so a range is selected by
 * writing the encoded value and the enables are OR'd on top. */
#define GYRO_ST_ALL        0xE0
#define ACCEL_ST_ALL       0xE0

/* PWR_MGMT_1 */
#define PWR1_H_RESET       0x80
#define PWR1_CLKSEL_AUTO   0x01   /* PLL when ready, else internal oscillator */

/* PS §6.3 "start-up time for register read/write", from power-up: 11 ms
 * typical, 100 ms MAX.  The max is what a driver has to honour — it is the
 * point before which the part is not obliged to answer a register access at
 * all, and §7.1's note puts configuration writes "immediately after waiting
 * for" it.  Linux's inv_mpu6050 uses the same figure as
 * INV_MPU6050_POWER_UP_TIME. */
#define REG_STARTUP_US     100000

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

/* Gyro start-up, 35 ms typical from sleep (PS Table 1 — Table 2's 20/30 ms is
 * the ACCELEROMETER, and reset() waits on §6.3's register-interface figure,
 * which is a third question again).  Doubled, as Linux's inv_mpu6050 does
 * for this family: the datasheet gives a typical, and FIFO_EN (Register 35)
 * says the FIFO buffers gyro data "even though that data path is not enabled",
 * so a FIFO started early batches samples nothing downstream can tell are
 * bad. */
#define GYRO_STARTUP_US    70000

/*
 * Self-test measurement setup.  The response is a fixed deflection, so it is
 * taken on the most sensitive ranges — ±250 dps and ±2 g — at the 1 kHz
 * internal rate with DLPF_CFG 2, which Table 3 gives as 92 Hz: more bandwidth
 * than a DC deflection needs, and quieter than the settings above it.
 *
 * ST_SETTLE_US is the wait after the actuation is switched, and 25 ms is a
 * chosen figure rather than a datasheet one — the deflection has to travel the
 * 92 Hz filter's 3.9 ms group delay, and the FIFO is restarted after the wait
 * rather than drained, so nothing spanning the transition reaches an average.
 *
 * ST_SAMPLES is 0.2 s of sample-sets at that rate, which averages this part's
 * ~1 LSB noise floor down by an order of magnitude.
 */
#define ST_SMPLRT_DIV      0x00
#define ST_DLPF_CFG        0x02
#define ST_ACCEL_DLPF_CFG  0x02
#define ST_SETTLE_US       25000
#define ST_SAMPLES         192
#define ST_BURST           64
#define ST_TRIES           2000

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

/* The driver that does answer for a WHO_AM_I this family recognises, so a
 * mis-selected driver can name its replacement rather than just refusing. */
static const char *driver_for_whoami(uint8_t who)
{
    switch (who) {
    case WHO_AM_I_MPU6500: return "mpu6500";
    case WHO_AM_I_MPU9250: return "mpu9250";
    case WHO_AM_I_MPU9255: return "mpu9255";
    default:               return NULL;
    }
}

/*
 * mpu_probe — identify the part, and on the nine-axis parts prove the
 * magnetometer is really there.
 *
 * `expect` is the caller's WHO_AM_I; probe_common accepts it and diagnoses
 * every wrong answer this family can give by name.  `want_mag` is what
 * separates the three drivers: only the 925x pair has a compass to find, and
 * the MPU-6500 must not be failed for lacking one.  Enabling bypass here is
 * safe and idempotent: init() sets it again as its last step.
 */
static int probe_common(const imud_bus_t *bus, uint8_t expect, const char *part,
                        bool want_mag)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
        LOG_E("%s: WHO_AM_I read failed: %s\n", part, strerror(errno));
        return -1;
    }
    if (who != expect) {
        const char *other = driver_for_whoami(who);
        if (who == WHO_AM_I_MPU6500)
            LOG_E("%s: WHO_AM_I = 0x%02X — this is an MPU-6500, not an %s. It "
                  "has no magnetometer; boards sold as MPU-9250 are often "
                  "relabelled MPU-6500s. Select driver 'mpu6500' to run it as "
                  "the six-axis part it is.\n", part, who, part);
        else if (other)
            LOG_E("%s: WHO_AM_I = 0x%02X, expected 0x%02X — this is another "
                  "part in the family; select driver '%s' instead.\n",
                  part, who, expect, other);
        else
            LOG_E("%s: WHO_AM_I = 0x%02X, expected 0x%02X\n", part, who, expect);
        return -1;
    }

    /* Nothing on the auxiliary bus to find, and no bypass worth opening. */
    if (!want_mag) return 0;

    /*
     * Open the bypass and check the AK8963 answers.  A genuine part always
     * does; a clone with no magnetometer does not, and catching it here turns
     * the most common failure mode into one clear message instead of an
     * unexplained I2C error later, from the mag driver, about a different
     * address.
     *
     * Read-modify-write, because probe() must leave a configured part alone:
     * clear I2C_MST_EN, which is the one bit bypass needs down, and set
     * BYPASS_EN.  A blanket USER_CTRL = 0x00 also clears FIFO_EN, which is
     * invisible at startup — the daemon probes before init() — and wrong for
     * any caller that probes a running part.
     */
    uint8_t uc, pin;
    if (bus_reg_read(bus, REG_USER_CTRL, &uc) < 0) return -1;
    if (bus_reg_read(bus, REG_INT_PIN_CFG, &pin) < 0) return -1;
    if (bus_reg_write(bus, REG_USER_CTRL,
                      (uint8_t)(uc & ~USER_CTRL_I2C_MST_EN)) < 0) return -1;
    if (bus_reg_write(bus, REG_INT_PIN_CFG,
                      (uint8_t)(pin | INT_PIN_BYPASS_EN)) < 0) return -1;
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
              "WHO_AM_I said 0x%02X but the compass did not answer. Select "
              "driver 'mpu6500' to run it as a six-axis part.\n",
              part, AK8963_I2C_ADDR, who);
        return -1;
    }
    return 0;
}

static int mpu9250_probe(const imud_bus_t *bus)
{
    return probe_common(bus, WHO_AM_I_MPU9250, "mpu9250", true);
}

static int mpu6500_probe(const imud_bus_t *bus)
{
    return probe_common(bus, WHO_AM_I_MPU6500, "mpu6500", false);
}

static int mpu9255_probe(const imud_bus_t *bus)
{
    return probe_common(bus, WHO_AM_I_MPU9255, "mpu9255", true);
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

    /*
     * Wait the register interface out BEFORE reading it back.  A poll that
     * starts 1 ms in is questioning a part that PS §6.3 does not require to
     * answer for another 99, and the failure is silent in the worst
     * direction: a read that comes back 0x00 looks exactly like H_RESET
     * having cleared, so the loop exits early and every later write lands on
     * a part still resetting.
     */
    usleep(REG_STARTUP_US);

    for (int i = 0; i < 100; i++) {
        uint8_t val;
        if (bus_reg_read(bus, REG_PWR_MGMT_1, &val) < 0) return -1;
        if (!(val & PWR1_H_RESET)) goto reset_done;
        usleep(1000);
    }
    LOG_W("mpu925x: H_RESET did not clear after 100 ms\n");
    return -1;

reset_done:
    /*
     * Reset the three digital signal paths, then wait the interface out
     * again.  Linux's inv_mpu6050 does exactly this and applies it to
     * MPU-6000/6500/6515/6880/9250/9255 by name — the MPU-6050 it skips — so
     * it is a requirement of this generation rather than of the family.
     *
     * It matters most for the gyro.  Register 35's note is that an enabled
     * FIFO_EN bit buffers its slots "even though that data path is not
     * enabled", so a signal path left unreset does not produce an empty FIFO
     * or a short frame that a reader could notice: it produces full-width
     * sample-sets at the right rate whose gyro words are whatever a stopped
     * path emits.  Framing stays valid, accel stays correct, and only the
     * numbers are wrong.
     */
    if (bus_reg_write(bus, REG_SIGPATH_RESET, SIGPATH_RST_ALL) < 0) return -1;
    usleep(REG_STARTUP_US);
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

/* ── Built-in self-test ────────────────────────────────────────────────────── */

/*
 * st_average — mean of ST_SAMPLES consecutive sample-sets, taken out of the
 * FIFO through read().
 *
 * Through read() rather than the output registers, because on this part they
 * are two different paths and a part can drive one and not the other: the FIFO
 * is the one the daemon consumes, so it is the one worth proving.  It also
 * satisfies the contract's "wait for fresh samples rather than sleeping" with
 * no data-ready poll — the FIFO hands out each sample-set exactly once, so an
 * average here can never be one register image counted 192 times.
 *
 * An overflow has destroyed the framing (see mpu_read), so the partial sum is
 * dropped and the count restarts rather than mixing two framings.
 */
static int st_average(const imud_bus_t *bus, double acc[3], double gyr[3])
{
    imu_sample_t buf[ST_BURST];
    double sa[3] = { 0, 0, 0 }, sg[3] = { 0, 0, 0 };
    int got = 0;

    for (int t = 0; t < ST_TRIES && got < ST_SAMPLES; t++) {
        int n = 0;
        int rc = mpu_read(bus, buf, ST_BURST, &n);
        if (rc < 0) return -1;
        if (rc == 1) {
            got = 0;
            for (int k = 0; k < 3; k++) { sa[k] = 0.0; sg[k] = 0.0; }
            continue;
        }
        if (n == 0) { usleep(500); continue; }
        for (int i = 0; i < n && got < ST_SAMPLES; i++, got++)
            for (int k = 0; k < 3; k++) {
                sa[k] += (double)buf[i].accel[k];
                sg[k] += (double)buf[i].gyro[k];
            }
    }

    if (got < ST_SAMPLES) {
        LOG_E("mpu925x: self-test averaged %d of %d sample-sets — the part is "
              "producing no data\n", got, ST_SAMPLES);
        return -1;
    }
    for (int k = 0; k < 3; k++) { acc[k] = sa[k] / got; gyr[k] = sg[k] / got; }
    return 0;
}

/*
 * mpu_self_test — actuate the proof masses and report how far the output moved.
 *
 * PS Rev 1.0 §6.5 defines the figure as the difference between the output with
 * self-test enabled and without it, so the part is averaged twice with only
 * ACCEL_CONFIG and GYRO_CONFIG's top three bits changing between the passes.
 * Axes are the board frame read() returns, but that remap is sign-only on this
 * part, so a per-axis magnitude is the chip's own either way.
 *
 * The window is left 0/0 — ungraded, and imutest says so on the row.  §6.5
 * gives the limits only in AN-MPU-9250A-03, which this tree does not carry,
 * and the factory values in SELF_TEST_[XYZ]_GYRO / _ACCEL (§4.1, §4.2) are
 * per axis where imu_selftest_t's window is per sensor.  The response alone
 * still separates a live sensing element from a quiet one, which is the whole
 * job of the hook.
 */
static int mpu_self_test(const imud_bus_t *bus, imu_selftest_t *out)
{
    memset(out, 0, sizeof *out);

    if (mpu_reset(bus) < 0) return -1;

    if (bus_reg_write(bus, REG_PWR_MGMT_1, PWR1_CLKSEL_AUTO) < 0) return -1;
    usleep(5000);
    if (bus_reg_write(bus, REG_PWR_MGMT_2, 0x00) < 0) return -1;
    if (bus_reg_write(bus, REG_CONFIG,
                      CONFIG_FIFO_STREAM | ST_DLPF_CFG) < 0) return -1;
    if (bus_reg_write(bus, REG_SMPLRT_DIV, ST_SMPLRT_DIV) < 0) return -1;
    if (bus_reg_write(bus, REG_GYRO_CONFIG,  0x00) < 0) return -1;
    if (bus_reg_write(bus, REG_ACCEL_CONFIG, 0x00) < 0) return -1;
    if (bus_reg_write(bus, REG_ACCEL_CONFIG2, ST_ACCEL_DLPF_CFG) < 0) return -1;

    /* read() scales with whatever init() last left in s, so the two ranges
     * selected above have to be published to it here too. */
    (void)gyro_fs_encode(250, &s.gyro_scale);
    (void)accel_fs_encode(2,  &s.accel_scale);

    usleep(GYRO_STARTUP_US);
    if (fifo_restart(bus) < 0) return -1;

    int rc = -1;
    double a_off[3], g_off[3], a_on[3], g_on[3];

    if (st_average(bus, a_off, g_off) < 0) goto done;

    if (bus_reg_write(bus, REG_GYRO_CONFIG,  GYRO_ST_ALL)  < 0) goto done;
    if (bus_reg_write(bus, REG_ACCEL_CONFIG, ACCEL_ST_ALL) < 0) goto done;
    usleep(ST_SETTLE_US);
    if (fifo_restart(bus) < 0) goto done;

    if (st_average(bus, a_on, g_on) < 0) goto done;

    for (int k = 0; k < 3; k++) {
        out->accel_mg[k] = fabs(a_on[k] - a_off[k]) / 9.80665 * 1000.0;
        out->gyro_dps[k] = fabs(g_on[k] - g_off[k]) * 180.0 / M_PI;
    }
    rc = 0;

done:
    /* Self-test off on every path, including the failures: the contract
     * promises the caller a part that is merely misconfigured, never one still
     * driving its proof masses. */
    if (bus_reg_write(bus, REG_GYRO_CONFIG,  0x00) < 0) rc = -1;
    if (bus_reg_write(bus, REG_ACCEL_CONFIG, 0x00) < 0) rc = -1;
    return rc;
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
    .self_test        = mpu_self_test,
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
    .self_test        = mpu_self_test,
    .supported_odr_mhz  = { 100000, 125000, 200000, 250000, 333333,
                            500000, 1000000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 250, 500, 1000, 2000, 0 },
    .actual_odr_mhz      = mpu_actual_odr_mhz,
};

/*
 * Six-axis only.  Everything below the compass is the same silicon, so the
 * rates, ranges and hooks are the 9255's — pair it with a separate mag driver
 * in [mag], or run it with none.
 *
 * init() still writes INT_PIN_CFG's BYPASS_EN as its last step.  On this part
 * that connects auxiliary pins with nothing behind them to the host bus,
 * which is harmless and keeps one init() serving all three ops structs.
 */
const imu_ops_t mpu6500_ops = {
    .name             = "mpu6500",
    .experimental     = false,
    .probe            = mpu6500_probe,
    .reset            = mpu_reset,
    .init             = mpu_init,
    .read             = mpu_read,
    .has_fifo         = true,
    .has_hw_timestamp = false,
    .self_test        = mpu_self_test,
    .supported_odr_mhz  = { 100000, 125000, 200000, 250000, 333333,
                            500000, 1000000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 250, 500, 1000, 2000, 0 },
    .actual_odr_mhz      = mpu_actual_odr_mhz,
};
