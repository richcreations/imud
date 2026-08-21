/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ak8963.c — AKM AK8963 magnetometer driver (MPU-9250 / MPU-9255 compass)
 *
 * Requires I2C bypass mode (BYPASS_EN=1 in INT_PIN_CFG, set by the mpu925x
 * driver's init), which exposes the AK8963 on the host I2C bus at 0x0C.
 * imu_ctx_open() always brings the IMU up before the magnetometer, which is
 * what makes that ordering work.
 *
 * There is no external interrupt pin in bypass mode; mag_reader_thread uses a
 * 10 ms nanosleep loop and checks DRDY in read().  has_interrupt=false tells
 * imu.c to skip GPIO line setup for the mag.
 *
 * AK8963 has no degauss coil — set_reset is NULL.
 *
 * NOT interchangeable with the AK09916 in the ICM-20948, despite both being
 * AKM parts behind an InvenSense bypass:
 *   - different device ID: WIA (0x00) reads 0x48, against WIA2 (0x01) = 0x09
 *   - different register map: ST1 at 0x02 (not 0x10), data at 0x03 (not 0x11),
 *     ST2 at 0x09 (not 0x18), CNTL1 at 0x0A (not 0x31)
 *   - selectable 14-/16-bit output; this driver uses 16-bit, 0.15 µT/LSB
 *   - a per-device factory sensitivity adjustment in fuse ROM (see below)
 *   - a DIFFERENT axis alignment relative to the host gyro/accel (see below)
 *
 * Sensitivity adjustment: ASAX/ASAY/ASAZ are factory constants burned into
 * fuse ROM, readable only in FUSE_ROM_ACCESS mode.  Applied per axis as
 *
 *     H_adj = H × ((ASA − 128) × 0.5 / 128 + 1)
 *
 * imud fits its own hard- and soft-iron calibration afterwards, but this is
 * still applied: it is a known per-device constant, and leaving it out just
 * pushes a diagonal scale error into the soft-iron fit, where it is estimated
 * less precisely and conflated with the vessel's iron.
 *
 * Register references: MPU-9250 Register Map RM-MPU-9250A-00 Rev 1.4 §5 and
 * MPU-9255 Register Map RM-000008 Rev 1.0 §5 (identical).  Axis alignment
 * from PS-MPU-9250A-01 Rev 1.0 Figures 4 and 5, and DS-000007 Rev 1.0
 * Figures 4 and 5 — the two documents agree.  The continuous-measurement
 * rates (8 Hz and 100 Hz) are AK8963 part behaviour; the InvenSense register
 * maps name the modes but do not quote their rates.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "log.h"

/* ── Register addresses (§5, accessible in bypass mode) ───────────────────── */

#define REG_WIA    0x00   /* Device ID — reads 0x48                          */
#define REG_ST1    0x02   /* Status 1: DRDY (bit0), DOR (bit1)               */
#define REG_HXL    0x03   /* Measurement data HXL…HZH (6 bytes, LE)          */
#define REG_ST2    0x09   /* Status 2: HOFL (bit3), BITM (bit4)              */
#define REG_CNTL1  0x0A   /* Control 1: BIT (bit4) | MODE[3:0]               */
#define REG_CNTL2  0x0B   /* Control 2: SRST (bit0)                          */
#define REG_ASAX   0x10   /* Fuse ROM: ASAX/ASAY/ASAZ (fuse mode only)       */

#define WIA_VALUE  0x48

/* CNTL1 mode codes (low nibble) */
#define MODE_POWER_DOWN 0x00
#define MODE_CONT_1     0x02   /* continuous measurement mode 1 —   8 Hz */
#define MODE_CONT_2     0x06   /* continuous measurement mode 2 — 100 Hz */
#define MODE_FUSE_ROM   0x0F

/* CNTL1 bit 4: output bit setting. 1 = 16-bit (0.15 µT/LSB, ±4912 µT). */
#define CNTL1_16BIT     0x10

/* ST2 bit 3: magnetic sensor overflow — data is not reliable this sample. */
#define ST2_HOFL        0x08

/*
 * Sensitivity in 16-bit output mode: ±32760 counts over ±4912 µT
 * (RM §5.7 measurement-data table).  The 14-bit mode would be 0.6 µT/LSB.
 */
#define AK8963_SCALE    0.15f

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float adj[3];   /* per-axis fuse-ROM sensitivity multiplier, chip axes */
} s = { .adj = { 1.0f, 1.0f, 1.0f } };

/* Select the continuous mode code for the requested ODR. */
static uint8_t odr_to_mode(int odr_mhz)
{
    return (odr_mhz <= 8000) ? MODE_CONT_1 : MODE_CONT_2;
}

/*
 * Mode changes must pass through power-down: writing CNTL1 re-initialises
 * registers 0x02–0x09, and the part needs a settling gap before it will
 * accept the new mode.
 */
static int set_mode(const imud_bus_t *bus, uint8_t mode)
{
    if (bus_reg_write(bus, REG_CNTL1, MODE_POWER_DOWN) < 0) return -1;
    usleep(1000);
    if (bus_reg_write(bus, REG_CNTL1, mode) < 0) return -1;
    usleep(1000);
    return 0;
}

/* ── Driver operations ───────────────────────────────────────────────────── */

static int ak_probe(const imud_bus_t *bus)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WIA, &who) < 0) {
        LOG_E("ak8963: WIA read failed: %s — is the MPU-925x I2C bypass open? "
              "The IMU driver must be initialised first.\n", strerror(errno));
        return -1;
    }
    if (who != WIA_VALUE) {
        LOG_E("ak8963: WIA = 0x%02X, expected 0x%02X\n", who, WIA_VALUE);
        return -1;
    }
    return 0;
}

static int ak_reset(const imud_bus_t *bus)
{
    /* Power-down first, then soft reset. */
    if (bus_reg_write(bus, REG_CNTL1, MODE_POWER_DOWN) < 0) return -1;
    usleep(1000);

    if (bus_reg_write(bus, REG_CNTL2, 0x01) < 0) return -1;  /* SRST=1 */

    for (int i = 0; i < 20; i++) {
        usleep(1000);
        uint8_t val;
        if (bus_reg_read(bus, REG_CNTL2, &val) < 0) return -1;
        if (!(val & 0x01)) return 0;   /* self-clears when the reset completes */
    }
    LOG_W("ak8963: SRST did not clear after 20 ms\n");
    return -1;
}

static int ak_init(const imud_bus_t *bus, const mag_cfg_t *cfg)
{
    /* ── Read the factory sensitivity adjustment from fuse ROM ───────────── */
    if (set_mode(bus, MODE_FUSE_ROM) < 0) return -1;

    uint8_t asa[3];
    if (bus_burst_read(bus, REG_ASAX, asa, 3) < 0) {
        LOG_E("ak8963: fuse-ROM read failed: %s\n", strerror(errno));
        return -1;
    }

    /*
     * A programmed fuse ROM does not read as all-zero or all-ones: those are
     * what a die with no fuse ROM to read returns.  Say so loudly, because
     * the arithmetic below will not — ASA 0x00 yields a plausible-looking
     * 0.5x adjustment and 0xFF a 1.5x one, so a counterfeit or dead die would
     * otherwise go on to produce confidently-scaled nonsense.  Init still
     * succeeds: the reading is degraded, not impossible, and refusing to
     * start the daemon over a heuristic is the wrong trade.  The adjustment
     * is dropped to unity so at least no invented scale factor is applied.
     */
    bool no_fuse = (asa[0] == 0x00 && asa[1] == 0x00 && asa[2] == 0x00)
                || (asa[0] == 0xFF && asa[1] == 0xFF && asa[2] == 0xFF);
    if (no_fuse) {
        LOG_E("ak8963: fuse-ROM sensitivity reads %u/%u/%u, which no genuine "
              "AK8963 returns — the magnetometer die is very likely "
              "counterfeit or dead. Heading from it cannot be trusted. "
              "Applying no sensitivity adjustment.\n",
              asa[0], asa[1], asa[2]);
        for (int i = 0; i < 3; i++) s.adj[i] = 1.0f;
    } else {
        for (int i = 0; i < 3; i++)
            s.adj[i] = ((float)asa[i] - 128.0f) * 0.5f / 128.0f + 1.0f;
    }

    LOG_D("ak8963: fuse-ROM ASA = %u/%u/%u -> adj %.4f/%.4f/%.4f\n",
          asa[0], asa[1], asa[2],
          (double)s.adj[0], (double)s.adj[1], (double)s.adj[2]);

    /* ── Continuous measurement, 16-bit output ───────────────────────────── */
    return set_mode(bus, (uint8_t)(CNTL1_16BIT | odr_to_mode(cfg->odr_mhz)));
}

/*
 * ak_read — read one magnetometer sample.
 *
 * Polls ST1 DRDY for up to 15 ms (the longest inter-measurement gap at 100 Hz
 * is 10 ms).  If DRDY does not assert within that window, returns 1 — no data
 * yet, not an I2C error, so the reader retries instead of counting toward the
 * error-reset threshold.
 *
 * The 7-byte burst covers HXL(0x03) through ST2(0x09) in one transaction.
 * Reading ST2 is mandatory: it is what tells the part the host has finished
 * with this measurement, and DRDY does not clear until a data register or ST2
 * is read.
 */
static int ak_read(const imud_bus_t *bus, mag_sample_t *out)
{
    uint8_t st1 = 0;
    for (int i = 0; i < 15; i++) {
        if (bus_reg_read(bus, REG_ST1, &st1) < 0) return -1;
        if (st1 & 0x01) break;   /* DRDY asserted */
        usleep(1000);
    }
    if (!(st1 & 0x01))
        return 1;   /* no data yet — sensor may be powered down or still busy */

    /* HXL…HZH then ST2, little-endian 16-bit signed. */
    uint8_t raw[7];
    if (bus_burst_read(bus, REG_HXL, raw, 7) < 0) return -1;

    if (raw[6] & ST2_HOFL) {
        /* Magnetic overflow: this measurement is not reliable.  Transient
         * condition, not an I2C fault — skip the sample. */
        return 1;
    }

    int16_t hx = reg_s16le(&raw[0]);
    int16_t hy = reg_s16le(&raw[2]);
    int16_t hz = reg_s16le(&raw[4]);

    /* Factory sensitivity adjustment, applied in the AK8963's own axes. */
    float mx = (float)hx * s.adj[0] * AK8963_SCALE;
    float my = (float)hy * s.adj[1] * AK8963_SCALE;
    float mz = (float)hz * s.adj[2] * AK8963_SCALE;

    /*
     * Remap to the NED-compatible board frame (X=bow, Y=starboard, Z=down).
     *
     * Careful here — the AK8963 is NOT aligned with the MPU's gyro/accel
     * axes, unlike the AK09916 inside the ICM-20948.  Per PS-MPU-9250A-01
     * Figures 4 and 5 (and DS-000007 Figures 4 and 5, which agree), with the
     * package drawn in one orientation:
     *
     *     AK8963 X  =  MPU Y
     *     AK8963 Y  =  MPU X
     *     AK8963 Z  = −MPU Z
     *
     * The mpu925x driver maps MPU chip axes to the board frame as
     * X_board = MPU_X, Y_board = −MPU_Y, Z_board = −MPU_Z.  Composing:
     *
     *     X_board (bow)       =  MPU_X  =  AK_Y   ->  +my
     *     Y_board (starboard) = −MPU_Y  = −AK_X   ->  −mx
     *     Z_board (down)      = −MPU_Z  =  AK_Z   ->  +mz
     *
     * That is a proper rotation (determinant +1), not a mirror.  If your
     * board is mounted differently, correct it in rotation_euler_deg rather
     * than here.
     *
     * This mapping is exactly what `imud-imutest --spin` checks on real
     * hardware: it compares the mag heading against the gyro-Z integral, so a
     * sign or axis error here shows up as a frame-agreement failure.
     */
    out->field[0] =  my;   /* µT */
    out->field[1] = -mx;
    out->field[2] =  mz;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    out->valid   = true;

    return 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const mag_ops_t ak8963_ops = {
    .name            = "ak8963",
    .experimental    = true,
    .probe           = ak_probe,
    .reset           = ak_reset,
    .init            = ak_init,
    .read            = ak_read,
    .set_reset       = NULL,     /* no degauss coil */
    .has_interrupt   = false,    /* no external INT pin in bypass mode */
    .has_set_reset   = false,
    .supported_odr_mhz = { 8000, 100000, 0 },
};
