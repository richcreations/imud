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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "chip_ts.h"
#include "st_freq_fine.h"
#include "st_fifo_ts.h"
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
#define REG_FUNC_CFG_ACCESS   0x01  /* FUNC_CFG_EN | SHUB_REG_ACCESS | … */
#define REG_CTRL4_C           0x13  /* … | DRDY_MASK | I2C_disable | … */
#define CTRL4_I2C_DISABLE     0x04  /* bit 2, DS13012 Table 49 */
#define REG_CTRL5_C           0x14  /* ROUNDING[1:0] | ST[1:0]_G | ST[1:0]_XL */
#define REG_STATUS_REG        0x1E  /* … | TDA | GDA | XLDA (primary interface) */
#define REG_CTRL9_XL          0x18  /* DEN_* | DEVICE_CONF | 0 */
#define REG_CTRL10_C          0x19  /* 0[7:6] | TIMESTAMP_EN | 0[4:0] */
#define REG_OUT_TEMP_L        0x20  /* temperature output L; H at 0x21 (256 LSB/°C, 0=25°C) */
#define REG_OUTX_L_G          0x22  /* gyro  X/Y/Z, little-endian, through 0x27 */
#define REG_OUTX_L_A          0x28  /* accel X/Y/Z, little-endian, through 0x2D */
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

/*
 * FIFO words per transaction, bounded by TIME rather than by count.
 *
 * A single SPI transfer holds the shared controller for its whole duration, so
 * a burst sized against one clock becomes a monopoly at a slower one.  32
 * words is 224 bytes: ~180 us at 10 MHz, but 1.8 ms at 1 MHz -- and the
 * magnetometer shares this controller.
 *
 * This is a BOUND, not a fix for anything observed.  It was written believing
 * it would cure a magnetometer starvation seen below 2.5 MHz; it did not --
 * the mag still delivered 0.0 samples/s at 1.0 and 1.5 MHz with the burst
 * time-bounded.  That fault is unrelated (it reproduces with the IMU at
 * 13 Hz, three bus transactions a second) and is recorded where it belongs, in
 * mmc5983ma.c.  What this does buy is that a burst sized against one clock
 * cannot silently become a controller monopoly at a slower one.
 *
 * So the cap is whatever fits in ST_FIFO_BURST_US, floored at one word.  The
 * 32-word ceiling still applies: past ~180 us a longer burst buys little
 * against the ~42 us per-transaction overhead, and it bounds the RP1
 * chip-select exposure bus_io.h documents for multi-byte reads.
 */
#define ST_FIFO_BURST_WORDS 32
#define ST_FIFO_BURST_US    400

static int st_fifo_burst_words(uint32_t spi_hz)
{
    if (spi_hz == 0) return ST_FIFO_BURST_WORDS;      /* unknown: assume fast */
    /* bytes that fit in the budget = hz * us / 1e6 / 8 */
    uint64_t bytes = (uint64_t)spi_hz * ST_FIFO_BURST_US / 8000000u;
    int words = (int)(bytes / 7u);
    if (words < 1)                   words = 1;
    if (words > ST_FIFO_BURST_WORDS) words = ST_FIFO_BURST_WORDS;
    return words;
}

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float    accel_scale;       /* LSB → m/s² */
    float    gyro_scale;        /* LSB → rad/s */
    float    last_temp;         /* °C; persists across drains (temp batched at
                                 * 12.5 Hz, slower than we drain the FIFO) */
    uint32_t seq;               /* monotonic sample counter across all bursts */
    uint32_t ticks_per_sample;  /* chip timer ticks between adjacent samples */
    uint64_t ts_rejects;        /* bursts whose batched anchor failed its
                                 * check and fell back — see ism_read */
    uint64_t ts_reject_next;    /* next count worth a log line */
    uint64_t ts_fwd_rejects;    /* counter reads refused as too far ahead */
    uint64_t ts_fwd_next;       /* next count worth a log line */
    uint64_t ts_bwd_rejects;    /* counter reads refused as too far behind */
    uint64_t ts_bwd_next;       /* next count worth a log line */
    chip_ts_guard_t ts_guard;   /* keeps chip_ts increasing across burst seams */
} s;

/* A backward step larger than this is a counter reset, not read jitter, and is
 * re-seeded rather than corrected — see chip_ts.h.  One second of 25 µs ticks,
 * generous next to the millisecond-scale lag being corrected. */
#define TS_MAX_JITTER_TICKS  40000u

/*
 * Slack on the forward bound, in sample periods.
 *
 * The next burst's oldest sample follows the previous burst's newest by ONE
 * sample period -- the FIFO queues what the reader missed, so starvation makes
 * bursts bigger, not later.  Eight periods is loose enough for scheduler jitter
 * and tight enough to catch the class of bad read it exists for: at
 * 104 Hz that is 3072 ticks against the 65,706 one landed at.  Overflow is the
 * only legitimate break in the chain, and the guard is reset on it instead.
 */
#define TS_FWD_SLACK_SETS    8u

/* ── ODR and full-scale encoding helpers ───────────────────────────────────── */

/*
 * The ladder is the divider chain in MILLI-HERTZ, not the datasheet's labels.
 *
 * DS13012 labels the ODR ladder 12.5 / 26 / 52 / 104 / 208 / 416 / 833 /
 * 1.67k / 3.33k / 6.67k, but those labels are a rounded view of one binary
 * divider chain: 6664 / 2^n gives 3332, 1666, 833, 416.5, 208.25, 104.125,
 * 52.06, 26.03 and 13.016.  Every label matches its rung to better than 0.4%
 * except the last, which the table calls 12.5 where the divider gives 13.016.
 *
 * Measured on the reference part over 1,624 samples in 119.8 s:
 * every rung lands on 6664/2^n scaled by this die's INTERNAL_FREQ_FINE trim
 * (+27 steps, x1.0405) to within 0.4%, the bottom rung included -- 13.55 Hz
 * predicted, 13.55 measured, against 13.03 if the rung really were 12.5.
 * chip_ts tracked the wall clock at 1.00063 across the same run, so the
 * timebase is not what is moving.
 *
 * Carrying the chain in milli-Hz removes the rounding entirely: 13016 IS the
 * rung, so nothing has to decide between 12 and 13.  ticks_per_sample follows
 * exactly — 40000000/13016 = 3073, against 3333 when the rung was called 12.
 * See the unit note at the top of drivers.h.
 */
/*
 * Returns the 4-bit ODR code for CTRL1_XL / CTRL2_G (Table 43 / Table 46).
 * Rounds up to the nearest supported rate.
 */
static uint8_t odr_encode(int mhz)
{
    if (mhz <=   13016) return 0x1;
    if (mhz <=   26031) return 0x2;
    if (mhz <=   52063) return 0x3;
    if (mhz <=  104125) return 0x4;
    if (mhz <=  208250) return 0x5;
    if (mhz <=  416500) return 0x6;
    if (mhz <=  833000) return 0x7;
    if (mhz <= 1666000) return 0x8;
    if (mhz <= 3332000) return 0x9;
    return 0xA;  /* 6664 Hz */
}

/*
 * The rate odr_encode() above will actually select, for the ticks_per_sample
 * calculation: the lowest supported rate >= hz, clamped to 6664. Rounds UP,
 * not to nearest — it has to mirror odr_encode() exactly or the chip-timer
 * tick arithmetic would be scaled for a rate the part is not running at.
 *
 * Kept local rather than calling snap_odr_up() from imu_math.h so the drivers
 * stay free of daemon-side dependencies; the steps[] table below must match
 * ism330dhcx_ops.supported_odr_mhz, which test_drivers_registry pins.
 *
 * The loop bound and the clamp both derive from the table, so growing it is a
 * one-line edit.  They did not, before 3332 and 6664 were added, and the
 * hand-written "< 7" plus a literal fallthrough is exactly the shape that
 * silently keeps returning a stale ceiling when a rung is appended.
 */
static int odr_actual(int mhz)
{
    static const int steps[] = { 13016, 26031, 52063, 104125, 208250,
                                 416500, 833000, 1666000, 3332000, 6664000 };
    static const int n = (int)(sizeof steps / sizeof steps[0]);
    for (int i = 0; i < n; i++)
        if (mhz <= steps[i]) return steps[i];
    return steps[n - 1];
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

static int ism_probe(const imud_bus_t *bus)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
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

static int ism_reset(const imud_bus_t *bus)
{
    /* Trigger software reset (bit 0 of CTRL3_C); self-clears after ~50 µs. */
    if (bus_reg_write(bus, REG_CTRL3_C, 0x01) < 0) return -1;

    for (int i = 0; i < 50; i++) {
        usleep(1000);
        uint8_t val;
        if (bus_reg_read(bus, REG_CTRL3_C, &val) < 0) return -1;
        if (!(val & 0x01)) goto reset_done;
    }
    LOG_W("ism330dhcx: SW_RESET did not clear after 50 ms\n");
    return -1;

reset_done:
    usleep(35000);  /* Ton = 35 ms from electrical characteristics table */
    return 0;
}

static int ism_init(const imud_bus_t *bus, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_mhz);
    uint8_t xlfs = xl_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t gyfs = gy_fs_encode(cfg->gyro_dps, &gyro_scale);

    /*
     * Put the register bank back to the main one, first thing.
     *
     * Belt and braces against a stray write to FUNC_CFG_ACCESS having switched
     * it -- which is exactly what an 8-bit-word SPI write could do on a Pi 5
     * before spi_reg_write() was made atomic (see bus_io.h). A switched bank
     * makes every subsequent read come from the embedded-function registers
     * and presents as total bus corruption that survives reset() and BOOT, so
     * it is worth one write to rule out, including on a part some other tool
     * or an older build left that way.
     */
    if (bus_reg_write(bus, REG_FUNC_CFG_ACCESS, 0x00) < 0) return -1;

    /*
     * SPI: disable the I2C block.  DS13012's device-initialisation procedure
     * (§ "The procedure to correctly initialize the device", step 3) is
     * explicit and has two branches:
     *
     *     a. SPI interface:  I2C_disable = 1 in CTRL4_C (13h)
     *                        and DEVICE_CONF = 1 in CTRL9_XL (18h).
     *     b. I2C interface:  I2C_disable = 0 (default) in CTRL4_C (13h)
     *                        and DEVICE_CONF = 1 in CTRL9_XL (18h).
     *
     * This driver has always done the DEVICE_CONF half and never the other,
     * so every SPI install has run with the I2C block live on a part being
     * driven over SPI.  §5 says the same thing on its own: "In order to
     * disable the I2C block, (I2C_disable) = 1 must be written in CTRL4_C".
     *
     * Conditional on the transport rather than unconditional, because branch
     * (b) requires the opposite value: on I2C the bit must stay 0.
     *
     * Written FIRST, before any other configuration, so the interface is
     * settled before the registers that matter are set.
     */
    if (bus->kind == BUS_SPI &&
        bus_reg_write(bus, REG_CTRL4_C, CTRL4_I2C_DISABLE) < 0) return -1;

    /* Accel: ODR | FS | LPF2_XL_EN=1 | bit0=0 (must be 0) */
    if (bus_reg_write(bus, REG_CTRL1_XL, (uint8_t)((odr << 4) | xlfs | 0x02)) < 0) return -1;
    /* Gyro: ODR | FS */
    if (bus_reg_write(bus, REG_CTRL2_G,  (uint8_t)((odr << 4) | gyfs))         < 0) return -1;
    /* BDU=1 (no partial reads), IF_INC=1 (burst-read support) */
    if (bus_reg_write(bus, REG_CTRL3_C,  0x44)                                  < 0) return -1;
    /* DEVICE_CONF=1 — recommended by datasheet for all configurations */
    if (bus_reg_write(bus, REG_CTRL9_XL, 0x02)                                  < 0) return -1;
    /* Enable 32-bit hardware timestamp counter (25 µs/tick, regs 0x40–0x43) */
    if (bus_reg_write(bus, REG_CTRL10_C, 0x20)                                  < 0) return -1;

    /*
     * FIFO watermark: config value is in sample-sets (accel+gyro pairs).
     * Each sample-set = 2 FIFO words → multiply by 2 for the 9-bit WTM field.
     * WTM[8:0] spans FIFO_CTRL1 (bits [7:0]) and FIFO_CTRL2 (bit 0).
     */
    /*
     * Park the FIFO in Bypass before touching its configuration, so init()
     * always starts from an empty buffer whatever state the part was in.
     * Bypass is the ONLY thing that clears FIFO content: DS13012 §6.5.1,
     * "in Bypass mode the FIFO is not operational and it remains empty",
     * and §6.5.2, "to reset FIFO content, Bypass mode should be selected by
     * writing FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0]) to '000'".
     *
     * Writing Continuous over Continuous therefore flushes nothing, and a
     * second init() on a running part inherits the old buffer and lands on a
     * different register image than the first — which is exactly what
     * imud-imutest's imu.init.idempotent reports.  Every other FIFO driver
     * here already flushes (icm42688p's FIFO_FLUSH, icm20948's and mpu925x's
     * FIFO_RST pulse); the ST pair was the omission.
     *
     * Production behaviour is unchanged: both callers pair reset() with
     * init(), and SW_RESET leaves FIFO_MODE at its 000 default, so this write
     * is a no-op on that path.  What it buys is that the guarantee does not
     * depends on the caller remembering to reset first.
     *
     * No settle wait — the mode bits do not self-clear, and §6.5 has the part
     * accepting ODR/BDR changes "without disabling FIFO batching", so
     * reconfiguring on the fly is expected.
     */
    if (bus_reg_write(bus, REG_FIFO_CTRL4, 0x00)                        < 0) return -1;

    int wm = cfg->fifo_wm * 2;
    if (wm > 511) wm = 511;
    if (bus_reg_write(bus, REG_FIFO_CTRL1, (uint8_t)(wm & 0xFF))        < 0) return -1;
    if (bus_reg_write(bus, REG_FIFO_CTRL2, (uint8_t)((wm >> 8) & 0x01)) < 0) return -1;
    /* Batch accel and gyro at the same rate as ODR (BDR code == ODR code).
     * BDR is a separate encoding from ODR (Table 29, §9.5) that happens to
     * line up over the whole ODR range, 0x1 = 12.5 Hz through 0xA = 6667 Hz.
     * It diverges only at 0xB — 6.5 Hz on the gyro, 1.6 Hz on the accel —
     * which no ODR code can reach through (odr << 4) | odr. */
    if (bus_reg_write(bus, REG_FIFO_CTRL3, (uint8_t)((odr << 4) | odr)) < 0) return -1;
    /* Continuous mode + temperature batched at 12.5 Hz (ODR_T_BATCH = 10)
     * + the chip's own timestamp, decimated (DEC_TS_BATCH, bits [7:6]).
     * Temp words feed gyro thermal compensation and imud-cal fit-temp; at
     * 12.5 Hz they add ~1.5% FIFO traffic next to 833 Hz accel+gyro.  The
     * timestamp words are what let ism_read date a burst without reading the
     * counter after the drain — see st_fifo_ts.h, including why the
     * decimation is what it is and what it costs the watermark. */
    if (bus_reg_write(bus, REG_FIFO_CTRL4,
                      (uint8_t)((st_fifo_ts_dec_batch(cfg->fifo_wm) << 6) | 0x26))
                                                                        < 0) return -1;
    /* Assert INT1 on FIFO watermark threshold (INT1_FIFO_TH = bit 3) */
    if (bus_reg_write(bus, REG_INT1_CTRL,  0x08)                        < 0) return -1;

    s.accel_scale      = accel_scale;
    s.gyro_scale       = gyro_scale;
    /* Seed last_temp from the live temperature register so samples emitted
     * before the first batched FIFO temp word carry a real reading, not a
     * 25 °C placeholder. Falls back to 25 °C only if the read fails. */
    {
        uint8_t tb[2];
        if (bus_burst_read(bus, REG_OUT_TEMP_L, tb, 2) == 0) {
            int16_t rt = reg_s16le(tb);
            s.last_temp = (float)rt / 256.0f + 25.0f;
        } else {
            s.last_temp = 25.0f;
        }
    }
    s.seq              = 0;
    s.ts_rejects       = 0;
    s.ts_reject_next   = 1;
    s.ts_fwd_rejects   = 0;
    s.ts_fwd_next      = 1;
    s.ts_bwd_rejects   = 0;
    s.ts_bwd_next      = 1;
    chip_ts_guard_reset(&s.ts_guard);
    /* Chip timer ticks between samples: 1 s / (25 µs/tick) / ODR_hz.
     *
     * Integer division, and it is inexact at most rungs — the 25 µs tick is
     * not a multiple of the sample period.  The relative error shrinks as the
     * rate climbs, so the two rates added last are the *best* behaved: 1666 Hz
     * is 24.010 -> 24 (0.04 %), while 3332 Hz is 12.005 -> 12 and 6664 Hz is
     * 6.002 -> 6, both 0.04 %.  Leave it alone.  The value only ages samples
     * backwards within one burst from an anchor that is re-read every drain
     * (see ism_read), so the error is bounded by the watermark depth rather
     * than accumulating. */
    s.ticks_per_sample =
        (uint32_t)(40000000u / (unsigned)odr_actual(cfg->odr_mhz));

    return 0;
}

/*
 * This die's own timestamp period, from the factory trim (see st_freq_fine.h).
 * The bench part declares +27 steps = 24027 ns against the 25000 typical, which
 * agrees with the 24029 ns the wall-clock ratio implied — the chip and the
 * measurement telling the same story from opposite directions.
 */
static uint32_t ism_ts_tick_ns_actual(const imud_bus_t *bus)
{
    return st_freq_fine_tick_ns(bus, 25000);
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
 *  -1  — bus error
 */
static int ism_read(const imud_bus_t *bus,
                    imu_sample_t *buf, int max, int *n_out)
{
    /* ── 1. Read FIFO status ─────────────────────────────────────────────── */
    uint8_t st[2];
    if (bus_burst_read(bus, REG_FIFO_STATUS1, st, 2) < 0) return -1;

    int  n_words  = (((int)(st[1] & 0x03)) << 8) | st[0]; /* DIFF_FIFO[9:0] */
    int  overflow = (st[1] & 0x40) != 0;                   /* FIFO_OVR_IA */

    if (n_words == 0) {
        *n_out = 0;
        return overflow ? 1 : 0;
    }

    /* ── 2. Drain FIFO words, pair accel+gyro into samples ──────────────── */

    float    p_accel[3] = {0.0f, 0.0f, 0.0f};
    float    p_gyro[3]  = {0.0f, 0.0f, 0.0f};
    int      have_accel = 0;
    int      have_gyro  = 0;
    int      produced   = 0;
    uint8_t  set_tag    = 0;   /* tag byte of the word that completes a set */
    st_fifo_ts_t fts;
    st_fifo_ts_begin(&fts);
    const int burst_words = st_fifo_burst_words(bus->spi_hz);

    /*
     * Read many FIFO words per transaction, not one.
     *
     * This was one bus_burst_read() of 7 bytes per word, so a 37-set drain was
     * 74 ioctls.  At 6664 Hz that is ~42 us per transaction, 73
     * words per drain, 3.1 ms of transfer per drain at 216 drains/s -- the
     * reader spent about 0.67 s of every second BLOCKED in ioctl while using
     * 17 % of one core.  Not compute-bound, latency-bound: which is why extra
     * cores did not help, and why raising the SPI clock past 4 MHz stopped
     * helping.  What remains once clocking is free is per-transaction cost.
     *
     * The consequence was not merely slow.  The FIFO never fell below the
     * watermark, so a LEVEL interrupt never de-asserted, so no rising edge
     * arrived and the reader ran on its timer at every rate above 3332 Hz --
     * drains=0/N on a line that was working perfectly.
     *
     * DS13012 6.5.7 documents the 7-byte word (tag + 6 data) but does NOT say
     * that a read past 7Eh advances to the next word.  Every tag is checked
     * below, and imu.seq.gapless is the guard: a part that did not advance
     * would repeat one word, and accel/gyro must alternate.
     */
    for (int i = 0; i < n_words && produced < max; ) {
      uint8_t block[ST_FIFO_BURST_WORDS * 7];
      int want = n_words - i;
      if (want > burst_words) want = burst_words;
      /* i < n_words holds here, so want >= 1 and the read is >= 7 bytes.
       * Stated rather than left to the arithmetic: the static analyser
       * otherwise explores a one-byte read and reports the parse below taking
       * block[1] as a garbage value.  Same shape as the guards in mpu925x.c
       * and icm20948.c, and for the same reason -- a suppression would hide
       * the finding if the path ever became reachable.  The guard is on the
       * BYTE count, not the word count: bounding `want` alone left the
       * analyser unable to propagate through the multiplication. */
      int nbytes = want * 7;
      if (nbytes < 7) break;
      if (bus_burst_read(bus, REG_FIFO_DATA_OUT_TAG,
                         block, (uint16_t)nbytes) < 0) return -1;

      for (int w = 0; w < want && produced < max; w++, i++) {
        const uint8_t *word = block + w * 7;

        uint8_t tag   = (word[0] >> 3) & 0x1F;
        int16_t raw_x = reg_s16le(&word[1]);
        int16_t raw_y = reg_s16le(&word[3]);
        int16_t raw_z = reg_s16le(&word[5]);

        switch (tag) {
        case TAG_ACCEL_NC:
            p_accel[0] = raw_x * s.accel_scale;
            p_accel[1] = raw_y * s.accel_scale;
            p_accel[2] = raw_z * s.accel_scale;
            have_accel = 1;
            set_tag    = word[0];
            break;

        case TAG_GYRO_NC:
            p_gyro[0] = raw_x * s.gyro_scale;
            p_gyro[1] = raw_y * s.gyro_scale;
            p_gyro[2] = raw_z * s.gyro_scale;
            have_gyro = 1;
            set_tag    = word[0];
            break;

        case ST_TAG_TIMESTAMP:
            /* The chip's own counter, written into the stream alongside the
             * samples it dates.  st_fifo_ts.h holds the payload layout, the
             * TAG_CNT slot match, and what happens when either is wrong. */
            st_fifo_ts_note_word(&fts, word, produced);
            break;

        case TAG_TEMP:
            /* 16-bit signed, 256 LSB/°C, 0 LSB = 25 °C (Table 4).
             * Persist: batched slower than we drain, so most drains carry no
             * temp word and must reuse the last reading (not reset to 25). */
            s.last_temp = (float)raw_x / 256.0f + 25.0f;
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
            buf[produced].temp_c   = s.last_temp;
            buf[produced].seq      = s.seq++;
            buf[produced].chip_ts  = 0;  /* filled below */
            produced++;
            have_accel = have_gyro = 0;
            st_fifo_ts_note_set(&fts, set_tag);
        }
      }
    }

    /* ── 3. Timestamp the burst ─────────────────────────────────────────── */

    if (produced > 0) {
        uint8_t ts[4];
        if (bus_burst_read(bus, REG_TIMESTAMP0, ts, 4) == 0) {
            uint32_t now_ts = (uint32_t)ts[0]
                            | ((uint32_t)ts[1] <<  8)
                            | ((uint32_t)ts[2] << 16)
                            | ((uint32_t)ts[3] << 24);

            /*
             * Preferred: anchor on a timestamp word the chip wrote into the
             * FIFO alongside the samples, so nothing depends on when this
             * read happened.  st_fifo_ts.h carries the reasoning and the
             * checks; the register read above is kept as its cross-check even
             * on this path, which is the point of passing it in.
             */
            bool anchored = st_fifo_ts_apply(&fts, buf, produced,
                                             s.ticks_per_sample, now_ts);

            if (!anchored) {
                /*
                 * Fallback: treat now_ts as the newest sample's time and step
                 * back one sample period per older sample.
                 *
                 * "Treat" is load-bearing: this register reads NOW, and the
                 * newest sample was taken some time before this read
                 * completed.  That lag varies with bus timing and scheduler
                 * jitter, so two bursts can overlap and chip_ts can go
                 * backwards across the seam at 2-4
                 * reversals per 5 s.  chip_ts.h holds that correction.
                 *
                 * Reached whenever no timestamp word landed in this drain,
                 * which is normal at small watermarks and expected at
                 * fifo_wm < 8, where nothing is batched at all.
                 */
                /*
                 * An overflow is the one legitimate break in the chain: samples
                 * were dropped, so this burst's oldest does NOT follow the last
                 * burst's newest by a sample period and the guard has nothing
                 * useful to say.  Re-seed rather than widen the bound to cover it.
                 */
                if (overflow) chip_ts_guard_reset(&s.ts_guard);

                uint32_t burst_ts = now_ts;
                uint32_t span  = (uint32_t)(produced - 1) * s.ticks_per_sample;
                uint32_t first = burst_ts - span;
                /*
                 * Before trusting now_ts, ask whether it is credible. A garbage
                 * counter read lands the whole burst in the future, and the
                 * backward guard below cannot see that -- it only corrects
                 * overlaps. st_fifo_ts_apply() above has already rejected its
                 * anchor for exactly this reason when it fires, so using the
                 * same now_ts here unchecked is what let one bad read stamp
                 * nine samples 54 s ahead on the reference part.
                 */
                /*
                 * Both directions, and refuse either.  A read far
                 * BEHIND the previous burst was once passed
                 * through as a counter reset — see chip_ts.h for
                 * why that premise does not hold inside a run, and
                 * for the nine samples it sent out near 2^32.
                 */
                bool fwd_bad = !chip_ts_guard_forward_ok(
                        &s.ts_guard, first,
                        s.ticks_per_sample * TS_FWD_SLACK_SETS);
                bool bwd_bad = !chip_ts_guard_backward_ok(
                        &s.ts_guard, first, TS_MAX_JITTER_TICKS);
                if (fwd_bad || bwd_bad) {
                    /*
                     * Refusing read after read means the ANCHOR is stale, not that
                     * the part keeps lying: every correct reading looks equally
                     * implausible against a bad `last`.  Past the limit, take the
                     * reading and re-seed -- extrapolating again only walks the
                     * stamps further from real time.  See chip_ts.h.
                     */
                    if (chip_ts_guard_refused(&s.ts_guard)) {
                        chip_ts_guard_accepted(&s.ts_guard);   /* burst_ts stands */
                    } else {
                    first    = chip_ts_guard_next(&s.ts_guard,
                                                  s.ticks_per_sample);
                    burst_ts = first + span;
                    if (bwd_bad) {
                        if (++s.ts_bwd_rejects >= s.ts_bwd_next) {
                            LOG_W("ism330dhcx: %llu post-drain timestamp "
                                  "read(s) implausibly far behind; "
                                  "extrapolating from the previous burst\n",
                                  (unsigned long long)s.ts_bwd_rejects);
                            s.ts_bwd_next *= 10;
                        }
                    } else if (++s.ts_fwd_rejects >= s.ts_fwd_next) {
                        LOG_W("ism330dhcx: %llu post-drain timestamp read(s) "
                              "implausibly far ahead; extrapolating from the "
                              "previous burst\n",
                              (unsigned long long)s.ts_fwd_rejects);
                        s.ts_fwd_next *= 10;
                    }
                    }
                } else {
                    chip_ts_guard_accepted(&s.ts_guard);
                    burst_ts += chip_ts_guard_shift(&s.ts_guard, first,
                                                    s.ticks_per_sample,
                                                    TS_MAX_JITTER_TICKS);
                }
                for (int i = 0; i < produced; i++) {
                    uint32_t age = (uint32_t)(produced - 1 - i) * s.ticks_per_sample;
                    buf[i].chip_ts = burst_ts - age;
                }

                /*
                 * Only a word that was PRESENT and then refused is a finding.
                 * Say so at 1, 10, 100, ...: one line cannot tell a part that
                 * stumbled once from a payload layout that is not what
                 * st_fifo_ts.h assumes, and that distinction is the diagnosis.
                 */
                if (fts.have_word && ++s.ts_rejects >= s.ts_reject_next) {
                    LOG_W("ism330dhcx: %llu burst(s) whose batched FIFO "
                          "timestamp failed its check — using the post-drain "
                          "TIMESTAMP0 read for chip_ts\n",
                          (unsigned long long)s.ts_rejects);
                    s.ts_reject_next *= 10;
                }
            } else {
                /*
                 * The anchored path cannot produce the seam overlap the guard
                 * exists for, since the stamp came from the chip rather than
                 * from a read after the fact.  Run it anyway: two comparisons
                 * that should never fire, and the guard must not be left
                 * holding a stale anchor if a later burst falls back.
                 */
                uint32_t shift = chip_ts_guard_shift(&s.ts_guard, buf[0].chip_ts,
                                                     s.ticks_per_sample,
                                                     TS_MAX_JITTER_TICKS);
                for (int i = 0; shift && i < produced; i++)
                    buf[i].chip_ts += shift;
            }
            chip_ts_guard_note(&s.ts_guard, buf[produced - 1].chip_ts);
        }
        /* If the timestamp read fails, chip_ts stays 0; anchor detects this. */
    }

    *n_out = produced;
    return overflow ? 1 : 0;
}

/* ── Built-in self-test ───────────────────────────────────────────────────── */

/*
 * DS13012 Rev 7 defines the measurement (Table 2 notes 13 and 16, p.12): with
 * the device stationary, the absolute value of OUTPUT with self-test enabled
 * minus OUTPUT with it disabled.  The windows it must land in are Table 2
 * itself (p.11) — accel 40..1700 mg, full-scale independent per note 14; gyro
 * 150..700 dps, quoted per full scale, which is why the ranges below are
 * programmed rather than left at whatever the daemon configured.
 *
 * Diagnostic only: the daemon never calls this, and it leaves the part
 * configured for the measurement rather than for use.  See drivers.h.
 */

#define ST_ACCEL_LO_MG     40.0
#define ST_ACCEL_HI_MG   1700.0
#define ST_GYRO_LO_DPS    150.0
#define ST_GYRO_HI_DPS    700.0

/* Sensitivity of the two ranges programmed below, Table 2 cols LA_So / G_So. */
#define ST_MG_PER_LSB      0.122   /* ±4 g       */
#define ST_MDPS_PER_LSB   70.0     /* ±2000 dps  */

/*
 * Settle before each average.  Ton is 35 ms (electrical characteristics table,
 * p.11) and the same figure covers a full-scale or self-test change reaching
 * the output; 100 ms is that with margin, and at the 52 Hz programmed below it
 * is also five sample periods, so the discarded sample is genuinely stale.
 */
#define ST_SETTLE_US   100000
#define ST_AVG_N            5

#define ST_STATUS_XLDA  0x01
#define ST_STATUS_GDA   0x02

/*
 * Average ST_AVG_N fresh sample-sets from the direct output registers,
 * discarding one first.
 *
 * Gated on STATUS_REG rather than timed: a blind sleep would average the same
 * register image several times over if the part were not producing samples,
 * and read as a plausible number.  A part that produces nothing here fails the
 * call instead, which is the answer this whole check exists to get.
 */
static int st_average(const imud_bus_t *bus, uint8_t base, uint8_t ready,
                      double out[3])
{
    double acc[3] = { 0.0, 0.0, 0.0 };

    for (int i = 0; i < ST_AVG_N + 1; i++) {
        int waited;
        for (waited = 0; waited < 200; waited++) {
            uint8_t st;
            if (bus_reg_read(bus, REG_STATUS_REG, &st) < 0) return -1;
            if (st & ready) break;
            usleep(1000);
        }
        if (waited == 200) {
            LOG_E("ism330dhcx: self-test: no sample within 200 ms "
                  "(STATUS_REG bit 0x%02X never set)\n", ready);
            return -1;
        }

        uint8_t b[6];
        if (bus_burst_read(bus, base, b, 6) < 0) return -1;
        if (i == 0) continue;   /* discard the first: it may predate the settle */
        for (int a = 0; a < 3; a++)
            acc[a] += (double)reg_s16le(&b[a * 2]);
    }

    for (int a = 0; a < 3; a++) out[a] = acc[a] / (double)ST_AVG_N;
    return 0;
}

static int ism_self_test(const imud_bus_t *bus, imu_selftest_t *out)
{
    memset(out, 0, sizeof *out);
    out->accel_lo_mg  = ST_ACCEL_LO_MG;
    out->accel_hi_mg  = ST_ACCEL_HI_MG;
    out->gyro_lo_dps  = ST_GYRO_LO_DPS;
    out->gyro_hi_dps  = ST_GYRO_HI_DPS;

    if (ism_reset(bus) < 0) return -1;

    /* Bank and transport settled first, for the reasons ism_init gives. */
    if (bus_reg_write(bus, REG_FUNC_CFG_ACCESS, 0x00) < 0) return -1;
    if (bus->kind == BUS_SPI &&
        bus_reg_write(bus, REG_CTRL4_C, CTRL4_I2C_DISABLE) < 0) return -1;
    /* BDU=1 so a six-byte burst cannot straddle two samples, IF_INC=1 so it
     * walks the address at all. */
    if (bus_reg_write(bus, REG_CTRL3_C,  0x44) < 0) return -1;
    if (bus_reg_write(bus, REG_CTRL9_XL, 0x02) < 0) return -1;
    /* 52 Hz (ODR code 0x3) at ±4 g and ±2000 dps — the ranges the limits above
     * are quoted for.  LPF2 is deliberately left off: the datasheet defines the
     * measurement on the unfiltered output. */
    if (bus_reg_write(bus, REG_CTRL1_XL, (uint8_t)((0x3 << 4) | 0x08)) < 0) return -1;
    if (bus_reg_write(bus, REG_CTRL2_G,  (uint8_t)((0x3 << 4) | 0x0C)) < 0) return -1;

    int rc = -1;
    double off[3], on[3];

    usleep(ST_SETTLE_US);

    /* Accelerometer: ST1_XL=0, ST0_XL=1 — positive sign (Table 54). */
    if (st_average(bus, REG_OUTX_L_A, ST_STATUS_XLDA, off) < 0) goto done;
    if (bus_reg_write(bus, REG_CTRL5_C, 0x01) < 0) goto done;
    usleep(ST_SETTLE_US);
    if (st_average(bus, REG_OUTX_L_A, ST_STATUS_XLDA, on) < 0) goto done;
    if (bus_reg_write(bus, REG_CTRL5_C, 0x00) < 0) goto done;
    for (int a = 0; a < 3; a++)
        out->accel_mg[a] = fabs(on[a] - off[a]) * ST_MG_PER_LSB;

    /* Gyroscope: ST1_G=0, ST0_G=1 — positive sign (Table 53).  Its own
     * baseline rather than the accelerometer's, so nothing rests on the two
     * sensors' offsets having held still across the pass above. */
    usleep(ST_SETTLE_US);
    if (st_average(bus, REG_OUTX_L_G, ST_STATUS_GDA, off) < 0) goto done;
    if (bus_reg_write(bus, REG_CTRL5_C, 0x04) < 0) goto done;
    usleep(ST_SETTLE_US);
    if (st_average(bus, REG_OUTX_L_G, ST_STATUS_GDA, on) < 0) goto done;
    for (int a = 0; a < 3; a++)
        out->gyro_dps[a] = fabs(on[a] - off[a]) * ST_MDPS_PER_LSB / 1000.0;
    rc = 0;

done:
    /* Self-test off on every path, including the failures: the contract
     * promises the caller a part that is merely misconfigured, never one still
     * driving its proof masses. */
    if (bus_reg_write(bus, REG_CTRL5_C, 0x00) < 0) rc = -1;
    return rc;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const imu_ops_t ism330dhcx_ops = {
    .name             = "ism330dhcx",
    .experimental     = false,
    /* DS13012 Rev 7 §5.1.2 (protocol, mode 3) and §4.4.1 Table 5 (10 MHz).
     * No auto-increment bit: multi-byte transfers step the address when
     * CTRL3_C's IF_INC is set, which ism_init already does (0x44). */
    /*
     * Mode 0, not 3. DS13012 §5.1: "The device is compatible with SPI modes 0
     * and 3." Both sample on the rising edge; they differ only in the level
     * SCLK idles at.
     *
     * Mode 0 is chosen because the IMU and the magnetometer commonly share one
     * SPI controller (spidev0.0 and 0.1 on the reference rig) and a controller
     * cannot idle its clock at two levels at once, so they must agree.
     *
     * DS13012 permits either, and the MMC5983MA's datasheet points at mode 3
     * ("SCK ... is stopped high when CS is high", CPOL=1), so the choice rests
     * on measurement: with both parts in mode 3 the magnetometer delivers 92
     * samples/s at 6664 Hz against 103 in mode 0.
     */
    .bus_caps         = { .spi_capable = true, .spi_mode = 0,
                          .spi_max_hz = 10000000, .spi_inc_mask = 0 },
    .probe            = ism_probe,
    .reset            = ism_reset,
    .init             = ism_init,
    .read             = ism_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .ts_tick_ns       = 25000,   /* 32-bit counter, 25 µs/tick typical */
    .ts_tick_ns_actual = ism_ts_tick_ns_actual,
    .self_test        = ism_self_test,
    /* DS13012 Rev 7 Table 43 (CTRL1_XL, §9.12) and Table 46 (CTRL2_G, §9.13):
     * both give 0x9 = 3.33 kHz and 0xA = 6.66 kHz in high-performance mode.
     * The two tables agreeing at the top is what makes writing one shared odr
     * code to both registers valid.  Batching keeps up: Table 29 (§9.5) gives
     * BDR_XL/BDR_GY 1001 = 3333 Hz and 1010 = 6667 Hz, so the (odr << 4) | odr
     * written to FIFO_CTRL3 still selects the matching batch rate. */
    .supported_odr_mhz  = { 13016, 26031, 52063, 104125, 208250, 416500,
                            833000, 1666000, 3332000, 6664000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};
