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
#define REG_CTRL9_XL          0x18  /* DEN_* | DEVICE_CONF | 0 */
#define REG_CTRL10_C          0x19  /* 0[7:6] | TIMESTAMP_EN | 0[4:0] */
#define REG_OUT_TEMP_L        0x20  /* temperature output L; H at 0x21 (256 LSB/°C, 0=25°C) */
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
    float    last_temp;         /* °C; persists across drains (temp batched at
                                 * 12.5 Hz, slower than we drain the FIFO) */
    uint32_t seq;               /* monotonic sample counter across all bursts */
    uint32_t ticks_per_sample;  /* chip timer ticks between adjacent samples */
    uint64_t ts_rejects;        /* bursts whose batched anchor failed its
                                 * check and fell back — see ism_read */
    uint64_t ts_reject_next;    /* next count worth a log line */
    uint64_t ts_fwd_rejects;    /* counter reads refused as too far ahead */
    uint64_t ts_fwd_next;       /* next count worth a log line */
    chip_ts_guard_t ts_guard;   /* keeps chip_ts increasing across burst seams */
} s;

/* A backward step larger than this is a counter reset, not read jitter, and is
 * re-seeded rather than corrected — see chip_ts.h.  One second of 25 µs ticks,
 * generous next to the millisecond-scale lag being corrected. */
#define TS_MAX_JITTER_TICKS  40000u

/* A burst that lands this far AHEAD of the previous one means the post-drain
 * counter read is garbage, not that time passed -- 25 us/tick, so 10 s. See
 * chip_ts_guard_forward_ok(). */
#define TS_MAX_FWD_TICKS     400000u

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
    if (hz <= 1660) return 0x8;
    if (hz <= 3332) return 0x9;
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
 * ism330dhcx_ops.supported_odr_hz, which test_drivers_registry pins.
 *
 * The loop bound and the clamp both derive from the table, so growing it is a
 * one-line edit.  They did not, before 3332 and 6664 were added, and the
 * hand-written "< 7" plus a literal fallthrough is exactly the shape that
 * silently keeps returning the old ceiling when a rung is appended.
 */
static int odr_actual(int hz)
{
    static const int steps[] = { 12, 26, 52, 104, 208, 416, 833, 1660,
                                 3332, 6664 };
    static const int n = (int)(sizeof steps / sizeof steps[0]);
    for (int i = 0; i < n; i++)
        if (hz <= steps[i]) return steps[i];
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
    uint8_t odr  = odr_encode(cfg->odr_hz);
    uint8_t xlfs = xl_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t gyfs = gy_fs_encode(cfg->gyro_dps, &gyro_scale);

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
    chip_ts_guard_reset(&s.ts_guard);
    /* Chip timer ticks between samples: 1 s / (25 µs/tick) / ODR_hz.
     *
     * Integer division, and it is inexact at most rungs — the 25 µs tick is
     * not a multiple of the sample period.  The relative error shrinks as the
     * rate climbs, so the two rates added last are the *best* behaved: 1660 Hz
     * is 24.096 -> 24 (0.4 %), while 3332 Hz is 12.005 -> 12 and 6664 Hz is
     * 6.002 -> 6, both 0.04 %.  Leave it alone.  The value only ages samples
     * backwards within one burst from an anchor that is re-read every drain
     * (see ism_read), so the error is bounded by the watermark depth rather
     * than accumulating. */
    s.ticks_per_sample = (uint32_t)(40000u / (unsigned)odr_actual(cfg->odr_hz));

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
 *  -1  — I2C error
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

    for (int i = 0; i < n_words && produced < max; i++) {
        uint8_t word[7];
        if (bus_burst_read(bus, REG_FIFO_DATA_OUT_TAG, word, 7) < 0) return -1;

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
                 * backwards across the seam — measured on a Pi 5 at 2-4
                 * reversals per 5 s.  chip_ts.h holds that correction.
                 *
                 * Reached whenever no timestamp word landed in this drain,
                 * which is normal at small watermarks and expected at
                 * fifo_wm < 8, where nothing is batched at all.
                 */
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
                if (!chip_ts_guard_forward_ok(&s.ts_guard, first,
                                              TS_MAX_FWD_TICKS)) {
                    first    = chip_ts_guard_next(&s.ts_guard,
                                                  s.ticks_per_sample);
                    burst_ts = first + span;
                    if (++s.ts_fwd_rejects >= s.ts_fwd_next) {
                        LOG_W("ism330dhcx: %llu post-drain timestamp read(s) "
                              "implausibly far ahead; extrapolating from the "
                              "previous burst\n",
                              (unsigned long long)s.ts_fwd_rejects);
                        s.ts_fwd_next *= 10;
                    }
                } else {
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

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const imu_ops_t ism330dhcx_ops = {
    .name             = "ism330dhcx",
    .experimental     = false,
    /* DS13012 Rev 7 §5.1.2 (protocol, mode 3) and §4.4.1 Table 5 (10 MHz).
     * No auto-increment bit: multi-byte transfers step the address when
     * CTRL3_C's IF_INC is set, which ism_init already does (0x44). */
    .bus_caps         = { .spi_capable = true, .spi_mode = 3,
                          .spi_max_hz = 10000000, .spi_inc_mask = 0 },
    .probe            = ism_probe,
    .reset            = ism_reset,
    .init             = ism_init,
    .read             = ism_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .ts_tick_ns       = 25000,   /* 32-bit counter, 25 µs/tick typical */
    .ts_tick_ns_actual = ism_ts_tick_ns_actual,
    /* DS13012 Rev 7 Table 43 (CTRL1_XL, §9.12) and Table 46 (CTRL2_G, §9.13):
     * both give 0x9 = 3.33 kHz and 0xA = 6.66 kHz in high-performance mode.
     * The two tables agreeing at the top is what makes writing one shared odr
     * code to both registers valid.  Batching keeps up: Table 29 (§9.5) gives
     * BDR_XL/BDR_GY 1001 = 3333 Hz and 1010 = 6667 Hz, so the (odr << 4) | odr
     * written to FIFO_CTRL3 still selects the matching batch rate. */
    .supported_odr_hz   = { 12, 26, 52, 104, 208, 416, 833, 1660, 3332, 6664, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};
