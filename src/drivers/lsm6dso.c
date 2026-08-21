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

#include "drivers.h"
#include "bus_io.h"
#include "chip_ts.h"
#include "st_freq_fine.h"
#include "st_fifo_ts.h"
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
    uint64_t ts_rejects;        /* bursts whose batched anchor was refused */
    uint64_t ts_reject_next;    /* next count worth a log line */
    uint64_t ts_fwd_rejects;    /* counter reads refused as too far ahead */
    uint64_t ts_fwd_next;       /* next count worth a log line */
    uint64_t ts_bwd_rejects;    /* counter reads refused as too far behind */
    uint64_t ts_bwd_next;       /* next count worth a log line */
    chip_ts_guard_t ts_guard;   /* see chip_ts.h */
} ls;

/* One second of 25 us ticks; see chip_ts.h. */
#define TS_MAX_JITTER_TICKS  40000u

/* A burst that lands this far AHEAD of the previous one means the post-drain
 * counter read is garbage, not that time passed -- 25 us/tick, so 10 s. See
 * chip_ts_guard_forward_ok(). */
#define TS_MAX_FWD_TICKS     400000u

/* ── ODR and full-scale encoding helpers ───────────────────────────────────── */

/*
 * The lowest rung is 13, not the 12.5 the datasheet's table prints.
 *
 * DS13012 labels the ODR ladder 12.5 / 26 / 52 / 104 / 208 / 416 / 833 /
 * 1.67k / 3.33k / 6.67k, but those labels are a rounded view of one binary
 * divider chain: 6664 / 2^n gives 3332, 1666, 833, 416.5, 208.25, 104.125,
 * 52.06, 26.03 and 13.016.  Every label matches its rung to better than 0.4%
 * except the last, which the table calls 12.5 where the divider gives 13.016.
 *
 * Measured on the reference part 2026-08-20, over 1,624 samples in 119.8 s:
 * every rung lands on 6664/2^n scaled by this die's INTERNAL_FREQ_FINE trim
 * (+27 steps, x1.0405) to within 0.4%, the bottom rung included -- 13.55 Hz
 * predicted, 13.55 measured, against 13.03 if the rung really were 12.5.
 * chip_ts tracked the wall clock at 1.00063 across the same run, so the
 * timebase is not what is moving.
 *
 * Declaring 12 was worse than the datasheet's own label: 12.5 rounds to 13 by
 * ordinary convention, and 13 is independently what the divider produces.  At
 * 12 the advertised rate sat 7.8% below what the part delivers, which is what
 * imu.odr reported at that rung on every run.  This also sharpens
 * ticks_per_sample: 40000/13 = 3076 against a true 3073, where 40000/12 gave
 * 3333.
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

/* Bound and clamp both derive from the table — see ism330dhcx.c, same shape
 * and the same reason. */
static int odr_actual(int mhz)
{
    static const int steps[] = { 13016, 26031, 52063, 104125, 208250,
                                 416500, 833000, 1666000, 3332000, 6664000 };
    static const int n = (int)(sizeof steps / sizeof steps[0]);
    for (int i = 0; i < n; i++)
        if (mhz <= steps[i]) return steps[i];
    return steps[n - 1];
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

static int lsm_probe(const imud_bus_t *bus)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
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

static int lsm_reset(const imud_bus_t *bus)
{
    if (bus_reg_write(bus, REG_CTRL3_C, 0x01) < 0) return -1;

    for (int i = 0; i < 50; i++) {
        usleep(1000);
        uint8_t val;
        if (bus_reg_read(bus, REG_CTRL3_C, &val) < 0) return -1;
        if (!(val & 0x01)) goto reset_done;
    }
    LOG_W("lsm6dso: SW_RESET did not clear after 50 ms\n");
    return -1;

reset_done:
    usleep(35000);
    return 0;
}

static int lsm_init(const imud_bus_t *bus, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_mhz);
    uint8_t xlfs = xl_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t gyfs = gy_fs_encode(cfg->gyro_dps, &gyro_scale);

    if (bus_reg_write(bus, REG_CTRL1_XL, (uint8_t)((odr << 4) | xlfs | 0x02)) < 0) return -1;
    if (bus_reg_write(bus, REG_CTRL2_G,  (uint8_t)((odr << 4) | gyfs))         < 0) return -1;
    if (bus_reg_write(bus, REG_CTRL3_C,  0x44)                                  < 0) return -1;
    /* No DEVICE_CONF write — not present on LSM6DSO */
    if (bus_reg_write(bus, REG_CTRL10_C, 0x20)                                  < 0) return -1;

    /* Park the FIFO in Bypass before reconfiguring it: Bypass is the only
     * thing that clears FIFO content (DS13012 §6.5.1/§6.5.2), so rewriting
     * Continuous over Continuous would leave a second init() holding the old
     * buffer.  See ism330dhcx.c for the full reasoning. */
    if (bus_reg_write(bus, REG_FIFO_CTRL4, 0x00)                        < 0) return -1;

    int wm = cfg->fifo_wm * 2;
    if (wm > 511) wm = 511;
    if (bus_reg_write(bus, REG_FIFO_CTRL1, (uint8_t)(wm & 0xFF))        < 0) return -1;
    if (bus_reg_write(bus, REG_FIFO_CTRL2, (uint8_t)((wm >> 8) & 0x01)) < 0) return -1;
    if (bus_reg_write(bus, REG_FIFO_CTRL3, (uint8_t)((odr << 4) | odr)) < 0) return -1;
    /* Continuous mode + temperature batched at 12.5 Hz (ODR_T_BATCH = 10)
     * + the decimated chip timestamp (DEC_TS_BATCH), matching ism330dhcx.c —
     * temp feeds thermal comp and imud-cal fit-temp, and the timestamp words
     * let lsm_read date a burst without a post-drain read (st_fifo_ts.h). */
    if (bus_reg_write(bus, REG_FIFO_CTRL4,
                      (uint8_t)((st_fifo_ts_dec_batch(cfg->fifo_wm) << 6) | 0x26))
                                                                        < 0) return -1;
    if (bus_reg_write(bus, REG_INT1_CTRL,  0x08)                        < 0) return -1;

    /* Seed last_temp from the live temperature register (see ism330dhcx.c). */
    {
        uint8_t tb[2];
        if (bus_burst_read(bus, REG_OUT_TEMP_L, tb, 2) == 0) {
            int16_t rt = reg_s16le(tb);
            ls.last_temp = (float)rt / 256.0f + 25.0f;
        } else {
            ls.last_temp = 25.0f;
        }
    }
    ls.accel_scale      = accel_scale;
    ls.gyro_scale       = gyro_scale;
    ls.seq              = 0;
    ls.ts_rejects       = 0;
    ls.ts_reject_next   = 1;
    ls.ts_fwd_rejects   = 0;
    ls.ts_fwd_next      = 1;
    ls.ts_bwd_rejects   = 0;
    ls.ts_bwd_next      = 1;
    chip_ts_guard_reset(&ls.ts_guard);
    ls.ticks_per_sample =
        (uint32_t)(40000000u / (unsigned)odr_actual(cfg->odr_mhz));

    return 0;
}

/*
 * This die's own timestamp period, from the factory trim (see st_freq_fine.h).
 * DS12140 Rev 3 §9.52 documents INTERNAL_FREQ_FINE identically to the
 * ISM330DHCX — same address, same 0.15% step, same "effective ODR (and
 * timestamp rate)" wording — but does not restate the TS_Res formula, so the
 * arithmetic is DS13012's applied to a register this datasheet defines the
 * same way.
 */
static uint32_t lsm_ts_tick_ns_actual(const imud_bus_t *bus)
{
    return st_freq_fine_tick_ns(bus, 25000);
}

static int lsm_read(const imud_bus_t *bus,
                    imu_sample_t *buf, int max, int *n_out)
{
    uint8_t st[2];
    if (bus_burst_read(bus, REG_FIFO_STATUS1, st, 2) < 0) return -1;

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
    uint8_t set_tag  = 0;
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
            p_accel[0] = raw_x * ls.accel_scale;
            p_accel[1] = raw_y * ls.accel_scale;
            p_accel[2] = raw_z * ls.accel_scale;
            have_accel = 1;
            set_tag    = word[0];
            break;

        case TAG_GYRO_NC:
            p_gyro[0] = raw_x * ls.gyro_scale;
            p_gyro[1] = raw_y * ls.gyro_scale;
            p_gyro[2] = raw_z * ls.gyro_scale;
            have_gyro  = 1;
            set_tag    = word[0];
            break;

        case ST_TAG_TIMESTAMP:
            st_fifo_ts_note_word(&fts, word, produced);   /* st_fifo_ts.h */
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
            st_fifo_ts_note_set(&fts, set_tag);
        }
    }

    if (produced > 0) {
        uint8_t ts[4];
        if (bus_burst_read(bus, REG_TIMESTAMP0, ts, 4) == 0) {
            uint32_t now_ts = (uint32_t)ts[0]
                            | ((uint32_t)ts[1] <<  8)
                            | ((uint32_t)ts[2] << 16)
                            | ((uint32_t)ts[3] << 24);

            /* Anchor on the chip's own batched timestamp where one landed in
             * this drain; st_fifo_ts.h holds the reasoning and the checks,
             * and takes now_ts as its cross-check.  Same shape as
             * ism330dhcx.c, which carries the fuller commentary. */
            bool anchored = st_fifo_ts_apply(&fts, buf, produced,
                                             ls.ticks_per_sample, now_ts);

            if (!anchored) {
                /* Read AFTER the drain, so it is "now" rather than the newest
                 * sample's time, and the lag varies with bus and scheduler
                 * jitter — which makes consecutive bursts overlap.  chip_ts.h
                 * carries the correction and the full reasoning. */
                uint32_t burst_ts = now_ts;
                uint32_t span  = (uint32_t)(produced - 1) * ls.ticks_per_sample;
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
                 * Both directions, and refuse either.  A read far BEHIND the
                 * previous burst used to be passed through as a counter
                 * reset — see chip_ts.h for why that premise does not hold
                 * inside a run, and for what it emitted on the bench.
                 */
                bool fwd_bad = !chip_ts_guard_forward_ok(
                        &ls.ts_guard, first, TS_MAX_FWD_TICKS);
                bool bwd_bad = !chip_ts_guard_backward_ok(
                        &ls.ts_guard, first, TS_MAX_JITTER_TICKS);
                if (fwd_bad || bwd_bad) {
                    first    = chip_ts_guard_next(&ls.ts_guard,
                                                  ls.ticks_per_sample);
                    burst_ts = first + span;
                    if (bwd_bad) {
                        if (++ls.ts_bwd_rejects >= ls.ts_bwd_next) {
                            LOG_W("lsm6dso: %llu post-drain timestamp "
                                  "read(s) implausibly far behind; "
                                  "extrapolating from the previous burst\n",
                                  (unsigned long long)ls.ts_bwd_rejects);
                            ls.ts_bwd_next *= 10;
                        }
                    } else if (++ls.ts_fwd_rejects >= ls.ts_fwd_next) {
                        LOG_W("lsm6dso: %llu post-drain timestamp read(s) "
                              "implausibly far ahead; extrapolating from the "
                              "previous burst\n",
                              (unsigned long long)ls.ts_fwd_rejects);
                        ls.ts_fwd_next *= 10;
                    }
                } else {
                    burst_ts += chip_ts_guard_shift(&ls.ts_guard, first,
                                                    ls.ticks_per_sample,
                                                    TS_MAX_JITTER_TICKS);
                }
                for (int i = 0; i < produced; i++) {
                    uint32_t age = (uint32_t)(produced - 1 - i) * ls.ticks_per_sample;
                    buf[i].chip_ts = burst_ts - age;
                }
                if (fts.have_word && ++ls.ts_rejects >= ls.ts_reject_next) {
                    LOG_W("lsm6dso: %llu burst(s) whose batched FIFO timestamp "
                          "failed its check — using the post-drain TIMESTAMP0 "
                          "read for chip_ts\n",
                          (unsigned long long)ls.ts_rejects);
                    ls.ts_reject_next *= 10;
                }
            } else {
                uint32_t shift = chip_ts_guard_shift(&ls.ts_guard, buf[0].chip_ts,
                                                     ls.ticks_per_sample,
                                                     TS_MAX_JITTER_TICKS);
                for (int i = 0; shift && i < produced; i++)
                    buf[i].chip_ts += shift;
            }
            chip_ts_guard_note(&ls.ts_guard, buf[produced - 1].chip_ts);
        }
    }

    *n_out = produced;
    return overflow ? 1 : 0;
}

/* ── Driver descriptors ────────────────────────────────────────────────────── */

const imu_ops_t lsm6dso_ops = {
    .name             = "lsm6dso",
    .experimental     = true,
    /* DS12140 Rev 3 §5.1.2 (protocol, mode 3), §4.4.1 Table 5 (10 MHz).
     * Same family as the ISM330DHCX: multi-byte steps the address from
     * CTRL3_C IF_INC, which lsm_init writes as part of 0x44. */
    /*
     * Mode 0, not 3. DS13012 §5.1: "The device is compatible with SPI modes 0
     * and 3." Both sample on the rising edge; they differ only in the level
     * SCLK idles at.
     *
     * Mode 0 is chosen because the IMU and the magnetometer commonly share one
     * SPI controller (spidev0.0 and 0.1 on the reference rig), the MMC5983MA
     * requires mode 0, and a controller cannot idle its clock at two levels at
     * once. Mixing them measurably breaks the bus: with this part left on
     * mode 3 beside a mode-0 magnetometer the daemon started, settled, and
     * then never produced a sample.
     */
    .bus_caps         = { .spi_capable = true, .spi_mode = 0,
                          .spi_max_hz = 10000000, .spi_inc_mask = 0 },
    .probe            = lsm_probe,
    .reset            = lsm_reset,
    .init             = lsm_init,
    .read             = lsm_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .ts_tick_ns       = 25000,   /* 32-bit counter, 25 µs/tick typical */
    .ts_tick_ns_actual = lsm_ts_tick_ns_actual,
    .supported_odr_mhz  = { 13016, 26031, 52063, 104125, 208250, 416500,
                            833000, 1666000, 3332000, 6664000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};

const imu_ops_t lsm6dsox_ops = {
    .name             = "lsm6dsox",
    .experimental     = true,
    /*
     * Mode 0, not 3. DS13012 §5.1: "The device is compatible with SPI modes 0
     * and 3." Both sample on the rising edge; they differ only in the level
     * SCLK idles at.
     *
     * Mode 0 is chosen because the IMU and the magnetometer commonly share one
     * SPI controller (spidev0.0 and 0.1 on the reference rig), the MMC5983MA
     * requires mode 0, and a controller cannot idle its clock at two levels at
     * once. Mixing them measurably breaks the bus: with this part left on
     * mode 3 beside a mode-0 magnetometer the daemon started, settled, and
     * then never produced a sample.
     */
    .bus_caps         = { .spi_capable = true, .spi_mode = 0,
                          .spi_max_hz = 10000000, .spi_inc_mask = 0 },
    .probe            = lsm_probe,
    .reset            = lsm_reset,
    .init             = lsm_init,
    .read             = lsm_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    .ts_tick_ns       = 25000,   /* 32-bit counter, 25 µs/tick typical */
    .ts_tick_ns_actual = lsm_ts_tick_ns_actual,
    .supported_odr_mhz  = { 13016, 26031, 52063, 104125, 208250, 416500,
                            833000, 1666000, 3332000, 6664000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 4000, 0 },
};
