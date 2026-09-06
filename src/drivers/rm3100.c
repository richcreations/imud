/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * rm3100.c — PNI RM3100 magnetometer driver (MagI2C ASIC)
 *
 * A different technology from every other mag in this tree.  Each axis is a
 * coil forming the inductive element of an LR relaxation oscillator; the
 * field is read out as the difference in oscillation count between the two
 * bias polarities.  That has three consequences the code below depends on:
 *
 *   - There is nothing magnetised to restore, so there is NO SET/RESET coil.
 *     set_reset is NULL and [mag] set_period_s does nothing on this part.
 *   - Resolution is set by the CYCLE COUNT — how many oscillations are
 *     counted per measurement — which trades directly against how fast the
 *     part can sample.  init() picks it from the requested ODR; see below.
 *   - Output is 24 bits, not 16 or 18.
 *
 * Runs in continuous measurement mode on all three axes; the caller wakes on
 * the DRDY edge and calls read() to pull the completed sample.
 *
 * TRANSPORT NOTE — the datasheet's "Read Address" column is a SPI command
 * byte, not an I2C sub-address.  Tables 5-2 and 5-5 list reads as 0x84, 0xA4,
 * 0xB4: that is the read bit plus the 7-bit register, which is exactly what
 * spi_burst_read() builds from BUS_SPI_READ.  The I2C transaction diagram in
 * V11.0 §5.8.4 writes the PLAIN address (0b00100100 = 0x24) before the read.
 * So bus_burst_read() is correct on both transports with no adjustment here —
 * the opposite of lis3mdl.c, which does need an explicit I2C-side bit.
 * Following Table 5-5 literally on I2C would address register 0xA4, which is
 * undefined on this part and NACKs (HSHAKE NACK0).
 *
 * AXIS FRAME — identity, deliberately.  The RM3100 is three separate coils
 * plus an ASIC rather than a monolithic package, so which coil is X is the
 * board integrator's wiring, not a die orientation.  V11.0 Figure 4-4 is
 * PNI's recommended north-east-down layout, and NED is already imud's board
 * frame (X forward, Y starboard, Z down), so a board wired per that figure
 * needs no remap.  Any other wiring is [mount] rotation_euler_deg.  This is
 * why there are no sign flips below — not an oversight.
 *
 * Register references: RM3100 & RM2100 Sensor Suite User Manual, Doc 1017252
 * V11.0 (PNI Sensor).  Gain figures: Table 3-1.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "log.h"

/* ── Register addresses (Table 5-1) ────────────────────────────────────────── */

#define REG_POLL        0x00  /* W: single-measurement trigger (unused here) */
#define REG_CMM         0x01  /* RW: continuous measurement mode control */
#define REG_CCX         0x04  /* RW: X cycle count, MSB then LSB (0x04-0x05) */
#define REG_TMRC        0x0B  /* RW: continuous-mode update rate */
#define REG_MX          0x24  /* R: results, 9 bytes X/Y/Z, 24-bit BE each */
#define REG_STATUS      0x34  /* R: bit 7 = DRDY */
#define REG_REVID       0x36  /* R: revision identification */

/* ── Chip constants ────────────────────────────────────────────────────────── */

#define STATUS_DRDY     0x80u
#define RESULTS_BYTES   9         /* 3 axes x 24 bits, MX through MZ */

/*
 * CMM: CMZ|CMY|CMX (all three axes) + START, with DRDM = 0 so DRDY rises once
 * the whole three-axis sequence is done rather than per axis.
 *
 * The value disagrees with the datasheet by one bit and is right anyway.
 * §5.2's bit table gives bit 3 as reserved-zero, which would make this 0x71,
 * but both worked examples — §5.7.2 over SPI and §5.8.3 over I2C, whose data
 * byte is spelled out as 0b01111001 — write 0x79.  The examples are what
 * silicon has been exercised against, so follow them; flagging the conflict
 * here is cheaper than rediscovering it.
 */
#define CMM_START_ALL   0x79u

/* Power-on defaults, restored by reset(): Table 5-1 and Table 5-2. */
#define CC_DEFAULT      200       /* 0x00C8 in each of the three CC registers */
#define TMRC_DEFAULT    0x96u     /* ~37 Hz */

/* ── ODR and cycle-count encoding ──────────────────────────────────────────── */

/*
 * Pick the cycle count, its gain, and the TMRC code for a requested ODR.
 *
 * Cycle count sets gain AND the rate ceiling, so the two cannot be chosen
 * independently: Table 3-1 gives a max single-axis rate per cycle count, a
 * third of which is the three-axis rate this driver runs at.  Asking TMRC for
 * more than the cycle count can deliver does not fail, it just silently
 * samples slower (§5.2.1's note), which would put the filter's dt and the
 * hardware permanently out of step.  So one function decides both, the way
 * mmc5983ma.c picks its bandwidth from the requested rate.
 *
 * The three cycle counts here are exactly the three Table 3-1 specifies gain
 * for.  Interpolating to other counts is possible but would be a
 * manufacturer's formula this driver cannot cite.
 */
static void odr_encode(int mhz, uint16_t *cc, float *gain, uint8_t *tmrc)
{
    /* Cycle count: resolution where the rate allows it. */
    if (mhz <= 150000) {
        *cc = 200; *gain = 75.0f;        /* 13 nT sensitivity, ~150 Hz 3-axis */
    } else if (mhz <= 300000) {
        *cc = 100; *gain = 38.0f;        /* 26 nT, ~283 Hz */
    } else {
        *cc =  50; *gain = 20.0f;        /* 50 nT, ~533 Hz */
    }

    /*
     * TMRC (Table 5-4).  The upper nibble is fixed at 0x9.  In milli-Hz the
     * three fractional rungs are exact — 1.2, 2.3 and 4.5 Hz — where the old
     * whole-Hz table rounded them DOWN to 1, 2 and 4 so the driver would never
     * claim to be faster than it is.  That compromise is gone.
     *
     * The sub-1 Hz codes 0x9C-0x9F are still not offered.  They ARE expressible
     * now (0.6 Hz is 600), so this is a choice rather than a limitation: no
     * fusion configuration here wants a magnetometer slower than 1 Hz, and
     * adding rungs needs a bench check that the part behaves at them.
     */
    *tmrc = (mhz <=   1200) ? 0x9Bu
          : (mhz <=   2300) ? 0x9Au
          : (mhz <=   4500) ? 0x99u
          : (mhz <=   9000) ? 0x98u
          : (mhz <=  18000) ? 0x97u
          : (mhz <=  37000) ? 0x96u
          : (mhz <=  75000) ? 0x95u
          : (mhz <= 150000) ? 0x94u
          : (mhz <= 300000) ? 0x93u
          :                   0x92u;   /* ~600 Hz */
}

/* ── Driver state ──────────────────────────────────────────────────────────── */

/*
 * Gain is not a compile-time constant on this part, so read() needs what
 * init() chose.  Seeded with the power-on cycle count's gain so a read()
 * before init() still scales by something defined rather than zero.
 */
static struct {
    float gain;   /* LSB per µT */
} s = { .gain = 75.0f };

/* ── Helpers ───────────────────────────────────────────────────────────────── */

/* Write one 16-bit cycle count, MSB first, to a CC register pair. */
static int write_cc(const imud_bus_t *bus, uint8_t reg, uint16_t cc)
{
    if (bus_reg_write(bus, reg,              (uint8_t)(cc >> 8)) < 0) return -1;
    if (bus_reg_write(bus, (uint8_t)(reg + 1), (uint8_t)(cc & 0xFF)) < 0) return -1;
    return 0;
}

/* All three axes to the same cycle count (0x04-0x09). */
static int write_cc_all(const imud_bus_t *bus, uint16_t cc)
{
    for (uint8_t axis = 0; axis < 3; axis++)
        if (write_cc(bus, (uint8_t)(REG_CCX + axis * 2), cc) < 0) return -1;
    return 0;
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

/*
 * rm_probe — identify the part without an identity register.
 *
 * The RM3100 has no WHO_AM_I, and REVID (0x36) is read-only with no value
 * given in either PNI manual, so it cannot be compared against anything.  It
 * still rules out the two ways a missing device reads: 0xFF from a floating
 * SDA, 0x00 from a line held low.
 *
 * What actually establishes presence is the write/read-back below.  No
 * floating bus returns what was just written to a 16-bit register pair.  The
 * value used is the part's own power-on cycle count, so probe() leaves no
 * configuration behind at startup — and CCX is put back as it was found, so a
 * probe of a part init() has already configured does not overwrite the cycle
 * count it chose while the driver goes on using the gain that went with it.
 */
static int rm_probe(const imud_bus_t *bus)
{
    uint8_t rev;
    if (bus_reg_read(bus, REG_REVID, &rev) < 0) {
        LOG_E("rm3100: REVID read failed: %s\n", strerror(errno));
        return -1;
    }
    if (rev == 0x00 || rev == 0xFF) {
        LOG_E("rm3100: REVID = 0x%02X — no device responding\n", rev);
        return -1;
    }

    uint8_t was[2];
    if (bus_burst_read(bus, REG_CCX, was, 2) < 0) {
        LOG_E("rm3100: cycle-count read failed: %s\n", strerror(errno));
        return -1;
    }

    if (write_cc(bus, REG_CCX, CC_DEFAULT) < 0) {
        LOG_E("rm3100: cycle-count write failed: %s\n", strerror(errno));
        return -1;
    }

    uint8_t back[2];
    if (bus_burst_read(bus, REG_CCX, back, 2) < 0) {
        LOG_E("rm3100: cycle-count read-back failed: %s\n", strerror(errno));
        return -1;
    }
    uint16_t got = (uint16_t)(((uint16_t)back[0] << 8) | back[1]);
    if (got != CC_DEFAULT) {
        LOG_E("rm3100: cycle-count read-back = %u, expected %u\n",
                got, (unsigned)CC_DEFAULT);
        return -1;
    }

    uint16_t prev = (uint16_t)(((uint16_t)was[0] << 8) | was[1]);
    if (prev != CC_DEFAULT && write_cc(bus, REG_CCX, prev) < 0) {
        LOG_E("rm3100: cycle-count restore failed: %s\n", strerror(errno));
        return -1;
    }

    LOG_I("rm3100: REVID 0x%02X\n", rev);
    return 0;
}

/*
 * rm_reset — return the part to its power-on state.
 *
 * There is no software-reset register on the RM3100, so the usual
 * "write the reset bit, wait for it to self-clear" does not apply and its
 * absence here is deliberate.  Equivalent state by hand: stop continuous
 * mode, restore the default cycle count and update rate, then clear any
 * pending DRDY by draining the results registers (reading them is what
 * clears it — HSHAKE DRC1, set by default).
 */
static int rm_reset(const imud_bus_t *bus)
{
    if (bus_reg_write(bus, REG_CMM, 0x00) < 0) return -1;
    if (write_cc_all(bus, CC_DEFAULT) < 0) return -1;
    if (bus_reg_write(bus, REG_TMRC, TMRC_DEFAULT) < 0) return -1;

    uint8_t drain[RESULTS_BYTES];
    if (bus_burst_read(bus, REG_MX, drain, RESULTS_BYTES) < 0) return -1;

    s.gain = 75.0f;    /* matches CC_DEFAULT, per Table 3-1 */
    usleep(1000);      /* let the stopped sequence settle before init() */
    return 0;
}

static int rm_init(const imud_bus_t *bus, const mag_cfg_t *cfg)
{
    uint16_t cc;
    uint8_t  tmrc;
    float    gain;
    odr_encode(cfg->odr_mhz, &cc, &gain, &tmrc);

    /* Cycle counts first: they bound what TMRC can actually deliver. */
    if (write_cc_all(bus, cc) < 0) return -1;
    if (bus_reg_write(bus, REG_TMRC, tmrc) < 0) return -1;

    /* Start continuous measurement on all three axes. */
    if (bus_reg_write(bus, REG_CMM, CMM_START_ALL) < 0) return -1;

    s.gain = gain;
    return 0;
}

/*
 * rm_read — read one completed three-axis sample.
 *
 * Called by the mag_reader thread after a DRDY rising edge.
 *
 * Returns:
 *   0  — sample written to *out, out->valid = true
 *   1  — measurement not complete yet (DRDY clear); wait for the next edge
 *  -1  — bus error
 */
static int rm_read(const imud_bus_t *bus, mag_sample_t *out)
{
    uint8_t status;
    if (bus_reg_read(bus, REG_STATUS, &status) < 0) return -1;
    if (!(status & STATUS_DRDY)) return 1;

    /*
     * Nine bytes: X, Y, Z, each 24-bit big-endian two's complement (§5.5).
     * The read itself clears DRDY, so there is no status write-back to do.
     */
    uint8_t raw[RESULTS_BYTES];
    if (bus_burst_read(bus, REG_MX, raw, RESULTS_BYTES) < 0) return -1;

    /* Gain is LSB/µT (Table 3-1), so divide.  Identity axis map — see the
     * file header for why. */
    out->field[0] = (float)reg_s24be(&raw[0]) / s.gain;
    out->field[1] = (float)reg_s24be(&raw[3]) / s.gain;
    out->field[2] = (float)reg_s24be(&raw[6]) / s.gain;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    out->valid   = true;

    return 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const mag_ops_t rm3100_ops = {
    .name             = "rm3100",
    .experimental     = true,
    /* V11.0 §4.4: SCLK 1 MHz or less, and the part accepts CPOL = CPHA = 0 or
     * CPOL = CPHA = 1 — mode 0 or mode 3.  Mode 3 to match every other part
     * here.  The register pointer auto-increments on a multi-byte transfer
     * (§5.7.1), so no explicit increment bit.  Selecting SPI is a board-level
     * decision as much as a config one: I2CEN (pin 22) must be tied LOW. */
    .bus_caps         = { .spi_capable = true, .spi_mode = 3,
                          .spi_max_hz = 1000000, .spi_inc_mask = 0 },
    .probe            = rm_probe,
    .reset            = rm_reset,
    .init             = rm_init,
    .read             = rm_read,
    /* Magneto-inductive: no AMR elements to restore, so no degaussing coil.
     * This is a property of the technology, not a driver gap. */
    .set_reset        = NULL,
    .has_interrupt    = true,
    .has_set_reset    = false,
    /*
     * Table 5-4's update rates, rounded DOWN to integers so the advertised
     * rate is never faster than the silicon (1.2 -> 1, 2.3 -> 2, 4.5 -> 4);
     * lis3mdl.c drops its 0.625 Hz rung for the same reason.  The four
     * sub-1 Hz codes are omitted entirely — an int table cannot hold them.
     *
     * The top two rungs are reachable only at reduced cycle count, which
     * odr_encode() selects: 300 Hz needs CC = 100 and 600 Hz needs CC = 50,
     * costing resolution (Table 3-1: 26 nT and 50 nT against 13 nT at the
     * default CC = 200).  They are offered because the part can do them, not
     * because a heading reference needs them.
     */
    .supported_odr_mhz = { 1200, 2300, 4500, 9000, 18000, 37000, 75000,
                           150000, 300000, 600000, 0 },
};
