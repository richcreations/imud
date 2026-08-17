
/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mmc5983ma.c — MMC5983MA magnetometer driver
 *
 * Implements mag_ops_t for the MMC5983MA via Linux I2C_RDWR ioctl.
 * Runs in continuous measurement mode; caller wakes on GPIO27 INT edge,
 * then calls read() to pull the completed 18-bit sample.
 *
 * Register references: MMC5983MA datasheet Rev A (MEMSIC, 2019-04-03).
 * Sensitivity:         datasheet §Specifications, 16384 counts/G at 18-bit.
 */

#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "log.h"

/* ── Register addresses (datasheet §Register Map) ──────────────────────────── */

#define REG_XOUT0       0x00  /* Xout[17:10] */
#define REG_XOUT1       0x01  /* Xout[9:2] */
#define REG_YOUT0       0x02  /* Yout[17:10] */
#define REG_YOUT1       0x03  /* Yout[9:2] */
#define REG_ZOUT0       0x04  /* Zout[17:10] */
#define REG_ZOUT1       0x05  /* Zout[9:2] */
#define REG_XYZOUT2     0x06  /* bits[7:6]=Xout[1:0], [5:4]=Yout[1:0], [3:2]=Zout[1:0] */
#define REG_TOUT        0x07  /* temperature, 0.8 °C/LSB, 0x00 = -75 °C */
#define REG_STATUS      0x08  /* R/W: Meas_M_Done[0], Meas_T_Done[1]; write 1 to clear */
#define REG_CTRL0       0x09  /* W: TM_M[0] TM_T[1] INT_en[2] Set[3] Reset[4] AutoSR[5] */
#define REG_CTRL1       0x0A  /* W: BW[1:0] X_inhibit[2] YZ_inhibit[4:3] SW_RST[7] */
#define REG_CTRL2       0x0B  /* W: CM_Freq[2:0] Cmm_en[3] Prd_set[6:4] En_prd_set[7] */
#define REG_PRODUCT_ID  0x2F  /* R: fixed 0x30 */

/* ── CTRL0 bit definitions ─────────────────────────────────────────────────── */

#define CTRL0_TM_M      0x01  /* trigger single magnetic measurement (self-clears) */
#define CTRL0_INT_EN    0x04  /* enable measurement-done interrupt on INT pin */
#define CTRL0_SET       0x08  /* issue SET pulse ~500 ns (self-clears) */
#define CTRL0_RESET     0x10  /* issue RESET pulse ~500 ns (self-clears) */

/* ── Chip constants ────────────────────────────────────────────────────────── */

#define PRODUCT_ID_VALUE  0x30u
#define NULL_FIELD        131072u   /* 2^17 — unsigned raw output for zero field */
#define STATUS_M_DONE     0x01u

/*
 * ── A write to CTRL0 also lands in CTRL1 ───────────────────────────────────
 *
 * Measured on an MMC5983MA (SparkFun 9DoF SEN-19895) over SPI on a Pi 5,
 * 2026-08-16. Writing CTRL0 applies the same byte to CTRL1 as well:
 *
 *   CTRL0 = 0x04 (INT_en)   → CTRL1 = 0x04 → X-inhibit; X stops measuring
 *   CTRL0 = 0x18 (Set|Reset)→ CTRL1 = 0x18 → YZ-inhibit; Y and Z stop
 *   CTRL0 = 0x80            → CTRL1 = 0x80 → SW_RST; continuous mode stops
 *
 * CTRL0 still receives the write — the SET and RESET pulses do fire, and the
 * field term inverts between them. It is both registers, not the wrong one.
 *
 * The effect is specific to CTRL0: a CTRL1 write does not disturb CTRL2 (which
 * would stop continuous mode) and a CTRL2 write does not disturb CTRL3 (whose
 * self-test coil moves the mean by hundreds of µT and is unmistakable). It is
 * also not a bus-timing artefact — identical at 10 MHz, 1 MHz and 100 kHz.
 *
 * The consequence for this driver: X-inhibit is set by init()'s own last
 * write, so the X axis silently stops measuring while Y and Z look fine. That
 * is a dead magnetometer axis presented as a working one — heading comes from
 * atan2(-my, mx), so it takes the heading with it.
 *
 * The fix is ordering: CTRL1 is written LAST, after every CTRL0 write, which
 * puts the intended value back. mmc_degauss() has to do the same, so the
 * programmed CTRL1 is remembered here. Written by init() (from main at startup,
 * or from the mag_reader thread on error recovery) and read by degauss() on the
 * mag_reader thread, so it is _Atomic — see CLAUDE.md's concurrency rule.
 */
static _Atomic uint8_t g_ctrl1 = 0;

/* ── ODR encoding ──────────────────────────────────────────────────────────── */

/*
 * Map requested ODR to BW bits (CTRL1[1:0]) and CM_Freq bits (CTRL2[2:0]).
 *
 * BW selection drives measurement duration and noise floor (Rev A pp.4, 15):
 *   BW=00  8 ms  0.4 mG RMS  max ODR  50 Hz
 *   BW=01  4 ms  0.6 mG RMS  max ODR 100 Hz
 *   BW=10  2 ms  0.8 mG RMS  max ODR 225 Hz
 *   BW=11  0.5ms 1.2 mG RMS  max ODR 580/1000 Hz
 *
 * We pick the minimum BW that supports the requested ODR (lowest noise).
 *
 * ── Only four of the seven CM_Freq codes work ──────────────────────────────
 *
 * The datasheet lists seven continuous rates. On the reference part three of
 * them are not honoured: the part enters continuous mode and then free-runs at
 * whatever its bandwidth allows, ignoring CM_Freq entirely. Every CM_Freq code
 * against every bandwidth, counted as distinct output images per second with
 * no STATUS write involved (SparkFun 9DoF SEN-19895, Pi 5, 2026-08-16):
 *
 *   CM_Freq          BW=00   BW=01   BW=10   BW=11     nominal
 *   001               *1       1       1       1         1 Hz
 *   010              130     256     497    1689        10 Hz   never taken
 *   011              *22      21      21      21        20 Hz
 *   100              130     256     497    1689        50 Hz   never taken
 *   101              106     *105    105     105       100 Hz
 *   110              130     256     497    1690       200 Hz   never taken
 *   111              120     241     402   *1206      1000 Hz
 *
 * The three failing rows are exactly each bandwidth's conversion ceiling
 * (8 ms → 125, 4 ms → 250, 2 ms → 500, 0.5 ms → 2000), so bandwidth is not the
 * constraint — the codes simply do not take. Nor is it how CTRL2 is written:
 * splitting the frequency and the enable into separate writes, with or without
 * a delay between them, and stopping the mode first, all give the identical
 * table. 111 needing BW=11 is real and is documented on p.15; the rest is not
 * documented anywhere.
 *
 * So the driver advertises only what the part delivers — see supported_odr_hz
 * below, which snap_odr_up() rounds a request up to. Advertising the other
 * three meant `[mag] odr_hz = 50` silently ran at ~130 Hz while fusion sized
 * the magnetometer's noise variance for 50.
 *
 * This is one die. If a part is found that honours the even codes, this and
 * supported_odr_hz are what to revisit; imud-imutest's mag.rate is the check
 * that shows it.
 */
static void odr_encode(int hz, uint8_t *bw_out, uint8_t *cmfreq_out)
{
    if (hz <=    1) { *bw_out = 0x0; *cmfreq_out = 0x1; return; }
    if (hz <=   20) { *bw_out = 0x0; *cmfreq_out = 0x3; return; }
    if (hz <=  100) { *bw_out = 0x1; *cmfreq_out = 0x5; return; }
    /* 1000 Hz */     *bw_out = 0x3; *cmfreq_out = 0x7;   /* BW=11 required */
}

/* ── Driver operations ─────────────────────────────────────────────────────── */

static int mmc_probe(const imud_bus_t *bus)
{
    uint8_t id;
    if (bus_reg_read(bus, REG_PRODUCT_ID, &id) < 0) {
        LOG_E("mmc5983ma: product ID read failed: %s\n", strerror(errno));
        return -1;
    }
    if (id != PRODUCT_ID_VALUE) {
        LOG_E("mmc5983ma: product ID = 0x%02X, expected 0x%02X\n",
                id, PRODUCT_ID_VALUE);
        return -1;
    }
    return 0;
}

static int mmc_reset(const imud_bus_t *bus)
{
    /* SW_RST is bit 7 of CTRL1; self-clears. Power-on time: 10 ms. */
    if (bus_reg_write(bus, REG_CTRL1, 0x80) < 0) return -1;
    usleep(10000);
    return 0;
}

/*
 * mmc_init — configure the part, then start it.
 *
 * Two properties of this sequence are load-bearing, and both were established
 * on hardware (SparkFun 9DoF SEN-19895, Pi 5, 2026-08-16).
 *
 * 1. CTRL1 is written after every CTRL0 write, or the alias documented above
 *    leaves X-inhibit set and the X axis silently stops measuring.
 *
 * 2. CTRL2 is written LAST, and is followed by a quiet period.
 *
 *    Order: BW is an input to what CM_Freq means. Rev A p.15 introduces the
 *    CM_Freq table with "the frequency is based on the assumption that
 *    BW[1:0] = 00", and two rows carry a prerequisite in the row itself — 110
 *    needs BW=01, 111 needs BW=11. Enabling continuous mode before CTRL1 runs
 *    the part at the reset default BW=00, rated for 50 Hz (p.4), against
 *    whatever CM_Freq was just programmed.
 *
 *    That is not theoretical. At odr_hz = 1000 (CM_Freq=111, which needs
 *    BW=11) the stock order intermittently fails to start continuous mode at
 *    all — the first read waits 500 ms for Meas_M_Done and gives up:
 *
 *      CTRL2 before CTRL1   failed to start 7 of 20 runs
 *      CTRL2 after  CTRL1   failed to start 0 of 20 runs
 *
 *    (12 of 33 vs 0 of 33 across every run of the session.) In the daemon that
 *    surfaces as init() succeeding — every write is ACKed — followed by a mag
 *    that never produces a sample.
 *
 *    Quiet: writing anything within ~40 ms of enabling continuous mode leaves
 *    the bridge saturated — every axis reads a few hundred µT and stays there
 *    for the rest of the run, and CM_Freq does not take, so the measured rate
 *    runs to ~256 Hz against a configured 100. Five runs per point:
 *
 *      post-CTRL2 quiet    0    10   20   30   40   50   60   80  100 ms
 *      healthy runs       0/5  0/5  0/5  4/5  5/5  5/5  5/5  5/5  5/5
 *
 *    Both this order and the stock one give that same table, so writing CTRL2
 *    last does NOT shorten the wait — it was tried for exactly that and did
 *    not deliver. 100 ms is ~2.5x the boundary.
 *
 *    It is this window specifically, not write spacing in general: a quiet
 *    period after CTRL0 or CTRL1 alone does not help, and writes issued once
 *    the window has passed are harmless. It is not the bus — the same
 *    threshold appears at 10 MHz, 1 MHz and 100 kHz — and it is not warm-up,
 *    because delaying the first measurement by up to 2 s while writing
 *    back-to-back does not help at all.
 *
 *    The over-rate in the order argument is NOT this mechanism: more time at
 *    BW=00 is measurably healthier, which runs backwards. It is a spec
 *    violation worth not committing, not an explanation.
 *
 * Nothing may follow the CTRL2 write inside that window, here or in the
 * caller — mmc_read's per-sample STATUS write lands in it if the mag reader
 * starts promptly, which is why the wait is inside init rather than left to
 * the caller.
 *
 * No datasheet number backs the ~40 ms. Rev A gives a 10 ms power-on time for
 * SW_RST and says nothing about entering continuous mode.
 */
static int mmc_init(const imud_bus_t *bus, const mag_cfg_t *cfg)
{
    uint8_t bw, cmfreq;
    odr_encode(cfg->odr_hz, &bw, &cmfreq);

    /* Clear CTRL0: disable Auto_SR_en; we do periodic manual SET instead. */
    if (bus_reg_write(bus, REG_CTRL0, 0x00) < 0) return -1;

    /* Enable INT pin on measurement completion. */
    if (bus_reg_write(bus, REG_CTRL0, CTRL0_INT_EN) < 0) return -1;

    /*
     * Measurement bandwidth — after every CTRL0 write, because a CTRL0 write
     * also lands in CTRL1 and would otherwise leave X-inhibit set. See the
     * g_ctrl1 comment near the top of this file.
     */
    if (bus_reg_write(bus, REG_CTRL1, bw) < 0) return -1;
    atomic_store(&g_ctrl1, bw);

    /* Start continuous mode: Cmm_en=1 (bit 3) | CM_Freq. LAST — nothing may
     * follow it, here or in the caller, for the settle below. */
    if (bus_reg_write(bus, REG_CTRL2, (uint8_t)(0x08u | cmfreq)) < 0) return -1;

    /* Then leave the part alone. See the threshold table above: below 30 ms the
     * bridge saturates every time. 100 ms is ~2.5x the measured boundary. */
    usleep(100000);

    return 0;
}

/*
 * mmc_read — read one completed 18-bit sample.
 *
 * Called by mag_reader thread after GPIO27 rising edge (INT active-high).
 *
 * Returns:
 *   0  — sample written to *out, out->valid = true
 *   1  — measurement not complete yet (Meas_M_Done not set); caller should
 *         wait for next interrupt rather than spinning
 *  -1  — I2C error
 */
static int mmc_read(const imud_bus_t *bus, mag_sample_t *out)
{
    /* Confirm data is ready. Should always be true after the GPIO edge, but
     * a spurious wakeup or race with the clear path can violate that. */
    uint8_t status;
    if (bus_reg_read(bus, REG_STATUS, &status) < 0) return -1;
    if (!(status & STATUS_M_DONE)) return 1;

    /* Burst-read Xout0…XYZout2 (registers 0x00–0x06, 7 bytes). */
    uint8_t raw[7];
    if (bus_burst_read(bus, REG_XOUT0, raw, 7) < 0) return -1;

    /* Clear the measurement-done interrupt so the next rising edge is clean. */
    if (bus_reg_write(bus, REG_STATUS, STATUS_M_DONE) < 0) return -1;

    /*
     * Reconstruct 18-bit unsigned values from split registers.
     *
     * Xout[17:10] = XOUT0[7:0]
     * Xout[9:2]   = XOUT1[7:0]
     * Xout[1:0]   = XYZOUT2[7:6]
     *
     * Same pattern for Y (XYZOUT2[5:4]) and Z (XYZOUT2[3:2]).
     */
    uint32_t rx = ((uint32_t)raw[0] << 10)
                | ((uint32_t)raw[1] <<  2)
                | ((raw[6] >> 6) & 0x03u);
    uint32_t ry = ((uint32_t)raw[2] << 10)
                | ((uint32_t)raw[3] <<  2)
                | ((raw[6] >> 4) & 0x03u);
    uint32_t rz = ((uint32_t)raw[4] << 10)
                | ((uint32_t)raw[5] <<  2)
                | ((raw[6] >> 2) & 0x03u);

    /*
     * Convert to µT.
     *   Null field = 131072 counts (2^17, datasheet §Specifications)
     *   Sensitivity = 16384 counts/G; 1 G = 100 µT
     *   → scale = 100.0 / 16384.0 µT/count ≈ 0.006104 µT/count
     */
    const float scale = 100.0f / 16384.0f;
    /*
     * Remap to NED-compatible board frame (X=bow, Y=starboard, Z=down),
     * matching the ISM330DHCX driver convention.
     *
     * On SparkFun 9DoF SEN-19895 the MMC5983MA is mounted with:
     *   X: same orientation as ISM X — no flip.
     *   Y: same orientation as ISM chip-native Y (port) — flip to starboard.
     *   Z: physically opposite to ISM chip-native Z (SparkFun "MAG -Z" dot).
     *      Because the ISM driver already flips its own Z from up→down, raw
     *      MMC Z is already in the down direction — no additional sign needed.
     */
    out->field[0] =  (float)((int32_t)rx - (int32_t)NULL_FIELD) * scale;
    out->field[1] = -(float)((int32_t)ry - (int32_t)NULL_FIELD) * scale;
    out->field[2] =  (float)((int32_t)rz - (int32_t)NULL_FIELD) * scale;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    out->valid   = true;

    return 0;
}

/*
 * mmc_degauss — issue one degauss pulse in the requested direction.
 *
 * A strong external field (motors, alternator, steel structure) can residually
 * magnetise the AMR elements. The pulse applies a large current for ~500 ns,
 * forcing the internal magnetisation to a known direction.
 *
 * SET and RESET drive it opposite ways, which flips the sign of the field term
 * in a subsequent reading while leaving the bridge's own offset alone — see
 * mag_ops_t::degauss in include/drivers.h for what a caller does with that.
 *
 * INT_EN is re-asserted with the pulse bit, not left out of the write. CTRL0's
 * Set and Reset bits self-clear, but INT_en does NOT — it is a persistent mode
 * bit (datasheet Rev A §Register Map, and the CTRL0 comment above). Writing the
 * pulse bit alone therefore also switches off the measurement-done interrupt
 * that mmc_init() enabled, which is what this driver used to do: after the
 * first periodic SET the INT line went quiet for the rest of the run and
 * mag_reader silently fell back to its 20 ms poll.
 *
 * The CTRL1 write that follows is not redundant: this CTRL0 write also lands in
 * CTRL1, where INT_en's bit 2 reads as X-inhibit and stops the X axis dead.
 * Restoring the programmed value is what keeps a degauss from costing an axis —
 * and the periodic SET runs every 5 s, so without it X measures for exactly one
 * degauss interval after startup and never again.
 *
 * The driver sleeps 1 ms after the pulse for bridge settling before the next
 * measurement is accepted.
 */
static int mmc_degauss(const imud_bus_t *bus, mag_degauss_t dir)
{
    uint8_t pulse = (dir == MAG_DEGAUSS_RESET) ? CTRL0_RESET : CTRL0_SET;
    if (bus_reg_write(bus, REG_CTRL0, (uint8_t)(CTRL0_INT_EN | pulse)) < 0)
        return -1;
    if (bus_reg_write(bus, REG_CTRL1, atomic_load(&g_ctrl1)) < 0)
        return -1;
    usleep(1000);  /* 1 ms settling before next read */
    return 0;
}

/*
 * mmc_set_reset — the production degauss, invoked by the mag_reader thread on
 * the configured interval (default 5 s). SET is the direction that leaves the
 * part measuring the field in the polarity the read path expects.
 */
static int mmc_set_reset(const imud_bus_t *bus)
{
    return mmc_degauss(bus, MAG_DEGAUSS_SET);
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const mag_ops_t mmc5983ma_ops = {
    .name             = "mmc5983ma",
    .experimental     = false,
    /* Rev A pp.4-7: 10 MHz, SCK idle high and captured on the rising edge —
     * mode 3 — and multi-byte transfers add 8-clock blocks, so no
     * auto-increment bit. The command byte's address field is only six bits
     * wide (bit 6 is don't-care); every register here is <= 0x2F, so reg|0x80
     * addresses them all correctly. */
    .bus_caps         = { .spi_capable = true, .spi_mode = 3,
                          .spi_max_hz = 10000000, .spi_inc_mask = 0 },
    .probe            = mmc_probe,
    .reset            = mmc_reset,
    .init             = mmc_init,
    .read             = mmc_read,
    .set_reset        = mmc_set_reset,
    .degauss          = mmc_degauss,
    .has_interrupt    = true,
    .has_set_reset    = true,
    /* Four of the datasheet's seven; the other three are not honoured by the
     * part. Measured table and method are above odr_encode(). */
    .supported_odr_hz = { 1, 20, 100, 1000, 0 },
};
