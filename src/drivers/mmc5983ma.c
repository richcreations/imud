
/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mmc5983ma.c — MMC5983MA magnetometer driver
 *
 * Implements mag_ops_t for the MMC5983MA via Linux I2C_RDWR ioctl.
 * Runs in continuous measurement mode; caller wakes on the INT edge,
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
 * Edge-driven reads skip the Meas_M_Done status gate.
 *
 * INT is latched and is re-armed only by writing 1 to Meas_M_Done, which also
 * clears the status bit read() would gate on (Rev A p.13).  In continuous mode
 * the output registers always hold the last complete conversion, so the edge
 * alone signals data-ready; gating on the status bit as well costs one wasted
 * edge and one full timeout per sample.
 *
 * g_prev_raw is the staleness guard for that path, holding the last 7 output
 * bytes packed into one word.  An exact match on all three axes means the
 * conversion has not advanced, so a dead INT line degrades to the reader's
 * polling fallback instead of re-feeding the filter duplicates it cannot
 * distinguish from real data.  Both are written by init() from either thread,
 * hence _Atomic.
 */
static _Atomic bool     g_int_driven = false;
static _Atomic uint64_t g_prev_raw   = 0;   /* 7 output bytes, packed; 0 = none */

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
 * Picks the minimum BW that supports the requested ODR, for the lowest noise.
 *
 * The delivered rates sit a few percent off nominal — the part's own
 * oscillator — so supported_odr_mhz is the ladder an operator may REQUEST and
 * mmc_actual_odr_mhz() below is what the silicon DELIVERS.
 */
static void odr_encode(int mhz, uint8_t *bw_out, uint8_t *cmfreq_out)
{
    /* Every code the datasheet lists.  BW is the lowest whose Max Output Data
     * Rate (Rev A p.4: 50/100/225/580) covers the rate, except the two the
     * CM_Freq table pairs explicitly: 110 with BW=01, 111 with BW=11. */
    if (mhz <=   1000) { *bw_out = 0x0; *cmfreq_out = 0x1; return; }   /*    1 */
    if (mhz <=  11000) { *bw_out = 0x0; *cmfreq_out = 0x2; return; }   /*   10 */
    if (mhz <=  21000) { *bw_out = 0x0; *cmfreq_out = 0x3; return; }   /*   20 */
    if (mhz <=  53000) { *bw_out = 0x0; *cmfreq_out = 0x4; return; }   /*   50 */
    if (mhz <= 106000) { *bw_out = 0x1; *cmfreq_out = 0x5; return; }   /*  100 */
    if (mhz <= 211000) { *bw_out = 0x1; *cmfreq_out = 0x6; return; }   /*  200 */
    /* 1000 Hz */     *bw_out = 0x3; *cmfreq_out = 0x7;             /* 1000 */
}

/*
 * What the codes above really deliver.
 *
 * Snaps up over the delivered rates rather than mapping thresholds to them,
 * because imu.c passes the resolved rate back into the driver, so
 * actual(actual(x)) must equal actual(x).  A threshold form gets that wrong in
 * a way nothing else would catch — 105 is "<= 100 ? no" and would resolve a
 * second time to 1206.
 *
 * One entry here per honoured branch in odr_encode(); the two move together.
 */
static int mmc_actual_odr_mhz(int req_mhz)
{
    /*
     * Milli-Hz, but NOT to three significant figures more than the
     * measurement supports: 100 Hz read 105.3, 105.4, 105.6 and 105.7 across
     * one day on the reference die, a 0.4% spread that is drift rather than a
     * figure.  These stay the rounded whole-Hz measurements scaled by 1000,
     * and the honest caveat is that they are one die at one temperature.
     */
    static const int delivered[] = { 1000, 11000, 21000, 53000, 106000,
                                     211000, 1205000, 0 };

    for (int i = 0; delivered[i]; i++)
        if (req_mhz <= delivered[i]) return delivered[i];
    return 1205000;                      /* clamp to the fastest */
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
 * CTRL2 is written LAST and is followed by a 100 ms quiet period.  Both are
 * load-bearing.
 *
 * Order: BW is an input to what CM_Freq means.  Rev A p.15 introduces the
 * CM_Freq table with "the frequency is based on the assumption that
 * BW[1:0] = 00", and codes 110 and 111 name a required BW in the row itself.
 * Enabling continuous mode before CTRL1 runs the part at the reset default
 * BW=00, rated for 50 Hz (p.4), against whatever CM_Freq was just programmed;
 * at odr_hz = 1000 that intermittently fails to enter continuous mode at all,
 * which surfaces as every write ACKing and no sample ever arriving.
 *
 * Quiet: writing anything within ~40 ms of enabling continuous mode leaves the
 * bridge saturated — every axis stuck at a few hundred µT for the rest of the
 * run — and CM_Freq does not take.  Nothing may follow the CTRL2 write inside
 * that window, here or in the caller: mmc_read's per-sample STATUS write lands
 * in it if the mag reader starts promptly, which is why the wait is here
 * rather than left to the caller.  No datasheet number backs the 40 ms; Rev A
 * gives a 10 ms power-on time for SW_RST and says nothing about entering
 * continuous mode, so 100 ms is ~2.5x the measured boundary.
 */
static int mmc_init(const imud_bus_t *bus, const mag_cfg_t *cfg)
{
    uint8_t bw, cmfreq;
    odr_encode(cfg->odr_mhz, &bw, &cmfreq);

    /* How this caller waits decides what read() is able to check — and the
     * staleness guard must not carry a sample across a reconfigure. */
    atomic_store(&g_int_driven, cfg->int_driven);
    atomic_store(&g_prev_raw, 0);

    /* Clear CTRL0: disable Auto_SR_en; we do periodic manual SET instead. */
    if (bus_reg_write(bus, REG_CTRL0, 0x00) < 0) return -1;

    /* Enable INT pin on measurement completion. */
    if (bus_reg_write(bus, REG_CTRL0, CTRL0_INT_EN) < 0) return -1;

    /* Measurement bandwidth, before continuous mode starts. */
    if (bus_reg_write(bus, REG_CTRL1, bw) < 0) return -1;

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
 * Called by the mag_reader thread after a rising edge on the mag's INT line
 * (active-high; the line is `[mag] int_gpio`, 27 by default), or by a polling
 * caller such as imud-imutest and imud-cal.  g_int_driven selects which of the
 * two data-ready tests below applies.
 *
 * Returns:
 *   0  — sample written to *out, out->valid = true
 *   1  — no new measurement: Meas_M_Done not set (polled), or the output
 *         registers have not advanced since the last read (edge-driven).
 *         Caller should wait rather than spinning
 *  -1  — bus error
 */
static int mmc_read(const imud_bus_t *bus, mag_sample_t *out)
{
    const bool int_driven = atomic_load(&g_int_driven);

    /*
     * Polled: the status bit is the only signal available, and it works — for a
     * caller that polls, which is what keeps it re-asserting.
     */
    if (!int_driven) {
        uint8_t status;
        if (bus_reg_read(bus, REG_STATUS, &status) < 0) return -1;
        if (!(status & STATUS_M_DONE)) return 1;
    }

    /* Burst-read Xout0…XYZout2 (registers 0x00–0x06, 7 bytes). */
    uint8_t raw[7];
    if (bus_burst_read(bus, REG_XOUT0, raw, 7) < 0) return -1;

    /* Clear the measurement-done interrupt so the next rising edge is clean.
     * Required on BOTH paths: this write is what re-arms the latched INT. */
    if (bus_reg_write(bus, REG_STATUS, STATUS_M_DONE) < 0) return -1;

    /*
     * Edge-driven staleness guard. The edge said a conversion landed; if the
     * output registers say otherwise, believe the registers. This is what stops
     * a failed INT line from turning the reader's timeout into a stream of one
     * duplicated sample. The packing is 7 bytes into one word so the compare and
     * the store are each single and tear-free.
     */
    if (int_driven) {
        uint64_t packed = 0;
        for (int i = 0; i < 7; i++)
            packed |= (uint64_t)raw[i] << (8 * i);
        packed |= (uint64_t)1u << 56;          /* tag: distinguishes "all zero" */
        if (atomic_exchange(&g_prev_raw, packed) == packed) return 1;
    }

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
 * The pulse drives a large current for ~500 ns, forcing the AMR elements'
 * magnetisation to a known direction after a strong external field (motors,
 * alternator, steel structure) has residually magnetised them.  SET and RESET
 * drive it opposite ways, which flips the sign of the field term in a
 * subsequent reading while leaving the bridge's own offset alone — see
 * mag_ops_t::degauss in include/drivers.h for what a caller does with that.
 *
 * INT_EN is re-asserted with the pulse bit rather than left out of the write:
 * CTRL0's Set and Reset bits self-clear but INT_en does not, so writing the
 * pulse alone would also switch off the measurement-done interrupt
 * mmc_init() enabled, dropping mag_reader to its polling fallback for the
 * rest of the run.
 *
 * Sleeps 1 ms after the pulse for bridge settling before the next read.
 */
static int mmc_degauss(const imud_bus_t *bus, mag_degauss_t dir)
{
    uint8_t pulse = (dir == MAG_DEGAUSS_RESET) ? CTRL0_RESET : CTRL0_SET;
    if (bus_reg_write(bus, REG_CTRL0, (uint8_t)(CTRL0_INT_EN | pulse)) < 0)
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
    /*
     * SPI mode 0 at 10 MHz, the datasheet maximum (Rev A p.4, fc(SCK)).
     *
     * A low explicit [imu] spi_speed_hz stops this part measuring when the IMU
     * shares the SPI controller.  This part selects its interface from the CS
     * level and has no I2C_disable bit, so while its CS is high it sits on the
     * IMU's clock and data lines as an I2C slave; below about 2.3 MHz the run
     * of zero bytes spidev sends as a burst read's data phase passes its input
     * filter and clears CTRL2's Cmm_en.  The part still answers SPI, still
     * reads its product id and still converts on a single-shot trigger, so
     * only the sample rate shows it, and re-running init() does not hold
     * because the next burst clears it again.
     *
     * spi_speed_hz = 0 gives each part its declared maximum and is unaffected;
     * config.c warns when a slower IMU clock is configured alongside a
     * shared-bus magnetometer.
     */
    .bus_caps         = { .spi_capable = true, .spi_mode = 0,
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
    /* The datasheet ladder: what may be REQUESTED. What each one actually
     * delivers is mmc_actual_odr_mhz() — see the table above odr_encode(). */
    .supported_odr_mhz = { 1000, 10000, 20000, 50000, 100000, 200000,
                           1000000, 0 },
    .actual_odr_mhz   = mmc_actual_odr_mhz,
};
