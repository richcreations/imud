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
 * Register references: ICM-42688-P datasheet DS-000347 Rev 1.2, except where a
 * comment cites Rev 1.6 — the SPI characteristics and the ODR tables were
 * read from that later revision, which is the one TDK publishes now.  Only
 * the sections named in those comments have been re-checked against it.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "drivers.h"
#include "bus_io.h"
#include "chip_ts.h"
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

/*
 * HEADER_TIMESTAMP_FSYNC, bits [3:2] (DS-000347 Rev 1.6 §6.2):
 *   00  no timestamp or FSYNC data in this packet
 *   01  reserved
 *   10  bytes [14:15] are an ODR timestamp   <- what this driver wants
 *   11  bytes [14:15] are FSYNC time, first ODR after an FSYNC event
 * icm_init leaves TMST_FSYNC_EN clear in TMST_CONFIG, so 11 should never
 * appear; the check is written to accept only 10 rather than to reject 11,
 * so a part configured differently degrades instead of misreading FSYNC
 * time as a sample time.
 */
#define FIFO_HDR_TS_MASK  0x0C
#define FIFO_HDR_TS_ODR   0x08

/* ── Static driver state ───────────────────────────────────────────────────── */

static struct {
    float    accel_scale;       /* LSB → m/s² */
    float    gyro_scale;        /* LSB → rad/s */
    uint32_t seq;
    uint32_t ticks_per_sample;  /* chip-timer ticks between adjacent samples,
                                 * at 32/30 µs each — see icm_init */
    uint32_t ts_last_raw;       /* last raw 20-bit TMSTVAL read */
    uint32_t ts_hi;             /* accumulated wrap carry (multiples of 2^20) */
    bool     ts_fifo_usable;    /* the configured ODR's sample period fits
                                 * inside the FIFO stamp's 16-bit repeat */
    uint64_t ts_rejects;        /* bursts whose batched stamps failed the
                                 * plausibility check and fell back */
    uint64_t ts_reject_next;    /* next count worth a log line — see icm_read */
    uint64_t ts_fwd_rejects;    /* counter reads refused as too far ahead */
    uint64_t ts_fwd_next;       /* next count worth a log line */
    uint64_t ts_bwd_rejects;    /* counter reads refused as too far behind */
    uint64_t ts_bwd_next;       /* next count worth a log line */
    chip_ts_guard_t ts_guard;   /* see chip_ts.h */
} ic;

/* One second of ticks at 32/30 us; see chip_ts.h. */
#define TS_MAX_JITTER_TICKS  937500u

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

/*
 * The FIFO's own timestamp field is 16 bits, so it repeats every 65536 ticks
 * (~70 ms).  Its epoch is recovered from the 32-bit TMSTVAL read after the
 * drain, which only works while the newest sample is less than one repeat
 * behind that read.  Reject well short of the boundary rather than at it: a
 * burst whose newest sample is more than three quarters of a repeat old is
 * one where the next scheduling hiccup would land on the wrong side, and a
 * whole-wrap error in chip_ts is far worse than falling back.
 */
#define TS_FIFO_WRAP_TICKS   65536u
#define TS_FIFO_MAX_LAG      49152u   /* 3/4 of a repeat, ~52 ms */

/* ── ODR and full-scale encoding ───────────────────────────────────────────── */

/*
 * GYRO_CONFIG0 / ACCEL_CONFIG0 bits [3:0] — DS-000347 Rev 1.6 §5.6.
 *
 * The codes are not monotonic in rate: 500 Hz is 1111, parked at the end of
 * the field past the reserved codes rather than in sequence between 200 Hz
 * and 1 kHz.  That is a datasheet quirk, not a typo here, and it is why 500
 * Hz went missing in the first place — reading the table top to bottom skips
 * straight from 200 to 1000.
 *
 * 16 kHz and 32 kHz are low-noise mode only on the accel (they are the only
 * two rates the accel table does not also offer in LP).  init() writes
 * PWR_MGMT0 = 0x0F, gyro and accel both LN, so all twelve rates are reachable.
 */
/*
 * The bottom rung is 12500 milli-Hz — 12.5 Hz, exactly, with no rounding left
 * to argue about.
 *
 * Declaring it 12, as a whole-Hz ladder would, puts the
 * advertised rate 4% below what the part produces and, worse, made
 * ticks_per_sample 937500/12 = 78125 where the true spacing is 937500/12.5 =
 * 75000.  That division was exact, which made it look like the right number
 * while being 4.2% wrong — the exactness was a coincidence around a wrong
 * rate.  In milli-Hz it is simply 937500000/12500 = 75000.
 *
 * Unlike the ST parts this was never a divider-chain artefact: TDK's ladder
 * runs 32k/2^n down to 1 kHz and then a decimal 200/100/50/25/12.5, and 12.5
 * is the datasheet's exact figure.
 *
 * Still not verified on silicon — there is no ICM-42688-P on the bench, which
 * is also why this driver is experimental.  The values are the datasheet's.
 */static uint8_t odr_encode(int mhz)
{
    if (mhz <=    12500) return 0x0B;  /* 12.5 Hz, exactly */
    if (mhz <=    25000) return 0x0A;
    if (mhz <=    50000) return 0x09;
    if (mhz <=   100000) return 0x08;
    if (mhz <=   200000) return 0x07;
    if (mhz <=   500000) return 0x0F;  /* out of sequence — see above */
    if (mhz <=  1000000) return 0x06;
    if (mhz <=  2000000) return 0x05;
    if (mhz <=  4000000) return 0x04;
    if (mhz <=  8000000) return 0x03;
    if (mhz <= 16000000) return 0x02;
    return 0x01;  /* 32000 Hz */
}

/* Must mirror odr_encode() exactly; both bound and clamp derive from the
 * table so appending a rung cannot leave the ceiling behind. */
static int odr_actual(int mhz)
{
    static const int steps[] = { 12500, 25000, 50000, 100000, 200000,
                                 500000, 1000000, 2000000,
                                 4000000, 8000000, 16000000, 32000000 };
    static const int n = (int)(sizeof steps / sizeof steps[0]);
    for (int i = 0; i < n; i++)
        if (mhz <= steps[i]) return steps[i];
    return steps[n - 1];
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

static int icm_probe(const imud_bus_t *bus)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
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

static int icm_reset(const imud_bus_t *bus)
{
    /* Ensure we are in Bank 0 before reset. */
    if (bus_reg_write(bus, REG_BANK_SEL, 0) < 0) return -1;

    if (bus_reg_write(bus, REG_DEVICE_CONFIG, 0x01) < 0) return -1;  /* SOFT_RESET */

    for (int i = 0; i < 100; i++) {
        usleep(1000);
        uint8_t val;
        if (bus_reg_read(bus, REG_DEVICE_CONFIG, &val) < 0) return -1;
        if (!(val & 0x01)) goto reset_done;
    }
    LOG_W("icm42688p: SOFT_RESET did not clear after 100 ms\n");
    return -1;

reset_done:
    usleep(1000);
    return 0;
}

static int icm_init(const imud_bus_t *bus, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_mhz);
    uint8_t gfs  = gyro_fs_encode(cfg->gyro_dps,  &gyro_scale);
    uint8_t afs  = accel_fs_encode(cfg->accel_g, &accel_scale);

    /* Must clear INT_ASYNC_RESET after reset (datasheet §12.6). */
    if (bus_reg_write(bus, REG_INT_CONFIG1, 0x00) < 0) return -1;

    /* Enable hardware timestamp latch to Bank 1 registers. */
    if (bus_reg_write(bus, REG_TMST_CONFIG, 0x11) < 0) return -1; /* TO_REGS_EN|EN */

    /* Enable accel + gyro in low-noise mode; wait for power-up. */
    if (bus_reg_write(bus, REG_PWR_MGMT0, 0x0F) < 0) return -1;
    usleep(200);

    /* Configure ODR and full-scale. */
    if (bus_reg_write(bus, REG_GYRO_CONFIG0,  (uint8_t)(gfs | odr)) < 0) return -1;
    if (bus_reg_write(bus, REG_ACCEL_CONFIG0, (uint8_t)(afs | odr)) < 0) return -1;

    /* Watermark in packets (one 16-byte packet per sample-pair). */
    int wm = cfg->fifo_wm;
    if (wm > 511) wm = 511;
    if (bus_reg_write(bus, REG_FIFO_CONFIG2, (uint8_t)(wm & 0xFF))        < 0) return -1;
    if (bus_reg_write(bus, REG_FIFO_CONFIG3, (uint8_t)((wm >> 8) & 0x0F)) < 0) return -1;

    /* Route FIFO threshold to INT1. */
    if (bus_reg_write(bus, REG_INT_SOURCE0, 0x04) < 0) return -1; /* FIFO_THS_INT1_EN */

    /* Enable accel, gyro, temp, and timestamp in FIFO packets. */
    if (bus_reg_write(bus, REG_FIFO_CONFIG1, 0x0B) < 0) return -1; /* TMST|GYRO|ACCEL */

    /* Flush FIFO before enabling stream mode. */
    if (bus_reg_write(bus, REG_SIGNAL_PATH_RESET, 0x02) < 0) return -1; /* FIFO_FLUSH */
    usleep(1000);

    /* Start stream mode: FIFO_CONFIG bits[7:6] = 01. */
    if (bus_reg_write(bus, REG_FIFO_CONFIG, 0x40) < 0) return -1;

    ic.accel_scale      = accel_scale;
    ic.gyro_scale       = gyro_scale;
    ic.seq              = 0;
    /* Ticks between adjacent samples at the actual ODR.
     *
     * NOT 1000000/ODR.  The timestamp counter's LSB is 32/30 µs, not the 1 µs
     * TMST_RES nominally selects — DS-000347 §12.7, which gives the scaling
     * explicitly and works the example: at 1 kHz the FIFO interval reads 937/938
     * for a true 1000 µs, and 937.5 * 32/30 = 1000.  The 32/30 branch is the
     * unconditional one here, since it applies whenever the part is not clocked
     * from an external RTC on CLKIN, and this driver never sets RTC_MODE.  So
     * one second is 937500 ticks, and ts_tick_ns is 1067 to match.
     *
     * 937500 = 2^2 * 3 * 5^7, so the division is exact at 12, 25, 50, 100 and
     * 500 Hz and nowhere else on the ladder: 200 Hz lands 0.011 % short, 1 kHz
     * 0.053 %, 2-8 kHz 0.16 %, and the top two rungs 1.0 %.  Deliberate, and
     * the same trade the previous constant made.  The value only walks backwards
     * from a per-burst anchor (see the age calculation in icm_read), so the
     * error is bounded by the watermark depth rather than accumulating across
     * bursts.  Sub-tick resolution would buy nothing the counter can support. */
    ic.ticks_per_sample =
        (uint32_t)(937500000u / (unsigned)odr_actual(cfg->odr_mhz));
    ic.ts_last_raw      = 0;    /* counter restarts with the chip */
    ic.ts_hi            = 0;
    /*
     * Can the per-packet FIFO timestamp be unwrapped at this rate?  Only if
     * adjacent samples are less than one 16-bit repeat apart, since that is
     * what makes the newest-to-oldest walk in icm_read unambiguous.  Every
     * rung from 25 Hz up clears it easily (37500 ticks at 25 Hz, 937 at
     * 1 kHz); 12 Hz does not — 78125 ticks is more than the 65536-tick
     * repeat, so two adjacent samples there could be one repeat apart and
     * read as adjacent.  That rung keeps the back-calculated timestamps.
     * The bound carries margin rather than sitting at the repeat itself.
     */
    ic.ts_fifo_usable   = (ic.ticks_per_sample <= TS_FIFO_MAX_LAG);
    ic.ts_rejects       = 0;
    ic.ts_reject_next   = 1;
    ic.ts_fwd_rejects   = 0;
    ic.ts_fwd_next      = 1;
    ic.ts_bwd_rejects   = 0;
    ic.ts_bwd_next      = 1;
    chip_ts_guard_reset(&ic.ts_guard);

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
 *   [14–15] FIFO timestamp, big endian, valid when the header says so
 *
 * TIMESTAMPS.  Bytes [14:15] are the chip's own record of when it took the
 * sample, so they are used where every packet in the burst carries them.  That
 * is strictly better than the fallback below, which reads the live TMSTVAL
 * counter AFTER the drain and calls that the newest sample's time: that
 * register reads *now*, the lag to the newest sample moves with bus and
 * scheduler jitter, and consecutive bursts can therefore overlap (see
 * chip_ts.h).  The batched stamps have no such lag, and they cost nothing —
 * the driver was already reading those two bytes and discarding them.
 *
 * The field is only 16 bits, repeating every 65536 ticks (~70 ms), so its
 * epoch still comes from TMSTVAL: reconstruct newest-first, stepping back one
 * repeat whenever a candidate would land after the sample that follows it.
 * If that reconstruction fails its plausibility checks the burst falls back to
 * the old path whole, rather than mixing two notions of time in one burst.
 */
static int icm_read(const imud_bus_t *bus,
                    imu_sample_t *buf, int max, int *n_out)
{
    /* ── 1. Read FIFO byte count (big endian; COUNTH read latches both) ─── */
    uint8_t cnt[2];
    if (bus_burst_read(bus, REG_FIFO_COUNTH, cnt, 2) < 0) return -1;

    int n_bytes = (((int)(cnt[0] & 0x3F)) << 8) | cnt[1];
    int n_pkts  = n_bytes / 16;

    if (n_pkts == 0) {
        *n_out = 0;
        return 0;
    }

    /* ── 2. Drain FIFO packets ──────────────────────────────────────────── */
    int  produced    = 0;
    /* Every packet must carry an ODR timestamp for the batched path to be
     * used.  One that does not makes the whole burst fall back, because a
     * burst half-stamped by the chip and half back-calculated would have a
     * seam in the middle of it and no way to tell which side is which. */
    bool all_stamped = true;

    for (int i = 0; i < n_pkts && produced < max; i++) {
        uint8_t pkt[16];
        if (bus_burst_read(bus, REG_FIFO_DATA, pkt, 16) < 0) return -1;

        uint8_t hdr = pkt[0];
        if (hdr & FIFO_HDR_MSG) continue;                    /* empty/padding */
        if (!(hdr & FIFO_HDR_ACCEL) || !(hdr & FIFO_HDR_GYRO)) continue;

        int16_t ax = reg_s16be(&pkt[1]);
        int16_t ay = reg_s16be(&pkt[3]);
        int16_t az = reg_s16be(&pkt[5]);
        int16_t gx = reg_s16be(&pkt[7]);
        int16_t gy = reg_s16be(&pkt[9]);
        int16_t gz = reg_s16be(&pkt[11]);

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
        /* Park the raw 16-bit stamp here; phase 3 either widens it in place or
         * overwrites the lot with the back-calculation.  Using the field
         * itself avoids a second 128-entry array on the hot path. */
        if ((hdr & FIFO_HDR_TS_MASK) == FIFO_HDR_TS_ODR) {
            buf[produced].chip_ts = ((uint32_t)pkt[14] << 8) | pkt[15];
        } else {
            buf[produced].chip_ts = 0;
            all_stamped = false;
        }
        produced++;
    }

    /* ── 3. Timestamp: read Bank 1 TMSTVAL, back-calculate per sample ───── */
    if (produced > 0) {
        if (bus_reg_write(bus, REG_BANK_SEL, 1) == 0) {
            uint8_t ts[3];
            if (bus_burst_read(bus, REG_TMSTVAL0, ts, 3) == 0) {
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

                uint32_t now_ts = ic.ts_hi + raw;
                bool     batched = false;

                /*
                 * Preferred path: widen each packet's own 16-bit stamp into
                 * the epoch TMSTVAL just established.
                 *
                 * Walk newest to oldest.  Each candidate starts in the epoch
                 * of the sample after it and drops one repeat if that would
                 * place it later — which is the whole unwrap.  Note what it
                 * does NOT require: the burst may span many repeats (at 25 Hz
                 * a 64-packet watermark spans 36 of them), because only each
                 * ADJACENT pair has to be under one repeat apart.  Whether
                 * that holds is a property of the ODR, decided once in init
                 * as ts_fifo_usable.
                 *
                 * Differences are taken as signed 32-bit so the arithmetic
                 * stays correct across the counter's own 32-bit wrap.
                 */
                if (all_stamped && ic.ts_fifo_usable) {
                    uint32_t prev = now_ts;
                    batched = true;

                    for (int i = produced - 1; i >= 0; i--) {
                        uint32_t cand = (prev & ~(TS_FIFO_WRAP_TICKS - 1))
                                      | (buf[i].chip_ts & (TS_FIFO_WRAP_TICKS - 1));
                        if ((int32_t)(cand - prev) > 0)
                            cand -= TS_FIFO_WRAP_TICKS;
                        /* Strictly increasing within the burst.  A field that
                         * is stuck, zeroed, or not a timestamp at all repeats
                         * a value and fails here on the second packet. */
                        if (i < produced - 1 && cand == prev) { batched = false; break; }
                        buf[i].chip_ts = cand;
                        prev = cand;
                    }

                    /*
                     * The epoch seed has to be sound too: the newest sample is
                     * placed relative to a TMSTVAL read taken after the drain,
                     * and if that gap approaches a repeat the epoch was a coin
                     * flip.  The gap is bounded in practice by the drain
                     * duration plus one sample period, both far short of it.
                     */
                    if (batched &&
                        (uint32_t)(now_ts - buf[produced - 1].chip_ts) > TS_FIFO_MAX_LAG)
                        batched = false;

                    /*
                     * Say so, at 1, 10, 100, ... rejections.  A one-shot line
                     * cannot distinguish a part that stumbled once from one
                     * whose timestamp field is not what this driver thinks it
                     * is, and that distinction is the whole diagnosis; logging
                     * every burst would bury it at 13 drains a second.
                     */
                    if (!batched && ++ic.ts_rejects >= ic.ts_reject_next) {
                        LOG_W("icm42688p: %llu burst(s) with FIFO timestamps "
                              "that failed their plausibility check — using "
                              "the post-drain TMSTVAL read for chip_ts\n",
                              (unsigned long long)ic.ts_rejects);
                        ic.ts_reject_next *= 10;
                    }
                }

                /*
                 * Fallback: read the counter after the drain and step back one
                 * sample period per sample.  That register reads *now*, and
                 * the lag to the newest sample moves with bus and scheduler
                 * jitter, so consecutive bursts can overlap and chip_ts can go
                 * backwards across the seam — which is what chip_ts.h corrects.
                 */
                if (!batched) {
                    uint32_t burst_ts = now_ts;
                    uint32_t span  = (uint32_t)(produced - 1) * ic.ticks_per_sample;
                    uint32_t first = burst_ts - span;
                    /*
                     * Is now_ts credible at all? A garbage counter read lands
                     * the whole burst in the future, which the backward guard
                     * cannot see because it only corrects overlaps. Measured on
                     * the reference ISM330DHCX, same fallback shape: one burst
                     * stamped 54 s ahead, recovered only on the burst after.
                     */
                    /*
                     * Both directions, and refuse either.  A read far BEHIND the
                     * previous burst was once passed through as a counter
                     * reset — see chip_ts.h for why that premise does not hold
                     * inside a run, and for what it emitted on the bench.
                     */
                    bool fwd_bad = !chip_ts_guard_forward_ok(
                            &ic.ts_guard, first,
                        ic.ticks_per_sample * TS_FWD_SLACK_SETS);
                    bool bwd_bad = !chip_ts_guard_backward_ok(
                            &ic.ts_guard, first, TS_MAX_JITTER_TICKS);
                    if (fwd_bad || bwd_bad) {
                        /*
                         * Refusing read after read means the ANCHOR is stale, not that
                         * the part keeps lying: every correct reading looks equally
                         * implausible against a bad `last`.  Past the limit, take the
                         * reading and re-seed -- extrapolating again only walks the
                         * stamps further from real time.  See chip_ts.h.
                         */
                        if (chip_ts_guard_refused(&ic.ts_guard)) {
                            chip_ts_guard_accepted(&ic.ts_guard);   /* burst_ts stands */
                        } else {
                        first    = chip_ts_guard_next(&ic.ts_guard,
                                                      ic.ticks_per_sample);
                        burst_ts = first + span;
                        if (bwd_bad) {
                            if (++ic.ts_bwd_rejects >= ic.ts_bwd_next) {
                                LOG_W("icm42688p: %llu post-drain timestamp "
                                      "read(s) implausibly far behind; "
                                      "extrapolating from the previous burst\n",
                                      (unsigned long long)ic.ts_bwd_rejects);
                                ic.ts_bwd_next *= 10;
                            }
                        } else if (++ic.ts_fwd_rejects >= ic.ts_fwd_next) {
                            LOG_W("icm42688p: %llu post-drain timestamp "
                                  "read(s) implausibly far ahead; "
                                  "extrapolating from the previous burst\n",
                                  (unsigned long long)ic.ts_fwd_rejects);
                            ic.ts_fwd_next *= 10;
                        }
                        }
                    } else {
                        chip_ts_guard_accepted(&ic.ts_guard);
                        burst_ts += chip_ts_guard_shift(&ic.ts_guard, first,
                                                        ic.ticks_per_sample,
                                                        TS_MAX_JITTER_TICKS);
                    }
                    for (int i = 0; i < produced; i++) {
                        uint32_t age = (uint32_t)(produced - 1 - i) * ic.ticks_per_sample;
                        buf[i].chip_ts = burst_ts - age;
                    }
                } else {
                    /* The batched path cannot produce the seam overlap the
                     * guard exists for, since the stamps come from the chip at
                     * the sample instant rather than being inferred after the
                     * fact.  Run it anyway: it is a contract check that costs
                     * two comparisons and should never fire here, and the two
                     * paths must not leave it holding a stale anchor if a
                     * later burst falls back. */
                    uint32_t shift = chip_ts_guard_shift(&ic.ts_guard,
                                                         buf[0].chip_ts,
                                                         ic.ticks_per_sample,
                                                         TS_MAX_JITTER_TICKS);
                    for (int i = 0; shift && i < produced; i++)
                        buf[i].chip_ts += shift;
                }
                chip_ts_guard_note(&ic.ts_guard, buf[produced - 1].chip_ts);
            }
            bus_reg_write(bus, REG_BANK_SEL, 0);
        }
    }

    *n_out = produced;
    return 0;
}

/* ── Driver descriptor ─────────────────────────────────────────────────────── */

const imu_ops_t icm42688p_ops = {
    .name             = "icm42688p",
    .experimental     = true,
    /* DS-000347 Rev 1.6 §9.6: MSB-first, latched on the rising edge and
     * transitioned on the falling — satisfied by mode 0 and mode 3 alike, and
     * mode 3 is what the rest of the tree uses. 24 MHz, 7-bit address behind
     * the R/W bit, burst reads auto-increment. The part supports single writes
     * only, which is all bus_reg_write ever issues. */
    .bus_caps         = { .spi_capable = true, .spi_mode = 3,
                          .spi_max_hz = 24000000, .spi_inc_mask = 0 },
    .probe            = icm_probe,
    .reset            = icm_reset,
    .init             = icm_init,
    .read             = icm_read,
    .has_fifo         = true,
    .has_hw_timestamp = true,
    /* TMSTVAL, unwrapped to 32 bits.  1067 = 32/30 µs, not the 1 µs TMST_RES
     * names: DS-000347 §12.7 scales the counter by 32/30 whenever the part is
     * not clocked from CLKIN, which is every configuration this driver
     * programs.  There is no ts_tick_ns_actual hook because the correction is
     * a property of the clock source rather than of the individual die, and
     * the part exposes no trim register to ask.  Whether TMSTVAL shares the
     * scaling with the FIFO's own timestamp field is not stated — it is the
     * same counter, and imud-imutest's imu.chipts.wall prints the implied tick,
     * so one bench run settles it either way. */
    .ts_tick_ns       = 1067,
    /*
     * Everything the silicon does (DS-000347 Rev 1.6 §5.6), including the two
     * rates a Raspberry Pi will not survive.
     *
     * 16 kHz and 32 kHz are offered because the part has them and imud is not
     * a Pi-only daemon; on a host with the headroom they are usable, and
     * withholding them would be imud deciding what hardware someone runs.  On
     * a Pi they are not: 32 kHz fills the 256-deep imu_ring_t in 8 ms and asks
     * the fusion thread for 32000 MEKF predictions a second, so the realistic
     * outcome is this driver's FIFO-overflow path (read() returning 1) rather
     * than data.  Choosing them should be a decision, not a discovery —
     * manual §5 says the same thing where an operator will see it.
     */
    .supported_odr_mhz  = { 12500, 25000, 50000, 100000, 200000, 500000,
                            1000000, 2000000, 4000000,
                            8000000, 16000000, 32000000, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 125, 250, 500, 1000, 2000, 0 },
};
