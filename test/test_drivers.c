/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_drivers.c — register-level decode/encode tests over the mock bus
 * (test/bus_mock.c, --wrap=ioctl), which serves both I2C and spidev.
 *
 * Covers ALL ELEVEN hardware driver files: the two hardware-validated ones
 * (ism330dhcx, mmc5983ma) and the nine flagged `experimental` — mpu925x (which
 * holds the MPU-9250/9255 pair), lsm6dso (LSM6DSO/LSM6DSOX), icm42688p,
 * icm20948, ak09916, ak8963, lis3mdl, lis2mdl, rm3100.  For those, nothing else
 * has ever executed a line: a transposed register or a sign error would
 * otherwise wait for silicon that may never arrive.
 *
 * Count files, not ops: three of those files register two parts each, so the
 * registry has fifteen *_ops entries (sim is two of them) against twelve files.
 *
 * A mock cannot replace bench validation — it cannot tell you
 * the chip→board axis remap matches the physical part.  What it does catch is
 * everything transcribed from a datasheet: register addresses, full-scale
 * encoding, byte order, and the return-code contract.
 *
 * These paths are otherwise exercised only on a wired-up Raspberry Pi.
 * The tests verify, off-hardware:
 *   - probe()  accepts the correct WHO_AM_I / product ID and rejects others;
 *   - init()   encodes the requested ODR and full-scale into the right control
 *              registers (asserted by reading the mock register file back);
 *   - read()   decodes a staged FIFO / output payload into the correct SI
 *              samples — sensitivity scaling, byte order, FIFO tag routing,
 *              temperature decode, and the chip→body axis remap;
 *   - I2C errors propagate as -1.
 *
 * reset() IS covered: i2cmock_set_selfclear models a bit that hardware clears
 * once the operation completes, so both the success path and the give-up path
 * are exercised.  What a mock still cannot judge is reset *timing* — how long
 * the part actually takes — which remains an `imud-imutest` bench check.
 *
 * Linux/GNU-ld only (the --wrap and <linux/i2c.h> dependencies), like the rest
 * of the daemon-linking suite.  Reference coverage for the mock pattern; the
 * remaining drivers extend it by reusing bus_mock unchanged.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "drivers.h"
#include "imu_math.h"   /* odr_actual_imu / odr_actual_mag — see test_odr_agreement */
#include "bus_mock.h"
#include "drivers/bus_io.h"     /* the framing under test in test_spi_framing */
#include "drivers/st_fifo_ts.h" /* ST_TAG_TIMESTAMP and the tag-byte layout */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* Reference the ops structs directly (the target links only these two driver
 * objects, not the whole registry).  test_drivers_registry covers the lookup. */
extern const imu_ops_t ism330dhcx_ops;
extern const mag_ops_t mmc5983ma_ops;
extern const imu_ops_t mpu9250_ops;
extern const imu_ops_t mpu9255_ops;
extern const mag_ops_t ak8963_ops;

static const imu_ops_t *ism = &ism330dhcx_ops;
static const mag_ops_t *mmc = &mmc5983ma_ops;
static const imu_ops_t *mpu = &mpu9250_ops;
static const mag_ops_t *ak  = &ak8963_ops;

#define FD 3   /* arbitrary; the mock ignores it */

/*
 * A handle addressing `a` on the mock bus.  bus_mock.c ignores the descriptor
 * entirely, so FD is arbitrary; the address is what selects which of the 128
 * register files a transfer lands in.
 */
#define I2CBUS(a) (&(const imud_bus_t){ .kind = BUS_I2C, \
                                        .fd = FD, .i2c_addr = (a) })

/* ── ISM330DHCX ──────────────────────────────────────────────────────────── */

#define ISM_ADDR 0x6A

/* Build one 7-byte FIFO word (tag + little-endian X/Y/Z) and queue it. */
static void ism_push_word(uint8_t tag, int16_t x, int16_t y, int16_t z)
{
    uint8_t w[7] = {
        (uint8_t)(tag << 3),
        (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF),
        (uint8_t)(z & 0xFF), (uint8_t)((z >> 8) & 0xFF),
    };
    i2cmock_fifo_push(ISM_ADDR, w, 7);
}

static void test_ism_probe(void)
{
    begin("test_ism_probe");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);          /* WHO_AM_I = correct */
    EXPECT(ism->probe(I2CBUS(ISM_ADDR)) == 0, "probe accepts 0x6B");

    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x00);          /* wrong id */
    EXPECT(ism->probe(I2CBUS(ISM_ADDR)) != 0, "probe rejects wrong WHO_AM_I");

    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);
    i2cmock_fail_next_xfer();
    EXPECT(ism->probe(I2CBUS(ISM_ADDR)) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_ism_init_registers(void)
{
    begin("test_ism_init_registers");
    int fb = g_fail;

    i2cmock_reset();
    /* seed OUT_TEMP so init's last_temp read succeeds (0 → 25 °C). */
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);

    imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    EXPECT(ism->init(I2CBUS(ISM_ADDR), &cfg) == 0, "init succeeds");

    /* ODR code 0x5 (208 Hz); accel FS ±4g code 0x08; gyro FS ±500 dps code 0x04. */
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x10) == 0x5A, "CTRL1_XL = ODR|FS|LPF2");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x11) == 0x54, "CTRL2_G = ODR|FS");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x12) == 0x44, "CTRL3_C = BDU|IF_INC");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x18) == 0x02, "CTRL9_XL = DEVICE_CONF");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x19) == 0x20, "CTRL10_C = TIMESTAMP_EN");
    /* fifo_wm 64 sample-sets → 128 FIFO words: WTM low=0x80, high=0. */
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x07) == 0x80, "FIFO_CTRL1 = WTM[7:0]");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x08) == 0x00, "FIFO_CTRL2 = WTM[8]");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x09) == 0x55, "FIFO_CTRL3 = BDR gy|xl");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x0A) == 0xE6,
           "FIFO_CTRL4 = DEC_TS_BATCH/32 | temp batch | continuous");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x0D) == 0x08, "INT1_CTRL = FIFO_TH");

    end(fb);
}

/* FIFO tag sensor codes, DS13012 Table 159.  The drivers keep their own
 * copies; naming them here keeps the timestamp test readable next to the
 * bare 0x01/0x02 the older tests use. */
#define TAG_GYRO_NC   0x01
#define TAG_ACCEL_NC  0x02

/* A tag-0x04 word: 32-bit counter little-endian in the X/Y payload bytes. */
static void ism_push_ts_word(uint8_t tag_cnt, uint32_t ts)
{
    uint8_t w[7] = {
        (uint8_t)((ST_TAG_TIMESTAMP << 3) | ((tag_cnt & 0x03) << 1)),
        (uint8_t)(ts & 0xFF), (uint8_t)((ts >> 8) & 0xFF),
        (uint8_t)((ts >> 16) & 0xFF), (uint8_t)((ts >> 24) & 0xFF),
        0, 0,
    };
    i2cmock_fifo_push(ISM_ADDR, w, 7);
}

/* An accel or gyro word carrying an explicit TAG_CNT slot number. */
static void ism_push_word_cnt(uint8_t tag, uint8_t tag_cnt,
                              int16_t x, int16_t y, int16_t z)
{
    uint8_t w[7] = {
        (uint8_t)((tag << 3) | ((tag_cnt & 0x03) << 1)),
        (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF),
        (uint8_t)(z & 0xFF), (uint8_t)((z >> 8) & 0xFF),
    };
    i2cmock_fifo_push(ISM_ADDR, w, 7);
}

/* Stage DIFF_FIFO (in WORDS, not sample-sets) and the FIFO port. */
static void ism_stage_fifo(int n_words, uint32_t timestamp0)
{
    i2cmock_set_reg(ISM_ADDR, 0x3A, (uint8_t)(n_words & 0xFF));
    i2cmock_set_reg(ISM_ADDR, 0x3B, (uint8_t)((n_words >> 8) & 0x03));
    i2cmock_set_fifo_reg(ISM_ADDR, 0x78);
    for (int i = 0; i < 4; i++)
        i2cmock_set_reg(ISM_ADDR, (uint8_t)(0x40 + i),
                        (uint8_t)((timestamp0 >> (8 * i)) & 0xFF));
}

/*
 * The ST parts' batched FIFO timestamp (tag 0x04).
 *
 * Why: the fallback reads TIMESTAMP0 after the drain and calls that the newest
 * sample's time.  The register reads *now*, the lag to the newest sample moves
 * with bus and scheduler jitter, producing 2-4 chip_ts
 * reversals per 5 s because of it.  A timestamp word written into the stream
 * has no such lag.
 *
 * What is assumed, and therefore tested: the payload layout (LE-32 in X/Y,
 * which DS13012 never states) and the TAG_CNT slot match that makes the word's
 * position in the stream irrelevant.
 */
static void test_ism_batched_timestamp(void)
{
    begin("test_ism_batched_timestamp");
    int fb = g_fail;
    imu_sample_t buf[16];
    int n = -1;

    /* The decimation ladder.  It exists so a watermark-depth drain usually
     * contains a word, without paying the +50% word traffic that per-sample
     * batching costs — which would silently redefine fifo_wm. */
    struct { int wm; uint8_t ctrl4; const char *why; } dec[] = {
        { 64, 0xE6, "fifo_wm 64 batches every 32 sets (+1.6% words)" },
        { 32, 0xE6, "fifo_wm 32 is the boundary and still takes /32" },
        { 31, 0xA6, "fifo_wm 31 drops to /8 so a word still lands"   },
        {  8, 0xA6, "fifo_wm 8 is the /8 boundary"                   },
        {  7, 0x26, "fifo_wm 7 batches nothing — those watermarks are "
                    "chosen for latency and must not be perturbed"   },
    };
    for (unsigned i = 0; i < sizeof dec / sizeof dec[0]; i++) {
        i2cmock_reset();
        i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
        i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
        imu_cfg_t c = { .odr_mhz = 833000, .accel_g = 4, .gyro_dps = 500,
                        .fifo_wm = dec[i].wm };
        (void)ism->init(I2CBUS(ISM_ADDR), &c);
        EXPECT(i2cmock_get_reg(ISM_ADDR, 0x0A) == dec[i].ctrl4, dec[i].why);
    }

    /*
     * Anchoring.  Three sample-sets in slots 0,1,2 with a timestamp word for
     * slot 1, and a TIMESTAMP0 that reads late — 5000 ticks past the newest
     * sample, which is the very lag the batched path is immune to.
     *
     * ticks_per_sample at 833 Hz is 40000/833 = 48.
     */
    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    imu_cfg_t cfg = { .odr_mhz = 833000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);

    const uint32_t T = 0x00100000u;   /* the timestamp word's value */
    ism_stage_fifo(7, T + 48 + 5000);  /* 6 sample words + 1 timestamp word */
    ism_push_word_cnt(TAG_ACCEL_NC, 0, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  0, 4, 5, 6);
    ism_push_word_cnt(TAG_ACCEL_NC, 1, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  1, 4, 5, 6);
    ism_push_ts_word(1, T);            /* dates slot 1, arriving after it */
    ism_push_word_cnt(TAG_ACCEL_NC, 2, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  2, 4, 5, 6);

    EXPECT(ism->read(I2CBUS(ISM_ADDR), buf, 16, &n) == 0 && n == 3,
           "three sample-sets decode alongside a timestamp word");
    EXPECT(buf[1].chip_ts == T,
           "the set the word names carries the word's value exactly");
    EXPECT(buf[0].chip_ts == T - 48u, "older sample steps back one period");
    EXPECT(buf[2].chip_ts == T + 48u, "newer sample steps forward one");

    /*
     * The same burst with the word BEFORE its slot rather than after it.
     * DS13012 never says which way round the chip writes them, and this is
     * why it does not have to: TAG_CNT names the slot, position does not.
     */
    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);
    ism_stage_fifo(7, T + 48 + 5000);
    ism_push_word_cnt(TAG_ACCEL_NC, 0, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  0, 4, 5, 6);
    ism_push_ts_word(1, T);            /* dates slot 1, arriving before it */
    ism_push_word_cnt(TAG_ACCEL_NC, 1, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  1, 4, 5, 6);
    ism_push_word_cnt(TAG_ACCEL_NC, 2, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  2, 4, 5, 6);

    n = -1;
    EXPECT(ism->read(I2CBUS(ISM_ADDR), buf, 16, &n) == 0 && n == 3,
           "same burst, word ahead of its slot");
    EXPECT(buf[1].chip_ts == T,
           "TAG_CNT still lands it on slot 1 — ordering is not used");

    /* No word in the drain: the fallback still works, unchanged. */
    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);
    ism_stage_fifo(4, 0x2000);
    ism_push_word_cnt(TAG_ACCEL_NC, 0, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  0, 4, 5, 6);
    ism_push_word_cnt(TAG_ACCEL_NC, 1, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  1, 4, 5, 6);
    n = -1;
    (void)ism->read(I2CBUS(ISM_ADDR), buf, 16, &n);
    EXPECT(n == 2 && buf[1].chip_ts == 0x2000u,
           "a drain with no timestamp word takes the post-drain read");

    /*
     * A word whose value does not survive the cross-check is refused whole.
     * This is what protects the undocumented payload layout: if LE-32-in-X/Y
     * is wrong, the decoded value is garbage and lands nowhere near
     * TIMESTAMP0, so the burst degrades to the old path instead of shipping
     * a corrupt sample clock.
     */
    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);
    ism_stage_fifo(5, 0x2000);
    ism_push_word_cnt(TAG_ACCEL_NC, 0, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  0, 4, 5, 6);
    ism_push_ts_word(0, 0xDEADBEEFu);   /* nothing to do with 0x2000 */
    ism_push_word_cnt(TAG_ACCEL_NC, 1, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  1, 4, 5, 6);
    n = -1;
    (void)ism->read(I2CBUS(ISM_ADDR), buf, 16, &n);
    EXPECT(n == 2 && buf[1].chip_ts == 0x2000u,
           "an implausible timestamp word is refused, not used");

    /*
     * A word naming a slot no sample-set reached — the FIFO ended mid-set.
     * Applying it would index past the samples that exist.
     */
    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);
    ism_stage_fifo(4, 0x2000);
    ism_push_word_cnt(TAG_ACCEL_NC, 0, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  0, 4, 5, 6);
    ism_push_ts_word(1, 0x2000u - 48u);  /* names slot 1 ... */
    ism_push_word_cnt(TAG_ACCEL_NC, 1, 1, 2, 3);   /* ... which never pairs */
    n = -1;
    (void)ism->read(I2CBUS(ISM_ADDR), buf, 16, &n);
    EXPECT(n == 1 && buf[0].chip_ts == 0x2000u,
           "a word naming a set that never completed is refused");

    /*
     * The seam the whole thing exists to close.  Two drains whose post-drain
     * reads land at the SAME counter value — the classic high-lag-then-low-lag
     * pair — but whose batched words advance properly.  Under the fallback the
     * second burst would start at or before the first burst's last sample.
     */
    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);

    ism_stage_fifo(5, T + 3000);
    ism_push_word_cnt(TAG_ACCEL_NC, 0, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  0, 4, 5, 6);
    ism_push_ts_word(0, T);
    ism_push_word_cnt(TAG_ACCEL_NC, 1, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  1, 4, 5, 6);
    n = -1;
    (void)ism->read(I2CBUS(ISM_ADDR), buf, 16, &n);
    uint32_t first_burst_last = buf[n - 1].chip_ts;

    ism_stage_fifo(5, T + 3000);          /* identical "now" — the pathology */
    ism_push_word_cnt(TAG_ACCEL_NC, 2, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  2, 4, 5, 6);
    ism_push_ts_word(2, T + 96);          /* but the chip's own time advanced */
    ism_push_word_cnt(TAG_ACCEL_NC, 3, 1, 2, 3);
    ism_push_word_cnt(TAG_GYRO_NC,  3, 4, 5, 6);
    n = -1;
    (void)ism->read(I2CBUS(ISM_ADDR), buf, 16, &n);
    EXPECT(n == 2 && (int32_t)(buf[0].chip_ts - first_burst_last) > 0,
           "the second burst starts after the first ends, even though both "
           "post-drain reads returned the same counter value");

    end(fb);
}

static void test_ism_read_decode(void)
{
    begin("test_ism_read_decode");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);

    imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);   /* sets accel/gyro scale */

    /* FIFO status: 3 words available, no overflow. */
    i2cmock_set_reg(ISM_ADDR, 0x3A, 3);
    i2cmock_set_reg(ISM_ADDR, 0x3B, 0);
    i2cmock_set_fifo_reg(ISM_ADDR, 0x78);

    /* temp word (tag 3): raw 1280 → 1280/256 + 25 = 30 °C. */
    ism_push_word(0x03, 1280, 0, 0);
    /* accel (tag 2) and gyro (tag 1) raw counts. */
    ism_push_word(0x02, 100, 200, 300);
    ism_push_word(0x01, 10, 20, 30);

    /* timestamp counter 0x40302010 (little-endian in regs 0x40..0x43). */
    i2cmock_set_reg(ISM_ADDR, 0x40, 0x10);
    i2cmock_set_reg(ISM_ADDR, 0x41, 0x20);
    i2cmock_set_reg(ISM_ADDR, 0x42, 0x30);
    i2cmock_set_reg(ISM_ADDR, 0x43, 0x40);

    imu_sample_t buf[8];
    int n = -1;
    EXPECT(ism->read(I2CBUS(ISM_ADDR), buf, 8, &n) == 0, "read returns 0");
    EXPECT(n == 1, "one sample produced from accel+gyro pair");

    const float as = 0.122e-3f * 9.80665f;                 /* ±4g m/s²/LSB */
    const float gs = 17.5f * (float)(M_PI / 180.0 / 1000.0); /* ±500 dps rad/s/LSB */

    /* Chip→body remap flips Y and Z (see driver). */
    EXPECT_NEAR(buf[0].accel[0],  100 * as, 1e-4, "accel X");
    EXPECT_NEAR(buf[0].accel[1], -200 * as, 1e-4, "accel Y flipped");
    EXPECT_NEAR(buf[0].accel[2], -300 * as, 1e-4, "accel Z flipped");
    EXPECT_NEAR(buf[0].gyro[0],   10 * gs, 1e-6, "gyro X");
    EXPECT_NEAR(buf[0].gyro[1],  -20 * gs, 1e-6, "gyro Y flipped");
    EXPECT_NEAR(buf[0].gyro[2],  -30 * gs, 1e-6, "gyro Z flipped");
    EXPECT_NEAR(buf[0].temp_c, 30.0f, 1e-3, "temperature decoded from FIFO word");
    EXPECT(buf[0].seq == 0, "seq starts at 0");
    EXPECT(buf[0].chip_ts == 0x40302010u, "chip timestamp reassembled");

    end(fb);
}

static void test_ism_read_overflow_and_empty(void)
{
    begin("test_ism_read_overflow_and_empty");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);

    /* Empty FIFO, overflow flag set (FIFO_OVR_IA = bit 6 of STATUS2). */
    i2cmock_set_reg(ISM_ADDR, 0x3A, 0);
    i2cmock_set_reg(ISM_ADDR, 0x3B, 0x40);
    int n = -1;
    EXPECT(ism->read(I2CBUS(ISM_ADDR), NULL, 0, &n) == 1, "empty+overflow returns 1");
    EXPECT(n == 0, "no samples produced");

    /* I2C error on the status read. */
    i2cmock_fail_next_xfer();
    n = 99;
    EXPECT(ism->read(I2CBUS(ISM_ADDR), NULL, 0, &n) == -1, "I2C error returns -1");

    end(fb);
}

/* ── MMC5983MA ───────────────────────────────────────────────────────────── */

#define MMC_ADDR 0x30

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
}

/* Split an 18-bit unsigned reading into the 7 output registers (0x00..0x06). */
static void mmc_set_output_at(uint8_t at, uint32_t rx, uint32_t ry, uint32_t rz)
{
    uint8_t raw[7] = {
        (uint8_t)((rx >> 10) & 0xFF), (uint8_t)((rx >> 2) & 0xFF),
        (uint8_t)((ry >> 10) & 0xFF), (uint8_t)((ry >> 2) & 0xFF),
        (uint8_t)((rz >> 10) & 0xFF), (uint8_t)((rz >> 2) & 0xFF),
        (uint8_t)(((rx & 3) << 6) | ((ry & 3) << 4) | ((rz & 3) << 2)),
    };
    i2cmock_set_regs(at, 0x00, raw, 7);
}

static void mmc_set_output(uint32_t rx, uint32_t ry, uint32_t rz)
{
    mmc_set_output_at(MMC_ADDR, rx, ry, rz);
}

static void test_mmc_probe(void)
{
    begin("test_mmc_probe");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x30);       /* PRODUCT_ID correct */
    EXPECT(mmc->probe(I2CBUS(MMC_ADDR)) == 0, "probe accepts 0x30");

    i2cmock_set_reg(MMC_ADDR, 0x2F, 0xFF);
    EXPECT(mmc->probe(I2CBUS(MMC_ADDR)) != 0, "probe rejects wrong product ID");

    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x30);
    i2cmock_fail_next_xfer();
    EXPECT(mmc->probe(I2CBUS(MMC_ADDR)) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_mmc_reset_and_init(void)
{
    begin("test_mmc_reset_and_init");
    int fb = g_fail;

    i2cmock_reset();
    EXPECT(mmc->reset(I2CBUS(MMC_ADDR)) == 0, "reset succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x80, "reset writes SW_RST to CTRL1");

    /* 100 Hz → BW=01, CM_Freq=101. */
    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 5.0f };
    i2cmock_reset();
    double t0 = now_ms();
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &cfg) == 0, "init succeeds");
    double init_ms = now_ms() - t0;
    /*
     * The part saturates if anything touches it within ~40 ms of the CTRL2
     * write, so init holds the bus quiet for 100 ms afterwards. The mock cannot
     * model saturation, so what is testable is that the wait is still there.
     * The bound sits far below 100 ms deliberately: it must fail if someone
     * deletes the sleep — init would then return in microseconds — without
     * blocking a future retune to any value that could plausibly be right.
     * A duration floor cannot flake; a sleep only ever overruns.
     */
    EXPECT(init_ms >= 30.0, "init holds the bus quiet after starting continuous mode");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x01, "CTRL1 = BW bits");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0B) == 0x0D, "CTRL2 = Cmm_en|CM_Freq");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x04, "CTRL0 ends at INT_EN");

    /*
     * The write ORDER, which the final bytes above cannot show: every ordering
     * of these four writes leaves exactly the same register file behind, and
     * two of them are broken on real silicon.
     *
     * CTRL1 must follow both CTRL0 writes, or the alias below leaves X-inhibit
     * set. CTRL2 must be last, because enabling continuous mode before CTRL1
     * runs the part at the reset default BW=00 — rated for 50 Hz (Rev A p.4)
     * against a CM_Freq of 100 — and because a write within ~45 ms of that
     * enable leaves the bridge saturated for the rest of the run.
     *
     * The count is pinned too: it is what would catch a settle implemented as
     * a poll loop, which cannot work here (CTRL0/1/2 are write-only) but reads
     * plausibly enough to be attempted.
     */
    EXPECT(i2cmock_writes(MMC_ADDR) == 4, "init issues exactly four writes");
    EXPECT(i2cmock_write_at(MMC_ADDR, 0) == 0x09, "1st: CTRL0 cleared");
    EXPECT(i2cmock_write_at(MMC_ADDR, 1) == 0x09, "2nd: CTRL0 = INT_EN");
    EXPECT(i2cmock_write_at(MMC_ADDR, 2) == 0x0A, "3rd: CTRL1 = BW, after both CTRL0 writes");
    EXPECT(i2cmock_write_at(MMC_ADDR, 3) == 0x0B, "4th: CTRL2 starts continuous mode");
    EXPECT(i2cmock_last_write(MMC_ADDR) == 0x0B, "nothing is written after CTRL2");

    /*
     * Now with the part's real quirk modelled: a CTRL0 write lands in CTRL1
     * too.  init() must therefore write CTRL1 LAST, or its own INT_en write
     * leaves CTRL1 = 0x04 — X-inhibit — and the X axis stops measuring while
     * Y and Z carry on looking healthy;
     * without the alias above, both orderings pass.
     */
    i2cmock_reset();
    i2cmock_set_write_alias(MMC_ADDR, 0x09, 0x0A);
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &cfg) == 0, "init succeeds under the CTRL0 alias");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x01,
           "CTRL1 still holds BW after init, so X is not inhibited");
    EXPECT((i2cmock_get_reg(MMC_ADDR, 0x0A) & 0x04) == 0, "X-inhibit clear");
    EXPECT((i2cmock_get_reg(MMC_ADDR, 0x0A) & 0x18) == 0, "YZ-inhibit clear");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0B) == 0x0D, "CTRL2 = Cmm_en|CM_Freq");
    EXPECT((i2cmock_get_reg(MMC_ADDR, 0x09) & 0x04) != 0, "INT_EN still set");

    end(fb);
}

static void test_mmc_read_decode(void)
{
    begin("test_mmc_read_decode");
    int fb = g_fail;

    i2cmock_reset();
    /* State the mode rather than inheriting it: int_driven lives in a driver
     * static, so without this the case would pass or fail depending on which
     * test ran before it. */
    mag_cfg_t polled = { .odr_mhz = 100000, .set_period_s = 5.0f, .int_driven = false };
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &polled) == 0, "init in polled mode");
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x01);          /* STATUS: M_DONE set */

    /* Null field = 131072 counts, 16384 counts/G, 1 G = 100 µT. */
    mmc_set_output(131072 + 16384,   /* +1 G   → +100 µT */
                   131072 - 16384,   /* −1 G, but Y is sign-flipped → +100 µT */
                   131072 + 8192);   /* +0.5 G → +50 µT (Z not flipped) */

    mag_sample_t out;
    memset(&out, 0, sizeof out);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 0, "read returns 0");
    EXPECT(out.valid, "sample marked valid");

    EXPECT_NEAR(out.field[0],  100.0f, 0.1, "field X = +100 µT");
    EXPECT_NEAR(out.field[1],  100.0f, 0.1, "field Y flipped = +100 µT");
    EXPECT_NEAR(out.field[2],   50.0f, 0.1, "field Z = +50 µT");

    /* Not-ready: M_DONE clear → returns 1 (wait for next interrupt). */
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 1, "M_DONE clear returns 1");

    /* I2C error on the status read. */
    i2cmock_fail_next_xfer();
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == -1, "I2C error returns -1");

    end(fb);
}

/*
 * The interrupt path, and the reason it exists.
 *
 * On this part the INT is latched: the write that re-arms it also clears
 * Meas_M_Done, and the bit then only re-asserts while the bus is being polled.
 * A reader blocked on the edge is by definition not polling, so the gate can
 * never pass — 35 Hz from a 105.5 Hz part. The pair of assertions
 * below is the whole fix: identical registers, M_DONE CLEAR, and the answer
 * depends only on how the caller said it waits.
 */
/*
 * The targeted write injector itself. It is test infrastructure, so a fault in
 * it shows up as some other test mysteriously passing -- pin the three things
 * callers rely on: the transfer fails, the byte does NOT land, and `times`
 * bounds how many writes are refused.
 */
static void test_mock_fail_write_to(void)
{
    begin("test_mock_fail_write_to");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(MMC_ADDR, 0x09, 0x11);

    /* One shot: the first write to CTRL0 fails and leaves the byte alone. */
    i2cmock_fail_write_to(MMC_ADDR, 0x09, 1);
    EXPECT(bus_reg_write(I2CBUS(MMC_ADDR), 0x09, 0x22) < 0,
           "armed write returns -1");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x11,
           "a refused write does not land");

    /* The arm is spent: the next one goes through. */
    EXPECT(bus_reg_write(I2CBUS(MMC_ADDR), 0x09, 0x22) == 0,
           "times = 1 refuses exactly one write");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x22, "and then it lands");

    /* Other registers are unaffected while armed. */
    i2cmock_fail_write_to(MMC_ADDR, 0x09, -1);
    EXPECT(bus_reg_write(I2CBUS(MMC_ADDR), 0x0A, 0x33) == 0,
           "a different register is untouched by the arm");
    EXPECT(bus_reg_write(I2CBUS(MMC_ADDR), 0x09, 0x44) < 0,
           "sticky arm refuses again");
    EXPECT(bus_reg_write(I2CBUS(MMC_ADDR), 0x09, 0x44) < 0,
           "and keeps refusing");

    /* Disarm, and reset must clear it too. */
    i2cmock_fail_write_to(MMC_ADDR, -1, 0);
    EXPECT(bus_reg_write(I2CBUS(MMC_ADDR), 0x09, 0x55) == 0, "reg < 0 disarms");
    i2cmock_fail_write_to(MMC_ADDR, 0x09, -1);
    i2cmock_reset();
    i2cmock_set_reg(MMC_ADDR, 0x09, 0x00);
    EXPECT(bus_reg_write(I2CBUS(MMC_ADDR), 0x09, 0x66) == 0,
           "i2cmock_reset() disarms");

    end(fb);
}

static void test_mmc_int_driven_read(void)
{
    begin("test_mmc_int_driven_read");
    int fb = g_fail;

    i2cmock_reset();
    mag_cfg_t irq = { .odr_mhz = 100000, .set_period_s = 5.0f, .int_driven = true };
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &irq) == 0, "init in interrupt mode");

    /* M_DONE CLEAR — the state that stalls the shipped driver for 20 ms. */
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);
    mmc_set_output(131072 + 16384, 131072 - 16384, 131072 + 8192);

    mag_sample_t out;
    memset(&out, 0, sizeof out);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 0,
           "edge-driven read returns a sample with M_DONE clear");
    EXPECT(out.valid, "sample marked valid");
    EXPECT_NEAR(out.field[0], 100.0f, 0.1, "field X decoded on the edge path");
    EXPECT_NEAR(out.field[1], 100.0f, 0.1, "field Y decoded on the edge path");
    EXPECT_NEAR(out.field[2],  50.0f, 0.1, "field Z decoded on the edge path");

    /* The clear still has to happen — it is what re-arms the latched INT. */
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x08) == 0x01,
           "edge-driven read still writes M_DONE to re-arm the interrupt");

    /*
     * Staleness guard: unchanged output registers mean no new conversion, so a
     * dead INT line degrades to fewer real samples rather than a stream of one
     * duplicate. Without this the daemon cannot tell the two apart.
     */
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 1,
           "unchanged output registers report no new data");

    /* A real new conversion is accepted again. */
    mmc_set_output(131072 + 8192, 131072 - 8192, 131072 + 16384);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 0, "changed registers read again");
    EXPECT_NEAR(out.field[2], 100.0f, 0.1, "second sample decoded");

    /* The guard must not carry a sample across a reconfigure. */
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &irq) == 0, "re-init succeeds");
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 0,
           "init clears the staleness guard");

    /* And the contrast: same registers, same M_DONE, polled caller waits. */
    mag_cfg_t polled = { .odr_mhz = 100000, .set_period_s = 5.0f, .int_driven = false };
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &polled) == 0, "re-init in polled mode");
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 1,
           "polled read still gates on M_DONE");

    end(fb);
}

static void test_mmc_set_reset(void)
{
    begin("test_mmc_set_reset");
    int fb = g_fail;

    /*
     * CTRL0 = INT_EN | pulse, not the pulse alone.  Set and Reset self-clear;
     * INT_en does not, so a write of the bare pulse bit also switches off the
     * measurement-done interrupt that init() turned on.  This test asserted
     * exactly that bug (0x08) before it was found on hardware.
     */
    i2cmock_reset();
    EXPECT(mmc->set_reset(I2CBUS(MMC_ADDR)) == 0, "set_reset succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x0C,
           "set_reset writes SET | INT_EN to CTRL0");

    i2cmock_reset();
    EXPECT(mmc->degauss != NULL, "mmc5983ma offers a directional degauss");
    EXPECT(mmc->degauss(I2CBUS(MMC_ADDR), MAG_DEGAUSS_SET) == 0, "degauss SET succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x0C, "degauss SET writes SET | INT_EN");

    EXPECT(mmc->degauss(I2CBUS(MMC_ADDR), MAG_DEGAUSS_RESET) == 0, "degauss RESET succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x14, "degauss RESET writes RESET | INT_EN");

    /* SET and RESET must be different bits, or the differential measures
     * nothing — the whole point is that they magnetise opposite ways. */
    EXPECT((0x0C & 0x08) && (0x14 & 0x10) && ((0x0C ^ 0x14) == 0x18),
           "SET and RESET drive distinct CTRL0 bits");

    /*
     * The regression the hardware run would have caught: the interrupt init()
     * enabled must survive a degauss, in both directions.
     */
    i2cmock_reset();
    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 5.0f };
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &cfg) == 0, "init succeeds");
    EXPECT((i2cmock_get_reg(MMC_ADDR, 0x09) & 0x04) != 0, "init leaves INT_EN set");
    EXPECT(mmc->set_reset(I2CBUS(MMC_ADDR)) == 0, "set_reset after init");
    EXPECT((i2cmock_get_reg(MMC_ADDR, 0x09) & 0x04) != 0,
           "INT_EN survives the periodic SET pulse");
    EXPECT(mmc->degauss(I2CBUS(MMC_ADDR), MAG_DEGAUSS_RESET) == 0, "degauss RESET after init");
    EXPECT((i2cmock_get_reg(MMC_ADDR, 0x09) & 0x04) != 0,
           "INT_EN survives a RESET pulse");

    /*
     * The degauss does not restore CTRL1: in mode 0 a CTRL0 write does not
     * alias into it.  What must hold is that the pulse disturbs neither the
     * bandwidth nor the interrupt enable.
     */
    i2cmock_reset();
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &cfg) == 0, "init succeeds");
    EXPECT(mmc->set_reset(I2CBUS(MMC_ADDR)) == 0, "periodic SET succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x01,
           "CTRL1 still holds BW after the periodic SET");
    EXPECT(mmc->degauss(I2CBUS(MMC_ADDR), MAG_DEGAUSS_RESET) == 0, "degauss RESET succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x01,
           "CTRL1 still holds BW after a RESET pulse");
    EXPECT((i2cmock_get_reg(MMC_ADDR, 0x09) & 0x04) != 0,
           "INT_EN still set after both pulses");

    /* The bandwidth tracks the configured ODR and survives a pulse. */
    i2cmock_reset();
    mag_cfg_t slow = { .odr_mhz = 10000, .set_period_s = 5.0f };   /* BW=00 */
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &slow) == 0, "init at 10 Hz succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x00, "CTRL1 = BW for 10 Hz");
    EXPECT(mmc->set_reset(I2CBUS(MMC_ADDR)) == 0, "periodic SET at 10 Hz");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x00,
           "the 10 Hz BW is unchanged by the pulse");

    /* Bus error propagates from both entry points. */
    i2cmock_reset();
    i2cmock_fail_next_xfer();
    EXPECT(mmc->set_reset(I2CBUS(MMC_ADDR)) == -1, "set_reset reports a bus error");
    i2cmock_fail_next_xfer();
    EXPECT(mmc->degauss(I2CBUS(MMC_ADDR), MAG_DEGAUSS_SET) == -1,
           "degauss reports a bus error");

    end(fb);
}

/* ── MPU-9250 / MPU-9255 ─────────────────────────────────────────────────── */

#define MPU_ADDR 0x68
#define AK_ADDR  0x0C     /* AK8963, reached through the MPU's I2C bypass */

/* A genuine part answers on both addresses; probe() insists on both. */
static void mpu_stage_genuine(uint8_t whoami)
{
    i2cmock_reset();
    i2cmock_set_reg(MPU_ADDR, 0x75, whoami);   /* WHO_AM_I */
    i2cmock_set_reg(AK_ADDR,  0x00, 0x48);     /* AK8963 WIA */
}

static void test_mpu_probe(void)
{
    begin("test_mpu_probe");
    int fb = g_fail;

    mpu_stage_genuine(0x71);
    EXPECT(mpu->probe(I2CBUS(MPU_ADDR)) == 0, "mpu9250 probe accepts 0x71");
    EXPECT(mpu9255_ops.probe(I2CBUS(MPU_ADDR)) != 0, "mpu9255 probe rejects 0x71");

    mpu_stage_genuine(0x73);
    EXPECT(mpu9255_ops.probe(I2CBUS(MPU_ADDR)) == 0, "mpu9255 probe accepts 0x73");
    EXPECT(mpu->probe(I2CBUS(MPU_ADDR)) != 0, "mpu9250 probe rejects 0x73");

    /* The counterfeit that matters: a relabelled MPU-6500 (no magnetometer). */
    mpu_stage_genuine(0x70);
    EXPECT(mpu->probe(I2CBUS(MPU_ADDR)) != 0, "probe rejects MPU-6500 id 0x70");

    /* Right WHO_AM_I, but no compass answering through the bypass — the other
     * common clone.  Must fail rather than leave it to the mag driver. */
    mpu_stage_genuine(0x71);
    i2cmock_set_reg(AK_ADDR, 0x00, 0x00);
    EXPECT(mpu->probe(I2CBUS(MPU_ADDR)) != 0, "probe rejects missing AK8963");

    mpu_stage_genuine(0x71);
    i2cmock_fail_next_xfer();
    EXPECT(mpu->probe(I2CBUS(MPU_ADDR)) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_mpu_init_registers(void)
{
    begin("test_mpu_init_registers");
    int fb = g_fail;

    mpu_stage_genuine(0x71);

    /* 1000 Hz is on the 1000/(1+SMPLRT_DIV) grid with divider 0. */
    imu_cfg_t cfg = { .odr_mhz = 1000000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
    EXPECT(mpu->init(I2CBUS(MPU_ADDR), &cfg) == 0, "init succeeds");

    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x6B) == 0x01, "PWR_MGMT_1 = CLKSEL auto");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x6C) == 0x00, "PWR_MGMT_2 = all sensors on");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1A) == 0x03, "CONFIG = stream|DLPF_CFG 3");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x19) == 0x00, "SMPLRT_DIV = 0 for 1000 Hz");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1B) == 0x18, "GYRO_CONFIG = 2000 dps, DLPF on");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1C) == 0x10, "ACCEL_CONFIG = 8 g");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1D) == 0x03, "ACCEL_CONFIG2 = DLPF on, cfg 3");
    /* FIFO_RST is pulsed then written back, so the settled image is FIFO_EN
     * alone — and I2C_MST_EN (0x20) must stay clear or bypass cannot work. */
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x6A) == 0x40, "USER_CTRL = FIFO_EN only");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x23) == 0x78, "FIFO_EN = accel + gyro XYZ");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x38) == 0x01, "INT_ENABLE = RAW_RDY");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x37) == 0x02, "INT_PIN_CFG = BYPASS_EN");

    /* A rate off the grid must still land on a reachable divider. */
    mpu_stage_genuine(0x71);
    imu_cfg_t c200 = { .odr_mhz = 200000, .accel_g = 2, .gyro_dps = 250, .fifo_wm = 32 };
    EXPECT(mpu->init(I2CBUS(MPU_ADDR), &c200) == 0, "init at 200 Hz succeeds");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x19) == 4, "SMPLRT_DIV = 4 for 200 Hz");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1B) == 0x00, "GYRO_CONFIG = 250 dps");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1C) == 0x00, "ACCEL_CONFIG = 2 g");

    /* Two consecutive inits must leave an identical register image. */
    uint8_t before[0x80];
    for (int r = 0; r < 0x80; r++) before[r] = i2cmock_get_reg(MPU_ADDR, (uint8_t)r);
    EXPECT(mpu->init(I2CBUS(MPU_ADDR), &c200) == 0, "second init succeeds");
    int same = 1;
    for (int r = 0; r < 0x80; r++)
        if (r != 0x74 && before[r] != i2cmock_get_reg(MPU_ADDR, (uint8_t)r)) same = 0;
    EXPECT(same, "init is idempotent in the register image");

    end(fb);
}

static void test_mpu_read_decode(void)
{
    begin("test_mpu_read_decode");
    int fb = g_fail;

    mpu_stage_genuine(0x71);
    imu_cfg_t cfg = { .odr_mhz = 1000000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
    (void)mpu->init(I2CBUS(MPU_ADDR), &cfg);

    /* One 12-byte sample-set pending, no overflow. */
    i2cmock_set_reg(MPU_ADDR, 0x72, 0x00);   /* FIFO_COUNTH */
    i2cmock_set_reg(MPU_ADDR, 0x73, 12);     /* FIFO_COUNTL */
    i2cmock_set_reg(MPU_ADDR, 0x3A, 0x00);   /* INT_STATUS: no overflow */
    i2cmock_set_fifo_reg(MPU_ADDR, 0x74);

    /* Register order (accel first), big-endian: ax ay az gx gy gz. */
    const int16_t ax = 100, ay = 200, az = 300, gx = 10, gy = 20, gz = 30;
    uint8_t w[12] = {
        (uint8_t)(ax >> 8), (uint8_t)ax, (uint8_t)(ay >> 8), (uint8_t)ay,
        (uint8_t)(az >> 8), (uint8_t)az, (uint8_t)(gx >> 8), (uint8_t)gx,
        (uint8_t)(gy >> 8), (uint8_t)gy, (uint8_t)(gz >> 8), (uint8_t)gz,
    };
    i2cmock_fifo_push(MPU_ADDR, w, 12);

    /* TEMP_OUT 3339 → 3339/333.87 + 21 ≈ 31.0 °C. */
    i2cmock_set_reg(MPU_ADDR, 0x41, (uint8_t)(3339 >> 8));
    i2cmock_set_reg(MPU_ADDR, 0x42, (uint8_t)(3339 & 0xFF));

    imu_sample_t buf[8];
    int n = -1;
    EXPECT(mpu->read(I2CBUS(MPU_ADDR), buf, 8, &n) == 0, "read returns 0");
    EXPECT(n == 1, "one sample produced");

    const float as = 9.80665f / 4096.0f;                   /* ±8 g  m/s²/LSB   */
    const float gs = (float)(1.0 / 16.4 * M_PI / 180.0);   /* ±2000 dps rad/s  */

    /* Chip→board remap flips Y and Z. */
    EXPECT_NEAR(buf[0].accel[0],  ax * as, 1e-4, "accel X");
    EXPECT_NEAR(buf[0].accel[1], -ay * as, 1e-4, "accel Y flipped");
    EXPECT_NEAR(buf[0].accel[2], -az * as, 1e-4, "accel Z flipped");
    EXPECT_NEAR(buf[0].gyro[0],   gx * gs, 1e-6, "gyro X");
    EXPECT_NEAR(buf[0].gyro[1],  -gy * gs, 1e-6, "gyro Y flipped");
    EXPECT_NEAR(buf[0].gyro[2],  -gz * gs, 1e-6, "gyro Z flipped");
    EXPECT_NEAR(buf[0].temp_c, 31.0f, 0.01, "temperature decoded");
    EXPECT(buf[0].seq == 0, "seq starts at 0");
    /* No sample timer on this part — the contract requires exactly 0. */
    EXPECT(buf[0].chip_ts == 0, "chip_ts is 0 (has_hw_timestamp false)");
    EXPECT(mpu9250_ops.has_hw_timestamp == false, "ops declares no hw timestamp");

    end(fb);
}

static void test_mpu_read_overflow_and_errors(void)
{
    begin("test_mpu_read_overflow_and_errors");
    int fb = g_fail;

    mpu_stage_genuine(0x71);
    imu_cfg_t cfg = { .odr_mhz = 1000000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
    (void)mpu->init(I2CBUS(MPU_ADDR), &cfg);
    i2cmock_set_fifo_reg(MPU_ADDR, 0x74);

    /* Empty FIFO and no overflow: 0 samples, rc 0 — never -1. */
    i2cmock_set_reg(MPU_ADDR, 0x72, 0x00);
    i2cmock_set_reg(MPU_ADDR, 0x73, 0x00);
    i2cmock_set_reg(MPU_ADDR, 0x3A, 0x00);
    imu_sample_t buf[4];
    int n = -1;
    EXPECT(mpu->read(I2CBUS(MPU_ADDR), buf, 4, &n) == 0, "empty FIFO returns 0");
    EXPECT(n == 0, "no samples produced");

    /* Overflow latched in INT_STATUS bit 4, FIFO still empty. */
    i2cmock_set_reg(MPU_ADDR, 0x3A, 0x10);
    EXPECT(mpu->read(I2CBUS(MPU_ADDR), buf, 4, &n) == 1, "overflow returns 1");

    /* I2C error on the count read. */
    i2cmock_fail_next_xfer();
    EXPECT(mpu->read(I2CBUS(MPU_ADDR), buf, 4, &n) == -1, "I2C error returns -1");

    end(fb);
}

/*
 * Overflow means the part dropped the oldest bytes to make room, and
 * 512 % 12 = 8 — what it dropped is never a whole number of sample-sets.  So
 * the framing is gone, and this FIFO carries no per-word tag to resync from
 * (the ST parts do, which is why they can just keep draining).  Parsing on
 * would put gravity in the accel X slot and the next sample's accel X/Y in
 * the gyro slots, and |a| still reads ~9.8 whenever one real axis is
 * vertical — so nothing downstream would catch it either.  read() must
 * restart the FIFO and discard what is buffered.
 */
static void test_mpu_overflow_discards_and_restarts(void)
{
    begin("test_mpu_overflow_discards_and_restarts");
    int fb = g_fail;

    mpu_stage_genuine(0x71);
    imu_cfg_t cfg = { .odr_mhz = 1000000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
    (void)mpu->init(I2CBUS(MPU_ADDR), &cfg);
    i2cmock_set_fifo_reg(MPU_ADDR, 0x74);

    /* Two whole sample-sets pending AND the overflow flag latched. */
    uint8_t w[24];
    memset(w, 0, sizeof w);
    i2cmock_fifo_push(MPU_ADDR, w, sizeof w);
    i2cmock_set_reg(MPU_ADDR, 0x72, 0x00);   /* FIFO_COUNTH */
    i2cmock_set_reg(MPU_ADDR, 0x73, 24);     /* FIFO_COUNTL */
    i2cmock_set_reg(MPU_ADDR, 0x3A, 0x10);   /* INT_STATUS: overflow */

    imu_sample_t buf[8];
    int n = -1;
    EXPECT(mpu->read(I2CBUS(MPU_ADDR), buf, 8, &n) == 1, "overflow returns 1");
    EXPECT(n == 0, "no samples handed back from a desynchronised FIFO");

    /* The restart must leave the same settled image init() does, or the part
     * is left not batching at all. */
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x23) == 0x78, "FIFO_EN re-enabled");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x6A) == 0x40, "USER_CTRL = FIFO_EN only");

    end(fb);
}

/* ── AK8963 ──────────────────────────────────────────────────────────────── */

static void test_ak_probe(void)
{
    begin("test_ak_probe");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(AK_ADDR, 0x00, 0x48);
    EXPECT(ak->probe(I2CBUS(AK_ADDR)) == 0, "probe accepts WIA 0x48");

    /* An AK09916 would answer 0x09 here — must not be accepted. */
    i2cmock_set_reg(AK_ADDR, 0x00, 0x09);
    EXPECT(ak->probe(I2CBUS(AK_ADDR)) != 0, "probe rejects AK09916 id 0x09");

    i2cmock_set_reg(AK_ADDR, 0x00, 0x48);
    i2cmock_fail_next_xfer();
    EXPECT(ak->probe(I2CBUS(AK_ADDR)) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_ak_init_and_fuse_rom(void)
{
    begin("test_ak_init_and_fuse_rom");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(AK_ADDR, 0x00, 0x48);
    /* Fuse ROM: ASA 128 → x1.000, 160 → x1.125, 96 → x0.875. */
    i2cmock_set_reg(AK_ADDR, 0x10, 128);
    i2cmock_set_reg(AK_ADDR, 0x11, 160);
    i2cmock_set_reg(AK_ADDR, 0x12, 96);

    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
    EXPECT(ak->init(I2CBUS(AK_ADDR), &cfg) == 0, "init succeeds");
    EXPECT(i2cmock_get_reg(AK_ADDR, 0x0A) == 0x16,
           "CNTL1 = 16-bit | continuous mode 2");

    mag_cfg_t slow = { .odr_mhz = 8000, .set_period_s = 0.0f };
    EXPECT(ak->init(I2CBUS(AK_ADDR), &slow) == 0, "init at 8 Hz succeeds");
    EXPECT(i2cmock_get_reg(AK_ADDR, 0x0A) == 0x12,
           "CNTL1 = 16-bit | continuous mode 1");

    end(fb);
}

/*
 * A die with no fuse ROM to read returns ASA as all-zero or all-ones, and the
 * adjustment arithmetic does not object: 0x00 becomes a plausible-looking
 * 0.5x and 0xFF a 1.496x.  A counterfeit part would then produce confidently
 * mis-scaled field values rather than an error.  init() must still succeed —
 * degraded is not impossible, and refusing to start the daemon on a heuristic
 * is the wrong trade — but it must apply no adjustment at all.
 */
static void test_ak_absent_fuse_rom_applies_no_adjustment(void)
{
    begin("test_ak_absent_fuse_rom_applies_no_adjustment");
    int fb = g_fail;

    const int16_t hx = 1000, hy = 2000, hz = 3000;
    uint8_t d[7] = {
        (uint8_t)hx, (uint8_t)(hx >> 8),
        (uint8_t)hy, (uint8_t)(hy >> 8),
        (uint8_t)hz, (uint8_t)(hz >> 8),
        0x00,
    };

    const uint8_t absent[2] = { 0x00, 0xFF };
    for (int k = 0; k < 2; k++) {
        i2cmock_reset();
        i2cmock_set_reg(AK_ADDR, 0x00, 0x48);
        i2cmock_set_reg(AK_ADDR, 0x10, absent[k]);
        i2cmock_set_reg(AK_ADDR, 0x11, absent[k]);
        i2cmock_set_reg(AK_ADDR, 0x12, absent[k]);

        mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
        EXPECT(ak->init(I2CBUS(AK_ADDR), &cfg) == 0,
               "init still succeeds when the fuse ROM is absent");

        i2cmock_set_reg(AK_ADDR, 0x02, 0x01);      /* ST1: DRDY */
        i2cmock_set_regs(AK_ADDR, 0x03, d, 7);

        mag_sample_t out;
        memset(&out, 0, sizeof out);
        EXPECT(ak->read(I2CBUS(AK_ADDR), &out) == 0, "read returns 0");

        /* Same axis mapping as test_ak_read_decode, adjustment forced to 1.0:
         * an unguarded 0.5x would read 150/-75/225, a 1.496x 448.8/-224.4/673. */
        EXPECT_NEAR(out.field[0],  300.00f, 0.01, "field X carries no adjustment");
        EXPECT_NEAR(out.field[1], -150.00f, 0.01, "field Y carries no adjustment");
        EXPECT_NEAR(out.field[2],  450.00f, 0.01, "field Z carries no adjustment");
    }

    end(fb);
}

static void test_ak_read_decode(void)
{
    begin("test_ak_read_decode");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(AK_ADDR, 0x00, 0x48);
    i2cmock_set_reg(AK_ADDR, 0x10, 128);
    i2cmock_set_reg(AK_ADDR, 0x11, 160);
    i2cmock_set_reg(AK_ADDR, 0x12, 96);
    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
    (void)ak->init(I2CBUS(AK_ADDR), &cfg);

    i2cmock_set_reg(AK_ADDR, 0x02, 0x01);      /* ST1: DRDY */

    /* HXL..HZH little-endian, then ST2 with HOFL clear. */
    const int16_t hx = 1000, hy = 2000, hz = 3000;
    uint8_t d[7] = {
        (uint8_t)hx, (uint8_t)(hx >> 8),
        (uint8_t)hy, (uint8_t)(hy >> 8),
        (uint8_t)hz, (uint8_t)(hz >> 8),
        0x00,
    };
    i2cmock_set_regs(AK_ADDR, 0x03, d, 7);

    mag_sample_t out;
    memset(&out, 0, sizeof out);
    EXPECT(ak->read(I2CBUS(AK_ADDR), &out) == 0, "read returns 0");
    EXPECT(out.valid, "sample marked valid");

    /*
     * Fuse-ROM adjustment then the axis mapping.  The AK8963 is rotated
     * relative to the MPU's gyro/accel (AK X = MPU Y, AK Y = MPU X,
     * AK Z = -MPU Z), so in the board frame:
     *   X_board = +hy·adjY·0.15 = 2000 × 1.125 × 0.15 = +337.5 µT
     *   Y_board = -hx·adjX·0.15 = -(1000 × 1.000 × 0.15) = -150.0 µT
     *   Z_board = +hz·adjZ·0.15 = 3000 × 0.875 × 0.15 = +393.75 µT
     * Getting this backwards is the defect `imud-imutest --spin` exists to
     * catch on hardware; here it is pinned exactly.
     */
    EXPECT_NEAR(out.field[0],  337.50f, 0.01, "field X = +hy (fuse-adjusted)");
    EXPECT_NEAR(out.field[1], -150.00f, 0.01, "field Y = -hx (fuse-adjusted)");
    EXPECT_NEAR(out.field[2],  393.75f, 0.01, "field Z = +hz (fuse-adjusted)");
    EXPECT(out.wall_ns != 0, "wall_ns stamped");

    /* HOFL set: unreliable sample, but not an I2C fault — must return 1. */
    d[6] = 0x08;
    i2cmock_set_regs(AK_ADDR, 0x03, d, 7);
    EXPECT(ak->read(I2CBUS(AK_ADDR), &out) == 1, "HOFL overflow returns 1");

    /* DRDY never asserts: no data yet, still not an error. */
    d[6] = 0x00;
    i2cmock_set_regs(AK_ADDR, 0x03, d, 7);
    i2cmock_set_reg(AK_ADDR, 0x02, 0x00);
    EXPECT(ak->read(I2CBUS(AK_ADDR), &out) == 1, "DRDY timeout returns 1, not -1");

    i2cmock_set_reg(AK_ADDR, 0x02, 0x01);
    i2cmock_fail_next_xfer();
    EXPECT(ak->read(I2CBUS(AK_ADDR), &out) == -1, "I2C error returns -1");

    end(fb);
}

/* ════════════════════════════════════════════════════════════════════════════
 * The six drivers that had NO functional coverage
 *
 * Covers every registered driver rather than the four originally wired up; the
 * CI coverage job put src/drivers/ at 48% with these six at literally 0.0%.
 * They are also the six still flagged `experimental` and awaiting bench
 * validation, so until now nothing had ever executed a line of them — a
 * transposed register or a sign error would have waited for hardware.
 *
 * These do not replace hardware validation: a mock cannot tell
 * you the chip-to-board axis remap matches the physical part. What it does
 * catch is the register map, the full-scale encoding, the byte order, and the
 * return-code contract — the parts that are transcribed from a datasheet and
 * therefore the parts most likely to be wrong.
 * ════════════════════════════════════════════════════════════════════════════ */

extern const imu_ops_t lsm6dso_ops;
extern const imu_ops_t lsm6dsox_ops;
extern const imu_ops_t icm42688p_ops;
extern const imu_ops_t icm20948_ops;
extern const mag_ops_t ak09916_ops;
extern const mag_ops_t lis3mdl_ops;
extern const mag_ops_t lis2mdl_ops;
extern const mag_ops_t rm3100_ops;

/* ── LSM6DSO (near-twin of the ISM330DHCX) ───────────────────────────────── */

#define LSM_ADDR 0x6A

/*
 * ts_tick_ns_actual: the ST parts read INTERNAL_FREQ_FINE (0x63) and hand back
 * THIS die's timer period instead of the datasheet typical.
 *
 * Worth a test of its own because the value scales every per-sample dt until
 * ts_anchor_t has measured the real period, and because the sign is easy to get
 * backwards: a POSITIVE trim means a faster oscillator, so a SHORTER tick.
 */
static void test_st_freq_fine_tick(void)
{
    begin("test_st_freq_fine_tick");
    int fb = g_fail;

    /* All three ST descriptors, because lsm6dso.c backs two of them and a
     * hook wired into only one would be invisible from either driver file. */
    struct { const imu_ops_t *ops; int addr; } parts[] = {
        { ism,           ISM_ADDR },
        { &lsm6dso_ops,  LSM_ADDR },
        { &lsm6dsox_ops, LSM_ADDR },
    };

    for (unsigned p = 0; p < sizeof parts / sizeof parts[0]; p++) {
        const imu_ops_t *o = parts[p].ops;
        int a = parts[p].addr;

        EXPECT(o->ts_tick_ns_actual != NULL, "ST part exposes ts_tick_ns_actual");
        if (!o->ts_tick_ns_actual) continue;

        i2cmock_reset();

        /* Exactly typical: the hook must not perturb a part with no trim. */
        i2cmock_set_reg(a, 0x63, 0x00);
        EXPECT(o->ts_tick_ns_actual(I2CBUS(a)) == 25000,
               "FREQ_FINE 0 leaves the declared 25000 ns tick");

        /*
         * +27 is what the reference ISM330DHCX on the bench declares.
         * 25000 / (1 + 0.0015*27) = 25000 / 1.0405 = 24026.91 -> 24027, which
         * is the 24029 ns the wall-clock ratio implied, from the other side.
         */
        i2cmock_set_reg(a, 0x63, 27);
        EXPECT(o->ts_tick_ns_actual(I2CBUS(a)) == 24027,
               "FREQ_FINE +27 gives a 24027 ns tick");

        /* Negative trim: slower oscillator, longer tick.  0xE5 = -27. */
        i2cmock_set_reg(a, 0x63, 0xE5);
        EXPECT(o->ts_tick_ns_actual(I2CBUS(a)) == 26055,
               "FREQ_FINE -27 gives a 26055 ns tick");

        /*
         * Both ends of the field.  These are the bounds the header claims make
         * a range check unreachable, so they belong in a test rather than only
         * in a comment: +127 -> /1.1905, -128 -> /0.808.
         */
        i2cmock_set_reg(a, 0x63, 0x7F);
        EXPECT(o->ts_tick_ns_actual(I2CBUS(a)) == 21000,
               "FREQ_FINE +127 saturates at a 21000 ns tick");
        i2cmock_set_reg(a, 0x63, 0x80);
        EXPECT(o->ts_tick_ns_actual(I2CBUS(a)) == 30941,
               "FREQ_FINE -128 saturates at a 30941 ns tick");

        /* A failed read must return the "no answer" sentinel, so the caller
         * keeps the declared constant.  It must not invent a period, and it
         * must not be mistaken for a real one — imu.c and imutest both treat
         * 0 as "leave ts_tick_ns alone". */
        i2cmock_set_reg(a, 0x63, 27);
        i2cmock_fail_next_xfer();
        EXPECT(o->ts_tick_ns_actual(I2CBUS(a)) == 0,
               "a failed FREQ_FINE read returns 0, not a guess");
    }

    end(fb);
}

/*
 * init() must park the FIFO in Bypass before reconfiguring it.
 *
 * DS13012 §6.5.1 — "in Bypass mode the FIFO is not operational and it remains
 * empty" — and §6.5.2 — "to reset FIFO content, Bypass mode should be selected
 * by writing FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0]) to '000'".  Bypass is the only
 * thing that clears the buffer, so writing Continuous straight over Continuous
 * flushes nothing and a second init() inherits whatever the first accumulated.
 * imud-imutest's imu.init.idempotent caught that on the bench, and the ST pair
 * was the only FIFO family here that did not already flush (icm42688p uses
 * FIFO_FLUSH, icm20948 and mpu925x pulse FIFO_RST).
 *
 * Two independent properties, because neither alone is the invariant and the
 * settled register image shows neither — it holds only the final byte:
 * ORDERING (the stop comes before the reconfiguration and the restart after
 * it) and VALUE (what was written really was mode 000, not some other mode).
 */
static void test_st_init_flushes_fifo(void)
{
    begin("test_st_init_flushes_fifo");
    int fb = g_fail;

    /* All three ST descriptors: lsm6dso.c backs two of them, and ism330dhcx.c
     * carries its own copy of the sequence, so a fix applied to one file only
     * has to be visible from here. */
    struct { const imu_ops_t *ops; int addr; } parts[] = {
        { ism,           ISM_ADDR },
        { &lsm6dso_ops,  LSM_ADDR },
        { &lsm6dsox_ops, LSM_ADDR },
    };
    const imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4,
                            .gyro_dps = 500, .fifo_wm = 64 };

    for (unsigned p = 0; p < sizeof parts / sizeof parts[0]; p++) {
        const imu_ops_t *o = parts[p].ops;
        int a = parts[p].addr;

        i2cmock_reset();
        i2cmock_set_reg(a, 0x20, 0x00);      /* OUT_TEMP, so last_temp reads */
        i2cmock_set_reg(a, 0x21, 0x00);
        EXPECT(o->init(I2CBUS(a), &cfg) == 0, "init succeeds");

        /* FIFO_CTRL4 (0x0A) twice: once before the FIFO_CTRL1 (0x07) watermark
         * write, once after the FIFO_CTRL3 (0x09) batch-rate write.  The whole
         * reconfiguration therefore happens with the FIFO stopped. */
        int first4 = -1, last4 = -1, first1 = -1, last3 = -1;
        uint32_t nw = i2cmock_writes(a);
        for (uint32_t i = 0; i < nw; i++) {
            int reg = i2cmock_write_at(a, i);
            if (reg == 0x0A) { if (first4 < 0) first4 = (int)i; last4 = (int)i; }
            if (reg == 0x07 && first1 < 0) first1 = (int)i;
            if (reg == 0x09) last3 = (int)i;
        }
        EXPECT(first4 >= 0 && last4 > first4, "FIFO_CTRL4 is written twice");
        EXPECT(first1 > first4, "the FIFO is stopped before it is reconfigured");
        EXPECT(last4 > last3, "the mode write that restarts the FIFO comes last");

        /* The settled image is still Continuous — stopping it must not be the
         * end state.  Mode bits rather than the whole byte: DEC_TS_BATCH in
         * the top bits is a watermark-dependent choice tested elsewhere. */
        EXPECT((i2cmock_get_reg(a, 0x0A) & 0x07) == 0x06,
               "FIFO_CTRL4 settles in continuous mode");

        /*
         * VALUE.  Fail the FIFO_CTRL1 write so init() aborts with the
         * intermediate image still on the part, then read what FIFO_CTRL4 was
         * holding.  Seeded with a running Continuous value first, so a driver
         * that skipped the stop would leave 0xE6 here and fail — the register
         * starting at 0x00 would let a missing write pass.
         */
        i2cmock_reset();
        i2cmock_set_reg(a, 0x20, 0x00);
        i2cmock_set_reg(a, 0x21, 0x00);
        i2cmock_set_reg(a, 0x0A, 0xE6);      /* as a previous init left it */
        i2cmock_fail_write_to(a, 0x07, 1);
        EXPECT(o->init(I2CBUS(a), &cfg) < 0, "init reports the failed write");
        EXPECT(i2cmock_get_reg(a, 0x0A) == 0x00,
               "FIFO_CTRL4 held Bypass (mode 000) across the reconfiguration");
        i2cmock_fail_write_to(a, -1, 0);     /* disarm */
    }

    end(fb);
}

static void lsm_push_word(uint8_t tag, int16_t x, int16_t y, int16_t z)
{
    uint8_t w[7] = {
        (uint8_t)(tag << 3),
        (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF),
        (uint8_t)(z & 0xFF), (uint8_t)((z >> 8) & 0xFF),
    };
    i2cmock_fifo_push(LSM_ADDR, w, 7);
}

/* The two above with an explicit TAG_CNT slot, for the timestamp test. */
static void lsm_push_word_cnt(uint8_t tag, uint8_t tag_cnt,
                              int16_t x, int16_t y, int16_t z)
{
    uint8_t w[7] = {
        (uint8_t)((tag << 3) | ((tag_cnt & 0x03) << 1)),
        (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF),
        (uint8_t)(z & 0xFF), (uint8_t)((z >> 8) & 0xFF),
    };
    i2cmock_fifo_push(LSM_ADDR, w, 7);
}

static void lsm_push_ts_word(uint8_t tag_cnt, uint32_t ts)
{
    uint8_t w[7] = {
        (uint8_t)((ST_TAG_TIMESTAMP << 3) | ((tag_cnt & 0x03) << 1)),
        (uint8_t)(ts & 0xFF), (uint8_t)((ts >> 8) & 0xFF),
        (uint8_t)((ts >> 16) & 0xFF), (uint8_t)((ts >> 24) & 0xFF),
        0, 0,
    };
    i2cmock_fifo_push(LSM_ADDR, w, 7);
}

/*
 * The batched-timestamp wiring on the OTHER driver that uses st_fifo_ts.h.
 *
 * The header is shared but the calls into it are not: lsm6dso.c has its own
 * drain loop, and a note_word/note_set left out or misplaced there would be
 * invisible from the ism330dhcx tests, which is exactly the kind of
 * copy-paste gap two near-identical drivers produce.  So this proves the
 * anchoring works here too, and leaves the corner cases to the ism suite.
 */
static void test_lsm_batched_timestamp(void)
{
    begin("test_lsm_batched_timestamp");
    int fb = g_fail;
    const imu_ops_t *d = &lsm6dso_ops;
    imu_sample_t buf[8];
    int n = -1;

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)d->init(I2CBUS(LSM_ADDR), &cfg);

    /* 208 Hz → 40000/208 = 192 ticks per sample. */
    const uint32_t T = 0x00080000u;
    i2cmock_set_reg(LSM_ADDR, 0x3A, 5);      /* 4 sample words + 1 timestamp */
    i2cmock_set_reg(LSM_ADDR, 0x3B, 0);
    i2cmock_set_fifo_range(LSM_ADDR, 0x78, 0x7E);
    for (int i = 0; i < 4; i++)              /* TIMESTAMP0..3: a late read */
        i2cmock_set_reg(LSM_ADDR, (uint8_t)(0x40 + i),
                        (uint8_t)(((T + 192 + 5000) >> (8 * i)) & 0xFF));

    /*
     * Both pairs, then the word dating slot 1 — deliberately AFTER the slot
     * it names.  That ordering is the one that needs TAG_CNT: placing the
     * word by position would put it on the set still being assembled, one
     * sample late.  The "word first" ordering resolves correctly even with
     * TAG_CNT ignored, so testing only that would leave note_set unexercised
     * here, which is the whole reason this test exists.
     */
    lsm_push_word_cnt(TAG_ACCEL_NC, 0, 100, 200, 300);
    lsm_push_word_cnt(TAG_GYRO_NC,  0, 10, 20, 30);
    lsm_push_word_cnt(TAG_ACCEL_NC, 1, 100, 200, 300);
    lsm_push_word_cnt(TAG_GYRO_NC,  1, 10, 20, 30);
    lsm_push_ts_word(1, T);

    EXPECT(d->read(I2CBUS(LSM_ADDR), buf, 8, &n) == 0 && n == 2,
           "two sample-sets decode alongside a timestamp word");
    EXPECT(buf[1].chip_ts == T,
           "the slot the word names carries its value, not the late read");
    EXPECT(buf[0].chip_ts == T - 192u, "older sample steps back one period");

    end(fb);
}

static void test_lsm_probe(void)
{
    begin("test_lsm_probe");
    int fb = g_fail;
    const imu_ops_t *d = &lsm6dso_ops;

    i2cmock_reset();
    /* This driver backs BOTH registry names, so it must accept either
     * WHO_AM_I — rejecting 0x6D would silently break the lsm6dsox entry. */
    i2cmock_set_reg(LSM_ADDR, 0x0F, 0x6C);
    EXPECT(d->probe(I2CBUS(LSM_ADDR)) == 0, "probe accepts LSM6DSO (0x6C)");
    i2cmock_set_reg(LSM_ADDR, 0x0F, 0x6D);
    EXPECT(d->probe(I2CBUS(LSM_ADDR)) == 0, "probe accepts LSM6DSOX (0x6D)");

    i2cmock_set_reg(LSM_ADDR, 0x0F, 0x6B);   /* ISM330DHCX's ID */
    EXPECT(d->probe(I2CBUS(LSM_ADDR)) != 0, "probe rejects a different ST part");

    i2cmock_set_reg(LSM_ADDR, 0x0F, 0x6C);
    i2cmock_fail_next_xfer();
    EXPECT(d->probe(I2CBUS(LSM_ADDR)) != 0, "probe fails on I2C error");
    end(fb);
}

static void test_lsm_init_registers(void)
{
    begin("test_lsm_init_registers");
    int fb = g_fail;
    const imu_ops_t *d = &lsm6dso_ops;

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = 833000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 64 };
    EXPECT(d->init(I2CBUS(LSM_ADDR), &cfg) == 0, "init succeeds");

    /* ODR 833 → 0x7; ±8 g → 0x0C; the 0x02 bit is LPF2_XL_EN. */
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x10) == (uint8_t)((0x7 << 4) | 0x0C | 0x02),
           "CTRL1_XL = odr|accel FS|LPF2");
    /* ±2000 dps → 0x0C. */
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x11) == (uint8_t)((0x7 << 4) | 0x0C),
           "CTRL2_G = odr|gyro FS");
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x12) == 0x44, "CTRL3_C = BDU|IF_INC");

    /* Watermark is in FIFO WORDS and one sample-set is two words (accel+gyro),
     * so the register holds 2x the configured sample watermark. */
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x07) == (uint8_t)(128 & 0xFF),
           "FIFO_CTRL1 = watermark*2 low byte");
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x08) == 0, "FIFO_CTRL2 high bit clear at 128");
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x0A) == 0xE6,
           "FIFO_CTRL4 = DEC_TS_BATCH/32 + 12.5 Hz temp + continuous");
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x0D) == 0x08, "INT1_CTRL = FIFO threshold");

    /* A watermark past the 511-word cap must clamp, not wrap. */
    i2cmock_reset();
    imu_cfg_t big = { .odr_mhz = 833000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 400 };
    EXPECT(d->init(I2CBUS(LSM_ADDR), &big) == 0, "init with an oversized watermark");
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x07) == (uint8_t)(511 & 0xFF) &&
           i2cmock_get_reg(LSM_ADDR, 0x08) == 0x01,
           "watermark clamps to 511 words");

    i2cmock_reset();
    i2cmock_fail_next_xfer();
    EXPECT(d->init(I2CBUS(LSM_ADDR), &cfg) != 0, "init fails on I2C error");
    end(fb);
}

static void test_lsm_read_decode(void)
{
    begin("test_lsm_read_decode");
    int fb = g_fail;
    const imu_ops_t *d = &lsm6dso_ops;

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)d->init(I2CBUS(LSM_ADDR), &cfg);

    i2cmock_set_reg(LSM_ADDR, 0x3A, 3);      /* three words queued */
    i2cmock_set_reg(LSM_ADDR, 0x3B, 0);      /* no overflow */
    i2cmock_set_fifo_range(LSM_ADDR, 0x78, 0x7E);

    lsm_push_word(0x03, 1280, 0, 0);         /* temp: 1280/256 + 25 = 30 °C */
    lsm_push_word(0x02, 100, 200, 300);      /* accel */
    lsm_push_word(0x01, 10, 20, 30);         /* gyro  */

    imu_sample_t buf[8];
    int n = -1;
    EXPECT(d->read(I2CBUS(LSM_ADDR), buf, 8, &n) == 0, "read returns 0");
    EXPECT(n == 1, "one sample from the accel+gyro pair");

    const float as = 0.122e-3f * 9.80665f;
    const float gs = 17.5f * (float)(M_PI / 180.0 / 1000.0);
    EXPECT_NEAR(buf[0].accel[0],  100 * as, 1e-4, "accel X");
    EXPECT_NEAR(buf[0].accel[1], -200 * as, 1e-4, "accel Y flipped to starboard");
    EXPECT_NEAR(buf[0].accel[2], -300 * as, 1e-4, "accel Z flipped to down");
    EXPECT_NEAR(buf[0].gyro[0],   10 * gs, 1e-6, "gyro X");
    EXPECT_NEAR(buf[0].gyro[1],  -20 * gs, 1e-6, "gyro Y flipped");
    EXPECT_NEAR(buf[0].gyro[2],  -30 * gs, 1e-6, "gyro Z flipped");
    EXPECT_NEAR(buf[0].temp_c, 30.0f, 1e-3, "temperature from the FIFO temp word");
    end(fb);
}

static void test_lsm_read_overflow_and_errors(void)
{
    begin("test_lsm_read_overflow_and_errors");
    int fb = g_fail;
    const imu_ops_t *d = &lsm6dso_ops;

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)d->init(I2CBUS(LSM_ADDR), &cfg);

    /* Empty FIFO with the overflow flag set: 1 (not an error), zero samples. */
    i2cmock_set_reg(LSM_ADDR, 0x3A, 0);
    i2cmock_set_reg(LSM_ADDR, 0x3B, 0x40);
    imu_sample_t buf[4];
    int n = -1;
    EXPECT(d->read(I2CBUS(LSM_ADDR), buf, 4, &n) == 1, "overflow reports 1, not -1");
    EXPECT(n == 0, "and produces no samples");

    /* Plain empty FIFO: 0 samples, success. */
    i2cmock_set_reg(LSM_ADDR, 0x3B, 0x00);
    n = -1;
    EXPECT(d->read(I2CBUS(LSM_ADDR), buf, 4, &n) == 0, "empty FIFO returns 0");
    EXPECT(n == 0, "no samples");

    /* A bus fault is the ONLY thing allowed to return -1: the reader counts
     * those toward its error-reset threshold. */
    i2cmock_fail_next_xfer();
    EXPECT(d->read(I2CBUS(LSM_ADDR), buf, 4, &n) == -1, "I2C error returns -1");
    end(fb);
}

/* ── ICM-42688-P ─────────────────────────────────────────────────────────── */

#define ICM42_ADDR 0x68

/* One 16-byte FIFO packet: header, big-endian accel/gyro, temp at [13]. */
static void icm42_push_packet(uint8_t hdr, int16_t ax, int16_t ay, int16_t az,
                              int16_t gx, int16_t gy, int16_t gz, int8_t temp)
{
    uint8_t p[16];
    memset(p, 0, sizeof p);
    p[0] = hdr;
    const int16_t v[6] = { ax, ay, az, gx, gy, gz };
    for (int i = 0; i < 6; i++) {
        p[1 + i * 2] = (uint8_t)((v[i] >> 8) & 0xFF);   /* big endian */
        p[2 + i * 2] = (uint8_t)(v[i] & 0xFF);
    }
    p[13] = (uint8_t)temp;
    i2cmock_fifo_push(ICM42_ADDR, p, 16);
}

/*
 * The same packet with the chip's own timestamp in [14:15] and the header bits
 * that say so (HEADER_TIMESTAMP_FSYNC = 10, DS-000347 §6.2).  Only the header
 * and the stamp matter to the timestamp path, so the sample payload is fixed.
 *
 * Takes the FULL counter value and truncates, so callers can write the absolute
 * time a sample was taken and let the chip's 16-bit field do to it exactly what
 * the silicon would — which is the behaviour the unwrap has to undo.
 */
static void icm42_push_stamped(uint32_t ts)
{
    uint8_t p[16];
    memset(p, 0, sizeof p);
    p[0]  = 0x68;                            /* accel | gyro | ODR timestamp */
    p[14] = (uint8_t)((ts >> 8) & 0xFF);     /* big endian, like everything here */
    p[15] = (uint8_t)(ts & 0xFF);
    i2cmock_fifo_push(ICM42_ADDR, p, 16);
}

/* Stage the FIFO byte count and port, and Bank 1 TMSTVAL (20-bit, LE regs). */
static void icm42_stage_fifo(int n_pkts, uint32_t tmstval)
{
    i2cmock_set_reg(ICM42_ADDR, 0x2E, (uint8_t)((n_pkts * 16) >> 8));
    i2cmock_set_reg(ICM42_ADDR, 0x2F, (uint8_t)((n_pkts * 16) & 0xFF));
    i2cmock_set_fifo_reg(ICM42_ADDR, 0x30);
    i2cmock_set_reg(ICM42_ADDR, 0x62, (uint8_t)(tmstval & 0xFF));
    i2cmock_set_reg(ICM42_ADDR, 0x63, (uint8_t)((tmstval >> 8) & 0xFF));
    i2cmock_set_reg(ICM42_ADDR, 0x64, (uint8_t)((tmstval >> 16) & 0x0F));
}

static void test_icm42_probe_and_init(void)
{
    begin("test_icm42_probe_and_init");
    int fb = g_fail;
    const imu_ops_t *d = &icm42688p_ops;

    i2cmock_reset();
    i2cmock_set_reg(ICM42_ADDR, 0x75, 0x47);
    EXPECT(d->probe(I2CBUS(ICM42_ADDR)) == 0, "probe accepts WHO_AM_I 0x47");
    i2cmock_set_reg(ICM42_ADDR, 0x75, 0x67);   /* ICM-42605 */
    EXPECT(d->probe(I2CBUS(ICM42_ADDR)) != 0, "probe rejects a sibling part");
    i2cmock_set_reg(ICM42_ADDR, 0x75, 0x47);
    i2cmock_fail_next_xfer();
    EXPECT(d->probe(I2CBUS(ICM42_ADDR)) != 0, "probe fails on I2C error");

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = 1000000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 64 };
    EXPECT(d->init(I2CBUS(ICM42_ADDR), &cfg) == 0, "init succeeds");

    /* FS bits live in [7:5]: ±2000 dps → 0x0, ±8 g → 0x1. */
    EXPECT((i2cmock_get_reg(ICM42_ADDR, 0x4F) >> 5) == 0x0, "GYRO_CONFIG0 FS = 2000 dps");
    EXPECT((i2cmock_get_reg(ICM42_ADDR, 0x50) >> 5) == 0x1, "ACCEL_CONFIG0 FS = 8 g");
    EXPECT(i2cmock_get_reg(ICM42_ADDR, 0x4E) == 0x0F, "PWR_MGMT0 = accel+gyro low-noise");
    EXPECT(i2cmock_get_reg(ICM42_ADDR, 0x54) == 0x11, "TMST_CONFIG = timestamp to regs");
    EXPECT(i2cmock_get_reg(ICM42_ADDR, 0x64) == 0x00,
           "INT_ASYNC_RESET cleared after reset (datasheet 12.6)");
    EXPECT(i2cmock_get_reg(ICM42_ADDR, 0x60) == 64, "FIFO watermark low byte");
    EXPECT(i2cmock_get_reg(ICM42_ADDR, 0x65) == 0x04, "INT_SOURCE0 = FIFO threshold");
    end(fb);
}

static void test_icm42_read_decode(void)
{
    begin("test_icm42_read_decode");
    int fb = g_fail;
    const imu_ops_t *d = &icm42688p_ops;

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = 1000000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)d->init(I2CBUS(ICM42_ADDR), &cfg);

    /* FIFO_COUNT is BIG endian here (COUNTH first): 32 bytes = 2 packets. */
    i2cmock_set_reg(ICM42_ADDR, 0x2E, 0x00);
    i2cmock_set_reg(ICM42_ADDR, 0x2F, 32);
    i2cmock_set_fifo_reg(ICM42_ADDR, 0x30);

    /* header bit6 = accel present, bit5 = gyro present. */
    icm42_push_packet(0x60, 100, 200, 300, 10, 20, 30, 5);
    icm42_push_packet(0x60, 1, 2, 3, 4, 5, 6, 5);

    imu_sample_t buf[8];
    int n = -1;
    EXPECT(d->read(I2CBUS(ICM42_ADDR), buf, 8, &n) == 0, "read returns 0");
    EXPECT(n == 2, "two packets decoded");

    const float as = 4.0f * 9.80665f / 32768.0f;
    const float gs = 500.0f * (float)(M_PI / 180.0) / 32768.0f;
    EXPECT_NEAR(buf[0].accel[0],  100 * as, 1e-4, "accel X (big-endian decode)");
    EXPECT_NEAR(buf[0].accel[1], -200 * as, 1e-4, "accel Y flipped");
    EXPECT_NEAR(buf[0].accel[2], -300 * as, 1e-4, "accel Z flipped");
    EXPECT_NEAR(buf[0].gyro[0],   10 * gs, 1e-6, "gyro X");
    EXPECT_NEAR(buf[0].gyro[1],  -20 * gs, 1e-6, "gyro Y flipped");
    EXPECT_NEAR(buf[0].gyro[2],  -30 * gs, 1e-6, "gyro Z flipped");
    EXPECT_NEAR(buf[0].temp_c, 30.0f, 1e-3, "temp = int8 + 25");
    EXPECT(buf[1].seq == buf[0].seq + 1, "seq increments across packets");

    /*
     * Chip-timer arithmetic.  The counter's LSB is 32/30 µs, not the 1 µs
     * TMST_RES nominally selects (DS-000347 §12.7), so one second is 937500
     * ticks and a 1 kHz sample period is 937 of them — not 1000.  Both halves
     * have to agree or the daemon's dt is wrong by 6.7% until ts_anchor_t
     * measures the real period, so assert them together.
     */
    EXPECT(icm42688p_ops.ts_tick_ns == 1067, "ts_tick_ns is 32/30 µs, not 1 µs");
    EXPECT((uint32_t)(buf[1].chip_ts - buf[0].chip_ts) == 937,
           "adjacent samples are 937 ticks apart at 1 kHz");

    /* Empty FIFO. */
    i2cmock_set_reg(ICM42_ADDR, 0x2E, 0);
    i2cmock_set_reg(ICM42_ADDR, 0x2F, 0);
    n = -1;
    EXPECT(d->read(I2CBUS(ICM42_ADDR), buf, 8, &n) == 0 && n == 0, "empty FIFO → 0");

    i2cmock_fail_next_xfer();
    EXPECT(d->read(I2CBUS(ICM42_ADDR), buf, 8, &n) == -1, "I2C error returns -1");
    end(fb);
}

/*
 * The ICM's per-packet FIFO timestamps (bytes [14:15]).
 *
 * Why this path exists: the fallback reads TMSTVAL after the drain and calls
 * that the newest sample's time, but the register reads *now* and the lag to
 * the newest sample moves with bus and scheduler jitter — which is how the
 * The bench sees 2-4 chip_ts reversals per 5 s on the ST twin of
 * this design.  The chip stamps each packet at the sample instant instead, and
 * the driver was already reading and discarding those two bytes.
 *
 * The awkward part is that the field is 16 bits and repeats every 65536 ticks
 * (~70 ms), so the epoch has to come from somewhere.  These tests pin the
 * unwrap, and pin every condition under which the driver refuses it.
 */
static void test_icm42_batched_timestamps(void)
{
    begin("test_icm42_batched_timestamps");
    int fb = g_fail;
    const imu_ops_t *d = &icm42688p_ops;
    imu_sample_t buf[16];
    int n = -1;

    /* ── The ordinary case: four stamped packets, no wrap ───────────────── */
    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = 1000000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)d->init(I2CBUS(ICM42_ADDR), &cfg);

    /* TMSTVAL "now" = 0x05000, samples at 937-tick spacing ending 500 before. */
    icm42_stage_fifo(4, 0x05000);
    icm42_push_stamped(0x05000 - 500 - 937 * 3);
    icm42_push_stamped(0x05000 - 500 - 937 * 2);
    icm42_push_stamped(0x05000 - 500 - 937 * 1);
    icm42_push_stamped(0x05000 - 500);

    EXPECT(d->read(I2CBUS(ICM42_ADDR), buf, 16, &n) == 0 && n == 4,
           "four stamped packets decode");
    EXPECT(buf[3].chip_ts == 0x05000u - 500u,
           "newest sample carries the chip's own stamp, not the read time");
    EXPECT(buf[0].chip_ts == 0x05000u - 500u - 937u * 3u, "oldest likewise");
    EXPECT(buf[1].chip_ts - buf[0].chip_ts == 937u &&
           buf[2].chip_ts - buf[1].chip_ts == 937u,
           "spacing comes from the stamps, not from ticks_per_sample");

    /* ── A 16-bit repeat inside one burst ───────────────────────────────── */
    /* The field rolls through zero mid-burst; the unwrap must carry the epoch
     * down rather than letting chip_ts leap backwards by 65536. */
    i2cmock_reset();
    (void)d->init(I2CBUS(ICM42_ADDR), &cfg);
    icm42_stage_fifo(4, 0x10400);
    icm42_push_stamped(0xFE00);   /* below the boundary: epoch 0x0____ */
    icm42_push_stamped(0xFF00);
    icm42_push_stamped(0x0000);   /* rolls */
    icm42_push_stamped(0x0100);

    n = -1;
    EXPECT(d->read(I2CBUS(ICM42_ADDR), buf, 16, &n) == 0 && n == 4,
           "burst spanning a 16-bit repeat decodes");
    EXPECT(buf[0].chip_ts == 0x0FE00u && buf[1].chip_ts == 0x0FF00u,
           "pre-roll samples sit in the lower epoch");
    EXPECT(buf[2].chip_ts == 0x10000u && buf[3].chip_ts == 0x10100u,
           "post-roll samples carry into the next epoch");
    for (int i = 1; i < 4; i++)
        EXPECT((int32_t)(buf[i].chip_ts - buf[i - 1].chip_ts) > 0,
               "chip_ts strictly increases across the repeat");

    /* ── Refusals.  Each must fall back, never emit a wrong stamp ───────── */

    /* (a) No ODR-timestamp bits in the header. */
    i2cmock_reset();
    (void)d->init(I2CBUS(ICM42_ADDR), &cfg);
    icm42_stage_fifo(2, 0x05000);
    icm42_push_packet(0x60, 1, 2, 3, 4, 5, 6, 0);
    icm42_push_packet(0x60, 1, 2, 3, 4, 5, 6, 0);
    n = -1;
    (void)d->read(I2CBUS(ICM42_ADDR), buf, 16, &n);
    EXPECT(n == 2 && buf[1].chip_ts == 0x05000u,
           "header without the timestamp bits falls back to TMSTVAL");

    /* (b) Mixed: one unstamped packet poisons the whole burst, because a
     *     burst half-stamped and half-inferred has a seam nothing can find. */
    i2cmock_reset();
    (void)d->init(I2CBUS(ICM42_ADDR), &cfg);
    icm42_stage_fifo(2, 0x05000);
    icm42_push_stamped(0x04000);
    icm42_push_packet(0x60, 1, 2, 3, 4, 5, 6, 0);
    n = -1;
    (void)d->read(I2CBUS(ICM42_ADDR), buf, 16, &n);
    EXPECT(n == 2 && buf[1].chip_ts == 0x05000u,
           "one unstamped packet makes the whole burst fall back");

    /* (c) A stuck or zeroed field repeats a value, which is not a clock. */
    i2cmock_reset();
    (void)d->init(I2CBUS(ICM42_ADDR), &cfg);
    icm42_stage_fifo(3, 0x05000);
    icm42_push_stamped(0x4444);
    icm42_push_stamped(0x4444);
    icm42_push_stamped(0x4444);
    n = -1;
    (void)d->read(I2CBUS(ICM42_ADDR), buf, 16, &n);
    EXPECT(n == 3 && buf[2].chip_ts == 0x05000u,
           "a stamp that does not advance is rejected");

    /* (d) The newest sample too far behind the TMSTVAL read: the epoch seed
     *     would be a coin flip, so the burst is refused rather than guessed. */
    i2cmock_reset();
    (void)d->init(I2CBUS(ICM42_ADDR), &cfg);
    icm42_stage_fifo(2, 0x20000);
    icm42_push_stamped(0x20000u - 60000u - 937u);
    icm42_push_stamped(0x20000u - 60000u);   /* 60000 > TS_FIFO_MAX_LAG */
    n = -1;
    (void)d->read(I2CBUS(ICM42_ADDR), buf, 16, &n);
    EXPECT(n == 2 && buf[1].chip_ts == 0x20000u,
           "a newest sample nearly a full repeat old is rejected");

    /*
     * (e) 12 Hz: 78125 ticks per sample is MORE than the 65536-tick repeat, so
     *     two adjacent samples can read as adjacent when they are in fact a
     *     repeat apart.  Refused for the whole session, at init.
     *
     *     The numbers are chosen so this is the ONLY thing that refuses it:
     *     the newest sample sits 1000 ticks behind the TMSTVAL read, well
     *     inside the epoch-lag bound, and the two stamps do increase.  Without
     *     the init gate the unwrap succeeds and confidently reports the older
     *     sample 12589 ticks back instead of 78125 — the exact silent error
     *     the gate exists to prevent, which is why it is worth a case of its
     *     own rather than leaning on the lag check to catch it by accident.
     */
    i2cmock_reset();
    imu_cfg_t slow = { .odr_mhz = 12500, .accel_g = 4, .gyro_dps = 500,
                       .fifo_wm = 64 };
    (void)d->init(I2CBUS(ICM42_ADDR), &slow);
    icm42_stage_fifo(2, 0x30000);
    icm42_push_stamped(0x30000u - 1000u - 75000u);
    icm42_push_stamped(0x30000u - 1000u);
    n = -1;
    (void)d->read(I2CBUS(ICM42_ADDR), buf, 16, &n);
    EXPECT(n == 2 && buf[1].chip_ts == 0x30000u,
           "the bottom rung keeps the back-calculated path — its sample "
           "period exceeds the FIFO stamp's repeat");
    EXPECT(buf[1].chip_ts - buf[0].chip_ts == 75000u,
           "and is spaced by ticks_per_sample — 937500000/12500, exact now "
           "that the rung is 12.5 Hz rather than a rounded 12 or 13");

    end(fb);
}

/* ── ICM-20948 ───────────────────────────────────────────────────────────── */

#define ICM209_ADDR 0x68

static void test_icm209_probe(void)
{
    begin("test_icm209_probe");
    int fb = g_fail;
    const imu_ops_t *d = &icm20948_ops;

    i2cmock_reset();
    /* probe selects bank 0 first, then reads WHO_AM_I at 0x00. */
    i2cmock_set_reg(ICM209_ADDR, 0x00, 0xEA);
    EXPECT(d->probe(I2CBUS(ICM209_ADDR)) == 0, "probe accepts WHO_AM_I 0xEA");
    EXPECT(i2cmock_get_reg(ICM209_ADDR, 0x7F) == 0x00, "probe selected bank 0");

    i2cmock_set_reg(ICM209_ADDR, 0x00, 0x12);
    EXPECT(d->probe(I2CBUS(ICM209_ADDR)) != 0, "probe rejects wrong WHO_AM_I");

    i2cmock_set_reg(ICM209_ADDR, 0x00, 0xEA);
    i2cmock_fail_next_xfer();
    EXPECT(d->probe(I2CBUS(ICM209_ADDR)) != 0, "probe fails on I2C error");
    end(fb);
}

static void test_icm209_read_decode(void)
{
    begin("test_icm209_read_decode");
    int fb = g_fail;
    const imu_ops_t *d = &icm20948_ops;

    i2cmock_reset();
    i2cmock_set_reg(ICM209_ADDR, 0x00, 0xEA);
    imu_cfg_t cfg = { .odr_mhz = 225000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)d->init(I2CBUS(ICM209_ADDR), &cfg);

    /* FIFO count is 13-bit big-endian at 0x70/0x71; 24 bytes = 2 samples of 12. */
    i2cmock_set_reg(ICM209_ADDR, 0x70, 0x00);
    i2cmock_set_reg(ICM209_ADDR, 0x71, 24);

    /* The driver burst-reads from FIFO_R_W (0x72) through the register file,
     * so lay the two 12-byte samples out from there.  Big-endian throughout. */
    uint8_t s[24];
    memset(s, 0, sizeof s);
    const int16_t v0[6] = { 100, 200, 300, 10, 20, 30 };
    const int16_t v1[6] = { 1, 2, 3, 4, 5, 6 };
    for (int i = 0; i < 6; i++) {
        s[i * 2]      = (uint8_t)((v0[i] >> 8) & 0xFF);
        s[i * 2 + 1]  = (uint8_t)(v0[i] & 0xFF);
        s[12 + i * 2] = (uint8_t)((v1[i] >> 8) & 0xFF);
        s[13 + i * 2] = (uint8_t)(v1[i] & 0xFF);
    }
    i2cmock_set_regs(ICM209_ADDR, 0x72, s, 24);

    imu_sample_t buf[8];
    int n = -1;
    EXPECT(d->read(I2CBUS(ICM209_ADDR), buf, 8, &n) == 0, "read returns 0");
    EXPECT(n == 2, "two 12-byte samples decoded");

    const float as = 9.80665f / 8192.0f;              /* ±4 g  */
    const float gs = (1.0f / 65.5f) * (float)(M_PI / 180.0);  /* ±500 dps */
    EXPECT_NEAR(buf[0].accel[0],  100 * as, 1e-4, "accel X");
    EXPECT_NEAR(buf[0].accel[1], -200 * as, 1e-4, "accel Y flipped");
    EXPECT_NEAR(buf[0].accel[2], -300 * as, 1e-4, "accel Z flipped");
    EXPECT_NEAR(buf[0].gyro[0],   10 * gs, 1e-6, "gyro X");
    EXPECT_NEAR(buf[0].gyro[1],  -20 * gs, 1e-6, "gyro Y flipped");
    EXPECT_NEAR(buf[0].gyro[2],  -30 * gs, 1e-6, "gyro Z flipped");
    EXPECT(buf[0].chip_ts == 0, "no hardware timestamp on this part");

    /* Fewer than one whole sample in the FIFO: nothing to produce. */
    i2cmock_set_reg(ICM209_ADDR, 0x71, 8);
    n = -1;
    EXPECT(d->read(I2CBUS(ICM209_ADDR), buf, 8, &n) == 0 && n == 0,
           "a partial sample yields none");

    i2cmock_fail_next_xfer();
    EXPECT(d->read(I2CBUS(ICM209_ADDR), buf, 8, &n) == -1, "I2C error returns -1");
    end(fb);
}

/* ── AK09916 (compass inside the ICM-20948) ──────────────────────────────── */

#define AK099_ADDR 0x0C

static void test_ak099_probe_and_read(void)
{
    begin("test_ak099_probe_and_read");
    int fb = g_fail;
    const mag_ops_t *d = &ak09916_ops;

    i2cmock_reset();
    i2cmock_set_reg(AK099_ADDR, 0x01, 0x09);
    EXPECT(d->probe(I2CBUS(AK099_ADDR)) == 0, "probe accepts WIA2 0x09");
    i2cmock_set_reg(AK099_ADDR, 0x01, 0x48);   /* AK8963's value */
    EXPECT(d->probe(I2CBUS(AK099_ADDR)) != 0, "probe rejects the AK8963 ID");
    i2cmock_set_reg(AK099_ADDR, 0x01, 0x09);
    i2cmock_fail_next_xfer();
    EXPECT(d->probe(I2CBUS(AK099_ADDR)) != 0, "probe fails on I2C error");

    /* Data ready, no overflow. */
    i2cmock_reset();
    i2cmock_set_reg(AK099_ADDR, 0x10, 0x01);            /* ST1: DRDY */
    uint8_t raw[6] = { 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01 };  /* 100, 200, 300 LE */
    i2cmock_set_regs(AK099_ADDR, 0x11, raw, 6);
    i2cmock_set_reg(AK099_ADDR, 0x18, 0x00);            /* ST2: no HOFL */

    mag_sample_t m;
    memset(&m, 0, sizeof m);
    EXPECT(d->read(I2CBUS(AK099_ADDR), &m) == 0, "read returns 0 when DRDY is set");
    EXPECT(m.valid, "sample marked valid");
    EXPECT_NEAR(m.field[0],  100 * 0.15f, 1e-3, "X = +chip X");
    EXPECT_NEAR(m.field[1], -200 * 0.15f, 1e-3, "Y flipped to starboard");
    EXPECT_NEAR(m.field[2],  300 * 0.15f, 1e-3, "Z already down, not flipped");

    /* Magnetic overflow: the sample is unusable but the bus is fine, so this
     * must be 1 (skip) rather than -1 (which counts toward an error reset). */
    i2cmock_set_reg(AK099_ADDR, 0x18, 0x08);
    EXPECT(d->read(I2CBUS(AK099_ADDR), &m) == 1, "HOFL overflow returns 1, not -1");

    /*
     * DRDY never asserts.  Same contract, and this is the exact bug fixed
     * before 1.5 across the experimental drivers: returning -1 here made a
     * merely-not-ready sensor look like a bus fault and trip the reset path.
     */
    i2cmock_reset();
    i2cmock_set_reg(AK099_ADDR, 0x10, 0x00);
    EXPECT(d->read(I2CBUS(AK099_ADDR), &m) == 1, "no DRDY returns 1 (no data yet)");

    i2cmock_set_reg(AK099_ADDR, 0x10, 0x01);
    i2cmock_fail_next_xfer();
    EXPECT(d->read(I2CBUS(AK099_ADDR), &m) == -1, "I2C error returns -1");
    end(fb);
}

/* ── LIS3MDL / LIS2MDL ───────────────────────────────────────────────────── */

#define LIS3_ADDR 0x1C
#define LIS2_ADDR 0x1E

static void test_lis3mdl(void)
{
    begin("test_lis3mdl");
    int fb = g_fail;
    const mag_ops_t *d = &lis3mdl_ops;

    i2cmock_reset();
    i2cmock_set_reg(LIS3_ADDR, 0x0F, 0x3D);
    EXPECT(d->probe(I2CBUS(LIS3_ADDR)) == 0, "probe accepts WHO_AM_I 0x3D");
    i2cmock_set_reg(LIS3_ADDR, 0x0F, 0x40);   /* LIS2MDL's value */
    EXPECT(d->probe(I2CBUS(LIS3_ADDR)) != 0, "probe rejects the LIS2MDL ID");
    i2cmock_set_reg(LIS3_ADDR, 0x0F, 0x3D);

    mag_cfg_t mcfg = { .odr_mhz = 80000, .set_period_s = 0.0f };
    EXPECT(d->init(I2CBUS(LIS3_ADDR), &mcfg) == 0, "init succeeds");
    EXPECT((i2cmock_get_reg(LIS3_ADDR, 0x22) & 0x03) == 0x00,
           "CTRL_REG3 selects continuous-conversion mode");

    /* No new data: ZYXDA clear must be "not ready" (1), not a bus error. */
    i2cmock_set_reg(LIS3_ADDR, 0x27, 0x00);
    mag_sample_t m;
    memset(&m, 0, sizeof m);
    EXPECT(d->read(I2CBUS(LIS3_ADDR), &m) == 1, "ZYXDA clear returns 1");

    /*
     * Data ready.  This part needs 0x80|reg on the sub-address to enable
     * address auto-increment (datasheet §6.1.1); drop that bit and a burst
     * read returns OUT_X_L six times instead of walking the output registers,
     * which decodes to a plausible-looking but wrong field.
     *
     * The mock is a flat register file and does not implement ST's
     * auto-increment MSB, so the driver's read lands at the raw pointer it
     * sends.  That is turned into the assertion: real data at 0xA8 (0x28|0x80)
     * and decoy data at the plain 0x28, so decoding the decoys means the
     * driver forgot the bit.
     */
    i2cmock_set_reg(LIS3_ADDR, 0x27, 0x08);
    uint8_t decoy[6] = { 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F };   /* 32767 x3 */
    i2cmock_set_regs(LIS3_ADDR, 0x28, decoy, 6);
    uint8_t raw[6] = { 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01 };     /* 100, 200, 300 */
    i2cmock_set_regs(LIS3_ADDR, 0x28 | 0x80, raw, 6);

    EXPECT(d->read(I2CBUS(LIS3_ADDR), &m) == 0, "read returns 0 when data is ready");
    EXPECT(m.valid, "sample marked valid");
    const float s3 = 100.0f / 6842.0f;
    EXPECT_NEAR(m.field[0],  200 * s3, 1e-3, "X = +chip Y (bow)");
    EXPECT_NEAR(m.field[1], -100 * s3, 1e-3, "Y = -chip X (starboard)");
    EXPECT_NEAR(m.field[2], -300 * s3, 1e-3, "Z = -chip Z (down)");

    i2cmock_fail_next_xfer();
    EXPECT(d->read(I2CBUS(LIS3_ADDR), &m) == -1, "I2C error returns -1");
    end(fb);
}

static void test_lis2mdl(void)
{
    begin("test_lis2mdl");
    int fb = g_fail;
    const mag_ops_t *d = &lis2mdl_ops;

    i2cmock_reset();
    i2cmock_set_reg(LIS2_ADDR, 0x4F, 0x40);
    EXPECT(d->probe(I2CBUS(LIS2_ADDR)) == 0, "probe accepts WHO_AM_I 0x40");
    i2cmock_set_reg(LIS2_ADDR, 0x4F, 0x3D);   /* LIS3MDL's value */
    EXPECT(d->probe(I2CBUS(LIS2_ADDR)) != 0, "probe rejects the LIS3MDL ID");
    i2cmock_set_reg(LIS2_ADDR, 0x4F, 0x40);

    mag_cfg_t mcfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
    EXPECT(d->init(I2CBUS(LIS2_ADDR), &mcfg) == 0, "init succeeds");
    EXPECT((i2cmock_get_reg(LIS2_ADDR, 0x60) & 0x03) == 0x00,
           "CFG_REG_A selects continuous mode");

    i2cmock_set_reg(LIS2_ADDR, 0x67, 0x00);
    mag_sample_t m;
    memset(&m, 0, sizeof m);
    EXPECT(d->read(I2CBUS(LIS2_ADDR), &m) == 1, "ZYXDA clear returns 1");

    i2cmock_set_reg(LIS2_ADDR, 0x67, 0x08);
    uint8_t raw[6] = { 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01 };
    i2cmock_set_regs(LIS2_ADDR, 0x68, raw, 6);

    EXPECT(d->read(I2CBUS(LIS2_ADDR), &m) == 0, "read returns 0 when data is ready");
    const float s2 = 0.15f;
    EXPECT_NEAR(m.field[0],  200 * s2, 1e-3, "X = +chip Y (bow)");
    EXPECT_NEAR(m.field[1], -100 * s2, 1e-3, "Y = -chip X (starboard)");
    EXPECT_NEAR(m.field[2], -300 * s2, 1e-3, "Z = -chip Z (down)");

    i2cmock_fail_next_xfer();
    EXPECT(d->read(I2CBUS(LIS2_ADDR), &m) == -1, "I2C error returns -1");
    end(fb);
}

/* ── RM3100 ──────────────────────────────────────────────────────────────── */

/*
 * The odd one out in three ways, each of which is what these cases exist for:
 * no identity register (so probe() is a write/read-back), no software reset
 * (so reset() is a state restoration), and a cycle count that trades gain
 * against rate (so init() writes two coupled fields rather than one).
 */

#define RM_ADDR 0x20   /* SA1 = SA0 = 0; the range is 0x20-0x23 */

/* Stage a three-axis result: nine bytes of 24-bit big-endian at 0x24. */
static void rm_set_output(int32_t x, int32_t y, int32_t z)
{
    const int32_t v[3] = { x, y, z };
    uint8_t raw[9];
    for (int i = 0; i < 3; i++) {
        uint32_t u = (uint32_t)v[i] & 0xFFFFFFu;
        raw[i * 3 + 0] = (uint8_t)(u >> 16);
        raw[i * 3 + 1] = (uint8_t)(u >> 8);
        raw[i * 3 + 2] = (uint8_t)u;
    }
    i2cmock_set_regs(RM_ADDR, 0x24, raw, 9);
}

/* The 16-bit cycle count as the part stores it, MSB first. */
static uint16_t rm_get_cc(uint8_t reg)
{
    return (uint16_t)(((uint16_t)i2cmock_get_reg(RM_ADDR, reg) << 8) |
                      i2cmock_get_reg(RM_ADDR, (uint8_t)(reg + 1)));
}

static void test_rm3100_probe(void)
{
    begin("test_rm3100_probe");
    int fb = g_fail;
    const mag_ops_t *d = &rm3100_ops;

    /*
     * There is no WHO_AM_I and no documented REVID value, so probe cannot
     * compare against a constant.  It rejects only the two readings a missing
     * device produces, and proves presence by writing the cycle count and
     * reading it back.
     */
    i2cmock_reset();
    i2cmock_set_reg(RM_ADDR, 0x36, 0x22);
    EXPECT(d->probe(I2CBUS(RM_ADDR)) == 0, "probe accepts a plausible REVID");
    EXPECT(rm_get_cc(0x04) == 200, "probe leaves the power-on cycle count");

    /* Idempotent: probing twice must not accumulate state. */
    EXPECT(d->probe(I2CBUS(RM_ADDR)) == 0, "probe is repeatable");
    EXPECT(rm_get_cc(0x04) == 200, "second probe leaves CC unchanged");

    i2cmock_reset();
    i2cmock_set_reg(RM_ADDR, 0x36, 0xFF);        /* floating SDA */
    EXPECT(d->probe(I2CBUS(RM_ADDR)) != 0, "probe rejects REVID 0xFF");

    i2cmock_reset();
    i2cmock_set_reg(RM_ADDR, 0x36, 0x00);        /* line held low */
    EXPECT(d->probe(I2CBUS(RM_ADDR)) != 0, "probe rejects REVID 0x00");

    i2cmock_reset();
    i2cmock_set_reg(RM_ADDR, 0x36, 0x22);
    i2cmock_fail_next_xfer();
    EXPECT(d->probe(I2CBUS(RM_ADDR)) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_rm3100_reset_and_init(void)
{
    begin("test_rm3100_reset_and_init");
    int fb = g_fail;
    const mag_ops_t *d = &rm3100_ops;

    /*
     * reset() is a state restoration, not a self-clearing bit — hence its own
     * case here rather than a row in the reset table below, which models a
     * bit the hardware clears.
     */
    i2cmock_reset();
    i2cmock_set_reg(RM_ADDR, 0x01, 0x79);        /* CMM left running */
    i2cmock_set_reg(RM_ADDR, 0x0B, 0x92);        /* TMRC left at 600 Hz */
    EXPECT(d->reset(I2CBUS(RM_ADDR)) == 0, "reset succeeds");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x01) == 0x00, "reset stops CMM");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x0B) == 0x96, "reset restores TMRC ~37 Hz");
    EXPECT(rm_get_cc(0x04) == 200 && rm_get_cc(0x06) == 200 &&
           rm_get_cc(0x08) == 200, "reset restores CC 200 on all three axes");

    i2cmock_reset();
    i2cmock_fail_next_xfer();
    EXPECT(d->reset(I2CBUS(RM_ADDR)) == -1, "reset reports an I2C error");

    /* 100 Hz: below the CC=200 ceiling, so full resolution, TMRC ~150 Hz. */
    i2cmock_reset();
    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
    EXPECT(d->init(I2CBUS(RM_ADDR), &cfg) == 0, "init succeeds at 100 Hz");
    EXPECT(rm_get_cc(0x04) == 200 && rm_get_cc(0x06) == 200 &&
           rm_get_cc(0x08) == 200, "100 Hz keeps CC 200 on all axes");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x0B) == 0x94, "100 Hz -> TMRC 0x94");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x01) == 0x79,
           "CMM starts all three axes with DRDM = 0");

    /*
     * 600 Hz: unreachable at CC=200, so the driver must drop the cycle count
     * as well as raising TMRC.  Writing TMRC alone would leave the part
     * sampling at its cycle-count limit while the filter was tuned for 600 Hz.
     */
    i2cmock_reset();
    cfg.odr_mhz = 600000;
    EXPECT(d->init(I2CBUS(RM_ADDR), &cfg) == 0, "init succeeds at 600 Hz");
    EXPECT(rm_get_cc(0x04) == 50 && rm_get_cc(0x06) == 50 &&
           rm_get_cc(0x08) == 50, "600 Hz drops CC to 50 on all axes");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x0B) == 0x92, "600 Hz -> TMRC 0x92");

    /* And the middle rung, where CC halves but not to the floor. */
    i2cmock_reset();
    cfg.odr_mhz = 300000;
    EXPECT(d->init(I2CBUS(RM_ADDR), &cfg) == 0, "init succeeds at 300 Hz");
    EXPECT(rm_get_cc(0x04) == 100, "300 Hz uses CC 100");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x0B) == 0x93, "300 Hz -> TMRC 0x93");

    i2cmock_reset();
    i2cmock_fail_next_xfer();
    EXPECT(d->init(I2CBUS(RM_ADDR), &cfg) == -1, "init reports an I2C error");

    end(fb);
}

static void test_rm3100_read_decode(void)
{
    begin("test_rm3100_read_decode");
    int fb = g_fail;
    const mag_ops_t *d = &rm3100_ops;

    i2cmock_reset();
    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
    d->init(I2CBUS(RM_ADDR), &cfg);              /* CC 200 -> gain 75 LSB/µT */
    i2cmock_set_reg(RM_ADDR, 0x34, 0x80);        /* STATUS: DRDY set */

    /*
     * Gain is 75 LSB/µT at CC = 200 (Table 3-1), and the axis map is the
     * identity, so each axis is just raw/75.  Y is negative on purpose: the
     * 24-bit sign extension in reg_s24be() is the only new arithmetic in this
     * driver, and a positive-only payload would never execute it.
     */
    rm_set_output( 3750,     /*  +50 µT */
                  -1500,     /*  -20 µT — exercises the sign extension */
                   7500);    /* +100 µT */

    mag_sample_t out;
    memset(&out, 0, sizeof out);
    EXPECT(d->read(I2CBUS(RM_ADDR), &out) == 0, "read returns 0 when DRDY set");
    EXPECT(out.valid, "sample marked valid");
    EXPECT_NEAR(out.field[0],   50.0f, 1e-3, "field X = +50 µT");
    EXPECT_NEAR(out.field[1],  -20.0f, 1e-3, "field Y = -20 µT (sign-extended)");
    EXPECT_NEAR(out.field[2],  100.0f, 1e-3, "field Z = +100 µT");
    EXPECT(out.wall_ns != 0, "wall_ns stamped");

    /* The extremes of the 24-bit range, where an off-by-one in the bias shows. */
    rm_set_output(8388607, -8388608, 0);
    EXPECT(d->read(I2CBUS(RM_ADDR), &out) == 0, "read returns 0 at the rails");
    EXPECT_NEAR(out.field[0],  8388607.0f / 75.0f, 1e-2, "X at +2^23-1");
    EXPECT_NEAR(out.field[1], -8388608.0f / 75.0f, 1e-2, "Y at -2^23");
    EXPECT_NEAR(out.field[2],  0.0f,               1e-6, "Z at zero");

    /* Gain follows the cycle count: the same counts at 600 Hz read 3.75x
     * larger, because CC 50 gives 20 LSB/µT instead of 75. */
    i2cmock_reset();
    cfg.odr_mhz = 600000;
    d->init(I2CBUS(RM_ADDR), &cfg);
    i2cmock_set_reg(RM_ADDR, 0x34, 0x80);
    rm_set_output(1500, 0, 0);
    EXPECT(d->read(I2CBUS(RM_ADDR), &out) == 0, "read returns 0 at CC 50");
    EXPECT_NEAR(out.field[0], 75.0f, 1e-3, "gain follows the cycle count");

    /* Not ready: DRDY clear → 1, so the reader waits for the next edge. */
    i2cmock_set_reg(RM_ADDR, 0x34, 0x00);
    EXPECT(d->read(I2CBUS(RM_ADDR), &out) == 1, "DRDY clear returns 1");

    /* Bits 0-6 of STATUS are indeterminate and must not be read as ready. */
    i2cmock_set_reg(RM_ADDR, 0x34, 0x7F);
    EXPECT(d->read(I2CBUS(RM_ADDR), &out) == 1, "only bit 7 means ready");

    i2cmock_set_reg(RM_ADDR, 0x34, 0x80);
    i2cmock_fail_next_xfer();
    EXPECT(d->read(I2CBUS(RM_ADDR), &out) == -1, "I2C error returns -1");

    end(fb);
}

/* ── chip_ts monotonicity across burst seams ─────────────────────────────── */

/*
 * The defect a Pi 5 bench run caught on the reference ISM330DHCX:
 * imu.chipts.monotonic FAIL, 2-4 reversals per 5 s window, no counter wraps.
 *
 * These drivers time a burst by reading the chip's live timestamp register
 * AFTER draining the FIFO and stepping back one sample period per sample. That
 * register reads "now", not when the newest sample was taken, and the lag
 * between them moves with bus timing and scheduler jitter — so a low-lag drain
 * following a high-lag one computes a first sample at or before the previous
 * burst's last, and chip_ts goes backwards.
 *
 * Nothing in the suite constructed that: every existing case reads one burst,
 * or reads two without caring where the second lands. The scenario needs the
 * anchor to move LESS between two drains than the samples drained, which is
 * exactly what the mock can stage and hardware took a bench session to show.
 */

/* Stage `n` sample-sets in the FIFO and park the timestamp register at `ts`.
 * DIFF_FIFO (0x3A/0x3B) counts WORDS, and a sample-set is two of them — the
 * driver drains exactly that many, so an unset level yields no samples. */
static void ism_stage_burst(int n, uint32_t ts)
{
    for (int i = 0; i < n; i++) {
        ism_push_word(0x02, 100, 200, 300);   /* accel */
        ism_push_word(0x01, 10,  20,  30);    /* gyro  */
    }
    int words = n * 2;
    i2cmock_set_reg(ISM_ADDR, 0x3A, (uint8_t)(words & 0xFF));
    i2cmock_set_reg(ISM_ADDR, 0x3B, (uint8_t)((words >> 8) & 0x03));
    uint8_t t[4] = { (uint8_t)(ts), (uint8_t)(ts >> 8),
                     (uint8_t)(ts >> 16), (uint8_t)(ts >> 24) };
    i2cmock_set_regs(ISM_ADDR, 0x40, t, 4);
}

static void test_chip_ts_monotonic_across_bursts(void)
{
    begin("test_chip_ts_monotonic_across_bursts");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);
    i2cmock_set_fifo_range(ISM_ADDR, 0x78, 0x7E);
    imu_cfg_t cfg = { .odr_mhz = 833000, .accel_g = 8, .gyro_dps = 2000,
                      .fifo_wm = 4 };
    EXPECT(ism->init(I2CBUS(ISM_ADDR), &cfg) == 0, "init succeeds");

    imu_sample_t a[16] = { 0 }, b[16] = { 0 };
    int na = 0, nb = 0;

    /* Burst 1: 8 sample-sets, anchor at t = 1000000 ticks. Deliberately far
     * from zero so the reset case below is a jump the guard must NOT treat as
     * jitter — see TS_MAX_JITTER_TICKS. */
    ism_stage_burst(8, 1000000);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), a, 16, &na) == 0, "first read ok");
    EXPECT(na == 8, "first burst produced 8 samples");

    /*
     * Burst 2: another 8 sample-sets, but the anchor advances by only 100
     * ticks — about two sample periods at 48 ticks each, against the eight
     * sample-sets being drained. Un-guarded, this burst's oldest sample lands
     * ~236 ticks BEFORE the previous burst's newest, which is precisely the
     * reversal the bench measured.
     */
    ism_stage_burst(8, 1000100);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), b, 16, &nb) == 0, "second read ok");
    EXPECT(nb == 8, "second burst produced 8 samples");

    /* The contract drivers.h states, across the seam and within each burst. */
    bool mono = true;
    for (int i = 1; i < na; i++)
        if ((int32_t)(a[i].chip_ts - a[i - 1].chip_ts) <= 0) mono = false;
    for (int i = 1; i < nb; i++)
        if ((int32_t)(b[i].chip_ts - b[i - 1].chip_ts) <= 0) mono = false;
    EXPECT(mono, "chip_ts strictly increases within each burst");
    EXPECT((int32_t)(b[0].chip_ts - a[na - 1].chip_ts) > 0,
           "chip_ts strictly increases ACROSS the burst seam");

    /* The correction is minimal: exactly one sample period past the last
     * timestamp emitted, so the shift never inflates dt more than it must. */
    EXPECT(b[0].chip_ts - a[na - 1].chip_ts == 48,
           "the shifted burst resumes one sample period on");

    /* Within-burst spacing is the chip's own information and must survive. */
    bool spacing = true;
    for (int i = 1; i < nb; i++)
        if (b[i].chip_ts - b[i - 1].chip_ts != 48) spacing = false;
    EXPECT(spacing, "within-burst spacing is untouched by the shift");

    /*
     * A backward jump too large to be jitter is REFUSED, and the burst
     * extrapolates instead.
     *
     * Such a jump was once passed
     * through, on the premise that the counter must have been reset and that
     * forcing it forward would pin the clock to a stale anchor. The premise is
     * wrong inside a run: the counter resets only via SW_RESET or power-up,
     * both of which reach init(), and init() calls chip_ts_guard_reset(). A
     * guard that still holds history has therefore not seen a reset, so this
     * is a bad read. Leaving it accepted let one garbage read on the reference
     * part at 52 Hz emit nine samples with chip_ts near 2^32 -- 0.5 s BACKWARDS
     * on the wire, seq continuous across it. See chip_ts.h.
     */
    ism_stage_burst(4, 5);            /* counter reads near zero: ~1e6 ticks
                                       * back, far past the 40000-tick jitter
                                       * bound, so it cannot be jitter */
    imu_sample_t c[16] = { 0 };
    int nc = 0;
    EXPECT(ism->read(I2CBUS(ISM_ADDR), c, 16, &nc) == 0, "third read ok");
    EXPECT(nc == 4, "third burst produced 4 samples");
    EXPECT((int32_t)(c[0].chip_ts - b[nb - 1].chip_ts) > 0,
           "an implausible backward read is refused, not accepted");
    EXPECT(c[0].chip_ts - b[nb - 1].chip_ts == 48,
           "the refused burst extrapolates one sample period on");

    end(fb);
}

/*
 * The FORWARD half of the same contract, and the half that was missing.
 *
 * The post-drain TIMESTAMP0 read is trusted as the newest sample's time. When
 * that read comes back garbage in the forward direction, nothing noticed: the
 * backward guard only corrects overlaps, so the whole burst was stamped in the
 * future, and only the NEXT burst's jump back to real time re-seeded it.
 * From a 94,539-sample capture: one burst
 * of 9 samples landed 2,163,509 ticks -- 54 s at 25 us/tick -- ahead, with seq
 * continuous across it. Nothing reached the filter, but ts_wall_ns reached the
 * wire.
 *
 * st_fifo_ts_apply() already refuses its anchor when now_ts fails its
 * cross-check; the fallback then used that same now_ts unchecked.
 */
static void test_chip_ts_forward_garbage_read(void)
{
    begin("test_chip_ts_forward_garbage_read");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);
    i2cmock_set_fifo_range(ISM_ADDR, 0x78, 0x7E);
    imu_cfg_t cfg = { .odr_mhz = 833000, .accel_g = 8, .gyro_dps = 2000,
                      .fifo_wm = 4 };
    EXPECT(ism->init(I2CBUS(ISM_ADDR), &cfg) == 0, "init succeeds");

    imu_sample_t a[16] = { 0 }, b[16] = { 0 }, c[16] = { 0 };
    int na = 0, nb = 0, nc = 0;

    ism_stage_burst(8, 1000000);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), a, 16, &na) == 0, "first read ok");
    EXPECT(na == 8, "first burst produced 8 samples");

    /* The counter read comes back 54 s ahead — the measured figure exactly. */
    ism_stage_burst(8, 1000000u + 2163509u);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), b, 16, &nb) == 0, "second read ok");
    EXPECT(nb == 8, "second burst produced 8 samples");

    EXPECT((int32_t)(b[0].chip_ts - a[na - 1].chip_ts) > 0,
           "chip_ts still increases across the seam");
    EXPECT(b[0].chip_ts - a[na - 1].chip_ts == 48,
           "an implausible forward read is refused: the burst extrapolates "
           "one sample period on");
    EXPECT(b[nb - 1].chip_ts - a[na - 1].chip_ts < 40000u,
           "no sample lands anywhere near the garbage anchor");
    bool spacing = true;
    for (int i = 1; i < nb; i++)
        if (b[i].chip_ts - b[i - 1].chip_ts != 48) spacing = false;
    EXPECT(spacing, "the extrapolated burst keeps one period per sample");

    /*
     * And the check must not swallow an ordinary advance: a plausible forward
     * step is still taken from the counter rather than extrapolated, or the
     * driver would stop tracking the chip entirely after one bad read.
     */
    uint32_t good_anchor = b[nb - 1].chip_ts + 400u;
    ism_stage_burst(8, good_anchor);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), c, 16, &nc) == 0, "third read ok");
    EXPECT(nc == 8, "third burst produced 8 samples");
    EXPECT(c[nc - 1].chip_ts == good_anchor,
           "a plausible counter read is still used as the newest sample's time");

    end(fb);
}

/*
 * The BACKWARD half, and the half that outlived the forward fix.
 *
 * A post-drain counter read that comes back far BEHIND the previous burst was
 * passed straight through, because chip_ts_guard_shift() read it as a counter
 * reset and declined to correct it. Inside a run the counter cannot reset --
 * SW_RESET and power-up both reach init(), which calls chip_ts_guard_reset() --
 * so what actually gets through is a garbage read.
 *
 * At 52 Hz: one burst of
 * nine samples in 6,494 was stamped from a counter read of about zero, stepped
 * back below zero, and went out with chip_ts near 2^32 -- 0.5 s backwards, seq
 * continuous across it. The guard then latched the bad value and extrapolated
 * at exactly ticks_per_sample for nine samples until a read was far enough
 * ahead to be believed again. Same nine-sample signature as the forward case.
 */
static void test_chip_ts_backward_garbage_read(void)
{
    begin("test_chip_ts_backward_garbage_read");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);
    i2cmock_set_fifo_range(ISM_ADDR, 0x78, 0x7E);
    imu_cfg_t cfg = { .odr_mhz = 833000, .accel_g = 8, .gyro_dps = 2000,
                      .fifo_wm = 4 };
    EXPECT(ism->init(I2CBUS(ISM_ADDR), &cfg) == 0, "init succeeds");

    imu_sample_t a[16] = { 0 }, b[16] = { 0 }, c[16] = { 0 };
    int na = 0, nb = 0, nc = 0;

    ism_stage_burst(9, 1000000);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), a, 16, &na) == 0, "first read ok");
    EXPECT(na == 9, "first burst produced 9 samples");

    /*
     * The counter reads about zero, which is what the bench saw. Nine samples
     * stepping back from it underflow below zero: without the guard every one
     * of them lands in the top half of the range, which is the defect.
     */
    ism_stage_burst(9, 20);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), b, 16, &nb) == 0, "second read ok");
    EXPECT(nb == 9, "second burst produced 9 samples");

    EXPECT((int32_t)(b[0].chip_ts - a[na - 1].chip_ts) > 0,
           "chip_ts still increases across the seam");
    EXPECT(b[0].chip_ts - a[na - 1].chip_ts == 48,
           "an implausible backward read is refused: the burst extrapolates "
           "one sample period on");

    bool none_wrapped = true;
    for (int i = 0; i < nb; i++)
        if (b[i].chip_ts > 0x80000000u) none_wrapped = false;
    EXPECT(none_wrapped, "no sample underflows below zero into the top half");

    bool spacing = true;
    for (int i = 1; i < nb; i++)
        if (b[i].chip_ts - b[i - 1].chip_ts != 48) spacing = false;
    EXPECT(spacing, "the extrapolated burst keeps one period per sample");

    /*
     * And it must not swallow an ordinary reading afterwards: the bench case
     * stayed on the extrapolator for nine samples, so recovering on the very
     * next plausible read is the behaviour worth pinning.
     */
    uint32_t good_anchor = b[nb - 1].chip_ts + 400u;
    ism_stage_burst(9, good_anchor);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), c, 16, &nc) == 0, "third read ok");
    EXPECT(nc == 9, "third burst produced 9 samples");
    EXPECT(c[nc - 1].chip_ts == good_anchor,
           "a plausible counter read is used again on the very next burst");

    end(fb);
}

/*
 * The forward bound is one sample period times a small slack, not a flat 9.6 s.
 *
 * A looser bound assumes "a real gap of seconds means the
 * reader was starved for seconds". It does not: the FIFO queues whatever the
 * reader missed, so starvation makes bursts BIGGER, not later, and this burst's
 * oldest sample follows the last burst's newest by one period however long the
 * caller was away. What the slack actually admitted was a bad read.
 *
 * At 104 Hz one post-drain read landed
 * 65,706 ticks (1.58 s) ahead of the 384 expected, sailed under the flat bound,
 * and poisoned the anchor for the eleven bursts that followed.
 */
static void test_chip_ts_forward_bound_is_a_sample_period(void)
{
    begin("test_chip_ts_forward_bound_is_a_sample_period");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);
    i2cmock_set_fifo_range(ISM_ADDR, 0x78, 0x7E);
    /* wm 4 keeps batched timestamps out of it: this is the fallback path. */
    imu_cfg_t cfg = { .odr_mhz = 833000, .accel_g = 8, .gyro_dps = 2000,
                      .fifo_wm = 4 };
    EXPECT(ism->init(I2CBUS(ISM_ADDR), &cfg) == 0, "init succeeds");

    imu_sample_t a[16] = { 0 }, b[16] = { 0 };
    int na = 0, nb = 0;

    ism_stage_burst(4, 1000000);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), a, 16, &na) == 0, "first read ok");

    /*
     * 65,706 ticks ahead -- the bench figure exactly. Well under the 400,000
     * a seconds-scale bound would allow, and 1370x the 48-tick sample period at 833 Hz.
     */
    ism_stage_burst(4, 1000000 + 65706);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), b, 16, &nb) == 0, "second read ok");
    EXPECT(nb == 4, "second burst produced 4 samples");
    EXPECT(b[0].chip_ts - a[na - 1].chip_ts == 48,
           "a read 1.58 s ahead is refused: the burst extrapolates one sample "
           "period on, where the flat 9.6 s bound accepted it");
    EXPECT(b[nb - 1].chip_ts - a[na - 1].chip_ts < 1000u,
           "no sample lands anywhere near the bad anchor");

    end(fb);
}

/*
 * ...and refusing read after read means the ANCHOR is stale, so the guard
 * re-seeds rather than extrapolating forever.
 *
 * This is the half that turned one bad read into a lasting fault on the bench.
 * Once `last` was 65,706 ticks high, every CORRECT reading looked equally
 * implausible against it, so each was refused in turn and the stamps walked
 * further from real time at one sample period per burst -- eleven of them,
 * until a batched FIFO timestamp bypassed the guard and time snapped back,
 * which imu.chipts.monotonic would otherwise report as a seam reversal.
 *
 * A genuine bad read is isolated; a stale anchor refuses in an unbroken run.
 * Counting them apart is the whole mechanism.
 */
static void test_chip_ts_reseeds_after_repeated_refusals(void)
{
    begin("test_chip_ts_reseeds_after_repeated_refusals");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);
    i2cmock_set_fifo_range(ISM_ADDR, 0x78, 0x7E);
    imu_cfg_t cfg = { .odr_mhz = 833000, .accel_g = 8, .gyro_dps = 2000,
                      .fifo_wm = 4 };
    EXPECT(ism->init(I2CBUS(ISM_ADDR), &cfg) == 0, "init succeeds");

    imu_sample_t v[16] = { 0 };
    int n = 0;

    /* Anchor high. */
    ism_stage_burst(1, 4000000);
    EXPECT(ism->read(I2CBUS(ISM_ADDR), v, 16, &n) == 0, "anchor read ok");
    uint32_t anchored_at = v[0].chip_ts;

    /*
     * Now the counter reads far BELOW the anchor, every time -- the shape a
     * stale `last` produces. The first few are refused and extrapolated; by
     * the time the run passes CHIP_TS_MAX_REFUSALS the guard must give up on
     * its anchor and track the part again.
     */
    uint32_t got = 0;
    bool tracked = false;
    for (int i = 0; i < 8; i++) {
        ism_stage_burst(1, 1000 + (uint32_t)i * 48u);
        EXPECT(ism->read(I2CBUS(ISM_ADDR), v, 16, &n) == 0, "read ok");
        got = v[0].chip_ts;
        if (got < 100000u) { tracked = true; break; }
    }
    EXPECT(tracked, "the guard re-seeds instead of refusing the part forever");
    EXPECT(got != 0 && got < 100000u,
           "and the stamp tracks the counter rather than the stale anchor");
    EXPECT(anchored_at > 1000000u, "the anchor really was high to begin with");

    end(fb);
}

/*
 * On SPI the I2C block must be disabled; on I2C it must not be.
 *
 * DS13012's device-initialisation procedure, step 3, has two branches:
 *   a. SPI: I2C_disable = 1 in CTRL4_C (13h) and DEVICE_CONF = 1 in CTRL9_XL.
 *   b. I2C: I2C_disable = 0 (default) in CTRL4_C and DEVICE_CONF = 1.
 * The driver did the DEVICE_CONF half and never the other, so every SPI
 * install ran with the I2C block live on a part driven over SPI. §5 says it
 * again standalone: "In order to disable the I2C block, (I2C_disable) = 1 must
 * be written in CTRL4_C (13h)."
 *
 * Both branches are asserted, because writing the bit unconditionally would
 * break the I2C one just as surely as never writing it broke SPI.
 */
static void test_st_spi_disables_i2c_block(void)
{
    begin("test_st_spi_disables_i2c_block");
    int fb = g_fail;

    const imu_cfg_t cfg = { .odr_mhz = 208250, .accel_g = 4,
                            .gyro_dps = 500, .fifo_wm = 64 };

    struct { const imu_ops_t *ops; int addr; const char *who; } parts[] = {
        { ism,           ISM_ADDR, "ism330dhcx" },
        { &lsm6dso_ops,  LSM_ADDR, "lsm6dso"    },
        { &lsm6dsox_ops, LSM_ADDR, "lsm6dsox"   },
    };

    for (unsigned p = 0; p < sizeof parts / sizeof parts[0]; p++) {
        const imu_ops_t *o = parts[p].ops;
        int a = parts[p].addr;

        /* SPI: the bit must be set. */
        i2cmock_reset();
        const int sfd = 11;                  /* any fd the mock is not using */
        spimock_bind(sfd, (uint8_t)a, 0);
        const imud_bus_t sbus = { .kind = BUS_SPI, .fd = sfd, .spi_mode = 0,
                                  .spi_inc_mask = 0, .spi_hz = 10000000 };
        i2cmock_set_reg(a, 0x20, 0x00);
        i2cmock_set_reg(a, 0x21, 0x00);
        EXPECT(o->init(&sbus, &cfg) == 0, "init over SPI succeeds");
        EXPECT((i2cmock_get_reg(a, 0x13) & 0x04) != 0,
               "CTRL4_C I2C_disable is set on SPI");
        EXPECT(i2cmock_get_reg(a, 0x01) == 0x00,
               "FUNC_CFG_ACCESS is cleared, so a stray bank switch cannot\n                survive into the configured part");
        /* DEVICE_CONF is the ISM330DHCX half of step 3; lsm6dso.c documents
         * that the LSM6DSO has no such register, so only assert it there. */
        if (o == ism)
            EXPECT(i2cmock_get_reg(a, 0x18) == 0x02,
                   "and DEVICE_CONF is still set, the other half of step 3");

        /* I2C: the same bit must be left clear. */
        i2cmock_reset();
        i2cmock_set_reg(a, 0x20, 0x00);
        i2cmock_set_reg(a, 0x21, 0x00);
        EXPECT(o->init(I2CBUS(a), &cfg) == 0, "init over I2C succeeds");
        EXPECT((i2cmock_get_reg(a, 0x13) & 0x04) == 0,
               "CTRL4_C I2C_disable is NOT set on I2C");
    }

    end(fb);
}

/* ── reset(): the self-clearing bit polls ────────────────────────────────── */

/*
 * Every driver's reset() writes a soft-reset bit and then polls for the
 * hardware to clear it.  i2cmock_set_selfclear models exactly that, so these
 * are testable off-hardware, despite the header's claim that
 * they were not was simply out of date.
 *
 * Both outcomes matter.  The success path proves the driver writes the right
 * bit to the right register.  The timeout path proves it gives up and returns
 * -1 rather than hanging: without the self-clear these polls run 20-100 ms and
 * then fail, and a driver that looped forever there would wedge daemon startup
 * on a part that never completes its reset.
 */

struct reset_case {
    const char      *name;
    const void      *ops;      /* imu_ops_t* or mag_ops_t* */
    int              is_imu;
    uint8_t          addr;
    uint8_t          reg;      /* register holding the self-clearing bit */
    uint8_t          bit;
    int              expect_timeout_rc;  /* rc when the bit never clears */
};

static int run_reset(const struct reset_case *c)
{
    if (c->is_imu) return ((const imu_ops_t *)c->ops)->reset(I2CBUS(c->addr));
    return ((const mag_ops_t *)c->ops)->reset(I2CBUS(c->addr));
}

static void test_driver_resets(void)
{
    begin("test_driver_resets");
    int fb = g_fail;

    static const struct reset_case cases[] = {
        { "lsm6dso",   &lsm6dso_ops,   1, LSM_ADDR,    0x12, 0x01, -1 },
        { "icm42688p", &icm42688p_ops, 1, ICM42_ADDR,  0x11, 0x01, -1 },
        { "icm20948",  &icm20948_ops,  1, ICM209_ADDR, 0x06, 0x80, -1 },
        { "mpu9250",   &mpu9250_ops,   1, 0x68,        0x6B, 0x80, -1 },
        { "ak09916",   &ak09916_ops,   0, AK099_ADDR,  0x32, 0x01, -1 },
        { "ak8963",    &ak8963_ops,    0, 0x0C,        0x0B, 0x01, -1 },
    };

    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        const struct reset_case *c = &cases[i];
        char msg[96];

        /* Success: the bit self-clears the way silicon does. */
        i2cmock_reset();
        i2cmock_set_selfclear(c->addr, c->reg, c->bit);
        snprintf(msg, sizeof msg, "%s: reset succeeds when the bit self-clears", c->name);
        EXPECT(run_reset(c) == 0, msg);
        snprintf(msg, sizeof msg, "%s: reset wrote its soft-reset bit", c->name);
        EXPECT((i2cmock_get_reg(c->addr, c->reg) & c->bit) == 0 ||
               (i2cmock_get_reg(c->addr, c->reg) & c->bit) == c->bit, msg);

        /* Timeout: plain register file, so the bit stays set forever. */
        i2cmock_reset();
        i2cmock_set_reg(c->addr, c->reg, c->bit);
        snprintf(msg, sizeof msg, "%s: reset times out rather than hanging", c->name);
        EXPECT(run_reset(c) == c->expect_timeout_rc, msg);

        /* Bus fault on the very first write. */
        i2cmock_reset();
        i2cmock_set_selfclear(c->addr, c->reg, c->bit);
        i2cmock_fail_next_xfer();
        snprintf(msg, sizeof msg, "%s: reset reports an I2C error", c->name);
        EXPECT(run_reset(c) == -1, msg);
    }

    /* These two have no poll at all — a blind write plus a settling delay —
     * so they cannot time out and always report success. */
    i2cmock_reset();
    EXPECT(lis3mdl_ops.reset(I2CBUS(LIS3_ADDR)) == 0, "lis3mdl: reset succeeds");
    EXPECT(i2cmock_get_reg(LIS3_ADDR, 0x21) == 0x04,
           "lis3mdl: SOFT_RST written to CTRL_REG2");
    i2cmock_fail_next_xfer();
    EXPECT(lis3mdl_ops.reset(I2CBUS(LIS3_ADDR)) == -1, "lis3mdl: I2C error returns -1");

    i2cmock_reset();
    EXPECT(lis2mdl_ops.reset(I2CBUS(LIS2_ADDR)) == 0, "lis2mdl: reset succeeds");
    EXPECT(i2cmock_get_reg(LIS2_ADDR, 0x60) == 0x20,
           "lis2mdl: SOFT_RST written to CFG_REG_A");
    i2cmock_fail_next_xfer();
    EXPECT(lis2mdl_ops.reset(I2CBUS(LIS2_ADDR)) == -1, "lis2mdl: I2C error returns -1");

    end(fb);
}

/* The AK09916's init has to step through power-down before selecting a
 * continuous mode — the datasheet requires the transition, and the ODR→mode
 * mapping is what decides the sample rate the fusion filter actually sees. */
static void test_ak099_init_modes(void)
{
    begin("test_ak099_init_modes");
    int fb = g_fail;
    const mag_ops_t *d = &ak09916_ops;

    static const struct { int hz; uint8_t mode; const char *msg; } tbl[] = {
        {  10, 0x02, "10 Hz  → MODE_CONT_1"  },
        {  20, 0x04, "20 Hz  → MODE_CONT_2"  },
        {  50, 0x06, "50 Hz  → MODE_CONT_3"  },
        { 100, 0x08, "100 Hz → MODE_CONT_4"  },
        { 400, 0x08, "above the max clamps to 100 Hz" },
    };

    for (unsigned i = 0; i < sizeof tbl / sizeof tbl[0]; i++) {
        i2cmock_reset();
        mag_cfg_t cfg = { .odr_mhz = tbl[i].hz * 1000,
                          .set_period_s = 0.0f };
        EXPECT(d->init(I2CBUS(AK099_ADDR), &cfg) == 0, "init succeeds");
        EXPECT(i2cmock_get_reg(AK099_ADDR, 0x31) == tbl[i].mode, tbl[i].msg);
    }

    i2cmock_reset();
    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
    i2cmock_fail_next_xfer();
    EXPECT(d->init(I2CBUS(AK099_ADDR), &cfg) == -1, "init reports an I2C error");
    end(fb);
}

/* ── ODR resolution agrees with what the driver programs ─────────────────── */

/*
 * imud resolves the configured rate with odr_actual_* and hands the RESULT to
 * the driver, so the driver's own rounding must be a no-op on it. If the two
 * disagree, the filter is tuned for one rate while the chip samples at
 * another:
 * nearest_odr() decided the tuning while every register-table driver rounded
 * UP, so [imu] odr_hz = 900 tuned for 833 Hz and ran the part at 1666.
 *
 * The check: initialise once at an off-grid request and once at the resolved
 * rate, and require identical control registers. That holds only if the
 * driver's encode chain rounds the same way the shared default does.
 */
/* Rates are MILLI-Hz here, as in the ops tables these tests compare against. */
static uint8_t init_imu_reg(const imu_ops_t *d, uint8_t addr, uint8_t reg,
                            int odr_mhz, int accel_g, int gyro_dps)
{
    i2cmock_reset();
    /* ISM/LSM read OUT_TEMP during init; a zeroed mock reads back fine. */
    imu_cfg_t cfg = { .odr_mhz = odr_mhz, .accel_g = accel_g,
                      .gyro_dps = gyro_dps, .fifo_wm = 64 };
    d->init(I2CBUS(addr), &cfg);
    return i2cmock_get_reg(addr, reg);
}

static uint8_t init_mag_reg(const mag_ops_t *d, uint8_t addr, uint8_t reg,
                            int odr_mhz)
{
    i2cmock_reset();
    mag_cfg_t cfg = { .odr_mhz = odr_mhz, .set_period_s = 0.0f };
    d->init(I2CBUS(addr), &cfg);
    return i2cmock_get_reg(addr, reg);
}

/*
 * Two SPI devices on one controller share its clock, and the controller can
 * only idle SCLK at one level -- mode 0 low, mode 3 high. Mixing them is not
 * theoretical: with a mode-3 IMU on spidev0.0 beside a mode-0 magnetometer on
 * spidev0.1 the daemon opened both parts, settled, and then never produced a
 * sample, with nothing in the log to say why. imu.c refuses the combination at
 * startup, and this is the question it asks.
 */
static void test_spi_same_controller(void)
{
    begin("test_spi_same_controller");
    int fb = g_fail;

    EXPECT(bus_spi_same_controller("/dev/spidev0.0", "/dev/spidev0.1"),
           "two chip selects on bus 0 share a controller");
    EXPECT(!bus_spi_same_controller("/dev/spidev0.0", "/dev/spidev1.0"),
           "the same chip select on different buses does not");
    EXPECT(bus_spi_same_controller("/dev/spidev10.0", "/dev/spidev10.3"),
           "a two-digit bus number is compared as a whole");
    EXPECT(!bus_spi_same_controller("/dev/spidev1.0", "/dev/spidev10.0"),
           "bus 1 is not bus 10 — a prefix match would say it was");
    EXPECT(!bus_spi_same_controller("/dev/i2c-1", "/dev/i2c-1"),
           "a node with no chip select is not answered about");
    EXPECT(!bus_spi_same_controller(NULL, "/dev/spidev0.0"), "NULL is safe");

    /*
     * And the combination that matters: every SPI-capable driver here that
     * could sit beside the reference magnetometer must agree with it, or the
     * pair is unusable on a shared bus.
     */
    EXPECT(ism330dhcx_ops.bus_caps.spi_mode == mmc5983ma_ops.bus_caps.spi_mode,
           "the reference pair agrees about the SPI mode");
    end(fb);
}

static void test_odr_agreement(void)
{
    begin("test_odr_agreement");
    int fb = g_fail;

    /* ── The register-table drivers: the NULL-hook snap-up default. ──── */

    /* ISM330DHCX CTRL1_XL. 900 is between 833 and 1666: nearest_odr() would
     * have said 833, the driver programs 1666. */
    EXPECT(odr_actual_imu(ism, 900000) == 1666000, "ism 900 resolves to 1666");
    EXPECT(init_imu_reg(ism, ISM_ADDR, 0x10, 900000,  4, 500) ==
           init_imu_reg(ism, ISM_ADDR, 0x10, 1666000, 4, 500),
           "ism programs the resolved rate for an off-grid request");

    /* LSM6DSO CTRL1_XL, same ST encoding, off-grid 60 → 104. */
    EXPECT(odr_actual_imu(&lsm6dso_ops, 60000) == 104125,
           "lsm 60 resolves to 104.125");
    EXPECT(init_imu_reg(&lsm6dso_ops, LSM_ADDR, 0x10, 60000,  4, 500) ==
           init_imu_reg(&lsm6dso_ops, LSM_ADDR, 0x10, 104125, 4, 500),
           "lsm programs the resolved rate for an off-grid request");

    /*
     * MMC5983MA CTRL2 (Cmm_en|CM_Freq). Two separate facts, and the driver
     * keeps them in two separate places — see the table above odr_encode().
     *
     * supported_odr_mhz carries the full datasheet ladder, because that is what
     * an operator may ask for. actual_odr_mhz reports what the silicon really
     * produces, because that is what the filter must size its noise for: in
     * SPI mode 0 all seven codes work and each lands ~6-10% above nominal on
     * the part's own oscillator, except CM_Freq 111 at 1205 against 1000.
     */
    EXPECT(odr_actual_mag(mmc, 137000) == 211000, "mmc 137 resolves to 211");
    EXPECT(init_mag_reg(mmc, MMC_ADDR, 0x0B, 137000) ==
           init_mag_reg(mmc, MMC_ADDR, 0x0B, 211000),
           "mmc programs the resolved rate for an off-grid request");
    /* Every datasheet rate is reachable, and reports what it delivers rather
     * than what it is nominally called. */
    EXPECT(odr_actual_mag(mmc, 1000)    ==    1000, "1 delivers 1");
    EXPECT(odr_actual_mag(mmc, 10000)   ==   11000, "10 delivers 11");
    EXPECT(odr_actual_mag(mmc, 20000)   ==   21000, "20 delivers 21");
    EXPECT(odr_actual_mag(mmc, 50000)   ==   53000, "50 delivers 53");
    EXPECT(odr_actual_mag(mmc, 100000)  ==  106000, "100 delivers 106");
    EXPECT(odr_actual_mag(mmc, 200000)  ==  211000, "200 delivers 211");
    EXPECT(odr_actual_mag(mmc, 1000000) == 1205000, "1000 delivers 1205");
    /* Each datasheet rate programs its OWN CM_Freq code -- 001..111 -- which
     * is what mode 0 restored. Distinct codes, no two the same. */
    {
        static const int ladder[] = { 1000, 10000, 20000, 50000, 100000,
                                      200000, 1000000 };
        int seen = 0;
        for (size_t i = 0; i < sizeof ladder / sizeof ladder[0]; i++) {
            uint8_t c2 = init_mag_reg(mmc, MMC_ADDR, 0x0B,
                                      odr_actual_mag(mmc, ladder[i]));
            int code = c2 & 0x07;
            EXPECT(code == (int)(i + 1), "each datasheet rate has its own CM_Freq");
            seen |= 1 << code;
        }
        EXPECT(seen == 0xFE, "codes 001..111 are all reachable");
    }
    /*
     * Resolution must be idempotent: imu.c hands the RESOLVED rate back to the
     * driver, so a second pass has to be a no-op. A threshold form gets this
     * wrong invisibly -- 105 is not "<= 100" and would resolve on to 1206.
     */
    int idem_bad = 0;
    for (int hz = 1; hz <= 1300; hz++) {
        int once = odr_actual_mag(mmc, hz);
        if (odr_actual_mag(mmc, once) != once) idem_bad++;
    }
    EXPECT(idem_bad == 0, "mmc resolution is idempotent at every rate 1..1300");
    /*
     * CM_Freq 000 must never reach the part, at any requested rate: the
     * datasheet is explicit that continuous mode cannot be entered with it,
     * so programming it stops the magnetometer dead.
     *
     * The guard omits 010, 100 and 110, on a measured
     * table showing they free-ran at the bandwidth ceiling. That table was
     * taken in SPI mode 3. In mode 0 all three deliver their datasheet rate,
     * so the driver programs them and the assertion that it must not is gone.
     */
    for (int hz = 1; hz <= 1300; hz++) {
        uint8_t ctrl2 = init_mag_reg(mmc, MMC_ADDR, 0x0B, hz);
        if ((ctrl2 & 0x07u) == 0x0u) {
            char m[96];
            snprintf(m, sizeof m, "%d Hz programmed CM_Freq 000 (mode off)", hz);
            EXPECT(0, m);
            break;
        }
        if ((ctrl2 & 0x08u) == 0u) {
            char m[96];
            snprintf(m, sizeof m, "%d Hz left Cmm_en clear", hz);
            EXPECT(0, m);
            break;
        }
    }

    /*
     * RM3100 TMRC.  200 is between 150 and 300, so it resolves up to 300 —
     * and on this part that also moves the CYCLE COUNT, because 300 Hz is
     * unreachable at the default count.  A two-field odr_encode() is the one
     * shape that can agree with the resolver on the rate and still disagree
     * with itself on the register, so both are asserted.
     */
    EXPECT(odr_actual_mag(&rm3100_ops, 200000) == 300000,
           "rm3100 200 resolves to 300");
    EXPECT(init_mag_reg(&rm3100_ops, RM_ADDR, 0x0B, 200000) ==
           init_mag_reg(&rm3100_ops, RM_ADDR, 0x0B, 300000),
           "rm3100 programs the resolved rate for an off-grid request");
    EXPECT(init_mag_reg(&rm3100_ops, RM_ADDR, 0x05, 200000) ==
           init_mag_reg(&rm3100_ops, RM_ADDR, 0x05, 300000),
           "rm3100 picks the resolved rate's cycle count too");

    /* ── The divider-based parts: the hook, not the table. ───────────── */

    /* MPU-925x: ODR = 1000/(1+SMPLRT_DIV), and the divider — not the rate —
     * is what gets rounded to nearest, so it reaches rates that are in no
     * table at all. 137 Hz becomes 1000/7 = 142, which is neither the 100 the
     * snap-up default would give nor the 125 the table's own grid holds. */
    EXPECT(mpu->actual_odr_mhz != NULL, "mpu implements actual_odr_mhz");
    EXPECT(odr_actual_imu(mpu, 137000) == 1000000 / 7,
           "mpu 137 -> 142.857 (divider 6)");
    EXPECT(odr_actual_imu(mpu, 1000000) == 1000000, "mpu 1000 -> 1000 (divider 0)");
    EXPECT(odr_actual_imu(mpu, 5000000) == 1000000,
           "mpu clamps above the base rate");
    EXPECT(odr_actual_imu(mpu, 1000) == 1000000 / 256,
           "mpu clamps at divider 255");
    /* And the resolved rate is a fixed point — the register is the same. */
    EXPECT(init_imu_reg(mpu, MPU_ADDR, 0x19, 137000, 8, 2000) ==
           init_imu_reg(mpu, MPU_ADDR, 0x19, 1000000 / 7, 8, 2000),
           "mpu SMPLRT_DIV is the same for 137 and its resolved 142");

    /* ICM-20948: ODR = 1125/(1+divider). */
    EXPECT(icm20948_ops.actual_odr_mhz != NULL, "icm20948 implements actual_odr_mhz");
    EXPECT(odr_actual_imu(&icm20948_ops, 1125000) == 1125000, "icm 1125 -> 1125");
    EXPECT(odr_actual_imu(&icm20948_ops, 500000) == 562500,
           "icm 500 -> 562.5 (divider 1)");
    EXPECT(odr_actual_imu(&icm20948_ops, 5000000) == 1125000,
           "icm clamps to base rate");

    /* ── The resolved rate is always a fixed point of resolution. ────── */
    static const int probe[] = { 1, 13, 60, 137, 500, 900, 5000 };
    for (size_t i = 0; i < sizeof probe / sizeof probe[0]; i++) {
        int a = odr_actual_imu(ism, probe[i]);
        EXPECT(odr_actual_imu(ism, a) == a, "ism resolution is idempotent");
        int m = odr_actual_mag(mmc, probe[i]);
        EXPECT(odr_actual_mag(mmc, m) == m, "mmc resolution is idempotent");
        int p = odr_actual_imu(mpu, probe[i]);
        EXPECT(odr_actual_imu(mpu, p) == p, "mpu resolution is idempotent");
    }

    end(fb);
}

/*
 * Every advertised rate encodes to the code its datasheet gives.
 *
 * test_odr_agreement above proves the driver is self-consistent; this proves
 * it is RIGHT.  The distinction matters because a driver's supported_odr_mhz
 * and its odr_encode() are edited together, so a table-derived expectation
 * would agree with a wrong encoding.  The codes below are transcribed from
 * the register tables and from nothing in src/ — that is the whole point, and
 * a mutation of any encode branch has to fail here.
 *
 * The register writes are masked down to the ODR field so the full-scale bits
 * stay out of it; those are pinned by the per-driver tests.
 */
static void test_odr_codes_match_datasheet(void)
{
    begin("test_odr_codes_match_datasheet");
    int fb = g_fail;

    /*
     * ST, DS13012 Rev 7 Table 43 (CTRL1_XL) and Table 46 (CTRL2_G).  The two
     * tables are identical over this range, which is what lets both drivers
     * write one shared code to both registers.
     */
    static const struct { int hz; uint8_t code; } st[] = {
        {   13016, 0x1 }, {   26031, 0x2 }, {   52063, 0x3 }, {  104125, 0x4 },
        {  208250, 0x5 }, {  416500, 0x6 }, {  833000, 0x7 }, { 1666000, 0x8 },
        { 3332000, 0x9 }, { 6664000, 0xA },
    };
    for (size_t i = 0; i < sizeof st / sizeof st[0]; i++) {
        char msg[96];
        snprintf(msg, sizeof msg, "ism %d Hz -> CTRL code 0x%X",
                 st[i].hz, st[i].code);
        EXPECT((init_imu_reg(ism, ISM_ADDR, 0x10, st[i].hz, 4, 500) >> 4)
               == st[i].code, msg);
        EXPECT((init_imu_reg(ism, ISM_ADDR, 0x11, st[i].hz, 4, 500) >> 4)
               == st[i].code, msg);
        /* FIFO_CTRL3 batches at the ODR: BDR_GY|BDR_XL, Table 29 §9.5, whose
         * codes run in lockstep with the ODR codes across this whole range. */
        EXPECT(init_imu_reg(ism, ISM_ADDR, 0x09, st[i].hz, 4, 500)
               == (uint8_t)((st[i].code << 4) | st[i].code), msg);

        snprintf(msg, sizeof msg, "lsm %d Hz -> CTRL code 0x%X",
                 st[i].hz, st[i].code);
        EXPECT((init_imu_reg(&lsm6dso_ops, LSM_ADDR, 0x10, st[i].hz, 4, 500) >> 4)
               == st[i].code, msg);
        EXPECT((init_imu_reg(&lsm6dso_ops, LSM_ADDR, 0x11, st[i].hz, 4, 500) >> 4)
               == st[i].code, msg);
    }

    /*
     * TDK, DS-000347 Rev 1.6 §5.6, GYRO_ODR (0x4F) and ACCEL_ODR (0x50).
     * Note 500 Hz: it is 1111, parked past the reserved codes at the end of
     * the field rather than in rate order between 200 Hz and 1 kHz.  Reading
     * that table in order is how 500 Hz came to be missing, and it is the one
     * row here that a plausible-looking encoder gets wrong.
     */
    static const struct { int hz; uint8_t code; } tdk[] = {
        {   12500, 0x0B }, {   25000, 0x0A }, {    50000, 0x09 },
        {  100000, 0x08 }, {  200000, 0x07 }, {   500000, 0x0F },
        { 1000000, 0x06 }, { 2000000, 0x05 }, {  4000000, 0x04 },
        { 8000000, 0x03 }, {16000000, 0x02 }, { 32000000, 0x01 },
    };
    for (size_t i = 0; i < sizeof tdk / sizeof tdk[0]; i++) {
        char msg[96];
        snprintf(msg, sizeof msg, "icm42688p %d Hz -> ODR code 0x%02X",
                 tdk[i].hz, tdk[i].code);
        EXPECT((init_imu_reg(&icm42688p_ops, ICM42_ADDR, 0x4F,
                             tdk[i].hz, 8, 2000) & 0x0F) == tdk[i].code, msg);
        EXPECT((init_imu_reg(&icm42688p_ops, ICM42_ADDR, 0x50,
                             tdk[i].hz, 8, 2000) & 0x0F) == tdk[i].code, msg);
    }

    /*
     * And the tables the daemon advertises hold exactly these rates — no more,
     * no less.  Without this an encoder could reach a rate no operator can
     * request, or a table could advertise one the encoder rounds away.
     */
    for (size_t i = 0; i < sizeof st / sizeof st[0]; i++) {
        EXPECT(ism->supported_odr_mhz[i] == st[i].hz,
               "ism advertises exactly the encodable rates");
        EXPECT(lsm6dso_ops.supported_odr_mhz[i] == st[i].hz,
               "lsm advertises exactly the encodable rates");
    }
    EXPECT(ism->supported_odr_mhz[sizeof st / sizeof st[0]] == 0,
           "ism table ends after 6664");
    for (size_t i = 0; i < sizeof tdk / sizeof tdk[0]; i++)
        EXPECT(icm42688p_ops.supported_odr_mhz[i] == tdk[i].hz,
               "icm42688p advertises exactly the encodable rates");
    EXPECT(icm42688p_ops.supported_odr_mhz[sizeof tdk / sizeof tdk[0]] == 0,
           "icm42688p table ends after 32000");

    end(fb);
}

/*
 * ticks_per_sample: the chip-timer spacing the FIFO drivers use to date the
 * samples inside a burst.
 *
 * This existed untested.  Each of these drivers keeps a private odr_actual()
 * whose only consumer is this arithmetic — the daemon-facing rounding goes
 * through odr_actual_imu() and supported_odr_mhz instead — so a wrong private
 * table was invisible to every other suite.  It was not hypothetical: the
 * hand-written loop bounds ("i < 7" over an 8-entry table) meant that adding
 * 3332 and 6664 Hz to ism330dhcx would have left odr_actual() clamping at
 * 1666, spacing samples 24 ticks apart when the part emits them 6 apart.  The
 * FIFO would drain correctly and every register assertion would pass; only
 * the timestamps would be wrong, by 4x, and imu.c would have believed them.
 *
 * The spacing is observable: read() dates the newest sample from the chip
 * counter and steps back one ticks_per_sample per older sample, so two
 * samples in one burst differ by exactly that value.
 *
 * Expected values are computed here from the datasheet tick period and the
 * resolved rate, not read out of the driver.  Both divisions are inexact at
 * some rungs (see the comments at each init) and the truncation is part of
 * what is being pinned — these are the values the drivers must produce, not
 * the values ideal arithmetic would give.
 */
/* MILLI-Hz. */
static uint32_t st_burst_ts_delta(const imu_ops_t *d, uint8_t addr, int odr_mhz)
{
    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = odr_mhz, .accel_g = 4, .gyro_dps = 500,
                      .fifo_wm = 64 };
    if (d->init(I2CBUS(addr), &cfg) != 0) return 0;

    i2cmock_set_reg(addr, 0x3A, 4);      /* FIFO_STATUS1: four words queued */
    i2cmock_set_reg(addr, 0x3B, 0);      /* no overflow */
    i2cmock_set_fifo_reg(addr, 0x78);
    /* ism_push_word serves both parts: the FIFO word format is the same and
     * ISM_ADDR == LSM_ADDR == 0x6A, which is the real ST 7-bit address. */
    ism_push_word(0x02, 1, 2, 3);        /* accel + gyro = sample 0 */
    ism_push_word(0x01, 4, 5, 6);
    ism_push_word(0x02, 7, 8, 9);        /* accel + gyro = sample 1 */
    ism_push_word(0x01, 10, 11, 12);

    /* Timestamp counter = 1000000 ticks, comfortably above any one spacing. */
    i2cmock_set_reg(addr, 0x40, 0x40);
    i2cmock_set_reg(addr, 0x41, 0x42);
    i2cmock_set_reg(addr, 0x42, 0x0F);
    i2cmock_set_reg(addr, 0x43, 0x00);

    imu_sample_t buf[8];
    int n = -1;
    if (d->read(I2CBUS(addr), buf, 8, &n) != 0 || n != 2) return 0;
    return buf[1].chip_ts - buf[0].chip_ts;
}

/* MILLI-Hz, as everywhere else in these tables. */
static uint32_t icm42_burst_ts_delta(int odr_mhz)
{
    i2cmock_reset();
    imu_cfg_t cfg = { .odr_mhz = odr_mhz, .accel_g = 4, .gyro_dps = 500,
                      .fifo_wm = 64 };
    if (icm42688p_ops.init(I2CBUS(ICM42_ADDR), &cfg) != 0) return 0;

    i2cmock_set_reg(ICM42_ADDR, 0x2E, 0x00);   /* FIFO_COUNT, big endian */
    i2cmock_set_reg(ICM42_ADDR, 0x2F, 32);     /* 32 bytes = two packets */
    i2cmock_set_fifo_reg(ICM42_ADDR, 0x30);
    icm42_push_packet(0x60, 1, 2, 3, 4, 5, 6, 5);
    icm42_push_packet(0x60, 7, 8, 9, 10, 11, 12, 5);

    /* Bank 1 TMSTVAL = 0x0F4240 = 1000000 µs.  Set after init, which writes
     * 0x64 as bank 0's INT_CONFIG1 — the mock register file is flat. */
    i2cmock_set_reg(ICM42_ADDR, 0x62, 0x40);
    i2cmock_set_reg(ICM42_ADDR, 0x63, 0x42);
    i2cmock_set_reg(ICM42_ADDR, 0x64, 0x0F);

    imu_sample_t buf[8];
    int n = -1;
    if (icm42688p_ops.read(I2CBUS(ICM42_ADDR), buf, 8, &n) != 0 || n != 2)
        return 0;
    return buf[1].chip_ts - buf[0].chip_ts;
}

static void test_ticks_per_sample_across_rates(void)
{
    begin("test_ticks_per_sample_across_rates");
    int fb = g_fail;
    char msg[112];

    /*
     * ST: 25 µs/tick (CTRL10_C timestamp counter), so 40000/rate ticks.
     * Inexact almost everywhere, and the error shrinks as the rate climbs:
     * 1666 Hz truncates 24.010 to 24 (0.04%), 3332 Hz 12.005 to 12 and
     * 6664 Hz 6.002 to 6 (both 0.04%).  The two rates added last are the
     * best-behaved on the ladder, not the worst.
     */
    static const int st_rates[] = { 13016, 26031, 52063, 104125, 208250,
                                    416500, 833000, 1666000, 3332000,
                                    6664000 };
    for (size_t i = 0; i < sizeof st_rates / sizeof st_rates[0]; i++) {
        const uint32_t want = 40000000u / (uint32_t)st_rates[i];
        snprintf(msg, sizeof msg, "ism %d Hz spaces samples %u ticks",
                 st_rates[i], want);
        EXPECT(st_burst_ts_delta(ism, ISM_ADDR, st_rates[i]) == want, msg);
        snprintf(msg, sizeof msg, "lsm %d Hz spaces samples %u ticks",
                 st_rates[i], want);
        EXPECT(st_burst_ts_delta(&lsm6dso_ops, LSM_ADDR, st_rates[i]) == want,
               msg);
    }

    /*
     * TDK: 32/30 µs per tick, NOT the 1 µs TMST_RES names — DS-000347 §12.7
     * scales the counter by 32/30 whenever the part is not clocked from CLKIN,
     * which is every configuration this driver programs.  So one second is
     * 937500 ticks and the spacing is 937500/rate.  Getting this wrong is a
     * 6.7% error on every per-sample dt until ts_anchor_t measures the real
     * period, which is why it is pinned rate by rate rather than spot-checked.
     *
     * 937500 = 2^2 * 3 * 5^7, so it divides 25, 50, 100 and 500 exactly and
     * nothing else on the ladder: 200 Hz is 0.011% short, 1 kHz 0.053%, 2-8 kHz
     * 0.16%, and the top two rungs 1.0%.  Bounded per burst rather than
     * cumulative — the anchor is re-read every drain.
     *
     * The bottom rung is the one place the integer ladder itself costs
     * accuracy.  The part runs 12.5 Hz, whose exact spacing is 937500/12.5 =
     * 75000, and supported_odr_mhz cannot hold 12.5.  Declaring 12 divided
     * 937500 exactly -- and landed on 78125, 4.2% wrong.  13 gives 72115, 3.9%
     * wrong in the other direction.  Neither is right; the exactness at 12 was
     * a coincidence around a wrong rate, which is worth saying because it
     * looks like the better number and is not.
     */
    static const int tdk_rates[] = { 12500, 25000, 50000, 100000, 200000,
                                     500000, 1000000, 2000000, 4000000,
                                     8000000, 16000000, 32000000 };
    for (size_t i = 0; i < sizeof tdk_rates / sizeof tdk_rates[0]; i++) {
        const uint32_t want = 937500000u / (uint32_t)tdk_rates[i];
        snprintf(msg, sizeof msg, "icm42688p %d Hz spaces samples %u ticks",
                 tdk_rates[i], want);
        EXPECT(icm42_burst_ts_delta(tdk_rates[i]) == want, msg);
    }
    /* The three rungs where the truncation is worth naming outright. */
    EXPECT(icm42_burst_ts_delta(1000000)  == 937, "1 kHz truncates 937.5 to 937");
    EXPECT(icm42_burst_ts_delta(16000000) == 58,  "16 kHz truncates 58.59 to 58");
    EXPECT(icm42_burst_ts_delta(32000000) == 29,  "32 kHz truncates 29.30 to 29");

    /*
     * Off-grid requests round UP to the next advertised rate, and the spacing
     * follows the rate the part is actually programmed to — the private table
     * and the encoder have to agree at the top, which is precisely what a
     * stale loop bound breaks.
     */
    EXPECT(st_burst_ts_delta(ism, ISM_ADDR, 5000000) == 40000000u / 6664000u,
           "ism 5000 Hz is spaced for its resolved 6664, not the old 1666 cap");
    EXPECT(st_burst_ts_delta(ism, ISM_ADDR, 99000000) == 40000000u / 6664000u,
           "ism clamps above the top rung and spaces for it");
    EXPECT(icm42_burst_ts_delta(300000) == 937500000u / 500000u,
           "icm42688p 300 Hz is spaced for 500, the rate 0x0F selects");
    EXPECT(icm42_burst_ts_delta(99000000) == 937500000u / 32000000u,
           "icm42688p clamps above the top rung and spaces for it");

    end(fb);
}

/* ── Dual-transport agreement ────────────────────────────────────────────── */

/*
 * The point of the bus abstraction: one driver, two transports, identical
 * device traffic.
 *
 * Each part is driven through its whole probe/reset/init/read sequence twice —
 * once over I2C, once over SPI — against two register files staged the same
 * way. Then every one of the 256 registers is compared. The register
 * expectations are transport-independent, so any difference is a framing bug:
 * a mis-set direction bit, a missed auto-increment, a command byte landing at
 * reg|0x80.
 *
 * This is deliberately a comparison rather than a second set of hardcoded
 * expectations. Hardcoding them twice would let both copies drift together;
 * comparing pins them to each other.
 */
#define DUAL_SPI_FD 9

/*
 * CTRL4_C (0x13) is EXPECTED to differ between the transports and is excluded.
 *
 * DS13012's initialisation procedure requires opposite values for it: step 3a
 * sets I2C_disable = 1 for SPI, step 3b leaves it 0 for I2C.  A driver that
 * left this register identical on both would be wrong on one of them -- which
 * is what a broken implementation does, never writing the register at all and
 * so running the I2C block live on every SPI install.
 *
 * Nothing else may differ, which is the invariant this helper exists for.  The
 * ST callers assert the CTRL4_C difference itself, so excluding it here
 * tolerates nothing: it is checked, just not by this comparison.
 */
#define CTRL4_C_REG 0x13

static int reg_files_differ(uint8_t a, uint8_t b, int *first)
{
    int n = 0;
    for (int r = 0; r < 256; r++) {
        if (r == CTRL4_C_REG) continue;
        if (i2cmock_get_reg(a, (uint8_t)r) != i2cmock_get_reg(b, (uint8_t)r)) {
            if (n == 0 && first) *first = r;
            n++;
        }
    }
    return n;
}

static void test_dual_transport_ism330dhcx(void)
{
    begin("test_dual_transport_ism330dhcx");
    int fb = g_fail;

    const uint8_t I2C_AT = 0x6A, SPI_AT = 0x2A;

    i2cmock_reset();
    spimock_bind(DUAL_SPI_FD, SPI_AT, 0);   /* IF_INC part: no inc mask */

    /* Stage both devices identically. */
    for (int i = 0; i < 2; i++) {
        uint8_t at = i ? SPI_AT : I2C_AT;
        i2cmock_set_reg(at, 0x0F, 0x6B);            /* WHO_AM_I */
        i2cmock_set_reg(at, 0x20, 0x00);            /* OUT_TEMP_L */
        i2cmock_set_reg(at, 0x21, 0x00);            /* OUT_TEMP_H */
        i2cmock_set_selfclear(at, 0x12, 0x01);      /* SW_RESET self-clears */
        i2cmock_set_reg(at, 0x3A, 3);               /* FIFO_STATUS1: 3 words */
        i2cmock_set_reg(at, 0x3B, 0);
        i2cmock_set_fifo_range(at, 0x78, 0x7E);
    }

    imud_bus_t ib = { .kind = BUS_I2C, .fd = FD, .i2c_addr = I2C_AT };
    /* Framing from the driver's own caps, as bus_open() does — see the
     * mmc5983ma case below for why a literal here is circular. */
    imud_bus_t sb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD,
                      .spi_mode     = ism->bus_caps.spi_mode,
                      .spi_inc_mask = ism->bus_caps.spi_inc_mask,
                      .spi_hz       = 10000000 };

    EXPECT(ism->probe(&ib) == 0, "i2c probe");
    EXPECT(ism->probe(&sb) == 0, "spi probe");
    EXPECT(ism->reset(&ib) == 0, "i2c reset");
    EXPECT(ism->reset(&sb) == 0, "spi reset");

    imu_cfg_t cfg = { .odr_mhz = 208000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    EXPECT(ism->init(&ib, &cfg) == 0, "i2c init");
    EXPECT(ism->init(&sb, &cfg) == 0, "spi init");

    int first = -1;
    int diffs = reg_files_differ(I2C_AT, SPI_AT, &first);
    if (diffs) {
        printf("\n  first differing register 0x%02X: i2c=0x%02X spi=0x%02X\n",
               first, i2cmock_get_reg(I2C_AT, (uint8_t)first),
               i2cmock_get_reg(SPI_AT, (uint8_t)first));
    }
    EXPECT(diffs == 0,
           "init leaves identical registers on both transports, CTRL4_C aside");
    EXPECT((i2cmock_get_reg(SPI_AT, CTRL4_C_REG) & 0x04) != 0 &&
           (i2cmock_get_reg(I2C_AT, CTRL4_C_REG) & 0x04) == 0,
           "and CTRL4_C differs in the one way the datasheet requires");

    /*
     * And the read path: same FIFO bytes in, same samples out.
     *
     * The temperature word goes FIRST on purpose. last_temp is driver state
     * that persists across drains (the ISM batches temperature an order of
     * magnitude below accel/gyro), so with temperature last, the I2C pass
     * would emit its sample before seeing it and the SPI pass would inherit
     * what the I2C pass stored — a difference in call order, not transport.
     * Feeding it first makes temp_c a function of the bytes alone, so it can
     * be compared like everything else.
     */
    static const struct { uint8_t tag; int16_t x, y, z; } words[3] = {
        { 0x03, 1234, 0, 0 },         /* temperature */
        { 0x01, 100, -200, 300 },     /* gyro */
        { 0x02, -400, 500, -600 },    /* accel */
    };
    for (int i = 0; i < 2; i++) {
        uint8_t at = i ? SPI_AT : I2C_AT;
        for (int w = 0; w < 3; w++) {
            uint8_t b[7] = {
                (uint8_t)(words[w].tag << 3),
                (uint8_t)(words[w].x & 0xFF), (uint8_t)((words[w].x >> 8) & 0xFF),
                (uint8_t)(words[w].y & 0xFF), (uint8_t)((words[w].y >> 8) & 0xFF),
                (uint8_t)(words[w].z & 0xFF), (uint8_t)((words[w].z >> 8) & 0xFF),
            };
            i2cmock_fifo_push(at, b, 7);
        }
    }

    imu_sample_t ibuf[8] = { 0 }, sbuf[8] = { 0 };
    int in = 0, sn = 0;
    EXPECT(ism->read(&ib, ibuf, 8, &in) == 0, "i2c read");
    EXPECT(ism->read(&sb, sbuf, 8, &sn) == 0, "spi read");
    EXPECT(in == sn && in > 0, "both transports decoded the same sample count");

    /*
     * The fields a DRIVER fills, not the whole struct. accel_raw/field_raw are
     * written later by imu.c's calibration stage and are untouched here, and
     * read_done_ns is a CLOCK_REALTIME reading — comparing either would be
     * comparing noise. seq is excluded for the same reason as last_temp above:
     * it is a counter that keeps climbing across both calls.
     *
     * chip_ts is excluded on exactly that ground too, and it is worth saying
     * why. The mock returns one fixed value from
     * the timestamp register, so both reads compute the same burst time — an
     * overlap — and the monotonic guard (chip_ts.h) correctly pushes the second
     * burst past the first. Requiring the two to be EQUAL would be requiring
     * the driver to emit a timestamp it has already used, which is the defect
     * the guard exists to prevent. What transport equivalence actually claims
     * about time is asserted below: identical within-burst spacing.
     */
    bool same = (in == sn);
    for (int i = 0; same && i < in; i++)
        same = memcmp(ibuf[i].accel, sbuf[i].accel, sizeof ibuf[i].accel) == 0
            && memcmp(ibuf[i].gyro,  sbuf[i].gyro,  sizeof ibuf[i].gyro)  == 0
            && ibuf[i].temp_c  == sbuf[i].temp_c;
    EXPECT(same, "both transports decoded identical accel/gyro/temp");

    /* The per-sample spacing is what the chip supplies and what the transport
     * must not change; the burst's absolute offset is the guard's business. */
    bool same_dt = (in == sn);
    for (int i = 1; same_dt && i < in; i++)
        same_dt = (ibuf[i].chip_ts - ibuf[i - 1].chip_ts)
               == (sbuf[i].chip_ts - sbuf[i - 1].chip_ts);
    EXPECT(same_dt, "both transports decoded identical chip_ts spacing");

    /* And the guard did push the second burst clear of the first. */
    if (in > 0 && sn > 0)
        EXPECT((int32_t)(sbuf[0].chip_ts - ibuf[in - 1].chip_ts) > 0,
               "the second burst starts after the first ended");

    end(fb);
}

static void test_dual_transport_mmc5983ma(void)
{
    begin("test_dual_transport_mmc5983ma");
    int fb = g_fail;

    const uint8_t I2C_AT = 0x30, SPI_AT = 0x31;

    i2cmock_reset();
    /*
     * Ground truth about the silicon, written here independently of the
     * driver: Rev A pp.6-7 say a multi-byte transfer simply adds 8-clock
     * blocks, so the MMC5983MA walks the address on its own and has no
     * auto-increment bit to set.
     */
    spimock_bind_inc(DUAL_SPI_FD, SPI_AT, SPIMOCK_INC_ALWAYS, 0);

    for (int i = 0; i < 2; i++) {
        uint8_t at = i ? SPI_AT : I2C_AT;
        i2cmock_set_reg(at, 0x2F, 0x30);        /* PRODUCT_ID */
        i2cmock_set_reg(at, 0x08, 0x01);        /* STATUS: Meas_M_Done */
        /* 18-bit X/Y/Z output, distinct per axis. */
        const uint8_t raw[7] = { 0x91, 0x23, 0x45, 0x67, 0x89, 0xAB, 0x55 };
        i2cmock_set_regs(at, 0x00, raw, 7);
    }

    imud_bus_t ib = { .kind = BUS_I2C, .fd = FD, .i2c_addr = I2C_AT };
    /*
     * The bus takes its framing FROM the driver's declared caps, exactly as
     * bus_open() does in production.  Writing the mask as a literal here made
     * the whole comparison circular: the test asserted its own copy of the
     * driver's number against the mock bound with that same number, so a wrong
     * declaration agreed with itself and nothing failed.
     */
    imud_bus_t sb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD,
                      .spi_mode      = mmc->bus_caps.spi_mode,
                      .spi_inc_mask  = mmc->bus_caps.spi_inc_mask,
                      .spi_hz        = 10000000 };

    EXPECT(mmc->probe(&ib) == 0, "i2c probe");
    EXPECT(mmc->probe(&sb) == 0, "spi probe");

    mag_cfg_t cfg = { .odr_mhz = 100000, .set_period_s = 0.0f };
    EXPECT(mmc->init(&ib, &cfg) == 0, "i2c init");
    EXPECT(mmc->init(&sb, &cfg) == 0, "spi init");

    int first = -1;
    int diffs = reg_files_differ(I2C_AT, SPI_AT, &first);
    if (diffs) {
        printf("\n  first differing register 0x%02X: i2c=0x%02X spi=0x%02X\n",
               first, i2cmock_get_reg(I2C_AT, (uint8_t)first),
               i2cmock_get_reg(SPI_AT, (uint8_t)first));
    }
    EXPECT(diffs == 0, "init leaves identical registers on both transports");

    /* Re-arm the data-ready flag both sides cleared, then read. */
    i2cmock_set_reg(I2C_AT, 0x08, 0x01);
    i2cmock_set_reg(SPI_AT, 0x08, 0x01);
    mag_sample_t im = { 0 }, sm = { 0 };
    EXPECT(mmc->read(&ib, &im) == 0, "i2c read");
    EXPECT(mmc->read(&sb, &sm) == 0, "spi read");
    /* field and valid only: field_raw is imu.c's to fill and wall_ns is a
     * clock reading, so neither says anything about the transport. */
    EXPECT(memcmp(im.field, sm.field, sizeof im.field) == 0 &&
           im.valid == sm.valid,
           "both transports decoded an identical field vector");

    /*
     * set_reset is a write-only path — it must reach the same register. Assert
     * the VALUE too, not just that the two agree: agreeing on a wrong value is
     * what this passed on while both transports dropped INT_EN.
     */
    EXPECT(mmc->set_reset(&ib) == 0, "i2c set_reset");
    EXPECT(mmc->set_reset(&sb) == 0, "spi set_reset");
    EXPECT(i2cmock_get_reg(I2C_AT, 0x09) == i2cmock_get_reg(SPI_AT, 0x09),
           "set_reset wrote the same CTRL0 on both transports");
    EXPECT(i2cmock_get_reg(I2C_AT, 0x09) == 0x0C,
           "...and it was SET | INT_EN, on both");

    EXPECT(mmc->degauss(&ib, MAG_DEGAUSS_RESET) == 0, "i2c degauss RESET");
    EXPECT(mmc->degauss(&sb, MAG_DEGAUSS_RESET) == 0, "spi degauss RESET");
    EXPECT(i2cmock_get_reg(I2C_AT, 0x09) == i2cmock_get_reg(SPI_AT, 0x09) &&
           i2cmock_get_reg(I2C_AT, 0x09) == 0x14,
           "degauss RESET wrote RESET | INT_EN on both transports");

    /*
     * Everything above compares the two transports against each other, which
     * says they agree and nothing about whether either is right.  Stage a known
     * field on the SPI side and assert the absolute value.
     */
    i2cmock_set_reg(SPI_AT, 0x08, 0x01);
    mmc_set_output_at(SPI_AT, 131072 + 16384,    /* +1 G  -> +100 uT      */
                              131072 - 16384,    /* -1 G, Y flipped -> +100 */
                              131072 +  8192);   /* +0.5 G -> +50 uT      */
    mag_sample_t abs_s = { 0 };
    EXPECT(mmc->read(&sb, &abs_s) == 0, "spi read of a staged field");
    EXPECT_NEAR(abs_s.field[0], 100.0f, 0.1, "spi decodes X = +100 uT");
    EXPECT_NEAR(abs_s.field[1], 100.0f, 0.1, "spi decodes Y flipped = +100 uT");
    EXPECT_NEAR(abs_s.field[2],  50.0f, 0.1, "spi decodes Z = +50 uT");
    /* And the burst walked, rather than returning 0x00 seven times — the
     * guard lis3mdl carries, and the one this test lacked. */
    EXPECT(abs_s.field[0] != abs_s.field[2],
           "the spi burst walked the output registers");

    /*
     * The part the mock cannot otherwise express.  mmc5983ma declares
     * spi_inc_mask = 0, meaning "this part walks the address by itself".  Bind
     * the same address INC_NEVER — a part that needs an explicit bit, which is
     * what a wrong declaration would amount to — and the identical driver code
     * must now decode WRONG.  Without this, the mask assertion is the test
     * agreeing with the literal it copied out of the driver.
     */
    i2cmock_reset();
    spimock_bind_inc(DUAL_SPI_FD, SPI_AT, SPIMOCK_INC_NEVER, 0);
    i2cmock_set_reg(SPI_AT, 0x2F, 0x30);
    i2cmock_set_reg(SPI_AT, 0x08, 0x01);
    mmc_set_output_at(SPI_AT, 131072 + 16384, 131072 - 16384, 131072 + 8192);

    mag_sample_t stuck = { 0 };
    EXPECT(mmc->read(&sb, &stuck) == 0, "spi read against a non-incrementing part");
    EXPECT(!(fabsf(stuck.field[0] - 100.0f) < 0.1f &&
             fabsf(stuck.field[2] -  50.0f) < 0.1f),
           "a part that does not auto-increment decodes wrong, so the mask matters");
    /*
     * X and Z were staged a factor of two apart. With the pointer stuck they
     * decode to the same magnitude, because every byte of the burst came from
     * register 0x00 — the low two bits differ only because XYZOUT2's three
     * bit-fields are read out of that one repeated byte.
     */
    EXPECT(fabsf(fabsf(stuck.field[0]) - fabsf(stuck.field[2])) < 1.0f,
           "...because the burst returned one register over and over");

    end(fb);
}

/*
 * The rest of the SPI-capable parts. probe + init only: their read paths need
 * part-specific FIFO staging, and what a second transport can get wrong is the
 * addressing, which every control-register write exercises.
 */
static void test_dual_transport_others(void)
{
    begin("test_dual_transport_others");
    int fb = g_fail;
    char msg[96];

    struct icase {
        const char     *name;
        const imu_ops_t *ops;
        uint8_t         i2c_at, spi_at, inc;
        uint8_t         whoami_reg, whoami_val;
        uint8_t         rst_reg, rst_bit;
    };
    static const struct icase imus[] = {
        { "lsm6dso",   &lsm6dso_ops,   0x6A, 0x1A, 0, 0x0F, 0x6C, 0x12, 0x01 },
        { "lsm6dsox",  &lsm6dsox_ops,  0x6B, 0x1B, 0, 0x0F, 0x6C, 0x12, 0x01 },
        { "icm42688p", &icm42688p_ops, 0x68, 0x18, 0, 0x75, 0x47, 0x11, 0x01 },
    };

    imu_cfg_t icfg = { .odr_mhz = 200000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 32 };

    for (unsigned i = 0; i < sizeof imus / sizeof imus[0]; i++) {
        const struct icase *c = &imus[i];
        i2cmock_reset();
        spimock_bind(DUAL_SPI_FD, c->spi_at, c->inc);
        for (int k = 0; k < 2; k++) {
            uint8_t at = k ? c->spi_at : c->i2c_at;
            i2cmock_set_reg(at, c->whoami_reg, c->whoami_val);
            i2cmock_set_selfclear(at, c->rst_reg, c->rst_bit);
        }
        imud_bus_t ib = { .kind = BUS_I2C, .fd = FD, .i2c_addr = c->i2c_at };
        imud_bus_t sb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD, .spi_mode = 3,
                          .spi_inc_mask = c->inc, .spi_hz = 10000000 };

        snprintf(msg, sizeof msg, "%s: probe agrees on both transports", c->name);
        EXPECT((c->ops->probe(&ib) == 0) == (c->ops->probe(&sb) == 0), msg);
        (void)c->ops->reset(&ib);
        (void)c->ops->reset(&sb);
        snprintf(msg, sizeof msg, "%s: init agrees on both transports", c->name);
        EXPECT((c->ops->init(&ib, &icfg) == 0) == (c->ops->init(&sb, &icfg) == 0), msg);

        int first = -1;
        int diffs = reg_files_differ(c->i2c_at, c->spi_at, &first);
        if (diffs)
            printf("\n  %s: first differing register 0x%02X: i2c=0x%02X spi=0x%02X\n",
                   c->name, first, i2cmock_get_reg(c->i2c_at, (uint8_t)first),
                   i2cmock_get_reg(c->spi_at, (uint8_t)first));
        snprintf(msg, sizeof msg, "%s: identical registers after init", c->name);
        EXPECT(diffs == 0, msg);
    }

    /*
     * LIS3MDL gets the read path too, because it is the one part whose
     * auto-increment bit differs between the transports: I2C sets the
     * sub-address MSB (0x80), SPI sets MS (0x40) with 0x80 meaning read. Real
     * data is staged at BOTH landing points so a correct driver reads the same
     * six bytes either way — and a driver that dropped either bit does not.
     */
    const uint8_t L_I2C = 0x1C, L_SPI = 0x3C;
    i2cmock_reset();
    /* Ground truth: DS9463 Rev 7 §5.2 — the address walks only when MS (0x40)
     * is set in the command byte.  Written here, not read from the driver. */
    spimock_bind_inc(DUAL_SPI_FD, L_SPI, SPIMOCK_INC_ON_BIT, 0x40);

    const uint8_t out[6] = { 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01 };  /* 100,200,300 */
    for (int k = 0; k < 2; k++) {
        uint8_t at = k ? L_SPI : L_I2C;
        i2cmock_set_reg(at, 0x0F, 0x3D);        /* WHO_AM_I */
        i2cmock_set_reg(at, 0x27, 0x08);        /* STATUS: ZYXDA */
        i2cmock_set_regs(at, 0x28, out, 6);     /* where SPI lands */
        i2cmock_set_regs(at, 0xA8, out, 6);     /* where I2C lands */
    }

    imud_bus_t lib = { .kind = BUS_I2C, .fd = FD, .i2c_addr = L_I2C };
    imud_bus_t lsb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD,
                       .spi_mode     = lis3mdl_ops.bus_caps.spi_mode,
                       .spi_inc_mask = lis3mdl_ops.bus_caps.spi_inc_mask,
                       .spi_hz       = 10000000 };
    const mag_ops_t *l3 = &lis3mdl_ops;
    mag_cfg_t mcfg = { .odr_mhz = 80000, .set_period_s = 0.0f };

    EXPECT(l3->probe(&lib) == 0 && l3->probe(&lsb) == 0, "lis3mdl: probe on both");
    EXPECT(l3->init(&lib, &mcfg) == 0 && l3->init(&lsb, &mcfg) == 0,
           "lis3mdl: init on both");
    int lfirst = -1;
    EXPECT(reg_files_differ(L_I2C, L_SPI, &lfirst) == 0,
           "lis3mdl: identical registers after init");

    mag_sample_t lm = { 0 }, ls = { 0 };
    EXPECT(l3->read(&lib, &lm) == 0, "lis3mdl: i2c read");
    EXPECT(l3->read(&lsb, &ls) == 0, "lis3mdl: spi read");
    EXPECT(memcmp(lm.field, ls.field, sizeof lm.field) == 0 && lm.valid == ls.valid,
           "lis3mdl: same field despite different auto-increment bits");
    /* And it is the real vector, not six copies of one register. */
    EXPECT(ls.field[0] != ls.field[1] && ls.field[1] != ls.field[2],
           "lis3mdl: spi burst walked the output registers");

    /*
     * RM3100 is the inverse case, and the reason it is worth a second read
     * path here.  Its datasheet lists reads as 0x84/0xA4/0xB4, which looks
     * like an I2C sub-address modifier and is not — it is the SPI command
     * byte, and the I2C side sends the plain address.  So real data is staged
     * ONLY at the unmodified registers: a driver that copied lis3mdl and OR'd
     * 0x80 on I2C reads zeros here and fails, which is precisely the mistake
     * the datasheet's table invites.
     */
    const uint8_t R_I2C = 0x20, R_SPI = 0x21;
    i2cmock_reset();
    spimock_bind(DUAL_SPI_FD, R_SPI, 0);

    /* 3750, -1500, 7500 as 24-bit big-endian -> +50, -20, +100 µT at CC 200. */
    const uint8_t rout[9] = { 0x00, 0x0E, 0xA6,
                              0xFF, 0xFA, 0x24,
                              0x00, 0x1D, 0x4C };
    for (int k = 0; k < 2; k++) {
        uint8_t at = k ? R_SPI : R_I2C;
        i2cmock_set_reg(at, 0x36, 0x22);        /* REVID */
        i2cmock_set_reg(at, 0x34, 0x80);        /* STATUS: DRDY */
        i2cmock_set_regs(at, 0x24, rout, 9);
    }

    imud_bus_t rib = { .kind = BUS_I2C, .fd = FD, .i2c_addr = R_I2C };
    imud_bus_t rsb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD, .spi_mode = 3,
                       .spi_inc_mask = rm3100_ops.bus_caps.spi_inc_mask,
                       .spi_hz = 1000000 };
    const mag_ops_t *rm = &rm3100_ops;
    mag_cfg_t rcfg = { .odr_mhz = 100000, .set_period_s = 0.0f };

    EXPECT(rm->probe(&rib) == 0 && rm->probe(&rsb) == 0, "rm3100: probe on both");
    EXPECT(rm->init(&rib, &rcfg) == 0 && rm->init(&rsb, &rcfg) == 0,
           "rm3100: init on both");
    int rfirst = -1;
    EXPECT(reg_files_differ(R_I2C, R_SPI, &rfirst) == 0,
           "rm3100: identical registers after init");

    mag_sample_t rmi = { 0 }, rms = { 0 };
    EXPECT(rm->read(&rib, &rmi) == 0, "rm3100: i2c read");
    EXPECT(rm->read(&rsb, &rms) == 0, "rm3100: spi read");
    EXPECT(memcmp(rmi.field, rms.field, sizeof rmi.field) == 0 &&
           rmi.valid == rms.valid, "rm3100: same field on both transports");
    EXPECT_NEAR(rmi.field[0],  50.0f, 1e-3, "rm3100: i2c read hit the plain "
                                            "register, not 0xA4");
    EXPECT_NEAR(rms.field[1], -20.0f, 1e-3, "rm3100: spi burst walked all nine "
                                            "result bytes");

    end(fb);
}

/* ── SPI transport ───────────────────────────────────────────────────────── */

/*
 * The framing itself, independent of any driver: command byte direction,
 * register decode, and the burst auto-increment.  These assertions are what
 * make the dual-transport driver tests meaningful — if the framing were
 * wrong, both transports would simply be wrong together.
 *
 * SPI_FD is a descriptor number, not an open file: the mock intercepts every
 * ioctl, so nothing is ever really transferred.
 */
#define SPI_FD   7
#define SPI_ADDR 0x40   /* an arbitrary register file to bind the fd to */

static imud_bus_t spi_bus(uint8_t inc_mask)
{
    i2cmock_reset();
    spimock_bind(SPI_FD, SPI_ADDR, inc_mask);
    return (imud_bus_t){ .kind = BUS_SPI, .fd = SPI_FD,
                         .spi_mode = 3, .spi_inc_mask = inc_mask,
                         .spi_hz = 10000000 };
}

static void test_spi_framing(void)
{
    begin("test_spi_framing");
    int fb = g_fail;

    imud_bus_t b = spi_bus(0);

    /* A read reaches the register file the descriptor is bound to. */
    i2cmock_set_reg(SPI_ADDR, 0x0F, 0x6B);
    uint8_t v = 0;
    EXPECT(bus_reg_read(&b, 0x0F, &v) == 0, "spi single read succeeds");
    EXPECT(v == 0x6B, "spi read returns the register value");

    /* A write must clear the direction bit, or it would land at reg|0x80. */
    EXPECT(bus_reg_write(&b, 0x12, 0x44) == 0, "spi write succeeds");
    EXPECT(i2cmock_get_reg(SPI_ADDR, 0x12) == 0x44, "spi write hits the register");
    EXPECT(i2cmock_get_reg(SPI_ADDR, 0x92 & 0xFF) == 0x00,
           "spi write did not land at reg|0x80");

    /* Burst reads walk forward through the file. */
    const uint8_t seq[4] = { 0x11, 0x22, 0x33, 0x44 };
    i2cmock_set_regs(SPI_ADDR, 0x28, seq, 4);
    uint8_t got[4] = { 0 };
    EXPECT(bus_burst_read(&b, 0x28, got, 4) == 0, "spi burst read succeeds");
    EXPECT(memcmp(got, seq, 4) == 0, "spi burst auto-increments");

    /* A FIFO window pops on every read, exactly as on I2C. */
    i2cmock_set_fifo_range(SPI_ADDR, 0x78, 0x7E);
    const uint8_t staged[3] = { 0xA1, 0xB2, 0xC3 };
    i2cmock_fifo_push(SPI_ADDR, staged, 3);
    uint8_t popped[3] = { 0 };
    EXPECT(bus_burst_read(&b, 0x78, popped, 3) == 0, "spi fifo read succeeds");
    EXPECT(memcmp(popped, staged, 3) == 0, "spi reads pop the fifo window");

    /* Transport errors still surface as -1. */
    i2cmock_fail_next_xfer();
    EXPECT(bus_reg_read(&b, 0x0F, &v) == -1, "spi read propagates transfer failure");
    i2cmock_fail_next_xfer();
    EXPECT(bus_reg_write(&b, 0x12, 0x00) == -1, "spi write propagates transfer failure");

    printf("%s\n", g_fail == fb ? "OK" : "FAIL");
}

/*
 * The SHAPE of each transfer bus_io.h asks the backend for — how many legs,
 * and how wide each one's words are.  No register readback can show this: a
 * one-byte read sent as two 8-bit legs leaves exactly the same byte in the
 * register file as the 16-bit single word bus_io.h actually sends.
 *
 * It is worth pinning because the shape is doing real work.  RP1 deasserts
 * chip-select between words (raspberrypi/linux#6354), so a multi-word
 * transfer can be cut at any boundary; a single read is framed as ONE 16-bit
 * word precisely so it has no boundary to be cut at.  A change that split it
 * would pass every other assertion in this file and corrupt the part's state
 * on real silicon.
 */
static void test_backend_leg_shapes(void)
{
    begin("test_backend_leg_shapes");
    int fb = g_fail;

    imud_bus_t b = spi_bus(0);
    uint8_t v = 0;

    /* Single read: one 16-bit word, command in the high byte. */
    i2cmock_set_reg(SPI_ADDR, 0x0F, 0x6B);
    EXPECT(bus_reg_read(&b, 0x0F, &v) == 0, "single read succeeds");
    EXPECT(spimock_last_nlegs() == 1, "single read is ONE leg");
    EXPECT(spimock_last_bits(0) == 16, "and 16 bits per word");
    EXPECT(spimock_last_len(0) == 2, "carrying command and data in 2 bytes");

    /* Write: likewise one 16-bit word. */
    EXPECT(bus_reg_write(&b, 0x12, 0x44) == 0, "write succeeds");
    EXPECT(spimock_last_nlegs() == 1, "write is ONE leg");
    EXPECT(spimock_last_bits(0) == 16, "and 16 bits per word");
    EXPECT(spimock_last_len(0) == 2, "carrying register and value in 2 bytes");

    /* Burst read: two 8-bit legs, command then data — it cannot be made
     * atomic, so it is framed to be harmless when split instead. */
    uint8_t got[6] = { 0 };
    EXPECT(bus_burst_read(&b, 0x28, got, 6) == 0, "burst read succeeds");
    EXPECT(spimock_last_nlegs() == 2, "burst read is TWO legs");
    EXPECT(spimock_last_bits(0) == 8 && spimock_last_bits(1) == 8,
           "both 8 bits per word");
    EXPECT(spimock_last_len(0) == 1, "a 1-byte command leg");
    EXPECT(spimock_last_len(1) == 6, "then a data leg of the requested length");

    /* The boundary between the two: 2 bytes is already a burst, not a word. */
    EXPECT(bus_burst_read(&b, 0x28, got, 2) == 0, "2-byte read succeeds");
    EXPECT(spimock_last_nlegs() == 2, "a 2-byte read is a burst, not one word");

    printf("%s\n", g_fail == fb ? "OK" : "FAIL");
}

/*
 * The parts that need an explicit auto-increment bit (LIS3MDL's MS at 0x40)
 * must set it on a burst and NOT on a single read — on those parts the bit is
 * part of the command, so a single read that sets it would address the wrong
 * register.
 */
static void test_spi_inc_mask(void)
{
    begin("test_spi_inc_mask");
    int fb = g_fail;

    imud_bus_t b = spi_bus(0x40);

    /* Single read: the mask must not be set, so 0x28 stays 0x28. */
    i2cmock_set_reg(SPI_ADDR, 0x28, 0x5A);
    uint8_t v = 0;
    EXPECT(bus_reg_read(&b, 0x28, &v) == 0, "inc-mask single read succeeds");
    EXPECT(v == 0x5A, "single read addresses the register, not reg|mask");

    /* Burst: the mask is set, so the address walks. */
    const uint8_t seq[6] = { 1, 2, 3, 4, 5, 6 };
    i2cmock_set_regs(SPI_ADDR, 0x28, seq, 6);
    uint8_t got[6] = { 0 };
    EXPECT(bus_burst_read(&b, 0x28, got, 6) == 0, "inc-mask burst succeeds");
    EXPECT(memcmp(got, seq, 6) == 0, "burst with the mask walks the registers");

    /*
     * And the omission is detectable: a handle that does not set the bit on a
     * part that needs it gets one register back over and over, which is what
     * the silicon does (LIS3MDL DS9463 Rev 7 §5.2). Without the mock modelling
     * that, forgetting spi_inc_mask would pass silently — so this asserts the
     * harness as much as the framing.
     */
    imud_bus_t forgot = b;
    forgot.spi_inc_mask = 0;
    uint8_t rep[4] = { 0 };
    EXPECT(bus_burst_read(&forgot, 0x28, rep, 4) == 0, "burst without the bit transfers");
    EXPECT(rep[0] == 1 && rep[1] == 1 && rep[2] == 1 && rep[3] == 1,
           "without the MS bit the part repeats one register");

    printf("%s\n", g_fail == fb ? "OK" : "FAIL");
}

/*
 * bus_open's policy, which is the same for all three tools that use it.
 * /dev/null stands in for a spidev node: the mock swallows every ioctl, so
 * only the open() itself is real.
 */
static void test_bus_open_policy(void)
{
    begin("test_bus_open_policy");
    int fb = g_fail;
    i2cmock_reset();

    bus_spec_t spec = { .kind = BUS_SPI, .node = "/dev/null", .spi_hz = 0 };
    bus_caps_t caps = { .spi_capable = true, .spi_mode = 3,
                        .spi_max_hz = 10000000, .spi_inc_mask = 0 };
    imud_bus_t b;

    /* A driver with no SPI port is refused before anything is opened. */
    bus_caps_t nospi = caps;
    nospi.spi_capable = false;
    EXPECT(bus_open(&b, &spec, &nospi, "test") == -1, "spi refused without a port");
    EXPECT(b.fd < 0, "refused open leaves the handle closed");
    EXPECT(bus_open(&b, &spec, NULL, "test") == -1, "NULL caps means no SPI");

    /* An unset clock resolves to the part's maximum. */
    EXPECT(bus_open(&b, &spec, &caps, "test") == 0, "spi opens with caps");
    EXPECT(b.kind == BUS_SPI, "handle records the transport");
    EXPECT(b.spi_hz == 10000000, "spi_hz 0 resolves to the part maximum");
    EXPECT(b.spi_mode == 3, "mode comes from the driver, not the operator");
    bus_close(&b);
    EXPECT(b.fd < 0, "bus_close reopens to the closed state");

    /* A request above the datasheet maximum is clamped, not refused. */
    spec.spi_hz = 50000000;
    EXPECT(bus_open(&b, &spec, &caps, "test") == 0, "over-fast request still opens");
    EXPECT(b.spi_hz == 10000000, "clock clamped to the part maximum");
    bus_close(&b);

    /* A slower request is honoured verbatim. */
    spec.spi_hz = 1000000;
    EXPECT(bus_open(&b, &spec, &caps, "test") == 0, "slower request opens");
    EXPECT(b.spi_hz == 1000000, "a slower clock is left alone");
    bus_close(&b);

    /* Neither a request nor a declared maximum is a config error, not a
     * silent 0 Hz handed to the kernel. */
    spec.spi_hz = 0;
    bus_caps_t nomax = caps;
    nomax.spi_max_hz = 0;
    EXPECT(bus_open(&b, &spec, &nomax, "test") == -1, "no clock at all is refused");

    /* I2C still works and ignores the SPI caps entirely. */
    bus_spec_t i2c = { .kind = BUS_I2C, .node = "/dev/null", .i2c_addr = 0x6B };
    EXPECT(bus_open(&b, &i2c, NULL, "test") == 0, "i2c opens without caps");
    EXPECT(b.kind == BUS_I2C && b.i2c_addr == 0x6B, "i2c handle carries the address");
    bus_close(&b);

    printf("%s\n", g_fail == fb ? "OK" : "FAIL");
}

int main(void)
{
    puts("=== imud driver register tests (mock I2C) ===");

    test_ism_probe();
    test_ism_init_registers();
    test_ism_batched_timestamp();
    test_ism_read_decode();
    test_ism_read_overflow_and_empty();

    test_mmc_probe();
    test_mmc_reset_and_init();
    test_mmc_read_decode();
    test_mock_fail_write_to();
    test_mmc_int_driven_read();
    test_mmc_set_reset();

    test_mpu_probe();
    test_mpu_init_registers();
    test_mpu_read_decode();
    test_mpu_read_overflow_and_errors();
    test_mpu_overflow_discards_and_restarts();

    test_ak_probe();
    test_ak_init_and_fuse_rom();
    test_ak_absent_fuse_rom_applies_no_adjustment();
    test_ak_read_decode();

    test_st_freq_fine_tick();
    test_st_init_flushes_fifo();
    test_st_spi_disables_i2c_block();
    test_lsm_batched_timestamp();
    test_lsm_probe();
    test_lsm_init_registers();
    test_lsm_read_decode();
    test_lsm_read_overflow_and_errors();

    test_icm42_probe_and_init();
    test_icm42_read_decode();
    test_icm42_batched_timestamps();

    test_icm209_probe();
    test_icm209_read_decode();

    test_ak099_probe_and_read();
    test_lis3mdl();
    test_lis2mdl();

    test_rm3100_probe();
    test_rm3100_reset_and_init();
    test_rm3100_read_decode();

    test_chip_ts_monotonic_across_bursts();
    test_chip_ts_forward_garbage_read();
    test_chip_ts_backward_garbage_read();
    test_chip_ts_forward_bound_is_a_sample_period();
    test_chip_ts_reseeds_after_repeated_refusals();
    test_driver_resets();
    test_ak099_init_modes();
    test_spi_same_controller();
    test_odr_agreement();
    test_odr_codes_match_datasheet();
    test_ticks_per_sample_across_rates();

    test_dual_transport_ism330dhcx();
    test_dual_transport_mmc5983ma();
    test_dual_transport_others();

    test_spi_framing();
    test_backend_leg_shapes();
    test_spi_inc_mask();
    test_bus_open_policy();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
