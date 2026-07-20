/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_drivers.c — register-level decode/encode tests for the two hardware-
 * validated drivers, over the mock I2C bus (test/i2c_mock.c, --wrap=ioctl).
 *
 * These paths were previously only ever exercised on a wired-up Raspberry Pi.
 * The tests verify, off-hardware:
 *   - probe()  accepts the correct WHO_AM_I / product ID and rejects others;
 *   - init()   encodes the requested ODR and full-scale into the right control
 *              registers (asserted by reading the mock register file back);
 *   - read()   decodes a staged FIFO / output payload into the correct SI
 *              samples — sensitivity scaling, byte order, FIFO tag routing,
 *              temperature decode, and the chip→body axis remap;
 *   - I2C errors propagate as -1.
 *
 * Linux/GNU-ld only (the --wrap and <linux/i2c.h> dependencies), like the rest
 * of the daemon-linking suite.  Reference coverage for the mock pattern; the
 * remaining drivers extend it by reusing i2c_mock unchanged.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "drivers.h"
#include "i2c_mock.h"

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

static const imu_ops_t *ism = &ism330dhcx_ops;
static const mag_ops_t *mmc = &mmc5983ma_ops;

#define FD 3   /* arbitrary; the mock ignores it */

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
    EXPECT(ism->probe(FD, ISM_ADDR) == 0, "probe accepts 0x6B");

    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x00);          /* wrong id */
    EXPECT(ism->probe(FD, ISM_ADDR) != 0, "probe rejects wrong WHO_AM_I");

    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);
    i2cmock_fail_next_ioctl();
    EXPECT(ism->probe(FD, ISM_ADDR) != 0, "probe fails on I2C error");

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

    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    EXPECT(ism->init(FD, ISM_ADDR, &cfg) == 0, "init succeeds");

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
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x0A) == 0x26, "FIFO_CTRL4 = cont|temp batch");
    EXPECT(i2cmock_get_reg(ISM_ADDR, 0x0D) == 0x08, "INT1_CTRL = FIFO_TH");

    end(fb);
}

static void test_ism_read_decode(void)
{
    begin("test_ism_read_decode");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);

    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)ism->init(FD, ISM_ADDR, &cfg);   /* sets accel/gyro scale */

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
    EXPECT(ism->read(FD, ISM_ADDR, buf, 8, &n) == 0, "read returns 0");
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
    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)ism->init(FD, ISM_ADDR, &cfg);

    /* Empty FIFO, overflow flag set (FIFO_OVR_IA = bit 6 of STATUS2). */
    i2cmock_set_reg(ISM_ADDR, 0x3A, 0);
    i2cmock_set_reg(ISM_ADDR, 0x3B, 0x40);
    int n = -1;
    EXPECT(ism->read(FD, ISM_ADDR, NULL, 0, &n) == 1, "empty+overflow returns 1");
    EXPECT(n == 0, "no samples produced");

    /* I2C error on the status read. */
    i2cmock_fail_next_ioctl();
    n = 99;
    EXPECT(ism->read(FD, ISM_ADDR, NULL, 0, &n) == -1, "I2C error returns -1");

    end(fb);
}

/* ── MMC5983MA ───────────────────────────────────────────────────────────── */

#define MMC_ADDR 0x30

/* Split an 18-bit unsigned reading into the 7 output registers (0x00..0x06). */
static void mmc_set_output(uint32_t rx, uint32_t ry, uint32_t rz)
{
    uint8_t raw[7] = {
        (uint8_t)((rx >> 10) & 0xFF), (uint8_t)((rx >> 2) & 0xFF),
        (uint8_t)((ry >> 10) & 0xFF), (uint8_t)((ry >> 2) & 0xFF),
        (uint8_t)((rz >> 10) & 0xFF), (uint8_t)((rz >> 2) & 0xFF),
        (uint8_t)(((rx & 3) << 6) | ((ry & 3) << 4) | ((rz & 3) << 2)),
    };
    i2cmock_set_regs(MMC_ADDR, 0x00, raw, 7);
}

static void test_mmc_probe(void)
{
    begin("test_mmc_probe");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x30);       /* PRODUCT_ID correct */
    EXPECT(mmc->probe(FD, MMC_ADDR) == 0, "probe accepts 0x30");

    i2cmock_set_reg(MMC_ADDR, 0x2F, 0xFF);
    EXPECT(mmc->probe(FD, MMC_ADDR) != 0, "probe rejects wrong product ID");

    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x30);
    i2cmock_fail_next_ioctl();
    EXPECT(mmc->probe(FD, MMC_ADDR) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_mmc_reset_and_init(void)
{
    begin("test_mmc_reset_and_init");
    int fb = g_fail;

    i2cmock_reset();
    EXPECT(mmc->reset(FD, MMC_ADDR) == 0, "reset succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x80, "reset writes SW_RST to CTRL1");

    /* 100 Hz → BW=01, CM_Freq=101. */
    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 5.0f };
    EXPECT(mmc->init(FD, MMC_ADDR, &cfg) == 0, "init succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0A) == 0x01, "CTRL1 = BW bits");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x0B) == 0x0D, "CTRL2 = Cmm_en|CM_Freq");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x04, "CTRL0 ends at INT_EN");

    end(fb);
}

static void test_mmc_read_decode(void)
{
    begin("test_mmc_read_decode");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x01);          /* STATUS: M_DONE set */

    /* Null field = 131072 counts, 16384 counts/G, 1 G = 100 µT. */
    mmc_set_output(131072 + 16384,   /* +1 G   → +100 µT */
                   131072 - 16384,   /* −1 G, but Y is sign-flipped → +100 µT */
                   131072 + 8192);   /* +0.5 G → +50 µT (Z not flipped) */

    mag_sample_t out;
    memset(&out, 0, sizeof out);
    EXPECT(mmc->read(FD, MMC_ADDR, &out) == 0, "read returns 0");
    EXPECT(out.valid, "sample marked valid");

    EXPECT_NEAR(out.field[0],  100.0f, 0.1, "field X = +100 µT");
    EXPECT_NEAR(out.field[1],  100.0f, 0.1, "field Y flipped = +100 µT");
    EXPECT_NEAR(out.field[2],   50.0f, 0.1, "field Z = +50 µT");

    /* Not-ready: M_DONE clear → returns 1 (wait for next interrupt). */
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);
    EXPECT(mmc->read(FD, MMC_ADDR, &out) == 1, "M_DONE clear returns 1");

    /* I2C error on the status read. */
    i2cmock_fail_next_ioctl();
    EXPECT(mmc->read(FD, MMC_ADDR, &out) == -1, "I2C error returns -1");

    end(fb);
}

static void test_mmc_set_reset(void)
{
    begin("test_mmc_set_reset");
    int fb = g_fail;

    i2cmock_reset();
    EXPECT(mmc->set_reset(FD, MMC_ADDR) == 0, "set_reset succeeds");
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x08, "SET pulse bit written to CTRL0");

    end(fb);
}

int main(void)
{
    puts("=== imud driver register tests (mock I2C) ===");

    test_ism_probe();
    test_ism_init_registers();
    test_ism_read_decode();
    test_ism_read_overflow_and_empty();

    test_mmc_probe();
    test_mmc_reset_and_init();
    test_mmc_read_decode();
    test_mmc_set_reset();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
