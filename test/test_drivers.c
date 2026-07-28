/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_drivers.c — register-level decode/encode tests over the mock I2C bus
 * (test/i2c_mock.c, --wrap=ioctl).
 *
 * Covers the two hardware-validated drivers (ism330dhcx, mmc5983ma) plus the
 * MPU-925x pair, whose magnetometer axis mapping and fuse-ROM sensitivity
 * correction are easy to get wrong and cheap to pin down here.
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
 * Not covered here: the reset() self-clear polls.  The mock is a plain
 * register file, so a bit written as 1 reads back as 1 forever and every
 * polling reset would simply time out.  Reset timing is an `imud-imutest`
 * bench check instead.
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
extern const imu_ops_t mpu9250_ops;
extern const imu_ops_t mpu9255_ops;
extern const mag_ops_t ak8963_ops;

static const imu_ops_t *ism = &ism330dhcx_ops;
static const mag_ops_t *mmc = &mmc5983ma_ops;
static const imu_ops_t *mpu = &mpu9250_ops;
static const mag_ops_t *ak  = &ak8963_ops;

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
    EXPECT(mpu->probe(FD, MPU_ADDR) == 0, "mpu9250 probe accepts 0x71");
    EXPECT(mpu9255_ops.probe(FD, MPU_ADDR) != 0, "mpu9255 probe rejects 0x71");

    mpu_stage_genuine(0x73);
    EXPECT(mpu9255_ops.probe(FD, MPU_ADDR) == 0, "mpu9255 probe accepts 0x73");
    EXPECT(mpu->probe(FD, MPU_ADDR) != 0, "mpu9250 probe rejects 0x73");

    /* The counterfeit that matters: a relabelled MPU-6500 (no magnetometer). */
    mpu_stage_genuine(0x70);
    EXPECT(mpu->probe(FD, MPU_ADDR) != 0, "probe rejects MPU-6500 id 0x70");

    /* Right WHO_AM_I, but no compass answering through the bypass — the other
     * common clone.  Must fail rather than leave it to the mag driver. */
    mpu_stage_genuine(0x71);
    i2cmock_set_reg(AK_ADDR, 0x00, 0x00);
    EXPECT(mpu->probe(FD, MPU_ADDR) != 0, "probe rejects missing AK8963");

    mpu_stage_genuine(0x71);
    i2cmock_fail_next_ioctl();
    EXPECT(mpu->probe(FD, MPU_ADDR) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_mpu_init_registers(void)
{
    begin("test_mpu_init_registers");
    int fb = g_fail;

    mpu_stage_genuine(0x71);

    /* 1000 Hz is on the 1000/(1+SMPLRT_DIV) grid with divider 0. */
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
    EXPECT(mpu->init(FD, MPU_ADDR, &cfg) == 0, "init succeeds");

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
    imu_cfg_t c200 = { .odr_hz = 200, .accel_g = 2, .gyro_dps = 250, .fifo_wm = 32 };
    EXPECT(mpu->init(FD, MPU_ADDR, &c200) == 0, "init at 200 Hz succeeds");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x19) == 4, "SMPLRT_DIV = 4 for 200 Hz");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1B) == 0x00, "GYRO_CONFIG = 250 dps");
    EXPECT(i2cmock_get_reg(MPU_ADDR, 0x1C) == 0x00, "ACCEL_CONFIG = 2 g");

    /* Two consecutive inits must leave an identical register image. */
    uint8_t before[0x80];
    for (int r = 0; r < 0x80; r++) before[r] = i2cmock_get_reg(MPU_ADDR, (uint8_t)r);
    EXPECT(mpu->init(FD, MPU_ADDR, &c200) == 0, "second init succeeds");
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
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
    (void)mpu->init(FD, MPU_ADDR, &cfg);

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
    EXPECT(mpu->read(FD, MPU_ADDR, buf, 8, &n) == 0, "read returns 0");
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
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
    (void)mpu->init(FD, MPU_ADDR, &cfg);
    i2cmock_set_fifo_reg(MPU_ADDR, 0x74);

    /* Empty FIFO and no overflow: 0 samples, rc 0 — never -1. */
    i2cmock_set_reg(MPU_ADDR, 0x72, 0x00);
    i2cmock_set_reg(MPU_ADDR, 0x73, 0x00);
    i2cmock_set_reg(MPU_ADDR, 0x3A, 0x00);
    imu_sample_t buf[4];
    int n = -1;
    EXPECT(mpu->read(FD, MPU_ADDR, buf, 4, &n) == 0, "empty FIFO returns 0");
    EXPECT(n == 0, "no samples produced");

    /* Overflow latched in INT_STATUS bit 4, FIFO still empty. */
    i2cmock_set_reg(MPU_ADDR, 0x3A, 0x10);
    EXPECT(mpu->read(FD, MPU_ADDR, buf, 4, &n) == 1, "overflow returns 1");

    /* I2C error on the count read. */
    i2cmock_fail_next_ioctl();
    EXPECT(mpu->read(FD, MPU_ADDR, buf, 4, &n) == -1, "I2C error returns -1");

    end(fb);
}

/* ── AK8963 ──────────────────────────────────────────────────────────────── */

static void test_ak_probe(void)
{
    begin("test_ak_probe");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_reg(AK_ADDR, 0x00, 0x48);
    EXPECT(ak->probe(FD, AK_ADDR) == 0, "probe accepts WIA 0x48");

    /* An AK09916 would answer 0x09 here — must not be accepted. */
    i2cmock_set_reg(AK_ADDR, 0x00, 0x09);
    EXPECT(ak->probe(FD, AK_ADDR) != 0, "probe rejects AK09916 id 0x09");

    i2cmock_set_reg(AK_ADDR, 0x00, 0x48);
    i2cmock_fail_next_ioctl();
    EXPECT(ak->probe(FD, AK_ADDR) != 0, "probe fails on I2C error");

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

    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 0.0f };
    EXPECT(ak->init(FD, AK_ADDR, &cfg) == 0, "init succeeds");
    EXPECT(i2cmock_get_reg(AK_ADDR, 0x0A) == 0x16,
           "CNTL1 = 16-bit | continuous mode 2");

    mag_cfg_t slow = { .odr_hz = 8, .set_period_s = 0.0f };
    EXPECT(ak->init(FD, AK_ADDR, &slow) == 0, "init at 8 Hz succeeds");
    EXPECT(i2cmock_get_reg(AK_ADDR, 0x0A) == 0x12,
           "CNTL1 = 16-bit | continuous mode 1");

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
    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 0.0f };
    (void)ak->init(FD, AK_ADDR, &cfg);

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
    EXPECT(ak->read(FD, AK_ADDR, &out) == 0, "read returns 0");
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
    EXPECT(ak->read(FD, AK_ADDR, &out) == 1, "HOFL overflow returns 1");

    /* DRDY never asserts: no data yet, still not an error. */
    d[6] = 0x00;
    i2cmock_set_regs(AK_ADDR, 0x03, d, 7);
    i2cmock_set_reg(AK_ADDR, 0x02, 0x00);
    EXPECT(ak->read(FD, AK_ADDR, &out) == 1, "DRDY timeout returns 1, not -1");

    i2cmock_set_reg(AK_ADDR, 0x02, 0x01);
    i2cmock_fail_next_ioctl();
    EXPECT(ak->read(FD, AK_ADDR, &out) == -1, "I2C error returns -1");

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

    test_mpu_probe();
    test_mpu_init_registers();
    test_mpu_read_decode();
    test_mpu_read_overflow_and_errors();

    test_ak_probe();
    test_ak_init_and_fuse_rom();
    test_ak_read_decode();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
