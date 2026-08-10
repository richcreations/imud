/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_drivers.c — register-level decode/encode tests over the mock I2C bus
 * (test/bus_mock.c, --wrap=ioctl).
 *
 * Covers ALL ELEVEN hardware drivers: the two hardware-validated ones
 * (ism330dhcx, mmc5983ma), the MPU-925x pair, and the seven flagged
 * `experimental` — lsm6dso, icm42688p, icm20948, ak09916, lis3mdl, lis2mdl,
 * rm3100.  For those, nothing else has ever executed a line: a transposed
 * register or a sign error would otherwise wait for silicon that may never
 * arrive.
 *
 * A mock cannot replace bench validation (ROADMAP §1) — it cannot tell you
 * the chip→board axis remap matches the physical part.  What it does catch is
 * everything transcribed from a datasheet: register addresses, full-scale
 * encoding, byte order, and the return-code contract.
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
#include <math.h>

#include "drivers.h"
#include "imu_math.h"   /* odr_actual_imu / odr_actual_mag — see test_odr_agreement */
#include "bus_mock.h"
#include "drivers/bus_io.h"   /* the framing under test in test_spi_framing */

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
    i2cmock_fail_next_ioctl();
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

    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
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
    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    (void)ism->init(I2CBUS(ISM_ADDR), &cfg);

    /* Empty FIFO, overflow flag set (FIFO_OVR_IA = bit 6 of STATUS2). */
    i2cmock_set_reg(ISM_ADDR, 0x3A, 0);
    i2cmock_set_reg(ISM_ADDR, 0x3B, 0x40);
    int n = -1;
    EXPECT(ism->read(I2CBUS(ISM_ADDR), NULL, 0, &n) == 1, "empty+overflow returns 1");
    EXPECT(n == 0, "no samples produced");

    /* I2C error on the status read. */
    i2cmock_fail_next_ioctl();
    n = 99;
    EXPECT(ism->read(I2CBUS(ISM_ADDR), NULL, 0, &n) == -1, "I2C error returns -1");

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
    EXPECT(mmc->probe(I2CBUS(MMC_ADDR)) == 0, "probe accepts 0x30");

    i2cmock_set_reg(MMC_ADDR, 0x2F, 0xFF);
    EXPECT(mmc->probe(I2CBUS(MMC_ADDR)) != 0, "probe rejects wrong product ID");

    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x30);
    i2cmock_fail_next_ioctl();
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
    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 5.0f };
    EXPECT(mmc->init(I2CBUS(MMC_ADDR), &cfg) == 0, "init succeeds");
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
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 0, "read returns 0");
    EXPECT(out.valid, "sample marked valid");

    EXPECT_NEAR(out.field[0],  100.0f, 0.1, "field X = +100 µT");
    EXPECT_NEAR(out.field[1],  100.0f, 0.1, "field Y flipped = +100 µT");
    EXPECT_NEAR(out.field[2],   50.0f, 0.1, "field Z = +50 µT");

    /* Not-ready: M_DONE clear → returns 1 (wait for next interrupt). */
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == 1, "M_DONE clear returns 1");

    /* I2C error on the status read. */
    i2cmock_fail_next_ioctl();
    EXPECT(mmc->read(I2CBUS(MMC_ADDR), &out) == -1, "I2C error returns -1");

    end(fb);
}

static void test_mmc_set_reset(void)
{
    begin("test_mmc_set_reset");
    int fb = g_fail;

    i2cmock_reset();
    EXPECT(mmc->set_reset(I2CBUS(MMC_ADDR)) == 0, "set_reset succeeds");
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
    i2cmock_fail_next_ioctl();
    EXPECT(mpu->probe(I2CBUS(MPU_ADDR)) != 0, "probe fails on I2C error");

    end(fb);
}

static void test_mpu_init_registers(void)
{
    begin("test_mpu_init_registers");
    int fb = g_fail;

    mpu_stage_genuine(0x71);

    /* 1000 Hz is on the 1000/(1+SMPLRT_DIV) grid with divider 0. */
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
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
    imu_cfg_t c200 = { .odr_hz = 200, .accel_g = 2, .gyro_dps = 250, .fifo_wm = 32 };
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
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
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
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 32 };
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
    i2cmock_fail_next_ioctl();
    EXPECT(mpu->read(I2CBUS(MPU_ADDR), buf, 4, &n) == -1, "I2C error returns -1");

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
    i2cmock_fail_next_ioctl();
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

    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 0.0f };
    EXPECT(ak->init(I2CBUS(AK_ADDR), &cfg) == 0, "init succeeds");
    EXPECT(i2cmock_get_reg(AK_ADDR, 0x0A) == 0x16,
           "CNTL1 = 16-bit | continuous mode 2");

    mag_cfg_t slow = { .odr_hz = 8, .set_period_s = 0.0f };
    EXPECT(ak->init(I2CBUS(AK_ADDR), &slow) == 0, "init at 8 Hz succeeds");
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
    i2cmock_fail_next_ioctl();
    EXPECT(ak->read(I2CBUS(AK_ADDR), &out) == -1, "I2C error returns -1");

    end(fb);
}

/* ════════════════════════════════════════════════════════════════════════════
 * The six drivers that had NO functional coverage
 *
 * Before this, only ism330dhcx/mmc5983ma/mpu925x/ak8963 were exercised; the
 * CI coverage job put src/drivers/ at 48% with these six at literally 0.0%.
 * They are also the six still flagged `experimental` and awaiting bench
 * validation, so until now nothing had ever executed a line of them — a
 * transposed register or a sign error would have waited for hardware.
 *
 * These do not replace hardware validation (ROADMAP §1): a mock cannot tell
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
    i2cmock_fail_next_ioctl();
    EXPECT(d->probe(I2CBUS(LSM_ADDR)) != 0, "probe fails on I2C error");
    end(fb);
}

static void test_lsm_init_registers(void)
{
    begin("test_lsm_init_registers");
    int fb = g_fail;
    const imu_ops_t *d = &lsm6dso_ops;

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_hz = 833, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 64 };
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
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x0A) == 0x26,
           "FIFO_CTRL4 = continuous + 12.5 Hz temp batching");
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x0D) == 0x08, "INT1_CTRL = FIFO threshold");

    /* A watermark past the 511-word cap must clamp, not wrap. */
    i2cmock_reset();
    imu_cfg_t big = { .odr_hz = 833, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 400 };
    EXPECT(d->init(I2CBUS(LSM_ADDR), &big) == 0, "init with an oversized watermark");
    EXPECT(i2cmock_get_reg(LSM_ADDR, 0x07) == (uint8_t)(511 & 0xFF) &&
           i2cmock_get_reg(LSM_ADDR, 0x08) == 0x01,
           "watermark clamps to 511 words");

    i2cmock_reset();
    i2cmock_fail_next_ioctl();
    EXPECT(d->init(I2CBUS(LSM_ADDR), &cfg) != 0, "init fails on I2C error");
    end(fb);
}

static void test_lsm_read_decode(void)
{
    begin("test_lsm_read_decode");
    int fb = g_fail;
    const imu_ops_t *d = &lsm6dso_ops;

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
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
    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
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
    i2cmock_fail_next_ioctl();
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
    i2cmock_fail_next_ioctl();
    EXPECT(d->probe(I2CBUS(ICM42_ADDR)) != 0, "probe fails on I2C error");

    i2cmock_reset();
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 8, .gyro_dps = 2000, .fifo_wm = 64 };
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
    imu_cfg_t cfg = { .odr_hz = 1000, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
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

    /* Empty FIFO. */
    i2cmock_set_reg(ICM42_ADDR, 0x2E, 0);
    i2cmock_set_reg(ICM42_ADDR, 0x2F, 0);
    n = -1;
    EXPECT(d->read(I2CBUS(ICM42_ADDR), buf, 8, &n) == 0 && n == 0, "empty FIFO → 0");

    i2cmock_fail_next_ioctl();
    EXPECT(d->read(I2CBUS(ICM42_ADDR), buf, 8, &n) == -1, "I2C error returns -1");
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
    i2cmock_fail_next_ioctl();
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
    imu_cfg_t cfg = { .odr_hz = 225, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
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

    i2cmock_fail_next_ioctl();
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
    i2cmock_fail_next_ioctl();
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
    i2cmock_fail_next_ioctl();
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

    mag_cfg_t mcfg = { .odr_hz = 80, .set_period_s = 0.0f };
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

    i2cmock_fail_next_ioctl();
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

    mag_cfg_t mcfg = { .odr_hz = 100, .set_period_s = 0.0f };
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

    i2cmock_fail_next_ioctl();
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
    i2cmock_fail_next_ioctl();
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
    i2cmock_fail_next_ioctl();
    EXPECT(d->reset(I2CBUS(RM_ADDR)) == -1, "reset reports an I2C error");

    /* 100 Hz: below the CC=200 ceiling, so full resolution, TMRC ~150 Hz. */
    i2cmock_reset();
    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 0.0f };
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
    cfg.odr_hz = 600;
    EXPECT(d->init(I2CBUS(RM_ADDR), &cfg) == 0, "init succeeds at 600 Hz");
    EXPECT(rm_get_cc(0x04) == 50 && rm_get_cc(0x06) == 50 &&
           rm_get_cc(0x08) == 50, "600 Hz drops CC to 50 on all axes");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x0B) == 0x92, "600 Hz -> TMRC 0x92");

    /* And the middle rung, where CC halves but not to the floor. */
    i2cmock_reset();
    cfg.odr_hz = 300;
    EXPECT(d->init(I2CBUS(RM_ADDR), &cfg) == 0, "init succeeds at 300 Hz");
    EXPECT(rm_get_cc(0x04) == 100, "300 Hz uses CC 100");
    EXPECT(i2cmock_get_reg(RM_ADDR, 0x0B) == 0x93, "300 Hz -> TMRC 0x93");

    i2cmock_reset();
    i2cmock_fail_next_ioctl();
    EXPECT(d->init(I2CBUS(RM_ADDR), &cfg) == -1, "init reports an I2C error");

    end(fb);
}

static void test_rm3100_read_decode(void)
{
    begin("test_rm3100_read_decode");
    int fb = g_fail;
    const mag_ops_t *d = &rm3100_ops;

    i2cmock_reset();
    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 0.0f };
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
    cfg.odr_hz = 600;
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
    i2cmock_fail_next_ioctl();
    EXPECT(d->read(I2CBUS(RM_ADDR), &out) == -1, "I2C error returns -1");

    end(fb);
}

/* ── reset(): the self-clearing bit polls ────────────────────────────────── */

/*
 * Every driver's reset() writes a soft-reset bit and then polls for the
 * hardware to clear it.  i2cmock_set_selfclear models exactly that, so these
 * are testable off-hardware — the earlier claim in this file's header that
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
        i2cmock_fail_next_ioctl();
        snprintf(msg, sizeof msg, "%s: reset reports an I2C error", c->name);
        EXPECT(run_reset(c) == -1, msg);
    }

    /* These two have no poll at all — a blind write plus a settling delay —
     * so they cannot time out and always report success. */
    i2cmock_reset();
    EXPECT(lis3mdl_ops.reset(I2CBUS(LIS3_ADDR)) == 0, "lis3mdl: reset succeeds");
    EXPECT(i2cmock_get_reg(LIS3_ADDR, 0x21) == 0x04,
           "lis3mdl: SOFT_RST written to CTRL_REG2");
    i2cmock_fail_next_ioctl();
    EXPECT(lis3mdl_ops.reset(I2CBUS(LIS3_ADDR)) == -1, "lis3mdl: I2C error returns -1");

    i2cmock_reset();
    EXPECT(lis2mdl_ops.reset(I2CBUS(LIS2_ADDR)) == 0, "lis2mdl: reset succeeds");
    EXPECT(i2cmock_get_reg(LIS2_ADDR, 0x60) == 0x20,
           "lis2mdl: SOFT_RST written to CFG_REG_A");
    i2cmock_fail_next_ioctl();
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
        mag_cfg_t cfg = { .odr_hz = tbl[i].hz, .set_period_s = 0.0f };
        EXPECT(d->init(I2CBUS(AK099_ADDR), &cfg) == 0, "init succeeds");
        EXPECT(i2cmock_get_reg(AK099_ADDR, 0x31) == tbl[i].mode, tbl[i].msg);
    }

    i2cmock_reset();
    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 0.0f };
    i2cmock_fail_next_ioctl();
    EXPECT(d->init(I2CBUS(AK099_ADDR), &cfg) == -1, "init reports an I2C error");
    end(fb);
}

/* ── ODR resolution agrees with what the driver programs ─────────────────── */

/*
 * imud resolves the configured rate with odr_actual_* and hands the RESULT to
 * the driver, so the driver's own rounding must be a no-op on it. If the two
 * disagree, the filter is tuned for one rate while the chip samples at
 * another — which is exactly what happened before this was pinned:
 * nearest_odr() decided the tuning while every register-table driver rounded
 * UP, so [imu] odr_hz = 900 tuned for 833 Hz and ran the part at 1660.
 *
 * The check: initialise once at an off-grid request and once at the resolved
 * rate, and require identical control registers. That holds only if the
 * driver's encode chain rounds the same way the shared default does.
 */
static uint8_t init_imu_reg(const imu_ops_t *d, uint8_t addr, uint8_t reg,
                            int odr_hz, int accel_g, int gyro_dps)
{
    i2cmock_reset();
    /* ISM/LSM read OUT_TEMP during init; a zeroed mock reads back fine. */
    imu_cfg_t cfg = { .odr_hz = odr_hz, .accel_g = accel_g,
                      .gyro_dps = gyro_dps, .fifo_wm = 64 };
    d->init(I2CBUS(addr), &cfg);
    return i2cmock_get_reg(addr, reg);
}

static uint8_t init_mag_reg(const mag_ops_t *d, uint8_t addr, uint8_t reg,
                            int odr_hz)
{
    i2cmock_reset();
    mag_cfg_t cfg = { .odr_hz = odr_hz, .set_period_s = 0.0f };
    d->init(I2CBUS(addr), &cfg);
    return i2cmock_get_reg(addr, reg);
}

static void test_odr_agreement(void)
{
    begin("test_odr_agreement");
    int fb = g_fail;

    /* ── The register-table drivers: the NULL-hook snap-up default. ──── */

    /* ISM330DHCX CTRL1_XL. 900 is between 833 and 1660: nearest_odr() would
     * have said 833, the driver programs 1660. */
    EXPECT(odr_actual_imu(ism, 900) == 1660, "ism 900 resolves to 1660");
    EXPECT(init_imu_reg(ism, ISM_ADDR, 0x10, 900,  4, 500) ==
           init_imu_reg(ism, ISM_ADDR, 0x10, 1660, 4, 500),
           "ism programs the resolved rate for an off-grid request");

    /* LSM6DSO CTRL1_XL, same ST encoding, off-grid 60 → 104. */
    EXPECT(odr_actual_imu(&lsm6dso_ops, 60) == 104, "lsm 60 resolves to 104");
    EXPECT(init_imu_reg(&lsm6dso_ops, LSM_ADDR, 0x10, 60,  4, 500) ==
           init_imu_reg(&lsm6dso_ops, LSM_ADDR, 0x10, 104, 4, 500),
           "lsm programs the resolved rate for an off-grid request");

    /* MMC5983MA CTRL2 (Cmm_en|CM_Freq). 137 is the case the audit found:
     * nearest_odr() said 100, the driver programs 200. */
    EXPECT(odr_actual_mag(mmc, 137) == 200, "mmc 137 resolves to 200");
    EXPECT(init_mag_reg(mmc, MMC_ADDR, 0x0B, 137) ==
           init_mag_reg(mmc, MMC_ADDR, 0x0B, 200),
           "mmc programs the resolved rate for an off-grid request");

    /*
     * RM3100 TMRC.  200 is between 150 and 300, so it resolves up to 300 —
     * and on this part that also moves the CYCLE COUNT, because 300 Hz is
     * unreachable at the default count.  A two-field odr_encode() is the one
     * shape that can agree with the resolver on the rate and still disagree
     * with itself on the register, so both are asserted.
     */
    EXPECT(odr_actual_mag(&rm3100_ops, 200) == 300, "rm3100 200 resolves to 300");
    EXPECT(init_mag_reg(&rm3100_ops, RM_ADDR, 0x0B, 200) ==
           init_mag_reg(&rm3100_ops, RM_ADDR, 0x0B, 300),
           "rm3100 programs the resolved rate for an off-grid request");
    EXPECT(init_mag_reg(&rm3100_ops, RM_ADDR, 0x05, 200) ==
           init_mag_reg(&rm3100_ops, RM_ADDR, 0x05, 300),
           "rm3100 picks the resolved rate's cycle count too");

    /* ── The divider-based parts: the hook, not the table. ───────────── */

    /* MPU-925x: ODR = 1000/(1+SMPLRT_DIV), and the divider — not the rate —
     * is what gets rounded to nearest, so it reaches rates that are in no
     * table at all. 137 Hz becomes 1000/7 = 142, which is neither the 100 the
     * snap-up default would give nor the 125 the table's own grid holds. */
    EXPECT(mpu->actual_odr_hz != NULL, "mpu implements actual_odr_hz");
    EXPECT(odr_actual_imu(mpu, 137) == 142, "mpu 137 -> 142 (divider 6)");
    EXPECT(odr_actual_imu(mpu, 1000) == 1000, "mpu 1000 -> 1000 (divider 0)");
    EXPECT(odr_actual_imu(mpu, 5000) == 1000, "mpu clamps above the base rate");
    EXPECT(odr_actual_imu(mpu, 1) == 1000 / 256, "mpu clamps at divider 255");
    /* And the resolved rate is a fixed point — the register is the same. */
    EXPECT(init_imu_reg(mpu, MPU_ADDR, 0x19, 137, 8, 2000) ==
           init_imu_reg(mpu, MPU_ADDR, 0x19, 142, 8, 2000),
           "mpu SMPLRT_DIV is the same for 137 and its resolved 142");

    /* ICM-20948: ODR = 1125/(1+divider). */
    EXPECT(icm20948_ops.actual_odr_hz != NULL, "icm20948 implements actual_odr_hz");
    EXPECT(odr_actual_imu(&icm20948_ops, 1125) == 1125, "icm 1125 -> 1125");
    EXPECT(odr_actual_imu(&icm20948_ops, 500) == 562, "icm 500 -> 562 (divider 1)");
    EXPECT(odr_actual_imu(&icm20948_ops, 5000) == 1125, "icm clamps to base rate");

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
 * it is RIGHT.  The distinction matters because a driver's supported_odr_hz
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
        {   12, 0x1 }, {   26, 0x2 }, {   52, 0x3 }, {  104, 0x4 },
        {  208, 0x5 }, {  416, 0x6 }, {  833, 0x7 }, { 1660, 0x8 },
        { 3332, 0x9 }, { 6664, 0xA },
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
        {    12, 0x0B }, {    25, 0x0A }, {    50, 0x09 }, {   100, 0x08 },
        {   200, 0x07 }, {   500, 0x0F }, {  1000, 0x06 }, {  2000, 0x05 },
        {  4000, 0x04 }, {  8000, 0x03 }, { 16000, 0x02 }, { 32000, 0x01 },
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
        EXPECT(ism->supported_odr_hz[i] == st[i].hz,
               "ism advertises exactly the encodable rates");
        EXPECT(lsm6dso_ops.supported_odr_hz[i] == st[i].hz,
               "lsm advertises exactly the encodable rates");
    }
    EXPECT(ism->supported_odr_hz[sizeof st / sizeof st[0]] == 0,
           "ism table ends after 6664");
    for (size_t i = 0; i < sizeof tdk / sizeof tdk[0]; i++)
        EXPECT(icm42688p_ops.supported_odr_hz[i] == tdk[i].hz,
               "icm42688p advertises exactly the encodable rates");
    EXPECT(icm42688p_ops.supported_odr_hz[sizeof tdk / sizeof tdk[0]] == 0,
           "icm42688p table ends after 32000");

    end(fb);
}

/*
 * ticks_per_sample: the chip-timer spacing the FIFO drivers use to date the
 * samples inside a burst.
 *
 * This existed untested.  Each of these drivers keeps a private odr_actual()
 * whose only consumer is this arithmetic — the daemon-facing rounding goes
 * through odr_actual_imu() and supported_odr_hz instead — so a wrong private
 * table was invisible to every other suite.  It was not hypothetical: the
 * hand-written loop bounds ("i < 7" over an 8-entry table) meant that adding
 * 3332 and 6664 Hz to ism330dhcx would have left odr_actual() clamping at
 * 1660, spacing samples 24 ticks apart when the part emits them 6 apart.  The
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
static uint32_t st_burst_ts_delta(const imu_ops_t *d, uint8_t addr, int odr_hz)
{
    i2cmock_reset();
    imu_cfg_t cfg = { .odr_hz = odr_hz, .accel_g = 4, .gyro_dps = 500,
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

static uint32_t icm42_burst_ts_delta(int odr_hz)
{
    i2cmock_reset();
    imu_cfg_t cfg = { .odr_hz = odr_hz, .accel_g = 4, .gyro_dps = 500,
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
     * 1660 Hz truncates 24.096 to 24 (0.4%), 3332 Hz 12.005 to 12 and
     * 6664 Hz 6.002 to 6 (both 0.04%).  The two rates added last are the
     * best-behaved on the ladder, not the worst.
     */
    static const int st_rates[] = { 12, 26, 52, 104, 208, 416, 833, 1660,
                                    3332, 6664 };
    for (size_t i = 0; i < sizeof st_rates / sizeof st_rates[0]; i++) {
        const uint32_t want = 40000u / (uint32_t)st_rates[i];
        snprintf(msg, sizeof msg, "ism %d Hz spaces samples %u ticks",
                 st_rates[i], want);
        EXPECT(st_burst_ts_delta(ism, ISM_ADDR, st_rates[i]) == want, msg);
        snprintf(msg, sizeof msg, "lsm %d Hz spaces samples %u ticks",
                 st_rates[i], want);
        EXPECT(st_burst_ts_delta(&lsm6dso_ops, LSM_ADDR, st_rates[i]) == want,
               msg);
    }

    /*
     * TDK: 1 µs/tick, so 1000000/rate.  Exact through 8 kHz; 16 kHz truncates
     * 62.5 to 62 and 32 kHz 31.25 to 31, both 0.8% short.  Bounded per burst
     * rather than cumulative — the anchor is re-read every drain.
     */
    static const int tdk_rates[] = { 12, 25, 50, 100, 200, 500, 1000, 2000,
                                     4000, 8000, 16000, 32000 };
    for (size_t i = 0; i < sizeof tdk_rates / sizeof tdk_rates[0]; i++) {
        const uint32_t want = 1000000u / (uint32_t)tdk_rates[i];
        snprintf(msg, sizeof msg, "icm42688p %d Hz spaces samples %u ticks",
                 tdk_rates[i], want);
        EXPECT(icm42_burst_ts_delta(tdk_rates[i]) == want, msg);
    }
    EXPECT(icm42_burst_ts_delta(16000) == 62, "16 kHz truncates 62.5 to 62");
    EXPECT(icm42_burst_ts_delta(32000) == 31, "32 kHz truncates 31.25 to 31");

    /*
     * Off-grid requests round UP to the next advertised rate, and the spacing
     * follows the rate the part is actually programmed to — the private table
     * and the encoder have to agree at the top, which is precisely what a
     * stale loop bound breaks.
     */
    EXPECT(st_burst_ts_delta(ism, ISM_ADDR, 5000) == 40000u / 6664u,
           "ism 5000 Hz is spaced for its resolved 6664, not the old 1660 cap");
    EXPECT(st_burst_ts_delta(ism, ISM_ADDR, 99000) == 40000u / 6664u,
           "ism clamps above the top rung and spaces for it");
    EXPECT(icm42_burst_ts_delta(300) == 1000000u / 500u,
           "icm42688p 300 Hz is spaced for 500, the rate 0x0F selects");
    EXPECT(icm42_burst_ts_delta(99000) == 1000000u / 32000u,
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

static int reg_files_differ(uint8_t a, uint8_t b, int *first)
{
    int n = 0;
    for (int r = 0; r < 256; r++) {
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
    imud_bus_t sb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD,
                      .spi_mode = 3, .spi_inc_mask = 0, .spi_hz = 10000000 };

    EXPECT(ism->probe(&ib) == 0, "i2c probe");
    EXPECT(ism->probe(&sb) == 0, "spi probe");
    EXPECT(ism->reset(&ib) == 0, "i2c reset");
    EXPECT(ism->reset(&sb) == 0, "spi reset");

    imu_cfg_t cfg = { .odr_hz = 208, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 64 };
    EXPECT(ism->init(&ib, &cfg) == 0, "i2c init");
    EXPECT(ism->init(&sb, &cfg) == 0, "spi init");

    int first = -1;
    int diffs = reg_files_differ(I2C_AT, SPI_AT, &first);
    if (diffs) {
        printf("\n  first differing register 0x%02X: i2c=0x%02X spi=0x%02X\n",
               first, i2cmock_get_reg(I2C_AT, (uint8_t)first),
               i2cmock_get_reg(SPI_AT, (uint8_t)first));
    }
    EXPECT(diffs == 0, "init leaves identical registers on both transports");

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
     */
    bool same = (in == sn);
    for (int i = 0; same && i < in; i++)
        same = memcmp(ibuf[i].accel, sbuf[i].accel, sizeof ibuf[i].accel) == 0
            && memcmp(ibuf[i].gyro,  sbuf[i].gyro,  sizeof ibuf[i].gyro)  == 0
            && ibuf[i].temp_c  == sbuf[i].temp_c
            && ibuf[i].chip_ts == sbuf[i].chip_ts;
    EXPECT(same, "both transports decoded identical accel/gyro/temp/chip_ts");

    end(fb);
}

static void test_dual_transport_mmc5983ma(void)
{
    begin("test_dual_transport_mmc5983ma");
    int fb = g_fail;

    const uint8_t I2C_AT = 0x30, SPI_AT = 0x31;

    i2cmock_reset();
    spimock_bind(DUAL_SPI_FD, SPI_AT, 0);

    for (int i = 0; i < 2; i++) {
        uint8_t at = i ? SPI_AT : I2C_AT;
        i2cmock_set_reg(at, 0x2F, 0x30);        /* PRODUCT_ID */
        i2cmock_set_reg(at, 0x08, 0x01);        /* STATUS: Meas_M_Done */
        /* 18-bit X/Y/Z output, distinct per axis. */
        const uint8_t raw[7] = { 0x91, 0x23, 0x45, 0x67, 0x89, 0xAB, 0x55 };
        i2cmock_set_regs(at, 0x00, raw, 7);
    }

    imud_bus_t ib = { .kind = BUS_I2C, .fd = FD, .i2c_addr = I2C_AT };
    imud_bus_t sb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD,
                      .spi_mode = 3, .spi_inc_mask = 0, .spi_hz = 10000000 };

    EXPECT(mmc->probe(&ib) == 0, "i2c probe");
    EXPECT(mmc->probe(&sb) == 0, "spi probe");

    mag_cfg_t cfg = { .odr_hz = 100, .set_period_s = 0.0f };
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

    /* set_reset is a write-only path — it must reach the same register. */
    EXPECT(mmc->set_reset(&ib) == 0, "i2c set_reset");
    EXPECT(mmc->set_reset(&sb) == 0, "spi set_reset");
    EXPECT(i2cmock_get_reg(I2C_AT, 0x09) == i2cmock_get_reg(SPI_AT, 0x09),
           "set_reset wrote the same CTRL0 on both transports");

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

    imu_cfg_t icfg = { .odr_hz = 200, .accel_g = 4, .gyro_dps = 500, .fifo_wm = 32 };

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
    spimock_bind(DUAL_SPI_FD, L_SPI, 0x40);

    const uint8_t out[6] = { 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01 };  /* 100,200,300 */
    for (int k = 0; k < 2; k++) {
        uint8_t at = k ? L_SPI : L_I2C;
        i2cmock_set_reg(at, 0x0F, 0x3D);        /* WHO_AM_I */
        i2cmock_set_reg(at, 0x27, 0x08);        /* STATUS: ZYXDA */
        i2cmock_set_regs(at, 0x28, out, 6);     /* where SPI lands */
        i2cmock_set_regs(at, 0xA8, out, 6);     /* where I2C lands */
    }

    imud_bus_t lib = { .kind = BUS_I2C, .fd = FD, .i2c_addr = L_I2C };
    imud_bus_t lsb = { .kind = BUS_SPI, .fd = DUAL_SPI_FD, .spi_mode = 3,
                       .spi_inc_mask = 0x40, .spi_hz = 10000000 };
    const mag_ops_t *l3 = &lis3mdl_ops;
    mag_cfg_t mcfg = { .odr_hz = 80, .set_period_s = 0.0f };

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
                       .spi_inc_mask = 0, .spi_hz = 1000000 };
    const mag_ops_t *rm = &rm3100_ops;
    mag_cfg_t rcfg = { .odr_hz = 100, .set_period_s = 0.0f };

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
    i2cmock_fail_next_ioctl();
    EXPECT(bus_reg_read(&b, 0x0F, &v) == -1, "spi read propagates ioctl failure");
    i2cmock_fail_next_ioctl();
    EXPECT(bus_reg_write(&b, 0x12, 0x00) == -1, "spi write propagates ioctl failure");

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

    test_lsm_probe();
    test_lsm_init_registers();
    test_lsm_read_decode();
    test_lsm_read_overflow_and_errors();

    test_icm42_probe_and_init();
    test_icm42_read_decode();

    test_icm209_probe();
    test_icm209_read_decode();

    test_ak099_probe_and_read();
    test_lis3mdl();
    test_lis2mdl();

    test_rm3100_probe();
    test_rm3100_reset_and_init();
    test_rm3100_read_decode();

    test_driver_resets();
    test_ak099_init_modes();
    test_odr_agreement();
    test_odr_codes_match_datasheet();
    test_ticks_per_sample_across_rates();

    test_dual_transport_ism330dhcx();
    test_dual_transport_mmc5983ma();
    test_dual_transport_others();

    test_spi_framing();
    test_spi_inc_mask();
    test_bus_open_policy();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
