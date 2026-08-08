/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_drivers.c — register-level decode/encode tests over the mock I2C bus
 * (test/bus_mock.c, --wrap=ioctl).
 *
 * Covers ALL TEN hardware drivers: the two hardware-validated ones
 * (ism330dhcx, mmc5983ma), the MPU-925x pair, and the six that until now had
 * no functional coverage at all — lsm6dso, icm42688p, icm20948, ak09916,
 * lis3mdl, lis2mdl.  Those six are also the ones still flagged
 * `experimental`, so before this nothing had ever executed a line of them and
 * a transposed register or a sign error would have waited for silicon that
 * may never arrive.
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
extern const imu_ops_t icm42688p_ops;
extern const imu_ops_t icm20948_ops;
extern const mag_ops_t ak09916_ops;
extern const mag_ops_t lis3mdl_ops;
extern const mag_ops_t lis2mdl_ops;

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

    /* Burst: the mask is set, and the mock strips it back off. */
    const uint8_t seq[6] = { 1, 2, 3, 4, 5, 6 };
    i2cmock_set_regs(SPI_ADDR, 0x28, seq, 6);
    uint8_t got[6] = { 0 };
    EXPECT(bus_burst_read(&b, 0x28, got, 6) == 0, "inc-mask burst succeeds");
    EXPECT(memcmp(got, seq, 6) == 0, "burst with the mask still reads from reg");

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

    test_driver_resets();
    test_ak099_init_modes();
    test_odr_agreement();

    test_spi_framing();
    test_spi_inc_mask();
    test_bus_open_policy();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
