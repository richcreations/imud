/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_imutest.c — the imud-imutest checker logic over the mock I2C bus.
 *
 * The point of this test is the direction the bench cannot cover.  On real
 * hardware a check that says PASS only tells you the silicon behaved; it says
 * nothing about whether the check would have *caught* a broken driver.  Here
 * both directions are exercised: a good chip is staged and the check must
 * pass, then a specific defect is staged and the same check must fail, with
 * the diagnosis naming it.
 *
 * The guided phases are covered too.  imt_ui_t is scripted rather than
 * interactive: prompt() restages the mock for whichever face or turn is about
 * to be measured, and progress() — which the core is contractually required to
 * call once per collection iteration — pushes the next synthetic FIFO word.
 *
 * Linux/GNU-ld only (--wrap=ioctl and <linux/i2c.h>), like test_drivers.
 * Links the driver ops structs directly rather than src/drivers.c, and stubs
 * imt_gpio_count_edges so nothing here needs -lgpiod.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "imutest.h"
#include "i2c_mock.h"

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── GPIO stub ───────────────────────────────────────────────────────────── */

/*
 * Replaces src/imutest_gpio.c so the test links without libgpiod.  Both
 * branches of the caller's handling get exercised by flipping g_gpio_edges.
 */
static int g_gpio_edges = -1;             /* <0 → report "no chip" */
static imt_gpio_why_t g_gpio_why = IMT_GPIO_ENOCHIP;

int imt_gpio_count_edges(const char *chip, int gpio, long window_ms,
                         void (*drain)(void *), void *user,
                         imt_gpio_why_t *why)
{
    (void)chip; (void)gpio; (void)window_ms;
    if (drain) drain(user);
    *why = g_gpio_why;
    return g_gpio_edges;
}

/* ── Drivers under test ──────────────────────────────────────────────────── */

extern const imu_ops_t ism330dhcx_ops;
extern const mag_ops_t mmc5983ma_ops;

#define FD        3          /* the mock ignores it */
#define ISM_ADDR  0x6A
#define MMC_ADDR  0x30

/* ── Staging helpers ─────────────────────────────────────────────────────── */

/* Accel/gyro counts the ISM330 driver will decode at +/-4 g, +/-500 dps. */
#define ISM_ACCEL_LSB (0.122e-3f * 9.80665f)
#define ISM_GYRO_LSB  (17.5f * (float)(M_PI / 180.0 / 1000.0))

static void ism_push(uint8_t tag, int16_t x, int16_t y, int16_t z)
{
    uint8_t w[7] = {
        (uint8_t)(tag << 3),
        (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF),
        (uint8_t)(z & 0xFF), (uint8_t)((z >> 8) & 0xFF),
    };
    i2cmock_fifo_push(ISM_ADDR, w, 7);
}

/*
 * The ISM330 reads its 32-bit timestamp counter once per burst and
 * back-calculates each sample from it.  Advance it by one sample period
 * (40000 ticks/s ÷ 208 Hz ≈ 192) per staged pair so chip_ts looks like real
 * silicon — otherwise it stays 0 and the has_hw_timestamp check fails for a
 * reason that has nothing to do with the driver.
 */
#define ISM_TICKS_PER_SAMPLE 192
static uint32_t g_chip_ts;

static void ism_advance_ts(void)
{
    g_chip_ts += ISM_TICKS_PER_SAMPLE;
    i2cmock_set_reg(ISM_ADDR, 0x40, (uint8_t)(g_chip_ts));
    i2cmock_set_reg(ISM_ADDR, 0x41, (uint8_t)(g_chip_ts >> 8));
    i2cmock_set_reg(ISM_ADDR, 0x42, (uint8_t)(g_chip_ts >> 16));
    i2cmock_set_reg(ISM_ADDR, 0x43, (uint8_t)(g_chip_ts >> 24));
}

/*
 * Stage one accel+gyro sample-pair whose decoded value is the requested SI
 * vector.  The core sees a real driver decoding real register bytes.
 */
static int g_stage_saturated;

static void stage_sample(const double accel_ms2[3], const double gyro_rads[3])
{
    /*
     * Guard: the staged value has to fit the int16 the driver decodes, or the
     * counts wrap and the test silently measures nonsense.  At +/-500 dps the
     * ceiling is about 573 deg/s, and at +/-4 g about 4 g.
     */
    for (int k = 0; k < 3; k++) {
        if (fabs(accel_ms2[k]) / ISM_ACCEL_LSB > 32767.0 ||
            fabs(gyro_rads[k]) / ISM_GYRO_LSB  > 32767.0)
            g_stage_saturated++;
    }

    /* The driver flips Y and Z, so pre-invert to get the SI value we want. */
    int16_t ax = (int16_t)lrint( accel_ms2[0] / ISM_ACCEL_LSB);
    int16_t ay = (int16_t)lrint(-accel_ms2[1] / ISM_ACCEL_LSB);
    int16_t az = (int16_t)lrint(-accel_ms2[2] / ISM_ACCEL_LSB);
    int16_t gx = (int16_t)lrint( gyro_rads[0] / ISM_GYRO_LSB);
    int16_t gy = (int16_t)lrint(-gyro_rads[1] / ISM_GYRO_LSB);
    int16_t gz = (int16_t)lrint(-gyro_rads[2] / ISM_GYRO_LSB);

    ism_push(0x02, ax, ay, az);
    ism_push(0x01, gx, gy, gz);
    /* Two more FIFO words are now pending. */
    uint8_t cur = i2cmock_get_reg(ISM_ADDR, 0x3A);
    i2cmock_set_reg(ISM_ADDR, 0x3A, (uint8_t)(cur + 2));
    ism_advance_ts();
}

static void mmc_set_field(double ut_x, double ut_y, double ut_z);

static void mock_base(void)
{
    i2cmock_reset();
    g_chip_ts = 1000;
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x6B);       /* WHO_AM_I */
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);       /* OUT_TEMP */
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x3A, 0);          /* FIFO depth */
    i2cmock_set_reg(ISM_ADDR, 0x3B, 0);          /* no overflow */
    i2cmock_set_fifo_reg(ISM_ADDR, 0x78);
    /* CTRL3_C bit 0 is SW_RESET, which the chip clears when the reset lands;
     * imud-imutest always calls reset(), so the mock has to model that. */
    i2cmock_set_selfclear(ISM_ADDR, 0x12, 0x01);

    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x30);       /* PRODUCT_ID */
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x01);       /* STATUS: M_DONE */
    /* Zeroed output registers decode to about -800 uT on every axis, which
     * would fail the field-magnitude check for a reason unrelated to the
     * driver.  Start from a plausible earth field. */
    mmc_set_field(22.0, 4.0, 42.0);
    ism_advance_ts();
}

/* Split an 18-bit unsigned reading into the MMC's 7 output registers. */
static void mmc_set_field(double ut_x, double ut_y, double ut_z)
{
    /* Driver: value = (raw - 131072) * 100/16384 µT, with Y flipped. */
    const double lsb = 100.0 / 16384.0;
    uint32_t rx = (uint32_t)lrint(131072.0 +  ut_x / lsb);
    uint32_t ry = (uint32_t)lrint(131072.0 + -ut_y / lsb);
    uint32_t rz = (uint32_t)lrint(131072.0 +  ut_z / lsb);
    uint8_t raw[7] = {
        (uint8_t)((rx >> 10) & 0xFF), (uint8_t)((rx >> 2) & 0xFF),
        (uint8_t)((ry >> 10) & 0xFF), (uint8_t)((ry >> 2) & 0xFF),
        (uint8_t)((rz >> 10) & 0xFF), (uint8_t)((rz >> 2) & 0xFF),
        (uint8_t)(((rx & 3) << 6) | ((ry & 3) << 4) | ((rz & 3) << 2)),
    };
    i2cmock_set_regs(MMC_ADDR, 0x00, raw, 7);
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x01);
}

static void base_config(imud_config_t *cfg)
{
    config_defaults(cfg);
    snprintf(cfg->imu_driver, sizeof cfg->imu_driver, "ism330dhcx");
    snprintf(cfg->mag_driver, sizeof cfg->mag_driver, "mmc5983ma");
    cfg->imu_addr     = ISM_ADDR;
    cfg->mag_addr     = MMC_ADDR;
    cfg->imu_int_gpio = 0;       /* the GPIO stub covers that path separately */
    cfg->mag_int_gpio = 0;
    cfg->imu_odr_hz   = 208;
    cfg->imu_accel_g  = 4;
    cfg->imu_gyro_dps = 500;
    cfg->imu_fifo_wm  = 8;
    cfg->mag_odr_hz   = 100;
}

/* Short windows: the core measures real elapsed time, so the whole suite has
 * to stay quick.  These are the smallest values the checks still work at. */
static void fast_opts(imt_opts_t *o)
{
    imt_opts_defaults(o);
    o->odr_window_s    = 0.30;
    o->noise_window_s  = 0.30;
    o->drdy_window_s   = 0.05;
    o->mag_window_s    = 0.20;
    o->face_settle_s   = 0.02;
    o->face_collect_s  = 0.10;
    o->turn_timeout_s  = 1.0;
    o->spin_timeout_s  = 2.0;
    o->fs_sweep        = false;   /* enabled explicitly in its own test */
    o->induce_overflow = false;
    o->regdiff         = true;
}

/* ── Scripted UI ─────────────────────────────────────────────────────────── */

typedef struct {
    /* What to feed while collecting. */
    double accel[3], gyro[3];
    double mag[3];
    bool   feed_imu, feed_mag;
    /* Guided-phase scripting. */
    int    prompt_calls;
    int    ticks;
    int    done_after;       /* poll_done returns 1 after this many ticks */
    bool   skip_all;
    bool   abort_now;
    /* Phase B: per-face vectors, indexed by the "face.N" id. */
    double face_vec[6][3];
    bool   use_face_vecs;
    int    cur_face;
    /* Phase C: per-axis rate, deg/s, for the gyro turns. */
    double turn_rate_dps[3];
    int    cur_turn;
    /* Phase D: spin. */
    bool   spinning;
    double spin_deg;
    double spin_dir;         /* +1 normal, -1 to invert the mag sweep */
    double spin_step_deg;    /* heading advance per tick */
    /* Re-arm the mock's one-shot I2C failure on every iteration, so a whole
     * check sees errors rather than just the first read. */
    bool   inject_errors;
} script_t;

static script_t g_s;

static int s_prompt(void *user, const char *id, const char *title,
                    const char *body)
{
    script_t *s = user;
    (void)title; (void)body;
    s->prompt_calls++;
    s->ticks = 0;

    if (s->abort_now) return -1;
    if (s->skip_all)  return 1;

    if (strncmp(id, "face.", 5) == 0) {
        int n = atoi(id + 5) - 1;
        if (n >= 0 && n < 6) {
            s->cur_face = n;
            if (s->use_face_vecs) memcpy(s->accel, s->face_vec[n], sizeof s->accel);
        }
    } else if (strcmp(id, "gyro.x") == 0) s->cur_turn = 0;
    else if   (strcmp(id, "gyro.y") == 0) s->cur_turn = 1;
    else if   (strcmp(id, "gyro.z") == 0) s->cur_turn = 2;
    else if   (strcmp(id, "spin")   == 0) { s->spinning = true; s->spin_deg = 0; }

    return 0;
}

static int s_poll_done(void *user)
{
    script_t *s = user;
    return (s->done_after > 0 && s->ticks >= s->done_after) ? 1 : 0;
}

/*
 * The core's contract: progress() is called at least once per collection
 * iteration of every timed phase.  That makes it the data-injection hook —
 * each call stages the next sample the driver will decode.
 */
static void s_progress(void *user, const char *id, double frac,
                       const char *detail)
{
    script_t *s = user;
    (void)frac; (void)detail;
    s->ticks++;

    if (s->inject_errors) i2cmock_fail_all(1);

    if (s->feed_imu) {
        double g[3] = { s->gyro[0], s->gyro[1], s->gyro[2] };
        /* During a gyro turn, spin the commanded axis. */
        if (strncmp(id, "gyro.", 5) == 0 && s->cur_turn >= 0) {
            g[0] = g[1] = g[2] = 0.0;
            g[s->cur_turn] = s->turn_rate_dps[s->cur_turn] * M_PI / 180.0;
        }
        stage_sample(s->accel, g);
    }
    if (s->feed_mag && s->spinning) {
        /* Sweep the horizontal field so the heading unwraps, at the same rate
         * the gyro is being fed — a correct driver must show them agreeing. */
        s->spin_deg += s->spin_step_deg;
        double th = s->spin_deg * M_PI / 180.0 * s->spin_dir;
        mmc_set_field(45.0 * cos(th), -45.0 * sin(th), 40.0);
    }
}

static void s_coverage(void *user, const int *sectors, int nsec, int cur,
                       int n, double radius)
{
    (void)sectors; (void)nsec; (void)cur; (void)n; (void)radius;
    s_progress(user, "spin", -1.0, NULL);
}

static void script_reset(imt_opts_t *o)
{
    memset(&g_s, 0, sizeof g_s);
    g_s.cur_turn = -1;
    g_s.accel[2] = -9.80665;      /* flat, component-side up */
    g_s.feed_imu = true;
    o->ui.prompt    = s_prompt;
    o->ui.poll_done = s_poll_done;
    o->ui.progress  = s_progress;
    o->ui.coverage  = s_coverage;
    o->ui.user      = &g_s;
}

/* ── Assertion helpers ───────────────────────────────────────────────────── */

static imt_status_t status_of(const imt_report_t *r, const char *id)
{
    const imt_check_t *c = imt_find(r, id);
    return c ? c->status : (imt_status_t)-1;
}

static bool note_contains(const imt_report_t *r, const char *id, const char *needle)
{
    const imt_check_t *c = imt_find(r, id);
    return c && strstr(c->note, needle) != NULL;
}

static imt_report_t *run(imud_config_t *cfg, imt_opts_t *o)
{
    imt_report_t *r = calloc(1, sizeof *r);
    char err[256] = "";
    int rc = imt_run_ops(FD, &ism330dhcx_ops, &mmc5983ma_ops, cfg, o, r,
                         err, sizeof err);
    if (rc < 0) fprintf(stderr, "  imt_run_ops: %s\n", err);
    return r;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_bringup_good(void)
{
    begin("test_bringup_good");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    EXPECT(status_of(r, "imu.probe")        == IMT_PASS, "probe passes");
    EXPECT(status_of(r, "imu.probe.reject") == IMT_PASS, "bogus address rejected");
    EXPECT(status_of(r, "imu.reset.rc")     == IMT_PASS, "reset passes");
    EXPECT(status_of(r, "imu.init.rc")      == IMT_PASS, "init passes");
    EXPECT(status_of(r, "mag.probe")        == IMT_PASS, "mag probe passes");
    EXPECT(status_of(r, "mag.init.rc")      == IMT_PASS, "mag init passes");
    /* init() writes CTRL registers, so the diff must be non-empty. */
    EXPECT(status_of(r, "imu.init.regdiff") == IMT_PASS, "regdiff sees writes");
    EXPECT(r->raw.n_regdiff_imu > 0, "regdiff recorded rows");
    EXPECT(status_of(r, "imu.init.idempotent") == IMT_PASS, "init is idempotent");
    EXPECT(r->imu_experimental == false, "ism330dhcx not flagged experimental");
    EXPECT(r->have_mag, "mag present");

    free(r);
    end(fb);
}

static void test_bringup_bad_whoami(void)
{
    begin("test_bringup_bad_whoami");
    int fb = g_fail;

    mock_base();
    i2cmock_set_reg(ISM_ADDR, 0x0F, 0x00);       /* wrong WHO_AM_I */

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    EXPECT(status_of(r, "imu.probe") == IMT_FAIL, "probe fails on wrong id");
    EXPECT(r->n_fail > 0, "run records a failure");
    EXPECT(!r->recommend_clear_experimental, "no recommendation after a FAIL");
    /* Everything downstream must be absent rather than falsely passing. */
    EXPECT(imt_find(r, "imu.odr") == NULL, "downstream checks not run");

    free(r);
    end(fb);
}

static void test_odr_and_seq(void)
{
    begin("test_odr_and_seq");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    /* The mock feeds one sample per progress() tick, so the measured rate is
     * whatever the loop achieved — the point here is the contract checks. */
    EXPECT(r->raw.odr_n > 0, "samples were collected");
    EXPECT(status_of(r, "imu.seq.monotonic") == IMT_PASS, "seq monotonic");
    EXPECT(status_of(r, "imu.seq.gapless")   == IMT_PASS, "seq gapless");
    EXPECT(r->raw.seq_backwards == 0, "no seq reversals recorded");
    /* ISM330 has a hardware timestamp, so chip_ts must be present. */
    EXPECT(status_of(r, "imu.chipts.presence") == IMT_PASS, "chip_ts present");
    EXPECT(status_of(r, "imu.odr.rounding") == IMT_INFO, "ODR rounding is INFO");
    EXPECT(r->eff_odr_hz == 208, "208 Hz is on the ISM330 grid");

    free(r);
    end(fb);
}

static void test_error_contract_both_ways(void)
{
    begin("test_error_contract_both_ways");
    int fb = g_fail;

    /* Healthy bus: no spurious -1, and an empty FIFO reports 0. */
    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);
    g_s.feed_imu = false;                     /* leave the FIFO empty */

    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.err.nodata_not_error") == IMT_PASS,
           "empty FIFO reports 0, not -1");
    EXPECT(status_of(r, "imu.err.no_spurious") == IMT_PASS,
           "no spurious -1 on a healthy bus");
    free(r);

    /*
     * The half the bench genuinely cannot prove: that these checks would
     * actually catch a driver returning -1 when it should not.  Wedge the bus
     * from the first progress() tick — the error-contract loop has no UI
     * callback of its own, so a one-shot injection would be consumed long
     * before it runs.
     */
    mock_base();
    script_reset(&o);
    g_s.feed_imu      = false;
    g_s.inject_errors = true;

    r = run(&cfg, &o);
    i2cmock_fail_all(0);
    EXPECT(r->raw.rcneg_count > 0, "injected I2C errors reached the driver");
    EXPECT(status_of(r, "imu.err.no_spurious") == IMT_FAIL,
           "sustained -1 returns are caught as a failure");
    EXPECT(note_contains(r, "imu.err.no_spurious", "errno"),
           "diagnosis reports the errno");
    EXPECT(status_of(r, "imu.err.nodata_not_error") == IMT_FAIL,
           "a wedged bus is not mistaken for an empty FIFO");
    free(r);

    end(fb);
}

static void test_gravity_and_stuck_axis(void)
{
    begin("test_gravity_and_stuck_axis");
    int fb = g_fail;

    /* Correct gravity, with a little noise so no axis looks stuck. */
    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);
    g_s.accel[0] = 0.01; g_s.accel[1] = -0.01; g_s.accel[2] = -9.80665;
    g_s.gyro[0]  = 0.001;

    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.rest.gravity") == IMT_PASS, "gravity reads 9.807");
    EXPECT(fabs(r->raw.grav_mean - 9.80665) < 0.05, "grav_mean recorded");
    free(r);

    /* Half-scale gravity: the classic wrong-sensitivity bug.  The check must
     * fail AND the note must name the 2x ratio. */
    mock_base();
    script_reset(&o);
    g_s.accel[0] = 0.01; g_s.accel[1] = -0.01; g_s.accel[2] = -9.80665 / 2.0;
    g_s.gyro[0]  = 0.001;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.rest.gravity") == IMT_FAIL, "half gravity fails");
    EXPECT(note_contains(r, "imu.rest.gravity", "2.00") ||
           note_contains(r, "imu.rest.gravity", "power of two"),
           "diagnosis names the 2x sensitivity error");
    free(r);

    end(fb);
}

static void test_temperature_placeholder(void)
{
    begin("test_temperature_placeholder");
    int fb = g_fail;

    /* OUT_TEMP raw 0 decodes to exactly 25.000 °C on the ISM330 and never
     * changes — the signature of a temperature word that is never decoded. */
    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.temp.plausible") == IMT_FAIL,
           "pinned 25.000 C is caught");
    EXPECT(note_contains(r, "imu.temp.plausible", "placeholder"),
           "diagnosis names the placeholder");
    free(r);

    /* A varying, plausible temperature must pass. */
    mock_base();
    i2cmock_set_reg(ISM_ADDR, 0x20, 0x00);
    i2cmock_set_reg(ISM_ADDR, 0x21, 0x05);      /* raw 1280 -> 30 C */
    script_reset(&o);

    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.temp.plausible") == IMT_PASS,
           "a real temperature passes");
    free(r);

    end(fb);
}

static void test_gpio_both_branches(void)
{
    begin("test_gpio_both_branches");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;

    /* int_gpio 0: the check is skipped for the right reason. */
    mock_base(); script_reset(&o);
    cfg.imu_int_gpio = 0;
    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_SKIP, "no GPIO -> SKIP");
    EXPECT(r->raw.gpio_why == IMT_GPIO_DISABLED, "reason recorded as disabled");
    free(r);

    /* Line held by another process: SKIP, never FAIL — that would blame the
     * driver for the daemon holding the line. */
    mock_base(); script_reset(&o);
    cfg.imu_int_gpio = 17;
    g_gpio_edges = -1;
    g_gpio_why   = IMT_GPIO_EBUSY;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_SKIP, "EBUSY -> SKIP not FAIL");
    free(r);

    /* Zero edges on a requested line is a real defect. */
    mock_base(); script_reset(&o);
    g_gpio_edges = 0;
    g_gpio_why   = IMT_GPIO_OK;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_FAIL, "0 edges -> FAIL");
    free(r);

    g_gpio_edges = -1;
    g_gpio_why   = IMT_GPIO_ENOCHIP;
    end(fb);
}

static void test_faces_good_and_swapped(void)
{
    begin("test_faces_good_and_swapped");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_FACES;

    /* Correct NED frame: the axis pointing down reads -g. */
    const double good[6][3] = {
        {  0,  0, -9.80665 },   /* flat, component up   */
        {  0,  0,  9.80665 },   /* flat, upside down    */
        { -9.80665, 0, 0 },     /* nose down            */
        {  9.80665, 0, 0 },     /* nose up              */
        {  0, -9.80665, 0 },    /* starboard down       */
        {  0,  9.80665, 0 },    /* port down            */
    };

    mock_base(); script_reset(&o);
    memcpy(g_s.face_vec, good, sizeof good);
    g_s.use_face_vecs = true;

    imt_report_t *r = run(&cfg, &o);
    EXPECT(g_s.prompt_calls == 6, "all six faces prompted");
    EXPECT(status_of(r, "faces.frame") == IMT_PASS, "correct frame passes");
    EXPECT(status_of(r, "face.1.sign") == IMT_PASS, "face 1 sign correct");
    EXPECT(status_of(r, "face.5.sign") == IMT_PASS, "face 5 sign correct");
    EXPECT(r->raw.n_faces == 6, "six face rows recorded");
    free(r);

    /* Y and Z swapped — the defect a wrong chip-to-board remap produces. */
    double swapped[6][3];
    for (int i = 0; i < 6; i++) {
        swapped[i][0] = good[i][0];
        swapped[i][1] = good[i][2];
        swapped[i][2] = good[i][1];
    }
    mock_base(); script_reset(&o);
    memcpy(g_s.face_vec, swapped, sizeof swapped);
    g_s.use_face_vecs = true;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "faces.frame") == IMT_FAIL, "swapped axes fail");
    EXPECT(note_contains(r, "face.1.sign", "swapped") ||
           note_contains(r, "face.1.sign", "Y"),
           "diagnosis names the swapped axis");
    free(r);

    /* Z sign inverted only — the other classic remap bug. */
    double flipped[6][3];
    memcpy(flipped, good, sizeof good);
    for (int i = 0; i < 6; i++) flipped[i][2] = -flipped[i][2];
    mock_base(); script_reset(&o);
    memcpy(g_s.face_vec, flipped, sizeof flipped);
    g_s.use_face_vecs = true;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "face.1.sign") == IMT_FAIL, "inverted Z fails");
    EXPECT(note_contains(r, "face.1.sign", "sign"), "diagnosis names the sign");
    free(r);

    end(fb);
}

static void test_faces_skipped_not_absent(void)
{
    begin("test_faces_skipped_not_absent");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_FACES;

    mock_base(); script_reset(&o);
    g_s.skip_all = true;

    imt_report_t *r = run(&cfg, &o);
    /* A skipped guided check must be present as SKIP, never silently missing:
     * an absent check would let the recommendation logic pass by omission. */
    EXPECT(status_of(r, "face.1.sign") == IMT_SKIP, "skipped face is SKIP");
    EXPECT(status_of(r, "faces.frame") == IMT_SKIP, "rollup is SKIP");
    EXPECT(!r->recommend_clear_experimental, "no recommendation when skipped");
    free(r);

    end(fb);
}

static void test_gyro_sign(void)
{
    begin("test_gyro_sign");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases   = IMT_PHASE_GYRO;
    o.turn_deg = 90.0;

    /* Positive rotation on each axis. */
    mock_base(); script_reset(&o);
    g_s.done_after = 40;
    for (int k = 0; k < 3; k++) g_s.turn_rate_dps[k] = 400.0;

    imt_report_t *r = run(&cfg, &o);
    EXPECT(g_s.prompt_calls == 3, "three turns prompted");
    EXPECT(status_of(r, "gyro.x.sign") == IMT_PASS, "X sign correct");
    EXPECT(status_of(r, "gyro.y.sign") == IMT_PASS, "Y sign correct");
    EXPECT(status_of(r, "gyro.z.sign") == IMT_PASS, "Z sign correct");
    free(r);

    /* Inverted X gyro. */
    mock_base(); script_reset(&o);
    g_s.done_after = 40;
    g_s.turn_rate_dps[0] = -400.0;
    g_s.turn_rate_dps[1] =  400.0;
    g_s.turn_rate_dps[2] =  400.0;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "gyro.x.sign") == IMT_FAIL, "inverted X gyro fails");
    EXPECT(note_contains(r, "gyro.x.sign", "inverted"),
           "diagnosis says inverted");
    EXPECT(status_of(r, "gyro.y.sign") == IMT_PASS, "Y still passes");
    free(r);

    end(fb);
}

static void test_spin_frame_agreement(void)
{
    begin("test_spin_frame_agreement");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_SPIN;

    /*
     * The script sweeps the horizontal field by +6 deg per tick and feeds a
     * gyro Z rate that integrates to the same 6 deg over the one sample staged
     * per tick, so a correct driver shows both sensors agreeing.
     */
    const double rate_dps     = 400.0;                   /* inside +/-500 dps */
    const double deg_per_tick = rate_dps / 208.0;        /* one sample per tick */
    const double gyro_z       = rate_dps * M_PI / 180.0;

    mock_base();
    mmc_set_field(45.0, 0.0, 40.0);      /* a valid field before the first read */
    script_reset(&o);
    g_s.feed_mag   = true;
    g_s.spin_dir   = +1.0;
    g_s.done_after    = 220;
    g_s.gyro[2]       = gyro_z;
    g_s.spin_step_deg = deg_per_tick;

    imt_report_t *r = run(&cfg, &o);
    EXPECT(r->raw.spin_n > 0, "spin collected mag samples");
    EXPECT(r->raw.spin_covered > 0, "sector coverage advanced");
    EXPECT(status_of(r, "spin.magnitude") == IMT_PASS, "|B| in the earth band");
    EXPECT(status_of(r, "spin.frame_agreement") == IMT_PASS,
           "mag heading agrees with gyro Z");
    free(r);

    /*
     * Invert the mag sweep only.  The heading now runs opposite to the gyro —
     * exactly the signature of an inverted X or Y in the magnetometer's remap,
     * and the defect this whole phase exists to catch.
     */
    mock_base();
    mmc_set_field(45.0, 0.0, 40.0);
    script_reset(&o);
    g_s.feed_mag   = true;
    g_s.spin_dir   = -1.0;
    g_s.done_after    = 220;
    g_s.gyro[2]       = gyro_z;
    g_s.spin_step_deg = deg_per_tick;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "spin.frame_agreement") == IMT_FAIL,
           "inverted mag sweep fails frame agreement");
    EXPECT(note_contains(r, "spin.frame_agreement", "OPPOSITE"),
           "diagnosis names the opposite-direction signature");
    free(r);

    end(fb);
}

static void test_report_and_exit_codes(void)
{
    begin("test_report_and_exit_codes");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);
    g_s.accel[2] = -9.80665;

    imt_report_t *r = run(&cfg, &o);

    /* Exit-code mapping, all four combinations. */
    imt_report_t t;
    memset(&t, 0, sizeof t);
    EXPECT(imt_exit_code(&t) == 0, "clean run exits 0");
    t.n_warn = 1;  EXPECT(imt_exit_code(&t) == 3, "warnings exit 3");
    t.n_fail = 1;  EXPECT(imt_exit_code(&t) == 2, "failures exit 2");
    t.aborted = true; EXPECT(imt_exit_code(&t) == 130, "abort exits 130");

    /* The report file must round-trip and stay well-formed Markdown. */
    const char *path = "test_imutest_report.md";
    char err[256] = "";
    EXPECT(imt_write_md(r, path, err, sizeof err) == 0, "report writes");

    FILE *f = fopen(path, "r");
    EXPECT(f != NULL, "report file exists");
    if (f) {
        static char buf[262144];
        size_t n = fread(buf, 1, sizeof buf - 1, f);
        buf[n] = '\0';
        fclose(f);

        EXPECT(strstr(buf, "# imud driver validation") != NULL, "has a title");
        EXPECT(strstr(buf, "ism330dhcx") != NULL, "names the driver");
        EXPECT(strstr(buf, "## 7. How to read this") != NULL, "has the legend");
        EXPECT(strstr(buf, "PASS") != NULL, "reports statuses");

        /* Every check must appear by id, or the report is lying by omission. */
        bool all_present = true;
        for (int i = 0; i < r->n_checks; i++)
            if (!strstr(buf, r->check[i].id)) all_present = false;
        EXPECT(all_present, "every check id appears in the report");

        /*
         * The results rows carry driver-authored note text, so they are where
         * an unescaped '|' would silently break the table.  Those rows all
         * start "| `<id>` |" and have exactly six columns.
         */
        int rows_checked = 0;
        bool rows_ok = true;
        for (char *line = strtok(buf, "\n"); line; line = strtok(NULL, "\n")) {
            if (strncmp(line, "| `", 3) != 0) continue;
            int bars = 0;
            for (char *p = line; *p; p++)
                if (*p == '|' && (p == line || p[-1] != '\\')) bars++;
            rows_checked++;
            if (bars != 7) rows_ok = false;
        }
        EXPECT(rows_checked > 0, "results rows were emitted");
        EXPECT(rows_ok, "no unescaped pipes broke a results row");
        unlink(path);
    }

    free(r);
    end(fb);
}

static void test_sim_like_no_recommendation(void)
{
    begin("test_sim_like_no_recommendation");
    int fb = g_fail;

    /* A passive-only run must never recommend clearing the flag: the guided
     * phases are where the axis remap is actually proved. */
    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);
    EXPECT(!r->recommend_clear_experimental,
           "passive-only run does not recommend clearing experimental");
    EXPECT(r->verdict[0] != '\0', "a verdict sentence is always written");
    free(r);

    end(fb);
}

int main(void)
{
    puts("=== imud-imutest checker tests (mock I2C) ===");

    test_bringup_good();
    test_bringup_bad_whoami();
    test_odr_and_seq();
    test_error_contract_both_ways();
    test_gravity_and_stuck_axis();
    test_temperature_placeholder();
    test_gpio_both_branches();
    test_faces_good_and_swapped();
    test_faces_skipped_not_absent();
    test_gyro_sign();
    test_spin_frame_agreement();
    test_report_and_exit_codes();
    test_sim_like_no_recommendation();

    /* If any test staged a value the int16 registers cannot hold, the counts
     * wrapped and whatever it "measured" was noise.  Fail loudly rather than
     * let a green run hide it. */
    begin("test_no_staging_saturation");
    int fb = g_fail;
    EXPECT(g_stage_saturated == 0, "no staged sample exceeded the full scale");
    end(fb);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
