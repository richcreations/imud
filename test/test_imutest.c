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
#include "bus_mock.h"
/* For the mock self-check below: the same single-byte read path the register
 * sweep uses, so the harness is exercised exactly the way the tool exercises
 * it. */
#include "drivers/bus_io.h"

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
 *
 * check_drdy counts twice over the same window — once draining on every edge,
 * once not — so the stub answers the two passes separately.  That split IS
 * the thing under test: a level-triggered watermark interrupt yields a normal
 * count while something empties the FIFO and about one edge when nothing
 * does, and only the pair can tell that from an edge-per-sample line.
 */
static int g_gpio_edges = -1;             /* <0 → report "no chip" */
static int g_gpio_edges_idle = 1;         /* the undrained pass */
static imt_gpio_why_t g_gpio_why = IMT_GPIO_ENOCHIP;

int imt_gpio_count_edges(const char *chip, int gpio, long window_ms,
                         void (*drain)(void *), void *user,
                         imt_gpio_why_t *why)
{
    (void)chip; (void)gpio; (void)window_ms;
    *why = g_gpio_why;
    if (!drain) return g_gpio_edges < 0 ? g_gpio_edges : g_gpio_edges_idle;
    drain(user);
    return g_gpio_edges;
}

/* ── Drivers under test ──────────────────────────────────────────────────── */

extern const imu_ops_t ism330dhcx_ops;
extern const mag_ops_t mmc5983ma_ops;

#define FD        3          /* the mock ignores it */
#define ISM_ADDR  0x6A
#define MMC_ADDR  0x30

/* Handles on the mock bus; the descriptor is ignored, the address selects
 * which register file a transfer lands in. */
#define I2CBUS(a) (&(const imud_bus_t){ .kind = BUS_I2C, \
                                        .fd = FD, .i2c_addr = (a) })

/*
 * The same two parts reached over SPI.  There is no address on the wire, so
 * the descriptor stands in for the chip select and spimock_bind() maps it onto
 * the register file the I2C side addresses by number — one device, two
 * framings, which is what makes a transport-conditional check testable.
 */
#define SPI_FD_IMU 71
#define SPI_FD_MAG 72
#define SPIBUS(f) (&(const imud_bus_t){ .kind = BUS_SPI, .fd = (f), \
                                        .spi_mode = 3, .spi_inc_mask = 0, \
                                        .spi_hz = 10000000 })

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
    /* FIFO_DATA_OUT is 0x78 (tag) through 0x7E (Z high) and a read anywhere in
     * that window pops a word, so a register sweep that enters it silently
     * eats data.  Model the whole window, not just the first register. */
    i2cmock_set_fifo_range(ISM_ADDR, 0x78, 0x7E);
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
    bool   abort_now;         /* prompt returns -1: the operator declined */
    bool   sigint_now;        /* prompt raises SIGINT's abort, then continues */
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
    /* Ctrl-C while the operator is at a prompt: the signal handler calls
     * imt_request_abort() and the prompt still returns normally. */
    if (s->sigint_now) { imt_request_abort(); return 0; }
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

/*
 * Nearest supported IMU ODR at or below / at or above `hz`; 0 if the grid has
 * none.  The rate checks pick their configured ODR FROM what the host actually
 * measured, so a slow or contended box cannot drift a case across the
 * tolerance band and fail a precondition instead of the behaviour under test.
 * A fixed 208 Hz against a ~400 Hz loop looks like plenty of margin until the
 * dev box — a dual-core 1.6 GHz i5 running a 4 GB VM — has something else on it
 * the loop halves.  The multipliers below are deliberately generous for the
 * same reason: the grid is coarse enough that a 3x ask costs nothing.
 */
static int grid_below(const imt_report_t *r, double hz)
{
    int best = 0;
    for (int i = 0; i < 16 && r->imu_odr_tab[i]; i++)
        if (r->imu_odr_tab[i] <= hz && r->imu_odr_tab[i] > best)
            best = r->imu_odr_tab[i];
    return best;
}

static int grid_above(const imt_report_t *r, double hz)
{
    int best = 0;
    for (int i = 0; i < 16 && r->imu_odr_tab[i]; i++)
        if (r->imu_odr_tab[i] >= hz && (best == 0 || r->imu_odr_tab[i] < best))
            best = r->imu_odr_tab[i];
    return best;
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
    int rc = imt_run_ops(I2CBUS(ISM_ADDR), I2CBUS(MMC_ADDR),
                         &ism330dhcx_ops, &mmc5983ma_ops, cfg, o, r,
                         err, sizeof err);
    if (rc < 0) fprintf(stderr, "  imt_run_ops: %s\n", err);
    return r;
}

/* The same run over SPI, with whatever bindings the caller has already set. */
static imt_report_t *run_spi_bound(imud_config_t *cfg, imt_opts_t *o)
{
    imt_report_t *r = calloc(1, sizeof *r);
    char err[256] = "";
    int rc = imt_run_ops(SPIBUS(SPI_FD_IMU), SPIBUS(SPI_FD_MAG),
                         &ism330dhcx_ops, &mmc5983ma_ops, cfg, o, r,
                         err, sizeof err);
    if (rc < 0) fprintf(stderr, "  imt_run_ops(spi): %s\n", err);
    return r;
}

/* Bind both parts as parts that walk the address unaided, then run.  Bind
 * after mock_base(), which resets the mock and forgets every binding. */
static imt_report_t *run_spi(imud_config_t *cfg, imt_opts_t *o)
{
    spimock_bind_inc(SPI_FD_IMU, ISM_ADDR, SPIMOCK_INC_ALWAYS, 0);
    spimock_bind_inc(SPI_FD_MAG, MMC_ADDR, SPIMOCK_INC_ALWAYS, 0);
    return run_spi_bound(cfg, o);
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

/*
 * The harness's own fidelity, asserted first because everything below leans on
 * it.  test_fifo_port_window_not_swept can only catch a sweep that walks into
 * the FIFO port if the mock actually pops on a read anywhere in the window; if
 * the mock modelled only the first register, that test would keep passing
 * while the defect it exists for went undetected.
 */
static void test_mock_models_the_whole_fifo_window(void)
{
    begin("test_mock_models_the_whole_fifo_window");
    int fb = g_fail;

    i2cmock_reset();
    i2cmock_set_fifo_range(ISM_ADDR, 0x78, 0x7E);
    const uint8_t staged[4] = { 0xA1, 0xB2, 0xC3, 0xD4 };
    i2cmock_fifo_push(ISM_ADDR, staged, 4);

    /* A single-byte read from the middle of the window must pop, not return
     * the register file — that is what makes a blind sweep destructive. */
    uint8_t v = 0;
    EXPECT(bus_reg_read(I2CBUS(ISM_ADDR), 0x7B, &v) == 0, "mid-window read succeeds");
    EXPECT(v == 0xA1, "a read inside the window pops the queue");
    EXPECT(bus_reg_read(I2CBUS(ISM_ADDR), 0x7E, &v) == 0, "end-of-window read succeeds");
    EXPECT(v == 0xB2, "the queue advanced, so the whole window is a port");

    /* Just outside it is an ordinary register. */
    i2cmock_set_reg(ISM_ADDR, 0x77, 0x5A);
    EXPECT(bus_reg_read(I2CBUS(ISM_ADDR), 0x77, &v) == 0, "outside-window read succeeds");
    EXPECT(v == 0x5A, "0x77 is still the register file");

    end(fb);
}

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
    /* Nothing moves in a plain register file, so nothing is filtered and the
     * whole mapped range is compared. */
    EXPECT(r->raw.n_volatile_imu == 0, "static mock has no volatile registers");
    EXPECT(r->raw.n_scanned_imu > 100, "the mapped range was compared");
    /*
     * The ST FIFO port is seven registers wide (0x78 tag, then X/Y/Z low/high
     * through 0x7E) and a single-byte read of any of them pops a word.  The
     * sweep must not enter that window; when only 0x78/0x79 were excluded,
     * 0x7A/0x7B/0x7D turned up in a shipped report's diff as though they were
     * control registers.
     */
    EXPECT(r->raw.n_regdiff_imu > 0, "diff is over control registers");
    for (int i = 0; i < r->raw.n_regdiff_imu; i++)
        EXPECT(r->raw.regdiff_imu[i].reg < 0x78 || r->raw.regdiff_imu[i].reg > 0x7E,
               "no FIFO-port register in the diff");

    free(r);
    end(fb);
}

/*
 * The registers that move on their own must not reach either register check.
 * Before the volatile scan existed, the whole live range — sensor output, FIFO
 * level, the timestamp counter — landed in the diff and made a second init()
 * look state-dependent: a shipped ISM330DHCX report showed 29 "changed"
 * registers of which only 9 were writes, and 23 "differing" on the repeat.
 */
static void test_volatile_registers_filtered(void)
{
    begin("test_volatile_registers_filtered");
    int fb = g_fail;

    mock_base();
    /*
     * Registers the ISM driver never touches, so making them move exercises
     * the scan without perturbing the FIFO level or the timestamp the driver
     * itself reads.  On silicon these would be STATUS_REG and the output
     * words; here they only have to change with no write in between.
     */
    i2cmock_set_live(ISM_ADDR, 0x1E, 3);
    i2cmock_set_live(ISM_ADDR, 0x50, 7);
    i2cmock_set_live(ISM_ADDR, 0x51, 1);

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    EXPECT(r->raw.n_volatile_imu >= 3, "the live registers were detected");
    EXPECT(r->raw.n_volatile_imu < 16, "the scan did not over-match");
    for (int i = 0; i < r->raw.n_regdiff_imu; i++) {
        uint8_t g = r->raw.regdiff_imu[i].reg;
        EXPECT(g != 0x1E && g != 0x50 && g != 0x51,
               "a live register reached the diff");
    }
    /* The point of the filter: a moving register must not read as a
     * state-dependent init(). */
    EXPECT(status_of(r, "imu.init.idempotent") == IMT_PASS,
           "live registers do not break idempotency");
    EXPECT(status_of(r, "imu.init.regdiff") == IMT_PASS,
           "control-register writes still show through the filter");
    EXPECT(r->raw.n_scanned_imu + r->raw.n_volatile_imu > 100,
           "excluded plus compared covers the mapped range");

    free(r);
    end(fb);
}

/*
 * The register sweep must not enter the FIFO port window.  On the ST parts
 * FIFO_DATA_OUT is 0x78-0x7E and a read of any of the seven pops a word; a skip
 * list naming only 0x78 and 0x79 left the sweep eating five words per snapshot,
 * and 0x7A/0x7B/0x7D turned up in a shipped report's diff as though they were
 * control registers.
 */
static void test_fifo_port_window_not_swept(void)
{
    begin("test_fifo_port_window_not_swept");
    int fb = g_fail;

    mock_base();
    /*
     * Park a distinctive pattern in the FIFO and leave FIFO_STATUS at 0, so
     * the driver itself never pops it.  Anything that reaches these bytes did
     * so by sweeping the port window.  Every byte is nonzero and no two
     * snapshots would see the same ones, so a sweep that entered the window
     * shows up both as a "changed" register and as a volatile one.
     */
    for (int i = 0; i < 512; i++) {
        uint8_t w[7] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                         (uint8_t)(0x80 + (i & 0x3F)) };
        i2cmock_fifo_push(ISM_ADDR, w, 7);
    }
    i2cmock_set_reg(ISM_ADDR, 0x3A, 0);

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);
    g_s.feed_imu = false;          /* nothing must top the FIFO up */

    imt_report_t *r = run(&cfg, &o);

    for (int i = 0; i < r->raw.n_regdiff_imu; i++)
        EXPECT(r->raw.regdiff_imu[i].reg < 0x78 ||
               r->raw.regdiff_imu[i].reg > 0x7E,
               "a FIFO-port register reached the diff");
    EXPECT(r->raw.n_volatile_imu == 0,
           "nothing in the port window looked volatile, so nothing read it");
    EXPECT(status_of(r, "imu.init.regdiff") == IMT_PASS,
           "the control-register diff still ran");

    free(r);
    end(fb);
}

/*
 * A write-only control register file cannot be diffed at all, and saying so is
 * the only honest result.  Grading it WARN — which is what happened before —
 * blamed the MMC5983MA driver for a property of the silicon.
 */
/* Position of a check in the report, so ordering can be asserted. -1 if absent. */
static int index_of(const imt_report_t *r, const char *id)
{
    for (int i = 0; i < r->n_checks; i++)
        if (strcmp(r->check[i].id, id) == 0) return i;
    return -1;
}

/*
 * The degauss pulse must land BEFORE the field is measured, and the SET/RESET
 * pair must leave the part SET.
 *
 * Ordering is the whole point: run last, as it was, mag.field_magnitude and
 * mag.noise graded whatever magnetisation the part arrived in. The mock has no
 * coil, so the differential correctly recovers no field here — which is the
 * dead-coil signature, and is exactly what the check should say.
 */
static void test_mag_degauss_ordering_and_restore(void)
{
    begin("test_mag_degauss_ordering_and_restore");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    int i_set = index_of(r, "mag.set_reset");
    int i_fld = index_of(r, "mag.field_magnitude");
    EXPECT(i_set >= 0 && i_fld >= 0, "both checks ran");
    EXPECT(i_set < i_fld, "the degauss pulse is graded before the field is");
    EXPECT(note_contains(r, "mag.set_reset", "before the measurements"),
           "the note says which way round it runs");

    /* The differential ran, and on a coil that does nothing it reports the
     * reading as offset rather than as field. */
    EXPECT(status_of(r, "mag.degauss.differential") != (imt_status_t)-1,
           "the differential ran");
    EXPECT(index_of(r, "mag.degauss.differential") > i_fld,
           "the differential runs after the steady-state measurements");

    /*
     * CTRL0 is left in the SET state. RESET inverts the field term, so
     * finishing there would hand a sign-flipped magnetometer to anything that
     * ran next — including a daemon started straight after imutest.
     */
    EXPECT(i2cmock_get_reg(MMC_ADDR, 0x09) == 0x0C,
           "the part is left SET, not RESET");

    free(r);
    end(fb);
}

/* A driver with no directional degauss must SKIP, not fail or crash. */
static void test_mag_degauss_skips_without_the_op(void)
{
    begin("test_mag_degauss_skips_without_the_op");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    /* Same part, with the optional op withheld — every other mag driver. */
    mag_ops_t noop = mmc5983ma_ops;
    noop.degauss = NULL;

    imt_report_t *r = calloc(1, sizeof *r);
    char err[256] = "";
    imt_run_ops(I2CBUS(ISM_ADDR), I2CBUS(MMC_ADDR),
                &ism330dhcx_ops, &noop, &cfg, &o, r, err, sizeof err);

    EXPECT(status_of(r, "mag.degauss.differential") == IMT_SKIP,
           "no directional degauss skips the differential");
    EXPECT(note_contains(r, "mag.degauss.differential", "RESET"),
           "the skip says what is missing");
    /* The plain SET pulse still runs — set_reset is unaffected. */
    EXPECT(status_of(r, "mag.set_reset") == IMT_PASS,
           "the production degauss is unaffected");

    free(r);
    end(fb);
}

static void test_mag_writeonly_ctrl_skips(void)
{
    begin("test_mag_writeonly_ctrl_skips");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    EXPECT(status_of(r, "mag.init.regdiff") == IMT_SKIP,
           "write-only control registers skip the diff");
    EXPECT(note_contains(r, "mag.init.regdiff", "write-only"),
           "the skip names the reason");
    /* The IMU's registers do read back, so its diff must still run. */
    EXPECT(status_of(r, "imu.init.regdiff") == IMT_PASS,
           "the IMU diff is unaffected");

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

/*
 * A rate ABOVE the configured ODR is a different finding from one below it,
 * and grading them alike is how an MMC5983MA reading 130 Hz against a
 * configured 100 Hz reached a bench report as a WARN nobody acted on.
 *
 * Below: the poll loop bounds the measurement from above, so a low reading may
 * be pacing — WARN.  Above: nothing in the sampling path can invent samples
 * the part did not produce, so the part is not running at the rate init()
 * asked for — FAIL.
 *
 * The lever is the CONFIGURED rate, not the loop: at a configured 1 Hz the
 * mock's poll loop overshoots by orders of magnitude whatever speed the host
 * manages, so the direction is deterministic even though the rate is not.
 */
static void test_rate_above_configured_odr_fails(void)
{
    begin("test_rate_above_configured_odr_fails");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    cfg.mag_odr_hz = 1;                  /* on the MMC grid; loop far outruns it */
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    EXPECT(r->mag_eff_odr_hz == 1, "1 Hz is on the MMC grid");
    EXPECT(r->raw.mag_n >= 5, "enough samples to grade the rate at all");
    EXPECT(r->raw.mag_rate_hz > r->mag_eff_odr_hz * (1.0 + o.odr_tol_warn),
           "the measurement really is above the configured rate");
    EXPECT(status_of(r, "mag.rate") == IMT_FAIL,
           "a mag rate above the configured ODR FAILs");
    EXPECT(note_contains(r, "mag.rate", "ABOVE"),
           "the note says which direction it is off");
    EXPECT(note_contains(r, "mag.rate", "did not land"),
           "the note names the likely cause");
    EXPECT(!note_contains(r, "mag.rate", "can be pacing"),
           "the pacing excuse is not offered for an over-rate reading");

    /*
     * imu.odr has the same shape. Direction decides which excuses apply, but
     * NOT the grade: a part's own oscillator tolerance can exceed odr_tol_warn
     * on good silicon — the reference MMC5983MA runs 5.4% fast — so an
     * over-rate reading is graded on the same ladder and only FAILs past
     * odr_tol_fail. Both sides of that are pinned below.
     *
     * The configured rate is chosen FROM the measured one rather than fixed,
     * so a slow or contended host cannot drift the case across the boundary
     * and fail a precondition instead of the behaviour.
     */
    int slow = grid_below(r, r->raw.odr_measured_hz / 2.0);
    free(r);
    EXPECT(slow > 0, "the ISM330 grid has an entry well under the loop rate");
    if (slow <= 0) { end(fb); return; }

    cfg.imu_odr_hz = slow;
    mock_base();
    script_reset(&o);
    r = run(&cfg, &o);

    double imu_err = fabs(r->raw.odr_measured_hz - r->eff_odr_hz) / r->eff_odr_hz;
    free(r);

    /* Far above the configured rate: past odr_tol_fail, so a FAIL. */
    mock_base();
    script_reset(&o);
    r = run(&cfg, &o);
    EXPECT(r->raw.odr_measured_hz > r->eff_odr_hz * (1.0 + o.odr_tol_warn),
           "the IMU loop overshoots the rate picked for it");
    EXPECT(imu_err > o.odr_tol_fail, "and by more than the fail tolerance");
    EXPECT(status_of(r, "imu.odr") == IMT_FAIL,
           "a large over-rate imu.odr FAILs");
    EXPECT(note_contains(r, "imu.odr", "ABOVE") ||
           note_contains(r, "imu.odr", "instead"),
           "the note explains an over-rate reading");
    free(r);

    /*
     * The same overshoot, inside a widened warn band: still a WARN. Promoting
     * this to FAIL is what a bench run caught as wrong — the reference
     * magnetometer measures 105.4 Hz against a configured 100 on both
     * transports, which is the die's oscillator, not a driver defect.
     */
    mock_base();
    script_reset(&o);
    o.odr_tol_fail = imu_err * 4.0;
    r = run(&cfg, &o);

    EXPECT(fabs(r->raw.odr_measured_hz - r->eff_odr_hz) / r->eff_odr_hz
           <= o.odr_tol_fail,
           "the over-rate error really is inside the widened warn band");
    EXPECT(status_of(r, "imu.odr") == IMT_WARN,
           "an over-rate reading inside the band stays a WARN");

    free(r);
    end(fb);
}

/*
 * The other side, and the reason the split is not just "grade everything off
 * FAIL": a low reading keeps its WARN, because the read loop genuinely can be
 * what bounded it.
 *
 * Driven on imu.odr, where both directions are reachable: the mock's IMU loop
 * manages a few hundred Hz, so configuring 833 Hz produces a real shortfall.
 * The mag path cannot express this — its poll loop outruns the MMC5983MA's
 * fastest grid entry (1000 Hz), so mag.rate can only ever measure high here.
 * Same rule, same shape, one site of it exercised.
 */
static void test_rate_below_configured_odr_warns(void)
{
    begin("test_rate_below_configured_odr_warns");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    /* Measure first, then ask for a grid rate at twice what the host managed,
     * so the shortfall is guaranteed rather than assumed. */
    imt_report_t *probe = run(&cfg, &o);
    int fast = grid_above(probe, probe->raw.odr_measured_hz * 3.0);
    free(probe);
    EXPECT(fast > 0, "the ISM330 grid has an entry well above the loop rate");
    if (fast <= 0) { end(fb); return; }

    cfg.imu_odr_hz = fast;
    mock_base();
    script_reset(&o);
    probe = run(&cfg, &o);
    EXPECT(probe->raw.odr_measured_hz < probe->eff_odr_hz * (1.0 - o.odr_tol_warn),
           "the loop really does fall short of the rate picked for it");
    double err = fabs(probe->raw.odr_measured_hz - probe->eff_odr_hz)
                 / probe->eff_odr_hz;
    free(probe);

    /* Widen the fail bound past the shortfall: a low reading in the warn band
     * must stay a WARN, where an equally-sized overshoot became a FAIL above. */
    mock_base();
    script_reset(&o);
    o.odr_tol_fail = err * 2.0;
    imt_report_t *r = run(&cfg, &o);

    EXPECT(r->raw.odr_measured_hz < r->eff_odr_hz,
           "still measuring below the configured rate");
    EXPECT(status_of(r, "imu.odr") == IMT_WARN,
           "a rate below the configured ODR stays a WARN");
    EXPECT(!note_contains(r, "imu.odr", "ABOVE"),
           "a low reading is not described as over-rate");

    free(r);
    end(fb);
}

/*
 * mag.burst_framing: an N-byte burst must land where N single reads do.
 *
 * bus_burst_read() passes spi_inc_mask only when len > 1, so the two take
 * different paths through the command byte — the burst asserts the part's
 * auto-increment bit, the singles do not. A wrong mask makes them disagree,
 * which is the on-hardware half of what test_drivers proves against the mock.
 *
 * It cannot see a fault inside spi_burst_read() itself, since bus_reg_read()
 * is bus_burst_read(len=1) and both sides go through it. That is stated in the
 * check's own comment so the report is not over-read.
 */
static void test_burst_framing(void)
{
    begin("test_burst_framing");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;

    mock_base();
    script_reset(&o);
    imt_report_t *r = run(&cfg, &o);

    EXPECT(status_of(r, "mag.burst_framing") == IMT_PASS,
           "burst and single reads agree on a part that walks the address");
    EXPECT(r->raw.mag_bf_n == 7, "the whole 0x00-0x06 window was captured raw");
    EXPECT(memcmp(r->raw.mag_bf_burst, r->raw.mag_bf_single,
                  (size_t)r->raw.mag_bf_n) == 0,
           "...and the two recorded halves are identical");
    /* Non-degeneracy: staged output, not seven zeroes agreeing with themselves. */
    EXPECT(r->raw.mag_bf_burst[0] != r->raw.mag_bf_burst[2],
           "the captured window holds real staged data");
    free(r);

    /*
     * Now the failure it exists to catch: a part whose address does not walk.
     * The burst returns register 0x00 over and over while the singles step
     * correctly, which is exactly what a wrong spi_inc_mask produces.
     */
    mock_base();
    spimock_bind_inc(SPI_FD_IMU, ISM_ADDR, SPIMOCK_INC_ALWAYS, 0);
    spimock_bind_inc(SPI_FD_MAG, MMC_ADDR, SPIMOCK_INC_NEVER, 0);
    script_reset(&o);
    imt_report_t *stuck = run_spi_bound(&cfg, &o);

    EXPECT(status_of(stuck, "mag.burst_framing") == IMT_FAIL,
           "a burst that does not walk the address FAILs the framing check");
    EXPECT(note_contains(stuck, "mag.burst_framing", "spi_inc_mask"),
           "the note names the field to go and check");
    free(stuck);

    end(fb);
}

/*
 * probe.reject cannot be run over SPI, and must say so rather than fail.
 *
 * The check mutates bus.i2c_addr to a reserved address and requires probe() to
 * reject it.  On SPI the chip select does the addressing and i2c_addr never
 * reaches the wire, so the "bogus" probe reads the same part, gets the right
 * WHO_AM_I, and returns 0 — the check misfiring, not the driver failing it.
 * Graded FAIL it put two phantom rows and a nonzero exit on every SPI report.
 *
 * SKIP is deliberately still a blocker for clearing `experimental`: the
 * evidence genuinely was not obtained, and the honest report of that is "not
 * verified here", not "verified".
 */
static void test_probe_reject_skips_on_spi(void)
{
    begin("test_probe_reject_skips_on_spi");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;

    /* I2C first: the check is real there and must keep working. */
    mock_base();
    script_reset(&o);
    imt_report_t *i2c = run(&cfg, &o);
    EXPECT(status_of(i2c, "imu.probe.reject") == IMT_PASS,
           "on I2C the IMU bogus-address probe still runs and passes");
    EXPECT(status_of(i2c, "mag.probe.reject") == IMT_PASS,
           "on I2C the mag bogus-address probe still runs and passes");
    free(i2c);

    mock_base();
    script_reset(&o);
    imt_report_t *spi = run_spi(&cfg, &o);

    /* Non-degeneracy: the SPI run has to have got far enough to probe at all,
     * or the SKIPs below would just be absent checks. */
    EXPECT(status_of(spi, "imu.probe") == IMT_PASS, "the IMU probed over SPI");
    EXPECT(status_of(spi, "mag.probe") == IMT_PASS, "the mag probed over SPI");

    EXPECT(status_of(spi, "imu.probe.reject") == IMT_SKIP,
           "imu.probe.reject SKIPs on SPI rather than failing");
    EXPECT(status_of(spi, "mag.probe.reject") == IMT_SKIP,
           "mag.probe.reject SKIPs on SPI rather than failing");
    EXPECT(note_contains(spi, "imu.probe.reject", "chip select"),
           "the SKIP says why it cannot be tested");
    EXPECT(note_contains(spi, "mag.probe.reject", "chip select"),
           "the mag SKIP says why too");

    free(spi);
    end(fb);
}

/*
 * imutest must never READ a write-only register.
 *
 * The MMC5983MA's CTRL0..CTRL3 (0x09-0x0C) are Mode W in Rev A, and its
 * readable file ends at 0x08 — 0x0D-0x2E is reserved, 0x2F is the product ID.
 * Reading a write-only register returns undefined data that the volatile scan
 * and the diff would then reason about, and on another part could have side
 * effects.
 *
 * Two independent things keep it off them, and this pins the OUTCOME rather
 * than either mechanism: ctrl_writeonly skips the mag snapshot entirely, and
 * the regmap range stops at the last readable register.  Losing one is
 * survivable; losing both is the defect.
 *
 * Only a read TALLY can show this. A readback proves nothing, because the harm
 * of reading a write-only register IS the read.
 */
static void test_sweep_avoids_write_only_registers(void)
{
    begin("test_sweep_avoids_write_only_registers");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases  = IMT_PHASE_PASSIVE;
    o.regdiff = true;                    /* the sweep only runs for regdiff */
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    /*
     * Non-degeneracy: the tally has to be live, or every zero below is
     * meaningless.  0x2F is the product ID, which probe() reads.
     */
    EXPECT(i2cmock_read_count(MMC_ADDR, 0x2F) > 0,
           "the read tally is recording mag reads at all");

    for (uint8_t reg = 0x09; reg <= 0x0C; reg++) {
        char msg[64];
        snprintf(msg, sizeof msg, "write-only CTRL 0x%02X was never read", reg);
        EXPECT(i2cmock_read_count(MMC_ADDR, reg) == 0, msg);
    }
    EXPECT(i2cmock_read_count(MMC_ADDR, 0x10) == 0, "reserved 0x10 was never read");
    EXPECT(i2cmock_read_count(MMC_ADDR, 0x1F) == 0, "reserved 0x1F was never read");

    /* The check itself still SKIPs, and still for the silicon's reason. */
    EXPECT(status_of(r, "mag.init.regdiff") == IMT_SKIP,
           "regdiff SKIPs on a write-only control file");

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
    /*
     * The note's two conversions used to be fed one argument, so the second
     * read off the end of the va_list and a shipped report printed "averaged
     * over 1 s; ratio to true g is 0.000" for a 10 s window at ratio 0.995.
     * Both numbers have to be the real ones.
     */
    EXPECT(note_contains(r, "imu.rest.gravity", "1.000"),
           "the note carries the true ratio, not a garbage 0.000");
    EXPECT(!note_contains(r, "imu.rest.gravity", "0.000"),
           "no uninitialised value in the note");
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

/*
 * The two-pass DRDY count.
 *
 * ROADMAP section 1.1 left the reference part's edge rate uncharacterised:
 * ~18.3 Hz at 833 Hz with fifo_wm 64 fits neither the per-sample model (833)
 * nor the watermark model (13), and a single count cannot say why.  The
 * surviving hypothesis was that INT1_FIFO_TH is a LEVEL condition oscillating
 * across the threshold while the drain empties it — in which case the number
 * is an artifact of draining, not a rate.
 *
 * Counting again with nothing draining separates the candidates: a level
 * condition asserts once and stays asserted, an edge-per-sample line keeps
 * pulsing.  What is asserted here is that the tool reports both and stops
 * grading a part down for fitting neither model, which is what the reference
 * part — healthy — actually does.
 */
static void test_drdy_two_pass(void)
{
    begin("test_drdy_two_pass");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    cfg.imu_int_gpio = 17;
    g_gpio_why = IMT_GPIO_OK;

    /* A level condition: edges while draining, one when not. */
    mock_base(); script_reset(&o);
    g_gpio_edges      = 12;
    g_gpio_edges_idle = 1;
    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_PASS,
           "a rate fitting neither model is no longer graded down");
    EXPECT(r->raw.gpio_edges == 12 && r->raw.gpio_edges_idle == 1,
           "both counts reach the report");
    EXPECT(r->raw.gpio_idle_valid, "the second pass is marked valid");
    EXPECT(note_contains(r, "imu.drdy.edges", "level condition"),
           "and the note names the level condition, which is the finding");
    free(r);

    /* An edge per sample: the undrained count tracks the drained one. */
    mock_base(); script_reset(&o);
    g_gpio_edges      = 12;
    g_gpio_edges_idle = 12;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_PASS, "edge-per-sample passes");
    EXPECT(note_contains(r, "imu.drdy.edges", "undrained too"),
           "the note says the line pulses independently of the drain");
    free(r);

    /*
     * More edges than the part has samples is still a finding — it cannot be
     * an interrupt this part raised, so the line is floating or shared.
     */
    mock_base(); script_reset(&o);
    g_gpio_edges      = 100000;
    g_gpio_edges_idle = 1;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_WARN,
           "a rate above the sample rate still warns");
    free(r);

    g_gpio_edges      = -1;
    g_gpio_edges_idle = 1;
    g_gpio_why        = IMT_GPIO_ENOCHIP;
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
    /*
     * All three turns must reach the appendix.  A row cap of 2 against a
     * 3-slot array silently dropped Z: it was graded here but never printed,
     * so the shipped report's rotation table had two rows and no sign of the
     * third.
     */
    EXPECT(r->raw.n_turns == 3, "every turn recorded a row");
    EXPECT(r->raw.turn[2].axis == 2, "the third row is the Z turn");
    EXPECT(r->raw.turn[2].theta[2] > 0.0, "the Z row carries its integral");
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
    /* The sweep is on here because its two appendix tables are part of what
     * this test checks the formatting of. */
    o.fs_sweep = true;
    script_reset(&o);
    g_s.accel[2] = -9.80665;

    imt_report_t *r = run(&cfg, &o);
    EXPECT(r->raw.n_fs_gyro > 1, "the gyro full-scale table has rows to format");

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

        /*
         * Appendix numbering must not skip.  5.6 used to be emitted only when
         * the mag diff had rows, so a part whose control registers do not read
         * back left a hole between 5.5 and 5.7 that read as a missing section.
         */
        EXPECT(strstr(buf, "### 5.5 Control-register diff, IMU") != NULL,
               "5.5 present");
        EXPECT(strstr(buf, "### 5.6 Control-register diff, mag") != NULL,
               "5.6 present even with nothing to show");
        EXPECT(strstr(buf, "write-only") != NULL,
               "5.6 says why there is nothing to show");
        /* The filtering has to be visible, or a narrower test reads as a
         * cleaner chip. */
        EXPECT(strstr(buf, "excluded as volatile") != NULL,
               "the volatile count is reported");
        /* "-0.00" was printed for the first gyro full-scale row, which has no
         * previous range to compare against. */
        EXPECT(strstr(buf, "-0.00") == NULL, "no negative-zero ratio cell");

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

    /*
     * The note is the diagnosis, so losing its tail loses the part that says
     * what to do about the number.  snprintf truncates silently, and a note
     * written just over the buffer reads fine in review and arrives cut off:
     * "the counter runs faster than ts_tick_ns say".  Nothing may fill the
     * buffer exactly.
     */
    for (int i = 0; i < r->n_checks; i++) {
        size_t len = strlen(r->check[i].note);
        if (len >= sizeof r->check[i].note - 1) {
            fprintf(stderr, "  note truncated: %s\n", r->check[i].id);
            EXPECT(false, "a check note was truncated to the buffer");
        }
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

/*
 * imt_decide_verdict() must consult the experimental flags rather than assume
 * they are set.  A shipped report told the reader to clear `experimental` for
 * two drivers that had shipped with it clear for releases, which reads as
 * stale advice in the issue the report gets pasted into.
 *
 * Driven directly rather than through a run: the clean branch needs every
 * required check to PASS, which the mock bus cannot deliver, so a test that
 * went through imt_run_ops would land in the blocker branch and pass whether
 * or not this logic was right.
 */
static void stage_clean_report(imt_report_t *r, bool imu_exp, bool mag_exp)
{
    static const char *required[] = {
        "imu.probe", "imu.probe.reject", "imu.reset.rc", "imu.init.rc",
        "imu.odr", "imu.seq.monotonic", "imu.seq.gapless",
        "imu.err.nodata_not_error", "imu.err.no_spurious",
        "imu.noise.accel", "imu.noise.gyro", "imu.rest.gravity",
        "imu.temp.plausible", "imu.chipts.presence", "imu.fs.accel",
        "faces.frame", "gyro.x.sign", "gyro.y.sign", "gyro.z.sign",
        "mag.probe", "mag.init.rc", "mag.rate", "mag.nodata_not_error",
        "mag.field_magnitude", "mag.noise", "mag.wall_ns",
        "spin.magnitude", "spin.frame_agreement",
    };
    memset(r, 0, sizeof *r);
    snprintf(r->imu_driver, sizeof r->imu_driver, "ism330dhcx");
    snprintf(r->mag_driver, sizeof r->mag_driver, "mmc5983ma");
    r->have_mag          = true;
    r->imu_experimental  = imu_exp;
    r->mag_experimental  = mag_exp;
    r->phases_requested  = IMT_PHASE_ALL;
    r->phases_run        = IMT_PHASE_ALL;
    for (size_t i = 0; i < sizeof required / sizeof required[0]; i++) {
        imt_check_t *c = &r->check[r->n_checks++];
        snprintf(c->id, sizeof c->id, "%s", required[i]);
        c->status = IMT_PASS;
        r->n_pass++;
    }
}

/*
 * The chip-time/wall-time bands, which are asymmetric by design.
 *
 * A fast oscillator and a dropped counter wrap produce deviations of the same
 * magnitude in opposite directions, and only one of them is a defect: since
 * 1.8 the daemon measures the counter's real period per anchor, so a part
 * running fast is absorbed, while time that has gone missing cannot be. The
 * 1.041 the Pi 5 bench reported on the reference ISM330DHCX is the case that
 * used to warn and now must not.
 */
/*
 * The SET/RESET split. Pure arithmetic, so it is asserted directly rather than
 * through the mock: nothing on a software bus can magnetise a bridge, and a
 * test that staged two register blocks and read the difference back would be
 * grading its own staging.
 */
static bool near_d(double a, double b) { return fabs(a - b) < 1e-9; }

static void test_degauss_split(void)
{
    begin("test_degauss_split");
    int fb = g_fail;

    double field[3], offset[3];

    /* The case the check exists for, and the 2026-08-15 bench numbers it was
     * written against: a ~50 uT field buried under a ~1100 uT bridge offset.
     * Both halves read far out of range; only the split says which is which. */
    const double vS[3] = { 1130.0,  -30.0,  1150.0 };
    const double vR[3] = { 1070.0,  -70.0,  1050.0 };
    imt_degauss_split(vS, vR, field, offset);
    EXPECT(near_d(field[0], 30.0), "field X = (vS-vR)/2");
    EXPECT(near_d(field[1], 20.0), "field Y = (vS-vR)/2");
    EXPECT(near_d(field[2], 50.0), "field Z = (vS-vR)/2");
    EXPECT(near_d(offset[0], 1100.0), "offset X = (vS+vR)/2");
    EXPECT(near_d(offset[1], -50.0), "offset Y = (vS+vR)/2");
    EXPECT(near_d(offset[2], 1100.0), "offset Z = (vS+vR)/2");

    /* A part with no offset: RESET is the exact negation of SET, so the field
     * is the whole reading and the offset vanishes. */
    const double pS[3] = {  20.0, -30.0,  45.0 };
    const double pR[3] = { -20.0,  30.0, -45.0 };
    imt_degauss_split(pS, pR, field, offset);
    EXPECT(near_d(field[0], 20.0), "no offset: field is the reading");
    EXPECT(near_d(field[2], 45.0), "no offset: field is the reading (Z)");
    EXPECT(near_d(offset[0], 0.0), "no offset: offset is zero");
    EXPECT(near_d(offset[2], 0.0), "no offset: offset is zero (Z)");

    /* A dead coil: the pulse changes nothing, so both halves are identical.
     * The whole reading lands in the offset and the field is zero — which is
     * how a degauss path that is not working reports itself. */
    const double dS[3] = { 320.0, -764.0, -761.0 };
    imt_degauss_split(dS, dS, field, offset);
    EXPECT(near_d(field[0], 0.0), "dead coil: no field recovered");
    EXPECT(near_d(field[1], 0.0), "dead coil: no field recovered (Y)");
    EXPECT(near_d(field[2], 0.0), "dead coil: no field recovered (Z)");
    EXPECT(near_d(offset[0], 320.0), "dead coil: reading is all offset");
    EXPECT(near_d(offset[2], -761.0), "dead coil: reading is all offset (Z)");

    /* Sign convention: swapping the two halves flips the field and leaves the
     * offset alone. A driver that had SET and RESET the wrong way round would
     * report a negated field, not a wrong magnitude — so |field| cannot catch
     * it and the vector in the report is what a reader needs. */
    double f2[3], o2[3];
    imt_degauss_split(vR, vS, f2, o2);
    imt_degauss_split(vS, vR, field, offset);
    EXPECT(near_d(f2[0], -field[0]), "swapping the halves negates the field");
    EXPECT(near_d(o2[0], offset[0]), "swapping the halves leaves the offset");

    /* Either output is optional. */
    imt_degauss_split(vS, vR, field, NULL);
    EXPECT(near_d(field[2], 50.0), "NULL offset is accepted");
    imt_degauss_split(vS, vR, NULL, offset);
    EXPECT(near_d(offset[2], 1100.0), "NULL field is accepted");

    end(fb);
}

static void test_chipts_wall_bands(void)
{
    begin("test_chipts_wall_bands");
    int fb = g_fail;

    /* Values sit clearly inside each band, never on a boundary: 1.02 - 1.0 is
     * 0.020000000000000018 in binary, so an exact-edge assertion would test
     * the float representation rather than the rule. The bands are engineering
     * tolerances, not contracts anyone reads to three decimal places. */
    EXPECT(imt_chipts_wall_status(1.0000) == IMT_PASS, "exact ratio passes");
    EXPECT(imt_chipts_wall_status(1.0150) == IMT_PASS, "+1.5% is inside the pass band");
    EXPECT(imt_chipts_wall_status(0.9850) == IMT_PASS, "-1.5% is inside the pass band");

    /* The measured bench case, and the reason this function exists. */
    EXPECT(imt_chipts_wall_status(1.0410) == IMT_INFO,
           "a 4% fast oscillator is reported, not warned");
    EXPECT(imt_chipts_wall_status(1.0900) == IMT_INFO, "+9% fast still INFO");

    /* Same magnitude, other direction: chip time missing is never benign. */
    EXPECT(imt_chipts_wall_status(0.9590) == IMT_WARN,
           "4% slow warns — a dropped wrap is unrecoverable");
    EXPECT(imt_chipts_wall_status(0.9100) == IMT_WARN, "-9% slow still WARN");

    /* Past the band the tick simply is not this counter's period. */
    EXPECT(imt_chipts_wall_status(1.2000) == IMT_FAIL, "+20% fails");
    EXPECT(imt_chipts_wall_status(0.8000) == IMT_FAIL, "-20% fails");
    /* A stalled counter reads 0.0 and must not be mistaken for "slow". */
    EXPECT(imt_chipts_wall_status(0.0)    == IMT_FAIL, "a stopped counter fails");

    end(fb);
}

static void test_verdict_respects_experimental_flag(void)
{
    begin("test_verdict_respects_experimental_flag");
    int fb = g_fail;

    static imt_report_t r;

    /* Both flags already clear — the case that shipped wrong. */
    stage_clean_report(&r, false, false);
    imt_decide_verdict(&r);
    EXPECT(r.recommend_clear_experimental, "a clean full run is recognised");
    EXPECT(strstr(r.verdict, "RECOMMEND clearing") == NULL,
           "no recommendation to clear a flag that is already clear");
    EXPECT(strstr(r.verdict, "already") != NULL,
           "the verdict says the flag was already clear");

    /* Both set — the recommendation must still be made, and name both. */
    stage_clean_report(&r, true, true);
    imt_decide_verdict(&r);
    EXPECT(strstr(r.verdict, "RECOMMEND clearing") != NULL,
           "an experimental driver still gets the recommendation");
    EXPECT(strstr(r.verdict, "ism330dhcx") != NULL &&
           strstr(r.verdict, "mmc5983ma") != NULL,
           "both experimental drivers are named");

    /* Only the mag is experimental: the recommendation must name the mag, and
     * must not name the IMU whose flag is already clear. */
    stage_clean_report(&r, false, true);
    imt_decide_verdict(&r);
    EXPECT(strstr(r.verdict, "RECOMMEND clearing") != NULL,
           "the experimental mag still gets the recommendation");
    EXPECT(strstr(r.verdict, "mmc5983ma") != NULL, "the mag is named");
    EXPECT(strstr(r.verdict, "ism330dhcx") == NULL,
           "the already-clear IMU is not named");

    end(fb);
}

/*
 * imt_print writes the short terminal digest — what an operator actually reads
 * when the run finishes, as opposed to the Markdown file they attach to an
 * issue.  It takes a FILE*, so no stdout juggling is needed.
 */
static void test_terminal_digest(void)
{
    begin("test_terminal_digest");
    int fb = g_fail;

    imt_report_t r;
    memset(&r, 0, sizeof r);
    snprintf(r.imud_version, sizeof r.imud_version, "1.8");
    snprintf(r.imu_driver,   sizeof r.imu_driver,   "ism330dhcx");
    snprintf(r.mag_driver,   sizeof r.mag_driver,   "mmc5983ma");
    r.have_mag = true;
    r.n_pass = 20; r.n_warn = 2; r.n_fail = 1; r.n_skip = 3; r.n_info = 4;
    r.wall_duration_s = 42.0;
    r.n_checks = 2;
    r.check[0].status = IMT_FAIL;
    snprintf(r.check[0].id,   sizeof r.check[0].id,   "imu.odr");
    snprintf(r.check[0].name, sizeof r.check[0].name, "measured output rate");
    r.check[1].status = IMT_PASS;
    snprintf(r.check[1].id,   sizeof r.check[1].id,   "imu.probe");
    snprintf(r.check[1].name, sizeof r.check[1].name, "WHO_AM_I");
    snprintf(r.verdict, sizeof r.verdict, "One check failed.");

    char buf[8192] = "";
    FILE *f = tmpfile();
    EXPECT(f != NULL, "tmpfile for the digest");
    if (f) {
        imt_print(&r, f);
        rewind(f);
        size_t n = fread(buf, 1, sizeof buf - 1, f);
        buf[n] = '\0';
        fclose(f);
    }

    EXPECT(strstr(buf, "ism330dhcx") != NULL, "names the IMU driver");
    EXPECT(strstr(buf, "mmc5983ma")  != NULL, "names the magnetometer");
    EXPECT(strstr(buf, "1.8")        != NULL, "carries the imud version");
    EXPECT(strstr(buf, "20 PASS")    != NULL, "reports the pass count");
    EXPECT(strstr(buf, "imu.odr")    != NULL, "lists the failing check");
    EXPECT(strstr(buf, "imu.probe")  == NULL,
           "does NOT list passing checks — the digest is the exceptions only");
    EXPECT(strstr(buf, "One check failed.") != NULL, "prints the verdict");

    /* The sim and daemon-running warnings are the two that change how a
     * reader should weigh everything else, so they must be prominent. */
    r.is_sim = true;
    r.daemon_was_running = true;
    f = tmpfile();
    if (f) {
        imt_print(&r, f);
        rewind(f);
        size_t n = fread(buf, 1, sizeof buf - 1, f);
        buf[n] = '\0';
        fclose(f);
    }
    EXPECT(strstr(buf, "sim") != NULL, "flags a sim run as not-hardware");
    EXPECT(strstr(buf, "imud was running") != NULL,
           "warns that a running daemon makes the timings untrustworthy");
    end(fb);
}

/*
 * SIGINT arrives as imt_request_abort() while the run is live — the handler in
 * imutest_main.c does exactly that.  imt_run_ops clears the flag on entry (so
 * a stale one cannot poison the next run) and folds it back into r->aborted at
 * the end, which is what makes imt_exit_code return 130 instead of a clean 0.
 * A half-finished report exiting 0 would read as a pass.
 */
static void test_abort_stops_the_run(void)
{
    begin("test_abort_stops_the_run");
    int fb = g_fail;

    mock_base();
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE | IMT_PHASE_FACES;
    script_reset(&o);
    g_s.accel[2] = -9.80665;
    g_s.sigint_now = true;          /* Ctrl-C at the first prompt */

    imt_report_t *r = run(&cfg, &o);
    EXPECT(r->aborted, "the report is marked aborted");
    EXPECT(imt_exit_code(r) == 130, "and the exit code says so (130)");
    free(r);

    /* The flag must not survive into the next run, or one Ctrl-C would taint
     * every later invocation in the same process. */
    g_s.sigint_now = false;
    script_reset(&o);
    o.phases = IMT_PHASE_PASSIVE;
    g_s.accel[2] = -9.80665;
    imt_report_t *r2 = run(&cfg, &o);
    EXPECT(!r2->aborted, "a later run starts with the abort flag cleared");
    free(r2);
    end(fb);
}

int main(void)
{
    puts("=== imud-imutest checker tests (mock I2C) ===");

    test_mock_models_the_whole_fifo_window();
    test_bringup_good();
    test_bringup_bad_whoami();
    test_volatile_registers_filtered();
    test_fifo_port_window_not_swept();
    test_mag_degauss_ordering_and_restore();
    test_mag_degauss_skips_without_the_op();
    test_mag_writeonly_ctrl_skips();
    test_burst_framing();
    test_probe_reject_skips_on_spi();
    test_sweep_avoids_write_only_registers();
    test_rate_above_configured_odr_fails();
    test_rate_below_configured_odr_warns();
    test_odr_and_seq();
    test_error_contract_both_ways();
    test_gravity_and_stuck_axis();
    test_temperature_placeholder();
    test_gpio_both_branches();
    test_drdy_two_pass();
    test_faces_good_and_swapped();
    test_faces_skipped_not_absent();
    test_gyro_sign();
    test_spin_frame_agreement();
    test_report_and_exit_codes();
    test_sim_like_no_recommendation();
    test_degauss_split();
    test_chipts_wall_bands();
    test_verdict_respects_experimental_flag();

    test_terminal_digest();
    test_abort_stops_the_run();   /* LAST: g_abort never resets (see below) */

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
