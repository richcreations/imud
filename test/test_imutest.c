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

#include <sys/socket.h>
#include <sys/un.h>

#include "imutest.h"
#include "imu_gpio.h"
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
 * branches of the caller's handling get exercised by flipping g_gpio_rate.
 *
 * check_drdy counts twice over the same window — once draining on every edge,
 * once not — so the stub answers the two passes separately.  That split IS
 * the thing under test: a level-triggered watermark interrupt yields a normal
 * count while something empties the FIFO and about one edge when nothing
 * does, and only the pair can tell that from an edge-per-sample line.
 */
/*
 * Edge RATES, in Hz, not counts.
 *
 * A real interrupt line produces edges at a rate, so a longer window yields
 * more of them. A stub returning a fixed count whatever window it was
 * given, which meant any change to how a window is chosen silently changed
 * what the tests measured -- and briefly talked me out of a correct design
 * because widening a window "broke" a test that a faithful stub would not have
 * noticed. A stub's limitations must not decide production behaviour.
 *
 * Negative still means "no chip", which is the caller's error path.
 */
static double g_gpio_rate    = -1.0;      /* <0 → report "no chip" */
static double g_gpio_rate_idle = 1.0;     /* the undrained pass */
static imt_gpio_why_t g_gpio_why = IMT_GPIO_ENOCHIP;
/*
 * How many times the stub invokes drain().  The real function calls it once per
 * edge; the stub defaults to 1 because check_drdy's cases grade the edge COUNT
 * and one of them uses 100000 edges, which is not a number to actually iterate.
 * mag.drdy.rate counts what the drain returns rather than the edges, so its
 * cases raise this.
 */
static int g_gpio_drain_calls = 1;

/* Chip addresses, up here because the GPIO stub below asserts against one. */
#define ISM_ADDR  0x6A
#define MMC_ADDR  0x30

/*
 * Set to arm a write failure from INSIDE the DRDY window.  check_mag_drdy
 * re-inits the mag, calls this, then restores -- so this stub is the one place
 * in a run where the restore can be aimed at without also hitting the several
 * earlier inits that write the same register.
 */
static int g_gpio_fail_addr_after = -1;
static int g_gpio_fail_reg_after  = -1;

/*
 * The last register written before edge counting started. The DRDY check must
 * acknowledge a latched interrupt first -- see the priming read in
 * check_mag_drdy -- and the acknowledge is a write to STATUS.
 */
static int g_gpio_last_write_at_entry = -2;

/*
 * The daemon's edge wait, stubbed for the same reason imt_gpio_count_edges is:
 * this suite runs against the mock bus with no GPIO chip, and linking libgpiod
 * would make it need one. imud-imutest itself links src/imu.c and calls the
 * real thing -- that is the point of the refactor -- so what is stubbed here is
 * only the hardware, not the logic under test.
 *
 * open() hands back a sentinel rather than NULL so drain_pace() takes the
 * interrupt-driven branch, and the wait returns at once: a mock bus always has
 * a sample ready, and making the suite sleep the daemon's real cadence between
 * drains would buy nothing but wall time. NULL would instead exercise the
 * self-paced branch and cost 10 ms per drain, which is what the windows here
 * are far too short to absorb.
 */
static int g_fake_line;

imu_gpio_line_t *imu_gpio_open(const char *chip, unsigned int offset,
                               const char *consumer)
{
    (void)chip; (void)offset; (void)consumer;
    return (imu_gpio_line_t *)&g_fake_line;
}

int imu_gpio_wait_edge(imu_gpio_line_t *line, long timeout_ms)
{
    (void)line; (void)timeout_ms;
    return 1;                    /* the mock always has data waiting */
}

void imu_gpio_close(imu_gpio_line_t *line) { (void)line; }

int imt_gpio_count_edges(const char *chip, int gpio, long window_ms,
                         void (*drain)(void *), void *user,
                         imt_gpio_why_t *why, void (*prime)(void *),
                         int odr_hz)
{
    (void)odr_hz;
    (void)chip; (void)gpio; (void)window_ms;
    *why = g_gpio_why;
    if (g_gpio_fail_reg_after >= 0)
        i2cmock_fail_write_to((uint8_t)g_gpio_fail_addr_after,
                              g_gpio_fail_reg_after, -1);
    if (prime) prime(user);
    /* Snapshot AFTER priming: the acknowledge now happens inside the window,
     * which is the whole point -- doing it beforehand left a race the line
     * could go high in. */
    g_gpio_last_write_at_entry = i2cmock_last_write(MMC_ADDR);
    /* Edges scale with the window, as a real line's do. */
    double secs = (double)window_ms / 1000.0;
    if (!drain)
        return g_gpio_rate_idle < 0 ? (int)g_gpio_rate_idle
                                    : (int)(g_gpio_rate_idle * secs + 0.5);
    if (g_gpio_rate < 0) return (int)g_gpio_rate;
    for (int i = 0; i < g_gpio_drain_calls; i++) drain(user);
    return (int)(g_gpio_rate * secs + 0.5);
}

/* ── Drivers under test ──────────────────────────────────────────────────── */

extern const imu_ops_t ism330dhcx_ops;
extern const mag_ops_t mmc5983ma_ops;

#define FD        3          /* the mock ignores it */

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

/*
 * When set, stage_sample() feeds the FIFO and leaves the direct output
 * registers alone, so a test can drive the two apart on purpose.  That is the
 * defect imu.direct.accel exists to catch: the sensor is producing data and
 * the FIFO path is decoding it into the wrong place.
 */
static bool g_stage_skew_direct;

/* Write one 16-bit little-endian value into an ISM330DHCX register pair. */
static void ism_set_le16(uint8_t reg, int16_t v)
{
    i2cmock_set_reg(ISM_ADDR, reg,             (uint8_t)((uint16_t)v & 0xFF));
    i2cmock_set_reg(ISM_ADDR, (uint8_t)(reg + 1), (uint8_t)((uint16_t)v >> 8));
}

/*
 * The ST direct output window: OUT_TEMP_L 0x20, gyro 0x22-0x27, accel
 * 0x28-0x2D, little-endian (DS13012 §9.27-9.33).  Counts are chip-frame, the
 * same as the FIFO words -- the driver's Y/Z flip is applied after both.
 */
static void ism_set_direct(int16_t ax, int16_t ay, int16_t az,
                           int16_t gx, int16_t gy, int16_t gz)
{
    ism_set_le16(0x22, gx); ism_set_le16(0x24, gy); ism_set_le16(0x26, gz);
    ism_set_le16(0x28, ax); ism_set_le16(0x2A, ay); ism_set_le16(0x2C, az);
}

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
    /*
     * The same sample in the part's DIRECT output registers, which real
     * silicon updates alongside the FIFO and which imu.direct.* reads to
     * cross-check the FIFO decode.  Staging only the FIFO would model a part
     * whose output registers are dead, and the cross-check would correctly
     * call that a fault in every test in this file.
     */
    if (!g_stage_skew_direct) ism_set_direct(ax, ay, az, gx, gy, gz);
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
    g_stage_skew_direct = false;
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
    cfg->imu_odr_mhz   = 208000;
    cfg->imu_accel_g  = 4;
    cfg->imu_gyro_dps = 500;
    cfg->imu_fifo_wm  = 8;
    cfg->mag_odr_mhz   = 100000;
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
    /* Stage nothing while this check id is collecting, so the phase reaches
     * its too-few-samples path.  NULL feeds every face. */
    const char *starve_id;
    /* Phase C: per-axis rate, deg/s, for the gyro turns. */
    double turn_rate_dps[3];
    int    cur_turn;
    /* Phase D: spin. */
    bool   spinning;
    double spin_deg;
    double spin_dir;         /* +1 normal, -1 to invert the mag sweep */
    double spin_step_deg;    /* heading advance per tick */
    double spin_off[2];      /* hard-iron offset added to the swept locus */
    /* Re-arm the mock's one-shot I2C failure on every iteration, so a whole
     * check sees errors rather than just the first read. */
    bool   inject_errors;
    /*
     * Push a TAG_TEMP word carrying this temperature into the FIFO on every
     * iteration, so the FIFO's temperature and the direct register's diverge.
     * Setting the register alone cannot do it: ism330dhcx.c seeds last_temp
     * from that same register at init(), so both sides move together and the
     * comparison agrees at whatever value was staged.
     */
    bool   push_fifo_temp;
    double fifo_temp_c;
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

    if (s->feed_imu && !(s->starve_id && strcmp(id, s->starve_id) == 0)) {
        double g[3] = { s->gyro[0], s->gyro[1], s->gyro[2] };
        /* During a gyro turn, spin the commanded axis. */
        if (strncmp(id, "gyro.", 5) == 0 && s->cur_turn >= 0) {
            g[0] = g[1] = g[2] = 0.0;
            g[s->cur_turn] = s->turn_rate_dps[s->cur_turn] * M_PI / 180.0;
        }
        stage_sample(s->accel, g);
        if (s->push_fifo_temp) {
            /* ST temperature word: 256 LSB/degC with 0 = 25 degC. */
            ism_push(0x03, (int16_t)lrint((s->fifo_temp_c - 25.0) * 256.0), 0, 0);
            uint8_t cur = i2cmock_get_reg(ISM_ADDR, 0x3A);
            i2cmock_set_reg(ISM_ADDR, 0x3A, (uint8_t)(cur + 1));
        }
    }
    if (s->feed_mag && s->spinning) {
        /* Sweep the horizontal field so the heading unwraps, at the same rate
         * the gyro is being fed — a correct driver must show them agreeing. */
        s->spin_deg += s->spin_step_deg;
        double th = s->spin_deg * M_PI / 180.0 * s->spin_dir;
        mmc_set_field(45.0 * cos(th) + s->spin_off[0],
                     -45.0 * sin(th) + s->spin_off[1], 40.0);
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
/* `hz` is Hz; the table and the result are MILLI-Hz. */
static int grid_below(const imt_report_t *r, double hz)
{
    int best = 0;
    for (int i = 0; i < 16 && r->imu_odr_tab[i]; i++)
        if ((double)r->imu_odr_tab[i] * 1e-3 <= hz && r->imu_odr_tab[i] > best)
            best = r->imu_odr_tab[i];
    return best;
}

/* `hz` is Hz; the table and the result are MILLI-Hz. */
static int grid_above(const imt_report_t *r, double hz)
{
    int best = 0;
    for (int i = 0; i < 16 && r->imu_odr_tab[i]; i++)
        if ((double)r->imu_odr_tab[i] * 1e-3 >= hz &&
            (best == 0 || r->imu_odr_tab[i] < best))
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
    /* Nothing moves in a plain register file, so the observational scan finds
     * nothing.  The 16 that ARE excluded are declared rather than observed:
     * the output window 0x20-0x2D (14) and FIFO_STATUS1/2 (2).  Neither can be
     * caught by observation -- outputs read static on a quiet platform, and
     * DIFF_FIFO saturates.  Asserting the exact count still catches a scan
     * that starts flagging registers spuriously. */
    EXPECT(imt_regmap_known_volatile("ism330dhcx", 0x3A) &&
           imt_regmap_known_volatile("ism330dhcx", 0x3B),
           "the FIFO status pair is declared volatile");
    EXPECT(r->raw.n_volatile_imu == 16,
           "static mock adds nothing beyond the 16 declared registers");
    /* 69 documented, readable registers (DS13012 Table 19 marks about 60 of
     * the 0x00-0x7F span RESERVED and the sweep does not touch them), less the
     * 16 declared volatile, leaves 53 actually compared. */
    EXPECT(r->raw.n_scanned_imu > 45, "the documented range was compared");
    EXPECT(r->raw.n_scanned_imu < 70, "and the reserved span was not");
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
/*
 * The sweep must never read a RESERVED or undefined address.
 *
 * Walking lo..hi blind, skipping only the FIFO port, would put about
 * 60 reserved addresses in every snapshot and ~420 reserved reads in every run
 * once the volatile scan and the idempotency compare are counted. That is not
 * harmless on the reference ISM330DHCX: the part reads cleanly at power-up and
 * after ONE run roughly 1 register read in 100 comes back with the wrong byte,
 * persistently and across processes. A power cycle clears it; nothing in
 * software does.
 *
 * DS13012 Table 19 marks 0x00, 03-06, 1F, 2E-34, 3C-3F, 44-55, 60-62, 64-6E
 * and 76-77 reserved -- and 60-62 are reserved READ/WRITE, with the
 * embedded-function bank behind FUNC_CFG_ACCESS at 0x01.
 */
static void test_sweep_avoids_reserved_registers(void)
{
    begin("test_sweep_avoids_reserved_registers");
    int fb = g_fail;

    mock_base();
    /* Count every read the sweep makes, by address. */
    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);
    imt_report_t *r = run(&cfg, &o);
    free(r);

    static const struct { uint8_t lo, hi; const char *why; } resv[] = {
        { 0x00, 0x00, "0x00 is not in the register map at all" },
        { 0x03, 0x06, "0x03-0x06 RESERVED" },
        { 0x1F, 0x1F, "0x1F RESERVED" },
        { 0x2E, 0x34, "0x2E-0x34 RESERVED" },
        { 0x3C, 0x3F, "0x3C-0x3F RESERVED" },
        { 0x44, 0x55, "0x44-0x55 RESERVED" },
        { 0x60, 0x62, "0x60-0x62 RESERVED (read/write)" },
        { 0x64, 0x6E, "0x64-0x6E RESERVED" },
        { 0x76, 0x77, "0x76-0x77 RESERVED" },
    };
    char msg[96];
    for (unsigned i = 0; i < sizeof resv / sizeof resv[0]; i++)
        for (int reg = resv[i].lo; reg <= (int)resv[i].hi; reg++) {
            snprintf(msg, sizeof msg, "never read 0x%02X (%s)",
                     reg, resv[i].why);
            EXPECT(i2cmock_read_count(ISM_ADDR, (uint8_t)reg) == 0, msg);
        }

    /* And the documented ones ARE still read, or the sweep measures nothing. */
    EXPECT(i2cmock_read_count(ISM_ADDR, 0x0F) > 0, "WHO_AM_I is still read");
    EXPECT(i2cmock_read_count(ISM_ADDR, 0x63) > 0, "FREQ_FINE is still read");

    end(fb);
}

/*
 * The AKM magnetometers carry read/WRITE test registers that the vendor marks
 * DO NOT ACCESS, and a regmap that sweeps straight across them:
 * hi was 0x1F on the AK8963 and 0x3F on the AK09916, while the parts document
 * registers only to 0x12 and 0x32.  A validation tool that pokes a part's
 * shipment-test registers is worse than no tool, so this pins the exclusions
 * by address rather than by trusting the range bounds.
 *
 * The DRDY entries matter for a different reason: the datasheet says the bit
 * "returns to 0 when any one of ST2 register or the measurement data
 * registers (HXL to HZH) is read", so a sweep touching any of them consumes
 * the sample the next check is waiting for.
 */
static void test_akm_sweep_avoids_forbidden_registers(void)
{
    begin("test_akm_sweep_avoids_forbidden_registers");
    int fb = g_fail;

    static const struct {
        const char *drv; uint8_t lo, hi; const char *why;
    } forbidden[] = {
        { "ak8963",  0x0D, 0x0E, "TS1/TS2 — datasheet: DO NOT ACCESS" },
        { "ak8963",  0x13, 0x1F, "0x13 RSV DO NOT ACCESS, 0x14+ not a register" },
        { "ak8963",  0x03, 0x09, "HXL..HZH and ST2 — reading them clears DRDY" },
        { "ak09916", 0x33, 0x34, "TS1/TS2 — datasheet: DO NOT ACCESS" },
        { "ak09916", 0x35, 0x3F, "past the end of the register file" },
        { "ak09916", 0x11, 0x18, "HXL..HZH and ST2 — reading them clears DRDY" },
    };
    char msg[128];
    for (unsigned i = 0; i < sizeof forbidden / sizeof forbidden[0]; i++)
        for (int reg = forbidden[i].lo; reg <= (int)forbidden[i].hi; reg++) {
            snprintf(msg, sizeof msg, "%s never reads 0x%02X (%s)",
                     forbidden[i].drv, reg, forbidden[i].why);
            EXPECT(!imt_regmap_reads(forbidden[i].drv, (uint8_t)reg), msg);
        }

    /* ...and the registers that make the snapshot worth taking still are. */
    EXPECT(imt_regmap_reads("ak8963",  0x00), "ak8963 WHO_AM_I is read");
    EXPECT(imt_regmap_reads("ak8963",  0x0A), "ak8963 CNTL1 is read");
    EXPECT(imt_regmap_reads("ak8963",  0x0B), "ak8963 CNTL2 is read");
    EXPECT(imt_regmap_reads("ak09916", 0x00), "ak09916 WIA1 is read");
    EXPECT(imt_regmap_reads("ak09916", 0x31), "ak09916 CNTL2 is read");

    end(fb);
}

static void test_volatile_registers_filtered(void)
{
    begin("test_volatile_registers_filtered");
    int fb = g_fail;

    mock_base();
    /*
     * Registers the ISM driver never touches, so making them move exercises
     * the scan without perturbing the FIFO level or the timestamp the driver
     * itself reads: STATUS_REG and two gyro output words.
     *
     * Not 0x50 and 0x51, which DS13012 Table 19 marks RESERVED.
     * Using undefined addresses as stand-ins is the same habit that put ~420
     * reserved reads into every run of the tool -- and the sweep now skips
     * them, so the test would have been asserting against addresses nothing
     * looks at.
     */
    i2cmock_set_live(ISM_ADDR, 0x1E, 3);   /* STATUS_REG */
    i2cmock_set_live(ISM_ADDR, 0x22, 7);   /* OUTX_L_G  */
    i2cmock_set_live(ISM_ADDR, 0x23, 1);   /* OUTX_H_G  */

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    /*
     * 16 are declared up front (output window 0x20-0x2D plus FIFO_STATUS1/2).
     * Of the three made live here, 0x22 and 0x23 fall INSIDE that window and
     * were already excluded; 0x1E STATUS_REG is outside it and has to be
     * caught by observation. So exactly 17 -- which asserts both that the
     * scan still detects a live register the declaration does not cover, and
     * that it does not over-match everything else.
     */
    EXPECT(imt_regmap_known_volatile("ism330dhcx", 0x22) &&
           imt_regmap_known_volatile("ism330dhcx", 0x23),
           "the live output bytes are covered by the declaration");
    EXPECT(!imt_regmap_known_volatile("ism330dhcx", 0x1E),
           "STATUS_REG is not declared, so observation must find it");
    EXPECT(r->raw.n_volatile_imu == 17,
           "16 declared + STATUS_REG found by observation, and nothing else");
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
    EXPECT(r->raw.n_scanned_imu + r->raw.n_volatile_imu > 60,
           "excluded plus compared covers the documented range");

    free(r);
    end(fb);
}

/*
 * A non-idempotent init() must be reported by NAME, not by count.
 *
 * The check spent three bench sessions reporting "2 registers differ" at
 * 208/1660/6664 Hz, which nobody could act on: the count says a defect exists
 * and withholds every fact needed to find it.  The registers now go into
 * raw.idem_imu the same way the reset->init diff does, and into the note.
 *
 * Driven with a wrapper init() that is deliberately state-dependent — it lands
 * on a different register the second time it is called — because the mock is a
 * plain register file and the real driver, once fixed, is idempotent over it.
 * The register chosen (0x14, CTRL5_C) is one no ST driver here writes, so
 * nothing else in the run disturbs it, and it is stable between the volatile
 * scan's reads, so it must survive that filter rather than be excluded by it.
 */
static int g_nonidem_calls;

static int init_nonidem(const imud_bus_t *bus, const imu_cfg_t *cfg)
{
    int rc = ism330dhcx_ops.init(bus, cfg);
    if (rc == 0)
        i2cmock_set_reg(ISM_ADDR, 0x14, g_nonidem_calls++ ? 0x5A : 0x00);
    return rc;
}

static void test_nonidempotent_init_names_registers(void)
{
    begin("test_nonidempotent_init_names_registers");
    int fb = g_fail;

    mock_base();
    g_nonidem_calls = 0;

    imu_ops_t fake = ism330dhcx_ops;
    fake.init = init_nonidem;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = calloc(1, sizeof *r);
    char err[256] = "";
    int rc = imt_run_ops(I2CBUS(ISM_ADDR), I2CBUS(MMC_ADDR),
                         &fake, &mmc5983ma_ops, &cfg, &o, r, err, sizeof err);
    if (rc < 0) fprintf(stderr, "  imt_run_ops: %s\n", err);

    EXPECT(status_of(r, "imu.init.idempotent") == IMT_WARN,
           "a state-dependent init() warns");
    EXPECT(r->raw.n_idem_imu == 1, "exactly one register recorded");
    if (r->raw.n_idem_imu == 1) {
        EXPECT(r->raw.idem_imu[0].reg    == 0x14, "the differing register is named");
        EXPECT(r->raw.idem_imu[0].before == 0x00, "image after one init recorded");
        EXPECT(r->raw.idem_imu[0].after  == 0x5A, "image after two inits recorded");
    }
    /* The note has to carry it too — the appendix is a page away, and a
     * reader scanning the check list is the one who needs the lead. */
    EXPECT(note_contains(r, "imu.init.idempotent", "0x14"),
           "the note names the register");

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
    /* 16, not zero: the declared output window plus the FIFO status pair. None
     * of them is in the port window this test is about (0x78-0x7E), so the
     * claim it makes is unchanged -- nothing there was read or flagged. */
    EXPECT(r->raw.n_volatile_imu == 16,
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
    cfg.mag_odr_mhz = 1000;                  /* on the MMC grid; loop far outruns it */
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    script_reset(&o);

    imt_report_t *r = run(&cfg, &o);

    EXPECT(r->mag_eff_odr_mhz == 1000, "1 Hz is on the MMC grid");
    EXPECT(r->raw.mag_n >= 5, "enough samples to grade the rate at all");
    EXPECT(r->raw.mag_rate_hz > r->mag_eff_odr_mhz * 1e-3 * (1.0 + o.odr_tol_warn),
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

    cfg.imu_odr_mhz = slow;
    mock_base();
    script_reset(&o);
    r = run(&cfg, &o);

    double imu_err = fabs(r->raw.odr_measured_hz - r->eff_odr_mhz * 1e-3)
                   / (r->eff_odr_mhz * 1e-3);
    free(r);

    /* Far above the configured rate: past odr_tol_fail, so a FAIL. */
    mock_base();
    script_reset(&o);
    r = run(&cfg, &o);
    EXPECT(r->raw.odr_measured_hz > r->eff_odr_mhz * 1e-3 * (1.0 + o.odr_tol_warn),
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

    EXPECT(fabs(r->raw.odr_measured_hz - r->eff_odr_mhz * 1e-3) / (r->eff_odr_mhz * 1e-3)
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

    cfg.imu_odr_mhz = fast;
    mock_base();
    script_reset(&o);
    probe = run(&cfg, &o);
    EXPECT(probe->raw.odr_measured_hz < probe->eff_odr_mhz * 1e-3 * (1.0 - o.odr_tol_warn),
           "the loop really does fall short of the rate picked for it");
    double err = fabs(probe->raw.odr_measured_hz - probe->eff_odr_mhz * 1e-3)
                 / (probe->eff_odr_mhz * 1e-3);
    free(probe);

    /* Widen the fail bound past the shortfall: a low reading in the warn band
     * must stay a WARN, where an equally-sized overshoot became a FAIL above. */
    mock_base();
    script_reset(&o);
    o.odr_tol_fail = err * 2.0;
    imt_report_t *r = run(&cfg, &o);

    EXPECT(r->raw.odr_measured_hz < r->eff_odr_mhz * 1e-3,
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
 * imu.direct.* — the FIFO measured against the part's own output registers.
 *
 * The mock stages every sample into both, as silicon does, so the agreeing
 * case is the default.  The cases that matter are the disagreements, and the
 * one worth naming is the second: gravity on Z in the direct registers and on
 * X in the FIFO is literally the framing defect fixed in 689133e, which no
 * other check in this file can see — every one of them reads the FIFO, and the
 * FIFO's numbers are perfectly plausible.
 */
static void test_direct_vs_fifo(void)
{
    begin("test_direct_vs_fifo");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;

    mock_base();
    script_reset(&o);
    imt_report_t *r = run(&cfg, &o);

    EXPECT(status_of(r, "imu.direct.accel") == IMT_PASS,
           "FIFO and direct registers agree when both carry the same sample");
    EXPECT(r->raw.direct_n > 0, "the comparison recorded how many reads it made");
    EXPECT(r->raw.direct_angle_deg >= 0.0 && r->raw.direct_angle_deg < 5.0,
           "...and the recorded angle is small");
    /* Non-degeneracy: a real gravity vector on both sides, not two zeroes
     * agreeing with themselves. */
    EXPECT(fabs(r->raw.direct_accel[2]) > 100.0,
           "the direct side holds a real staged gravity vector, in counts");
    EXPECT(fabs(r->raw.fifo_accel[2]) > 5.0,
           "the FIFO side holds a real gravity vector, in m/s^2");
    EXPECT(status_of(r, "imu.direct.temp") == IMT_PASS,
           "the FIFO temperature tracks the direct temperature register");
    free(r);

    /*
     * The framing defect: the FIFO says gravity is on X, the registers say Z.
     * Staged by freezing the direct window while the FIFO is fed a rotated
     * vector, which is what a decode landing an axis in the wrong slot looks
     * like from outside.
     */
    mock_base();
    script_reset(&o);
    g_stage_skew_direct = true;
    ism_set_direct(0, 0, (int16_t)lrint(9.80665 / ISM_ACCEL_LSB), 0, 0, 0);
    g_s.accel[0] = -9.80665; g_s.accel[1] = 0; g_s.accel[2] = 0;
    imt_report_t *sk = run(&cfg, &o);

    EXPECT(status_of(sk, "imu.direct.accel") == IMT_FAIL,
           "gravity on Z in the registers and on X in the FIFO FAILs");
    EXPECT(sk->raw.direct_angle_deg > 45.0,
           "...and the reported angle shows how far apart they are");
    EXPECT(note_contains(sk, "imu.direct.accel", "FIFO framing"),
           "the note sends the reader to the FIFO decode, not to the part");
    free(sk);

    /* Output registers that never update at all: a distinct fault, and one
     * the angle comparison cannot express. */
    mock_base();
    script_reset(&o);
    g_stage_skew_direct = true;
    ism_set_direct(0, 0, 0, 0, 0, 0);
    imt_report_t *dead = run(&cfg, &o);
    EXPECT(status_of(dead, "imu.direct.accel") == IMT_FAIL,
           "direct registers stuck at zero FAIL rather than comparing");
    EXPECT(note_contains(dead, "imu.direct.accel", "not being updated"),
           "...and say so, rather than reporting a direction");
    free(dead);

    /*
     * Temperature is the framing canary: one byte of offset moves it tens of
     * degrees.  The FIFO is fed 65 C while register 0x20 is left at its 25 C
     * default -- the register cannot be moved instead, because the driver
     * seeds its running temperature FROM that register at init(), so both
     * sides would shift together and agree.
     */
    mock_base();
    script_reset(&o);
    g_s.push_fifo_temp = true;
    g_s.fifo_temp_c    = 65.0;
    imt_report_t *hot = run(&cfg, &o);
    EXPECT(status_of(hot, "imu.direct.temp") == IMT_FAIL,
           "a FIFO temperature far from the direct register's FAILs");
    EXPECT(fabs(hot->raw.direct_temp_c - 25.0) < 1.0 &&
           hot->raw.fifo_temp_c > 60.0,
           "...and the appendix records both sides, not just the verdict");
    free(hot);

    end(fb);
}

/*
 * The direct-window table itself, for every registered driver.
 *
 * This is the half no mock run can reach: the bus mock stands in for the
 * ISM330DHCX and the MMC5983MA, and a wrong window on an MPU or an ICM would
 * be found by whoever owns that board, on their bench, by reading reserved
 * space on it.  imt_regmap_direct() refuses an unsafe window, so requiring it
 * to succeed pins the bounds check as well as the declaration.
 */
static void test_direct_window_table(void)
{
    begin("test_direct_window_table");
    int fb = g_fail;

    /* Every IMU driver that has a FIFO to cross-check should declare one. */
    static const struct { const char *drv; uint8_t base, len; bool fifo_temp; }
    want[] = {
        { "ism330dhcx", 0x20, 14, true  },
        { "lsm6dso",    0x20, 14, true  },
        { "lsm6dsox",   0x20, 14, true  },
        { "icm42688p",  0x1D, 14, true  },
        /* icm20948.c and mpu925x.c read temperature from the direct register
         * for every sample rather than from the FIFO, so comparing the two
         * would compare a register against itself. */
        { "icm20948",   0x2D, 14, false },
        { "mpu9250",    0x3B, 14, false },
        { "mpu9255",    0x3B, 14, false },
    };

    for (size_t i = 0; i < sizeof want / sizeof want[0]; i++) {
        uint8_t base = 0, len = 0;
        bool ft = false;
        char msg[128];
        bool have = imt_regmap_direct(want[i].drv, &base, &len, &ft);
        snprintf(msg, sizeof msg, "%s declares a safe direct window",
                 want[i].drv);
        EXPECT(have, msg);
        snprintf(msg, sizeof msg, "%s direct window starts at 0x%02X",
                 want[i].drv, want[i].base);
        EXPECT(have && base == want[i].base, msg);
        snprintf(msg, sizeof msg, "%s direct window is %u bytes",
                 want[i].drv, want[i].len);
        EXPECT(have && len == want[i].len, msg);
        snprintf(msg, sizeof msg, "%s fifo_temp matches what its driver does",
                 want[i].drv);
        EXPECT(have && ft == want[i].fifo_temp, msg);
    }

    /*
     * The declared window must lie inside a range the map already calls
     * volatile: both describe the same output registers, and a window that
     * escaped that range would be pointing at control registers.
     */
    for (size_t i = 0; i < sizeof want / sizeof want[0]; i++) {
        uint8_t base = 0, len = 0;
        char msg[128];
        if (!imt_regmap_direct(want[i].drv, &base, &len, NULL)) continue;
        bool all_vol = true;
        for (int reg = base; reg < base + len; reg++)
            if (!imt_regmap_known_volatile(want[i].drv, (uint8_t)reg))
                all_vol = false;
        snprintf(msg, sizeof msg,
                 "%s direct window lies inside its known-volatile range",
                 want[i].drv);
        EXPECT(all_vol, msg);
    }

    /* Magnetometers have no FIFO to cross-check, and must not declare one. */
    static const char *no_window[] = { "mmc5983ma", "lis3mdl", "lis2mdl",
                                       "ak8963", "ak09916", "rm3100" };
    for (size_t i = 0; i < sizeof no_window / sizeof no_window[0]; i++) {
        char msg[128];
        snprintf(msg, sizeof msg, "%s declares no direct window", no_window[i]);
        EXPECT(!imt_regmap_direct(no_window[i], NULL, NULL, NULL), msg);
    }

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
    /* 208 is NOT on the grid any more: the ST ladder is the divider chain, so
     * the rung is 208.25 Hz and a request for 208 rounds up to it. */
    EXPECT(r->eff_odr_mhz == 208250, "208 resolves to the 208.25 Hz rung");

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
     * Fed one argument for two conversions, the second
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
    g_gpio_rate = -1.0;
    g_gpio_why   = IMT_GPIO_EBUSY;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_SKIP, "EBUSY -> SKIP not FAIL");
    free(r);

    /* Zero edges on a requested line is a real defect. */
    mock_base(); script_reset(&o);
    g_gpio_rate = 0.0;
    g_gpio_why   = IMT_GPIO_OK;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_FAIL, "0 edges -> FAIL");
    free(r);

    g_gpio_rate = -1.0;
    g_gpio_why   = IMT_GPIO_ENOCHIP;
    end(fb);
}

/*
 * mag.drdy.rate — the mag rate measured the way the DAEMON gets it.
 *
 * Every other mag check in the tool polls.  The daemon does not; it blocks on
 * the mag interrupt.  That difference can be a factor of three on
 * real silicon (105.4 Hz polled, 35 Hz in the daemon) and no check here could
 * see it, because none of them measured the production path.
 *
 * What the mock CAN decide is the branch structure, which is what these cases
 * pin: no interrupt configured and an unavailable line must SKIP rather than
 * blame the driver, and edges arriving with nothing readable behind them must
 * FAIL.  The rate COMPARISON itself grades two live measurements of the same
 * part against each other, so it is verified on the bench, not here.
 */
static void test_mag_drdy_rate(void)
{
    begin("test_mag_drdy_rate");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;
    /* fast_opts uses a 0.05 s window to keep the suite quick. That is far too
     * short to resolve a rate tolerance, and the check now says so rather than
     * grading it -- so ask for a window a real run would use. The GPIO stub
     * ignores the duration, so this costs no wall time. */
    o.drdy_window_s = 0.6;   /* >= 40 expected samples: see imt_rate_quantum */

    /* No mag interrupt: the reader polls, so there is nothing to measure. */
    mock_base(); script_reset(&o);
    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "mag.drdy.rate") == IMT_SKIP,
           "mag.int_gpio 0 -> SKIP");
    EXPECT(r->raw.mag_drdy_edges < 0, "sentinel says the check did not run");
    free(r);

    /* Line held by someone else — SKIP, never FAIL: that would blame the
     * driver for the daemon holding the line. */
    mock_base(); script_reset(&o);
    cfg.mag_int_gpio  = 27;
    g_gpio_rate      = -1.0;
    g_gpio_why        = IMT_GPIO_EBUSY;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "mag.drdy.rate") == IMT_SKIP, "EBUSY -> SKIP not FAIL");
    free(r);

    /*
     * Edges arrive, but every read behind them comes back empty.  This is the
     * shape of the defect the check exists for: the interrupt is working and
     * the daemon still gets nothing.
     */
    mock_base(); script_reset(&o);
    g_gpio_rate       = 13.3;
    g_gpio_why         = IMT_GPIO_OK;
    g_gpio_drain_calls = 4;
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);      /* M_DONE clear */
    r = run(&cfg, &o);
    EXPECT(status_of(r, "mag.drdy.rate") != IMT_SKIP,
           "with a line and edges the check runs");
    EXPECT(r->raw.mag_drdy_edges == 8, "edge count recorded");
    free(r);

    /*
     * The part must be left POLLED. check_mag_drdy re-inits the driver
     * interrupt-driven to measure the line, and the mode is a driver global —
     * so a run that failed to put it back would leave every later mag check
     * reading through a path that skips the status gate. Observable from here:
     * with M_DONE clear, a polled read says "no data" (1); an interrupt-driven
     * one burst-reads and hands back the stale conversion (0).
     */
    mock_base(); script_reset(&o);
    g_gpio_rate       = 13.3;
    g_gpio_why         = IMT_GPIO_OK;
    g_gpio_drain_calls = 4;
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);      /* M_DONE clear */
    r = run(&cfg, &o);
    free(r);

    /*
     * Stage FRESH output registers with M_DONE still clear. That is the one
     * state the two modes answer differently: polled consults the status bit
     * and reports no data (1), interrupt-driven skips it, sees the registers
     * have advanced, and hands the sample back (0). Leaving the registers
     * unchanged would NOT do — the staleness guard also returns 1 then, and
     * the assertion would pass in either mode.
     */
    uint8_t fresh[7] = { 0xAA, 0x55, 0xBB, 0x66, 0xCC, 0x77, 0x00 };
    i2cmock_set_regs(MMC_ADDR, 0x00, fresh, 7);
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x00);      /* M_DONE still clear */

    mag_sample_t ms;
    memset(&ms, 0, sizeof ms);
    EXPECT(mmc5983ma_ops.read(I2CBUS(MMC_ADDR), &ms) == 1,
           "run leaves the magnetometer in polled mode");

    /*
     * The interrupt must be ACKNOWLEDGED before edges are counted. This part's
     * INT is latched and re-armed only by the write clearing Meas_M_Done, so
     * its resting state is high and a rising-edge wait on an unprimed line
     * never fires -- 0 edges in 3 s against a part feeding the daemon 105 Hz,
     * measured on the bench. The acknowledge is a write to STATUS (0x08), so
     * that must be the last register written when counting begins.
     */
    EXPECT(g_gpio_last_write_at_entry == 0x08,
           "the DRDY check acknowledges the interrupt inside the edge window");

    /*
     * The restore FAILING is its own verdict, and it is the one line in the
     * report that says "distrust everything below me" -- so it has to be
     * reachable, not merely written. Arm a sticky failure on the first
     * register mmc_init writes, from inside the DRDY window, so the restore
     * cannot succeed.
     */
    mock_base(); script_reset(&o);
    g_gpio_rate           = 13.3;
    g_gpio_why             = IMT_GPIO_OK;
    g_gpio_drain_calls     = 4;
    g_gpio_fail_addr_after = MMC_ADDR;
    g_gpio_fail_reg_after   = 0x09;             /* MMC CTRL0 */
    r = run(&cfg, &o);
    EXPECT(status_of(r, "mag.drdy.restore") == IMT_FAIL,
           "a failed restore is reported, not swallowed");
    free(r);
    g_gpio_fail_reg_after = -1;
    i2cmock_fail_write_to(MMC_ADDR, -1, 0);

    /*
     * A window too short to resolve the tolerance SKIPs rather than grading.
     * One drain per edge over a handful of edges is a +/-1/N measurement, and
     * at 1 Hz over 3 s that is +/-33% being judged against +/-5% — which
     * FAILed a working part on the bench.
     */
    /*
     * The resolution rule itself. A run cannot reach the case that matters --
     * the mock answers every poll, so it lands hundreds of percent out -- and
     * the distinction is the whole point: a miss no bigger than one sample is
     * a rounding boundary, a gross one is a defect however few were expected.
     */
    EXPECT(imt_rate_quantum(1.0, 5.0) == 0.4,
           "1 Hz over 5 s: two samples in five, 40% — a window that can\n            hold 5, 6 or 7 of them");
    EXPECT(imt_rate_quantum(100.0, 3.0) < 0.01,
           "100 Hz over 3 s still resolves far finer than the tolerance");
    EXPECT(imt_rate_quantum(0.0, 3.0) == 1.0,
           "a zero nominal cannot resolve anything");

    g_gpio_rate       = -1.0;
    g_gpio_why         = IMT_GPIO_ENOCHIP;
    g_gpio_drain_calls = 1;
    cfg.mag_int_gpio   = 0;
    end(fb);
}

/*
 * The two-pass DRDY count.
 *
 * The reference part's edge rate was uncharacterised:
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
    g_gpio_rate      = 240.0;
    g_gpio_rate_idle = 20.0;
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
    g_gpio_rate      = 240.0;
    g_gpio_rate_idle = 240.0;
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
    g_gpio_rate      = 2000000.0;
    g_gpio_rate_idle = 20.0;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_WARN,
           "a rate above the sample rate still warns");
    free(r);

    g_gpio_rate      = -1.0;
    g_gpio_rate_idle = 20.0;
    g_gpio_why        = IMT_GPIO_ENOCHIP;
    end(fb);
}

/*
 * A face verdict is a dominant-axis decision on a mean vector, so "sign wrong"
 * on its own says nothing about why. Print what was actually measured — the
 * sample count especially, since a collection window that accumulated nothing
 * leaves the mean at (0,0,0) and makes dominant_axis() arbitrary.
 */
/*
 * The sign check is only reached for a face with >= 10 samples; below that the
 * phase skips it. Assert the budget actually delivered them, so a too-tight
 * face_collect_s fails on its own terms instead of showing up as a wrong
 * verdict somewhere else.
 */
static void expect_faces_sampled(const imt_report_t *r)
{
    int thin = 0;
    for (int i = 0; i < r->raw.n_faces; i++)
        if (r->raw.face[i].n < 10) thin++;
    EXPECT(thin == 0, "every face collected the 10 samples the sign check needs");
    /*
     * The mechanism that starves the later faces if the mock's queue lets
     * head and tail only advance, so FIFOSZ was a lifetime budget and the six
     * faces shared 292 staged samples between them. Faster machines spent it
     * sooner, which is why it read as a timing flake. Pin it directly.
     */
    EXPECT(i2cmock_fifo_drops(ISM_ADDR) == 0,
           "the mock queue dropped no staged samples");
}

static void dump_faces_on_failure(const imt_report_t *r, int fail_before)
{
    if (g_fail == fail_before) return;
    for (int i = 0; i < r->raw.n_faces; i++) {
        const imt_face_row_t *f = &r->raw.face[i];
        printf("      face %d  n=%-5d a = [%8.4f %8.4f %8.4f]  |a|=%7.4f  "
               "want axis %d sign %+d, got axis %d sign %+d\n",
               f->idx + 1, f->n, f->a[0], f->a[1], f->a[2], f->norm,
               f->exp_axis, f->exp_sign, f->got_axis, f->got_sign);
    }
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
    expect_faces_sampled(r);
    dump_faces_on_failure(r, fb);

    /*
     * The rendered table has to carry the sample count, not just the mean
     * vector and its magnitude.  Ten samples and ten thousand produce the
     * same three numbers, so the count is the only thing in the report that
     * says a face was measured on almost no data.
     */
    {
        const char *path = "test_imutest_faces.md";
        char err[256] = "";
        static char buf[262144];
        EXPECT(imt_write_md(r, path, err, sizeof err) == 0, "the report writes");
        FILE *f = fopen(path, "r");
        EXPECT(f != NULL, "the report file exists");
        if (f) {
            buf[fread(buf, 1, sizeof buf - 1, f)] = '\0';
            fclose(f);
            char *sec = strstr(buf, "### 5.8 Six-face orientation");
            EXPECT(sec != NULL, "the six-face appendix is present");
            if (sec) {
                /* Bound the search to 5.8 itself — a bare "| 41 |" would
                 * otherwise match a count printed by any later table. */
                char *end = strstr(sec, "\nDerived accel calibration");
                if (end) *end = '\0';
                EXPECT(strstr(sec, "| Face | Expected | Measured (m/s^2) | "
                                   "\\|a\\| | Samples | Verdict |") != NULL,
                       "5.8 heads a sample count column");
                char want[64];
                EXPECT(r->raw.face[0].n > 0, "face 1 did collect samples");
                snprintf(want, sizeof want, "| %d | ", r->raw.face[0].n);
                EXPECT(strstr(sec, want) != NULL,
                       "and face 1 prints the count it recorded");
            }
        }
        remove(path);
    }
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
    expect_faces_sampled(r);
    dump_faces_on_failure(r, fb);
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
    expect_faces_sampled(r);
    dump_faces_on_failure(r, fb);
    free(r);

    end(fb);
}

/*
 * A remap permutes and negates axes, so it cannot change |a|.  When the faces
 * come back at a magnitude that is not gravity -- a board that was moving, or
 * never actually held against anything -- the dominant axis is not a rotation
 * of g and carries no information about the remap.  imud-imutest must not
 * report that case as "the chip-to-board axis remap is wrong", on four faces
 * reading about 0.1 m/s^2, which is a confident instruction to go rewrite code
 * that is correct.  It must skip instead, and it must not fit a calibration
 * model through those readings either.
 *
 * The magnitudes here are far enough out to trip grav_tol_fail while keeping a
 * clean dominant axis, so the ONLY thing suppressing the verdict is the
 * magnitude gate -- not an ambiguous axis.
 */
static void test_faces_bad_magnitude_does_not_blame_the_remap(void)
{
    begin("test_faces_bad_magnitude_does_not_blame_the_remap");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_FACES;

    /* Correct NED directions, but a tenth of gravity: nothing was held. */
    const double faint[6][3] = {
        {  0,  0, -0.10 }, {  0,  0,  0.10 },
        { -0.10, 0, 0   }, {  0.10, 0, 0   },
        {  0, -0.10, 0  }, {  0,  0.10, 0  },
    };
    mock_base(); script_reset(&o);
    memcpy(g_s.face_vec, faint, sizeof faint);
    g_s.use_face_vecs = true;

    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "face.1.mag") == IMT_FAIL,
           "the magnitude check itself still fails — the problem is reported");
    EXPECT(status_of(r, "face.1.sign") == IMT_SKIP,
           "the axis verdict is skipped, not failed");
    EXPECT(status_of(r, "faces.frame") == IMT_SKIP,
           "the frame rollup is skipped, not failed");
    EXPECT(!note_contains(r, "faces.frame", "remap is wrong"),
           "no remap is blamed when |a| rules a remap out");
    EXPECT(status_of(r, "faces.symmetry") == IMT_SKIP,
           "no offset/scale is fitted through faces that never saw gravity");
    expect_faces_sampled(r);
    dump_faces_on_failure(r, fb);
    free(r);

    /*
     * The case that actually happened, and the one the gate exists for: faint
     * AND with the axes landing wherever the noise put them.  Without the gate
     * this is the report that says "the chip-to-board axis remap is wrong" --
     * about a board that simply was not held against anything.
     */
    const double faint_scrambled[6][3] = {
        {  0.10, 0,  0    }, {  0, -0.10, 0    },
        {  0,  0,   0.10  }, {  0,  0.10, 0    },
        { -0.10, 0,  0    }, {  0,  0,  -0.10  },
    };
    mock_base(); script_reset(&o);
    memcpy(g_s.face_vec, faint_scrambled, sizeof faint_scrambled);
    g_s.use_face_vecs = true;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "faces.frame") == IMT_SKIP,
           "scrambled faint axes skip rather than blaming the remap");
    EXPECT(!note_contains(r, "faces.frame", "remap is wrong"),
           "the impossible diagnosis is not printed");
    EXPECT(!note_contains(r, "face.1.sign", "swapped"),
           "no per-face swap is diagnosed from a vector that is not gravity");
    expect_faces_sampled(r);
    dump_faces_on_failure(r, fb);
    free(r);

    /*
     * The gate must not swallow a real defect: same wrong axes as
     * test_faces_good_and_swapped, at full gravity, still FAIL.
     */
    const double good[6][3] = {
        {  0,  0, -9.80665 }, {  0,  0,  9.80665 },
        { -9.80665, 0, 0 },   {  9.80665, 0, 0 },
        {  0, -9.80665, 0 },  {  0,  9.80665, 0 },
    };
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
    EXPECT(status_of(r, "faces.frame") == IMT_FAIL,
           "a genuine swap at full gravity still fails");
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

/*
 * A face that collected almost nothing is the one the sample count exists to
 * expose, so it must reach the table — and it must not count toward a rollup
 * that speaks for all six faces.  Both are the same conflation: n_faces is the
 * number of rows, not the number of faces judged.
 */
static void test_faces_thin_face(void)
{
    begin("test_faces_thin_face");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_FACES;

    /* The last face: its row is the one a row count that stops at five drops. */
    mock_base(); script_reset(&o);
    g_s.starve_id = "face.6";

    imt_report_t *r = run(&cfg, &o);
    EXPECT(r->raw.n_faces == 6, "the starved face still gets a row");
    EXPECT(r->raw.face[5].n < 10, "and that row records how little it collected");
    EXPECT(status_of(r, "face.6.sign") == IMT_SKIP, "its sign check is SKIP");
    EXPECT(status_of(r, "faces.frame") == IMT_SKIP,
           "the frame rollup does not speak for a face it never judged");

    {
        const char *path = "test_imutest_thin.md";
        char err[256] = "";
        static char buf[262144];
        EXPECT(imt_write_md(r, path, err, sizeof err) == 0, "the report writes");
        FILE *f = fopen(path, "r");
        EXPECT(f != NULL, "the report file exists");
        if (f) {
            buf[fread(buf, 1, sizeof buf - 1, f)] = '\0';
            fclose(f);
            char *sec = strstr(buf, "### 5.8 Six-face orientation");
            EXPECT(sec != NULL, "the six-face appendix is present");
            if (sec) {
                char *end = strstr(sec, "\nDerived accel calibration");
                if (end) *end = '\0';
                EXPECT(strstr(sec, "| 6. Port side down |") != NULL,
                       "the starved face is a row in 5.8, not an absence");
            }
        }
        remove(path);
    }
    free(r);

    /* The same conflation, one face earlier: five good faces and a starved
     * first one must not read as six confirmed. */
    mock_base(); script_reset(&o);
    g_s.starve_id = "face.1";

    r = run(&cfg, &o);
    EXPECT(r->raw.n_faces == 6, "six rows either way");
    EXPECT(status_of(r, "faces.frame") == IMT_SKIP,
           "a starved first face skips the rollup too");
    EXPECT(status_of(r, "faces.symmetry") != IMT_INFO,
           "and no offset/scale is fitted through a face that was not measured");
    free(r);

    end(fb);
}

/*
 * imu.direct.gyro — telling a silent gyro from a mis-decoded FIFO.
 *
 * The three sign checks cannot make this distinction: both causes integrate to
 * nothing and both are reported as "the board did not move or the gyro is not
 * responding". The direct registers sit upstream of the FIFO, so they settle
 * it. The second case here is the shape reported in issue #31 — a gyro whose
 * instantaneous figures look healthy while a commanded 90-degree turn
 * integrates to a fraction of a degree.
 */
static void test_direct_gyro_vs_fifo(void)
{
    begin("test_direct_gyro_vs_fifo");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases   = IMT_PHASE_GYRO;
    o.turn_deg = 90.0;

    /* A healthy part: the FIFO and the registers both see the turn. */
    mock_base(); script_reset(&o);
    g_s.done_after = 40;
    for (int k = 0; k < 3; k++) g_s.turn_rate_dps[k] = 400.0;
    imt_report_t *ok = run(&cfg, &o);
    EXPECT(status_of(ok, "imu.direct.gyro") == IMT_PASS,
           "a turn both sides can see PASSes");
    EXPECT(ok->raw.direct_gyro_peak[0] > 0.0,
           "the direct gyro peak reaches the appendix");
    free(ok);

    /*
     * The FIFO decode loses the turn while the registers show it. This is the
     * case the sign checks report as a dead gyro and that this one has to
     * contradict, naming the FIFO path instead.
     */
    mock_base(); script_reset(&o);
    g_s.done_after = 40;
    for (int k = 0; k < 3; k++) g_s.turn_rate_dps[k] = 0.0;
    g_stage_skew_direct = true;
    ism_set_direct(0, 0, (int16_t)lrint(9.80665 / ISM_ACCEL_LSB), 5000, 5000, 5000);
    imt_report_t *lost = run(&cfg, &o);
    EXPECT(status_of(lost, "imu.direct.gyro") == IMT_FAIL,
           "registers turning while the FIFO integrates nothing FAILs");
    EXPECT(note_contains(lost, "imu.direct.gyro", "gyro IS responding"),
           "...and the note contradicts the dead-gyro reading of the same run");
    EXPECT(note_contains(lost, "imu.direct.gyro", "FIFO path"),
           "...and sends the reader to the FIFO decode");
    /* The sign checks, on the same run, say the opposite — which is exactly
     * why this check had to exist. */
    EXPECT(status_of(lost, "gyro.x.sign") == IMT_FAIL,
           "the sign check still fails, reading it as a dead gyro");
    free(lost);

    /* Nothing anywhere: the data path really is silent, upstream of the FIFO. */
    mock_base(); script_reset(&o);
    g_s.done_after = 40;
    for (int k = 0; k < 3; k++) g_s.turn_rate_dps[k] = 0.0;
    g_stage_skew_direct = true;
    ism_set_direct(0, 0, (int16_t)lrint(9.80665 / ISM_ACCEL_LSB), 0, 0, 0);
    imt_report_t *dead = run(&cfg, &o);
    EXPECT(status_of(dead, "imu.direct.gyro") == IMT_FAIL,
           "neither side seeing the turn FAILs");
    EXPECT(note_contains(dead, "imu.direct.gyro", "not producing data"),
           "...and names the gyro data path rather than the decode");
    free(dead);

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

    /*
     * A gyro that produces nothing through the commanded turn.  The sign of an
     * integral near zero is whichever way the noise fell, so grading the sign
     * alone scored three PASSes on a dead data path — and these three checks
     * are in the set that clears a driver's `experimental` flag, so the run
     * recommended clearing it.  A shipped report did exactly that at +0.1 deg
     * against a commanded +90.
     */
    mock_base(); script_reset(&o);
    g_s.done_after = 40;
    for (int k = 0; k < 3; k++) g_s.turn_rate_dps[k] = 0.5;

    r = run(&cfg, &o);
    EXPECT(r->raw.n_turns == 3, "every turn still recorded a row");
    EXPECT(fabs(r->raw.turn[0].theta[0]) < 0.25 * o.turn_deg,
           "the X integral really is below the floor");
    EXPECT(status_of(r, "gyro.x.sign") == IMT_FAIL, "a gyro that did not move fails");
    EXPECT(status_of(r, "gyro.y.sign") == IMT_FAIL, "on every axis");
    EXPECT(status_of(r, "gyro.z.sign") == IMT_FAIL, "on every axis");
    EXPECT(r->raw.turn[0].status == IMT_FAIL, "the appendix row carries the verdict");
    EXPECT(note_contains(r, "gyro.x.sign", "not responding"),
           "diagnosis names a gyro that is not responding");
    EXPECT(!note_contains(r, "gyro.x.sign", "inverted"),
           "and does not misdiagnose it as an inverted sign");

    /*
     * The rendered table has to carry the sample count and the interval, not
     * just the integral.  A driver that delivered nothing and one that
     * delivered samples reading zero both integrate to ~0 deg, so the count is
     * the only thing in the report that tells the two apart — and this run is
     * the second of them.
     */
    {
        const char *path = "test_imutest_turns.md";
        char err[256] = "";
        static char buf[262144];
        EXPECT(imt_write_md(r, path, err, sizeof err) == 0, "the report writes");
        FILE *f = fopen(path, "r");
        EXPECT(f != NULL, "the report file exists");
        if (f) {
            buf[fread(buf, 1, sizeof buf - 1, f)] = '\0';
            fclose(f);
            const char *sec = strstr(buf, "### 5.9 Gyro rotation");
            EXPECT(sec != NULL, "the rotation appendix is present");
            if (sec) {
                EXPECT(strstr(sec, "| Axis | Commanded | thetaX | thetaY | "
                                   "thetaZ | Samples | Duration | dt source | "
                                   "Verdict |") != NULL,
                       "5.9 heads a sample count and a duration column");
                char want[64];
                EXPECT(r->raw.turn[0].n > 0, "the X turn did deliver samples");
                snprintf(want, sizeof want, "| %d | ", r->raw.turn[0].n);
                EXPECT(strstr(sec, want) != NULL,
                       "and the X row prints the count it recorded");
                snprintf(want, sizeof want, "| %.1f s |",
                         r->raw.turn[0].dur_s);
                EXPECT(strstr(sec, want) != NULL,
                       "beside the interval it integrated over");
            }
        }
        remove(path);
    }
    free(r);

    /*
     * Either side of the floor, pinned against the integral the run actually
     * recorded rather than against the scripted rate: X below it fails, Y above
     * it is graded on its sign as before.
     */
    mock_base(); script_reset(&o);
    g_s.done_after = 40;
    g_s.turn_rate_dps[0] = 100.0;
    g_s.turn_rate_dps[1] = 160.0;
    g_s.turn_rate_dps[2] = 400.0;

    r = run(&cfg, &o);
    EXPECT(r->raw.turn[0].theta[0] < 0.25 * o.turn_deg,
           "X landed below the floor");
    EXPECT(r->raw.turn[1].theta[1] > 0.25 * o.turn_deg,
           "Y landed above it");
    EXPECT(status_of(r, "gyro.x.sign") == IMT_FAIL, "below the floor fails");
    EXPECT(status_of(r, "gyro.y.sign") == IMT_PASS,
           "a turn over the floor is still graded on its sign");
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

/*
 * Hard iron moves the swept locus off the origin.  Once the offset exceeds the
 * field radius the locus no longer encloses the origin, and a heading taken
 * about the ORIGIN stops winding: it oscillates through a limited arc and the
 * total comes out near zero however far the board turned.  The check must not
 * read that near-zero total as a negative ratio and report "an X or Y sign is
 * inverted in the magnetometer's remap" -- a confident diagnosis of a defect
 * that was not there.  It happened twice on the reference rig, at offsets of
 * 49 and 50 uT against field radii of 22 and 18, with a correct driver.
 *
 * Taking the heading about the CENTRE of the locus, as the coverage map has
 * always done, makes it wind again.  Offset 80 against radius 45 here, so the
 * origin is well outside the circle, which an uncentred heading cannot pass.
 */
static void test_spin_hard_iron_still_tracks(void)
{
    begin("test_spin_hard_iron_still_tracks");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_SPIN;

    const double rate_dps     = 400.0;
    const double deg_per_tick = rate_dps / 208.0;
    const double gyro_z       = rate_dps * M_PI / 180.0;

    mock_base();
    mmc_set_field(45.0 + 80.0, 0.0, 40.0);
    script_reset(&o);
    g_s.feed_mag      = true;
    g_s.spin_dir      = +1.0;
    g_s.done_after    = 220;
    g_s.gyro[2]       = gyro_z;
    g_s.spin_step_deg = deg_per_tick;
    g_s.spin_off[0]   = 80.0;      /* offset > radius: origin is outside */

    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "spin.frame_agreement") == IMT_PASS,
           "a correct driver still agrees when the locus is offset");
    EXPECT(!note_contains(r, "spin.frame_agreement", "OPPOSITE"),
           "hard iron is never reported as an inverted axis");
    free(r);

    /*
     * And the gate must not blind the check: the same offset with the sweep
     * inverted is still the defect this phase exists to catch.
     */
    mock_base();
    mmc_set_field(45.0 + 80.0, 0.0, 40.0);
    script_reset(&o);
    g_s.feed_mag      = true;
    g_s.spin_dir      = -1.0;
    g_s.done_after    = 220;
    g_s.gyro[2]       = gyro_z;
    g_s.spin_step_deg = deg_per_tick;
    g_s.spin_off[0]   = 80.0;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "spin.frame_agreement") == IMT_FAIL,
           "an inverted sweep still fails even with hard iron present");
    EXPECT(note_contains(r, "spin.frame_agreement", "OPPOSITE"),
           "and is still diagnosed as the opposite-direction signature");
    free(r);

    /*
     * A heading that barely moves while the gyro turns a long way is not
     * evidence of anything -- its sign is whichever way the residual wobble
     * fell.  That must SKIP and say so, never FAIL as an inverted axis, which
     * is what produced the false diagnosis on the bench.
     */
    mock_base();
    mmc_set_field(45.0, 0.0, 40.0);
    script_reset(&o);
    g_s.feed_mag      = true;
    g_s.spin_dir      = +1.0;
    g_s.done_after    = 220;
    g_s.gyro[2]       = gyro_z;
    g_s.spin_step_deg = 0.05;      /* mag creeps; the gyro turns a long way */
    g_s.spin_off[0]   = 0.0;

    r = run(&cfg, &o);
    EXPECT(status_of(r, "spin.frame_agreement") == IMT_SKIP,
           "a heading that did not track the turn skips rather than failing");
    EXPECT(!note_contains(r, "spin.frame_agreement", "OPPOSITE"),
           "and no inversion is claimed from evidence that cannot show one");
    free(r);

    end(fb);
}

/*
 * The report has to name the transport each part was actually opened on.  It
 * printed `[device] i2c_bus` unconditionally, so a SPI run -- which never
 * opens that node -- produced a report attributing the run to I2C.  That is
 * the artifact a reviewer reads to clear a driver's `experimental` flag, and
 * a driver validated on SPI is not thereby validated on I2C.
 */
static void test_report_names_the_transport(void)
{
    begin("test_report_names_the_transport");

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;

    const char *path = "test_imutest_bus_report.md";
    char err[256] = "";
    static char buf[262144];

    /* I2C: the default node, named as I2C. */
    mock_base();
    script_reset(&o);
    imt_report_t *i2c = run(&cfg, &o);
    EXPECT(imt_write_md(i2c, path, err, sizeof err) == 0, "I2C report writes");
    FILE *f = fopen(path, "r");
    EXPECT(f != NULL, "I2C report exists");
    if (f) {
        buf[fread(buf, 1, sizeof buf - 1, f)] = '\0';
        fclose(f);
        EXPECT(strstr(buf, "| IMU bus | I2C `/dev/i2c-1` |") != NULL,
               "an I2C run names the I2C bus");
        EXPECT(strstr(buf, "| magnetometer bus | I2C `/dev/i2c-1` |") != NULL,
               "and names it for the magnetometer too");
    }
    free(i2c);

    /* SPI: the spidev nodes, named as SPI, and the I2C default nowhere. */
    cfg.imu_bus_kind = BUS_SPI;
    cfg.mag_bus_kind = BUS_SPI;
    snprintf(cfg.imu_spi_dev, sizeof cfg.imu_spi_dev, "/dev/spidev0.0");
    snprintf(cfg.mag_spi_dev, sizeof cfg.mag_spi_dev, "/dev/spidev0.1");

    mock_base();
    script_reset(&o);
    imt_report_t *spi = run_spi(&cfg, &o);
    EXPECT(imt_write_md(spi, path, err, sizeof err) == 0, "SPI report writes");
    f = fopen(path, "r");
    EXPECT(f != NULL, "SPI report exists");
    if (f) {
        buf[fread(buf, 1, sizeof buf - 1, f)] = '\0';
        fclose(f);
        EXPECT(strstr(buf, "| IMU bus | SPI `/dev/spidev0.0` |") != NULL,
               "a SPI run names the IMU's spidev node");
        EXPECT(strstr(buf, "| magnetometer bus | SPI `/dev/spidev0.1` |") != NULL,
               "and the magnetometer's, which is a different chip select");
        /* The defect exactly: the I2C default appearing in a SPI report. */
        EXPECT(strstr(buf, "/dev/i2c-") == NULL,
               "a SPI run names no I2C device anywhere in the report");
    }
    free(spi);
    remove(path);
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
         * Appendix numbering must not skip.  Emitting 5.6 only when
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
 * running fast is absorbed, while time that has gone missing cannot be.  A
 * ratio of 1.041 — a few percent fast — is expected silicon behaviour and
 * must not warn.
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

    /* The case the check exists for, and the bench numbers it was
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

/*
 * chip_ts accounting. Driven directly rather than through the fake part,
 * because the mock reaches the counter only through a real driver and cannot
 * stage the three cases that matter as a chosen sequence.
 */
static void test_chipts_accounting(void)
{
    begin("test_chipts_accounting");
    int fb = g_fail;

    /* An ordinary run: each stamp one sample period after the last. */
    imt_ts_acc_t a = { 0 };
    EXPECT(imt_ts_acc_step(&a, 1000) == 0, "the first stamp yields no delta");
    EXPECT(imt_ts_acc_step(&a, 1192) == 192, "the second yields the period");
    EXPECT(imt_ts_acc_step(&a, 1384) == 192, "and so does the third");
    EXPECT(a.reversals == 0 && a.repeats == 0 && a.zeros == 0,
           "a clean run scores no faults");
    EXPECT(a.first == 1000 && a.last == 1384, "both endpoints are tracked");

    /*
     * A zero stamp is what ism330dhcx.c and lsm6dso.c leave across a whole
     * burst when the post-drain timestamp read fails. Comparing it would
     * score a reversal going in and then hand the next real sample a delta of
     * most of the counter coming out — which imu.chipts.rate takes the median
     * of, so one failed register read moved two checks.
     */
    /*
     * A reversal at a burst seam is not the same fault as one inside a burst.
     * Inside, every sample comes from one anchor and time can only move
     * forwards. At a seam the anchor is re-derived, and a timer-paced drain
     * can place the new burst before the old one ended -- the daemon, woken by
     * the watermark, scored 0 in 53,708 samples where the polled check scored
     * several per window on the same part.
     */
    imt_ts_acc_t sm = { 0 };
    imt_ts_acc_step(&sm, 1000);
    imt_ts_acc_step(&sm, 1192);
    imt_ts_acc_seam(&sm);                 /* next sample opens a new read */
    imt_ts_acc_step(&sm, 900);            /* lands before the previous burst */
    EXPECT(sm.reversals == 1, "the seam reversal is counted");
    EXPECT(sm.seam_reversals == 1, "and attributed to the seam");

    imt_ts_acc_t in = { 0 };
    imt_ts_acc_step(&in, 1000);
    imt_ts_acc_seam(&in);
    imt_ts_acc_step(&in, 1192);           /* the seam sample itself is fine */
    imt_ts_acc_step(&in, 900);            /* backwards INSIDE the burst */
    EXPECT(in.reversals == 1, "the in-burst reversal is counted");
    EXPECT(in.seam_reversals == 0,
           "and is NOT excused as a seam — only the first sample after a "
           "seam can be one");

    imt_ts_acc_t z = { 0 };
    imt_ts_acc_step(&z, 1000);
    EXPECT(imt_ts_acc_step(&z, 0) == 0, "a zero stamp yields no delta");
    EXPECT(z.zeros == 1, "the zero is counted");
    EXPECT(z.reversals == 0 && z.repeats == 0, "but is neither fault");
    EXPECT(imt_ts_acc_step(&z, 1192) == 192,
           "the sample after a zero measures from the last real stamp");

    /* A zero before anything else must not become the window's start. */
    imt_ts_acc_t z0 = { 0 };
    EXPECT(imt_ts_acc_step(&z0, 0) == 0, "a leading zero yields no delta");
    EXPECT(!z0.have, "and does not open the window");
    EXPECT(imt_ts_acc_step(&z0, 500) == 0, "the first real stamp opens it");
    EXPECT(z0.first == 500, "the window starts at the first real stamp");

    /* A repeat is one reading stamped across a burst, not a reversal. */
    imt_ts_acc_t rp = { 0 };
    imt_ts_acc_step(&rp, 1000);
    EXPECT(imt_ts_acc_step(&rp, 1000) == 0, "a repeated tick yields no delta");
    EXPECT(rp.repeats == 1 && rp.reversals == 0,
           "an identical tick is a repeat, not a reversal");

    /* A genuine reversal. */
    imt_ts_acc_t rv = { 0 };
    imt_ts_acc_step(&rv, 1000);
    EXPECT(imt_ts_acc_step(&rv, 900) == 0, "a reversal yields no delta");
    EXPECT(rv.reversals == 1 && rv.repeats == 0, "and is counted as a reversal");

    /*
     * A 32-bit wrap is a forward step. This is what the modular difference
     * buys: 0xFFFFFF80 + 192 is 0x40, so the delta reads as the period and not
     * as a jump back across the whole counter.
     */
    imt_ts_acc_t w = { 0 };
    imt_ts_acc_step(&w, 0xFFFFFF80u);
    EXPECT(imt_ts_acc_step(&w, 0x00000040u) == 192, "a wrap reads forward");
    EXPECT(w.wraps == 1, "and is counted as a wrap");
    EXPECT(w.reversals == 0 && w.repeats == 0, "not as a fault");

    end(fb);
}

/*
 * imu.chipts.rate must divide by the rate the part is actually running at, not
 * the one it was asked for. The check exists to compare chip against chip; if
 * one side is the configured rate then a part whose oscillator is fast reports
 * a TICK error for what is really an ODR error, which imu.odr already owns.
 *
 * Pinned as an invariant rather than by staging a fast part, because the rate
 * the mock achieves is set by how quickly the poll loop runs and is not ours
 * to dictate: whatever it turns out to be, the implied tick must be consistent
 * with THAT and with the measured median delta. Recomputing from eff_odr_mhz
 * breaks it, since the two are never equal in a real run.
 */
/*
 * mag.rate must describe the DIRECTION it measured. The grade and the wording
 * were driven by one boolean meaning "above nominal AND outside tolerance", so
 * a reading above nominal but inside the band fell through to the low branch:
 * the bench read 21.0 Hz against a configured 20 and reported "5.0% low",
 * which points a reader at the poll loop when the part is running fast.
 */
/*
 * The watermark first asserts at fifo_wm/odr seconds. When that is longer than
 * the window, no edge is possible and grading the absence is grading an
 * impossibility -- at 12 Hz with the shipped fifo_wm = 64 that is 5.3 s
 * against a 3 s window, and the check reported "the line is not wired" about a
 * working line.
 */
static void test_drdy_window_must_allow_an_edge(void)
{
    begin("test_drdy_window_must_allow_an_edge");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases        = IMT_PHASE_PASSIVE;
    o.drdy_window_s = 0.6;   /* >= 40 expected samples: see imt_rate_quantum */
    cfg.imu_int_gpio = 17;

    /* 64 sample-sets at 12 Hz needs 5.3 s; the window is 0.3 s. */
    mock_base(); script_reset(&o);
    cfg.imu_odr_mhz = 12000;
    cfg.imu_fifo_wm = 64;
    g_gpio_rate = 0.0;
    imt_report_t *r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") == IMT_SKIP,
           "a window the watermark cannot fill in skips rather than failing");
    free(r);

    /* At 833 Hz the same watermark fills in 77 ms, well inside the window, so
     * an absence of edges really is a defect. */
    mock_base(); script_reset(&o);
    cfg.imu_odr_mhz = 833000;
    g_gpio_rate = 0.0;
    r = run(&cfg, &o);
    EXPECT(status_of(r, "imu.drdy.edges") != IMT_SKIP,
           "where the watermark can fill, no edges is still graded");
    free(r);

    cfg.imu_int_gpio = 0;
    end(fb);
}

static void test_mag_rate_names_the_direction(void)
{
    begin("test_mag_rate_names_the_direction");
    int fb = g_fail;

    /*
     * The classification directly, because this is the case a run cannot
     * reach: the mock answers every poll, so its measured rate lands hundreds
     * of percent high and only ever exercises dir = +1.
     */
    EXPECT(imt_rate_dir(19.0, 20.0, 0.05) == -1, "below nominal is low");
    EXPECT(imt_rate_dir(20.0, 20.0, 0.05) == -1, "exactly nominal is not high");
    EXPECT(imt_rate_dir(21.0, 20.0, 0.05) ==  0,
           "5% high is HIGH, in band — the case that read as \"5.0% low\"");
    EXPECT(imt_rate_dir(20.9, 20.0, 0.05) ==  0, "just inside the band is high");
    EXPECT(imt_rate_dir(21.1, 20.0, 0.05) ==  1, "past the band is high and out");
    EXPECT(imt_rate_dir(1204.0, 1000.0, 0.05) == 1, "the MMC at 1000 Hz is out");

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;

    mock_base(); script_reset(&o);
    imt_report_t *r = run(&cfg, &o);

    /* The mock answers every poll, so the measured rate lands above the
     * configured one -- the case the wording most easily gets backwards. */
    if (r->raw.mag_rate_hz > (double)r->mag_eff_odr_mhz * 1e-3) {
        EXPECT(!note_contains(r, "mag.rate", "low"),
               "a rate above nominal is never described as low");
        /* "ABOVE" out of band, "above" inside it: the emphasis carries the
         * severity, so accept either and pin only the direction. */
        EXPECT(note_contains(r, "mag.rate", "ABOVE")
               || note_contains(r, "mag.rate", "above"),
               "and is described as above");
    } else {
        EXPECT(note_contains(r, "mag.rate", "low"),
               "a rate at or below nominal keeps the low wording");
    }
    free(r);
    end(fb);
}

/*
 * The tick estimate must see within-burst deltas only.
 *
 * A burst's first sample carries the gap to the PREVIOUS burst, across which
 * the driver re-derived its anchor from a post-drain timestamp read. That
 * delta is the drain cadence, not the sample period, and letting it in is what
 * put imu.chipts.rate into WARN at 26, 52 and 104 Hz on a healthy part: below
 * ~208 Hz a 10 ms drain returns fewer than two samples, so almost every delta
 * was a seam and the median landed on a multiple of the pacing.
 *
 * Bursts here are deliberately lopsided — a 100-tick sample period against a
 * 5000-tick gap — so an included seam could not hide inside the median.
 */
/*
 * A full-scale sweep is graded against the median of its own rows.
 *
 * Against the median rather than the preceding row: sigma across a gyro's
 * ranges is expected to be flat, so one step inflated by a knock on the bench
 * makes the NEXT step read as a halving and puts the WARN on the innocent
 * row.
 */
static void fs_set(imt_fs_row_t *r, int fs, double sigma)
{
    r->fs = fs;
    for (int k = 0; k < 3; k++) r->sigma[k] = sigma;
    r->status = IMT_INFO;
}

/*
 * imu.fifo.overflow's decision table.
 *
 * The loop itself is not unit-tested: it sleeps its way up to ~9 s looking for
 * the fill, which would dominate this suite's runtime. What changed is the
 * DECISION, and that is here. Bench coverage for the loop is the ODR ladder on
 * the reference part, where 12 and 26 Hz are the rates that cannot fill in the
 * window.
 */
/*
 * imu.bus.integrity's decision table.
 *
 * The loop itself is exercised on the PASS path by every mock run; what is
 * worth pinning is the grading, and specifically that it has NO tolerance
 * band.  A tolerance band would let a real defect read as an
 * acceptable rate: on the reference rig the check sat at a few tenths of a
 * percent and was waved past for a whole bench session.  There is no rate low
 * enough to be fine, because the value compared against is hard-wired and the
 * driver reads it once in probe() -- 0.2% is one probe in five hundred
 * rejecting a part that is physically present.
 */
/*
 * The register imu.bus.integrity compares against must be an INVARIANT.
 *
 * A factory trim is not invariant, so INTERNAL_FREQ_FINE (0x63) cannot be
 * the hammered byte.  Measured on the reference
 * ISM330DHCX, that register reads 0x1B while the part runs and 0x1A with the
 * sensors powered down -- it reports the trim of an oscillator that can be
 * switched off.  The check scored the difference as bus corruption and failed
 * a part whose every other check passed.
 *
 * WHO_AM_I is hard-wired and identical in every power mode, which is the
 * property the check actually requires.  Pinned per driver against the values
 * in each driver's own header, because a wrong entry here is indistinguishable
 * from a corrupt bus in the field.
 */
/*
 * A saturated counter is volatile and unprovably so.
 *
 * reg_volatile_scan() marks a register volatile when it MOVES across several
 * passes.  FIFO_STATUS1/2 on the ST parts carry DIFF_FIFO, and at a high ODR
 * the FIFO refills to capacity between passes, so the counter reads its
 * maximum every time and the scan calls it static.  init() then flushes the
 * FIFO and imu.init.idempotent reports "2 registers differ" -- a question
 * about whether the FIFO was emptied, dressed as a question about init().
 * That false positive cost a bench investigation.
 *
 * Pinned as a declaration rather than as observed behaviour, because the
 * defect is precisely that the behaviour cannot be observed: a mock returning
 * a constant is indistinguishable from a saturated counter, so a test that
 * watched the scan would pass either way.
 */
/*
 * The daemon-conflict guard must check BOTH sockets.
 *
 * Probing only the configured one is not enough: a bench config naming a private
 * path -- socket = "/tmp/imud-bench.sock" -- therefore connected to nothing,
 * concluded no daemon was running, and allowed the run, while the installed
 * daemon sat on the default path draining the same FIFO. Every measurement
 * taken that day was contended and had to be thrown away, and the guard's own
 * comment described precisely the failure it had just permitted.
 *
 * Sockets live under /tmp deliberately: virtiofs refuses operations on a socket
 * in the mounted repo.
 */
static int listen_at(const char *path)
{
    unlink(path);
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un a;
    memset(&a, 0, sizeof a);
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", path);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0 || listen(fd, 4) < 0) {
        close(fd); unlink(path); return -1;
    }
    return fd;
}

static void test_daemon_conflict_checks_both_sockets(void)
{
    begin("test_daemon_conflict_checks_both_sockets");
    int fb = g_fail;

    const char *cfgpath = "/tmp/imt_conflict_cfg.sock";
    const char *dfltpath = "/tmp/imt_conflict_dflt.sock";
    unlink(cfgpath); unlink(dfltpath);

    /* Nothing listening anywhere: no conflict. */
    EXPECT(!imt_daemon_conflict(cfgpath, dfltpath),
           "no listener on either path is not a conflict");

    /* A daemon on the CONFIGURED socket -- the case that always worked. */
    int a = listen_at(cfgpath);
    EXPECT(a >= 0, "bound the configured socket");
    if (a >= 0) {
        EXPECT(imt_daemon_conflict(cfgpath, dfltpath),
               "a daemon on the configured socket is a conflict");
        close(a); unlink(cfgpath);
    }

    /* THE REGRESSION: a daemon on the DEFAULT socket while the config names a
     * private path. This is what silently voided a day of measurements. */
    int b = listen_at(dfltpath);
    EXPECT(b >= 0, "bound the default socket");
    if (b >= 0) {
        EXPECT(imt_daemon_conflict(cfgpath, dfltpath),
               "a daemon on the DEFAULT socket is a conflict even when the "
               "config names a private path");
        close(b); unlink(dfltpath);
    }

    /* Degenerate inputs must not crash or report a phantom daemon. */
    EXPECT(!imt_daemon_conflict(NULL, NULL), "NULL paths are not a conflict");
    EXPECT(!imt_daemon_conflict("", ""), "empty paths are not a conflict");
    EXPECT(!imt_daemon_conflict(cfgpath, cfgpath),
           "identical paths with nothing listening are not a conflict");

    end(fb);
}

static void test_saturating_counters_are_declared_volatile(void)
{
    begin("test_saturating_counters_are_declared_volatile");
    int fb = g_fail;

    char msg[112];

    /* ST: FIFO status (saturating counter) AND the output window. A 70-cell
     * bench matrix produced four idempotency WARNs, at 0x23, 0x25 and 0x27 --
     * OUTX/Y/Z_H_G, every one a gyro output high byte, none a control
     * register. On a quiet platform those sit at 0x00 through every probing
     * pass, then a dock rock between the two init()s flips one. */
    static const char *st[] = { "ism330dhcx", "lsm6dso", "lsm6dsox" };
    for (unsigned i = 0; i < sizeof st / sizeof st[0]; i++) {
        snprintf(msg, sizeof msg, "%s declares FIFO_STATUS1/2 volatile", st[i]);
        EXPECT(imt_regmap_known_volatile(st[i], 0x3A) &&
               imt_regmap_known_volatile(st[i], 0x3B), msg);
        snprintf(msg, sizeof msg, "%s declares the gyro output bytes volatile", st[i]);
        EXPECT(imt_regmap_known_volatile(st[i], 0x23) &&
               imt_regmap_known_volatile(st[i], 0x25) &&
               imt_regmap_known_volatile(st[i], 0x27), msg);
        snprintf(msg, sizeof msg, "%s covers the whole output window 0x20-0x2D", st[i]);
        EXPECT(imt_regmap_known_volatile(st[i], 0x20) &&
               imt_regmap_known_volatile(st[i], 0x2D), msg);
    }

    /*
     * The same class on every other part, each window taken from that part's
     * datasheet rather than assumed. A window that is too WIDE silently
     * suppresses real idempotency findings, which is worse than a false WARN,
     * so the upper and lower bounds are both pinned.
     */
    static const struct { const char *drv; uint8_t lo, hi, below, above; } win[] = {
        { "icm42688p", 0x1D, 0x2C, 0x1C, 0x2D },  /* TEMP_DATA1..TMST_FSYNCL   */
        { "icm20948",  0x2D, 0x3A, 0x2C, 0x3B },  /* ACCEL_XOUT_H..TEMP_OUT_L  */
        { "mpu9250",   0x3B, 0x48, 0x3A, 0x49 },  /* ACCEL_XOUT_H..GYRO_ZOUT_L */
        { "mpu9255",   0x3B, 0x48, 0x3A, 0x49 },
        { "lis3mdl",   0x28, 0x2D, 0x27, 0x2E },  /* OUT_X_L..OUT_Z_H          */
        { "lis2mdl",   0x68, 0x6D, 0x67, 0x6E },  /* OUTX_L..OUTZ_H            */
    };
    for (unsigned i = 0; i < sizeof win / sizeof win[0]; i++) {
        snprintf(msg, sizeof msg, "%s declares 0x%02X volatile",
                 win[i].drv, win[i].lo);
        EXPECT(imt_regmap_known_volatile(win[i].drv, win[i].lo), msg);
        snprintf(msg, sizeof msg, "%s declares 0x%02X volatile",
                 win[i].drv, win[i].hi);
        EXPECT(imt_regmap_known_volatile(win[i].drv, win[i].hi), msg);
        snprintf(msg, sizeof msg, "%s does not over-reach below 0x%02X",
                 win[i].drv, win[i].lo);
        EXPECT(!imt_regmap_known_volatile(win[i].drv, win[i].below), msg);
        snprintf(msg, sizeof msg, "%s does not over-reach above 0x%02X",
                 win[i].drv, win[i].hi);
        EXPECT(!imt_regmap_known_volatile(win[i].drv, win[i].above), msg);
    }

    /* Not a blanket exclusion: control registers must still be compared, or
     * the idempotency check stops checking anything at all. */
    for (unsigned i = 0; i < sizeof st / sizeof st[0]; i++) {
        snprintf(msg, sizeof msg, "%s still compares CTRL1_XL", st[i]);
        EXPECT(!imt_regmap_known_volatile(st[i], 0x10), msg);
        snprintf(msg, sizeof msg, "%s still compares WHO_AM_I", st[i]);
        EXPECT(!imt_regmap_known_volatile(st[i], 0x0F), msg);
    }

    end(fb);
}

static void test_bus_integrity_uses_an_invariant(void)
{
    begin("test_bus_integrity_uses_an_invariant");
    int fb = g_fail;

    static const struct { const char *drv; uint8_t reg, val; } id[] = {
        { "ism330dhcx", 0x0F, 0x6B },
        { "lsm6dso",    0x0F, 0x6C },
        { "lsm6dsox",   0x0F, 0x6D },
        { "icm42688p",  0x75, 0x47 },
        { "mpu9250",    0x75, 0x71 },
        { "mpu9255",    0x75, 0x73 },
    };
    char msg[96];
    for (unsigned i = 0; i < sizeof id / sizeof id[0]; i++) {
        uint8_t reg = 0, val = 0;
        bool have = imt_regmap_identity(id[i].drv, &reg, &val);
        snprintf(msg, sizeof msg, "%s has an identity register", id[i].drv);
        EXPECT(have, msg);
        snprintf(msg, sizeof msg, "%s identity is 0x%02X", id[i].drv, id[i].reg);
        EXPECT(have && reg == id[i].reg, msg);
        snprintf(msg, sizeof msg, "%s identity reads 0x%02X", id[i].drv, id[i].val);
        EXPECT(have && val == id[i].val, msg);

        /* Never grade against a trim register. */
        snprintf(msg, sizeof msg, "%s does not compare against 0x63", id[i].drv);
        EXPECT(!have || reg != 0x63, msg);
    }

    /* icm20948's WHO_AM_I is bank-0 register 0x00, which collides with the
     * "none" sentinel, so it must fall back to probe() rather than compare
     * against register zero. */
    EXPECT(!imt_regmap_identity("icm20948", NULL, NULL),
           "icm20948 falls back to probe() rather than reading register 0");

    end(fb);
}

/*
 * imt_field_status grades whether a magnetometer is WORKING, not whether it is
 * CALIBRATED.
 *
 * Demanding Earth's 25-65 uT would grade the installation instead: hard iron
 * and the AMR bridge's own offset move |B| by tens of uT, and a clean bench
 * rig reads ~67 uT raw with the sensor as far from metal as it can get.
 *
 * What a DRIVER can get wrong moves |B| by a factor -- 16-bit data read as
 * 18-bit is 4x, a saturated bridge pins at full scale, a decode fault is
 * arbitrary. The band has to be wide enough to pass any real install and still
 * catch those.
 */
static void test_field_status_grades_the_driver_not_the_install(void)
{
    begin("test_field_status_grades_the_driver_not_the_install");
    int fb = g_fail;

    /* Earth's field, anywhere. */
    EXPECT(imt_field_status(25.0) == IMT_PASS, "25 uT (equatorial) passes");
    EXPECT(imt_field_status(48.5) == IMT_PASS, "48.5 uT (mid-latitude) passes");
    EXPECT(imt_field_status(65.0) == IMT_PASS, "65 uT (polar) passes");

    /* A real uncalibrated install. 67 uT is what the reference rig reads. */
    EXPECT(imt_field_status(67.3) == IMT_PASS,
           "67.3 uT — the reference rig, uncalibrated — must not be a finding");
    EXPECT(imt_field_status(90.0) == IMT_PASS, "heavy hard iron still passes");
    EXPECT(imt_field_status(23.0) == IMT_PASS,
           "an offset opposing the field still passes");

    /* What the check exists for: errors by a FACTOR. */
    EXPECT(imt_field_status(48.5 * 4.0) != IMT_PASS,
           "4x — 16-bit data decoded as 18-bit — is caught");
    EXPECT(imt_field_status(48.5 / 4.0) != IMT_PASS, "1/4x is caught");
    EXPECT(imt_field_status(800000.0) == IMT_FAIL, "a saturated bridge fails");
    EXPECT(imt_field_status(0.5) == IMT_FAIL, "a near-zero reading fails");

    /* Degenerate inputs must not pass by accident. */
    EXPECT(imt_field_status(0.0) == IMT_FAIL, "zero fails");
    EXPECT(imt_field_status(-48.0) == IMT_FAIL, "a negative magnitude fails");
    EXPECT(imt_field_status(NAN) == IMT_FAIL, "NaN fails");
    EXPECT(imt_field_status(INFINITY) == IMT_FAIL, "infinity fails");

    end(fb);
}

static void test_bus_integrity_status(void)
{
    begin("test_bus_integrity_status");
    int fb = g_fail;

    EXPECT(imt_bus_integrity_status(0, 2000) == IMT_PASS,
           "a bus that never misreads passes");
    EXPECT(imt_bus_integrity_status(1, 2000) == IMT_FAIL,
           "one bad read in 2000 is a failure, not a warning");
    EXPECT(imt_bus_integrity_status(9, 2000) == IMT_FAIL,
           "0.45% fails — there is no tolerance band");
    EXPECT(imt_bus_integrity_status(10, 2000) == IMT_FAIL,
           "0.5% and up cannot carry a measurement");
    EXPECT(imt_bus_integrity_status(2000, 2000) == IMT_FAIL,
           "a bus that always misreads fails");
    /* No input may grade WARN any more. */
    for (int b = 0; b <= 2000; b += 137)
        EXPECT(imt_bus_integrity_status(b, 2000) != IMT_WARN,
               "the check never returns WARN at any rate");
    /* Degenerate inputs must not divide by zero or grade a run that never
     * happened. */
    EXPECT(imt_bus_integrity_status(0, 0) == IMT_PASS, "no reads is not a fault");
    EXPECT(imt_bus_integrity_status(-1, 2000) == IMT_PASS,
           "a negative count is not a fault");

    end(fb);
}

static void test_overflow_status(void)
{
    begin("test_overflow_status");
    int fb = g_fail;

    EXPECT(imt_overflow_status(1, false) == IMT_PASS,
           "rc 1 is the overflow being reported, which is the contract");
    EXPECT(imt_overflow_status(1, true) == IMT_PASS,
           "and rc 1 stands however the fill was going");
    EXPECT(imt_overflow_status(-1, false) == IMT_FAIL,
           "a bus error is a failure, not a missing overflow");
    EXPECT(imt_overflow_status(-1, true) == IMT_FAIL,
           "likewise while still filling");

    /* Graded apart, not as one answer. */
    EXPECT(imt_overflow_status(0, false) == IMT_WARN,
           "at capacity and still rc 0: the driver is not surfacing the bit");
    EXPECT(imt_overflow_status(0, true) == IMT_SKIP,
           "still filling: the window was short, which is not the part's "
           "fault and must not be graded as one");

    end(fb);
}

/*
 * imu.fs.gyro's precondition: does sigma actually track full scale?
 *
 * Decided by monotonicity rather than a spread statistic, which is a
 * coin flip. Comparing CV(sigma) with CV(sigma/fs) let ONE degenerate step
 * open the gate -- a near-zero sigma inflates CV(sigma) -- and that same row
 * then sat below half the median and WARNed. One bad measurement both unlocked
 * the door and set off the alarm, which is why the check fired on some bench
 * sweeps and not others with nothing about the part changing.
 */
static void test_fs_scales_with_range(void)
{
    begin("test_fs_scales_with_range");
    int fb = g_fail;

    imt_fs_row_t rows[6];

    /* Quantisation-dominated: sigma tracks full scale across the span. */
    for (int i = 0; i < 6; i++) fs_set(&rows[i], 125 << i, 1.0e-4 * (1 << i));
    EXPECT(imt_fs_scales_with_range(rows, 6),
           "sigma spanning the full-scale range is the quantisation model");

    /* Analogue-dominated: the real bench numbers, 104.125 Hz on the reference
     * ISM330DHCX. Half-medians 0.0060 vs 0.0040 against a bar of 2.83. */
    fs_set(&rows[0],  125, 0.0061406);
    fs_set(&rows[1],  250, 0.0028145);
    fs_set(&rows[2],  500, 0.0060096);
    fs_set(&rows[3], 1000, 0.0053275);
    fs_set(&rows[4], 2000, 0.0039081);
    fs_set(&rows[5], 4000, 0.0039607);
    EXPECT(!imt_fs_scales_with_range(rows, 6),
           "the measured bench sweep is not graded");

    /*
     * THE FIRST REGRESSION. One degenerate step among otherwise flat rows must
     * not open the gate. The CV test returned true here, and the median test
     * then flagged the very row that had opened it.
     */
    for (int i = 0; i < 6; i++) fs_set(&rows[i], 125 << i, 1.9e-3);
    fs_set(&rows[2], 500, 1.0e-6);          /* a failed measurement */
    EXPECT(!imt_fs_scales_with_range(rows, 6),
           "one degenerate step does not unlock grading");

    /*
     * THE SECOND REGRESSION. Flat sigma that happens to rise at 4 of 5 steps --
     * which noise does about 19% of the time with six ranges. The rise-counting
     * gate opened here; proportionality does not, because the span is nowhere
     * near the full-scale span.
     */
    fs_set(&rows[0],  125, 1.90e-3);
    fs_set(&rows[1],  250, 1.95e-3);
    fs_set(&rows[2],  500, 2.00e-3);
    fs_set(&rows[3], 1000, 1.98e-3);        /* the one dip */
    fs_set(&rows[4], 2000, 2.05e-3);
    fs_set(&rows[5], 4000, 2.10e-3);
    EXPECT(!imt_fs_scales_with_range(rows, 6),
           "monotonic but flat is still analogue noise, not quantisation");

    /* Too few rows to split into halves. */
    fs_set(&rows[0], 125, 1.0e-4);
    fs_set(&rows[1], 250, 2.0e-4);
    fs_set(&rows[2], 500, 4.0e-4);
    EXPECT(!imt_fs_scales_with_range(rows, 3), "three rows decide nothing");

    /* Unmeasured rows are skipped, not read as a floor of zero. */
    for (int i = 0; i < 6; i++) fs_set(&rows[i], 125 << i, 1.0e-4 * (1 << i));
    fs_set(&rows[5], 4000, 0.0);
    EXPECT(imt_fs_scales_with_range(rows, 6),
           "a row with no measurement is skipped, not counted against");

    end(fb);
}

static void test_fs_grade_median(void)
{
    begin("test_fs_grade_median");
    int fb = g_fail;

    imt_fs_row_t rows[5];

    /* Flat sweep with ONE step inflated by a bump: nothing is wrong with the
     * part, and nothing may be graded. The pairwise form flagged row 3 here. */
    fs_set(&rows[0], 125,  1.9e-3);
    fs_set(&rows[1], 250,  1.9e-3);
    fs_set(&rows[2], 500,  9.0e-3);      /* the knock */
    fs_set(&rows[3], 1000, 1.9e-3);
    fs_set(&rows[4], 2000, 1.9e-3);
    EXPECT(imt_fs_grade_median(rows, 5, 0.5) == 0,
           "one inflated step does not condemn its neighbour");
    EXPECT(rows[3].status == IMT_INFO, "the innocent row stays ungraded");

    /* A range whose sensitivity constant is wrong reads low against ALL the
     * others, which is the defect the check exists for. */
    fs_set(&rows[0], 125,  1.9e-3);
    fs_set(&rows[1], 250,  1.9e-3);
    fs_set(&rows[2], 500,  0.2e-3);      /* the broken branch */
    fs_set(&rows[3], 1000, 1.9e-3);
    fs_set(&rows[4], 2000, 1.9e-3);
    EXPECT(imt_fs_grade_median(rows, 5, 0.5) == 1, "the low range is caught");
    EXPECT(rows[2].status == IMT_WARN, "and it is the one marked");
    EXPECT(rows[1].status == IMT_INFO && rows[3].status == IMT_INFO,
           "its neighbours are not");

    /* A flat sweep is clean. */
    for (int i = 0; i < 5; i++) fs_set(&rows[i], 125 << i, 1.9e-3);
    EXPECT(imt_fs_grade_median(rows, 5, 0.5) == 0, "a flat sweep grades nothing");

    /* Too few rows for a median: grade nothing rather than guess. */
    fs_set(&rows[0], 125, 1.9e-3);
    fs_set(&rows[1], 250, 0.1e-3);
    EXPECT(imt_fs_grade_median(rows, 2, 0.5) == 0,
           "two rows are not enough to grade");
    EXPECT(rows[1].status == IMT_INFO, "and nothing is marked");

    /* Zero sigmas are skipped, not treated as infinitely low. */
    for (int i = 0; i < 5; i++) fs_set(&rows[i], 125 << i, 1.9e-3);
    fs_set(&rows[2], 500, 0.0);
    EXPECT(imt_fs_grade_median(rows, 5, 0.5) == 0,
           "a row with no measurement is not a finding");

    end(fb);
}

static void test_ts_collect_excludes_seams(void)
{
    begin("test_ts_collect_excludes_seams");
    int fb = g_fail;

    imt_ts_acc_t a;
    memset(&a, 0, sizeof a);
    double out[16];
    int n = 0;

    const uint32_t b1[] = { 1000, 1100, 1200 };
    const uint32_t b2[] = { 6200, 6300, 6400 };   /* 5000-tick gap at the seam */
    n = imt_ts_collect_burst(&a, b1, 3, out, 16, n);
    EXPECT(n == 2, "first burst yields one delta per sample after the first");
    n = imt_ts_collect_burst(&a, b2, 3, out, 16, n);
    EXPECT(n == 4, "second burst adds two, not three — the seam is dropped");

    int all_inner = 1;
    for (int i = 0; i < n; i++) if (out[i] != 100.0) all_inner = 0;
    EXPECT(all_inner, "every collected delta is the 100-tick sample period");

    /*
     * The seam still reaches the accumulator: a reversal there is precisely
     * what imu.chipts.monotonic reports, so excluding it from the RATE must
     * not exclude it from the record.
     */
    const uint32_t b3[] = { 6350, 6450 };         /* starts before b2 ended */
    n = imt_ts_collect_burst(&a, b3, 2, out, 16, n);
    EXPECT(a.reversals == 1, "the backwards seam was counted");
    EXPECT(a.seam_reversals == 1, "and attributed to a seam");
    EXPECT(n == 5, "a reversed seam contributes no delta, its successor does");

    /* The cap is honoured rather than overrun. */
    int capped = imt_ts_collect_burst(&a, b1, 3, out, 5, 5);
    EXPECT(capped == 5, "a full buffer collects nothing further");

    end(fb);
}

static void test_chipts_rate_uses_measured(void)
{
    begin("test_chipts_rate_uses_measured");
    int fb = g_fail;

    imud_config_t cfg; base_config(&cfg);
    imt_opts_t o;      fast_opts(&o);
    o.phases = IMT_PHASE_PASSIVE;

    mock_base(); script_reset(&o);
    imt_report_t *r = run(&cfg, &o);

    if (r->raw.ts_median_delta > 0 && r->raw.odr_measured_hz > 0) {
        double want = 1e9 / (r->raw.odr_measured_hz * r->raw.ts_median_delta);
        double got  = r->raw.ts_implied_tick_ns;
        EXPECT(fabs(got - want) <= want * 0.001,
               "implied tick is derived from the measured rate");
        /* And demonstrably NOT from the configured one, unless they coincide. */
        double nominal = 1e9 / ((double)r->eff_odr_mhz * 1e-3 * r->raw.ts_median_delta);
        EXPECT(fabs(r->raw.odr_measured_hz - (double)r->eff_odr_mhz * 1e-3) < 1e-9
               || fabs(got - nominal) > want * 0.001,
               "and not from the configured rate");
    } else {
        EXPECT(status_of(r, "imu.chipts.rate") == IMT_SKIP,
               "no deltas means the check skipped rather than guessed");
    }
    free(r);
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

/*
 * Some required checks cannot run on some transports, and no operator action
 * changes that.  `imu.probe.reject` over SPI is the case: chip select addresses
 * the part, so there is no bogus address to reject.  Counting that SKIP as work
 * still owed makes every SPI report say it "does not yet support clearing
 * experimental" while asking for something no SPI report can ever produce --
 * and it did, on a run that was otherwise 76 PASS / 0 FAIL.
 *
 * A skip that IS the operator's to fix must still block, or the distinction is
 * just a way of ignoring gaps.
 */
static void test_structural_skip_is_not_a_blocker(void)
{
    begin("test_structural_skip_is_not_a_blocker");
    int fb = g_fail;

    static imt_report_t r;
    imt_check_t *c;

    /* Structural: unrunnable on this transport, so the run is still complete. */
    stage_clean_report(&r, false, false);
    c = (imt_check_t *)imt_find(&r, "imu.probe.reject");
    EXPECT(c != NULL, "the required check is staged");
    if (c) { c->status = IMT_SKIP; c->structural = true; r.n_pass--; r.n_skip++; }
    imt_decide_verdict(&r);
    EXPECT(r.recommend_clear_experimental,
           "a structurally unrunnable check does not withhold the verdict");
    EXPECT(strstr(r.verdict, "did not run") == NULL,
           "and is not reported as an outstanding blocker");
    /*
     * But it must still be SAID.  The spec's original reasoning for blocking
     * was that unobtained evidence is not a pass, and that is right; what was
     * wrong was making it an unachievable gate.  Naming it keeps the honesty
     * without the deadlock.
     */
    EXPECT(strstr(r.verdict, "imu.probe.reject") != NULL,
           "the unobtainable check is named in the verdict, not hidden");
    EXPECT(strlen(r.verdict) < sizeof r.verdict - 1,
           "the verdict is not truncated by the added clause");

    /* Not structural: the same id, skipped for a reason the operator owns. */
    stage_clean_report(&r, false, false);
    c = (imt_check_t *)imt_find(&r, "imu.probe.reject");
    if (c) { c->status = IMT_SKIP; c->structural = false; r.n_pass--; r.n_skip++; }
    imt_decide_verdict(&r);
    EXPECT(!r.recommend_clear_experimental,
           "an ordinary skip of a required check still blocks");
    EXPECT(strstr(r.verdict, "did not run") != NULL,
           "and is named in the verdict");

    end(fb);
}

/*
 * The FAIL verdict framed itself around clearing `experimental` whether or not
 * the flag was set, so a driver already cleared was told it "is not ready to
 * have its experimental flag cleared" -- a state that does not exist.  Same
 * reasoning the clean-run branch has always applied.
 */
static void test_fail_verdict_respects_experimental_flag(void)
{
    begin("test_fail_verdict_respects_experimental_flag");
    int fb = g_fail;

    static imt_report_t r;

    stage_clean_report(&r, false, false);
    r.n_fail = 1;
    imt_decide_verdict(&r);
    EXPECT(strstr(r.verdict, "experimental") == NULL,
           "a FAIL on an already-cleared driver does not mention the flag");
    EXPECT(strstr(r.verdict, "FAILED") != NULL, "but still reports the failure");

    stage_clean_report(&r, true, false);
    r.n_fail = 1;
    imt_decide_verdict(&r);
    EXPECT(strstr(r.verdict, "experimental") != NULL,
           "an experimental driver is still told what the failure costs it");

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
    test_sweep_avoids_reserved_registers();
    test_akm_sweep_avoids_forbidden_registers();
    test_volatile_registers_filtered();
    test_nonidempotent_init_names_registers();
    test_fifo_port_window_not_swept();
    test_mag_degauss_ordering_and_restore();
    test_mag_degauss_skips_without_the_op();
    test_mag_writeonly_ctrl_skips();
    test_burst_framing();
    test_direct_vs_fifo();
    test_direct_window_table();
    test_probe_reject_skips_on_spi();
    test_sweep_avoids_write_only_registers();
    test_rate_above_configured_odr_fails();
    test_rate_below_configured_odr_warns();
    test_odr_and_seq();
    test_error_contract_both_ways();
    test_gravity_and_stuck_axis();
    test_temperature_placeholder();
    test_gpio_both_branches();
    test_mag_drdy_rate();
    test_drdy_two_pass();
    test_faces_good_and_swapped();
    test_faces_bad_magnitude_does_not_blame_the_remap();
    test_faces_skipped_not_absent();
    test_faces_thin_face();
    test_gyro_sign();
    test_direct_gyro_vs_fifo();
    test_spin_frame_agreement();
    test_spin_hard_iron_still_tracks();
    test_report_and_exit_codes();
    test_report_names_the_transport();
    test_sim_like_no_recommendation();
    test_degauss_split();
    test_chipts_accounting();
    test_drdy_window_must_allow_an_edge();
    test_mag_rate_names_the_direction();
    test_daemon_conflict_checks_both_sockets();
    test_saturating_counters_are_declared_volatile();
    test_bus_integrity_uses_an_invariant();
    test_field_status_grades_the_driver_not_the_install();
    test_bus_integrity_status();
    test_overflow_status();
    test_fs_grade_median();
    test_fs_scales_with_range();
    test_ts_collect_excludes_seams();
    test_chipts_rate_uses_measured();
    test_chipts_wall_bands();
    test_verdict_respects_experimental_flag();
    test_structural_skip_is_not_a_blocker();
    test_fail_verdict_respects_experimental_flag();

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
