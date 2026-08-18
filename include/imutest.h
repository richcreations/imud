/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imutest.h — hardware validation of a registered sensor driver (ROADMAP §1).
 *
 * A driver's register map is checked against the datasheet when it is written,
 * and test/test_drivers.c proves its encode/decode against a mock I2C bus.
 * Neither says anything about timing, real ODR, FIFO or DRDY behaviour, or
 * whether the chip→board axis remap is right.  This fills that gap: it drives
 * a real chip through the whole imu_ops_t / mag_ops_t contract and produces a
 * report a maintainer can act on without owning the hardware.
 *
 * Structure follows fit_ra.h: the core (src/imutest.c) fills an imt_report_t
 * and never touches a terminal, and the formatters (src/imutest_report.c) turn
 * it into a digest and a Markdown file.  The operator-guided phases live in
 * the core too — the six-face sign table, the right-hand-rule expectations and
 * every tolerance are the part worth testing — reaching the outside world only
 * through the imt_ui_t callbacks below.  test/test_imutest.c supplies a
 * scripted imt_ui_t and drives the whole thing against test/bus_mock.c.
 *
 * imt_report_t is large (tens of kB).  calloc it; do not put it on the stack.
 */
#ifndef IMUD_IMUTEST_H
#define IMUD_IMUTEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "drivers.h"

#define IMT_MAX_CHECKS     160
#define IMT_MAX_REGDIFF     96    /* per device */
#define IMT_MAX_FS_ROWS      8    /* matches supported_accel_g/gyro_dps */
#define IMT_MAG_SECTORS     24    /* 15° each; matches cal_main.c N_SECTORS */
#define IMT_MAX_FIFO_STEPS  12
#define IMT_G_MS2            9.80665

/* ── Check results ─────────────────────────────────────────────────────────── */

/*
 * FAIL means the driver violates drivers.h or the driver guide on evidence a
 * bench cannot explain away.  WARN means out of band with a plausible physical
 * cause (unlevel surface, a moving board, magnetic clutter, a starved
 * scheduler) or a check that is one-sided by nature — it asks the maintainer
 * to read the number, and never blocks clearing `experimental`.  SKIP means
 * the capability is absent, the phase was not selected, a prerequisite failed,
 * or the resource was unavailable.  INFO is recorded with no pass criterion.
 */
typedef enum {
    IMT_SKIP = 0,
    IMT_INFO,
    IMT_PASS,
    IMT_WARN,
    IMT_FAIL
} imt_status_t;

typedef struct {
    /* Stable and greppable: "imu.odr", "face.3.sign", "spin.frame_agreement".
     * imt_find() matches on the whole string, so this must be wide enough for
     * the longest id or lookups silently miss. */
    char         id[32];
    char         name[64];     /* "Measured ODR against the rate the driver reports" */
    imt_status_t status;
    char         measured[56]; /* "831.4 Hz", "-9.803 m/s^2", "0x6B" */
    char         expected[56]; /* "833 Hz +/-5%", "0x6B" */
    char         note[192];    /* one-line diagnosis, already interpreted */
} imt_check_t;

/* ── Raw measurements (report appendix) ────────────────────────────────────── */

typedef struct { uint8_t reg, before, after; } imt_regdiff_t;

typedef struct {              /* Welford accumulation, finalised */
    uint64_t n;
    double   mean[3], sigma[3], min[3], max[3];
} imt_stats3_t;

typedef struct {
    int          fs;          /* g for accel rows, dps for gyro rows */
    double       grav_mean;   /* m/s², accel rows only */
    double       ratio;       /* IMT_G_MS2 / grav_mean; 2.00 = classic 2x error */
    double       sigma[3];    /* per-axis stddev at this full scale */
    int          n;
    imt_status_t status;
} imt_fs_row_t;

typedef struct {
    int          idx;         /* 0..5 */
    const char  *label;       /* static string owned by imutest.c */
    int          exp_axis;    /* 0=X 1=Y 2=Z */
    int          exp_sign;    /* +1 / -1 */
    double       a[3], norm;  /* measured mean vector and |a| */
    int          got_axis, got_sign;
    int          n;
    imt_status_t status;
} imt_face_row_t;

typedef struct {
    int          axis;        /* 0=X roll, 1=Y pitch, 2=Z yaw */
    double       theta[3];    /* integrated angle, degrees, all three axes */
    double       cmd_deg, dur_s;
    int          n;
    bool         used_chip_ts;/* dt from chip_ts, else from 1/eff_odr */
    imt_status_t status;
} imt_turn_row_t;

typedef enum {
    IMT_GPIO_OK = 0,
    IMT_GPIO_DISABLED,        /* int_gpio == 0 in the config */
    IMT_GPIO_EBUSY,           /* another process holds the line — is imud up? */
    IMT_GPIO_ENOCHIP,
    IMT_GPIO_EIO,
    IMT_GPIO_NOEDGES,
    IMT_GPIO_UNSUPPORTED      /* built without libgpiod (test stub) */
} imt_gpio_why_t;

typedef struct {
    /* IMU, passive */
    double        odr_measured_hz, odr_window_s, odr_max_loop_gap_ms;
    uint64_t      odr_n;
    int           odr_best_table_hz;   /* supported_odr_hz entry actually matched */
    uint32_t      seq_first, seq_last;
    uint64_t      seq_gaps, seq_backwards, seq_max_gap;
    int           rc1_count, rcneg_count, last_errno;
    double        reset_ms, mag_reset_ms;
    imt_stats3_t  accel, gyro;
    double        grav_mean, grav_sigma;
    double        temp_min, temp_max, temp_mean;
    int           temp_distinct;
    uint32_t      ts_first, ts_last;
    double        ts_median_delta, ts_implied_tick_ns, ts_wall_ratio;
    /* Reversals and repeats are separate faults: a repeat is one reading
     * stamped across a burst, a reversal is a later sample carrying an
     * earlier tick. Zero-stamped samples are excluded from both. */
    int           ts_backwards, ts_repeats, ts_zero_count, ts_wraps;
    int           fifo_steps;
    double        fifo_wait_s[IMT_MAX_FIFO_STEPS];
    int           fifo_depth[IMT_MAX_FIFO_STEPS];
    double        overflow_after_s;
    /*
     * DRDY edges, counted twice over the same window: once draining the FIFO
     * on every edge (what the daemon does), once not draining at all.  The
     * pair is the discriminator the single count never was — see check_drdy.
     */
    int           gpio_edges, gpio_edges_idle;
    double        gpio_window_s, gpio_rate_hz, gpio_rate_idle_hz;
    bool          gpio_idle_valid;   /* the second pass ran and succeeded */
    imt_gpio_why_t gpio_why;
    int           n_fs_accel, n_fs_gyro;
    imt_fs_row_t  fs_accel[IMT_MAX_FS_ROWS], fs_gyro[IMT_MAX_FS_ROWS];
    int           n_regdiff_imu;
    imt_regdiff_t regdiff_imu[IMT_MAX_REGDIFF];
    bool          regdiff_imu_mapped;   /* a safe-register map existed */
    /* Registers the volatile scan excluded, and the number left to compare.
     * Both are reported: filtering that happens silently reads as a cleaner
     * chip rather than a narrower test. */
    int           n_volatile_imu, n_scanned_imu;

    /* Magnetometer, passive */
    double        mag_rate_hz, mag_window_s;
    uint64_t      mag_n;
    int           mag_rc1, mag_rcneg;
    /*
     * The same rate measured the way the DAEMON gets it: waiting on the mag
     * interrupt instead of polling.  Kept separate because the two can disagree
     * — on a part whose DRDY is a latched interrupt, acknowledging the edge is
     * what clears the status bit a polled read gates on, so a driver that gates
     * unconditionally stalls until its timeout and the polled figure describes a
     * rate the daemon cannot reach.  mag_drdy_edges < 0 means the check did not
     * run (no interrupt configured, or the line was unavailable).
     */
    double        mag_drdy_rate_hz, mag_drdy_window_s;
    int           mag_drdy_edges, mag_drdy_samples;
    imt_stats3_t  magf;
    double        mag_norm_mean, mag_norm_min, mag_norm_max;

    /*
     * SET/RESET differential.  mag_dg_n is 0 when the part has no directional
     * degauss or the pair could not be collected; everything below is only
     * meaningful when it is non-zero.  See imt_degauss_split().
     */
    uint64_t      mag_dg_n;                     /* samples in the smaller half */
    double        mag_dg_set[3], mag_dg_reset[3];
    double        mag_dg_field[3], mag_dg_offset[3];
    double        mag_dg_field_norm, mag_dg_offset_norm;

    /*
     * mag.burst_framing: the output window read as one burst and again one
     * register at a time.  Raw bytes, because the check reports a verdict and
     * the bytes are what anyone diagnosing a mismatch actually needs — an
     * off-by-one shift and a stuck pointer look identical in a pass/fail.
     * mag_bf_n is 0 when the check did not run.
     */
    int           mag_bf_n;                     /* bytes in each half */
    uint8_t       mag_bf_burst[32];
    uint8_t       mag_bf_single[32];
    int           n_regdiff_mag;
    imt_regdiff_t regdiff_mag[IMT_MAX_REGDIFF];
    bool          regdiff_mag_mapped;
    bool          regdiff_mag_writeonly;  /* control registers do not read back */
    int           n_volatile_mag;

    /* Guided phases */
    int            n_faces;
    imt_face_row_t face[6];
    double         face_offset[3], face_scale[3];  /* the cal model, INFO only */
    int            n_turns;
    imt_turn_row_t turn[3];
    int            spin_sectors[IMT_MAG_SECTORS], spin_covered;
    double         spin_heading_delta_deg, spin_gyro_z_deg;
    double         spin_bz_mean, spin_range[3], spin_norm_mean;
    uint64_t       spin_n;
} imt_raw_t;

/* ── The report ────────────────────────────────────────────────────────────── */

typedef struct {
    /* Environment */
    char     imud_version[16];
    /* All four come from struct utsname, whose fields are 65 bytes on glibc.
     * Sized to hold one whole — a hostname clipped at 63 characters in a
     * report written to be pasted into an issue helps nobody. */
    char     sysname[96], release[96], machine[96], hostname[96];
    bool     gpiod_v2;
    char     started_utc[32];       /* ISO-8601 Z */
    double   wall_duration_s;
    char     config_path[256], i2c_bus[64], gpio_chip[32];
    bool     daemon_was_running;
    char     cmdline[512];          /* reproduce-this-run line */

    /* Subject under test */
    char     imu_driver[32], mag_driver[32];
    int      imu_addr, mag_addr, imu_int_gpio, mag_int_gpio;
    bool     imu_experimental, mag_experimental, have_mag, is_sim;
    int      req_odr_hz, eff_odr_hz, accel_g, gyro_dps, fifo_wm;
    int      mag_req_odr_hz, mag_eff_odr_hz;
    float    mag_set_period_s;
    bool     imu_has_fifo, imu_has_hw_ts;
    uint32_t imu_ts_tick_ns;         /* the driver's declared typical */
    uint32_t imu_ts_tick_actual_ns;  /* what the checks graded against: the
                                      * part's own period where it declares
                                      * one (imu_ops_t.ts_tick_ns_actual),
                                      * else a copy of the above */
    int      imu_odr_tab[16], imu_accel_tab[8], imu_gyro_tab[8];
    bool     mag_has_interrupt, mag_has_set_reset, mag_set_reset_nonnull;
    int      mag_odr_tab[16];

    /* What actually ran */
    unsigned phases_requested, phases_run;
    bool     aborted;               /* SIGINT, or ui->prompt returned -1 */

    /* Results */
    int         n_checks;
    imt_check_t check[IMT_MAX_CHECKS];
    imt_raw_t   raw;

    int      n_pass, n_warn, n_fail, n_skip, n_info;
    bool     recommend_clear_experimental;
    char     verdict[256];          /* the report's headline sentence */
} imt_report_t;

/* ── Operator interface ────────────────────────────────────────────────────── */

/*
 * How the core reaches the outside world.  The tool main implements these with
 * a terminal; test/test_imutest.c implements them by restaging the mock, which
 * is what makes the guided phases testable off hardware.
 */
typedef struct {
    /*
     * Blocking.  Show `title` and `body`, wait for acknowledgement.  `id` is
     * the check-group id ("face.1", "gyro.x", "spin"), so a scripted
     * implementation can stage the right data for what is about to be measured.
     * Returns 0 to proceed, 1 to skip this item, -1 to abort the whole run.
     */
    int  (*prompt)(void *user, const char *id,
                   const char *title, const char *body);

    /*
     * Non-blocking: has the operator signalled "done" since the last prompt()?
     * 1 = done, 0 = not yet, -1 = abort.  Polled once per collection iteration
     * of the open-ended guided phases (gyro turns and the mag spin).
     */
    int  (*poll_done)(void *user);

    /*
     * Non-blocking progress.  CONTRACT: the core calls this at least once per
     * collection iteration of every timed phase, even when it has nothing new
     * to say — test_imutest.c uses it as its data-injection hook.  `frac` is
     * 0..1, or negative when the phase is open-ended.
     */
    void (*progress)(void *user, const char *id, double frac, const char *detail);

    /* Sector-coverage bar for the mag spin.  NULL falls back to progress(). */
    void (*coverage)(void *user, const int *sectors, int nsec, int cur,
                     int n_samples, double radius_ut);

    void *user;
} imt_ui_t;

/* ── Options ───────────────────────────────────────────────────────────────── */

#define IMT_PHASE_PASSIVE (1u << 0)
#define IMT_PHASE_FACES   (1u << 1)
#define IMT_PHASE_GYRO    (1u << 2)
#define IMT_PHASE_SPIN    (1u << 3)
#define IMT_PHASE_ALL     (IMT_PHASE_PASSIVE | IMT_PHASE_FACES | \
                           IMT_PHASE_GYRO    | IMT_PHASE_SPIN)

typedef struct {
    unsigned phases;
    double odr_window_s;     /* 5.0; also the seq and chip_ts window. Min 3.0 —
                              * the ICM-42688-P's 20-bit counter wraps at about
                              * 1.05 s and the wrap check needs several. */
    double noise_window_s;   /* 10.0 — noise floor and temperature */
    double drdy_window_s;    /* 3.0 */
    double mag_window_s;     /* 5.0 */
    double face_settle_s;    /* 0.7 */
    double face_collect_s;   /* 2.0 */
    double turn_deg;         /* 90.0 */
    double turn_timeout_s;   /* 30.0 */
    double spin_timeout_s;   /* 180.0 */
    double grav_tol_warn;    /* 0.25 m/s² */
    double grav_tol_fail;    /* 0.60 m/s² */
    double odr_tol_warn;     /* 0.05 fractional */
    double odr_tol_fail;     /* 0.15 fractional */
    bool   fs_sweep;         /* re-init at every supported full scale */
    bool   induce_overflow;  /* deliberately let the FIFO overflow */
    bool   regdiff;          /* snapshot control registers around init() */
    imt_ui_t ui;
} imt_opts_t;

/* Fill defaults.  ui is zeroed: the caller must supply it. */
void imt_opts_defaults(imt_opts_t *o);

/* ── Running ───────────────────────────────────────────────────────────────── */

/*
 * Full run: driver lookup, bus_open for each sensor, imt_run_ops, close.
 * Returns 0 when the run completed (the check statuses carry the verdict),
 * -1 on a setup error that prevented any measurement (message in errbuf).
 */
int imt_run(const imud_config_t *cfg, const imt_opts_t *opts,
            imt_report_t *out, char *errbuf, size_t errbufsz);

/*
 * Lower layer: the caller owns the buses and the ops pointers.  `mag` may be
 * NULL for an IMU-only board, in which case every mag check and the spin phase
 * report SKIP — `mbus` is then ignored and may be a closed handle.  This is
 * the entry point test_imutest.c uses, so the test can reference driver ops
 * structs directly and never link src/drivers.c.
 */
int imt_run_ops(const imud_bus_t *ibus, const imud_bus_t *mbus,
                const imu_ops_t *imu, const mag_ops_t *mag,
                const imud_config_t *cfg, const imt_opts_t *opts,
                imt_report_t *out, char *errbuf, size_t errbufsz);

/* Ask a running imt_run_ops to stop at the next check boundary (SIGINT). */
void imt_request_abort(void);

/* ── Reporting ─────────────────────────────────────────────────────────────── */

void        imt_print(const imt_report_t *r, FILE *out);   /* terminal digest */
int         imt_write_md(const imt_report_t *r, const char *path,
                         char *errbuf, size_t errbufsz);
int         imt_exit_code(const imt_report_t *r);
const char *imt_status_str(imt_status_t s);
/* Look a check up by id — for tests and for the recommendation logic. */
const imt_check_t *imt_find(const imt_report_t *r, const char *id);

/*
 * Fill in r->verdict and r->recommend_clear_experimental from the checks and
 * flags already in `r`.  imt_run_ops() calls this at the end of every run; it
 * is exposed because it is a pure function over the report and the branch that
 * matters most — a clean run against a driver whose `experimental` flag is
 * already clear — cannot be reached from the mock bus, where several passive
 * checks have no way to pass.  A test that could only reach it by accident
 * would pass whether or not the logic was right.
 */
void imt_decide_verdict(imt_report_t *r);

/*
 * Grade chip-time against wall-time, given their ratio.
 *
 * Exposed for the same reason as imt_decide_verdict(): it is a pure function
 * whose bands are the substance, and the mock bus cannot drive a chip counter
 * to an arbitrary rate — the timestamp is a 32-bit burst read, so a test that
 * tried would be steering a byte-wise "live register" and grading whatever
 * ratio fell out, which asserts nothing about the rule.
 *
 * The rule is asymmetric on purpose.  A counter running FAST (ratio > 1) is
 * ordinary part-to-part oscillator tolerance, and since 1.8 imu.c measures the
 * real period per anchor instead of trusting ts_tick_ns, so it is reported and
 * not faulted.  A counter running SLOW (ratio < 1) means chip time has gone
 * missing — a dropped counter wrap, which no measured period can recover — so
 * it still warns at the same magnitude.
 */
imt_status_t imt_chipts_wall_status(double ratio);

/*
 * chip_ts accounting over a window, one sample at a time.
 *
 * Zero-initialised is "nothing seen yet". Exposed and factored out for the same
 * reason as the helpers around it: the classification is the substance, and the
 * mock bus drives the counter through a real driver, so it cannot stage the
 * three cases that matter — a zero stamp, a repeated tick, a genuine reversal —
 * as a chosen sequence. A test that went through the fake part would grade
 * whatever the driver happened to emit.
 *
 * Three distinct outcomes, deliberately not merged:
 *
 *   zeros      the stamp is absent, not early. ism330dhcx.c and lsm6dso.c leave
 *              a whole burst at 0 when the post-drain timestamp read fails.
 *              Excluded from every comparison, because comparing one scores a
 *              reversal going in and then hands the next real sample a delta of
 *              most of the counter coming out — which would land in the median
 *              that imu.chipts.rate reports.
 *   repeats    consecutive samples carry the identical tick: a burst stamped
 *              from one reading rather than per sample.
 *   reversals  a later sample carries an earlier tick, which on a counter
 *              narrower than 32 bits is usually a missing unwrap.
 *
 * Returns the forward delta to feed the rate estimate, or 0 when this sample
 * contributed none (zero, repeat, or reversal).
 */
typedef struct {
    bool     have;                 /* a nonzero stamp has been seen */
    uint32_t first, prev, last;
    int      reversals, repeats, zeros, wraps;
} imt_ts_acc_t;

uint32_t imt_ts_acc_step(imt_ts_acc_t *a, uint32_t ts);

/*
 * Split a SET/RESET pair into the field it measured and the bridge offset it
 * carried.  Pure, and exposed for the same reason as the two above: the
 * arithmetic is the substance, and the mock bus cannot magnetise anything.
 *
 * An AMR bridge reads +/-S*B + offset.  SET and RESET drive the film opposite
 * ways, so between the two the field term changes sign and the offset does
 * not:
 *
 *     vS = +S*B + offset      vR = -S*B + offset
 *       field  = (vS - vR) / 2
 *       offset = (vS + vR) / 2
 *
 * Either output may be NULL.  This is what separates "the sensor is looking at
 * a strong magnet" from "the sensor has a large offset it is not removing" —
 * two explanations for one high reading that are otherwise indistinguishable
 * without a second transport or a known reference field.
 */
void imt_degauss_split(const double vS[3], const double vR[3],
                       double field[3], double offset[3]);

/* ── GPIO edge counting (src/imutest_gpio.c) ───────────────────────────────── */

/*
 * The only translation unit that includes <gpiod.h>, so everything else links
 * without -lgpiod and the test can stub this out.
 *
 * Requests `gpio` on `chip` for rising edges and counts them for window_ms,
 * calling drain(user) after each edge so a watermark line re-arms — without
 * that, a level that stays asserted yields exactly one edge.
 * Returns the edge count, or -1 with *why set.
 */
int imt_gpio_count_edges(const char *chip, int gpio, long window_ms,
                         void (*drain)(void *), void *user,
                         imt_gpio_why_t *why);

#endif /* IMUD_IMUTEST_H */
