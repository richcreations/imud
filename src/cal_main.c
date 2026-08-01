/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cal_main.c — imud-cal: sensor calibration tool
 *
 * Usage: imud-cal [--config PATH] <mode>
 *
 * Modes:
 *   mag    Magnetometer hard/soft-iron calibration.
 *          Drive the vessel slowly through at least two full 360° circles.
 *          Press Ctrl-C when done.  Run with daemon stopped.
 *
 *   gyro   Gyroscope bias capture.
 *          Hold the sensor completely still for the collection window.
 *          Run with daemon stopped.
 *
 *   accel  Accelerometer 6-position calibration (bench, before mounting).
 *          Follow prompts to orient the sensor on each face in turn.
 *          Run with daemon stopped and sensor removed from vessel.
 *
 * Reads sensor config from imud.conf.  Output written to cal_file from config.
 * If cal.json already exists, only the sections just calibrated are updated;
 * previously calibrated fields are preserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>

#include "cal.h"
#include "cal_math.h"
#include "capture.h"
#include "config.h"
#include "imu.h"
#include "drivers.h"
#include "fit_ra.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define G_MS2            9.80665f

/* Mag mode */
#define MAX_MAG_SAMPLES  16000    /* ~2.7 min at 100 Hz */
#define N_SECTORS        24       /* 15° each */
#define MAG_POLL_US      5000     /* poll interval when sensor not ready */

/* Gyro mode */
#define GYRO_COLLECT_S   5        /* averaging window */

/* Accel mode */
#define ACCEL_SETTLE_MS  700      /* discard after repositioning */
#define ACCEL_COLLECT_MS 2000     /* averaging window per position */

/* ── Signal handling ────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* Forward declaration — defined below after imu_collect. */
static int imu_collect(int fd, const imu_ops_t *ops, uint8_t addr,
                       int duration_ms,
                       double gyro_sum[3], double accel_sum[3]);

/* ── Startup settle helper ───────────────────────────────────────────────── */

/*
 * Discards startup_settle_sec worth of IMU samples so thermal gyro drift
 * does not pollute the calibration window.  Signal handler must be installed
 * before calling so g_stop is set on Ctrl-C.
 * Returns false normally, true if interrupted (caller should abort and exit).
 */
static bool settle_imu(int fd, const imu_ops_t *ops, const imud_config_t *cfg)
{
    if (cfg->startup_settle_sec <= 0.0) return false;
    printf("Settling %.0f s for thermal stabilization...\n", cfg->startup_settle_sec);
    imu_collect(fd, ops, (uint8_t)cfg->imu_addr,
                (int)(cfg->startup_settle_sec * 1000.0), NULL, NULL);
    return g_stop != 0;
}

/* ── Terminal progress line (guided swing feedback) ─────────────────────── */

/*
 * One live line, rewritten in place:
 *
 *   Samples:  1234  [#####o##....##..........] 12/24 (50%)  r=48.3 µT rms=0.81
 *
 * '#' = sector covered, '.' = not yet, 'o' = the direction you are pointing
 * NOW — so gaps in the bar are literally "keep turning until the o has
 * visited every dot". Once a running sphere fit exists, live radius and RMS
 * show the fit converging; at full coverage the line says so (with a bell)
 * and invites one more circle before Ctrl-C.
 */
static void print_mag_progress(int n, const int sectors[N_SECTORS], int cur,
                               bool have_fit, double radius, double rms)
{
    int filled = cal_cov_count(sectors, N_SECTORS);
    printf("\r  Samples: %5d  [", n);
    for (int i = 0; i < N_SECTORS; i++)
        putchar(i == cur ? 'o' : (sectors[i] ? '#' : '.'));
    printf("] %2d/%d (%3.0f%%)", filled, N_SECTORS,
           100.0 * filled / N_SECTORS);
    if (have_fit)
        printf("  r=%.1f µT rms=%.2f", radius, rms);
    if (filled == N_SECTORS)
        printf("  FULL CIRCLE — one more improves the fit; Ctrl-C to finish");
    printf("\033[K");
    fflush(stdout);
}

/* ── I2C open helper ────────────────────────────────────────────────────── */

static int open_i2c(const imud_config_t *cfg)
{
    int fd = open(cfg->i2c_bus, O_RDWR);
    if (fd < 0)
        fprintf(stderr, "cal: cannot open %s: %s\n",
                cfg->i2c_bus, strerror(errno));
    return fd;
}

/* ── IMU FIFO drain helper ──────────────────────────────────────────────── */

/*
 * Drain the IMU FIFO for up to duration_ms milliseconds, accumulating
 * samples into arrays (may be NULL to discard).  Returns sample count.
 */
static int imu_collect(int fd, const imu_ops_t *ops, uint8_t addr,
                       int duration_ms,
                       double gyro_sum[3], double accel_sum[3])
{
    imu_sample_t buf[128];
    int n, total = 0;

    struct timespec t0, now;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (!g_stop) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec  - t0.tv_sec)  * 1000
                        + (now.tv_nsec - t0.tv_nsec) / 1000000L;
        if (elapsed_ms >= duration_ms) break;

        int rc = ops->read(fd, addr, buf, 128, &n);
        if (rc < 0 || n == 0) { usleep(10000); continue; }

        for (int i = 0; i < n; i++) {
            if (gyro_sum) {
                gyro_sum[0]  += buf[i].gyro[0];
                gyro_sum[1]  += buf[i].gyro[1];
                gyro_sum[2]  += buf[i].gyro[2];
            }
            if (accel_sum) {
                accel_sum[0] += buf[i].accel[0];
                accel_sum[1] += buf[i].accel[1];
                accel_sum[2] += buf[i].accel[2];
            }
            total++;
        }
        usleep(5000);
    }
    return total;
}

/* ── Mode: magnetometer hard/soft-iron ─────────────────────────────────── */

static int do_mag(const imud_config_t *cfg, imud_cal_t *cal)
{
    const mag_ops_t *ops = mag_driver_find(cfg->mag_driver);
    if (!ops) {
        fprintf(stderr, "cal: unknown mag driver '%s'\n", cfg->mag_driver);
        return -1;
    }

    int fd = open_i2c(cfg);
    if (fd < 0 && strcmp(cfg->mag_driver, "sim") != 0) return -1;

    mag_cfg_t mcfg = { .odr_hz = cfg->mag_odr_hz, .set_period_s = 0.0f };

    if (ops->probe(fd, (uint8_t)cfg->mag_addr) < 0 ||
        ops->reset(fd, (uint8_t)cfg->mag_addr) < 0 ||
        ops->init (fd, (uint8_t)cfg->mag_addr, &mcfg) < 0) {
        fprintf(stderr, "cal: mag sensor init failed\n");
        if (fd >= 0) close(fd);
        return -1;
    }

    signal(SIGINT, on_sigint);

    if (cfg->startup_settle_sec > 0.0) {
        printf("Settling %.0f s for thermal stabilization...\n", cfg->startup_settle_sec);
        struct timespec t0, now;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        long settle_ms = (long)(cfg->startup_settle_sec * 1000.0);
        mag_sample_t tmp;
        while (!g_stop) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            long elapsed = (now.tv_sec - t0.tv_sec) * 1000
                         + (now.tv_nsec - t0.tv_nsec) / 1000000L;
            if (elapsed >= settle_ms) break;
            ops->read(fd, (uint8_t)cfg->mag_addr, &tmp);
            usleep(MAG_POLL_US);
        }
        if (g_stop) { signal(SIGINT, SIG_DFL); if (fd >= 0) close(fd); return 0; }
    }

    printf("imud-cal: magnetometer calibration\n");
    printf("Drive the vessel slowly through at least two full 360 deg circles.\n");
    printf("The bar below is the heading circle: '#' covered, '.' still needed,\n");
    printf("'o' where you are pointing now. Turn until every '.' becomes '#';\n");
    printf("it will tell you when the circle is complete. Ctrl-C when done.\n\n");

    /* Storage for all raw samples (for residual and soft-iron computation) */
    float (*samps)[3] = malloc(MAX_MAG_SAMPLES * sizeof(*samps));
    if (!samps) { perror("malloc"); if (fd >= 0) close(fd); return -1; }

    sphere_accum_t acc = {0};
    int    sectors[N_SECTORS] = {0};
    int    n = 0;

    /* Running center estimate used for coverage display once we have > 50 pts */
    double cx = 0, cy = 0;
    bool   have_center = false;

    /* Live fit-quality feedback, refreshed every 50 samples */
    bool   have_fit = false;
    double live_r = 0.0, live_rms = 0.0;
    bool   full_announced = false;

    while (!g_stop && n < MAX_MAG_SAMPLES) {
        mag_sample_t s;
        int rc = ops->read(fd, (uint8_t)cfg->mag_addr, &s);
        if (rc < 0) { fprintf(stderr, "\ncal: mag read error\n"); break; }
        if (rc != 0) { usleep(MAG_POLL_US); continue; }   /* not ready */
        if (!s.valid) { usleep(MAG_POLL_US); continue; }

        samps[n][0] = s.field[0];
        samps[n][1] = s.field[1];
        samps[n][2] = s.field[2];
        sphere_add(&acc, s.field[0], s.field[1], s.field[2]);
        n++;

        /* Update center estimate + live fit quality every 50 samples */
        if (n % 50 == 0 && n >= 50) {
            double r;
            double ctr[3];
            if (sphere_fit(&acc, ctr, &r) == 0) {
                cx = ctr[0]; cy = ctr[1];
                have_center = true;
                double ss = 0.0;
                for (int i = 0; i < n; i++) {
                    double dx = samps[i][0] - ctr[0];
                    double dy = samps[i][1] - ctr[1];
                    double dz = samps[i][2] - ctr[2];
                    double res = sqrt(dx*dx + dy*dy + dz*dz) - r;
                    ss += res * res;
                }
                live_r   = r;
                live_rms = sqrt(ss / n);
                have_fit = true;
            }
        }

        int cur = cal_cov_mark(sectors, N_SECTORS, s.field[0], s.field[1],
                               have_center ? cx : 0.0, have_center ? cy : 0.0);
        if (!full_announced && cal_cov_count(sectors, N_SECTORS) == N_SECTORS) {
            full_announced = true;
            putchar('\a');   /* audible cue: eyes are on the helm, not the screen */
        }
        print_mag_progress(n, sectors, cur, have_fit, live_r, live_rms);
    }

    signal(SIGINT, SIG_DFL);
    printf("\n");

    /* ── Sphere fit ──────────────────────────────────────────────────── */

    double center[3], radius;
    if (sphere_fit(&acc, center, &radius) < 0) {
        fprintf(stderr, "cal: sphere fit failed — not enough distinct samples\n");
        free(samps); if (fd >= 0) close(fd); return -1;
    }

    /* ── Soft-iron: diagonal scale from per-axis half-range ─────────── */

    double rms = 0.0;
    ellipse_accum_t eacc = {0};
    extent_accum_t  xacc = {0};

    for (int i = 0; i < n; i++) {
        double dx = samps[i][0] - center[0];
        double dy = samps[i][1] - center[1];
        double dz = samps[i][2] - center[2];
        double res = sqrt(dx*dx + dy*dy + dz*dz) - radius;
        rms += res * res;
        ellipse_add(&eacc, (float)dx, (float)dy);
        extent_add(&xacc, dx, dy, dz);
    }
    rms = sqrt(rms / n);
    free(samps);

    /* Per-axis half-range after centering */
    double half[3];
    if (extent_half(&xacc, half) < 0) {
        fprintf(stderr, "cal: no samples to measure\n");
        if (fd >= 0) close(fd);
        return -1;
    }

    /*
     * Horizontal soft iron: 2D ellipse fit on the centered X/Y samples.
     * Unlike the per-axis half-range scale, this recovers the CROSS term —
     * a distortion ellipse whose axes are rotated away from the sensor
     * axes, which is the common case and shows up as a periodic heading
     * error around the swing circle. Falls back to the diagonal method
     * when the fit is degenerate (poor coverage).
     */
    double S2[2][2] = { {1.0, 0.0}, {0.0, 1.0} };
    /* Horizontal radius: half the mean horizontal extent (the sphere-fit
     * radius includes Z and over-scales flat swing data). */
    double r_h = 0.5 * (half[0] + half[1]);
    bool ellipse_ok = (ellipse_fit(&eacc, r_h, S2) == 0);

    /* Only apply Z soft-iron correction if we have meaningful Z coverage.
     * For horizontal boat data, half[2] << radius; forcing a correction
     * there would amplify noise rather than remove distortion. */
    double si[3];
    cal_softiron_diag(half, radius, si);

    /* ── Results ─────────────────────────────────────────────────────── */

    int covered = cal_cov_count(sectors, N_SECTORS);
    printf("Results:\n");
    printf("  Hard iron (µT):   [%7.2f, %7.2f, %7.2f]\n",
           center[0], center[1], center[2]);
    printf("  Field radius:     %.1f µT  (typical: 25–65 µT)\n", radius);
    if (ellipse_ok) {
        printf("  Soft iron (2D):   [%.4f %+.4f; %+.4f %.4f]  Z=%.4f%s\n",
               S2[0][0], S2[0][1], S2[1][0], S2[1][1], si[2],
               half[2] < 0.3 * radius ? "  (Z: no 3D coverage, left as 1.0)" : "");
        if (fabs(S2[0][1]) > 0.005)
            printf("                    (cross term %+.4f: distortion axes are "
                   "rotated — a diagonal fit would miss this)\n", S2[0][1]);
    } else {
        printf("  Soft iron diag:   [%.4f, %.4f, %.4f]%s  (ellipse fit degenerate "
               "— using per-axis fallback)\n",
               si[0], si[1], si[2],
               half[2] < 0.3 * radius ? "  (Z: no 3D coverage, left as 1.0)" : "");
    }
    printf("  RMS residual:     %.2f µT  (< 1.0 µT is good)\n", rms);
    printf("  Coverage:         %d/%d sectors (%.0f%%)%s\n",
           covered, N_SECTORS, 100.0 * covered / N_SECTORS,
           covered < 18 ? "  WARNING: < 75%, swing a wider arc" : "");
    printf("\n");

    /* ── Sanity checks ───────────────────────────────────────────────── */

    if (radius < 15.0 || radius > 100.0)
        printf("WARNING: field radius %.1f µT is outside 15–100 µT — "
               "check sensor and environment\n", radius);
    if (rms > 2.0)
        printf("WARNING: RMS residual %.2f µT > 2 µT — "
               "consider re-running in a cleaner magnetic environment\n", rms);
    if (covered < 18)
        printf("WARNING: only %d/24 sectors covered — "
               "do at least two full circles for a reliable fit\n", covered);

    /* ── Confirm and save ────────────────────────────────────────────── */

    printf("Write mag calibration to %s? [y/N] ", cfg->cal_file);
    fflush(stdout);
    char ans[8] = {0};
    if (!fgets(ans, sizeof(ans), stdin) || (ans[0] != 'y' && ans[0] != 'Y')) {
        printf("Calibration not saved.\n");
        if (fd >= 0) close(fd);
        return 0;
    }

    cal->mag_hard_iron[0] = (float)center[0];
    cal->mag_hard_iron[1] = (float)center[1];
    cal->mag_hard_iron[2] = (float)center[2];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cal->mag_soft_iron[i][j] = (i == j) ? (float)si[i] : 0.0f;
    if (ellipse_ok) {
        /* 2×2 horizontal block (with cross term) + Z from the guard above. */
        cal->mag_soft_iron[0][0] = (float)S2[0][0];
        cal->mag_soft_iron[0][1] = (float)S2[0][1];
        cal->mag_soft_iron[1][0] = (float)S2[1][0];
        cal->mag_soft_iron[1][1] = (float)S2[1][1];
    }

    cal->has_mag = true;

    if (fd >= 0) close(fd);
    return 0;
}

/* ── Mode: gyro bias ────────────────────────────────────────────────────── */

static int do_gyro(const imud_config_t *cfg, imud_cal_t *cal)
{
    const imu_ops_t *ops = imu_driver_find(cfg->imu_driver);
    if (!ops) {
        fprintf(stderr, "cal: unknown IMU driver '%s'\n", cfg->imu_driver);
        return -1;
    }

    int fd = open_i2c(cfg);
    if (fd < 0 && strcmp(cfg->imu_driver, "sim") != 0) return -1;

    imu_cfg_t icfg = {
        .odr_hz   = cfg->imu_odr_hz,
        .accel_g  = cfg->imu_accel_g,
        .gyro_dps = cfg->imu_gyro_dps,
        .fifo_wm  = cfg->imu_fifo_wm,
    };

    if (ops->probe(fd, (uint8_t)cfg->imu_addr) < 0 ||
        ops->reset(fd, (uint8_t)cfg->imu_addr) < 0 ||
        ops->init (fd, (uint8_t)cfg->imu_addr, &icfg) < 0) {
        fprintf(stderr, "cal: IMU sensor init failed\n");
        if (fd >= 0) close(fd);
        return -1;
    }

    printf("imud-cal: gyroscope bias calibration\n");
    printf("Hold the sensor completely still.\n");

    signal(SIGINT, on_sigint);

    if (settle_imu(fd, ops, cfg)) {
        signal(SIGINT, SIG_DFL);
        if (fd >= 0) close(fd);
        return 0;
    }

    printf("Collecting for %d s...\n", GYRO_COLLECT_S);

    /* Collect in two halves: differing half-means reveal motion or drift
     * during the window (rocking dock, hand-held sensor) that a single
     * mean would silently absorb into the bias. */
    double sum_a[3] = {0}, sum_b[3] = {0};
    int na = imu_collect(fd, ops, (uint8_t)cfg->imu_addr,
                         GYRO_COLLECT_S * 500, sum_a, NULL);
    int nb = imu_collect(fd, ops, (uint8_t)cfg->imu_addr,
                         GYRO_COLLECT_S * 500, sum_b, NULL);

    signal(SIGINT, SIG_DFL);

    int n = na + nb;
    if (na < 5 || nb < 5) {
        fprintf(stderr, "cal: too few samples (%d) — sensor not producing data\n", n);
        if (fd >= 0) close(fd);
        return -1;
    }

    double bias[3], half_diff = 0.0;
    for (int k = 0; k < 3; k++) {
        bias[k] = (sum_a[k] + sum_b[k]) / n;
        double d = fabs(sum_a[k] / na - sum_b[k] / nb);
        if (d > half_diff) half_diff = d;
    }

    /* Convert to deg/s for display */
    double d2r = M_PI / 180.0;
    printf("Results (%d samples):\n", n);
    printf("  Gyro bias (rad/s): [%.6f, %.6f, %.6f]\n",
           bias[0], bias[1], bias[2]);
    printf("  Gyro bias (deg/s): [%.4f, %.4f, %.4f]\n",
           bias[0]/d2r, bias[1]/d2r, bias[2]/d2r);

    double mag = sqrt(bias[0]*bias[0] + bias[1]*bias[1] + bias[2]*bias[2]);
    if (mag > 0.05)
        printf("WARNING: bias magnitude %.4f rad/s (%.2f deg/s) is large — "
               "was the sensor moving?\n", mag, mag / d2r);
    if (half_diff > 0.0035)   /* 0.2 °/s between window halves */
        printf("WARNING: half-window means differ by %.3f deg/s — "
               "the sensor moved during capture; re-run on a still surface\n",
               half_diff / d2r);

    printf("\nWrite gyro calibration to %s? [y/N] ", cfg->cal_file);
    fflush(stdout);
    char ans[8] = {0};
    if (!fgets(ans, sizeof(ans), stdin) || (ans[0] != 'y' && ans[0] != 'Y')) {
        printf("Calibration not saved.\n");
        if (fd >= 0) close(fd);
        return 0;
    }

    cal->gyro_bias[0] = (float)bias[0];
    cal->gyro_bias[1] = (float)bias[1];
    cal->gyro_bias[2] = (float)bias[2];
    cal->has_gyro = true;

    if (fd >= 0) close(fd);
    return 0;
}

/* ── Mode: accel 6-position ─────────────────────────────────────────────── */

static int do_accel(const imud_config_t *cfg, imud_cal_t *cal)
{
    const imu_ops_t *ops = imu_driver_find(cfg->imu_driver);
    if (!ops) {
        fprintf(stderr, "cal: unknown IMU driver '%s'\n", cfg->imu_driver);
        return -1;
    }

    int fd = open_i2c(cfg);
    if (fd < 0 && strcmp(cfg->imu_driver, "sim") != 0) return -1;

    imu_cfg_t icfg = {
        .odr_hz   = cfg->imu_odr_hz,
        .accel_g  = cfg->imu_accel_g,
        .gyro_dps = cfg->imu_gyro_dps,
        .fifo_wm  = cfg->imu_fifo_wm,
    };

    if (ops->probe(fd, (uint8_t)cfg->imu_addr) < 0 ||
        ops->reset(fd, (uint8_t)cfg->imu_addr) < 0 ||
        ops->init (fd, (uint8_t)cfg->imu_addr, &icfg) < 0) {
        fprintf(stderr, "cal: IMU sensor init failed\n");
        if (fd >= 0) close(fd);
        return -1;
    }

    signal(SIGINT, on_sigint);

    if (settle_imu(fd, ops, cfg)) {
        signal(SIGINT, SIG_DFL);
        if (fd >= 0) close(fd);
        return 0;
    }

    printf("imud-cal: accelerometer 6-position calibration\n");
    printf("Place the sensor on a flat, level surface in each orientation.\n");
    printf("Press Enter when steady, wait for the beep, then move to the next.\n");
    printf("\n");
    printf("Board frame: X forward, Y starboard, Z DOWN. The axis pointing UP\n");
    printf("reads +g; the axis pointing DOWN reads -g.\n\n");

    /*
     * Positions and the fit both live in cal_math.c so they can be tested
     * without hardware, and so imud-imutest asks for the same six
     * orientations in the same words.
     *
     * meas[axis][0] = reading when axis points up   (expected +g on that axis)
     * meas[axis][1] = reading when axis points down (expected -g on that axis)
     */
    float meas[3][2][3];   /* [axis][up/down][xyz] */

    for (int p = 0; p < 6 && !g_stop; p++) {
        const cal_accel_pos_t *pos = &cal_accel_positions[p];
        int axis = pos->axis;
        int slot = (pos->sign > 0) ? 0 : 1;

        float want[3] = { 0.0f, 0.0f, 0.0f };
        want[axis] = (float)pos->sign * G_MS2;

        printf("Position %d/6: %s\n", p + 1, pos->label);
        printf("  %s\n", pos->detail);
        printf("  Expect roughly [%+.2f, %+.2f, %+.2f] m/s²\n",
               (double)want[0], (double)want[1], (double)want[2]);
        printf("  Press Enter when steady... ");
        fflush(stdout);

        /* Consume any pending input then wait for Enter */
        int c;
        while ((c = getchar()) != '\n' && c != EOF) {}

        printf("  Settling (%.1f s)...", ACCEL_SETTLE_MS / 1000.0);
        fflush(stdout);
        imu_collect(fd, ops, (uint8_t)cfg->imu_addr,
                    ACCEL_SETTLE_MS, NULL, NULL);

        printf("\r  Collecting (%.1f s)...     \n", ACCEL_COLLECT_MS / 1000.0);
        double asum[3] = {0};
        int n = imu_collect(fd, ops, (uint8_t)cfg->imu_addr,
                            ACCEL_COLLECT_MS, NULL, asum);

        if (n < 10) {
            fprintf(stderr, "cal: too few samples at position %d — aborting\n", p + 1);
            signal(SIGINT, SIG_DFL);
            if (fd >= 0) close(fd);
            return -1;
        }

        meas[axis][slot][0] = (float)(asum[0] / n);
        meas[axis][slot][1] = (float)(asum[1] / n);
        meas[axis][slot][2] = (float)(asum[2] / n);

        printf("  Measured [%.3f, %.3f, %.3f] m/s²  (expected: %s%.3f on axis %c)\n",
               meas[axis][slot][0], meas[axis][slot][1], meas[axis][slot][2],
               pos->sign > 0 ? "+" : "-", (double)G_MS2,
               "XYZ"[axis]);
    }

    signal(SIGINT, SIG_DFL);
    if (g_stop) { if (fd >= 0) close(fd); return 0; }
    if (fd >= 0) close(fd);

    /*
     * Geometry check before anything else.  A reading taken in the wrong
     * orientation says nothing about the axis it was filed under, and a
     * calibration fitted from it is worse than no calibration — so this
     * aborts rather than warning.
     */
    float offset[3], scale[3];
    char  err[512] = {0};
    if (cal_accel_fit(meas, offset, scale, err, sizeof(err)) < 0) {
        printf("\nResults:\n  %s\n", err);
        fprintf(stderr,
                "cal: accelerometer geometry check failed — nothing written.\n");
        return -1;
    }

    printf("\nResults:\n");

    bool any_warn = false;
    for (int k = 0; k < 3; k++) {
        printf("  Axis %c: offset %+.4f m/s²  scale %.6f\n",
               "XYZ"[k], offset[k], scale[k]);

        /* Soft checks: the reading is geometrically sane but looks unusual,
         * which is a judgement call for the operator rather than an error. */
        if (fabsf(offset[k]) > 0.5f * G_MS2) {
            printf("  WARNING: axis %c offset %.3f is large\n", "XYZ"[k], offset[k]);
            any_warn = true;
        }
        if (scale[k] < 0.90f || scale[k] > 1.10f) {
            printf("  WARNING: axis %c scale %.4f is outside 0.90–1.10\n",
                   "XYZ"[k], scale[k]);
            any_warn = true;
        }
    }
    if (any_warn)
        printf("  Review warnings before saving.\n");

    printf("\nWrite accel calibration to %s? [y/N] ", cfg->cal_file);
    fflush(stdout);
    char ans[8] = {0};
    if (!fgets(ans, sizeof(ans), stdin) || (ans[0] != 'y' && ans[0] != 'Y')) {
        printf("Calibration not saved.\n");
        return 0;
    }

    for (int k = 0; k < 3; k++) {
        cal->accel_offset[k] = offset[k];
        cal->accel_scale[k]  = scale[k];
    }
    cal->has_accel = true;

    return 0;
}

/* ── Entry point ────────────────────────────────────────────────────────── */

/* ── Offline capture analysis (characterize / fit-temp) ──────────────────── */

/*
 * Load the IMU records of an .imucap into block-averaged double arrays.
 * Long overnight records are decimated to at most CAP_ANALYZE_MAX samples
 * per axis — block-averaging leaves the Allan deviation at the surviving
 * cluster times untouched (θ at block boundaries is exact).
 * Returns sample count, 0 when the file has no IMU records, -1 on error.
 */
#define CAP_ANALYZE_MAX (1u << 22)

static void free_capture(double *gyro[3], double *accel[3], double *temp);

static long load_capture(const char *path, double settle_sec, double *fs_out,
                         double *gyro[3], double *accel[3], double **temp)
{
    cap_reader_t r;
    int rc = cap_reader_open(&r, path);
    if (rc != 0) {
        fprintf(stderr, "cal: cannot open %s (%s)\n", path,
                rc == CAP_ERR_FORMAT ? "not an .imucap / wrong version"
                                     : strerror(errno));
        return -1;
    }

    /* Pass 1: count IMU records and measure the time span. */
    cap_record_t rec;
    uint64_t first_mono = 0, last_mono = 0;
    size_t   count = 0;
    while (cap_reader_next(&r, &rec) == 1) {
        if (rec.type != CAP_REC_IMU) continue;
        if (count == 0) first_mono = rec.mono_ns;
        last_mono = rec.mono_ns;
        count++;
    }
    if (count < 16 || last_mono <= first_mono) {
        fprintf(stderr, "cal: %s holds too little IMU data (%zu records)\n",
                path, count);
        cap_reader_close(&r);
        return 0;
    }

    double span_s = (double)(last_mono - first_mono) * 1e-9;
    double fs_raw = (double)(count - 1) / span_s;
    size_t decim  = count / CAP_ANALYZE_MAX + 1;
    size_t n      = count / decim;

    for (int a = 0; a < 3; a++) {
        gyro[a]  = calloc(n, sizeof(double));
        accel[a] = calloc(n, sizeof(double));
    }
    *temp = calloc(n, sizeof(double));
    if (!gyro[0] || !gyro[1] || !gyro[2] ||
        !accel[0] || !accel[1] || !accel[2] || !*temp) {
        fprintf(stderr, "cal: out of memory (%zu samples)\n", n);
        free_capture(gyro, accel, *temp);
        *temp = NULL;
        cap_reader_close(&r);
        return -1;
    }

    /* Pass 2: block-average into the arrays.  Skip the first settle_sec of
     * data, exactly as the daemon's fusion discards startup_settle_sec: the
     * gyro needs ~5 s to settle after power-on, and those transient samples
     * both trip the motion gate and inflate the Allan noise floor. */
    cap_reader_rewind(&r);
    uint64_t settle_ns = (uint64_t)(settle_sec > 0.0 ? settle_sec * 1e9 : 0.0);
    size_t in_block = 0, out = 0, n_skip = 0;
    double acc_g[3] = {0}, acc_a[3] = {0}, acc_t = 0;
    while (cap_reader_next(&r, &rec) == 1 && out < n) {
        if (rec.type != CAP_REC_IMU) continue;
        if (rec.mono_ns - first_mono < settle_ns) { n_skip++; continue; }
        for (int a = 0; a < 3; a++) {
            acc_g[a] += rec.imu.gyro[a];
            acc_a[a] += rec.imu.accel[a];
        }
        acc_t += rec.imu.temp_c;
        if (++in_block == decim) {
            for (int a = 0; a < 3; a++) {
                gyro[a][out]  = acc_g[a] / (double)decim;
                accel[a][out] = acc_a[a] / (double)decim;
                acc_g[a] = acc_a[a] = 0;
            }
            (*temp)[out] = acc_t / (double)decim;
            acc_t = 0;
            in_block = 0;
            out++;
        }
    }
    cap_reader_close(&r);

    *fs_out = fs_raw / (double)decim;
    printf("Loaded %s: %zu samples over %.1f min (%.1f Hz",
           path, count, span_s / 60.0, fs_raw);
    if (decim > 1)
        printf(", analyzed at %.1f Hz after %zux averaging", *fs_out, decim);
    printf(")\n");
    if (n_skip)
        printf("Skipped first %.1f s startup settle (%zu samples)\n",
               settle_sec, n_skip);
    return (long)out;
}

static void free_capture(double *gyro[3], double *accel[3], double *temp)
{
    for (int a = 0; a < 3; a++) { free(gyro[a]); free(accel[a]); }
    free(temp);
}

/* Warn when the record was not stationary — both analyses assume it. */
static void motion_gate(double *gyro[3], double *accel[3], size_t n)
{
    double gmin = gyro[0][0], gmax = gmin;
    double asum = 0, asum2 = 0;
    for (size_t i = 0; i < n; i++) {
        for (int a = 0; a < 3; a++) {
            if (gyro[a][i] < gmin) gmin = gyro[a][i];
            if (gyro[a][i] > gmax) gmax = gyro[a][i];
        }
        double m = sqrt(accel[0][i] * accel[0][i] +
                        accel[1][i] * accel[1][i] +
                        accel[2][i] * accel[2][i]);
        asum += m; asum2 += m * m;
    }
    double amean = asum / (double)n;
    double astd  = sqrt(asum2 / (double)n - amean * amean);
    if (gmax - gmin > 0.1 || astd > 0.2)
        printf("WARNING: capture does not look stationary "
               "(gyro p-p %.3f rad/s, |a| std %.3f m/s^2) — "
               "results will be pessimistic\n", gmax - gmin, astd);
}

static int do_characterize(imud_cal_t *cal, const char *from, double settle_sec)
{
    double  fs;
    double *gyro[3], *accel[3], *temp;
    long    n = load_capture(from, settle_sec, &fs, gyro, accel, &temp);
    if (n <= 0) return -1;

    motion_gate(gyro, accel, (size_t)n);

    static const char axis[3] = { 'X', 'Y', 'Z' };
    avar_pt_t pts[32];
    bool floor_seen = true;

    printf("\nGyroscope (Allan deviation, %ld samples):\n", n);
    printf("  axis  noise density        bias instability\n");
    for (int a = 0; a < 3; a++) {
        int np = allan_deviation(gyro[a], (size_t)n, fs, pts, 32);
        double nd, bi;
        int imin = allan_characterize(pts, np, &nd, &bi);
        if (imin < 0) {
            fprintf(stderr, "cal: record too short to characterize\n");
            free_capture(gyro, accel, temp);
            return -1;
        }
        if (imin == np - 1) floor_seen = false;
        cal->gyro_noise_density[a]    = (float)nd;
        cal->gyro_bias_instability[a] = (float)bi;
        printf("   %c    %.3e rad/s/vHz   %.3e rad/s (%.2f deg/h)\n",
               axis[a], nd, bi, bi * 180.0 / M_PI * 3600.0);
    }

    printf("\nAccelerometer:\n");
    printf("  axis  noise density\n");
    for (int a = 0; a < 3; a++) {
        int np = allan_deviation(accel[a], (size_t)n, fs, pts, 32);
        double nd, bi;
        if (allan_characterize(pts, np, &nd, &bi) < 0) {
            fprintf(stderr, "cal: record too short to characterize\n");
            free_capture(gyro, accel, temp);
            return -1;
        }
        cal->accel_noise_density[a] = (float)nd;
        printf("   %c    %.3e m/s^2/vHz\n", axis[a], nd);
    }

    if (!floor_seen)
        printf("\nNote: the bias-instability floor was not reached — the\n"
               "values above are upper bounds; a longer (overnight) capture\n"
               "pins them down.\n");
    printf("\nSaved as informational per-unit sensor characterization. These numbers\n"
           "never feed the filter: the MEKF always uses its tuned [fusion] mekf_*\n"
           "values, which are held above the raw sensor floor on purpose.\n");

    cal->has_noise = true;
    free_capture(gyro, accel, temp);
    return 0;
}

#define GYRO_TEMP_REF_C 25.0

static int do_fit_temp(imud_cal_t *cal, const char *from, double settle_sec)
{
    double  fs;
    double *gyro[3], *accel[3], *temp;
    long    n = load_capture(from, settle_sec, &fs, gyro, accel, &temp);
    if (n <= 0) return -1;

    motion_gate(gyro, accel, (size_t)n);

    double tmin = temp[0], tmax = temp[0];
    for (long i = 0; i < n; i++) {
        if (temp[i] < tmin) tmin = temp[i];
        if (temp[i] > tmax) tmax = temp[i];
    }
    printf("Die temperature span: %.1f to %.1f degC\n", tmin, tmax);

    static const char axis[3] = { 'X', 'Y', 'Z' };
    printf("\nGyro bias vs temperature (linear fit, ref %.0f degC):\n",
           GYRO_TEMP_REF_C);
    for (int a = 0; a < 3; a++) {
        double coeff, bias;
        if (gyro_temp_fit(temp, gyro[a], (size_t)n,
                          GYRO_TEMP_REF_C, &coeff, &bias) != 0) {
            fprintf(stderr, "cal: temperature span too small (%.1f degC) — "
                    "capture a cold-boot warm-up (span >= a few degC)\n",
                    tmax - tmin);
            free_capture(gyro, accel, temp);
            return -1;
        }
        cal->gyro_temp_coeff[a] = (float)coeff;
        printf("   %c    %+.3e rad/s per degC\n", axis[a], coeff);
    }
    cal->gyro_temp_ref_c = (float)GYRO_TEMP_REF_C;
    cal->has_gyro_temp   = true;

    printf("\nThe daemon subtracts coeff*(T - %.0f) from the gyro before\n"
           "fusion, so the filter only tracks the residual bias.\n",
           GYRO_TEMP_REF_C);
    free_capture(gyro, accel, temp);
    return 0;
}

/*
 * fit-ra — measurement-model check against a rough-water capture.
 *
 * Unlike every other mode this one writes nothing: the answer belongs in
 * imud.conf's [fusion] section, not in cal.json, because it is filter tuning
 * rather than a property of the sensor. See ROADMAP §10.1.
 */
static int do_fit_ra(const imud_config_t *cfg, const imud_cal_t *cal,
                     const char *path)
{
    fitra_report_t rep;
    char err[256] = {0};
    int rc = fitra_run(path, cfg, cal, cfg->startup_settle_sec,
                       &rep, err, sizeof err);
    if (rc != 0) {
        fprintf(stderr, "fit-ra: %s\n", err[0] ? err : "failed");
        return -1;
    }
    fitra_print(&rep, path);
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--config PATH] [--output PATH] <mode>\n"
        "\n"
        "Modes:\n"
        "  mag          Magnetometer calibration (vessel swing, daemon stopped)\n"
        "  gyro         Gyroscope bias capture   (hold still, daemon stopped)\n"
        "  accel        Accelerometer 6-position (bench, daemon stopped)\n"
        "  characterize Allan-variance noise analysis of a stationary capture\n"
        "               (requires --from FILE; record with [capture] enabled)\n"
        "  fit-temp     Gyro bias/temperature fit from a warm-up capture\n"
        "               (requires --from FILE)\n"
        "  fit-ra       Check the MEKF accel measurement model against a\n"
        "               rough-water capture (requires --from FILE).\n"
        "               Reports only; writes nothing.\n"
        "\n"
        "  --config PATH   Config file (default: /etc/imud/imud.conf)\n"
        "  --output PATH   Override cal.json output path from config\n"
        "  --from FILE     .imucap capture to analyze (offline modes)\n",
        prog);
}

int main(int argc, char **argv)
{
    char        config_path[256];
    const char *output_path = NULL;
    const char *from_path   = NULL;
    const char *mode        = NULL;

    snprintf(config_path, sizeof(config_path), "/etc/imud/imud.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof(config_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            output_path = argv[++i];
        } else if (strcmp(argv[i], "--from") == 0 && i + 1 < argc) {
            from_path = argv[++i];
        } else if (!mode && argv[i][0] != '-') {
            mode = argv[i];
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            usage(argv[0]); return 1;
        }
    }

    if (!mode) { usage(argv[0]); return 1; }

    if (strcmp(mode, "mag")          != 0 &&
        strcmp(mode, "gyro")         != 0 &&
        strcmp(mode, "accel")        != 0 &&
        strcmp(mode, "characterize") != 0 &&
        strcmp(mode, "fit-temp")     != 0 &&
        strcmp(mode, "fit-ra")       != 0) {
        fprintf(stderr, "unknown mode '%s'\n", mode);
        usage(argv[0]); return 1;
    }
    if ((strcmp(mode, "characterize") == 0 || strcmp(mode, "fit-temp") == 0 ||
         strcmp(mode, "fit-ra") == 0)
        && !from_path) {
        fprintf(stderr, "%s requires --from FILE (an .imucap capture)\n", mode);
        usage(argv[0]); return 1;
    }

    /* Load config.  The offline analysis modes only need it for the
     * cal.json path, so a MISSING file falls back to defaults there;
     * sensor modes stay strict (defaults could probe the wrong bus). */
    bool offline = strcmp(mode, "characterize") == 0 ||
                   strcmp(mode, "fit-temp")     == 0 ||
                   strcmp(mode, "fit-ra")       == 0;
    imud_config_t cfg;
    config_defaults(&cfg);
    int crc = config_load(config_path, &cfg);
    if (crc == CONFIG_ERR_PARSE || (crc < 0 && !offline)) {
        fprintf(stderr, "cal: cannot load config from %s\n", config_path);
        return 1;
    }

    if (output_path)
        snprintf(cfg.cal_file, sizeof(cfg.cal_file), "%s", output_path);

    /* Load existing cal.json so we preserve sections we're not updating */
    imud_cal_t cal;
    if (cal_load(cfg.cal_file, &cal) < 0) return 1;

    /* Dispatch */
    int rc = 0;
    if      (strcmp(mode, "mag")          == 0) rc = do_mag  (&cfg, &cal);
    else if (strcmp(mode, "gyro")         == 0) rc = do_gyro (&cfg, &cal);
    else if (strcmp(mode, "accel")        == 0) rc = do_accel(&cfg, &cal);
    else if (strcmp(mode, "characterize") == 0) rc = do_characterize(&cal, from_path, cfg.startup_settle_sec);
    else if (strcmp(mode, "fit-temp")     == 0) rc = do_fit_temp(&cal, from_path, cfg.startup_settle_sec);
    else if (strcmp(mode, "fit-ra")       == 0) rc = do_fit_ra(&cfg, &cal, from_path);

    if (rc < 0) return 1;

    /* Write if any section was updated */
    if (cal.has_mag || cal.has_gyro || cal.has_accel ||
        cal.has_noise || cal.has_gyro_temp) {
        if (cal_write(cfg.cal_file, &cal) < 0) return 1;
        printf("Saved to %s\n", cfg.cal_file);
    }

    return 0;
}
