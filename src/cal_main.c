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
#include "config.h"
#include "imu.h"
#include "drivers.h"

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

/* ── Coverage tracking ──────────────────────────────────────────────────── */

static void sector_mark(int sectors[N_SECTORS], float x, float y,
                        double cx, double cy)
{
    double angle = atan2((double)y - cy, (double)x - cx);
    if (angle < 0) angle += 2.0 * M_PI;
    int s = (int)(angle / (2.0 * M_PI / N_SECTORS)) % N_SECTORS;
    sectors[s] = 1;
}

static int sector_count(const int sectors[N_SECTORS])
{
    int n = 0;
    for (int i = 0; i < N_SECTORS; i++) n += sectors[i];
    return n;
}

/* ── Terminal progress line ─────────────────────────────────────────────── */

static void print_mag_progress(int n, const int sectors[N_SECTORS])
{
    int filled = sector_count(sectors);
    printf("\r  Samples: %5d  [", n);
    for (int i = 0; i < N_SECTORS; i++)
        putchar(sectors[i] ? '#' : '.');
    printf("] %2d/%d sectors\033[K", filled, N_SECTORS);
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
    printf("Press Ctrl-C when done.\n\n");

    /* Storage for all raw samples (for residual and soft-iron computation) */
    float (*samps)[3] = malloc(MAX_MAG_SAMPLES * sizeof(*samps));
    if (!samps) { perror("malloc"); if (fd >= 0) close(fd); return -1; }

    sphere_accum_t acc = {0};
    int    sectors[N_SECTORS] = {0};
    int    n = 0;

    /* Running center estimate used for coverage display once we have > 50 pts */
    double cx = 0, cy = 0;
    bool   have_center = false;

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

        /* Update center estimate every 50 samples for coverage display */
        if (n % 50 == 0 && n >= 50) {
            double r;
            double ctr[3];
            if (sphere_fit(&acc, ctr, &r) == 0) {
                cx = ctr[0]; cy = ctr[1];
                have_center = true;
            }
        }

        sector_mark(sectors, s.field[0], s.field[1],
                    have_center ? cx : 0.0, have_center ? cy : 0.0);
        print_mag_progress(n, sectors);
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

    double mn[3] = { samps[0][0], samps[0][1], samps[0][2] };
    double mx[3] = { samps[0][0], samps[0][1], samps[0][2] };
    double rms = 0.0;

    for (int i = 0; i < n; i++) {
        double dx = samps[i][0] - center[0];
        double dy = samps[i][1] - center[1];
        double dz = samps[i][2] - center[2];
        double res = sqrt(dx*dx + dy*dy + dz*dz) - radius;
        rms += res * res;
        for (int k = 0; k < 3; k++) {
            if (samps[i][k] - center[k] < mn[k]) mn[k] = samps[i][k] - center[k];
            if (samps[i][k] - center[k] > mx[k]) mx[k] = samps[i][k] - center[k];
        }
    }
    rms = sqrt(rms / n);
    free(samps);

    /* Per-axis half-range after centering */
    double half[3];
    for (int k = 0; k < 3; k++) half[k] = (mx[k] - mn[k]) / 2.0;

    /* Only apply Z soft-iron correction if we have meaningful Z coverage.
     * For horizontal boat data, half[2] << radius; forcing a correction
     * there would amplify noise rather than remove distortion. */
    double si[3];
    for (int k = 0; k < 3; k++)
        si[k] = (half[k] > 0.3 * radius) ? (radius / half[k]) : 1.0;

    /* ── Results ─────────────────────────────────────────────────────── */

    int covered = sector_count(sectors);
    printf("Results:\n");
    printf("  Hard iron (µT):   [%7.2f, %7.2f, %7.2f]\n",
           center[0], center[1], center[2]);
    printf("  Field radius:     %.1f µT  (typical: 25–65 µT)\n", radius);
    printf("  Soft iron diag:   [%.4f, %.4f, %.4f]%s\n",
           si[0], si[1], si[2],
           half[2] < 0.3 * radius ? "  (Z: no 3D coverage, left as 1.0)" : "");
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

    double gyro_sum[3] = {0};
    int n = imu_collect(fd, ops, (uint8_t)cfg->imu_addr,
                        GYRO_COLLECT_S * 1000, gyro_sum, NULL);

    signal(SIGINT, SIG_DFL);

    if (n < 10) {
        fprintf(stderr, "cal: too few samples (%d) — sensor not producing data\n", n);
        if (fd >= 0) close(fd);
        return -1;
    }

    double bias[3] = {
        gyro_sum[0] / n,
        gyro_sum[1] / n,
        gyro_sum[2] / n,
    };

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
    printf("Press Enter when steady, wait for the beep, then move to the next.\n\n");

    /*
     * Six positions: for each axis, one face pointing up (+g) and one down (-g).
     * When axis K points up, specific force reads approximately +g on axis K.
     */
    static const struct {
        const char *label;
        int axis;   /* 0=X, 1=Y, 2=Z */
        int sign;   /* +1 = axis up, -1 = axis down */
    } positions[6] = {
        { "+Z up  (flat, normal side up)",   2,  1 },
        { "+Z down (flat, upside down)",     2, -1 },
        { "+X up  (right edge down)",        0,  1 },
        { "+X down (left edge down)",        0, -1 },
        { "+Y up  (front edge down)",        1,  1 },
        { "+Y down (back edge down)",        1, -1 },
    };

    /*
     * meas[axis][0] = reading when axis points up   (expected +g on that axis)
     * meas[axis][1] = reading when axis points down (expected -g on that axis)
     */
    float meas[3][2][3];   /* [axis][up/down][xyz] */

    for (int p = 0; p < 6 && !g_stop; p++) {
        int axis = positions[p].axis;
        int slot = (positions[p].sign > 0) ? 0 : 1;

        printf("Position %d/6: %s\n", p + 1, positions[p].label);
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
               positions[p].sign > 0 ? "+" : "-", (double)G_MS2,
               "XYZ"[axis]);
    }

    signal(SIGINT, SIG_DFL);
    if (g_stop) { if (fd >= 0) close(fd); return 0; }
    if (fd >= 0) close(fd);

    printf("\nResults:\n");

    float offset[3], scale[3];
    bool any_warn = false;
    for (int k = 0; k < 3; k++) {
        float plus  = meas[k][0][k];   /* on-axis reading when face up   */
        float minus = meas[k][1][k];   /* on-axis reading when face down */

        /*
         * Calibration model: a_cal = (a_raw - offset) * scale
         * offset: midpoint of +g and -g readings (should be 0)
         * scale:  true g / measured half-range
         */
        offset[k] = (plus + minus) / 2.0f;
        float half_range = (plus - minus) / 2.0f;
        scale[k]  = (half_range > 0.1f) ? (G_MS2 / half_range) : 1.0f;

        printf("  Axis %c: offset %+.4f m/s²  scale %.6f\n",
               "XYZ"[k], offset[k], scale[k]);

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

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--config PATH] [--output PATH] <mode>\n"
        "\n"
        "Modes:\n"
        "  mag    Magnetometer calibration (vessel swing, daemon stopped)\n"
        "  gyro   Gyroscope bias capture   (hold still, daemon stopped)\n"
        "  accel  Accelerometer 6-position (bench, daemon stopped)\n"
        "\n"
        "  --config PATH   Config file (default: /etc/imud/imud.conf)\n"
        "  --output PATH   Override cal.json output path from config\n",
        prog);
}

int main(int argc, char **argv)
{
    char        config_path[256];
    const char *output_path = NULL;
    const char *mode        = NULL;

    snprintf(config_path, sizeof(config_path), "/etc/imud/imud.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof(config_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            output_path = argv[++i];
        } else if (!mode && argv[i][0] != '-') {
            mode = argv[i];
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            usage(argv[0]); return 1;
        }
    }

    if (!mode) { usage(argv[0]); return 1; }

    if (strcmp(mode, "mag")   != 0 &&
        strcmp(mode, "gyro")  != 0 &&
        strcmp(mode, "accel") != 0) {
        fprintf(stderr, "unknown mode '%s'\n", mode);
        usage(argv[0]); return 1;
    }

    /* Load config */
    imud_config_t cfg;
    config_defaults(&cfg);
    if (config_load(config_path, &cfg) < 0) {
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
    if      (strcmp(mode, "mag")   == 0) rc = do_mag  (&cfg, &cal);
    else if (strcmp(mode, "gyro")  == 0) rc = do_gyro (&cfg, &cal);
    else if (strcmp(mode, "accel") == 0) rc = do_accel(&cfg, &cal);

    if (rc < 0) return 1;

    /* Write if any section was updated */
    if (cal.has_mag || cal.has_gyro || cal.has_accel) {
        if (cal_write(cfg.cal_file, &cal) < 0) return 1;
        printf("Saved to %s\n", cfg.cal_file);
    }

    return 0;
}
