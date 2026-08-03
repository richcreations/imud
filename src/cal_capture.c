/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cal_capture.c — .imucap loading for imud-cal's offline analysis modes
 *
 * Moved out of cal_main.c so it can be tested; see include/cal_capture.h.
 * The arithmetic is unchanged; the printing moved to the caller.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cal_capture.h"
#include "capture.h"

void cal_capture_free(double *gyro[3], double *accel[3], double *temp)
{
    for (int a = 0; a < 3; a++) {
        free(gyro[a]);  gyro[a]  = NULL;
        free(accel[a]); accel[a] = NULL;
    }
    free(temp);
}

long cal_capture_load(const char *path, double settle_sec, size_t max_samples,
                      double *gyro[3], double *accel[3], double **temp,
                      cal_capture_stats_t *st, int *cap_rc)
{
    /* Null everything first: the caller may hand these straight to
     * cal_capture_free() on any error return. */
    for (int a = 0; a < 3; a++) { gyro[a] = NULL; accel[a] = NULL; }
    *temp = NULL;
    memset(st, 0, sizeof *st);
    *cap_rc = 0;

    if (max_samples == 0) max_samples = CAP_ANALYZE_MAX;

    cap_reader_t r;
    int rc = cap_reader_open(&r, path);
    if (rc != 0) { *cap_rc = rc; return -1; }

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
    st->count = count;
    if (count < 16 || last_mono <= first_mono) {
        cap_reader_close(&r);
        return 0;
    }

    double span_s = (double)(last_mono - first_mono) * 1e-9;
    double fs_raw = (double)(count - 1) / span_s;
    size_t decim  = count / max_samples + 1;
    size_t n      = count / decim;

    st->span_s = span_s;
    st->fs_raw = fs_raw;
    st->decim  = decim;
    st->fs_out = fs_raw / (double)decim;

    for (int a = 0; a < 3; a++) {
        gyro[a]  = calloc(n, sizeof(double));
        accel[a] = calloc(n, sizeof(double));
    }
    *temp = calloc(n, sizeof(double));
    if (!gyro[0] || !gyro[1] || !gyro[2] ||
        !accel[0] || !accel[1] || !accel[2] || !*temp) {
        cal_capture_free(gyro, accel, *temp);
        *temp = NULL;
        cap_reader_close(&r);
        return -1;
    }

    /* Pass 2: block-average into the arrays, skipping the settle window. */
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

    st->n_skip = n_skip;
    st->n_out  = out;
    return (long)out;
}

bool cal_capture_motion_ok(double *gyro[3], double *accel[3], size_t n,
                           double *gyro_pp, double *accel_std)
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
    double avar  = asum2 / (double)n - amean * amean;
    double astd  = sqrt(avar > 0.0 ? avar : 0.0);   /* rounding can go negative */

    *gyro_pp   = gmax - gmin;
    *accel_std = astd;

    return !(*gyro_pp > 0.1 || astd > 0.2);
}
