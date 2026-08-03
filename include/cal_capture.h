/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cal_capture.h — .imucap loading for imud-cal's offline analysis modes
 *
 * `imud-cal characterize` and `imud-cal fit-temp` both start by reading an
 * .imucap recording into per-axis double arrays: count the IMU records and
 * measure their span, then block-average into arrays of at most max_samples
 * entries, skipping a startup settle window.  That is a file parser with
 * arithmetic in it, and it lived inside cal_main.c where nothing could reach
 * it (audit.md L2).
 *
 * Both functions are silent.  The originals printed their own progress and
 * warnings; those messages now come from cal_main.c, driven by
 * cal_capture_stats_t and by cal_capture_motion_ok()'s return — which is also
 * what makes the stationarity decision assertable, since in the original the
 * decision and the warning were the same statement.
 */

#ifndef IMUD_CAL_CAPTURE_H
#define IMUD_CAL_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>

/* Default cap on the post-decimation sample count, per axis (2^22).  An
 * overnight record at 833 Hz is ~30 M samples; block-averaging down to this
 * leaves the Allan deviation at the surviving cluster times untouched, because
 * theta at a block boundary is exact. */
#define CAP_ANALYZE_MAX (1u << 22)

typedef struct {
    size_t count;    /* IMU records in the file, before decimation */
    double span_s;   /* first to last IMU record */
    double fs_raw;   /* sample rate as recorded */
    double fs_out;   /* fs_raw / decim — the rate of the returned arrays */
    size_t decim;    /* block-average factor; 1 when no decimation was needed */
    size_t n_skip;   /* records dropped by the settle window */
    size_t n_out;    /* samples written per axis (the return value) */
} cal_capture_stats_t;

/*
 * Load the IMU records of the .imucap at `path`.
 *
 * On success allocates gyro[0..2], accel[0..2] and *temp — free them with
 * cal_capture_free() — and returns the per-axis sample count.  Returns 0 when
 * the file holds too little IMU data to analyse (fewer than 16 records, or no
 * time span), and -1 on error, with *cap_rc set to the cap_reader_open() code
 * so the caller can distinguish "not an .imucap" from an errno failure.
 * *st is filled on the 0 and positive returns.
 *
 * settle_sec discards that much data from the start of the record, matching
 * the daemon's startup_settle_sec: the gyro needs a few seconds after power-on
 * and those samples both trip the motion gate and inflate the Allan noise
 * floor.  max_samples caps the output length; 0 selects CAP_ANALYZE_MAX.
 * The parameter exists so the decimation arithmetic is testable without
 * writing four million records.
 *
 * Every out-pointer is left safe to pass to cal_capture_free() on any return.
 */
long cal_capture_load(const char *path, double settle_sec, size_t max_samples,
                      double *gyro[3], double *accel[3], double **temp,
                      cal_capture_stats_t *st, int *cap_rc);

void cal_capture_free(double *gyro[3], double *accel[3], double *temp);

/*
 * True when the record looks stationary — both offline analyses assume it.
 * Always fills *gyro_pp (peak-to-peak over all three gyro axes) and
 * *accel_std (standard deviation of |a|), so the caller can report the numbers
 * whichever way the decision went.  n must be > 0.
 */
bool cal_capture_motion_ok(double *gyro[3], double *accel[3], size_t n,
                           double *gyro_pp, double *accel_std);

#endif /* IMUD_CAL_CAPTURE_H */
