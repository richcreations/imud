/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * status_fmt.h — the text imud-status prints
 *
 * The daemon's health thread answers a connection on the AF_UNIX status
 * socket with a plain-text report: chip IDs, rates, fusion state, attitude,
 * declination, sea state, capture, output configuration, counters, uptime,
 * and the recent WARN/ERROR lines.  This is that report as a pure function of
 * a snapshot, so it can be tested — src/main.c does the gathering (imu_get_*,
 * log_recent) and the single write().
 *
 * Same shape as src/prom_metrics.c and src/sk_delta.c: state struct in,
 * bounded text out, no I/O.
 *
 * The report is a human interface, not a wire format: no consumer parses it
 * (libimud and the bridges read the binary stream), so lines may be added or
 * reworded.  Truncation is the invariant that matters — see status_format().
 */

#ifndef IMUD_STATUS_FMT_H
#define IMUD_STATUS_FMT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "config.h"
#include "imu.h"      /* imu_stats_t */
#include "types.h"    /* fused_state_t */

typedef struct {
    const imud_config_t *cfg;
    fused_state_t        state;
    imu_stats_t          stats;
    time_t               uptime_s;

    /* Capture status, only read when cfg->capture_enabled. */
    const char *capture_path;
    uint64_t    capture_bytes;
    uint64_t    capture_drops;
    bool        capture_active;

    const char *recent;    /* recent WARN/ERROR lines; NULL or "" for none */
} status_input_t;

/*
 * Format the report into buf, always NUL-terminating.  Returns the number of
 * bytes written, not counting the NUL — so the caller writes exactly that
 * many.
 *
 * Never writes more than sz - 1 bytes and never overruns, at any sz including
 * 0 and 1: a line that does not fit is truncated and every line after it is
 * dropped, rather than the buffer wrapping or the length running past the end.
 * That bound is what the caller relies on to write() the result blind, and it
 * is the part of this file worth testing.
 */
size_t status_format(char *buf, size_t sz, const status_input_t *in);

#endif /* IMUD_STATUS_FMT_H */
