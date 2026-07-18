/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * position.h — background position source thread (gpsd / SignalK)
 *
 * When either source is enabled in [position] config, a single
 * position_thread() runs alongside the fusion threads.  It connects to
 * gpsd (persistent TCP JSON stream) or polls the SignalK REST API, and
 * calls imu_ctx_set_declination() whenever the vessel's position changes
 * enough to warrant a WMM recompute (≥ 0.05° ≈ 5 km).
 *
 * Priority:
 *   gpsd (streaming, preferred) > SignalK (polled fallback) > static config
 *
 * When gpsd is enabled but unavailable, SignalK is polled during the retry
 * interval so position stays current.
 */

#ifndef IMUD_POSITION_H
#define IMUD_POSITION_H

#include <signal.h>
#include <stdbool.h>
#include <time.h>
#include "config.h"

/* Forward declaration — keeps position.h independent of imu.h internals. */
typedef struct imu_ctx imu_ctx_t;

typedef struct {
    const imud_config_t  *cfg;   /* daemon config (read-only after start) */
    imu_ctx_t            *imu;   /* target for imu_ctx_set_declination() */
    char                 wmm_file[256]; /* private copy: the shared cfg's
                                  * pos_wmm_file is [hot] (SIGHUP writer), and
                                  * this thread reads it only once at start */
    _Atomic sig_atomic_t stop;   /* set to 1 to request thread exit */
} pos_ctx_t;

/*
 * position_thread — background thread entry point.
 *
 * Pass a pos_ctx_t * as `arg`.  Returns NULL when ctx->stop is set.
 * If neither gpsd_enabled nor signalk_enabled, returns immediately.
 */
void *position_thread(void *arg);

/*
 * pos_json_double — extract a numeric field from a JSON string.
 *
 * Finds the first occurrence of "key":VALUE and converts VALUE with strtod.
 * Works on any nesting depth; does not require a full JSON parser.
 * Exposed here for unit tests.
 *
 * Returns true and writes *out on success; returns false if the key is absent
 * or the value is not a number (e.g. null, string).
 */
bool pos_json_double(const char *json, const char *key, double *out);

/*
 * pos_fix_valid — finiteness + range gate for a position fix
 * (lat ∈ [-90,90], lon ∈ [-180,180], alt finite).  Exposed for unit tests.
 */
bool pos_fix_valid(double lat, double lon, double alt);

/*
 * check_fix_ttl — clear declination when the last GPS fix is older than
 * cfg->pos_fix_max_age_h hours.  On expiry it zeroes the declination via
 * imu_ctx_set_declination(), zeroes *last_fix_time, and resets the
 * last_lat/last_lon outputs to the force-update sentinel so the next valid fix
 * recomputes declination immediately.  No-op when the TTL is disabled
 * (fix_max_age_h = 0), no fix was received yet, or the fix is fresh.
 * Exposed here for unit tests.
 */
void check_fix_ttl(pos_ctx_t *ctx, time_t *last_fix_time,
                   double *last_lat, double *last_lon);

#endif /* IMUD_POSITION_H */
