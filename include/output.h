/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/* output.h — UDP output helpers and thread startup */

/*
 * output.h — output threads for imud (§7, §8)
 *
 * Four threads:
 *   nmea_out_thread   — NMEA 0183 UDP (port 10110, default 10 Hz)
 *   hirate_out_thread — binary packet UDP (port 10111, default 500 Hz)
 *   json_out_thread   — NDJSON UDP (port 10112, default 100 Hz)
 *   stream_out_thread — binary packets over a local AF_UNIX SOCK_STREAM
 *                       subscription socket (default 100 Hz per subscriber)
 *
 * Each thread is only started when its stream is enabled in config.
 * Pass the same out_ctx_t * as the void * argument to pthread_create.
 */

#ifndef IMUD_OUTPUT_H
#define IMUD_OUTPUT_H

#include "config.h"
#include "imu.h"

/* Opaque context (defined in output.c) */
typedef struct out_ctx out_ctx_t;

/*
 * Validate destination addresses, open UDP sockets, apply socket options.
 * Returns 0 on success, -1 on any fatal error (logged to stderr).
 * Must be called before starting output threads.
 */
int  out_ctx_open(out_ctx_t **ctx_out,
                  const imud_config_t *cfg,
                  imu_ctx_t           *imu);

/*
 * Emit one final high-rate packet with FLAG_SHUTDOWN set.
 * Call before out_ctx_stop so the packet goes out while the socket is open.
 * No-op if the high-rate stream is not configured.
 */
void out_ctx_send_shutdown(out_ctx_t *ctx);

/* Signal output threads to stop. */
void out_ctx_stop(out_ctx_t *ctx);

/* Release sockets and context.  Call after threads have joined. */
void out_ctx_free(out_ctx_t *ctx);

/* Thread entry points */
void *nmea_out_thread  (void *arg);
void *hirate_out_thread(void *arg);
void *json_out_thread  (void *arg);
void *stream_out_thread(void *arg);

#endif /* IMUD_OUTPUT_H */
