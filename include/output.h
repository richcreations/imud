/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/* output.h — UDP output helpers and thread startup */

/*
 * output.h — output threads for imud (§7, §8)
 *
 * Three threads:
 *   nmea_out_thread   — NMEA 0183 UDP (port 10110, default 10 Hz), plus an
 *                       optional TCP listener serving the same bursts
 *   hirate_out_thread — binary packet UDP (port 10111, default 500 Hz)
 *   stream_out_thread — binary packets over a local AF_UNIX SOCK_STREAM
 *                       subscription socket (default 100 Hz per subscriber),
 *                       plus an optional TCP listener (same framed packets)
 *
 * Each thread is only started when its stream is enabled in config (for
 * nmea/stream: when the UDP/AF_UNIX side or its TCP listener is enabled).
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

/*
 * Disown the sockets belonging to an output thread that FAILED TO START.
 *
 * out_ctx_open binds everything before any thread exists, so a pthread_create
 * that fails leaves a listener up with nothing behind it: a client connects
 * successfully into the backlog and then waits forever, because the thread
 * that would accept() is the one that failed.  Silence is the worst
 * diagnostic available; ECONNREFUSED is the honest answer.  (audit N6)
 *
 * Call INSTEAD OF starting the thread, never alongside a running one — these
 * close fds the thread would otherwise own.  Safe before out_ctx_free: the fds
 * are set to -1 and netserv_close tolerates a second call.
 */
void out_ctx_disable_nmea  (out_ctx_t *ctx);   /* UDP fd + [nmea] tcp listener */
void out_ctx_disable_hirate(out_ctx_t *ctx);   /* UDP fd (no listener exists)  */

/* Republish hot-reloadable output rates after a SIGHUP config reload. */
void out_ctx_reload(out_ctx_t *ctx, const imud_config_t *cfg);

/* Release sockets and context.  Call after threads have joined. */
void out_ctx_free(out_ctx_t *ctx);

/* Thread entry points */
void *nmea_out_thread  (void *arg);
void *hirate_out_thread(void *arg);
void *stream_out_thread(void *arg);

#endif /* IMUD_OUTPUT_H */
