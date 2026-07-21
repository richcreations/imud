/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bridge.h — shared scaffolding for the bridge daemons
 *
 * The five bridges (imud-signalk, -mqtt, -influxdb, -mavlink, -prometheus)
 * share their startup and main-loop plumbing verbatim: signal handling, CLI
 * parsing, config load, SIGHUP reload, stream connect/retry, and the emit-
 * tick timespec math. This module is that plumbing as plain helper functions
 * — each bridge keeps its own main() and loop shape (prometheus is poll()-
 * driven, mavlink runs two deadlines) and calls in for the identical parts.
 * All log/usage text is parameterized through bridge_info_t so the output of
 * a converted bridge is byte-identical to its hand-rolled predecessor.
 *
 * Bridges-only: bridge.c calls libimud, so this must not be linked into the
 * daemon (sdnotify.c is the libc-only piece the daemon shares).
 */

#ifndef IMUD_BRIDGE_H
#define IMUD_BRIDGE_H

#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include <sys/socket.h>

#include "config.h"
#include "../lib/imud.h"

#ifndef SOCK_CLOEXEC
# define SOCK_CLOEXEC 0
#endif

/* ── Per-bridge identity (parameterizes all shared log/usage text) ───────── */

typedef struct {
    const char *prog;          /* binary name for --version, e.g. "imud-signalk" */
    const char *tag;           /* log prefix body, e.g. "signalk" → "[signalk]" */
    const char *section;       /* config section name, e.g. "imud-signalk" */
    const char *default_conf;  /* default config path */
    const char *usage_desc;    /* usage() description block (indented lines,
                                * each newline-terminated) */
} bridge_info_t;

/* ── Signals ─────────────────────────────────────────────────────────────── */

extern volatile sig_atomic_t bridge_stop;    /* SIGTERM / SIGINT */
extern volatile sig_atomic_t bridge_reload;  /* SIGHUP */

/* SIGTERM/SIGINT → bridge_stop, SIGHUP → bridge_reload, SIGPIPE ignored. */
void bridge_install_signals(void);

/* Sleep `secs` in 1 s steps, waking on stop/reload. */
void bridge_sleep_interruptible(int secs);

/* ── CLI + config ────────────────────────────────────────────────────────── */

/*
 * Parse --config/--version/--help; fills config_path (starts at
 * bi->default_conf). Returns 0 to continue, 1 when handled (--version/--help;
 * caller exits 0), -1 on an unknown option (usage printed; caller exits 1).
 */
int bridge_parse_cli(int argc, char **argv, const bridge_info_t *bi,
                     char *config_path, size_t pathsz);

/*
 * config_defaults + config_load + the daemon-matching logging setup
 * (journal style under systemd, [logging] level). On CONFIG_ERR_PARSE logs
 * the refusing-to-start error and returns -1 (caller exits 1).
 */
int bridge_load_config(const bridge_info_t *bi, const char *path,
                       imud_config_t *cfg);

/* "[tag] disabled in config ([section] enabled = false)" + READY=1.
 * Caller returns 0 so systemd records a clean start and does not restart. */
void bridge_exit_disabled(const bridge_info_t *bi);

/*
 * Consume a pending SIGHUP. Returns 0 when none is pending, and 0 after
 * logging the keep-current warning when the file no longer parses. On a good
 * parse fills *nc, applies its log level, and returns 1 — the caller applies
 * its bridge-specific field updates / restart warnings, assigns cfg = *nc,
 * and calls bridge_reload_done().
 */
int bridge_reload_begin(const bridge_info_t *bi, const char *path,
                        imud_config_t *nc);

/* Log "[tag] config reloaded". */
void bridge_reload_done(const bridge_info_t *bi);

/* ── Emit-tick timing (pure timespec math) ───────────────────────────────── */

/* 1e9/rate_hz, or fallback_ns when rate_hz <= 0. */
long bridge_period_ns(int rate_hz, long fallback_ns);

/* Milliseconds from now until next, rounded up (never negative) so a
 * sub-millisecond remainder can't busy-spin an imud_read() wait. */
int bridge_wait_ms(const struct timespec *now, const struct timespec *next);

/* True once now has reached (or passed) next. */
bool bridge_due(const struct timespec *now, const struct timespec *next);

/* Advance next by period_ns, skipping missed ticks after a stall. */
void bridge_advance(struct timespec *next, const struct timespec *now,
                    long period_ns);

/* The earlier of two deadlines (for multi-deadline loops). */
const struct timespec *bridge_earlier(const struct timespec *a,
                                      const struct timespec *b);

/* ── Stream plumbing ─────────────────────────────────────────────────────── */

/*
 * Ensure *stream is connected. Returns 1 if it already was; 2 on a fresh
 * connect (logged — callers reset their tick deadline on this); 0 when the
 * socket is unavailable (logged, and slept retry_sleep_s interruptibly when
 * > 0 — callers clear their cached packet and continue).
 */
int bridge_stream_ensure(imud_t **stream, const char *sock_path,
                         const char *tag, int retry_sleep_s);

/* Log "[tag] stream disconnected", free and NULL *stream. */
void bridge_stream_drop(imud_t **stream, const char *tag);

/* ── Output transports ───────────────────────────────────────────────────── */

/* Resolve host:port (getaddrinfo) and open a datagram socket; store the
 * destination. SO_BROADCAST is set (harmless for unicast). */
int bridge_open_udp(const char *host, int port, const char *tag,
                    struct sockaddr_storage *dst, socklen_t *dlen);

/* Blocking write of the whole buffer; retries EINTR. 0 ok, -1 error. */
int bridge_write_all(int fd, const char *b, int n);

#endif /* IMUD_BRIDGE_H */
