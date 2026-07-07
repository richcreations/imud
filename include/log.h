/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * log.h — level-gated, thread-safe logging for imud
 *
 * Use LOG_D / LOG_I / LOG_W / LOG_E instead of bare fprintf(stderr) for any
 * output that should be suppressible by the operator via [logging] level.
 * (CLI tools' interactive output — prompts, results tables — stays plain
 * printf: it is the product, not logging.)
 *
 * Gating ([logging] level, hot-reloadable via SIGHUP):
 *   error  — only LOG_E output
 *   warn   — LOG_W and LOG_E
 *   info   — LOG_I, LOG_W, LOG_E  (default)
 *   debug  — everything
 *
 * Output styles (log_set_style, chosen once at daemon startup):
 *   PLAIN   — message only; default for CLI tools and tests
 *   JOURNAL — sd-daemon "<N>" priority prefix so journald records real
 *             priorities (journalctl -p warning works); systemd's
 *             SyslogLevelPrefix (default on) consumes the prefix
 *   FILE    — "YYYY-MM-DD HH:MM:SS [X] " prefix for raw log files
 *
 * Implementation notes (src/log.c):
 *   - each message is fully formatted then emitted with ONE stdio call, so
 *     concurrent threads cannot interleave fragments of a line
 *   - syslog-style repeat suppression: identical consecutive messages are
 *     counted and flushed as "last message repeated N times"
 *   - the most recent WARN/ERROR lines are kept in a small ring for
 *     imud-status ("Recent warnings") via log_recent()
 */

#ifndef IMUD_LOG_H
#define IMUD_LOG_H

#include <stddef.h>

#define LOG_DEBUG  0
#define LOG_INFO   1
#define LOG_WARN   2
#define LOG_ERROR  3

#define LOG_STYLE_PLAIN    0
#define LOG_STYLE_JOURNAL  1
#define LOG_STYLE_FILE     2

extern int g_log_level;

#if defined(__GNUC__) || defined(__clang__)
void log_emit(int lvl, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
#else
void log_emit(int lvl, const char *fmt, ...);
#endif

/* Select the output style (default LOG_STYLE_PLAIN). Not thread-safe by
 * design: call once at startup before threads exist. */
void log_set_style(int style);

/* Set g_log_level from a config string: "debug" | "info" | "warn" | "error".
 * Anything else (including NULL) falls back to info. */
void log_set_level_str(const char *name);

/*
 * Copy the most recent WARN/ERROR lines (oldest first, one per line, each
 * prefixed "  HH:MM:SS X ") into buf. Returns the number of lines copied.
 * Thread-safe.
 */
int log_recent(char *buf, size_t bufsz);

#define LOG(lvl, ...)  log_emit((lvl), __VA_ARGS__)
#define LOG_D(...)  LOG(LOG_DEBUG, __VA_ARGS__)
#define LOG_I(...)  LOG(LOG_INFO,  __VA_ARGS__)
#define LOG_W(...)  LOG(LOG_WARN,  __VA_ARGS__)
#define LOG_E(...)  LOG(LOG_ERROR, __VA_ARGS__)

#endif /* IMUD_LOG_H */
