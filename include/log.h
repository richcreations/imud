/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * log.h — log level gating for imud
 *
 * g_log_level is set from config [logging] level at startup.
 * Use LOG_I / LOG_W / LOG_E in preference to bare fprintf(stderr) for any
 * output that should be suppressible by the operator.
 *
 * Current gating:
 *   error  — only LOG_E output (silences stats and info lines)
 *   warn   — LOG_W and LOG_E
 *   info   — LOG_I, LOG_W, LOG_E  (default)
 *   debug  — all output (LOG_D adds verbose internal tracing when added)
 */

#ifndef IMUD_LOG_H
#define IMUD_LOG_H

#include <stdio.h>

#define LOG_DEBUG  0
#define LOG_INFO   1
#define LOG_WARN   2
#define LOG_ERROR  3

extern int g_log_level;

#define LOG(lvl, ...) \
    do { if ((lvl) >= g_log_level) fprintf(stderr, __VA_ARGS__); } while (0)

#define LOG_D(...)  LOG(LOG_DEBUG, __VA_ARGS__)
#define LOG_I(...)  LOG(LOG_INFO,  __VA_ARGS__)
#define LOG_W(...)  LOG(LOG_WARN,  __VA_ARGS__)
#define LOG_E(...)  LOG(LOG_ERROR, __VA_ARGS__)

#endif /* IMUD_LOG_H */
