/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * log.c — level-gated, thread-safe logging (see include/log.h)
 *
 * All state lives behind one leaf mutex (log_emit takes no other locks, so
 * no lock-order cycle is possible with callers holding their own locks).
 */

#include "log.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <pthread.h>

int g_log_level = LOG_INFO;

static int             g_style = LOG_STYLE_PLAIN;
static pthread_mutex_t g_mtx   = PTHREAD_MUTEX_INITIALIZER;

/* Repeat suppression: last emitted message + count of suppressed copies.
 * A persistent flapper still logs a count line every REPEAT_FLUSH_S. */
#define LOG_LINE_MAX     512
#define REPEAT_FLUSH_S   30
static char   g_last[LOG_LINE_MAX];
static int    g_repeat;
static time_t g_repeat_t;

/* Ring of recent WARN/ERROR lines for imud-status. */
#define RECENT_N    8
#define RECENT_LEN  120
static char g_recent[RECENT_N][RECENT_LEN];
static int  g_recent_head, g_recent_cnt;

void log_set_style(int style)
{
    g_style = style;
}

void log_set_level_str(const char *name)
{
    if      (name && strcmp(name, "debug") == 0) g_log_level = LOG_DEBUG;
    else if (name && strcmp(name, "warn")  == 0) g_log_level = LOG_WARN;
    else if (name && strcmp(name, "error") == 0) g_log_level = LOG_ERROR;
    else                                         g_log_level = LOG_INFO;
}

/* Emit one complete line in the configured style. g_mtx must be held. */
static void emit_line(int lvl, const char *line)
{
    static const int  pri[4]  = { 7, 6, 4, 3 };   /* sd-daemon priorities */
    static const char tag[4]  = { 'D', 'I', 'W', 'E' };

    if (g_style == LOG_STYLE_JOURNAL) {
        fprintf(stderr, "<%d>%s", pri[lvl & 3], line);
    } else if (g_style == LOG_STYLE_FILE) {
        char ts[24];
        time_t now = time(NULL);
        struct tm tm;
        localtime_r(&now, &tm);
        strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", &tm);
        fprintf(stderr, "%s [%c] %s", ts, tag[lvl & 3], line);
    } else {
        fputs(line, stderr);
    }
}

/* Flush a pending repeat count, if any. g_mtx must be held. */
static void flush_repeats(int lvl)
{
    if (g_repeat > 0) {
        char rep[64];
        snprintf(rep, sizeof rep, "last message repeated %d times\n", g_repeat);
        emit_line(lvl, rep);
        g_repeat = 0;
    }
}

void log_emit(int lvl, const char *fmt, ...)
{
    if (lvl < g_log_level) return;

    char line[LOG_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&g_mtx);

    time_t now = time(NULL);
    if (strcmp(line, g_last) == 0) {
        g_repeat++;
        if (now - g_repeat_t >= REPEAT_FLUSH_S) {
            flush_repeats(lvl);
            g_repeat_t = now;
        }
        pthread_mutex_unlock(&g_mtx);
        return;
    }

    flush_repeats(lvl);
    snprintf(g_last, sizeof g_last, "%s", line);
    g_repeat_t = now;

    emit_line(lvl, line);

    if (lvl >= LOG_WARN) {
        char *slot = g_recent[g_recent_head];
        char ts[12];
        struct tm tm;
        localtime_r(&now, &tm);
        strftime(ts, sizeof ts, "%H:%M:%S", &tm);
        snprintf(slot, RECENT_LEN, "%s %c %s",
                 ts, (lvl >= LOG_ERROR) ? 'E' : 'W', line);
        size_t n = strlen(slot);
        if (n > 0 && slot[n-1] == '\n') slot[n-1] = '\0';
        g_recent_head = (g_recent_head + 1) % RECENT_N;
        if (g_recent_cnt < RECENT_N) g_recent_cnt++;
    }

    pthread_mutex_unlock(&g_mtx);
}

int log_recent(char *buf, size_t bufsz)
{
    if (!buf || bufsz == 0) return 0;
    buf[0] = '\0';

    pthread_mutex_lock(&g_mtx);
    int    lines = 0;
    size_t off   = 0;
    for (int i = 0; i < g_recent_cnt; i++) {
        int idx = (g_recent_head - g_recent_cnt + i + RECENT_N) % RECENT_N;
        int r = snprintf(buf + off, bufsz - off, "  %s\n", g_recent[idx]);
        if (r < 0 || (size_t)r >= bufsz - off) break;
        off += (size_t)r;
        lines++;
    }
    pthread_mutex_unlock(&g_mtx);
    return lines;
}
