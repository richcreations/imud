/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_log.c — unit tests for the logging core (src/log.c)
 *
 * stderr is redirected to a temp file around each emission burst so the
 * produced bytes can be asserted exactly. The test's own PASS/FAIL output
 * goes to stdout only.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include "../include/log.h"

#define TMPLOG "/tmp/imud_test_log.txt"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("\n  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── stderr capture helpers ──────────────────────────────────────────────── */

static int g_saved_stderr = -1;

static void capture_begin(void)
{
    fflush(stderr);
    if (g_saved_stderr < 0) g_saved_stderr = dup(STDERR_FILENO);
    FILE *f = freopen(TMPLOG, "w", stderr);
    (void)f;
}

/* Ends capture and returns the captured text in a static buffer. */
static const char *capture_end(void)
{
    static char buf[4096];
    fflush(stderr);
    dup2(g_saved_stderr, STDERR_FILENO);

    FILE *f = fopen(TMPLOG, "r");
    size_t n = f ? fread(buf, 1, sizeof buf - 1, f) : 0;
    if (f) fclose(f);
    buf[n] = '\0';
    return buf;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_level_gating(void)
{
    begin("test_level_gating");
    int fb = g_fail;

    log_set_style(LOG_STYLE_PLAIN);
    log_set_level_str("warn");

    capture_begin();
    LOG_I("gate-info line\n");
    LOG_W("gate-warn line\n");
    LOG_E("gate-error line\n");
    const char *out = capture_end();

    EXPECT(strstr(out, "gate-info")  == NULL, "info suppressed at warn level");
    EXPECT(strstr(out, "gate-warn")  != NULL, "warn passes at warn level");
    EXPECT(strstr(out, "gate-error") != NULL, "error passes at warn level");

    log_set_level_str("info");
    end(fb);
}

static void test_plain_style_exact(void)
{
    begin("test_plain_style_exact");
    int fb = g_fail;

    log_set_style(LOG_STYLE_PLAIN);
    capture_begin();
    LOG_I("[test] plain %d/%s\n", 7, "seven");
    const char *out = capture_end();

    EXPECT(strcmp(out, "[test] plain 7/seven\n") == 0,
           "PLAIN output is byte-identical to the formatted message");
    end(fb);
}

static void test_journal_style(void)
{
    begin("test_journal_style");
    int fb = g_fail;

    log_set_style(LOG_STYLE_JOURNAL);
    capture_begin();
    LOG_I("journal info\n");
    LOG_W("journal warn\n");
    LOG_E("journal err\n");
    const char *out = capture_end();
    log_set_style(LOG_STYLE_PLAIN);

    EXPECT(strstr(out, "<6>journal info\n") != NULL, "info gets <6> prefix");
    EXPECT(strstr(out, "<4>journal warn\n") != NULL, "warn gets <4> prefix");
    EXPECT(strstr(out, "<3>journal err\n")  != NULL, "error gets <3> prefix");
    end(fb);
}

static void test_file_style(void)
{
    begin("test_file_style");
    int fb = g_fail;

    log_set_style(LOG_STYLE_FILE);
    capture_begin();
    LOG_W("file-style line\n");
    const char *out = capture_end();
    log_set_style(LOG_STYLE_PLAIN);

    /* "YYYY-MM-DD HH:MM:SS [W] file-style line\n" */
    EXPECT(strlen(out) > 24 && isdigit((unsigned char)out[0]) &&
           out[4] == '-' && out[7] == '-' && out[10] == ' ' &&
           out[13] == ':' && out[16] == ':',
           "FILE style starts with a timestamp");
    EXPECT(strstr(out, " [W] file-style line\n") != NULL,
           "FILE style carries level tag and message");
    end(fb);
}

static void test_level_str_parsing(void)
{
    begin("test_level_str_parsing");
    int fb = g_fail;

    log_set_level_str("debug"); EXPECT(g_log_level == LOG_DEBUG, "debug");
    log_set_level_str("info");  EXPECT(g_log_level == LOG_INFO,  "info");
    log_set_level_str("warn");  EXPECT(g_log_level == LOG_WARN,  "warn");
    log_set_level_str("error"); EXPECT(g_log_level == LOG_ERROR, "error");
    log_set_level_str("bogus"); EXPECT(g_log_level == LOG_INFO,  "bogus → info");
    log_set_level_str(NULL);    EXPECT(g_log_level == LOG_INFO,  "NULL → info");
    end(fb);
}

static void test_repeat_suppression(void)
{
    begin("test_repeat_suppression");
    int fb = g_fail;

    log_set_style(LOG_STYLE_PLAIN);
    capture_begin();
    for (int i = 0; i < 5; i++)
        LOG_W("flapping error X\n");
    LOG_W("different message Y\n");
    const char *out = capture_end();

    /* First occurrence emitted, 4 repeats suppressed, count flushed when
     * the next distinct message arrives. */
    int occurrences = 0;
    for (const char *p = out; (p = strstr(p, "flapping error X")); p++)
        occurrences++;
    EXPECT(occurrences == 1, "identical message emitted once");
    EXPECT(strstr(out, "last message repeated 4 times") != NULL,
           "suppressed count flushed on message change");
    EXPECT(strstr(out, "different message Y") != NULL,
           "next distinct message emitted");
    end(fb);
}

static void test_recent_ring(void)
{
    begin("test_recent_ring");
    int fb = g_fail;

    log_set_style(LOG_STYLE_PLAIN);
    capture_begin();
    LOG_I("ring info — must NOT be captured\n");
    LOG_W("ring warn A\n");
    LOG_E("ring error B\n");
    capture_end();

    char buf[2048];
    int n = log_recent(buf, sizeof buf);
    EXPECT(n >= 2, "at least the two W/E lines captured");
    EXPECT(strstr(buf, "ring warn A")  != NULL, "warn line in ring");
    EXPECT(strstr(buf, "ring error B") != NULL, "error line in ring");
    EXPECT(strstr(buf, "must NOT be captured") == NULL, "info not captured");
    char *a = strstr(buf, "ring warn A");
    char *b = strstr(buf, "ring error B");
    EXPECT(a && b && a < b, "oldest first");
    EXPECT(strstr(buf, " W ring warn A") != NULL, "level tag in ring line");
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud log tests ===");

    test_level_gating();
    test_plain_style_exact();
    test_journal_style();
    test_file_style();
    test_level_str_parsing();
    test_repeat_suppression();
    test_recent_ring();

    remove(TMPLOG);
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
