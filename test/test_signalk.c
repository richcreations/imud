/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_signalk.c — unit tests for the Signal K delta encoder (src/sk_delta.c)
 *
 * Drives sk_build_delta() with crafted imud packets and asserts on the emitted
 * JSON: which paths appear under which conditions, SI-unit conversions, the
 * attitude sign convention, timestamp shape, and structural well-formedness.
 * No sockets — the encoder is a pure function.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "sk_delta.h"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* Extract the numeric value that follows "path":...,"value": for `path`. */
static int find_value(const char *json, const char *path, double *out)
{
    char needle[96];
    snprintf(needle, sizeof needle, "\"path\":\"%s\",\"value\":", path);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p += strlen(needle);
    *out = strtod(p, NULL);
    return 1;
}

/* Brace/bracket balance check — 0 at end, never negative = well-formed nesting. */
static int braces_balanced(const char *s)
{
    int depth = 0;
    for (const char *p = s; *p; p++) {
        if (*p == '{' || *p == '[') depth++;
        else if (*p == '}' || *p == ']') { depth--; if (depth < 0) return 0; }
    }
    return depth == 0;
}

/* A packet with a fixed, known timestamp (2021-05-06T13:33:20.123Z). */
static imud_packet_t make_pkt(void)
{
    imud_packet_t p;
    memset(&p, 0, sizeof p);
    p.magic       = IMUD_MAGIC;
    p.version     = IMUD_VERSION;
    p.ts_wall_ns  = 1620307999123000000ULL + 1000000ULL;  /* ...123 ms */
    p.heading_deg = 90.0f;      /* → π/2 rad */
    p.rate_of_turn = 60.0f;     /* deg/min → 0.0174533 rad/s */
    p.roll  = 0.10f;            /* rad */
    p.pitch = -0.05f;           /* rad */
    p.yaw   = 1.23f;            /* rad */
    p.heave_m = 0.42f;
    return p;
}

static void test_core_paths_and_units(void)
{
    begin("test_core_paths_and_units");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[512];
    int n = sk_build_delta(buf, sizeof buf, &p, "imud", false);
    EXPECT(n > 0, "encode succeeds");
    EXPECT(braces_balanced(buf), "JSON braces/brackets balanced");
    EXPECT(strstr(buf, "\"source\":{\"label\":\"imud\"}") != NULL, "source label present");

    double v;
    EXPECT(find_value(buf, "navigation.headingMagnetic", &v), "headingMagnetic present");
    EXPECT(fabs(v - M_PI/2.0) < 1e-4, "headingMagnetic 90° → π/2 rad");

    EXPECT(find_value(buf, "navigation.rateOfTurn", &v), "rateOfTurn present");
    EXPECT(fabs(v - 60.0*(M_PI/180.0)/60.0) < 1e-6, "rateOfTurn 60 dpm → rad/s");

    end(fb);
}

static void test_attitude_object_and_sign(void)
{
    begin("test_attitude_object_and_sign");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[512];
    sk_build_delta(buf, sizeof buf, &p, "imud", false);

    /* attitude is a compound object */
    EXPECT(strstr(buf, "\"path\":\"navigation.attitude\",\"value\":{\"roll\":") != NULL,
           "attitude emitted as {roll,pitch,yaw} object");
    /* roll is negated to the SK convention; pitch/yaw pass through */
    const char *a = strstr(buf, "\"navigation.attitude\"");
    double roll = 0, pitch = 0, yaw = 0;
    if (a) {
        const char *rp = strstr(a, "\"roll\":");   if (rp)  roll  = strtod(rp + 7, NULL);
        const char *pp = strstr(a, "\"pitch\":");  if (pp)  pitch = strtod(pp + 8, NULL);
        const char *yp = strstr(a, "\"yaw\":");    if (yp)  yaw   = strtod(yp + 6, NULL);
    }
    EXPECT(fabs(roll - (-0.10)) < 1e-4,  "roll negated (imud +stbd-up → SK +stbd-down)");
    EXPECT(fabs(pitch - (-0.05)) < 1e-4, "pitch passed through");
    EXPECT(fabs(yaw - 1.23) < 1e-4,      "yaw passed through");
    end(fb);
}

static void test_declination_gated(void)
{
    begin("test_declination_gated");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[512];
    double v;

    /* Without the flag: no true heading, no variation. */
    p.flags = 0;
    p.declination_deg = 13.2f;
    sk_build_delta(buf, sizeof buf, &p, "imud", false);
    EXPECT(!find_value(buf, "navigation.headingTrue", &v),
           "headingTrue absent without DECLINATION_VALID");
    EXPECT(!find_value(buf, "navigation.magneticVariation", &v),
           "magneticVariation absent without DECLINATION_VALID");

    /* With the flag: both present, variation in radians (E+). */
    p.flags = IMUD_FLAG_DECLINATION_VALID;
    sk_build_delta(buf, sizeof buf, &p, "imud", false);
    EXPECT(find_value(buf, "navigation.magneticVariation", &v), "variation present");
    EXPECT(fabs(v - 13.2*(M_PI/180.0)) < 1e-4, "variation 13.2°E → rad (E positive)");
    EXPECT(find_value(buf, "navigation.headingTrue", &v), "headingTrue present");
    /* 90 mag + 13.2 E = 103.2° true → rad */
    EXPECT(fabs(v - 103.2*(M_PI/180.0)) < 1e-4, "headingTrue = mag + declination");
    end(fb);
}

static void test_heave_gated(void)
{
    begin("test_heave_gated");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[512];
    double v;

    sk_build_delta(buf, sizeof buf, &p, "imud", false);
    EXPECT(!find_value(buf, "environment.heave", &v), "heave absent when emit_heave=false");

    sk_build_delta(buf, sizeof buf, &p, "imud", true);
    EXPECT(find_value(buf, "environment.heave", &v), "heave present when emit_heave=true");
    EXPECT(fabs(v - 0.42) < 1e-3, "heave passed through in metres");
    end(fb);
}

static void test_timestamp_format(void)
{
    begin("test_timestamp_format");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char buf[512];
    sk_build_delta(buf, sizeof buf, &p, "imud", false);

    const char *t = strstr(buf, "\"timestamp\":\"");
    EXPECT(t != NULL, "timestamp field present");
    if (t) {
        t += strlen("\"timestamp\":\"");
        /* YYYY-MM-DDThh:mm:ss.sssZ */
        int ok = isdigit((unsigned char)t[0]) && t[2]=='2' && t[4]=='-' &&
                 t[7]=='-' && t[10]=='T' && t[13]==':' && t[16]==':' &&
                 t[19]=='.' && t[23]=='Z';
        EXPECT(ok, "timestamp is ISO-8601 UTC with milliseconds");
        /* ms field derives from ts_wall_ns: (…124000000 ns) → .124Z */
        EXPECT(strncmp(t + 20, "124Z", 4) == 0, "millisecond field matches ts_wall_ns");
    }
    end(fb);
}

static void test_buffer_too_small(void)
{
    begin("test_buffer_too_small");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    char small[32];
    EXPECT(sk_build_delta(small, sizeof small, &p, "imud", true) == -1,
           "returns -1 when buffer too small");
    end(fb);
}

int main(void)
{
    puts("=== imud signalk delta tests ===");
    test_core_paths_and_units();
    test_attitude_object_and_sign();
    test_declination_gated();
    test_heave_gated();
    test_timestamp_format();
    test_buffer_too_small();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
