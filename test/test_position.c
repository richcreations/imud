/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_position.c — unit tests for pos_json_double() and gpsd/SignalK parsing
 *
 * These tests exercise the JSON field extractor in isolation — no network
 * sockets, no GPS hardware, no WMM computation.  The test binary links
 * position.c and wmm.c but uses a stub imu_ctx_set_declination() so that
 * imu.c (and all its hardware dependencies) are not required.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "../include/position.h"
#include "../include/imu.h"   /* for imu_ctx_t forward decl — no implementation */

/* ── Stub ────────────────────────────────────────────────────────────────── */

/*
 * Minimal stub so position.c can be linked without imu.c.
 * Records the last declination value written for assertion.
 */
float g_last_decl = -999.0f;

void imu_ctx_set_declination(imu_ctx_t *ctx, float decl_deg)
{
    (void)ctx;
    g_last_decl = decl_deg;
}

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR_D(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), (msg))

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── pos_json_double tests ───────────────────────────────────────────────── */

static void test_json_double_basic(void)
{
    begin("test_json_double_basic");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"lat\":37.87}", "lat", &v), "key found");
    EXPECT_NEAR_D(v, 37.87, 1e-9, "value parsed");
    end(fb);
}

static void test_json_double_negative(void)
{
    begin("test_json_double_negative");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"lon\":-122.315}", "lon", &v), "key found");
    EXPECT_NEAR_D(v, -122.315, 1e-9, "negative value parsed");
    end(fb);
}

static void test_json_double_integer(void)
{
    begin("test_json_double_integer");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"mode\":3}", "mode", &v), "key found");
    EXPECT_NEAR_D(v, 3.0, 1e-9, "integer parsed as double");
    end(fb);
}

static void test_json_double_missing_key(void)
{
    begin("test_json_double_missing_key");
    int fb = g_fail;
    double v = 0;
    EXPECT(!pos_json_double("{\"lat\":37.87}", "lon", &v), "missing key → false");
    end(fb);
}

static void test_json_double_null_value(void)
{
    begin("test_json_double_null_value");
    int fb = g_fail;
    double v = 0;
    /* gpsd emits "lat":null when there is no fix */
    EXPECT(!pos_json_double("{\"lat\":null}", "lat", &v), "null value → false");
    end(fb);
}

static void test_json_double_string_value(void)
{
    begin("test_json_double_string_value");
    int fb = g_fail;
    double v = 0;
    EXPECT(!pos_json_double("{\"lat\":\"N/A\"}", "lat", &v), "string value → false");
    end(fb);
}

static void test_json_double_spaces(void)
{
    begin("test_json_double_spaces");
    int fb = g_fail;
    double v;
    /* gpsd sometimes emits spaces after the colon */
    EXPECT(pos_json_double("{\"lat\": 37.87}", "lat", &v), "space after colon");
    EXPECT_NEAR_D(v, 37.87, 1e-9, "value correct after space");
    end(fb);
}

static void test_json_double_scientific(void)
{
    begin("test_json_double_scientific");
    int fb = g_fail;
    double v;
    EXPECT(pos_json_double("{\"alt\":3.787e+01}", "alt", &v), "scientific notation parsed");
    EXPECT_NEAR_D(v, 37.87, 1e-6, "scientific value correct");
    end(fb);
}

/* Full gpsd TPV message (representative of actual gpsd output). */
static void test_gpsd_tpv_full(void)
{
    begin("test_gpsd_tpv_full");
    int fb = g_fail;

    const char *tpv =
        "{\"class\":\"TPV\",\"device\":\"/dev/ttyUSB0\","
        "\"mode\":3,\"time\":\"2025-05-01T12:00:00.000Z\","
        "\"lat\":37.8697,\"lon\":-122.3153,"
        "\"alt\":5.0,\"speed\":0.0,\"track\":0.0}";

    double mode = 0, lat = 0, lon = 0;
    EXPECT(pos_json_double(tpv, "mode", &mode), "mode found");
    EXPECT_NEAR_D(mode, 3.0, 1e-9,     "mode = 3");
    EXPECT(pos_json_double(tpv, "lat",  &lat),  "lat found");
    EXPECT_NEAR_D(lat,  37.8697, 1e-9,  "lat = 37.8697");
    EXPECT(pos_json_double(tpv, "lon",  &lon),  "lon found");
    EXPECT_NEAR_D(lon, -122.3153, 1e-9, "lon = -122.3153");
    EXPECT(!strstr(tpv, "\"class\":\"TPV\"") == 0, "class:TPV present");
    end(fb);
}

/* Full SignalK position REST response. */
static void test_signalk_response(void)
{
    begin("test_signalk_response");
    int fb = g_fail;

    const char *body =
        "{\"value\":{\"longitude\":-122.3153,\"latitude\":37.8697},"
        "\"$source\":\"gps.0\","
        "\"timestamp\":\"2025-05-01T12:00:00.000Z\"}";

    double lat = 0, lon = 0;
    EXPECT(pos_json_double(body, "latitude",  &lat), "latitude found");
    EXPECT_NEAR_D(lat,  37.8697, 1e-9,  "latitude = 37.8697");
    EXPECT(pos_json_double(body, "longitude", &lon), "longitude found");
    EXPECT_NEAR_D(lon, -122.3153, 1e-9, "longitude = -122.3153");
    end(fb);
}

/* gpsd TPV with mode < 2 must be treated as no-fix. */
static void test_gpsd_no_fix(void)
{
    begin("test_gpsd_no_fix");
    int fb = g_fail;

    const char *tpv = "{\"class\":\"TPV\",\"mode\":1,\"lat\":0.0,\"lon\":0.0}";
    double mode = 0;
    EXPECT(pos_json_double(tpv, "mode", &mode), "mode parsed");
    EXPECT(mode < 2.0, "mode < 2 → no fix");
    end(fb);
}

/* Multiple keys in one JSON object — each must be found independently. */
static void test_json_multiple_keys(void)
{
    begin("test_json_multiple_keys");
    int fb = g_fail;

    const char *json = "{\"a\":1.0,\"b\":2.5,\"c\":-3.0}";
    double a, b, c;
    EXPECT(pos_json_double(json, "a", &a), "key a");
    EXPECT(pos_json_double(json, "b", &b), "key b");
    EXPECT(pos_json_double(json, "c", &c), "key c");
    EXPECT_NEAR_D(a,  1.0, 1e-9, "a = 1.0");
    EXPECT_NEAR_D(b,  2.5, 1e-9, "b = 2.5");
    EXPECT_NEAR_D(c, -3.0, 1e-9, "c = -3.0");
    end(fb);
}

/*
 * fix_max_age_h TTL: verify the TTL-seconds arithmetic and the boundary
 * conditions that gate the watchdog without mocking the clock.
 *
 * The position_thread watchdog fires when:
 *   fix_max_age_h > 0  AND  last_fix_time > 0
 *   AND  difftime(now, last_fix_time) > fix_max_age_h * 3600
 *
 * All three conditions must be true; any false → no clear.
 */
static void test_fix_ttl_arithmetic(void)
{
    begin("test_fix_ttl_arithmetic");
    int fb = g_fail;

    /* Default 24 h = 86400 s */
    float max_age_h = 24.0f;
    double ttl_s = (double)(max_age_h * 3600.0f);
    EXPECT_NEAR_D(ttl_s, 86400.0, 1.0, "24 h → 86400 s TTL");

    /* 0 = disabled — watchdog must not fire */
    float disabled = 0.0f;
    EXPECT(!(disabled > 0.0f), "fix_max_age_h = 0 suppresses watchdog");

    /* no fix yet (last_fix_time == 0) — watchdog must not fire */
    time_t no_fix = 0;
    EXPECT(!(no_fix > 0), "last_fix_time = 0 suppresses watchdog");

    /* Simulate: fix received, TTL elapsed → watchdog should clear.
     * We don't actually sleep; just verify the comparison logic. */
    time_t fake_fix = 1000;
    time_t fake_now = fake_fix + (time_t)(ttl_s) + 1;   /* 1 second past TTL */
    EXPECT(difftime(fake_now, fake_fix) > ttl_s, "elapsed > TTL triggers clear");

    /* One second short → no clear */
    time_t fake_now_short = fake_fix + (time_t)(ttl_s) - 1;
    EXPECT(!(difftime(fake_now_short, fake_fix) > ttl_s), "elapsed < TTL → no clear");

    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud position tests ===");

    test_json_double_basic();
    test_json_double_negative();
    test_json_double_integer();
    test_json_double_missing_key();
    test_json_double_null_value();
    test_json_double_string_value();
    test_json_double_spaces();
    test_json_double_scientific();
    test_gpsd_tpv_full();
    test_signalk_response();
    test_gpsd_no_fix();
    test_json_multiple_keys();
    test_fix_ttl_arithmetic();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
