/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_drivers_registry.c — registry lookup + ops-struct sanity for every
 * registered driver (src/drivers.c and every chip under src/drivers).
 *
 * The real drivers are never functionally unit-tested off-hardware, so a
 * mistake in an ops struct — a null function pointer, an unsorted or
 * unterminated ODR/scale table, a has_hw_timestamp with ts_tick_ns==0 —
 * would only surface on a wired-up Pi.  This test validates the descriptor
 * tables and the registry lookups without touching any hardware: it reads
 * struct fields and calls imu_driver_find / mag_driver_find only.  It never
 * invokes probe/init/read, so no I2C fd is required.
 *
 * Cross-platform (no --wrap, no gpiod): builds and runs on the dev box too.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "drivers.h"

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* Every registered driver's config name.  Kept in sync with drivers.c by
 * the lookup tests below (an added chip that is not listed here goes
 * untested, but the count assertions catch a removed one). */
static const char *imu_names[] = {
    "ism330dhcx", "lsm6dso", "lsm6dsox", "icm42688p", "icm20948", "sim", NULL
};
static const char *mag_names[] = {
    "mmc5983ma", "ak09916", "lis3mdl", "lis2mdl", "sim", NULL
};

/* ── Shared table validator ──────────────────────────────────────────────── */

/* A supported_* table must hold >=1 entry, be strictly ascending, and carry
 * a 0 terminator within its declared capacity. */
static void check_table(const int *tab, int cap, const char *what)
{
    int n = 0;
    while (n < cap && tab[n] != 0) n++;
    EXPECT(n >= 1, what);                 /* at least one supported value */
    EXPECT(n < cap, what);                /* 0 terminator present, not overrun */
    for (int i = 1; i < n; i++)
        EXPECT(tab[i] > tab[i - 1], what); /* strictly ascending */
}

/* ── Per-struct invariants ───────────────────────────────────────────────── */

static void check_imu_ops(const imu_ops_t *o, const char *key)
{
    EXPECT(o != NULL, key);
    if (!o) return;
    EXPECT(o->name && strcmp(o->name, key) == 0, "imu name matches lookup key");
    EXPECT(o->probe != NULL, "imu probe non-NULL");
    EXPECT(o->reset != NULL, "imu reset non-NULL");
    EXPECT(o->init  != NULL, "imu init non-NULL");
    EXPECT(o->read  != NULL, "imu read non-NULL");
    check_table(o->supported_odr_hz,   16, "imu supported_odr_hz");
    check_table(o->supported_accel_g,   8, "imu supported_accel_g");
    check_table(o->supported_gyro_dps,  8, "imu supported_gyro_dps");
    /* A chip advertising a hardware timer must give its tick period, or the
     * chip_to_wall offset math silently collapses to zero. */
    if (o->has_hw_timestamp)
        EXPECT(o->ts_tick_ns != 0, "has_hw_timestamp implies ts_tick_ns != 0");
}

static void check_mag_ops(const mag_ops_t *o, const char *key)
{
    EXPECT(o != NULL, key);
    if (!o) return;
    EXPECT(o->name && strcmp(o->name, key) == 0, "mag name matches lookup key");
    EXPECT(o->probe != NULL, "mag probe non-NULL");
    EXPECT(o->reset != NULL, "mag reset non-NULL");
    EXPECT(o->init  != NULL, "mag init non-NULL");
    EXPECT(o->read  != NULL, "mag read non-NULL");
    check_table(o->supported_odr_hz, 16, "mag supported_odr_hz");
    /* A coil-bearing chip must expose the SET-pulse op it claims. */
    if (o->has_set_reset)
        EXPECT(o->set_reset != NULL, "has_set_reset implies set_reset != NULL");
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_imu_lookups(void)
{
    begin("test_imu_lookups");
    int fb = g_fail;
    int n = 0;
    for (const char **p = imu_names; *p; p++, n++)
        check_imu_ops(imu_driver_find(*p), *p);
    EXPECT(n == 6, "6 IMU drivers registered");
    end(fb);
}

static void test_mag_lookups(void)
{
    begin("test_mag_lookups");
    int fb = g_fail;
    int n = 0;
    for (const char **p = mag_names; *p; p++, n++)
        check_mag_ops(mag_driver_find(*p), *p);
    EXPECT(n == 5, "5 mag drivers registered");
    end(fb);
}

static void test_unknown_lookups(void)
{
    begin("test_unknown_lookups");
    int fb = g_fail;
    EXPECT(imu_driver_find("bogus") == NULL, "unknown IMU name -> NULL");
    EXPECT(imu_driver_find("")      == NULL, "empty IMU name -> NULL");
    EXPECT(mag_driver_find("bogus") == NULL, "unknown mag name -> NULL");
    EXPECT(mag_driver_find("")      == NULL, "empty mag name -> NULL");
    /* Registries must not cross-resolve: a mag chip is not an IMU. */
    EXPECT(imu_driver_find("mmc5983ma") == NULL, "mag name not in IMU registry");
    EXPECT(mag_driver_find("ism330dhcx") == NULL, "IMU name not in mag registry");
    end(fb);
}

/* The two hardware-validated production drivers must not be flagged
 * experimental (the flag prints a "not validated on hardware" warning). */
static void test_validated_not_experimental(void)
{
    begin("test_validated_not_experimental");
    int fb = g_fail;
    const imu_ops_t *ism = imu_driver_find("ism330dhcx");
    const mag_ops_t *mmc = mag_driver_find("mmc5983ma");
    EXPECT(ism && !ism->experimental, "ism330dhcx not experimental");
    EXPECT(mmc && !mmc->experimental, "mmc5983ma not experimental");
    /* sim is a pure software driver — also never experimental. */
    const imu_ops_t *simi = imu_driver_find("sim");
    const mag_ops_t *simm = mag_driver_find("sim");
    EXPECT(simi && !simi->experimental, "sim IMU not experimental");
    EXPECT(simm && !simm->experimental, "sim mag not experimental");
    end(fb);
}

int main(void)
{
    puts("=== imud driver registry tests ===");

    test_imu_lookups();
    test_mag_lookups();
    test_unknown_lookups();
    test_validated_not_experimental();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
