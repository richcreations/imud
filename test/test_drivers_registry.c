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
 * Needs no --wrap and no gpiod, but it is NOT cross-platform: it links every
 * driver, and each of those reaches <linux/i2c.h> through i2c_io.h, so it
 * builds only on Linux (the container, or CI).  The claim that it "builds and
 * runs on the dev box too" was here and was wrong.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "drivers.h"
#include "imu_math.h"   /* odr_actual_imu / odr_actual_mag / snap_odr_up */
#include "rate_ladder.h" /* the ladder test_fusion walks; guarded below */

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
    "ism330dhcx", "lsm6dso", "lsm6dsox", "icm42688p", "icm20948",
    "mpu9250", "mpu9255", "sim", NULL
};
static const char *mag_names[] = {
    "mmc5983ma", "ak09916", "ak8963", "lis3mdl", "lis2mdl", "sim", NULL
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

/*
 * Every driver must resolve every plausible request to a usable rate.
 *
 * imud hands the resolved rate straight to the driver AND to mekf_init, where
 * it becomes 1/dt and the noise variances, so a zero or negative here is a
 * division by zero in the filter. When the driver takes the NULL default the
 * answer must additionally be one of its own advertised rates — that is the
 * default rule's entire contract, and it is what lets imu.c pass the resolved
 * value back to the driver and get a no-op rounding.
 *
 * The spread deliberately straddles table boundaries and runs off both ends.
 */
static void check_odr_resolution(const int *tab, bool has_hook,
                                 int (*resolve)(const void *, int),
                                 const void *ops, const char *what)
{
    static const int requests[] = {
        1, 7, 12, 13, 60, 99, 100, 137, 225, 500, 833, 900, 1125, 5000, 100000
    };
    for (size_t i = 0; i < sizeof requests / sizeof requests[0]; i++) {
        int got = resolve(ops, requests[i]);
        EXPECT(got > 0, what);
        if (has_hook) continue;   /* divider parts reach off-table rates */
        bool in_table = false;
        for (int j = 0; tab[j] != 0; j++)
            if (tab[j] == got) { in_table = true; break; }
        EXPECT(in_table, what);
        EXPECT(got == snap_odr_up(tab, requests[i]), what);
    }
}

static int resolve_imu(const void *o, int hz)
{
    return odr_actual_imu((const imu_ops_t *)o, hz);
}
static int resolve_mag(const void *o, int hz)
{
    return odr_actual_mag((const mag_ops_t *)o, hz);
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
    check_odr_resolution(o->supported_odr_hz, o->actual_odr_hz != NULL,
                         resolve_imu, o, "imu ODR resolves to a usable rate");
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
    check_odr_resolution(o->supported_odr_hz, o->actual_odr_hz != NULL,
                         resolve_mag, o, "mag ODR resolves to a usable rate");
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_imu_lookups(void)
{
    begin("test_imu_lookups");
    int fb = g_fail;
    int n = 0;
    for (const char **p = imu_names; *p; p++, n++)
        check_imu_ops(imu_driver_find(*p), *p);
    EXPECT(n == 8, "8 IMU drivers registered");
    end(fb);
}

static void test_mag_lookups(void)
{
    begin("test_mag_lookups");
    int fb = g_fail;
    int n = 0;
    for (const char **p = mag_names; *p; p++, n++)
        check_mag_ops(mag_driver_find(*p), *p);
    EXPECT(n == 6, "6 mag drivers registered");
    end(fb);
}

/*
 * The rate ladder in test/rate_ladder.h must cover every rung the drivers
 * actually advertise.
 *
 * test_fusion walks that ladder to exercise the filter's derivations at all 351
 * IMU x mag pairings.  It cannot walk the registry itself: the registries here
 * are static with no enumeration API, and every driver .c reaches <linux/i2c.h>
 * through i2c_io.h, so a test that links them will not build on a non-Linux dev
 * host — and test_fusion has to build everywhere.
 *
 * Hence a duplicated list, and hence this: the duplication is GUARDED, not
 * trusted.  Add a rung to any driver without adding it to rate_ladder.h and
 * this fails here, naming the driver and the rate, in a test that links the
 * real registry and runs in CI.  Do not delete this and leave the list.
 */
static bool ladder_has(const int *ladder, int n, int hz)
{
    for (int i = 0; i < n; i++) if (ladder[i] == hz) return true;
    return false;
}

static void test_rate_ladder_covers_registry(void)
{
    begin("test_rate_ladder_covers_registry");
    int fb = g_fail;

    for (const char **p = imu_names; *p; p++) {
        const imu_ops_t *o = imu_driver_find(*p);
        if (!o) continue;                      /* the lookup tests own this */
        for (int i = 0; o->supported_odr_hz[i]; i++) {
            char msg[128];
            snprintf(msg, sizeof msg,
                     "rate_ladder_imu covers %s %d Hz", *p,
                     o->supported_odr_hz[i]);
            EXPECT(ladder_has(rate_ladder_imu, RATE_LADDER_IMU_N,
                              o->supported_odr_hz[i]), msg);
        }
    }

    for (const char **p = mag_names; *p; p++) {
        const mag_ops_t *o = mag_driver_find(*p);
        if (!o) continue;
        for (int i = 0; o->supported_odr_hz[i]; i++) {
            char msg[128];
            snprintf(msg, sizeof msg,
                     "rate_ladder_mag covers %s %d Hz", *p,
                     o->supported_odr_hz[i]);
            EXPECT(ladder_has(rate_ladder_mag, RATE_LADDER_MAG_N,
                              o->supported_odr_hz[i]), msg);
        }
    }

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
    test_rate_ladder_covers_registry();
    test_unknown_lookups();
    test_validated_not_experimental();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
