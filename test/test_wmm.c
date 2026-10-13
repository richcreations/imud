/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_wmm.c — unit tests for src/wmm.c
 *
 * Validates wmm_load() and wmm_declination() against official NOAA
 * WMM2025 test values (WMM2025_TestValues.txt, released 2024-12-17).
 *
 * Test cases use:
 *   Field 1: decimal year
 *   Field 2: altitude km → m
 *   Field 3: geodetic latitude (deg)
 *   Field 4: geodetic longitude (deg)
 *   Field 5: expected declination (deg)
 *
 * Tolerance: ±0.05° — well within navigator accuracy requirements.
 * Failure at this tolerance indicates a coding error in the spherical
 * harmonic evaluation, not floating-point noise.
 */

#include "../include/wmm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WMM_COF_PATH "data/WMM.COF"
#define TOL_DEG      0.05

static int passed = 0;
static int failed = 0;

static void check(const char *label,
                  double year, double alt_km,
                  double lat, double lon,
                  double expected_decl,
                  const wmm_t *wmm)
{
    double got = wmm_declination(lat, lon, alt_km * 1000.0, year, wmm);
    double err = fabs(got - expected_decl);
    if (err <= TOL_DEG) {
        printf("  PASS  %-40s  got %+7.3f°  exp %+7.3f°  err %.4f°\n",
               label, got, expected_decl, err);
        passed++;
    } else {
        printf("  FAIL  %-40s  got %+7.3f°  exp %+7.3f°  err %.4f°  (tol %.2f°)\n",
               label, got, expected_decl, err, TOL_DEG);
        failed++;
    }
}

int main(void)
{
    printf("=== test_wmm ===\n");

    /* ── Load coefficient file ─────────────────────────────────────────── */
    wmm_t wmm;
    if (wmm_load(WMM_COF_PATH, &wmm) != 0) {
        fprintf(stderr, "FATAL: could not load %s\n", WMM_COF_PATH);
        return 1;
    }
    printf("Loaded %s  epoch=%.1f\n", WMM_COF_PATH, wmm.epoch);

    /* ── Official NOAA WMM2025 test values (subset) ───────────────────── */
    /* year     alt_km   lat     lon     decl    label */
    check("2025.0 28km  89N -121E",  2025.0, 28,  89.0, -121.0,  -99.77, &wmm);
    check("2025.0 48km  80N  -96E",  2025.0, 48,  80.0,  -96.0,  -29.91, &wmm);
    check("2025.0 54km  82N   87E",  2025.0, 54,  82.0,   87.0,   54.89, &wmm);
    check("2025.0 65km  43N   93E",  2025.0, 65,  43.0,   93.0,    0.50, &wmm);
    check("2025.0 51km -33N  109E",  2025.0, 51, -33.0,  109.0,   -5.49, &wmm);
    check("2025.0 39km -59N   -8E",  2025.0, 39, -59.0,   -8.0,  -15.75, &wmm);
    check("2025.0  3km -50N -103E",  2025.0,  3, -50.0, -103.0,   27.96, &wmm);
    check("2025.0 66km  14N  143E",  2025.0, 66,  14.0,  143.0,   -0.19, &wmm);
    check("2025.0 18km   0N   21E",  2025.0, 18,   0.0,   21.0,    1.29, &wmm);
    check("2025.5  6km -36N -137E",  2025.5,  6, -36.0, -137.0,   20.28, &wmm);
    check("2025.5 63km  26N   81E",  2025.5, 63,  26.0,   81.0,    0.51, &wmm);
    check("2025.5 44km  33N -118E",  2025.5, 44,  33.0, -118.0,   11.10, &wmm);
    check("2026.0 74km -57N    3E",  2026.0, 74, -57.0,    3.0,  -22.51, &wmm);
    check("2026.0 46km -24N -122E",  2026.0, 46, -24.0, -122.0,   14.01, &wmm);
    check("2026.0 69km  23N   63E",  2026.0, 69,  23.0,   63.0,    1.17, &wmm);
    check("2027.0 37km -66N   -5E",  2027.0, 37, -66.0,   -5.0,  -17.22, &wmm);
    check("2027.0 44km  22N  174E",  2027.0, 44,  22.0,  174.0,    6.46, &wmm);
    check("2028.0 49km  20N  167E",  2028.0, 49,  20.0,  167.0,    5.10, &wmm);
    check("2029.0 57km  34N  -13E",  2029.0, 57,  34.0,  -13.0,   -1.89, &wmm);

    /* ── Practical spot checks ─────────────────────────────────────────── */
    /* Seattle WA: known ~+14°–15° E declination in mid-2025 */
    {
        double d = wmm_declination(47.6, -122.3, 0.0, 2025.5, &wmm);
        printf("  INFO  Seattle 47.6N -122.3E  2025.5 → %.2f°E  (expect ~+14.5°)\n", d);
    }
    /* Reykjavik: high declination area */
    {
        double d = wmm_declination(64.1, -21.9, 0.0, 2025.5, &wmm);
        printf("  INFO  Reykjavik 64.1N -21.9E  2025.5 → %.2f°E  (expect ~-13°)\n", d);
    }

    /* ── wmm_decimal_year sanity ───────────────────────────────────────── */
    {
        double y = wmm_decimal_year();
        int ok = (y >= 2025.0 && y <= 2031.0);
        printf("  %s  wmm_decimal_year() = %.4f  (expect 2025–2031)\n",
               ok ? "PASS" : "FAIL", y);
        if (ok) passed++; else failed++;
    }

    /* ── Epoch value ───────────────────────────────────────────────────── */
    {
        double ep = wmm.epoch;
        int ok = (fabs(ep - 2025.0) < 1e-9);
        printf("  %s  wmm.epoch = %.1f  (expect 2025.0)\n",
               ok ? "PASS" : "FAIL", ep);
        if (ok) passed++; else failed++;
    }

    /* ── Error paths ───────────────────────────────────────────────────── */
    {
        wmm_t bad;
        int rc = wmm_load("/tmp/no_such_wmm_file_xyz.COF", &bad);
        int ok = (rc == -1);
        printf("  %s  wmm_load(bad path) returns -1  (got %d)\n",
               ok ? "PASS" : "FAIL", rc);
        if (ok) passed++; else failed++;
    }

    /* ── Full field vector (NOAA 2025 test values, nT) ─────────────────── */
    {
        struct { double year, alt_km, lat, lon, X, Y, Z; } fv[] = {
            { 2025.0, 28, 89, -121,  -255.388723, -1482.460628, 56194.288771 },
            { 2025.0, 65, 43,   93, 24299.852822,   210.517066, 50037.923998 },
        };
        for (int i = 0; i < 2; i++) {
            double ned[3];
            wmm_field_ned(fv[i].lat, fv[i].lon, fv[i].alt_km * 1000.0,
                          fv[i].year, &wmm, ned);
            int ok = fabs(ned[0] - fv[i].X) < 20.0 &&
                     fabs(ned[1] - fv[i].Y) < 20.0 &&
                     fabs(ned[2] - fv[i].Z) < 20.0;
            printf("  %s  wmm_field_ned(%g,%g) = [%.1f %.1f %.1f] nT "
                   "(exp [%.1f %.1f %.1f])\n",
                   ok ? "PASS" : "FAIL", fv[i].lat, fv[i].lon,
                   ned[0], ned[1], ned[2], fv[i].X, fv[i].Y, fv[i].Z);
            if (ok) passed++; else failed++;
        }
    }

    /* A header-only / truncated COF must be rejected, not silently loaded
     * as an all-zero model whose declination is a meaningless 0.0. */
    {
        const char *tpath = "/tmp/imud_test_truncated.COF";
        FILE *fp = fopen(tpath, "w");
        int ok = 0;
        if (fp) {
            fprintf(fp, "  2025.0   WMM-2025   11/13/2024\n"
                        "  1  0  -29351.8       0.0       12.6        0.0\n");
            fclose(fp);
            wmm_t bad;
            ok = (wmm_load(tpath, &bad) == -1);
            remove(tpath);
        }
        printf("  %s  wmm_load(truncated file) returns -1\n",
               ok ? "PASS" : "FAIL");
        if (ok) passed++; else failed++;
    }

    /*
     * The model is never run out past the years its coefficients cover.  A
     * board with no clock battery comes up in 1970, and evaluated there the
     * secular variation moves San Francisco's declination by more than four
     * degrees — which the daemon then published as valid.
     */
    {
        const double end = wmm.epoch + WMM_VALID_YEARS;
        struct { double year, want; int held; const char *what; } c[] = {
            { 2027.3,       2027.3,    0, "a year inside the window is used as it is" },
            { 1970.0,       wmm.epoch, 1, "1970 is held at the epoch" },
            { 2060.0,       end,       1, "2060 is held at the window's end" },
            { (double)NAN,  wmm.epoch, 1, "a year that is not a number is held too" },
        };
        for (unsigned i = 0; i < sizeof c / sizeof c[0]; i++) {
            bool held = !c[i].held;
            double y = wmm_model_year(&wmm, c[i].year, &held);
            int ok = (y == c[i].want) && (held == (c[i].held != 0));
            printf("  %s  %s  (%.1f)\n", ok ? "PASS" : "FAIL", c[i].what, y);
            if (ok) passed++; else failed++;
        }

        double sf_1970  = wmm_declination(37.8, -122.4, 0.0, 1970.0,    &wmm);
        double sf_epoch = wmm_declination(37.8, -122.4, 0.0, wmm.epoch, &wmm);
        double sf_2060  = wmm_declination(37.8, -122.4, 0.0, 2060.0,    &wmm);
        double sf_end   = wmm_declination(37.8, -122.4, 0.0, end,       &wmm);
        int ok = (sf_1970 == sf_epoch) && (sf_2060 == sf_end);
        printf("  %s  San Francisco with the clock at 1970 reads the epoch's "
               "%.2f°E, and at 2060 the window's last %.2f°E\n",
               ok ? "PASS" : "FAIL", sf_1970, sf_2060);
        if (ok) passed++; else failed++;
    }

    printf("\n%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
