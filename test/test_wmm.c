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

    printf("\n%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
