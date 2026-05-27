/*
 * wmm.c — World Magnetic Model degree-12 spherical harmonic computation
 *
 * Algorithm: NOAA WMM Technical Note 28, Appendix A (Langel 1987 recursion).
 * Coordinate system: geodetic WGS-84 input, geocentric spherical for field
 * evaluation, geodetic NED for output.
 *
 * SPDX-License-Identifier: MIT
 */

#include "wmm.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ── WGS-84 constants ───────────────────────────────────────────────────── */

#define WGS84_A  6378137.0          /* semi-major axis (m) */
#define WGS84_F  (1.0/298.257223563)
#define WGS84_E2 (2.0*WGS84_F - WGS84_F*WGS84_F)

/* WMM reference radius in km — matches nT coefficient units */
#define WMM_A_KM  6371.2

/* ── wmm_load ────────────────────────────────────────────────────────────── */

int wmm_load(const char *path, wmm_t *out)
{
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;

    memset(out, 0, sizeof(*out));

    /* Header: "  2025.0   WMM-2025   11/13/2024" */
    if (fscanf(fp, " %lf", &out->epoch) != 1) { fclose(fp); return -1; }
    /* Skip the rest of the header line */
    { char buf[256]; if (!fgets(buf, sizeof(buf), fp)) { fclose(fp); return -1; } }

    int n, m;
    double g, h, gs, hs;
    while (fscanf(fp, " %d %d %lf %lf %lf %lf", &n, &m, &g, &h, &gs, &hs) == 6) {
        if (n > 12) break;          /* 999…9 terminator */
        if (n < 1 || m < 0 || m > n) continue;
        out->g[n][m]    = g;
        out->h[n][m]    = h;
        out->g_sv[n][m] = gs;
        out->h_sv[n][m] = hs;
    }

    fclose(fp);
    return 0;
}

/* ── wmm_decimal_year ───────────────────────────────────────────────────── */

double wmm_decimal_year(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm t;
    gmtime_r(&ts.tv_sec, &t);
    int year = t.tm_year + 1900;
    int leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    double doy = t.tm_yday + (ts.tv_sec % 86400) / 86400.0;
    return year + doy / (leap ? 366.0 : 365.0);
}

/* ── wmm_declination ────────────────────────────────────────────────────── */

/*
 * Degree-12 spherical harmonic evaluation per WMM TN-28 §A.
 *
 * Schmidt quasi-normal ALF recursion (Langel 1987):
 *
 *   Seeds:
 *     P[0][0] = 1,  P[1][0] = cos θ,  P[1][1] = sin θ
 *
 *   Sectorial (m = n, n ≥ 2):
 *     P[n][n] = sin θ · √((2n-1)/(2n)) · P[n-1][n-1]
 *
 *   Sub-diagonal (m = n-1, n ≥ 2):
 *     P[n][n-1] = cos θ · √(2n-1) · P[n-1][n-1]
 *
 *   Tesseral (1 ≤ m ≤ n-2) and zonal (m = 0, n ≥ 2):
 *     m = 0:  P[n][0] = [(2n-1) cos θ P[n-1][0] - (n-1) P[n-2][0]] / n
 *     m ≥ 1:  P[n][m] = A · cos θ · P[n-1][m] - B · P[n-2][m]
 *             A = √((4n²-1)/(n²-m²))
 *             B = √(((n-1)²-m²) / ((2n-3)(2n-1)))
 *
 * Field components (geocentric spherical, a = WMM_A_KM, r in km):
 *
 *   X_gc (north) = + Σ_n (a/r)^{n+2} Σ_m [g cos(mλ) + h sin(mλ)] dP/dθ
 *   Y_gc (east)  = + Σ_n (a/r)^{n+2} Σ_m m [h cos(mλ) - g sin(mλ)] P / sin θ
 *   Z_gc (down)  = - Σ_n (n+1)(a/r)^{n+2} Σ_m [g cos(mλ) + h sin(mλ)] P
 *
 * Geodetic correction (small oblate-Earth angle ψ = φ_c - φ):
 *   X_ned = X_gc cos ψ + Z_gc sin ψ
 *   Y_ned = Y_gc
 *
 * Declination = atan2(Y_ned, X_ned).
 */
double wmm_declination(double lat_deg, double lon_deg,
                       double alt_m, double decimal_year,
                       const wmm_t *wmm)
{
    const int NMAX = 12;

    /* ── Step 1: secular variation ──────────────────────────────────────── */
    double dt = decimal_year - wmm->epoch;
    double g[13][13], h[13][13];
    for (int n = 1; n <= NMAX; n++)
        for (int m = 0; m <= n; m++) {
            g[n][m] = wmm->g[n][m] + wmm->g_sv[n][m] * dt;
            h[n][m] = wmm->h[n][m] + wmm->h_sv[n][m] * dt;
        }

    /* ── Step 2: geodetic → geocentric spherical ────────────────────────── */
    double lat_r  = lat_deg * M_PI / 180.0;
    double lon_r  = lon_deg * M_PI / 180.0;

    double sin_lat = sin(lat_r);
    double cos_lat = cos(lat_r);

    /* Radius of curvature in prime vertical */
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    /* ECEF */
    double px = (N + alt_m) * cos_lat;
    double pz = (N * (1.0 - WGS84_E2) + alt_m) * sin_lat;

    /* Geocentric radius (km) and co-latitude θ */
    double r_m      = sqrt(px*px + pz*pz);
    double r_km     = r_m * 1e-3;
    double theta    = acos(pz / r_m);           /* co-latitude from north pole */
    double sin_th   = sin(theta);
    double cos_th   = cos(theta);
    double lambda   = lon_r;

    /* ── Step 3: Schmidt quasi-normal Legendre functions ────────────────── */
    /*
     * Seeds: P[0][0]=1, P[1][0]=cos θ, P[1][1]=sin θ
     *
     * For n ≥ 2, two cases:
     *
     *   Sectorial (m = n):
     *     P[n][n] = √((2n-1)/(2n)) · sin θ · P[n-1][n-1]
     *
     *   All other m (0 ≤ m ≤ n-1):
     *     A = (2n-1) / √(n²-m²)
     *     B = √((n-m-1)(n+m-1) / ((n-m)(n+m)))    — zero when m = n-1
     *     P[n][m] = A · cos θ · P[n-1][m] - B · P[n-2][m]
     *
     * This single formula covers zonal (m=0), sub-diagonal (m=n-1, B=0),
     * and all tesseral terms.  Derivation: Schmidt quasi-normal recursion
     * from the Legendre polynomial 3-term relation (Langel 1987 / TN-28 §A).
     */
    double P[14][14]  = {{0}};
    double dP[14][14] = {{0}};

    /* n = 0 and n = 1 seeds */
    P[0][0]  = 1.0;   dP[0][0] = 0.0;
    P[1][0]  =  cos_th;   dP[1][0] = -sin_th;
    P[1][1]  =  sin_th;   dP[1][1] =  cos_th;

    for (int n = 2; n <= NMAX; n++) {
        /* Sectorial: m = n */
        double fs = sqrt((2.0*n - 1.0) / (2.0*n));
        P[n][n]  = sin_th * fs * P[n-1][n-1];
        dP[n][n] = fs * (sin_th * dP[n-1][n-1] + cos_th * P[n-1][n-1]);

        /* Zonal + sub-diagonal + tesseral: 0 ≤ m ≤ n-1 */
        for (int m = 0; m <= n-1; m++) {
            double nm2 = (double)(n*n - m*m);
            double A   = (2.0*n - 1.0) / sqrt(nm2);
            /* B = 0 when m = n-1 (n-m-1 = 0); avoid sqrt of negative */
            double num = (double)((n-m-1)*(n+m-1));
            double den = (double)((n-m)*(n+m));
            double B   = (num > 0.0) ? sqrt(num / den) : 0.0;
            P[n][m]  = A * cos_th  * P[n-1][m]  - B * P[n-2][m];
            dP[n][m] = A * (cos_th * dP[n-1][m] - sin_th * P[n-1][m]) - B * dP[n-2][m];
        }
    }

    /* ── Step 4: sum field components ───────────────────────────────────── */
    double X_gc = 0.0, Y_gc = 0.0, Z_gc = 0.0;

    /* Avoid pow() inside the n-loop: start at (a/r)^3 (n=1 term) and
     * multiply by (a/r) each iteration instead of 12 separate pow() calls. */
    double ar_base = WMM_A_KM / r_km;
    double ar_n2   = ar_base * ar_base * ar_base;  /* (a/r)^3 for n=1 */

    for (int n = 1; n <= NMAX; n++) {
        double Xn = 0.0, Yn = 0.0, Zn = 0.0;

        for (int m = 0; m <= n; m++) {
            double cos_ml = cos((double)m * lambda);
            double sin_ml = sin((double)m * lambda);
            /* gcml = g cos(mλ) + h sin(mλ)  (X and Z) */
            double gcml = g[n][m] * cos_ml + h[n][m] * sin_ml;
            /* gsml = g sin(mλ) - h cos(mλ)  (Y: from -∂V/∂λ sign chain) */
            double gsml = g[n][m] * sin_ml - h[n][m] * cos_ml;

            Xn += gcml * dP[n][m];                          /* X: north (+) */
            Yn += (double)m * gsml * P[n][m];               /* Y: east       */
            Zn -= (double)(n + 1) * gcml * P[n][m];         /* Z: down (-)   */
        }

        /* Guard against sin θ ≈ 0 at the poles */
        double st = (sin_th < 1e-10) ? 1e-10 : sin_th;

        X_gc += ar_n2 * Xn;
        Y_gc += ar_n2 * Yn / st;
        Z_gc += ar_n2 * Zn;

        ar_n2 *= ar_base;   /* advance to (a/r)^(n+3) for next n */
    }

    /* ── Step 5: rotate geocentric NED → geodetic NED ───────────────────── */
    /*
     * delta = geodetic_lat − geocentric_lat > 0 in the northern hemisphere.
     * The geodetic "up" direction tilts slightly poleward vs. the geocentric
     * radial, so geodetic "north" tilts equatorward.  Component formula:
     *
     *   X_ned = X_gc · cos δ + Z_gc · sin δ
     *   Y_ned = Y_gc   (east unchanged)
     */
    double gc_lat  = M_PI / 2.0 - theta;   /* geocentric latitude */
    double delta   = lat_r - gc_lat;        /* geodetic − geocentric, > 0 NH */
    double X_ned   = X_gc * cos(delta) + Z_gc * sin(delta);
    double Y_ned   = Y_gc;

    return atan2(Y_ned, X_ned) * (180.0 / M_PI);
}
