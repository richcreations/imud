#ifndef IMUD_WMM_H
#define IMUD_WMM_H

/*
 * wmm.h — World Magnetic Model degree-12 spherical harmonic computation
 *
 * Implements the NOAA WMM Technical Note 28 algorithm.
 * Coefficient file: WMM.COF (standard NOAA ASCII format).
 * Current model: WMM2025, valid 2025.0 – 2030.0.
 *
 * SPDX-License-Identifier: MIT
 */

/* WMM degree-12 Gauss coefficients + secular variation */
typedef struct {
    double epoch;           /* model epoch year, e.g. 2025.0 */
    double g[13][13];       /* g_nm (nT) */
    double h[13][13];       /* h_nm (nT) */
    double g_sv[13][13];    /* secular variation dg/dt (nT/year) */
    double h_sv[13][13];    /* secular variation dh/dt (nT/year) */
} wmm_t;

/*
 * wmm_load — parse a WMM.COF coefficient file into *out.
 * Returns 0 on success, -1 on error (file not found, parse error, etc.).
 */
int wmm_load(const char *path, wmm_t *out);

/*
 * wmm_declination — compute magnetic declination in degrees (East positive).
 *
 *   lat_deg      geodetic latitude  (degrees, +N / -S)
 *   lon_deg      geodetic longitude (degrees, +E / -W)
 *   alt_m        height above WGS-84 ellipsoid (metres; pass 0.0 for sea level)
 *   decimal_year e.g. 2025.4
 *   wmm          loaded coefficient struct
 */
double wmm_declination(double lat_deg, double lon_deg,
                       double alt_m, double decimal_year,
                       const wmm_t *wmm);

/*
 * wmm_decimal_year — return the current date as a decimal year using
 * CLOCK_REALTIME (e.g. 2025.38 for mid-May 2025).
 */
double wmm_decimal_year(void);

#endif /* IMUD_WMM_H */
