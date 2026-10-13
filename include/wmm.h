/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

#ifndef IMUD_WMM_H
#define IMUD_WMM_H

/*
 * wmm.h — World Magnetic Model degree-12 spherical harmonic computation
 *
 * Implements the NOAA WMM Technical Note 28 algorithm.
 * Coefficient file: WMM.COF (standard NOAA ASCII format).
 * Current model: WMM2025, valid 2025.0 – 2030.0.
 */

#include <stdbool.h>

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

/* How long after its epoch a set of coefficients is good for, in years. */
#define WMM_VALID_YEARS 5.0

/*
 * wmm_model_year — the year the model is evaluated at: `year`, held inside
 * the WMM_VALID_YEARS its coefficients cover.  *held, if not NULL, says
 * whether it had to be.
 *
 * wmm_field_ned applies this itself, so the secular variation is never run
 * out past either end.  A board with no clock battery comes up in 1970, and
 * 55 years of it moved San Francisco's declination by 4.6°.  Held at the
 * nearer end the answer is within five years' drift of the truth whichever
 * way the clock is wrong.  Callers use it to say so.
 */
double wmm_model_year(const wmm_t *wmm, double year, bool *held);

/*
 * wmm_field_ned — full geomagnetic field vector in geodetic NED, nT.
 * ned_nT[0] = north, [1] = east, [2] = down (positive into the earth, NH).
 * Same inputs as wmm_declination (which is atan2(east, north) of this).
 * Horizontal magnitude H = hypot(ned[0], ned[1]); the MEKF magnetic
 * reference in its heading-magnetic frame is [H, 0, ned[2]].
 */
void wmm_field_ned(double lat_deg, double lon_deg,
                   double alt_m, double decimal_year,
                   const wmm_t *wmm, double ned_nT[3]);

/*
 * wmm_derive_refs — declination (degrees East) plus the MEKF magnetic-
 * reference field invariants (nT → Gauss) from one wmm_field_ned() result.
 * Shared by the daemon's static-position path (main.c) and the GPS path
 * (position.c), which each need the NED vector once for both outputs.
 */
void wmm_derive_refs(const double ned_nT[3], double *decl_deg,
                     float *mref_h_gauss, float *mref_z_gauss);

/*
 * wmm_decimal_year — return the current date as a decimal year using
 * CLOCK_REALTIME (e.g. 2025.38 for mid-May 2025).
 */
double wmm_decimal_year(void);

#endif /* IMUD_WMM_H */
