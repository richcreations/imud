/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_wmm.c — libFuzzer harness for the WMM coefficient-file parser
 * (src/wmm.c).  wmm_load takes a path, so each input lands in a
 * per-process temp file first, exactly like fuzz_config.
 *
 * Parsing is only half the target.  A .COF file that parses "successfully"
 * can still carry absurd or non-finite coefficients, and those flow
 * straight into the spherical-harmonic evaluation and out into the MEKF's
 * magnetic reference.  So a successful load is followed by field
 * evaluations at the awkward points of the coordinate system — the poles
 * (where the associated Legendre recursion divides by cos(lat)), the date
 * line, and the equator — plus epochs far from the model's own, which
 * scales the secular-variation terms without bound.
 *
 * Run with -close_fd_mask=3 to silence the parser's error logging.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "wmm.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static char path[64];
    if (!path[0])
        snprintf(path, sizeof(path), "/tmp/imud_fuzz_wmm_%d", (int)getpid());

    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    if (size) fwrite(data, 1, size, f);
    fclose(f);

    wmm_t wmm;
    if (wmm_load(path, &wmm) != 0)
        return 0;               /* rejected — that is the parser working */

    /* Latitudes include the exact poles: the Legendre recursion and the
     * geodetic-to-geocentric conversion are the parts most likely to blow
     * up on a degenerate coefficient set. */
    static const double lat[] = {  90.0, -90.0, 89.999, 0.0, -33.9, 60.0 };
    static const double lon[] = { 180.0, -180.0,   0.0, 1e3, 151.2, -7.6 };
    /* Epochs well outside the model window: decimal_year - epoch multiplies
     * every secular-variation term, so a large offset amplifies garbage. */
    static const double yr[]  = { 2025.4, 1900.0, 2200.0 };

    for (size_t i = 0; i < sizeof(lat) / sizeof(lat[0]); i++) {
        for (size_t k = 0; k < sizeof(yr) / sizeof(yr[0]); k++) {
            double ned[3];
            wmm_field_ned(lat[i], lon[i], 0.0, yr[k], &wmm, ned);
            wmm_declination(lat[i], lon[i], 30000.0, yr[k], &wmm);
        }
    }
    return 0;
}
