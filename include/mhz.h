/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mhz.h — print a milli-hertz rate the way a person writes it.
 *
 * Rates are milli-Hz everywhere inward of config.c (see the unit note at the
 * top of include/drivers.h): whole Hz could not hold the TDK parts' 12.5 Hz
 * rung, nor the 13.016 Hz the ST divider chain actually produces, and rounding
 * either way left the advertised rate percent away from the silicon.
 *
 * Header-only and libc-only, same reasoning as fileio.h and cloexec.h — and
 * for one reason more: imud-status links neither log.c nor imu_math.c, and
 * that independence is deliberate.  A formatter it could not use would be a
 * formatter it did not call.
 */
#ifndef IMUD_MHZ_H
#define IMUD_MHZ_H

#include <stddef.h>
#include <stdio.h>

/*
 * 833000 -> "833", 12500 -> "12.5", 13016 -> "13.016".  Trailing zeros in the
 * fraction are trimmed and a whole rate prints with no decimal point at all,
 * so the ordinary case reads exactly as it did when rates were integers.
 *
 * Returns `buf`, so it can be used directly as a printf argument.  Use the
 * MHZ_STR() wrapper on an array to get the size right.
 */
static inline char *imu_mhz_str(char *buf, size_t n, int mhz)
{
    if (n == 0) return buf;
    int whole = mhz / 1000, frac = mhz % 1000;
    if (frac < 0) frac = -frac;
    if (frac == 0)            snprintf(buf, n, "%d", whole);
    else if (frac % 100 == 0) snprintf(buf, n, "%d.%d", whole, frac / 100);
    else if (frac % 10 == 0)  snprintf(buf, n, "%d.%02d", whole, frac / 10);
    else                      snprintf(buf, n, "%d.%03d", whole, frac);
    return buf;
}

#define MHZ_STR(buf, mhz) imu_mhz_str((buf), sizeof (buf), (mhz))

#endif /* IMUD_MHZ_H */
