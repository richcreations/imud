/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * sk_delta.c — Signal K delta encoder (see sk_delta.h)
 *
 * Field/units/sign mapping researched against the Signal K 1.7 vessel schema
 * (signalk.org/specification, vesselsBranch.html):
 *   navigation.headingMagnetic  rad
 *   navigation.headingTrue      rad
 *   navigation.magneticVariation rad, Easterly positive (matches imud °E+)
 *   navigation.rateOfTurn       rad/s, +ve = starboard (matches imud +right)
 *   navigation.attitude         rad {roll, pitch, yaw}
 *   environment.heave           m, "vertical movement of the vessel due to waves"
 *
 * Attitude sign convention: imud uses pitch + = bow up, roll + = starboard up,
 * yaw = magnetic heading. Signal K's schema does not state attitude sign
 * directions; the widely-used convention is roll + = starboard down, so imud's
 * roll is NEGATED here. Pitch and yaw are passed through. Revisit against a
 * live Signal K display if roll appears inverted.
 */

#include "sk_delta.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD  (M_PI / 180.0)

/* ISO-8601 UTC with milliseconds, e.g. 2026-07-08T12:34:56.789Z */
static void iso8601_ms(uint64_t wall_ns, char *out, size_t sz)
{
    time_t     secs = (time_t)(wall_ns / 1000000000ULL);
    unsigned   ms   = (unsigned)((wall_ns / 1000000ULL) % 1000ULL);
    struct tm  tm;
    gmtime_r(&secs, &tm);
    char base[24];
    strftime(base, sizeof base, "%Y-%m-%dT%H:%M:%S", &tm);
    snprintf(out, sz, "%s.%03uZ", base, ms);
}

/*
 * Append `frag` to buf tracking the running length. `*pos` never advances past
 * the buffer end, so a truncated field is detected as overflow by the caller.
 */
#define APPEND(...) do {                                              \
        int _r = snprintf(buf + pos, (pos < (int)sz) ? sz - pos : 0,  \
                          __VA_ARGS__);                               \
        if (_r < 0) return -1;                                        \
        pos += _r;                                                    \
    } while (0)

int sk_build_delta(char *buf, size_t sz, const imud_packet_t *p,
                   const char *source_label, bool emit_heave)
{
    if (!buf || sz == 0 || !p) return -1;

    char ts[28];
    iso8601_ms(p->ts_wall_ns, ts, sizeof ts);

    int pos = 0;

    APPEND("{\"updates\":[{\"source\":{\"label\":\"%s\"},"
           "\"timestamp\":\"%s\",\"values\":[", source_label, ts);

    /* navigation.headingMagnetic (always) */
    APPEND("{\"path\":\"navigation.headingMagnetic\",\"value\":%.5f}",
           p->heading_deg * DEG2RAD);

    /* True heading + variation only when declination is known. */
    if (p->flags & IMUD_FLAG_DECLINATION_VALID) {
        float th = imud_true_heading(p);   /* degrees, or -1 (guarded above) */
        APPEND(",{\"path\":\"navigation.headingTrue\",\"value\":%.5f}",
               th * DEG2RAD);
        APPEND(",{\"path\":\"navigation.magneticVariation\",\"value\":%.5f}",
               p->declination_deg * DEG2RAD);
    }

    /* navigation.rateOfTurn: deg/min → rad/s (both +ve = starboard). */
    APPEND(",{\"path\":\"navigation.rateOfTurn\",\"value\":%.6f}",
           p->rate_of_turn * (DEG2RAD / 60.0));

    /* navigation.attitude: roll/pitch/yaw already radians; roll negated to the
     * Signal K convention (starboard-down positive). */
    APPEND(",{\"path\":\"navigation.attitude\",\"value\":"
           "{\"roll\":%.5f,\"pitch\":%.5f,\"yaw\":%.5f}}",
           -p->roll, p->pitch, p->yaw);

    /* environment.heave (metres, +up) — only once the estimator has settled
     * (~10·τ), so subscribers never see the startup transient. */
    if (emit_heave && (p->flags & IMUD_FLAG_HEAVE_VALID))
        APPEND(",{\"path\":\"environment.heave\",\"value\":%.3f}", p->heave_m);

    APPEND("]}]}");

    if (pos >= (int)sz) return -1;   /* truncated */
    return pos;
}
