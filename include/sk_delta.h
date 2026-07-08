/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * sk_delta.h — Signal K delta encoder for the imud-signalk bridge
 *
 * Builds one Signal K delta message (JSON) from an imud binary packet, mapping
 * every imud field that has a standard Signal K path (SI units — radians,
 * rad/s, metres). Pure and self-contained (no sockets, no globals) so it can
 * be unit-tested directly.
 */
#ifndef IMUD_SK_DELTA_H
#define IMUD_SK_DELTA_H

#include <stddef.h>
#include <stdbool.h>
#include "../lib/imud_client.h"  /* imud_packet_t + IMUD_FLAG_* + imud_true_heading */

/*
 * sk_build_delta — format a Signal K delta from one packet.
 *
 *   buf, sz       output buffer (a few hundred bytes is plenty)
 *   p             a validated imud packet
 *   source_label  Signal K delta source.label (e.g. "imud")
 *   emit_heave    include environment.heave (set when the daemon's heave
 *                 estimator is enabled; heave_m is 0.0 otherwise)
 *
 * Emits, in SI units:
 *   navigation.headingMagnetic            (always)
 *   navigation.headingTrue                (when IMUD_FLAG_DECLINATION_VALID)
 *   navigation.magneticVariation          (when IMUD_FLAG_DECLINATION_VALID)
 *   navigation.rateOfTurn                 (always)
 *   navigation.attitude {roll,pitch,yaw}  (always)
 *   environment.heave                     (when emit_heave)
 *
 * timestamp is ISO-8601 UTC with milliseconds, derived from p->ts_wall_ns.
 *
 * Returns the number of bytes written (excluding the NUL), or -1 if the
 * buffer was too small.
 */
int sk_build_delta(char *buf, size_t sz, const imud_packet_t *p,
                   const char *source_label, bool emit_heave);

#endif /* IMUD_SK_DELTA_H */
