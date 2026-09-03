/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * influx_line.h — InfluxDB line-protocol encoder for the imud-influxdb bridge
 *
 * Builds one InfluxDB line-protocol point from an imud binary packet:
 *
 *   <measurement>,source=<label> <field_set> <ts_ns>
 *
 * Fields: quaternion (qw,qx,qy,qz — unitless), roll/pitch/yaw, heading,
 * heading_true* , variation* , rate_of_turn, heave** , temp, and seq (integer).
 *   *  emitted only when IMUD_FLAG_DECLINATION_VALID is set
 *   ** emitted only when emit_heave is true
 * Angles are degrees (deg = true) or radians (deg = false); rate_of_turn is
 * °/min or rad/s to match. The timestamp is p->ts_wall_ns (nanoseconds).
 *
 * Pure and self-contained (no sockets, no globals) so it can be unit-tested.
 * Returns bytes written (excluding the NUL) or -1 if the buffer was too small.
 */
#ifndef IMUD_INFLUX_LINE_H
#define IMUD_INFLUX_LINE_H

#include <stddef.h>
#include <stdbool.h>
#include "../lib/imud_client.h"  /* imud_packet_t + IMUD_FLAG_* + imud_true_heading */

/*
 * Detail levels, CUMULATIVE: each emits every field of the level below it and
 * adds its own. Ordered so the numbers can be compared, which is what makes
 * the emitter a series of `if (detail >= …)` rather than a set of switches.
 *
 * HEALTH is the default and is exactly what this bridge emitted before the
 * levels existed, so an upgrade changes no existing series.
 */
typedef enum {
    INFLUX_DETAIL_ATTITUDE   = 1,  /* which way it is pointing            */
    INFLUX_DETAIL_NAVIGATION = 2,  /* + earth reference and vertical      */
    INFLUX_DETAIL_SEASTATE   = 3,  /* + the sea-state suite               */
    INFLUX_DETAIL_HEALTH     = 4,  /* + filter and compass diagnostics    */
    INFLUX_DETAIL_FULL       = 5,  /* + sensor vectors and chip timestamps */
} influx_detail_t;

/* Config name -> level; -1 if the name is not one of the five. */
int influx_detail_from_name(const char *name);

/* The name for a level, for logging; "health" if the level is out of range. */
const char *influx_detail_name(int detail);

int influx_build_line(char *buf, size_t sz, const imud_packet_t *p,
                      const char *measurement, const char *source_label,
                      bool emit_heave, bool deg, int detail);

#endif /* IMUD_INFLUX_LINE_H */
