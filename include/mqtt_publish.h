/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mqtt_publish.h — MQTT message builders for the imud-mqtt bridge
 *
 * Turns an imud binary packet into a set of scalar "one value per topic"
 * MQTT messages, plus the matching Home Assistant MQTT-discovery config
 * messages. Pure and self-contained (no libmosquitto, no sockets, no globals)
 * so it can be unit-tested directly; the daemon (mqtt_main.c) owns the broker
 * connection and just publishes what these build.
 *
 * Units follow `deg`: when true (default) angles are degrees, rate of turn is
 * °/min, and heave/temperature are m/°C; when false everything angular is SI
 * (radians, rad/s). Home Assistant sees the right unit via the discovery
 * `unit_of_meas` field.
 */
#ifndef IMUD_MQTT_PUBLISH_H
#define IMUD_MQTT_PUBLISH_H

#include <stdbool.h>
#include "../lib/imud_client.h"  /* imud_packet_t + IMUD_FLAG_* + imud_true_heading */

/* One MQTT message. payload is sized for the largest case (a discovery config). */
typedef struct {
    char topic[128];
    char payload[384];
} mqtt_msg_t;

/*
 * mqtt_build_state — scalar value topics for one packet, under `prefix`
 * (e.g. "imud/attitude/roll"). headingTrue and magneticVariation are emitted
 * only when IMUD_FLAG_DECLINATION_VALID is set; environment/heave only when
 * `emit_heave`. Returns the number of messages written (<= max).
 */
int mqtt_build_state(mqtt_msg_t *out, int max, const imud_packet_t *p,
                     const char *prefix, bool emit_heave, bool deg);

/*
 * mqtt_build_discovery — retained Home Assistant MQTT-discovery config messages,
 * one per sensor mqtt_build_state can emit, published to
 * "<ha_prefix>/sensor/<node_id>_<sensor>/config". All sensors share one HA
 * device ("imud_<node_id>") and an availability topic "<prefix>/status/online".
 * heave is advertised only when `emit_heave`. Returns the count (<= max).
 */
int mqtt_build_discovery(mqtt_msg_t *out, int max, const char *prefix,
                         const char *ha_prefix, const char *node_id,
                         bool emit_heave, bool deg);

#endif /* IMUD_MQTT_PUBLISH_H */
