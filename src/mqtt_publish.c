/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mqtt_publish.c — MQTT message builders (see mqtt_publish.h)
 *
 * Scalar "one value per topic" telemetry plus Home Assistant MQTT-discovery
 * configs. Attitude/heading use imud's native NED convention (roll + = starboard
 * up, pitch + = bow up) — unlike the Signal K bridge, no sign is flipped; this
 * is raw telemetry for dashboards, and the discovery `unit_of_meas` documents
 * the units.
 */

#include "mqtt_publish.h"

#include <stdio.h>
#include <string.h>

#define RAD2DEG  57.29577951308232
#define DEG2RAD  0.017453292519943295

typedef enum { GATE_ALWAYS, GATE_DECL, GATE_HEAVE } gate_t;
typedef enum { U_ANGLE, U_ROT, U_HEAVE, U_VELOCITY, U_TEMP } unit_t;

/* The published fields, in a fixed order. field_value() below switches on the
 * same index, so keep the two in lockstep. */
static const struct field {
    const char *sub;    /* topic subpath, e.g. "navigation/headingMagnetic" */
    const char *oid;    /* HA object id / unique-id suffix, e.g. "heading_magnetic" */
    const char *name;   /* HA friendly name */
    unit_t      unit;
    gate_t      gate;
} FIELDS[] = {
    { "navigation/headingMagnetic",   "heading_magnetic",   "Heading (magnetic)", U_ANGLE, GATE_ALWAYS },
    { "navigation/headingTrue",       "heading_true",       "Heading (true)",     U_ANGLE, GATE_DECL   },
    { "navigation/magneticVariation", "magnetic_variation", "Magnetic variation", U_ANGLE, GATE_DECL   },
    { "navigation/rateOfTurn",        "rate_of_turn",       "Rate of turn",       U_ROT,   GATE_ALWAYS },
    { "attitude/roll",                "roll",               "Roll",               U_ANGLE, GATE_ALWAYS },
    { "attitude/pitch",               "pitch",              "Pitch",              U_ANGLE, GATE_ALWAYS },
    { "attitude/yaw",                 "yaw",                "Yaw",                U_ANGLE, GATE_ALWAYS },
    { "environment/heave",            "heave",              "Heave",              U_HEAVE,    GATE_HEAVE  },
    { "environment/heaveRate",        "heave_rate",         "Heave rate",         U_VELOCITY, GATE_HEAVE  },
    { "imu/temperature",              "temperature",        "Temperature",        U_TEMP,     GATE_ALWAYS },
};
#define NFIELDS ((int)(sizeof FIELDS / sizeof FIELDS[0]))

/* Numeric value + print precision for field `i`. Cases MUST track FIELDS order.
 * Only ever called for a field whose gate passed (so case 1 sees a valid
 * declination and imud_true_heading never returns -1 here). */
static double field_value(int i, const imud_packet_t *p, bool deg, int *prec)
{
    switch (i) {
    case 0: *prec = deg ? 2 : 4; return deg ? p->heading_deg : p->heading_deg * DEG2RAD;
    case 1: { float th = imud_true_heading(p);
              *prec = deg ? 2 : 4; return deg ? th : th * DEG2RAD; }
    case 2: *prec = deg ? 2 : 4; return deg ? p->declination_deg : p->declination_deg * DEG2RAD;
    case 3: *prec = deg ? 1 : 5; return deg ? p->rate_of_turn : p->rate_of_turn * DEG2RAD / 60.0;
    case 4: *prec = deg ? 2 : 4; return deg ? p->roll  * RAD2DEG : p->roll;
    case 5: *prec = deg ? 2 : 4; return deg ? p->pitch * RAD2DEG : p->pitch;
    case 6: *prec = deg ? 2 : 4; return deg ? p->yaw   * RAD2DEG : p->yaw;
    case 7: *prec = 3; return p->heave_m;
    case 8: *prec = 3; return p->heave_rate;   /* m/s, SI — no deg conversion */
    case 9: *prec = 1; return p->temp_c;
    default: *prec = 2; return 0.0;
    }
}

static bool field_on(gate_t g, const imud_packet_t *p, bool emit_heave)
{
    switch (g) {
    case GATE_DECL:  return (p->flags & IMUD_FLAG_DECLINATION_VALID) != 0;
    /* Withhold heave/heave-rate state until the estimator has settled (~10·τ);
     * discovery is still advertised on config alone (see mqtt_build_discovery). */
    case GATE_HEAVE: return emit_heave && (p->flags & IMUD_FLAG_HEAVE_VALID) != 0;
    default:         return true;
    }
}

static const char *unit_str(unit_t u, bool deg)
{
    switch (u) {
    case U_ANGLE: return deg ? "°"     : "rad";
    case U_ROT:   return deg ? "°/min" : "rad/s";
    case U_HEAVE: return "m";
    case U_VELOCITY: return "m/s";
    case U_TEMP:  return "°C";
    default:      return "";
    }
}

int mqtt_build_state(mqtt_msg_t *out, int max, const imud_packet_t *p,
                     const char *prefix, bool emit_heave, bool deg)
{
    if (!out || !p || max <= 0) return 0;

    int n = 0;
    for (int i = 0; i < NFIELDS && n < max; i++) {
        if (!field_on(FIELDS[i].gate, p, emit_heave)) continue;
        int prec;
        double v = field_value(i, p, deg, &prec);
        snprintf(out[n].topic, sizeof out[n].topic, "%s/%s", prefix, FIELDS[i].sub);
        snprintf(out[n].payload, sizeof out[n].payload, "%.*f", prec, v);
        n++;
    }
    return n;
}

int mqtt_build_discovery(mqtt_msg_t *out, int max, const char *prefix,
                         const char *ha_prefix, const char *node_id,
                         bool emit_heave, bool deg)
{
    if (!out || max <= 0) return 0;

    int n = 0;
    for (int i = 0; i < NFIELDS && n < max; i++) {
        /* Advertise every field except heave when heave is off. Declination
         * fields are always advertised (they publish state only when valid). */
        if (FIELDS[i].gate == GATE_HEAVE && !emit_heave) continue;

        snprintf(out[n].topic, sizeof out[n].topic,
                 "%s/sensor/%s_%s/config", ha_prefix, node_id, FIELDS[i].oid);
        snprintf(out[n].payload, sizeof out[n].payload,
            "{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"stat_t\":\"%s/%s\","
            "\"unit_of_meas\":\"%s\",\"avty_t\":\"%s/status/online\","
            "\"dev\":{\"ids\":[\"imud_%s\"],\"name\":\"imud\","
            "\"mf\":\"imud\",\"mdl\":\"IMU daemon\"}}",
            FIELDS[i].name, node_id, FIELDS[i].oid, prefix, FIELDS[i].sub,
            unit_str(FIELDS[i].unit, deg), prefix, node_id);
        n++;
    }
    return n;
}
