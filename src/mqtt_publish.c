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

/* GATE_MAG: the magnetometer is being fused, calibrated or not, so heading
 * really is magnetic.  MQTT topics are RETAINED — a subscriber connecting
 * later is handed the last value published — so a heading left on
 * navigation/headingMagnetic after the magnetometer died would sit there
 * indefinitely, presented as current.  Withholding is the only safe option
 * for a topic whose name states what it carries. */
typedef enum { GATE_ALWAYS, GATE_DECL, GATE_HEAVE, GATE_WAVE, GATE_MAG } gate_t;
typedef enum { U_ANGLE, U_ROT, U_HEAVE, U_VELOCITY, U_TEMP, U_SECONDS } unit_t;

/* The published fields, in a fixed order. field_value() below switches on the
 * same index, so keep the two in lockstep. */
static const struct field {
    const char *sub;    /* topic subpath, e.g. "navigation/headingMagnetic" */
    const char *oid;    /* HA object id / unique-id suffix, e.g. "heading_magnetic" */
    const char *name;   /* HA friendly name */
    unit_t      unit;
    gate_t      gate;
} FIELDS[] = {
    { "navigation/headingMagnetic",   "heading_magnetic",   "Heading (magnetic)", U_ANGLE, GATE_MAG    },
    { "navigation/headingTrue",       "heading_true",       "Heading (true)",     U_ANGLE, GATE_DECL   },
    { "navigation/magneticVariation", "magnetic_variation", "Magnetic variation", U_ANGLE, GATE_DECL   },
    { "navigation/rateOfTurn",        "rate_of_turn",       "Rate of turn",       U_ROT,   GATE_ALWAYS },
    { "attitude/roll",                "roll",               "Roll",               U_ANGLE, GATE_ALWAYS },
    { "attitude/pitch",               "pitch",              "Pitch",              U_ANGLE, GATE_ALWAYS },
    { "attitude/yaw",                 "yaw",                "Yaw",                U_ANGLE, GATE_ALWAYS },
    { "environment/heave",            "heave",              "Heave",              U_HEAVE,    GATE_HEAVE  },
    { "environment/heaveRate",        "heave_rate",         "Heave rate",         U_VELOCITY, GATE_HEAVE  },
    { "environment/waveHeight",       "wave_height",        "Wave height (sig.)", U_HEAVE,    GATE_WAVE   },
    { "environment/wavePeriod",       "wave_period",        "Wave period",        U_SECONDS,  GATE_WAVE   },
    { "environment/rollPeriod",       "roll_period",        "Roll period",        U_SECONDS,  GATE_WAVE   },
    { "environment/rollAmplitude",    "roll_amplitude",     "Roll amplitude (sig.)",  U_ANGLE, GATE_WAVE  },
    { "environment/pitchPeriod",      "pitch_period",       "Pitch period",       U_SECONDS,  GATE_WAVE   },
    { "environment/pitchAmplitude",   "pitch_amplitude",    "Pitch amplitude (sig.)", U_ANGLE, GATE_WAVE  },
    { "imu/temperature",              "temperature",        "Temperature",        U_TEMP,     GATE_ALWAYS },
};
#define NFIELDS ((int)(sizeof FIELDS / sizeof FIELDS[0]))

/* Numeric value + print precision for field `i`. Cases MUST track FIELDS order.
 * Only ever called for a field whose gate passed, so case 1 sees a true heading
 * libimud has already computed and wrapped into [0, 360). */
static double field_value(int i, const imud_data_t *d, bool deg, int *prec)
{
    switch (i) {
    case 0: *prec = deg ? 2 : 4; return deg ? d->heading_deg : d->heading_deg * DEG2RAD;
    case 1: *prec = deg ? 2 : 4; return deg ? d->heading_true_deg
                                            : d->heading_true_deg * DEG2RAD;
    case 2: *prec = deg ? 2 : 4; return deg ? d->declination_deg : d->declination_deg * DEG2RAD;
    case 3: *prec = deg ? 1 : 5; return deg ? d->rate_of_turn : d->rate_of_turn * DEG2RAD / 60.0;
    case 4: *prec = deg ? 2 : 4; return deg ? d->roll  * RAD2DEG : d->roll;
    case 5: *prec = deg ? 2 : 4; return deg ? d->pitch * RAD2DEG : d->pitch;
    case 6: *prec = deg ? 2 : 4; return deg ? d->yaw   * RAD2DEG : d->yaw;
    case 7: *prec = 3; return d->heave_m;
    case 8: *prec = 3; return d->heave_rate;   /* m/s, SI — no deg conversion */
    case 9:  *prec = 2; return d->wave_height_m;   /* m,  SI */
    case 10: *prec = 1; return d->wave_period_s;   /* s,  SI */
    case 11: *prec = 1; return d->roll_period_s;   /* s,  SI */
    case 12: *prec = deg ? 2 : 4; return deg ? d->roll_amplitude  * RAD2DEG : d->roll_amplitude;
    case 13: *prec = 1; return d->pitch_period_s;  /* s,  SI */
    case 14: *prec = deg ? 2 : 4; return deg ? d->pitch_amplitude * RAD2DEG : d->pitch_amplitude;
    case 15: *prec = 1; return d->temp_c;
    default: *prec = 2; return 0.0;
    }
}

static bool field_on(gate_t g, const imud_data_t *d, bool emit_heave)
{
    switch (g) {
    /* Declination is only reachable through a magnetic heading, so GATE_DECL
     * carries the mag condition too: a true heading computed from a
     * dead-reckoned one is not a true heading. */
    case GATE_DECL:  return (d->flags & IMUD_FLAG_DECLINATION_VALID) != 0 &&
                            (d->flags & (IMUD_FLAG_MAG_VALID |
                                         IMUD_FLAG_MAG_UNCAL)) != 0;
    case GATE_MAG:   return (d->flags & (IMUD_FLAG_MAG_VALID |
                                         IMUD_FLAG_MAG_UNCAL)) != 0;
    /* Withhold heave/heave-rate state until the estimator has settled (~10·τ);
     * discovery is still advertised on config alone (see mqtt_build_discovery). */
    case GATE_HEAVE: return emit_heave && (d->flags & IMUD_FLAG_HEAVE_VALID) != 0;
    /* Sea state derives from heave, so it rides the publish_heave key; state
     * is withheld until the wave statistics settle (~2·wave_tau_s). Periods
     * publish 0.0 when becalmed/not rolling — that IS the measurement. */
    case GATE_WAVE:  return emit_heave && (d->flags & IMUD_FLAG_WAVE_VALID) != 0;
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
    case U_SECONDS: return "s";
    default:      return "";
    }
}

int mqtt_build_state(mqtt_msg_t *out, int max, const imud_data_t *d,
                     const char *prefix, bool emit_heave, bool deg)
{
    if (!out || !d || max <= 0) return 0;

    int n = 0;
    for (int i = 0; i < NFIELDS && n < max; i++) {
        if (!field_on(FIELDS[i].gate, d, emit_heave)) continue;
        int prec;
        double v = field_value(i, d, deg, &prec);
        snprintf(out[n].topic, sizeof out[n].topic, "%s/%s", prefix, FIELDS[i].sub);
        snprintf(out[n].payload, sizeof out[n].payload, "%.*f", prec, v);
        n++;
    }

    /* Engine state (v14): ON/OFF from FLAG_ENGINE_ON — a binary topic, so it
     * lives outside the numeric FIELDS table. Always published; it simply
     * mirrors the daemon's engine-vibration detector (OFF when detection is
     * disabled in the daemon config). */
    if (n < max) {
        snprintf(out[n].topic, sizeof out[n].topic, "%s/engine/running", prefix);
        snprintf(out[n].payload, sizeof out[n].payload, "%s",
                 (d->flags & IMUD_FLAG_ENGINE_ON) ? "ON" : "OFF");
        n++;
    }

    /* Why the heading topics may be missing.  Without this a subscriber sees
     * navigation/headingMagnetic simply stop, which is indistinguishable from
     * the bridge dying; published always, so the answer is always retained. */
    if (n < max) {
        snprintf(out[n].topic, sizeof out[n].topic,
                 "%s/navigation/headingReferenced", prefix);
        snprintf(out[n].payload, sizeof out[n].payload, "%s",
                 field_on(GATE_MAG, d, emit_heave) ? "ON" : "OFF");
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
        if ((FIELDS[i].gate == GATE_HEAVE || FIELDS[i].gate == GATE_WAVE)
            && !emit_heave) continue;

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

    /* Engine running: Home Assistant binary_sensor (device_class: running). */
    if (n < max) {
        snprintf(out[n].topic, sizeof out[n].topic,
                 "%s/binary_sensor/%s_engine/config", ha_prefix, node_id);
        snprintf(out[n].payload, sizeof out[n].payload,
            "{\"name\":\"Engine running\",\"uniq_id\":\"%s_engine\","
            "\"stat_t\":\"%s/engine/running\","
            "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"running\","
            "\"avty_t\":\"%s/status/online\","
            "\"dev\":{\"ids\":[\"imud_%s\"],\"name\":\"imud\","
            "\"mf\":\"imud\",\"mdl\":\"IMU daemon\"}}",
            node_id, prefix, prefix, node_id);
        n++;
    }
    return n;
}
