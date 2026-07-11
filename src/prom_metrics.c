/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * prom_metrics.c — Prometheus text-exposition builder (see prom_metrics.h)
 *
 * One GAUGE(...) line per exported value, HELP/TYPE included so the page is
 * self-describing in the Prometheus UI. Everything is base SI (the
 * Prometheus convention) — no deg/rad configuration: radians, metres,
 * seconds, celsius. Heading is the deliberate exception (degrees, named
 * imud_heading_degrees) because 0–360° is the universal helm convention.
 *
 * Flag bits become 0/1 gauges so dashboards can alert on them directly;
 * imud_up says whether the exporter currently holds a live packet.
 */

#include <stdio.h>
#include <inttypes.h>
#include "prom_metrics.h"

#define APPEND(...)                                                    \
    do {                                                               \
        int _r = snprintf(buf + pos, sz > (size_t)pos ? sz - pos : 0, \
                          __VA_ARGS__);                                \
        if (_r < 0) return -1;                                         \
        pos += _r;                                                     \
    } while (0)

/* HELP + TYPE + value in one shot. */
#define GAUGE(name, help, fmt, val)                                    \
    APPEND("# HELP " name " " help "\n"                                \
           "# TYPE " name " gauge\n"                                   \
           name " " fmt "\n", (val))

#define FLAG_GAUGE(name, help, bit)                                    \
    GAUGE(name, help, "%d", (d->flags & (bit)) ? 1 : 0)

int prom_build_metrics(char *buf, size_t sz, const imud_data_t *d,
                       uint64_t packets_total)
{
    if (!buf || sz == 0) return -1;
    int pos = 0;

    GAUGE("imud_up", "1 when a live packet has been received from imud.",
          "%d", d ? 1 : 0);
    APPEND("# HELP imud_packets_total Packets received from the imud stream socket.\n"
           "# TYPE imud_packets_total counter\n"
           "imud_packets_total %" PRIu64 "\n", packets_total);

    if (!d) {
        if (pos >= (int)sz) return -1;
        return pos;
    }

    /* Attitude + heading */
    GAUGE("imud_heading_degrees", "Magnetic heading, 0-360 degrees.",
          "%.4f", d->heading_deg);
    if (d->flags & IMUD_FLAG_DECLINATION_VALID)
        GAUGE("imud_heading_true_degrees", "True heading, 0-360 degrees.",
              "%.4f", d->heading_true_deg);
    GAUGE("imud_roll_radians",  "Roll, NED (+starboard up).",  "%.6f", d->roll);
    GAUGE("imud_pitch_radians", "Pitch, NED (+bow up).",       "%.6f", d->pitch);
    GAUGE("imud_yaw_radians",   "Yaw, NED magnetic.",          "%.6f", d->yaw);
    GAUGE("imud_rate_of_turn_radians_per_second",
          "Rate of turn (+ = turning right).",
          "%.6f", d->rate_of_turn * 0.017453292519943295 / 60.0);

    /* Heave + sea state */
    GAUGE("imud_heave_meters", "Vertical displacement, + up.", "%.4f", d->heave_m);
    GAUGE("imud_heave_rate_meters_per_second", "Vertical velocity, + up.",
          "%.4f", d->heave_rate);
    GAUGE("imud_wave_height_meters",
          "Significant wave height Hs = 4*sigma(heave).", "%.3f", d->wave_height_m);
    GAUGE("imud_wave_period_seconds",
          "Mean zero-crossing wave period Tz; 0 = becalmed / not settled.",
          "%.2f", d->wave_period_s);
    GAUGE("imud_roll_period_seconds",
          "Vessel roll period; 0 = not rolling / not settled.",
          "%.2f", d->roll_period_s);
    GAUGE("imud_roll_amplitude_radians",
          "Significant single roll amplitude, 2*sigma(roll).",
          "%.5f", d->roll_amplitude);
    GAUGE("imud_pitch_period_seconds",
          "Vessel pitch period; 0 = not pitching / not settled.",
          "%.2f", d->pitch_period_s);
    GAUGE("imud_pitch_amplitude_radians",
          "Significant single pitch amplitude, 2*sigma(pitch).",
          "%.5f", d->pitch_amplitude);

    /* Diagnostics */
    GAUGE("imud_mag_anomaly_ratio",
          "EMA of the fractional field-magnitude deviation vs the reference; "
          "magnetic interference / iron-cal drift metric.",
          "%.5f", d->mag_anomaly);
    GAUGE("imud_mag_residual_radians",
          "EMA of the absolute heading innovation; compass calibration health.",
          "%.5f", d->mag_residual);
    GAUGE("imud_accel_quiescence_ratio",
          "EMA of (|a|/g - 1)^2; platform-disturbance metric.",
          "%.6f", d->accel_quiescence);
    GAUGE("imud_gyro_bias_x_radians_per_second", "Estimated gyro bias, body X.",
          "%.6f", d->gyro_bias[0]);
    GAUGE("imud_gyro_bias_y_radians_per_second", "Estimated gyro bias, body Y.",
          "%.6f", d->gyro_bias[1]);
    GAUGE("imud_gyro_bias_z_radians_per_second", "Estimated gyro bias, body Z.",
          "%.6f", d->gyro_bias[2]);
    GAUGE("imud_temperature_celsius", "IMU die temperature.", "%.2f", d->temp_c);

    /* Flag bits as 0/1 gauges */
    FLAG_GAUGE("imud_mag_valid",   "Magnetometer calibrated and healthy.",
               IMUD_FLAG_MAG_VALID);
    FLAG_GAUGE("imud_converged",   "MEKF covariance below the converged threshold.",
               IMUD_FLAG_FUSION_CONVERGED);
    FLAG_GAUGE("imud_heave_valid", "Heave estimator settled.",
               IMUD_FLAG_HEAVE_VALID);
    FLAG_GAUGE("imud_wave_valid",  "Sea-state statistics settled.",
               IMUD_FLAG_WAVE_VALID);
    FLAG_GAUGE("imud_engine_on",   "Engine-vibration detector asserting.",
               IMUD_FLAG_ENGINE_ON);

    if (pos >= (int)sz) return -1;   /* truncated */
    return pos;
}
