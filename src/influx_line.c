/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * influx_line.c — InfluxDB line-protocol encoder (see influx_line.h)
 *
 * Attitude uses imud's native NED convention (roll + = starboard up, pitch +
 * = bow up); no sign is flipped. `measurement` and `source_label` are assumed
 * to be simple identifiers (no spaces/commas) — the defaults are.
 */

#include "influx_line.h"

#include <stdio.h>
#include <string.h>

#define RAD2DEG  57.29577951308232
#define DEG2RAD  0.017453292519943295

/* Append, tracking length; a truncated field is detected as overflow (pos>=sz). */
#define APPEND(...) do {                                              \
        int _r = snprintf(buf + pos, (pos < (int)sz) ? sz - pos : 0,  \
                          __VA_ARGS__);                               \
        if (_r < 0) return -1;                                        \
        pos += _r;                                                    \
    } while (0)

/* The five names, indexed by level; NAMES[0] is unused so NAMES[n] is level n. */
static const char *const NAMES[] = {
    "", "attitude", "navigation", "seastate", "health", "full"
};

int influx_detail_from_name(const char *name)
{
    if (!name) return -1;
    for (int i = INFLUX_DETAIL_ATTITUDE; i <= INFLUX_DETAIL_FULL; i++)
        if (strcmp(name, NAMES[i]) == 0) return i;
    return -1;
}

const char *influx_detail_name(int detail)
{
    if (detail < INFLUX_DETAIL_ATTITUDE || detail > INFLUX_DETAIL_FULL)
        return NAMES[INFLUX_DETAIL_HEALTH];
    return NAMES[detail];
}

int influx_build_line(char *buf, size_t sz, const imud_data_t *d,
                      const char *measurement, const char *source_label,
                      bool emit_heave, bool deg, int detail)
{
    if (!buf || sz == 0 || !d) return -1;

    int    pos = 0;
    double a   = deg ? RAD2DEG : 1.0;   /* roll/pitch/yaw are radians in the view */

    /* A level outside the five means a config this build does not understand.
     * Fall back to the default rather than emitting an empty field set, which
     * is not valid line protocol and would be rejected point by point. */
    if (detail < INFLUX_DETAIL_ATTITUDE || detail > INFLUX_DETAIL_FULL)
        detail = INFLUX_DETAIL_HEALTH;

    /* measurement + tag set, then a space before the field set */
    APPEND("%s,source=%s ", measurement, source_label);

    /* quaternion (unitless), then Euler angles */
    APPEND("qw=%.6f,qx=%.6f,qy=%.6f,qz=%.6f",
           d->quat[0], d->quat[1], d->quat[2], d->quat[3]);
    APPEND(",roll=%.5f,pitch=%.5f,yaw=%.5f",
           d->roll * a, d->pitch * a, d->yaw * a);

    /* heading (magnetic); degrees native or → radians.
     *
     * Emitted unconditionally, unlike the user-facing bridges: this is the
     * diagnostics sink, and the drift of an unreferenced heading is itself
     * something you would come here to measure. The two booleans below say
     * what the number means, the same way heave_valid does for heave —
     * mag_absent means no magnetometer is fitted at all, heading_ref means
     * the magnetometer is being fused and the number really is magnetic. */
    APPEND(",heading=%.5f", deg ? d->heading_deg : d->heading_deg * DEG2RAD);
    APPEND(",heading_ref=%s",
           (d->flags & (IMUD_FLAG_MAG_VALID | IMUD_FLAG_MAG_UNCAL)) ? "t" : "f");

    /* rate of turn: °/min native, or → rad/s */
    APPEND(",rate_of_turn=%.6f",
           deg ? d->rate_of_turn : d->rate_of_turn * DEG2RAD / 60.0);

    APPEND(",temp=%.2f,seq=%ui", d->temp_c, d->imu_seq);

    /* ── navigation ──────────────────────────────────────────────────────── */
    if (detail >= INFLUX_DETAIL_NAVIGATION) {
        APPEND(",mag_absent=%s",
               (d->flags_ext & IMUD_FLAG_EXT_MAG_ABSENT) ? "t" : "f");

        /* true heading + variation only when declination is known; libimud
         * has already added the variation and wrapped into [0, 360). */
        if (d->flags & IMUD_FLAG_DECLINATION_VALID) {
            double ht  = d->heading_true_deg;    /* degrees */
            double var = d->declination_deg;     /* degrees, E+ */
            if (!deg) { ht *= DEG2RAD; var *= DEG2RAD; }
            APPEND(",heading_true=%.5f,variation=%.5f", ht, var);
    }

    /* heave: this is the diagnostics sink, so — unlike the user-facing bridges —
     * emit from t=0 regardless of settle, and expose the validity flag as a
     * boolean field so the pre-settle transient can be filtered out downstream. */
    if (emit_heave) {
        APPEND(",heave=%.4f,heave_rate=%.4f", d->heave_m, d->heave_rate);
        APPEND(",heave_valid=%s",
               (d->flags & IMUD_FLAG_HEAVE_VALID) ? "t" : "f");
    }
    }

    /* ── seastate ────────────────────────────────────────────────────────── */
    if (detail >= INFLUX_DETAIL_SEASTATE) {
        /* Sea state (v14): same diagnostics-sink policy as heave — always emitted
         * (values are 0.0 until the estimator settles) with the validity flag as
         * a boolean field. Frame-neutral SI (m, s, rad), never unit-converted. */
        APPEND(",wave_height=%.3f,wave_period=%.2f,roll_period=%.2f",
               d->wave_height_m, d->wave_period_s, d->roll_period_s);
        APPEND(",roll_amplitude=%.4f,pitch_period=%.2f,pitch_amplitude=%.4f",
               d->roll_amplitude, d->pitch_period_s, d->pitch_amplitude);
        APPEND(",wave_valid=%s", (d->flags & IMUD_FLAG_WAVE_VALID) ? "t" : "f");
    }

    /* ── health ──────────────────────────────────────────────────────────── */
    if (detail >= INFLUX_DETAIL_HEALTH) {
        /* Gyro-bias estimate, its variance (MEKF P diagonal), and the accel-quiescence
         * metric: frame-neutral SI diagnostics, never unit-converted, always emitted. */
        APPEND(",gbias_x=%.6f,gbias_y=%.6f,gbias_z=%.6f",
               d->gyro_bias[0], d->gyro_bias[1], d->gyro_bias[2]);
        APPEND(",gbias_var_x=%.3e,gbias_var_y=%.3e,gbias_var_z=%.3e",
               d->gyro_bias_var[0], d->gyro_bias_var[1], d->gyro_bias_var[2]);
        APPEND(",quiescence=%.6f", d->accel_quiescence);

        /* Compass health (v14): always-on diagnostics, unitless / rad. */
        APPEND(",mag_anomaly=%.5f,mag_residual=%.5f",
               d->mag_anomaly, d->mag_residual);

        /* MEKF update-gate health (v17): same always-on diagnostics policy. */
        APPEND(",innov_weight=%.5f,innov_reject=%.5f",
               d->innov_weight, d->innov_reject);

        /* MEKF measurement-model consistency (v17): rolling NIS, 1.0 = the
         * covariance is consistent with the innovations actually seen. */
        APPEND(",nis_accel=%.5f,nis_mag=%.5f", d->nis_accel, d->nis_mag);
    }

    /* ── full ────────────────────────────────────────────────────────────── */
    if (detail >= INFLUX_DETAIL_FULL) {
        /* The sensor vectors, calibrated then raw, in the view's own SI units:
         * accel m/s², gyro rad/s, mag µT. Never unit-converted — `deg` governs the
         * angle fields, and turning a gyro rate into °/s here would make gyro_x
         * disagree with gbias_x above, which is the number you compare it to. */
        APPEND(",accel_x=%.5f,accel_y=%.5f,accel_z=%.5f",
               d->accel[0], d->accel[1], d->accel[2]);
        APPEND(",accel_raw_x=%.5f,accel_raw_y=%.5f,accel_raw_z=%.5f",
               d->accel_raw[0], d->accel_raw[1], d->accel_raw[2]);
        APPEND(",gyro_x=%.6f,gyro_y=%.6f,gyro_z=%.6f",
               d->gyro[0], d->gyro[1], d->gyro[2]);
        APPEND(",gyro_raw_x=%.6f,gyro_raw_y=%.6f,gyro_raw_z=%.6f",
               d->gyro_raw[0], d->gyro_raw[1], d->gyro_raw[2]);
        APPEND(",mag_x=%.4f,mag_y=%.4f,mag_z=%.4f",
               d->mag[0], d->mag[1], d->mag[2]);
        APPEND(",mag_raw_x=%.4f,mag_raw_y=%.4f,mag_raw_z=%.4f",
               d->mag_raw[0], d->mag_raw[1], d->mag_raw[2]);

        /* Timestamps other than ts_wall_ns, which is the point's own time below.
         * Integer fields (the i suffix) so Influx does not store them as doubles
         * and lose nanoseconds to the 53-bit mantissa. */
        APPEND(",ts_tai_ns=%llui,ts_chip_ticks=%ui,anchor_gen=%ui",
               (unsigned long long)d->ts_tai_ns, d->ts_chip_ticks, d->anchor_gen);
    }

    /* space, then the nanosecond timestamp */
    APPEND(" %llu", (unsigned long long)d->ts_wall_ns);

    if (pos >= (int)sz) return -1;   /* truncated */
    return pos;
}
