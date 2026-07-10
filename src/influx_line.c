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

#define RAD2DEG  57.29577951308232
#define DEG2RAD  0.017453292519943295

/* Append, tracking length; a truncated field is detected as overflow (pos>=sz). */
#define APPEND(...) do {                                              \
        int _r = snprintf(buf + pos, (pos < (int)sz) ? sz - pos : 0,  \
                          __VA_ARGS__);                               \
        if (_r < 0) return -1;                                        \
        pos += _r;                                                    \
    } while (0)

int influx_build_line(char *buf, size_t sz, const imud_packet_t *p,
                      const char *measurement, const char *source_label,
                      bool emit_heave, bool deg)
{
    if (!buf || sz == 0 || !p) return -1;

    int    pos = 0;
    double a   = deg ? RAD2DEG : 1.0;   /* roll/pitch/yaw are radians in the packet */

    /* measurement + tag set, then a space before the field set */
    APPEND("%s,source=%s ", measurement, source_label);

    /* quaternion (unitless), then Euler angles */
    APPEND("qw=%.6f,qx=%.6f,qy=%.6f,qz=%.6f",
           p->quat_w, p->quat_x, p->quat_y, p->quat_z);
    APPEND(",roll=%.5f,pitch=%.5f,yaw=%.5f",
           p->roll * a, p->pitch * a, p->yaw * a);

    /* heading (magnetic); degrees native or → radians */
    APPEND(",heading=%.5f", deg ? p->heading_deg : p->heading_deg * DEG2RAD);

    /* true heading + variation only when declination is known */
    if (p->flags & IMUD_FLAG_DECLINATION_VALID) {
        double ht  = imud_true_heading(p);   /* degrees */
        double var = p->declination_deg;     /* degrees, E+ */
        if (!deg) { ht *= DEG2RAD; var *= DEG2RAD; }
        APPEND(",heading_true=%.5f,variation=%.5f", ht, var);
    }

    /* rate of turn: °/min native, or → rad/s */
    APPEND(",rate_of_turn=%.6f",
           deg ? p->rate_of_turn : p->rate_of_turn * DEG2RAD / 60.0);

    /* heave: this is the diagnostics sink, so — unlike the user-facing bridges —
     * emit from t=0 regardless of settle, and expose the validity flag as a
     * boolean field so the pre-settle transient can be filtered out downstream. */
    if (emit_heave) {
        APPEND(",heave=%.4f,heave_rate=%.4f", p->heave_m, p->heave_rate);
        APPEND(",heave_valid=%s",
               (p->flags & IMUD_FLAG_HEAVE_VALID) ? "t" : "f");
    }

    /* Gyro-bias estimate, its variance (MEKF P diagonal), and the accel-quiescence
     * metric: frame-neutral SI diagnostics, never unit-converted, always emitted. */
    APPEND(",gbias_x=%.6f,gbias_y=%.6f,gbias_z=%.6f",
           p->gyro_bias_x, p->gyro_bias_y, p->gyro_bias_z);
    APPEND(",gbias_var_x=%.3e,gbias_var_y=%.3e,gbias_var_z=%.3e",
           p->gyro_bias_var_x, p->gyro_bias_var_y, p->gyro_bias_var_z);
    APPEND(",quiescence=%.6f", p->accel_quiescence);

    APPEND(",temp=%.2f,seq=%ui", p->temp_c, p->imu_seq);

    /* space, then the nanosecond timestamp */
    APPEND(" %llu", (unsigned long long)p->ts_wall_ns);

    if (pos >= (int)sz) return -1;   /* truncated */
    return pos;
}
