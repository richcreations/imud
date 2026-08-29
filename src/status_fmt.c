/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * status_fmt.c — build the imud-status report text
 *
 * Moved out of main.c's write_status_response() so it can be tested; see
 * include/status_fmt.h.  The text is unchanged.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "status_fmt.h"
#include "mhz.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

size_t status_format(char *buf, size_t sz, const status_input_t *in)
{
    if (!buf || sz == 0) return 0;
    buf[0] = '\0';

    const imud_config_t *cfg   = in->cfg;
    const fused_state_t *state = &in->state;
    const imu_stats_t   *st    = &in->stats;

    float pitch_deg = state->pitch * (float)(180.0 / M_PI);
    float roll_deg  = state->roll  * (float)(180.0 / M_PI);
    float cov_trace = state->cov[0] + state->cov[4] + state->cov[8];
    bool  converged = (state->flags & FLAG_FUSION_CONVERGED) != 0;

    int hh = (int)(in->uptime_s / 3600);
    int mm = (int)((in->uptime_s % 3600) / 60);
    int ss = (int)(in->uptime_s % 60);

    /* WS() tracks (wp, wr) so snprintf truncation never advances wp past the
     * buffer end — safe even if a single line exceeds the remaining space.
     * Once wr reaches 1 only the NUL is left and every later line is dropped. */
    char  *wp = buf;
    char   rbuf[16];         /* MHZ_STR scratch */
    size_t wr = sz;

#define WS(fmt, ...) do { \
        if (wr > 1) { \
            int _r = snprintf(wp, wr, fmt, ##__VA_ARGS__); \
            if (_r > 0 && (size_t)_r < wr) { wp += _r; wr -= (size_t)_r; } \
            else if (_r > 0)                { wp += wr - 1; wr = 1; } \
        } } while (0)

    WS("Chip IDs:       %s 0x%02X   %s 0x%02X\n",
        cfg->imu_driver, cfg->imu_addr,
        cfg->mag_driver, cfg->mag_addr);

    WS("IMU ODR:        %s Hz  (FIFO watermark: %d sample-sets)\n",
        MHZ_STR(rbuf, cfg->imu_odr_mhz), cfg->imu_fifo_wm);

    WS("Mag ODR:        %s Hz  (SET every %.0f s%s)\n",
        MHZ_STR(rbuf, cfg->mag_odr_mhz), cfg->mag_set_period_s,
        cfg->mag_set_period_s > 0 ? "" : ", disabled");

    WS("Fusion:         MEKF %s  cov_trace=%.2e rad2\n",
        converged ? "converged" : "converging",
        cov_trace);

    WS("Calibration:    accel %s  gyro %s  mag %s\n",
        (state->flags & FLAG_ACCEL_CAL) ? "yes" : "no",
        (state->flags & FLAG_GYRO_CAL)  ? "yes" : "no",
        (state->flags & FLAG_MAG_CAL)   ? "yes" : "no");

    WS("Attitude:       pitch=%.1f  roll=%.1f  heading=%.1f M\n",
        pitch_deg, roll_deg, state->heading_deg);

    /*
     * Three states, and the operator needs to tell them apart.  MAG_VALID is
     * a calibrated, healthy, running yaw update; MAG_UNCAL is a running yaw
     * update from an uncalibrated field; neither is dead reckoning.  The
     * "Calibration: mag no" line above is a statement about a FILE, this is a
     * statement about whether the number can be believed.
     *
     * Say it next to the number, because the number itself looks entirely
     * reasonable either way.  Measured on a static bench with the mag not
     * fused at all, heading walked 220 degrees in 24 minutes while pitch and
     * roll stayed correct to a tenth of a degree.
     */
    if (state->flags & FLAG_MAG_UNCAL)
        WS("                heading is UNCALIBRATED — fused heading-only from\n"
           "                the raw field, so it is bounded and repeatable but\n"
           "                offset by the uncorrected hard iron.  Run\n"
           "                `imud-cal mag` for an accurate number.\n");
    else if (!(state->flags & FLAG_MAG_VALID))
        WS("                heading is DEAD RECKONED — the magnetometer is not\n"
           "                being fused, so it drifts at the gyro bias rate\n"
           "                without bound.  Run `imud-cal mag`.\n");

    if (state->flags & FLAG_DECLINATION_VALID) {
        float true_hdg = fmodf(state->heading_deg + state->declination_deg
                               + 360.0f, 360.0f);
        WS("Declination:    %+.2f E  (true heading %.1f T)\n",
            state->declination_deg, true_hdg);
    } else {
        WS("Declination:    unknown  (no true heading output)\n");
    }

    if (cfg->heave_tau_s > 0.0f)
        WS("Heave:          %+.2f m\n", state->heave_m);

    if (cfg->heave_tau_s > 0.0f && cfg->wave_tau_s > 0.0f) {
        if (state->flags & FLAG_WAVE_VALID)
            WS("Sea state:      Hs %.2f m  Tz %.1f s  roll period %.1f s\n",
                state->wave_height_m, state->wave_period_s,
                state->roll_period_s);
        else
            WS("Sea state:      settling\n");
    }

    if (cfg->capture_enabled) {
        if (in->capture_active)
            WS("Capture:        %s  (%.1f MB, %llu dropped)\n",
                in->capture_path ? in->capture_path : "",
                (double)in->capture_bytes / (1024.0 * 1024.0),
                (unsigned long long)in->capture_drops);
        else
            WS("Capture:        stopped (see log)\n");
    }

    if (cfg->nmea_enabled && cfg->nmea_tcp_enabled) {
        WS("NMEA out:       %d Hz  (UDP port %d, TCP port %d)\n",
            cfg->nmea_rate_hz, cfg->nmea_dest_port, cfg->nmea_tcp_port);
    } else if (cfg->nmea_enabled) {
        WS("NMEA out:       %d Hz  (port %d)\n",
            cfg->nmea_rate_hz, cfg->nmea_dest_port);
    } else if (cfg->nmea_tcp_enabled) {
        WS("NMEA out:       %d Hz  (TCP port %d)\n",
            cfg->nmea_rate_hz, cfg->nmea_tcp_port);
    } else {
        WS("NMEA out:       disabled\n");
    }

    if (cfg->highrate_enabled) {
        WS("Hi-rate out:    %d Hz  (port %d, %s)\n",
            cfg->highrate_rate_hz, cfg->highrate_dest_port,
            cfg->highrate_coord_frame);
    } else {
        WS("Hi-rate out:    disabled\n");
    }

    WS("IMU samples:    %llu  overflows: %llu\n",
        (unsigned long long)st->imu_samples,
        (unsigned long long)st->fifo_overflows);

    WS("Uptime:         %02d:%02d:%02d\n", hh, mm, ss);

    /* Last few WARN/ERROR lines — what went wrong while unattended. */
    if (in->recent && in->recent[0])
        WS("Recent warnings:\n%s", in->recent);

#undef WS

    return (size_t)(wp - buf);
}
