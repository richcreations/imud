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
#include "version.h"

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

/* ── JSON ─────────────────────────────────────────────────────────────────── */

/*
 * Escape s into out as a JSON string, the surrounding quotes included.
 * Returns the length, or 0 if it does not fit — the caller then leaves the
 * element out rather than emitting a broken one.
 *
 * Bytes above 0x7F pass through: every string here is a config value, a path
 * or a log line, so it is already UTF-8, and \u-escaping a raw byte would
 * transcode it as Latin-1 rather than preserve it.
 */
static size_t json_quote(char *out, size_t osz, const char *s, size_t n)
{
    size_t o = 0;
    if (osz < 3) return 0;
    out[o++] = '"';
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        char esc[7];
        size_t elen;
        switch (c) {
        case '"':  memcpy(esc, "\\\"", 2); elen = 2; break;
        case '\\': memcpy(esc, "\\\\", 2); elen = 2; break;
        case '\b': memcpy(esc, "\\b",  2); elen = 2; break;
        case '\f': memcpy(esc, "\\f",  2); elen = 2; break;
        case '\n': memcpy(esc, "\\n",  2); elen = 2; break;
        case '\r': memcpy(esc, "\\r",  2); elen = 2; break;
        case '\t': memcpy(esc, "\\t",  2); elen = 2; break;
        default:
            if (c < 0x20) { snprintf(esc, sizeof esc, "\\u%04x", c); elen = 6; }
            else          { esc[0] = (char)c; elen = 1; }
        }
        if (o + elen + 2 > osz) return 0;    /* + closing quote and NUL */
        memcpy(out + o, esc, elen);
        o += elen;
    }
    out[o++] = '"';
    out[o]   = '\0';
    return o;
}

size_t status_format_json(char *buf, size_t sz, const status_input_t *in)
{
    if (!buf || sz == 0) return 0;
    buf[0] = '\0';

    const imud_config_t *cfg   = in->cfg;
    const fused_state_t *state = &in->state;
    const imu_stats_t   *st    = &in->stats;

    bool   over = false;         /* latches the first overflow */
    char  *wp   = buf;
    size_t wr   = sz;
    char   q[512];               /* json_quote scratch */

    /* As WS() above, but an overflow is fatal to the whole object rather than
     * merely dropping the rest of it. */
#define JW(fmt, ...) do { \
        if (wr > 1) { \
            int _r = snprintf(wp, wr, fmt, ##__VA_ARGS__); \
            if (_r < 0)                      { over = true; } \
            else if ((size_t)_r < wr)        { wp += _r; wr -= (size_t)_r; } \
            else { over = true; wp += wr - 1; wr = 1; } \
        } else { over = true; } \
    } while (0)

#define JSTR(key, s) do { \
        const char *_s = (s); \
        if (_s && json_quote(q, sizeof q, _s, strlen(_s))) \
            JW("\"" key "\":%s", q); \
        else \
            JW("\"" key "\":null"); \
    } while (0)

#define JBOOL(key, v)  JW("\"" key "\":%s", (v) ? "true" : "false")

/* A float as a JSON number, or null when it is not finite or its subsystem is
 * off.  "nan" and "inf" are not JSON, and a divergent filter is exactly when a
 * monitoring script must still get a parsable answer. */
#define JNUM(key, fmt, v, live) do { \
        double _v = (double)(v); \
        if ((live) && isfinite(_v)) JW("\"" key "\":" fmt, _v); \
        else                        JW("\"" key "\":null"); \
    } while (0)

    bool converged = (state->flags & FLAG_FUSION_CONVERGED) != 0;
    bool decl_ok   = (state->flags & FLAG_DECLINATION_VALID) != 0;
    bool heave_on  = cfg->heave_tau_s > 0.0f;
    bool wave_on   = heave_on && cfg->wave_tau_s > 0.0f;
    bool wave_ok   = wave_on && (state->flags & FLAG_WAVE_VALID);

    /* The three heading states the text report spells out in a paragraph.  A
     * consumer needs to tell them apart for the same reason an operator does:
     * a dead-reckoned heading looks entirely reasonable. */
    const char *hsrc = (state->flags & FLAG_MAG_UNCAL)  ? "uncalibrated"
                     : (state->flags & FLAG_MAG_VALID)  ? "calibrated"
                     :                                    "dead_reckoned";

    JW("{\"imud_version\":\"%s\"", IMUD_VERSION_STR);
    JW(",\"uptime_s\":%lld", (long long)in->uptime_s);

    JW(",\"imu\":{");
    JSTR("driver", cfg->imu_driver);
    JW(",\"addr\":%d,\"odr_mhz\":%d,\"fifo_watermark\":%d}",
       cfg->imu_addr, cfg->imu_odr_mhz, cfg->imu_fifo_wm);

    JW(",\"mag\":{");
    JSTR("driver", cfg->mag_driver);
    JW(",\"addr\":%d,\"odr_mhz\":%d,", cfg->mag_addr, cfg->mag_odr_mhz);
    JNUM("set_period_s", "%.6g", cfg->mag_set_period_s, true);
    JW("}");

    JW(",\"fusion\":{");
    JBOOL("converged", converged);
    JW(",");
    JNUM("cov_trace_rad2", "%.6g",
         state->cov[0] + state->cov[4] + state->cov[8], true);
    JW("}");

    JW(",\"calibration\":{");
    JBOOL("accel", state->flags & FLAG_ACCEL_CAL);
    JW(",");
    JBOOL("gyro", state->flags & FLAG_GYRO_CAL);
    JW(",");
    JBOOL("mag", state->flags & FLAG_MAG_CAL);
    JW("}");

    JW(",\"attitude\":{");
    JNUM("pitch_deg", "%.6g", state->pitch * (180.0 / M_PI), true);
    JW(",");
    JNUM("roll_deg", "%.6g", state->roll * (180.0 / M_PI), true);
    JW(",");
    JNUM("heading_deg", "%.6g", state->heading_deg, true);
    JW(",");
    JSTR("heading_source", hsrc);
    JW("}");

    JW(",\"declination\":{");
    JBOOL("valid", decl_ok);
    JW(",");
    JNUM("declination_deg", "%.6g", state->declination_deg, decl_ok);
    JW(",");
    JNUM("true_heading_deg", "%.6g",
         fmodf(state->heading_deg + state->declination_deg + 360.0f, 360.0f),
         decl_ok);
    JW("}");

    JW(",\"heave\":{");
    JBOOL("enabled", heave_on);
    JW(",");
    JNUM("heave_m", "%.6g", state->heave_m, heave_on);
    JW("}");

    JW(",\"sea_state\":{");
    JBOOL("enabled", wave_on);
    JW(",");
    JBOOL("valid", wave_ok);
    JW(",");
    JNUM("wave_height_m", "%.6g", state->wave_height_m, wave_ok);
    JW(",");
    JNUM("wave_period_s", "%.6g", state->wave_period_s, wave_ok);
    JW(",");
    JNUM("roll_period_s", "%.6g", state->roll_period_s, wave_ok);
    JW("}");

    JW(",\"capture\":{");
    JBOOL("enabled", cfg->capture_enabled);
    JW(",");
    JBOOL("active", cfg->capture_enabled && in->capture_active);
    JW(",");
    if (cfg->capture_enabled && in->capture_active) {
        JSTR("path", in->capture_path);
        JW(",\"bytes\":%llu,\"drops\":%llu}",
           (unsigned long long)in->capture_bytes,
           (unsigned long long)in->capture_drops);
    } else {
        JW("\"path\":null,\"bytes\":null,\"drops\":null}");
    }

    JW(",\"nmea\":{");
    JBOOL("udp_enabled", cfg->nmea_enabled);
    JW(",");
    JBOOL("tcp_enabled", cfg->nmea_tcp_enabled);
    JW(",\"rate_hz\":%d,\"udp_port\":%d,\"tcp_port\":%d}",
       cfg->nmea_rate_hz, cfg->nmea_dest_port, cfg->nmea_tcp_port);

    JW(",\"highrate\":{");
    JBOOL("enabled", cfg->highrate_enabled);
    JW(",\"rate_hz\":%d,\"port\":%d,",
       cfg->highrate_rate_hz, cfg->highrate_dest_port);
    JSTR("coord_frame", cfg->highrate_coord_frame);
    JW("}");

    JW(",\"counters\":{\"imu_samples\":%llu,\"fifo_overflows\":%llu}",
       (unsigned long long)st->imu_samples,
       (unsigned long long)st->fifo_overflows);

    /*
     * The last WARN/ERROR lines, one array element each, log_recent()'s
     * two-space indent stripped.  Every other field above is bounded by the
     * config; this is the one that can be long, so an element is added only
     * while `]}\n` still fits after it.  Dropping the tail keeps the object
     * valid, which is worth more to a consumer than the last warning.
     */
    JW(",\"warnings\":[");
    int nw = 0;
    for (const char *p = in->recent; p && *p; ) {
        const char *eol = strchr(p, '\n');
        size_t      len = eol ? (size_t)(eol - p) : strlen(p);
        while (len && (*p == ' ' || *p == '\t')) { p++; len--; }
        if (len) {
            size_t qn = json_quote(q, sizeof q, p, len);
            if (qn && qn + (nw ? 1u : 0u) + 3u < wr) {
                JW("%s%s", nw ? "," : "", q);
                nw++;
            }
        }
        if (!eol) break;
        p = eol + 1;
    }
    JW("]}\n");

#undef JNUM
#undef JBOOL
#undef JSTR
#undef JW

    if (over) { buf[0] = '\0'; return 0; }
    return (size_t)(wp - buf);
}
