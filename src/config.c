/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * config.c — TOML config parser for imud (§9)
 *
 * No external dependencies. Handles sections, key=value, inline comments,
 * quoted strings, decimal/hex integers, doubles, booleans, and ~/paths.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "config.h"
#include "log.h"

/* ── String helpers ─────────────────────────────────────────────────────── */

static char *lstrip(char *s)
{
    while (*s && isspace((unsigned char)*s)) s++;
    return s;
}

static void rstrip(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = '\0';
}

/* Remove trailing inline comment (#), but not one inside a quoted string. */
static void strip_comment(char *s)
{
    bool in_q = false;
    for (char *p = s; *p; p++) {
        if (*p == '"') in_q = !in_q;
        if (!in_q && *p == '#') { *p = '\0'; break; }
    }
}

/* Replace a leading ~/ with $HOME/. */
static void expand_tilde(char *buf, size_t bufsz)
{
    if (buf[0] != '~' || buf[1] != '/') return;
    const char *home = getenv("HOME");
    if (!home) return;
    /* Expand in-place: shift the path suffix right to make room for $HOME. */
    size_t hlen = strlen(home);
    size_t rlen = strlen(buf + 1);   /* length of "/rest/of/path" */
    if (hlen + rlen >= bufsz) return; /* would overflow — leave unexpanded */
    memmove(buf + hlen, buf + 1, rlen + 1);  /* move "/rest..." right */
    memcpy(buf, home, hlen);                 /* write $HOME at front */
}

/*
 * Copy TOML string value into out: strip surrounding quotes, expand tilde.
 * Returns false if the value did not fit and was truncated — callers surface
 * that, because a silently shortened path (a socket, a cal file) binds or
 * opens something the user never asked for.
 */
static bool copy_str(const char *val, char *out, size_t outsz)
{
    int n;
    if (*val == '"') {
        val++;
        size_t len = strlen(val);
        if (len > 0 && val[len - 1] == '"') len--;
        n = snprintf(out, outsz, "%.*s", (int)len, val);
    } else {
        n = snprintf(out, outsz, "%s", val);
    }
    expand_tilde(out, outsz);
    return n >= 0 && (size_t)n < outsz;
}

static bool parse_bool(const char *val, bool *out)
{
    if (strcmp(val, "true")  == 0) { *out = true;  return true; }
    if (strcmp(val, "false") == 0) { *out = false; return true; }
    return false;
}

static bool parse_int(const char *val, int *out)
{
    char *end;
    long v = strtol(val, &end, 0);   /* base 0: accepts 0x hex */
    if (end == val || *end != '\0') return false;
    *out = (int)v;
    return true;
}

static bool parse_double(const char *val, double *out)
{
    char *end;
    *out = strtod(val, &end);
    return (end != val && *end == '\0');
}

/* ── Defaults (spec §9) ─────────────────────────────────────────────────── */

void config_defaults(imud_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    /* [device] */
    snprintf(cfg->i2c_bus,   sizeof(cfg->i2c_bus),   "/dev/i2c-1");
    snprintf(cfg->gpio_chip, sizeof(cfg->gpio_chip), "gpiochip0");
    cfg->sim_file[0] = '\0';
    cfg->sim_loop    = false;
    cfg->sim_speed   = 1.0f;

    /* [capture] */
    cfg->capture_enabled   = false;
    snprintf(cfg->capture_dir, sizeof(cfg->capture_dir), "/var/lib/imud");
    cfg->capture_max_mb    = 256;
    cfg->capture_max_files = 8;
    cfg->capture_flush_s   = 5;

    /* [imu] */
    snprintf(cfg->imu_driver, sizeof(cfg->imu_driver), "ism330dhcx");
    cfg->imu_addr     = 0x6B;
    cfg->imu_int_gpio = 17;
    cfg->imu_odr_hz   = 833;
    cfg->imu_accel_g  = 8;
    cfg->imu_gyro_dps = 2000;
    cfg->imu_fifo_wm  = 64;

    /* [mag] */
    snprintf(cfg->mag_driver, sizeof(cfg->mag_driver), "mmc5983ma");
    cfg->mag_addr         = 0x30;
    cfg->mag_int_gpio     = 27;
    cfg->mag_odr_hz       = 100;
    cfg->mag_set_period_s = 5.0f;

    /* [fusion] */
    cfg->mag_yaw_only      = true;   /* marine default: mag corrects heading only */
    cfg->heave_tau_s       = 12.0f;
    cfg->wave_tau_s        = 120.0f;
    cfg->mekf_gyro_noise   = 0.007;   /* tuned Kalman Q, intentionally above the datasheet gyro floor; see imud.conf */
    cfg->mekf_gyro_bias    = 0.00015;
    cfg->mekf_accel_noise  = 0.0022;
    cfg->mekf_mag_noise    = 0.0004;
    /* Gauss–Markov wave-acceleration state; tuned over the 12-seed wave
     * benchmark, see docs/math.md §4.7. */
    cfg->mekf_wave_accel       = 0.8;
    cfg->mekf_wave_accel_tau_s = 0.5;
    /* Measured residual dip error after the daemon's 5 s averaged alignment in
     * a seaway is +0.86°; 1.0 is that rounded up. See docs/math.md §4.8.1. */
    cfg->mekf_mag_dip_sigma_deg = 1.0;
    /* Strong-anomaly threshold (nearby iron/magnet). 0.05 G ≈ 10% of the
     * Earth field: transient attitude wobble in a seaway must not trip it —
     * fine-grained consistency is handled by the χ² innovation gates. */
    cfg->mag_reject_gauss          = 0.05;
    cfg->accel_skip_thresh         = 0.05;
    cfg->engine_vibration_g2       = 0.0;    /* disabled */
    cfg->engine_accel_skip_thresh  = 0.20;

    /* [calibration] */
    snprintf(cfg->cal_file, sizeof(cfg->cal_file), "/etc/imud/cal.json");
    cfg->startup_settle_sec = 5.0;
    cfg->gyro_bias_sec = 2.0;
    /* 5 s, not the ~1 s this used to average. One second is a fifth of a
     * typical roll period, so in a seaway it aligns to an arbitrary point in
     * the cycle: measured over the wave benchmark, a 1 s window gives 47.7°
     * of attitude RMS in the marine default against 2.2° at 5 s. Everything
     * is flat from ~3 s. See docs/math.md §4.3. */
    cfg->align_window_sec = 5.0;

    /* [nmea] — off by default since 1.6: a stock daemon emits only on the
     * local [stream] socket; every network output is an explicit opt-in. */
    cfg->nmea_enabled = false;
    cfg->nmea_rate_hz = 10;
    snprintf(cfg->nmea_dest_addr, sizeof(cfg->nmea_dest_addr), "255.255.255.255");
    cfg->nmea_dest_port = 10110;
    cfg->nmea_tcp_enabled = false;
    snprintf(cfg->nmea_tcp_bind_addr, sizeof(cfg->nmea_tcp_bind_addr), "0.0.0.0");
    cfg->nmea_tcp_port = 10110;

    /* [highrate] */
    cfg->highrate_enabled = false;  /* opt-in; primary output is NMEA */
    cfg->highrate_rate_hz = 500;
    snprintf(cfg->highrate_dest_addr, sizeof(cfg->highrate_dest_addr), "239.255.0.1");
    cfg->highrate_dest_port = 10111;
    snprintf(cfg->highrate_coord_frame, sizeof(cfg->highrate_coord_frame), "NED");

    /* [stream] — on by default since 1.6: the local AF_UNIX socket is the
     * one output a stock daemon provides (bridges and libimud read it). */
    cfg->stream_enabled = true;
    snprintf(cfg->stream_socket, sizeof(cfg->stream_socket),
             "/run/imud/imud-stream.sock");
    cfg->stream_rate_hz = 100;
    cfg->stream_tcp_enabled = false;
    snprintf(cfg->stream_tcp_bind_addr, sizeof(cfg->stream_tcp_bind_addr), "0.0.0.0");
    cfg->stream_tcp_port = 10112;

    /* [imud-signalk] */
    cfg->sk_enabled   = false;
    cfg->sk_udp_enabled = false;
    snprintf(cfg->sk_dest_addr, sizeof(cfg->sk_dest_addr), "127.0.0.1");
    cfg->sk_dest_port = 10113;
    cfg->sk_rate_hz   = 10;
    snprintf(cfg->sk_source_label, sizeof(cfg->sk_source_label), "imud");
    cfg->sk_tcp_enabled = false;
    snprintf(cfg->sk_tcp_bind_addr, sizeof(cfg->sk_tcp_bind_addr), "0.0.0.0");
    cfg->sk_tcp_port  = 10113;

    /* Bridge-shared keys (imud-signalk / imud-mqtt). */
    cfg->publish_heave = true;   /* imud's heave estimator is on by default */

    /* [imud-mqtt] */
    cfg->mqtt_enabled       = false;
    cfg->mqtt_broker_enabled = false;
    snprintf(cfg->mqtt_broker_addr, sizeof(cfg->mqtt_broker_addr), "127.0.0.1");
    cfg->mqtt_broker_port = 1883;
    snprintf(cfg->mqtt_client_id,    sizeof(cfg->mqtt_client_id),    "imud");
    snprintf(cfg->mqtt_topic_prefix, sizeof(cfg->mqtt_topic_prefix), "imud");
    cfg->mqtt_rate_hz     = 5;
    cfg->mqtt_qos         = 0;
    cfg->mqtt_retain      = true;
    snprintf(cfg->mqtt_units, sizeof(cfg->mqtt_units), "deg");
    cfg->mqtt_username[0]   = '\0';
    cfg->mqtt_password[0]   = '\0';
    cfg->mqtt_tls         = false;
    cfg->mqtt_tls_cafile[0] = '\0';
    cfg->mqtt_keepalive_s = 30;
    cfg->mqtt_ha_discovery = true;
    snprintf(cfg->mqtt_ha_prefix, sizeof(cfg->mqtt_ha_prefix), "homeassistant");

    /* [imud-influxdb] */
    cfg->influx_enabled = false;
    cfg->influx_transport[0] = '\0';  /* empty = unset; legacy key mapped in bridge */
    cfg->influx_rate_hz = 10;
    snprintf(cfg->influx_measurement,  sizeof(cfg->influx_measurement),  "imud");
    snprintf(cfg->influx_source_label, sizeof(cfg->influx_source_label), "imud");
    snprintf(cfg->influx_units,        sizeof(cfg->influx_units),        "deg");
    cfg->influx_udp_enabled = false;
    snprintf(cfg->influx_udp_addr,     sizeof(cfg->influx_udp_addr),     "127.0.0.1");
    cfg->influx_udp_port = 8089;
    cfg->influx_http_enabled = false;
    snprintf(cfg->influx_http_host,    sizeof(cfg->influx_http_host),    "127.0.0.1");
    cfg->influx_http_port = 8086;
    snprintf(cfg->influx_http_path,    sizeof(cfg->influx_http_path),
             "/write?db=imud&precision=ns");
    cfg->influx_http_token[0] = '\0';

    /* [imud-prometheus] */
    cfg->prom_enabled = false;
    cfg->prom_http_enabled = false;
    snprintf(cfg->prom_listen_addr, sizeof(cfg->prom_listen_addr), "127.0.0.1");
    cfg->prom_listen_port = 9815;

    /* [imud-mavlink] */
    cfg->mav_enabled     = false;
    cfg->mav_version     = 2;
    cfg->mav_system_id   = 1;
    cfg->mav_component_id = 1;
    cfg->mav_rate_hz     = 10;
    cfg->mav_send_attitude = true;
    cfg->mav_send_attitude_quaternion = true;
    cfg->mav_udp_enabled = false;
    snprintf(cfg->mav_udp_addr, sizeof(cfg->mav_udp_addr), "127.0.0.1");
    cfg->mav_udp_port    = 14550;
    cfg->mav_serial_enabled = false;
    snprintf(cfg->mav_serial_device, sizeof(cfg->mav_serial_device), "/dev/serial0");
    cfg->mav_serial_baud = 57600;
    cfg->mav_tcp_enabled = false;
    snprintf(cfg->mav_tcp_bind_addr, sizeof(cfg->mav_tcp_bind_addr), "0.0.0.0");
    cfg->mav_tcp_port    = 5760;

    /* [position] */
    cfg->pos_declination_deg = 0.0f;   /* disabled; set to local declination to enable */
    cfg->pos_declination_valid = false;
    cfg->pos_lat_deg  = 0.0;
    cfg->pos_lon_deg  = 0.0;
    cfg->pos_wmm_file[0] = '\0';   /* "" = auto-resolve, see resolve_wmm_file() */

    cfg->pos_gpsd_enabled = false;
    snprintf(cfg->pos_gpsd_host, sizeof(cfg->pos_gpsd_host), "localhost");
    cfg->pos_gpsd_port = 2947;

    cfg->pos_signalk_enabled = false;
    snprintf(cfg->pos_signalk_host, sizeof(cfg->pos_signalk_host), "localhost");
    cfg->pos_signalk_port = 3000;
    snprintf(cfg->pos_signalk_path, sizeof(cfg->pos_signalk_path),
             "/signalk/v1/api/vessels/self/navigation/position");

    cfg->pos_fix_max_age_h = 24.0f;

    /* [logging] */
    snprintf(cfg->log_level, sizeof(cfg->log_level), "warn");
    cfg->log_file[0] = '\0';   /* empty = use stderr; set a path to redirect */
    cfg->log_stats_hz = 1;
}

bool config_apply_influx_transport_compat(imud_config_t *cfg)
{
    /* New configs use udp_enabled/http_enabled; only fall back to the legacy
     * `transport` selector when neither new enable was set. */
    if (cfg->influx_udp_enabled || cfg->influx_http_enabled)
        return false;
    if (strcmp(cfg->influx_transport, "http") == 0) {
        cfg->influx_http_enabled = true;
        return true;
    }
    if (strcmp(cfg->influx_transport, "udp") == 0) {
        cfg->influx_udp_enabled = true;
        return true;
    }
    return false;
}

/* ── Mount rotation helper ──────────────────────────────────────────────── */

/*
 * Parse "[a, b, c, ...]" into exactly n doubles.
 *
 * The count is checked exactly. The earlier version accepted a short array
 * (only a completely empty one was an error), so `[0, 0]` silently left yaw at
 * whatever the default was — a wrong mount rotation that biases every sample
 * with nothing to catch it. Returns 0 on success, -1 having already logged.
 */
static int parse_float_array(const char *val, double *out, int n,
                             const char *path, int lineno, const char *key)
{
    const char *p = strchr(val, '[');
    if (!p) {
        LOG_E("%s:%d: '%s': expected array of %d numbers in [ ]\n",
              path, lineno, key, n);
        return -1;
    }
    p++;
    int count = 0;
    while (*p && *p != ']') {
        while (*p && (isspace((unsigned char)*p) || *p == ',')) p++;
        if (!*p || *p == ']') break;
        char *end;
        double v = strtod(p, &end);
        if (end == p) {
            LOG_E("%s:%d: '%s': malformed number at offset %d\n",
                  path, lineno, key, (int)(p - val));
            return -1;
        }
        if (count < n) out[count] = v;
        count++;
        p = end;
    }
    if (count != n) {
        LOG_E("%s:%d: '%s': expected exactly %d numbers, got %d\n",
              path, lineno, key, n, count);
        return -1;
    }
    return 0;
}

/*
 * Reject a mount matrix that is not a proper rotation.
 *
 * Only reachable for a hand-entered rotation_matrix — euler_deg_to_rot output
 * is orthonormal by construction. Checks both RᵀR ≈ I and det(R) > 0: an
 * orthogonality-only test happily accepts a reflection, and a sign-flipped
 * axis (det = −1) is the likeliest way to mistype this by hand.
 */
static int validate_rotation(const double R[3][3],
                             const char *path, int lineno, const char *key)
{
    double err = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double d = 0.0;
            for (int k = 0; k < 3; k++) d += R[k][i] * R[k][j];
            d -= (i == j) ? 1.0 : 0.0;
            err += d * d;
        }
    if (sqrt(err) > 1e-6) {
        LOG_E("%s:%d: '%s': not orthonormal (|RᵀR − I| = %.2e) — "
              "rows/columns must be unit length and mutually perpendicular\n",
              path, lineno, key, sqrt(err));
        return -1;
    }
    double det = R[0][0]*(R[1][1]*R[2][2] - R[1][2]*R[2][1])
               - R[0][1]*(R[1][0]*R[2][2] - R[1][2]*R[2][0])
               + R[0][2]*(R[1][0]*R[2][1] - R[1][1]*R[2][0]);
    if (det < 0.0) {
        LOG_E("%s:%d: '%s': determinant is %.3f, not +1 — this is a reflection, "
              "not a rotation (check for a sign-flipped axis)\n",
              path, lineno, key, det);
        return -1;
    }
    return 0;
}

/* Build R = Rz(yaw) * Ry(pitch) * Rx(roll) from [roll, pitch, yaw] in degrees. */
static void euler_deg_to_rot(const double euler[3], double R[3][3])
{
    double roll  = euler[0] * (M_PI / 180.0);
    double pitch = euler[1] * (M_PI / 180.0);
    double yaw   = euler[2] * (M_PI / 180.0);
    double cr = cos(roll),  sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);
    double Rx[3][3] = { {1, 0,   0  }, {0,  cr, -sr}, {0, sr, cr} };
    double Ry[3][3] = { {cp, 0,  sp }, {0,   1,   0}, {-sp, 0, cp} };
    double Rz[3][3] = { {cy, -sy, 0 }, {sy,  cy,  0}, {0,   0,  1} };
    double tmp[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            tmp[i][j] = 0.0;
            for (int k = 0; k < 3; k++) tmp[i][j] += Ry[i][k] * Rx[k][j];
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            R[i][j] = 0.0;
            for (int k = 0; k < 3; k++) R[i][j] += Rz[i][k] * tmp[k][j];
        }
}

/* ── Parser ─────────────────────────────────────────────────────────────── */

typedef enum {
    SEC_NONE = 0,
    SEC_UNKNOWN,
    SEC_MOUNT,
    SEC_DEVICE,
    SEC_CAPTURE,
    SEC_IMU,
    SEC_MAG,
    SEC_FUSION,
    SEC_CALIBRATION,
    SEC_NMEA,
    SEC_HIGHRATE,
    SEC_STREAM,
    SEC_SIGNALK,
    SEC_MQTT,
    SEC_INFLUX,
    SEC_PROM,
    SEC_MAVLINK,
    SEC_LOGGING,
    SEC_POSITION
} section_t;

static section_t parse_section(const char *s)
{
    if (strcmp(s, "[mount]")       == 0) return SEC_MOUNT;
    if (strcmp(s, "[device]")      == 0) return SEC_DEVICE;
    if (strcmp(s, "[capture]")     == 0) return SEC_CAPTURE;
    if (strcmp(s, "[imu]")         == 0) return SEC_IMU;
    if (strcmp(s, "[mag]")         == 0) return SEC_MAG;
    if (strcmp(s, "[fusion]")      == 0) return SEC_FUSION;
    if (strcmp(s, "[calibration]") == 0) return SEC_CALIBRATION;
    if (strcmp(s, "[nmea]")        == 0) return SEC_NMEA;
    if (strcmp(s, "[highrate]")    == 0) return SEC_HIGHRATE;
    if (strcmp(s, "[stream]")      == 0) return SEC_STREAM;
    if (strcmp(s, "[imud-signalk]")== 0) return SEC_SIGNALK;
    if (strcmp(s, "[imud-mqtt]")   == 0) return SEC_MQTT;
    if (strcmp(s, "[imud-influxdb]")== 0) return SEC_INFLUX;
    if (strcmp(s, "[imud-prometheus]") == 0) return SEC_PROM;
    if (strcmp(s, "[imud-mavlink]")== 0) return SEC_MAVLINK;
    if (strcmp(s, "[logging]")     == 0) return SEC_LOGGING;
    if (strcmp(s, "[position]")    == 0) return SEC_POSITION;
    return SEC_UNKNOWN;
}

/*
 * Dispatch a parsed key=value pair to the right config field.
 * Returns 0 on success, -1 on a hard parse error (bad type).
 * Unknown keys are warned but not fatal.
 */
static int apply_kv(imud_config_t *cfg, section_t sec,
                    const char *key, const char *val,
                    const char *path, int lineno)
{
    int    iv;
    double dv;
    bool   bv;

#define WARN_UNKNOWN() \
    LOG_W("%s:%d: unknown key '%s' — ignored\n", path, lineno, key)

#define NEED_INT(field) \
    do { if (!parse_int(val, &iv)) { \
        LOG_E("%s:%d: '%s': expected integer\n", path, lineno, key); \
        return -1; } \
        (field) = iv; } while (0)

#define NEED_FLT(field) \
    do { if (!parse_double(val, &dv)) { \
        LOG_E("%s:%d: '%s': expected number\n", path, lineno, key); \
        return -1; } \
        (field) = (float)dv; } while (0)

#define NEED_DBL(field) \
    do { if (!parse_double(val, &dv)) { \
        LOG_E("%s:%d: '%s': expected number\n", path, lineno, key); \
        return -1; } \
        (field) = dv; } while (0)

#define NEED_BOOL(field) \
    do { if (!parse_bool(val, &bv)) { \
        LOG_E("%s:%d: '%s': expected true or false\n", path, lineno, key); \
        return -1; } \
        (field) = bv; } while (0)

/*
 * A noise density that is zero or negative is not a tuning choice, it is a
 * broken filter: every one of these becomes a variance in a denominator.
 * mekf_accel_noise = 0 gives Ra = 0, hence a Kalman gain of exactly 1 — the
 * filter would snap its attitude onto every raw accelerometer sample, waves
 * and all. Fatal rather than clamped, because silently substituting a value
 * would hide a config typo behind plausible-looking output.
 */
#define NEED_POS_DBL(field) \
    do { if (!parse_double(val, &dv)) { \
        LOG_E("%s:%d: '%s': expected number\n", path, lineno, key); \
        return -1; } \
        if (!(dv > 0.0)) { \
        LOG_E("%s:%d: '%s': must be greater than zero (got %g) — " \
              "a zero or negative noise density makes the filter degenerate\n", \
              path, lineno, key, dv); \
        return -1; } \
        (field) = dv; } while (0)

/*
 * Same reasoning as NEED_POS_DBL, for the sample and publish rates. A rate of
 * zero or less is not a slow rate, it is an impossible one: it divides into a
 * period, and on the sensor side it also lands in the filter's noise
 * variances — [mag] odr_hz = 0 gives Rm = 0, hence a magnetometer Kalman gain
 * of exactly 1. Fatal rather than clamped, for the same reason: substituting a
 * plausible default would hide the typo.
 */
#define NEED_POS_INT(field) \
    do { if (!parse_int(val, &iv)) { \
        LOG_E("%s:%d: '%s': expected integer\n", path, lineno, key); \
        return -1; } \
        if (iv <= 0) { \
        LOG_E("%s:%d: '%s': must be greater than zero (got %d) — " \
              "a rate of zero or less is not a valid sample or publish rate\n", \
              path, lineno, key, iv); \
        return -1; } \
        (field) = iv; } while (0)

#define NEED_STR(field) \
    do { if (!copy_str(val, (field), sizeof(field))) \
        LOG_W("%s:%d: '%s': value too long (max %zu chars) — truncated\n", \
              path, lineno, key, sizeof(field) - 1); } while (0)

    switch (sec) {

    case SEC_MOUNT:
        if (strcmp(key, "rotation_euler_deg") == 0) {
            double angs[3];
            if (parse_float_array(val, angs, 3, path, lineno, key) < 0)
                return -1;
            cfg->mount_set = true;
            for (int i = 0; i < 3; i++) cfg->mount_euler_deg[i] = angs[i];
            euler_deg_to_rot(cfg->mount_euler_deg, cfg->mount_rot);
            return 0;
        } else if (strcmp(key, "rotation_matrix") == 0) {
            /* Row-major 3×3: v_body = R · v_board. Last mount key wins. */
            double m[9];
            if (parse_float_array(val, m, 9, path, lineno, key) < 0)
                return -1;
            double R[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) R[i][j] = m[i*3 + j];
            if (validate_rotation(R, path, lineno, key) < 0)
                return -1;
            cfg->mount_set = true;
            memcpy(cfg->mount_rot, R, sizeof R);
            /* Euler angles are not back-derived; mount_rot is authoritative. */
            cfg->mount_euler_deg[0] = cfg->mount_euler_deg[1] =
                cfg->mount_euler_deg[2] = 0.0;
            return 0;
        } else if (strcmp(key, "preset") == 0) {
            /* Named preset — string value. Validate BEFORE committing: this
             * used to set mount_set = true up front and only warn on an
             * unrecognised name, so a typo left the daemon running with
             * whatever angles happened to be in the struct. A silently wrong
             * mount rotation biases every sample, so it is fatal. */
            char preset[32];
            copy_str(val, preset, sizeof(preset));
            double e[3] = {0.0, 0.0, 0.0};
            if (strcasecmp(preset, "identity") == 0 || strcasecmp(preset, "board_forward") == 0) {
                e[0] = 0.0; e[1] = 0.0; e[2] = 0.0;
            } else if (strcasecmp(preset, "yaw_90") == 0 || strcasecmp(preset, "rot_z_90") == 0) {
                e[0] = 0.0; e[1] = 0.0; e[2] = 90.0;
            } else if (strcasecmp(preset, "yaw_180") == 0 || strcasecmp(preset, "rot_z_180") == 0) {
                e[0] = 0.0; e[1] = 0.0; e[2] = 180.0;
            } else if (strcasecmp(preset, "yaw_270") == 0 || strcasecmp(preset, "rot_z_270") == 0) {
                e[0] = 0.0; e[1] = 0.0; e[2] = 270.0;
            } else if (strcasecmp(preset, "roll_90") == 0 || strcasecmp(preset, "rot_x_90") == 0) {
                e[0] = 90.0; e[1] = 0.0; e[2] = 0.0;
            } else if (strcasecmp(preset, "roll_270") == 0 || strcasecmp(preset, "rot_x_270") == 0) {
                e[0] = 270.0; e[1] = 0.0; e[2] = 0.0;
            } else if (strcasecmp(preset, "pitch_90") == 0 || strcasecmp(preset, "rot_y_90") == 0) {
                e[0] = 0.0; e[1] = 90.0; e[2] = 0.0;
            } else if (strcasecmp(preset, "pitch_270") == 0 || strcasecmp(preset, "rot_y_270") == 0) {
                e[0] = 0.0; e[1] = 270.0; e[2] = 0.0;
            } else {
                LOG_E("%s:%d: '%s': unknown mount preset '%s'\n",
                      path, lineno, key, preset);
                return -1;
            }
            snprintf(cfg->mount_preset, sizeof(cfg->mount_preset), "%s", preset);
            cfg->mount_set = true;
            for (int i = 0; i < 3; i++) cfg->mount_euler_deg[i] = e[i];
            euler_deg_to_rot(cfg->mount_euler_deg, cfg->mount_rot);
            return 0;
        }
        WARN_UNKNOWN();
        break;

    case SEC_NONE:
        LOG_W("%s:%d: key '%s' appears before any section — ignored\n",
                path, lineno, key);
        break;

    case SEC_UNKNOWN:
        /* silently skip keys inside unrecognised sections */
        break;

    case SEC_DEVICE:
        if      (strcmp(key, "i2c_bus")   == 0) NEED_STR(cfg->i2c_bus);
        else if (strcmp(key, "gpio_chip") == 0) NEED_STR(cfg->gpio_chip);
        else if (strcmp(key, "sim_file")  == 0) NEED_STR(cfg->sim_file);
        else if (strcmp(key, "sim_loop")  == 0) NEED_BOOL(cfg->sim_loop);
        else if (strcmp(key, "sim_speed") == 0) NEED_FLT(cfg->sim_speed);
        else WARN_UNKNOWN();
        break;
    case SEC_CAPTURE:
        if      (strcmp(key, "enabled")   == 0) NEED_BOOL(cfg->capture_enabled);
        else if (strcmp(key, "dir")       == 0) NEED_STR(cfg->capture_dir);
        else if (strcmp(key, "max_mb")    == 0) NEED_INT(cfg->capture_max_mb);
        else if (strcmp(key, "max_files") == 0) NEED_INT(cfg->capture_max_files);
        else if (strcmp(key, "flush_s")   == 0) NEED_INT(cfg->capture_flush_s);
        else WARN_UNKNOWN();
        break;

    case SEC_IMU:
        if      (strcmp(key, "driver")   == 0) NEED_STR(cfg->imu_driver);
        else if (strcmp(key, "i2c_addr") == 0) NEED_INT(cfg->imu_addr);
        else if (strcmp(key, "int_gpio") == 0) NEED_INT(cfg->imu_int_gpio);
        else if (strcmp(key, "odr_hz")   == 0) NEED_POS_INT(cfg->imu_odr_hz);
        else if (strcmp(key, "accel_g")  == 0) NEED_INT(cfg->imu_accel_g);
        else if (strcmp(key, "gyro_dps") == 0) NEED_INT(cfg->imu_gyro_dps);
        else if (strcmp(key, "fifo_wm")  == 0) NEED_INT(cfg->imu_fifo_wm);
        else WARN_UNKNOWN();
        break;

    case SEC_MAG:
        if      (strcmp(key, "driver")       == 0) NEED_STR(cfg->mag_driver);
        else if (strcmp(key, "i2c_addr")     == 0) NEED_INT(cfg->mag_addr);
        else if (strcmp(key, "int_gpio")     == 0) NEED_INT(cfg->mag_int_gpio);
        else if (strcmp(key, "odr_hz")       == 0) NEED_POS_INT(cfg->mag_odr_hz);
        else if (strcmp(key, "set_period_s") == 0) NEED_FLT(cfg->mag_set_period_s);
        else WARN_UNKNOWN();
        break;

    case SEC_FUSION:
        if      (strcmp(key, "mag_yaw_only")      == 0) NEED_BOOL(cfg->mag_yaw_only);
        else if (strcmp(key, "heave_tau_s")       == 0) NEED_FLT(cfg->heave_tau_s);
        else if (strcmp(key, "wave_tau_s")        == 0) NEED_FLT(cfg->wave_tau_s);
        else if (strcmp(key, "mekf_gyro_noise")   == 0) NEED_POS_DBL(cfg->mekf_gyro_noise);
        else if (strcmp(key, "mekf_gyro_bias")    == 0) NEED_POS_DBL(cfg->mekf_gyro_bias);
        else if (strcmp(key, "mekf_accel_noise")  == 0) NEED_POS_DBL(cfg->mekf_accel_noise);
        else if (strcmp(key, "mekf_mag_noise")    == 0) NEED_POS_DBL(cfg->mekf_mag_noise);
        /* NEED_DBL, not NEED_POS_DBL: 0 is the documented "disabled" value. */
        else if (strcmp(key, "mekf_wave_accel")       == 0) NEED_DBL(cfg->mekf_wave_accel);
        else if (strcmp(key, "mekf_wave_accel_tau_s") == 0) NEED_DBL(cfg->mekf_wave_accel_tau_s);
        else if (strcmp(key, "mekf_mag_dip_sigma_deg") == 0) NEED_DBL(cfg->mekf_mag_dip_sigma_deg);
        else if (strcmp(key, "mag_reject_gauss")          == 0) NEED_DBL(cfg->mag_reject_gauss);
        else if (strcmp(key, "accel_skip_thresh")         == 0) NEED_DBL(cfg->accel_skip_thresh);
        else if (strcmp(key, "engine_vibration_g2")       == 0) NEED_DBL(cfg->engine_vibration_g2);
        else if (strcmp(key, "engine_accel_skip_thresh")  == 0) NEED_DBL(cfg->engine_accel_skip_thresh);
        else WARN_UNKNOWN();
        break;

    case SEC_CALIBRATION:
        if      (strcmp(key, "file")          == 0) NEED_STR(cfg->cal_file);
        else if (strcmp(key, "startup_settle_sec") == 0) NEED_DBL(cfg->startup_settle_sec);
        else if (strcmp(key, "gyro_bias_sec") == 0) NEED_DBL(cfg->gyro_bias_sec);
        else if (strcmp(key, "align_window_sec") == 0) NEED_DBL(cfg->align_window_sec);
        else WARN_UNKNOWN();
        break;

    case SEC_NMEA:
        if      (strcmp(key, "enabled")   == 0) NEED_BOOL(cfg->nmea_enabled);
        else if (strcmp(key, "rate_hz")   == 0) NEED_POS_INT(cfg->nmea_rate_hz);
        else if (strcmp(key, "dest_addr") == 0) NEED_STR(cfg->nmea_dest_addr);
        else if (strcmp(key, "dest_port") == 0) NEED_INT(cfg->nmea_dest_port);
        else if (strcmp(key, "tcp_enabled")   == 0) NEED_BOOL(cfg->nmea_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr") == 0) NEED_STR(cfg->nmea_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")      == 0) NEED_INT(cfg->nmea_tcp_port);
        else WARN_UNKNOWN();
        break;

    case SEC_HIGHRATE:
        if      (strcmp(key, "enabled")     == 0) NEED_BOOL(cfg->highrate_enabled);
        else if (strcmp(key, "rate_hz")     == 0) NEED_POS_INT(cfg->highrate_rate_hz);
        else if (strcmp(key, "dest_addr")   == 0) NEED_STR(cfg->highrate_dest_addr);
        else if (strcmp(key, "dest_port")   == 0) NEED_INT(cfg->highrate_dest_port);
        else if (strcmp(key, "coord_frame") == 0) NEED_STR(cfg->highrate_coord_frame);
        else WARN_UNKNOWN();
        break;

    case SEC_STREAM:
        if      (strcmp(key, "enabled") == 0) NEED_BOOL(cfg->stream_enabled);
        else if (strcmp(key, "socket")  == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "rate_hz") == 0) NEED_POS_INT(cfg->stream_rate_hz);
        else if (strcmp(key, "tcp_enabled")   == 0) NEED_BOOL(cfg->stream_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr") == 0) NEED_STR(cfg->stream_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")      == 0) NEED_INT(cfg->stream_tcp_port);
        else WARN_UNKNOWN();
        break;

    case SEC_SIGNALK:
        if      (strcmp(key, "enabled")      == 0) NEED_BOOL(cfg->sk_enabled);
        else if (strcmp(key, "udp_enabled")  == 0) NEED_BOOL(cfg->sk_udp_enabled);
        else if (strcmp(key, "socket")       == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "dest_addr")    == 0) NEED_STR(cfg->sk_dest_addr);
        else if (strcmp(key, "dest_port")    == 0) NEED_INT(cfg->sk_dest_port);
        else if (strcmp(key, "rate_hz")      == 0) NEED_POS_INT(cfg->sk_rate_hz);
        else if (strcmp(key, "source_label") == 0) NEED_STR(cfg->sk_source_label);
        else if (strcmp(key, "publish_heave")== 0) NEED_BOOL(cfg->publish_heave);
        else if (strcmp(key, "tcp_enabled")   == 0) NEED_BOOL(cfg->sk_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr") == 0) NEED_STR(cfg->sk_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")      == 0) NEED_INT(cfg->sk_tcp_port);
        else WARN_UNKNOWN();
        break;

    case SEC_MQTT:
        if      (strcmp(key, "enabled")       == 0) NEED_BOOL(cfg->mqtt_enabled);
        else if (strcmp(key, "broker_enabled")== 0) NEED_BOOL(cfg->mqtt_broker_enabled);
        else if (strcmp(key, "socket")        == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "broker_addr")   == 0) NEED_STR(cfg->mqtt_broker_addr);
        else if (strcmp(key, "broker_port")   == 0) NEED_INT(cfg->mqtt_broker_port);
        else if (strcmp(key, "client_id")     == 0) NEED_STR(cfg->mqtt_client_id);
        else if (strcmp(key, "topic_prefix")  == 0) NEED_STR(cfg->mqtt_topic_prefix);
        else if (strcmp(key, "rate_hz")       == 0) NEED_POS_INT(cfg->mqtt_rate_hz);
        else if (strcmp(key, "qos")           == 0) NEED_INT(cfg->mqtt_qos);
        else if (strcmp(key, "retain")        == 0) NEED_BOOL(cfg->mqtt_retain);
        else if (strcmp(key, "units")         == 0) NEED_STR(cfg->mqtt_units);
        else if (strcmp(key, "publish_heave") == 0) NEED_BOOL(cfg->publish_heave);
        else if (strcmp(key, "username")      == 0) NEED_STR(cfg->mqtt_username);
        else if (strcmp(key, "password")      == 0) NEED_STR(cfg->mqtt_password);
        else if (strcmp(key, "tls")           == 0) NEED_BOOL(cfg->mqtt_tls);
        else if (strcmp(key, "tls_cafile")    == 0) NEED_STR(cfg->mqtt_tls_cafile);
        else if (strcmp(key, "keepalive_s")   == 0) NEED_INT(cfg->mqtt_keepalive_s);
        else if (strcmp(key, "ha_discovery")  == 0) NEED_BOOL(cfg->mqtt_ha_discovery);
        else if (strcmp(key, "ha_prefix")     == 0) NEED_STR(cfg->mqtt_ha_prefix);
        else WARN_UNKNOWN();
        break;

    case SEC_INFLUX:
        if      (strcmp(key, "enabled")       == 0) NEED_BOOL(cfg->influx_enabled);
        else if (strcmp(key, "socket")        == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "transport")     == 0) NEED_STR(cfg->influx_transport);
        else if (strcmp(key, "rate_hz")       == 0) NEED_POS_INT(cfg->influx_rate_hz);
        else if (strcmp(key, "measurement")   == 0) NEED_STR(cfg->influx_measurement);
        else if (strcmp(key, "source_label")  == 0) NEED_STR(cfg->influx_source_label);
        else if (strcmp(key, "units")         == 0) NEED_STR(cfg->influx_units);
        else if (strcmp(key, "publish_heave") == 0) NEED_BOOL(cfg->publish_heave);
        else if (strcmp(key, "udp_enabled")   == 0) NEED_BOOL(cfg->influx_udp_enabled);
        else if (strcmp(key, "udp_addr")      == 0) NEED_STR(cfg->influx_udp_addr);
        else if (strcmp(key, "udp_port")      == 0) NEED_INT(cfg->influx_udp_port);
        else if (strcmp(key, "http_enabled")  == 0) NEED_BOOL(cfg->influx_http_enabled);
        else if (strcmp(key, "http_host")     == 0) NEED_STR(cfg->influx_http_host);
        else if (strcmp(key, "http_port")     == 0) NEED_INT(cfg->influx_http_port);
        else if (strcmp(key, "http_path")     == 0) NEED_STR(cfg->influx_http_path);
        else if (strcmp(key, "http_token")    == 0) NEED_STR(cfg->influx_http_token);
        else WARN_UNKNOWN();
        break;

    case SEC_PROM:
        if      (strcmp(key, "enabled")      == 0) NEED_BOOL(cfg->prom_enabled);
        else if (strcmp(key, "http_enabled") == 0) NEED_BOOL(cfg->prom_http_enabled);
        else if (strcmp(key, "socket")       == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "listen_addr")  == 0) NEED_STR(cfg->prom_listen_addr);
        else if (strcmp(key, "listen_port")  == 0) NEED_INT(cfg->prom_listen_port);
        else WARN_UNKNOWN();
        break;

    case SEC_MAVLINK:
        if      (strcmp(key, "enabled")        == 0) NEED_BOOL(cfg->mav_enabled);
        else if (strcmp(key, "socket")         == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "version")        == 0) NEED_INT(cfg->mav_version);
        else if (strcmp(key, "system_id")      == 0) NEED_INT(cfg->mav_system_id);
        else if (strcmp(key, "component_id")   == 0) NEED_INT(cfg->mav_component_id);
        else if (strcmp(key, "rate_hz")        == 0) NEED_POS_INT(cfg->mav_rate_hz);
        else if (strcmp(key, "send_attitude")  == 0) NEED_BOOL(cfg->mav_send_attitude);
        else if (strcmp(key, "send_attitude_quaternion") == 0) NEED_BOOL(cfg->mav_send_attitude_quaternion);
        else if (strcmp(key, "udp_enabled")    == 0) NEED_BOOL(cfg->mav_udp_enabled);
        else if (strcmp(key, "udp_addr")       == 0) NEED_STR(cfg->mav_udp_addr);
        else if (strcmp(key, "udp_port")       == 0) NEED_INT(cfg->mav_udp_port);
        else if (strcmp(key, "serial_enabled") == 0) NEED_BOOL(cfg->mav_serial_enabled);
        else if (strcmp(key, "serial_device")  == 0) NEED_STR(cfg->mav_serial_device);
        else if (strcmp(key, "serial_baud")    == 0) NEED_INT(cfg->mav_serial_baud);
        else if (strcmp(key, "tcp_enabled")    == 0) NEED_BOOL(cfg->mav_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr")  == 0) NEED_STR(cfg->mav_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")       == 0) NEED_INT(cfg->mav_tcp_port);
        else WARN_UNKNOWN();
        break;

    case SEC_LOGGING:
        if      (strcmp(key, "level")    == 0) NEED_STR(cfg->log_level);
        else if (strcmp(key, "file")     == 0) NEED_STR(cfg->log_file);
        else if (strcmp(key, "stats_hz") == 0) NEED_INT(cfg->log_stats_hz);
        else WARN_UNKNOWN();
        break;

    case SEC_POSITION:
        if      (strcmp(key, "declination_deg")  == 0) {
            NEED_FLT(cfg->pos_declination_deg);
            /* Config-key semantics stay "0.0 = disabled"; only WMM-computed
             * declination may be valid at exactly 0.0 (agonic line). */
            cfg->pos_declination_valid = (cfg->pos_declination_deg != 0.0f);
        }
        else if (strcmp(key, "lat_deg")           == 0) NEED_DBL(cfg->pos_lat_deg);
        else if (strcmp(key, "lon_deg")           == 0) NEED_DBL(cfg->pos_lon_deg);
        else if (strcmp(key, "wmm_file")          == 0) NEED_STR(cfg->pos_wmm_file);
        else if (strcmp(key, "gpsd_enabled")      == 0) NEED_BOOL(cfg->pos_gpsd_enabled);
        else if (strcmp(key, "gpsd_host")         == 0) NEED_STR(cfg->pos_gpsd_host);
        else if (strcmp(key, "gpsd_port")         == 0) NEED_INT(cfg->pos_gpsd_port);
        else if (strcmp(key, "signalk_enabled")   == 0) NEED_BOOL(cfg->pos_signalk_enabled);
        else if (strcmp(key, "signalk_host")      == 0) NEED_STR(cfg->pos_signalk_host);
        else if (strcmp(key, "signalk_port")      == 0) NEED_INT(cfg->pos_signalk_port);
        else if (strcmp(key, "signalk_path")      == 0) NEED_STR(cfg->pos_signalk_path);
        else if (strcmp(key, "fix_max_age_h")    == 0) NEED_FLT(cfg->pos_fix_max_age_h);
        else WARN_UNKNOWN();
        break;
    }

    return 0;

#undef WARN_UNKNOWN
#undef NEED_INT
#undef NEED_POS_INT
#undef NEED_FLT
#undef NEED_DBL
#undef NEED_POS_DBL
#undef NEED_BOOL
#undef NEED_STR
}

/*
 * WMM coefficient file resolution. An explicit [position] wmm_file is used
 * as-is; the empty default auto-resolves so the packaging split works:
 *   1. /etc/imud/WMM.COF        — operator override (newer model drop-in;
 *                                 also where ≤1.4 installs put the file)
 *   2. /usr/share/imud/WMM.COF  — package data (imud-wmm-data / the
 *                                 install-wmm-data target)
 * Re-run on every load, so a dropped-in override is picked up by SIGHUP.
 */
static void resolve_wmm_file(imud_config_t *cfg)
{
    if (cfg->pos_wmm_file[0] != '\0') return;
    if (access("/etc/imud/WMM.COF", R_OK) == 0)
        snprintf(cfg->pos_wmm_file, sizeof(cfg->pos_wmm_file),
                 "/etc/imud/WMM.COF");
    else
        snprintf(cfg->pos_wmm_file, sizeof(cfg->pos_wmm_file),
                 "/usr/share/imud/WMM.COF");
}

int config_load(const char *path, imud_config_t *cfg)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        /* A missing file is survivable — defaults are the documented
         * behaviour.  Anything else (EACCES on a 0640 file the daemon is not
         * in the group for, EIO, ...) means a config exists that we would be
         * silently ignoring; report it separately so callers can refuse. */
        int err = errno;
        LOG_E("config: cannot open '%s': %s\n", path, strerror(err));
        resolve_wmm_file(cfg);   /* daemon proceeds on defaults */
        return (err == ENOENT) ? CONFIG_ERR_OPEN : CONFIG_ERR_PERM;
    }

    section_t sec = SEC_NONE;
    char line[512];
    int  lineno = 0;
    int  rc = 0;

    while (fgets(line, sizeof(line), f)) {
        lineno++;

        char *p = lstrip(line);
        rstrip(p);

        /* Blank line or pure comment */
        if (*p == '\0' || *p == '#') continue;

        /* Section header: [name] */
        if (*p == '[') {
            strip_comment(p);
            rstrip(p);
            section_t s = parse_section(p);
            if (s == SEC_UNKNOWN) {
                LOG_W("%s:%d: unknown section '%s' — skipping\n",
                        path, lineno, p);
            }
            sec = s;
            continue;
        }

        /* key = value */
        strip_comment(p);
        rstrip(p);

        char *eq = strchr(p, '=');
        if (!eq) {
            LOG_W("%s:%d: no '=' found — line skipped\n", path, lineno);
            continue;
        }

        *eq = '\0';
        char *key = p;
        char *val = eq + 1;

        rstrip(key);
        key = lstrip(key);
        val = lstrip(val);
        rstrip(val);

        if (*key == '\0') {
            LOG_W("%s:%d: empty key — line skipped\n", path, lineno);
            continue;
        }

        /* Report and keep parsing so every error in the file is surfaced
         * in one pass. The caller decides whether a bad file is fatal;
         * aborting here would silently discard all settings after the
         * first bad line while keeping the ones before it. */
        if (apply_kv(cfg, sec, key, val, path, lineno) != 0)
            rc = CONFIG_ERR_PARSE;
    }

    fclose(f);
    resolve_wmm_file(cfg);
    return rc;
}
