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
#include <limits.h>
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

/*
 * Replace a leading ~/ with $HOME/.
 *
 * The overflow guard below is unreachable from copy_str(), which rejects a
 * value whose expansion would not fit before it writes anything.  It stays
 * because this is a standalone helper and a silent non-expansion — a literal
 * "~/..." opened relative to the cwd — is the failure it exists to prevent.
 */
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
 * Why copy_str reports two distinct failures rather than a bool: the message is
 * half the fix.  "value too long" is a lie for a value that fits perfectly well
 * and only overflows once ~/ becomes $HOME, and an operator told the wrong
 * thing shortens the wrong string.
 */
typedef enum {
    COPY_OK = 0,
    COPY_TOO_LONG,          /* the value itself does not fit the field       */
    COPY_TILDE_TOO_LONG     /* it fits, but not once ~/ expands to $HOME     */
} copy_rc_t;

/*
 * Copy TOML string value into out: strip surrounding quotes, expand tilde.
 *
 * Returns non-OK if the value did not fit — callers must surface that, because
 * a silently shortened path (a socket, a cal file) binds or opens something the
 * user never asked for.  NEED_STR makes it fatal.
 *
 * On failure out is left UNTOUCHED, so a rejected key keeps its default.  That
 * matters beyond tidiness: imud-mon deliberately ignores config_load's return
 * ("defaults have the right port numbers"), so it is the one consumer that runs
 * on a config the daemon refused, and leaving the truncated value behind would
 * point it at a socket nobody is listening on.
 *
 * Hence the fit is decided before anything is written — including the tilde
 * case, which the old snprintf-then-measure form could not see at all: it
 * measured the unexpanded value, and expand_tilde then declined silently.
 */
static copy_rc_t copy_str(const char *val, char *out, size_t outsz)
{
    const char *s = val;
    size_t len;

    if (*s == '"') {
        s++;
        len = strlen(s);
        if (len > 0 && s[len - 1] == '"') len--;
    } else {
        len = strlen(s);
    }

    if (len >= outsz) return COPY_TOO_LONG;

    /* Same arithmetic expand_tilde uses: $HOME replaces the '~', so the result
     * is strlen(home) + the length of the "/rest/of/path" that follows it.
     * No $HOME set means no expansion happens at all — a documented behaviour,
     * and not a length failure. */
    if (len >= 2 && s[0] == '~' && s[1] == '/') {
        const char *home = getenv("HOME");
        if (home && strlen(home) + (len - 1) >= outsz)
            return COPY_TILDE_TOO_LONG;
    }

    memcpy(out, s, len);
    out[len] = '\0';
    expand_tilde(out, outsz);
    return COPY_OK;
}

static bool parse_bool(const char *val, bool *out)
{
    if (strcmp(val, "true")  == 0) { *out = true;  return true; }
    if (strcmp(val, "false") == 0) { *out = false; return true; }
    return false;
}

/*
 * Range matters as much as syntax, and it used to be unchecked: errno was
 * never cleared or read, and the long → int conversion of an out-of-range
 * value is implementation-defined (C17 6.3.1.3p3) — on every target imud ships
 * to, it wraps.  So "dest_port = 4294977414" became 10118, *a different and
 * entirely valid port*, with config_load returning 0 and no message at any log
 * level.  "odr_hz = 4294968129" wrapped to 833, landing positive, so even
 * NEED_POS_INT waved it through — its guard tests the post-cast value.
 *
 * Fatal rather than clamped, for the reason NEED_POS_INT and NEED_STR are: a
 * plausible substitute hides the typo behind output that looks fine.
 *
 * Two distinct failures, because the message is half the fix — the same
 * reasoning as copy_str's: reporting "expected integer" for 4294977414 tells
 * the operator to check the syntax of a value whose syntax is perfectly good.
 *
 * The #if is not decoration.  On LP64 the range test is the one that fires;
 * on armhf long is 32-bit, INT_MAX == LONG_MAX, and `v > INT_MAX` is always
 * false — which -Wtype-limits (in -Wextra) reports, and CI builds armhf.  It
 * is also why the round-trip idiom `(int)v != v` is not used here: that would
 * lean on the very implementation-defined conversion this guards against.
 */
typedef enum {
    INT_OK = 0,
    INT_MALFORMED,      /* not an integer at all                             */
    INT_UNREPRESENTABLE /* an integer, but too large or small for an int     */
} parse_int_rc_t;

static parse_int_rc_t parse_int(const char *val, int *out)
{
    char *end;
    errno = 0;
    long v = strtol(val, &end, 0);   /* base 0: accepts 0x hex */
    if (end == val || *end != '\0') return INT_MALFORMED;
    if (errno == ERANGE) return INT_UNREPRESENTABLE;
#if LONG_MAX > INT_MAX
    if (v < INT_MIN || v > INT_MAX) return INT_UNREPRESENTABLE;
#endif
    *out = (int)v;
    return INT_OK;
}

/*
 * The finiteness test is not belt-and-braces: strtod converts "nan" and "inf"
 * happily, and overflows "1e999" to HUGE_VAL with ERANGE, so without it any
 * float key could be set non-finite from a config file.  NEED_POS_DBL was no
 * defence — its only test is `dv > 0.0`, which infinity passes; it rejected
 * NaN by accident, since every comparison against NaN is false.
 *
 * This is the one chokepoint every float key routes through, so guarding it
 * here covers NEED_FLT, NEED_DBL and NEED_POS_DBL at once.  Fatal rather than
 * clamped, for the reason NEED_POS_INT is: a plausible substitute would hide
 * the typo.
 */
static bool parse_double(const char *val, double *out)
{
    char *end;
    *out = strtod(val, &end);
    return (end != val && *end == '\0' && isfinite(*out));
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

void config_apply_hot(imud_config_t *dst, const imud_config_t *src)
{
    /* Output rates and the stats heartbeat.  The output threads keep their own
     * _Atomic copies; out_ctx_reload() pushes these across after the call. */
    dst->nmea_rate_hz     = src->nmea_rate_hz;
    dst->highrate_rate_hz = src->highrate_rate_hz;
    dst->stream_rate_hz   = src->stream_rate_hz;
    dst->log_stats_hz     = src->log_stats_hz;

    /* Level only.  log_file is [restart]: reopening it is what lets logrotate
     * move the old file, and the caller does that — the path itself must not
     * change under a running daemon. */
    snprintf(dst->log_level, sizeof dst->log_level, "%s", src->log_level);

    /* Fusion gains and thresholds — pushed into the running filter by
     * imu_ctx_update_config(). */
    dst->mag_yaw_only              = src->mag_yaw_only;
    dst->heave_tau_s               = src->heave_tau_s;
    dst->wave_tau_s                = src->wave_tau_s;
    dst->mekf_gyro_noise           = src->mekf_gyro_noise;
    dst->mekf_gyro_bias            = src->mekf_gyro_bias;
    dst->mekf_accel_noise          = src->mekf_accel_noise;
    dst->mekf_mag_noise            = src->mekf_mag_noise;
    dst->mekf_wave_accel           = src->mekf_wave_accel;
    dst->mekf_wave_accel_tau_s     = src->mekf_wave_accel_tau_s;
    dst->mekf_mag_dip_sigma_deg    = src->mekf_mag_dip_sigma_deg;
    dst->mag_reject_gauss          = src->mag_reject_gauss;
    dst->accel_skip_thresh         = src->accel_skip_thresh;
    dst->engine_vibration_g2       = src->engine_vibration_g2;
    dst->engine_accel_skip_thresh  = src->engine_accel_skip_thresh;

    /* Declination and the WMM-derived field invariants.  The caller runs the
     * WMM recomputation over `src` before calling, so these are results, not
     * raw keys — which is why pos_lat_deg / pos_lon_deg are NOT copied.  They
     * have no other reader in the daemon: everything downstream consumes the
     * derived values below, and the next SIGHUP re-reads the file anyway.
     * Copying pos_mref_* is not optional — without it the MEKF keeps the
     * startup m_ref after a lat/lon or wmm_file change, because
     * imu_ctx_update_config() only applies them when pos_mref_valid. */
    snprintf(dst->pos_wmm_file, sizeof dst->pos_wmm_file, "%s",
             src->pos_wmm_file);
    dst->pos_declination_deg   = src->pos_declination_deg;
    dst->pos_declination_valid = src->pos_declination_valid;
    dst->pos_mref_h_gauss      = src->pos_mref_h_gauss;
    dst->pos_mref_z_gauss      = src->pos_mref_z_gauss;
    dst->pos_mref_valid        = src->pos_mref_valid;
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

/* Shared by every int macro: parse into iv, or report and bail.  Split out so
 * the two messages exist once — an unrepresentable value is not a malformed
 * one, and saying so is the difference between an operator checking the digits
 * and checking the magnitude. */
#define PARSE_INT_OR_FAIL() \
    do { switch (parse_int(val, &iv)) { \
         case INT_OK: break; \
         case INT_MALFORMED: \
            LOG_E("%s:%d: '%s': expected integer\n", path, lineno, key); \
            return -1; \
         case INT_UNREPRESENTABLE: \
            LOG_E("%s:%d: '%s': integer out of range for this platform " \
                  "(%d..%d)\n", path, lineno, key, INT_MIN, INT_MAX); \
            return -1; \
         } } while (0)

#define NEED_INT(field) \
    do { PARSE_INT_OR_FAIL(); \
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
    do { PARSE_INT_OR_FAIL(); \
        if (iv <= 0) { \
        LOG_E("%s:%d: '%s': must be greater than zero (got %d) — " \
              "a rate of zero or less is not a valid sample or publish rate\n", \
              path, lineno, key, iv); \
        return -1; } \
        (field) = iv; } while (0)

/*
 * Semantic bounds, for the keys where an in-range int is still nonsense.
 * parse_int's range check above stops a value wrapping into a plausible one;
 * it cannot stop "dest_port = 70000", which fits an int perfectly well and
 * then becomes port 4464 the moment htons() truncates it to uint16_t. Same
 * failure — a listener nobody asked for — through a different door.
 *
 * The bounds and why they are where they are:
 *
 *   ports      1..65535  the wire range. Port 0 is a sentinel nowhere in this
 *                        tree (the bridges gate on their own *_enabled flags),
 *                        so it is a typo rather than a request.
 *   int_gpio   0..255    ZERO IS LEGAL AND DOCUMENTED — imu.c gates on
 *                        `int_gpio > 0` to mean "no interrupt line", which is
 *                        how config/sim.conf runs without hardware. The
 *                        ceiling is deliberately loose: imud is general-purpose
 *                        Linux, not Pi-only, and other boards expose gpiochips
 *                        with far more lines than a Pi's ~58. libgpiod already
 *                        gives a clear error for a line the chip lacks.
 *   i2c_addr   0x00..0x7F the 7-bit address space, NOT the textbook "valid"
 *                        0x08..0x77. That narrower range would reject
 *                        `i2c_addr = 0x00`, which config/sim.conf ships on
 *                        both sensors and which imud.conf documents for the
 *                        sim driver. Rejecting a shipped config to enforce a
 *                        convention the daemon does not rely on would be a
 *                        worse bug than the one this guards.
 *
 * Keys with a natural range but no wrap-into-plausible failure (mqtt qos,
 * mavlink version/system_id/component_id, serial_baud) are deliberately left
 * on plain NEED_INT: each bound would be a judgement call able to reject a
 * legitimate setup, which is the more expensive mistake.
 */
#define NEED_RANGE_INT(field, lo, hi, what) \
    do { PARSE_INT_OR_FAIL(); \
        if (iv < (lo) || iv > (hi)) { \
        LOG_E("%s:%d: '%s': %s must be %d..%d (got %d)\n", \
              path, lineno, key, (what), (lo), (hi), iv); \
        return -1; } \
        (field) = iv; } while (0)

#define NEED_PORT(field)     NEED_RANGE_INT((field), 1, 65535, "a TCP/UDP port")
#define NEED_GPIO(field)     NEED_RANGE_INT((field), 0, 255, "a GPIO line")

/* Its own macro rather than NEED_RANGE_INT(0x00, 0x7F, …) so the message is in
 * the base the operator wrote: every i2c_addr in the shipped configs is hex,
 * and "must be 0..127 (got 128)" makes them convert 0x80 in their head. */
#define NEED_I2C_ADDR(field) \
    do { PARSE_INT_OR_FAIL(); \
        if (iv < 0x00 || iv > 0x7F) { \
        if (iv < 0) \
            LOG_E("%s:%d: '%s': a 7-bit I2C address must be 0x00..0x7F " \
                  "(got %d)\n", path, lineno, key, iv); \
        else \
            LOG_E("%s:%d: '%s': a 7-bit I2C address must be 0x00..0x7F " \
                  "(got 0x%X)\n", path, lineno, key, (unsigned)iv); \
        return -1; } \
        (field) = iv; } while (0)

/*
 * A value that does not fit is fatal, not a warning.  This used to log LOG_W
 * and carry on, which is the worst of both: a [stream] socket of 130 characters
 * became a *different, perfectly valid* 107-character path, the daemon bound
 * that, and every bridge and libimud client then connected to the path written
 * in the config file and found nothing there.  A truncated [calibration] file
 * or [logging] file is the same shape.
 *
 * Fatal matches every other NEED_* macro, copy_str's own documented contract,
 * and main.c's policy that a config which parses badly must not start: a
 * plausible substitute hides the mistake, exactly as with NEED_POS_INT.
 */
#define NEED_STR(field) \
    do { switch (copy_str(val, (field), sizeof(field))) { \
         case COPY_OK: break; \
         case COPY_TOO_LONG: \
            LOG_E("%s:%d: '%s': value too long (max %zu chars)\n", \
                  path, lineno, key, sizeof(field) - 1); \
            return -1; \
         case COPY_TILDE_TOO_LONG: \
            LOG_E("%s:%d: '%s': too long once '~/' expands to $HOME " \
                  "(max %zu chars after expansion)\n", \
                  path, lineno, key, sizeof(field) - 1); \
            return -1; \
         } } while (0)

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
            /* Not NEED_STR — a preset name is matched, not stored — but the
             * return cannot be ignored either: copy_str leaves the buffer
             * alone when the value does not fit, and the strcasecmp chain
             * below would then read it uninitialised.  Reported as the length
             * error it is rather than as an unknown preset, which would send
             * the operator looking for a name that is in fact spelled right. */
            if (copy_str(val, preset, sizeof(preset)) != COPY_OK) {
                LOG_E("%s:%d: '%s': preset name too long (max %zu chars)\n",
                      path, lineno, key, sizeof(preset) - 1);
                return -1;
            }
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
        else if (strcmp(key, "i2c_addr") == 0) NEED_I2C_ADDR(cfg->imu_addr);
        else if (strcmp(key, "int_gpio") == 0) NEED_GPIO(cfg->imu_int_gpio);
        else if (strcmp(key, "odr_hz")   == 0) NEED_POS_INT(cfg->imu_odr_hz);
        else if (strcmp(key, "accel_g")  == 0) NEED_INT(cfg->imu_accel_g);
        else if (strcmp(key, "gyro_dps") == 0) NEED_INT(cfg->imu_gyro_dps);
        else if (strcmp(key, "fifo_wm")  == 0) NEED_INT(cfg->imu_fifo_wm);
        else WARN_UNKNOWN();
        break;

    case SEC_MAG:
        if      (strcmp(key, "driver")       == 0) NEED_STR(cfg->mag_driver);
        else if (strcmp(key, "i2c_addr")     == 0) NEED_I2C_ADDR(cfg->mag_addr);
        else if (strcmp(key, "int_gpio")     == 0) NEED_GPIO(cfg->mag_int_gpio);
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
        else if (strcmp(key, "dest_port") == 0) NEED_PORT(cfg->nmea_dest_port);
        else if (strcmp(key, "tcp_enabled")   == 0) NEED_BOOL(cfg->nmea_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr") == 0) NEED_STR(cfg->nmea_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")      == 0) NEED_PORT(cfg->nmea_tcp_port);
        else WARN_UNKNOWN();
        break;

    case SEC_HIGHRATE:
        if      (strcmp(key, "enabled")     == 0) NEED_BOOL(cfg->highrate_enabled);
        else if (strcmp(key, "rate_hz")     == 0) NEED_POS_INT(cfg->highrate_rate_hz);
        else if (strcmp(key, "dest_addr")   == 0) NEED_STR(cfg->highrate_dest_addr);
        else if (strcmp(key, "dest_port")   == 0) NEED_PORT(cfg->highrate_dest_port);
        else if (strcmp(key, "coord_frame") == 0) NEED_STR(cfg->highrate_coord_frame);
        else WARN_UNKNOWN();
        break;

    case SEC_STREAM:
        if      (strcmp(key, "enabled") == 0) NEED_BOOL(cfg->stream_enabled);
        else if (strcmp(key, "socket")  == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "rate_hz") == 0) NEED_POS_INT(cfg->stream_rate_hz);
        else if (strcmp(key, "tcp_enabled")   == 0) NEED_BOOL(cfg->stream_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr") == 0) NEED_STR(cfg->stream_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")      == 0) NEED_PORT(cfg->stream_tcp_port);
        else WARN_UNKNOWN();
        break;

    case SEC_SIGNALK:
        if      (strcmp(key, "enabled")      == 0) NEED_BOOL(cfg->sk_enabled);
        else if (strcmp(key, "udp_enabled")  == 0) NEED_BOOL(cfg->sk_udp_enabled);
        else if (strcmp(key, "socket")       == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "dest_addr")    == 0) NEED_STR(cfg->sk_dest_addr);
        else if (strcmp(key, "dest_port")    == 0) NEED_PORT(cfg->sk_dest_port);
        else if (strcmp(key, "rate_hz")      == 0) NEED_POS_INT(cfg->sk_rate_hz);
        else if (strcmp(key, "source_label") == 0) NEED_STR(cfg->sk_source_label);
        else if (strcmp(key, "publish_heave")== 0) NEED_BOOL(cfg->publish_heave);
        else if (strcmp(key, "tcp_enabled")   == 0) NEED_BOOL(cfg->sk_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr") == 0) NEED_STR(cfg->sk_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")      == 0) NEED_PORT(cfg->sk_tcp_port);
        else WARN_UNKNOWN();
        break;

    case SEC_MQTT:
        if      (strcmp(key, "enabled")       == 0) NEED_BOOL(cfg->mqtt_enabled);
        else if (strcmp(key, "broker_enabled")== 0) NEED_BOOL(cfg->mqtt_broker_enabled);
        else if (strcmp(key, "socket")        == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "broker_addr")   == 0) NEED_STR(cfg->mqtt_broker_addr);
        else if (strcmp(key, "broker_port")   == 0) NEED_PORT(cfg->mqtt_broker_port);
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
        else if (strcmp(key, "udp_port")      == 0) NEED_PORT(cfg->influx_udp_port);
        else if (strcmp(key, "http_enabled")  == 0) NEED_BOOL(cfg->influx_http_enabled);
        else if (strcmp(key, "http_host")     == 0) NEED_STR(cfg->influx_http_host);
        else if (strcmp(key, "http_port")     == 0) NEED_PORT(cfg->influx_http_port);
        else if (strcmp(key, "http_path")     == 0) NEED_STR(cfg->influx_http_path);
        else if (strcmp(key, "http_token")    == 0) NEED_STR(cfg->influx_http_token);
        else WARN_UNKNOWN();
        break;

    case SEC_PROM:
        if      (strcmp(key, "enabled")      == 0) NEED_BOOL(cfg->prom_enabled);
        else if (strcmp(key, "http_enabled") == 0) NEED_BOOL(cfg->prom_http_enabled);
        else if (strcmp(key, "socket")       == 0) NEED_STR(cfg->stream_socket);
        else if (strcmp(key, "listen_addr")  == 0) NEED_STR(cfg->prom_listen_addr);
        else if (strcmp(key, "listen_port")  == 0) NEED_PORT(cfg->prom_listen_port);
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
        else if (strcmp(key, "udp_port")       == 0) NEED_PORT(cfg->mav_udp_port);
        else if (strcmp(key, "serial_enabled") == 0) NEED_BOOL(cfg->mav_serial_enabled);
        else if (strcmp(key, "serial_device")  == 0) NEED_STR(cfg->mav_serial_device);
        else if (strcmp(key, "serial_baud")    == 0) NEED_INT(cfg->mav_serial_baud);
        else if (strcmp(key, "tcp_enabled")    == 0) NEED_BOOL(cfg->mav_tcp_enabled);
        else if (strcmp(key, "tcp_bind_addr")  == 0) NEED_STR(cfg->mav_tcp_bind_addr);
        else if (strcmp(key, "tcp_port")       == 0) NEED_PORT(cfg->mav_tcp_port);
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
        else if (strcmp(key, "gpsd_port")         == 0) NEED_PORT(cfg->pos_gpsd_port);
        else if (strcmp(key, "signalk_enabled")   == 0) NEED_BOOL(cfg->pos_signalk_enabled);
        else if (strcmp(key, "signalk_host")      == 0) NEED_STR(cfg->pos_signalk_host);
        else if (strcmp(key, "signalk_port")      == 0) NEED_PORT(cfg->pos_signalk_port);
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
