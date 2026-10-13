/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cal.c — calibration JSON loader and writer
 *
 * Shared between the daemon (main.c) and the calibration tool (cal_main.c).
 * The JSON parser is intentionally minimal: no library dependency, handles
 * the specific structure written by cal_write and nothing else.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "cal.h"
#include "fileio.h"
#include "log.h"

/* ── Non-finite guard ────────────────────────────────────────────────────── */

/*
 * Return the name of the first non-finite calibration field, or NULL.
 *
 * strtof converts "nan", "inf" and "1e999" without complaint, and every value
 * here is applied to a live sample: cal->gyro_bias in particular goes straight
 * into mekf_init's f->bias, which makes w = gyro - bias non-finite on every
 * predict step.  The filter has no way back from that — q_normalize cannot
 * repair a NaN, since NaN fails its `n > 1e-10f` test and the quaternion is
 * passed through untouched — so the published attitude stays NaN for the life
 * of the process, and %.4f renders that into an NMEA sentence as "nan".
 *
 * The loop closes inside imud's own tooling: cal_write formats with %.8f,
 * which round-trips a NaN back out as the literal "nan".  So the same guard
 * runs in both directions.
 *
 * gated: when true, inspect only sections whose has_* flag is set — that is
 * exactly what cal_write emits, so a stale NaN in a section that will not be
 * written is not an error.  The loader passes false: a number that appeared
 * in the file is rejected wherever it appeared, and sections the file did not
 * mention still hold the finite identity defaults cal_load set up front.
 */
static const char *cal_first_non_finite(const imud_cal_t *cal, bool gated)
{
    const struct {
        const char  *name;
        const float *v;
        int          n;
        bool         has;
    } fields[] = {
        { "mag.hard_iron",          cal->mag_hard_iron,         3, cal->has_mag       },
        { "mag.soft_iron",          &cal->mag_soft_iron[0][0],  9, cal->has_mag       },
        { "gyro.bias",              cal->gyro_bias,             3, cal->has_gyro      },
        { "accel.offset",           cal->accel_offset,          3, cal->has_accel     },
        { "accel.scale",            cal->accel_scale,           3, cal->has_accel     },
        { "noise.gyro_density",     cal->gyro_noise_density,    3, cal->has_noise     },
        { "noise.gyro_instability", cal->gyro_bias_instability, 3, cal->has_noise     },
        { "noise.accel_density",    cal->accel_noise_density,   3, cal->has_noise     },
        { "gyro_temp.coeff",        cal->gyro_temp_coeff,       3, cal->has_gyro_temp },
        { "gyro_temp.ref_c",        &cal->gyro_temp_ref_c,      1, cal->has_gyro_temp },
    };

    for (size_t i = 0; i < sizeof fields / sizeof fields[0]; i++) {
        if (gated && !fields[i].has) continue;
        for (int k = 0; k < fields[i].n; k++)
            if (!isfinite(fields[i].v[k])) return fields[i].name;
    }
    return NULL;
}

/*
 * Reject a calibration whose own arithmetic overflows on an ordinary sample.
 *
 * cal_first_non_finite above closes the question of a non-finite value being
 * *stored*. It does not ask whether a FINITE one is usable, and that gap is
 * reachable with the most ordinary reading there is. fuzz_cal found it on
 * main: mag.soft_iron[0][0] = 3.33e37 is finite, so it loaded — and the mag
 * correction is soft_iron × (sample − hard_iron), which for a sample of ZERO
 * is 3.33e37 × −11.4 = −3.8e38. A float holds 3.4e38. The result is −inf, in
 * the filter, from a calibration this loader called good.
 *
 * Bounding calibration RANGES was deferred deliberately: choosing a legitimate
 * maximum hard-iron offset or soft-iron element is a sensor-domain decision,
 * and too tight a bound rejects a valid calibration — a worse failure than
 * accepting a silly one. This sidesteps that decision entirely. It does not
 * ask whether a value is physically sensible, only whether the arithmetic
 * survives, so a calibration must be some thirty-five orders of magnitude out
 * before it trips. It cannot reject anything a bench produces.
 *
 * PROBE is above the full scale of every sensor in the tree (±800 µT for the
 * MMC5983MA, ±16 g ≈ 157 m/s², ±2000 dps ≈ 35 rad/s), so a calibration that
 * overflows here overflows on readings the hardware can actually produce.
 * Zero is probed explicitly because that is the value that exposed this.
 *
 * The arithmetic mirrors apply_imu_cal / apply_mag_cal in src/imu_math.c and
 * is duplicated rather than called: cal.c links against nothing but libc and
 * log.c, and test_cal does not link imu_math.c. fuzz_cal's oracle drives the
 * REAL functions, so if the two ever drift, it fires on a calibration this
 * accepted — which is the cross-check that keeps the duplication honest.
 */
static const char *cal_first_overflowing(const imud_cal_t *cal, bool gated)
{
    static const float probes[] = { 0.0f, 1000.0f, -1000.0f };

    for (size_t p = 0; p < sizeof probes / sizeof probes[0]; p++) {
        const float v = probes[p];

        if (!gated || cal->has_mag) {
            float tmp[3];
            for (int i = 0; i < 3; i++) tmp[i] = v - cal->mag_hard_iron[i];
            for (int i = 0; i < 3; i++) {
                float f = cal->mag_soft_iron[i][0] * tmp[0]
                        + cal->mag_soft_iron[i][1] * tmp[1]
                        + cal->mag_soft_iron[i][2] * tmp[2];
                if (!isfinite(f)) return "mag.hard_iron/soft_iron";
            }
        }

        if (!gated || cal->has_accel)
            for (int i = 0; i < 3; i++)
                if (!isfinite((v - cal->accel_offset[i]) * cal->accel_scale[i]))
                    return "accel.offset/scale";

        if (!gated || cal->has_gyro_temp) {
            float dT = v - cal->gyro_temp_ref_c;
            for (int i = 0; i < 3; i++)
                if (!isfinite(v - cal->gyro_temp_coeff[i] * dT))
                    return "gyro_temp.coeff/ref_c";
        }
    }
    return NULL;
}

/* ── JSON float-array parser ─────────────────────────────────────────────── */

/*
 * Parse the first [...] block at or after *s into out[0..n), and return how
 * many numbers the block holds.  Brackets are followed to the matching close,
 * so a flat [a,b,c] counts three and a nested [[a,b,c],[d,e,f],[g,h,i]] nine.
 * Returns -1 when the block is never closed or holds something that is not a
 * number.
 *
 * The count is the point: the caller compares it with n, because a block that
 * is short or long is a file that was cut off or edited wrong.
 */
static int parse_float_array(const char *s, float *out, int n)
{
    const char *p = strchr(s, '[');
    if (!p) return -1;

    int depth = 0, count = 0;
    for (;;) {
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ',')
            p++;
        if (*p == '[') { depth++; p++; continue; }
        if (*p == ']') {
            p++;
            if (--depth == 0) return count;
            continue;
        }
        if (!*p) return -1;

        char *end;
        float v = strtof(p, &end);
        if (end == p) return -1;
        if (count < n) out[count] = v;
        count++;
        p = end;
    }
}

/*
 * Load one array-valued key of a section.  Returns 1 when it is there and
 * whole, 0 when the section has no such key, and -1 — having said why — when
 * it is there but is not exactly n numbers.
 */
static int load_array(const char *path, const char *sec, const char *key,
                      const char *name, float *out, int n)
{
    const char *k = strstr(sec, key);
    if (!k) return 0;

    int got = parse_float_array(k, out, n);
    if (got == n) return 1;
    if (got < 0)
        LOG_E("[cal] %s: %s is not a complete array of numbers\n", path, name);
    else
        LOG_E("[cal] %s: %s holds %d number%s where %d are required\n",
              path, name, got, got == 1 ? "" : "s", n);
    return -1;
}

/*
 * True when the text is one closed JSON object.  A file cut off between two
 * sections holds nothing but whole arrays, and only this says it is not whole.
 */
static bool json_object_closed(const char *s)
{
    int  depth = 0;
    bool any   = false;
    for (; *s; s++) {
        if (*s == '{') { depth++; any = true; }
        else if (*s == '}' && --depth < 0) return false;
    }
    return any && depth == 0;
}

/* Parse the scalar after the next ':' at or after *s. */
static int parse_scalar(const char *s, float *out)
{
    const char *p = strchr(s, ':');
    if (!p) return 0;
    char *end;
    float v = strtof(p + 1, &end);
    if (end == p + 1) return 0;
    *out = v;
    return 1;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int cal_load(const char *path, imud_cal_t *cal)
{
    memset(cal, 0, sizeof(*cal));
    /* Identity defaults so apply_*_cal is a no-op when flags are false. */
    cal->accel_scale[0] = cal->accel_scale[1] = cal->accel_scale[2] = 1.0f;
    cal->mag_soft_iron[0][0] = cal->mag_soft_iron[1][1]
        = cal->mag_soft_iron[2][2] = 1.0f;

    FILE *f = fopen(path, "r");
    if (!f) {
        if (errno == ENOENT) {
            LOG_W("[cal] %s not found — running uncalibrated\n", path);
            return 0;
        }
        LOG_E("[cal] cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }

    char buf[8192];
    size_t nr = fread(buf, 1, sizeof(buf) - 1, f);
    int ferr = ferror(f);
    fclose(f);
    if (ferr) {
        LOG_E("[cal] read error on %s\n", path);
        return -1;
    }
    if (nr == sizeof(buf) - 1)
        LOG_W("[cal] WARNING: %s may be truncated (exceeds %zu bytes)\n",
                path, sizeof(buf) - 1);
    buf[nr] = '\0';

    /*
     * A file that is not whole is refused, like one holding a NaN below and
     * for the same reason: whatever loads from it reaches the filter as if it
     * were the calibration.  A write cut short leaves either an object that is
     * never closed or an array without all its numbers.
     */
    if (!json_object_closed(buf)) {
        LOG_E("[cal] %s is empty or cut off — refusing to start on part of a "
              "calibration\n", path);
        return -1;
    }

    int got;
    const char *sec = strstr(buf, "\"mag\"");
    if (sec) {
        got = load_array(path, sec, "\"hard_iron\"", "mag.hard_iron",
                         cal->mag_hard_iron, 3);
        if (got < 0) return -1;
        cal->has_mag = (got == 1);
        if (load_array(path, sec, "\"soft_iron\"", "mag.soft_iron",
                       &cal->mag_soft_iron[0][0], 9) < 0) return -1;
    }

    sec = strstr(buf, "\"accel\"");
    if (sec) {
        got = load_array(path, sec, "\"offset\"", "accel.offset",
                         cal->accel_offset, 3);
        if (got < 0) return -1;
        cal->has_accel = (got == 1);
        if (load_array(path, sec, "\"scale\"", "accel.scale",
                       cal->accel_scale, 3) < 0) return -1;
    }

    sec = strstr(buf, "\"gyro\"");
    if (sec) {
        got = load_array(path, sec, "\"bias\"", "gyro.bias", cal->gyro_bias, 3);
        if (got < 0) return -1;
        cal->has_gyro = (got == 1);
    }

    sec = strstr(buf, "\"noise\"");
    if (sec) {
        got = load_array(path, sec, "\"gyro_density\"", "noise.gyro_density",
                         cal->gyro_noise_density, 3);
        if (got < 0) return -1;
        cal->has_noise = (got == 1);
        if (load_array(path, sec, "\"gyro_instability\"",
                       "noise.gyro_instability",
                       cal->gyro_bias_instability, 3) < 0) return -1;
        if (load_array(path, sec, "\"accel_density\"", "noise.accel_density",
                       cal->accel_noise_density, 3) < 0) return -1;
    }

    sec = strstr(buf, "\"gyro_temp\"");
    if (sec) {
        got = load_array(path, sec, "\"coeff\"", "gyro_temp.coeff",
                         cal->gyro_temp_coeff, 3);
        if (got < 0) return -1;
        const char *rc = strstr(sec, "\"ref_c\"");
        if (got == 1 && rc && parse_scalar(rc, &cal->gyro_temp_ref_c))
            cal->has_gyro_temp = true;
    }

    /* Before anything reports success: a non-finite value here would reach
     * the filter and never wash out.  Fatal, not clamped — substituting a
     * plausible default would hide the corruption that produced it. */
    const char *bad = cal_first_non_finite(cal, false);
    if (bad) {
        LOG_E("[cal] %s: %s is not a finite number — refusing to start on a "
                "calibration that would put NaN into the filter\n", path, bad);
        return -1;
    }

    /* Finite is not the same as usable: a finite-but-absurd value overflows to
     * infinity the first time it is applied.  Same fatal treatment, since the
     * result reaching the filter is identical. */
    bad = cal_first_overflowing(cal, false);
    if (bad) {
        LOG_E("[cal] %s: %s overflows to infinity when applied to an ordinary "
                "sample — refusing to start on it\n", path, bad);
        return -1;
    }

    LOG_I("[cal] loaded %s  accel:%s  gyro:%s  mag:%s  noise:%s  temp:%s\n",
            path,
            cal->has_accel     ? "yes" : "no",
            cal->has_gyro      ? "yes" : "no",
            cal->has_mag       ? "yes" : "no",
            cal->has_noise     ? "yes" : "no",
            cal->has_gyro_temp ? "yes" : "no");
    return 0;
}

int cal_write(const char *path, const imud_cal_t *cal)
{
    /* A fit that produced a NaN is a failed fit; say so and leave what is on
     * disk alone. */
    const char *bad = cal_first_non_finite(cal, true);
    if (bad) {
        LOG_E("[cal] refusing to write %s: %s is not a finite number — "
                "the calibration on disk is unchanged\n", path, bad);
        return -1;
    }
    bad = cal_first_overflowing(cal, true);
    if (bad) {
        LOG_E("[cal] refusing to write %s: %s overflows to infinity when "
                "applied — the calibration on disk is unchanged\n", path, bad);
        return -1;
    }

    /*
     * Written beside the file and renamed over it, so cal.json is at every
     * instant either the old calibration or the new one.  Truncating it in
     * place left a power cut or a full disk with part of a file, which loaded.
     */
    char tmp[PATH_MAX];
    if (snprintf(tmp, sizeof tmp, "%s.tmp", path) >= (int)sizeof tmp) {
        LOG_E("[cal] cannot write %s: path too long\n", path);
        return -1;
    }
    FILE *f = fcreate(tmp, "w", IMUD_FILE_MODE);
    if (!f) {
        LOG_E("[cal] cannot write %s: %s — the calibration on disk is "
              "unchanged\n", tmp, strerror(errno));
        return -1;
    }

    fprintf(f, "{\n");

    if (cal->has_mag) {
        fprintf(f, "  \"mag\": {\n");
        fprintf(f, "    \"hard_iron\": [ %.6f, %.6f, %.6f ],\n",
                cal->mag_hard_iron[0], cal->mag_hard_iron[1],
                cal->mag_hard_iron[2]);
        fprintf(f, "    \"soft_iron\": [\n");
        for (int i = 0; i < 3; i++) {
            fprintf(f, "      [ %.8f, %.8f, %.8f ]%s\n",
                    cal->mag_soft_iron[i][0],
                    cal->mag_soft_iron[i][1],
                    cal->mag_soft_iron[i][2],
                    i < 2 ? "," : "");
        }
        fprintf(f, "    ]\n");
        fprintf(f, "  }%s\n", (cal->has_gyro || cal->has_accel ||
                               cal->has_noise || cal->has_gyro_temp) ? "," : "");
    }

    if (cal->has_gyro) {
        fprintf(f, "  \"gyro\": {\n");
        fprintf(f, "    \"bias\": [ %.8f, %.8f, %.8f ]\n",
                cal->gyro_bias[0], cal->gyro_bias[1], cal->gyro_bias[2]);
        fprintf(f, "  }%s\n", (cal->has_accel || cal->has_noise ||
                               cal->has_gyro_temp) ? "," : "");
    }

    if (cal->has_accel) {
        fprintf(f, "  \"accel\": {\n");
        fprintf(f, "    \"offset\": [ %.6f, %.6f, %.6f ],\n",
                cal->accel_offset[0], cal->accel_offset[1],
                cal->accel_offset[2]);
        fprintf(f, "    \"scale\":  [ %.8f, %.8f, %.8f ]\n",
                cal->accel_scale[0], cal->accel_scale[1],
                cal->accel_scale[2]);
        fprintf(f, "  }%s\n", (cal->has_noise || cal->has_gyro_temp) ? "," : "");
    }

    if (cal->has_noise) {
        fprintf(f, "  \"noise\": {\n");
        fprintf(f, "    \"gyro_density\":     [ %.6e, %.6e, %.6e ],\n",
                cal->gyro_noise_density[0], cal->gyro_noise_density[1],
                cal->gyro_noise_density[2]);
        fprintf(f, "    \"gyro_instability\": [ %.6e, %.6e, %.6e ],\n",
                cal->gyro_bias_instability[0], cal->gyro_bias_instability[1],
                cal->gyro_bias_instability[2]);
        fprintf(f, "    \"accel_density\":    [ %.6e, %.6e, %.6e ]\n",
                cal->accel_noise_density[0], cal->accel_noise_density[1],
                cal->accel_noise_density[2]);
        fprintf(f, "  }%s\n", cal->has_gyro_temp ? "," : "");
    }

    if (cal->has_gyro_temp) {
        fprintf(f, "  \"gyro_temp\": {\n");
        fprintf(f, "    \"coeff\": [ %.6e, %.6e, %.6e ],\n",
                cal->gyro_temp_coeff[0], cal->gyro_temp_coeff[1],
                cal->gyro_temp_coeff[2]);
        fprintf(f, "    \"ref_c\": %.2f\n", cal->gyro_temp_ref_c);
        fprintf(f, "  }\n");
    }

    fprintf(f, "}\n");

    /* A write that failed anywhere above is latched in the stream's error
     * indicator; fflush is where the buffer meets the disk, and fsync where
     * the disk is asked to mean it. */
    bool ok    = fflush(f) == 0 && !ferror(f) && fsync(fileno(f)) == 0;
    int  saved = errno;
    if (fclose(f) != 0 && ok) {
        ok    = false;
        saved = errno;
    }
    if (ok && rename(tmp, path) != 0) {
        ok    = false;
        saved = errno;
    }
    if (!ok) {
        LOG_E("[cal] writing %s failed: %s — the calibration on disk is "
              "unchanged\n", path, strerror(saved));
        unlink(tmp);
        return -1;
    }

    /* The rename is a change to the directory, so that is what must reach
     * the disk for it to survive a power cut.  Best effort: not every
     * filesystem lets a directory be opened and synced. */
    char dir[PATH_MAX];
    const char *slash = strrchr(path, '/');
    if (!slash)             snprintf(dir, sizeof dir, ".");
    else if (slash == path) snprintf(dir, sizeof dir, "/");
    else                    snprintf(dir, sizeof dir, "%.*s",
                                     (int)(slash - path), path);
    int dfd = open(dir, O_RDONLY | O_CLOEXEC);
    if (dfd >= 0) {
        (void)fsync(dfd);
        close(dfd);
    }
    return 0;
}
