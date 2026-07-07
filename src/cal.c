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
#include "cal.h"
#include "log.h"

/* ── JSON float-array parser ─────────────────────────────────────────────── */

/*
 * Parse up to n floats from the first [...] block at or after *s.
 * '[', ']', and ',' are treated as delimiters alongside whitespace so that
 * both flat [a,b,c] and nested [[a,b,c],[d,e,f],...] arrays are fully parsed.
 */
static int parse_float_array(const char *s, float *out, int n)
{
    const char *p = strchr(s, '[');
    if (!p) return 0;
    p++;

    int count = 0;
    while (count < n) {
        while (*p && ((*p == ' ') || (*p == '\t') || (*p == '\n') ||
                      (*p == '\r') || (*p == ',') || (*p == '[') || (*p == ']')))
            p++;
        if (!*p) break;

        char *end;
        float v = strtof(p, &end);
        if (end == p) break;
        out[count++] = v;
        p = end;
    }
    return count;
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

    const char *sec = strstr(buf, "\"mag\"");
    if (sec) {
        const char *hi = strstr(sec, "\"hard_iron\"");
        if (hi && parse_float_array(hi, cal->mag_hard_iron, 3) == 3)
            cal->has_mag = true;

        const char *si = strstr(sec, "\"soft_iron\"");
        if (si) parse_float_array(si, &cal->mag_soft_iron[0][0], 9);
    }

    sec = strstr(buf, "\"accel\"");
    if (sec) {
        const char *off = strstr(sec, "\"offset\"");
        if (off && parse_float_array(off, cal->accel_offset, 3) == 3)
            cal->has_accel = true;

        const char *sc = strstr(sec, "\"scale\"");
        if (sc) parse_float_array(sc, cal->accel_scale, 3);
    }

    sec = strstr(buf, "\"gyro\"");
    if (sec) {
        const char *bias = strstr(sec, "\"bias\"");
        if (bias && parse_float_array(bias, cal->gyro_bias, 3) == 3)
            cal->has_gyro = true;
    }

    LOG_I("[cal] loaded %s  accel:%s  gyro:%s  mag:%s\n", path,
            cal->has_accel ? "yes" : "no",
            cal->has_gyro  ? "yes" : "no",
            cal->has_mag   ? "yes" : "no");
    return 0;
}

int cal_write(const char *path, const imud_cal_t *cal)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        LOG_E("[cal] cannot write %s: %s\n", path, strerror(errno));
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
        fprintf(f, "  }%s\n", (cal->has_gyro || cal->has_accel) ? "," : "");
    }

    if (cal->has_gyro) {
        fprintf(f, "  \"gyro\": {\n");
        fprintf(f, "    \"bias\": [ %.8f, %.8f, %.8f ]\n",
                cal->gyro_bias[0], cal->gyro_bias[1], cal->gyro_bias[2]);
        fprintf(f, "  }%s\n", cal->has_accel ? "," : "");
    }

    if (cal->has_accel) {
        fprintf(f, "  \"accel\": {\n");
        fprintf(f, "    \"offset\": [ %.6f, %.6f, %.6f ],\n",
                cal->accel_offset[0], cal->accel_offset[1],
                cal->accel_offset[2]);
        fprintf(f, "    \"scale\":  [ %.8f, %.8f, %.8f ]\n",
                cal->accel_scale[0], cal->accel_scale[1],
                cal->accel_scale[2]);
        fprintf(f, "  }\n");
    }

    fprintf(f, "}\n");
    fclose(f);
    return 0;
}
