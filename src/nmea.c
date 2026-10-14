/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * nmea.c — NMEA 0183 sentence encoder for imud (§7)
 *
 * Coordinate conventions (§6 / fusion.h):
 *   pitch: + = bow up     (NED)
 *   roll:  + = starboard down, i.e. a heel to starboard (NED)
 *   heading_deg: 0–360° magnetic
 *   rate_of_turn: deg/min, + = turning right (clockwise from above)
 *   cov[0] = roll error variance (rad²)
 *   cov[4] = pitch error variance (rad²)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "nmea.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Checksum ────────────────────────────────────────────────────────────── */

/* XOR of all bytes in content (not including $ or *). */
static uint8_t nmea_xor(const char *content)
{
    uint8_t cs = 0;
    for (const char *p = content; *p; p++)
        cs ^= (uint8_t)*p;
    return cs;
}

/*
 * Append "$content*HH\r\n" to buf+*pos.
 * Returns 0 on success, -1 if there is not enough space.
 */
static int append_sentence(char *buf, size_t bufsz, int *pos, const char *content)
{
    uint8_t cs  = nmea_xor(content);
    int     len = snprintf(buf + *pos, bufsz - (size_t)*pos,
                           "$%s*%02X\r\n", content, cs);
    if (len < 0 || (size_t)(*pos + len) >= bufsz) return -1;
    *pos += len;
    return 0;
}

/* ── Individual sentence builders ────────────────────────────────────────── */

/*
 * $PASHR — attitude, in the Trimble/Applanix layout:
 *
 *   PASHR,hhmmss.ss,hhh.hh,T,roll,pitch,heave,roll_acc,pitch_acc,hdg_acc,gnss,status
 *
 * Always emitted, because roll, pitch and heave stay valid with no
 * magnetometer at all.  Heading is TRUE, so it and its accuracy are null
 * unless the magnetometer is fused and declination is known; time is null
 * when the sample had none.  No GNSS fix aids the solution, so its quality
 * is 0.  Status is the alignment code: 4 converged and earth-referenced, 3 not
 * yet converged, 2 heading dead-reckoned.
 */
static int build_pashr(char *buf, size_t bufsz, int *pos,
                       const fused_state_t *s, int hdg_referenced)
{
    const float r2d = (float)(180.0 / M_PI);

    char utc[16] = "";
    time_t secs = (time_t)(s->ts_wall_ns / 1000000000ULL);
    struct tm tm;
    if (s->ts_wall_ns && gmtime_r(&secs, &tm))
        snprintf(utc, sizeof utc, "%02d%02d%02d.%02u", tm.tm_hour, tm.tm_min,
                 tm.tm_sec, (unsigned)(s->ts_wall_ns % 1000000000ULL / 10000000ULL));

    char hdg[16] = "", hdg_acc[16] = "";
    if (hdg_referenced && (s->flags & FLAG_DECLINATION_VALID)) {
        snprintf(hdg, sizeof hdg, "%06.2f",
                 fmodf(s->heading_deg + s->declination_deg + 360.0f, 360.0f));
        /* The attitude variance about the vertical, d = R(q)ᵀ·down. */
        const float *q = s->q;
        float d[3] = { 2.0f*(q[1]*q[3] - q[0]*q[2]), 2.0f*(q[2]*q[3] + q[0]*q[1]),
                       1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]) };
        float var = 0.0f;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) var += d[i] * s->cov[3*i + j] * d[j];
        snprintf(hdg_acc, sizeof hdg_acc, "%.3f", sqrtf(fmaxf(var, 0.0f)) * r2d);
    }

    int status = !hdg_referenced ? 2
               : (s->flags & FLAG_FUSION_CONVERGED) ? 4 : 3;

    char content[160];
    snprintf(content, sizeof(content),
             "PASHR,%s,%s,T,%+.2f,%+.2f,%+.2f,%.3f,%.3f,%s,0,%d",
             utc, hdg, s->roll * r2d, s->pitch * r2d, s->heave_m,
             sqrtf(s->cov[0]) * r2d, sqrtf(s->cov[4]) * r2d, hdg_acc, status);

    return append_sentence(buf, bufsz, pos, content);
}

/* $HCHDM — magnetic heading; only emitted when the heading is magnetic */
static int build_hchdm(char *buf, size_t bufsz, int *pos, float heading_deg)
{
    char content[64];
    snprintf(content, sizeof(content), "HCHDM,%05.1f,M", heading_deg);
    return append_sentence(buf, bufsz, pos, content);
}

/*
 * $HCHDG — heading, deviation, variation; gated like $HCHDM.
 * Deviation fields are empty: hard/soft-iron calibration is applied upstream,
 * so there is no residual deviation estimate to report.
 * Variation fields carry the WMM/static declination when known (E = east/+,
 * W = west/-) and are empty otherwise — this lets consumers like Signal K
 * ingest magnetic variation directly and apply it themselves.
 */
static int build_hchdg(char *buf, size_t bufsz, int *pos,
                       float heading_deg, float decl_deg, int decl_valid)
{
    char content[64];
    if (decl_valid)
        snprintf(content, sizeof(content), "HCHDG,%05.1f,,,%.1f,%c",
                 heading_deg, fabsf(decl_deg),
                 decl_deg < 0.0f ? 'W' : 'E');
    else
        snprintf(content, sizeof(content), "HCHDG,%05.1f,,,,", heading_deg);
    return append_sentence(buf, bufsz, pos, content);
}

/* $HCHDT — true heading; adds FLAG_DECLINATION_VALID to $HCHDM's condition */
static int build_hchdt(char *buf, size_t bufsz, int *pos,
                       float heading_deg, float decl_deg)
{
    float true_hdg = fmodf(heading_deg + decl_deg + 360.0f, 360.0f);
    char content[64];
    snprintf(content, sizeof(content), "HCHDT,%05.1f,T", true_hdg);
    return append_sentence(buf, bufsz, pos, content);
}

/* $TIROT — rate of turn, deg/min, + = turning right */
static int build_tirot(char *buf, size_t bufsz, int *pos, float rot_deg_min)
{
    char content[64];
    snprintf(content, sizeof(content), "TIROT,%+.1f,A", rot_deg_min);
    return append_sentence(buf, bufsz, pos, content);
}

/*
 * $IIXDR — pitch and roll transducer.
 * Pitch: + = bow up.  Roll: + = starboard down.
 * Values always emitted with explicit sign per NMEA XDR convention.
 */
static int build_iixdr(char *buf, size_t bufsz, int *pos,
                       float pitch_deg, float roll_deg)
{
    char content[128];
    snprintf(content, sizeof(content),
             "IIXDR,A,%+.1f,D,PTCH,A,%+.1f,D,ROLL",
             pitch_deg, roll_deg);
    return append_sentence(buf, bufsz, pos, content);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int nmea_encode(char *buf, size_t bufsz, const fused_state_t *state)
{
    if (!buf || bufsz < NMEA_BUF_MIN) return -1;

    int   pos       = 0;
    float pitch_deg = state->pitch * (float)(180.0 / M_PI);
    float roll_deg  = state->roll  * (float)(180.0 / M_PI);

    int decl_valid = (state->flags & FLAG_DECLINATION_VALID) != 0;

    /*
     * Heading is earth-referenced only while the magnetometer is being fused,
     * calibrated (FLAG_MAG_VALID) or not (FLAG_MAG_UNCAL).  An uncalibrated
     * fuse still yields a magnetic heading — offset by the uncorrected hard
     * iron, but bounded — so it is published.  Both flags clear means no
     * magnetometer is fitted or it has stopped delivering, and heading is a
     * relative angle dead-reckoned from the gyro.  $HCHDM has no status field
     * and $HCHDG's is deviation, not validity, so neither can say that: the
     * only honest output is none.
     */
    int hdg_referenced =
        (state->flags & (FLAG_MAG_VALID | FLAG_MAG_UNCAL)) != 0;

    if (build_pashr(buf, bufsz, &pos, state, hdg_referenced) < 0) return -1;
    if (hdg_referenced) {
        if (build_hchdm(buf, bufsz, &pos, state->heading_deg) < 0) return -1;
        if (build_hchdg(buf, bufsz, &pos, state->heading_deg,
                        state->declination_deg, decl_valid)  < 0) return -1;
        if (state->flags & FLAG_DECLINATION_VALID) {
            if (build_hchdt(buf, bufsz, &pos,
                            state->heading_deg, state->declination_deg) < 0) return -1;
        }
    }
    if (build_tirot(buf, bufsz, &pos, state->rate_of_turn) < 0) return -1;
    if (build_iixdr(buf, bufsz, &pos, pitch_deg, roll_deg) < 0) return -1;

    buf[pos] = '\0';
    return pos;
}
