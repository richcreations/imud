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
 *   roll:  + = starboard up (NED)
 *   heading_deg: 0–360° magnetic
 *   rate_of_turn: deg/min, + = turning right (clockwise from above)
 *   cov[0] = roll error variance (rad²)
 *   cov[4] = pitch error variance (rad²)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
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
 * $PASHR — proprietary attitude sentence (magnetic heading, always 'M').
 * roll_acc / pitch_acc from MEKF attitude covariance (cov[0], cov[4]).
 */
static int build_pashr(char *buf, size_t bufsz, int *pos,
                       const fused_state_t *s)
{
    float pitch_deg     = s->pitch * (float)(180.0 / M_PI);
    float roll_deg      = s->roll  * (float)(180.0 / M_PI);
    float roll_acc_deg  = sqrtf(s->cov[0]) * (float)(180.0 / M_PI);
    float pitch_acc_deg = sqrtf(s->cov[4]) * (float)(180.0 / M_PI);

    char content[128];
    snprintf(content, sizeof(content),
             "PASHR,%05.1f,M,%+06.1f,%+06.1f,0.0,%04.1f,%04.1f,0,A,,",
             s->heading_deg, roll_deg, pitch_deg,
             roll_acc_deg, pitch_acc_deg);

    return append_sentence(buf, bufsz, pos, content);
}

/* $HCHDM — magnetic heading */
static int build_hchdm(char *buf, size_t bufsz, int *pos, float heading_deg)
{
    char content[64];
    snprintf(content, sizeof(content), "HCHDM,%05.1f,M", heading_deg);
    return append_sentence(buf, bufsz, pos, content);
}

/* $HCHDT — true heading; only emitted when FLAG_DECLINATION_VALID is set */
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
 * Pitch: + = bow up.  Roll: + = starboard up.
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

    if (build_pashr(buf, bufsz, &pos, state)               < 0) return -1;
    if (build_hchdm(buf, bufsz, &pos, state->heading_deg)  < 0) return -1;
    if (state->flags & FLAG_DECLINATION_VALID) {
        if (build_hchdt(buf, bufsz, &pos,
                        state->heading_deg, state->declination_deg) < 0) return -1;
    }
    if (build_tirot(buf, bufsz, &pos, state->rate_of_turn) < 0) return -1;
    if (build_iixdr(buf, bufsz, &pos, pitch_deg, roll_deg) < 0) return -1;

    buf[pos] = '\0';
    return pos;
}
