/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mon_parse.c — imud-mon's stream decoding
 *
 * Moved out of mon_main.c so it can be tested; see include/mon_parse.h.
 * mon_flag_str is rewritten around a table (its remaining-space arithmetic
 * could underflow — see the comment there); the other three are unchanged.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "mon_parse.h"

/* ── Packet CRC ──────────────────────────────────────────────────────────── */

uint32_t mon_crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── NMEA ────────────────────────────────────────────────────────────────── */

bool mon_nmea_get_field(const char *sentence, int field, float *out)
{
    const char *p = strchr(sentence, ',');
    for (int i = 0; i < field; i++) {
        if (!p) return false;
        p = strchr(p + 1, ',');
    }
    if (!p) return false;
    p++;
    if (!*p || *p == ',' || *p == '*') return false;
    *out = strtof(p, NULL);
    return true;
}

/*
 * PASHR layout: $PASHR,hdg,M,roll,pitch,...
 *   field 0 = heading, field 2 = roll, field 3 = pitch
 */
void mon_parse_nmea(mon_state_t *st, const char *buf, size_t len)
{
    if (len >= NMEA_MAX) len = NMEA_MAX - 1;
    memcpy(st->nmea_raw, buf, len);
    st->nmea_raw[len] = '\0';
    st->have_nmea = true;
    st->nmea_has_true_hdg = false;   /* cleared each burst; set below if $HCHDT present */

    const char *pas = strstr(st->nmea_raw, "$PASHR,");
    if (pas) {
        float hdg = 0, roll = 0, pitch = 0;
        if (sscanf(pas + 7, "%f,M,%f,%f", &hdg, &roll, &pitch) == 3) {
            st->nmea_hdg   = hdg;
            st->nmea_roll  = roll;
            st->nmea_pitch = pitch;
        }
    }

    const char *rot = strstr(st->nmea_raw, "$TIROT,");
    if (rot) {
        float r = 0;
        if (mon_nmea_get_field(rot, 0, &r))
            st->nmea_rot = r;
    }

    const char *hdt = strstr(st->nmea_raw, "$HCHDT,");
    if (hdt) {
        float h = 0;
        if (mon_nmea_get_field(hdt, 0, &h)) {
            st->nmea_true_hdg    = h;
            st->nmea_has_true_hdg = true;
        }
    }
}

/* ── Flag summary ────────────────────────────────────────────────────────── */

/*
 * C=converged V=mag-valid A=accel-cal G=gyro-cal M=mag-cal D=declination
 * S=startup !=fifo-overflow
 *
 * The previous version tracked the remaining space as a size_t and decremented
 * it once per set flag with no floor, so eight flags against a small sz walked
 * it 1 -> 0 -> SIZE_MAX and the next snprintf() wrote past the end.  It was
 * unreachable in the shipped program — mon_main's only call site passes a
 * 32-byte buffer against a 17-byte maximum — but calling this directly from a
 * test is exactly what would reach it, so the length is tracked forward
 * against sz instead.  Output is unchanged wherever a length-driven form was in
 * bounds.
 */
void mon_flag_str(uint16_t flags, char *buf, size_t sz)
{
    if (!buf || sz == 0) return;

    static const struct { uint16_t bit; char ch; } tbl[] = {
        { FLAG_FUSION_CONVERGED,  'C' },
        { FLAG_MAG_VALID,         'V' },
        { FLAG_ACCEL_CAL,         'A' },
        { FLAG_GYRO_CAL,          'G' },
        { FLAG_MAG_CAL,           'M' },
        { FLAG_DECLINATION_VALID, 'D' },
        { FLAG_STARTUP,           'S' },
        { FLAG_FIFO_OVERFLOW,     '!' },
        { FLAG_STATE_RESET,       'R' },
    };

    int n = snprintf(buf, sz, "0x%04X [", flags);
    if (n < 0 || (size_t)n >= sz) return;   /* snprintf already NUL-terminated */

    size_t len = (size_t)n;
    for (size_t i = 0; i < sizeof tbl / sizeof *tbl; i++) {
        if (!(flags & tbl[i].bit)) continue;
        if (len + 1 >= sz) break;           /* no room for the char and the NUL */
        buf[len++] = tbl[i].ch;
    }
    if (len + 1 < sz) buf[len++] = ']';
    buf[len] = '\0';
}
