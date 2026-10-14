/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * json_esc.h — a configured string as the inside of a JSON string.
 *
 * Header-only and static inline, like fileio.h, so that a bridge encoder can
 * use it with no Makefile change.
 */
#ifndef IMUD_JSON_ESC_H
#define IMUD_JSON_ESC_H

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* Room for any string of n characters, escaped: \u00XX is six. */
#define JSON_ESC_SIZE(n) (6 * (n) + 1)

/*
 * Copy s into out with '"', '\\' and the control characters escaped.  Always
 * terminates, and stops short rather than split an escape.  Returns out.
 */
static inline const char *json_esc(char *out, size_t sz, const char *s)
{
    size_t o = 0;
    for (; s && *s; s++) {
        unsigned char c = (unsigned char)*s;
        char e[8];
        int len = 1;
        if (c == '"' || c == '\\') { e[0] = '\\'; e[1] = (char)c; len = 2; }
        else if (c < 0x20)         len = snprintf(e, sizeof e, "\\u%04x", c);
        else                       e[0] = (char)c;
        if (o + (size_t)len >= sz) break;
        memcpy(out + o, e, (size_t)len);
        o += (size_t)len;
    }
    if (sz) out[o] = '\0';
    return out;
}

#endif /* IMUD_JSON_ESC_H */
