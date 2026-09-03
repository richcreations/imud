/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mon_parse.h — imud-mon's stream decoding
 *
 * imud-mon reads the two UDP output streams as any other consumer would and
 * prints a one-line summary of each per second.  Everything it has to decode
 * to do that lives here: the packet CRC, the NMEA field extraction, and the
 * flag-bit summary.  mon_main.c keeps the sockets, the select() loop and the
 * printing.
 *
 * All four functions are pure — they were already, but they were static in a
 * TU no test binary linked.  mon_parse_nmea in particular runs on bytes off
 * the wire, so its bounds are worth an assertion rather than a reading.
 */

#ifndef IMUD_MON_PARSE_H
#define IMUD_MON_PARSE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "types.h"

#define NMEA_MAX 512

typedef struct {
    /* NMEA — most recent burst, parsed fields */
    char  nmea_raw[NMEA_MAX];
    float nmea_hdg, nmea_roll, nmea_pitch, nmea_rot;
    float nmea_true_hdg;
    bool  have_nmea;
    bool  nmea_has_true_hdg;
    bool  nmea_has_hdg;      /* $PASHR carried a heading, not a null field */

    /* Binary — most recent valid packet */
    imu_packet_t bin_pkt;
    bool         have_binary;
} mon_state_t;

/* CRC-32/ISO-HDLC (the "IEEE" polynomial), as the wire packet carries. */
uint32_t mon_crc32_ieee(const uint8_t *data, size_t len);

/* Extract a float from field N (0-based) of a single NMEA sentence.
 * False when the field does not exist or is empty. */
bool mon_nmea_get_field(const char *sentence, int field, float *out);

/*
 * Parse a received NMEA buffer that may hold several sentences.  Extracts
 * heading/roll/pitch from $PASHR, rate-of-turn from $TIROT and true heading
 * from $HCHDT.  Buffers longer than NMEA_MAX - 1 are truncated.
 *
 * nmea_has_true_hdg is cleared on every call and set only if this burst
 * carried $HCHDT, so a declination that stops being available stops being
 * displayed.  nmea_has_hdg tracks $PASHR's heading field the same way, since
 * imud nulls it when no magnetometer is being fused.  The other fields persist
 * across bursts by design — a sentence absent from one datagram keeps its last
 * value rather than blanking.
 */
void mon_parse_nmea(mon_state_t *st, const char *buf, size_t len);

/* Compact flag summary: "0xNNNN [CVAGDS!]".  Always NUL-terminates and never
 * writes outside [buf, buf + sz); truncates when sz is too small. */
void mon_flag_str(uint16_t flags, char *buf, size_t sz);

#endif /* IMUD_MON_PARSE_H */
