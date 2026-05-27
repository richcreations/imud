/*
 * nmea.h — NMEA 0183 sentence encoder (§7)
 *
 * nmea_encode() builds one complete burst (4 or 5 sentences) into a flat
 * buffer ready for a single sendto() call.  All sentences use <CR><LF>
 * termination and correct XOR checksums.
 */

/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

#ifndef IMUD_NMEA_H
#define IMUD_NMEA_H

#include <stddef.h>
#include "types.h"
#include "config.h"

/*
 * Encode one 10 Hz burst into buf.
 *
 * Sentences emitted (in order):
 *   $PASHR   — attitude (magnetic heading, roll, pitch, accuracy)
 *   $HCHDM   — magnetic heading
 *   $HCHDT   — true heading (only when FLAG_DECLINATION_VALID is set)
 *   $TIROT   — rate of turn
 *   $IIXDR   — pitch and roll transducer
 *
 * Returns number of bytes written (≥ 0), or -1 if buf is too small.
 * buf must be at least NMEA_BUF_MIN bytes.
 */
#define NMEA_BUF_MIN  512

int nmea_encode(char *buf, size_t bufsz, const fused_state_t *state);

#endif /* IMUD_NMEA_H */
