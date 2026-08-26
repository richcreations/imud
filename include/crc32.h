/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * crc32.h — IEEE 802.3 CRC32, the checksum on the binary wire packet
 *
 * Bitwise reflected algorithm over polynomial 0xEDB88320, initial and final
 * XOR 0xFFFFFFFF.  CRC32("123456789") == 0xCBF43926.
 *
 * Header-only and static inline, like fileio.h and cloexec.h: the callers sit
 * in translation units that do not otherwise share an object — the daemon's
 * packet builder, the client library, and imud-mon's parser — so a shared .c
 * would put libimud on the daemon's link line, which src/bridge.c's comment
 * explains must not happen.
 *
 * test/test_packet.c keeps its own copy on purpose.  It recomputes the CRC
 * over a packet this code built and compares the two, so sharing one
 * implementation would have the test check that against itself.
 */

#ifndef IMUD_CRC32_H
#define IMUD_CRC32_H

#include <stddef.h>
#include <stdint.h>

static inline uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFFu;
}

#endif /* IMUD_CRC32_H */
