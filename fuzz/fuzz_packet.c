/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_packet.c — libFuzzer harness for the wire-packet validation and
 * decode path (packet_ok / fill_data in lib/libimud.c — static, so the TU
 * is included directly).  Two probes per input: the raw bytes as received,
 * and the same bytes normalized into a valid frame (magic/version/CRC fixed
 * up) so the field-decode path runs on arbitrary payloads too.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../lib/libimud.c"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    packet_ok(data, size);

    uint8_t buf[IMUD_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, size < sizeof(buf) ? size : sizeof(buf));

    uint32_t magic = IMUD_MAGIC;
    uint16_t ver   = IMUD_VERSION;
    memcpy(buf, &magic, 4);
    memcpy(buf + 4, &ver, 2);
    uint32_t crc = crc32_ieee(buf, offsetof(imud_packet_t, crc32));
    memcpy(buf + offsetof(imud_packet_t, crc32), &crc, 4);

    if (packet_ok(buf, sizeof(buf))) {
        imud_packet_t wire;
        memcpy(&wire, buf, sizeof(wire));
        imud_data_t d;
        memset(&d, 0, sizeof(d));
        fill_data(&d, &wire);
    }
    return 0;
}
