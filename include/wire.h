/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * wire.h — little-endian scalar accessors for imud's byte-oriented formats
 *
 * imud's binary formats (the 276-byte wire packet, the .imucap file) are
 * little-endian on the wire on every host.  These read and write them through
 * shifts on the VALUE, never through the host's in-memory representation, so
 * one compiled path is correct on either endianness — a big-endian host runs
 * exactly the bytes a little-endian host does, and the same code is what the
 * tests exercise.
 *
 * Floats go via a same-width unsigned integer, which is where the assumption
 * lives: the host's float byte order must match its integer byte order.  That
 * holds on every ABI imud targets; the historical exceptions (ARM OABI's
 * mixed-endian doubles) predate anything with a C11 compiler.
 *
 * Header-only and static inline on purpose: the callers span the daemon,
 * libimud and the capture tests, and this way none of them needs a Makefile
 * change.  Same reasoning as include/fileio.h.
 *
 * lib/imud_client.h carries its own copy of these, deliberately — that header
 * must compile standalone against libc, like the CLOEXEC helpers it also
 * duplicates.
 */
#ifndef IMUD_WIRE_H
#define IMUD_WIRE_H

#include <stdint.h>
#include <string.h>

/* ── Store: host value → little-endian bytes ─────────────────────────────── */

static inline void wire_put_u8(uint8_t *p, uint8_t v)
{
    p[0] = v;
}

static inline void wire_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void wire_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void wire_put_u64(uint8_t *p, uint64_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    p[4] = (uint8_t)(v >> 32);
    p[5] = (uint8_t)(v >> 40);
    p[6] = (uint8_t)(v >> 48);
    p[7] = (uint8_t)(v >> 56);
}

static inline void wire_put_f32(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    wire_put_u32(p, u);
}

/* ── Load: little-endian bytes → host value ──────────────────────────────── */

static inline uint8_t wire_get_u8(const uint8_t *p)
{
    return p[0];
}

static inline uint16_t wire_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t wire_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0]        | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t wire_get_u64(const uint8_t *p)
{
    return (uint64_t)p[0]         | ((uint64_t)p[1] << 8)
         | ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24)
         | ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40)
         | ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

static inline float wire_get_f32(const uint8_t *p)
{
    uint32_t u = wire_get_u32(p);
    float    v;
    memcpy(&v, &u, 4);
    return v;
}

#endif /* IMUD_WIRE_H */
