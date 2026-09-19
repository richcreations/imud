/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * drv_state.h — per-handle driver state for a hand-built handle.
 *
 * A suite that builds an imud_bus_t itself, rather than opening one through
 * imu_bus_open()/mag_bus_open(), has to supply the bus->drv those would have
 * allocated, or the driver's first hook dereferences NULL.
 *
 * Keyed by whatever identifies the handle — an I2C address, a SPI descriptor,
 * an ops pointer — so one key gets one slab that persists across calls, as a
 * real handle's state does, and two keys keep their state apart.
 *
 * Every slab is DRV_STATE_MAX regardless of the driver asking, because two
 * drivers can answer at one mock address (the LSM6DSO and the ISM330DHCX are
 * both 0x6A) and a slab sized for whichever asked first would be a heap
 * overrun for the other.  test_drv_state_fits pins the bound against the
 * registry.
 *
 * Header-only, and the slabs are never freed: a test process is short and the
 * state has to outlive every handle built from the same key.
 */
#ifndef IMUD_TEST_DRV_STATE_H
#define IMUD_TEST_DRV_STATE_H

#include <stdlib.h>
#include <string.h>

#define DRV_STATE_MAX 256

static inline void *drv_state(const void *key, const void *tmpl,
                              size_t tmpl_bytes)
{
    static struct { const void *key; void *st; } slot[32];

    if (tmpl_bytes > DRV_STATE_MAX) return NULL;   /* the bound test says so */
    for (unsigned i = 0; i < sizeof slot / sizeof slot[0]; i++) {
        if (slot[i].key == key) return slot[i].st;
        if (slot[i].key) continue;
        slot[i].key = key;
        slot[i].st  = calloc(1, DRV_STATE_MAX);
        if (slot[i].st && tmpl) memcpy(slot[i].st, tmpl, tmpl_bytes);
        return slot[i].st;
    }
    return NULL;   /* out of slots: the caller's assertions will say so */
}

/* The state for a driver named by its ops, which is the key a handle with no
 * address of its own can use. */
#define DRV_STATE_FOR(o)                                                      \
    ((o)->state_bytes ? drv_state((o), (o)->state_init, (o)->state_bytes)     \
                      : NULL)

#endif /* IMUD_TEST_DRV_STATE_H */
