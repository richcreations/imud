/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus.c — opening a sensor transport.  See include/bus.h.
 *
 * This is the one place that turns an operator's bus_spec_t into a live
 * descriptor, shared by imud, imud-cal and imud-imutest so all three refuse
 * the same configurations for the same reasons.  The per-transfer register
 * helpers are elsewhere, in src/drivers/bus_io.h, because they have to stay
 * static inline inside each driver TU.
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include "bus.h"
#include "log.h"

int bus_open(imud_bus_t *b, const bus_spec_t *spec, const char *who)
{
    bus_init(b);

    if (spec->kind != BUS_I2C) {
        LOG_E("[%s] unsupported bus kind %d\n", who, (int)spec->kind);
        return -1;
    }

    /* O_CLOEXEC on the open itself, per the tree-wide close-on-exec rule —
     * there is no window here for a fork to inherit the descriptor. */
    int fd = open(spec->node, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        LOG_E("[%s] cannot open %s: %s\n", who, spec->node, strerror(errno));
        return -1;
    }

    b->kind     = BUS_I2C;
    b->fd       = fd;
    b->i2c_addr = spec->i2c_addr;
    return 0;
}

void bus_close(imud_bus_t *b)
{
    if (!b) return;
    if (b->fd >= 0) close(b->fd);
    bus_init(b);
}
