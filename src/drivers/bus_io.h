/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_io.h — shared register helpers for the hardware drivers.
 *
 * static inline in a driver-private header on purpose: every driver TU still
 * issues its own single ioctl(fd, I2C_RDWR, &xfer) per transfer, which is
 * exactly what test/bus_mock.c intercepts via --wrap=ioctl. Do not reroute
 * these through I2C_SLAVE, SMBus calls, or read()/write() — the mock models
 * only I2C_RDWR transactions with 1 or 2 messages.
 *
 * The bus handle carries the address, so a driver names only the register it
 * wants. See include/bus.h.
 */

#ifndef IMUD_DRIVERS_BUS_IO_H
#define IMUD_DRIVERS_BUS_IO_H

#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "bus.h"

/*
 * Combined write-then-read in a single I2C transaction (no repeated-start
 * overhead). Saves ~40 µs vs separate transactions at 400 kHz.
 */
static inline int bus_burst_read(const imud_bus_t *b, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    uint8_t r = reg;
    struct i2c_msg msgs[2] = {
        { .addr = b->i2c_addr, .flags = 0,        .len = 1,   .buf = &r  },
        { .addr = b->i2c_addr, .flags = I2C_M_RD, .len = len, .buf = buf },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(b->fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static inline int bus_reg_write(const imud_bus_t *b, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = b->i2c_addr, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(b->fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static inline int bus_reg_read(const imud_bus_t *b, uint8_t reg, uint8_t *val)
{
    return bus_burst_read(b, reg, val, 1);
}

/* Signed 16-bit assembly from a register byte pair. */
static inline int16_t reg_s16le(const uint8_t *p)   /* low byte first */
{
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static inline int16_t reg_s16be(const uint8_t *p)   /* high byte first */
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

#endif /* IMUD_DRIVERS_BUS_IO_H */
