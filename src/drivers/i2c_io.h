/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * i2c_io.h — shared I2C_RDWR register helpers for the hardware drivers.
 *
 * static inline in a driver-private header on purpose: every driver TU still
 * issues its own single ioctl(fd, I2C_RDWR, &xfer) per transfer, which is
 * exactly what test/i2c_mock.c intercepts via --wrap=ioctl. Do not reroute
 * these through I2C_SLAVE, SMBus calls, or read()/write() — the mock models
 * only I2C_RDWR transactions with 1 or 2 messages.
 */

#ifndef IMUD_DRIVERS_I2C_IO_H
#define IMUD_DRIVERS_I2C_IO_H

#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

/*
 * Combined write-then-read in a single I2C transaction (no repeated-start
 * overhead). Saves ~40 µs vs separate transactions at 400 kHz.
 */
static inline int i2c_burst_read(int fd, uint8_t addr, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    struct i2c_msg msgs[2] = {
        { .addr = addr, .flags = 0,        .len = 1,   .buf = &reg },
        { .addr = addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static inline int i2c_reg_write(int fd, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = addr, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static inline int i2c_reg_read(int fd, uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_burst_read(fd, addr, reg, val, 1);
}

/* Signed 16-bit assembly from a register byte pair. */
static inline int16_t i2c_s16le(const uint8_t *p)   /* low byte first */
{
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static inline int16_t i2c_s16be(const uint8_t *p)   /* high byte first */
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

#endif /* IMUD_DRIVERS_I2C_IO_H */
