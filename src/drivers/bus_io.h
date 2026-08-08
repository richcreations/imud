/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_io.h — shared register helpers for the hardware drivers.
 *
 * static inline in a driver-private header on purpose: every driver TU still
 * issues its own single ioctl() per transfer, which is exactly what
 * test/bus_mock.c intercepts via --wrap=ioctl. Do not reroute these through
 * I2C_SLAVE, SMBus calls, or read()/write() — the mock models only
 * ioctl(I2C_RDWR) with 1 or 2 messages and ioctl(SPI_IOC_MESSAGE) with the
 * same 1-or-2 shape.
 *
 * The bus handle carries the address, the transport and the SPI clock, so a
 * driver names only the register it wants and the same register logic runs on
 * either bus. See include/bus.h.
 */

#ifndef IMUD_DRIVERS_BUS_IO_H
#define IMUD_DRIVERS_BUS_IO_H

#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "bus.h"

/*
 * SPI command framing. Every part imud drives spells the direction in the top
 * bit of the command byte — 1 read, 0 write — with the register address in
 * the low 7. ISM330DHCX DS13012 Rev 7 §5.1.2 is the reference statement of
 * it; the other ST parts and the InvenSense parts use the same convention.
 * A part that ever disagrees cannot use these helpers unchanged, and this
 * comment is where whoever adds it should notice.
 *
 * Multi-byte reads need an explicit auto-increment bit on some parts and none
 * on others (the ST 6-axis parts increment when CTRL3_C's IF_INC is set,
 * which their init() already does). That difference is a datasheet fact per
 * driver, so it rides in the handle as spi_inc_mask rather than being decided
 * here.
 */
#define BUS_SPI_READ 0x80u

/*
 * Combined write-then-read in a single I2C transaction (no repeated-start
 * overhead). Saves ~40 µs vs separate transactions at 400 kHz.
 */
static inline int i2c_burst_read(const imud_bus_t *b, uint8_t reg,
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

static inline int i2c_reg_write(const imud_bus_t *b, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = b->i2c_addr, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(b->fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

/*
 * Command byte, then data, as two transfers inside ONE SPI_IOC_MESSAGE so
 * chip-select stays asserted across both (cs_change is 0). This mirrors the
 * I2C pair above, and it is why nothing here needs a bounce buffer: the FIFO
 * drains in icm20948.c and mpu925x.c read a runtime-computed length that a
 * fixed-size staging array would have to cap or chunk.
 */
static inline int spi_burst_read(const imud_bus_t *b, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    uint8_t cmd = (uint8_t)(reg | BUS_SPI_READ |
                            (len > 1 ? b->spi_inc_mask : 0u));
    struct spi_ioc_transfer tr[2] = {
        { .tx_buf = (uintptr_t)&cmd, .len = 1,
          .speed_hz = b->spi_hz, .bits_per_word = 8 },
        { .rx_buf = (uintptr_t)buf,  .len = len,
          .speed_hz = b->spi_hz, .bits_per_word = 8 },
    };
    return ioctl(b->fd, SPI_IOC_MESSAGE(2), tr) < 0 ? -1 : 0;
}

static inline int spi_reg_write(const imud_bus_t *b, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { (uint8_t)(reg & (uint8_t)~BUS_SPI_READ), val };
    struct spi_ioc_transfer tr = {
        .tx_buf = (uintptr_t)buf, .len = 2,
        .speed_hz = b->spi_hz, .bits_per_word = 8,
    };
    return ioctl(b->fd, SPI_IOC_MESSAGE(1), &tr) < 0 ? -1 : 0;
}

/* ── Transport dispatch ─────────────────────────────────────────────────── */

static inline int bus_burst_read(const imud_bus_t *b, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    return b->kind == BUS_SPI ? spi_burst_read(b, reg, buf, len)
                              : i2c_burst_read(b, reg, buf, len);
}

static inline int bus_reg_write(const imud_bus_t *b, uint8_t reg, uint8_t val)
{
    return b->kind == BUS_SPI ? spi_reg_write(b, reg, val)
                              : i2c_reg_write(b, reg, val);
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
