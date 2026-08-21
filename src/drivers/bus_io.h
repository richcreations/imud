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
#include <string.h>
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
 * One part already disagrees, and it is worth knowing HOW it got away with it.
 * The MMC5983MA (MEMSIC, Rev A p.5) spells the command byte as RW, then one
 * don't-care bit, then a SIX-bit address — so the address sits in bits 5:0,
 * not 6:0. Every register imud touches on that part is under 0x40, which puts
 * bit 6 at zero either way, so the helpers happen to frame it correctly and
 * the difference has never bitten. A part with a register at 0x40 or above
 * would not be so lucky.
 *
 * What did bite was the other half of the same assumption. These helpers were
 * written to the ST convention and the MMC5983MA was given ST's SPI MODE to
 * match; it is a mode-0 part, and mode 3 corrupts its writes while leaving
 * most reads intact. That produced a list of invented silicon quirks before
 * anyone questioned the mode — see the block above odr_encode() in
 * mmc5983ma.c. Check a new part's framing AND its mode against its own
 * datasheet, not against the part next to it on the bus.
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
 * READS, and the two things that make a split one harmless.
 *
 * RP1 deasserts chip-select between words (raspberrypi/linux#6354; see
 * spi_reg_write below for the full account), so a multi-word transfer can be
 * cut anywhere.  For a read that matters in a way that is easy to miss: the
 * command byte is followed by a DATA PHASE, and whatever the master clocks out
 * during that phase is what the part sees as the next command if the transfer
 * is cut.
 *
 * 1. A ONE-BYTE read is command + one data byte = exactly 16 bits, so it goes
 *    as a single word and cannot be split at all.  That covers every
 *    bus_reg_read() in the tree.
 *
 * 2. A MULTI-byte read cannot be made atomic -- a 7-byte ST FIFO word is four
 *    16-bit words, with three boundaries -- so instead it clocks out a byte
 *    that is SAFE to be misread as a command.  spidev sends zeros when no
 *    tx_buf is given, and 0x00 has bit 7 clear, which on these parts means
 *    WRITE: a split read was issuing writes into register 0x00.  Clocking
 *    SPI_IDLE_CMD instead makes a split read a READ of WHO_AM_I, which changes
 *    nothing anywhere.
 *
 * The master's output is ignored by the part during a normal data phase, so
 * this is free: it only matters in the case that used to corrupt state.
 *
 * Doing multi-byte reads as repeated single reads was considered and rejected:
 * a 64-set watermark at 833 Hz is 128 FIFO words, which becomes 896 syscalls
 * per drain instead of 128 -- about 18 ms against a 10 ms drain cadence.  The
 * complete fix for the remaining boundaries is a GPIO chip select, which is a
 * device-tree change on the host rather than anything this file can do.
 */
static inline int spi_burst_read(const imud_bus_t *b, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    uint8_t cmd = (uint8_t)(reg | BUS_SPI_READ |
                            (len > 1 ? b->spi_inc_mask : 0u));

    if (len == 1) {
        /* Command and data in one 16-bit word: no boundary to split. */
        uint16_t tx = (uint16_t)((uint16_t)cmd << 8), rx = 0;
        struct spi_ioc_transfer tr = {
            .tx_buf = (uintptr_t)&tx, .rx_buf = (uintptr_t)&rx, .len = 2,
            .speed_hz = b->spi_hz, .bits_per_word = 16,
        };
        if (ioctl(b->fd, SPI_IOC_MESSAGE(1), &tr) < 0) return -1;
        buf[0] = (uint8_t)(rx & 0xFFu);
        return 0;
    }

    /*
     * Multi-byte: two transfers, data phase left as spidev's zero fill.
     *
     * Clocking a read opcode (0x8F) here instead was tried, on the reasoning
     * that a split would then be misread as a harmless READ rather than a
     * write into register 0x00.  It backfired and is recorded so it is not
     * tried again: a split read that becomes a READ COMMAND makes the part
     * start returning that register's contents, so the corruption moves out
     * of the part's state and into the SAMPLE STREAM.  Measured on the
     * reference rig at 13.016 Hz with a verified-clean bus (0 of 2000 bad
     * register reads): gravity read 18.2 m/s^2, the measured rate 5.6 Hz
     * against 13.016, and the FIFO depth probe saw nothing.  Zero fill
     * corrupts state, which imu.bus.integrity can see and a bank clear can
     * undo; an opcode fill corrupts data, which nothing downstream can.
     *
     * Neither is a fix.  The only complete one for the remaining boundaries is
     * a GPIO chip select on the host -- raspberrypi/linux#6354.
     */
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
    uint16_t word = (uint16_t)(((uint16_t)(reg & (uint8_t)~BUS_SPI_READ) << 8)
                               | val);
    struct spi_ioc_transfer tr = {
        .tx_buf = (uintptr_t)&word, .len = 2,
        .speed_hz = b->spi_hz, .bits_per_word = 16,
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

/*
 * Signed 24-bit big-endian, the width the RM3100's measurement registers use.
 *
 * The obvious spellings are both wrong by this project's standard. A signed
 * right shift of a negative value is implementation-defined, and so is
 * converting an out-of-range unsigned (0xFF800000u) to int32_t. Biasing
 * instead keeps every step defined: (u ^ 0x800000) is in [0, 0xFFFFFF], which
 * int32_t can represent, so the conversion is value-preserving, and the
 * subtraction lands in [-0x800000, 0x7FFFFF] without overflow.
 */
static inline int32_t reg_s24be(const uint8_t *p)   /* high byte first */
{
    uint32_t u = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
    return (int32_t)(u ^ 0x800000u) - 0x800000;
}

#endif /* IMUD_DRIVERS_BUS_IO_H */
