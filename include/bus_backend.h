/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_backend.h — the transport primitives a host has to supply.
 *
 * Everything above this line is portable: the twelve drivers name registers,
 * src/drivers/bus_io.h frames them, and src/bus.c applies the operator's
 * transport policy.  Everything below it is one host's answer.  The
 * implementation is src/bus_linux.c (i2c-dev + spidev ioctls) or
 * src/bus_null.c; the Makefile picks one, the same way it picks a GPIO
 * backend behind include/imu_gpio.h.
 *
 * Five entry points, and no kernel types in any of them — that is what makes
 * a second backend possible.  A FreeBSD one is iic(4)/spigen(4) behind the
 * same five; a macOS one is a USB bridge driven from userspace, where there
 * is no device node at all, which is why open() is in here rather than left
 * to src/bus.c.  The int a backend hands back is its own token: a descriptor
 * on the two that have device nodes, an index on one that does not.  Nothing
 * above dereferences it — imud_bus_t.fd is read only by src/bus.c and by the
 * helpers in src/drivers/bus_io.h, both of which pass it straight back.
 *
 * The transfer calls take the whole handle because the address, the SPI clock
 * and the framing all ride in it.  See include/bus.h.
 */
#ifndef IMUD_BUS_BACKEND_H
#define IMUD_BUS_BACKEND_H

#include <stdint.h>

#include "bus.h"

/*
 * One leg of a SPI message.  Every leg of a message clocks under a single
 * chip select, which is the property the framing in bus_io.h depends on: a
 * one-byte register read goes as ONE 16-bit leg precisely so it cannot be
 * split, and a burst read goes as two 8-bit legs because it cannot be made
 * atomic.  A backend that merges or reorders legs breaks that.
 *
 * tx NULL clocks zeros; rx NULL discards what comes back.
 */
typedef struct {
    const uint8_t *tx;
    uint8_t       *rx;
    uint32_t       len;
    uint8_t        bits;   /* bits per word: 8 or 16 */
} bus_spi_leg_t;

/* Maximum legs in one message.  Two is what the tree emits; the bound exists
 * so a backend can put the array on its stack. */
#define BUS_SPI_MAX_LEGS 4

/*
 * Open `node` and return a backend token, or -1 with errno set.  The token is
 * close-on-exec where the host has such a concept, per the tree-wide rule in
 * include/cloexec.h.
 */
int  bus_be_open(const char *node);

/* Release a token from bus_be_open.  Safe on -1. */
void bus_be_close(int h);

/*
 * Configure a SPI transport for `mode` (0..3) at `hz`, `bits` per word.
 * Returns 0, or -1 with errno set.  Never called for I2C.
 */
int  bus_be_spi_setup(int h, uint8_t mode, uint8_t bits, uint32_t hz);

/*
 * One I2C transaction to b->i2c_addr: write `txlen` bytes, then — when
 * `rxlen` is non-zero — a repeated start and `rxlen` bytes read back.  The
 * repeated start is what makes a register read one transaction rather than
 * two; see i2c_burst_read() in src/drivers/bus_io.h.  Returns 0, or -1.
 */
int  bus_be_i2c_xfer(const imud_bus_t *b, const uint8_t *tx, uint16_t txlen,
                     uint8_t *rx, uint16_t rxlen);

/*
 * One SPI message: `n` legs (1..BUS_SPI_MAX_LEGS) under a single chip select,
 * at b->spi_hz.  Returns 0, or -1.
 */
int  bus_be_spi_msg(const imud_bus_t *b, const bus_spi_leg_t *legs, unsigned n);

#endif /* IMUD_BUS_BACKEND_H */
