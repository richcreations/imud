/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * i2c_mock.h — an in-memory I2C register-map device for driver unit tests.
 *
 * The sensor drivers do all their I/O through ioctl(fd, I2C_RDWR, ...).  Built
 * with `-Wl,--wrap=ioctl` (GNU ld), this harness replaces that syscall with a
 * software model so a driver's probe/init/read can be exercised with no
 * hardware:
 *
 *   - A per-address 256-byte register file with the usual auto-increment
 *     semantics: a write message's first byte is the register pointer, the
 *     rest are stored; a read message returns successive registers from that
 *     pointer.  Covers WHO_AM_I, control-register writes, status, temperature,
 *     timestamp, and the magnetometer's burst output read.
 *   - An optional FIFO register (e.g. the ISM330DHCX's 0x78): reads from it pop
 *     from a byte queue instead of the register file, modelling a hardware FIFO
 *     that returns a fresh word on each read without advancing the register
 *     pointer.
 *   - Self-clearing bits (i2cmock_set_selfclear), so a driver's reset() can
 *     poll for its own reset bit to drop, the way real silicon behaves.
 *   - One-shot error injection to exercise the drivers' I2C-failure paths.
 *
 * A test typically: i2cmock_reset(); preload WHO_AM_I and any data registers;
 * call the driver op; then read back control registers with i2cmock_get_reg to
 * assert the config→register encoding.
 */
#ifndef IMUD_TEST_I2C_MOCK_H
#define IMUD_TEST_I2C_MOCK_H

#include <stdint.h>

/* Clear all register files, FIFOs, FIFO-register declarations, and any pending
 * error injection.  Call at the start of every test case. */
void i2cmock_reset(void);

/* Register-file access (addr is the 7-bit I2C address). */
void    i2cmock_set_reg(uint8_t addr, uint8_t reg, uint8_t val);
void    i2cmock_set_regs(uint8_t addr, uint8_t reg, const uint8_t *buf, int len);
uint8_t i2cmock_get_reg(uint8_t addr, uint8_t reg);

/* Declare `reg` on `addr` as a FIFO port: reads from it pop from the byte queue
 * pushed with i2cmock_fifo_push, rather than from the register file. */
void i2cmock_set_fifo_reg(uint8_t addr, uint8_t reg);
void i2cmock_fifo_push(uint8_t addr, const uint8_t *buf, int len);

/*
 * Declare the bits in `mask` at `reg` as self-clearing: a read returns them as
 * currently set, then drops them.  This models the reset and trigger bits that
 * hardware clears once the operation completes — without it, every driver
 * reset() that polls for its own bit to clear would time out against the plain
 * register file.  Bits accumulate; i2cmock_reset() forgets them.
 */
void i2cmock_set_selfclear(uint8_t addr, uint8_t reg, uint8_t mask);

/* Make the next wrapped ioctl() fail (-1, errno=EIO), then resume normally.
 * Drives the drivers' "I2C error" branches. */
void i2cmock_fail_next_ioctl(void);

/*
 * Sticky version: every ioctl fails until disabled, modelling a wedged bus.
 * Needed where the code under test has no per-iteration hook to re-arm the
 * one-shot through — imud-imutest's sustained-read check, for one.
 */
void i2cmock_fail_all(int enable);

#endif /* IMUD_TEST_I2C_MOCK_H */
