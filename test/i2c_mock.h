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

/* Make the next wrapped ioctl() fail (-1, errno=EIO), then resume normally.
 * Drives the drivers' "I2C error" branches. */
void i2cmock_fail_next_ioctl(void);

#endif /* IMUD_TEST_I2C_MOCK_H */
