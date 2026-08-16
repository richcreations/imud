/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_mock.h — an in-memory I2C register-map device for driver unit tests.
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

/* Clear all register files, FIFOs, FIFO-register declarations, SPI bindings,
 * and any pending error injection.  Call at the start of every test case. */
void i2cmock_reset(void);

/*
 * How the part's address pointer behaves during a multi-byte SPI transfer.
 *
 * Stated rather than inferred from the mask, because inferring it is what made
 * the mask untestable: a driver declaring spi_inc_mask = 0 and a mock reading
 * "mask 0 means it always increments" agree because the test copied the
 * driver's literal, not because the part behaves that way.  SPIMOCK_INC_NEVER
 * is the state the mock could not previously express, and it is the one that
 * shows what a wrong mask costs — the burst returns one register over and over
 * and the decode is silently wrong.
 */
typedef enum {
    SPIMOCK_INC_ALWAYS = 0,  /* part walks the address on its own */
    SPIMOCK_INC_ON_BIT,      /* walks only when `mask` is set in the command */
    SPIMOCK_INC_NEVER,       /* never walks: the same register every byte */
} spimock_inc_t;

/*
 * Point a spidev descriptor at one of the register files.
 *
 * SPI puts no address on the wire — the chip select does the addressing — so
 * a transfer on `fd` is serviced by the file `addr` names, and a driver
 * exercised on both transports sees one device rather than two mocks.
 * `mask` is the part's multi-byte auto-increment bit, stripped from the
 * command byte before the register is decoded.
 *
 * A transfer on an unbound descriptor fails with ENODEV rather than defaulting
 * to file 0, so a forgotten bind shows up as a test failure.
 */
void spimock_bind_inc(int fd, uint8_t addr, spimock_inc_t mode, uint8_t mask);

/* Convenience form: mask 0 binds ALWAYS, non-zero binds ON_BIT with that mask.
 * That is the historical behaviour, and fine where the increment is incidental
 * to what is being tested; reach for spimock_bind_inc when it is the point. */
void spimock_bind(int fd, uint8_t addr, uint8_t inc_mask);

/* Register-file access (addr is the 7-bit I2C address). */
void    i2cmock_set_reg(uint8_t addr, uint8_t reg, uint8_t val);
void    i2cmock_set_regs(uint8_t addr, uint8_t reg, const uint8_t *buf, int len);
uint8_t i2cmock_get_reg(uint8_t addr, uint8_t reg);

/* Declare `reg` on `addr` as a FIFO port: reads from it pop from the byte queue
 * pushed with i2cmock_fifo_push, rather than from the register file. */
void i2cmock_set_fifo_reg(uint8_t addr, uint8_t reg);

/*
 * The same, over a register window.  A real data port is usually several
 * registers wide — the ST 6-axis parts' FIFO_DATA_OUT spans 0x78-0x7E — and a
 * read of any of them pops.  Modelling only the first register would let a
 * blind register sweep walk through the rest of the window and look harmless,
 * which is exactly the bug this exists to catch.
 */
void i2cmock_set_fifo_range(uint8_t addr, uint8_t lo, uint8_t hi);

void i2cmock_fifo_push(uint8_t addr, const uint8_t *buf, int len);

/*
 * Declare the bits in `mask` at `reg` as self-clearing: a read returns them as
 * currently set, then drops them.  This models the reset and trigger bits that
 * hardware clears once the operation completes — without it, every driver
 * reset() that polls for its own bit to clear would time out against the plain
 * register file.  Bits accumulate; i2cmock_reset() forgets them.
 */
void i2cmock_set_selfclear(uint8_t addr, uint8_t reg, uint8_t mask);

/*
 * Declare `reg` on `addr` as live: every read returns the current value and
 * then advances it by `step`.  Models the registers that move with no write in
 * between — a timestamp counter, a FIFO level, a status word, sensor output —
 * which is what imud-imutest's volatile scan has to detect and exclude before
 * it can diff control registers across init().  `step` of 0 makes it static
 * again; i2cmock_reset() forgets it.
 */
void i2cmock_set_live(uint8_t addr, uint8_t reg, uint8_t step);

/*
 * Declare that a write to `reg` on `addr` also writes `also` with the same
 * byte.  The MMC5983MA does exactly this from CTRL0 into CTRL1 (measured on
 * hardware 2026-08-16, at 10 MHz, 1 MHz and 100 kHz alike), which turns
 * INT_en's bit 2 into CTRL1's X-inhibit and stops the X axis measuring.
 *
 * It exists because the ordering fix is otherwise untestable: a driver that
 * writes CTRL1 first and one that writes it last leave the same final bytes in
 * a mock that models one write as one register.  i2cmock_reset() forgets it.
 */
void i2cmock_set_write_alias(uint8_t addr, uint8_t reg, uint8_t also);

/*
 * How many times `reg` on `addr` has been read since i2cmock_reset().
 *
 * For asserting that code does NOT touch a register — a write-only control
 * register, a reserved address — which no readback can show, because the
 * damage of reading one is the read itself.  Reads inside a FIFO window are
 * not counted: they pop the queue rather than address a register.
 */
uint32_t i2cmock_read_count(uint8_t addr, uint8_t reg);

/*
 * The order registers were written in, since i2cmock_reset().
 *
 * Final register values cannot express order — a driver that starts a sensor
 * before configuring it and one that configures it first leave identical bytes
 * behind. Where the part cares, that difference is the whole bug: the
 * MMC5983MA must have CTRL1's bandwidth in place before CTRL2 enables
 * continuous mode (Rev A p.15 makes BW an input to what CM_Freq means), and
 * must have nothing written at all in the ~45 ms after it (measured on
 * hardware 2026-08-16 — a write inside that window leaves the bridge
 * saturated).
 *
 * i2cmock_write_at returns the register written at 0-based position `n`, or -1
 * past the end or past the recorded prefix. i2cmock_last_write is tracked
 * separately so it stays correct however many writes ran. Both count only the
 * addressed register: a write alias is a side effect of one write, not two.
 */
uint32_t i2cmock_writes(uint8_t addr);
int      i2cmock_write_at(uint8_t addr, uint32_t n);
int      i2cmock_last_write(uint8_t addr);

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
