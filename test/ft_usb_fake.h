/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ft_usb_fake.h — a scripted FT232H behind include/ft_usb.h.
 *
 * The second implementation of that seam, beside src/ft_usb_linux.c, and what
 * lets src/bus_ft232h.c be tested with no dongle on the machine.
 *
 * It does not record bytes and compare them against a golden stream, because
 * that would pin the encoding rather than the behaviour: any equivalent MPSSE
 * spelling would fail it, and a stream that encodes the WRONG I2C would pass
 * as long as it did not change.  Instead it INTERPRETS the command stream as
 * a bus would — tracking SCL and SDA through every SET_BITS_LOW, raising a
 * start when SDA falls with SCL high and a stop when it rises — so what the
 * suite asserts is the I2C that reached the wire.
 */
#ifndef IMUD_TEST_FT_USB_FAKE_H
#define IMUD_TEST_FT_USB_FAKE_H

#include <stdint.h>

/* Forget every device, script and observation.  Call between cases. */
void ftfake_reset(void);

/* An address that answers.  Anything else NAKs, as an empty bus would. */
void ftfake_add_device(uint8_t addr);

/* What the next reads clock back, consumed in order.  Exhausted returns 0xFF. */
void ftfake_set_read_data(const uint8_t *d, unsigned n);

/* Make the n-th byte written in a transaction NAK (1 = the address).  A
 * transaction that never reaches it is unaffected.  -1 disarms. */
void ftfake_nak_nth_byte(int n);

/* Make ft_usb_open() fail, for the no-such-dongle path. */
void ftfake_fail_open(int fail);

/* ── what the bus saw ────────────────────────────────────────────────────── */

unsigned ftfake_starts(void);        /* start conditions, repeated included */
unsigned ftfake_stops(void);
unsigned ftfake_n_addr(void);        /* address bytes, as written incl. R/W */
uint8_t  ftfake_addr(unsigned i);
unsigned ftfake_n_wrote(void);       /* payload bytes written after an address */
uint8_t  ftfake_wrote(unsigned i);
unsigned ftfake_n_read(void);        /* bytes clocked in */

/* Did the master NAK the last byte it read?  An I2C master must, to tell the
 * slave to let go of SDA before the stop. */
int ftfake_last_read_nakked(void);

/* The MPSSE configuration the backend applied, for the init assertions. */
int      ftfake_three_phase(void);   /* 0x8D seen */
int      ftfake_drive_zero(void);    /* 0x9E seen, and its low-byte mask */
uint16_t ftfake_divisor(void);       /* the 0x86 operand */

/* The device the last ft_usb_open() was asked for. */
const char *ftfake_want(void);

#endif /* IMUD_TEST_FT_USB_FAKE_H */
