/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ft_usb.h — the USB transport an FT232H bridge sits on.
 *
 * The fourth portability seam, beside include/{bus_backend,imu_gpio,host_time}.h
 * and for the same reason: src/bus_ft232h.c is the FTDI part — MPSSE opcodes,
 * I2C bit sequences, clock divisors — and none of that is host-specific.  What
 * is host-specific is how a process reaches a USB endpoint at all, which is
 * these six calls.
 *
 * src/ft_usb_linux.c is usbfs, src/ft_usb_darwin.c is IOKit and
 * src/ft_usb_freebsd.c is libusb20 — each its host's own USB layer, none of
 * them a dependency the tree does not already assume.  A further host writes
 * one file against this header and names it in the Makefile as FT_USB_SRC,
 * with nothing else in the build to edit.
 *
 * Everything here is synchronous and blocking.  src/bus_ft232h.c serialises
 * its own access, so no call below has to be reentrant on one handle.
 */
#ifndef IMUD_FT_USB_H
#define IMUD_FT_USB_H

#include <stdint.h>

typedef struct ft_usb ft_usb_t;

/*
 * Open the first device matching `vid`/`pid` that also matches `want`:
 *
 *   ""          the first one found
 *   "<serial>"  by USB serial string, for a board whose EEPROM carries one
 *   "<bus>.<dev>"  by USB topology, e.g. "1.6", for a board that does not
 *
 * The kernel driver on the interface is detached — an FT232H arrives claimed
 * by a serial driver, and MPSSE is unreachable until it lets go — and restored
 * by ft_usb_close().  Returns NULL with errno set.
 *
 * `ifno` is the FTDI interface, 0 for the FT232H's only one.
 */
ft_usb_t *ft_usb_open(const char *want, uint16_t vid, uint16_t pid,
                      unsigned ifno);

/* Release the interface, hand it back to the kernel driver, and free. */
void ft_usb_close(ft_usb_t *u);

/*
 * One vendor control transfer to the interface, host to device, with no data
 * stage — every FTDI command imud issues is of that shape.  Returns 0, or -1
 * with errno set.
 */
int ft_usb_control(ft_usb_t *u, uint8_t request, uint16_t value);

/*
 * One bulk transfer.  Returns the byte count moved, or -1 with errno set; a
 * read that times out with nothing to report returns 0 rather than failing,
 * because an FT232H with no data queued is the normal case.
 *
 * A read is one USB packet at most: an FT232H prefixes EVERY packet with two
 * modem-status bytes, so a caller that asked for more than a packet would have
 * to find and strip each header itself.  ft_usb_packet_size() is the bound.
 */
int ft_usb_write(ft_usb_t *u, const uint8_t *buf, unsigned len,
                 unsigned timeout_ms);
int ft_usb_read(ft_usb_t *u, uint8_t *buf, unsigned len, unsigned timeout_ms);

/* The IN endpoint's wMaxPacketSize: 512 on a high-speed FT232H, 64 if it ever
 * enumerates at full speed. */
unsigned ft_usb_packet_size(const ft_usb_t *u);

/* How the device was reached, for a log line.  Never NULL. */
const char *ft_usb_name(const ft_usb_t *u);

#endif /* IMUD_FT_USB_H */
