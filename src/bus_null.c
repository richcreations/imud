/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_null.c — include/bus_backend.h with no bus behind it.
 *
 * Selected in place of src/bus_linux.c on a host with no i2c-dev and no
 * spidev, which is what lets imud build and link on a BSD or a Mac before
 * either has a backend of its own.
 *
 * open() and close() are real, because they are POSIX rather than Linux and
 * refusing them would buy nothing: what such a host lacks is the ioctls.  So
 * a `driver = sim` build runs the whole pipeline here — sim touches the bus
 * on no code path (src/drivers/sim.c) — and that is the smoke test a port
 * starts from.  Every real driver instead fails at its first probe read, with
 * ENOSYS naming the missing piece rather than a bus error blaming the wiring.
 *
 * ENOSYS rather than ENODEV or ENOTSUP, matching src/imu_gpio_null.c: those
 * say a device was looked for and not found, which a caller may reasonably
 * report against the hardware.  This is the build having no way to look.
 */

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "bus_backend.h"

int bus_be_open(const char *node)
{
    return open(node, O_RDWR | O_CLOEXEC);
}

void bus_be_close(int h)
{
    if (h >= 0) close(h);
}

int bus_be_spi_setup(int h, uint8_t mode, uint8_t bits, uint32_t hz)
{
    (void)h; (void)mode; (void)bits; (void)hz;
    errno = ENOSYS;
    return -1;
}

int bus_be_i2c_xfer(const imud_bus_t *b, const uint8_t *tx, uint16_t txlen,
                    uint8_t *rx, uint16_t rxlen)
{
    (void)b; (void)tx; (void)txlen; (void)rx; (void)rxlen;
    errno = ENOSYS;
    return -1;
}

int bus_be_spi_msg(const imud_bus_t *b, const bus_spi_leg_t *legs, unsigned n)
{
    (void)b; (void)legs; (void)n;
    errno = ENOSYS;
    return -1;
}
