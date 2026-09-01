/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_linux.c — include/bus_backend.h over i2c-dev and spidev.
 *
 * The one file in the tree that includes the Linux kernel headers.  Keep it
 * that way: a
 * kernel type reaching src/bus.c, src/drivers/bus_io.h or a driver puts the
 * seam back where it was.
 *
 * Every transfer is a single ioctl, which is what the drivers were written
 * against and what the timing in src/drivers/ism330dhcx.c assumes.  Do not
 * reroute these through I2C_SLAVE, SMBus calls or read()/write().
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "bus_backend.h"

int bus_be_open(const char *node)
{
    /* O_CLOEXEC on the open itself, per the tree-wide close-on-exec rule —
     * there is no window here for a fork to inherit the descriptor. */
    return open(node, O_RDWR | O_CLOEXEC);
}

void bus_be_close(int h)
{
    if (h >= 0) close(h);
}

int bus_be_spi_setup(int h, uint8_t mode, uint8_t bits, uint32_t hz)
{
    if (ioctl(h, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(h, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(h, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0)
        return -1;
    return 0;
}

int bus_be_i2c_xfer(const imud_bus_t *b, const uint8_t *tx, uint16_t txlen,
                    uint8_t *rx, uint16_t rxlen)
{
    /* struct i2c_msg.buf is __u8 *, but the kernel only writes it on a
     * message flagged I2C_M_RD.  The write leg here never is. */
    struct i2c_msg msgs[2] = {
        { .addr = b->i2c_addr, .flags = 0,        .len = txlen,
          .buf = (uint8_t *)(uintptr_t)tx },
        { .addr = b->i2c_addr, .flags = I2C_M_RD, .len = rxlen, .buf = rx },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = rxlen ? 2u : 1u };
    return ioctl(b->fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

int bus_be_spi_msg(const imud_bus_t *b, const bus_spi_leg_t *legs, unsigned n)
{
    if (n < 1 || n > BUS_SPI_MAX_LEGS) { errno = EINVAL; return -1; }

    struct spi_ioc_transfer tr[BUS_SPI_MAX_LEGS];
    memset(tr, 0, sizeof tr);
    for (unsigned i = 0; i < n; i++) {
        tr[i].tx_buf        = (uintptr_t)legs[i].tx;
        tr[i].rx_buf        = (uintptr_t)legs[i].rx;
        tr[i].len           = legs[i].len;
        tr[i].speed_hz      = b->spi_hz;
        tr[i].bits_per_word = legs[i].bits;
    }

    /* SPI_IOC_MESSAGE(n) encodes the leg count in the request itself, so it
     * cannot be a runtime value.  Two is what the tree emits. */
    switch (n) {
    case 1: return ioctl(b->fd, SPI_IOC_MESSAGE(1), tr) < 0 ? -1 : 0;
    case 2: return ioctl(b->fd, SPI_IOC_MESSAGE(2), tr) < 0 ? -1 : 0;
    case 3: return ioctl(b->fd, SPI_IOC_MESSAGE(3), tr) < 0 ? -1 : 0;
    default: return ioctl(b->fd, SPI_IOC_MESSAGE(4), tr) < 0 ? -1 : 0;
    }
}
