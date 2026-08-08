/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus.c — opening a sensor transport.  See include/bus.h.
 *
 * This is the one place that turns an operator's bus_spec_t into a live
 * descriptor, shared by imud, imud-cal and imud-imutest so all three refuse
 * the same configurations for the same reasons.  The per-transfer register
 * helpers are elsewhere, in src/drivers/bus_io.h, because they have to stay
 * static inline inside each driver TU.
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "bus.h"
#include "log.h"

static int open_spi(imud_bus_t *b, const bus_spec_t *spec,
                    const bus_caps_t *caps, const char *who)
{
    if (!caps || !caps->spi_capable) {
        LOG_E("[%s] this driver has no SPI support — use bus = \"i2c\"\n", who);
        return -1;
    }

    /*
     * An unset spi_speed_hz means "as fast as the part allows", which is the
     * useful default: SPI exists here to cut transfer time. A request above
     * the datasheet maximum is clamped rather than refused — the same
     * requested-vs-actual shape the ODR resolution already uses, so an
     * optimistic config still runs and says what it really got.
     */
    uint32_t hz = spec->spi_hz ? spec->spi_hz : caps->spi_max_hz;
    if (caps->spi_max_hz && hz > caps->spi_max_hz) {
        LOG_I("[%s] SPI clock %u Hz requested, %u Hz is this part's maximum\n",
              who, hz, caps->spi_max_hz);
        hz = caps->spi_max_hz;
    }
    if (hz == 0) {
        LOG_E("[%s] no SPI clock: set spi_speed_hz, the driver declares no "
              "maximum\n", who);
        return -1;
    }

    int fd = open(spec->node, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        LOG_E("[%s] cannot open %s: %s\n", who, spec->node, strerror(errno));
        return -1;
    }

    uint8_t  mode = caps->spi_mode;
    uint8_t  bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
        LOG_E("[%s] cannot configure %s for SPI mode %u at %u Hz: %s\n",
              who, spec->node, mode, hz, strerror(errno));
        close(fd);
        return -1;
    }

    b->kind         = BUS_SPI;
    b->fd           = fd;
    b->spi_mode     = mode;
    b->spi_hz       = hz;
    b->spi_inc_mask = caps->spi_inc_mask;
    LOG_I("[%s] %s: SPI mode %u, %u Hz\n", who, spec->node, mode, hz);
    return 0;
}

static int open_i2c(imud_bus_t *b, const bus_spec_t *spec, const char *who)
{
    /* O_CLOEXEC on the open itself, per the tree-wide close-on-exec rule —
     * there is no window here for a fork to inherit the descriptor. */
    int fd = open(spec->node, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        LOG_E("[%s] cannot open %s: %s\n", who, spec->node, strerror(errno));
        return -1;
    }

    b->kind     = BUS_I2C;
    b->fd       = fd;
    b->i2c_addr = spec->i2c_addr;
    return 0;
}

int bus_open(imud_bus_t *b, const bus_spec_t *spec, const bus_caps_t *caps,
             const char *who)
{
    bus_init(b);

    switch (spec->kind) {
    case BUS_I2C: return open_i2c(b, spec, who);
    case BUS_SPI: return open_spi(b, spec, caps, who);
    }

    LOG_E("[%s] unsupported bus kind %d\n", who, (int)spec->kind);
    return -1;
}

void bus_close(imud_bus_t *b)
{
    if (!b) return;
    if (b->fd >= 0) close(b->fd);
    bus_init(b);
}
