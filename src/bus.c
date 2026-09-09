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
 *
 * The policy here is host-independent — which transports a part offers, what
 * clock it can take — so the host's answer is reached through
 * include/bus_backend.h rather than written inline.
 */

#include <errno.h>
#include <string.h>

#include "bus.h"
#include "bus_backend.h"
#include "log.h"

/*
 * The backends this build carries, innermost first.  The Makefile writes the
 * list from the same decision that picks $(BUS_SRC), so a build cannot name a
 * backend it did not compile, and adding one is a Makefile line rather than an
 * edit here.  The default is the answer for a host with no backend at all.
 */
#ifndef IMUD_BUS_BACKENDS
#define IMUD_BUS_BACKENDS &bus_null_backend
#endif

static const bus_backend_t *const g_backends[] = { IMUD_BUS_BACKENDS };

#define N_BACKENDS ((unsigned)(sizeof g_backends / sizeof g_backends[0]))

/*
 * Which backend serves this node.  A backend with a scheme takes the nodes
 * that start with it ("ftdi:"); the one without takes everything else, which
 * is how a plain /dev path keeps reaching the host's own bus.  Returns NULL
 * when a scheme was written that no backend in this build answers to — an
 * `ftdi:` node in a build without the FT232H backend, say, which is worth a
 * distinct message rather than a confusing open() failure.
 */
static const bus_backend_t *pick_backend(const char *node)
{
    const bus_backend_t *fallback = NULL;

    for (unsigned i = 0; i < N_BACKENDS; i++) {
        const char *s = g_backends[i]->scheme;
        if (!s) {
            if (!fallback) fallback = g_backends[i];
            continue;
        }
        if (strncmp(node, s, strlen(s)) == 0) return g_backends[i];
    }

    /*
     * A node that names some other scheme must not fall through to the
     * device-node backend, which would try to open() a path that was never
     * one.  "scheme" here means a prefix up to a colon that is not a path.
     */
    const char *colon = strchr(node, ':');
    if (colon && node[0] != '/') return NULL;

    return fallback;
}

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

    int fd = b->be->open(spec->node);
    if (fd < 0) {
        LOG_E("[%s] cannot open %s: %s\n", who, spec->node, strerror(errno));
        return -1;
    }

    uint8_t mode = caps->spi_mode;
    if (b->be->spi_setup(fd, mode, 8, hz) < 0) {
        LOG_E("[%s] cannot configure %s for SPI mode %u at %u Hz: %s\n",
              who, spec->node, mode, hz, strerror(errno));
        b->be->close(fd);
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
    int fd = b->be->open(spec->node);
    if (fd < 0) {
        LOG_E("[%s] cannot open %s: %s\n", who, spec->node, strerror(errno));
        return -1;
    }

    b->kind     = BUS_I2C;
    b->fd       = fd;
    b->i2c_addr = spec->i2c_addr;
    return 0;
}

/*
 * Do these two spidev nodes hang off one controller?  /dev/spidevB.C -- B is
 * the bus, C the chip select, so the bus number decides.
 *
 * Compared as text up to the last dot rather than parsed: anything not of that
 * shape is not a spidev node, and answering "same controller" about it would
 * be a guess. Two devices on one controller must agree about the clock mode,
 * because the controller has a single SCLK and mode 0 idles it low while mode
 * 3 idles it high -- see the check in imu.c that uses this.
 */
bool bus_spi_same_controller(const char *a, const char *b)
{
    if (!a || !b) return false;
    const char *da = strrchr(a, '.'), *db = strrchr(b, '.');
    if (!da || !db) return false;
    size_t la = (size_t)(da - a), lb = (size_t)(db - b);
    return la == lb && strncmp(a, b, la) == 0;
}

int bus_open(imud_bus_t *b, const bus_spec_t *spec, const bus_caps_t *caps,
             const char *who)
{
    bus_init(b);

    if (!spec->node || spec->node[0] == '\0') {
        LOG_E("[%s] no device node configured\n", who);
        return -1;
    }

    b->be = pick_backend(spec->node);
    if (!b->be) {
        LOG_E("[%s] no bus backend in this build handles %s\n",
              who, spec->node);
        return -1;
    }

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
    if (b->be) b->be->close(b->fd);
    bus_init(b);
}
