/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus.h — the sensor transport handle (§4)
 *
 * Drivers neither open nor address a bus: they are handed an imud_bus_t and
 * issue their register reads and writes through src/drivers/bus_io.h.
 * Everything that distinguishes one transport from another — the file
 * descriptor, the I2C slave address — lives in the handle, so a driver's
 * register logic is written once and does not care how the bytes travel.
 *
 * bus_spec_t is what the operator asked for, straight out of the config;
 * bus_open() turns it into a live imud_bus_t.  The two are separate because
 * the config is parsed long before any device is opened, and by three
 * different programs: imud, imud-cal and imud-imutest each build a spec and
 * open it themselves.
 *
 * The IMU and the magnetometer get one handle each, never a shared one.  On
 * I2C that means the same node is opened twice, which is harmless — an
 * I2C_RDWR message carries its own address — and it is what lets the two
 * sensors sit on different buses.
 */
#ifndef IMUD_BUS_H
#define IMUD_BUS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Deliberately no kernel headers here, and none in src/bus.c or
 * src/drivers/bus_io.h either: every one of them lives behind
 * include/bus_backend.h, in src/bus_linux.c.  That is what lets a host with
 * no i2c-dev take src/bus_null.c instead and still build.
 */

typedef enum {
    BUS_I2C = 0,
    BUS_SPI = 1,
} bus_kind_t;

/* A live bus.  fd < 0 means "not open". */
typedef struct {
    bus_kind_t kind;
    /* The backend's token, from bus_be_open() — a file descriptor on the
     * hosts that have device nodes, not necessarily on one that does not.
     * Only src/bus.c and src/drivers/bus_io.h read it, and both hand it
     * straight back to the backend. */
    int        fd;
    uint8_t    i2c_addr;     /* BUS_I2C: the 7-bit slave address */
    /* BUS_SPI: resolved at open from the driver's limits and the operator's
     * request.  spi_inc_mask is OR'd into the command byte on multi-byte
     * reads for the parts that need an explicit auto-increment bit; 0 for the
     * parts that increment on their own. */
    uint8_t    spi_mode;
    uint8_t    spi_inc_mask;
    uint32_t   spi_hz;
} imud_bus_t;

/* What the operator asked for, before any device is touched. */
typedef struct {
    bus_kind_t  kind;
    const char *node;      /* device node — [device] i2c_bus, or [imu]/[mag] spi_dev */
    uint8_t     i2c_addr;  /* BUS_I2C — [imu]/[mag] i2c_addr */
    uint32_t    spi_hz;    /* BUS_SPI — 0 means "the part's datasheet maximum" */
} bus_spec_t;

/*
 * What the driver's silicon can do.  These are datasheet facts, not operator
 * policy, so they come from the ops struct rather than the config: the
 * operator chooses the transport and may ask for a slower clock, but cannot
 * invent SPI on a part without a SPI port or outrun its timing.
 */
typedef struct {
    bool     spi_capable;
    uint8_t  spi_mode;      /* SPI mode 0..3 */
    uint32_t spi_max_hz;    /* datasheet maximum; 0 = unspecified */
    uint8_t  spi_inc_mask;  /* multi-byte auto-increment bits; 0 = automatic */
} bus_caps_t;

/*
 * Put a handle in the closed state.  Call this before any error path that can
 * reach bus_close(): a handle inside a calloc'd context starts at fd 0, and
 * closing that would take stdin down with it.
 */
static inline void bus_init(imud_bus_t *b)
{
    b->kind         = BUS_I2C;
    b->fd           = -1;
    b->i2c_addr     = 0;
    b->spi_mode     = 0;
    b->spi_inc_mask = 0;
    b->spi_hz       = 0;
}

/*
 * Open `spec` into `b` for a driver with `caps`.  `who` is "imu" or "mag" and
 * appears only in log messages, so a failure says which sensor could not be
 * reached.  A NULL `caps` means "no SPI", which is what an I2C-only caller
 * wants.  Returns 0, or -1 with the reason already logged.  On failure the
 * handle is left closed, so bus_close() is safe either way.
 */
int  bus_open(imud_bus_t *b, const bus_spec_t *spec, const bus_caps_t *caps,
              const char *who);

/* Close and mark closed.  Safe on an unopened or already-closed handle. */
void bus_close(imud_bus_t *b);

/*
 * True when two spidev node paths name the same SPI controller (the B in
 * /dev/spidevB.C).  Devices that share a controller share its clock, so their
 * drivers must agree about the SPI mode; imu.c refuses the combination rather
 * than letting one part corrupt the other's transfers.
 */
bool bus_spi_same_controller(const char *a, const char *b);

#endif /* IMUD_BUS_H */
