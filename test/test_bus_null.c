/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_bus_null.c — the no-i2c-dev bus backend (src/bus_null.c).
 *
 * Links that one file and nothing else: it is what imud carries on a host
 * with neither i2c-dev nor spidev, so a suite that needed a driver to link
 * would not be testing the case.  This runs on every platform.
 *
 * What is pinned here is the contract src/bus.c and the drivers depend on,
 * not the stubs themselves.  The split is the whole point of the file:
 * open() and close() are POSIX and must keep working, because that is what
 * lets a `driver = sim` build run the whole pipeline on a host with no bus;
 * the transfers must fail, and fail with ENOSYS specifically, so a real
 * driver's probe says the build has no backend rather than blaming the
 * wiring.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "bus_backend.h"

static int g_fail;
static int g_checks;

#define EXPECT(c, msg) do {                                       \
        g_checks++;                                               \
        if (!(c)) { printf("  FAIL: %s\n", (msg)); g_fail++; }     \
    } while (0)

/*
 * open() and close() stay real.  A backend that refused them would take the
 * sim driver down with it — src/imu.c calls bus_open() unconditionally, for
 * every driver — and buy nothing, since a host without i2c-dev still has
 * open().
 */
static void test_open_and_close_are_real(void)
{
    printf("test_open_and_close_are_real\n");

    int h = bus_be_open("/dev/null");
    EXPECT(h >= 0, "open of a real node succeeds");
    bus_be_close(h);

    errno = 0;
    EXPECT(bus_be_open("/nonexistent/imud-bus-null") < 0,
           "open of a missing node fails");
    EXPECT(errno != 0, "leaving errno set for the caller's message");

    /* bus_close() reaches here with a handle that was never opened, because
     * bus_init() starts one at -1 and every error path is safe to close. */
    bus_be_close(-1);
    EXPECT(1, "close(-1) returns");
}

static void test_transfers_fail_with_enosys(void)
{
    printf("test_transfers_fail_with_enosys\n");

    imud_bus_t b;
    bus_init(&b);
    b.i2c_addr = 0x6B;
    b.spi_hz   = 1000000;

    uint8_t tx[2] = { 0x0F, 0x00 }, rx[8] = { 0 };

    errno = 0;
    EXPECT(bus_be_i2c_xfer(&b, tx, 1, rx, sizeof rx) == -1, "i2c read fails");
    EXPECT(errno == ENOSYS, "with ENOSYS");

    errno = 0;
    EXPECT(bus_be_i2c_xfer(&b, tx, 2, NULL, 0) == -1, "i2c write fails");
    EXPECT(errno == ENOSYS, "with ENOSYS");

    bus_spi_leg_t legs[2] = {
        { .tx = tx, .len = 1,        .bits = 8 },
        { .rx = rx, .len = sizeof rx, .bits = 8 },
    };

    errno = 0;
    EXPECT(bus_be_spi_msg(&b, legs, 2) == -1, "spi burst read fails");
    EXPECT(errno == ENOSYS, "with ENOSYS");

    errno = 0;
    EXPECT(bus_be_spi_msg(&b, legs, 1) == -1, "spi single-leg message fails");
    EXPECT(errno == ENOSYS, "with ENOSYS");

    errno = 0;
    EXPECT(bus_be_spi_setup(0, 3, 8, 1000000) == -1, "spi setup fails");
    EXPECT(errno == ENOSYS, "with ENOSYS");
}

/*
 * ENOSYS rather than EIO or ENODEV is the load-bearing part, and it is the
 * same distinction src/imu_gpio_null.c draws: those two say a device was
 * looked for and not found, which an operator would reasonably chase into the
 * wiring.  This build cannot look at all.
 */
static void test_no_transfer_reports_a_bus_error(void)
{
    printf("test_no_transfer_reports_a_bus_error\n");

    imud_bus_t b;
    bus_init(&b);
    uint8_t reg = 0x0F, val = 0;

    errno = 0;
    (void)bus_be_i2c_xfer(&b, &reg, 1, &val, 1);
    EXPECT(errno != EIO,    "not EIO — that would read as a wiring fault");
    EXPECT(errno != ENODEV, "not ENODEV — that would read as a missing part");
    EXPECT(errno == ENOSYS, "ENOSYS names the missing backend");
}

/* The buffers must come back untouched: a driver that reads a register into a
 * stack byte and ignores the return would otherwise decode whatever the stub
 * happened to leave there. */
static void test_failed_transfer_writes_nothing(void)
{
    printf("test_failed_transfer_writes_nothing\n");

    imud_bus_t b;
    bus_init(&b);

    uint8_t reg = 0x0F;
    uint8_t rx[4] = { 0xAA, 0xBB, 0xCC, 0xDD };
    (void)bus_be_i2c_xfer(&b, &reg, 1, rx, sizeof rx);
    EXPECT(rx[0] == 0xAA && rx[1] == 0xBB && rx[2] == 0xCC && rx[3] == 0xDD,
           "i2c read leaves the caller's buffer alone");

    uint8_t srx[2] = { 0x11, 0x22 };
    bus_spi_leg_t legs[2] = {
        { .tx = &reg, .len = 1,         .bits = 8 },
        { .rx = srx,  .len = sizeof srx, .bits = 8 },
    };
    (void)bus_be_spi_msg(&b, legs, 2);
    EXPECT(srx[0] == 0x11 && srx[1] == 0x22,
           "spi read leaves the caller's buffer alone");
}

int main(void)
{
    printf("=== test_bus_null ===\n");

    test_open_and_close_are_real();
    test_transfers_fail_with_enosys();
    test_no_transfer_reports_a_bus_error();
    test_failed_transfer_writes_nothing();

    printf("\n%d passed, %d failed\n", g_checks - g_fail, g_fail);
    return g_fail ? 1 : 0;
}
