/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_bus_ft232h.c — the FT232H bus backend (src/bus_ft232h.c).
 *
 * What is pinned here is the I2C, not the MPSSE: test/ft_usb_fake.c interprets
 * the command stream as a bus does, so these assertions are about start
 * conditions, addresses, the repeated start before a read and the NAK that
 * ends one — the things a driver's register access depends on and that no
 * readback would reveal on its own.  A register read that skipped its repeated
 * start would still return the right byte from a forgiving slave.
 *
 * Runs everywhere: no dongle, no usbfs, no kernel headers.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "bus.h"
#include "bus_backend.h"
#include "drivers/bus_io.h"
#include "ft_usb_fake.h"

#define IMU_ADDR 0x6A
#define MAG_ADDR 0x1C

static int g_fail;
static int g_checks;

#define EXPECT(c, msg) do {                                       \
        g_checks++;                                               \
        if (!(c)) { printf("  FAIL: %s\n", (msg)); g_fail++; }     \
    } while (0)

#define EXPECT_EQ(got, want, msg) do {                            \
        g_checks++;                                               \
        long g_ = (long)(got), w_ = (long)(want);                 \
        if (g_ != w_) {                                           \
            printf("  FAIL: %s (got %ld, want %ld)\n", (msg), g_, w_); \
            g_fail++;                                             \
        }                                                         \
    } while (0)

/* Open one handle on the fake, with `addr` answering. */
static int open_bus(imud_bus_t *b, const char *node, uint8_t addr)
{
    ftfake_reset();
    ftfake_add_device(addr);
    bus_spec_t spec = { .kind = BUS_I2C, .node = node, .i2c_addr = addr };
    return bus_open(b, &spec, NULL, "imu");
}

/*
 * MPSSE has to be put into three-phase clocking and open-drain before it can
 * do I2C at all: two-phase samples SDA on the wrong edge, and a push-pull SDA
 * fights the slave's ACK rather than reading it.
 */
static void test_init_configures_for_i2c(void)
{
    printf("test_init_configures_for_i2c\n");

    imud_bus_t b;
    EXPECT(open_bus(&b, "ftdi:", IMU_ADDR) == 0, "opens the bridge");
    EXPECT(ftfake_three_phase(), "three-phase clocking enabled");
    EXPECT_EQ(ftfake_drive_zero(), 0x07, "SCL, SDA out and SDA in open-drain");
    /* 400 kHz default: div = 20e6/hz - 1. */
    EXPECT_EQ(ftfake_divisor(), 49, "divisor for the 400 kHz default");
    bus_close(&b);
}

static void test_register_write_is_one_transaction(void)
{
    printf("test_register_write_is_one_transaction\n");

    imud_bus_t b;
    if (open_bus(&b, "ftdi:", IMU_ADDR) < 0) { EXPECT(0, "open"); return; }

    unsigned s0 = ftfake_starts(), p0 = ftfake_stops();
    EXPECT_EQ(bus_reg_write(&b, 0x10, 0x4C), 0, "the write succeeds");

    EXPECT_EQ(ftfake_starts() - s0, 1, "one start condition");
    EXPECT_EQ(ftfake_stops()  - p0, 1, "one stop condition");
    EXPECT_EQ(ftfake_n_addr(), 1, "one address byte");
    EXPECT_EQ(ftfake_addr(0), IMU_ADDR << 1, "addressed for writing");
    EXPECT_EQ(ftfake_n_wrote(), 2, "register and value");
    EXPECT_EQ(ftfake_wrote(0), 0x10, "the register");
    EXPECT_EQ(ftfake_wrote(1), 0x4C, "the value");
    bus_close(&b);
}

/*
 * The repeated start is the whole point of the read framing: without it the
 * write and the read are two transactions, and anything else on the bus may
 * interleave between them.
 */
static void test_register_read_uses_a_repeated_start(void)
{
    printf("test_register_read_uses_a_repeated_start\n");

    imud_bus_t b;
    if (open_bus(&b, "ftdi:", IMU_ADDR) < 0) { EXPECT(0, "open"); return; }

    const uint8_t who[] = { 0x6C };
    ftfake_set_read_data(who, sizeof who);

    unsigned s0 = ftfake_starts(), p0 = ftfake_stops();
    uint8_t v = 0;
    EXPECT_EQ(bus_reg_read(&b, 0x0F, &v), 0, "the read succeeds");
    EXPECT_EQ(v, 0x6C, "the byte the part drove");

    EXPECT_EQ(ftfake_starts() - s0, 2, "a start and a repeated start");
    EXPECT_EQ(ftfake_stops()  - p0, 1, "one stop, at the end");
    EXPECT_EQ(ftfake_n_addr(), 2, "addressed twice");
    EXPECT_EQ(ftfake_addr(0), IMU_ADDR << 1, "write phase");
    EXPECT_EQ(ftfake_addr(1), (IMU_ADDR << 1) | 1, "read phase");
    EXPECT_EQ(ftfake_n_wrote(), 1, "only the register goes out");
    EXPECT_EQ(ftfake_wrote(0), 0x0F, "the register");
    EXPECT(ftfake_last_read_nakked(), "the master NAKs the final byte read");
    bus_close(&b);
}

/*
 * A burst is what the FIFO drain issues, and it is where an off-by-one in the
 * ACK/NAK decision shows: every byte but the last must be ACKed, or the slave
 * stops driving after the first.
 */
static void test_burst_read_acks_all_but_the_last(void)
{
    printf("test_burst_read_acks_all_but_the_last\n");

    imud_bus_t b;
    if (open_bus(&b, "ftdi:", IMU_ADDR) < 0) { EXPECT(0, "open"); return; }

    const uint8_t fifo[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
    ftfake_set_read_data(fifo, sizeof fifo);

    uint8_t buf[6] = { 0 };
    EXPECT_EQ(bus_burst_read(&b, 0x28, buf, sizeof buf), 0, "the burst succeeds");
    EXPECT_EQ(ftfake_n_read(), 6, "six bytes clocked in");
    EXPECT(memcmp(buf, fifo, sizeof fifo) == 0, "delivered in order");
    EXPECT(ftfake_last_read_nakked(), "the last byte is NAKed");
    EXPECT_EQ(ftfake_stops(), 1, "still one transaction");
    bus_close(&b);
}

/* A read long enough to need more than one USB write must stay ONE
 * transaction: the bus holds its state across the split, so there is no stop
 * in the middle. */
static void test_long_burst_stays_one_transaction(void)
{
    printf("test_long_burst_stays_one_transaction\n");

    imud_bus_t b;
    if (open_bus(&b, "ftdi:", IMU_ADDR) < 0) { EXPECT(0, "open"); return; }

    uint8_t src[900], buf[900];
    for (unsigned i = 0; i < sizeof src; i++) src[i] = (uint8_t)(i * 7u);
    ftfake_set_read_data(src, sizeof src);
    memset(buf, 0, sizeof buf);

    EXPECT_EQ(bus_burst_read(&b, 0x28, buf, sizeof buf), 0,
              "a 900-byte burst succeeds");
    EXPECT_EQ(ftfake_starts(), 2, "one start and one repeated start");
    EXPECT_EQ(ftfake_stops(), 1, "exactly one stop, at the very end");
    EXPECT(memcmp(buf, src, sizeof src) == 0, "every byte in order");
    bus_close(&b);
}

/*
 * An address nobody answers is ENXIO — no such part — where a byte refused
 * part-way through is a bus fault.  The distinction is what lets a driver's
 * probe say "wrong address" instead of "wiring".
 */
static void test_address_nak_is_enxio(void)
{
    printf("test_address_nak_is_enxio\n");

    imud_bus_t b;
    /* Open against an address the fake does not host. */
    ftfake_reset();
    ftfake_add_device(IMU_ADDR);
    bus_spec_t spec = { .kind = BUS_I2C, .node = "ftdi:", .i2c_addr = 0x7B };
    if (bus_open(&b, &spec, NULL, "imu") < 0) { EXPECT(0, "open"); return; }

    uint8_t v = 0xEE;
    errno = 0;
    EXPECT_EQ(bus_reg_read(&b, 0x0F, &v), -1, "the read fails");
    EXPECT_EQ(errno, ENXIO, "with ENXIO");
    EXPECT_EQ(v, 0xEE, "and leaves the caller's buffer alone");
    bus_close(&b);
}

static void test_mid_transfer_nak_is_eio(void)
{
    printf("test_mid_transfer_nak_is_eio\n");

    imud_bus_t b;
    if (open_bus(&b, "ftdi:", IMU_ADDR) < 0) { EXPECT(0, "open"); return; }

    /* Byte 1 is the address; byte 2 is the register, refused. */
    ftfake_nak_nth_byte(2);
    errno = 0;
    EXPECT_EQ(bus_reg_write(&b, 0x10, 0x4C), -1, "the write fails");
    EXPECT_EQ(errno, EIO, "with EIO, not ENXIO");
    bus_close(&b);
}

/*
 * imud opens the IMU and the magnetometer separately, and on this transport
 * they are one dongle.  The second open must share it rather than claiming the
 * interface twice, and closing one handle must leave the other working.
 */
static void test_two_handles_share_one_dongle(void)
{
    printf("test_two_handles_share_one_dongle\n");

    ftfake_reset();
    ftfake_add_device(IMU_ADDR);
    ftfake_add_device(MAG_ADDR);

    imud_bus_t ib, mb;
    bus_spec_t is = { .kind = BUS_I2C, .node = "ftdi:", .i2c_addr = IMU_ADDR };
    bus_spec_t ms = { .kind = BUS_I2C, .node = "ftdi:", .i2c_addr = MAG_ADDR };

    EXPECT(bus_open(&ib, &is, NULL, "imu") == 0, "the IMU handle opens");
    EXPECT(bus_open(&mb, &ms, NULL, "mag") == 0, "the mag handle opens");
    EXPECT_EQ(ib.fd, mb.fd, "both name the same device");

    const uint8_t d[] = { 0x6C, 0x3D };
    ftfake_set_read_data(d, sizeof d);

    uint8_t v = 0;
    EXPECT_EQ(bus_reg_read(&ib, 0x0F, &v), 0, "the IMU reads");
    EXPECT_EQ(v, 0x6C, "its own value");
    EXPECT_EQ(bus_reg_read(&mb, 0x0F, &v), 0, "the mag reads");
    EXPECT_EQ(v, 0x3D, "its own value");
    EXPECT_EQ(ftfake_addr(1), (IMU_ADDR << 1) | 1, "each addressed separately");
    EXPECT_EQ(ftfake_addr(3), (MAG_ADDR << 1) | 1, "on the shared bus");

    /* Closing one must not take the device out from under the other. */
    bus_close(&ib);
    const uint8_t d2[] = { 0x3D };
    ftfake_set_read_data(d2, sizeof d2);
    EXPECT_EQ(bus_reg_read(&mb, 0x0F, &v), 0, "the survivor still reads");
    bus_close(&mb);
}

/* The node carries an optional clock, because a breadboard is not a PCB. */
static void test_node_speed_suffix(void)
{
    printf("test_node_speed_suffix\n");

    imud_bus_t b;
    EXPECT(open_bus(&b, "ftdi:@100000", IMU_ADDR) == 0, "a speed is accepted");
    EXPECT_EQ(ftfake_divisor(), 199, "divisor for 100 kHz");
    EXPECT(strcmp(ftfake_want(), "") == 0, "and is not part of the match");
    bus_close(&b);

    EXPECT(open_bus(&b, "ftdi:1-2@100000", IMU_ADDR) == 0, "with a match too");
    EXPECT(strcmp(ftfake_want(), "1-2") == 0, "which reaches the transport");
    bus_close(&b);

    ftfake_reset();
    ftfake_add_device(IMU_ADDR);
    bus_spec_t bad = { .kind = BUS_I2C, .node = "ftdi:@3", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&b, &bad, NULL, "imu") < 0, "a clock below the range is refused");
}

/*
 * Routing.  Only this backend is linked here, so a plain path has nothing to
 * fall back to — which is the case a Mac build is in, and it must say so
 * rather than open() a path that was never one.
 */
static void test_node_routing(void)
{
    printf("test_node_routing\n");

    ftfake_reset();
    ftfake_add_device(IMU_ADDR);

    imud_bus_t b;
    bus_spec_t dev = { .kind = BUS_I2C, .node = "/dev/i2c-1", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&b, &dev, NULL, "imu") < 0,
           "a device path finds no backend in this build");

    bus_spec_t other = { .kind = BUS_I2C, .node = "mystery:0", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&b, &other, NULL, "imu") < 0, "nor does an unknown scheme");

    bus_spec_t empty = { .kind = BUS_I2C, .node = "", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&b, &empty, NULL, "imu") < 0, "nor an empty node");
}

/* A dongle that is not there must fail the open, not the first transfer. */
static void test_missing_dongle_fails_open(void)
{
    printf("test_missing_dongle_fails_open\n");

    ftfake_reset();
    ftfake_fail_open(1);

    imud_bus_t b;
    bus_spec_t spec = { .kind = BUS_I2C, .node = "ftdi:", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&b, &spec, NULL, "imu") < 0, "the open fails");
    EXPECT(b.fd < 0, "leaving the handle closed");
    bus_close(&b);
    EXPECT(1, "and bus_close on it returns");
}

/*
 * SPI is not implemented, and must say so rather than clock something wrong
 * onto a bus.  ENOSYS is the same answer src/bus_null.c gives for a transport
 * the build cannot do — see issue #73.
 */
static void test_spi_is_enosys(void)
{
    printf("test_spi_is_enosys\n");

    imud_bus_t b;
    bus_init(&b);
    b.be = &bus_ft232h_backend;

    errno = 0;
    EXPECT_EQ(b.be->spi_setup(0, 0, 8, 1000000), -1, "spi setup fails");
    EXPECT_EQ(errno, ENOSYS, "with ENOSYS");

    uint8_t tx = 0x8F, rx = 0;
    bus_spi_leg_t leg = { .tx = &tx, .rx = &rx, .len = 1, .bits = 8 };
    errno = 0;
    EXPECT_EQ(b.be->spi_msg(&b, &leg, 1), -1, "spi message fails");
    EXPECT_EQ(errno, ENOSYS, "with ENOSYS");

    /* And a config asking for it is refused at open, by bus.c's own policy. */
    bus_caps_t caps = { .spi_capable = false };
    bus_spec_t spec = { .kind = BUS_SPI, .node = "ftdi:", .spi_hz = 1000000 };
    imud_bus_t sb;
    EXPECT(bus_open(&sb, &spec, &caps, "imu") < 0, "a SPI open is refused");
}

static void test_backend_struct_is_complete(void)
{
    printf("test_backend_struct_is_complete\n");

    const bus_backend_t *B = &bus_ft232h_backend;
    EXPECT(B->name != NULL, "the backend names itself");
    EXPECT(B->scheme != NULL && strcmp(B->scheme, "ftdi:") == 0,
           "and claims the ftdi: scheme");
    EXPECT(B->open && B->close && B->spi_setup && B->i2c_xfer && B->spi_msg,
           "every entry point is set");
}

int main(void)
{
    printf("=== test_bus_ft232h ===\n");

    test_backend_struct_is_complete();
    test_init_configures_for_i2c();
    test_register_write_is_one_transaction();
    test_register_read_uses_a_repeated_start();
    test_burst_read_acks_all_but_the_last();
    test_long_burst_stays_one_transaction();
    test_address_nak_is_enxio();
    test_mid_transfer_nak_is_eio();
    test_two_handles_share_one_dongle();
    test_node_speed_suffix();
    test_node_routing();
    test_missing_dongle_fails_open();
    test_spi_is_enosys();

    printf("\n%d passed, %d failed\n", g_checks - g_fail, g_fail);
    return g_fail ? 1 : 0;
}
