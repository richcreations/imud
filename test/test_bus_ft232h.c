/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_bus_ft232h.c — the FT232H bus backend (src/bus_ft232h.c).
 *
 * What is pinned here is the bus, not the MPSSE: test/ft_usb_fake.c interprets
 * the command stream as a bus does, so these assertions are about start
 * conditions, addresses, the repeated start before a read and the NAK that
 * ends one — the things a driver's register access depends on and that no
 * readback would reveal on its own.  A register read that skipped its repeated
 * start would still return the right byte from a forgiving slave.
 *
 * The SPI cases are the same idea one layer over: a "message" is one chip
 * select held low, so leg count and word width are assertable where a readback
 * would show nothing.  A one-byte register read sent as two 8-bit legs instead
 * of the single 16-bit word returns the identical byte and is a different
 * transfer — see spi_burst_read() in src/drivers/bus_io.h.
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
    /* Two handles, one claimed interface: the token is per sensor, because on
     * SPI each carries its own chip select, mode and clock.  That they share
     * the dongle is what the reads below and the survivor at the end show. */
    EXPECT(ib.fd != mb.fd, "each gets its own handle");

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
 * Routing, in the shape a macOS build really has: the bridge plus the null
 * backend.  A plain device path must reach the null one and OPEN — that is
 * what lets `driver = sim` run the whole pipeline on a host with no bus of its
 * own, and config/sim.conf's "/dev/null" is exactly this path.  Routing it
 * nowhere instead took every sim-driver suite down on the Mac.
 */
static void test_node_routing(void)
{
    printf("test_node_routing\n");

    ftfake_reset();
    ftfake_add_device(IMU_ADDR);

    imud_bus_t b;
    bus_spec_t dev = { .kind = BUS_I2C, .node = "/dev/null", .i2c_addr = IMU_ADDR };
    int routed = bus_open(&b, &dev, NULL, "imu");
    EXPECT(routed == 0,
           "a plain device path falls back rather than failing to route");

    /* Guarded: a regression here must report, not crash.  bus_io.h
     * dereferences b->be, so a handle whose open failed cannot be handed to
     * it — see the note on imud_bus_t.be in include/bus.h. */
    if (routed == 0) {
        EXPECT(b.be != NULL && strcmp(b.be->name, "null") == 0,
               "reaching the null backend, not the bridge");

        /* And it is the TRANSFER that fails there, with the errno that names
         * a missing backend rather than a wiring fault. */
        uint8_t v = 0;
        errno = 0;
        EXPECT_EQ(bus_reg_read(&b, 0x0F, &v), -1, "whose transfers fail");
        EXPECT_EQ(errno, ENOSYS, "with ENOSYS");
        bus_close(&b);
    }

    /* A scheme nobody answers to is still an error at open: it names a
     * transport this build has not got, which no fallback can serve. */
    bus_spec_t other = { .kind = BUS_I2C, .node = "mystery:0", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&b, &other, NULL, "imu") < 0, "an unknown scheme is refused");

    bus_spec_t empty = { .kind = BUS_I2C, .node = "", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&b, &empty, NULL, "imu") < 0, "and so is an empty node");
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

/* Open a SPI handle for a part clocking `mode`, at its 10 MHz maximum.  Does
 * NOT reset the fake: the two-handle cases open a second one behind the first. */
static int open_spi(imud_bus_t *b, const char *node, uint8_t mode)
{
    bus_caps_t caps = { .spi_capable = true, .spi_mode = mode,
                        .spi_max_hz = 10000000, .spi_inc_mask = 0 };
    bus_spec_t spec = { .kind = BUS_SPI, .node = node, .spi_hz = 0 };
    return bus_open(b, &spec, &caps, "imu");
}

/*
 * SPI wants the opposite of I2C on two of the three settings: two-phase
 * clocking, because a SPI part samples on a clock edge rather than mid-bit,
 * and push-pull, because nothing on the bus is open-drain.  Getting either
 * wrong still clocks bytes, which is what makes them worth asserting.
 */
static void test_spi_init_configures_for_spi(void)
{
    printf("test_spi_init_configures_for_spi\n");

    ftfake_reset();
    imud_bus_t b;
    EXPECT(open_spi(&b, "ftdi:/cs0", 0) == 0, "opens the bridge for SPI");
    EXPECT(!ftfake_three_phase(), "two-phase clocking");
    EXPECT_EQ(ftfake_drive_zero(), 0x00, "push-pull, not open-drain");
    /* 10 MHz, the part's maximum: div = 30e6/hz - 1. */
    EXPECT_EQ(ftfake_divisor(), 2, "divisor for 10 MHz");
    EXPECT(ftfake_spi_idle(), "every chip select high at rest");
    bus_close(&b);
}

/*
 * The byte-order assertion, and the one most easily got wrong: bus_io.h builds
 * a register write as ONE native-endian uint16_t and the wire is MSB first, so
 * on a little-endian host the pair swaps.  A backend that emitted the buffer
 * as it found it would write the value into the register named by the value.
 */
static void test_spi_register_write_is_one_selected_word(void)
{
    printf("test_spi_register_write_is_one_selected_word\n");

    ftfake_reset();
    imud_bus_t b;
    EXPECT(open_spi(&b, "ftdi:/cs0", 0) == 0, "opens");

    EXPECT(bus_reg_write(&b, 0x12, 0x44) == 0, "the write succeeds");
    EXPECT_EQ(ftfake_spi_msgs(), 1, "one chip-select assertion");
    EXPECT_EQ(ftfake_spi_cs(0), 0, "on cs0");
    EXPECT_EQ(ftfake_spi_legs(0), 1, "as a single leg");
    EXPECT_EQ(ftfake_spi_bytes(0), 2, "of two bytes");
    EXPECT_EQ(ftfake_spi_op(0, 0), 0x11, "mode 0 clocks out on the falling edge");
    EXPECT_EQ(ftfake_spi_mosi(0), 0x12, "the register goes first");
    EXPECT_EQ(ftfake_spi_mosi(1), 0x44, "then the value");
    EXPECT(ftfake_spi_idle(), "and the select is released");
    bus_close(&b);
}

/*
 * A one-byte read is command and data in one 16-bit word precisely so it
 * cannot be split — see spi_burst_read() in src/drivers/bus_io.h.  One leg is
 * the whole claim; a backend that sent two would return the same byte.
 */
static void test_spi_one_byte_read_is_one_word(void)
{
    printf("test_spi_one_byte_read_is_one_word\n");

    ftfake_reset();
    imud_bus_t b;
    EXPECT(open_spi(&b, "ftdi:/cs0", 0) == 0, "opens");

    const uint8_t back[] = { 0xAA, 0x6B };   /* dummy under the command, data */
    ftfake_set_read_data(back, sizeof back);

    uint8_t v = 0;
    EXPECT(bus_reg_read(&b, 0x0F, &v) == 0, "the read succeeds");
    EXPECT_EQ(v, 0x6B, "and lands the data byte, not the dummy");
    EXPECT_EQ(ftfake_spi_msgs(), 1, "one chip-select assertion");
    EXPECT_EQ(ftfake_spi_legs(0), 1, "one leg: a 16-bit word cannot be split");
    EXPECT_EQ(ftfake_spi_bytes(0), 2, "of two bytes");
    EXPECT_EQ(ftfake_spi_op(0, 0), 0x31, "duplex, so MOSI is driven throughout");
    EXPECT_EQ(ftfake_spi_mosi(0), 0x8F, "the register with the read bit set");
    bus_close(&b);
}

/*
 * A multi-byte read is two legs under one select, and its data phase must
 * clock ZEROS.  bus_io.h reasons about that fill explicitly — an in-only
 * opcode would leave MOSI wherever the command byte left it, and a 1 in bit 7
 * is a read opcode to these parts.
 */
static void test_spi_burst_read_is_two_legs_one_select(void)
{
    printf("test_spi_burst_read_is_two_legs_one_select\n");

    ftfake_reset();
    imud_bus_t b;
    EXPECT(open_spi(&b, "ftdi:/cs0", 0) == 0, "opens");

    const uint8_t back[] = { 1, 2, 3, 4, 5, 6 };
    ftfake_set_read_data(back, sizeof back);

    uint8_t buf[6] = { 0 };
    EXPECT(bus_burst_read(&b, 0x22, buf, sizeof buf) == 0, "the burst succeeds");
    EXPECT(memcmp(buf, back, sizeof back) == 0, "and returns what was clocked");

    EXPECT_EQ(ftfake_spi_msgs(), 1, "one chip select for the whole burst");
    EXPECT_EQ(ftfake_spi_legs(0), 2, "command leg then data leg");
    EXPECT_EQ(ftfake_spi_bytes(0), 7, "one command byte and six data");
    EXPECT_EQ(ftfake_spi_op(0, 0), 0x11, "the command is written");
    EXPECT_EQ(ftfake_spi_op(0, 1), 0x31, "the data phase is duplex");
    EXPECT_EQ(ftfake_spi_mosi(0), 0xA2, "0x22 with the read bit");

    int zeros = 1;
    for (unsigned i = 1; i < 7; i++) if (ftfake_spi_mosi(i) != 0x00) zeros = 0;
    EXPECT(zeros, "and the data phase clocks zeros, as spidev would");
    bus_close(&b);
}

/*
 * The topology this transport exists for: two sensors, one dongle, a chip
 * select each.  They share the claimed USB interface — keying the device table
 * on the whole node string would try to claim it twice, which usbfs refuses.
 */
static void test_spi_two_handles_get_their_own_select(void)
{
    printf("test_spi_two_handles_get_their_own_select\n");

    ftfake_reset();
    imud_bus_t imu, mag;
    EXPECT(open_spi(&imu, "ftdi:/cs0", 0) == 0, "the IMU opens on cs0");
    EXPECT(open_spi(&mag, "ftdi:/cs1", 0) == 0, "the mag opens on cs1");
    EXPECT(imu.fd != mag.fd, "with tokens of their own");

    EXPECT(bus_reg_write(&imu, 0x10, 0x60) == 0, "the IMU writes");
    EXPECT(bus_reg_write(&mag, 0x09, 0x01) == 0, "the mag writes");

    EXPECT_EQ(ftfake_spi_msgs(), 2, "two selections");
    EXPECT_EQ(ftfake_spi_cs(0), 0, "the first on cs0");
    EXPECT_EQ(ftfake_spi_cs(1), 1, "the second on cs1");
    EXPECT(ftfake_spi_idle(), "both released afterwards");

    bus_close(&imu);
    bus_close(&mag);
}

/*
 * MPSSE reaches CPHA=1 only by borrowing three-phase clocking, which leaves
 * SCLK at a 25/75 duty cycle.  Refusing beats shipping a signal nothing here
 * can validate — the operator gets an error naming the mode, not a part that
 * reads plausible rubbish.
 */
static void test_spi_modes_1_and_3_are_refused(void)
{
    printf("test_spi_modes_1_and_3_are_refused\n");

    imud_bus_t b;
    ftfake_reset();
    EXPECT(open_spi(&b, "ftdi:/cs0", 1) < 0, "mode 1 is refused");
    ftfake_reset();
    EXPECT(open_spi(&b, "ftdi:/cs0", 3) < 0, "mode 3 is refused");

    ftfake_reset();
    EXPECT(open_spi(&b, "ftdi:/cs0", 2) == 0, "mode 2 is native and accepted");
    bus_close(&b);
}

/* Mode 2 idles SCK high and drives on the rising edge, so it needs the other
 * pair of opcodes.  Same bytes on the wire, different edges under them. */
static void test_spi_mode_2_uses_its_own_opcodes(void)
{
    printf("test_spi_mode_2_uses_its_own_opcodes\n");

    ftfake_reset();
    imud_bus_t b;
    EXPECT(open_spi(&b, "ftdi:/cs0", 2) == 0, "opens in mode 2");
    EXPECT(bus_reg_write(&b, 0x12, 0x44) == 0, "the write succeeds");
    EXPECT_EQ(ftfake_spi_op(0, 0), 0x10, "clocks out on the rising edge");

    uint8_t v = 0;
    const uint8_t back[] = { 0x00, 0x77 };
    ftfake_set_read_data(back, sizeof back);
    EXPECT(bus_reg_read(&b, 0x0F, &v) == 0, "the read succeeds");
    EXPECT_EQ(ftfake_spi_op(1, 0), 0x34, "and reads on the falling one");
    bus_close(&b);
}

/*
 * A FIFO drain is longer than one MPSSE command buffer, so it chunks across
 * USB writes.  The select is an ADBUS latch and holds its level across them —
 * if it did not, the burst would break into separate transfers and the part
 * would restart its auto-increment mid-drain.
 */
static void test_spi_long_burst_stays_one_select(void)
{
    printf("test_spi_long_burst_stays_one_select\n");

    ftfake_reset();
    imud_bus_t b;
    EXPECT(open_spi(&b, "ftdi:/cs0", 0) == 0, "opens");

    static uint8_t buf[1500];
    EXPECT(bus_burst_read(&b, 0x3E, buf, sizeof buf) == 0, "the burst succeeds");
    EXPECT_EQ(ftfake_spi_msgs(), 1, "still one chip-select assertion");
    EXPECT_EQ(ftfake_spi_bytes(0), 1 + sizeof buf, "carrying every byte");
    EXPECT(ftfake_spi_legs(0) > 2, "split into chunks under it");
    EXPECT(ftfake_spi_idle(), "and released at the end");
    bus_close(&b);
}

/*
 * The node has to say which pin selects the part: two sensors defaulting to
 * one select would collide silently, so there is no default.  And "@<hz>" is
 * the I2C clock — accepting it on a SPI node would read as setting the SPI one.
 */
static void test_spi_node_must_name_a_select(void)
{
    printf("test_spi_node_must_name_a_select\n");

    imud_bus_t b;
    ftfake_reset();
    EXPECT(open_spi(&b, "ftdi:", 0) < 0, "a bare node is refused for SPI");
    ftfake_reset();
    EXPECT(open_spi(&b, "ftdi:@100000/cs0", 0) < 0,
           "a bus clock on a SPI node is refused");
    ftfake_reset();
    EXPECT(open_spi(&b, "ftdi:/cs9", 0) < 0, "a select out of range is refused");
    ftfake_reset();
    EXPECT(open_spi(&b, "ftdi:/xx0", 0) < 0, "a malformed tail is refused");

    /* And a driver with no SPI at all is still refused by bus.c's own policy. */
    bus_caps_t caps = { .spi_capable = false };
    bus_spec_t spec = { .kind = BUS_SPI, .node = "ftdi:/cs0", .spi_hz = 1000000 };
    ftfake_reset();
    EXPECT(bus_open(&b, &spec, &caps, "imu") < 0,
           "a driver without SPI is refused");
}

/* One dongle drives one protocol: SPI and I2C want the same three pins with
 * incompatible framing, so a mixed pair could not share a wire in any case. */
static void test_spi_and_i2c_cannot_share_a_dongle(void)
{
    printf("test_spi_and_i2c_cannot_share_a_dongle\n");

    ftfake_reset();
    ftfake_add_device(IMU_ADDR);

    imud_bus_t i2c, spi;
    bus_spec_t spec = { .kind = BUS_I2C, .node = "ftdi:", .i2c_addr = IMU_ADDR };
    EXPECT(bus_open(&i2c, &spec, NULL, "imu") == 0, "the I2C handle opens");
    EXPECT(open_spi(&spi, "ftdi:/cs0", 0) < 0, "and a SPI one on it is refused");
    bus_close(&i2c);
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

    test_spi_init_configures_for_spi();
    test_spi_register_write_is_one_selected_word();
    test_spi_one_byte_read_is_one_word();
    test_spi_burst_read_is_two_legs_one_select();
    test_spi_two_handles_get_their_own_select();
    test_spi_modes_1_and_3_are_refused();
    test_spi_mode_2_uses_its_own_opcodes();
    test_spi_long_burst_stays_one_select();
    test_spi_node_must_name_a_select();
    test_spi_and_i2c_cannot_share_a_dongle();

    printf("\n%d passed, %d failed\n", g_checks - g_fail, g_fail);
    return g_fail ? 1 : 0;
}
