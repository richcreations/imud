/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ft_usb_fake.c — a scripted FT232H behind include/ft_usb.h.  See ft_usb_fake.h.
 *
 * The interpreter is deliberately a BUS model rather than a stream recorder:
 * it tracks SCL and SDA through each SET_BITS_LOW and raises a start when SDA
 * falls with SCL high, so the suite asserts the I2C that reached the wire and
 * not the particular MPSSE spelling that produced it.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ft_usb.h"
#include "ft_usb_fake.h"

/* MPSSE opcodes this interpreter understands.  Anything else is consumed as a
 * one-byte command, which is true of every configuration opcode. */
#define OP_SETB_LOW   0x80
#define OP_SETB_HIGH  0x82
#define OP_GETB_LOW   0x81
#define OP_CLK_DIV    0x86
#define OP_FLUSH      0x87
#define OP_3PHASE     0x8C
#define OP_DRIVE_ZERO 0x9E
#define OP_BOGUS      0xAB
#define OP_3PHASE_OFF 0x8D
#define OP_OUT_BYTES  0x11
#define OP_OUT_BITS   0x13
#define OP_IN_BYTES   0x20
#define OP_IN_BITS    0x22

/* The SPI spellings of the byte opcodes: mode 0 drives on the falling edge,
 * mode 2 on the rising one, and a duplex opcode does both directions at once. */
#define OP_OUT_BYTES2 0x10
#define OP_IN_BYTES2  0x24
#define OP_IO_BYTES   0x31
#define OP_IO_BYTES2  0x34

#define N_CS   5           /* AD3–AD7 */
#define MAXLEG 8

#define MAXOBS 512
#define MAXQ   8192

struct ft_usb { int dummy; };
static struct ft_usb g_handle;

static struct {
    /* bus levels */
    int scl, sda, prev_scl, prev_sda, primed;

    /* transaction */
    int      in_xact, expect_addr;
    unsigned byte_no;          /* bytes written this transaction, address = 1 */
    int      last_ack;         /* 0 = ACK, 1 = NAK, for the next bit read */

    /* script */
    uint8_t  devs[128];
    uint8_t  rdata[MAXQ];
    unsigned rdata_n, rdata_i;
    int      nak_nth;
    int      fail_open;

    /* observations */
    unsigned starts, stops, n_addr, n_wrote, n_read;
    uint8_t  addrs[MAXOBS], wrote[MAXOBS];
    int      last_read_nakked;
    int      three_phase, drive_zero;
    uint16_t divisor;
    char     want[128];

    /* SPI: chip-select levels, and one "message" per select held low */
    int      cs[N_CS];
    int      spi_cs;                 /* the select currently low, or -1 */
    unsigned spi_msgs;
    int      msg_cs[MAXOBS];
    unsigned msg_bytes[MAXOBS];
    unsigned msg_legs[MAXOBS];
    uint8_t  msg_op[MAXOBS][MAXLEG];
    unsigned spi_n_mosi;
    uint8_t  spi_mosi[MAXOBS];

    /* reply queue back to the host */
    uint8_t  q[MAXQ];
    unsigned qn, qi;

    /* partial command carried between bulk writes.  Big enough for one SPI
     * opcode at the backend's chunk size, which is far larger than any I2C
     * command — a short buffer here would silently drop payload. */
    uint8_t  pend[2048];
    unsigned pendn;
} F;

void ftfake_reset(void)
{
    memset(&F, 0, sizeof F);
    F.nak_nth = -1;
    F.scl = F.sda = F.prev_scl = F.prev_sda = 1;
    F.spi_cs = -1;
    for (int i = 0; i < N_CS; i++) F.cs[i] = 1;   /* deasserted */
}

void ftfake_add_device(uint8_t addr)   { F.devs[addr & 0x7F] = 1; }
void ftfake_nak_nth_byte(int n)        { F.nak_nth = n; }
void ftfake_fail_open(int fail)        { F.fail_open = fail; }

void ftfake_set_read_data(const uint8_t *d, unsigned n)
{
    if (n > MAXQ) n = MAXQ;
    memcpy(F.rdata, d, n);
    F.rdata_n = n;
    F.rdata_i = 0;
}

unsigned ftfake_starts(void)          { return F.starts; }
unsigned ftfake_stops(void)           { return F.stops; }
unsigned ftfake_n_addr(void)          { return F.n_addr; }
uint8_t  ftfake_addr(unsigned i)      { return i < F.n_addr ? F.addrs[i] : 0; }
unsigned ftfake_n_wrote(void)         { return F.n_wrote; }
uint8_t  ftfake_wrote(unsigned i)     { return i < F.n_wrote ? F.wrote[i] : 0; }
unsigned ftfake_n_read(void)          { return F.n_read; }
int      ftfake_last_read_nakked(void){ return F.last_read_nakked; }
int      ftfake_three_phase(void)     { return F.three_phase; }
int      ftfake_drive_zero(void)      { return F.drive_zero; }
uint16_t ftfake_divisor(void)         { return F.divisor; }
const char *ftfake_want(void)         { return F.want; }

unsigned ftfake_spi_msgs(void)        { return F.spi_msgs; }
unsigned ftfake_spi_n_mosi(void)      { return F.spi_n_mosi; }

int ftfake_spi_cs(unsigned i)
{
    return i < F.spi_msgs && i < MAXOBS ? F.msg_cs[i] : -1;
}

unsigned ftfake_spi_bytes(unsigned i)
{
    return i < F.spi_msgs && i < MAXOBS ? F.msg_bytes[i] : 0;
}

unsigned ftfake_spi_legs(unsigned i)
{
    return i < F.spi_msgs && i < MAXOBS ? F.msg_legs[i] : 0;
}

uint8_t ftfake_spi_op(unsigned i, unsigned leg)
{
    if (i >= F.spi_msgs || i >= MAXOBS || leg >= MAXLEG) return 0;
    return leg < F.msg_legs[i] ? F.msg_op[i][leg] : 0;
}

uint8_t ftfake_spi_mosi(unsigned i)
{
    return i < F.spi_n_mosi && i < MAXOBS ? F.spi_mosi[i] : 0;
}

int ftfake_spi_idle(void)
{
    for (int i = 0; i < N_CS; i++) if (!F.cs[i]) return 0;
    return 1;
}

static void push(uint8_t b)
{
    if (F.qn < MAXQ) F.q[F.qn++] = b;
}

/* An open-drain line reads high unless something drives it low. */
static void set_pins(uint8_t val, uint8_t dir)
{
    F.prev_scl = F.scl;
    F.prev_sda = F.sda;
    F.scl = (dir & 0x01) ? ((val >> 0) & 1) : 1;
    F.sda = (dir & 0x02) ? ((val >> 1) & 1) : 1;

    /*
     * Chip selects on AD3–AD7.  An undriven one reads high, as a pull-up or
     * the part's own would leave it, so a select the backend forgot to make an
     * output looks deasserted here rather than silently asserted.  A falling
     * edge opens a message and a rising edge closes it.
     */
    for (int k = 0; k < N_CS; k++) {
        uint8_t bit = (uint8_t)(0x08u << k);
        int lvl = (dir & bit) ? ((val & bit) ? 1 : 0) : 1;
        if (lvl == F.cs[k]) continue;
        F.cs[k] = lvl;

        if (!lvl) {
            F.spi_cs = k;
            if (F.spi_msgs < MAXOBS) {
                F.msg_cs[F.spi_msgs]    = k;
                F.msg_bytes[F.spi_msgs] = 0;
                F.msg_legs[F.spi_msgs]  = 0;
            }
            F.spi_msgs++;
        } else if (F.spi_cs == k) {
            F.spi_cs = -1;
        }
    }

    if (!F.primed) { F.primed = 1; return; }

    if (F.prev_scl && F.scl) {
        if (F.prev_sda && !F.sda) {          /* SDA falls with SCL high */
            F.starts++;
            F.in_xact     = 1;
            F.expect_addr = 1;
            F.byte_no     = 0;
        } else if (!F.prev_sda && F.sda) {   /* SDA rises with SCL high */
            if (F.in_xact) F.stops++;
            F.in_xact = 0;
        }
    }
}

static void wrote_byte(uint8_t v)
{
    if (!F.in_xact) return;
    F.byte_no++;

    int present;
    if (F.expect_addr) {
        if (F.n_addr < MAXOBS) F.addrs[F.n_addr] = v;
        F.n_addr++;
        F.expect_addr = 0;
        present       = F.devs[(v >> 1) & 0x7F];
    } else {
        if (F.n_wrote < MAXOBS) F.wrote[F.n_wrote] = v;
        F.n_wrote++;
        present = 1;   /* the slave already answered its address */
    }

    int forced = (F.nak_nth > 0 && (unsigned)F.nak_nth == F.byte_no);
    F.last_ack = (present && !forced) ? 0 : 1;
}

/*
 * Record one byte opcode against the message the open chip select started.
 * `mosi` is NULL for an opcode that only reads, which is how a test tells a
 * duplex read leg (zeros on MOSI, as spidev would send) from an in-only one.
 */
static void spi_leg(uint8_t op, const uint8_t *mosi, unsigned len, int reads)
{
    if (F.spi_cs < 0 || F.spi_msgs == 0) return;

    unsigned m = F.spi_msgs - 1;
    if (m < MAXOBS) {
        if (F.msg_legs[m] < MAXLEG) F.msg_op[m][F.msg_legs[m]] = op;
        F.msg_legs[m]++;
        F.msg_bytes[m] += len;
    }

    if (mosi) {
        for (unsigned i = 0; i < len; i++) {
            if (F.spi_n_mosi < MAXOBS) F.spi_mosi[F.spi_n_mosi] = mosi[i];
            F.spi_n_mosi++;
        }
    }

    if (!reads) return;
    for (unsigned i = 0; i < len; i++)
        push(F.rdata_i < F.rdata_n ? F.rdata[F.rdata_i++] : 0xFF);
}

/* Consume one complete command from `p`, returning its length, or 0 when the
 * buffer holds only part of one. */
static unsigned step(const uint8_t *p, unsigned n)
{
    uint8_t op = p[0];

    switch (op) {
    case OP_SETB_LOW:
        if (n < 3) return 0;
        set_pins(p[1], p[2]);
        return 3;

    case OP_SETB_HIGH:
        return n < 3 ? 0 : 3;

    case OP_CLK_DIV:
        if (n < 3) return 0;
        F.divisor = (uint16_t)(p[1] | ((uint16_t)p[2] << 8));
        return 3;

    case OP_DRIVE_ZERO:
        if (n < 3) return 0;
        F.drive_zero = p[1];
        return 3;

    case OP_3PHASE:
        F.three_phase = 1;
        return 1;

    case OP_3PHASE_OFF:
        F.three_phase = 0;
        return 1;

    case OP_BOGUS:
        push(0xFA);
        push(OP_BOGUS);
        return 1;

    case OP_GETB_LOW:
        push((uint8_t)((F.scl ? 1 : 0) | (F.sda ? 2 : 0) | 0xFC));
        return 1;

    case OP_OUT_BYTES:
    case OP_OUT_BYTES2: {
        if (n < 3) return 0;
        unsigned len = (unsigned)(p[1] | ((unsigned)p[2] << 8)) + 1u;
        if (n < 3 + len) return 0;
        if (F.spi_cs >= 0) spi_leg(op, p + 3, len, 0);
        else if (op == OP_OUT_BYTES)
            for (unsigned i = 0; i < len; i++) wrote_byte(p[3 + i]);
        return 3 + len;
    }

    /* Duplex: a payload goes out and the same count comes back. */
    case OP_IO_BYTES:
    case OP_IO_BYTES2: {
        if (n < 3) return 0;
        unsigned len = (unsigned)(p[1] | ((unsigned)p[2] << 8)) + 1u;
        if (n < 3 + len) return 0;
        spi_leg(op, p + 3, len, 1);
        return 3 + len;
    }

    case OP_IN_BYTES2: {
        if (n < 3) return 0;
        unsigned len = (unsigned)(p[1] | ((unsigned)p[2] << 8)) + 1u;
        spi_leg(op, NULL, len, 1);
        return 3;
    }

    case OP_OUT_BITS:
        /* The master's own acknowledgement, MSB first: bit 7 set is a NAK. */
        if (n < 3) return 0;
        F.last_read_nakked = (p[2] & 0x80) ? 1 : 0;
        return 3;

    case OP_IN_BYTES: {
        if (n < 3) return 0;
        unsigned len = (unsigned)(p[1] | ((unsigned)p[2] << 8)) + 1u;
        if (F.spi_cs >= 0) { spi_leg(op, NULL, len, 1); return 3; }
        for (unsigned i = 0; i < len; i++) {
            push(F.rdata_i < F.rdata_n ? F.rdata[F.rdata_i++] : 0xFF);
            F.n_read++;
        }
        return 3;
    }

    case OP_IN_BITS:
        /* One bit: the slave's ACK, delivered as the byte's low bit. */
        if (n < 2) return 0;
        push(F.last_ack ? 0xFFu : 0x00u);
        return 2;

    case OP_FLUSH:
        return 1;

    default:
        return 1;   /* a configuration opcode with no operands */
    }
}

/* ── include/ft_usb.h ────────────────────────────────────────────────────── */

ft_usb_t *ft_usb_open(const char *want, uint16_t vid, uint16_t pid,
                      unsigned ifno)
{
    (void)vid; (void)pid; (void)ifno;
    snprintf(F.want, sizeof F.want, "%s", want ? want : "");
    if (F.fail_open) { errno = ENODEV; return NULL; }
    return &g_handle;
}

void ft_usb_close(ft_usb_t *u) { (void)u; }

int ft_usb_control(ft_usb_t *u, uint8_t request, uint16_t value)
{
    (void)u; (void)request; (void)value;
    return 0;
}

int ft_usb_write(ft_usb_t *u, const uint8_t *buf, unsigned len,
                 unsigned timeout_ms)
{
    (void)u; (void)timeout_ms;

    for (unsigned i = 0; i < len; i++) {
        if (F.pendn < sizeof F.pend) F.pend[F.pendn++] = buf[i];

        unsigned used;
        while (F.pendn && (used = step(F.pend, F.pendn)) != 0) {
            memmove(F.pend, F.pend + used, F.pendn - used);
            F.pendn -= used;
        }
    }
    return (int)len;
}

int ft_usb_read(ft_usb_t *u, uint8_t *buf, unsigned len, unsigned timeout_ms)
{
    (void)u; (void)timeout_ms;
    if (len < 2) return 0;

    /* Every packet carries the two modem-status bytes first, exactly as the
     * chip does — stripping them is part of what src/bus_ft232h.c must get
     * right. */
    buf[0] = 0x32;
    buf[1] = 0x60;

    unsigned room = len - 2, i = 0;
    while (i < room && F.qi < F.qn) buf[2 + i++] = F.q[F.qi++];

    if (F.qi == F.qn) { F.qi = F.qn = 0; }
    return (int)(2 + i);
}

unsigned ft_usb_packet_size(const ft_usb_t *u) { (void)u; return 512; }

const char *ft_usb_name(const ft_usb_t *u) { (void)u; return "fake-ft232h"; }
