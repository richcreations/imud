/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_ft232h.c — include/bus_backend.h over an FT232H USB bridge.
 *
 * For a host with no bus on a header: a Mac, a laptop, a BSD box, or a Pi
 * whose header is already spoken for.  The chip's MPSSE engine clocks I2C in
 * software, so this file is the framing an I2C master would otherwise be —
 * start and stop conditions, a clocked byte, the ACK bit after it — expressed
 * as MPSSE opcodes.  The USB underneath is include/ft_usb.h, which is the only
 * part a second host has to write.
 *
 * WIRING.  AD0 is SCL, AD1 is SDA out and AD2 is SDA in; AD1 and AD2 must be
 * tied together at the board, because MPSSE always samples its input on AD2
 * and there is no way to read back the pin it drives.  Pull-ups on both lines
 * are the operator's, as on any I2C bus — the FT232H has none.
 *
 * SPEED.  One transaction is one USB round trip, so latency rather than clock
 * rate sets the sample ceiling: ~1 ms per register read against ~20 us on a
 * header.  A FIFO burst amortises that over its whole payload, which is why
 * the ST drivers' batched drain matters far more here than on i2c-dev.
 *
 * NO INTERRUPT.  Nothing here offers an edge line, so the reader threads fall
 * back to their rate-sized timer exactly as they do with src/imu_gpio_null.c.
 *
 * SPI is not implemented.  The two entry points fail with ENOSYS rather than
 * guessing at a chip-select pin, its polarity and how a second sensor would
 * get one — decisions a wired part should make, not this file.  Issue #73.
 */

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bus_backend.h"
#include "ft_usb.h"
#include "log.h"

/* ── FTDI and MPSSE constants ────────────────────────────────────────────── */

#define FT_VID              0x0403
#define FT_PID_232H         0x6014

#define SIO_RESET           0x00
#define SIO_SET_LATENCY     0x09
#define SIO_SET_BITMODE     0x0B

#define SIO_RESET_PURGE_RX  1
#define SIO_RESET_PURGE_TX  2

#define BITMODE_RESET       0x0000
#define BITMODE_MPSSE       0x0200      /* (0x02 << 8) | mask 0x00 */

/* AN_108.  Only the opcodes this file emits. */
#define MC_SETB_LOW         0x80        /* set ADBUS value + direction */
#define MC_CLK_DIV          0x86
#define MC_FLUSH            0x87        /* send immediate */
#define MC_CLK_DIV5_OFF     0x8A
#define MC_3PHASE_ON        0x8D        /* required for I2C */
#define MC_ADAPTIVE_OFF     0x97
#define MC_DRIVE_ZERO       0x9E        /* open-drain: FT232H only */
#define MC_BOGUS            0xAB        /* answered with FA AB */

#define MC_OUT_BYTES        0x11        /* MSB first, clock out on -ve edge */
#define MC_OUT_BITS         0x13
#define MC_IN_BYTES         0x20        /* MSB first, clock in on +ve edge */
#define MC_IN_BITS          0x22

/* ADBUS bits.  AD2 is an input always and so never appears in a direction. */
#define PIN_SCL             0x01
#define PIN_SDA             0x02
#define DIR_DRIVE           (PIN_SCL | PIN_SDA)   /* SCL out, SDA out */
#define DIR_LISTEN          PIN_SCL               /* SCL out, SDA released */

/*
 * The ACK arrives as the low bit of the byte a one-bit read returns.  Measured
 * on an FT232H against an LSM6DSOX and an LIS3MDL: the two present addresses
 * came back with bit 0 clear and every absent one with it set.
 */
#define I2C_ACKED(b)        (((b) & 0x01u) == 0u)

/*
 * MPSSE runs from a 60 MHz master clock, and three-phase clocking stretches
 * each bit over three half-periods instead of two.  So SCL = 60e6 / ((1+div)
 * * 2) * 2/3, and div = 20e6/hz - 1.
 */
#define DIV_FOR(hz)         ((uint16_t)((20000000u / (hz)) - 1u))

/* 400 kHz: fast mode, which every part in the tree supports.  An operator on
 * long jumper leads can ask for less with an "@<hz>" suffix on the node. */
#define DEFAULT_I2C_HZ      400000u
#define MIN_I2C_HZ          10000u
#define MAX_I2C_HZ          1000000u

/* One flush's worth of MPSSE.  A byte read costs 12 command bytes, so this is
 * about 340 payload bytes per USB round trip. */
#define CMDBUF              4096u

/* ── Open devices ────────────────────────────────────────────────────────── */

/*
 * imud opens the IMU and the magnetometer as two handles, and on this
 * transport they are commonly the same dongle.  So a device is opened once and
 * shared, refcounted, with a lock held across each transaction: the reader
 * threads issue transfers concurrently, and unlike i2c-dev there is no kernel
 * below us to serialise them.
 */
#define MAX_DEV 4

struct ftdev {
    ft_usb_t       *u;
    int             refs;
    uint32_t        hz;
    char            spec[80];   /* config i2c_bus is char[64] */
    pthread_mutex_t lock;
};

static struct ftdev g_dev[MAX_DEV];
static pthread_mutex_t g_table = PTHREAD_MUTEX_INITIALIZER;

/* ── MPSSE plumbing ──────────────────────────────────────────────────────── */

/*
 * Read exactly `want` payload bytes.  Every USB packet an FT232H sends starts
 * with two modem-status bytes, which carry no data and must come off before
 * anything is appended; a packet holding only those two means "nothing yet".
 */
static int mpsse_reply(struct ftdev *d, uint8_t *out, unsigned want)
{
    unsigned pkt = ft_usb_packet_size(d->u);
    uint8_t  buf[512];
    if (pkt > sizeof buf) pkt = sizeof buf;

    unsigned got = 0;
    for (unsigned tries = 0; tries < 200 && got < want; tries++) {
        int n = ft_usb_read(d->u, buf, pkt, 50);
        if (n < 0) return -1;
        if (n <= 2) continue;

        unsigned avail = (unsigned)n - 2;
        if (avail > want - got) avail = want - got;
        memcpy(out + got, buf + 2, avail);
        got += avail;
    }

    if (got < want) { errno = ETIMEDOUT; return -1; }
    return 0;
}

/* A command stream under construction, with the replies it will produce. */
typedef struct {
    struct ftdev *d;
    uint8_t       cmd[CMDBUF];
    unsigned      n;          /* bytes queued */
    uint8_t      *reply;      /* caller's collector */
    unsigned      replied;    /* bytes already collected */
    unsigned      pending;    /* replies the queued commands will produce */
    int           failed;
} mpsse_t;

static void emit(mpsse_t *m, const uint8_t *b, unsigned n)
{
    if (m->failed) return;
    if (m->n + n > CMDBUF) { m->failed = 1; errno = EOVERFLOW; return; }
    memcpy(m->cmd + m->n, b, n);
    m->n += n;
}

#define EMIT(m, ...) do {                             \
        const uint8_t bytes_[] = { __VA_ARGS__ };     \
        emit((m), bytes_, (unsigned)sizeof bytes_);   \
    } while (0)

/* Send what is queued and collect the replies it owes. */
static int mpsse_flush(mpsse_t *m)
{
    if (m->failed) return -1;
    if (m->n == 0) return 0;

    if (m->pending) EMIT(m, MC_FLUSH);
    if (m->failed) return -1;

    if (ft_usb_write(m->d->u, m->cmd, m->n, 1000) < 0) return -1;
    m->n = 0;

    if (m->pending) {
        if (mpsse_reply(m->d, m->reply + m->replied, m->pending) < 0) return -1;
        m->replied += m->pending;
        m->pending  = 0;
    }
    return 0;
}

/*
 * Room for `need` more command bytes, flushing if there is not.  Callers ask
 * for the whole of what they are about to emit INCLUDING the stop condition
 * that follows the loop, since a flush cannot happen between the two.
 */
static int mpsse_room(mpsse_t *m, unsigned need)
{
    if (m->n + need + 1 > CMDBUF) return mpsse_flush(m);
    return m->failed ? -1 : 0;
}

/* What one call emits, for the reservations above: a stop condition is three
 * held levels plus the release, and a byte either way is four commands. */
#define COST_STOP  40u
#define COST_BYTE  16u

/*
 * A level change is emitted several times over so it is held for a few MPSSE
 * clocks: setup and hold on the start and stop conditions are what an I2C
 * slave keys off, and a single command would leave them at one 60 MHz tick.
 */
static void hold(mpsse_t *m, uint8_t value, uint8_t dir)
{
    for (int i = 0; i < 4; i++) EMIT(m, MC_SETB_LOW, value, dir);
}

static void i2c_start(mpsse_t *m)
{
    hold(m, PIN_SCL | PIN_SDA, DIR_DRIVE);   /* both high */
    hold(m, PIN_SCL,           DIR_DRIVE);   /* SDA falls with SCL high */
    EMIT(m, MC_SETB_LOW, 0x00, DIR_DRIVE);   /* SCL low: ready to clock */
}

static void i2c_stop(mpsse_t *m)
{
    hold(m, 0x00,              DIR_DRIVE);
    hold(m, PIN_SCL,           DIR_DRIVE);   /* SCL rises, SDA still low */
    hold(m, PIN_SCL | PIN_SDA, DIR_DRIVE);   /* SDA rises: stop */
    EMIT(m, MC_SETB_LOW, PIN_SCL | PIN_SDA, 0x00);  /* release the bus */
}

/* Clock a byte out, then one bit in for the slave's ACK. */
static void i2c_write_byte(mpsse_t *m, uint8_t v)
{
    EMIT(m, MC_SETB_LOW, 0x00, DIR_DRIVE);
    EMIT(m, MC_OUT_BYTES, 0x00, 0x00, v);
    EMIT(m, MC_SETB_LOW, 0x00, DIR_LISTEN);
    EMIT(m, MC_IN_BITS, 0x00);
    m->pending++;
}

/* Clock a byte in, then drive ACK (more to come) or NAK (this was the last). */
static void i2c_read_byte(mpsse_t *m, int nak)
{
    EMIT(m, MC_SETB_LOW, 0x00, DIR_LISTEN);
    EMIT(m, MC_IN_BYTES, 0x00, 0x00);
    EMIT(m, MC_SETB_LOW, 0x00, DIR_DRIVE);
    EMIT(m, MC_OUT_BITS, 0x00, (uint8_t)(nak ? 0x80 : 0x00));
    m->pending++;
}

/*
 * Put the engine in MPSSE mode and configure it for I2C.  The sync exchange at
 * the end is the only proof the engine is really running: a bad opcode is
 * answered with FA and the opcode back, and silence means the chip is still in
 * whatever mode the last user left it.
 */
static int mpsse_init(struct ftdev *d)
{
    ft_usb_control(d->u, SIO_RESET, 0);
    ft_usb_control(d->u, SIO_RESET, SIO_RESET_PURGE_RX);
    ft_usb_control(d->u, SIO_RESET, SIO_RESET_PURGE_TX);
    ft_usb_control(d->u, SIO_SET_LATENCY, 1);

    if (ft_usb_control(d->u, SIO_SET_BITMODE, BITMODE_RESET) < 0 ||
        ft_usb_control(d->u, SIO_SET_BITMODE, BITMODE_MPSSE) < 0) {
        LOG_E("ft232h: cannot enter MPSSE mode: %s\n", strerror(errno));
        return -1;
    }

    uint16_t div = DIV_FOR(d->hz);
    mpsse_t m;
    memset(&m, 0, sizeof m);
    m.d = d;

    EMIT(&m, MC_CLK_DIV5_OFF, MC_ADAPTIVE_OFF, MC_3PHASE_ON);
    EMIT(&m, MC_CLK_DIV, (uint8_t)(div & 0xFF), (uint8_t)(div >> 8));
    EMIT(&m, MC_DRIVE_ZERO, 0x07, 0x00);   /* AD0-2 open-drain */
    hold(&m, PIN_SCL | PIN_SDA, DIR_DRIVE);
    if (mpsse_flush(&m) < 0) {
        LOG_E("ft232h: cannot configure MPSSE: %s\n", strerror(errno));
        return -1;
    }

    uint8_t echo[2] = { 0, 0 };
    m.reply   = echo;
    m.replied = 0;
    EMIT(&m, MC_BOGUS);
    m.pending = 2;
    if (mpsse_flush(&m) < 0 || echo[0] != 0xFA || echo[1] != MC_BOGUS) {
        LOG_E("ft232h: MPSSE did not answer the sync exchange\n");
        errno = EIO;
        return -1;
    }

    LOG_I("ft232h: I2C at %u Hz on %s\n", d->hz, ft_usb_name(d->u));
    return 0;
}

/* ── Node parsing ────────────────────────────────────────────────────────── */

/*
 * "ftdi:[<match>][@<hz>]" — the match is a serial, a "<bus>.<dev>" or a sysfs
 * port path, all resolved by include/ft_usb.h; empty means the first FT232H.
 * Returns 0, or -1 with errno set on a speed outside what MPSSE can divide to.
 */
static int parse_node(const char *node, char *match, size_t msz, uint32_t *hz)
{
    const char *s = node + strlen("ftdi:");
    const char *at = strrchr(s, '@');

    *hz = DEFAULT_I2C_HZ;
    if (at) {
        char *end = NULL;
        unsigned long v = strtoul(at + 1, &end, 10);
        if (!end || *end != '\0' || v < MIN_I2C_HZ || v > MAX_I2C_HZ) {
            errno = EINVAL;
            return -1;
        }
        *hz = (uint32_t)v;
    }

    size_t n = at ? (size_t)(at - s) : strlen(s);
    if (n >= msz) { errno = ENAMETOOLONG; return -1; }
    memcpy(match, s, n);
    match[n] = '\0';
    return 0;
}

/* ── include/bus_backend.h ───────────────────────────────────────────────── */

static int ft232h_open(const char *node)
{
    char     match[64];
    uint32_t hz;
    if (parse_node(node, match, sizeof match, &hz) < 0) {
        LOG_E("ft232h: cannot parse %s (want ftdi:[<match>][@<hz>])\n", node);
        return -1;
    }

    pthread_mutex_lock(&g_table);

    /* Already open?  Two sensors on one dongle share the device. */
    int free_slot = -1;
    for (int i = 0; i < MAX_DEV; i++) {
        if (g_dev[i].refs > 0) {
            if (strcmp(g_dev[i].spec, node) == 0) {
                g_dev[i].refs++;
                pthread_mutex_unlock(&g_table);
                return i;
            }
        } else if (free_slot < 0) {
            free_slot = i;
        }
    }

    if (free_slot < 0) {
        pthread_mutex_unlock(&g_table);
        LOG_E("ft232h: no free slot (%d devices already open)\n", MAX_DEV);
        errno = EMFILE;
        return -1;
    }

    struct ftdev *d = &g_dev[free_slot];
    memset(d, 0, sizeof *d);
    d->hz = hz;
    snprintf(d->spec, sizeof d->spec, "%s", node);

    d->u = ft_usb_open(match, FT_VID, FT_PID_232H, 0);
    if (!d->u) {
        pthread_mutex_unlock(&g_table);
        return -1;
    }

    if (pthread_mutex_init(&d->lock, NULL) != 0) {
        ft_usb_close(d->u);
        d->u = NULL;
        pthread_mutex_unlock(&g_table);
        errno = ENOMEM;
        return -1;
    }

    if (mpsse_init(d) < 0) {
        pthread_mutex_destroy(&d->lock);
        ft_usb_close(d->u);
        d->u = NULL;
        pthread_mutex_unlock(&g_table);
        return -1;
    }

    d->refs = 1;
    pthread_mutex_unlock(&g_table);
    return free_slot;
}

static void ft232h_close(int h)
{
    if (h < 0 || h >= MAX_DEV) return;

    pthread_mutex_lock(&g_table);
    struct ftdev *d = &g_dev[h];
    if (d->refs > 0 && --d->refs == 0) {
        ft_usb_control(d->u, SIO_SET_BITMODE, BITMODE_RESET);
        ft_usb_close(d->u);
        pthread_mutex_destroy(&d->lock);
        memset(d, 0, sizeof *d);
    }
    pthread_mutex_unlock(&g_table);
}

/*
 * One I2C transaction, as include/bus_backend.h defines it: the address and
 * `txlen` bytes, then a repeated start and `rxlen` bytes read back when one is
 * asked for.  The stream is chunked across USB writes where it has to be — the
 * bus holds its state between them, so a transaction is not broken by the
 * split — and the replies come back in issue order: one ACK per byte written,
 * then the data.
 */
static int ft232h_i2c_xfer(const imud_bus_t *b, const uint8_t *tx,
                           uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    if (b->fd < 0 || b->fd >= MAX_DEV) { errno = EBADF; return -1; }
    struct ftdev *d = &g_dev[b->fd];
    if (!d->u) { errno = EBADF; return -1; }

    unsigned nack = 1u + txlen + (rxlen ? 1u : 0u);   /* ACK bits expected */
    uint8_t *reply = malloc(nack + rxlen);
    if (!reply) { errno = ENOMEM; return -1; }

    pthread_mutex_lock(&d->lock);

    mpsse_t m;
    memset(&m, 0, sizeof m);
    m.d     = d;
    m.reply = reply;

    i2c_start(&m);
    i2c_write_byte(&m, (uint8_t)(b->i2c_addr << 1));
    for (uint16_t i = 0; i < txlen && !m.failed; i++) {
        if (mpsse_room(&m, COST_BYTE + COST_STOP) < 0) break;
        i2c_write_byte(&m, tx[i]);
    }

    if (rxlen && !m.failed) {
        i2c_start(&m);                                   /* repeated start */
        i2c_write_byte(&m, (uint8_t)((b->i2c_addr << 1) | 1u));
        for (uint16_t i = 0; i < rxlen && !m.failed; i++) {
            if (mpsse_room(&m, COST_BYTE + COST_STOP) < 0) break;
            i2c_read_byte(&m, i + 1 == rxlen);
        }
    }

    i2c_stop(&m);
    int rc = m.failed ? -1 : mpsse_flush(&m);

    pthread_mutex_unlock(&d->lock);

    if (rc == 0) {
        /*
         * Only a WRITTEN byte produces a reply: its ACK is clocked in.  A read
         * byte's own acknowledgement is the master's and is clocked out, so it
         * produces none.  Every write in a transaction precedes every read, so
         * the stream is all the ACKs, then all the data.
         */
        unsigned ai = 0, di = nack;
        for (; ai < nack; ai++) {
            if (!I2C_ACKED(reply[ai])) {
                /* An address that does not answer is ENXIO — no such part —
                 * where a byte refused mid-transfer is a bus fault. */
                errno = (ai == 0 || (rxlen && ai == 1u + txlen)) ? ENXIO : EIO;
                rc = -1;
                break;
            }
        }
        if (rc == 0 && rxlen) memcpy(rx, reply + di, rxlen);
    }

    free(reply);
    return rc;
}

static int ft232h_spi_setup(int h, uint8_t mode, uint8_t bits, uint32_t hz)
{
    (void)h; (void)mode; (void)bits; (void)hz;
    LOG_E("ft232h: this backend has no SPI yet — use bus = \"i2c\"\n");
    errno = ENOSYS;
    return -1;
}

static int ft232h_spi_msg(const imud_bus_t *b, const bus_spi_leg_t *legs,
                          unsigned n)
{
    (void)b; (void)legs; (void)n;
    errno = ENOSYS;
    return -1;
}

const bus_backend_t bus_ft232h_backend = {
    .name      = "ft232h",
    .scheme    = "ftdi:",
    .open      = ft232h_open,
    .close     = ft232h_close,
    .spi_setup = ft232h_spi_setup,
    .i2c_xfer  = ft232h_i2c_xfer,
    .spi_msg   = ft232h_spi_msg,
};
