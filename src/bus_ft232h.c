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
 * WIRING, I2C.  AD0 is SCL, AD1 is SDA out and AD2 is SDA in; AD1 and AD2 must
 * be tied together at the board, because MPSSE always samples its input on AD2
 * and there is no way to read back the pin it drives.  Pull-ups on both lines
 * are the operator's, as on any I2C bus — the FT232H has none.
 *
 * WIRING, SPI.  AD0 is SCK, AD1 is MOSI, AD2 is MISO, and AD3–AD7 are five
 * chip selects, "/cs0" through "/cs4" in the node.  The board's D1/D2 tie —
 * a switch on the post-2020 Adafruit revision — must be OFF: it shorts MOSI to
 * MISO for I2C and SPI cannot work through it.
 *
 * All five chip selects are driven, high, from the moment the first SPI handle
 * opens, rather than only the ones a handle claims.  imud opens and configures
 * the IMU before it opens the magnetometer, so a select left as an input would
 * float through every register write of that first init — and a part that
 * chooses its interface from the CS level, as the MMC5983MA does, is watching
 * the clock the whole time.  The cost is AD5, which is the only pin MPSSE's
 * 0x88 wait-on-high can watch; narrow SPI_CS_ALL if that is ever wanted.
 *
 * SPEED.  One transaction is one USB round trip, so latency rather than clock
 * rate sets the sample ceiling: ~1 ms per register read against ~20 us on a
 * header.  A FIFO burst amortises that over its whole payload, which is why
 * the ST drivers' batched drain matters far more here than on i2c-dev.  SPI
 * costs one opcode per leg where I2C costs several USB transfers per byte, so
 * it is the faster half of this backend by a wide margin.
 *
 * NO INTERRUPT.  Nothing here offers an edge line, so the reader threads fall
 * back to their rate-sized timer exactly as they do with src/imu_gpio_null.c.
 *
 * SPI MODES 0 AND 2 ONLY.  MPSSE clocks CPHA=0 natively and reaches modes 1
 * and 3 only by borrowing three-phase clocking, which leaves SCLK at a 25/75
 * duty cycle.  spi_setup refuses them rather than ship a signal nothing here
 * can validate; that rules this transport out for icm42688p, lis3mdl and
 * rm3100.  Issue #73.
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
/*
 * AN_108's 0x8A/0x8B and 0x8C/0x8D are disable/enable pairs, and the two
 * three-phase opcodes are easy to get the wrong way round — 0x8E is not the
 * disable, it is "clock n bits with no data transfer" and swallows two operand
 * bytes, which desynchronises the whole command stream silently.
 *
 * Measured on an FT232H rather than read off a table: 600 bytes at a 50 kHz
 * two-phase divisor take 96.5 ms with 0x8D and 144.4 ms with 0x8C, exactly the
 * 1.5x that stretching each bit from two half-periods to three predicts.
 */
#define MC_3PHASE_ON        0x8C        /* required for I2C */
#define MC_ADAPTIVE_OFF     0x97
#define MC_DRIVE_ZERO       0x9E        /* open-drain: FT232H only */
#define MC_BOGUS            0xAB        /* answered with FA AB */

#define MC_3PHASE_OFF       0x8D        /* two-phase: what SPI wants */

#define MC_OUT_BYTES        0x11        /* MSB first, clock out on -ve edge */
#define MC_OUT_BITS         0x13
#define MC_IN_BYTES         0x20        /* MSB first, clock in on +ve edge */
#define MC_IN_BITS          0x22

/*
 * SPI byte opcodes, by mode.  CPHA=0 means the part samples on the first edge
 * of each bit, so the master must present data on the edge before it: mode 0
 * idles SCK low and drives on the falling edge, mode 2 idles high and drives
 * on the rising one.  The duplex opcode is what a leg carrying both tx and rx
 * needs — a one-byte register read is exactly that, and cannot be split.
 */
#define MC_SPI0_OUT         0x11        /* out -ve            */
#define MC_SPI0_IO          0x31        /* out -ve, in +ve    */
#define MC_SPI2_OUT         0x10        /* out +ve            */
#define MC_SPI2_IO          0x34        /* out +ve, in -ve    */

/* ADBUS bits.  AD2 is an input always and so never appears in a direction. */
#define PIN_SCL             0x01
#define PIN_SDA             0x02
#define DIR_DRIVE           (PIN_SCL | PIN_SDA)   /* SCL out, SDA out */
#define DIR_LISTEN          PIN_SCL               /* SCL out, SDA released */

/* The same two pins under their SPI names, plus the chip selects on AD3–AD7. */
#define PIN_SCK             0x01
#define PIN_MOSI            0x02
#define SPI_CS_MAX          4                     /* /cs0 … /cs4 */
#define SPI_CS_BIT(n)       ((uint8_t)(0x08u << (n)))
#define SPI_CS_ALL          0xF8u                 /* AD3–AD7 */
#define SPI_DIR             ((uint8_t)(PIN_SCK | PIN_MOSI | SPI_CS_ALL))

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
#define DIV_I2C(hz)         ((uint16_t)((20000000u / (hz)) - 1u))

/*
 * SPI runs two-phase, so a bit is two half-periods and SCK = 60e6/((1+div)*2)
 * — div = 30e6/hz - 1.  The division rounds UP so the result is never faster
 * than asked: bus.c has already clamped the request to the part's datasheet
 * maximum, and overshooting that here would undo it.
 */
#define DIV_SPI(hz)         ((uint16_t)(((30000000u + (hz) - 1u) / (hz)) - 1u))
#define SPI_HZ_FOR_DIV(d)   (30000000u / ((uint32_t)(d) + 1u))

/* 400 kHz: fast mode, which every part in the tree supports.  An operator on
 * long jumper leads can ask for less with an "@<hz>" suffix on the node. */
#define DEFAULT_I2C_HZ      400000u
#define MIN_I2C_HZ          10000u
#define MAX_I2C_HZ          1000000u

/* The divisor is 16 bits, which floors SCK at 60e6/(65536*2) ≈ 458 Hz. */
#define MIN_SPI_HZ          1000u
#define MAX_SPI_HZ          30000000u

/* One flush's worth of MPSSE.  A byte read costs 12 command bytes, so this is
 * about 340 payload bytes per USB round trip. */
#define CMDBUF              4096u

/* Most a single SPI opcode carries, so one chunk plus its framing always fits
 * a fresh command buffer.  Even, so chunking never splits a 16-bit word. */
#define SPI_CHUNK           1024u

/* ── Open devices ────────────────────────────────────────────────────────── */

/*
 * imud opens the IMU and the magnetometer as two handles, and on this
 * transport they are commonly the same dongle.  So a device is opened once and
 * shared, refcounted, with a lock held across each transaction: the reader
 * threads issue transfers concurrently, and unlike i2c-dev there is no kernel
 * below us to serialise them.
 *
 * The token this backend hands back is therefore a HANDLE, not a device: two
 * sensors on one dongle need one claimed interface between them but their own
 * chip select, SPI mode and clock.  The seam blesses that — bus_backend.h
 * documents the token as "an index on one that does not [have a device node]"
 * — and nothing above dereferences it.
 *
 * A device is keyed by the MATCH part of the node alone, so "ftdi:/cs0" and
 * "ftdi:/cs1" resolve to one dongle.  Keying on the whole node string would
 * claim the same USB interface twice, which usbfs refuses.
 */
#define MAX_DEV     4
#define MAX_HANDLE  (MAX_DEV * (SPI_CS_MAX + 1))

typedef enum {
    PROTO_NONE = 0,
    PROTO_I2C,
    PROTO_SPI,
} proto_t;

struct ftdev {
    ft_usb_t       *u;
    int             refs;
    uint32_t        hz;         /* I2C bus clock, from the node's "@<hz>" */
    char            match[80];  /* config i2c_bus is char[64] */
    pthread_mutex_t lock;

    /* One dongle drives one protocol: SPI and I2C want the same three pins
     * with different framing, so a mixed pair could not share a wire even if
     * this file let them. */
    proto_t         proto;

    /* SPI, shared across the handles on this dongle: the divisor and mode
     * currently programmed, so a message re-emits them only when its handle
     * differs from the last one served. */
    uint32_t        spi_hz;
    uint8_t         spi_mode;
    uint8_t         adbus;      /* current ADBUS output levels */
};

/*
 * SPI state is per sensor; cs < 0 means the node named no chip select and so
 * asked for I2C.  `used` rather than a sentinel in `dev` because the table is
 * static and starts zeroed, which would otherwise read as handle 0 holding
 * device 0.
 */
struct fthandle {
    int      used;
    int      dev;
    int      cs;
    uint8_t  mode;
    uint32_t hz;
};

static struct ftdev    g_dev[MAX_DEV];
static struct fthandle g_handle[MAX_HANDLE];
static pthread_mutex_t g_table = PTHREAD_MUTEX_INITIALIZER;

/* The handle table is written under g_table and read without it, on the same
 * basis as g_dev: entries are built before open() returns the token and torn
 * down only after the last user has closed. */
static struct ftdev *dev_for(int h, struct fthandle **out)
{
    if (h < 0 || h >= MAX_HANDLE) return NULL;
    struct fthandle *H = &g_handle[h];
    if (!H->used || H->dev < 0 || H->dev >= MAX_DEV) return NULL;
    struct ftdev *d = &g_dev[H->dev];
    if (!d->u) return NULL;
    if (out) *out = H;
    return d;
}

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
 * The I2C configuration: three-phase clocking so SDA is sampled mid-bit,
 * open-drain on the three bus pins, and both lines released high.
 */
static void cfg_i2c(mpsse_t *m, struct ftdev *d)
{
    uint16_t div = DIV_I2C(d->hz);
    EMIT(m, MC_3PHASE_ON);
    EMIT(m, MC_CLK_DIV, (uint8_t)(div & 0xFF), (uint8_t)(div >> 8));
    EMIT(m, MC_DRIVE_ZERO, 0x07, 0x00);   /* AD0-2 open-drain */
    hold(m, PIN_SCL | PIN_SDA, DIR_DRIVE);
}

/*
 * The SPI configuration: two-phase clocking, push-pull because a SPI master
 * drives both levels, and every chip select an output at rest — see the wiring
 * note at the top of this file for why all five and not just the one a handle
 * claims.  The divisor and the SCK idle level wait for spi_setup, which is
 * where the mode arrives; nothing is selected until then, so an idle level
 * that is briefly wrong for the mode reaches no part.
 */
static void cfg_spi(mpsse_t *m, struct ftdev *d)
{
    d->adbus = (uint8_t)SPI_CS_ALL;
    EMIT(m, MC_3PHASE_OFF);
    EMIT(m, MC_DRIVE_ZERO, 0x00, 0x00);
    EMIT(m, MC_SETB_LOW, d->adbus, SPI_DIR);
}

/*
 * Put the engine in MPSSE mode and configure it for the protocol the node
 * asked for.  The sync exchange at the end is the only proof the engine is
 * really running: a bad opcode is answered with FA and the opcode back, and
 * silence means the chip is still in whatever mode the last user left it.
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

    mpsse_t m;
    memset(&m, 0, sizeof m);
    m.d = d;

    EMIT(&m, MC_CLK_DIV5_OFF, MC_ADAPTIVE_OFF);
    if (d->proto == PROTO_SPI) cfg_spi(&m, d);
    else                       cfg_i2c(&m, d);
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

    if (d->proto == PROTO_SPI)
        LOG_I("ft232h: SPI on %s\n", ft_usb_name(d->u));
    else
        LOG_I("ft232h: I2C at %u Hz on %s\n", d->hz, ft_usb_name(d->u));
    return 0;
}

/* ── Node parsing ────────────────────────────────────────────────────────── */

/*
 * "ftdi:[<match>][@<hz>][/cs<N>]" — the match is a serial, a "<bus>.<dev>" or
 * a sysfs port path, all resolved by include/ft_usb.h; empty means the first
 * FT232H.  "@<hz>" is the I2C bus clock.  "/cs<N>" names a chip select and is
 * what makes the node a SPI one; `*cs` comes back -1 when it is absent.
 *
 * Returns 0, or -1 with errno set on a malformed tail or a speed outside what
 * MPSSE can divide to.  `*hz_set` reports whether a speed was written, so a
 * SPI node carrying one can be refused rather than silently ignored.
 */
static int parse_node(const char *node, char *match, size_t msz, uint32_t *hz,
                      int *hz_set, int *cs)
{
    const char *s = node + strlen("ftdi:");
    size_t      n = strlen(s);

    *hz     = DEFAULT_I2C_HZ;
    *hz_set = 0;
    *cs     = -1;

    /* "/cs<N>" tail.  No match this file resolves contains a slash — a serial
     * is alphanumeric, a sysfs port path is "1-2", a location ID is hex — so
     * the last one is unambiguously the separator. */
    const char *slash = strrchr(s, '/');
    if (slash) {
        char *end = NULL;
        if (strncmp(slash, "/cs", 3) != 0) { errno = EINVAL; return -1; }
        unsigned long v = strtoul(slash + 3, &end, 10);
        if (!end || end == slash + 3 || *end != '\0' || v > SPI_CS_MAX) {
            errno = EINVAL;
            return -1;
        }
        *cs = (int)v;
        n   = (size_t)(slash - s);
    }

    /* "@<hz>" before it, searched within what the tail left. */
    const char *at = NULL;
    for (size_t i = n; i > 0; i--) {
        if (s[i - 1] == '@') { at = s + i - 1; break; }
    }
    if (at) {
        char *end = NULL;
        unsigned long v = strtoul(at + 1, &end, 10);
        if (!end || end != s + n || v < MIN_I2C_HZ || v > MAX_I2C_HZ) {
            errno = EINVAL;
            return -1;
        }
        *hz     = (uint32_t)v;
        *hz_set = 1;
        n       = (size_t)(at - s);
    }

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
    int      hz_set, cs;
    if (parse_node(node, match, sizeof match, &hz, &hz_set, &cs) < 0) {
        LOG_E("ft232h: cannot parse %s "
              "(want ftdi:[<match>][@<hz>][/cs0-/cs%d])\n", node, SPI_CS_MAX);
        return -1;
    }

    /* The node declares the protocol: a chip select is only meaningful to SPI,
     * and a bus clock only to I2C. */
    proto_t want = cs >= 0 ? PROTO_SPI : PROTO_I2C;
    if (want == PROTO_SPI && hz_set) {
        LOG_E("ft232h: %s names both a chip select and a bus clock; \"@<hz>\" "
              "is the I2C clock — set the SPI clock with spi_speed_hz\n", node);
        errno = EINVAL;
        return -1;
    }

    pthread_mutex_lock(&g_table);

    int hslot = -1;
    for (int i = 0; i < MAX_HANDLE; i++) {
        if (!g_handle[i].used) { hslot = i; break; }
    }
    if (hslot < 0) {
        pthread_mutex_unlock(&g_table);
        LOG_E("ft232h: no free handle (%d already open)\n", MAX_HANDLE);
        errno = EMFILE;
        return -1;
    }

    /* Already open?  Two sensors on one dongle share the claimed interface. */
    int dslot = -1, free_slot = -1;
    for (int i = 0; i < MAX_DEV; i++) {
        if (g_dev[i].refs > 0) {
            if (strcmp(g_dev[i].match, match) == 0) { dslot = i; break; }
        } else if (free_slot < 0) {
            free_slot = i;
        }
    }

    if (dslot >= 0) {
        struct ftdev *d = &g_dev[dslot];
        if (d->proto != want) {
            pthread_mutex_unlock(&g_table);
            LOG_E("ft232h: %s is already open for %s; one dongle drives one "
                  "protocol, since both want the same three pins\n",
                  node, d->proto == PROTO_SPI ? "SPI" : "I2C");
            errno = EBUSY;
            return -1;
        }
        if (want == PROTO_I2C && d->hz != hz) {
            pthread_mutex_unlock(&g_table);
            LOG_E("ft232h: %s is already open at %u Hz; one dongle has one "
                  "I2C clock\n", node, d->hz);
            errno = EBUSY;
            return -1;
        }
        d->refs++;
    } else {
        if (free_slot < 0) {
            pthread_mutex_unlock(&g_table);
            LOG_E("ft232h: no free slot (%d devices already open)\n", MAX_DEV);
            errno = EMFILE;
            return -1;
        }

        struct ftdev *d = &g_dev[free_slot];
        memset(d, 0, sizeof *d);
        d->hz    = hz;
        d->proto = want;
        snprintf(d->match, sizeof d->match, "%s", match);

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
        dslot   = free_slot;
    }

    g_handle[hslot].used = 1;
    g_handle[hslot].dev  = dslot;
    g_handle[hslot].cs   = cs;
    g_handle[hslot].mode = 0;
    g_handle[hslot].hz   = 0;

    pthread_mutex_unlock(&g_table);
    return hslot;
}

static void ft232h_close(int h)
{
    if (h < 0 || h >= MAX_HANDLE) return;

    pthread_mutex_lock(&g_table);
    struct fthandle *H = &g_handle[h];
    if (H->used && H->dev >= 0 && H->dev < MAX_DEV) {
        struct ftdev *d = &g_dev[H->dev];
        if (d->refs > 0 && --d->refs == 0) {
            ft_usb_control(d->u, SIO_SET_BITMODE, BITMODE_RESET);
            ft_usb_close(d->u);
            pthread_mutex_destroy(&d->lock);
            memset(d, 0, sizeof *d);
        }
    }
    memset(H, 0, sizeof *H);
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
    struct ftdev *d = dev_for(b->fd, NULL);
    if (!d) { errno = EBADF; return -1; }
    if (d->proto != PROTO_I2C) { errno = EINVAL; return -1; }

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

/*
 * Configure this handle's chip select for `mode` at `hz`.
 *
 * `bits` goes unused: MPSSE clocks bytes MSB first, so a 16-bit word is two of
 * them and the only difference is byte order, which spi_msg applies to the
 * caller's buffer rather than to the engine.
 */
static int ft232h_spi_setup(int h, uint8_t mode, uint8_t bits, uint32_t hz)
{
    (void)bits;

    struct fthandle *H = NULL;
    struct ftdev    *d = dev_for(h, &H);
    if (!d) { errno = EBADF; return -1; }

    if (H->cs < 0 || d->proto != PROTO_SPI) {
        LOG_E("ft232h: this node names no chip select — append \"/cs0\" "
              "through \"/cs%d\" to spi_dev to say which pin selects the "
              "part\n", SPI_CS_MAX);
        errno = EINVAL;
        return -1;
    }

    if (mode != 0 && mode != 2) {
        LOG_E("ft232h: SPI mode %u reaches CPHA=1 only by borrowing "
              "three-phase clocking, which leaves SCLK at a 25/75 duty cycle; "
              "this backend clocks modes 0 and 2 only\n", mode);
        errno = EINVAL;
        return -1;
    }

    if (hz < MIN_SPI_HZ || hz > MAX_SPI_HZ) {
        LOG_E("ft232h: SPI clock %u Hz is outside %u–%u Hz\n",
              hz, MIN_SPI_HZ, MAX_SPI_HZ);
        errno = EINVAL;
        return -1;
    }

    pthread_mutex_lock(&d->lock);

    /*
     * Every part on one dongle shares SCK, so they share its idle level and a
     * mode-0 and a mode-2 part cannot both be right.  src/imu.c refuses that
     * pairing at startup with a message naming both drivers; this is the
     * backstop for imud-cal and imud-imutest, which build their own specs.
     */
    if (d->spi_hz && d->spi_mode != mode) {
        pthread_mutex_unlock(&d->lock);
        LOG_E("ft232h: this dongle is already clocking SPI mode %u, and one "
              "SCK cannot idle at two levels\n", d->spi_mode);
        errno = EINVAL;
        return -1;
    }

    uint16_t div    = DIV_SPI(hz);
    uint32_t actual = SPI_HZ_FOR_DIV(div);
    if (actual != hz)
        LOG_I("ft232h: SPI clock %u Hz requested, %u Hz is the nearest the "
              "divisor reaches without exceeding it\n", hz, actual);

    mpsse_t m;
    memset(&m, 0, sizeof m);
    m.d = d;

    d->adbus = (uint8_t)((mode == 2 ? PIN_SCK : 0u) | SPI_CS_ALL);
    EMIT(&m, MC_CLK_DIV, (uint8_t)(div & 0xFF), (uint8_t)(div >> 8));
    EMIT(&m, MC_SETB_LOW, d->adbus, SPI_DIR);
    int rc = mpsse_flush(&m);

    if (rc == 0) {
        d->spi_hz   = actual;
        d->spi_mode = mode;
        H->hz       = actual;
        H->mode     = mode;
    }

    pthread_mutex_unlock(&d->lock);

    if (rc < 0) {
        LOG_E("ft232h: cannot configure SPI: %s\n", strerror(errno));
        return -1;
    }
    LOG_I("ft232h: SPI mode %u at %u Hz on cs%d\n", mode, actual, H->cs);
    return 0;
}

/* One chunk's payload, in the byte order the wire wants. */
static void spi_payload(uint8_t *dst, const bus_spi_leg_t *L,
                        uint32_t off, uint32_t len)
{
    if (!L->tx) { memset(dst, 0, len); return; }

    if (L->bits == 16) {
        /* The caller's buffer holds native-endian uint16_t and the wire is MSB
         * first, so the pairs swap on a little-endian host.  test/bus_mock.c
         * unpacks the same way, and spi_reg_write() in src/drivers/bus_io.h is
         * what builds them. */
        for (uint32_t i = 0; i + 1 < len; i += 2) {
            uint16_t w;
            memcpy(&w, L->tx + off + i, 2);
            dst[i]     = (uint8_t)(w >> 8);
            dst[i + 1] = (uint8_t)(w & 0xFFu);
        }
        return;
    }
    memcpy(dst, L->tx + off, len);
}

/* Deal the collected replies back out to the legs that asked for them. */
static void spi_scatter(const bus_spi_leg_t *legs, unsigned n,
                        const uint8_t *src)
{
    for (unsigned i = 0; i < n; i++) {
        const bus_spi_leg_t *L = &legs[i];
        if (!L->rx) continue;

        if (L->bits == 16) {
            for (uint32_t k = 0; k + 1 < L->len; k += 2) {
                uint16_t w = (uint16_t)(((uint16_t)src[k] << 8) | src[k + 1]);
                memcpy(L->rx + k, &w, 2);
            }
        } else {
            memcpy(L->rx, src, L->len);
        }
        src += L->len;
    }
}

/*
 * One SPI message: every leg under a single chip select, per
 * include/bus_backend.h.
 *
 * A flush inside the loop is safe, so a leg longer than the command buffer is
 * chunked rather than refused: the chip select is an ADBUS latch and holds its
 * level across USB writes, the same property the I2C transaction relies on to
 * split without releasing the bus.
 */
static int ft232h_spi_msg(const imud_bus_t *b, const bus_spi_leg_t *legs,
                          unsigned n)
{
    struct fthandle *H = NULL;
    struct ftdev    *d = dev_for(b->fd, &H);
    if (!d) { errno = EBADF; return -1; }
    if (d->proto != PROTO_SPI || H->cs < 0 || H->hz == 0) {
        errno = EINVAL;
        return -1;
    }
    if (n < 1 || n > BUS_SPI_MAX_LEGS) { errno = EINVAL; return -1; }

    unsigned nrx = 0;
    for (unsigned i = 0; i < n; i++) {
        if (legs[i].len == 0) { errno = EINVAL; return -1; }
        /* A 16-bit leg is whole words or it cannot be byte-swapped. */
        if (legs[i].bits == 16 && (legs[i].len & 1u)) {
            errno = EINVAL;
            return -1;
        }
        if (legs[i].rx) nrx += legs[i].len;
    }

    uint8_t *reply = NULL;
    if (nrx) {
        reply = malloc(nrx);
        if (!reply) { errno = ENOMEM; return -1; }
    }

    const uint8_t out_op = (H->mode == 2) ? MC_SPI2_OUT : MC_SPI0_OUT;
    const uint8_t io_op  = (H->mode == 2) ? MC_SPI2_IO  : MC_SPI0_IO;

    pthread_mutex_lock(&d->lock);

    mpsse_t m;
    memset(&m, 0, sizeof m);
    m.d     = d;
    m.reply = reply;

    /* Reprogram only when the last message served a handle at another clock;
     * the mode cannot differ, spi_setup having refused a mixed pair. */
    if (d->spi_hz != H->hz) {
        uint16_t div = DIV_SPI(H->hz);
        EMIT(&m, MC_CLK_DIV, (uint8_t)(div & 0xFF), (uint8_t)(div >> 8));
        d->spi_hz = H->hz;
    }

    const uint8_t idle = d->adbus;
    const uint8_t sel  = (uint8_t)(idle & ~SPI_CS_BIT(H->cs));
    EMIT(&m, MC_SETB_LOW, sel, SPI_DIR);

    for (unsigned i = 0; i < n && !m.failed; i++) {
        const bus_spi_leg_t *L = &legs[i];
        for (uint32_t off = 0; off < L->len && !m.failed; ) {
            uint32_t chunk = L->len - off;
            if (chunk > SPI_CHUNK) chunk = SPI_CHUNK;
            if (mpsse_room(&m, chunk + 8u) < 0) break;

            /*
             * A leg that reads uses the duplex opcode even when it carries no
             * tx, so its data phase clocks the zeros spidev would send.
             * src/drivers/bus_io.h reasons about that fill explicitly — an
             * in-only opcode would leave MOSI at whatever the previous leg
             * left it, which is a 1 bit often enough to matter.
             */
            uint8_t pay[SPI_CHUNK];
            EMIT(&m, L->rx ? io_op : out_op,
                 (uint8_t)((chunk - 1u) & 0xFFu),
                 (uint8_t)((chunk - 1u) >> 8));
            spi_payload(pay, L, off, chunk);
            emit(&m, pay, chunk);

            if (L->rx) m.pending += chunk;
            off += chunk;
        }
    }

    EMIT(&m, MC_SETB_LOW, idle, SPI_DIR);
    int rc = m.failed ? -1 : mpsse_flush(&m);

    pthread_mutex_unlock(&d->lock);

    if (rc == 0 && nrx) spi_scatter(legs, n, reply);
    free(reply);
    return rc;
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
