/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bus_mock.c — a register-map device behind include/bus_backend.h.
 *
 * The third backend, beside src/bus_linux.c and src/bus_null.c: a suite links
 * this one instead, and the drivers under test reach it through exactly the
 * framing they use on real silicon.  No --wrap and no kernel headers, so the
 * driver suites build wherever the tree does.
 *
 * See bus_mock.h.
 */

#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "bus_backend.h"
#include "bus_mock.h"

#define NADDR   128     /* 7-bit I2C address space */
#define REGSZ   256     /* one 8-bit register file per address */
#define FIFOSZ  4096    /* per-address FIFO byte queue */
#define NSPIFD  8       /* bound spidev descriptors */
#define WLOGSZ  64      /* recorded prefix of the write-order log */

static uint8_t g_reg[NADDR][REGSZ];
static uint8_t g_fifo[NADDR][FIFOSZ];
static int     g_fifo_head[NADDR];   /* next byte to pop */
static int     g_fifo_tail[NADDR];   /* next free slot */
static int     g_fifo_reg[NADDR];    /* first FIFO-port register, or -1 */
static int     g_fifo_reg_hi[NADDR]; /* last FIFO-port register */
static uint32_t g_fifo_drops[NADDR];  /* pushes discarded at FIFOSZ */
static uint8_t g_selfclear[NADDR][REGSZ];  /* self-clearing bit masks */
static uint8_t g_live[NADDR][REGSZ];       /* post-read increment, 0 = static */
static int     g_alias[NADDR][REGSZ];      /* write also lands here, or -1 */
static i2cmock_write_cb g_on_write;        /* fired after each write, or NULL */
static uint32_t g_reads[NADDR][REGSZ];     /* per-register read tally */
static uint8_t  g_wlog[NADDR][WLOGSZ];     /* registers written, in order */
static uint32_t g_wlog_n[NADDR];           /* total writes; may exceed WLOGSZ */
static int      g_wlast[NADDR];            /* last register written, or -1 */
static int     g_fail_next;          /* one-shot transfer failure */
static int     g_fail_all;           /* sticky transfer failure (wedged bus) */

/* Targeted write failure: fail transfers that write a NAMED register, rather
 * than the Nth transfer.  See i2cmock_fail_write_to() in the header for why
 * the predicate rather than a call index. */
static int     g_wfail_addr = -1;
static int     g_wfail_reg  = -1;
static int     g_wfail_left;         /* >0 = that many; <0 = sticky */
static int     g_wfail_hit;          /* set by dev_write, read by dispatch */

/* spidev descriptor → register file. SPI puts no address on the wire, so the
 * binding is what stands in for the chip select. */
static int     g_spi_fd[NSPIFD];     /* bound descriptor, or -1 */
static uint8_t g_spi_addr[NSPIFD];
static uint8_t g_spi_inc_mask[NADDR];
static spimock_inc_t g_spi_inc_mode[NADDR];

/* Shape of the last SPI message, for test_backend_leg_shapes.  The framing in
 * bus_io.h is a claim about leg count and word width, and final register
 * values cannot express it: a one-byte read sent as two 8-bit legs leaves the
 * same bytes behind as the 16-bit single word that cannot be split. */
static unsigned g_last_nlegs;
static uint8_t  g_last_bits[BUS_SPI_MAX_LEGS];
static uint32_t g_last_len[BUS_SPI_MAX_LEGS];

static int spi_addr_for_fd(int fd)
{
    for (int i = 0; i < NSPIFD; i++)
        if (g_spi_fd[i] == fd) return g_spi_addr[i] & (NADDR - 1);
    return -1;
}

/* ── Public harness API ──────────────────────────────────────────────────── */

void i2cmock_reset(void)
{
    memset(g_reg, 0, sizeof g_reg);
    memset(g_fifo, 0, sizeof g_fifo);
    memset(g_fifo_head, 0, sizeof g_fifo_head);
    memset(g_fifo_tail, 0, sizeof g_fifo_tail);
    memset(g_fifo_drops, 0, sizeof g_fifo_drops);
    memset(g_selfclear, 0, sizeof g_selfclear);
    memset(g_live, 0, sizeof g_live);
    memset(g_reads, 0, sizeof g_reads);
    memset(g_wlog, 0, sizeof g_wlog);
    memset(g_wlog_n, 0, sizeof g_wlog_n);
    for (int i = 0; i < NADDR; i++) {
        g_fifo_reg[i] = -1; g_fifo_reg_hi[i] = -1;
        g_wlast[i] = -1;
        for (int r = 0; r < REGSZ; r++) g_alias[i][r] = -1;
    }
    memset(g_spi_inc_mask, 0, sizeof g_spi_inc_mask);
    memset(g_spi_inc_mode, 0, sizeof g_spi_inc_mode);
    for (int i = 0; i < NSPIFD; i++) { g_spi_fd[i] = -1; g_spi_addr[i] = 0; }
    g_fail_next = 0;
    g_fail_all  = 0;
    g_wfail_addr = -1;
    g_wfail_reg  = -1;
    g_wfail_left = 0;
    g_wfail_hit  = 0;
    g_on_write   = NULL;
    g_last_nlegs = 0;
    memset(g_last_bits, 0, sizeof g_last_bits);
    memset(g_last_len,  0, sizeof g_last_len);
}

void spimock_bind_inc(int fd, uint8_t addr, spimock_inc_t mode, uint8_t mask)
{
    g_spi_inc_mask[addr & (NADDR - 1)] = mask;
    g_spi_inc_mode[addr & (NADDR - 1)] = mode;
    for (int i = 0; i < NSPIFD; i++) {
        if (g_spi_fd[i] == fd || g_spi_fd[i] < 0) {
            g_spi_fd[i]   = fd;
            g_spi_addr[i] = addr;
            return;
        }
    }
    /* Out of slots: leave it unbound so dispatch_spi fails loudly rather than
     * quietly aliasing onto someone else's register file. */
}

void spimock_bind(int fd, uint8_t addr, uint8_t inc_mask)
{
    spimock_bind_inc(fd, addr, inc_mask ? SPIMOCK_INC_ON_BIT : SPIMOCK_INC_ALWAYS,
                     inc_mask);
}

void i2cmock_set_selfclear(uint8_t addr, uint8_t reg, uint8_t mask)
{
    g_selfclear[addr & (NADDR - 1)][reg] |= mask;
}

void i2cmock_set_live(uint8_t addr, uint8_t reg, uint8_t step)
{
    g_live[addr & (NADDR - 1)][reg] = step;
}

void i2cmock_set_write_alias(uint8_t addr, uint8_t reg, uint8_t also)
{
    g_alias[addr & (NADDR - 1)][reg] = also;
}

void i2cmock_on_write(i2cmock_write_cb cb)
{
    g_on_write = cb;
}

uint32_t i2cmock_read_count(uint8_t addr, uint8_t reg)
{
    return g_reads[addr & (NADDR - 1)][reg];
}

uint32_t i2cmock_writes(uint8_t addr)
{
    return g_wlog_n[addr & (NADDR - 1)];
}

int i2cmock_write_at(uint8_t addr, uint32_t n)
{
    int a = addr & (NADDR - 1);
    if (n >= g_wlog_n[a] || n >= WLOGSZ) return -1;
    return g_wlog[a][n];
}

int i2cmock_last_write(uint8_t addr)
{
    return g_wlast[addr & (NADDR - 1)];
}

void i2cmock_set_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    g_reg[addr & (NADDR - 1)][reg] = val;
}

void i2cmock_set_regs(uint8_t addr, uint8_t reg, const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        g_reg[addr & (NADDR - 1)][(uint8_t)(reg + i)] = buf[i];
}

uint8_t i2cmock_get_reg(uint8_t addr, uint8_t reg)
{
    return g_reg[addr & (NADDR - 1)][reg];
}

void i2cmock_set_fifo_reg(uint8_t addr, uint8_t reg)
{
    i2cmock_set_fifo_range(addr, reg, reg);
}

void i2cmock_set_fifo_range(uint8_t addr, uint8_t lo, uint8_t hi)
{
    int a = addr & (NADDR - 1);
    g_fifo_reg[a]    = lo;
    g_fifo_reg_hi[a] = hi < lo ? lo : hi;
}

void i2cmock_fifo_push(uint8_t addr, const uint8_t *buf, int len)
{
    int a = addr & (NADDR - 1);
    /*
     * Reclaim a fully drained queue. Without this, head and tail only ever
     * advance and FIFOSZ becomes a LIFETIME budget rather than a depth: a test
     * that pushes and drains repeatedly — which is what the timed phases do,
     * one sample per progress call — silently stops being fed partway through.
     * That is a capacity limit masquerading as a timing flake, and it scales
     * with how FAST the machine runs, because more iterations spend more of the
     * budget. It cost a real debugging session; keep the reclaim.
     */
    if (g_fifo_head[a] == g_fifo_tail[a]) g_fifo_head[a] = g_fifo_tail[a] = 0;

    for (int i = 0; i < len; i++) {
        if (g_fifo_tail[a] >= FIFOSZ) { g_fifo_drops[a]++; continue; }
        g_fifo[a][g_fifo_tail[a]++] = buf[i];
    }
}

uint32_t i2cmock_fifo_drops(uint8_t addr)
{
    return g_fifo_drops[addr & (NADDR - 1)];
}

void i2cmock_fail_next_xfer(void)
{
    g_fail_next = 1;
}

void i2cmock_fail_write_to(uint8_t addr, int reg, int times)
{
    g_wfail_addr = addr & (NADDR - 1);
    g_wfail_reg  = reg;
    g_wfail_left = (reg < 0) ? 0 : times;
    g_wfail_hit  = 0;
}

void i2cmock_fail_all(int enable)
{
    g_fail_all = enable;
}

/* ── FIFO pop ────────────────────────────────────────────────────────────── */

static uint8_t fifo_pop(int a)
{
    if (g_fifo_head[a] < g_fifo_tail[a]) {
        uint8_t v = g_fifo[a][g_fifo_head[a]++];
        /* Drained: reclaim now rather than waiting for the next push, so a
         * reader that never pushes again still leaves the queue reusable. */
        if (g_fifo_head[a] == g_fifo_tail[a]) g_fifo_head[a] = g_fifo_tail[a] = 0;
        return v;
    }
    return 0;   /* underflow → zero-fill (a malformed-test guard, not a device
                 * behaviour tests should rely on) */
}

/* ── Device semantics, shared by both transports ─────────────────────────── */

/*
 * One byte out of the device at *reg_ptr, advancing the pointer unless the
 * read landed in a FIFO window.  Both the I2C and the SPI arm go through
 * here, which is the point: a driver exercised on either bus must see exactly
 * the same device, or the dual-transport tests would be comparing two
 * different mocks rather than two framings of one.
 */
static uint8_t dev_read(int a, int *reg_ptr, bool advance)
{
    if (g_fifo_reg[a] >= 0 && *reg_ptr >= g_fifo_reg[a] &&
        *reg_ptr <= g_fifo_reg_hi[a]) {
        /* FIFO port: pointer stays put, and a read anywhere in the window
         * pops.  Real data ports are several registers wide (the ST parts'
         * FIFO_DATA_OUT is 0x78-0x7E) and popping on a read of any of them is
         * what makes a blind register sweep destructive. */
        return fifo_pop(a);
    }

    uint8_t r = (uint8_t)*reg_ptr;
    uint8_t v = g_reg[a][r];
    g_reads[a][r]++;
    /* Self-clearing bits read back set once, then drop — the behaviour every
     * reset()/trigger poll is waiting for. */
    if (g_selfclear[a][r]) g_reg[a][r] &= (uint8_t)~g_selfclear[a][r];
    /* Live registers advance on their own, like a timestamp counter or a FIFO
     * level.  Stepping them on read is the only hook available here, and it is
     * enough: what the code under test sees is a register whose value moves
     * with no write in between. */
    if (g_live[a][r]) g_reg[a][r] += g_live[a][r];
    /* A part whose multi-byte reads need an explicit auto-increment bit
     * returns the SAME register over and over when the bit is absent. Modelling
     * that is what makes a forgotten bit a test failure rather than a silent
     * pass. */
    if (advance) (*reg_ptr)++;
    return v;
}

static void dev_write(int a, int *reg_ptr, uint8_t v)
{
    uint8_t r = (uint8_t)*reg_ptr;

    /* Armed for this register? Refuse it and leave the byte alone, so the
     * caller is not told a transfer failed that in fact landed. The dispatch
     * turns the flag into the -1/EIO the driver sees. */
    if (g_wfail_left != 0 && a == g_wfail_addr && r == (uint8_t)g_wfail_reg) {
        if (g_wfail_left > 0) g_wfail_left--;
        g_wfail_hit = 1;
        (*reg_ptr)++;
        return;
    }

    g_reg[a][r] = v;
    /* Order log. The alias below is a side effect of this one write, not a
     * write of its own, so only the addressed register is recorded. */
    if (g_wlog_n[a] < WLOGSZ) g_wlog[a][g_wlog_n[a]] = r;
    g_wlog_n[a]++;
    g_wlast[a] = r;
    /* A part that applies one write to two registers. Real: the MMC5983MA
     * lands a CTRL0 write in CTRL1 as well, which is how INT_en's bit 2 turns
     * into X-inhibit. Without this the mock cannot tell a correct write order
     * from a broken one, because both leave the same final bytes. */
    if (g_alias[a][r] >= 0) g_reg[a][g_alias[a][r]] = v;
    (*reg_ptr)++;

    /* Last, so the callback sees the register file the write has already
     * landed in and can respond to the part's new configuration. */
    if (g_on_write) g_on_write((uint8_t)a, r, v);
}

/* ── The backend ─────────────────────────────────────────────────────────── */

unsigned spimock_last_nlegs(void)      { return g_last_nlegs; }
uint8_t  spimock_last_bits(unsigned i) { return i < BUS_SPI_MAX_LEGS ? g_last_bits[i] : 0; }
uint32_t spimock_last_len(unsigned i)  { return i < BUS_SPI_MAX_LEGS ? g_last_len[i]  : 0; }

static int dispatch_i2c(const imud_bus_t *b, const uint8_t *tx, uint16_t txlen,
                        uint8_t *rx, uint16_t rxlen)
{
    int a = b->i2c_addr & (NADDR - 1);

    /* The register pointer persists across the legs of a single transaction,
     * so burst_read's [write reg][read data] pair works: the write sets the
     * pointer the following read consumes from. */
    int reg_ptr = 0;

    /* Write: first byte is the register pointer, the rest are stored with
     * auto-increment. */
    if (txlen >= 1) {
        reg_ptr = tx[0];
        for (uint16_t i = 1; i < txlen; i++)
            dev_write(a, &reg_ptr, tx[i]);
    }

    for (uint16_t i = 0; i < rxlen; i++)
        rx[i] = dev_read(a, &reg_ptr, true);

    return 0;
}

/*
 * A SPI message: the first leg's first byte is the command — top bit the
 * direction, the rest the register — and the data is either the remaining
 * bytes of that same leg (write) or the following leg's rx buffer (read).
 * That is exactly the 1-or-2 leg shape bus_io.h emits.
 *
 * There is no address on the wire, so which register file a transfer lands in
 * comes from spimock_bind(fd, ...): the chip select IS the addressing.
 */
static int dispatch_spi(int fd, const bus_spi_leg_t *legs, unsigned n)
{
    int a = spi_addr_for_fd(fd);
    if (a < 0) {
        /* An unbound descriptor means the test forgot spimock_bind; failing
         * loudly beats silently servicing register file 0. */
        errno = ENODEV;
        return -1;
    }

    /*
     * A 16-bit-word leg carries [address, value] as ONE native-endian word,
     * transmitted MSB first -- see spi_reg_write() for why the drivers send
     * writes that way on a Pi 5.  Unpack it to the byte order the wire sees,
     * or the mock reads the value as the address on a little-endian host and
     * every write lands in the wrong register.
     */
    uint8_t w16buf[2];
    const uint8_t *tx0;
    if (legs[0].bits == 16 && legs[0].tx) {
        uint16_t w;
        memcpy(&w, legs[0].tx, sizeof w);
        w16buf[0] = (uint8_t)(w >> 8);
        w16buf[1] = (uint8_t)(w & 0xFF);
        tx0 = w16buf;
    } else {
        tx0 = legs[0].tx;
    }
    if (!tx0 || legs[0].len < 1) { errno = EINVAL; return -1; }

    bool is_read = (tx0[0] & 0x80u) != 0;
    int reg_ptr  = tx0[0] & (uint8_t)~(0x80u | g_spi_inc_mask[a]);

    /*
     * On a part with an explicit auto-increment bit, the address only walks
     * when that bit is set — with it clear the same register is returned for
     * every byte (LIS3MDL DS9463 Rev 7 §5.2). Parts that increment on their own
     * are bound ALWAYS. The binding says which, rather than the mask implying
     * it: deriving "no mask means it always walks" from the same literal the
     * driver declares makes the driver's claim agree with itself.
     */
    bool advance;
    switch (g_spi_inc_mode[a]) {
    case SPIMOCK_INC_NEVER:  advance = false; break;
    case SPIMOCK_INC_ON_BIT: advance = (tx0[0] & g_spi_inc_mask[a]) != 0; break;
    default:                 advance = true;  break;
    }

    if (is_read) {
        /*
         * A one-byte read is a single 16-bit word (command in the high byte,
         * data clocked back in the low byte) -- see spi_burst_read() for why
         * the drivers send it that way on a Pi 5.  Modelling only the
         * two-transfer form would let that change pass untested.
         */
        if (n == 1) {
            if (!legs[0].rx || legs[0].len < 2) { errno = EINVAL; return -1; }
            uint16_t w = (uint16_t)(dev_read(a, &reg_ptr, advance) & 0xFF);
            memcpy(legs[0].rx, &w, sizeof w);
            return 0;
        }
        if (n < 2) { errno = EINVAL; return -1; }
        uint8_t *rx = legs[1].rx;
        if (!rx) { errno = EINVAL; return -1; }
        for (uint32_t i = 0; i < legs[1].len; i++)
            rx[i] = dev_read(a, &reg_ptr, advance);
    } else {
        for (uint32_t i = 1; i < legs[0].len; i++)
            dev_write(a, &reg_ptr, tx0[i]);
    }

    return 0;
}

/* Injected failures are armed per transfer, so both entry points share this. */
static int fail_armed(void)
{
    if (g_fail_next || g_fail_all) {
        g_fail_next = 0;
        errno = EIO;
        return 1;
    }
    return 0;
}

/* ── include/bus_backend.h ───────────────────────────────────────────────── */

/*
 * open() and close() are real: a suite hands the drivers a descriptor from a
 * node it can actually open (/dev/null serves), and spimock_bind keys the SPI
 * register file off that descriptor.
 */
int bus_be_open(const char *node)
{
    return open(node, O_RDWR | O_CLOEXEC);
}

void bus_be_close(int h)
{
    if (h >= 0) close(h);
}

/* Accepted and ignored, so bus_open() works unchanged against the mock. */
int bus_be_spi_setup(int h, uint8_t mode, uint8_t bits, uint32_t hz)
{
    (void)h; (void)mode; (void)bits; (void)hz;
    return 0;
}

int bus_be_i2c_xfer(const imud_bus_t *b, const uint8_t *tx, uint16_t txlen,
                    uint8_t *rx, uint16_t rxlen)
{
    if (fail_armed()) return -1;
    int rc = dispatch_i2c(b, tx, txlen, rx, rxlen);
    if (g_wfail_hit) { g_wfail_hit = 0; errno = EIO; return -1; }
    return rc;
}

int bus_be_spi_msg(const imud_bus_t *b, const bus_spi_leg_t *legs, unsigned n)
{
    if (n < 1 || n > BUS_SPI_MAX_LEGS) { errno = EINVAL; return -1; }

    /* Recorded before the failure check: the shape of a transfer is what the
     * driver asked for, whether or not the bus was armed to refuse it. */
    g_last_nlegs = n;
    for (unsigned i = 0; i < BUS_SPI_MAX_LEGS; i++) {
        g_last_bits[i] = i < n ? legs[i].bits : 0;
        g_last_len[i]  = i < n ? legs[i].len  : 0;
    }

    if (fail_armed()) return -1;
    int rc = dispatch_spi(b->fd, legs, n);
    if (g_wfail_hit) { g_wfail_hit = 0; errno = EIO; return -1; }
    return rc;
}
