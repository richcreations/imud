/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * i2c_mock.c — __wrap_ioctl implementation of the register-map I2C device.
 * See i2c_mock.h.  Linux/GNU-ld only (relies on -Wl,--wrap=ioctl and
 * <linux/i2c.h>).
 */

#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "i2c_mock.h"

#define NADDR   128     /* 7-bit I2C address space */
#define REGSZ   256     /* one 8-bit register file per address */
#define FIFOSZ  4096    /* per-address FIFO byte queue */

static uint8_t g_reg[NADDR][REGSZ];
static uint8_t g_fifo[NADDR][FIFOSZ];
static int     g_fifo_head[NADDR];   /* next byte to pop */
static int     g_fifo_tail[NADDR];   /* next free slot */
static int     g_fifo_reg[NADDR];    /* declared FIFO register, or -1 */
static int     g_fail_next;          /* one-shot ioctl failure */

/* ── Public harness API ──────────────────────────────────────────────────── */

void i2cmock_reset(void)
{
    memset(g_reg, 0, sizeof g_reg);
    memset(g_fifo, 0, sizeof g_fifo);
    memset(g_fifo_head, 0, sizeof g_fifo_head);
    memset(g_fifo_tail, 0, sizeof g_fifo_tail);
    for (int i = 0; i < NADDR; i++) g_fifo_reg[i] = -1;
    g_fail_next = 0;
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
    g_fifo_reg[addr & (NADDR - 1)] = reg;
}

void i2cmock_fifo_push(uint8_t addr, const uint8_t *buf, int len)
{
    int a = addr & (NADDR - 1);
    for (int i = 0; i < len && g_fifo_tail[a] < FIFOSZ; i++)
        g_fifo[a][g_fifo_tail[a]++] = buf[i];
}

void i2cmock_fail_next_ioctl(void)
{
    g_fail_next = 1;
}

/* ── FIFO pop ────────────────────────────────────────────────────────────── */

static uint8_t fifo_pop(int a)
{
    if (g_fifo_head[a] < g_fifo_tail[a])
        return g_fifo[a][g_fifo_head[a]++];
    return 0;   /* underflow → zero-fill (a malformed-test guard, not a device
                 * behaviour tests should rely on) */
}

/* ── Wrapped ioctl ───────────────────────────────────────────────────────── */

/* Shared dispatch for the wrappers below. */
static int mock_dispatch(unsigned long request, void *arg)
{
    if (g_fail_next) {
        g_fail_next = 0;
        errno = EIO;
        return -1;
    }

    /* Only I2C_RDWR is modelled; anything else is a test bug. */
    if (request != I2C_RDWR) {
        errno = EINVAL;
        return -1;
    }

    struct i2c_rdwr_ioctl_data *d = arg;

    /* The register pointer persists across the messages of a single transfer,
     * so burst_read's [write reg][read data] pair works: the write sets the
     * pointer the following read consumes from. */
    int reg_ptr = 0;

    for (uint32_t m = 0; m < d->nmsgs; m++) {
        struct i2c_msg *msg = &d->msgs[m];
        int a = msg->addr & (NADDR - 1);

        if (msg->flags & I2C_M_RD) {
            for (uint16_t i = 0; i < msg->len; i++) {
                if (g_fifo_reg[a] >= 0 && reg_ptr == g_fifo_reg[a]) {
                    msg->buf[i] = fifo_pop(a);   /* FIFO port: pointer stays put */
                } else {
                    msg->buf[i] = g_reg[a][(uint8_t)reg_ptr];
                    reg_ptr++;
                }
            }
        } else {
            /* Write: first byte is the register pointer, the rest are stored
             * with auto-increment. */
            if (msg->len >= 1) {
                reg_ptr = msg->buf[0];
                for (uint16_t i = 1; i < msg->len; i++)
                    g_reg[a][(uint8_t)reg_ptr++] = msg->buf[i];
            }
        }
    }

    return 0;
}

/* The drivers call ioctl(fd, I2C_RDWR, &xfer).  On a 32-bit glibc built with
 * -D_TIME_BITS=64 (Debian armhf), <sys/ioctl.h> redirects every ioctl() call to
 * the symbol __ioctl_time64, so --wrap=ioctl alone would miss them and the real
 * syscall would run against the test's dummy fd (EBADF).  Wrap both symbols so
 * the mock intercepts on every architecture; the time64 wrapper is dead code on
 * 64-bit builds, where nothing references __ioctl_time64. */
int __wrap_ioctl(int fd, unsigned long request, ...)
{
    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    (void)fd;
    return mock_dispatch(request, arg);
}

int __wrap___ioctl_time64(int fd, unsigned long request, ...)
{
    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    (void)fd;
    return mock_dispatch(request, arg);
}
