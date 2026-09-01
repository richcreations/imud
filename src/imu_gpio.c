/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu_gpio.c — the libgpiod backend for include/imu_gpio.h.
 *
 * This is the only file in the tree that includes <gpiod.h>, which is what
 * lets src/imu_gpio_null.c stand in for it on a host that has no libgpiod:
 * the reader threads in src/imu.c hold an opaque imu_gpio_line_t * and branch
 * on whether it is NULL, so a backend that never returns one leaves them on
 * the rate-sized timer they already fall back to.
 *
 * It carries the v1/v2 split because that is the same kind of question and
 * belongs in the same place — v1 exposes struct gpiod_line *, v2 struct
 * gpiod_line_request *, and GPIOD_V2 selects between them.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <gpiod.h>

#include "imu_gpio.h"

/* libgpiod v1 exposes struct gpiod_line *; v2 uses struct gpiod_line_request *. */
#ifdef GPIOD_V2
typedef struct gpiod_line_request gpio_line_h;
#else
typedef struct gpiod_line         gpio_line_h;
#endif

/* ── libgpiod v1/v2, behind three static helpers ─────────────────────────── */

/*
 * wait_gpio_edge — wait up to timeout_ms ms for a rising-edge event.
 * Drains the event so the next call sees a fresh edge.
 * Returns 1 on event, 0 on timeout, -1 on error.
 */
static int wait_gpio_edge(gpio_line_h *line, long timeout_ms)
{
#ifdef GPIOD_V2
    struct gpiod_edge_event_buffer *evbuf = gpiod_edge_event_buffer_new(1);
    if (!evbuf) return -1;
    int r = gpiod_line_request_wait_edge_events(line,
                (int64_t)timeout_ms * 1000000LL);
    if (r == 1)
        gpiod_line_request_read_edge_events(line, evbuf, 1);
    gpiod_edge_event_buffer_free(evbuf);
    return r;
#else
    struct timespec ts = {
        .tv_sec  = timeout_ms / 1000,
        .tv_nsec = (timeout_ms % 1000) * 1000000L,
    };
    int r = gpiod_line_event_wait(line, &ts);
    if (r == 1) {
        struct gpiod_line_event ev;
        gpiod_line_event_read(line, &ev);
    }
    return r;
#endif
}

/*
 * open_gpio_line — request one GPIO line for rising-edge detection.
 * Returns an opaque handle on success, NULL on error.
 */
static gpio_line_h *open_gpio_line_as(struct gpiod_chip *chip,
                                      unsigned int offset,
                                      const char *consumer)
{
#ifdef GPIOD_V2
    struct gpiod_line_settings  *ls = gpiod_line_settings_new();
    struct gpiod_line_config    *lc = gpiod_line_config_new();
    struct gpiod_request_config *rc = gpiod_request_config_new();
    if (!ls || !lc || !rc) goto fail;
    gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(ls, GPIOD_LINE_EDGE_RISING);
    gpiod_line_config_add_line_settings(lc, &offset, 1, ls);
    gpiod_request_config_set_consumer(rc, consumer);
    gpio_line_h *req = gpiod_chip_request_lines(chip, rc, lc);
    gpiod_request_config_free(rc);
    gpiod_line_config_free(lc);
    gpiod_line_settings_free(ls);
    return req;
fail:
    if (rc) gpiod_request_config_free(rc);
    if (lc) gpiod_line_config_free(lc);
    if (ls) gpiod_line_settings_free(ls);
    return NULL;
#else
    gpio_line_h *line = gpiod_chip_get_line(chip, offset);
    if (!line) return NULL;
    if (gpiod_line_request_rising_edge_events(line, consumer) < 0)
        return NULL;
    return line;
#endif
}

static void release_gpio_line(gpio_line_h *line)
{
#ifdef GPIOD_V2
    gpiod_line_request_release(line);
#else
    gpiod_line_release(line);
#endif
}

/* ── The published entry points — see include/imu_gpio.h ─────────────────── */

/*
 * The daemon's reader threads and imud-imutest both wait on an interrupt by
 * calling these, rather than either carrying a second copy of the libgpiod
 * v1/v2 split.  Carrying one means every way the copy differed from the daemon
 * showed up as a defect reported against the driver.
 *
 * The handle owns its chip because libgpiod v1 hands back a line that BORROWS
 * the chip -- closing the chip there is a use-after-free -- while v2's request
 * owns what it needs.  Keeping both in one allocation makes that difference
 * invisible to a caller and impossible to get wrong at the call site.
 */
struct imu_gpio_line {
    struct gpiod_chip *chip;
    gpio_line_h       *line;
};

imu_gpio_line_t *imu_gpio_open(const char *chip_name, unsigned int offset,
                               const char *consumer)
{
    char path[80];
    snprintf(path, sizeof path, "/dev/%s", chip_name);

    imu_gpio_line_t *h = calloc(1, sizeof *h);
    if (!h) return NULL;

    h->chip = gpiod_chip_open(path);
    if (!h->chip) { int e = errno; free(h); errno = e; return NULL; }

    h->line = open_gpio_line_as(h->chip, offset, consumer);
    if (!h->line) {
        int e = errno;
        gpiod_chip_close(h->chip);
        free(h);
        errno = e;
        return NULL;
    }
    return h;
}

int imu_gpio_wait_edge(imu_gpio_line_t *h, long timeout_ms)
{
    return h ? wait_gpio_edge(h->line, timeout_ms) : -1;
}

void imu_gpio_close(imu_gpio_line_t *h)
{
    if (!h) return;
    release_gpio_line(h->line);
    gpiod_chip_close(h->chip);
    free(h);
}
