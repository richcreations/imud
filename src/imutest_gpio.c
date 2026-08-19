/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imutest_gpio.c — interrupt-line edge counting for imud-imutest.
 *
 * The only translation unit in the tool that includes <gpiod.h>, so
 * everything else links without -lgpiod and test/test_imutest.c can supply
 * its own imt_gpio_count_edges() stub.
 *
 * The v1/v2 split mirrors src/imu.c: libgpiod 2.x replaced the line API
 * wholesale, and -DGPIOD_V2 is set by the Makefile when pkg-config reports it.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <gpiod.h>

#include "imutest.h"

#ifdef GPIOD_V2
typedef struct gpiod_line_request imt_line_t;
#else
typedef struct gpiod_line         imt_line_t;
#endif

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static imt_line_t *request_line(struct gpiod_chip *chip, unsigned int offset)
{
#ifdef GPIOD_V2
    struct gpiod_line_settings  *ls = gpiod_line_settings_new();
    struct gpiod_line_config    *lc = gpiod_line_config_new();
    struct gpiod_request_config *rc = gpiod_request_config_new();
    imt_line_t *req = NULL;
    if (!ls || !lc || !rc) goto out;
    gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(ls, GPIOD_LINE_EDGE_RISING);
    gpiod_line_config_add_line_settings(lc, &offset, 1, ls);
    gpiod_request_config_set_consumer(rc, "imud-imutest");
    req = gpiod_chip_request_lines(chip, rc, lc);
out:
    if (rc) gpiod_request_config_free(rc);
    if (lc) gpiod_line_config_free(lc);
    if (ls) gpiod_line_settings_free(ls);
    return req;
#else
    imt_line_t *line = gpiod_chip_get_line(chip, offset);
    if (!line) return NULL;
    if (gpiod_line_request_rising_edge_events(line, "imud-imutest") < 0)
        return NULL;
    return line;
#endif
}

static void release_line(imt_line_t *line)
{
#ifdef GPIOD_V2
    gpiod_line_request_release(line);
#else
    gpiod_line_release(line);
#endif
}

/* Wait up to timeout_ms for a rising edge, draining the event.
 * 1 = edge, 0 = timeout, -1 = error. */
static int wait_edge(imt_line_t *line, long timeout_ms)
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

int imt_gpio_count_edges(const char *chip_name, int gpio, long window_ms,
                         void (*drain)(void *), void *user,
                         imt_gpio_why_t *why, void (*prime)(void *))
{
    *why = IMT_GPIO_OK;

    if (gpio <= 0) { *why = IMT_GPIO_DISABLED; return -1; }

    char path[80];
    snprintf(path, sizeof path, "/dev/%s", chip_name);

#ifdef GPIOD_V2
    struct gpiod_chip *chip = gpiod_chip_open(path);
#else
    struct gpiod_chip *chip = gpiod_chip_open(path);
#endif
    if (!chip) { *why = IMT_GPIO_ENOCHIP; return -1; }

    imt_line_t *line = request_line(chip, (unsigned)gpio);
    if (!line) {
        /*
         * EBUSY here almost always means the daemon holds the line.  That is
         * a reason to skip the check, never to fail the driver, so the caller
         * needs to be able to tell it apart from a real fault.
         */
        *why = (errno == EBUSY) ? IMT_GPIO_EBUSY : IMT_GPIO_EIO;
        gpiod_chip_close(chip);
        return -1;
    }

    /*
     * Acknowledge once from INSIDE the window, before the first wait.
     *
     * A latched data-ready sits HIGH until it is acknowledged, and the only
     * acknowledge here happens after an edge -- so if the line went high in
     * the gap between the caller preparing the part and request_line()
     * returning, there is no rising edge left to see and nothing will ever
     * create one. The wait then runs out with zero edges against a part that
     * is working perfectly.
     *
     * Measured on an MMC5983MA: conversions and INT edges agree exactly at
     * every rate from 1 to 1204 Hz, while this function returned 0 edges at
     * 1204 Hz, 6 of 63 at 21 Hz, and the full count at 105 Hz. The gap is
     * fixed and the conversion period is not, which is precisely why it looked
     * rate-dependent -- at 1204 Hz a conversion always lands inside it.
     *
     * `prime` is separate from `drain` because this one is not a measurement:
     * counting its sample would report one more sample than there were edges.
     * NULL for a level-triggered watermark, which has nothing to re-arm.
     */
    if (prime) prime(user);

    int  edges = 0;
    long t_end = now_ms() + window_ms;

    while (now_ms() < t_end) {
        long remaining = t_end - now_ms();
        if (remaining <= 0) break;
        if (remaining > 200) remaining = 200;

        int r = wait_edge(line, remaining);
        if (r < 0) { *why = IMT_GPIO_EIO; break; }
        if (r == 0) continue;

        edges++;
        /*
         * Drain after every edge.  A watermark interrupt stays asserted until
         * the FIFO drops below the threshold, so without this the line would
         * yield exactly one edge and the whole measurement would be wrong.
         */
        if (drain) drain(user);
    }

    release_line(line);
    gpiod_chip_close(chip);

    if (edges == 0 && *why == IMT_GPIO_OK) *why = IMT_GPIO_NOEDGES;
    return edges;
}
