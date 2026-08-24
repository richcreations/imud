/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imutest_gpio.c — edge counting for imud-imutest, on the DAEMON's edge wait.
 *
 * This file used to carry its own libgpiod v1/v2 split, a near-copy of the one
 * in src/imu.c. The copy is gone: imu_gpio_open/wait_edge/close come from
 * src/imu.c, which is linked into imud-imutest for exactly this reason.
 *
 * The duplication was not cosmetic. A tool that reimplements the path it is
 * measuring reports differences between the two copies as defects in the
 * driver, and this one did: chip_ts reversals the daemon never sees, a DRDY
 * rate of 0 Hz on a part feeding the daemon 105 Hz, and an interrupt line
 * called unwired because a window the daemon does not have was too short.
 *
 * What remains here is the counting policy, which is imutest's own business:
 * how long to watch, when to drain, and how to tell a busy line from a broken
 * one. The waiting itself is the daemon's.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "imutest.h"
#include "imu_gpio.h"
#include "imu_math.h"

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

int imt_gpio_count_edges(const char *chip_name, int gpio, long window_ms,
                         void (*drain)(void *), void *user,
                         imt_gpio_why_t *why, void (*prime)(void *),
                         int odr_hz)
{
    *why = IMT_GPIO_OK;

    if (gpio <= 0) { *why = IMT_GPIO_DISABLED; return -1; }

    imu_gpio_line_t *line = imu_gpio_open(chip_name, (unsigned)gpio,
                                          "imud-imutest");
    if (!line) {
        /*
         * EBUSY almost always means the daemon holds the line. That is a
         * reason to skip the check, never to fail the driver, so the caller
         * needs to tell it apart from a real fault.
         */
        *why = (errno == EBUSY)  ? IMT_GPIO_EBUSY
             : (errno == ENOENT) ? IMT_GPIO_ENOCHIP
                                 : IMT_GPIO_EIO;
        return -1;
    }

    /*
     * Acknowledge once from INSIDE the window, before the first wait.
     *
     * A latched data-ready sits HIGH until it is acknowledged, and the only
     * acknowledge here happens after an edge -- so if the line went high in
     * the gap between the caller preparing the part and the line being
     * requested, there is no rising edge left to see and nothing will ever
     * create one. The wait then runs out with zero edges against a part that
     * is working perfectly.
     *
     * Measured on an MMC5983MA: conversions and INT edges agree exactly at
     * every rate from 1 to 1204 Hz, while this function returned 0 edges at
     * 1204 Hz, 6 of 63 at 21 Hz, and the full count at 105 Hz. The gap is
     * fixed and the conversion period is not, which is why it looked
     * rate-dependent -- at 1204 Hz a conversion always lands inside it.
     *
     * `prime` is separate from `drain` because this one is not a measurement:
     * counting its sample would report one more sample than there were edges.
     * NULL for a level-triggered watermark, which has nothing to re-arm.
     */
    if (prime) prime(user);

    int  edges = 0, timeouts = 0;
    long t_end = now_ms() + window_ms;

    /*
     * The daemon's own recovery interval for this rate, from the same helper
     * its reader threads use -- not a constant chosen here.  That is the whole
     * point of this file linking src/imu.c: a number the tool picks for itself
     * is a number that can drift from what the daemon does.
     */
    /* depth 1 and one sample of grace: this counter is measuring EDGES, not
     * draining a batch, so it waits for a single conversion plus slack.  A
     * caller with a level watermark (the IMU FIFO) passes no prime and is
     * handled by `latched` below. */
    const long fallback_ms = imu_int_fallback_ms(odr_hz, 1, 1);
    const int  latched     = (prime != NULL);

    while (now_ms() < t_end) {
        long remaining = t_end - now_ms();
        if (remaining <= 0) break;
        if (remaining > 200) remaining = 200;

        /* Only a latched line needs the short cycle -- see below. */
        if (latched && remaining > fallback_ms) remaining = fallback_ms;

        int r = imu_gpio_wait_edge(line, remaining);
        if (r < 0) { *why = IMT_GPIO_EIO; break; }

        /*
         * Drain on an edge AND on a timeout, which is what the daemon does.
         *
         * On an edge, because a watermark interrupt stays asserted until the
         * FIFO drops below the threshold: without this the line would yield
         * exactly one edge and the measurement would be meaningless.
         *
         * On a TIMEOUT, because some parts stop advancing on a quiet bus. The
         * MMC5983MA does below about 50 Hz: acknowledged, the line goes low;
         * the status bit then sets and the line goes high, but no edge is
         * delivered until something touches the bus, and the queued edge then
         * arrives on the very next call. Waiting purely on edges therefore
         * measured 2 edges in 3 s at 20 Hz on a part the daemon reads at 21 Hz.
         *
         * The daemon's mag reader waits 20 ms and reads anyway; its IMU reader
         * waits 10 ms and does the same. That fallback is load-bearing, not a
         * belt-and-braces, and a tool that leaves it out is not measuring the
         * daemon's path.
         */
        if (r == 1) edges++;
        else        timeouts++;

        /*
         * Drain on an edge always; on a TIMEOUT only for a latched line.
         *
         * The two interrupt shapes need opposite treatment, and doing the same
         * thing to both breaks one of them.
         *
         * A LATCHED data-ready (the magnetometer) asserts on
         * conversion-complete and is re-armed only by the acknowledge a read
         * performs, so it yields one rising edge per acknowledge and a missed
         * edge is unrecoverable without reading anyway. Waiting LONGER there
         * produces fewer edges, not later ones: 64 edges in 3 s at 20 Hz with
         * a 20 ms fallback against 36 with a 95 ms one.
         *
         * A LEVEL watermark (the IMU FIFO) is the opposite. It asserts while
         * the FIFO holds at least fifo_wm sets and deasserts when a drain
         * empties it, so draining on a timeout keeps the FIFO permanently
         * below the threshold and the watermark never asserts at all --
         * measured as 0 edges on a line that was working perfectly.
         *
         * `latched` is inferred from `prime`: a caller that has something to
         * re-arm passes one, and a caller with a level watermark does not.
         */
        if (drain && (r == 1 || latched)) drain(user);
    }

    imu_gpio_close(line);

    (void)timeouts;
    if (edges == 0 && *why == IMT_GPIO_OK) *why = IMT_GPIO_NOEDGES;
    return edges;
}
