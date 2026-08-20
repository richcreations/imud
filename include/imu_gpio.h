/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu_gpio.h — the daemon's edge-wait, exposed so nothing has to reimplement it.
 *
 * The implementation lives in src/imu.c, beside the reader threads that use it.
 * It is declared here rather than kept static because imud-imutest must wait on
 * an interrupt exactly the way the daemon does, and the alternative -- a second
 * copy of the libgpiod v1/v2 split -- is what src/imutest_gpio.c used to be.
 *
 * That duplication was not free. imutest paced its own reads on a 5 ms timer
 * where the daemon waits on the watermark, and every difference between the two
 * showed up as a defect reported against the driver: chip_ts reversals the
 * daemon never sees, a DRDY rate of 0 Hz on a part feeding the daemon 105 Hz,
 * and an interrupt line called unwired because the watermark could not fill
 * inside a window the daemon does not have. A tool that measures a path it
 * reimplements is measuring its own reimplementation.
 *
 * The line handle is opaque so callers need no <gpiod.h>: the concrete type is
 * struct gpiod_line_request (v2) or struct gpiod_line (v1), and which one it is
 * is a build-time question that belongs in one file.
 */
#ifndef IMUD_IMU_GPIO_H
#define IMUD_IMU_GPIO_H

typedef struct imu_gpio_line imu_gpio_line_t;

/*
 * Request one line on `chip_name` (a bare name like "gpiochip4", not a path)
 * for rising-edge detection. `consumer` is what shows in `gpioinfo`.
 * Returns NULL on failure, leaving errno set — EBUSY means someone else holds
 * the line, which for a diagnostic is a reason to skip rather than to fail.
 */
imu_gpio_line_t *imu_gpio_open(const char *chip_name, unsigned int offset,
                               const char *consumer);

/*
 * Wait up to timeout_ms for a rising edge, consuming the event so the next
 * call sees a fresh one. Returns 1 on an edge, 0 on timeout, -1 on error.
 */
int imu_gpio_wait_edge(imu_gpio_line_t *line, long timeout_ms);

void imu_gpio_close(imu_gpio_line_t *line);

#endif /* IMUD_IMU_GPIO_H */
