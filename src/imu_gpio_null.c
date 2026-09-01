/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu_gpio_null.c — include/imu_gpio.h with no libgpiod behind it.
 *
 * Selected in place of src/imu_gpio.c on a host without libgpiod, which is
 * what lets imud link where the library does not exist.  Every open fails with
 * ENOSYS, so the reader threads in src/imu.c hold a NULL line and take the
 * rate-sized timer they already fall back to when no interrupt is wired.
 *
 * ENOSYS rather than ENODEV or ENOENT: those two say a line was looked for and
 * not found, which a caller may reasonably retry or report against the wiring.
 * This is the build having no way to look at all, and callers separate the two
 * -- imud-imutest reports it as IMT_GPIO_UNSUPPORTED rather than as a fault in
 * the part under test.
 */

#include <errno.h>
#include <stddef.h>

#include "imu_gpio.h"

imu_gpio_line_t *imu_gpio_open(const char *chip_name, unsigned int offset,
                               const char *consumer)
{
    (void)chip_name; (void)offset; (void)consumer;
    errno = ENOSYS;
    return NULL;
}

int imu_gpio_wait_edge(imu_gpio_line_t *line, long timeout_ms)
{
    (void)line; (void)timeout_ms;
    errno = ENOSYS;
    return -1;
}

void imu_gpio_close(imu_gpio_line_t *line)
{
    (void)line;
}
