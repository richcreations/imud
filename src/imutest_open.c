/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imutest_open.c — imt_run(): resolve driver names, open the bus, hand off.
 *
 * Split out of src/imutest.c for the same reason src/imutest_gpio.c is: it is
 * the only part of the core that touches the driver registry, and linking
 * drivers.c would drag every driver object into anything that wanted the
 * checks.  test/test_imutest.c calls imt_run_ops() with ops pointers it
 * supplies itself and never links this file.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "imutest.h"

int imt_run(const imud_config_t *cfg, const imt_opts_t *opts,
            imt_report_t *r, char *errbuf, size_t errbufsz)
{
    const imu_ops_t *imu = imu_driver_find(cfg->imu_driver);
    if (!imu) {
        snprintf(errbuf, errbufsz, "unknown IMU driver '%s'", cfg->imu_driver);
        return -1;
    }

    /* "none" is how an IMU-only breakout is described; several parts under
     * test ship without a compass and the daemon has no notion of that. */
    const mag_ops_t *mag = NULL;
    if (cfg->mag_driver[0] && strcmp(cfg->mag_driver, "none") != 0) {
        mag = mag_driver_find(cfg->mag_driver);
        if (!mag) {
            snprintf(errbuf, errbufsz, "unknown mag driver '%s'",
                     cfg->mag_driver);
            return -1;
        }
    }

    int fd = open(cfg->i2c_bus, O_RDWR);
    if (fd < 0) {
        snprintf(errbuf, errbufsz, "cannot open %s: %s",
                 cfg->i2c_bus, strerror(errno));
        return -1;
    }

    int rc = imt_run_ops(fd, imu, mag, cfg, opts, r, errbuf, errbufsz);
    close(fd);
    return rc;
}
