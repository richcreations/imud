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
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "imutest.h"

/*
 * Average |B| over a few completed measurements.  Returns -1 if the part never
 * produced one inside the window; read() returning 1 is "not ready yet".
 */
static int mag_field_mean(const mag_ops_t *mag, const imud_bus_t *bus,
                          double *out_uT)
{
    double sum = 0.0;
    int got = 0;
    for (int i = 0; i < 200 && got < 16; i++) {
        mag_sample_t m;
        int rc = mag->read(bus, &m);
        if (rc < 0) return -1;
        if (rc == 0) {
            sum += sqrt((double)m.field[0] * m.field[0] +
                        (double)m.field[1] * m.field[1] +
                        (double)m.field[2] * m.field[2]);
            got++;
        }
        usleep(20000);
    }
    if (got == 0) return -1;
    *out_uT = sum / got;
    return 0;
}

/*
 * imt_degauss — pulse the coil and report the field either side of it.
 *
 * A recovery action, not a check: it writes to the part and prints, and does
 * not produce a report.  reset() deliberately does not do this, so a bridge
 * left magnetised by a strong external field stays visible as a bad reading
 * rather than being silently repaired on every start.
 *
 * RESET then SET, which is the order that leaves the bridge in the polarity
 * the daemon runs it in.  One pass only: if a single pulse does not clear it,
 * that is a finding rather than something to repeat until it looks better.
 */
int imt_degauss(const imud_config_t *cfg, char *errbuf, size_t errbufsz)
{
    if (!cfg->mag_driver[0] || strcmp(cfg->mag_driver, "none") == 0) {
        snprintf(errbuf, errbufsz, "no magnetometer configured");
        return -1;
    }
    const mag_ops_t *mag = mag_driver_find(cfg->mag_driver);
    if (!mag) {
        snprintf(errbuf, errbufsz, "unknown mag driver '%s'", cfg->mag_driver);
        return -1;
    }
    if (!mag->degauss) {
        snprintf(errbuf, errbufsz,
                 "%s has no degauss coil to pulse", cfg->mag_driver);
        return -1;
    }

    imud_bus_t mbus;
    bus_spec_t spec;
    config_mag_bus_spec(cfg, &spec);
    if (bus_open(&mbus, &spec, &mag->bus_caps, "mag") < 0) {
        snprintf(errbuf, errbufsz, "cannot open %s for the magnetometer",
                 spec.node);
        return -1;
    }

    int rc = -1;
    double before = 0.0, after_reset = 0.0, after_set = 0.0;

    if (mag->probe(&mbus) < 0) {
        snprintf(errbuf, errbufsz, "%s did not answer probe()", cfg->mag_driver);
        goto out;
    }
    if (mag->reset(&mbus) < 0 || mag->init(&mbus, &(mag_cfg_t){
            .odr_mhz = cfg->mag_odr_mhz, .set_period_s = 0.0f,
            .int_driven = false }) < 0) {
        snprintf(errbuf, errbufsz, "%s did not initialise", cfg->mag_driver);
        goto out;
    }

    printf("imud-imutest --degauss — %s on %s\n\n", cfg->mag_driver, spec.node);

    if (mag_field_mean(mag, &mbus, &before) < 0) {
        snprintf(errbuf, errbufsz, "%s produced no measurement", cfg->mag_driver);
        goto out;
    }
    printf("  before        |B| = %7.2f uT\n", before);

    if (mag->degauss(&mbus, MAG_DEGAUSS_RESET) < 0) {
        snprintf(errbuf, errbufsz, "RESET pulse failed");
        goto out;
    }
    if (mag_field_mean(mag, &mbus, &after_reset) == 0)
        printf("  after RESET   |B| = %7.2f uT\n", after_reset);

    if (mag->degauss(&mbus, MAG_DEGAUSS_SET) < 0) {
        snprintf(errbuf, errbufsz, "SET pulse failed");
        goto out;
    }
    if (mag_field_mean(mag, &mbus, &after_set) < 0) {
        snprintf(errbuf, errbufsz, "no measurement after the SET pulse");
        goto out;
    }
    printf("  after SET     |B| = %7.2f uT\n\n", after_set);

    /*
     * Earth's field is 25-65 uT, and an installation adds hard iron on top, so
     * a plausible reading is a band rather than a number.  What this is really
     * separating is a magnitude in that neighbourhood from a saturated bridge,
     * which sits in the hundreds and does not move when pulsed.
     */
    if (after_set >= 20.0 && after_set <= 120.0) {
        printf("The bridge reads a plausible field. Nothing further to do.\n");
        rc = 0;
    } else {
        printf("Still %.2f uT after a full RESET/SET pass, which is outside\n"
               "anything an installation explains. The coil is not the fault:\n"
               "check for a magnet or motor next to the sensor, then re-run.\n",
               after_set);
        rc = 2;
    }

out:
    bus_close(&mbus);
    return rc;
}

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

    /* One handle per sensor. bus_open logs the errno detail; errbuf carries
     * the summary imud-imutest prints, so the operator sees both which sensor
     * and why. */
    imud_bus_t ibus, mbus;
    bus_spec_t spec;

    config_imu_bus_spec(cfg, &spec);
    if (bus_open(&ibus, &spec, &imu->bus_caps, "imu") < 0) {
        snprintf(errbuf, errbufsz, "cannot open %s for the IMU", spec.node);
        return -1;
    }

    /* An IMU-only board leaves mbus closed: imt_run_ops never touches it when
     * mag is NULL. */
    bus_init(&mbus);
    if (mag) {
        config_mag_bus_spec(cfg, &spec);
        if (bus_open(&mbus, &spec, &mag->bus_caps, "mag") < 0) {
            snprintf(errbuf, errbufsz, "cannot open %s for the magnetometer",
                     spec.node);
            bus_close(&ibus);
            return -1;
        }
    }

    int rc = imt_run_ops(&ibus, &mbus, imu, mag, cfg, opts, r, errbuf, errbufsz);
    bus_close(&ibus);
    bus_close(&mbus);
    return rc;
}
