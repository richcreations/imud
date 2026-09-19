
/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * drivers.c — driver registry
 *
 * Add a forward declaration and pointer entry here when adding a new chip.
 * No other files need to change.
 */

#include <stddef.h>
#include <string.h>
#include "drivers.h"

extern const imu_ops_t ism330dhcx_ops;
extern const imu_ops_t icm20948_ops;
extern const imu_ops_t icm42688p_ops;
extern const imu_ops_t lsm6dso_ops;
extern const imu_ops_t lsm6dsox_ops;
extern const imu_ops_t mpu6500_ops;
extern const imu_ops_t mpu9250_ops;
extern const imu_ops_t mpu9255_ops;
extern const imu_ops_t sim_imu_ops;

extern const mag_ops_t mmc5983ma_ops;
extern const mag_ops_t ak09916_ops;
extern const mag_ops_t ak8963_ops;
extern const mag_ops_t lis3mdl_ops;
extern const mag_ops_t lis2mdl_ops;
extern const mag_ops_t rm3100_ops;
extern const mag_ops_t sim_mag_ops;

static const imu_ops_t *imu_registry[] = {
    &ism330dhcx_ops,
    &icm20948_ops,
    &icm42688p_ops,
    &lsm6dso_ops,
    &lsm6dsox_ops,
    &mpu6500_ops,
    &mpu9250_ops,
    &mpu9255_ops,
    &sim_imu_ops,
    NULL,
};

static const mag_ops_t *mag_registry[] = {
    &mmc5983ma_ops,
    &ak09916_ops,
    &ak8963_ops,
    &lis3mdl_ops,
    &lis2mdl_ops,
    &rm3100_ops,
    &sim_mag_ops,
    NULL,
};

const imu_ops_t *imu_driver_find(const char *name)
{
    for (int i = 0; imu_registry[i]; i++)
        if (strcmp(imu_registry[i]->name, name) == 0)
            return imu_registry[i];
    return NULL;
}

const mag_ops_t *mag_driver_find(const char *name)
{
    for (int i = 0; mag_registry[i]; i++)
        if (strcmp(mag_registry[i]->name, name) == 0)
            return mag_registry[i];
    return NULL;
}

/*
 * The state is allocated even when the transport did not open, because
 * bus_open() leaves a usable closed handle and imud-cal runs the sim driver
 * on one — sim keeps state and never looks at the bus, so a NULL bus->drv
 * there would be a null dereference on a path that works today.
 */
static int drv_open(imud_bus_t *b, const bus_spec_t *spec,
                    const bus_caps_t *caps, size_t bytes, const void *tmpl,
                    const char *who)
{
    int rc = bus_open(b, spec, caps, who);
    if (bus_drv_alloc(b, bytes, tmpl) < 0) {
        bus_close(b);
        return -1;
    }
    return rc;
}

int imu_bus_open(imud_bus_t *b, const bus_spec_t *spec, const imu_ops_t *ops,
                 const char *who)
{
    return drv_open(b, spec, &ops->bus_caps, ops->state_bytes,
                    ops->state_init, who);
}

int mag_bus_open(imud_bus_t *b, const bus_spec_t *spec, const mag_ops_t *ops,
                 const char *who)
{
    return drv_open(b, spec, &ops->bus_caps, ops->state_bytes,
                    ops->state_init, who);
}
