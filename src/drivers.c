
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

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
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

/* Is `v` in a 0-terminated table of at most 8?  If not, list the table. */
static bool fs_in_table(const int tab[8], int v, char *list, size_t listsz)
{
    size_t pos = 0;
    list[0] = '\0';
    for (int i = 0; i < 8 && tab[i]; i++) {
        if (tab[i] == v) return true;
        int n = snprintf(list + pos, listsz - pos, "%s%d", i ? ", " : "", tab[i]);
        if (n < 0 || (size_t)n >= listsz - pos) break;
        pos += (size_t)n;
    }
    return false;
}

int imu_fs_check(const imu_ops_t *ops, int accel_g, int gyro_dps,
                 char *why, size_t whysz)
{
    char list[64];
    if (!fs_in_table(ops->supported_accel_g, accel_g, list, sizeof list)) {
        snprintf(why, whysz, "%s has no ±%d g accelerometer range: accel_g "
                 "must be one of %s", ops->name, accel_g, list);
        return -1;
    }
    if (!fs_in_table(ops->supported_gyro_dps, gyro_dps, list, sizeof list)) {
        snprintf(why, whysz, "%s has no %d dps gyro range: gyro_dps must be "
                 "one of %s", ops->name, gyro_dps, list);
        return -1;
    }
    return 0;
}
