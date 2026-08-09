/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 *
 * rate_ladder.h — every sample rate this project claims to support.
 *
 * The union of supported_odr_hz across all registered drivers: 29 IMU rates
 * spanning 12 Hz to 32 kHz, and 13 magnetometer rates from 1 Hz to 1 kHz.  Any
 * of the 377 pairings is a configuration a user can select and the daemon will
 * accept.
 *
 * WHY THIS IS A HAND-MAINTAINED LIST, and why that is safe.  The natural way to
 * walk every rate is to iterate the registry itself, but the registries in
 * src/drivers.c are static with no enumeration API, and every driver .c pulls in
 * <linux/i2c.h> through i2c_io.h — so a test that links them cannot build on a
 * non-Linux dev host.  test_fusion must build everywhere, since it is where the
 * filter's rate behaviour is exercised.
 *
 * So the list is duplicated here and CROSS-CHECKED against the real registry by
 * test_drivers_registry, which does link the drivers and does run in CI.  Adding
 * a driver rung without adding it here fails there, by name.  The duplication is
 * guarded rather than trusted; do not "tidy" the guard away.
 *
 * Kept ascending, both for readability and because the cross-check reports the
 * first missing value it finds.
 */
#ifndef IMUD_TEST_RATE_LADDER_H
#define IMUD_TEST_RATE_LADDER_H

/*
 * IMU rates, Hz.
 *   12,26,52,104,208,416,833,1660          sim
 *   +3332,6664                             ism330dhcx, lsm6dso, lsm6dsox
 *   12,25,50,100,200,500,1000,2000,        icm42688p
 *     4000,8000,16000,32000
 *   225,281,375,562,1125                   icm20948
 *   100,125,200,250,333,500,1000           mpu9250, mpu9255
 */
static const int rate_ladder_imu[] = {
    12, 25, 26, 50, 52, 100, 104, 125, 200, 208, 225, 250, 281, 333,
    375, 416, 500, 562, 833, 1000, 1125, 1660, 2000, 3332, 4000, 6664, 8000,
    16000, 32000
};
#define RATE_LADDER_IMU_N \
    ((int)(sizeof rate_ladder_imu / sizeof rate_ladder_imu[0]))

/*
 * Magnetometer rates, Hz.
 *   1,10,20,50,100,200,1000       mmc5983ma, sim mag
 *   10,20,50,100                  ak09916, lis2mdl
 *   1,2,5,10,20,40,80,155         lis3mdl
 *   8,100                         ak8963
 */
static const int rate_ladder_mag[] = {
    1, 2, 5, 8, 10, 20, 40, 50, 80, 100, 155, 200, 1000
};
#define RATE_LADDER_MAG_N \
    ((int)(sizeof rate_ladder_mag / sizeof rate_ladder_mag[0]))

#endif /* IMUD_TEST_RATE_LADDER_H */
