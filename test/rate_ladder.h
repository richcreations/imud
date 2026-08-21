/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 *
 * rate_ladder.h — every sample rate this project claims to support.
 *
 * The union of supported_odr_mhz across all registered drivers, in MILLI-HERTZ:
 * 37 IMU rates spanning 12 Hz to 32 kHz, and 24 magnetometer rates from 1 Hz to
 * 1 kHz.  Any of the 888 pairings is a configuration a user can select and the
 * daemon will accept.
 *
 * MILLI-HERTZ because whole Hz cannot hold the rates the parts really run —
 * the TDK 12.5 Hz rung, the ST chain's 13.016 Hz bottom, icm20948's 281.25 and
 * 562.5, mpu925x's 1000/7.  See the unit note at the top of include/drivers.h.
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
 * IMU rates, milli-Hz.  The Hz each driver means, for reading:
 *   12/26/52/104/208/416/833/1660               sim (synthetic, whole Hz)
 *   13.016/26.031/52.063/104.125/208.25/        ism330dhcx, lsm6dso, lsm6dsox
 *     416.5/833/1666/3332/6664                  (the 6664/2^n divider chain)
 *   12.5/25/50/100/200/500/1000/2000/           icm42688p
 *     4000/8000/16000/32000
 *   225/281.25/375/562.5/1125                   icm20948 (1125/(1+div))
 *   100/125/200/250/333.333/500/1000            mpu9250, mpu9255 (1000/(1+div))
 */
static const int rate_ladder_imu[] = {
    12000, 12500, 13016, 25000, 26000, 26031, 50000, 52000, 52063,
    100000, 104000, 104125, 125000, 200000, 208000, 208250, 225000,
    250000, 281250, 333333, 375000, 416000, 416500, 500000, 562500,
    833000, 1000000, 1125000, 1660000, 1666000, 2000000, 3332000,
    4000000, 6664000, 8000000, 16000000, 32000000
};
#define RATE_LADDER_IMU_N \
    ((int)(sizeof rate_ladder_imu / sizeof rate_ladder_imu[0]))

/*
 * Magnetometer rates, Hz.
 *   1,10,20,50,100,200,1000       mmc5983ma, sim mag
 *   10,20,50,100                  ak09916, lis2mdl
 *   1,2,5,10,20,40,80,155         lis3mdl
 *   8,100                         ak8963
 *   1,2,4,9,18,37,75,150,300,600  rm3100
 */
static const int rate_ladder_mag[] = {
    1000, 1200, 1250, 2300, 2500, 4500, 5000, 8000, 9000, 10000, 18000,
    20000, 37000, 40000, 50000, 75000, 80000, 100000, 150000, 155000,
    200000, 300000, 600000, 1000000
};
#define RATE_LADDER_MAG_N \
    ((int)(sizeof rate_ladder_mag / sizeof rate_ladder_mag[0]))

#endif /* IMUD_TEST_RATE_LADDER_H */
