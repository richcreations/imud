# Sensor datasheets

Manufacturer datasheets are **not** shipped in this repository: they are
copyrighted vendor documents with no redistribution license (they would also
make the source tarball non-DFSG-free). Fetch them from the manufacturer —
search the part number on the linked page if a deep link has rotted.

## Supported parts (in-tree drivers)

| Part | Role | Driver | Manufacturer page |
|---|---|---|---|
| ST ISM330DHCX | IMU (accel+gyro), reference | `ism330dhcx` | <https://www.st.com/en/mems-and-sensors/ism330dhcx.html> |
| Memsic MMC5983MA | magnetometer, reference | `mmc5983ma` | <https://www.memsic.com/> (search MMC5983MA) |
| TDK ICM-42688-P | IMU, experimental | `icm42688p` | <https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/> |
| TDK ICM-20948 | 9-axis IMU, experimental | `icm20948` | <https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/> |
| AKM AK09916 | magnetometer (inside ICM-20948), experimental | `ak09916` | documented in the TDK ICM-20948 datasheet above |
| ST LSM6DSO / LSM6DSOX | IMU, experimental | `lsm6dso` | <https://www.st.com/en/mems-and-sensors/lsm6dso.html> |
| TDK MPU-9250 / MPU-9255 | 9-axis IMU, experimental (NRND) | `mpu9250`, `mpu9255` | <https://invensense.tdk.com/> (search MPU-9250 / MPU-9255; register maps are separate documents from the product specifications) |
| AKM AK8963 | magnetometer (inside MPU-9250/9255), experimental | `ak8963` | documented in §5 of the MPU-9250 and MPU-9255 register maps above |
| ST LIS2MDL | magnetometer, experimental | `lis2mdl` | <https://www.st.com/en/mems-and-sensors/lis2mdl.html> |
| ST LIS3MDL | magnetometer, experimental | `lis3mdl` | <https://www.st.com/en/mems-and-sensors/lis3mdl.html> |

(The `sim` driver is synthetic and has no hardware.)

## Related references

| Document | Source |
|---|---|
| World Magnetic Model (WMM2025) — technical report, coefficients, test values | <https://www.ncei.noaa.gov/products/world-magnetic-model> |

## Parts researched but without drivers

Candidates evaluated during driver planning; datasheets at the manufacturer:
Honeywell HMC5883L (discontinued), QST QMC5883L (<https://www.qstcorp.com/>),
ST LSM6DS33 (<https://www.st.com/>), InvenSense MPU-6000
(<https://invensense.tdk.com/>), Bosch BMI270 and BMM150
(<https://www.bosch-sensortec.com/>).

Where a part has been evaluated in depth — what it would take, and why it did
or did not get a driver — the reasoning is recorded in
[hardware-evaluations.md](hardware-evaluations.md).
