# imud Driver Writing Guide

This guide explains how to add support for a new IMU or magnetometer chip.
The driver abstraction lives in `include/drivers.h`; existing drivers in
`src/drivers/` are the best reference.  Start by reading `src/drivers/sim.c`
(the simulation driver) — it is the minimal viable implementation of both
interfaces and has no hardware dependencies.

---

## Overview

imud separates chips into two independent driver types:

| Type | Interface | Example chips |
|---|---|---|
| **IMU** (`imu_ops_t`) | Accelerometer + gyroscope | ISM330DHCX, ICM-20948 |
| **Magnetometer** (`mag_ops_t`) | 3-axis magnetic field | MMC5983MA, AK09916 |

Each type is a struct of function pointers plus a few capability flags and
supported-rate tables.  The daemon calls through these pointers and never
touches chip-specific registers directly.

Both types communicate over I2C using the Linux `I2C_RDWR` ioctl — no
`smbus` dependency, no `/dev/i2c-N` `write()`/`read()` — so the same
low-level helpers work for all chips.

---

## Conventions that must be followed exactly

Getting these wrong produces silently wrong attitude output.

### Output coordinate frame — NED-compatible board frame

All drivers must output sensor data in the **NED-compatible board frame**:

| Axis | Direction |
|---|---|
| X | Chip X marking (typically toward bow / forward) |
| Y | Starboard (right when facing forward) |
| Z | **Down** (into the PCB / toward earth when component-side up) |

This is a right-handed frame.  With this convention and `rotation_euler_deg
= [0, 0, 0]` in config, a component-up installation with chip X pointing
bow maps directly to NED with no additional rotation.  Only yaw needs to be
set when the chip X axis does not align with the bow.

### Accelerometer output — specific force, m/s²

Drivers report **specific force** in the NED-compatible board frame.

- When the sensor is **flat, component-side up**: reads approximately
  `[0, 0, −9.80665]` m/s² on Z (reaction force is upward = −Z in this frame).
- When **free-falling** it reads `[0, 0, 0]`.

Scale raw ADC counts to m/s² using the sensitivity from your datasheet
(typically in mg/LSB × 9.80665 → m/s²/LSB), then apply any chip-specific
axis remapping needed to reach the board frame above.

### Gyroscope output — rad/s, right-hand rule

Scale raw counts to **rad/s** (not deg/s).  Typical conversion:

```c
const float d2r = (float)(M_PI / 180.0 / 1000.0);  /* mdps → rad/s */
gyro_scale = sensitivity_mdps_per_lsb * d2r;
```

Apply the same axis remapping as accel so gyro matches the NED-compatible
board frame.  Positive Z rotation = clockwise from above (yaw right) in
NED Z-down convention.

### Magnetometer output — µT

Output magnetic field in **microtesla (µT)** in the same NED-compatible
board frame as the IMU.  If your datasheet gives sensitivity in mGauss/LSB
or counts/Gauss, convert:

```
1 Gauss = 100 µT
```

### Axis remapping for the magnetometer

Many magnetometers are mounted on the same PCB as the IMU but with one or
more axes physically inverted.  After scaling, apply whatever sign flips are
needed so the magnetometer output is in the same NED-compatible board frame
as the IMU.

For the SparkFun 9DoF (SEN-19895): the MMC5983MA is mounted with Z opposite
to the ISM330DHCX chip-native Z (SparkFun marks this "MAG -Z").  After the
ISM driver remaps its own axes, the raw MMC Z already points down, so no
sign change is needed for Z.  The Y axis is the same physical direction as
the ISM chip-native Y (port), so it must be negated to reach starboard:

```c
out->field[1] = -(ry - NULL_FIELD) * scale;   /* flip Y: port → starboard */
out->field[2] =  (rz - NULL_FIELD) * scale;   /* Z already points down */
```

Check your target hardware's schematic and apply the appropriate flips.

---

## The I2C helpers pattern

All existing drivers use the same three helpers — copy them into your new
driver file:

```c
static int burst_read(int fd, uint8_t addr, uint8_t reg,
                      uint8_t *buf, uint16_t len)
{
    struct i2c_msg msgs[2] = {
        { .addr = addr, .flags = 0,        .len = 1,   .buf = &reg },
        { .addr = addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static int reg_write(int fd, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = addr, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static int reg_read(int fd, uint8_t addr, uint8_t reg, uint8_t *val)
{
    return burst_read(fd, addr, reg, val, 1);
}
```

`burst_read` issues a combined write-then-read in a single I2C transaction
(no repeated-start gap), saving ~40 µs vs two separate transactions at 400 kHz.
Use it for reading multiple consecutive registers in one call.

---

## Writing an IMU driver (`imu_ops_t`)

### Static driver state

Because the daemon runs exactly one IMU at a time, each driver stores its
runtime state in a file-scoped static struct:

```c
static struct {
    float    accel_scale;       /* LSB → m/s² */
    float    gyro_scale;        /* LSB → rad/s */
    uint32_t seq;               /* monotonic sample counter */
    uint32_t ticks_per_sample;  /* chip timer ticks between samples (0 if none) */
} s;
```

`seq` is a monotonic counter incremented for every sample produced.  It must
**never reset** while the daemon is running — the fusion thread uses gaps in
`seq` to detect dropped samples.

### `probe(fd, addr)` → 0 or -1

Read the WHO_AM_I (or equivalent) register and verify it against the expected
value from the datasheet.  Log a clear error with the received and expected
values on mismatch.  Return 0 on match, -1 otherwise.

```c
static int myimu_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (reg_read(fd, addr, REG_WHO_AM_I, &who) < 0) {
        fprintf(stderr, "myimu: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_EXPECTED) {
        fprintf(stderr, "myimu: WHO_AM_I = 0x%02X, expected 0x%02X\n",
                who, WHO_AM_I_EXPECTED);
        return -1;
    }
    return 0;
}
```

### `reset(fd, addr)` → 0 or -1

Trigger a software reset and wait for the bit to self-clear.  Always add the
chip's specified power-on time after the reset bit clears — skipping this
causes init failures on slower hardware.

```c
static int myimu_reset(int fd, uint8_t addr)
{
    if (reg_write(fd, addr, REG_CTRL, 0x01) < 0) return -1;   /* SW_RESET */
    for (int i = 0; i < 50; i++) {
        usleep(1000);
        uint8_t val;
        if (reg_read(fd, addr, REG_CTRL, &val) < 0) return -1;
        if (!(val & 0x01)) goto done;
    }
    fprintf(stderr, "myimu: SW_RESET did not clear after 50 ms\n");
    return -1;
done:
    usleep(20000);   /* chip startup time from datasheet */
    return 0;
}
```

### `init(fd, addr, cfg)` → 0 or -1

Configure ODR, full-scale range, FIFO mode (if applicable), and interrupt
routing.  Save the resulting sensitivity values to the static `s` struct.
`cfg->odr_hz` is the requested rate — round it to the nearest value your chip
supports.

```c
static int myimu_init(int fd, uint8_t addr, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_hz);
    uint8_t xlfs = xl_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t gyfs = gy_fs_encode(cfg->gyro_dps, &gyro_scale);

    if (reg_write(fd, addr, REG_ACCEL_CFG, (odr << 4) | xlfs) < 0) return -1;
    if (reg_write(fd, addr, REG_GYRO_CFG,  (odr << 4) | gyfs) < 0) return -1;
    /* ... FIFO, interrupt config ... */

    s.accel_scale = accel_scale;
    s.gyro_scale  = gyro_scale;
    s.seq         = 0;
    return 0;
}
```

### `read(fd, addr, buf, max, *n_out)` → 0, 1, or -1

This is the hot path — called at the configured ODR.  Fill `buf[]` with up to
`max` calibrated `imu_sample_t` samples, set `*n_out` to the number produced,
and return:

| Return | Meaning |
|---|---|
| `0` | Success |
| `1` | Success, but a **FIFO overflow** was detected (data gap) |
| `-1` | I2C error |

**For FIFO-based chips** (recommended — reduces I2C traffic):
1. Read FIFO status to get word count and overflow flag.
2. Burst-read all pending words.
3. Parse and scale each word into `imu_sample_t`.
4. Assign `seq` and `chip_ts` (see below).

**For non-FIFO chips** (DRDY polling):
1. Check DRDY bit; return `*n_out = 0` if not ready.
2. Burst-read the output registers.
3. Produce exactly one sample per call.

Each `imu_sample_t` must have:

```c
buf[i].accel[3]  /* m/s², calibrated */
buf[i].gyro[3]   /* rad/s, calibrated (bias NOT subtracted — MEKF does that) */
buf[i].temp_c    /* die temperature, °C */
buf[i].seq       /* s.seq++ — monotonic, never resets */
buf[i].chip_ts   /* see Hardware Timestamps below */
```

### Hardware timestamps (`has_hw_timestamp`)

Set `has_hw_timestamp = true` only if your chip has an internal sample timer
that increments at a **fixed, known rate independent of the I2C clock**.

The ISM330DHCX has a 32-bit counter at 40000 ticks/s (25 µs/tick).  If your
chip has something equivalent:

1. Read the counter once after draining the FIFO.
2. Back-calculate per-sample timestamps by stepping back by
   `ticks_per_sample = clock_rate / odr_hz` per sample:

```c
uint32_t burst_ts = read_chip_counter();
for (int i = 0; i < produced; i++) {
    uint32_t age = (uint32_t)(produced - 1 - i) * s.ticks_per_sample;
    buf[i].chip_ts = burst_ts - age;   /* 32-bit wrapping arithmetic is correct */
}
```

If your chip has **no hardware timer**, set `has_hw_timestamp = false` and
leave `chip_ts = 0` in every sample.  The anchor mechanism in `imu.c` handles
this case by updating the wall-clock anchor on every burst.

---

## Writing a magnetometer driver (`mag_ops_t`)

### `probe`, `reset`, `init`

Same pattern as the IMU driver.  `init` configures the ODR and enables
continuous measurement mode.  If the chip supports an interrupt pin, enable it
during `init` so the mag reader thread can wake on GPIO edge rather than
polling.

### `read(fd, addr, *out)` → 0, 1, or -1

| Return | Meaning |
|---|---|
| `0` | Sample written to `*out`, `out->valid = true` |
| `1` | Measurement not complete yet (DRDY not asserted) |
| `-1` | I2C error |

Always set `out->wall_ns` from `CLOCK_REALTIME` at read time:

```c
struct timespec ts;
clock_gettime(CLOCK_REALTIME, &ts);
out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
out->valid   = true;
```

Set `out->valid = false` and return 0 (not -1) only when the hardware signals
a measurement overflow or other non-fatal data quality issue.

### `set_reset` (optional)

Some magnetometers have a SET/RESET degaussing coil that restores the AMR
sensing elements after exposure to a strong external field.  If your chip has
this, implement `set_reset` and set `has_set_reset = true`.  The function
issues the pulse and sleeps for the specified settling time.  Set
`set_reset = NULL` and `has_set_reset = false` if your chip has no coil (e.g.
AK09916).

### `has_interrupt`

Set `has_interrupt = true` if the chip asserts an external interrupt pin when
a measurement completes.  The mag reader thread will request a GPIO edge on
the pin configured by `[mag] int_gpio` and call `read()` on each rising edge.

Set `has_interrupt = false` if the chip has no interrupt pin (e.g. AK09916 in
I2C bypass mode).  The reader thread will fall back to a 10 ms nanosleep loop
and poll `read()`.

---

## The `supported_odr_hz[]` and `supported_*` tables

These zero-terminated ascending integer arrays tell the daemon which rates your
chip actually supports.  The nearest-match logic in `imu.c` uses them to round
the user's requested rate to a real chip rate.

```c
const imu_ops_t myimu_ops = {
    .name               = "myimu",
    /* ... function pointers ... */
    .has_fifo           = true,
    .has_hw_timestamp   = false,
    .supported_odr_hz   = { 12, 26, 52, 104, 208, 416, 833, 0 },
    .supported_accel_g  = { 2, 4, 8, 16, 0 },
    .supported_gyro_dps = { 250, 500, 1000, 2000, 0 },
};
```

The last element must be `0` (sentinel).  Keep the list in ascending order.

---

## Registering the driver

Edit `src/drivers.c` — two additions per driver:

```c
/* 1. Forward declaration */
extern const imu_ops_t myimu_ops;

/* 2. Pointer in the registry array */
static const imu_ops_t *imu_registry[] = {
    &ism330dhcx_ops,
    &icm20948_ops,
    &myimu_ops,       /* ← add here */
    &sim_imu_ops,
    NULL,
};
```

Add the new source file to `Makefile`:

```makefile
DRIVER_SRCS = src/drivers/ism330dhcx.c \
              src/drivers/mmc5983ma.c \
              src/drivers/icm20948.c \
              src/drivers/ak09916.c \
              src/drivers/myimu.c \    # ← add here
              src/drivers/sim.c
```

Enable your chip in `config/imud.conf`:

```toml
[imu]
driver   = "myimu"
i2c_addr = 0x68
int_gpio = 17
odr_hz   = 500
```

---

## Testing without hardware — the sim driver

`src/drivers/sim.c` implements both `sim_imu_ops` and `sim_mag_ops`.  It
simulates a flat-mounted sensor yawing at 6°/s.  To test the full daemon
pipeline before your hardware arrives:

```toml
[imu]
driver   = "sim"
i2c_addr = 0x00
int_gpio = 0          # 0 disables GPIO — reader uses 10 ms timer

[mag]
driver   = "sim"
i2c_addr = 0x00
int_gpio = 0
```

Expected steady-state output with the sim driver:

```
pitch        ≈ ±2°  (8 s period)
roll         ≈ ±4°  (6 s period)
rate_of_turn ≈  360 deg/min (with wave-induced oscillation ±30 dpm)
heading      increases ~6°/s
```

---

## Pre-submission checklist

Before opening a pull request, verify each item against your driver:

- [ ] `probe()` reads and validates the chip identity register
- [ ] `reset()` waits for the reset bit to self-clear **and** adds the
      datasheet startup time afterward
- [ ] `init()` stores sensitivity values to the static struct before returning
- [ ] `read()` returns `-1` only on I2C errors, never on "no data yet"
- [ ] Accelerometer output is in **m/s², NED-compatible board frame**
      (flat component-up reads ≈ −9.81 on Z; Z=down convention)
- [ ] Gyroscope output is in **rad/s**
- [ ] Magnetometer output is in **µT**
- [ ] Z-axis sign flip applied if chip Z is opposite to board IMU Z
- [ ] `seq` is incremented for every sample, never reset
- [ ] `chip_ts` is 0 and `has_hw_timestamp = false` if no hardware timer
- [ ] `supported_odr_hz[]` is zero-terminated and in ascending order
- [ ] Driver is added to `src/drivers.c` registry and `Makefile`
- [ ] Tested with `driver = "sim"` before hardware (`make test` passes)
- [ ] Tested on real hardware with `imud-status` confirming sensor activity
