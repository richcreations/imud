# IMU Daemon Specification

## `imud` — a general-purpose IMU daemon ("gpsd for IMUs")

**Version:** 1.10.0  
**What:** owns an IMU + magnetometer, fuses to attitude/heading, and publishes
on NMEA 0183 (UDP/TCP), binary UDP, and a local stream socket with an
optional TCP listener, for any number of
consumers (marine nav, robotics, machine vision, gimbals/pointing).  
**Reference hardware:** SparkFun Qwiic 9DoF — ISM330DHCX + MMC5983MA
(SEN-19895); a driver layer supports other parts (see §4).  
**Platform:** Raspberry Pi / Linux (any host with I2C or SPI)  
**Language:** C11, POSIX — no external dependencies beyond libc and libgpiod

-----

## 1. Hardware Profile

This section documents the **initial hardware target**. The ISM330DHCX and
MMC5983MA are the reference chips around which the driver interface (§4) is
designed. Adding support for a different IMU or magnetometer requires only a
new driver source file — no changes to fusion, output, or lifecycle code.

### ISM330DHCX — Accelerometer + Gyroscope (ST Microelectronics)

|Parameter          |Value                                                    |
|-------------------|---------------------------------------------------------|
|I2C address        |`0x6B` default (`0x6A` via jumper)                       |
|Accel range        |±2/±4/±8/±16 g                                           |
|Gyro range         |±125/±250/±500/±1000/±2000/±4000 dps                     |
|Internal ODR       |12.5 Hz – 6.7 kHz (accel + gyro)                         |
|FIFO               |9 kbytes smart FIFO, watermark interrupt                 |
|Interrupts         |INT1, INT2 — data ready, FIFO watermark, FIFO full       |
|Temperature range  |−40°C to +105°C (industrial grade)                       |
|Same-die accel+gyro|Yes — guaranteed synchronization                         |
|Extras             |Machine Learning Core, 16 programmable FSM state machines|
|I2C modes          |Fast Mode (400 kHz); Fast Mode Plus (1 MHz) capable      |

**⚠️ Z-axis note:** The ISM330DHCX Z-axis is defined top-to-bottom. The MMC5983MA
Z-axis is bottom-to-top. A sign flip on mag_z must be applied before fusion.
SparkFun marks this with a "MAG -Z" dot on the PCB underside.

### MMC5983MA — Magnetometer (MEMSIC)

|Parameter         |Value                                             |
|------------------|--------------------------------------------------|
|I2C address       |`0x30` (fixed, no alternate)                      |
|FSR               |±8 Gauss                                          |
|Resolution        |18-bit (0.0625 mGauss/LSB)                        |
|RMS noise         |0.4 mGauss                                        |
|Heading accuracy  |±0.5°                                             |
|Max ODR           |1000 Hz (continuous mode)                         |
|Interrupt         |INT pin, active-high, measurement-complete        |
|Built-in SET/RESET|Yes — onboard degaussing coil, automatic or manual|

### SparkFun Board (SEN-19895) Specifics

- Both sensors on single Qwiic I2C bus
- 3.3V logic; supply range 1.71–3.6V
- **Mode 2 (Sensor Hub):** ISM330DHCX can master the MMC5983MA over secondary
  I2C pins (SDX/SCX) — requires cutting SDX/SCX jumpers. Not used by this
  daemon; we read both chips independently on the shared bus.
- Pull-up resistors on board — disable if daisy-chaining multiple Qwiic devices

-----

## 2. I2C Bus Strategy

### The 400 kHz Bottleneck

At 400 kHz I2C, a full burst read of both chips costs approximately:

```text
ISM330DHCX burst (14 bytes: accel x/y/z + gyro x/y/z + temp):
  ~14 × 9 bits + overhead ≈ 0.45 ms per read

MMC5983MA burst (7 bytes: mag x/y/z 18-bit packed):
  ~7 × 9 bits + overhead ≈ 0.25 ms per read

Total per cycle: ~0.7 ms → theoretical ceiling ~1400 Hz combined
Practical ceiling with Linux I2C driver overhead: ~800–1000 Hz
```

### FIFO-Driven Architecture (Critical Design Choice)

Do NOT poll the ISM330DHCX registers directly at high rate on Linux — jitter
from the OS scheduler will cause missed samples. Instead:

1. **Configure ISM330DHCX** to sample internally at 833 Hz and batch into its
   9 kByte FIFO.
1. **Set FIFO watermark** to 64 sample-sets (~77 ms of data at 833 Hz).
1. **Wire INT1 → GPIO pin** (e.g., GPIO 17). Configure as FIFO watermark interrupt.
1. **Reader thread** sleeps on GPIO edge event (gpiod), wakes on watermark,
   bursts all pending samples in one I2C transaction.
1. **MMC5983MA** runs at 100 Hz continuous mode. Use its INT pin → GPIO 27 for
   completion signaling.

This approach tolerates Linux scheduler jitter up to ~75 ms without losing a
single IMU sample.

### Raspberry Pi I2C Configuration

```ini
# /boot/firmware/config.txt  (Pi 4/5)  or  /boot/config.txt  (older)
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
core_freq=250          # ensures exact 400 kHz on Pi 3/4
```

Verify with `i2cdetect -y 1`. Expected output: `0x30` (MMC5983MA) and `0x6b`
(ISM330DHCX).

### GPIO Wiring

|Signal     |Pi Pin|BCM GPIO|Notes               |
|-----------|------|--------|--------------------|
|ISM330 INT1|Pin 11|GPIO 17 |FIFO watermark      |
|MMC5983 INT|Pin 13|GPIO 27 |Mag measurement done|
|I2C SDA    |Pin 3 |GPIO 2  |Shared Qwiic bus    |
|I2C SCL    |Pin 5 |GPIO 3  |Shared Qwiic bus    |
|3.3V       |Pin 1 |—       |Power               |
|GND        |Pin 9 |—       |Ground              |

-----

## 3. Daemon Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                            imud                                 │
│                                                                 │
│  ┌─────────────────────────┐   ┌─────────────────────────────┐ │
│  │    ism_reader thread    │   │      mag_reader thread      │ │
│  │                         │   │                             │ │
│  │  GPIO17 edge → wake     │   │  GPIO27 edge → wake         │ │
│  │  Burst FIFO (64 sets)   │   │  Read 7-byte burst          │ │
│  │  Unpack 16-bit × 6 axes │   │  Unpack 18-bit × 3 axes     │ │
│  │  Apply accel/gyro cal   │   │  Apply hard/soft-iron cal   │ │
│  │  Push → IMU ring buffer │   │  Flip Z sign (hardware req) │ │
│  │  833 Hz internal ODR    │   │  Push → mag ring buffer     │ │
│  └────────────┬────────────┘   └──────────────┬──────────────┘ │
│               └──────────────┬─────────────────┘               │
│                              ▼                                  │
│                  ┌───────────────────────┐                      │
│                  │     fusion thread     │                      │
│                  │                       │                      │
│                  │  MEKF @ 833 Hz        │                      │
│                  │  Mag inject @ 100 Hz  │                      │
│                  │  Outputs: quaternion  │                      │
│                  │  Euler, heading       │                      │
│                  └──────────┬────────────┘                      │
│                             │                                   │
│              ┌──────────────┴──────────────┐                    │
│              ▼                             ▼                    │
│    ┌──────────────────┐        ┌───────────────────────┐        │
│    │  nmea_out        │        │    hirate_out          │       │
│    │  10 Hz, NMEA0183 │        │    500 Hz, binary      │       │
│    │  UDP broadcast   │        │    UDP multicast/bcast  │      │
│    └────────┬─────────┘        └──────────┬────────────┘        │
└────────────┼──────────────────────────────┼────────────────────┘
             ▼                              ▼
       UDP :10110                     UDP :10111
       (NMEA 0183)                   (IMU binary)
```

### Thread Summary

|Thread        |Wake condition              |Work                                                                     |
|--------------|----------------------------|-------------------------------------------------------------------------|
|`ism_reader`  |GPIO17 FIFO watermark edge  |Burst 64 samples, scale to SI, push ring buffer                          |
|`mag_reader`  |GPIO27 INT edge             |Read MMC5983MA, apply cal, flip Z, push mag buffer                       |
|`fusion`      |New sample in ring buffer   |MEKF predict step at 833 Hz; mag update at 100 Hz                        |
|`nmea_out`    |100 ms timer (10 Hz)        |Snapshot fused state, encode NMEA sentences, broadcast UDP + serve TCP clients |
|`hirate_out`  |2 ms timer (500 Hz)         |Pack latest fused state + mag; send UDP                                  |
|`stream_out`  |rate_hz timer (100 Hz)      |Serve 276-byte binary packets to local AF_UNIX subscribers + TCP clients |
|`health`      |1 s timer + status socket   |Stats logging, serve imud-status queries, systemd watchdog heartbeat     |
|`position`    |gpsd TCP stream / 30 s poll |Read GPS fixes from gpsd or SignalK, recompute WMM declination on ≥5 km move, push into fusion |

`nmea_out` is only started when `nmea.enabled` or `nmea.tcp_enabled` is true.
`hirate_out` is only started when `highrate.enabled = true`.
`stream_out` is only started when `stream.enabled` or `stream.tcp_enabled`
is true.
`position` is only started when `position.gpsd_enabled` or
`position.signalk_enabled` is true (see §9 `[position]`).

Each optional TCP listener (netserv, `src/netserv.c`) is owned by exactly one
output thread: non-blocking accept each tick, broadcast-only (clients are
never read), at most 8 clients, and a slow-client policy of drop-payload on a
full kernel buffer / disconnect on error or partial write. Default posture
(1.6): only the local AF_UNIX stream socket is enabled — every network
output is an explicit config opt-in.

-----

## 4. Driver Abstraction Layer

The `ism_reader` and `mag_reader` threads call through a chip operations struct
rather than reading registers directly. Swapping hardware means selecting a
different driver in config — the rest of the daemon is unchanged.

Every op takes an `imud_bus_t` — the transport handle. It carries the
descriptor, the transport kind, and whatever that transport needs to address
the part: the 7-bit slave address on I²C, the resolved clock and framing bits
on SPI. A driver's register logic is therefore written once and runs on either
bus. The IMU and the magnetometer hold one handle each, never a shared one, so
they can sit on different buses.

```c
typedef enum { BUS_I2C = 0, BUS_SPI = 1 } bus_kind_t;

typedef struct {
    bus_kind_t kind;
    int        fd;              /* < 0 = not open */
    uint8_t    i2c_addr;        /* BUS_I2C: 7-bit slave address */
    uint8_t    spi_mode;        /* BUS_SPI: 0..3 */
    uint8_t    spi_inc_mask;    /* BUS_SPI: multi-byte auto-increment bits */
    uint32_t   spi_hz;          /* BUS_SPI: resolved clock */
} imud_bus_t;
```

On SPI the command byte carries the direction in its top bit — 1 read, 0 write
— and the register address in the low seven; a read is a command transfer
followed by a data transfer inside one `SPI_IOC_MESSAGE`, so chip-select stays
asserted across both. Parts differ in whether a multi-byte read needs an
explicit auto-increment bit, which is why `spi_inc_mask` is per-driver.

### IMU Driver Interface

```c
typedef struct {
    const char *name;               /* matches config [imu] driver = "..." */

    bool  experimental;             /* warn at startup; not validated on silicon */

    /* What this part's silicon can do on SPI — datasheet facts. Zeroed means
       spi_capable = false, and `bus = "spi"` is then refused by name. */
    bus_caps_t bus_caps;

    int (*probe)    (const imud_bus_t *bus);
    int (*reset)    (const imud_bus_t *bus);
    int (*init)     (const imud_bus_t *bus, const imu_cfg_t *cfg);

    /* Drain FIFO or read data-ready registers.
       Returns samples as calibrated SI (m/s², rad/s). Fills *n. */
    int (*read)     (const imud_bus_t *bus,
                     imu_sample_t *buf, int max, int *n);

    bool  has_fifo;                 /* false → single-sample DRDY read per wake */
    bool  has_hw_timestamp;         /* chip has internal sample timer */
    int   supported_odr_hz[16];     /* ascending list, 0-terminated */
    int   supported_accel_g[8];
    int   supported_gyro_dps[8];

    /* Rate this driver will really program for `requested`, in Hz.
       NULL → the default rule: lowest supported_odr_hz entry >= requested,
       clamped to the highest.  Divider-based parts must implement it. */
    int (*actual_odr_hz)(int requested);
} imu_ops_t;
```

### Magnetometer Driver Interface

```c
typedef struct {
    const char *name;               /* matches config [mag] driver = "..." */

    bool  experimental;
    bus_caps_t bus_caps;            /* as imu_ops_t */

    int (*probe)    (const imud_bus_t *bus);
    int (*reset)    (const imud_bus_t *bus);
    int (*init)     (const imud_bus_t *bus, const mag_cfg_t *cfg);
    int (*read)     (const imud_bus_t *bus, mag_sample_t *out);
    int (*set_reset)(const imud_bus_t *bus); /* NULL if chip has no degauss coil */

    bool  has_interrupt;
    bool  has_set_reset;
    int   supported_odr_hz[16];
    int (*actual_odr_hz)(int requested);    /* as imu_ops_t */
} mag_ops_t;
```

### Driver Registry

Drivers are registered in a static table in `drivers.c`. The daemon looks up
the configured name at startup and fatals if no match is found.

```c
static const imu_ops_t *imu_registry[] = {
    &ism330dhcx_ops,   /* ISM330DHCX — reference hardware  [I2C + SPI] */
    &icm20948_ops,     /* ICM-20948 — alternate IMU        [experimental] */
    &icm42688p_ops,    /* ICM-42688-P     [I2C + SPI]      [experimental] */
    &lsm6dso_ops,      /* LSM6DSO         [I2C + SPI]      [experimental] */
    &lsm6dsox_ops,     /* LSM6DSOX        [I2C + SPI]      [experimental] */
    &mpu6500_ops,      /* MPU-6500 — 6-axis, same driver   [experimental] */
    &mpu9250_ops,      /* MPU-9250                         [experimental] */
    &mpu9255_ops,      /* MPU-9255 (same driver)           [experimental] */
    &sim_imu_ops,      /* synthetic driver for testing without hardware */
    NULL
};

static const mag_ops_t *mag_registry[] = {
    &mmc5983ma_ops,    /* MMC5983MA — reference hardware   [I2C + SPI] */
    &ak09916_ops,      /* AK09916 — compass on ICM-20948   [experimental] */
    &ak8963_ops,       /* AK8963 — compass on MPU-925x     [experimental] */
    &lis3mdl_ops,      /* LIS3MDL         [I2C + SPI]      [experimental] */
    &lis2mdl_ops,      /* LIS2MDL                          [experimental] */
    &rm3100_ops,       /* RM3100          [I2C + SPI]      [experimental] */
    &sim_mag_ops,      /* synthetic driver for testing without hardware */
    NULL
};
```

Drivers marked `[experimental]` set `experimental = true` in their ops struct
and have not been validated on real silicon; the daemon logs a warning at
startup when one is selected. `imud-imutest` (§12) exercises any registered
driver against real hardware and writes the report that clears the flag.

Two of the IMU drivers pair with a magnetometer that lives on the same die
behind an I²C **bypass** rather than on the host bus in its own right:
`icm20948` with `ak09916`, and `mpu9250`/`mpu9255` with `ak8963`. Both mags
answer at 0x0C once the IMU's `init()` has opened the bypass, which is why
`imu_ctx_open()` always brings the IMU up before the magnetometer. The two
AKM parts are **not** interchangeable — different device IDs, register maps,
and axis alignments.

### Synthetic (sim) Driver

`driver = "sim"` in both `[imu]` and `[mag]` activates the synthetic driver,
which produces physically-correct data without any hardware. It simulates a
small boat under way with constant yaw sweep and wave-induced motion:

- **Yaw:** constant 6°/s (one full rotation every 60 s)
- **Roll:** ±4° sinusoidal, 6-second period
- **Pitch:** ±2° sinusoidal, 8-second period, phase-offset from roll
- **Heave:** ±0.3 m/s² vertical wave acceleration, 5-second period

The gyro output uses the full aerospace angular-velocity formula so cross-axis
coupling between roll/pitch/yaw is modelled correctly. The accelerometer output
includes both gravity tilt and heave. The magnetometer output rotates the NED
Earth field into body frame using the full attitude matrix.

**Playback mode.** When `[device] sim_file` names an `.imucap` capture file
(recorded by the `[capture]` black box), both sim ops replay the recorded raw
samples instead of synthesizing — identical pacing model, timing anchored to
the capture header, `sim_loop`/`sim_speed` control repetition and rate.
`imud --replay FILE` is the shortcut form, and differs in how it ends: it
plays the file once and exits (0 at end of capture after draining the last
samples through the filter, 1 if the capture cannot be read), ignoring
`sim_loop`. Configured via `sim_file`, the daemon keeps running at end of
file and the stream simply stops. The capture file format is
specified in docs/capture.md and include/capture.h; it is a separate,
append-only format — NOT the wire packet.

Expected fusion output: heading increases ~6°/s, roll tracks ±4°, pitch tracks ±2°,
rate_of_turn ~360 deg/min with small wave-induced oscillation.

Set `int_gpio = 0` for both sensors when using `sim` — the reader threads poll
at the batch period instead of waiting on GPIO edges.

### Adding a New Chip

1. Create `src/drivers/<chipname>.c` implementing `imu_ops_t` or `mag_ops_t`
2. Add the extern declaration and pointer to the registry in `src/drivers.c`
3. Add the source file to `DRIVER_SRCS` in `Makefile`
4. Set `driver = "<chipname>"` in `[imu]` or `[mag]` config

No other files change.

-----

## 5. Sensor Init Registers

### ISM330DHCX Init Sequence

```text
# Accel: 833 Hz, ±8g, LPF2 enabled
CTRL1_XL = 0x7E   # ODR=833Hz, FS=±8g, LPF2_XL_EN=1

# Gyro: 833 Hz, ±2000 dps
CTRL2_G  = 0x7C   # ODR=833Hz, FS=±2000dps

# BDU + register auto-increment
CTRL3_C  = 0x44   # BDU=1, IF_INC=1

# FIFO: continuous mode, watermark=64, both accel+gyro batched at 833 Hz
FIFO_CTRL1 = 0x40  # WTM[7:0] = 64
FIFO_CTRL2 = 0x00  # WTM[8]=0, STOP_ON_WTM=0
FIFO_CTRL3 = 0x77  # BDR_GY=833Hz [7:4]=0111, BDR_XL=833Hz [3:0]=0111
FIFO_CTRL4 = 0x06  # FIFO_MODE=Continuous (overwrite oldest)

# Enable timestamp counter (25 µs/tick, read via TIMESTAMP0–3 at 0x40–0x43)
CTRL10_C   = 0x20  # TIMESTAMP_EN=1 (bit 5)

# INT1: assert on FIFO watermark
INT1_CTRL  = 0x08  # INT1_FIFO_TH=1
```

### MMC5983MA Init Sequence

```text
# BW=01 → 4 ms conversion, 200 Hz filter bandwidth; required for 100 Hz ODR
# (BW=00 gives 8 ms conversion and max 50 Hz ODR — insufficient)
CTRL1 = 0x01       # BW[1:0]=01

# Continuous mode, 100 Hz ODR — AUTO_SR_en disabled (periodic manual SET instead)
CTRL0 = 0x00       # AUTO_SR_en=0 (clear all; INT_meas_done_en set separately below)
CTRL2 = 0x0D       # Cmm_en=1 (bit 3), CM_Freq=101 (bits [2:0]) = 100 Hz

# Enable interrupt on measurement complete
CTRL0 |= 0x04      # INT_MEAS_DONE_EN=1
```

-----

## 6. Sensor Fusion

### Multiplicative Extended Kalman Filter (MEKF)

The MEKF is the canonical algorithm for 9-DOF AHRS and is used in production
aerospace, autopilot (ArduPilot, PX4), and robotics systems. It is the correct
choice here for three reasons specific to this hardware:

**1. Proper 9-DOF sensor fusion.** Mahony and Madgwick treat the magnetometer
as a correction bolt-on to a 6-DOF filter. The MEKF treats all three sensor
modalities — gyroscope, accelerometer, magnetometer — as distinct measurement
sources with their own noise models in a unified probabilistic framework.

**2. Asynchronous update rates handled correctly.** The ISM330DHCX produces
data at 833 Hz; the MMC5983MA at 100 Hz. In the
MEKF this is natural: the prediction step runs at 833 Hz using gyro integration,
and the measurement update step runs at whichever rate each sensor produces
data. Complementary filters require awkward injection logic to handle this.

**3. Covariance output.** The filter maintains a 3×3 attitude error covariance
matrix at every timestep. This is emitted in the high-rate packet and is
directly consumable by any downstream Kalman filter, visual-inertial odometry
pipeline, or bundle adjustment — the consumer knows not just attitude but
*how uncertain* that attitude estimate is.

The covariance is held to what each measurement can actually support. A
gravity-direction measurement carries no information about rotation *about*
gravity, so the accelerometer update is constrained to leave that axis of the
covariance untouched, and to make no correction about it. With no magnetometer
aiding heading, the heading axis therefore grows without bound and
`fusion_converged` stays clear, rather than collapsing under a tilt disturbance
and reporting a confidence the filter has not earned. See `docs/math.md` §4.7.

### State and Noise Model

The error-state is a 3-vector δθ (small-angle rotation error), a 3-vector
gyro bias estimate b, and a 3-vector wave-acceleration estimate a_w — 9 states
total. The full quaternion q is maintained separately on the SO(3) manifold and
updated multiplicatively, avoiding the rank-deficiency problem of naively
applying EKF to a 4-component quaternion.

`a_w` (1.7) models the wave-orbital acceleration that contaminates the gravity
measurement in a seaway as a first-order Gauss–Markov process, tuned by
`mekf_wave_accel` (steady-state σ, m/s²) and `mekf_wave_accel_tau_s`
(correlation time). Without it the filter treats 833 strongly correlated
samples per second as 833 independent measurements of gravity, collapses its
own covariance, and diverges. Either knob at 0 disables the state and returns
the pre-1.7 6-state filter exactly. See `docs/math.md` §4.1.1 and §4.7.1.

The `mekf_*` noise parameters are **tuning knobs**, seeded from the sensor
datasheets but not bound to them. In particular `mekf_gyro_noise` (default
0.007) is a process-noise Q deliberately held **above** the datasheet noise
floor: it buys robustness to unmodeled disturbances (vibration, thermal
transients) at the cost of a little extra measurement weighting. This is why
`imud-cal characterize` reports measured Allan noise as **informational
only** — measured values are never fed into the filter automatically.
`mag_reject_gauss` is a further tuning decision, controlling how aggressively
to reject magnetometer samples that deviate from the expected field magnitude
after calibration.

### Predict / Update Cycle

```text
Every ISM330DHCX FIFO burst (833 Hz):
  for each sample in burst:
    1. PREDICT:
       q      ← q ⊗ exp(0.5 × (ω - b) × dt)   # quaternion kinematics
       P      ← F·P·Fᵀ + Q                      # covariance propagation

Every MMC5983MA measurement (~100 Hz), when mag_valid=1:
    2. MAG UPDATE:
       h      ← R(q) · m_ref                    # predicted mag in body frame
       K      ← P·Hᵀ·(H·P·Hᵀ + R_mag)⁻¹       # Kalman gain
       δθ     ← K · (m_meas - h)                # error-state correction
       q      ← q ⊗ exp(0.5 × δθ)              # multiplicative update
       b      ← b + K_b · δθ                    # bias update
       P      ← (I - K·H) · P                   # covariance update

Every accel sample (833 Hz, when |a| within 5% of 1g — not under lin. accel):
    3. ACCEL UPDATE:
       Same structure as mag update, using gravity reference vector
       Skipped when linear acceleration detected (|a - g| > threshold)
       K's attitude rows projected onto the plane orthogonal to h, so
       gravity neither rotates the estimate about gravity nor reduces
       the covariance there
```

The accel update skip condition is important for marine use — wave-induced
linear acceleration must not be mistaken for a tilt change. The 5% threshold
on `|a|` vs 1g is configurable via `accel_skip_thresh`.

### Reference Implementation

Joan Solà's *"Quaternion Kinematics for the Error-State Kalman Filter"* (arXiv
1711.02508) is the definitive public reference for this implementation. The
Jacobians and noise propagation equations are derived in full. The C
implementation follows this paper's notation directly so the code is
auditable against a published source.

### MMC5983MA SET/RESET (Degaussing)

The MMC5983MA has an onboard coil that can apply a SET or RESET pulse to
eliminate residual magnetization — a common failure mode near motors, wiring
looms, and steel structure on a vessel.

Running at 100 Hz, `AUTO_SR_EN` is **disabled**. Auto SET/RESET alternates
pulses on every measurement, which halves effective rate to 50 Hz and introduces
a per-sample polarity toggle that must be tracked and averaged — unnecessary
complexity at this rate.

Instead, the daemon issues a **periodic manual SET pulse** every 5 seconds via
the CTRL0 register. This degausses the sensor bridge regularly without
interrupting the 100 Hz measurement stream. In high-magnetic-disturbance
environments (near a running engine or alternator) this interval is
configurable down to 1 second.

When a SET pulse is issued, the daemon sets `FLAG_MAG_SET_RESET` (bit 1) in
the next output packet so consumers can detect the settling event.

### Calibration Data (cal.json)

Calibration is produced by `imud-cal` (§12) and loaded at startup. Missing
file is not fatal — the daemon runs uncalibrated with appropriate flags clear.

```json
{
  "mag": {
    "hard_iron": [dx, dy, dz],
    "soft_iron": [[sxx, sxy, sxz],
                  [syx, syy, syz],
                  [szx, szy, szz]]
  },
  "accel": {
    "offset": [ax, ay, az],
    "scale":  [sx, sy, sz]
  },
  "gyro": {
    "bias": [bx, by, bz]
  },
  "noise": {
    "gyro_density":     [nx, ny, nz],
    "gyro_instability": [ix, iy, iz],
    "accel_density":    [ax, ay, az]
  },
  "gyro_temp": {
    "coeff": [cx, cy, cz],
    "ref_c": t_ref
  }
}
```

The `noise` section (written by `imud-cal characterize`, §12) records
measured Allan noise densities and bias instability — **informational
only**, never fed into the MEKF (see the noise-model note in this section).
The `gyro_temp` section (written by `imud-cal fit-temp`) is a linear
gyro-bias/temperature fit that **is** applied at runtime: the daemon
subtracts `coeff × (temp − ref_c)` from each gyro sample.

Applied corrections. `m` is the sample as the driver delivered it, in **sensor
axes**; `R` is the `[mount]` rotation:

```text
accel_cal  = R × ((m_accel − accel_offset) × accel_scale)  (per-axis, then mount)
gyro_cal   = R × (m_gyro − gyro_bias)                      (bias subtracted in MEKF predict)
mag_cal    = R × (soft_iron × (m_mag − hard_iron))         (matrix × vector, then mount)
mag_z      = −raw_mag_z                                    (Z sign flip, always)
```

**Calibration is applied in sensor axes, before the mount rotation**, because
that is the frame it was measured in: `imud-cal` reads the driver directly and
never applies `[mount]`. Rotating first and calibrating second subtracts a
sensor-frame offset from body-frame data — invisible while `[mount]` is
identity, wrong on every axis as soon as it is not. Rotating the calibration
instead is not representable: `accel_scale` is a diagonal, and `R·diag·Rᵀ` is a
full matrix.

Note this means the `accel_raw_*` and `mag_raw_*` fields on the wire — which are
pre-calibration but **after** the mount rotation, i.e. `R × m` — cannot be
substituted into the expressions above unless `R` is the identity. They are
there to show what the sensor delivered in body axes, not as an intermediate of
the calibration chain.

The soft-iron matrix from `imud-cal mag` carries a full 2×2 horizontal block
(including the cross term) from a least-squares ellipse fit of the swing
circle — a distortion ellipse whose axes are rotated relative to the sensor
axes cannot be corrected by per-axis scales alone. Z stays 1.0 unless the
data has real 3D coverage.

### Rough-sea mechanics (v1.1 fusion revision)

- **Gauss–Markov wave-acceleration state** (1.7): the correlated part of the
  seaway's gravity disturbance is modelled in the filter state rather than
  mis-described as white measurement noise. See `docs/math.md` §4.7.1.
- **Joseph-form covariance updates** everywhere, plus explicit
  symmetrization after predict — P stays symmetric positive-definite in
  float arithmetic over multi-day runs.
- **Per-sample dt**: the predict step integrates over the measured
  inter-sample interval from hardware timestamps (clamped to [0.5×, 2×]
  nominal), so IMU oscillator tolerance does not scale integrated rotation.
- **χ² innovation handling** on accel and mag updates: the normalised
  innovation distance d² = νᵀS⁻¹ν is capped Huber-style at the χ² 99 %
  quantile (influence bounded, information still flows in a steady seaway)
  and rejected outright beyond 25× the gate. Because S contains P, the gate
  self-regulates: inactive while the filter is still acquiring.
- **`mag_yaw_only` (default true)**: scalar heading update via
  H = [−(R̂ e_D)ᵀ | 0]; the magnetometer never pulls on roll/pitch.
- **`mag_fuse_uncal` (default true)**: with no `cal.json` the magnetometer is
  still fused. The uncorrected hard iron offsets heading by a deviation that
  varies with heading, but it is bounded by asin(|b|/|H|) and repeatable,
  whereas gyro dead reckoning is unbounded. The update is forced heading-only
  regardless of `mag_yaw_only` (the uncorrected dip must not reach roll or
  pitch), the m_ref self-healing below is suspended (its invariants stop being
  attitude-independent once |B| varies with heading), and the converged-only
  3-vector reject gate is not applied (an uncalibrated residual stays large
  permanently, so the gate would silently drop every sample after
  convergence). `FLAG_MAG_UNCAL` (bit 15) is the wire signal for
  this state; `FLAG_MAG_VALID` keeps its original "calibrated and healthy"
  meaning and stays clear, so a consumer that only tests it is unaffected.
- **Uncalibrated withdrawal (`mag_uncal_reject_frac`, default 0.4)**: past
  |b_h| > |H| an uncalibrated compass is not merely offset — indicated heading
  stops being monotonic in true heading, so two headings read alike and the
  error's sense inverts over half the rose. The gate approximates that
  boundary as `mag_anomaly > frac · (|H_ref|/|B_ref|)` and reverts to dead
  reckoning above it, re-admitting hysteretically at 0.8×. Deliberately
  coarse: `mag_anomaly` is a 30 s mean rather than a peak (it under-reads a
  constant-rate turn by ≈2/π) and a mostly-vertical offset inflates it without
  harming heading-only fusion. A backstop against gross iron, not a boundary.
- **m_ref self-healing**: the magnetic reference's magnitude and dip are
  slowly (τ ≈ 5 min) re-estimated from attitude-independent invariants
  (field magnitude; angle between mag vector and the accel-measured gravity
  direction), but only while the platform is quiescent
  (EMA of (|a|/g−1)² < 2·10⁻⁴, τ ≈ 2 s). The horizontal direction — the
  heading anchor — is never adapted (no gauge feedback).
- **Alignment averaging**: initial tilt/heading alignment averages ~1 s of
  accel and all mag samples in that window instead of one instantaneous
  reading.
- **WMM-informed m_ref**: when position is known (static lat/lon or live
  GPS), the magnetic reference's magnitude and dip are set analytically
  from the WMM field vector (`wmm_field_ned`), direction-preservingly —
  removing the alignment bake-in error at the source.
- **Speed-aided centripetal correction**: with speed over ground from gpsd,
  ω×v (v ≈ speed along the bow) is subtracted from the accelerometer before
  the gravity update, so sustained turns no longer tilt the horizon; speed
  is invalidated with the GPS fix TTL.
- **Rate of turn** is the true Euler yaw rate
  ψ̇ = (ω_y·sinφ + ω_z·cosφ)/cosθ, not raw body-Z rate (falls back to ω_z
  near ±90° pitch).
- **Heave** (§7 PASHR / §8 binary packet): leaky double integration of NED vertical
  acceleration through a true output high-pass (exact zero at DC); triple
  pole at 1/τ ⇒ allow ~10·τ settling after startup.

Synthetic rough-sea benchmark (±15° roll @ 0.2 Hz, ±8° pitch, 0.12 g
orbital accel, run by `test_fusion`): attitude RMS 7.1° → 2.7°, heading RMS
4.9° → 2.4° versus the v1.0 filter.

-----

## 7. Output Stream A — NMEA 0183 (UDP broadcast and/or TCP listener)

**Port:** 10110 UDP broadcast (`255.255.255.255`) / configurable; optional
TCP listener on 10110 (`nmea.tcp_enabled`, `tcp_bind_addr:tcp_port`)  
**Rate:** 10 Hz (shared by both transports)  
**Format:** NMEA 0183 ASCII sentences, `<CR><LF>` terminated, correct checksums

Any chartplotter, MFD, multiplexer, OpenCPN, Expedition, SignalK server, or
logging tool on the vessel LAN receives the broadcast without configuration;
TCP-client consumers (most plotter apps) connect to `tcp://<host>:10110` and
receive the identical sentence bursts. Up to 8 TCP clients; a slow client
skips bursts (never stalls the daemon), and a client that errors or would
receive a partial burst is disconnected.

### Sentence Set

Three sentences are emitted per update cycle unconditionally — `$PASHR`,
`$TIROT`, `$IIXDR`. The heading sentences are conditional:

| Sentence | Emitted when |
|---|---|
| `$HCHDM`, `$HCHDG` | `FLAG_MAG_VALID` **or** `FLAG_MAG_UNCAL` — the magnetometer is being fused |
| `$HCHDT` | the above **and** `FLAG_DECLINATION_VALID` |

With both mag flags clear no magnetometer is fitted, or it has stopped
delivering, and heading is dead-reckoned from the gyro: a relative angle that
drifts without bound. `$HCHDM` has no status field and `$HCHDG`'s is
deviation, not validity, so a consumer could not tell — they are withheld
instead, and `$PASHR` sends a null heading field. An *uncalibrated* fuse is
still a magnetic heading, offset by the uncorrected hard iron but bounded, so
it is published normally.

**`$PASHR` — Attitude (primary)**

Always emitted: roll, pitch and heave stay valid with no magnetometer at all.

```text
$PASHR,HHH.H,M,RRR.R,PPP.P,0.0,ra.r,pa.p,0,A,,*hh<CR><LF>

Fields:
  HHH.H   Magnetic heading (0–360°); null when the magnetometer is not fused
  M       Always magnetic (PASHR carries magnetic heading regardless of declination)
  RRR.R   Roll, degrees (+ = starboard up)
  PPP.P   Pitch, degrees (+ = bow up)
  h.hh    Heave, metres, positive up (band-passed vertical-accel double
          integration; live when heave_tau_s > 0, else 0.0)
  ra.r    Roll accuracy estimate, degrees (from MEKF covariance)
  pa.p    Pitch accuracy estimate, degrees (from MEKF covariance)
  0       GPS quality flag (0 = no GPS)
  A       IMU status: A=valid

Example:
  $PASHR,214.7,M,-9.5,+3.1,0.0,0.3,0.3,0,A,,*hh<CR><LF>
  $PASHR,,M,-9.5,+3.1,0.0,0.3,0.3,0,A,,*hh<CR><LF>       (heading not fused)
```

The `A` in the status field stays `A`: the IMU is healthy and roll and pitch
are valid. It says nothing about heading.

**`$HCHDM` — Magnetic Heading** *(emitted only when the magnetometer is fused)*

```text
$HCHDM,HHH.H,M*hh<CR><LF>
Example: $HCHDM,214.7,M*3C<CR><LF>
```

**`$HCHDG` — Heading, Deviation, Variation** *(gated like `$HCHDM`)*

Deviation fields are always empty (hard/soft-iron calibration is applied
upstream). Variation fields carry the WMM/static declination when known
(`E` = east/+, `W` = west/-) and are empty otherwise — consumers like
Signal K read magnetic variation from this sentence directly.

```text
$HCHDG,HHH.H,,,VV.V,a*hh<CR><LF>

Example: $HCHDG,214.7,,,13.2,E*hh<CR><LF>   (declination known)
         $HCHDG,214.7,,,,*hh<CR><LF>        (declination unknown)
```

**`$HCHDT` — True Heading** *(gated like `$HCHDM`, plus `FLAG_DECLINATION_VALID`)*

Requires `[position]` declination to be configured (WMM auto-compute or static override).

```text
$HCHDT,HHH.H,T*hh<CR><LF>

  HHH.H   True heading = fmod(magnetic_heading + declination_deg + 360, 360)

Example: $HCHDT,229.2,T*hh<CR><LF>   (heading 214.7° mag + 14.5°E decl)
```

**`$TIROT` — Rate of Turn**

```text
$TIROT,RRR.R,A*hh<CR><LF>    (+ = turning right, deg/min)
Example: $TIROT,-6.2,A*hh<CR><LF>
```

**`$IIXDR` — Pitch and Roll Transducer**

```text
$IIXDR,A,PPP.P,D,PTCH,A,RRR.R,D,ROLL*hh<CR><LF>
Example: $IIXDR,A,+3.1,D,PTCH,A,-9.5,D,ROLL*hh<CR><LF>
```

### Talker ID Summary

|Talker|Meaning                   |Used for                        |
|------|--------------------------|--------------------------------|
|`HC`  |Heading/Compass           |`$HCHDM`, `$HCHDG`, `$HCHDT`    |
|`II`  |Integrated Instrumentation|`$IIXDR`                        |
|`TI`  |Turn Indicator            |`$TIROT`                        |
|`P`   |Proprietary               |`$PASHR`                        |

-----

## 8. Output Stream B — High-Rate IMU UDP

**Port:** 10111  
**Rate:** 500 Hz  
**Format:** Binary, little-endian, 276 bytes fixed  
**Wire load:** ~120 KB/s

Little-endian on the wire whatever the daemon's host byte order: `packet_encode`
converts field by field, so a consumer parses the same bytes either way.

### Packet Layout

<!-- BEGIN GENERATED: packet-layout -->
```text
Offset  Bytes  Type      Field             Notes
──────────────────────────────────────────────────────────────────
 0      4      uint32    magic             0x494D5544 (“IMUD”)
 4      2      uint16    version           = 18  (IMUD_VERSION; wire revision,
                                              not the release version)
 6      2      uint16    flags             see below
 8      8      uint64    ts_wall_ns        CLOCK_REALTIME ns, SAMPLE-TAKEN
16      8      uint64    ts_tai_ns         CLOCK_TAI ns
24      4      uint32    ts_chip_ticks     raw ISM330 counter (25 µs/tick)
28      4      uint32    anchor_gen        increments on re-anchor
32      4      float32   accel_x           m/s², calibrated + cal-corrected
36      4      float32   accel_y
40      4      float32   accel_z
44      4      float32   accel_raw_x       m/s², pre-calibration (after mount rot)
48      4      float32   accel_raw_y
52      4      float32   accel_raw_z
56      4      float32   gyro_x            rad/s, bias-corrected (fused)
60      4      float32   gyro_y
64      4      float32   gyro_z
68      4      float32   gyro_raw_x        rad/s, before bias correction
72      4      float32   gyro_raw_y
76      4      float32   gyro_raw_z
80      4      float32   mag_x             µT, calibrated (hard/soft-iron corrected)
84      4      float32   mag_y
88      4      float32   mag_z
92      4      float32   mag_raw_x         µT, pre-calibration (after mount rot)
96      4      float32   mag_raw_y
100     4      float32   mag_raw_z
104     4      float32   quat_w            MEKF output quaternion [w,x,y,z]
108     4      float32   quat_x
112     4      float32   quat_y
116     4      float32   quat_z
120     4      float32   pitch             rad, NED
124     4      float32   roll              rad, NED
128     4      float32   yaw               rad, magnetic
132     4      float32   heading_deg       0–360° magnetic
136     4      float32   rate_of_turn      deg/min, + = turning right
140     4      float32   temp_c            °C ISM330DHCX die sensor
144     9×4    float32   cov[9]            3×3 attitude error covariance,
                                          row-major (rad²); from MEKF P matrix
180     4      uint32    imu_seq           monotonic ISM330 sample counter
184     4      float32   declination_deg   °E+; 0.0 when FLAG_DECLINATION_VALID not set
188     4      float32   heave_m           m, + up; 0.0 when heave disabled (v1.1)
192     4      float32   gyro_bias_x       rad/s, estimated gyro bias (v1.2, body frame)
196     4      float32   gyro_bias_y
200     4      float32   gyro_bias_z
204     4      float32   gyro_bias_var_x   (rad/s)², gyro-bias variance (v1.2)
208     4      float32   gyro_bias_var_y
212     4      float32   gyro_bias_var_z
216     4      float32   heave_rate        m/s, + up; 0.0 when heave disabled (v1.2)
220     4      float32   accel_quiescence  EMA of (|a|/g−1)²; disturbance metric (v1.2)
224     4      float32   wave_height_m     significant wave height Hs = 4σ(heave), m;
                                          0.0 until FLAG_WAVE_VALID (v14)
228     4      float32   wave_period_s     mean zero-crossing wave period Tz, s;
                                          0.0 when becalmed or not settled (v14)
232     4      float32   roll_period_s     vessel roll period, s; 0.0 = not rolling (v14)
236     4      float32   roll_amplitude    significant single amplitude 2σ(roll), rad (v14)
240     4      float32   pitch_period_s    vessel pitch period, s; 0.0 = not pitching (v14)
244     4      float32   pitch_amplitude   significant single amplitude 2σ(pitch), rad (v14)
248     4      float32   mag_anomaly       EMA of ||B|−|B_ref||/|B_ref| — magnetic
                                          interference / iron-cal drift metric (v14)
252     4      float32   mag_residual      EMA of |heading innovation|, rad — compass
                                          calibration health (v14)
256     4      float32   innov_weight      EMA of the Huber weight √(γ/d²) applied
                                          to accepted MEKF updates; 1.0 = never
                                          capped, → 0.33 = sustained capping at
                                          the reject boundary (v17)
260     4      float32   innov_reject      EMA of the innovation-gate reject
                                          indicator; fraction of MEKF updates
                                          discarded outright (v17)
264     4      float32   nis_accel         EMA of the normalised innovation
                                          squared for the accel update, d²/2;
                                          1.0 = the filter's covariance is
                                          consistent with the innovations it
                                          actually sees, > 1 = over-confident
                                          (v17)
268     4      float32   nis_mag           Same for the mag update: d²/2 in
                                          3-D mode, d²/1 in yaw-only (v17)
272     4      uint32    flags_ext         see below; ignore unknown bits (v18)
276     8×1    uint8     reserved[8]       zero on the wire.  A future scalar is
                                          carved from the FRONT of this, so
                                          no existing offset moves (v18)
284     4      uint32    crc32             IEEE 802.3 CRC of bytes 0–283
────────────────────────────────────────────────────────────────────
Total: 288 bytes
```
<!-- END GENERATED: packet-layout -->

**Consistency metrics (`nis_accel` / `nis_mag`).** Rolling averages (τ ≈ 30 s)
of d² = νᵀS⁻¹ν, accumulated *before* the Huber cap and *including* updates the
gross-outlier gate rejected — the cap censors d², so a post-cap average would be bounded
by construction and could never report the inconsistency it exists to measure.
The divisor is the effective degrees of freedom: 2 for the unit-vector updates
(normalising the measurement removes the radial component, so `E[d²] = 2` for a
consistent filter — see `docs/math.md`), 1 for the yaw-only scalar update. This
puts "consistent" at 1.0 on every channel.

Where `innov_weight`/`innov_reject` report how hard the robustness machinery is
*working*, these report whether the noise model itself is *right*: they keep
climbing after the Huber cap saturates. The two channels are separate because
they run at very different rates (833 Hz accel vs ~104 Hz mag), and a combined
average would be ~8:1 accel with the mag signal invisible.

### Flags Bitmask

<!-- BEGIN GENERATED: packet-flags -->
```text
bit 0   mag_valid          1 = mag calibrated and sensor healthy
bit 1   mag_set_reset      SET pulse issued within last read cycle (settling)
bit 2   fusion_converged   MEKF covariance trace < threshold (filter settled)
bit 3   accel_cal          Accel calibration applied
bit 4   gyro_cal           Gyro bias applied
bit 5   mag_cal            Mag hard/soft-iron cal applied
bit 6   motion             RETIRED — never set. Use accel_quiescence (a
                           continuous disturbance metric) or bit 13. The bit
                           is not reused: a stale consumer would read a new
                           meaning through the old name.
bit 7   fifo_overflow      Samples were lost upstream (gap!) — the IMU's
                           hardware FIFO overflowed, or the daemon's own ring
                           dropped because fusion could not keep up. Both are
                           missing samples, which is all this bit claims;
                           imud-status counts them and the daemon logs which
                           kind at 1 Hz. LATCHED for about two seconds after
                           the gap, not a single-packet pulse — the fusion
                           loop runs per sample, so at 833 Hz a momentary bit
                           is invisible to every consumer.
bit 8   startup            Gyro bias estimation still in progress
bit 9   shutdown           Final packet before clean exit
bit 10  declination_valid  Declination known; true heading = heading_deg + declination_deg
bit 11  heave_valid        Heave estimator settled (heave_m / heave_rate trustworthy)
bit 12  wave_valid         Sea-state statistics settled (wave/roll/pitch
                           height, period, and amplitude fields trustworthy)
bit 13  engine_on          Engine-vibration detector currently asserting
bit 14  state_reset        The MEKF found a non-finite value in its own state
                           and reset itself: nominal state and covariance back
                           to their initial values, re-aligning from the next
                           good accel/mag pair. LATCHED from the reset until
                           the filter next converges, not a single-packet
                           pulse — at up to 500 Hz a momentary bit is invisible
                           to a 1 Hz consumer. Bit 2 is clear for the same
                           span. Attitude remains readable throughout; it is
                           unconverged, not invalid.
bit 15  mag_uncal          Heading is being corrected by a magnetometer with
                           NO calibration. The uncorrected hard iron offsets
                           heading by a deviation that varies with heading —
                           bounded by asin(|b|/|H|) and repeatable, so a
                           heading-hold consumer can use it, but not a number
                           to navigate on. Mutually exclusive with bit 0, which
                           keeps its original meaning and stays clear whenever
                           there is no calibration, so a consumer that only
                           tests bit 0 is unaffected. Both clear means heading
                           is dead-reckoned from the gyro and drifts without
                           bound. Withdrawn automatically under gross iron
                           (mag_uncal_reject_frac).
        all 16 bits are assigned; further flags go in flags_ext
```
<!-- END GENERATED: packet-flags -->

### Extended Flags Bitmask

A second, 32-bit flag word at offset 272, added in wire v18 because `flags`
had assigned all sixteen of its bits. It is a **separate** word: bit 0 here
and bit 0 of `flags` are different flags.

A consumer **must ignore bits it does not recognise**. That is the contract
that lets imud define a new flag here without another wire version bump — only
a change to an existing field's meaning or position needs one.

<!-- BEGIN GENERATED: packet-flags-ext -->
```text
bit 0   mag_absent         No magnetometer is configured, so
                           heading is gravity-referenced only: it starts at
                           zero in the orientation imud booted in and dead
                           reckons from the gyro for the life of the run.
                           Distinct from bits 0 and 15 both being clear,
                           which is the same output from a magnetometer that
                           IS fitted and has gone stale or failed — that one
                           can recover, this one cannot.
        a consumer MUST ignore bits it does not know; adding one needs no bump
```
<!-- END GENERATED: packet-flags-ext -->

### Coordinate Frame

NED (North-East-Down), right-handed:

- +X: forward (vessel bow / camera forward axis)
- +Y: starboard / right
- +Z: down

Configurable to ENU via `coord_frame = "ENU"`.

The v1.2 diagnostic fields (`gyro_bias_*`, `gyro_bias_var_*`, `heave_rate`,
`accel_quiescence`) and the v14 sea-state / compass-health fields
(`wave_height_m`, `wave_period_s`, `roll_period_s`, `roll_amplitude`,
`pitch_period_s`, `pitch_amplitude`, `mag_anomaly`, `mag_residual`,
`innov_weight`, `innov_reject`) are
body-frame or frame-neutral scalars and are **not** affected by `coord_frame`.

### Near-vertical pitch: read the quaternion, not the Euler angles

`quat_w/x/y/z` is the filter's actual attitude state. `pitch`, `roll`, `yaw`,
`heading_deg` and `rate_of_turn` are derived from it as ZYX intrinsic Euler
angles, and that parameterisation is **singular at pitch = ±90°** by
construction — not as a filter limitation but as a property of any three-angle
representation. Approaching vertical, roll and yaw stop being separately
determined: they trade off against each other, so small attitude changes
produce large and noisy swings in both, and `heading_deg` and `rate_of_turn`
degenerate with them. Exactly at ±90° they are undefined.

Consumers that can encounter steep pitch — aircraft, gimbals, tilting camera
rigs, a handheld unit pointed up or down — must read `quat_*` and derive what
they need from it. The quaternion is never singular and is always valid. The
covariance in `cov[9]` is likewise an error covariance in the tangent space of
the quaternion, so it stays well-conditioned where the Euler angles do not.

This is not a concern for surface-vessel use, where pitch stays small; the
Euler fields are provided because they are what marine consumers and NMEA
sentences want. The quaternion is the authoritative field regardless.

-----

## 9. Configuration (TOML)

`/etc/imud/imud.conf` or `~/.config/imud/imud.conf`

```toml
[device]
i2c_bus        = "/dev/i2c-1"
gpio_chip      = "gpiochip0"     # Pi 4 = gpiochip0; Pi 5 (RP1) = gpiochip4

[imu]
# [restart]
driver         = "ism330dhcx"    # ism330dhcx | icm20948 | icm42688p | lsm6dso | lsm6dsox | sim
i2c_addr       = 0x6B            # 0x6A via jumper
int_gpio       = 17              # BCM GPIO for FIFO watermark interrupt; 0 = timer fallback
odr_hz         = 833             # > 0; rounds UP: 12/26/52/104/208/416/833/1660
accel_g        = 8               # full-scale: 2 | 4 | 8 | 16
gyro_dps       = 2000            # full-scale: 125 | 250 | 500 | 1000 | 2000 | 4000
fifo_wm        = 64              # FIFO watermark in sample-sets (ignored if no FIFO)

# Synthetic driver — no hardware needed. Set int_gpio = 0 for timer fallback.
# driver       = "sim"
# i2c_addr     = 0x00
# int_gpio     = 0

[mag]
# [restart]
driver         = "mmc5983ma"     # mmc5983ma | ak09916 | ak8963 | lis3mdl | lis2mdl | rm3100 | sim
i2c_addr       = 0x30
int_gpio       = 27              # BCM GPIO for measurement-done interrupt; 0 = timer fallback
odr_hz         = 100             # > 0; rounds UP: 1/10/20/50/100/200/1000
set_period_s   = 5.0             # degauss pulse interval, seconds (0 = disable)

# driver       = "sim"
# i2c_addr     = 0x00
# int_gpio     = 0
# set_period_s = 0.0

[fusion]
# MEKF noise densities. accel/mag/gyro-bias are datasheet-derived; mekf_gyro_noise
# is a tuned process noise (Q) held ABOVE the raw gyro spec — do NOT "correct" it to
# the datasheet number (degrades wave tracking; the test_fusion benchmark guards it).
# [hot]: all fields below take effect on SIGHUP without restarting the daemon.
mekf_gyro_noise     = 0.007      # rad/s/√Hz — tuned Q (raw sensor ≈ 1.2e-4 = 7 mdps/√Hz)
mekf_gyro_bias      = 0.00015   # rad/s      (in-run bias instability)
mekf_accel_noise    = 0.0022    # m/s²/√Hz  (ISM330DHCX: ~186 µg/√Hz × 9.81)
mekf_mag_noise      = 0.0004    # Gauss/√Hz (MMC5983MA: 0.4 mGauss RMS)
# Gauss-Markov wave-acceleration state (1.7). THIS is the knob for "the sea is
# rough", not mekf_accel_noise above.
#
# Wave-orbital acceleration contaminates the gravity measurement and is
# correlated over roughly a second, not white. Modelling it as a first-order
# Gauss-Markov process lets 833 correlated samples per second correctly stop
# adding information; without it the filter treats them as 833 independent
# measurements of gravity, collapses its own covariance and diverges.
#
#   mekf_wave_accel       steady-state sigma of the disturbance, m/s^2
#   mekf_wave_accel_tau_s its correlation time, s
#
# Either at 0 disables the state entirely and returns the pre-1.7 6-state
# filter. NOTE: unrelated to wave_tau_s below, which is the sea-state
# REPORTING window.
#
# The defaults were tuned over the 12-seed wave benchmark and agree with three
# independent measurements of the same seaway (docs/math.md 4.7.1). Measured
# effect vs 1.6: accel NIS 19.3/25.2 -> 1.01/0.69, NEES(trace) 18.3/7.8 ->
# 3.47/0.99, 3-D heading RMS 3.07 -> 0.83 deg. Run `imud-cal fit-ra` on a
# capture of YOUR worst conditions to check both values; it prints suggestions.
# Sigma too small is the failure mode that hurts, so round up. Long tau (> ~1 s)
# lets the state absorb genuine tilt error - the benchmark degrades sharply
# past 2 s.
mekf_wave_accel       = 0.8     # m/s^2  (RMS wave-orbital acceleration)
mekf_wave_accel_tau_s = 0.5     # s      (correlation time)

# Uncertainty of the magnetic reference's DIP angle. 3-D vector fusion only
# (mag_yaw_only = false), where the dip constrains roll/pitch. Admits the
# alignment's residual dip error into P via a rank-1 anisotropic noise term.
# 0 = the dip is exact (a WMM reference removes it at the source).
mekf_mag_dip_sigma_deg = 1.0    # degrees

mag_reject_gauss    = 0.05      # strong-anomaly cutoff (~10% of Earth field);
                                 # fine consistency is handled by χ² innovation gates
accel_skip_thresh   = 0.05      # skip accel update if ||a| - 1g| > 5% of g

# Heading-only mag fusion (marine default): mag never pulls on roll/pitch.
# Set false for full 3D vector fusion (clean installs with true 3D cal).
mag_yaw_only        = true

# Heave estimator time constant; feeds $PASHR heave + binary heave_m. 0 = off.
heave_tau_s         = 12.0
wave_tau_s          = 120.0

# Engine-vibration detection: when the EMA of (|a|-g)² exceeds
# engine_vibration_g2, the accel-update skip window widens to
# engine_accel_skip_thresh (so a running diesel doesn't starve the filter)
# and the accel measurement noise is inflated ×4.
engine_vibration_g2      = 0.0   # g² threshold; 0.0 = detection disabled
engine_accel_skip_thresh = 0.20  # skip threshold used while engine detected

[calibration]
file               = "/etc/imud/cal.json"   # written by imud-cal, read by imud
startup_settle_sec = 5.0         # discard sensor data for this long after start
gyro_bias_sec      = 2.0         # stationary still-window at startup (0 to skip)
align_window_sec   = 5.0         # accel+mag averaging window for initial alignment

[nmea]
# [restart]: enabled, dest_addr, dest_port, tcp_enabled, tcp_bind_addr, tcp_port
# [hot]:     rate_hz
enabled        = false           # UDP broadcast (off by default since 1.6)
rate_hz        = 10
dest_addr      = "255.255.255.255"
dest_port      = 10110
tcp_enabled    = false           # NMEA-over-TCP listener (plotters connect)
tcp_bind_addr  = "0.0.0.0"       # 127.0.0.1 = host-local only
tcp_port       = 10110

[highrate]
# [restart]: enabled, dest_addr, dest_port
# [hot]:     rate_hz
enabled        = false
rate_hz        = 500
dest_addr      = "239.255.0.1"   # multicast (224.0.0.0/4), broadcast, or unicast
dest_port      = 10111
coord_frame    = "NED"           # "NED" or "ENU"

[stream]
# [restart]: enabled, socket, tcp_enabled, tcp_bind_addr, tcp_port
# [hot]:     rate_hz
# Local AF_UNIX subscription stream — 276-byte binary packets (§8 format)
# over SOCK_STREAM; loss-free for same-host consumers, ≤ 8 subscribers.
# The one output enabled by default (1.6).
enabled        = true
socket         = "/run/imud/imud-stream.sock"
rate_hz        = 100
tcp_enabled    = false           # TCP listener: same framed packets, remote consumers
tcp_bind_addr  = "0.0.0.0"       # 127.0.0.1 = host-local only
tcp_port       = 10112

[mount]
# Board → body rotation expressed as Euler angles [roll, pitch, yaw] in degrees.
# R = Rz(yaw) × Ry(pitch) × Rx(roll).  Applied to all sensor vectors at runtime.
rotation_euler_deg = [0.0, 0.0, 0.0]
# Named presets: identity | yaw_90 | yaw_180 | yaw_270 |
#                roll_90 | roll_270 | pitch_90 | pitch_270
# An unrecognised preset name is a fatal config error, not a warning.
# preset = "identity"
# Alternatively the rotation may be given directly, row-major (v_body = R*v_board).
# Validated at load: R^T R = I and det(R) = +1; the daemon refuses to start
# otherwise. Last of the three mount keys to appear wins.
# rotation_matrix = [1,0,0, 0,1,0, 0,0,1]

[logging]
# [hot]
level          = "warn"          # "debug" | "info" | "warn" | "error"
                                 # warn = errors and warnings only (default; safe for SD cards)
                                 # info = adds [stats] heartbeat once per stats_hz seconds
file           = ""              # empty = write to stderr (captured by journald)
                                 # set a path to redirect stderr to a log file
stats_hz       = 1               # interval for periodic stats line to log

[position]
# [restart]
# Magnetic declination → true heading.  Three modes (highest priority wins):
#
#   1. WMM auto-compute: set lat_deg and lon_deg (both non-zero).
#      imud loads wmm_file and computes declination at startup and on SIGHUP.
#      Accurate to ±0.01° against NOAA WMM2025 test values.
#
#   2. Static override: set declination_deg only (lat/lon = 0.0).
#      Find your value at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
#      Positive = East (+), negative = West (-).
#
#   3. Disabled (default): all zero → no true heading output.
#
# When active, imud emits $HCHDT in NMEA,
# and fills the declination_deg field in the binary packet (FLAG_DECLINATION_VALID).
declination_deg  = 0.0           # static °E+; ignored when lat/lon set
lat_deg          = 0.0           # geodetic latitude  (+N / -S); 0 = WMM disabled
lon_deg          = 0.0           # geodetic longitude (+E / -W); 0 = WMM disabled
wmm_file         = ""            # "" = auto: /etc/imud/WMM.COF override, else
                                 # /usr/share/imud/WMM.COF (imud-wmm-data package)

# Live position sources (Step 3).  Enable one to receive automatic GPS-driven
# WMM declination updates as the vessel moves.
#
#   WMM is recomputed when position changes by ≥ 0.05° (≈ 5 km).
#   The stale-fix TTL (fix_max_age_h) counts from the last GPS fix received,
#   not from the last WMM recompute — an anchored vessel with continuous GPS
#   keeps its declination valid indefinitely; the TTL only fires during a
#   genuine GPS outage lasting longer than fix_max_age_h hours.
#
# Priority: gpsd (live stream) > SignalK (polled) > static lat/lon > static decl
#
# gpsd — persistent TCP JSON stream (port 2947); reconnects automatically.
# SignalK — HTTP/1.0 REST poll every 30 s; used as fallback when gpsd enabled
#           (rate-limited even when gpsd is down, so it always polls at ≤1/30 s).
gpsd_enabled     = false
gpsd_host        = "localhost"   # gpsd host
gpsd_port        = 2947          # gpsd port

signalk_enabled  = false
signalk_host     = "localhost"   # SignalK server host
signalk_port     = 3000          # SignalK HTTP port
signalk_path     = "/signalk/v1/api/vessels/self/navigation/position"

# Stale-fix TTL.  Once a live GPS fix has updated the declination, keep that value
# for up to fix_max_age_h hours even without a new fix (e.g. GPS outage at anchor).
# After the window expires, FLAG_DECLINATION_VALID clears and true-heading output stops.
# Set to 0 to never expire (useful for vessels that stay in one port for days).
fix_max_age_h    = 24.0          # hours; 0 = never expire
```

-----

## 10. Output Stream C — Local AF_UNIX subscription stream + TCP listener

**Socket:** `/run/imud/imud-stream.sock` (configurable, mode 0660,
owner `imud:imud` — a consumer joins the `imud` group to connect)
**TCP listener (optional):** `stream.tcp_bind_addr`:`stream.tcp_port`
(default `0.0.0.0:10112`), enabled by `stream.tcp_enabled` (off by default)
**Rate:** 100 Hz default per subscriber (hot-reloadable via `stream.rate_hz`)
**Format:** identical 276-byte binary packets as Stream B (§8)
**Enabled by:** `stream.enabled = true` (the default — the one output a
stock daemon provides)

Local consumers subscribe by connecting (up to 8 concurrent); each receives
every packet as an exact 276-byte frame — no datagram loss, self-framing via
the fixed size plus magic/CRC, so client-library validation (libimud, or
the deprecated `lib/imud_client.h`) works unchanged on 276-byte reads. Sends are non-blocking: a slow consumer gets
dropped packets (detectable as `imu_seq` gaps); a partial write would corrupt
framing, so it disconnects that subscriber instead. The daemon never blocks
on a consumer.

The TCP listener carries the exact same framed packets to remote consumers
(≤ 8 clients, same slow-client policy, served by the same `stream_out`
thread): `imud_connect_tcp` in libimud, `ImudClient.connect_tcp` in Python,
the imud-arduino library on ESP32-class boards
(github.com/richcreations/imud-arduino), or `nc HOST 10112` for a raw
capture. Both transports receive the final
`FLAG_SHUTDOWN` packet on clean daemon exit.

-----

## 11. Daemon Lifecycle

### Startup Sequence

```text
1.  Parse CLI args
2.  Load config from file; apply CLI overrides
3.  Redirect stderr to log file if logging.file is set
4.  Clock health check: CLOCK_REALTIME sanity, CLOCK_TAI offset via adjtimex
5.  Load cal.json → warn if missing (not fatal); set cal flags
6.  Open /dev/i2cN; probe + reset + init both sensors
7.  Gyro bias estimation (gyro_bias_sec still window):
    → collect gyro samples; compute mean; seed MEKF bias
    → skip if gyro_bias_sec = 0 or cal.json has gyro.bias
8.  Open UDP output sockets
9.  Configure GPIO edges via gpiod (rising-edge detection; libgpiod v1 or v2 auto-detected)
    → int_gpio = 0 → skip GPIO; reader threads poll at the batch period
10. Open AF_UNIX status socket at /run/imud/imud.sock (mode 0660)
11. Start threads: ism_reader, mag_reader, fusion, health (always);
    nmea_out if nmea.enabled or nmea.tcp_enabled;
    hirate_out if highrate.enabled;
    stream_out if stream.enabled or stream.tcp_enabled;
    position if position.gpsd_enabled or position.signalk_enabled
12. Write /run/imud/imud.pid
13. sd_notify("READY=1")
```

A config file that exists but contains bad values is fatal at startup: every
error in the file is reported and the daemon exits 1 rather than running on a
partially-applied config. A missing config file is not an error (defaults).

### SIGHUP Hot-Reload

The following fields take effect immediately on SIGHUP without restarting:

| Section     | Fields                                                                 |
|-------------|------------------------------------------------------------------------|
| `[fusion]`  | All noise/threshold params, `mag_yaw_only`, `mag_fuse_uncal`, `mag_uncal_reject_frac`, `heave_tau_s`, `wave_tau_s`, `mekf_wave_accel`, `mekf_wave_accel_tau_s`, `mekf_mag_dip_sigma_deg` — applied next predict |
| `[nmea]`    | `rate_hz`                                                              |
| `[highrate]`| `rate_hz`                                                              |
| `[stream]`  | `rate_hz`                                                              |
| `[logging]` | `level`, `stats_hz`; the log file is also reopened (logrotate)         |
| `[position]`| `declination_deg`, `lat_deg`/`lon_deg`, `wmm_file` (WMM recomputed) — applied only when no live position source (gpsd/SignalK) is enabled; a live source owns declination |

Fields not listed require a full daemon restart (chip reinit / socket rebind).
If the reloaded file fails to parse, the previously loaded config is kept and
the failure is logged.

### Shutdown (SIGTERM / SIGINT)

```text
1. Emit final high-rate packet with FLAG_SHUTDOWN set (bit 9)
2. Signal output threads to stop; join them
3. Signal sensor and fusion threads to stop; join them
4. Release I2C bus, GPIO lines
5. Close and unlink /run/imud/imud.sock
6. Remove /run/imud/imud.pid
```

### systemd Unit

```ini
[Unit]
Description=IMU Daemon — ISM330DHCX + MMC5983MA 9DoF Bridge
After=network.target

[Service]
Type=notify
ExecStart=/usr/local/bin/imud --config /etc/imud/imud.conf
ExecReload=/bin/kill -HUP $MAINPID
Restart=on-failure
RestartSec=3
WatchdogSec=10
User=imud
Group=imud
SupplementaryGroups=gpio i2c
AmbientCapabilities=CAP_NET_BIND_SERVICE

[Install]
WantedBy=multi-user.target
```

The primary group is `imud` — it owns `/run/imud` and both Unix domain
sockets, so a consumer joins `imud` rather than a hardware group. `gpio` and
`i2c` are supplementary, for `/dev/gpiochip*` and `/dev/i2c-*`; the unit will
not start if either is missing, so the installer creates them.

Setup: `sudo groupadd -r gpio; sudo groupadd -r i2c;
sudo useradd -r -s /sbin/nologin -G i2c,gpio imud`, plus the shipped udev rule
`60-imud.rules` granting those groups the device nodes (Raspberry Pi OS
provides equivalents in `99-com.rules`).

-----

## 12. CLI Tools

### imud

```text
imud [OPTIONS]
  --config PATH      Config file (default: /etc/imud/imud.conf)
  --replay FILE      Replay an .imucap capture through the sim driver
  --skip-bias-cal    Skip startup gyro bias estimation
  --no-nmea          Disable NMEA output stream
  --no-highrate      Disable high-rate binary stream
  --foreground       Accepted and ignored (always foreground under systemd)
  --version          Print version and exit
  -h, --help         Print this help and exit
```

### imud-cal

Talks directly to the hardware — stop the daemon before running.

```text
imud-cal [--config PATH] [--output PATH] <mode>

Modes:
  mag    Magnetometer hard/soft-iron calibration.
         Drive the vessel slowly through at least two full 360° circles.
         Press Ctrl-C when done. Must be done in-situ after final mounting.

  gyro   Gyroscope bias capture.
         Hold the sensor completely still for the collection window.

  accel  Accelerometer 6-position calibration (bench, before mounting).
         Follow prompts to orient the sensor on each face in turn.

  characterize Allan-variance noise analysis of a stationary capture
         (offline; requires --from FILE; record with [capture] enabled).
         Writes the informational "noise" section of cal.json.

  fit-temp Gyro bias/temperature fit from a warm-up capture (offline;
         requires --from FILE). Writes the "gyro_temp" section of
         cal.json, applied by the daemon at runtime.

Options:
  --config PATH   Config file (default: /etc/imud/imud.conf)
  --output PATH   Override cal.json output path from config
  --from FILE     .imucap capture to analyze (offline modes)

Output: written to cal_file from config (or --output path). If a cal.json
already exists, only the sections just calibrated are updated; other sections
are preserved.
```

Live display during `mag` mode:

```text
  Samples:  1247  [################........] 16/24 sectors
```

Results printed before saving:

```text
Results:
  Hard iron (µT):   [ -12.34,   8.76,  -3.21]
  Field radius:     47.3 µT  (typical: 25–65 µT)
  Soft iron diag:   [1.0234, 0.9781, 1.0000]  (Z: no 3D coverage, left as 1.0)
  RMS residual:     0.23 µT  (< 1.0 µT is good)
  Coverage:         20/24 sectors (83%)
```

### imud-mon

Live monitor for the two UDP output streams (NMEA and high-rate binary; the
AF_UNIX stream is not monitored) — a receive-side sanity check
run on any machine on the vessel LAN (no daemon socket needed). Reads port
numbers and multicast addresses from the config file, joins multicast groups
where needed, and prints a once-per-second snapshot line per stream.

```text
imud-mon [--config PATH] [nmea] [binary]

  --config PATH  Config file (default: /etc/imud/imud.conf)

  nmea    Monitor NMEA 0183 stream (UDP port 10110)
  binary  Monitor binary stream    (UDP port 10111)

  With no stream arguments both streams are shown.
```

Binary packets are validated (magic + CRC32) before display.

### imud-imutest

Driver validation against real hardware. Exercises a registered `imu_ops_t` /
`mag_ops_t` pair through the whole contract and writes a Markdown report.
Ships in the `imud-utils` package.

```
imud-imutest [--config PATH] [--report PATH]
             [--imu-driver NAME] [--mag-driver NAME|none]
             [--imu-addr HEX] [--mag-addr HEX] [--i2c-bus PATH]
             [--gpio-chip NAME] [--int-gpio N]
             [--odr HZ] [--accel-g N] [--gyro-dps N] [--fifo-wm N]
             [--passive|--faces|--gyro|--spin|--all] [--non-interactive]
             [--odr-window S] [--noise-window S] [--drdy-window S]
             [--turn-deg N] [--grav-tol M] [--odr-tol PCT]
             [--no-fs-sweep] [--no-overflow] [--no-regdiff]
             [--degauss]
             [--force] [--quiet] [--version] [-h|--help]
```

`--degauss` is a recovery action rather than a check: it pulses the
magnetometer's degauss coil once (RESET then SET) and prints |B| either side,
for a bridge left magnetised by a strong external field. It runs instead of the
phases, writes no report, and exits 0 if the field came back into a plausible
range, 2 if it did not.

Four phases. **Passive** needs no operator: probe and bogus-address rejection,
reset timing, a control-register snapshot diff around `init()` plus an
idempotency check, measured ODR against the rate the driver reports it will
program, FIFO depth and
deliberate overflow and recovery, `seq` monotonicity and gap accounting, the
error-return contract, noise floor and stuck-axis detection, gravity
magnitude, temperature plausibility, `chip_ts` presence/rate/wrap handling,
interrupt edge counting, and a full-scale sweep re-initialising at every
advertised range. **Six-face**, **gyro rotation** and **magnetometer spin** are
operator-guided and are what actually prove the chip-to-board axis remap; the
spin phase cross-checks the magnetometer heading against the gyro Z integral.

Refuses to run while the daemon is up (detected via the stream socket) unless
`--force`: both would drain the same FIFO, halving the sample rate each sees.

Exit status: `0` all pass, `1` usage/config/I-O error, `2` a check failed, `3`
warnings only, `130` aborted. Per-check definitions are in
`docs/imud-utils/spec.md`.

### imud-status

Connects to the AF_UNIX socket at `/run/imud/imud.sock` and prints the
daemon's current state. Accepts `--socket PATH` to use an alternate socket.

```text
Chip IDs:       ism330dhcx 0x6B   mmc5983ma 0x30
IMU ODR:        833 Hz  (FIFO watermark: 64 sample-sets)
Mag ODR:        100 Hz  (SET every 5 s)
Fusion:         MEKF converged  cov_trace=4.2e-06 rad2
Calibration:    accel yes  gyro yes  mag yes
Attitude:       pitch=-3.1  roll=9.5  heading=214.7 M
Declination:    +13.20 E  (true heading 227.9 T)
Heave:          +0.42 m
NMEA out:       10 Hz  (port 10110)
Hi-rate out:    disabled
IMU samples:    1234567  overflows: 0
Uptime:         00:04:32
```

### Bridges

The optional bridge daemons — `imud-signalk`, `imud-mqtt`, `imud-influxdb`, and
`imud-mavlink` — consume the AF_UNIX stream (Output Stream C) and republish it in
other protocols (Signal K deltas, MQTT topics, InfluxDB line protocol, MAVLink).
They are **not** part of imud's own wire protocol. Each is a separate optional
package with its own **output spec**, manual, and man pages
(`docs/imud-<name>/spec.md` — installed to `/usr/share/doc/imud-<name>/spec.md` —
plus `imud-<name>(8)`), and its own config file `/etc/imud/imud-<name>.conf`.

-----

## 13. Error Handling

|Condition                  |Response                                                   |
|---------------------------|-----------------------------------------------------------|
|I2C probe fails at startup |Fatal — exit 1                                             |
|Single I2C read timeout    |Skip sample, increment error counter                       |
|> 10 consecutive I2C errors|Attempt chip soft-reset; re-init registers                 |
|ISM330 FIFO overflow       |Set `FLAG_FIFO_OVERFLOW`; log at 1 Hz rate-limit           |
|MMC5983MA anomaly          |Reject if field magnitude ±50% from expected; use last good|
|GPIO edge missed           |Fallback after fifo_wm+int_grace periods; FIFO absorbs gap |
|UDP send failure           |Log first occurrence; continue (fire-and-forget)           |
|SIGHUP                     |Hot-reload fusion params and output rates; see §11         |

-----

## 14. Performance Targets

Latency is stated as a **chain**, because the figure a consumer needs — how old
a sample is when it reaches them — is a sum, and budgeting only the daemon's own
term made the total look ten times smaller than it is.

    FIFO residence   sample taken        -> read completed    set by configuration
    pipeline         read completed      -> state fused       imud's own cost
    transport        state fused         -> bytes on a socket
                     -----------------------------------------------------------
    sample age       what a consumer actually has

### FIFO residence — the dominant term, and it is yours to set

The reader blocks on the FIFO watermark interrupt, with a fallback of
`fifo_wm + int_grace` sample periods if the line is late. The fallback is by
construction *later* than the watermark it backs up, so on a healthy line the
**watermark always decides**, at every rate.

> **`fifo_wm` sets sample latency at every ODR.** The oldest sample in a burst
> is about `fifo_wm` sample periods old when the read completes, so residence is
> `fifo_wm / odr_hz`. Lowering `fifo_wm` is the only way to lower it.

Mean samples per drain is `fifo_wm` itself. On the ST parts a watermark of 8 or
more also batches the chip timestamp into the FIFO, and those words share the
budget: at `wm = 64` a drain carries about 63 sample-sets above 833 Hz, falling
to 43 at 13 Hz where the timestamp rate is a much larger fraction of the sample
rate. At `wm = 8` and `wm = 32` the count is exactly the watermark.

Measured on a Pi 5, `ism330dhcx` over SPI, 2026-08-25, one run per row, `ovf`
noted where non-zero. `fifo` percentiles are log₂-bucket upper edges — a p50 of
`65.5 ms` means the true value is in `[32.8, 65.5)` — so the `max` column is the
one to quote against a model.

|`odr_hz`|mean/drain|`fifo` p50/p99|`fifo` max|`fifo_wm / odr_hz`|`pipe` p50/p99|woken edge/timeout|
|---|---|---|---|---|---|---|
|13.016|42.7|1048.6 / 1048.6 ms|3102.9 ms|4917 ms|0.26 / 0.51 ms|24 / 0|
|26.031|51.0|1048.6 / 1048.6 ms|1846.2 ms|2459 ms|0.26 / 0.26 ms|31 / 0|
|52.063|56.3|1048.6 / 1048.6 ms|1034.5 ms|1229 ms|0.26 / 0.51 ms|57 / 0|
|104.125|60.0|262.1 / 1048.6 ms|552.5 ms|615 ms|0.26 / 0.51 ms|108 / 0|
|208.25|61.3|262.1 / 524.3 ms|282.5 ms|307 ms|0.26 / 0.51 ms|212 / 0|
|416.5|62.1|131.1 / 262.1 ms|144.2 ms|154 ms|0.26 / 0.51 ms|419 / 0|
|833|63.1|65.5 / 131.1 ms|**75.8 ms**|**76.8 ms**|0.26 / 0.51 ms|823 / 1|
|1666|63.1|32.8 / 65.5 ms|41.8 ms|38.4 ms|0.26 / 0.51 ms|1647 / 1|
|3332|63.1|16.4 / 32.8 ms|23.3 ms|19.2 ms|0.26 / 0.51 ms|3295 / 1|
|6664|63.8|16.4 / 16.4 ms|14.6 ms|9.6 ms|0.26 / 0.51 ms|6510 / 1|

All ten rows are woken by the watermark, not the fallback — one timeout per run
above 833 Hz and none below. `max` tracks `fifo_wm / odr_hz` closely from 13 Hz
to 833 Hz (75.8 against a predicted 76.8 at 833). Above 1666 Hz it runs *over*
the prediction, because the read itself is no longer negligible against a
shrinking budget: 14.6 ms against 9.6 at 6664 Hz.

And the watermark is the lever, at both ends of the ladder:

|`odr_hz`|`fifo_wm`|mean/drain|`fifo` max|`pipe` p50/p99|
|---|---|---|---|---|
|833|8|8.0|11.8 ms|0.06 / 0.13 ms|
|833|32|32.1|39.6 ms|0.26 / 0.26 ms|
|833|64|63.1|75.8 ms|0.26 / 0.51 ms|
|6664|8|8.0|6.6 ms|0.06 / 0.06 ms|
|6664|32|32.0|10.7 ms|0.26 / 0.26 ms|
|6664|64|63.8|14.6 ms|0.26 / 0.51 ms|

### The three terms

|Term|Target|Measured (Pi 5, 833 Hz, SPI)|
|---|---|---|
|FIFO residence|`fifo_wm / odr_hz`|75.8 ms max at `wm = 64`; 11.8 ms at `wm = 8` — SPI, 833 Hz|
|Pipeline (read → fused)|< 1.5 ms|**0.06 ms p50 / 0.26 ms p99** — met with room|
|Transport (fused → socket)|—|0.86 ms, from 99,961 packets on the TCP stream|
|**Total sample age**|**`fifo + pipe + transport`**|**33.79 ms p50** measured end to end on I²C at `wm = 64`|

Transport is measured as `now − ts_wall_ns` at the consumer and lands *above*
the sum of the first two, which is the signature of an anchor that is not late.

**On I²C the same configuration is several times worse**: `fifo` p50 ran
8.2–32.8 ms against SPI's 4.1. Transport choice moves this term more than any
tuning does.

**During a replay the two terms are on two different clocks**, and imud keeps
them apart deliberately. The FIFO term is rebuilt on the *recording* host's
clock, so it reproduces the residence the live daemon measured rather than
anything about the machine replaying it — that is the point of capturing
`chip_ts`. The pipeline term is imud's own cost on the machine actually
running, so it stays on that machine's clock. Measuring both against one clock
measures the age of the recording instead: a capture replayed days later
reported a FIFO latency of 416010576.8 ms. `imu_sample_t` therefore carries
both instants (`read_done_ns` and `host_done_ns`), identical outside replay.

### Everything else

|Metric|Target|Measured|
|---|---|---|
|CPU on Pi 4B at 833 Hz MEKF / 500 Hz out|< 10% one core|unmeasured|
|CPU on Pi Zero 2W at 416 Hz / 100 Hz out|< 30% one core|unmeasured|
|Memory footprint|< 14 MB RSS|unmeasured|
|High-rate UDP loss (localhost)|0%|unmeasured|
|Heading accuracy post-cal, benign environment|±1–2° (chip spec: ±0.5°)|unmeasured|

*Unmeasured* is stated rather than left blank: silence in this table used to
read as agreement, and two of its latency rows were wrong for a long time
because of it.

-----

## 15. C Implementation Notes

### Dependencies

```text
libc        # pthreads, sockets, time, signal
libgpiod    # GPIO edge detection — only external dependency
```

The MEKF, NMEA encoder, binary packer, TOML parser, and CRC32 are all
implemented directly. No filter library, no JSON library, no scripting runtime.

### I2C Burst Read Pattern

Use a combined write+read transaction to eliminate the repeated-start overhead
(saves ~40 µs per burst vs separate transactions). Drivers do not write this
themselves: they call the framing helpers in `src/drivers/bus_io.h`, which
hand the transfer to the host's backend behind `include/bus_backend.h`.

```c
static inline int i2c_burst_read(const imud_bus_t *b, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    uint8_t r = reg;
    return bus_be_i2c_xfer(b, &r, 1, buf, len);
}
```

`src/bus_linux.c` turns that into one `ioctl(I2C_RDWR)` of two messages.
`src/bus_null.c` fails it with `ENOSYS`, which is what lets the tree build on
a host with no i2c-dev.

### Build

```makefile
CC      = gcc
CFLAGS  = -std=c11 -O2 -Wall -Wextra -pthread -Iinclude -D_GNU_SOURCE
LDFLAGS = -lgpiod -lm

# Auto-detect libgpiod version; v1 is Bookworm default, v2 on newer distros.
GPIOD_MAJ := $(shell pkg-config --modversion libgpiod 2>/dev/null | cut -d. -f1)
ifeq ($(GPIOD_MAJ),2)
    CFLAGS += -DGPIOD_V2
endif

DRIVER_SRCS = src/drivers/ism330dhcx.c src/drivers/mmc5983ma.c \
              src/drivers/icm20948.c   src/drivers/ak09916.c   \
              src/drivers/icm42688p.c  src/drivers/lsm6dso.c   \
              src/drivers/lis3mdl.c    src/drivers/lis2mdl.c   \
              src/drivers/mpu925x.c    src/drivers/ak8963.c    \
              src/drivers/rm3100.c     src/drivers/sim.c

all: imud imud-cal imud-status imud-mon

imud:        $(IMUD_OBJS) src/main.c
imud-cal:    $(CAL_OBJS)  src/cal_main.c
imud-status: src/status_main.c
imud-mon:    src/config.o src/mon_main.c
```

`make install` also installs the five man pages (imud.8, imud-cal.8,
imud.conf.5, imud-status.1, libimud.3); imud-mon.1 and imud-imutest.8 ship
with `make install-utils` (the imud-utils package), and each bridge's pair
with its own `make install-<name>`. WMM coefficient data is a separate
target — `make install-wmm-data` puts `data/WMM.COF` → `/usr/share/imud/` so
it can be packaged (imud-wmm-data) and updated independently of the daemon;
an operator-supplied `/etc/imud/WMM.COF` overrides it.

Cross-compile for Pi from x86 host:

```bash
apt install gcc-aarch64-linux-gnu
make CC=aarch64-linux-gnu-gcc
```

-----

## 16. Timestamp Architecture — Camera Correlation

### System Context

```text
  GNSS receiver + PPS
         │
  ┌──────▼──────────────────┐
  │  Time Server            │  Stratum 1, vessel LAN
  │  gpsd + chrony          │  ±1–2 µs to GPS
  └──────┬──────────────────┘
         │ Ethernet
    ┌────┴──────────────────────────────┐
    │                                   │
┌───▼──────┐   ┌────────────┐   ┌───────▼──────┐
│ IMU Pi   │   │ Camera Pi  │   │  Consumer    │
│ chrony   │   │ chrony     │   │  chrony      │
│ ±10–50µs │   │ ±10–50µs   │   │  ±10–50µs   │
│          │   │            │   │              │
│ imud     │   │ libcamera  │   │ correlates   │
│ Stream A │   │ SensorTs   │   │ by wall-clock│
│ Stream B │   │            │   │              │
└──────────┘   └────────────┘   └──────────────┘
```

All nodes discipline `CLOCK_REALTIME` to a single vessel LAN time server
running gpsd + chrony against a GNSS+PPS source. No node needs its own GPS
receiver. Client nodes sync over Ethernet and achieve ±10–50 µs offset from
the time server.

### Clock Selection

|Clock                |Monotonic  |Leap-second safe|Wall-time   |Use here                                 |
|---------------------|-----------|----------------|------------|-----------------------------------------|
|`CLOCK_MONOTONIC`    |✓          |✓               |✗ — no epoch|✗ Can't correlate across reboots or nodes|
|`CLOCK_MONOTONIC_RAW`|✓          |✓               |✗           |✗ Not NTP-disciplined, drifts freely     |
|`CLOCK_REALTIME`     |✗ leap step|✗ ambiguous     |✓           |✓ **Primary — use this**                 |
|`CLOCK_TAI`          |✓          |✓               |✓+offset    |✓ **Secondary — include as cross-check** |

**Use `CLOCK_REALTIME`** as the primary timestamp source. With GNSS+PPS
discipline, both nodes' `CLOCK_REALTIME` values are steered to within a few
microseconds of true UTC.

**Also include `CLOCK_TAI`** as a secondary field. CLOCK_TAI provides
monotonicity across the rare leap-second boundary at zero cost. Requires chrony
to set `tai_offset` — verify with `adjtimex`. If `tai_offset` is zero,
CLOCK_TAI equals CLOCK_REALTIME; the daemon logs a startup warning in this case.

**Internal use of `CLOCK_MONOTONIC`:** The IMU ring buffer condvar timeout uses
`CLOCK_MONOTONIC` (not CLOCK_REALTIME) to avoid a 1-second spin at leap seconds.

### IMU Timestamp Strategy

The ISM330DHCX embeds a 32-bit hardware timestamp counter (25 µs resolution,
enabled via `CTRL10_C.TIMESTAMP_EN`). The chip counter is the high-precision
per-sample reference; `CLOCK_REALTIME` is the shared epoch. They are bridged
once at startup and re-anchored every 60 seconds:

```text
Anchor procedure:
  t_wall_before  = clock_gettime(CLOCK_REALTIME)
  t_mono         = clock_gettime(CLOCK_MONOTONIC)
  t_chip         = read ISM330 TIMESTAMP registers
  t_wall_after   = clock_gettime(CLOCK_REALTIME)
  anchor_wall_ns = (t_wall_before + t_wall_after) / 2
  anchor_chip    = t_chip
  anchor_gen    += 1

  # Measure the counter's real period against the previous anchor, and keep
  # using the declared one until there is a measurement.  MONOTONIC, so an NTP
  # step inside the window is not read as the oscillator drifting.
  if have_previous_anchor:
      d_ticks = (t_chip - prev_chip) & 0xFFFFFFFF
      d_mono  = t_mono - prev_mono
      if d_ticks >= 1000 and d_mono >= 20e9:          # long enough to mean something
          meas = d_mono / d_ticks
          if 0.9 * 25_000 <= meas <= 1.1 * 25_000:    # else: reset counter, lost wrap
              tick_ns = meas if tick_ns is None else tick_ns + (meas - tick_ns) / 4
  prev_chip, prev_mono = t_chip, t_mono
```

Per-sample wall timestamp:

```python
def chip_to_wall_ns(chip_ts: int) -> int:
    delta = (chip_ts - anchor_chip) & 0xFFFFFFFF  # 32-bit rollover safe (~29.8 hr)
    return anchor_wall_ns + delta * (tick_ns or 25_000)
```

**The declared 25 µs is nominal, and the oscillator is not that good.** A
measured ISM330DHCX ran **4.08% fast** — 40800 ppm, not the 100 ppm this budget
originally assumed. Scaling ticks by the declared period therefore ran sample
timestamps ~2.4 s ahead over a 60 s anchor epoch before snapping back at the
re-anchor, and made every per-sample `dt` 4% long, which scales integrated
rotation by the same factor. Measuring the period rather than trusting it is
what keeps the budget below correct; `imud-imutest`'s `imu.chipts.wall` grades
the same quantity on a bench, and `imud` logs the measured period at startup
when it is more than 1% off nominal.

**Total timestamp uncertainty budget** (with the measured period in use):

|Source                                                  |Contribution                   |
|--------------------------------------------------------|-------------------------------|
|GNSS+PPS clock discipline (each node)                   |< 5 µs typical, < 20 µs worst  |
|Anchor I2C read half-width                              |~150–300 µs (dominant)         |
|ISM330 timer resolution                                 |25 µs                          |
|Residual period error after measurement (60 s × ~200 ppm)|< 12 ms → corrected at re-anchor|
|**Total per-sample, steady state**                      |**~300–500 µs**                |

Without the measurement that fourth row is 60 s × 40800 ppm ≈ **2.4 s**, which
dominates everything above it by four orders of magnitude.

### Camera Timestamp Correlation

libcamera exposes `controls::SensorTimestamp` per frame in nanoseconds, sourced
from `CLOCK_MONOTONIC` (verify — some Pi kernel versions use
`CLOCK_MONOTONIC_RAW`, which drifts freely and requires additional correction).

**Clock bridge on the camera side:**

```python
def get_clock_offset_ns() -> int:
    """Returns (CLOCK_REALTIME - CLOCK_MONOTONIC) in nanoseconds."""
    t_real = time.clock_gettime_ns(time.CLOCK_REALTIME)
    t_mono = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    return t_real - t_mono   # stable constant within a session

MONO_TO_REAL_OFFSET_NS = get_clock_offset_ns()

def sensor_ts_to_realtime_ns(sensor_ts_ns: int) -> int:
    return sensor_ts_ns + MONO_TO_REAL_OFFSET_NS
```

### Consumer Correlation Algorithm

```python
def correlate_frame(frame_realtime_ns: int,
                    imu_ring: deque[ImuPacket]) -> Attitude:

    # SensorTimestamp is start-of-readout; add half exposure for frame center
    exposure_ns = frame_metadata.get(controls.ExposureTime) * 1000
    frame_center_ns = frame_realtime_ns + exposure_ns // 2

    # Find bracketing IMU packets
    before = next((p for p in reversed(imu_ring)
                   if p.ts_wall_ns <= frame_center_ns), None)
    after  = next((p for p in imu_ring
                   if p.ts_wall_ns > frame_center_ns), None)

    if before is None or after is None:
        return None

    alpha = ((frame_center_ns - before.ts_wall_ns) /
             (after.ts_wall_ns - before.ts_wall_ns))

    # Slerp quaternion — do NOT lerp Euler angles (breaks at 0°/360° wrap)
    q = slerp(before.quat, after.quat, alpha)
    return Attitude(quat=q, pitch=quat_to_pitch(q),
                    roll=quat_to_roll(q), yaw=quat_to_yaw(q))
```

At 500 Hz IMU output (2 ms window) and ~500 µs timestamp uncertainty:

```text
Δattitude ≈ gyro_rate × timestamp_error ≈ 50°/s × 0.0005 s ≈ 0.025°
```

Well below any camera/stabilization requirement.

### Startup Clock Health Check

```text
[imud] CLOCK_REALTIME:    2026-05-14T09:41:22Z  OK
[imud] CLOCK_TAI offset:  37 s  OK  (tai_offset set by chrony)
[imud] WARNING: CLOCK_TAI offset is 0 — chrony has not set tai_offset.
         Is 'leapsectz right/UTC' in chrony.conf? ts_tai_ns unreliable.
```
