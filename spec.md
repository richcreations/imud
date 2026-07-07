# IMU Daemon Specification

## `imud` — SparkFun 9DoF IMU Bridge for NMEA 0183 + Machine Vision

**Version:** 1.0  
**Hardware:** SparkFun Qwiic 9DoF — ISM330DHCX + MMC5983MA (SEN-19895)  
**Platform:** Raspberry Pi (any model with I2C), Linux  
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
|`nmea_out`    |100 ms timer (10 Hz)        |Snapshot fused state, encode NMEA sentences, broadcast UDP               |
|`hirate_out`  |2 ms timer (500 Hz)         |Pack latest fused state + mag; send UDP                                  |
|`json_out`    |rate_hz timer (100 Hz)      |Encode fused state as NDJSON; send UDP                                   |
|`stream_out`  |rate_hz timer (100 Hz)      |Serve 192-byte binary packets to local AF_UNIX subscribers               |
|`health`      |1 s timer + status socket   |Stats logging, serve imud-status queries, systemd watchdog heartbeat     |
|`position`    |gpsd TCP stream / 30 s poll |Read GPS fixes from gpsd or SignalK, recompute WMM declination on ≥5 km move, push into fusion |

`nmea_out` is only started when `nmea.enabled = true`.
`hirate_out` is only started when `highrate.enabled = true`.
`json_out` is only started when `json.enabled = true`.
`stream_out` is only started when `stream.enabled = true`.
`position` is only started when `position.gpsd_enabled` or
`position.signalk_enabled` is true (see §9 `[position]`).

-----

## 4. Driver Abstraction Layer

The `ism_reader` and `mag_reader` threads call through a chip operations struct
rather than reading registers directly. Swapping hardware means selecting a
different driver in config — the rest of the daemon is unchanged.

### IMU Driver Interface

```c
typedef struct {
    const char *name;               /* matches config [imu] driver = "..." */

    int (*probe)    (int fd, uint8_t addr);
    int (*reset)    (int fd, uint8_t addr);
    int (*init)     (int fd, uint8_t addr, const imu_cfg_t *cfg);

    /* Drain FIFO or read data-ready registers.
       Returns samples as calibrated SI (m/s², rad/s). Fills *n. */
    int (*read)     (int fd, uint8_t addr,
                     imu_sample_t *buf, int max, int *n);

    bool  has_fifo;                 /* false → single-sample DRDY read per wake */
    bool  has_hw_timestamp;         /* chip has internal sample timer */
    int   supported_odr_hz[16];     /* ascending list, 0-terminated */
    int   supported_accel_g[8];
    int   supported_gyro_dps[8];
} imu_ops_t;
```

### Magnetometer Driver Interface

```c
typedef struct {
    const char *name;               /* matches config [mag] driver = "..." */

    int (*probe)    (int fd, uint8_t addr);
    int (*reset)    (int fd, uint8_t addr);
    int (*init)     (int fd, uint8_t addr, const mag_cfg_t *cfg);
    int (*read)     (int fd, uint8_t addr, mag_sample_t *out);
    int (*set_reset)(int fd, uint8_t addr); /* NULL if chip has no degauss coil */

    bool  has_interrupt;
    bool  has_set_reset;
    int   supported_odr_hz[16];
} mag_ops_t;
```

### Driver Registry

Drivers are registered in a static table in `drivers.c`. The daemon looks up
the configured name at startup and fatals if no match is found.

```c
static const imu_ops_t *imu_registry[] = {
    &ism330dhcx_ops,   /* ISM330DHCX — reference hardware */
    &icm20948_ops,     /* ICM-20948 — alternate IMU        [experimental] */
    &icm42688p_ops,    /* ICM-42688-P                      [experimental] */
    &lsm6dso_ops,      /* LSM6DSO                          [experimental] */
    &lsm6dsox_ops,     /* LSM6DSOX (alias of lsm6dso)      [experimental] */
    &sim_imu_ops,      /* synthetic driver for testing without hardware */
    NULL
};

static const mag_ops_t *mag_registry[] = {
    &mmc5983ma_ops,    /* MMC5983MA — reference hardware */
    &ak09916_ops,      /* AK09916 — compass on ICM-20948   [experimental] */
    &lis3mdl_ops,      /* LIS3MDL                          [experimental] */
    &lis2mdl_ops,      /* LIS2MDL                          [experimental] */
    &sim_mag_ops,      /* synthetic driver for testing without hardware */
    NULL
};
```

Drivers marked `[experimental]` set `experimental = true` in their ops struct
and have not been validated on real silicon; the daemon logs a warning at
startup when one is selected.

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

Expected fusion output: heading increases ~6°/s, roll tracks ±4°, pitch tracks ±2°,
rate_of_turn ~360 deg/min with small wave-induced oscillation.

Set `int_gpio = 0` for both sensors when using `sim` — the reader threads use
their 10 ms timer fallback instead of GPIO edges.

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

### State and Noise Model

The error-state is a 3-vector δθ (small-angle rotation error) plus a 3-vector
gyro bias estimate b — 6 states total. The full quaternion q is maintained
separately on the SO(3) manifold and updated multiplicatively, avoiding the
rank-deficiency problem of naively applying EKF to a 4-component quaternion.

Noise parameters map directly to the sensor datasheets and are **not tuning
knobs** — they are physical constants. The only genuine tuning decision is
`mag_reject_gauss`, which controls how aggressively to reject magnetometer
samples that deviate from the expected field magnitude after calibration.

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
  }
}
```

Applied corrections:

```text
accel_cal  = (accel_raw  − accel_offset) × accel_scale   (per-axis)
gyro_cal   = gyro_raw − gyro_bias                         (subtracted in MEKF predict)
mag_cal    = soft_iron × (mag_raw − hard_iron)            (matrix × vector)
mag_z      = −raw_mag_z                                   (Z sign flip, always)
```

The soft-iron matrix from `imud-cal mag` carries a full 2×2 horizontal block
(including the cross term) from a least-squares ellipse fit of the swing
circle — a distortion ellipse whose axes are rotated relative to the sensor
axes cannot be corrected by per-axis scales alone. Z stays 1.0 unless the
data has real 3D coverage.

### Rough-sea mechanics (v1.1 fusion revision)

- **Joseph-form covariance updates** everywhere, plus explicit
  symmetrization after predict — P stays symmetric positive-definite in
  float arithmetic over multi-day runs.
- **Per-sample dt**: the predict step integrates over the measured
  inter-sample interval from hardware timestamps (clamped to [0.5×, 2×]
  nominal), so IMU oscillator tolerance does not scale integrated rotation.
- **χ² innovation handling** on accel and mag updates: the normalised
  innovation distance d² = νᵀS⁻¹ν is capped Huber-style at the χ² 99 %
  quantile (influence bounded, information still flows in a steady seaway)
  and rejected outright beyond 9× the gate. Because S contains P, the gate
  self-regulates: inactive while the filter is still acquiring.
- **`mag_yaw_only` (default true)**: scalar heading update via
  H = [−(R̂ e_D)ᵀ | 0]; the magnetometer never pulls on roll/pitch.
- **m_ref self-healing**: the magnetic reference's magnitude and dip are
  slowly (τ ≈ 5 min) re-estimated from attitude-independent invariants
  (field magnitude; angle between mag vector and the accel-measured gravity
  direction), but only while the platform is quiescent
  (EMA of (|a|/g−1)² < 2·10⁻⁴, τ ≈ 2 s). The horizontal direction — the
  heading anchor — is never adapted (no gauge feedback).
- **Alignment averaging**: initial tilt/heading alignment averages ~1 s of
  accel and all mag samples in that window instead of one instantaneous
  reading.
- **Rate of turn** is the true Euler yaw rate
  ψ̇ = (ω_y·sinφ + ω_z·cosφ)/cosθ, not raw body-Z rate (falls back to ω_z
  near ±90° pitch).
- **Heave** (§7 PASHR / §10 JSON): leaky double integration of NED vertical
  acceleration through a true output high-pass (exact zero at DC); triple
  pole at 1/τ ⇒ allow ~10·τ settling after startup.

Synthetic rough-sea benchmark (±15° roll @ 0.2 Hz, ±8° pitch, 0.12 g
orbital accel, run by `test_fusion`): attitude RMS 7.1° → 2.7°, heading RMS
4.9° → 2.4° versus the v1.0 filter.

-----

## 7. Output Stream A — NMEA 0183 UDP

**Port:** 10110 UDP broadcast (`255.255.255.255`) / configurable  
**Rate:** 10 Hz  
**Format:** NMEA 0183 ASCII sentences, `<CR><LF>` terminated, correct checksums

Any chartplotter, MFD, multiplexer, OpenCPN, Expedition, SignalK server, or
logging tool on the vessel LAN receives this without configuration.

### Sentence Set

Four sentences are emitted per update cycle (five when magnetic declination is known):

**`$PASHR` — Attitude (primary)**

```text
$PASHR,HHH.H,M,RRR.R,PPP.P,0.0,ra.r,pa.p,0,A,,*hh<CR><LF>

Fields:
  HHH.H   Magnetic heading (0–360°)
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
```

**`$HCHDM` — Magnetic Heading**

```text
$HCHDM,HHH.H,M*hh<CR><LF>
Example: $HCHDM,214.7,M*3C<CR><LF>
```

**`$HCHDG` — Heading, Deviation, Variation**

Deviation fields are always empty (hard/soft-iron calibration is applied
upstream). Variation fields carry the WMM/static declination when known
(`E` = east/+, `W` = west/-) and are empty otherwise — consumers like
Signal K read magnetic variation from this sentence directly.

```text
$HCHDG,HHH.H,,,VV.V,a*hh<CR><LF>

Example: $HCHDG,214.7,,,13.2,E*hh<CR><LF>   (declination known)
         $HCHDG,214.7,,,,*hh<CR><LF>        (declination unknown)
```

**`$HCHDT` — True Heading** *(emitted only when `FLAG_DECLINATION_VALID` is set)*

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
**Format:** Binary, little-endian, 192 bytes fixed  
**Wire load:** ~96 KB/s

### Packet Layout

```text
Offset  Bytes  Type      Field             Notes
──────────────────────────────────────────────────────────────────
 0      4      uint32    magic             0x494D5544 (“IMUD”)
 4      2      uint16    version           = 10  (v1.0; encoded as major*10+minor)
 6      2      uint16    flags             see below
 8      8      uint64    ts_wall_ns        CLOCK_REALTIME ns
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
188     4      uint32    crc32             IEEE 802.3 CRC of bytes 0–187
────────────────────────────────────────────────────────────────────
Total: 192 bytes
```

### Flags Bitmask

```text
bit 0   mag_valid          1 = mag calibrated and sensor healthy
bit 1   mag_set_reset      SET pulse issued within last read cycle (settling)
bit 2   fusion_converged   MEKF covariance trace < threshold (filter settled)
bit 3   accel_cal          Accel calibration applied
bit 4   gyro_cal           Gyro bias applied
bit 5   mag_cal            Mag hard/soft-iron cal applied
bit 6   motion             Significant motion detected
bit 7   fifo_overflow      ISM330 FIFO overflowed since last packet (gap!)
bit 8   startup            Gyro bias estimation still in progress
bit 9   shutdown           Final packet before clean exit
bit 10  declination_valid  Declination known; true heading = heading_deg + declination_deg
bits 11–15  reserved
```

### Coordinate Frame

NED (North-East-Down), right-handed:

- +X: forward (vessel bow / camera forward axis)
- +Y: starboard / right
- +Z: down

Configurable to ENU via `coord_frame = "ENU"`.

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
odr_hz         = 833             # driver rounds to nearest: 12/26/52/104/208/416/833/1660
accel_g        = 8               # full-scale: 2 | 4 | 8 | 16
gyro_dps       = 2000            # full-scale: 125 | 250 | 500 | 1000 | 2000 | 4000
fifo_wm        = 64              # FIFO watermark in sample-sets (ignored if no FIFO)

# Synthetic driver — no hardware needed. Set int_gpio = 0 for timer fallback.
# driver       = "sim"
# i2c_addr     = 0x00
# int_gpio     = 0

[mag]
# [restart]
driver         = "mmc5983ma"     # mmc5983ma | ak09916 | lis3mdl | lis2mdl | sim
i2c_addr       = 0x30
int_gpio       = 27              # BCM GPIO for measurement-done interrupt; 0 = timer fallback
odr_hz         = 100             # driver rounds to nearest: 1/10/20/50/100/200/1000
set_period_s   = 5.0             # degauss pulse interval, seconds (0 = disable)

# driver       = "sim"
# i2c_addr     = 0x00
# int_gpio     = 0
# set_period_s = 0.0

[fusion]
# Noise parameters from ISM330DHCX and MMC5983MA datasheets — not tuning knobs.
# [hot]: all fields below take effect on SIGHUP without restarting the daemon.
mekf_gyro_noise     = 0.007      # rad/s/√Hz  (ISM330DHCX: 7 mdps/√Hz)
mekf_gyro_bias      = 0.00015   # rad/s      (in-run bias instability)
mekf_accel_noise    = 0.0022    # m/s²/√Hz  (ISM330DHCX: ~186 µg/√Hz × 9.81)
mekf_mag_noise      = 0.0004    # Gauss/√Hz (MMC5983MA: 0.4 mGauss RMS)
mag_reject_gauss    = 0.05      # strong-anomaly cutoff (~10% of Earth field);
                                 # fine consistency is handled by χ² innovation gates
accel_skip_thresh   = 0.05      # skip accel update if ||a| - 1g| > 5% of g

# Heading-only mag fusion (marine default): mag never pulls on roll/pitch.
# Set false for full 3D vector fusion (clean installs with true 3D cal).
mag_yaw_only        = true

# Heave estimator time constant; feeds $PASHR heave + JSON heave_m. 0 = off.
heave_tau_s         = 12.0

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

[nmea]
# [restart]: enabled, dest_addr, dest_port
# [hot]:     rate_hz
enabled        = true
rate_hz        = 10
dest_addr      = "255.255.255.255"
dest_port      = 10110

[highrate]
# [restart]: enabled, dest_addr, dest_port
# [hot]:     rate_hz
enabled        = false
rate_hz        = 500
dest_addr      = "239.255.0.1"   # multicast (224.0.0.0/4), broadcast, or unicast
dest_port      = 10111
coord_frame    = "NED"           # "NED" or "ENU"

[json]
# [restart]: enabled, dest_addr, dest_port
# [hot]:     rate_hz
enabled        = false
rate_hz        = 100
dest_addr      = "255.255.255.255"
dest_port      = 10112

[stream]
# [restart]: enabled, socket
# [hot]:     rate_hz
# Local AF_UNIX subscription stream — 192-byte binary packets (§8 format)
# over SOCK_STREAM; loss-free for same-host consumers, ≤ 8 subscribers.
enabled        = false
socket         = "/run/imud/imud-stream.sock"
rate_hz        = 100

[mount]
# Board → body rotation expressed as Euler angles [roll, pitch, yaw] in degrees.
# R = Rz(yaw) × Ry(pitch) × Rx(roll).  Applied to all sensor vectors at runtime.
rotation_euler_deg = [0.0, 0.0, 0.0]
# Named presets: identity | yaw_90 | yaw_180 | yaw_270 |
#                roll_90 | roll_270 | pitch_90 | pitch_270
# preset = "identity"

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
# When active, imud emits $HCHDT in NMEA, "true_heading_deg" in JSON,
# and fills the declination_deg field in the binary packet (FLAG_DECLINATION_VALID).
declination_deg  = 0.0           # static °E+; ignored when lat/lon set
lat_deg          = 0.0           # geodetic latitude  (+N / -S); 0 = WMM disabled
lon_deg          = 0.0           # geodetic longitude (+E / -W); 0 = WMM disabled
wmm_file         = "/etc/imud/WMM.COF"  # WMM2025, valid 2025–2030; replace every ~5 yrs

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

## 10. Output Stream C — NDJSON UDP (optional)

**Port:** 10112 UDP (configurable)
**Rate:** 100 Hz default (hot-reloadable via `json.rate_hz`)
**Format:** NDJSON — one JSON object per UDP datagram, newline-terminated
**Enabled by:** `json.enabled = true` in config (disabled by default)

One object per datagram:

```json
{
  "ts":          1715000000.123,
  "heading_deg": 214.7,
  "pitch_deg":   -3.1,
  "roll_deg":     9.5,
  "rot_dpm":     -6.2,
  "heave_m":     0.42,
  "quat":        [0.998, 0.001, -0.054, 0.031],
  "gyro_bias":   [0.00012, -0.00008, 0.00003],
  "cov_trace":   4.2e-6,
  "flags":       20,
  "declination_deg": 13.2,
  "true_heading_deg": 227.9
}
```

| Field | Units / notes |
| --- | --- |
| `ts` | UNIX epoch seconds (float), from `CLOCK_REALTIME` |
| `heading_deg` | 0–360° magnetic |
| `pitch_deg` | degrees, + = bow up |
| `roll_deg` | degrees, + = starboard up |
| `rot_dpm` | rate of turn deg/min, + = turning right |
| `heave_m` | vertical displacement, metres, + up (0.0 when `heave_tau_s = 0`) |
| `quat` | unit quaternion `[w, x, y, z]`, body→NED |
| `gyro_bias` | current MEKF bias estimate, rad/s |
| `cov_trace` | trace of attitude error covariance, rad² |
| `flags` | same bitmask as §8 binary packet |
| `declination_deg` | °E+; present only while `FLAG_DECLINATION_VALID` is set |
| `true_heading_deg` | 0–360° true; present only while `FLAG_DECLINATION_VALID` is set |

Useful for ROS2 bridges, web dashboards, SignalK plugins, or any consumer that prefers text over binary.

### Output Stream D — AF_UNIX subscription stream (optional)

**Socket:** `/run/imud/imud-stream.sock` (configurable, mode 0660)
**Rate:** 100 Hz default per subscriber (hot-reloadable via `stream.rate_hz`)
**Format:** identical 192-byte binary packets as Stream B (§8)
**Enabled by:** `stream.enabled = true` (disabled by default)

Local consumers subscribe by connecting (up to 8 concurrent); each receives
every packet as an exact 192-byte frame — no datagram loss, self-framing via
the fixed size plus magic/CRC, so `lib/imud_client.h` validation works
unchanged on 192-byte reads. Sends are non-blocking: a slow consumer gets
dropped packets (detectable as `imu_seq` gaps); a partial write would corrupt
framing, so it disconnects that subscriber instead. The daemon never blocks
on a consumer.

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
    → int_gpio = 0 → skip GPIO; reader threads use 10 ms timer fallback
10. Open AF_UNIX status socket at /run/imud/imud.sock (mode 0660)
11. Start threads: ism_reader, mag_reader, fusion, health (always);
    nmea_out if nmea.enabled; hirate_out if highrate.enabled;
    json_out if json.enabled; stream_out if stream.enabled;
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
| `[fusion]`  | All noise/threshold params, `mag_yaw_only`, `heave_tau_s` — applied next predict |
| `[nmea]`    | `rate_hz`                                                              |
| `[highrate]`| `rate_hz`                                                              |
| `[json]`    | `rate_hz`                                                              |
| `[stream]`  | `rate_hz`                                                              |
| `[logging]` | `stats_hz`                                                             |
| `[position]`| `declination_deg`, `lat_deg`/`lon_deg` (WMM recomputed) — applied only when no live position source (gpsd/SignalK) is enabled; a live source owns declination |

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
Group=gpio
SupplementaryGroups=i2c
AmbientCapabilities=CAP_NET_BIND_SERVICE

[Install]
WantedBy=multi-user.target
```

Setup: `sudo useradd -r -s /sbin/nologin -G i2c,gpio imud`

-----

## 12. CLI Tools

### imud

```text
imud [OPTIONS]
  --config PATH      Config file (default: /etc/imud/imud.conf)
  --skip-bias-cal    Skip startup gyro bias estimation
  --no-nmea          Disable NMEA output stream
  --no-highrate      Disable high-rate binary stream
  --no-json          Disable JSON output stream
  --foreground       Accepted and ignored (always foreground under systemd)
  --version          Print version and exit
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

Options:
  --config PATH   Config file (default: /etc/imud/imud.conf)
  --output PATH   Override cal.json output path from config

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

Live monitor for the three UDP output streams — a receive-side sanity check
run on any machine on the vessel LAN (no daemon socket needed). Reads port
numbers and multicast addresses from the config file, joins multicast groups
where needed, and prints a once-per-second snapshot line per stream.

```text
imud-mon [--config PATH] [nmea] [json] [binary]

  --config PATH  Config file (default: /etc/imud/imud.conf)

  nmea    Monitor NMEA 0183 stream (UDP port 10110)
  json    Monitor NDJSON stream    (UDP port 10112)
  binary  Monitor binary stream    (UDP port 10111)

  With no stream arguments all three streams are shown.
```

Binary packets are validated (magic + CRC32) before display.

### imud-status

Connects to the AF_UNIX socket at `/run/imud/imud.sock` and prints the
daemon's current state. Accepts `--socket PATH` to use an alternate socket.

```text
Chip IDs:       ism330dhcx 0x6B   mmc5983ma 0x30
ISM ODR:        833 Hz  (FIFO watermark: 64 sample-sets)
Mag ODR:        100 Hz  (SET every 5 s)
Fusion:         MEKF converged  cov_trace=4.2e-06 rad2
Calibration:    accel yes  gyro yes  mag yes
Attitude:       pitch=-3.1  roll=9.5  heading=214.7 M
NMEA out:       10 Hz  (port 10110)
Hi-rate out:    disabled
JSON out:       disabled
IMU samples:    1234567  overflows: 0
Uptime:         00:04:32
```

-----

## 13. Error Handling

|Condition                  |Response                                                   |
|---------------------------|-----------------------------------------------------------|
|I2C probe fails at startup |Fatal — exit 1                                             |
|Single I2C read timeout    |Skip sample, increment error counter                       |
|> 10 consecutive I2C errors|Attempt chip soft-reset; re-init registers                 |
|ISM330 FIFO overflow       |Set `FLAG_FIFO_OVERFLOW`; log at 1 Hz rate-limit           |
|MMC5983MA anomaly          |Reject if field magnitude ±50% from expected; use last good|
|GPIO edge missed           |Fallback 10 ms timer poll; FIFO absorbs gap for ISM330     |
|UDP send failure           |Log first occurrence; continue (fire-and-forget)           |
|SIGHUP                     |Hot-reload fusion params and output rates; see §11         |

-----

## 14. Performance Targets

|Metric                                       |Target                             |
|---------------------------------------------|-----------------------------------|
|ISM330 FIFO read jitter                      |< 5 ms p99 (FIFO absorbs OS jitter)|
|Fusion latency (sample → quaternion)         |< 1.5 ms                           |
|End-to-end (I2C sample → UDP emit)           |< 3 ms                             |
|CPU on Pi 4B at 833 Hz MEKF / 500 Hz out     |< 10% one core                     |
|CPU on Pi Zero 2W at 416 Hz / 100 Hz out     |< 30% one core                     |
|Memory footprint                             |< 14 MB RSS                        |
|High-rate UDP loss (localhost)               |0%                                 |
|Heading accuracy post-cal, benign environment|±1–2° (chip spec: ±0.5°)           |

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
(saves ~40 µs per burst vs separate transactions):

```c
static int burst_read(int fd, uint8_t addr, uint8_t reg,
                      uint8_t *buf, size_t len) {
    struct i2c_msg msgs[2] = {
        { .addr = addr, .flags = 0,        .len = 1,   .buf = &reg },
        { .addr = addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
    };
    struct i2c_rdwr_ioctl_data data = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &data);
}
```

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
              src/drivers/sim.c

all: imud imud-cal imud-status imud-mon

imud:        $(IMUD_OBJS) src/main.c
imud-cal:    $(CAL_OBJS)  src/cal_main.c
imud-status: src/status_main.c
imud-mon:    src/config.o src/mon_main.c
```

`make install` also installs the five man pages (imud.8, imud-cal.8,
imud.conf.5, imud-status.1, imud-mon.1) and `data/WMM.COF` → `/etc/imud/`.

Cross-compile for Pi from x86 host:

```bash
apt install gcc-aarch64-linux-gnu
make CC=aarch64-linux-gnu-gcc
```

-----

## 16. Open Items

1. **`LOG_*` migration (partial)** — The log-level infrastructure is in place:
   `include/log.h` defines `LOG_D/I/W/E` macros gated on `g_log_level`, and
   `main.c` sets `g_log_level` from the `[logging] level` config field at startup.
   Most code paths still use bare `fprintf(stderr, ...)`. Completing the migration
   means replacing those callsites with the appropriate `LOG_*` macro.

2. **Pi 5 compatibility** — Pi 5 uses the RP1 I/O chip; `gpiod` is the correct
   abstraction layer but interrupt latency should be re-profiled on Pi 5
   hardware. I2C is on `/dev/i2c-1` with a different underlying driver.

3. **Marine vibration characterization** — Near a diesel or outboard, the
   ISM330DHCX's embedded low-pass filter settings (LPF2) and the MEKF accel
   update skip threshold may need tuning. The ISM330DHCX's onboard Machine
   Learning Core (MLC) can detect engine-on state and assert a GPIO flag,
   allowing the daemon to tighten `accel_skip_thresh` automatically.

4. **`cal_file` default** — `config_defaults()` sets `/etc/imud/cal.json`
   (alongside the main config; written by `imud-cal`, read-only by the daemon).

-----

## 17. Timestamp Architecture — Camera Correlation

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
  t_chip         = read ISM330 TIMESTAMP registers
  t_wall_after   = clock_gettime(CLOCK_REALTIME)
  anchor_wall_ns = (t_wall_before + t_wall_after) / 2
  anchor_chip    = t_chip
  anchor_gen    += 1
```

Per-sample wall timestamp:

```python
def chip_to_wall_ns(chip_ts: int) -> int:
    delta = (chip_ts - anchor_chip) & 0xFFFFFFFF  # 32-bit rollover safe (~29.8 hr)
    return anchor_wall_ns + delta * 25_000         # 25 µs per tick → ns
```

**Total timestamp uncertainty budget:**

|Source                                                  |Contribution                   |
|--------------------------------------------------------|-------------------------------|
|GNSS+PPS clock discipline (each node)                   |< 5 µs typical, < 20 µs worst  |
|Anchor I2C read half-width                              |~150–300 µs (dominant)         |
|ISM330 timer resolution                                 |25 µs                          |
|ISM330 oscillator drift between anchors (60 s × 100 ppm)|< 6 ms → corrected at re-anchor|
|**Total per-sample, steady state**                      |**~300–500 µs**                |

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
