# imud Configuration Reference

imud is configured with a single TOML-like text file.

**Default locations** (first match wins):
1. Path given on the command line: `imud --config /path/to/imud.conf`
2. `/etc/imud/imud.conf` (installed by `make install`)

**Reload behaviour:**
- Keys marked **[restart]** take effect only after restarting the daemon (`systemctl restart imud`).
- Keys marked **[hot]** can be reloaded at runtime by sending `SIGHUP`:
  ```sh
  sudo systemctl reload imud   # or: kill -HUP $(pidof imud)
  ```
  Unknown keys and bad values in a SIGHUP reload are logged as warnings; the previously
  loaded values are kept.

**Syntax:**
- Comments start with `#` and may appear inline.
- Strings must be double-quoted: `driver = "ism330dhcx"`.
- Integers may be decimal (`833`) or hex (`0x6B`).
- Booleans: `true` / `false`.
- Arrays: `[0.0, 0.0, 0.0]` (used only for `rotation_euler_deg`).
- `~/` paths are expanded to `$HOME/`.

---

## `[device]`

Hardware bus and GPIO controller. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `i2c_bus` | string | `"/dev/i2c-1"` | I²C bus device node. Use `/dev/i2c-1` on Pi 4; `/dev/i2c-1` or `/dev/i2c-3` on Pi 5 depending on which header pins are used. |
| `gpio_chip` | string | `"gpiochip0"` | gpiochip device name. `"gpiochip0"` on Pi 4; `"gpiochip4"` on Pi 5 (RP1 GPIO controller). |

---

## `[imu]`

IMU (gyroscope + accelerometer) driver settings. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `driver` | string | `"ism330dhcx"` | Driver to load. See [Supported drivers](#supported-drivers). |
| `i2c_addr` | int | `0x6B` | I²C address. `0x6B` (SA0 high) or `0x6A` (SA0 low via jumper). |
| `int_gpio` | int | `17` | BCM GPIO number for the FIFO watermark interrupt (board pin 11). Set `0` to use a 10 ms polling timer instead of a hardware interrupt. |
| `odr_hz` | int | `833` | Output data rate in Hz. The driver rounds to the nearest supported rate. ISM330DHCX supports: `12`, `26`, `52`, `104`, `208`, `416`, `833`, `1660`. |
| `accel_g` | int | `8` | Accelerometer full-scale range in g. ISM330DHCX: `2`, `4`, `8`, `16`. |
| `gyro_dps` | int | `2000` | Gyroscope full-scale range in degrees/second. ISM330DHCX: `125`, `250`, `500`, `1000`, `2000`, `4000`. |
| `fifo_wm` | int | `64` | FIFO watermark in sample-sets. Controls interrupt latency vs. CPU wake-up frequency. At 833 Hz, `32` ≈ 38 ms; `64` ≈ 77 ms. |

---

## `[mag]`

Magnetometer driver settings. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `driver` | string | `"mmc5983ma"` | Driver to load. See [Supported drivers](#supported-drivers). |
| `i2c_addr` | int | `0x30` | I²C address. MMC5983MA has a fixed address; AK09916 (in ICM-20948) is accessed via the ICM's I²C master — leave at `0x30` and let the IMU driver handle it. |
| `int_gpio` | int | `27` | BCM GPIO number for the measurement-done interrupt (board pin 13). Set `0` to poll on a timer. |
| `odr_hz` | int | `100` | Output data rate in Hz. MMC5983MA supports: `1`, `10`, `20`, `50`, `100`, `200`, `1000`. |
| `set_period_s` | float | `5.0` | Interval in seconds between SET/RESET degauss pulses. Prevents gradual magnetisation of the sensor. Set `0` to disable. |

---

## `[fusion]`

Multiplicative Extended Kalman Filter (MEKF) noise parameters and tuning knobs. **[hot]**

These values are updated live on SIGHUP. Noise parameters come from sensor datasheets — they are
physical constants, not tuning knobs. The rejection thresholds (`mag_reject_gauss`,
`accel_skip_thresh`) are the main knobs to adjust for noisy environments.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `mekf_gyro_noise` | double | `0.007` | Gyro noise density in rad/s/√Hz. ISM330DHCX datasheet: 7 mdps/√Hz ≈ 0.000122 rad/s/√Hz. |
| `mekf_gyro_bias` | double | `0.00015` | In-run gyro bias instability in rad/s. Used to set the bias random-walk process noise. |
| `mekf_accel_noise` | double | `0.0022` | Accelerometer noise density in m/s²/√Hz. ISM330DHCX: ~186 µg/√Hz × 9.81. |
| `mekf_mag_noise` | double | `0.0004` | Magnetometer noise density in Gauss/√Hz. MMC5983MA: 0.4 mGauss RMS. |
| `mag_reject_gauss` | double | `0.0008` | Reject a magnetometer measurement if its post-calibration residual exceeds this value (Gauss). Protects against local magnetic disturbances. Increase if nearby ferrous objects cause frequent rejections. |
| `accel_skip_thresh` | double | `0.05` | Skip an accelerometer update if `||a| − 1g|` exceeds this fraction of g. Prevents linear acceleration from corrupting the tilt estimate. `0.05` = skip if more than 5% off 1g. |
| `engine_vibration_g2` | double | `0.0` | EMA threshold (m²/s⁴) for engine-vibration detection. The filter maintains an exponential moving average of `(|a| − g)²`; when the EMA exceeds this value, the engine is considered on and `engine_accel_skip_thresh` overrides `accel_skip_thresh`. Set `0` to disable (default). |
| `engine_accel_skip_thresh` | double | `0.20` | Accelerometer skip threshold override applied when engine vibration is detected. Should be tighter (smaller) than `accel_skip_thresh` to prevent high-frequency vibration from corrupting tilt. Only active when `engine_vibration_g2 > 0`. |

---

## `[calibration]`

Calibration file and startup behaviour. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `file` | string | `"/etc/imud/cal.json"` | Path to the calibration JSON file produced by `imud-cal`. Supports `~/` expansion. If the file does not exist, imud runs uncalibrated (magnetometer readings will be biased). |
| `startup_settle_sec` | double | `5.0` | Seconds to discard sensor data after chip initialisation. The ISM330DHCX gyro drifts ~0.02 rad/s for ~5 s after power-on; discarding this window avoids polluting the bias estimate. Set `0` for the `sim` driver (no thermal transient). |
| `gyro_bias_sec` | double | `2.0` | Length of the stationary averaging window used to estimate gyro bias at startup. The board must be still for this duration. Skipped if the calibration file already contains a gyro bias entry. Set `0` to skip bias estimation entirely. |

---

## `[nmea]`

NMEA 0183 UDP output stream. **[restart]**: `enabled`, `dest_addr`, `dest_port`. **[hot]**: `rate_hz`.

Sentences emitted per burst (at `rate_hz`):
- `$PASHR` — roll, pitch, heading, heave, accuracy flags
- `$HCHDM` — magnetic heading
- `$HCHDG` — heading + magnetic variation *(variation fields filled when declination is known, empty otherwise)*
- `$TIROT` — rate of turn (deg/min)
- `$IIXDR` — pitch and roll transducer readings
- `$HCHDT` — true heading *(only when declination is configured; see `[position]`)*

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `true` | Enable the NMEA output stream. |
| `rate_hz` | int | `10` | Output rate in Hz. Hot-reloadable. |
| `dest_addr` | string | `"255.255.255.255"` | Destination IP address. `255.255.255.255` = broadcast; use a unicast or multicast address to target a specific host. |
| `dest_port` | int | `10110` | Destination UDP port. Standard NMEA-over-UDP port is 10110. |

---

## `[highrate]`

High-rate binary UDP stream (500 Hz by default). **[restart]**: `enabled`, `dest_addr`, `dest_port`, `coord_frame`. **[hot]**: `rate_hz`.

The binary packet format is documented in [spec.md](../spec.md). Consumer libraries are in `lib/`.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable the high-rate binary stream. Disabled by default — opt in for machine vision, ROS2, or any consumer needing quaternion + covariance at high rate. |
| `rate_hz` | int | `500` | Output rate in Hz. The MEKF runs at the full `imu.odr_hz` internally; this controls how often results are published. Hot-reloadable. |
| `dest_addr` | string | `"239.255.0.1"` | Destination IP. Default is an IPv4 multicast group (TTL=1, link-local). Consumers must join with `IP_ADD_MEMBERSHIP`. Use `255.255.255.255` for broadcast or a unicast IP for point-to-point. |
| `dest_port` | int | `10111` | Destination UDP port. |
| `coord_frame` | string | `"NED"` | Output coordinate frame: `"NED"` (North-East-Down) or `"ENU"` (East-North-Up). Affects the quaternion, gyro, accel, and mag vector fields in the binary packet. |

---

## `[json]`

NDJSON UDP stream — one JSON object per datagram, newline-terminated. **[restart]**: `enabled`, `dest_addr`, `dest_port`. **[hot]**: `rate_hz`.

Fields per object: `ts`, `heading_deg`, `pitch_deg`, `roll_deg`, `rot_dpm`, `quat`, `gyro_bias`,
`cov_trace`, `flags`, and `true_heading_deg` (when declination is configured).

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable the NDJSON stream. |
| `rate_hz` | int | `100` | Output rate in Hz. Hot-reloadable. |
| `dest_addr` | string | `"255.255.255.255"` | Destination IP address. |
| `dest_port` | int | `10112` | Destination UDP port. |

---

## `[stream]`

Local AF_UNIX subscription stream — the same 192-byte binary packets as `[highrate]`, but over a `SOCK_STREAM` socket. Same-host consumers get a loss-free stream and subscribe by connecting (up to 8 at once). Slow consumers get dropped packets (visible as `imu_seq` gaps), never a stalled daemon. **[restart]**: `enabled`, `socket`. **[hot]**: `rate_hz`.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable the subscription stream. |
| `socket` | string | `"/run/imud/imud-stream.sock"` | Listen path (mode 0660). |
| `rate_hz` | int | `100` | Per-subscriber packet rate in Hz. Hot-reloadable. |

---

## `[mount]`

Board-to-body rotation for installations where the chip X axis does not point toward the bow.
**[restart]**

The rotation is expressed as ZYX intrinsic Euler angles: `R = Rz(yaw) × Ry(pitch) × Rx(roll)`.
In practice only the `yaw` component is non-zero — it corrects for the angle between the chip X
axis and the vessel's bow.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `rotation_euler_deg` | array | `[0.0, 0.0, 0.0]` | `[roll, pitch, yaw]` in degrees. Measure the angle between chip X and true bow once with a handbearing compass; set that value as `yaw`. |
| `preset` | string | *(unset)* | Named shortcut: `"identity"`, `"yaw_90"`, `"yaw_180"`, `"yaw_270"`, `"roll_90"`, `"roll_270"`, `"pitch_90"`, `"pitch_270"`. Overrides `rotation_euler_deg` when set. |

**Example** — chip X points to port (90° to the left of the bow):
```toml
rotation_euler_deg = [0.0, 0.0, -90.0]
# or equivalently:
preset = "yaw_270"
```

---

## `[logging]`

Diagnostic log output. **[hot]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `level` | string | `"warn"` | Log verbosity. One of `"debug"`, `"info"`, `"warn"`, `"error"`. `"warn"` is recommended for production (SD card friendly). `"info"` adds a periodic `[stats]` heartbeat. `"debug"` logs every sensor burst — very verbose. |
| `file` | string | `""` | Log destination. Empty string = write to stderr (captured by journald when running as a service). Set an absolute path to redirect to a file instead. |
| `stats_hz` | int | `1` | Rate of the `[stats]` heartbeat log line. Only visible when `level = "info"` or `"debug"`. |

---

## `[position]`

Magnetic declination and true heading. **[restart]**

When declination is known, imud adds `$HCHDT` to the NMEA stream, `true_heading_deg` to the
JSON stream, and sets `FLAG_DECLINATION_VALID` in the binary packet.

### Declination modes (highest priority wins)

1. **WMM auto-compute** — set `lat_deg` and `lon_deg` (both non-zero). imud loads the WMM2025
   coefficient file at startup and on SIGHUP, computes the local declination, and applies it
   immediately. Accurate to ±0.5° for a fixed installation.

2. **Static override** — set `declination_deg` only (leave `lat_deg` and `lon_deg` at `0.0`).
   Look up your value at <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>.
   East declination is positive; west is negative.

3. **Disabled** (default) — all three values at zero → no true heading output.

### Static position and WMM

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `declination_deg` | float | `0.0` | Static magnetic declination in degrees. East positive (+), west negative (−). Ignored when `lat_deg` and `lon_deg` are both non-zero. |
| `lat_deg` | double | `0.0` | Geodetic latitude in decimal degrees (+N / −S). Set with `lon_deg` to enable WMM auto-compute. |
| `lon_deg` | double | `0.0` | Geodetic longitude in decimal degrees (+E / −W). Set with `lat_deg` to enable WMM auto-compute. |
| `wmm_file` | string | `"/etc/imud/WMM.COF"` | Path to the WMM coefficient file. The WMM2025 file is installed to `/etc/imud/WMM.COF` by `make install`. Valid 2025.0–2030.0; replace with WMM2030 around late 2029. |

### Live position sources

Enable one or both to receive GPS-driven WMM updates as the vessel moves. WMM is recomputed
when position changes by ≥ 0.05° (≈ 5 km). The stale-fix TTL (`fix_max_age_h`) ensures true
heading output stops if GPS is lost for an extended period.

Priority: **gpsd** (live stream) > **SignalK** (polled fallback) > **static lat/lon** > **static declination**.

#### gpsd

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `gpsd_enabled` | bool | `false` | Connect to gpsd for live position updates. gpsd must be running and have a working GPS source. |
| `gpsd_host` | string | `"localhost"` | Hostname or IP address of the gpsd instance. |
| `gpsd_port` | int | `2947` | TCP port of the gpsd instance. |

imud subscribes to gpsd's JSON stream (`?WATCH={"enable":true,"json":true}`) and processes
`TPV` messages with `mode ≥ 2` (2D or 3D fix). The connection is persistent and reconnects
automatically after a dropout.

#### SignalK

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `signalk_enabled` | bool | `false` | Poll the SignalK REST API for position. |
| `signalk_host` | string | `"localhost"` | Hostname or IP address of the SignalK server. |
| `signalk_port` | int | `3000` | HTTP port of the SignalK server. |
| `signalk_path` | string | `"/signalk/v1/api/vessels/self/navigation/position"` | REST endpoint path. Override if your SignalK server uses a non-standard path or vessel ID. |

SignalK is polled every 30 seconds. When gpsd is also enabled, SignalK is used only as a
fallback (polled once when gpsd drops), still respecting the 30-second minimum interval.

#### Stale-fix TTL

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `fix_max_age_h` | float | `24.0` | How many hours to keep a GPS-derived declination after the last fix. The TTL resets on every GPS fix received — an anchored vessel with continuous GPS keeps its declination valid indefinitely. After the window expires without a fix, `FLAG_DECLINATION_VALID` clears and true-heading output stops. Set `0` to never expire (declination persists until the daemon restarts). |

---

## Supported drivers

| Driver name | Chip | Type | I²C address | Notes |
| --- | --- | --- | --- | --- |
| `"ism330dhcx"` | ST ISM330DHCX | IMU | 0x6A–0x6B | Primary validated IMU. FIFO + interrupt driven. |
| `"icm20948"` | TDK ICM-20948 | IMU | 0x68–0x69 | (experimental) Includes built-in AK09916 mag via I²C master. |
| `"icm42688p"` | TDK ICM-42688-P | IMU | 0x68–0x69 | (experimental) Best-in-class noise floor. FIFO + hw timestamp. |
| `"lsm6dso"` | ST LSM6DSO | IMU | 0x6A–0x6B | (experimental) Near-clone of ISM330DHCX. ODR up to 6664 Hz. |
| `"lsm6dsox"` | ST LSM6DSOX | IMU | 0x6A–0x6B | (experimental) LSM6DSO with machine-learning core; same driver. |
| `"mmc5983ma"` | MEMSIC MMC5983MA | Magnetometer | 0x30 | Primary validated mag. 18-bit, SET/RESET coil. |
| `"ak09916"` | AK09916 | Magnetometer | 0x0C | (experimental) Used via ICM-20948 I²C bypass mode. |
| `"lis3mdl"` | ST LIS3MDL | Magnetometer | 0x1C–0x1E | (experimental) Popular standalone mag. INT pin. ±4 G fixed. |
| `"lis2mdl"` | ST LIS2MDL | Magnetometer | 0x1E | (experimental) LIS3MDL successor. Fixed ±50 G, 0.15 µT/LSB. |
| `"sim"` | — | IMU + Mag | — | Software simulation. Simulates a small boat under way. No hardware required. Set `int_gpio = 0` on both sensors. |

Drivers marked **(experimental)** have their register maps verified against the datasheet but have not yet been validated on real hardware. A warning is printed at startup when an experimental driver is selected.

---

## Minimal example — sim mode (no hardware)

```toml
[device]
i2c_bus   = "/dev/null"
gpio_chip = "gpiochip0"

[imu]
driver   = "sim"
i2c_addr = 0x00
int_gpio = 0

[mag]
driver   = "sim"
i2c_addr = 0x00
int_gpio = 0

[calibration]
file               = "config/sim-cal.json"
startup_settle_sec = 0.0
gyro_bias_sec      = 0.0

[nmea]
enabled = true

[position]
lat_deg  = 37.8697   # Berkeley Marina — enables WMM auto-compute
lon_deg  = -122.3153
wmm_file = "data/WMM.COF"
```

Run with:
```sh
imud --config config/sim.conf
```

---

## Minimal example — hardware (ISM330DHCX + MMC5983MA, Pi 5)

```toml
[device]
i2c_bus   = "/dev/i2c-1"
gpio_chip = "gpiochip4"      # Pi 5

[imu]
driver   = "ism330dhcx"
i2c_addr = 0x6B
int_gpio = 17

[mag]
driver   = "mmc5983ma"
i2c_addr = 0x30
int_gpio = 27

[calibration]
file = "/etc/imud/cal.json"

[nmea]
enabled = true

[position]
lat_deg = 37.8697
lon_deg = -122.3153
```
