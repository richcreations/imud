# imud Manual

Complete operator and developer reference for **imud**, the IMU daemon for
Raspberry Pi. For a one-page introduction and quick start, see the
[README](../README.md). For the on-the-wire protocol (binary packet layout,
NMEA sentence formats, timestamp architecture), see [spec.md](../spec.md).

**Contents**

1. [Overview](#1-overview)
2. [Building and installing](#2-building-and-installing)
3. [Running imud](#3-running-imud)
4. [Configuration reference](#4-configuration-reference)
5. [Supported drivers](#5-supported-drivers)
6. [Calibration](#6-calibration)
7. [Output streams](#7-output-streams)
8. [Monitoring and diagnostics](#8-monitoring-and-diagnostics)
9. [Consumer libraries](#9-consumer-libraries)
10. [Troubleshooting](#10-troubleshooting)
11. [Writing a driver](#11-writing-a-driver)

---

## 1. Overview

imud is a general-purpose IMU daemon — *gpsd for IMUs*. It owns the inertial
sensor and does the real-time work once: it reads a gyroscope/accelerometer
(IMU) and a magnetometer over I²C, fuses them with a Multiplicative Extended
Kalman Filter (MEKF) into a real-time attitude estimate (quaternion, Euler
angles, magnetic and true heading, rate of turn, heave), and publishes the
result on several standard interfaces that any number of programs can consume
at once. Consumers get a fused, timestamped estimate rather than raw samples
to process themselves.

The same daemon serves marine navigation (NMEA 0183, true heading, heave),
robotics (binary attitude), machine vision and camera stabilization
(high-rate quaternion with hardware timestamps), and gimbal/pointing systems.
Output is use-agnostic; a few fusion options have marine defaults (noted where
they apply).

**Threads.** A reader thread drains the IMU FIFO on a GPIO watermark
interrupt; a second reads the magnetometer; a fusion thread runs the MEKF at
the full IMU rate; a health thread serves `imud-status` and pets the systemd
watchdog; optional output threads emit NMEA, binary, and a local
subscription stream; an optional position thread pulls GPS fixes from gpsd or
SignalK to keep magnetic declination current. See [spec.md §3](../spec.md)
for the full architecture.

**Dependencies.** C11, POSIX, `libgpiod` (1.x or 2.x, auto-detected), and the
C standard library. Nothing else.

**Binaries.**

| Binary | Purpose |
|---|---|
| `imud` | The daemon. |
| `imud-cal` | Calibration tool (gyro bias, accel 6-position, magnetometer swing). |
| `imud-status` | Query a running daemon's health over its Unix socket. |
| `imud-mon` | Live monitor of the UDP output streams from any host on the LAN. |
| `imud-signalk` | Bridge daemon (optional package): emits Signal K delta JSON over UDP (see [§9a Bridges](#9a-bridges)). |
| `imud-mqtt` | Bridge daemon (optional package): MQTT topics + Home Assistant discovery (see [§9a Bridges](#9a-bridges)). |
| `imud-influxdb` | Bridge daemon (optional package): InfluxDB line protocol over UDP/HTTP (see [§9a Bridges](#9a-bridges)). |
| `imud-mavlink` | Bridge daemon (optional package): MAVLink v1/v2 over UDP/serial (see [§9a Bridges](#9a-bridges)). |
| `imud-prometheus` | Bridge daemon (optional package): Prometheus `/metrics` exporter (see [§9a Bridges](#9a-bridges)). |

---

## 2. Building and installing

### Prerequisites

- Raspberry Pi running PiOS Bookworm (or any Debian Bookworm-based distro).
- I²C enabled: `sudo raspi-config` → Interface Options → I²C.
- `libgpiod-dev`. Bookworm ships 1.6.x; both 1.x and 2.x are supported and
  the Makefile auto-detects the installed version via `pkg-config`.

```sh
sudo apt update && sudo apt install -y build-essential libgpiod-dev
```

### Build

```sh
make            # builds imud, imud-cal, imud-status, imud-mon
make test       # builds and runs the host-side unit tests (no hardware needed)
```

`make test` must be run from the repository root — one test loads
`data/WMM.COF` by relative path.

### Install

```sh
sudo make install
```

This installs:

| Item | Destination |
|---|---|
| Binaries | `/usr/local/bin/` |
| Reference config | `/etc/imud/imud.conf` (skipped if it already exists) |
| Calibration file | `/etc/imud/cal.json` (if `config/cal.json` is present) |
| systemd unit | `/etc/systemd/system/imud.service` |
| Client libraries | `libimud.so` + `/usr/local/include/imud.h` (see `man 3 libimud`), `/usr/local/share/imud/imud_client.py` |
| Man pages | `imud.8`, `imud-cal.8`, `imud.conf.5`, `imud-status.1`, `libimud.3` |

`imud-mon` (the network stream monitor) is its own install target — it can
live on machines that don't run the daemon:

```sh
sudo make install-utils
```

WMM coefficient data is installed separately — it is versioned by model epoch
(WMM2025), not by imud release, so it ships as its own package
(`imud-wmm-data`) that can be updated independently:

```sh
sudo make install-wmm-data
```

This installs `WMM.COF` → `/usr/share/imud/WMM.COF`. To use a newer model
before the package updates, drop it at `/etc/imud/WMM.COF` — imud prefers
that path when it exists.

`make install` also creates a dedicated system user `imud` (in the `gpio`
and `i2c` groups) that the service runs as. `sudo make uninstall` reverses
the install but preserves `/etc/imud/` (your config and calibration).

Override install locations with the usual variables, e.g.
`sudo make install PREFIX=/opt/imud ETCDIR=/opt/imud/etc`.

---

## 3. Running imud

### In the foreground (testing)

```sh
imud --config config/imud.conf
```

To try the full pipeline with no hardware, use the bundled simulation
config, which selects the `sim` driver for both sensors:

```sh
imud --config config/sim.conf
```

### As a systemd service

```sh
sudo systemctl enable --now imud     # start now and on boot
sudo systemctl status imud
journalctl -u imud -f                # follow the log
```

The unit is `Type=notify` and hardened (`ProtectSystem=strict`,
`NoNewPrivileges`, `DevicePolicy=closed` with explicit `DeviceAllow` for
`/dev/i2c-1` and the gpiochip, `MemoryMax=32M`, a 10 s systemd watchdog). On
Raspberry Pi 5, uncomment the `gpiochip4` `DeviceAllow` line in
`etc/imud.service` and set `device.gpio_chip = "gpiochip4"` in the config.

### Command-line options

```
imud [OPTIONS]
  --config PATH      Config file (default: /etc/imud/imud.conf)
  --replay FILE      Replay an .imucap capture through the full pipeline
                     (forces both sensors onto the sim driver; docs/capture.md)
  --skip-bias-cal    Skip the startup gyro-bias estimation window
  --no-nmea          Disable the NMEA output stream
  --no-highrate      Disable the high-rate binary stream
  --foreground       Accepted and ignored (imud always runs in the foreground)
  --version          Print the version and exit
```

If `--config` is omitted, imud tries `/etc/imud/imud.conf` and then
`~/.config/imud/imud.conf`. A config file that exists but contains a bad
value is fatal — imud reports every error and refuses to start rather than
run on a half-applied config. A missing file is not an error (built-in
defaults are used).

### Signals

| Signal | Effect |
|---|---|
| `SIGHUP` | Reload the config. `[hot]` fields take effect immediately; `[restart]` fields are ignored. The log file, if configured, is reopened (for logrotate). If the file fails to parse, the running config is kept. |
| `SIGTERM`, `SIGINT` | Clean shutdown: emit a final binary packet with `FLAG_SHUTDOWN`, stop and join all threads, remove the PID file and sockets. |

Reload with `sudo systemctl reload imud` or `kill -HUP $(pidof imud)`.

---

## 4. Configuration reference

imud is configured with a single TOML-like text file.

**Default locations** (first match wins):
1. Path given on the command line: `imud --config /path/to/imud.conf`
2. `/etc/imud/imud.conf` (installed by `make install`)
3. `~/.config/imud/imud.conf`

**Reload behaviour:**
- Keys marked **[restart]** take effect only after restarting the daemon
  (`systemctl restart imud`).
- Keys marked **[hot]** can be reloaded at runtime by sending `SIGHUP`:
  ```sh
  sudo systemctl reload imud   # or: kill -HUP $(pidof imud)
  ```
  Unknown keys and bad values in a SIGHUP reload are logged as warnings; the
  previously loaded values are kept.

**Syntax:**
- Comments start with `#` and may appear inline.
- Strings must be double-quoted: `driver = "ism330dhcx"`.
- Integers may be decimal (`833`) or hex (`0x6B`).
- Booleans: `true` / `false`.
- Arrays: `[0.0, 0.0, 0.0]` (used only for `rotation_euler_deg`).
- `~/` paths are expanded to `$HOME/`.

The annotated reference file at `config/imud.conf` documents every key inline
and is the fastest way to see a working configuration.

### `[device]`

Hardware bus and GPIO controller. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `i2c_bus` | string | `"/dev/i2c-1"` | I²C bus device node. Use `/dev/i2c-1` on Pi 4; `/dev/i2c-1` or `/dev/i2c-3` on Pi 5 depending on which header pins are used. |
| `gpio_chip` | string | `"gpiochip0"` | gpiochip device name. `"gpiochip0"` on Pi 4; `"gpiochip4"` on Pi 5 (RP1 GPIO controller). |
| `sim_file` | string | `""` | An `.imucap` capture for the sim driver to replay (`driver = "sim"` in both `[imu]` and `[mag]`); empty selects the built-in synthetic scenario. `imud --replay FILE` is the shortcut. See [capture & replay](capture.md). |
| `sim_loop` | bool | `false` | Repeat the capture forever; timestamps and sequence numbers are rebased to stay monotonic. |
| `sim_speed` | float | `1.0` | Playback pacing: `2.0` = double speed, `0` = as fast as the pipeline accepts. |

### `[imu]`

IMU (gyroscope + accelerometer) driver settings. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `driver` | string | `"ism330dhcx"` | Driver to load. See [Supported drivers](#5-supported-drivers). |
| `i2c_addr` | int | `0x6B` | I²C address. `0x6B` (SA0 high) or `0x6A` (SA0 low via jumper). |
| `int_gpio` | int | `17` | BCM GPIO number for the FIFO watermark interrupt (board pin 11). Set `0` to use a 10 ms polling timer instead of a hardware interrupt. |
| `odr_hz` | int | `833` | Output data rate in Hz. The driver rounds to the nearest supported rate. ISM330DHCX supports: `12`, `26`, `52`, `104`, `208`, `416`, `833`, `1660`. |
| `accel_g` | int | `8` | Accelerometer full-scale range in g. ISM330DHCX: `2`, `4`, `8`, `16`. |
| `gyro_dps` | int | `2000` | Gyroscope full-scale range in degrees/second. ISM330DHCX: `125`, `250`, `500`, `1000`, `2000`, `4000`. |
| `fifo_wm` | int | `64` | FIFO watermark in sample-sets. Controls interrupt latency vs. CPU wake-up frequency. At 833 Hz, `32` ≈ 38 ms; `64` ≈ 77 ms. |

### `[mag]`

Magnetometer driver settings. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `driver` | string | `"mmc5983ma"` | Driver to load. See [Supported drivers](#5-supported-drivers). |
| `i2c_addr` | int | `0x30` | I²C address. MMC5983MA has a fixed address; AK09916 (in ICM-20948) is accessed via the ICM's I²C master — leave at `0x30` and let the IMU driver handle it. |
| `int_gpio` | int | `27` | BCM GPIO number for the measurement-done interrupt (board pin 13). Set `0` to poll on a timer. |
| `odr_hz` | int | `100` | Output data rate in Hz. MMC5983MA supports: `1`, `10`, `20`, `50`, `100`, `200`, `1000`. |
| `set_period_s` | float | `5.0` | Interval in seconds between SET/RESET degauss pulses. Prevents gradual magnetisation of the sensor. Set `0` to disable. |

### `[fusion]`

MEKF noise parameters and tuning knobs. **[hot]**

Applied live on SIGHUP. The `mekf_*` noise densities are **tuned constants**, not
raw sensor readouts: `mekf_gyro_noise` and `mekf_gyro_bias` build the filter's
process noise Q, held deliberately above the measured sensor floor so the filter
stays responsive and the gyro bias observable. The per-unit noise that
`imud-cal characterize` measures (cal.json `noise` section, see
[capture.md](capture.md)) is informational only and never supersedes them — driving
Q from the raw floor makes the filter too stiff. The knobs most worth touching
for a noisy install are `mag_reject_gauss`, `accel_skip_thresh`,
`mag_yaw_only`, `heave_tau_s`, and `wave_tau_s`.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `mekf_gyro_noise` | double | `0.007` | Gyro noise density in rad/s/√Hz. ISM330DHCX datasheet: 7 mdps/√Hz ≈ 0.000122 rad/s/√Hz. |
| `mekf_gyro_bias` | double | `0.00015` | In-run gyro bias instability in rad/s. Sets the bias random-walk process noise. |
| `mekf_accel_noise` | double | `0.0022` | Accelerometer noise density in m/s²/√Hz. ISM330DHCX: ~186 µg/√Hz × 9.81. |
| `mekf_mag_noise` | double | `0.0004` | Magnetometer noise density in Gauss/√Hz. MMC5983MA: 0.4 mGauss RMS. |
| `mag_reject_gauss` | double | `0.05` | Strong-anomaly cutoff: reject a magnetometer measurement whose post-calibration residual exceeds this value (Gauss). Guards against nearby iron/magnets. Fine-grained consistency is handled internally by χ² innovation gates that self-scale with filter confidence, so keep this a coarse threshold (~10% of the Earth field). |
| `accel_skip_thresh` | double | `0.05` | Skip an accelerometer update if `||a| − 1g|` exceeds this fraction of g. Prevents linear acceleration from corrupting the tilt estimate. `0.05` = skip if more than 5% off 1g. |
| `mag_yaw_only` | bool | `true` | Heading-only magnetometer fusion (marine default): the mag corrects heading and never pulls on roll/pitch. The swing-circle calibration is structurally 2D, so the field's vertical (dip) channel is its least-calibrated component. Set `false` for full 3D vector fusion — appropriate only for magnetically clean installs with a true 3D calibration. |
| `heave_tau_s` | float | `12.0` | Heave filter time constant in seconds. Heave (vertical displacement) is a band-passed double integration of vertical acceleration; it feeds the `$PASHR` heave field and the binary packet's `heave_m`. The passband covers ~2–15 s wave periods at the default; allow ~2 minutes of settling after startup. `0` disables (heave reads 0.0). |
| `wave_tau_s` | float | `120.0` | Sea-state averaging window in seconds. Significant wave height (Hs = 4·σ(heave)), mean zero-crossing wave period, and the vessel's roll/pitch periods and significant single amplitudes (2σ) are exponentially weighted statistics of the heave/roll/pitch oscillations over this window (packet fields `wave_height_m`, `wave_period_s`, `roll_period_s`, `roll_amplitude`, `pitch_period_s`, `pitch_amplitude`, gated by the `wave_valid` flag). Stats settle ~2 windows after heave settles. Requires `heave_tau_s` > 0; `0` disables. |
| `engine_vibration_g2` | double | `0.0` | EMA threshold (m²/s⁴) for engine-vibration detection. The filter tracks an exponential moving average of `(|a| − g)²`; when it exceeds this value the engine is considered on. Set `0` to disable (default). |
| `engine_accel_skip_thresh` | double | `0.20` | Accelerometer skip threshold applied while engine vibration is detected: wider than `accel_skip_thresh` so the filter isn't starved, while the accel noise is inflated ×4 so the vibration-contaminated samples that pass are trusted proportionally less. Only active when `engine_vibration_g2 > 0`. |

### `[calibration]`

Calibration file and startup behaviour. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `file` | string | `"/etc/imud/cal.json"` | Path to the calibration JSON produced by `imud-cal`. Supports `~/` expansion. If the file does not exist, imud runs uncalibrated (magnetometer readings will be biased). |
| `startup_settle_sec` | double | `5.0` | Seconds of sensor data to discard after chip initialisation. The ISM330DHCX gyro drifts ~0.02 rad/s for ~5 s after power-on; discarding this window keeps it out of the bias estimate. Set `0` for the `sim` driver. |
| `gyro_bias_sec` | double | `2.0` | Length of the stationary window used to estimate gyro bias at startup. The board must be still for this duration. Skipped if the cal file already has a gyro bias. If motion is detected during the window (gyro std > 0.5 °/s) the window is doubled once and a warning is logged. Set `0` to skip bias estimation. |

### `[nmea]`

NMEA 0183 output — UDP broadcast and/or a TCP listener, independently
enabled (both off by default; since 1.6 a stock daemon emits only on the
local `[stream]` socket). **[restart]**: `enabled`, `dest_addr`,
`dest_port`, `tcp_enabled`, `tcp_bind_addr`, `tcp_port`. **[hot]**:
`rate_hz`.

Sentences emitted per burst — see [§7 Output streams](#7-output-streams) for
details.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable the NMEA UDP output stream. |
| `rate_hz` | int | `10` | Output rate in Hz (shared by UDP and TCP). Hot-reloadable. |
| `dest_addr` | string | `"255.255.255.255"` | Destination IP address. `255.255.255.255` = broadcast; use a unicast or multicast address to target a specific host. |
| `dest_port` | int | `10110` | Destination UDP port. Standard NMEA-over-UDP port is 10110. |
| `tcp_enabled` | bool | `false` | NMEA-over-TCP listener. Chartplotter apps (OpenCPN, Navionics, phone/tablet nav apps) connect as TCP clients and each receives every sentence burst; up to 8 clients, slow clients skip bursts rather than stalling the daemon. |
| `tcp_bind_addr` | string | `"0.0.0.0"` | Listener bind address (numeric IPv4). `127.0.0.1` keeps it host-local. |
| `tcp_port` | int | `10110` | Listener TCP port (the de-facto NMEA-over-TCP port). |

### `[highrate]`

High-rate binary UDP stream (500 Hz by default). **[restart]**: `enabled`, `dest_addr`, `dest_port`, `coord_frame`. **[hot]**: `rate_hz`.

The 260-byte binary packet format is documented in [spec.md §8](../spec.md).
Consumer libraries are in `lib/`.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable the high-rate binary stream. Opt in for machine vision, ROS2, or any consumer needing quaternion + covariance at high rate. |
| `rate_hz` | int | `500` | Publish rate in Hz. The MEKF runs at the full `imu.odr_hz` internally; this controls how often results are published. Hot-reloadable. |
| `dest_addr` | string | `"239.255.0.1"` | Destination IP. Default is an IPv4 multicast group (TTL=1, link-local). Consumers join with `IP_ADD_MEMBERSHIP`. Use `255.255.255.255` for broadcast or a unicast IP for point-to-point. |
| `dest_port` | int | `10111` | Destination UDP port. |
| `coord_frame` | string | `"NED"` | Output coordinate frame: `"NED"` (North-East-Down) or `"ENU"` (East-North-Up). Affects the quaternion, gyro, accel, and mag vector fields in the binary packet. |

### `[stream]`

Local AF_UNIX subscription stream — the same 260-byte binary packets as
`[highrate]`, but over a `SOCK_STREAM` socket. Same-host consumers get a
loss-free stream and subscribe by connecting (up to 8 at once). Slow
consumers get dropped packets (visible as `imu_seq` gaps), never a stalled
daemon. An optional TCP listener serves the same framed packets to remote
consumers — a laptop's `imud-mon`/Python client or a bridge on another
machine (`imud_connect_tcp` / `ImudClient.connect_tcp`). **[restart]**:
`enabled`, `socket`, `tcp_enabled`, `tcp_bind_addr`, `tcp_port`. **[hot]**:
`rate_hz`.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `true` | Enable the subscription stream — the one output a stock daemon provides (the bridges and libimud consumers read it). |
| `socket` | string | `"/run/imud/imud-stream.sock"` | Listen path (mode 0660). |
| `rate_hz` | int | `100` | Per-subscriber packet rate in Hz (AF_UNIX and TCP). Hot-reloadable. |
| `tcp_enabled` | bool | `false` | TCP listener carrying the same framed packets over the network — lossless like the AF_UNIX socket. |
| `tcp_bind_addr` | string | `"0.0.0.0"` | Listener bind address (numeric IPv4). `127.0.0.1` keeps it host-local. |
| `tcp_port` | int | `10112` | Listener TCP port. |

### `[capture]`

The black box: records every raw sensor sample (pre-mount, pre-calibration —
exactly as the driver delivered it) to rotating `.imucap` files. Replay with
`imud --replay`; analyze with `imud-cal characterize` / `fit-temp`. Roughly
25 MB/hour at 104 Hz. See [capture & replay](capture.md). **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Record raw samples from both sensors. |
| `dir` | string | `"/var/lib/imud"` | Destination; files are `imud-YYYYMMDD-HHMMSS.imucap` (UTC). |
| `max_mb` | int | `256` | Rotate the file at this size. `0` = unlimited. |
| `max_files` | int | `8` | Keep the newest N files, deleting the oldest. `0` = keep all. |
| `flush_s` | int | `5` | Flush-to-storage interval, seconds. |

Reader threads never block on storage — a stalled SD card drops capture
records (counted; `imud-status` shows a `Capture:` line), never sensor
samples. A power cut leaves a valid file with a truncated tail.

### Bridge sections (their own files)

The optional bridge daemons — `imud-signalk`, `imud-mqtt`, `imud-influxdb`,
`imud-mavlink`, `imud-prometheus` — each read their **own** config file
(`/etc/imud/imud-<name>.conf`, an `[imud-<name>]` section), never `imud.conf`.
Their config keys are documented in each bridge's own manual and
`imud-<name>.conf(5)`; see [§9a Bridges](#9a-bridges). (Unrelated to the
`signalk_*` keys under `[position]`, which are the input side — imud reading
position *from* Signal K.)

### `[mount]`

Board-to-body rotation for installations where the chip X axis does not point
along the platform's forward axis (the bow, on a vessel). **[restart]**

The rotation is ZYX intrinsic Euler angles: `R = Rz(yaw) × Ry(pitch) ×
Rx(roll)`. In practice only `yaw` is non-zero — it corrects for the angle
between the chip X axis and the platform's forward direction.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `rotation_euler_deg` | array | `[0.0, 0.0, 0.0]` | `[roll, pitch, yaw]` in degrees. Measure the angle between chip X and the forward axis once (e.g. with a handbearing compass on a vessel); set that as `yaw`. |
| `preset` | string | *(unset)* | Named shortcut: `"identity"`, `"yaw_90"`, `"yaw_180"`, `"yaw_270"`, `"roll_90"`, `"roll_270"`, `"pitch_90"`, `"pitch_270"`. Overrides `rotation_euler_deg` when set. |

**Example** — chip X points aft (180° from forward; e.g. to a vessel's stern):
```toml
rotation_euler_deg = [0.0, 0.0, 180.0]
# or equivalently:
preset = "yaw_180"
```

### `[logging]`

Diagnostic log output. **[hot]**: `level`, `stats_hz`. **[restart]**: `file`
(path changes; the file itself is reopened on every reload).

Repeated identical messages are suppressed and logged as a `last message
repeated N times` count. The most recent warnings/errors are also shown by
`imud-status` ("Recent warnings").

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `level` | string | `"warn"` | Log verbosity: `"debug"`, `"info"`, `"warn"`, or `"error"`. `"warn"` is recommended for production (SD-card friendly). `"info"` adds lifecycle messages and a periodic `[stats]` heartbeat. Applied live on SIGHUP. |
| `file` | string | `""` | Log destination. Empty = stderr; under systemd, lines carry sd-daemon priority prefixes so journald records real priorities (`journalctl -p warning` filters work). Set an absolute path for a timestamped log file; it is reopened on SIGHUP so logrotate can rotate it (`postrotate: systemctl reload imud`). |
| `stats_hz` | int | `1` | Rate of the `[stats]` heartbeat log line. Only visible at `level = "info"` or `"debug"`. |

### `[position]`

Magnetic declination and true heading. **[hot]:** `declination_deg`,
`lat_deg`, `lon_deg`, `wmm_file` — declination and the WMM field reference
are recomputed on SIGHUP (a live position source owns declination while
gpsd/Signal K is enabled). **[restart]:** the `gpsd_*` / `signalk_*` source
keys and `fix_max_age_h`.

When declination is known, imud adds `$HCHDT` to the NMEA stream and sets
`FLAG_DECLINATION_VALID` (with the `declination_deg` field) in the binary
packet. A known position also lets the filter set its magnetic
reference (field strength and dip) analytically from the World Magnetic
Model, which improves heading accuracy, and — with live GPS — enables a
speed-aided centripetal correction that keeps the horizon level through
sustained turns.

**Declination modes (highest priority wins):**

1. **WMM auto-compute** — set `lat_deg` and `lon_deg` (both non-zero). imud
   loads the WMM2025 coefficient file at startup and on SIGHUP, computes the
   local declination, and applies it immediately. Accurate to ±0.5° for a
   fixed installation.
2. **Static override** — set `declination_deg` only (leave `lat_deg` and
   `lon_deg` at `0.0`). Look up your value at
   <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>. East
   declination is positive; west is negative.
3. **Disabled** (default) — all three at zero → no true-heading output.

**Static position and WMM**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `declination_deg` | float | `0.0` | Static declination in degrees. East positive (+), west negative (−). Ignored when `lat_deg` and `lon_deg` are both non-zero. |
| `lat_deg` | double | `0.0` | Geodetic latitude in decimal degrees (+N / −S). Set with `lon_deg` to enable WMM auto-compute. |
| `lon_deg` | double | `0.0` | Geodetic longitude in decimal degrees (+E / −W). Set with `lat_deg` to enable WMM auto-compute. |
| `wmm_file` | string | `""` (auto) | Path to the WMM coefficient file. Empty = auto-resolve: `/etc/imud/WMM.COF` if present (operator override), else `/usr/share/imud/WMM.COF` (`imud-wmm-data` package / `make install-wmm-data`). Bundled model WMM2025, valid 2025.0–2030.0. |

**Live position sources**

Enable one or both to receive GPS-driven WMM updates as the vessel moves. WMM
is recomputed when position changes by ≥ 0.05° (≈ 5 km). Priority: **gpsd**
(live stream) > **SignalK** (polled fallback) > **static lat/lon** > **static
declination**.

_gpsd:_

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `gpsd_enabled` | bool | `false` | Connect to gpsd for live position (and speed). gpsd must be running with a working GPS source. |
| `gpsd_host` | string | `"localhost"` | Hostname or IP of the gpsd instance. |
| `gpsd_port` | int | `2947` | TCP port of the gpsd instance. |

imud subscribes to gpsd's JSON stream and processes `TPV` messages with
`mode ≥ 2`. The connection is persistent and reconnects automatically.

_SignalK:_

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `signalk_enabled` | bool | `false` | Poll the SignalK REST API for position. |
| `signalk_host` | string | `"localhost"` | Hostname or IP of the SignalK server. |
| `signalk_port` | int | `3000` | HTTP port of the SignalK server. |
| `signalk_path` | string | `"/signalk/v1/api/vessels/self/navigation/position"` | REST endpoint path. Override for a non-standard path or vessel ID. |

SignalK is polled every 30 seconds. When gpsd is also enabled, SignalK is a
fallback (polled once when gpsd drops), still respecting the 30 s minimum.

_Stale-fix TTL:_

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `fix_max_age_h` | float | `24.0` | Hours to keep a GPS-derived declination after the last fix. The TTL resets on every fix — an anchored vessel with continuous GPS keeps declination valid indefinitely. After the window expires without a fix, `FLAG_DECLINATION_VALID` clears and true-heading output stops. Set `0` to never expire. |

---

## 5. Supported drivers

The names below are the values for `imu.driver` and `mag.driver` in the
config. Drivers live in `src/drivers/` and are registered in `src/drivers.c`.
Links to the manufacturers' datasheets are collected in
[datasheets.md](datasheets.md).

| Driver name | Chip | Type | I²C address | GPIO interrupt | Notes |
| --- | --- | --- | --- | --- | --- |
| `ism330dhcx` | ST ISM330DHCX | IMU | 0x6A–0x6B | BCM 17 · pin 11 | Primary reference IMU. FIFO + hardware timestamp. |
| `icm20948` | TDK ICM-20948 | IMU | 0x68–0x69 | BCM 17 · pin 11 | *Experimental.* Includes a built-in AK09916 mag via I²C master. No hardware timestamp. |
| `icm42688p` | TDK ICM-42688-P | IMU | 0x68–0x69 | BCM 17 · pin 11 | *Experimental.* Best-in-class noise floor. FIFO + hardware timestamp. |
| `lsm6dso` | ST LSM6DSO | IMU | 0x6A–0x6B | BCM 17 · pin 11 | *Experimental.* Near-clone of ISM330DHCX. ODR up to 6664 Hz. |
| `lsm6dsox` | ST LSM6DSOX | IMU | 0x6A–0x6B | BCM 17 · pin 11 | *Experimental.* LSM6DSO with ML core; same driver. |
| `mmc5983ma` | MEMSIC MMC5983MA | Magnetometer | 0x30 | BCM 27 · pin 13 | Primary reference mag. 18-bit, SET/RESET coil. |
| `ak09916` | AKM AK09916 | Magnetometer | 0x0C | none (polling) | *Experimental.* Used via the ICM-20948 I²C bypass; no external INT pin. |
| `lis3mdl` | ST LIS3MDL | Magnetometer | 0x1C–0x1E | BCM 27 · pin 13 | *Experimental.* Popular standalone mag. ±4 G fixed. |
| `lis2mdl` | ST LIS2MDL | Magnetometer | 0x1E | BCM 27 · pin 13 | *Experimental.* LIS3MDL successor. Fixed ±50 G. |
| `sim` | — | IMU + Magnetometer | — | none | Software simulation of a small boat under way. No hardware. Set `int_gpio = 0` on both. |

GPIO pins shown are the defaults (`imu.int_gpio = 17`, `mag.int_gpio = 27`).
Set `int_gpio = 0` to disable the interrupt and use a polling timer — useful
when the pin is wired differently or unavailable.

**Experimental** drivers have their register maps verified against the
datasheet but have **not** been validated on physical hardware. imud prints a
warning at startup when an experimental driver is selected. To add a new
driver, see [§11 Writing a driver](#11-writing-a-driver).

---

## 6. Calibration

Calibration is strongly recommended before first use. Stop the daemon first
(`imud-cal` talks to the hardware directly):

```sh
sudo systemctl stop imud
```

`imud-cal [--config PATH] [--output PATH] <mode>` writes to the path set by
`calibration.file` (default `/etc/imud/cal.json`), or to `--output PATH`. A
partial run updates only the section it calibrated, preserving the others.

| Mode | Procedure |
|---|---|
| `gyro` | Hold the board completely still. Captures gyro bias over a short window. |
| `accel` | Bench 6-position calibration; follow the on-screen prompts to orient each face in turn. Do this before final mounting. |
| `mag` | In-situ magnetometer swing — drive the vessel slowly through at least two full 360° circles, then press Ctrl-C. Guided: a live bar shows heading-circle coverage (`#` covered, `.` needed, `o` = where you're pointing), running coverage %, and live radius/RMS; a bell + "FULL CIRCLE" message tells you when coverage is complete. Must be done after final mounting, with the engine and typical electronics running. |
| `characterize` | Offline Allan-variance noise analysis of a **stationary** capture (`--from FILE`, recorded with `[capture]` enabled — overnight for a trustworthy bias-instability floor). Writes per-axis gyro/accel noise characteristics to cal.json's `noise` section as **informational** sensor characterization — the filter always keeps its tuned `mekf_*`; these numbers never feed it. Never touches the sensors. |
| `fit-temp` | Offline linear gyro-bias/temperature fit from a warm-up capture (`--from FILE`, cold boot → warm, several °C of span). The daemon subtracts the fitted term from the gyro before fusion. Never touches the sensors. |

The magnetometer calibration fits a hard-iron offset and a 2D soft-iron
correction (including the cross term, so a distortion ellipse whose axes are
rotated relative to the sensor is corrected — a per-axis scale cannot do
this). It reports the field radius, a fit residual, and swing coverage; a
residual under ~1 µT and coverage above ~75% indicate a good fit. Afterward,
watch the wire's `mag_residual` / `mag_anomaly` compass-health metrics (v14,
also on the InfluxDB and Prometheus bridges): a rising residual means the
calibration is degrading and the swing should be repeated.

Restart when done:

```sh
sudo systemctl start imud
```

---

## 7. Output streams

imud publishes on up to three streams simultaneously. Full wire formats are in
[spec.md §7–8, §10](../spec.md).

> **Default posture (since 1.6):** a stock daemon enables only the local
> AF_UNIX stream socket. Every network output — NMEA (UDP *and* TCP),
> high-rate UDP, the stream TCP listener — is off until explicitly enabled
> in `imud.conf`. Nothing touches the network out of the box.

### NMEA 0183 — UDP broadcast and/or TCP listener, port 10110 (default off)

Text sentences for chartplotters, autopilots, and marine software (Signal K,
OpenCPN). Two transports, independently enabled:

- **UDP broadcast** (`enabled`) — for listen-only consumers on the LAN.
- **TCP listener** (`tcp_enabled`) — most plotter apps (OpenCPN, Navionics,
  iOS/Android nav apps) connect as TCP clients; point them at
  `tcp://<host>:10110`. Up to 8 clients; a slow client skips bursts, never
  stalls the daemon.

Per burst at `nmea.rate_hz`:

| Sentence | Contents |
|---|---|
| `$PASHR` | Roll, pitch, heading, **heave**, accuracy flags. |
| `$HCHDM` | Magnetic heading. |
| `$HCHDG` | Heading with magnetic variation (variation fields filled when declination is known). |
| `$TIROT` | Rate of turn (deg/min). |
| `$IIXDR` | Pitch and roll as transducer measurements. |
| `$HCHDT` | True heading — **only** emitted when declination is configured. |

### High-rate binary — UDP port 10111 (default off)

A fixed 260-byte little-endian packet (wire v14) at up to 500 Hz:
calibrated and raw accel/gyro/mag, quaternion, Euler angles, heading,
rate-of-turn, heave, temperature, the 3×3 attitude covariance, timestamps
(wall + TAI + chip), declination, and an IEEE-802.3 CRC32. Every packet is
self-describing (magic + version + CRC), so consumers can validate each one
independently. See [spec.md §8](../spec.md) for the exact layout and the
consumer libraries in [§9](#9-consumer-libraries).

### Local subscription stream — AF_UNIX socket (default on) + TCP listener (default off)

> This is the one output a stock daemon enables — the bridges and libimud
> consumers read it.

The same 260-byte binary packets over a `SOCK_STREAM` socket at
`/run/imud/imud-stream.sock`. Same-host consumers connect and receive a
loss-free stream (no datagram drops). Ideal for co-located machine-vision or
gimbal processes. Up to 8 subscribers; a consumer that can't keep up gets
packet gaps (visible in `imu_seq`), never blocks the daemon.

With `[stream] tcp_enabled = true` the same framed packets are also served
on a TCP listener (default port 10112) — the payoff is **remote consumers**:
run `imud-mon`, the Python client (`ImudClient.connect_tcp("boat.local")`),
or any libimud program (`imud_connect_tcp`, see `libimud(3)`) on a laptop
against a daemon elsewhere on the network, with the same lossless framing as
the local socket. Both listeners are fed by one thread at `rate_hz`.

### NMEA 2000 — via Signal K (recipe, no imud code involved)

imud has no direct N2K output by design: the Signal K server already owns the
hard device-level work (ISO address claim, product info) and its
[`signalk-to-nmea2000`](https://github.com/SignalK/signalk-to-nmea2000)
plugin converts the paths imud-signalk emits into PGNs. The chain is:

```
imud → imud-signalk (UDP delta) → Signal K server → signalk-to-nmea2000 → CAN bus
```

**You need:** a CAN interface the server can transmit on — a SocketCAN hat
(PiCAN2, Waveshare MCP2515, …) using a `canbus (canboatjs)` connection, or an
Actisense NGT-1 (`actisense-serial` with `toChildProcess: nmea2000out`).

**Steps:**

1. Enable imud's stream + the bridge: `imud.conf` `[stream] enabled = true`;
   `imud-signalk.conf` `enabled = true`, destination = the Signal K host,
   port 10113 (default). `sudo systemctl enable --now imud-signalk`.
2. Signal K server → *Data Connections* → add: data type **SignalK**, source
   **UDP**, port **10113**. imud's paths now appear under `vessels.self`.
3. Add the N2K connection (canbus/canboatjs or Actisense) with output
   enabled, install `signalk-to-nmea2000` from the Appstore, and enable the
   conversions below in its plugin config.

**Verified conversions** (checked against the plugin source; each is a
separate toggle):

| PGN | Content | Plugin conversion | Feeds from |
| --- | --- | --- | --- |
| 127250 | Vessel heading (magnetic + variation) | `Heading (127250)` | `navigation.headingMagnetic` + `navigation.magneticVariation` |
| 127250 | Vessel heading (true) | `TrueHeading (127250)` | `navigation.headingTrue` |
| 127257 | Attitude (roll/pitch/yaw) | `Attitude (127257)` | `navigation.attitude` |
| 127258 | Magnetic variation | `Magnetic Variation (127258)` | `navigation.magneticVariation` |

Enable imud's declination (WMM via `[position]`, or a static value) — the
magnetic-heading conversion takes variation as an input, and the
true-heading and variation conversions have nothing to emit without it.

**Not covered by the official plugin:** rate of turn (PGN 127251) and heave
(127252) have no conversions there — imud emits `navigation.rateOfTurn`, but
nothing converts it. If you need 127251 on the backbone, the community fork
[`signalk-nmea2000-emitter-cannon`](https://github.com/NearlCrews/signalk-nmea2000-emitter-cannon)
advertises broader PGN coverage (unverified here), and most N2K autopilots
and displays derive turn rate from the heading PGN themselves.

---

## 8. Monitoring and diagnostics

### imud-status

Query a running daemon over its Unix socket:

```sh
imud-status
```

It prints chip IDs, ODRs, fusion convergence and covariance, calibration
flags, current attitude, declination and true heading, heave, per-stream
output status, sample counts and overflow counts, uptime, and a "Recent
warnings" section listing the last few warning/error log lines. Use
`--socket PATH` for a non-default socket.

### imud-mon

A receive-side monitor that listens on the UDP output ports from any host on
the LAN (it needs no access to the daemon's socket):

```sh
imud-mon                      # both UDP streams, one line each, once/sec
imud-mon nmea                 # only the named stream
imud-mon binary
imud-mon --config config/sim.conf   # read ports/addresses from a specific config
```

It reads the port numbers and multicast addresses from the config file and
validates each binary packet (magic + CRC) before display.

### Raw capture with netcat

```sh
nc -u -l 10110                    # watch NMEA sentences (UDP broadcast)
nc -u -l 10111 > raw_packets.bin  # capture raw binary packets
nc HOST 10110                     # NMEA over TCP ([nmea] tcp_enabled)
nc HOST 10112 | xxd | head        # framed binary stream over TCP
                                  #   ([stream] tcp_enabled)
```

### Logging

imud logs to stderr (captured by journald under systemd) or to a file set by
`logging.file`. The verbosity is `logging.level` and is hot-reloadable.
Repeated identical messages are collapsed into a repeat count, and under
systemd each line carries an sd-daemon priority so `journalctl -p warning`
works. For a file log, rotate it with logrotate and a `postrotate: systemctl
reload imud` (the file is reopened on SIGHUP). See the
[`[logging]`](#logging) config section.

---

## 9. Consumer libraries

The `lib/` directory has ready-to-use clients for the binary stream, and a
microcontroller client lives in its own repository:

- **C** — **libimud** (`#include <imud.h>`, link `-limud`, `pkg-config
  libimud`): the ABI-stable shared library — applications keep working
  across imud upgrades without recompiling. Connects to the local socket
  (`imud_connect_stream`), the UDP stream (`imud_connect_udp`), or a remote
  daemon's `[stream]` TCP listener (`imud_connect_tcp`, 1.6). See
  `man 3 libimud`.
- **Python** — `lib/imud_client.py`: Python 3.8+, standard library only.
  Receives UDP or connects to the TCP stream listener
  (`ImudClient.connect_tcp(host, port)`). The
  `ImudPacket.true_heading_deg` property returns `None` when declination is
  unavailable.
- **Arduino/ESP32** —
  [imud-arduino](https://github.com/richcreations/imud-arduino) (library
  name `ImudClient`), maintained in its own repository. Header-only Arduino
  library (Arduino IDE and PlatformIO) that receives the stream over the
  `[stream]` TCP listener (port 10112) or high-rate UDP (port 10111,
  multicast included) through any Arduino `Client`/`UDP` transport.
  Wire-pinned: it is updated alongside wire revisions.
- **C single-header** — `lib/imud_client.h` is **deprecated**: it pins the
  wire version (recompile per revision). Kept in the source tree for
  existing vendored copies and imud's own wire-pinned internals; no longer
  installed by `make install`.

All validate CRC32 and support multicast. `make install` installs libimud
(library, header, pkg-config) and the Python module to
`/usr/local/share/imud`. See [libimud/manual.md](libimud/manual.md) for full
usage and examples, and [libimud/spec.md](libimud/spec.md) for the API,
`imud_data_t` fields, and the ABI contract.

---

## 9a. Bridges

imud has optional **bridge daemons** that republish its stream into other
ecosystems. Each is a **separate, optional package** with its own config file,
systemd service, man pages, and documentation (README + manual + spec) — installed
under `/usr/share/doc/imud-<name>/`, and in this tree under `docs/imud-<name>/`:

| Bridge | Output | Docs |
|---|---|---|
| `imud-signalk` | Signal K delta JSON over UDP / TCP | `docs/imud-signalk/`, `imud-signalk(8)` |
| `imud-mqtt` | MQTT topics + Home Assistant discovery | `docs/imud-mqtt/`, `imud-mqtt(8)` |
| `imud-influxdb` | InfluxDB line protocol (UDP / HTTP) | `docs/imud-influxdb/`, `imud-influxdb(8)` |
| `imud-mavlink` | MAVLink v1/v2 (UDP / serial / TCP) | `docs/imud-mavlink/`, `imud-mavlink(8)` |
| `imud-prometheus` | Prometheus `/metrics` HTTP exporter | `docs/imud-prometheus/`, `imud-prometheus(8)` |

All read imud's local stream, so they require `[stream] enabled = true` in
`imud.conf`. Build them with `make bridges` and install each with
`sudo make install-<name>`. See each bridge's **README** for a quick overview, its
**manual** for configuration and setup, and its **spec** for the exact output
format.

---

## 10. Troubleshooting

| Symptom | Check |
|---|---|
| Sensors not detected | I²C enabled? `i2cdetect -y 1` should list `0x6b` (ISM330DHCX) and `0x30` (MMC5983MA). Check wiring and the `i2c_addr` values. |
| `WHO_AM_I` mismatch at startup | Wrong `imu.driver`/`mag.driver`, or a wrong `i2c_addr` (e.g. SA0 jumper → 0x6A vs 0x6B). |
| GPIO open/permission errors | `gpio_chip` must match the Pi model (`gpiochip0` on Pi 4, `gpiochip4` on Pi 5). Run as the `imud` service user or a member of the `gpio`/`i2c` groups. |
| Fusion never converges | Magnetometer uncalibrated, or a strong local magnetic disturbance. Run `imud-cal mag`; check the fit residual. |
| Heading is off by a constant | Mount rotation. Set `mount.rotation_euler_deg` yaw to the chip-X-to-bow angle. |
| No true heading output | Declination not configured. Set `position.lat_deg`/`lon_deg`, or `declination_deg`, or enable gpsd/SignalK. |
| No NMEA received | `nmea.enabled = true` (UDP) or `nmea.tcp_enabled = true` (TCP)? Both default **off** since 1.6. Consumer listening on / connecting to the right port? Broadcast reachable on the subnet? |
| Config change ignored | `[restart]` keys need a full restart, not a `reload`. Check the log for a parse error (a bad value keeps the old config on reload, and prevents startup on boot). |

`imud-status` and the daemon log (`journalctl -u imud`) are the first place to
look. The expected first-boot log sequence and common failure modes are in
[spec.md §11](../spec.md).

---

## 11. Writing a driver

This section explains how to add support for a new IMU or magnetometer chip.
The driver abstraction lives in `include/drivers.h`; existing drivers in
`src/drivers/` are the best reference. Start by reading `src/drivers/sim.c`
(the simulation driver) — it is the minimal viable implementation of both
interfaces and has no hardware dependencies.

### Overview

imud separates chips into two independent driver types:

| Type | Interface | Example chips |
|---|---|---|
| **IMU** (`imu_ops_t`) | Accelerometer + gyroscope | ISM330DHCX, ICM-20948 |
| **Magnetometer** (`mag_ops_t`) | 3-axis magnetic field | MMC5983MA, AK09916 |

Each type is a struct of function pointers plus a few capability flags and
supported-rate tables. The daemon calls through these pointers and never
touches chip-specific registers directly.

Both types communicate over I²C using the Linux `I2C_RDWR` ioctl — no
`smbus` dependency — so the same low-level helpers work for all chips.

### Conventions that must be followed exactly

Getting these wrong produces silently wrong attitude output.

#### Output coordinate frame — NED-compatible board frame

All drivers must output sensor data in the **NED-compatible board frame**:

| Axis | Direction |
|---|---|
| X | Chip X marking (typically toward bow / forward) |
| Y | Starboard (right when facing forward) |
| Z | **Down** (into the PCB / toward earth when component-side up) |

This is a right-handed frame. With this convention and `rotation_euler_deg =
[0, 0, 0]`, a component-up installation with chip X pointing bow maps directly
to NED with no additional rotation. Only yaw needs setting when the chip X
axis does not align with the bow.

#### Accelerometer output — specific force, m/s²

Drivers report **specific force** in the NED-compatible board frame.

- **Flat, component-side up:** reads approximately `[0, 0, −9.80665]` m/s² on
  Z (the reaction force is upward = −Z in this frame).
- **Free-falling:** reads `[0, 0, 0]`.

Scale raw ADC counts to m/s² using the datasheet sensitivity (typically
mg/LSB × 9.80665 → m/s²/LSB), then apply any axis remapping needed to reach
the board frame above.

#### Gyroscope output — rad/s, right-hand rule

Scale raw counts to **rad/s** (not deg/s):

```c
const float d2r = (float)(M_PI / 180.0 / 1000.0);  /* mdps → rad/s */
gyro_scale = sensitivity_mdps_per_lsb * d2r;
```

Apply the same axis remapping as accel. Positive Z rotation = clockwise from
above (yaw right) in the NED Z-down convention.

#### Magnetometer output — µT

Output magnetic field in **microtesla (µT)** in the same board frame as the
IMU. If your datasheet gives mGauss/LSB or counts/Gauss, convert with
`1 Gauss = 100 µT`.

#### Axis remapping for the magnetometer

Many magnetometers share a PCB with the IMU but with one or more axes
physically inverted. After scaling, apply the sign flips needed so the mag
output is in the same NED-compatible board frame as the IMU.

For the SparkFun 9DoF (SEN-19895): the MMC5983MA Z is opposite the ISM330DHCX
chip-native Z (SparkFun marks this "MAG -Z"). After the ISM driver remaps its
own axes, the raw MMC Z already points down, so no Z change is needed; the Y
axis is the same physical direction as the ISM chip-native Y (port), so it
must be negated to reach starboard:

```c
out->field[1] = -(ry - NULL_FIELD) * scale;   /* flip Y: port → starboard */
out->field[2] =  (rz - NULL_FIELD) * scale;   /* Z already points down */
```

### The I²C helpers pattern

All existing drivers use the same three helpers — copy them into your file:

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

`burst_read` issues a combined write-then-read in one I²C transaction (no
repeated-start gap), saving ~40 µs vs two transactions at 400 kHz.

### Writing an IMU driver (`imu_ops_t`)

#### Static driver state

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

`seq` is a monotonic counter incremented for every sample produced. It must
**never reset** while the daemon runs — the fusion thread uses gaps in `seq`
to detect dropped samples.

#### `probe(fd, addr)` → 0 or -1

Read the WHO_AM_I (or equivalent) register and verify it against the datasheet
value. Log a clear error with the received and expected values on mismatch.

```c
static int myimu_probe(int fd, uint8_t addr)
{
    uint8_t who;
    if (reg_read(fd, addr, REG_WHO_AM_I, &who) < 0) {
        LOG_E("myimu: WHO_AM_I read failed: %s\n", strerror(errno));
        return -1;
    }
    if (who != WHO_AM_I_EXPECTED) {
        LOG_E("myimu: WHO_AM_I = 0x%02X, expected 0x%02X\n",
              who, WHO_AM_I_EXPECTED);
        return -1;
    }
    return 0;
}
```

Use the `LOG_E` / `LOG_W` / `LOG_I` macros from `include/log.h` for all driver
output, not bare `fprintf` — they respect the operator's `logging.level`.

#### `reset(fd, addr)` → 0 or -1

Trigger a software reset and wait for the bit to self-clear. Always add the
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
    LOG_W("myimu: SW_RESET did not clear after 50 ms\n");
    return -1;
done:
    usleep(20000);   /* chip startup time from datasheet */
    return 0;
}
```

#### `init(fd, addr, cfg)` → 0 or -1

Configure ODR, full-scale range, FIFO mode (if applicable), and interrupt
routing. Save the resulting sensitivity values to the static `s` struct.
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

#### `read(fd, addr, buf, max, *n_out)` → 0, 1, or -1

The hot path — called at the configured ODR. Fill `buf[]` with up to `max`
calibrated `imu_sample_t` samples, set `*n_out` to the number produced, and
return:

| Return | Meaning |
|---|---|
| `0` | Success |
| `1` | Success, but a **FIFO overflow** was detected (data gap) |
| `-1` | I²C error |

**For FIFO-based chips** (recommended — reduces I²C traffic):
1. Read FIFO status to get word count and overflow flag.
2. Burst-read all pending words.
3. Parse and scale each word into `imu_sample_t`.
4. Assign `seq` and `chip_ts`.

**For non-FIFO chips** (DRDY polling):
1. Check DRDY; return `*n_out = 0` if not ready.
2. Burst-read the output registers.
3. Produce exactly one sample per call.

Each `imu_sample_t` must have:

```c
buf[i].accel[3]  /* m/s², calibrated */
buf[i].gyro[3]   /* rad/s, calibrated (bias NOT subtracted — MEKF does that) */
buf[i].temp_c    /* die temperature, °C */
buf[i].seq       /* s.seq++ — monotonic, never resets */
buf[i].chip_ts   /* see Hardware timestamps below */
```

#### Hardware timestamps (`has_hw_timestamp`)

Set `has_hw_timestamp = true` only if your chip has an internal sample timer
that increments at a **fixed, known rate independent of the I²C clock**.

The ISM330DHCX has a 32-bit counter at 40000 ticks/s (25 µs/tick). If your
chip has an equivalent:

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
leave `chip_ts = 0` in every sample. The anchor mechanism in `imu.c` handles
this by updating the wall-clock anchor on every burst.

### Writing a magnetometer driver (`mag_ops_t`)

#### `probe`, `reset`, `init`

Same pattern as the IMU driver. `init` configures the ODR and enables
continuous measurement. If the chip has an interrupt pin, enable it during
`init` so the mag reader thread can wake on a GPIO edge rather than polling.

#### `read(fd, addr, *out)` → 0, 1, or -1

| Return | Meaning |
|---|---|
| `0` | Sample written to `*out`, `out->valid = true` |
| `1` | Measurement not complete yet (DRDY not asserted) |
| `-1` | I²C error |

Always set `out->wall_ns` from `CLOCK_REALTIME` at read time:

```c
struct timespec ts;
clock_gettime(CLOCK_REALTIME, &ts);
out->wall_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
out->valid   = true;
```

Set `out->valid = false` and return 0 (not -1) only when the hardware signals
a measurement overflow or other non-fatal data-quality issue.

#### `set_reset` (optional)

Some magnetometers have a SET/RESET degaussing coil that restores the AMR
sensing elements after exposure to a strong field. If your chip has one,
implement `set_reset` and set `has_set_reset = true`; the function issues the
pulse and sleeps for the settling time. Set `set_reset = NULL` and
`has_set_reset = false` if there is no coil (e.g. AK09916).

#### `has_interrupt`

Set `has_interrupt = true` if the chip asserts an external interrupt pin on
measurement complete — the mag reader thread requests a GPIO edge on
`[mag] int_gpio` and calls `read()` on each rising edge. Set `false` if there
is no interrupt pin (e.g. AK09916 in bypass mode); the reader falls back to a
10 ms poll loop.

### The `supported_*` tables

Zero-terminated ascending integer arrays telling the daemon which rates your
chip supports. The nearest-match logic in `imu.c` rounds the user's requested
rate to a real chip rate.

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

The last element must be `0` (sentinel); keep the list ascending.

### Registering the driver

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

Add the source file to the `DRIVER_SRCS` list in the `Makefile`, then enable
your chip in the config:

```toml
[imu]
driver   = "myimu"
i2c_addr = 0x68
int_gpio = 17
odr_hz   = 500
```

If the chip is untested on real hardware, set `experimental = true` in its ops
struct so the daemon warns at startup.

### Testing without hardware — the sim driver

`src/drivers/sim.c` implements both `sim_imu_ops` and `sim_mag_ops`. It
simulates a small boat under way, yawing at 6°/s from a 60° start heading with
wave-induced roll and pitch. To exercise the full pipeline before hardware
arrives:

```toml
[imu]
driver   = "sim"
i2c_addr = 0x00
int_gpio = 0          # 0 disables GPIO — reader uses a 10 ms timer

[mag]
driver   = "sim"
i2c_addr = 0x00
int_gpio = 0
```

Expected steady-state output:

```
pitch        ≈ ±2°  (8 s period)
roll         ≈ ±4°  (6 s period)
rate_of_turn ≈ 360 deg/min (with wave-induced oscillation)
heading      increases ~6°/s from a 60° start
```

### Pre-submission checklist

- [ ] `probe()` reads and validates the chip identity register.
- [ ] `reset()` waits for the reset bit to self-clear **and** adds the
      datasheet startup time afterward.
- [ ] `init()` stores sensitivity values to the static struct before
      returning.
- [ ] `read()` returns `-1` only on I²C errors, never on "no data yet".
- [ ] Accelerometer output is m/s² in the NED-compatible board frame (flat
      component-up reads ≈ −9.81 on Z).
- [ ] Gyroscope output is rad/s; magnetometer output is µT.
- [ ] Z-axis sign flip applied if chip Z is opposite the board IMU Z.
- [ ] `seq` is incremented for every sample, never reset.
- [ ] `chip_ts = 0` and `has_hw_timestamp = false` if there is no hardware
      timer.
- [ ] `supported_odr_hz[]` is zero-terminated and ascending.
- [ ] Driver added to the `src/drivers.c` registry and the `Makefile`.
- [ ] Logs use `LOG_*`, not bare `fprintf`.
- [ ] Tested with `driver = "sim"` and `make test` passes.
- [ ] Tested on real hardware with `imud-status` confirming sensor activity.
