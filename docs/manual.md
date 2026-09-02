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
(IMU) and a magnetometer over I²C or SPI, fuses them with a Multiplicative Extended
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
C standard library. Nothing else. `libgpiod` is itself optional: `make
NO_GPIOD=1` builds without it, and the Makefile drops it automatically when
`pkg-config` cannot find it. Both interrupt lines are then unavailable and the
reader threads use their rate-sized timer, so set `int_gpio = 0` under `[imu]`
and `[mag]` — a line the config asks for is a startup failure.

**Binaries.**

| Binary | Purpose |
|---|---|
| `imud` | The daemon. |
| `imud-cal` | Calibration tool (gyro bias, accel 6-position, magnetometer swing). |
| `imud-status` | Query a running daemon's health over its Unix socket. |
| `imud-mon` | Live monitor of the UDP output streams from any host on the LAN. |
| `imud-imutest` | Validate a sensor driver against real hardware and write a report. |
| `imud-signalk` | Bridge daemon (optional package): emits Signal K delta JSON over UDP (see [§9a Bridges](#9a-bridges)). |
| `imud-mqtt` | Bridge daemon (optional package): MQTT topics + Home Assistant discovery (see [§9a Bridges](#9a-bridges)). |
| `imud-influxdb` | Bridge daemon (optional package): InfluxDB line protocol over UDP/HTTP (see [§9a Bridges](#9a-bridges)). |
| `imud-mavlink` | Bridge daemon (optional package): MAVLink v1/v2 over UDP/serial (see [§9a Bridges](#9a-bridges)). |
| `imud-prometheus` | Bridge daemon (optional package): Prometheus `/metrics` exporter (see [§9a Bridges](#9a-bridges)). |

---

## 2. Building and installing

### Prerequisites

- A Linux system with an I²C **or** SPI bus and a GPIO character device.
  Debian bookworm and trixie are the packaged targets (arm64 and armhf);
  Raspberry Pi OS is the most exercised host, not a requirement.
- **The armhf packages are built for ARMv6**, so they run on every Raspberry
  Pi that boots a 32-bit userland — the Pi 1 and Pi Zero/Zero W included.
  Raspberry Pi OS 32-bit is a separate port built to ARMv6, while Debian's own
  armhf baseline is ARMv7-A; both call themselves `armhf` and
  `dpkg --print-architecture` reports the same string on either, so a package
  built to the Debian baseline installs happily on an ARMv6 board and then
  dies on an illegal instruction. imud's armhf packages are therefore built on
  Raspbian, which makes them correct on both ports.
- The bus you intend to use, enabled:
  - **I²C** — on Raspberry Pi OS `sudo raspi-config` → Interface Options →
    I²C. Elsewhere, load `i2c-dev` and confirm a `/dev/i2c-*` node appears.
  - **SPI** — `sudo raspi-config` → Interface Options → SPI, or add
    `dtparam=spi=on` to `/boot/firmware/config.txt` and reboot. Confirm
    `/dev/spidev0.0` and `/dev/spidev0.1` appear. Not every driver supports
    SPI; see [Supported drivers](#5-supported-drivers).
- `libgpiod-dev`. Bookworm ships 1.6.x and trixie ships 2.x; both are supported
  and the Makefile auto-detects the installed version via `pkg-config`. Omit it
  and the build falls back to a GPIO backend that takes no interrupts; see
  **Dependencies** above.

```sh
sudo apt update && sudo apt install -y build-essential libgpiod-dev
```

### Build

```sh
make            # builds imud, imud-cal, imud-imutest, imud-status, imud-mon
make test       # builds and runs the host-side unit tests (no hardware needed)
```

`./configure` is optional — `make` detects libgpiod, 64-bit atomics and the
host on its own. Run it to check the host before building, and to record the
answers in `config.mk`, which the Makefile then includes ahead of its own
defaults:

```sh
./configure                  # prints what this host can and cannot build
./configure --help           # options, including --prefix and --without-gpiod
make distclean               # remove config.mk
```

It fails only on the daemon's own dependencies: a C11 compiler, pthreads,
libm, and the Linux `i2c-dev` and `spidev` headers. Host byte order is
reported, not required — the binary packet and `.imucap` are little-endian on
either endianness. Everything else — libmosquitto for the MQTT bridge, and the
tools that regenerate documentation — is reported with what its absence costs,
and never fails the run.

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

`imud-mon` and `imud-imutest` share their own install target. They are
operator tools rather than part of the running system: `imud-mon` reads the
UDP streams and can live on any machine on the network, while `imud-imutest`
talks to the sensor directly — over whichever bus is configured — and must run
on the daemon's own box, with the daemon stopped.

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

`make install` also creates the dedicated system user `imud` that the service
runs as. Its **primary group is `imud`**, which owns `/run/imud/` and both
AF_UNIX sockets; hardware access comes from the supplementary groups `gpio`
(for `/dev/gpiochip*`) and `i2c` (for `/dev/i2c-*`). Those two groups are
**created** if missing — Raspberry Pi OS ships them, stock Debian does not, and
`imud.service` will not start without them. A udev rule
(`/etc/udev/rules.d/60-imud.rules`, or `/usr/lib/udev/rules.d/` from the `.deb`)
gives those groups access to the device nodes, which is what Raspberry Pi OS
provides in its own `99-com.rules`.

To read the stream socket or run `imud-status` as an ordinary user, join the
`imud` group — not `gpio`:

```bash
sudo adduser "$USER" imud     # log out and back in for it to take effect
```

`sudo make uninstall` reverses the install but preserves `/etc/imud/` (your
config and calibration).

Override install locations with the usual variables, e.g.
`sudo make install PREFIX=/opt/imud ETCDIR=/opt/imud/etc`. Packagers also
override `UDEVDIR` (default `/etc/udev/rules.d`) — udev reads rules only from
`/etc/udev/rules.d`, `/run/udev/rules.d` and `/usr/lib/udev/rules.d`, never from
`PREFIX`.

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
`NoNewPrivileges`, `DevicePolicy=closed` with explicit `DeviceAllow` lines,
`MemoryMax=32M`, a 10 s systemd watchdog). The shipped unit allows every
supported node — `/dev/i2c-1`, `/dev/i2c-3`, `/dev/spidev0.0`,
`/dev/spidev0.1`, `/dev/gpiochip0` and `/dev/gpiochip4` — so it needs no edit
on either a Pi 4 or a Pi 5, on either transport; an allow for a node the board
does not have is inert. Only the config needs to match your board
(`device.gpio_chip`, `device.i2c_bus`, and `spi_dev` if you use SPI).

The daemon runs as `imud` with `SupplementaryGroups=gpio i2c spi`. All three
groups are created by the package's maintainer scripts (or `make install`),
and `60-imud.rules` gives them the device nodes — Raspberry Pi OS already does
this in its own `99-com.rules`.

If you point `i2c_bus`, `spi_dev` or `gpio_chip` at a node outside that list —
SPI1..6, or a second SPI bus — add a matching `DeviceAllow=` line to the unit.
Under `DevicePolicy=closed` the open is refused otherwise, and the failure
looks like a permissions problem rather than a policy one. `make check-devices`
compares the shipped config against the shipped unit and fails the build if
they disagree, which is what stops the two drifting apart.

All six units — the daemon and the five bridges — additionally carry an empty
`CapabilityBoundingSet=`, `SystemCallArchitectures=native`, a
`SystemCallFilter=@system-service` allow-list minus `@privileged` and
`@resources`, `RestrictNamespaces`, `LockPersonality`, `RestrictSUIDSGID`,
`ProtectHostname`, and `ProtectProc=invisible` with `ProcSubset=pid`. Together
those put every unit at OK under `systemd-analyze security`.

One deliberate exception: **`imud.service` does not set `ProtectClock=`**, and
re-allows `adjtimex` after its `~@privileged` line. The daemon calls `adjtimex`
read-only at startup to report the CLOCK_TAI offset; `ProtectClock=` blocks that
call outright, with no way to allow it back, which would turn the check into a
false "chrony has not set tai_offset" warning on every boot. Setting the clock
is already impossible without CAP_SYS_TIME, which the empty
`CapabilityBoundingSet=` removes. The five bridges never call `adjtimex` and do
set `ProtectClock=true`.

`imud.service` runs at `UMask=0027` and the bridges at `0077`; the daemon
tightens an inherited umask but never loosens it. Both AF_UNIX sockets
(`/run/imud/imud.sock` and the stream socket) are created at mode 0660 rather
than chmod'd down after `bind()`, so they are never momentarily wider — however
`imud` was launched, systemd or not.

### Command-line options

```
imud [OPTIONS]
  --config PATH      Config file (default: /etc/imud/imud.conf)
  --replay FILE      Replay an .imucap capture through the full pipeline
                     (forces both sensors onto the sim driver, plays the file
                     once and exits; docs/capture.md)
  --skip-bias-cal    Skip the startup gyro-bias estimation window
  --no-nmea          Disable the NMEA output stream
  --no-highrate      Disable the high-rate binary stream
  --foreground       Accepted and ignored (imud always runs in the foreground)
  --version          Print the version and exit
  -h, --help         Print this help and exit
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

**Default locations.** `--config /path/to/imud.conf` names the file outright;
without it the daemon searches, first match wins:
1. `/etc/imud/imud.conf` (installed by `make install`)
2. `~/.config/imud/imud.conf`

`--config` *replaces* the search rather than heading it. A path named there that
does not exist means built-in defaults and a warning — not a quiet fall back to
`~/.config/imud/imud.conf`, which would let a typo start the daemon on a
different configuration. Without `--config`, neither file existing is fine.

**Reload behaviour:**
- Keys marked **[restart]** take effect only after restarting the daemon
  (`systemctl restart imud`).
- Keys marked **[hot]** can be reloaded at runtime by sending `SIGHUP`:
  ```sh
  sudo systemctl reload imud   # or: kill -HUP $(pidof imud)
  ```
  Unknown keys and bad values in a SIGHUP reload are logged as warnings; the
  previously loaded values are kept.
- A reload re-reads **the file startup actually loaded**, including when that was
  the `~/.config/imud/imud.conf` fallback, and it starts from the built-in
  defaults before applying that file — exactly as startup does. So **deleting a
  [hot] key from the file restores its default** on the next reload, instead of
  leaving the running value in place until a restart; reload and restart agree.
- If the config file has since been deleted, the running configuration is kept.
  A deleted config does not revert a running daemon to defaults.

**Syntax:**
- Comments start with `#` and may appear inline.
- Strings must be double-quoted: `driver = "ism330dhcx"`.
- Integers may be decimal (`833`) or hex (`0x6B`). An integer too large for the
  platform's `int` is a fatal error, not silently wrapped — `dest_port =
  4294977414` is refused rather than becoming port 10118. Keys with a
  meaningful range are checked against it: ports `1`–`65535`, `int_gpio`
  `0`–`255` (`0` means the sensor has no interrupt line), `i2c_addr`
  `0x00`–`0x7F`.
- Booleans: `true` / `false`.
- Arrays: `[0.0, 0.0, 0.0]` (used by `rotation_euler_deg` and `rotation_matrix`). The element count is checked exactly.
- `~/` paths are expanded to `$HOME/`.
- A string value longer than the field it sets is a fatal error, and so is one
  that fits only until `~/` expands. imud never shortens a value and continues:
  a truncated socket or file path is a perfectly valid path to the wrong place,
  and every client would then connect to the path you actually wrote and find
  nothing there.

The annotated reference file at `config/imud.conf` documents every key inline
and is the fastest way to see a working configuration.

### `[device]`

Hardware bus and GPIO controller. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys device.1 -->
| `i2c_bus` | string | `"/dev/i2c-1"` | I²C bus device node. Use `/dev/i2c-1` on Pi 4; `/dev/i2c-1` or `/dev/i2c-3` on Pi 5 depending on which header pins are used. |
| `gpio_chip` | string | `"gpiochip0"` | gpiochip device name. **Run `gpiodetect` and use the chip it lists for the header pins** — the number comes from probe order and is not stable across kernels. `"gpiochip0"` is right on Pi 4, and on Pi 5 with kernels from mid-2024 onward (which renumber the RP1 controller to 0 like every other model). Earlier Pi 5 kernels exposed it as `"gpiochip4"`, and Pi OS keeps a `/dev/gpiochip4` symlink for compatibility — a symlink, not a second controller. |
| `sim_file` | string | `""` | An `.imucap` capture for the sim driver to replay (`driver = "sim"` in both `[imu]` and `[mag]`); empty selects the built-in synthetic scenario. `imud --replay FILE` is the shortcut. See [capture & replay](capture.md). |
| `sim_loop` | bool | `false` | Repeat the capture forever; timestamps and sequence numbers are rebased to stay monotonic. Ignored under `--replay`, which plays a file once and exits. |
| `sim_speed` | float | `1.0` | Playback pacing: `2.0` = double speed, `0` = as fast as the pipeline accepts. |
<!-- END GENERATED: config-keys device.1 -->

### `[imu]`

IMU (gyroscope + accelerometer) driver settings. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys imu.1 -->
| `driver` | string | `"ism330dhcx"` | Driver to load. See [Supported drivers](#5-supported-drivers). |
| `bus` | string | `"i2c"` | Transport: `"i2c"` or `"spi"`. SPI is faster per transfer, which is what unlocks the high sample rates and shortens the FIFO drain; not every driver has it, and the daemon refuses to start on one that does not (see [Supported drivers](#5-supported-drivers)). The IMU and the magnetometer choose independently. |
| `spi_dev` | string | `""` | spidev node, e.g. `"/dev/spidev0.0"` (CE0). **Required** when `bus = "spi"`; ignored otherwise. The chip select in the node name does the addressing, so `i2c_addr` is unused. |
| `spi_speed_hz` | int | `0` | SPI clock in Hz. `0` means the driver's datasheet maximum, which is the useful default. A request above that maximum is clamped rather than refused, and the daemon logs what it really programmed. **Do not set this below 2.5 MHz when a magnetometer shares the same SPI controller.** Measured on an ism330dhcx + mmc5983ma pair: driving the IMU below about 2.2 MHz stops the magnetometer completing measurements entirely, and re-running `init()` cannot hold it. The default is unaffected, and the daemon warns at startup when it sees a slow clock alongside a shared-bus magnetometer. |
| `i2c_addr` | int | `0x6B` | I²C address; used only when `bus = "i2c"`. `0x6B` (SA0 high) or `0x6A` (SA0 low via jumper). |
| `int_gpio` | int | `17` | BCM GPIO number for the FIFO watermark interrupt (board pin 11). Set `0` to poll instead of using a hardware interrupt. The polling cadence is then the same `fifo_wm + int_grace` sample periods the interrupt path waits for, unless `poll_ms` overrides it: a flat cadence under-polls at high ODR, where the FIFO overflows and the effective rate drops below the configured one, and burns reads at low ODR. |
| `odr_hz` | int | `833` | Output data rate in Hz; must be greater than zero. A rate the chip cannot produce is rounded **up** to the next one it can, and the filter is tuned for that actual rate — the daemon logs `requested, N Hz actual` at startup when the two differ. ISM330DHCX supports: `13.016`, `26.031`, `52.063`, `104.125`, `208.25`, `416.5`, `833`, `1666`, `3332`, `6664`. Other drivers differ — see the driver table in [Supported drivers](#5-supported-drivers), and note that the top rates of some parts are beyond what a Raspberry Pi can sustain. |
| `accel_g` | int | `8` | Accelerometer full-scale range in g. ISM330DHCX: `2`, `4`, `8`, `16`. |
| `gyro_dps` | int | `2000` | Gyroscope full-scale range in degrees/second. ISM330DHCX: `125`, `250`, `500`, `1000`, `2000`, `4000`. |
| `fifo_wm` | int | `64` | FIFO watermark in sample-sets: how deep the chip buffers before raising its interrupt. This sets **both buffer depth and sample latency** — with `int_grace`, the reader waits `fifo_wm + int_grace` sample periods, so 64 at 833 Hz batches about 77 ms. On the ST parts a watermark of 8 or more also batches the chip timestamp into the FIFO; at 32 and above that costs 1.6% of the word budget, so the same watermark holds about 63 sample-sets rather than 64. |
| `int_grace` | int | `2` | How late the interrupt may be, in **samples**, before the reader gives up and reads anyway. The wait is `fifo_wm + int_grace` sample periods, so the fallback fires only when the line is genuinely *late* — never merely because the batch is not ready yet. Counted in samples rather than milliseconds because a fixed time means a different thing at every rate: 2 ms is thirteen samples at 6664 Hz and three hundredths of one at 13 Hz. Ignored when `int_gpio = 0`, where there is no expected arrival to be late against. |
| `poll_ms` | int | `0` | Polling cadence in milliseconds for an install with **no** interrupt line. `0` derives it from the rate and watermark, which is normally what you want: a fixed interval under-polls at high ODR — the FIFO overflows and the delivered rate falls below the configured one — and burns reads at low ODR. Set a non-zero value only to force a specific cadence. **Ignored entirely when `int_gpio` is non-zero.** |
<!-- END GENERATED: config-keys imu.1 -->

### `[mag]`

Magnetometer driver settings. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys mag.1 -->
| `driver` | string | `"mmc5983ma"` | Driver to load. See [Supported drivers](#5-supported-drivers). |
| `bus` | string | `"i2c"` | Transport: `"i2c"` or `"spi"`, as for `[imu]` above. The AKM compasses (`ak09916`, `ak8963`) are I²C-only — they have no SPI port and are reached through the host IMU's bypass. |
| `spi_dev` | string | `""` | spidev node, e.g. `"/dev/spidev0.1"` (CE1 — the IMU usually takes CE0). **Required** when `bus = "spi"`. |
| `spi_speed_hz` | int | `0` | SPI clock in Hz; `0` means the driver's datasheet maximum. As for `[imu]`. Note that a magnetometer's *own* clock is not what starves it on a shared controller — a slow `[imu] spi_speed_hz` is. This key can be lowered safely. |
| `i2c_addr` | int | `0x30` | I²C address; used only when `bus = "i2c"`. MMC5983MA has a fixed address. The AKM compasses inside a 9-axis IMU — AK09916 in the ICM-20948, AK8963 in the MPU-9250/9255 — sit behind the host chip's I²C **bypass**, not its I²C master, and answer on the host bus at their own address: set `0x0C` for both. |
| `int_gpio` | int | `27` | BCM GPIO number for the measurement-done interrupt (board pin 13). Set `0` to poll on a timer. |
| `odr_hz` | int | `100` | Output data rate in Hz; must be greater than zero. Rounded **up** to a supported rate as for `[imu] odr_hz`, and the mag noise variance is sized for that actual rate. MMC5983MA supports: `1`, `10`, `20`, `50`, `100`, `200`, `1000`. |
| `set_period_s` | float | `5.0` | Interval in seconds between SET/RESET degauss pulses. Prevents gradual magnetisation of the sensor. Set `0` to disable. |
| `int_grace` | int | `2` | How late the interrupt may be, in **samples**, before the reader gives up and reads anyway. These magnetometers have no FIFO, so the line signals one finished conversion and the wait is `1 + int_grace` sample periods, so the fallback fires only when the line is genuinely *late* — never merely because the batch is not ready yet. Counted in samples rather than milliseconds because a fixed time means a different thing at every rate: 2 ms is thirteen samples at 6664 Hz and three hundredths of one at 13 Hz. Ignored when `int_gpio = 0`, where there is no expected arrival to be late against. |
| `poll_ms` | int | `0` | Polling cadence in milliseconds for an install with **no** interrupt line. `0` derives it from the rate, which is normally what you want: a fixed interval under-polls at high ODR — the FIFO overflows and the delivered rate falls below the configured one — and burns reads at low ODR. Set a non-zero value only to force a specific cadence. **Ignored entirely when `int_gpio` is non-zero.** |
<!-- END GENERATED: config-keys mag.1 -->

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
<!-- BEGIN GENERATED: config-keys fusion.1 -->
| `mekf_gyro_noise` | double | `0.007` | Gyro noise density in rad/s/√Hz. ISM330DHCX datasheet: 7 mdps/√Hz ≈ 0.000122 rad/s/√Hz. |
| `mekf_gyro_bias` | double | `0.00015` | In-run gyro bias instability in rad/s. Sets the bias random-walk process noise. |
| `mekf_accel_noise` | double | `0.0022` | Accelerometer **sensor** noise density in m/s²/√Hz. ISM330DHCX: ~186 µg/√Hz × 9.81. Not the knob for rough weather — see `mekf_wave_accel`. |
| `mekf_mag_noise` | double | `0.0004` | Magnetometer noise density in Gauss/√Hz. MMC5983MA: 0.4 mGauss RMS. |
| `mekf_wave_accel` | double | `0.8` | Steady-state σ, in m/s², of the wave-orbital acceleration contaminating the gravity measurement. **This is the knob for "the sea is rough."** The disturbance is correlated over roughly a second rather than white, so it is carried as a first-order Gauss–Markov state; without it the filter treats 833 strongly correlated samples per second as 833 independent measurements of gravity and diverges. `0` (or `mekf_wave_accel_tau_s = 0`) disables the state and restores the pre-1.7 6-state filter exactly. Too small is the failure mode that hurts, so round up; `imud-cal fit-ra` measures it from a capture. |
| `mekf_wave_accel_tau_s` | double | `0.5` | Correlation time in seconds of the Gauss–Markov wave-acceleration state. `0` disables. **Unrelated to `wave_tau_s`**, which is the sea-state reporting window. Attitude error and wave acceleration are told apart only by their dynamics, so too long a value lets the state absorb genuine tilt: benchmark yaw-only attitude RMS degrades 2.3° at 0.5 s → 7.1° at 2 s. Short is the safe side. |
| `mekf_mag_dip_sigma_deg` | double | `1.0` | Uncertainty in degrees of the magnetic reference's **dip** angle. Applies only to 3-D vector fusion (`mag_yaw_only = false`), where the dip constrains roll and pitch — a wrong dip reference therefore biases them, permanently, and the error cannot be learned out at sea. It is admitted into the covariance instead, as a rank-1 anisotropic term in the magnetometer noise. Set `0` when a position source supplies WMM field invariants, which removes the error at its source and is strictly better. Raising it improves covariance consistency and lowers the `nis_mag` wire field by design. |
| `mag_reject_gauss` | double | `0.05` | Strong-anomaly cutoff: reject a magnetometer measurement whose post-calibration residual exceeds this value (Gauss). Guards against nearby iron/magnets. Fine-grained consistency is handled internally by χ² innovation gates that self-scale with filter confidence, so keep this a coarse threshold (~10% of the Earth field). |
| `accel_skip_thresh` | double | `0.05` | Skip an accelerometer update if `||a| − 1g|` exceeds this fraction of g. Prevents linear acceleration from corrupting the tilt estimate. `0.05` = skip if more than 5% off 1g. |
| `mag_yaw_only` | bool | `true` | Heading-only magnetometer fusion (marine default): the mag corrects heading and never pulls on roll/pitch. The swing-circle calibration is structurally 2D, so the field's vertical (dip) channel is its least-calibrated component. Set `false` for full 3D vector fusion — appropriate only for magnetically clean installs with a true 3D calibration. |
| `mag_fuse_uncal` | bool | `true` | Fuse the magnetometer when no `cal.json` is present. The heading is then offset by the uncorrected hard iron — wrong by a bounded, repeatable, heading-dependent deviation rather than the unbounded drift of gyro dead reckoning, which is what a heading-hold consumer needs. The update is forced heading-only regardless of `mag_yaw_only`, so the uncorrected dip cannot reach roll or pitch, and it is withdrawn automatically above `mag_uncal_reject_frac`. `FLAG_MAG_UNCAL` (bit 15) is the wire signal for this state; `FLAG_MAG_VALID` keeps its original meaning and stays clear. Set `false` to restore dead reckoning. |
| `mag_uncal_reject_frac` | double | `0.4` | Withdrawal threshold for an uncalibrated magnetometer, as a fraction of the reference horizontal/total field ratio |H|/|B|. Once the horizontal hard iron exceeds the horizontal field, two true headings read the same and the sense of the error inverts over half the compass rose, so a heading-hold consumer diverges rather than degrades; that boundary is what this gate approximates, using the `mag_anomaly` field-magnitude EMA. Deliberately coarse: the EMA is a mean rather than a peak (it under-reads a constant-rate turn by about 2/π) and a mostly-vertical offset inflates it without harming heading-only fusion, so treat it as a backstop against gross iron, not a precise boundary. Re-admission is hysteretic at 0.8x. `0` never withdraws. Ignored unless `mag_fuse_uncal` is in effect. |
| `heave_tau_s` | float | `12.0` | Heave filter time constant in seconds. Heave (vertical displacement) is a band-passed double integration of vertical acceleration; it feeds the `$PASHR` heave field and the binary packet's `heave_m`. The passband covers ~2–15 s wave periods at the default; allow ~2 minutes of settling after startup. `0` disables (heave reads 0.0). Values far above the default are not useful — the passband stops covering waves — and were once actively dangerous: the filter's leak is sized by `dt/heave_tau_s`, and in single precision it stopped working entirely at a high ODR, turning a bounded band-pass into an unbounded double integrator (253 m of "heave" at 32 kHz with a 900 s constant). The accumulators are double precision as of this release, so the whole range now degrades gracefully; see [math.md §1](math.md). |
| `wave_tau_s` | float | `120.0` | Sea-state averaging window in seconds. Significant wave height (Hs = 4·σ(heave)), mean zero-crossing wave period, and the vessel's roll/pitch periods and significant single amplitudes (2σ) are exponentially weighted statistics of the heave/roll/pitch oscillations over this window (packet fields `wave_height_m`, `wave_period_s`, `roll_period_s`, `roll_amplitude`, `pitch_period_s`, `pitch_amplitude`, gated by the `wave_valid` flag). Stats settle ~2 windows after heave settles. Oceanographic practice is 10–20 minute records, and windows that long are supported — but note they interact with `[imu] odr_hz`, because the statistics' gain is `dt/wave_tau_s`. At 32 kHz with a 1200 s window that gain is 2.6e-8, which single precision cannot accumulate; significant wave height under-read by 18 % while still reporting `wave_valid`. Fixed by double-precision accumulators as of this release. Requires `heave_tau_s` > 0; `0` disables. |
| `engine_vibration_g2` | double | `0.0` | EMA threshold (m²/s⁴) for engine-vibration detection. The filter tracks an exponential moving average of `(|a| − g)²`; when it exceeds this value the engine is considered on. Set `0` to disable (default). |
| `engine_accel_skip_thresh` | double | `0.20` | Accelerometer skip threshold applied while engine vibration is detected: wider than `accel_skip_thresh` so the filter isn't starved, while the accel noise is inflated ×4 so the vibration-contaminated samples that pass are trusted proportionally less. Only active when `engine_vibration_g2 > 0`. |
<!-- END GENERATED: config-keys fusion.1 -->

### `[calibration]`

Calibration file and startup behaviour. **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys calibration.1 -->
| `file` | string | `"/etc/imud/cal.json"` | Path to the calibration JSON produced by `imud-cal`. Supports `~/` expansion. If the file does not exist, imud runs uncalibrated (magnetometer readings will be biased). |
| `startup_settle_sec` | double | `5.0` | Seconds of sensor data to discard after chip initialisation. The ISM330DHCX gyro drifts ~0.02 rad/s for ~5 s after power-on; discarding this window keeps it out of the bias estimate. Set `0` for the `sim` driver. |
| `align_window_sec` | double | `5.0` | Seconds of accelerometer and magnetometer averaging used for the one-shot initial attitude alignment. Whatever tilt error survives this window is baked permanently into the magnetic reference's dip, so in a seaway the length matters: 1 s (the value hardcoded before 1.7) is about a fifth of a roll period and aligns to an arbitrary point in the cycle — measured attitude RMS in the marine default is 47.7° at 1 s against 2.19° at 5 s, flat beyond. 3-D vector fusion keeps improving out to ~15 s. The only cost of a longer window is startup latency before usable attitude, so raise it when starting from a mooring, not underway. |
| `gyro_bias_sec` | double | `2.0` | Length of the stationary window used to estimate gyro bias at startup. The board must be still for this duration. Skipped if the cal file already has a gyro bias. If motion is detected during the window (gyro std > 0.5 °/s) the window is doubled once and a warning is logged. Set `0` to skip bias estimation. |
<!-- END GENERATED: config-keys calibration.1 -->

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
<!-- BEGIN GENERATED: config-keys nmea.1 -->
| `enabled` | bool | `false` | Enable the NMEA UDP output stream. |
| `rate_hz` | int | `10` | Output rate in Hz (shared by UDP and TCP); must be greater than zero. Hot-reloadable. |
| `dest_addr` | string | `"255.255.255.255"` | Destination IP address. `255.255.255.255` = broadcast; use a unicast or multicast address to target a specific host. |
| `dest_port` | int | `10110` | Destination UDP port. Standard NMEA-over-UDP port is 10110. |
| `tcp_enabled` | bool | `false` | NMEA-over-TCP listener. Chartplotter apps (OpenCPN, Navionics, phone/tablet nav apps) connect as TCP clients and each receives every sentence burst; up to 8 clients, slow clients skip bursts rather than stalling the daemon. |
| `tcp_bind_addr` | string | `"0.0.0.0"` | Listener bind address (numeric IPv4). `127.0.0.1` keeps it host-local. |
| `tcp_port` | int | `10110` | Listener TCP port (the de-facto NMEA-over-TCP port). |
<!-- END GENERATED: config-keys nmea.1 -->

### `[highrate]`

High-rate binary UDP stream (500 Hz by default). **[restart]**: `enabled`, `dest_addr`, `dest_port`, `coord_frame`. **[hot]**: `rate_hz`.

The 276-byte binary packet format is documented in [spec.md §8](../spec.md).
Consumer libraries are in `lib/`.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys highrate.1 -->
| `enabled` | bool | `false` | Enable the high-rate binary stream. Opt in for machine vision, ROS2, or any consumer needing quaternion + covariance at high rate. |
| `rate_hz` | int | `500` | Publish rate in Hz; must be greater than zero. The MEKF runs at the full `imu.odr_hz` internally; this controls how often results are published. Hot-reloadable. |
| `dest_addr` | string | `"239.255.0.1"` | Destination IP. Default is an IPv4 multicast group (TTL=1, link-local). Consumers join with `IP_ADD_MEMBERSHIP`. Use `255.255.255.255` for broadcast or a unicast IP for point-to-point. |
| `dest_port` | int | `10111` | Destination UDP port. |
| `coord_frame` | string | `"NED"` | Output coordinate frame: `"NED"` (North-East-Down) or `"ENU"` (East-North-Up). Affects the quaternion, gyro, accel, and mag vector fields in the binary packet. |
<!-- END GENERATED: config-keys highrate.1 -->

### `[stream]`

Local AF_UNIX subscription stream — the same 276-byte binary packets as
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
<!-- BEGIN GENERATED: config-keys stream.1 -->
| `enabled` | bool | `true` | Enable the subscription stream — the one output a stock daemon provides (the bridges and libimud consumers read it). |
| `socket` | string | `"/run/imud/imud-stream.sock"` | Listen path (mode 0660, owner `imud:imud` — a consumer needs `adduser <user> imud`). |
| `rate_hz` | int | `100` | Per-subscriber packet rate in Hz (AF_UNIX and TCP); must be greater than zero. Hot-reloadable. |
| `tcp_enabled` | bool | `false` | TCP listener carrying the same framed packets over the network — lossless like the AF_UNIX socket. |
| `tcp_bind_addr` | string | `"0.0.0.0"` | Listener bind address (numeric IPv4). `127.0.0.1` keeps it host-local. |
| `tcp_port` | int | `10112` | Listener TCP port. |
<!-- END GENERATED: config-keys stream.1 -->

### `[capture]`

The black box: records every raw sensor sample (pre-mount, pre-calibration —
exactly as the driver delivered it) to rotating `.imucap` files. Replay with
`imud --replay`; analyze with `imud-cal characterize` / `fit-temp`. Roughly
25 MB/hour at 104 Hz. See [capture & replay](capture.md). **[restart]**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys capture.1 -->
| `enabled` | bool | `false` | Record raw samples from both sensors. |
| `dir` | string | `"/var/lib/imud"` | Destination; files are `imud-YYYYMMDD-HHMMSS.imucap` (UTC). |
| `max_mb` | int | `256` | Rotate the file at this size. `0` = unlimited. |
| `max_files` | int | `8` | Keep the newest N files, deleting the oldest. `0` = keep all. |
| `flush_s` | int | `5` | Flush-to-storage interval, seconds. |
<!-- END GENERATED: config-keys capture.1 -->

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
<!-- BEGIN GENERATED: config-keys mount.1 -->
| `rotation_euler_deg` | array | `[0.0, 0.0, 0.0]` | `[roll, pitch, yaw]` in degrees. Measure the angle between chip X and the forward axis once (e.g. with a handbearing compass on a vessel); set that as `yaw`. |
| `preset` | string | *(unset)* | Named shortcut: `"identity"`, `"yaw_90"`, `"yaw_180"`, `"yaw_270"`, `"roll_90"`, `"roll_270"`, `"pitch_90"`, `"pitch_270"`. Overrides `rotation_euler_deg` when set. An unrecognised name is a fatal config error. |
| `rotation_matrix` | array of 9 | *(unset)* | Board→body rotation given directly, row-major (`v_body = R · v_board`). Validated at load against `RᵀR = I` and `det(R) = +1`; a non-orthonormal matrix or a reflection is a fatal config error. Whichever mount key appears last wins. |
<!-- END GENERATED: config-keys mount.1 -->

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

The `[stats]` line ends with a sample-latency clause —
`fifo=p50/p99 pipe=p50/p99 max=fifo/pipe` in milliseconds — on parts that have
a hardware timestamp counter. `fifo` is the age of a sample when its read
completed; `pipe` is the daemon's own cost from there to fused state, and is
the term to hold to a budget (0.26 ms p99 on a Pi 5 at 833 Hz).

Three things about reading it:

- **The percentiles are bucket upper edges, and `max=` is exact.** The
  histogram is log₂-spaced, so a reported `p99` of `16.4 ms` means the true
  value is somewhere in `[8.2, 16.4)`. It never flatters, but it can overstate
  by up to 2×. When you want a number rather than a bound, use `max=`.
- **`fifo` is set by `fifo_wm`.** The reader wakes on the watermark interrupt,
  with a fallback of `fifo_wm + int_grace` sample periods if the line is late.
  The fallback is by construction later than the watermark it backs up, so on a
  healthy line the watermark decides and residence is about `fifo_wm / odr_hz`
  — measured at 75.8 ms against a predicted 76.8 ms at 833 Hz with `wm = 64`.
  See the `fifo_wm` entry above.
- **It takes a few seconds to appear**, because the chip's real tick period
  has to be measured against the host clock first. On `icm20948` and
  `mpu925x`, which have no chip timer, it is absent entirely — both are normal
  rather than faults.

A drain clause follows it — `drains=E/T e/t n=MEAN max=DEEPEST` — saying how
the FIFO is actually being emptied. `E` is drains woken by the watermark
interrupt, `T` drains woken by the fallback timer, `n` the mean samples per
drain and `max` the deepest single burst since start.

Read the **split**, not the mean: it is a health check on the interrupt line.
`E` should dominate overwhelmingly — a healthy line gives one timeout per run or
none at all, because the fallback only expires when an edge is genuinely late.
A `T` that is climbing means the watermark interrupt is not arriving, and the
reader is running on its fallback instead; residence then stretches to
`fifo_wm + int_grace` sample periods rather than `fifo_wm`, and the first thing
to check is the wiring and `int_gpio`. On an install with no IMU interrupt line,
`E` is 0 by construction and the cadence is the poll instead.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys logging.1 -->
| `level` | string | `"warn"` | Log verbosity: `"debug"`, `"info"`, `"warn"`, or `"error"`. `"warn"` is recommended for production (SD-card friendly). `"info"` adds lifecycle messages and a periodic `[stats]` heartbeat. Applied live on SIGHUP. |
| `file` | string | `""` | Log destination. Empty = stderr; under systemd, lines carry sd-daemon priority prefixes so journald records real priorities (`journalctl -p warning` filters work). Set an absolute path for a timestamped log file; it is reopened on SIGHUP so logrotate can rotate it (`postrotate: systemctl reload imud`). |
| `stats_hz` | int | `1` | Rate of the `[stats]` heartbeat log line. Only visible at `level = "info"` or `"debug"`. |
<!-- END GENERATED: config-keys logging.1 -->

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
<!-- BEGIN GENERATED: config-keys position.1 -->
| `declination_deg` | float | `0.0` | Static declination in degrees. East positive (+), west negative (−). Ignored when `lat_deg` and `lon_deg` are both non-zero. |
| `lat_deg` | double | `0.0` | Geodetic latitude in decimal degrees (+N / −S). Set with `lon_deg` to enable WMM auto-compute. |
| `lon_deg` | double | `0.0` | Geodetic longitude in decimal degrees (+E / −W). Set with `lat_deg` to enable WMM auto-compute. |
| `wmm_file` | string | `""` (auto) | Path to the WMM coefficient file. Empty = auto-resolve: `/etc/imud/WMM.COF` if present (operator override), else `/usr/share/imud/WMM.COF` (`imud-wmm-data` package / `make install-wmm-data`). Bundled model WMM2025, valid 2025.0–2030.0. |
<!-- END GENERATED: config-keys position.1 -->

**Live position sources**

Enable one or both to receive GPS-driven WMM updates as the vessel moves. WMM
is recomputed when position changes by ≥ 0.05° (≈ 5 km). Priority: **gpsd**
(live stream) > **SignalK** (polled fallback) > **static lat/lon** > **static
declination**.

_gpsd:_

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys position.2 -->
| `gpsd_enabled` | bool | `false` | Connect to gpsd for live position (and speed). gpsd must be running with a working GPS source. |
| `gpsd_host` | string | `"localhost"` | Hostname or IP of the gpsd instance. |
| `gpsd_port` | int | `2947` | TCP port of the gpsd instance. |
<!-- END GENERATED: config-keys position.2 -->

imud subscribes to gpsd's JSON stream and processes `TPV` messages with
`mode ≥ 2`. The connection is persistent and reconnects automatically.

_SignalK:_

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys position.3 -->
| `signalk_enabled` | bool | `false` | Poll the SignalK REST API for position. |
| `signalk_host` | string | `"localhost"` | Hostname or IP of the SignalK server. |
| `signalk_port` | int | `3000` | HTTP port of the SignalK server. |
| `signalk_path` | string | `"/signalk/v1/api/vessels/self/navigation/position"` | REST endpoint path. Override for a non-standard path or vessel ID. |
<!-- END GENERATED: config-keys position.3 -->

SignalK is polled every 30 seconds. When gpsd is also enabled, SignalK is a
fallback (polled once when gpsd drops), still respecting the 30 s minimum.

_Stale-fix TTL:_

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys position.4 -->
| `fix_max_age_h` | float | `24.0` | Hours to keep a GPS-derived declination after the last fix. The TTL resets on every fix — an anchored vessel with continuous GPS keeps declination valid indefinitely. After the window expires without a fix, `FLAG_DECLINATION_VALID` clears and true-heading output stops. Set `0` to never expire. |
<!-- END GENERATED: config-keys position.4 -->

---

## 5. Supported drivers

The names below are the values for `imu.driver` and `mag.driver` in the
config. Drivers live in `src/drivers/` and are registered in `src/drivers.c`.
Links to the manufacturers' datasheets are collected in
[datasheets.md](datasheets.md).

<!-- BEGIN GENERATED: driver-table -->
| Driver name | Chip | Type | I²C address | GPIO interrupt | SPI | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `ism330dhcx` | ST ISM330DHCX | IMU | 0x6A–0x6B | BCM 17 · pin 11 | **yes** — mode 0, 10 MHz | Primary reference IMU. FIFO + hardware timestamp. ODR 12–6664 Hz. |
| `icm20948` | TDK ICM-20948 | IMU | 0x68–0x69 | BCM 17 · pin 11 | no — AKM compass behind the bypass | *Experimental.* Includes a built-in AK09916 mag via I²C master. No hardware timestamp. |
| `icm42688p` | TDK ICM-42688-P | IMU | 0x68–0x69 | BCM 17 · pin 11 | yes — mode 3, 24 MHz | *Experimental.* Best-in-class noise floor. FIFO + hardware timestamp. ODR 12–32000 Hz — **16000 and 32000 will not run on a Pi**, see below. |
| `lsm6dso` | ST LSM6DSO | IMU | 0x6A–0x6B | BCM 17 · pin 11 | yes — mode 0, 10 MHz | *Experimental.* Near-clone of ISM330DHCX. ODR 12–6664 Hz. |
| `lsm6dsox` | ST LSM6DSOX | IMU | 0x6A–0x6B | BCM 17 · pin 11 | yes — mode 0, 10 MHz | *Experimental.* LSM6DSO with ML core; same driver. |
| `mpu9250` | TDK MPU-9250 | IMU | 0x68–0x69 | BCM 17 · pin 11 | no — AKM compass behind the bypass | *Experimental.* Includes an AK8963 mag via I²C bypass. No hardware timestamp; 512-byte FIFO. NRND. |
| `mpu9255` | TDK MPU-9255 | IMU | 0x68–0x69 | BCM 17 · pin 11 | no — as `mpu9250` | *Experimental.* MPU-9250 with a different `WHO_AM_I`; same driver. |
| `mmc5983ma` | MEMSIC MMC5983MA | Magnetometer | 0x30 | BCM 27 · pin 13 | **yes** — mode 0, 10 MHz | Primary reference mag. 18-bit, SET/RESET coil. Do not set the IMU spi_speed_hz below 2.5 MHz while this part shares the controller — it stops measuring. |
| `ak09916` | AKM AK09916 | Magnetometer | 0x0C | none (polling) | no — part has no SPI port | *Experimental.* Used via the ICM-20948 I²C bypass; no external INT pin. |
| `ak8963` | AKM AK8963 | Magnetometer | 0x0C | none (polling) | no — part has no SPI port | *Experimental.* The MPU-9250/9255 compass, via I²C bypass. Applies the factory fuse-ROM sensitivity correction. Not the same part as AK09916. |
| `lis3mdl` | ST LIS3MDL | Magnetometer | 0x1C–0x1E | BCM 27 · pin 13 | yes — mode 3, 10 MHz | *Experimental.* Popular standalone mag. ±4 G fixed. ODR 1–155 Hz; the part's 300/560/1000 Hz modes need a lower-performance setting and are not offered, see below. |
| `lis2mdl` | ST LIS2MDL | Magnetometer | 0x1E | BCM 27 · pin 13 | no — 4-wire costs data-ready | *Experimental.* LIS3MDL successor. Fixed ±50 G. |
| `rm3100` | PNI RM3100 | Magnetometer | 0x20–0x23 | BCM 27 · pin 13 | yes — mode 3, 1 MHz | *Experimental.* Magneto-inductive, not AMR: no SET/RESET coil, so `set_period_s` does nothing here. ODR 1–600 Hz, but the top two rungs cost resolution — the cycle count sets both gain and rate ceiling, and the driver drops it from 200 to 100 above 150 Hz and to 50 above 300 Hz. Three separate coils plus an ASIC, so the axis assignment is your wiring: the driver assumes the manual's NED layout. |
| `sim` | — | IMU + Magnetometer | — | none | n/a | Software simulation of a small boat under way. No hardware. Set `int_gpio = 0` on both. |
<!-- END GENERATED: driver-table -->

The **SPI** column is what `[imu] bus` / `[mag] bus` will accept. Selecting
`bus = "spi"` on a driver marked "no" is refused at startup by name, rather
than tried and mis-framed. Only `ism330dhcx` and `mmc5983ma` have been
exercised on both transports against a mock device; the rest are
`experimental` on either bus.

The "no" rows each have a specific cause, and none of them is simply missing
work:

- **`ak09916`, `ak8963`** — the parts have no SPI port at all.
- **`icm20948`, `mpu9250`, `mpu9255`** — the AKM compass die inside them hangs
  off the host chip's *auxiliary* I²C bus and is reached through the I²C
  bypass, which only an I²C host can use. A SPI host would need the
  aux-I²C-master path (`I2C_SLV0_*` + `EXT_SENS_DATA_*` shadow reads), which
  imud does not implement, so a 9-axis board of that family runs on I²C.
- **`lis2mdl`** — its SPI defaults to *three* wires, and switching to 4-wire
  means writing `CFG_REG_C` bit 2, which the datasheet says disables the
  interrupt and data-ready signalling. imud drives this part from its
  data-ready line, so 4-wire would cost the interrupt and 3-wire needs
  half-duplex support the bus layer does not have.

### Sample rates, and which ones your host can actually run

Each driver advertises exactly the rates its part can be programmed to. Ask
for something else and imud rounds **up** to the next one the chip supports,
tunes the filter for that rate, and logs `requested, N Hz actual` at startup.

Two parts advertise rates a Raspberry Pi will not survive:

- **`icm42688p` at 16000 and 32000 Hz.** The silicon does them (low-noise
  mode, which is how imud configures it), and imud is not a Pi-only daemon —
  on a host with the headroom they are usable. On a Pi they are not. At
  32 kHz the 256-sample IMU ring fills in 8 ms and the fusion thread is asked
  for 32000 MEKF predictions a second; the realistic outcome is FIFO overflow
  rather than data. Treat them as available for larger hosts, not as a
  setting to try because it is the biggest number.
- **`ism330dhcx` / `lsm6dso` at 6664 Hz.** Comfortable on a Pi 4 or 5, and the
  rate the SPI transport exists to reach — I²C at 400 kHz cannot carry a
  6664 Hz FIFO drain.

For sea-state and heave work none of this is needed: those estimators track
0.05–1 Hz wave physics, and the sweep tests measure their accuracy as
essentially flat from 100 Hz to 16 kHz. High rates buy vibration rejection
and attitude bandwidth, not wave accuracy. The default of 833 Hz is the
right answer for almost every installation.

Two drivers deliberately advertise less than their part can do:

- **`lis3mdl`** stops at 155 Hz. Its 300, 560 and 1000 Hz modes require
  dropping the XY/Z operating mode below ultra-high-performance, which costs
  magnetometer noise performance — a bad trade when the MEKF's magnetometer
  update only needs heading, and 155 Hz is already far above what that needs.
  (Its 0.625 Hz bottom rung is also omitted: the rate table is integer Hz.)
- **`icm20948` and `mpu925x`** derive their rate from an integer divider
  (`1125/(1+div)` and `1000/(1+div)`), so they reach rates no table could
  list. Their advertised entries are a readable sample, not the whole set;
  both implement the `actual_odr_hz` hook and report what they really
  programmed.

GPIO pins shown are the defaults (`imu.int_gpio = 17`, `mag.int_gpio = 27`).
Set `int_gpio = 0` to disable the interrupt and use a polling timer — useful
when the pin is wired differently or unavailable.

**Experimental** drivers have their register maps verified against the
datasheet but have **not** been validated on physical hardware. imud prints a
warning at startup when an experimental driver is selected. To add a new
driver, see [§11 Writing a driver](#11-writing-a-driver).

If you have one of these parts, you can clear its experimental flag for
everyone: run `imud-imutest` (`man 8 imud-imutest`, shipped in `imud-utils`)
against it and open an issue with the report it writes.

### Fitting an MPU-9250 or MPU-9255

Wiring, the `config.txt` I²C settings, and a ready-made `imud.conf` follow.

Three things about this part differ from imud's shipped defaults, and all
three are worth knowing before you wire one up.

- **The FIFO is smaller than the default watermark.** 512 bytes ÷ 12 bytes per
  accel+gyro sample-set is about 42 sets, and `imu.fifo_wm` defaults to `64`,
  which this chip cannot reach. The driver logs the mismatch at startup and
  drains whatever is pending instead. Roughly 42 ms of headroom at 1 kHz also
  means a stalled reader raises `FLAG_FIFO_OVERFLOW` far sooner than on the
  ISM330DHCX's much larger FIFO.
- **The default ODR is not on the grid.** Output rate is
  `1000 / (1 + SMPLRT_DIV)`, giving 1000 / 500 / 333 / 250 / 200 / 125 /
  100 Hz. imud's default of 833 Hz is not reachable and rounds to 1000 Hz,
  which also costs more CPU on a Pi. Set `imu.odr_hz` explicitly.
- **Do not copy the datasheet noise figure into `mekf_accel_noise`.** The
  MPU-9250's accelerometer noise density (~300 µg/√Hz) converts naively to
  about `0.0035`, which lands *inside* the divergence region documented in
  [math.md §4.7](math.md) and `man 5 imud.conf`. `mekf_accel_noise` is a tuned
  filter parameter, not a per-chip datasheet transcription. A noisier IMU is
  not a reason to change it.

The magnetometer is a separate driver: set `mag.driver = "ak8963"` and
`mag.i2c_addr = 0x0C`. It is reached through the MPU's I²C bypass, which the
IMU driver opens during init, so the IMU must be configured too — imud always
brings the IMU up first.

Boards sold as MPU-9250 are very often relabelled **MPU-6500s**, which have no
magnetometer at all. `probe()` rejects those by name rather than letting the
failure surface later as an unexplained I²C error from the mag driver.

### Running a sensor on SPI

SPI moves the same registers over a faster link. That matters in two places:
the high sample rates a part advertises stop being limited by the bus, and the
FIFO drain — which `imud-mon` reports separately from the daemon's own
pipeline latency — gets shorter. An ISM330DHCX FIFO word costs roughly 180 µs
of bus time at 400 kHz I²C and under 10 µs at 10 MHz SPI.

Enable the bus (`dtparam=spi=on`, or `raspi-config` → Interface Options →
SPI), then wire each sensor to its own chip select and name the node:

```ini
[imu]
driver       = "ism330dhcx"
bus          = "spi"
spi_dev      = "/dev/spidev0.0"   # CE0, header pin 24
int_gpio     = 17                 # unchanged — the interrupt is a separate wire

[mag]
driver       = "mmc5983ma"
bus          = "spi"
spi_dev      = "/dev/spidev0.1"   # CE1, header pin 26
int_gpio     = 27
```

Points worth knowing:

- **The chip select does the addressing.** `i2c_addr` is unused on SPI, and
  the two sensors need *separate* chip selects — they cannot share one node.
- **The sections are independent.** A SPI IMU with an I²C compass is a legal
  and sometimes necessary rig: the AKM compasses have no SPI port, so a
  9-axis ICM-20948 or MPU-925x board stays on I²C entirely.
- **`spi_speed_hz = 0` means the part's datasheet maximum**, which is usually
  what you want. A higher request is clamped and the daemon logs the rate it
  really programmed.
- **The interrupt line is unchanged.** `int_gpio` is a separate wire either
  way; only the data path moves.
- **Wiring is not modelled by any test.** The tests prove both transports
  produce identical register traffic, which is a different claim from "the
  board is wired right". Run `imud-imutest --all` after changing transport —
  its report should match the one you get on I²C.

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
| `accel` | Bench 6-position calibration; follow the on-screen prompts to orient each face in turn (see below). Do this before final mounting. |
| `mag` | In-situ magnetometer swing — drive the vessel slowly through at least two full 360° circles, then press Ctrl-C. Guided: a live bar shows heading-circle coverage (`#` covered, `.` needed, `o` = where you're pointing), running coverage %, and live radius/RMS; a bell + "FULL CIRCLE" message tells you when coverage is complete. Must be done after final mounting, with the engine and typical electronics running. |
| `characterize` | Offline Allan-variance noise analysis of a **stationary** capture (`--from FILE`, recorded with `[capture]` enabled — overnight for a trustworthy bias-instability floor). Writes per-axis gyro/accel noise characteristics to cal.json's `noise` section as **informational** sensor characterization — the filter always keeps its tuned `mekf_*`; these numbers never feed it. Never touches the sensors. |
| `fit-temp` | Offline linear gyro-bias/temperature fit from a warm-up capture (`--from FILE`, cold boot → warm, several °C of span). The daemon subtracts the fitted term from the gyro before fusion. Never touches the sensors. |
| `fit-ra` | Offline check of the filter's accelerometer measurement model against a **rough-water** capture (`--from FILE`). Reports the gravity-direction residual, its correlation time, the innovation-distance distribution against the gates, and the mean NIS — the same statistic the daemon publishes live as `nis_accel`. **Writes nothing**: the value it comments on is filter tuning in `imud.conf`, not a sensor property. Needs a mag calibration in cal.json. Never touches the sensors. |

### The six accelerometer positions

Remember that the board frame has **Z pointing down** (§11), so the axis
pointing **up** reads +g and the axis pointing **down** reads −g. Flat and
component-side up therefore reads `[0, 0, −9.807]`, not `+9.807`.

| # | Position | Board axis | Expected reading |
|---|---|---|---|
| 1 | Flat, component side up | +Z points down | `[0, 0, −9.807]` |
| 2 | Flat, upside down | +Z points up | `[0, 0, +9.807]` |
| 3 | Nose down (bow edge on the bench) | +X points down | `[−9.807, 0, 0]` |
| 4 | Nose up (stern edge on the bench) | +X points up | `[+9.807, 0, 0]` |
| 5 | Starboard side down (right edge down) | +Y points down | `[0, −9.807, 0]` |
| 6 | Port side down (left edge down) | +Y points up | `[0, +9.807, 0]` |

Each prompt states the expected vector before you take the reading, and prints
what it actually measured afterwards, so a misplaced board shows up
immediately. If a reading is geometrically impossible — gravity landing on an
axis other than the one asked for, or a pair taken the wrong way round — the
run names the offending position and exits **without writing anything**. A
calibration fitted from a misplaced board is worse than no calibration, so it
is not offered for saving.

These are the same six orientations `imud-imutest` uses for its guided
axis-sign check, so the two tools can be cross-checked against each other.

The magnetometer calibration fits a hard-iron offset and a 2D soft-iron
correction (including the cross term, so a distortion ellipse whose axes are
rotated relative to the sensor is corrected — a per-axis scale cannot do
this). It reports the field radius, a fit residual, and swing coverage; a
residual under ~1 µT and coverage above ~75% indicate a good fit. Afterward,
watch the wire's `mag_residual` / `mag_anomaly` compass-health metrics (v14,
also on the InfluxDB and Prometheus bridges): a rising residual means the
calibration is degrading and the swing should be repeated.

### Checking the filter against real water

Two knobs decide how much the filter trusts the accelerometer, and they are
not interchangeable:

- **`mekf_accel_noise`** describes the *sensor's* white noise. It is the knob
  most likely to be "improved" into a worse configuration: the default is a
  sharp local optimum for the marine (yaw-only) configuration, and values a
  little above it are substantially *worse*, not a little worse — see
  `man 5 imud.conf`.
- **`mekf_wave_accel`** (with `mekf_wave_accel_tau_s`) describes the *seaway*.
  This is the one to reach for when the boat is in rough water. It models the
  wave-orbital disturbance as a correlated process in the filter state, which
  is what lets the filter stop over-counting 833 nearly identical samples a
  second as independent evidence about gravity.

If you run 3-D vector fusion (`mag_yaw_only = false`) there is a third, and it
is about the *magnetic reference* rather than either sensor:
`mekf_mag_dip_sigma_deg`. In that mode the field's dip constrains roll and
pitch, and the dip of the reference is fixed once at alignment — so a tilt
error at startup becomes a permanent roll/pitch bias. The best fix is a
position source, which supplies WMM field invariants and removes the error
outright; failing that, this key tells the filter how much the dip channel is
worth. `align_window_sec` matters here too: it sets how much of the wave cycle
is averaged out before the reference is fixed.

Before changing either, measure:

1. **Record a capture in the conditions you care about.** Enable `[capture]`
   and run for at least 20 minutes in the seaway you want the filter tuned
   for. The capture stores raw pre-calibration samples, so one recording can
   be re-analysed against any tuning.
2. **Analyse it offline** — this touches nothing and can run on any machine:

   ```sh
   imud-cal fit-ra --from /var/lib/imud/<capture>.imucap --config /etc/imud/imud.conf
   ```

3. **Read the mean NIS.** With the wave state enabled (the default) this
   **should sit near 1**: the correlated part of the residual is modelled
   rather than left for a white R to absorb. Well above 1 means the seaway is
   rougher than `mekf_wave_accel` allows for; well below 1 means calmer, which
   is safe but conservative. (With the wave state *disabled*, 10–30 is normal
   and is not by itself a reason to change anything — that reading is the
   problem the wave state exists to solve.)
4. **Read the wave-state block.** `fit-ra` replays the capture a second time
   with the state forced off, so it reports the disturbance the filter is
   actually up against — σ in m/s² and its correlation time — and suggests
   values for both knobs. Round them up.
5. **Check the residual *mean*, not just its spread.** A large per-axis mean
   points at a wrong mount rotation or a stale accelerometer calibration —
   neither of which any noise setting can fix.

Live, the same statistic is on the wire as `nis_accel` (and on the Prometheus
and InfluxDB bridges as `imud_nis_accel_ratio` / `nis_accel`), alongside
`innov_weight` and `innov_reject`, which say how hard the innovation gate is
having to work. All three of `mekf_accel_noise`, `mekf_wave_accel` and
`mekf_wave_accel_tau_s` are re-read on `SIGHUP`, so an A/B is: edit the value,
`systemctl reload imud`, and watch the metric settle for a couple of minutes.

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

A fixed 276-byte little-endian packet (wire v17) at up to 500 Hz:
calibrated and raw accel/gyro/mag, quaternion, Euler angles, heading,
rate-of-turn, heave, temperature, the 3×3 attitude covariance, timestamps
(wall + TAI + chip), declination, and an IEEE-802.3 CRC32. Every packet is
self-describing (magic + version + CRC), so consumers can validate each one
independently. See [spec.md §8](../spec.md) for the exact layout and the
consumer libraries in [§9](#9-consumer-libraries).

**Near-vertical pitch — read the quaternion.** The Euler fields (`pitch`,
`roll`, `yaw`, `heading_deg`, `rate_of_turn`) are derived from the quaternion
as ZYX intrinsic angles, which are singular at pitch = ±90°: approaching
vertical, roll and yaw stop being separately determined and swing wildly, and
at ±90° they are undefined. That is a property of any three-angle
representation, not a filter limitation. The quaternion is never singular.
Surface vessels never go near this and can use the Euler fields freely; a
gimbal, tilting camera rig, or anything that can point straight up or down
must read `quat_*` instead. Same for the NMEA sentences above, which are Euler
by definition.

### Local subscription stream — AF_UNIX socket (default on) + TCP listener (default off)

> This is the one output a stock daemon enables — the bridges and libimud
> consumers read it.

The same 276-byte binary packets over a `SOCK_STREAM` socket at
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

**If that thread cannot be started, imud exits 1** rather than running without
it — a daemon reporting active while producing nothing at all is worse than one
that is plainly down, and `imud.service` is `Restart=on-failure` with
`RestartSec=3`. The optional outputs (`[nmea]`, `[highrate]`, and the position
thread) are the other way round: they warn and the daemon carries on. In that
case whatever they had already bound is closed and unlinked, so a client gets
`ECONNREFUSED` immediately instead of connecting successfully to a listener
nobody is serving and then waiting forever. `READY=1` is sent to systemd only
once the threads are actually running, not when the sockets are bound.

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

1. Enable imud's stream + a bridge output: `imud.conf` `[stream] enabled = true`;
   `imud-signalk.conf` `udp_enabled = true` (the daemon is enabled by default),
   destination = the Signal K host, port 10113 (default).
   `sudo systemctl enable --now imud-signalk`.
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

### imud-imutest

Validates a sensor driver against real hardware and writes a Markdown report
suitable for attaching to an issue. Ships in `imud-utils`; full documentation
is in `man 8 imud-imutest` and under `/usr/share/doc/imud-utils/`.

```sh
sudo systemctl stop imud       # both would drain the same FIFO
imud-imutest --imu-driver icm20948 --mag-driver ak09916 --mag-addr 0x0C --all
sudo systemctl start imud
```

It runs a fully automatic pass (probe, reset timing, register readback,
measured ODR, FIFO depth and overflow, `seq` continuity, the error-return
contract, noise, gravity, temperature, `chip_ts`, interrupt edges, full-scale
sweep) and then three guided phases that prompt the operator to place and turn
the board — the only way to prove the chip-to-board axis remap is right.

This is how an experimental driver gets its flag cleared: run it, then open an
issue with the report.

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
`/usr/local/share/imud`. See `man 3 libimud` for the API, `imud_data_t`
fields and the ABI contract; the `libimud` package ships fuller usage examples
under `/usr/share/doc/libimud/`.

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

Each bridge separates the daemon-level `enabled` from its per-output enables. A
stock installed config ships the daemon `enabled = true` but **every output off**
(`udp_enabled` / `tcp_enabled` for signalk, `udp_enabled` / `http_enabled` for
influxdb, `broker_enabled` for mqtt, `http_enabled` for prometheus,
`udp_enabled` / `serial_enabled` / `tcp_enabled` for mavlink) — so the service
starts and stays healthy under systemd while emitting nothing until you turn on
the output(s) you want. Setting the daemon `enabled = false` makes it exit
cleanly (systemd stops it; no restart loop). (imud-influxdb's old
`transport = "udp"|"http"` key is deprecated but still honored, mapped to the
matching output enable with a warning.)

---

## 10. Troubleshooting

| Symptom | Check |
|---|---|
| Sensors not detected (I²C) | I²C enabled? `i2cdetect -y 1` should list `0x6b` (ISM330DHCX) and `0x30` (MMC5983MA). Check wiring and the `i2c_addr` values. |
| Sensors not detected (SPI) | SPI enabled (`dtparam=spi=on`) and `/dev/spidev0.*` present? Check `spi_dev` names the right chip select, that each sensor has its **own** CE line, and that the driver supports SPI at all — see [Supported drivers](#5-supported-drivers). A `probe()` failure here is usually MISO/MOSI swapped or the wrong CE. |
| `WHO_AM_I` mismatch at startup | Wrong `imu.driver`/`mag.driver`, or a wrong `i2c_addr` (e.g. SA0 jumper → 0x6A vs 0x6B). On SPI the address comes from the chip select, so suspect `spi_dev` instead. |
| GPIO open/permission errors | Run `gpiodetect` and set `gpio_chip` to the chip it lists for the header pins — the number comes from probe order, so it is not stable across kernels (Pi 5 moved from `gpiochip4` to `gpiochip0` in mid-2024). Run as the `imud` service user or a member of the `gpio`/`i2c`/`spi` groups. |
| `/dev/gpiochip4` vanished after an upgrade, on a Pi 5 | Pi OS aliases the header chip to its old name with a udev rule guarded by `test ! -c /dev/gpiochip4`. That guard oscillates: any `udevadm trigger --subsystem-match=gpio` deletes the symlink when it exists and recreates it when it does not. Re-trigger, or reboot, to get it back — and prefer `gpiochip0`, which is the real device. imud stopped triggering the `gpio` subsystem in 1.9.0 for exactly this reason. |
| Bus or GPIO open fails only under systemd, but works when run by hand | The unit's `DevicePolicy=closed` allows a fixed list of nodes. The shipped list covers `/dev/i2c-1`, `/dev/i2c-3`, `/dev/spidev0.0`, `/dev/spidev0.1`, `/dev/gpiochip0` and `/dev/gpiochip4`; anything else — SPI1..6, for instance — needs its own `DeviceAllow=` line. `systemd-analyze verify` will not catch this — it only bites at device-open time. |
| Filter reports `R` in `imud-mon`, or `imud_state_reset` is 1 | The MEKF found a non-finite value in its own state and reset itself; it re-aligns automatically and the flag clears when it re-converges. Repeated resets are a bug — capture the log line (`[fusion] non-finite filter state`) and the `.imucap` that produced it. |
| Fusion never converges | The flag needs every attitude axis to have a measurement behind it. Roll and pitch come from gravity; heading comes from the magnetometer. If the magnetometer is not being fused — `mag_fuse_uncal = false` with no `cal.json`, no magnetometer configured, or the withdrawal gate latched under gross iron — the heading axis is unobservable, its covariance grows without bound, and the flag correctly never sets. `imud-status` says which under Attitude. With the magnetometer fused, suspect a strong local magnetic disturbance: run `imud-cal mag` and check the fit residual. |
| Heading drifts steadily while pitch and roll stay put | The magnetometer is not being fused, so heading is dead-reckoned from the gyro and drifts without bound — several degrees per minute on a static bench, while pitch and roll hold to a tenth of a degree. The default `mag_fuse_uncal = true` fuses an uncalibrated field heading-only, which bounds the drift, so reaching this state means fusion is off: `mag_fuse_uncal = false`, no magnetometer configured, or the withdrawal gate latched. `imud-status` distinguishes the three cases under Attitude — DEAD RECKONED, UNCALIBRATED, or neither. The fix is `imud-cal mag`. |
| Heading stopped updating altogether | The magnetometer has stopped delivering. The daemon warns once per outage (`[mag_reader] no magnetometer sample for …`) and logs the recovery. On a shared SPI controller, check `[imu] spi_speed_hz` is not below 2.5 MHz. |
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

Both types are handed an `imud_bus_t` — a transport handle — and reach the
chip through the helpers in `src/drivers/bus_io.h`:

```c
int bus_reg_read  (const imud_bus_t *bus, uint8_t reg, uint8_t *val);
int bus_reg_write (const imud_bus_t *bus, uint8_t reg, uint8_t val);
int bus_burst_read(const imud_bus_t *bus, uint8_t reg, uint8_t *buf, uint16_t len);
```

The handle carries the transport, the backend's token, and whatever addresses
the part — the 7-bit slave address on I²C, the resolved clock and framing bits
on SPI. **Write the register logic once; it runs on either bus.** The helpers
frame the transfer and hand it to the host's backend
(`include/bus_backend.h`), so a driver is written against the framing and not
against any one kernel's ioctls.

To offer SPI, fill in `bus_caps` from the datasheet:

```c
.bus_caps = { .spi_capable = true, .spi_mode = 3,
              .spi_max_hz = 10000000, .spi_inc_mask = 0 },
```

Leave it zeroed and `bus = "spi"` is refused at startup by name — the right
outcome for a part with no SPI port, and better than silently mis-framing.
`spi_inc_mask` is the one field that is easy to get wrong: some parts step the
address on a multi-byte read automatically (the ST 6-axis parts do it from
`CTRL3_C`'s `IF_INC`), and some need an explicit bit in the command byte
(LIS3MDL's MS at `0x40`). Check the SPI section of the datasheet, not the I²C
one — the auto-increment bit is frequently in a different place on each.

If a part cannot do SPI, say why in the ops struct rather than leaving the
field zeroed without comment; `src/drivers/lis2mdl.c` is the worked example
(its 4-wire mode disables the data-ready line the driver depends on).

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

### The register helpers

All drivers share the register helpers in `src/drivers/bus_io.h` — include it
and call them; do not roll your own:

```c
#include "bus_io.h"

bus_burst_read(bus, reg, buf, len);   /* combined write-then-read */
bus_reg_write (bus, reg, val);
bus_reg_read  (bus, reg, &val);       /* burst_read of length 1 */
reg_s16le(p);  reg_s16be(p);          /* int16 from a register byte pair */
reg_s24be(p);                         /* int24, sign-extended (RM3100) */
```

They take the `imud_bus_t *` the daemon handed your op, not a descriptor and
an address: the handle carries the transport, the slave address and the SPI
framing, which is what lets one body of register logic run on either bus.
`bus_burst_read` dispatches to a combined write-then-read in a single I²C
transaction (no repeated-start gap, ~40 µs saved against two transactions at
400 kHz), or to a command byte plus data inside one `SPI_IOC_MESSAGE` so chip
select stays asserted across both.

If your chip needs a sub-address modifier and only on one transport, name the
transport rather than ORing the bit unconditionally — `src/drivers/lis3mdl.c`
is the worked example:

```c
uint8_t out_reg = REG_OUT_X_L;
if (bus->kind == BUS_I2C) out_reg |= 0x80;   /* I²C auto-increment */
```

The SPI half of that is not a call-site decision at all: it rides in the
handle as `spi_inc_mask`, from the driver's `bus_caps`. Beware datasheets that
tabulate a "read address" — on some parts (the RM3100) that column is the SPI
command byte with the read bit already set, and using it as an I²C
sub-address addresses a register that does not exist.

The helpers are `static inline`, so each driver still issues its own single
transfer, through the backend behind `include/bus_backend.h`. The mock bus
(`test/bus_mock.c`) is one of its three implementations, beside
`src/bus_linux.c` and `src/bus_null.c`. Keep any new I/O on this path; a raw
`ioctl()`, an SMBus call, `I2C_SLAVE` or `read()`/`write()` would bypass both
the mock and every non-Linux host.

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

#### `probe(bus)` → 0 or -1

Read the WHO_AM_I (or equivalent) register and verify it against the datasheet
value. Log a clear error with the received and expected values on mismatch.

```c
static int myimu_probe(const imud_bus_t *bus)
{
    uint8_t who;
    if (bus_reg_read(bus, REG_WHO_AM_I, &who) < 0) {
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

#### `reset(bus)` → 0 or -1

Trigger a software reset and wait for the bit to self-clear. Always add the
chip's specified power-on time after the reset bit clears — skipping this
causes init failures on slower hardware.

Not every part has a reset bit. The RM3100 has none at all, so its `reset()`
restores the power-on register values by hand; if yours is like that, say so
in the function's comment rather than leaving the absence to look like an
oversight.

```c
static int myimu_reset(const imud_bus_t *bus)
{
    if (bus_reg_write(bus, REG_CTRL, 0x01) < 0) return -1;   /* SW_RESET */
    for (int i = 0; i < 50; i++) {
        usleep(1000);
        uint8_t val;
        if (bus_reg_read(bus, REG_CTRL, &val) < 0) return -1;
        if (!(val & 0x01)) goto done;
    }
    LOG_W("myimu: SW_RESET did not clear after 50 ms\n");
    return -1;
done:
    usleep(20000);   /* chip startup time from datasheet */
    return 0;
}
```

#### `init(bus, cfg)` → 0 or -1

Configure ODR, full-scale range, FIFO mode (if applicable), and interrupt
routing. Save the resulting sensitivity values to the static `s` struct.
`cfg->odr_hz` is already resolved: the daemon asks your driver what rate it
will really program (see [`actual_odr_hz`](#the-actual_odr_hz-hook) below) and
passes that value back here, so your own rounding is a no-op on it. Round the
same way `actual_odr_hz` reports — by default, up to the next supported rate.

```c
static int myimu_init(const imud_bus_t *bus, const imu_cfg_t *cfg)
{
    float accel_scale, gyro_scale;
    uint8_t odr  = odr_encode(cfg->odr_hz);
    uint8_t xlfs = xl_fs_encode(cfg->accel_g,  &accel_scale);
    uint8_t gyfs = gy_fs_encode(cfg->gyro_dps, &gyro_scale);

    if (bus_reg_write(bus, REG_ACCEL_CFG, (odr << 4) | xlfs) < 0) return -1;
    if (bus_reg_write(bus, REG_GYRO_CFG,  (odr << 4) | gyfs) < 0) return -1;
    /* ... FIFO, interrupt config ... */

    s.accel_scale = accel_scale;
    s.gyro_scale  = gyro_scale;
    s.seq         = 0;
    return 0;
}
```

#### `read(bus, buf, max, *n_out)` → 0, 1, or -1

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
that increments at a **fixed rate independent of the I²C clock**.

`ts_tick_ns` is the datasheet's nominal period, and it is fine for it to be
only nominal: the daemon measures the counter's real period against the host's
`CLOCK_MONOTONIC` across consecutive 60 s anchors and uses the measurement in
place of your constant (see `ts_anchor_t` in `include/imu_math.h`). That is not
a refinement — a measured ISM330DHCX ran 4.08% fast, so trusting the constant
put sample timestamps seconds out and scaled every integrated rotation by the
same 4%. What the driver must get right is the *tick*, not its exact duration:
the counter has to advance monotonically at a rate that does not depend on bus
traffic or ODR. Get the period roughly right (within 10%, or the measurement is
rejected as implausible) and the daemon handles the rest.

If your part can say what *its own* timer period is, implement the optional
`ts_tick_ns_actual` hook and hand it back. The daemon calls it once after
`init()`, with the bus open, and uses the answer everywhere it would have used
your constant. Return 0 — on a failed read, or when there is nothing to ask —
and the declared value stands. This does not replace the runtime measurement
above; it fixes the *first minute*, before two anchors exist. On a part several
percent off nominal that window is not cosmetic: the extrapolated sample time
drifts far enough from the host clock that the sample-latency histogram stops
recording. The ST 6-axis parts carry the answer in `INTERNAL_FREQ_FINE` (0x63),
and `src/drivers/st_freq_fine.h` does the arithmetic for all of them.

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

#### `self_test` (optional, diagnostic)

Most 6-axis parts can apply a known electrostatic force to their own proof
masses and are specified for how far the output must move when they do.
Implement `self_test(bus, &out)` and `imud-imutest` will run it, grading the
response per axis against the window you report alongside it.

It is worth implementing because it answers a question nothing else here can.
At rest a working gyroscope and one whose sensing element is dead both read
about zero, with the same noise floor and the same sequence and timestamp
behaviour, so every other passive check passes both. Without a self-test the
only evidence that an element responds at all comes from the guided rotation
phase — which needs a person at the bench, and so cannot run unattended or in
`--passive`.

The contract is deliberately loose about state and strict about the
measurement:

- **The daemon never calls it.** It is a diagnostic, like `degauss` below.
- Report the response and the datasheet's window in **the datasheet's own
  units** — mg and dps — not the SI units the sample path carries. These
  numbers are only ever compared against a printed table and shown to whoever
  is reading the report. Leave a window at `0`/`0` and the check records the
  response without grading it.
- Program whatever ranges the limits are quoted for. They usually are
  per-range: the ISM330DHCX's accelerometer window is full-scale independent
  but its gyroscope one is not, so the driver selects ±2000 dps rather than
  grading against whatever the operator configured.
- **Wait for fresh samples rather than sleeping.** A blind sleep will average
  one register image several times over and report a confident number from a
  part that is producing nothing — the exact failure the check exists to
  catch. Gate each read on the part's data-ready status and fail the call if
  it never comes.
- You may leave the part configured however the measurement needed, but
  **self-test itself must be off on every exit path**, including the failures.
  The caller re-runs `reset()` and `init()` afterwards and checks that it
  worked.

Return `-1` if the measurement could not be made at all — a bus error, or no
samples to average. There is no third return here: unlike `read()`, nothing is
"not yet", because the implementation waits for what it needs.

### Writing a magnetometer driver (`mag_ops_t`)

#### `probe`, `reset`, `init`

Same pattern as the IMU driver. `init` configures the ODR and enables
continuous measurement. If the chip has an interrupt pin, enable it during
`init` so the mag reader thread can wake on a GPIO edge rather than polling.

#### `read(bus, *out)` → 0, 1, or -1

| Return | Meaning |
|---|---|
| `0` | Sample written to `*out`, `out->valid = true` |
| `1` | Measurement not complete yet (DRDY not asserted) |
| `-1` | I²C error |

#### Check whether your part's status bit survives the interrupt

Gating `read()` on a "data ready" status bit is the obvious implementation and
it is right for most parts. On some it is not, and the failure is quiet: the
driver works perfectly when polled and delivers a fraction of the sample rate
when the daemon waits on the interrupt — which is the only mode the daemon uses.

It goes wrong when **DRDY is a latched interrupt whose acknowledge write also
clears the status bit**. Acknowledging is what re-arms the edge, so it cannot be
skipped; and once it has also taken the status bit away, a reader that blocks on
the edge — and therefore stops touching the bus — may never see the bit come
back. Every wake is then spent on a read that reports "no data", and since the
interrupt is latched there is no second edge to recover with: the reader waits
out its timeout for every single sample.

Ask the datasheet two questions before you gate:

| Part | How DRDY clears | Safe to gate? |
|---|---|---|
| LIS3MDL | `ZYXDA` is a plain status flag; there is no acknowledge write | yes |
| LIS2MDL | the DRDY pin *is* the `Zyxda` bit, driven straight out | yes |
| RM3100 | the pin "is set LOW when the Measurement Result registers are read" | yes |
| **MMC5983MA** | **write 1 to `Meas_M_Done` — which is also the gate bit** | **no** |

Only the MMC5983MA among the supported parts answers badly, and it was measured
doing so: 35 Hz delivered from a part converting at 105.5 Hz.

When a part is like this, set `int_driven` in `mag_cfg_t`. The daemon sets it
whenever it has an interrupt line for the mag, and `read()` should then trust the
edge instead of the status bit, still perform the acknowledge write, and reject a
sample whose output registers have not changed — that last part is what keeps a
broken interrupt line from feeding duplicates to the filter. Polling callers
(`imud-cal`, `imud-imutest`) leave it false and keep the gate, which is correct,
because a poller has no edge to trust.

`imud-imutest`'s `mag.drdy.rate` check exists to catch this: it measures the mag
over its interrupt line and compares against the polled `mag.rate`. A driver that
gates when it should not shows up as a large gap between the two.

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

Write the whole control register, not just the pulse bit. On parts where the
pulse bits share a register with persistent mode bits — the MMC5983MA's CTRL0
carries `INT_en` beside `Set` and `Reset` — writing the pulse alone clears the
mode bits `init()` set, and the symptom is silent: the interrupt line simply
stops and the reader falls back to polling.

#### `degauss` (optional, diagnostic)

If the coil can be driven both ways, also implement
`degauss(bus, MAG_DEGAUSS_SET | MAG_DEGAUSS_RESET)` — the same pulse, with the
direction chosen by the caller. Leave it `NULL` otherwise; nothing requires it,
and `set_reset` stays the production path either way.

It exists because SET and RESET magnetise the film in opposite directions, so
the field term of a reading flips sign between them while the bridge's own
offset does not:

```
vS = +S*B + offset      (after SET)
vR = -S*B + offset      (after RESET)
  field  = (vS - vR) / 2
  offset = (vS + vR) / 2
```

`imud-imutest --passive` uses that to separate a genuine external field from a
bridge offset, which is otherwise indistinguishable without a second transport
or a known reference field. Implement it in terms of one shared helper with
`set_reset`, so the two cannot drift apart, and leave the part in the SET state
when done.

#### `has_interrupt`

Set `has_interrupt = true` if the chip asserts an external interrupt pin on
measurement complete — the mag reader thread requests a GPIO edge on
`[mag] int_gpio` and calls `read()` on each rising edge. Set `false` if there
is no interrupt pin (e.g. AK09916 in bypass mode); the reader falls back to a
rate-sized poll — one sample period plus `int_grace`, so it tracks the
configured ODR rather than a fixed interval.

### The `supported_*` tables

Zero-terminated ascending integer arrays telling the daemon which rates your
chip supports. `imu.c` resolves the operator's requested rate against these
before it reaches your `init()` — see the next section.

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

### The `actual_odr_hz` hook

```c
int (*actual_odr_hz)(int requested);
```

The rate your driver will really program for `requested`, in Hz. The daemon
calls this **before** `init()` and passes the answer back as `cfg->odr_hz`, so
one configured rate means one rate everywhere: the chip samples at it, the
MEKF's noise variances are sized for it, and `imud-imutest` measures against
it. Getting this wrong tunes the filter for a rate the hardware is not
running at.

**Leave it `NULL` if your chip has a fixed rate table** — which is the usual
case. The default rule is then the lowest entry in `supported_odr_hz` at or
above `requested`, clamped to the highest, and your `odr_encode()` chain must
round the same way (a `hz <= X` ladder over the same values does exactly
this).

**Implement it if your chip is divider-based**, where the reachable rates are
not the table:

```c
/* ODR = 1000 / (1 + SMPLRT_DIV): the DIVIDER is what rounds, so this part
 * reaches rates that are in no table — 137 Hz becomes 1000/7 = 142. */
static int myimu_actual_odr_hz(int requested)
{
    return odr_actual(smplrt_div_encode(requested));
}
```

Also implement it — as the identity — if your driver honours whatever rate it
is handed rather than snapping to a grid, as the `sim` driver does.

Resolve through `odr_actual_imu()` / `odr_actual_mag()` in `imu_math.h` rather
than calling the hook directly; that is where the `NULL` default lives.
`nearest_odr()` still exists but answers a different question — "which
advertised rate did the operator probably mean" — and must not be used to
decide tuning.

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
int_gpio = 0          # 0 disables GPIO — reader polls at the batch period

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
      datasheet startup time afterward — or, on a part with no reset bit,
      restores the power-on register values and says so in a comment.
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
- [ ] `odr_encode()` rounds **up** over exactly `supported_odr_hz[]` — or
      `actual_odr_hz` is implemented and reports what the chip really programs.
- [ ] Driver added to the `src/drivers.c` registry and the `Makefile`.
- [ ] Logs use `LOG_*`, not bare `fprintf`.
- [ ] Tested with `driver = "sim"` and `make test` passes.
- [ ] Tested on real hardware with `imud-status` confirming sensor activity.
- [ ] `imud-imutest --all` run against the part, with the report attached to
      the pull request or issue. This is what clears `experimental = true`.
