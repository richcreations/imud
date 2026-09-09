# imud — IMU Daemon

[![Latest release](https://img.shields.io/github/v/release/richcreations/imud?sort=semver)](https://github.com/richcreations/imud/releases)
[![Platform: Linux | macOS](https://img.shields.io/badge/platform-Linux%20%7C%20macOS-blue.svg)](docs/manual.md)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)  
[![CI](https://github.com/richcreations/imud/actions/workflows/ci.yml/badge.svg)](https://github.com/richcreations/imud/actions/workflows/ci.yml)
[![CodeQL](https://github.com/richcreations/imud/actions/workflows/codeql.yml/badge.svg)](https://github.com/richcreations/imud/actions/workflows/codeql.yml)
[![Fuzz (nightly)](https://github.com/richcreations/imud/actions/workflows/fuzz-nightly.yml/badge.svg)](https://github.com/richcreations/imud/actions/workflows/fuzz-nightly.yml)  
<!-- Scorecard badge: the canonical api.scorecard.dev/.../badge URL redirects to
     shields' ossf-scorecard route, which reads the diverged legacy host and has
     served a stale score since 2026-08-25.  Revert to the canonical URL once
     https://github.com/badges/shields/issues/12117 is fixed. -->
[![OpenSSF Scorecard](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fapi.scorecard.dev%2Fprojects%2Fgithub.com%2Frichcreations%2Fimud&query=%24.score&label=openssf%20scorecard&color=green)](https://scorecard.dev/viewer/?uri=github.com/richcreations/imud)
[![OpenSSF Best Practices](https://www.bestpractices.dev/projects/13917/badge)](https://www.bestpractices.dev/projects/13917)



[Project site](https://richcreations.github.io/imud/) ·
[apt repository](https://richcreations.github.io/imud/apt/)

**imud is a general-purpose IMU daemon — think of it as *gpsd for IMUs*.**
It owns the inertial sensor, does the hard real-time work once
(interrupt-driven sampling, calibration, sensor fusion, precise hardware
timestamps), and publishes a clean attitude/heading/motion estimate on
standard interfaces that any number of programs can read at the same time.

Instead of every application re-implementing sensor drivers and a Kalman filter,
you run one small daemon and consume its output. Like gpsd, it's meant to be
boring, always-on infrastructure: start it, forget it, and point your
software at the stream.

```
   IMU + magnetometer (I²C/SPI)                  consumers
            │                          ┌────────────────────────────┐
            ▼                          │  chartplotter / autopilot  │
   ┌─────────────────┐   NMEA 0183 ───▶│  ROS2 node                 │
   │      imud       │   binary UDP ──▶│  vision / stabilization    │
   │  drivers·MEKF·  │   AF_UNIX    ──▶│  gimbal / dish pointing    │
   │  timestamps     │                 │  loggers, dashboards, …    │
   └─────────────────┘                 └────────────────────────────┘
```

It depends on the C standard library and, for the interrupt lines, `libgpiod`
— nothing else, and `./configure` builds without `libgpiod` where it is
absent. Linux is the packaged target; imud also builds and runs on macOS,
where an FT232H USB dongle carries the I²C bus. License: MIT — see
[LICENSE](LICENSE).

## What it does

- **Owns the sensor, once.** Drains the IMU FIFO on a hardware interrupt,
  applies calibration, and runs a quaternion MEKF at the full sample rate —
  so consumers get a fused estimate, not raw samples to process themselves.
- **Publishes on standard interfaces, to many consumers at once.** NMEA 0183
  (UDP broadcast or a TCP listener plotters just connect to), a high-rate
  binary packet over UDP, and a loss-free stream of framed packets — local
  AF_UNIX socket or TCP — so several programs share one IMU without
  contention.
- **Clean, well-defined outputs.** Quaternion, Euler angles, magnetic and
  true heading, rate of turn, heave, sea-state statistics (significant wave
  height and period, roll/pitch periods and amplitudes), compass-health
  diagnostics, and the attitude covariance — each with wall-clock, TAI, and
  per-sample hardware timestamps for correlation with cameras and other
  sensors.
- **Pluggable hardware.** A thin driver layer hides chip differences behind
  one interface. Validated on silicon: the SparkFun 9DoF reference pair
  (ISM330DHCX + MMC5983MA), the TDK InvenSense MPU-9255 with its AKM AK8963
  compass, and the six-axis MPU-6500. Experimental drivers for ST LSM6DSO,
  LSM6DSOX, LIS2MDL and LIS3MDL, TDK InvenSense ICM-20948, ICM-42688-P and
  MPU-9250, AKM AK09916, and PNI RM3100; and a `sim` driver that runs the
  whole pipeline with no hardware. Addresses, interrupt pins and per-part
  notes are in the
  [driver table](docs/manual.md#5-supported-drivers).
- **I²C, SPI, or a USB dongle.** The sensor sits on a header's I²C or SPI bus,
  or on an FT232H USB bridge (`i2c_bus = "ftdi:"`) for a host that has no bus
  of its own — a laptop, a Mac, a Pi whose header is already spoken for. Same
  drivers, same config, no library and no root; the bridge has no interrupt
  line, so the readers poll. See
  [§5.2 of the manual](docs/manual.md#52-i²c-over-an-ft232h-usb-bridge).
- **6-DoF or 9-DoF.** With `mag.driver = "none"` imud runs a gyro+accelerometer
  board and everything that does not need a compass keeps working: roll, pitch,
  heave, sea state and rate of turn are all gravity- or gyro-referenced. Only
  heading changes — it starts at zero, is relative to the orientation imud
  started in rather than to earth north, and drifts.
- **A flight recorder built in.** The `[capture]` black box records every raw
  sensor sample to rotating files; `imud --replay` plays a capture back
  through the full pipeline, and `imud-cal` measures your unit's actual noise
  (Allan variance) and gyro temperature drift from the same files. See
  [docs/capture.md](docs/capture.md).
- **An ABI-stable client library.** `libimud` decodes the binary stream for C
  programs and keeps working across daemon upgrades without recompiling;
  a single-file Python client ships too, and an Arduino/ESP32 client
  ([imud-arduino](https://github.com/richcreations/imud-arduino)) that lives in
  its own repository.
- **Built to run unattended.** A hardened systemd unit with a watchdog,
  calibration tools, level-gated logging, and a status socket.

## Example uses

imud is output-agnostic; the same daemon serves very different consumers:

- **Marine navigation** — NMEA 0183 to chartplotters, autopilots, and
  Signal K, with true heading from the World Magnetic Model, heave, and live
  sea-state statistics. (The most exercised use case today; several fusion
  options are tuned for it.) The `imud-signalk` bridge also feeds Signal K
  natively over UDP or TCP when its NMEA parsing falls short.
- **Robotics / ROS2** — attitude and rate of turn over the binary stream.
- **Drones & autopilots** — the `imud-mavlink` bridge feeds MAVLink ATTITUDE to
  ArduPilot, PX4, or QGroundControl over UDP, serial, or TCP.
- **IoT / home automation & dashboards** — the `imud-mqtt` bridge publishes
  heading/attitude/heave to an MQTT broker with Home Assistant auto-discovery;
  `imud-influxdb` writes line-protocol points to InfluxDB for Grafana;
  `imud-prometheus` serves a `/metrics` endpoint for Prometheus alerting.
- **Machine vision & camera stabilization** — high-rate quaternion with
  hardware timestamps for frame-accurate correlation.
- **Gimbals, pan/tilt rigs, and antenna/dish pointing** — low-latency
  attitude over the local stream socket or binary UDP.

If you just need heading/pitch/roll for a chartplotter or autopilot, enable
the NMEA output (`[nmea] tcp_enabled = true` and connect your app to
`tcp://<host>:10110`, or `enabled = true` for UDP broadcast). If you need
high-rate quaternion for vision or control, enable the binary stream on port
10111, the local socket (on by default), or its TCP listener
(`[stream] tcp_enabled`, port 10112). A stock install emits only on the
local socket — network outputs are explicit opt-ins.

## Quick start

**Raspberry Pi OS / Debian (arm64/armhf) — install from the apt repository:**

The suite is read from `/etc/os-release`, so these are the same commands on
bookworm and trixie — nothing to substitute:

```sh
# 1. Trust the signing key
curl -fsSL https://richcreations.github.io/imud/apt/KEY.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/imud.gpg

# 2. Add the repository (suite detected from /etc/os-release)
sudo tee /etc/apt/sources.list.d/imud.sources >/dev/null <<EOF
Types: deb
URIs: https://richcreations.github.io/imud/apt
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: main
Signed-By: /usr/share/keyrings/imud.gpg
EOF

# 3. Install the daemon + World Magnetic Model data
sudo apt update && sudo apt install imud imud-wmm-data

# 4. Edit for your hardware, then start on boot
sudo nano /etc/imud/imud.conf
sudo systemctl enable --now imud
```

The package creates the `imud` user and the `gpio`, `i2c` and `spi` groups, and
installs a udev rule granting those groups the I²C, SPI and GPIO device nodes —
so this works on a stock Debian, not only on Raspberry Pi OS. To read the
stream socket or run `imud-status` as yourself, join the `imud` group:
`sudo adduser "$USER" imud`.

If you added `/etc/apt/sources.list.d/imud.list` under earlier instructions,
remove it (`sudo rm -f /etc/apt/sources.list.d/imud.list`) so apt does not see
the repository twice.

Optional bridges and the network monitor are separate packages:
`imud-signalk`, `imud-mqtt`, `imud-influxdb`, `imud-mavlink`, `imud-prometheus`,
`imud-utils`. See <https://richcreations.github.io/imud/apt/>.

**Or build from source** (any Linux host with I²C or SPI):

```sh
sudo apt update && sudo apt install -y build-essential libgpiod-dev
./configure                  # optional: reports what this host can build
make
sudo make install
sudo make install-wmm-data   # World Magnetic Model data (for true heading)
sudo nano /etc/imud/imud.conf
sudo systemctl enable --now imud
```

**On macOS** the build is `./configure && make` — configure is required there,
since it is what picks the backends a Mac has (add Homebrew's `mosquitto` for
the MQTT bridge). There is no header bus, so reach the sensor through an
FT232H dongle (`i2c_bus = "ftdi:"` and `int_gpio = 0`), or run the `sim`
driver with no hardware at all. `sudo make install` installs a launchd job
rather than a systemd unit. There is no package; CI builds and runs the whole
test suite on macOS 14 and 26 and on Intel.

Check it and watch the streams:

```sh
imud-status        # daemon health, attitude, declination, heave
imud-mon           # live view of the output streams
```

No hardware yet? Run the full pipeline in simulation — or replay a recorded
capture from a real vessel:

```sh
make
imud --config config/sim.conf              # synthetic scenario
imud --replay session.imucap               # recorded raw sensor data
```

Before first real use, calibrate: `imud-cal gyro`, `imud-cal accel`, and an
in-situ `imud-cal mag`. See the
[calibration guide](docs/manual.md#6-calibration).

## Tools

| Command | Purpose |
|---|---|
| `imud` | The daemon. |
| `imud-cal` | Gyro, accelerometer, and magnetometer calibration. |
| `imud-status` | Query a running daemon's health. |
| `imud-mon` | Live monitor of the output streams from any host on the network (`make install-utils`). |
| `imud-imutest` | Validate a sensor driver against real hardware; writes a Markdown report to attach to an issue (`make install-utils`). |
| `imud-signalk` | Bridge daemon (optional install): pushes Signal K deltas over UDP from the local stream socket. |
| `imud-mqtt` | Bridge daemon (optional install): publishes MQTT topics + Home Assistant discovery from the local stream socket. |
| `imud-influxdb` | Bridge daemon (optional install): writes InfluxDB line-protocol points (UDP/HTTP) for Grafana. |
| `imud-mavlink` | Bridge daemon (optional install): emits MAVLink (v1/v2) attitude over UDP/serial to autopilots and GCSs. |
| `imud-prometheus` | Bridge daemon (optional install): serves the fused state as Prometheus `/metrics` gauges. |

## Documentation

- **[Manual](docs/manual.md)** — installation, the complete configuration
  reference, calibration, output streams, monitoring, troubleshooting, and a
  guide to writing new drivers.
- **[Protocol spec](spec.md)** — architecture, the binary packet layout, NMEA
  sentence formats, and the timestamp design.
- **libimud** — the ABI-stable C client library and the Python client for the
  binary stream. Its own packages — `libimud0` for the runtime, `libimud-dev`
  for the header and pkg-config file: see `man 3 libimud`, with the README,
  manual and spec installed alongside them.
- **[imud-arduino](https://github.com/richcreations/imud-arduino)** — the
  Arduino/ESP32 client library (`ImudClient`) for the binary stream over TCP
  or UDP, maintained in its own repository.
- **[Capture & replay](docs/capture.md)** — the black box, playback, and
  offline noise/temperature analysis.
- **Bridges** — each optional bridge has its own docs under `docs/imud-<name>/`
  (README, manual, spec), installed to `/usr/share/doc/imud-<name>/`; see the
  [Bridges section](docs/manual.md#9a-bridges) of the manual.
- **[Contributing](CONTRIBUTING.md)** — build, test, coding conventions, and how
  to submit a pull request.
- **[Governance](GOVERNANCE.md)** — who maintains imud, how decisions get made,
  and what happens to the project if the maintainer stops.
- Man pages: `imud(8)`, `imud-cal(8)`, `imud.conf(5)`, `imud-status(1)`,
  `libimud(3)` (installed by `make install`); `imud-mon(1)` and
  `imud-imutest(8)` by `make install-utils`; `imud-signalk(8)` / `imud-mqtt(8)` /
  `imud-influxdb(8)` / `imud-mavlink(8)` / `imud-prometheus(8)` (each with
  an `imud-<name>.conf(5)`) by the matching `install-<name>` target.
- **[ROADMAP](docs/ROADMAP.md)** — future features, hardware support and
  project direction.
