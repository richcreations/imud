# imud — IMU Daemon

**imud is a general-purpose IMU daemon for Linux — think of it as *gpsd for
IMUs*.** It owns the inertial sensor, does the hard real-time work once
(interrupt-driven sampling, calibration, sensor fusion, precise hardware
timestamps), and publishes a clean attitude/heading/motion estimate on
standard interfaces that any number of programs can read at the same time.

Instead of every application re-implementing I²C drivers and a Kalman filter,
you run one small daemon and consume its output. Like gpsd, it's meant to be
boring, always-on infrastructure: start it, forget it, and point your
software at the stream.

```
   IMU + magnetometer (I²C)                      consumers
            │                          ┌────────────────────────────┐
            ▼                          │  chartplotter / autopilot   │
   ┌─────────────────┐   NMEA 0183 ───▶│  ROS2 node                  │
   │      imud       │   binary UDP ──▶│  vision / stabilization     │
   │  drivers·MEKF·  │   AF_UNIX    ──▶│  gimbal / dish pointing     │
   │  timestamps     │                 │  loggers, dashboards, …     │
   └─────────────────┘                 └────────────────────────────┘
```

It depends only on `libgpiod` and the C standard library. License: MIT — see
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
  one interface. Reference support for the SparkFun 9DoF (ISM330DHCX +
  MMC5983MA); experimental drivers for several other ST/TDK/AKM parts; and a
  `sim` driver that runs the whole pipeline with no hardware.
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

```sh
# 1. Trust the signing key and add the repo (use bookworm or trixie)
curl -fsSL https://richcreations.github.io/imud/apt/KEY.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/imud.gpg
echo 'deb [signed-by=/usr/share/keyrings/imud.gpg] https://richcreations.github.io/imud/apt trixie main' \
  | sudo tee /etc/apt/sources.list.d/imud.list

# 2. Install the daemon + World Magnetic Model data
sudo apt update && sudo apt install imud imud-wmm-data

# 3. Edit for your hardware, then start on boot
sudo nano /etc/imud/imud.conf
sudo systemctl enable --now imud
```

Optional bridges and the network monitor are separate packages:
`imud-signalk`, `imud-mqtt`, `imud-influxdb`, `imud-mavlink`, `imud-prometheus`,
`imud-utils`. See <https://richcreations.github.io/imud/apt/>.

**Or build from source** (any Linux host with I²C):

```sh
sudo apt update && sudo apt install -y build-essential libgpiod-dev
make
sudo make install
sudo make install-wmm-data   # World Magnetic Model data (for true heading)
sudo nano /etc/imud/imud.conf
sudo systemctl enable --now imud
```

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
- **[libimud](docs/libimud/)** — the ABI-stable C client library and the Python
  client for the binary stream: [README](docs/libimud/README.md),
  [manual](docs/libimud/manual.md), [spec](docs/libimud/spec.md).
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
- Man pages: `imud(8)`, `imud-cal(8)`, `imud.conf(5)`, `imud-status(1)`,
  `libimud(3)` (installed by `make install`); `imud-mon(1)` by
  `make install-utils`; `imud-signalk(8)` / `imud-mqtt(8)` /
  `imud-influxdb(8)` / `imud-mavlink(8)` / `imud-prometheus(8)` (each with
  an `imud-<name>.conf(5)`) by the matching `install-<name>` target.
- **[ROADMAP](docs/ROADMAP.md)** — deferred and future work.
