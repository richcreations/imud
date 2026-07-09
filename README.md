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
- **Publishes on standard interfaces, to many consumers at once.** NMEA 0183,
  a high-rate binary packet over UDP, and a loss-free local stream socket —
  broadcast/multicast so several programs share one IMU without contention.
- **Clean, well-defined outputs.** Quaternion, Euler angles, magnetic and
  true heading, rate of turn, heave, and the attitude covariance, each with
  wall-clock, TAI, and per-sample hardware timestamps for correlation with
  cameras and other sensors.
- **Pluggable hardware.** A thin driver layer hides chip differences behind
  one interface. Reference support for the SparkFun 9DoF (ISM330DHCX +
  MMC5983MA); experimental drivers for several other ST/TDK/AKM parts; and a
  `sim` driver that runs the whole pipeline with no hardware.
- **Built to run unattended.** A hardened systemd unit with a watchdog,
  calibration tools, level-gated logging, and a status socket.

## Example uses

imud is output-agnostic; the same daemon serves very different consumers:

- **Marine navigation** — NMEA 0183 to chartplotters, autopilots, and
  Signal K, with true heading from the World Magnetic Model and a heave
  estimate. (The most exercised use case today; several fusion options are
  tuned for it.) The `imud-signalk` bridge also feeds Signal K natively over
  UDP when its NMEA parsing falls short.
- **Robotics / ROS2** — attitude and rate of turn over the binary stream.
- **IoT / home automation & dashboards** — the `imud-mqtt` bridge publishes
  heading/attitude/heave to an MQTT broker with Home Assistant auto-discovery.
- **Machine vision & camera stabilization** — high-rate quaternion with
  hardware timestamps for frame-accurate correlation.
- **Gimbals, pan/tilt rigs, and antenna/dish pointing** — low-latency
  attitude over the local stream socket or binary UDP.

If you just need heading/pitch/roll for a chartplotter or autopilot, enable
the NMEA output and point your software at UDP port 10110. If you need
high-rate quaternion for vision or control, enable the binary stream on port
10111 or the local socket.

## Quick start

On a Raspberry Pi (or any Linux host with I²C):

```sh
# 1. Dependencies
sudo apt update && sudo apt install -y build-essential libgpiod-dev

# 2. Build and install (binaries, config, systemd service, man pages)
make
sudo make install

# 3. Edit for your hardware (I2C bus, GPIO pins, driver names)
sudo nano /etc/imud/imud.conf

# 4. Start now and on boot
sudo systemctl enable --now imud
```

Check it and watch the streams:

```sh
imud-status        # daemon health, attitude, declination, heave
imud-mon           # live view of the output streams
```

No hardware yet? Run the full pipeline in simulation:

```sh
make
imud --config config/sim.conf
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
| `imud-mon` | Live monitor of the output streams from any host on the network. |
| `imud-signalk` | Bridge daemon (optional install): pushes Signal K deltas over UDP from the local stream socket. |
| `imud-mqtt` | Bridge daemon (optional install): publishes MQTT topics + Home Assistant discovery from the local stream socket. |

## Documentation

- **[Manual](docs/manual.md)** — installation, the complete configuration
  reference, calibration, output streams, monitoring, troubleshooting, and a
  guide to writing new drivers.
- **[Protocol spec](spec.md)** — architecture, the binary packet layout, NMEA
  sentence formats, and the timestamp design.
- **[Client libraries](lib/README.md)** — C and Python libraries for the
  binary stream.
- Man pages: `imud(8)`, `imud-cal(8)`, `imud.conf(5)`, `imud-status(1)`,
  `imud-mon(1)` (installed by `make install`); `imud-signalk(8)` /
  `imud-mqtt(8)` by `make install-signalk` / `install-mqtt`.
- **[ROADMAP](docs/ROADMAP.md)** — deferred and future work.
