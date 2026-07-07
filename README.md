# imud — IMU Daemon

**imud** is a small, reliable attitude/heading daemon for Raspberry Pi. It
reads an IMU and a magnetometer over I²C, fuses them with a quaternion MEKF
into a real-time attitude estimate, and publishes heading, pitch, roll, rate
of turn, and heave on streams that navigation and robotics software can
consume directly.

It is built for the boat: NMEA 0183 for chartplotters and Signal K, a
high-rate binary stream for machine vision and gimbals, and true heading from
the World Magnetic Model. It depends only on `libgpiod` and the C standard
library.

License: MIT — see [LICENSE](LICENSE).

## Highlights

- **NMEA 0183** (UDP 10110) for chartplotters, autopilots, and Signal K —
  heading, pitch, roll, rate of turn, and heave, plus true heading when
  declination is known.
- **High-rate binary** (UDP 10111, up to 500 Hz) and **NDJSON** (UDP 10112)
  for machine vision, ROS2, and dashboards; plus a loss-free local
  subscription socket for co-located consumers.
- **True heading** from WMM2025 declination — set a fixed lat/lon, or track
  it live from gpsd or Signal K as the vessel moves.
- **Marine-tuned fusion** — heading-only magnetometer mode, rough-sea
  accelerometer gating, engine-vibration handling, and a heave estimator.
- **Small and dependency-light** — C11 + POSIX + `libgpiod`, one config file,
  a hardened systemd unit.

The reference hardware is the SparkFun 9DoF (ISM330DHCX + MMC5983MA), with
experimental drivers for several other ST/TDK/AKM parts and a `sim` driver
that runs the whole pipeline with no hardware. See the
[driver table](docs/manual.md#5-supported-drivers).

## Quick start

On the Pi:

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

Check it is running and watch the streams:

```sh
imud-status        # daemon health, attitude, declination, heave
imud-mon           # live view of the UDP output streams
```

No hardware yet? Run the full pipeline in simulation:

```sh
make
imud --config config/sim.conf
```

Before first real use, calibrate: `imud-cal gyro`, `imud-cal accel`, and an
in-situ `imud-cal mag` swing. See the
[calibration guide](docs/manual.md#6-calibration).

## Tools

| Command | Purpose |
|---|---|
| `imud` | The daemon. |
| `imud-cal` | Gyro, accelerometer, and magnetometer calibration. |
| `imud-status` | Query a running daemon's health. |
| `imud-mon` | Live monitor of the output streams from any host on the LAN. |

## Documentation

- **[Manual](docs/manual.md)** — installation, the complete configuration
  reference, calibration, output streams, monitoring, troubleshooting, and a
  guide to writing new drivers.
- **[Protocol spec](spec.md)** — architecture, the binary packet layout, NMEA
  sentence formats, and the timestamp design.
- **[Client libraries](lib/README.md)** — C and Python libraries for the
  binary stream.
- Man pages: `imud(8)`, `imud-cal(8)`, `imud.conf(5)`, `imud-status(1)`,
  `imud-mon(1)` (installed by `make install`).
- **[ROADMAP](docs/ROADMAP.md)** — deferred and future work.
