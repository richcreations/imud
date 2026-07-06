# imud — IMU Daemon (User Guide)

License: MIT — see [LICENSE](LICENSE)

imud is a small, reliable IMU daemon for Raspberry Pi that supports multiple IMU and magnetometer
combinations (for example ISM330DHCX + MMC5983MA), estimates attitude in real time, and publishes
easy-to-consume outputs for navigation and robotics.

Quick highlights:

- NMEA stream (10 Hz) for chartplotters and marine software — includes `$HCHDT` true heading when declination is configured
- High-rate binary stream (500 Hz) for machine vision and low-latency consumers
- WMM2025 magnetic declination: auto-computed from lat/lon, live-updated via gpsd or SignalK, or set statically
- Small footprint: depends only on `libgpiod` and standard C libraries

Supported hardware:

| Driver name | Chip | Type | GPIO interrupt | Notes |
| --- | --- | --- | --- | --- |
| `ism330dhcx` | ISM330DHCX (ST) | IMU | BCM 17 · pin 11 | Stable; primary reference platform |
| `icm20948` | ICM-20948 (TDK/InvenSense) | IMU | BCM 17 · pin 11 | Experimental |
| `icm42688p` | ICM-42688-P (TDK/InvenSense) | IMU | BCM 17 · pin 11 | Experimental |
| `lsm6dso` | LSM6DSO (ST) | IMU | BCM 17 · pin 11 | Experimental |
| `lsm6dsox` | LSM6DSOX (ST) | IMU | BCM 17 · pin 11 | Experimental |
| `mmc5983ma` | MMC5983MA (MEMSIC) | Magnetometer | BCM 27 · pin 13 | Stable; primary reference platform |
| `ak09916` | AK09916 (AKM) | Magnetometer | none (polling) | Experimental; no external INT pin in bypass mode |
| `lis3mdl` | LIS3MDL (ST) | Magnetometer | BCM 27 · pin 13 | Experimental |
| `lis2mdl` | LIS2MDL (ST) | Magnetometer | BCM 27 · pin 13 | Experimental |
| `sim` | — | IMU + Magnetometer | none | Software simulation; no hardware required |

GPIO pins shown are the defaults (`imu.int_gpio = 17`, `mag.int_gpio = 27`).
Set `int_gpio = 0` in config to disable the interrupt and use a polling timer
instead — useful when the pin is wired differently or unavailable.

Experimental drivers have their register maps verified from datasheets but have
not been validated on physical hardware.  imud prints a startup warning when an
experimental driver is active.

The driver config names above are the values for `imu.driver` and `mag.driver`
in `imud.conf`.  The drivers live in `src/drivers/` and the registry is in
`src/drivers.c`.

See the full protocol details in [spec.md](spec.md).

---

## Prerequisites

- Raspberry Pi running PiOS Bookworm (or any Debian Bookworm-based distro)
- I2C enabled (`sudo raspi-config` → Interface Options → I2C)
- `libgpiod` 1.x or 2.x — Bookworm ships 1.6.x by default; both are supported.
  The Makefile auto-detects the installed version via `pkg-config`.

```sh
sudo apt install -y libgpiod-dev
```

## Quick Start (on the Pi)

```sh
# 1. Install build dependencies
sudo apt update && sudo apt install -y build-essential libgpiod-dev

# 2. Build everything
make

# 3. Install binaries, config, and systemd service
sudo make install

# 4. Edit config for your hardware (I2C bus, GPIO pins, driver names)
sudo nano /etc/imud/imud.conf

# 5. Start (and enable on boot)
sudo systemctl enable --now imud
```

To test without installing, run in the foreground:

```sh
imud --config config/imud.conf
```

---

## What imud provides

- **NMEA 0183** (broadcast UDP port 10110): `$PASHR`, `$HCHDM`, `$HCHDG`, `$TIROT`, `$IIXDR` at up to 10 Hz.
  When magnetic declination is configured, `$HCHDT` (true heading) is added automatically and
  `$HCHDG` carries the magnetic variation fields.
- **High-rate binary** (UDP port 10111): calibrated and raw sensor samples, quaternion, covariance,
  timestamps, and — when declination is available — the `declination_deg` field so consumers can
  compute true heading themselves.
- **NDJSON** (UDP port 10112, optional): one JSON object per packet with heading, pitch, roll,
  rate-of-turn, quaternion, covariance, and `true_heading_deg` when declination is known.
- **Local subscription stream** (AF_UNIX socket, optional): the same binary packets over
  `SOCK_STREAM` for loss-free same-host consumers — subscribe by connecting to
  `/run/imud/imud-stream.sock`.
- `imud-status` — inspect sensor, fusion, and stream health at a glance.

If you only need heading/pitch/roll for a chartplotter or autopilot, enable the NMEA output and
point your consumer at UDP port 10110.

---

## Configuration

- The default configuration file is `/etc/imud/imud.conf`. A per-user file can live at `~/.config/imud/imud.conf`.
- Typical changes users make:
	- `nmea.enabled = true` — enable NMEA output
	- `nmea.dest_port = 10110` — change destination port
	- `highrate.enabled = true` — enable 500 Hz binary stream
	- `device.gpio_chip = "gpiochip4"` — Raspberry Pi 5 (RP1 GPIO)
	- `position.lat_deg` / `position.lon_deg` — enable WMM auto-computed magnetic declination
	  (imud loads `WMM.COF` at startup and emits true heading in all three output streams)
	- `position.declination_deg` — static declination override (if you know the local value)
	- `position.gpsd_enabled = true` — connect to gpsd for live GPS-driven declination updates
	- `position.signalk_enabled = true` — poll a SignalK server as a GPS fallback

To run with a custom config file:

```sh
imud --config /path/to/imud.conf
```

For a full description of every config key, see [docs/config.md](docs/config.md).
The annotated example file is at `config/imud.conf`.

---

## Calibration (recommended before first use)

- Gyro bias: keep the board still and run `imud-cal gyro`.
- Accelerometer: perform a 6-position calibration with `imud-cal accel`.
- Magnetometer: do at least two slow 360° circles with `imud-cal mag`.

Calibration is written to the path set by `calibration.file` in config (default `/etc/imud/cal.json`; override with `--output PATH`).

---

## Examples

- View daemon status:

```sh
imud-status
```

- Monitor all output streams live (updates once per second):

```sh
imud-mon
```

- Monitor specific streams only:

```sh
imud-mon nmea json
imud-mon binary
```

- Use a non-default config (e.g. sim mode):

```sh
imud-mon --config config/sim.conf
```

- Listen for NMEA sentences with netcat:

```sh
nc -u -l 10110
```

- Capture raw binary packets:

```sh
nc -u -l 10111 > raw_packets.bin
```

---

## Consumer libraries

The `lib/` directory contains ready-to-use client libraries for the binary stream:

- **C** — [`lib/imud_client.h`](lib/imud_client.h): single-header drop-in, no build system required.
  Includes `imud_true_heading(pkt)` which returns the true heading in [0°, 360°) when
  `IMUD_FLAG_DECLINATION_VALID` is set, or `-1.0f` if declination is not yet known.
- **Python** — [`lib/imud_client.py`](lib/imud_client.py): Python 3.8+, standard library only.
  The `ImudPacket.true_heading_deg` property returns `None` when declination is unavailable.

Both validate CRC32 and handle multicast. See [`lib/README.md`](lib/README.md) for usage and examples.

---

## Troubleshooting

- I2C devices not visible: enable I2C in Raspberry Pi configuration and verify with `i2cdetect -y 1`.
- Missing permissions accessing GPIO/I2C: run with appropriate privileges or use the systemd service installed earlier.
- No NMEA output: check `nmea.enabled` in the config and ensure consumers are listening on the correct UDP port.

If you need more help, check the `etc/imud.service` example or open an issue.

---

## Where to go next

- For protocol details and binary packet layout, see [spec.md](spec.md).
- For consumer library usage (C and Python), see [lib/README.md](lib/README.md).
- For a complete configuration reference (every key, type, default, and description), see [docs/config.md](docs/config.md).
- For advanced developer notes, see [docs/driver-guide.md](docs/driver-guide.md).
