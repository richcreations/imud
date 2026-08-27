# imud — Roadmap

Where imud is going: the features planned, the hardware it intends to support,
and the longer-term direction of the project.

This is not a task list and not a history. What has already shipped is in
[NEWS](../NEWS) and the git log; how the current design was arrived at is in
[spec.md](../spec.md) and [docs/math.md](math.md).

Nothing here is a commitment to a date. imud has no release calendar — a
release goes out when there is something worth shipping, and hardware-dependent
work moves when the hardware is available.

---

## What imud is for

A general-purpose IMU daemon for Linux: read an inertial sensor and a
magnetometer, fuse them into an attitude estimate, and publish that estimate to
any number of consumers over open protocols. "gpsd for IMUs" — a piece of
infrastructure other software builds on, rather than an application.

Marine navigation is the most developed use case, not the identity. Robotics,
machine vision, gimbals and antenna pointing are equal targets, and the
marine-specific features (heave, sea state, NMEA 0183, magnetic declination
from the WMM) are examples of what the daemon can carry, not what it is.

**What imud is deliberately not:**

- **An application.** No UI, no chart plotter, no autopilot. It publishes state;
  other things decide what to do with it.
- **A framework.** Consumers link a small ABI-stable C library, or read a
  documented wire format, or subscribe to a bridge. There is nothing to adopt.
- **A single-vendor stack.** The driver interface is deliberately narrow so a
  new part is a self-contained file, and the output side speaks protocols that
  already exist rather than inventing one.

---

## Hardware support

Two parts are validated on real silicon and are the reference pair:
**ISM330DHCX** (IMU) and **MMC5983MA** (magnetometer), over both I²C and SPI.

**Nine further parts ship marked experimental**: `lsm6dso`/`lsm6dsox`,
`icm42688p`, `icm20948` and `mpu9250`/`mpu9255` on the inertial side, and
`lis3mdl`, `lis2mdl`, `rm3100`, `ak09916`, `ak8963` on the magnetometer side.
That is eleven configurable driver names over nine drivers, because
`lsm6dso`/`lsm6dsox` and `mpu9250`/`mpu9255` are each one driver answering to
two part numbers.

Their register maps are checked against the vendor datasheets and their
encode/decode paths are covered by tests against a mock bus, but they have never
run against the physical part. The daemon says so at startup.

**The intent is for all of them to become supported.** A driver's experimental
flag clears when someone runs `imud-imutest` against the real board and attaches
the report — the tool exists to make that a ten-minute job for someone who owns
the hardware, not a design exercise. That is the main way the supported-parts
list is expected to grow, and contributions of new parts are welcome on the same
terms.

Two transport limits are known and are properties of the parts, not of imud:

- **AKM magnetometers behind an InvenSense host** (`icm20948` + `ak09916`,
  `mpu925x` + `ak8963`) reach the compass through I²C bypass, which only an I²C
  host can use. SPI support for those boards needs the auxiliary-I²C-master
  read path, which is a different way of reading the part rather than a
  different way of addressing it.
- **LIS2MDL over SPI** defaults to three wires, and its 4-wire mode disables the
  data-ready line the driver depends on. It needs either half-duplex support in
  the bus layer or a polled read path.

Beyond the sensors, the target platform is a Raspberry Pi class machine running
Linux, and imud is expected to keep running on the older and smaller ones — a
Pi Zero 2 W is a supported deployment, not a stretch goal.

---

## Future features

**Gyro bias temperature compensation.** The mechanism ships today: per-axis
linear coefficients in `cal.json`, applied per sample before fusion, fitted
offline by `imud-cal fit-temp`. What is missing is fitted coefficients for real
parts, which needs a capture spanning a real ambient temperature swing.

**Continuous accelerometer deweighting.** Engine vibration currently switches
the accelerometer between two trust levels. Grading that continuously — and
possibly driving it from the ISM330DHCX's on-chip Machine Learning Core rather
than from a software detector — is the detection and response halves of one
problem, and would be scoped together.

**More output bridges.** A bridge reads the daemon's stream socket, translates,
and re-emits; the core is untouched, so bridges are additive by construction.
The shortlist is **WebSocket/SSE** for browser dashboards and a live 3-D
attitude view, **Foxglove** for robotics-grade visualisation and replay, and
**OSC** for camera rigs, gimbals and AV installations.

**Language bindings on libimud.** The ABI-stable shared library makes bindings
close to free — a Rust crate, a Go package, a Python cffi module. The first
satellite client already exists: `imud-arduino`, the Arduino/ESP32 wire client,
in its own repository.

**A fuller aerospace path.** imud's specific-force correction assumes motion in
the yaw plane, which is right for a surface vessel and systematically wrong for
an aircraft in a coordinated turn, climb or descent. The full three-dimensional
correction, three-dimensional magnetometer calibration for a tumble swing
rather than a horizontal one, and alignment that can start while already moving
are each additive paths selected by configuration. The marine configuration
stays the default and does not regress.

**Multi-IMU redundancy.** Two sensor pairs fused, or at minimum hot failover
with cross-checking — the vessel-grade redundancy story the "gpsd for IMUs"
comparison implies. This is the largest item here and would begin with a design
document rather than code.

---

## Long-term direction

The project's centre of gravity is the **stable interfaces**: an append-only
wire packet, an append-only client ABI, and a driver contract narrow enough that
a new part is one file. Those are what let consumers and satellite projects
exist without tracking imud's internals, and they constrain everything else —
the packet grows, it does not change shape.

**Distribution** follows the same logic. imud already publishes a signed apt
repository; submission to the Debian archive proper is the next step, and the
packaging is laid out for it.

**Versioning** is `MAJOR.MINOR.PATCH`. A major version means a break in the
libimud ABI, which has never happened and is not planned; the wire packet is
versioned separately and consumers reject a mismatch outright.

---

## Considered and set aside

Recorded here so the same ground is not covered twice. Each reopens only
against new evidence or a concrete need.

- **A direct NMEA 2000 bridge.** Covered through Signal K, which already owns
  the device-level work — address claim, product info — that a C bridge would
  have to reimplement, and CAN hardware is needed either way. Worth revisiting
  only for an appliance install with no Signal K server.
- **ROS2 inside this repository.** It needs a different build system entirely.
  It belongs in its own project reusing the client library, the way the Arduino
  client does.
- **Barometer and air-data aiding.** Each needs a whole new driver class or
  input channel rather than a new fusion update step.
- **Retuned accelerometer noise, Huber covariance variants, and GNSS
  dual-antenna heading.** All three were implemented, measured against the
  benchmark, and did not improve on what ships. Closed with measurements, not
  open questions.
- **Geometric magnetometer fits.** The algebraic fits meet the accuracy targets;
  the geometric refinement would be picked up only if they stop doing so in the
  field.

---

## The one dated item

`data/WMM.COF` carries the World Magnetic Model, which is valid to **2030.0**.
NOAA/NCEI is expected to publish the next model around **December 2029**; imud
ships it as a separately versioned `imud-wmm-data` package so the coefficients
can be refreshed without an imud release, and an installation can drop a newer
file into `/etc/imud/WMM.COF` in the meantime.
