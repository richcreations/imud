# imud — Deferred & Future Work

Items identified during the 2026-07-06 audit and fusion passes that remain open.
Completed work is tracked in git history and spec.md §6, not here.

## 1. Hardware validation matrix  *(bench task — requires the Pi and real silicon)*

Seven drivers are implemented but marked `experimental = true` and have never run on
real hardware: `icm20948`, `ak09916`, `icm42688p`, `lsm6dso`, `lsm6dsox`, `lis3mdl`,
`lis2mdl`. Each needs a bench pass (probe/WHO_AM_I, init, ODR verification, FIFO/DRDY
behavior, sane values in all orientations) before clearing its flag. The driver
contract in the driver guide in `docs/manual.md` makes this mechanical. The reference pair
(`ism330dhcx`, `mmc5983ma`) has run on the boat and worked; the chip is confirmed
stern-facing, so the 180° mount yaw is genuine. The modest heading offset seen in
that testing likely mixes the since-fixed mekf_align mirror bug (small when booting
on a near-north/south heading, since the error is −2× boot heading) with declination
and residual iron cal. Some of the experimental chips require boards not currently
on hand; validate opportunistically as hardware becomes available.

**First action back at the Pi:** deploy this build and boot on two different
headings — any remaining offset should now be identical on both boots (mount trim /
declination / iron cal), with the boot-heading-dependent component gone. Then a
swing-circle recal with the new ellipse fit, and set lat/lon (or gpsd) so WMM
declination is active.

## 2. Gyro bias temperature compensation  *(code shipped 1.5 — needs Pi thermal data)*

✅ Code landed 2026-07-12 (1.5): cal.json `gyro_temp` section (per-axis linear
coefficients + reference temp), applied per-sample before fusion; fitted
offline with `imud-cal fit-temp --from <warm-up capture>`. **Remaining: record
a real cold-boot→warm capture on the Pi and fit actual coefficients** —
pair with the first hardware sessions.

## 3. Pi 5 interrupt latency re-profiling  *(pre-existing spec §16 item, bench)*

Pi 5 routes GPIO through the RP1; gpiod is the right abstraction but edge-interrupt
latency should be measured against the Pi 4 baseline once hardware testing starts.

## 4. ISM330DHCX MLC engine detection  *(pre-existing spec §16 item, optional)*

The current engine-vibration detection is software (EMA of (|a|−g)², with ×4 accel
noise inflation while active). The ISM330's on-chip Machine Learning Core could
assert a GPIO on engine-on instead. Only worth it if the software detector proves
finicky at sea.

## 5. Output bridges  *(new consumers of the existing streams — no core changes)*

A bridge subscribes to imud's loss-free AF_UNIX stream socket (never the lossy
UDP broadcast), parses the framed packets with `lib/imud_client.{h,py}`,
translates, and re-emits on another protocol; the core daemon is untouched. `imud-signalk` (shipped) is the
reference pattern — a small `Type=notify` daemon reading the stream and pushing
deltas over UDP. Simple text/UDP protocols fit that same pure-C mold; anything
that drags in a large middleware or toolchain (ROS2, CAN) is better as its own
project that reuses the client lib rather than something built under this Makefile.

**Marine**
- **NMEA 2000 / N2K** — DEMOTED 2026-07-11: covered via Signal K; ✅ recipe
  documented in docs/manual.md §7 (2026-07-11, v1.4). The path imud →
  imud-signalk → Signal K server → `signalk-to-nmea2000` emits, VERIFIED
  against the plugin source: 127250 heading (magnetic + true), 127257
  attitude, 127258 variation. NOT in the official plugin: 127251 rate of
  turn and 127252 heave (an earlier note here claimed 127251 — wrong; a
  community fork advertises it, and displays derive ROT from heading). The
  server owns the hard N2K device-level work (ISO address claim, product
  info) that a direct C bridge would have to reimplement; CAN hardware is
  needed either way. A direct bridge only serves the no-Signal-K "appliance"
  install — revisit if that demand materializes, or if 127251/127252 on the
  backbone become must-haves.

**Robotics / autonomy**
- **ROS2** — `sensor_msgs/Imu` (+ `MagneticField`, `Temperature`), NED → REP-103
  ENU/FLU. **Tracked as its own project** (needs an ament/colcon package; can't
  build under this Makefile). An `rclpy` node reusing `imud_client.py` is the light
  path; the frame conversion is the substantive work.
- **MAVLink** — ✅ **shipped** as `imud-mavlink` (2026-07-08): HEARTBEAT +
  ATTITUDE / ATTITUDE_QUATERNION, MAVLink v1 or v2, over UDP and/or serial.
  Hand-rolled pure-C encoder (pymavlink-verified), own config/service/install target.

**Telemetry / dashboards**
- **MQTT** — ✅ **shipped** as `imud-mqtt` (2026-07-08): scalar telemetry topics +
  Home Assistant MQTT discovery, human units (deg/m/°C), via libmosquitto. Own
  config/service/install target, out of `make all`.
- **InfluxDB line protocol** — ✅ **shipped** as `imud-influxdb` (2026-07-08):
  line-protocol points over UDP or HTTP, deg/rad units, own config/service/install
  target, out of `make all`. Pure C, no deps.
- **Prometheus exporter** — ✅ **shipped** as `imud-prometheus` (2026-07-11, v1.4):
  /metrics gauges in base SI units, flag bits as 0/1 gauges, imud_up/packets_total.
  First bridge built purely on libimud's ABI-stable `imud_data_t` (no wire pinning).

**Web / visualization**
- **WebSocket / SSE JSON** — browser dashboards and a live 3-D attitude view with no
  native client. *(medium — needs a tiny embedded HTTP/WS server)*
- **Foxglove** — WebSocket + Foxglove protocol (or MCAP capture) for robotics-grade
  visualization and replay. *(medium)*
- **OSC** — attitude over Open Sound Control (UDP) for camera rigs, gimbals, and
  AV / interactive installations. *(easy)*

Rough priority for the rest: N2K is covered via Signal K (docs recipe pending);
**ROS2** proceeds on its own track. (**MQTT**, **InfluxDB**, and **MAVLink**
shipped.)

## 6. Ideas beyond the audit roadmap  *(brainstorm 2026-07-11, roughly by leverage)*

- **Raw capture & replay (the keystone).** ✅ shipped 2026-07-12 (1.5): the
  `[capture]` black box records raw pre-mount/pre-cal samples to rotating
  .imucap files; the sim driver replays them (`imud --replay`, `[device]
  sim_file/sim_loop/sim_speed`); docs/capture.md specifies the format. The
  end-to-end regression test (sim scenario → file → offline MEKF) caught a
  real sim pitch-axis sign bug on day one. **First real captures + replay
  validation happen on the Pi trip.**
- **Sensor self-characterization (Allan variance) in imud-cal.** ✅ shipped
  2026-07-12 (1.5): `imud-cal characterize --from <capture>` computes per-axis
  noise density + bias instability into cal.json's `noise` section; the daemon
  prefers measured values over the datasheet `mekf_*` keys
  (`use_measured_noise`, default true). Overnight capture on the Pi still
  wanted for real numbers.
- **Calibration UX + health.** ✅ landed 2026-07-11 (v1.4): guided swing cal
  (live 24-sector coverage bar with current-direction marker, live radius/RMS,
  bell + "FULL CIRCLE" cue) + cal-health monitoring (`mag_anomaly` +
  `mag_residual` EMAs on the wire, from pre-gate innovations — they tell you
  WHEN to re-swing). **Online iron refinement deliberately deferred**: the
  heading reference must never be fed from the filter's own attitude (gauge
  feedback — the same reason m_ref healing only touches magnitude/dip), so an
  online fit buys little over "health metric says re-run the 10-minute guided
  swing" while carrying real risk. Revisit only with hardware evidence that
  iron drifts faster than re-swinging is practical.
- **Sea state** — ✅ shipped 2026-07-11 (v1.4, wire v14): Hs, wave period,
  roll/pitch periods + significant amplitudes; WAVE_VALID flag.
- **Hardened-to-a-fault CI.** ✅ ASan/UBSan (1.4) + TSan and libFuzzer jobs
  (1.5): four fuzz harnesses (config, gpsd/SK JSON, wire packet, .imucap
  reader) with seed corpora in test/fuzz/, a 60 s smoke on every push plus a
  nightly deep run (1 h/harness, 4-way fork, cached corpus that compounds
  coverage); all cross-thread stop flags became `_Atomic` to make the tree
  TSan-clean. systemd unit sandboxing already ships in every service file.
- **arm64 CI.** ✅ shipped 2026-07-11 (v1.4); 1.5 added the `debs` job on the
  same matrix — dpkg-buildpackage builds all ten packages on amd64 + arm64,
  and the Release workflow drafts a GitHub release with the .debs + tarball
  on every version tag.
- **Multi-IMU.** Two sensor pairs fused, or at minimum hot-failover with
  cross-checking — the vessel-grade redundancy story (gpsd's multi-receiver
  support is the precedent the name invokes).
- **Ecosystem gravity on libimud.** The ABI-stable .so makes bindings nearly
  free: Rust -sys crate, Go package, Python cffi → crates.io/PyPI. Plus the
  shareable browser demo: live 3-D horizon over the WebSocket bridge.
- **SPI transport.** Unlocks high-ODR modes (6.6 kHz ISM330) and lower jitter;
  pairs with the Pi 5 latency profiling item.

## 7. Calendar item: WMM2030 refresh  *(due ~December 2029)*

data/WMM.COF is WMM2025, valid 2025.0–2030.0. When NOAA/NCEI publishes
WMM2030 (expected December 2029), release a new imud-wmm-data package
(model-versioned, `2030.0` — the tzdata pattern; independent of imud
releases): replace the COF, re-verify against the official test values
(test_wmm), bump packaging/imud-wmm-data/changelog. Existing installs can
also drop the new file into /etc/imud/WMM.COF, which imud prefers over the
packaged /usr/share/imud/WMM.COF. Vendoring decision reviewed 2026-07-11: no
Debian package provides the official NOAA COF format (geographiclib-tools
only offers a *downloader* for its own binary format), and a marine daemon
must work offline — keep vendoring.

## 8. Small items

- `ctx->stop` in imu.c stays `volatile int` (not `_Atomic`) because its address feeds
  the `imu_ring_pop()` API; changing it means touching ring.h/ring.c/test_ring.
  Cosmetic C11-cleanliness only.
- Heave settling: ~10·τ (≈2 min) after boot before heave is trustworthy. Could be
  shortened by initializing the integrators from the first seconds of data.
- Centripetal correction models `v_body = [speed, 0, 0]` (no leeway); a leeway-angle
  estimate could refine it, but the residual is second-order for typical vessels.

---
*Compiled 2026-07-06; software roadmap items landed same day; output-bridges
section added 2026-07-08; ideas section + N2K demotion added 2026-07-11.*
