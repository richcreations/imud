# imud — Deferred & Future Work

Items identified during the 2026-07-06 audit and later passes that remain
open. Completed work is tracked in git history, NEWS, and spec.md §6, not
here.

## 1. Hardware validation matrix  *(bench task — requires the Pi and real silicon)*

Seven drivers are implemented but marked `experimental = true` and have never run on
real hardware: `icm20948`, `ak09916`, `icm42688p`, `lsm6dso`, `lsm6dsox`, `lis3mdl`,
`lis2mdl`. Each needs a bench pass (probe/WHO_AM_I, init, ODR verification, FIFO/DRDY
behavior, sane values in all orientations) before clearing its flag. The driver
contract in the driver guide in `docs/manual.md` makes this mechanical. Two of
them carry unverified pre-1.5-tag fixes to confirm on the bench: `ak09916`
(DRDY-timeout/overflow now return "no data" instead of counting toward the
error-reset threshold) and `icm42688p` (1 µs timestamp tick + 20-bit counter
unwrap — check `ts_wall_ns` monotonicity across the ~1 s TMSTVAL wrap). The reference pair
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

The mechanism shipped in 1.5: cal.json `gyro_temp` per-axis linear
coefficients applied per-sample before fusion, fitted offline with
`imud-cal fit-temp --from <warm-up capture>`, fed by 12.5 Hz FIFO
temperature batching. **Remaining: record a real cold-boot→warm capture on
the Pi and fit actual coefficients** — pair with the first hardware
sessions. The ISM330 self-heats only ~1 °C, so a usable fit needs a real
ambient swing of several °C (leave it across a garage day/night cycle, or
warm it gently mid-capture).

## 3. Pi 5 interrupt latency re-profiling  *(pre-existing spec §16 item, bench)*

Pi 5 routes GPIO through the RP1; gpiod is the right abstraction but edge-interrupt
latency should be measured against the Pi 4 baseline once hardware testing starts.

## 4. ISM330DHCX MLC engine detection  *(pre-existing spec §16 item, optional)*

The current engine-vibration detection is software (EMA of (|a|−g)², with ×4 accel
noise inflation while active). The ISM330's on-chip Machine Learning Core could
assert a GPIO on engine-on instead. Only worth it if the software detector proves
finicky at sea.

## 5. Output bridges  *(new consumers of the existing streams — no core changes)*

A bridge subscribes to imud's loss-free AF_UNIX stream socket (never the
lossy UDP broadcast), translates, and re-emits on another protocol; the core
daemon is untouched. Five have shipped (`imud-signalk`, `imud-mavlink`,
`imud-mqtt`, `imud-influxdb`, `imud-prometheus`) — new bridges should follow
`imud-prometheus`, which builds on libimud's ABI-stable `imud_data_t`
instead of the wire-pinned `imud_client.h`. Anything that drags in a large
middleware or toolchain (ROS2, CAN) is better as its own project that
reuses the client lib rather than something built under this Makefile.

Still open:

- **WebSocket / SSE JSON** — browser dashboards and a live 3-D attitude view with no
  native client. *(medium — needs a tiny embedded HTTP/WS server)*
- **Foxglove** — WebSocket + Foxglove protocol (or MCAP capture) for robotics-grade
  visualization and replay. *(medium)*
- **OSC** — attitude over Open Sound Control (UDP) for camera rigs, gimbals, and
  AV / interactive installations. *(easy)*
- **ROS2** — `sensor_msgs/Imu` (+ `MagneticField`, `Temperature`), NED → REP-103
  ENU/FLU. **Tracked as its own project** (needs an ament/colcon package; can't
  build under this Makefile). An `rclpy` node reusing `imud_client.py` is the light
  path; the frame conversion is the substantive work.
- **NMEA 2000 / N2K** — DEMOTED 2026-07-11: covered via Signal K, recipe in
  docs/manual.md §7 (imud → imud-signalk → Signal K server →
  `signalk-to-nmea2000`, verified against the plugin source: 127250 heading,
  127257 attitude, 127258 variation; the official plugin does NOT emit
  127251 rate of turn or 127252 heave). The server owns the hard N2K
  device-level work (ISO address claim, product info) that a direct C bridge
  would have to reimplement; CAN hardware is needed either way. A direct
  bridge only serves the no-Signal-K "appliance" install — revisit if that
  demand materializes, or if 127251/127252 on the backbone become must-haves.

## 6. Ideas beyond the audit roadmap  *(brainstorm 2026-07-11, roughly by leverage)*

- **Multi-IMU.** Two sensor pairs fused, or at minimum hot-failover with
  cross-checking — the vessel-grade redundancy story (gpsd's multi-receiver
  support is the precedent the name invokes).
- **Ecosystem gravity on libimud.** The ABI-stable .so makes bindings nearly
  free: Rust -sys crate, Go package, Python cffi → crates.io/PyPI. Plus the
  shareable browser demo: live 3-D horizon over the WebSocket bridge. First
  satellite client shipped 2026-07-19: **imud-arduino**
  (github.com/richcreations/imud-arduino), the Arduino/ESP32 wire client in
  its own repository.
- **SPI transport.** Unlocks high-ODR modes (6.6 kHz ISM330) and lower jitter;
  pairs with the Pi 5 latency profiling item.
- **Debian archive submission.** The self-hosted apt repo shipped (1.5
  follow-on, richcreations.github.io/imud/apt); the next step toward the
  original goal is submission to the Debian archive proper (source package +
  orig tarball; lintian refinements in packaging/README).

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

## 8. Code consolidation  *(next-release candidates — deferred from the 2026-07-15 pre-1.5 audit)*

Duplication clusters found by the pre-tag full-repo audit, deferred so a
refactor just before the release tag would not invalidate the testing 1.5
had already had. Still open after 1.6 (which added the TCP outputs without
touching these). None blocks anything; each is a mechanical extraction plus
a re-test.

- **`sd_notify_msg()` ×6** — byte-identical copies in `main.c` and all five
  bridge mains (the copies even say "mirrors src/signalk_main.c"). Extract a
  shared `src/sd_notify.c`.
- **Driver I2C register wrappers ×7** — `burst_read`/`reg_write`/`reg_read`
  are near-identical `I2C_RDWR` ioctl wrappers in seven drivers. A shared
  helper needs an auto-increment flag (`lis3mdl` sets `0x80|reg`).
- **`ism330dhcx.c` ↔ `lsm6dso.c` near-twin drivers** — FIFO-tag drain loop,
  axis remap, and timestamp back-calculation are copy-paste; only WHO_AM_I
  and a CTRL9_XL write differ by design. No drift observed yet, but a bug
  fixed in one silently won't propagate to the other.
- **Bridge stream-loop skeleton ×4–5** — the `while(!g_stop)` body
  (watchdog ping → SIGHUP reload → reconnect → read → deadline-advance) is
  structurally duplicated across the bridge mains; the deadline-advance
  block is verbatim in three of them.
- **UDP-open drift (user-visible symptom)** — `signalk_main.c` `open_udp_dest()`
  uses `inet_pton` (numeric IPv4 only) while influxdb/mavlink use
  `getaddrinfo`: the Signal K UDP destination rejects hostnames the other
  bridges accept. Unifying on a shared `getaddrinfo` helper fixes it.
  (Daemon *bind* addresses stay `inet_pton`-only by design — no DNS in the
  listen path.)
- **`crc32_ieee` ×3 in-tree C copies** — packet.c, libimud.c, mon_main.c
  could share one internal helper. The copies in `lib/imud_client.h`
  (self-contained header) and the Python client (zlib) stay by design.
- Minor notes from the same audit: `imud.service` `RestrictAddressFamilies`
  omits `AF_INET6` while the gpsd/Signal K position clients target
  `localhost` (may resolve `::1`); `FLAG_MOTION` (bit 6) is defined but
  never set — decide to wire it up or retire it at the next wire-version
  bump (marked "reserved" in all headers as of 1.5).

## 9. Small items

- `ctx->stop` in imu.c stays `volatile int` (not `_Atomic`) because its address feeds
  the `imu_ring_pop()` API; changing it means touching ring.h/ring.c/test_ring.
  Cosmetic C11-cleanliness only.
- Heave settling: ~10·τ (≈2 min) after boot before heave is trustworthy. Could be
  shortened by initializing the integrators from the first seconds of data.
- Centripetal correction models `v_body = [speed, 0, 0]` (no leeway); a leeway-angle
  estimate could refine it, but the residual is second-order for typical vessels.
- Validate capture → replay on real boat captures (the synthetic path is
  regression-tested); the §2 thermal capture can double as the test file.

---
*Compiled 2026-07-06; output-bridges section added 2026-07-08; ideas section
+ N2K demotion added 2026-07-11; pruned of shipped items 2026-07-19 (1.6).*
