# imud-utils — reference

Two tools: `imud-mon`, whose display is described first, and `imud-imutest`,
whose checks are described at the end.

## imud-mon — display reference

`imud-mon` listens on the UDP ports configured for `imud` and prints a live
display, refreshed once per second. By default it monitors both streams;
naming one or more streams (`nmea`, `binary`) restricts it to those.

It decodes the wire packet directly (via `imud_client.h`), so it is
**wire-pinned** to the daemon's packet version — unlike the bridges, which use
libimud. Keep `imud-utils` and `imud` at the same release.

## Streams

| Section | Source | Shows |
| --- | --- | --- |
| **NMEA** | UDP, `[nmea] dest_port` (default 10110) | Heading, pitch and roll parsed from `$PASHR`; rate of turn from `$TIROT`; true heading from `$HCHDT` when present; and the last received sentence verbatim. |
| **Binary** | UDP, `[highrate] dest_port` (default 10111) | Every field of the high-rate packet: heading, pitch, roll, rate of turn, quaternion, calibrated and raw gyro/accel/mag vectors, covariance, declination, die temperature, sequence number, and the status flags. |

A section reads `(no data)` while no packet has arrived — expected during the
daemon's startup settle, or when that stream is disabled in `imud.conf`.
Binary packets are validated (magic, version, CRC32) before display;
anything malformed is discarded silently.

## Status flags

Flags are shown as a compact string, one character per asserted flag:

| Char | Flag | Meaning |
| --- | --- | --- |
| `C` | `FUSION_CONVERGED` | fusion filter converged |
| `V` | `MAG_VALID` | magnetometer reading valid (not rejected) |
| `A` | `ACCEL_CAL` | accelerometer calibration loaded |
| `G` | `GYRO_CAL` | gyro bias applied (from cal.json, or estimated at startup) |
| `M` | `MAG_CAL` | magnetometer calibration loaded |
| `D` | `DECLINATION_VALID` | declination known; true heading available |
| `S` | `STARTUP` | startup / settling period active |
| `!` | `FIFO_OVERFLOW` | IMU FIFO overflow since the last packet |

So `CVM` means converged, mag valid, mag calibrated. The wire carries more
flags than `imud-mon` renders (see `spec.md §8` in the source root for the
full bitmask); these are the ones worth watching live.

## Transport

UDP only — `imud-mon` deliberately reads the broadcast/multicast streams, not
the daemon's local AF_UNIX socket, so it can watch a daemon from another
machine. The lossy transport is fine for a human-facing monitor; a dropped
datagram just means one skipped refresh, visible as a gap in the sequence
number.

## imud-imutest — what each check asserts

Check ids are stable and greppable, so a report can be diffed against a later
run. Anything not listed is recorded as `INFO` for the record.

### Phase A — passive

| id | Asserts |
| --- | --- |
| `imu.probe` | `probe()` accepts the part at the configured address. |
| `imu.probe.reject` | `probe()` returns −1 at an unused reserved address. A driver that passes here but not this one is not checking WHO_AM_I, or is swallowing the I²C error. |
| `imu.reset.rc` / `.ms` | `reset()` returns 0; elapsed time is recorded. Under 1 ms warns — the datasheet turn-on time is probably not being waited out. |
| `imu.init.rc` | `init()` returns 0 for the configured rate and full scales. |
| `imu.init.regdiff` | Control registers changed across `init()`. The diff is printed raw: decoding it needs the datasheet. |
| `imu.init.idempotent` | A second `init()` lands on a byte-identical register image — catches a bank left selected or a latched enable. |
| `imu.odr` | Measured rate matches `nearest_odr()`. On failure the note names which `supported_odr_hz[]` entry the measurement actually matches. |
| `imu.fifo.depth` | Depth grows with the wait, so `read()` drains a queue rather than one sample register. |
| `imu.fifo.overflow` / `.recovers` | Not draining eventually returns rc 1, and reads return to rc 0 afterwards. |
| `imu.seq.monotonic` / `.gapless` | `seq` never repeats or reverses, and gaps appear only where an overflow was reported. Deltas are unsigned, so the 32-bit wrap is handled. |
| `imu.err.nodata_not_error` | An empty FIFO returns 0 with `n = 0`, never −1. −1 is reserved for I²C faults; the daemon resets the chip on a run of them. |
| `imu.err.no_spurious` | 200 back-to-back reads on a healthy bus produce no −1. |
| `imu.noise.accel` / `.gyro` | Per-axis standard deviation is in a plausible band. **Exactly zero fails** — that axis is stuck and is not being decoded. |
| `imu.rest.gravity` | Mean \|a\| is 9.807 m/s². The note flags a power-of-two ratio, which is a wrong sensitivity constant. |
| `imu.temp.plausible` / `.varies` | Temperature is in range and moves. Pinned at exactly 25.000 fails: that is the placeholder, so the word is never decoded. |
| `imu.chipts.presence` | `chip_ts` is 0 if and only if `has_hw_timestamp` is false. |
| `imu.chipts.monotonic` / `.rate` / `.wall` | The counter advances, its period matches `ts_tick_ns`, and chip time tracks wall time across counter wraps. The report always prints the *implied* tick, which is the fastest way to spot a wrong `ts_tick_ns`. |
| `imu.drdy.edges` | The interrupt line produces edges at a rate matching either the per-sample or the watermark model; the report says which one fits. |
| `imu.fs.accel` | Gravity still reads 9.807 after re-initialising at every advertised accelerometer range — catches one wrong constant in one branch. |
| `imu.fs.gyro` | Noise scales with full scale between adjacent ranges. A proxy, stated as such: a stationary gyro reads ~0 at every range. |
| `mag.*` | The magnetometer equivalents, plus `mag.nodata_not_error` (not-ready must return 1, not −1), `mag.field_magnitude` (25–65 µT), and `mag.wall_ns`. |

### Phases B–D — guided

| id | Asserts |
| --- | --- |
| `face.N.sign` | The dominant axis and its sign match the expected value for that orientation. A wrong sign is diagnosed as a missing flip; a wrong axis as a swap. |
| `faces.frame` | Rollup: the board frame is NED (X forward, Y starboard, Z down). |
| `gyro.A.sign` | A commanded positive turn integrates positive, per the right-hand rule. |
| `gyro.A.scale` | The integrated angle is within 20% of commanded. Deliberately loose — it catches factor errors (57.3× for deg/s, 1/57.3 for a double conversion), not sensitivity. |
| `spin.frame_agreement` | The magnetometer heading and the gyro Z integral agree in direction and magnitude. Disagreement in direction means a mag axis is inverted relative to the IMU — the most common magnetometer-driver defect. |
| `spin.coverage` | The turn visited the whole heading circle (24 sectors of 15°). |
| `spin.dip` | The vertical component has the right sign for the configured hemisphere. Skipped when `position.latitude` is unset. |

The tool never grades a WARN as blocking, and a `SKIP` in the required set
suppresses the "clear experimental" recommendation and names which check was
missing.

## See also

- `man 1 imud-mon`, `man 8 imud-imutest` — options and examples
- `spec.md §8` (source root) — the wire packet format
- `docs/manual.md §11` (source root) — the driver contract these checks test
- [libimud](../libimud/spec.md) — the ABI-stable client path
