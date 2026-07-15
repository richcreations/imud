# imud-mon — display reference

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

## See also

- `man 1 imud-mon` — options and examples
- `spec.md §8` (source root) — the wire packet format
- [libimud](../libimud/spec.md) — the ABI-stable client path
