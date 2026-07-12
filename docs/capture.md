# Capture & replay — imud's black box

imud can record every raw sensor sample to disk and play recordings back
through the full processing pipeline. A boat session becomes a permanent
regression corpus; filter changes get validated against real seaways; a bug
report becomes "send me your capture". The same files feed the offline
analysis modes of `imud-cal` (noise characterization, gyro temperature
fitting).

Everything here ships in the base `imud` package — capture is built into the
daemon, playback into the sim driver, analysis into imud-cal.

## Recording ([capture])

Enable the black box in `imud.conf`:

```toml
[capture]                # all keys [restart]
enabled   = true
dir       = "/var/lib/imud"   # imud-YYYYMMDD-HHMMSS.imucap (UTC names)
max_mb    = 256               # rotate the file at this size; 0 = unlimited
max_files = 8                 # keep at most N files (oldest deleted); 0 = all
flush_s   = 5                 # flush interval
```

Samples are recorded **exactly as the driver's `read()` delivered them** —
before mount rotation, before calibration, before bias correction. Replay
therefore traverses the identical pipeline the live sensors do, and a capture
can be re-run against a *different* cal.json, mount setup, or filter tuning.

The reader threads never block on storage: they push into a ring and move on.
A stalled SD card costs dropped capture records (counted, logged, and shown
by `imud-status` in the `Capture:` line) — never sensor latency. A crashed or
power-cut daemon leaves a valid file with a truncated tail that readers treat
as clean EOF.

Sizing: one IMU record is 48 bytes on disk. At 104 Hz with a 100 Hz
magnetometer that is ~25 MB/hour; the default 8 × 256 MB ring holds several
days of continuous recording.

## Replay

Play a capture through the daemon — fusion, NMEA, binary streams, bridges,
everything:

```sh
imud --replay /var/lib/imud/imud-20260712-031500.imucap
```

`--replay` forces both sensors onto the sim driver in playback mode. The
equivalent config form (plus its knobs):

```toml
[imu]    driver = "sim"
[mag]    driver = "sim"
[device]
sim_file  = "/var/lib/imud/imud-20260712-031500.imucap"
sim_loop  = false   # true: repeat forever (seq/chip_ts/time stay monotonic)
sim_speed = 1.0     # pacing: 2.0 = double speed, 0 = as fast as possible
```

Playback preserves the recorded chip timestamps and IMU/mag relative timing
(both anchored to the capture header's start time). At end of file with
`sim_loop = false` the stream simply stops (the log says so, and
`imud-status` goes stale). Set `[imu] odr_hz` near the capture's rate — the
per-sample timing always comes from the recorded timestamps.

With no `sim_file`, `driver = "sim"` synthesizes the classic test scenario
(yaw sweep + waves) exactly as before — and running THAT with `[capture]
enabled` is how you generate a synthetic scenario file: the daemon is its own
scenario generator; there is no separate tool.

## Offline analysis (imud-cal)

Record a **stationary** capture (vessel tied up, daemon otherwise idle), then:

```sh
imud-cal characterize --from imud-20260712-031500.imucap
```

computes per-axis Allan-deviation noise characteristics — gyro noise density
(rad/s/√Hz), gyro bias instability (rad/s), accelerometer noise density
(m/s²/√Hz) — and writes them to cal.json's `noise` section. The daemon then
tunes the MEKF to *your* silicon instead of generic datasheet numbers
(`[fusion] use_measured_noise = true`, the default; the four `mekf_*` config
keys remain as the fallback and override). An overnight capture pins down the
bias-instability floor; short captures yield upper bounds and say so.

```sh
imud-cal fit-temp --from warmup.imucap
```

fits per-axis linear gyro-bias/temperature coefficients from a warm-up
capture (record from cold boot until the die temperature levels out; a span
of several °C is required). The daemon subtracts `coeff × (T − 25 °C)` from
the gyro before fusion, so the filter's bias estimator only tracks the
residual. Best captured on the bench across a real thermal swing.

Both modes only read the capture file — they never touch the sensors and can
run on any machine.

## Why capture lives in the daemon (not a bridge)

The wire packet does carry raw sensor fields, so a stream-socket consumer
*can* log them (`imud_client.py` will happily dump packets). But output
threads are samplers at their own rate: against the sensor ODR that aliases
(duplicates + gaps), a FIFO-burst configuration loses most of each burst, and
the wire carries no magnetometer sequence number or timestamp to reconstruct
mag sample identity. The in-daemon tap sees every sample of both sensors in
every configuration, and a black box should not depend on a second process
staying alive. Casual wire logging remains a fine lightweight option; the
capture file is the exact one.

## File format (.imucap)

Little-endian, packed; authoritative definitions in `include/capture.h`.

**Header (104 bytes):**

| field | type | notes |
|---|---|---|
| magic | char[8] | `"IMUCAP1\0"` |
| version | u16 | format version, currently 1 |
| hdr_len | u16 | offset of the first record (future header growth) |
| imu_odr_hz | u32 | configured IMU rate |
| imu_driver, mag_driver | char[16] ×2 | driver names |
| imud_version | char[16] | writer's release |
| t0_wall_ns, t0_mono_ns | u64 ×2 | CLOCK_REALTIME / CLOCK_MONOTONIC at start |
| reserved | u8[24] | zeroed |

**Records** — frame `{u8 type, u8 flags, u16 len, u64 mono_ns}` + payload:

| type | payload | bytes |
|---|---|---|
| 1 IMU | accel[3] f32, gyro[3] f32 (pre-mount/cal), temp_c f32, chip_ts u32, seq u32 | 36 |
| 2 MAG | field[3] f32 (pre-mount/cal), wall_ns u64; `valid` = frame flag bit 0 | 20 |
| 3 MARK | code u32 (annotations; reserved) | 4 |

Readers skip unknown record types via `len` and read-then-skip extended
payloads, so the format grows append-only like the wire packet. A truncated
trailing record is treated as clean EOF. The reader is fuzz-tested
(`fuzz/fuzz_capture.c`) — captures are exactly the artifact users share.
