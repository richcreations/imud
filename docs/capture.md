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
(both anchored to the capture header's start time). Set `[imu] odr_hz` near
the capture's rate — the per-sample timing always comes from the recorded
timestamps.

**Replay is deterministic, and it runs on the capture's clock.** Replaying one
file repeatedly produces bit-identical filter output — same quaternion, same
bias, same `dt`, frame for frame. Three properties make that true:

- Each read hands back the **burst that was recorded**, not whatever is due at
  the instant the reader thread is scheduled. The writer stamps every sample of
  a FIFO burst with one delivery instant, so the grouping is recoverable — and
  it has to be, because the daemon pins its chip→wall anchor to the last sample
  of a burst.
- The chip→wall anchor is built from the **recording** host's clocks, which the
  header carries, so `dt` is the one the live daemon computed rather than a
  function of when the replay happened to run.
- The reader threads' poll cadences are sized for a part filling a FIFO in real
  time. Replay bypasses them, so `sim_speed` alone decides pacing.

Because the timebase is the capture's, the `[stats]` latency figures during a
replay are the ones the **live** daemon measured (`fifo=`), paired with this
machine's own pipeline cost (`pipe=`); the two are on different clocks and
imud keeps them apart deliberately.

`sim_speed = 0` is a fidelity trade, not just a speed one: with pacing off, the
two sensor streams are no longer held in their recorded relationship, and the
IMU can outrun the ring and the aligner. Use it to get through a long file
quickly; use `sim_speed = 1` when the fused output is what you care about.

**The two forms end differently, on purpose.**

`--replay` is a one-shot run over a finite file, so it **exits when the file
runs out** — after waiting for the last samples to reach the filter, so the
tail is not lost. It exits `0` on a completed replay and `1` if the capture
cannot be opened or read, which makes `imud --replay f.imucap && …` work and
stops a mistyped path from idling. It ignores `sim_loop`: the contract of the
flag should not depend on a line in whatever config happens to be installed.

The **config form** (`[device] sim_file`) is unchanged: at end of file with
`sim_loop = false` the stream simply stops — the log says so and `imud-status`
goes stale — and the daemon keeps running, because it is a daemon.

One practical note for short captures: `[fusion] startup_settle_sec` and
`align_window_sec` default to 5 s **each**, counted in samples at the
configured rate, so the first ~10 s of any replay is consumed by startup.
Lower both when replaying a file shorter than that.

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
(m/s²/√Hz) — and writes them to cal.json's `noise` section as informational
per-unit sensor characterization: a record of what your silicon actually does.
They never feed the filter. The MEKF always uses its tuned `[fusion] mekf_*`
values, which are tuning constants — the gyro terms build the filter's process
noise Q, deliberately held above the raw sensor floor so the filter stays
responsive and the gyro bias observable. Driving Q from the measured floor makes
it too stiff, so no configuration path does it. An overnight capture pins down
the bias-instability floor; short captures yield upper bounds and say so.

```sh
imud-cal fit-temp --from warmup.imucap
```

fits per-axis linear gyro-bias/temperature coefficients from a warm-up
capture (record from cold boot until the die temperature levels out; a span
of several °C is required). The daemon subtracts `coeff × (T − 25 °C)` from
the gyro before fusion, so the filter's bias estimator only tracks the
residual. Best captured on the bench across a real thermal swing.

```sh
imud-cal fit-ra --from rough-water.imucap --config /etc/imud/imud.conf
```

checks the MEKF's accelerometer measurement model against a **rough-water**
capture — the opposite recording conditions to the two above. It reports the
gravity-direction residual, its correlation time, the innovation-distance
distribution against the filter's gates, and the mean NIS, which is the same
statistic the daemon publishes live as `nis_accel`. Use it to size
`[fusion] mekf_wave_accel`. Unlike the other two it **writes nothing**: what it
comments on is filter tuning in `imud.conf`, not a sensor property. It needs an
existing mag calibration in `cal.json`.

All three modes only read the capture file — they never touch the sensors and
can run on any machine.

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

Little-endian, packed; authoritative definitions in `include/capture.h`. The
byte order is the file's, not the host's — `src/capture.c` converts each field,
so a capture written on a big-endian machine reads identically anywhere.

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
