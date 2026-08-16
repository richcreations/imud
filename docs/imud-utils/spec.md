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
| `imu.probe.reject` | `probe()` returns −1 at an unused reserved address. A driver that passes here but not this one is not checking WHO_AM_I, or is swallowing the I²C error. **SKIPs on SPI**, where chip select does the addressing: the `i2c_addr` field never reaches the wire, so the "bogus" probe reads the same part and returns 0 — the check misfiring rather than the driver failing it. Probing a neighbouring chip select is not a safe substitute, because the foreign register address landed on could be write-only or read-to-clear on that part, and disturbing the other sensor mid-run costs more than the row is worth. The SKIP still blocks the `experimental` recommendation: the evidence genuinely was not obtained, and reporting that honestly is not the same as reporting a pass. |
| `imu.reset.rc` / `.ms` | `reset()` returns 0; elapsed time is recorded. Under 1 ms warns — the datasheet turn-on time is probably not being waited out. |
| `imu.init.rc` | `init()` returns 0 for the configured rate and full scales. |
| `imu.init.regdiff` | Control registers changed across `init()`. The diff is printed raw: decoding it needs the datasheet. |
| `imu.init.idempotent` | A second `init()` lands on a byte-identical register image — catches a bank left selected or a latched enable. Compared over non-volatile registers only; see *Which registers are read* below. |
| `imu.odr` | Measured rate matches the rate the driver reports it will program (`odr_actual_imu()`, the same resolution the daemon uses). On failure the note names which `supported_odr_hz[]` entry the measurement actually matches. **Direction decides how far the excuses reach.** A rate *below* the configured one can be the read loop rather than the part — a stalled reader and a missing FIFO both lose samples — so a shortfall is graded down to a WARN when either applies. A rate *above* it cannot be: nothing in the read path invents samples the part did not produce, so the part is not running at the rate `init()` asked for, and that FAILs at any margin rather than passing through the warn band. |
| `imu.fifo.depth` | Depth grows with the wait, so `read()` drains a queue rather than one sample register. |
| `imu.fifo.overflow` / `.recovers` | Not draining eventually returns rc 1, and reads return to rc 0 afterwards. |
| `imu.seq.monotonic` / `.gapless` | `seq` never repeats or reverses, and gaps appear only where an overflow was reported. Deltas are unsigned, so the 32-bit wrap is handled. |
| `imu.err.nodata_not_error` | An empty FIFO returns 0 with `n = 0`, never −1. −1 is reserved for I²C faults; the daemon resets the chip on a run of them. |
| `imu.err.no_spurious` | 200 back-to-back reads on a healthy bus produce no −1. |
| `imu.noise.accel` / `.gyro` | Per-axis standard deviation is in a plausible band. **Exactly zero fails** — that axis is stuck and is not being decoded. |
| `imu.rest.gravity` | Mean \|a\| is 9.807 m/s². The note flags a power-of-two ratio, which is a wrong sensitivity constant. |
| `imu.temp.plausible` / `.varies` | Temperature is in range and moves. Pinned at exactly 25.000 fails: that is the placeholder, so the word is never decoded. |
| `imu.chipts.presence` | `chip_ts` is 0 if and only if `has_hw_timestamp` is false. |
| `imu.chipts.monotonic` / `.rate` / `.wall` | The counter advances, its period matches `ts_tick_ns`, and chip time tracks wall time across counter wraps. The report always prints the *implied* tick, which is the fastest way to spot a wrong `ts_tick_ns`. `.rate` compares chip time against chip time and so cannot see an oscillator error; only `.wall` can. A ratio **below** 1.0 is a lost counter wrap in the driver's unwrap; **above** 1.0 cannot be — the counter is ticking faster than `ts_tick_ns` claims, which scales every per-sample `dt` the daemon derives from it. `.wall` is graded at ±2% against `imu.odr`'s ±5%, which is the same ratio measured a second way. That is deliberate, and both notes say so: `imu.odr` asks whether `init()` programmed the rate it was asked for, an encoding question with room to spare, while `.wall` asks whether `ts_tick_ns` describes the counter, and `imu.c` multiplies that constant into every `dt` it hands the filter. A part can legitimately pass the first and warn on the second; when it does, `imu.odr` points at `.wall` rather than leaving two verdicts on one number unexplained. |
| `imu.drdy.edges` | The interrupt line produces edges at a rate matching either the per-sample or the watermark model; the report says which one fits. |
| `imu.fs.accel` | Gravity still reads 9.807 after re-initialising at every advertised accelerometer range — catches one wrong constant in one branch. |
| `imu.fs.gyro` | **INFO, not graded.** Records the noise floor at every advertised range. It does *not* assert that sigma scales with full scale: that only holds when quantisation dominates the noise floor, and on a good part it does not — an ISM330DHCX at ±125 dps has a 7.6e-5 rad/s quantisation step against a ~1.9e-3 rad/s measured floor, so sigma is flat across every range and a band around the full-scale ratio grades coin flips. The one direction that still WARNs is sigma *falling* by more than half as the range widens, which has no benign reading. |
| `mag.*` | The magnetometer equivalents, plus `mag.nodata_not_error` (not-ready must return 1, not −1), `mag.field_magnitude` (25–65 µT), and `mag.wall_ns`. `mag.rate` splits by direction exactly as `imu.odr` does, and for the same reason — an MMC5983MA reading 130 Hz against a configured 100 Hz was the loudest signal in a bench report and was graded a WARN nobody acted on, because the note only ever excused a *low* reading. `mag.init.regdiff` SKIPs on a part whose control registers are write-only (the MMC5983MA): a readback diff is structurally empty there no matter what `init()` wrote, so grading it would blame the driver for a property of the silicon. |
| `mag.set_reset` | The degauss pulse is issued, and it is issued **before** the measurements above rather than after them. Ordering is the substance: run last, it left every field and noise figure graded in whatever magnetisation state the part happened to arrive in, which is not a property of the driver. |
| `mag.degauss.differential` | Splits one reading into the field it measured and the bridge offset it carried, by measuring once after SET and once after RESET. SET and RESET magnetise the AMR film opposite ways, so between them the field term changes sign and the offset does not: `field = (vS − vR)/2`, `offset = (vS + vR)/2`. Graded on the **field** (25–65 µT); the offset is always reported. This is what tells a part looking at real iron apart from a part failing to remove its own bias — one symptom, opposite causes, and no other check on a single-transport bench can separate them. SKIPs where `mag_ops_t::degauss` is NULL, which is every part but the MMC5983MA today. Deliberately **not** in the `experimental`-clearing set: it is a diagnostic, and requiring it would block every driver that cannot drive RESET. |

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
missing. Where the flag is already clear, a clean run says so rather than
recommending a change that has already been made.

### Which registers are read

`imu.init.regdiff` and `imu.init.idempotent` both rest on a register snapshot,
and two kinds of register have to stay out of it.

**Destructive to read** — a FIFO port, a read-to-clear status word. These
cannot be found by experiment without corrupting the run, so `imt_regmaps[]`
in `src/imutest.c` lists them per driver: `skip[]` for individual registers,
`nrd_lo..nrd_hi` for a contiguous window. A FIFO port is usually a window, not
one register: on the ST 6-axis parts `FIFO_DATA_OUT` spans 0x78–0x7E (tag,
then X/Y/Z low/high), and listing only the first two left the sweep popping
five FIFO words per snapshot. **When you add a driver, check how wide its data
port is.**

**Volatile** — sensor output, status, FIFO level, the timestamp counter. Safe
to read, but they move on their own, so a diff across `init()` is swamped by
them and an idempotency compare over them is meaningless. These are *not*
listed. The tool finds them by reading the mapped range several times with the
part running and no writes in between; anything that changes is volatile and is
excluded from both checks. That needs no datasheet and stays correct for
drivers that do not exist yet — which matters, because this tree ships no
datasheets (see `docs/datasheets.md`) and a hand-written volatile table would
be exactly the kind of unverified register knowledge it avoids.

The filter is not perfect and the report says so: a volatile register that
happens to hold the same value through every pass — a stationary
accelerometer's high byte is the usual case — is not caught. Every report
prints how many registers were excluded and how many were compared, so a
narrower test never looks like a cleaner chip.

**Write-only control registers** are a third case, and only a listed flag can
express them: `ctrl_writeonly` on the MMC5983MA, whose CTRL0/1/2 the datasheet
gives as W. There the check SKIPs, because no readback can say anything about
what `init()` wrote. The register writes themselves are covered off-hardware by
`test_drivers` against the mock bus.

## See also

- `man 1 imud-mon`, `man 8 imud-imutest` — options and examples
- `spec.md §8` (source root) — the wire packet format
- `docs/manual.md §11` (source root) — the driver contract these checks test
- [libimud](../libimud/spec.md) — the ABI-stable client path
