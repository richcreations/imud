# Hardware evaluations

Verdicts on "should imud support part X?" requests, so the reasoning is
recorded once rather than re-litigated each time a part is asked for.

An entry here is a **decision record, not tracked work**. Parts that were
evaluated and accepted become drivers (`docs/manual.md` §5); parts that were
evaluated and declined or deferred stay here with the reasoning. Open work
lives in [ROADMAP.md](ROADMAP.md).

Register addresses, sensitivity constants and noise figures quoted below are
from general part knowledge, not from a datasheet in this tree — the repo
deliberately ships none (see [datasheets.md](datasheets.md)). Treat them as
**to be confirmed against the datasheet** if an evaluation turns into a
driver; shipped drivers cite a specific datasheet revision in their header
comment.

---

## InvenSense/TDK MPU-9250 / MPU-9255 ("MPU-925x")

**Requested:** 2026-07-25, by a user.
**Verdict (2026-07-25):** technically suitable and cheap to build, but
**deferred** — recommend clearing the existing experimental-driver validation
backlog first.
**Superseded (2026-07-28): BUILT.** Shipped as
[`src/drivers/mpu925x.c`](../src/drivers/mpu925x.c) (`mpu9250` + `mpu9255`) and
[`src/drivers/ak8963.c`](../src/drivers/ak8963.c), both `experimental = true`.

The deferral was reversed rather than overruled: its central objection was that
the part would become the eighth and ninth driver with no way to clear the
experimental flag. `imud-imutest` — built alongside these drivers — removes
that objection for the whole backlog, not just this part. The remaining
reasons (NRND, second-tier on merit, hard to validate because of counterfeits)
are real but are arguments about *what to recommend*, not about whether the
driver should exist; the design notes below were followed as written, including
the counterfeit-detection policy.

**Everything below is the original evaluation, kept because it is the design
rationale for the drivers as built.** One correction found while implementing
it is marked inline.

### What it is

A single package containing an MPU-6500 (3-axis gyro + 3-axis accelerometer)
and an **AK8963** magnetometer. I²C at 0x68/0x69 (AD0-selected). The
magnetometer sits on the MPU's auxiliary bus and is reached by enabling I²C
bypass (`BYPASS_EN` in `INT_PIN_CFG`, with `I2C_MST_EN` clear in
`USER_CTRL`), which exposes the AK8963 to the host bus at 0x0C.

That is the same arrangement imud already drives for the ICM-20948 and its
internal AK09916 — see the header comment in
[`src/drivers/icm20948.c`](../src/drivers/icm20948.c). The structural fit is
therefore about as good as it gets: no changes to `imu_ops_t`, `mag_ops_t`, or
anything in the core daemon.

### Why it is deferred rather than declined

Nothing about the part is technically disqualifying. The reasons are about
sequencing:

1. **It is NRND/EOL, and its replacement is already supported.** TDK's own
   recommended successor is the **ICM-20948**, which imud already has a driver
   for, with the same 6-axis-plus-AKM-mag-over-bypass shape. A user asking for
   MPU-925x support can often be answered with "use the ICM-20948" — though
   not if they already own the board, which is the honest counter-argument.
2. **It would be the eighth and ninth unvalidated driver.** ROADMAP §1 already
   lists seven drivers marked `experimental = true` that have never run on
   real silicon. Adding two more (`mpu925x` plus a new `ak8963`) makes that
   matrix worse, not better.
3. **It is unusually hard to validate.** The parts available to buy are
   frequently counterfeit (below), so "it works on my board" is weaker
   evidence here than for any other part in the tree.
4. **It is second-tier on merit.** Against what is already in-tree it has no
   hardware timestamp, the smallest FIFO, and the highest noise floor.

The case *for* building it anyway is real and worth stating: the MPU-9250 is
probably the most widely deployed 9-axis breakout ever made, boards are a few
dollars, and the marginal implementation cost is low because the template
already exists.

### Fit against the driver contract

| `imu_ops_t` field | Value | Note |
|---|---|---|
| `has_fifo` | `true` | 512-byte FIFO, stream mode |
| `has_hw_timestamp` | `false` | no chip sample timer |
| `ts_tick_ns` | `0` | unused when there is no timestamp |
| `supported_odr_hz` | 100 … 1000 | `1000/(1+SMPLRT_DIV)` with DLPF enabled |
| `supported_accel_g` | 2, 4, 8, 16 | |
| `supported_gyro_dps` | 250, 500, 1000, 2000 | |

**No hardware timestamp** is an accepted configuration — `icm20948` is already
`has_hw_timestamp = false`, and [`src/imu.c:331`](../src/imu.c#L331)
re-anchors the wall-clock every FIFO burst when it is. Two things are lost:

- the per-sample `dt` refinement at
  [`src/imu.c:860-870`](../src/imu.c#L860-L870) is skipped, so `dt` stays at
  the nominal period and the correction for oscillator tolerance goes away;
- timestamp correlation (spec §17) degrades from sample-time to read-time.

**The magnetometer needs a new driver.** AK8963 is *not* an AK09916 under
another name — different device ID (`WIA` = 0x48 against 0x09), different
register map, a 14-bit/16-bit output mode select, and, uniquely, a factory
**fuse-ROM sensitivity adjustment**: per-axis constants `ASAX/ASAY/ASAZ` read
in `FUSE_ROM_ACCESS` mode and applied as

```
H_adjusted = H × ((ASA − 128) × 0.5 / 128 + 1)
```

That correction should be applied even though imud fits its own hard- and
soft-iron calibration afterwards. It is a known per-device factory constant;
leaving it out just pushes a diagonal scale error into the soft-iron fit,
where it is estimated less precisely and conflated with the vessel's iron.

> **Correction, found while implementing (2026-07-28).** This list of
> differences was incomplete in the way that mattered most. The AK8963 is also
> **rotated relative to the host gyro/accel**, which the AK09916 is not:
> per PS-MPU-9250A-01 Rev 1.0 Figures 4 and 5 (and DS-000007 Rev 1.0 Figures 4
> and 5, which agree), `AK_X = MPU_Y`, `AK_Y = MPU_X`, `AK_Z = −MPU_Z`.
> Copying `ak09916.c`'s "flip Y only" remap — which this evaluation's framing
> invited — would have silently swapped the horizontal axes and produced a
> heading mirrored about the bow-stern line. The shipped driver derives the
> composed board-frame mapping in a comment, and `test_drivers.c` pins the
> decode. The lesson generalises: for a compass on a host die, check the
> orientation figure, never infer it from a sibling part.

### Three mismatches with imud's shipped defaults

These matter to anyone fitting one of these boards, independent of whether a
driver ever lands.

**1. The FIFO is smaller than the default watermark.** 512 bytes ÷ 12 bytes
per accel+gyro sample-set ≈ **42 sets** (about 36 if temperature is batched
for the `fit-temp` gyro/temperature compensation). imud ships
`imu.fifo_wm = 64`, which this chip cannot reach. It is also a robustness
point rather than just a config one: roughly 42 ms of headroom at 1 kHz means
a stalled reader thread raises `FLAG_FIFO_OVERFLOW` far sooner than on the
ISM330DHCX's much larger FIFO.

**2. The default ODR is not on the grid.** Output rate is
`1000/(1 + SMPLRT_DIV)` with the DLPF enabled, giving 1000 / 500 / 333 / 250 /
200 / 125 / 100 Hz. imud's default of **833 Hz is not reachable**; the nearest
supported value is 1000 Hz, which also costs somewhat more CPU on a Pi.

**3. Do not copy the datasheet noise figure into `mekf_accel_noise`.** This is
the most likely way a user would break their own install after fitting one of
these boards. The MPU-9250's accelerometer noise density (~300 µg/√Hz against
the ISM330DHCX's ~186) converts naively to about `0.0035` — which lands
**inside the divergence region** documented in
[math.md §4.7](math.md) and `man 5 imud.conf`, where attitude RMS is
substantially worse than at either the default or at much larger values.

`mekf_accel_noise` is a **tuned filter parameter, not a per-chip datasheet
transcription.** The same already applies to `mekf_gyro_noise`, which is held
deliberately far above any sensor's floor. A noisier IMU is not a reason to
raise it; if you want to check the measurement model against your hardware,
record a capture and run `imud-cal fit-ra` (see the manual's
"Checking the filter against real water").

### Counterfeit parts — the actual support burden

The MPU-9250 breakout market is full of relabelled **MPU-6500** parts, which
have no magnetometer at all, plus assorted clones. `WHO_AM_I` (0x75)
distinguishes the genuine parts:

| Reads | Part |
|---|---|
| 0x71 | MPU-9250 |
| 0x73 | MPU-9255 |
| 0x70 | MPU-6500 — 6-axis only, no magnetometer |

**Adopted policy, if this is ever built:** accept 0x71/0x73, then verify the
AK8963 answers `WIA == 0x48` through bypass, and if it does not, fail
`probe()` with an explicit message naming the likely cause ("no AK8963 found —
this is probably a relabelled MPU-6500") rather than surfacing a bare I²C
error. Turning the single most common failure mode into a self-explaining
startup message is most of the support cost avoided up front.

### Proposed design, if built later

- **Two files:** `src/drivers/mpu925x.c` and `src/drivers/ak8963.c`, following
  `icm20948.c` and `ak09916.c` structurally. One IMU driver covers both the
  9250 and the 9255 — only `WHO_AM_I` differs — the same one-driver-two-parts
  pattern already used for `lsm6dso` / `lsm6dsox`.
- **Clamp and round, logging both once at init:** clamp `fifo_wm` to the
  chip's real capacity and round the requested ODR to the nearest supported
  value (833 → 1000 Hz), logging each adjustment so a surprised user can see
  why the daemon is not running at the rate their config asked for.
- `experimental = true`; register in `src/drivers.c`; add to `imu_names[]` and
  `mag_names[]` in `test/test_drivers_registry.c`; extend the mock-I²C harness
  (`test/test_drivers.c` with `test/i2c_mock.c`) to cover probe / init / read
  decode off-hardware; add rows to the driver table in `docs/manual.md` §5 and
  to `docs/datasheets.md`; add a line to ROADMAP §1's validation queue.
- Roughly 400–500 lines per driver plus tests and documentation.
