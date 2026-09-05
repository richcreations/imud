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
| **NMEA** | UDP, `[nmea] dest_port` (default 10110) | Heading, pitch and roll parsed from `$PASHR`; rate of turn from `$TIROT`; true heading from `$HCHDT` when present; and the last received sentence verbatim. Heading reads `--` when `$PASHR`'s heading field is null, which is how imud reports that no magnetometer is being fused. |
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
| `imu.probe.reject` | `probe()` returns −1 at an unused reserved address. A driver that passes here but not this one is not checking WHO_AM_I, or is swallowing the I²C error. **SKIPs on SPI**, where chip select does the addressing: the `i2c_addr` field never reaches the wire, so the "bogus" probe reads the same part and returns 0 — the check misfiring rather than the driver failing it. Probing a neighbouring chip select is not a safe substitute, because the foreign register address landed on could be write-only or read-to-clear on that part, and disturbing the other sensor mid-run costs more than the row is worth. This SKIP is **structural** — no report on this transport can ever produce the row, so blocking the `experimental` recommendation on it gated nothing and asked for something unachievable. It no longer blocks; the verdict names it instead, so the gap is still stated rather than silently passed. The rejection path itself is covered off-hardware by `test_drivers`, which sets a wrong WHO_AM_I and requires `probe()` to return non-zero — the same arrangement as `imu.err.no_spurious`. |
| `imu.bus.integrity` | The part's identity register — WHO_AM_I, the byte `probe()` compares — is read 2000 times and must return its hard-wired value every time. A mismatch at the real address is a corrupted transfer, not a wrong part. **Any bad read FAILs**: there is no rate low enough to be acceptable, because the driver reads that byte once in `probe()`, so 0.2% is one probe in five hundred rejecting a part that is physically present. The check runs after `reset()` and `init()`, on a part the driver has just brought up — the state the daemon actually operates in. Both of those were wrong at first and each produced a false result on the reference rig: it read `INTERNAL_FREQ_FINE`, which is not invariant (`0x1B` running, `0x1A` with the sensors powered down, since it reports the trim of an oscillator that can be switched off), and it ran before `init()`, measuring whatever state the previous process left behind. The expected byte comes from the driver registry rather than from the first read, so one corrupt read at the start cannot invert the whole result. Two defects reached the bench disguised as something else before this existed — a `TIMESTAMP0` read 1.58 s high that presented as a timestamp-chain fault, and a trim register reading differently across two `init()`s that presented as a state-dependent `init()`. The mag equivalent is `mag.bus.integrity`, and reading the two together is the point: the parts share SCK and MOSI and differ only in chip select, so both corrupting implicates the shared wiring while one alone implicates that part's own branch — its pull-ups, its chip select, its stub. They are not clocked alike, though: an MMC5983MA declares a 2 MHz maximum against the ISM330DHCX's 10 MHz, so a clean mag beside a dirty IMU narrows the search rather than settling it. |
| `imu.reset.rc` / `.ms` | `reset()` returns 0; elapsed time is recorded. Under 1 ms warns — the datasheet turn-on time is probably not being waited out. |
| `imu.init.rc` | `init()` returns 0 for the configured rate and full scales. |
| `imu.init.regdiff` | Control registers changed across `init()`. The diff is printed raw: decoding it needs the datasheet. |
| `imu.init.idempotent` | A second `init()` lands on a byte-identical register image — catches a bank left selected, a latched enable, or a buffer the second call did not clear. Compared over non-volatile registers only; see *Which registers are read* below. A failure **names the registers** rather than counting them: the note lists them and the register-diff appendix carries the value after one `init()` beside the value after two. A bare count was the original form and proved unactionable — the same "2 registers differ" came back from run after run with nothing in it to chase. |
| `imu.odr` | Measured rate matches the rate the driver reports it will program (`odr_actual_imu()`, the same resolution the daemon uses). On failure the note names which `supported_odr_hz[]` entry the measurement actually matches. **Direction decides how far the excuses reach — not the grade.** A rate *below* the configured one can be the read loop rather than the part — a stalled reader and a missing FIFO both lose samples — so a shortfall is graded down to a WARN when either applies. A rate *above* it cannot be: nothing in the read path invents samples the part did not produce, so those excuses do not apply and an over-rate reading keeps whatever the tolerance ladder gave it. It is *not* promoted straight to FAIL: a continuous-mode oscillator is specified as typical, and the reference MMC5983MA measures 105.4 Hz against a configured 100 Hz on both transports. Grading that a defect would fire on expected silicon behaviour, which is how a check teaches people to skip it. Past `odr_tol_fail` it FAILs either way. |
| `imu.fifo.depth` | Depth grows with the wait, so `read()` drains a queue rather than one sample register. |
| `imu.fifo.overflow` / `.recovers` | Not draining eventually returns rc 1, and reads return to rc 0 afterwards. |
| `imu.seq.monotonic` / `.gapless` | `seq` never repeats or reverses, and gaps appear only where an overflow was reported. Deltas are unsigned, so the 32-bit wrap is handled. |
| `imu.err.nodata_not_error` | An empty FIFO returns 0 with `n = 0`, never −1. −1 is reserved for I²C faults; the daemon resets the chip on a run of them. |
| `imu.err.no_spurious` | 200 back-to-back reads on a healthy bus produce no −1. |
| `imu.noise.accel` / `.gyro` | Per-axis standard deviation is in a plausible band. **Exactly zero fails** — that axis is stuck and is not being decoded. |
| `imu.rest.gravity` | Mean \|a\| is 9.807 m/s². The note flags a power-of-two ratio, which is a wrong sensitivity constant. |
| `imu.temp.plausible` / `.varies` | Temperature is in range and moves. Pinned at exactly 25.000 fails: that is the placeholder, so the word is never decoded. |
| `imu.chipts.presence` | `chip_ts` is 0 if and only if `has_hw_timestamp` is false. |
| `imu.chipts.monotonic` / `.rate` / `.wall` | The counter advances, its period matches `ts_tick_ns`, and chip time tracks wall time across counter wraps. `.monotonic` reports **reversals and repeats separately**, because they point at different code: a repeat means a burst was stamped from one reading rather than per sample, while a reversal means a later sample carries an earlier tick, which on a counter narrower than 32 bits is usually a missing unwrap. Samples whose `chip_ts` is 0 are excluded from both, and from `.rate` — a driver that fails its post-drain timestamp read leaves a whole burst at 0, and comparing those would score a reversal going in and a delta of most of the counter coming out, which is exactly the median `.rate` reports. The appendix prints all four counts so a clean window is distinguishable from one full of skipped zeros. The report always prints the *implied* tick, which is the fastest way to spot a wrong `ts_tick_ns`. `.rate` compares chip time against chip time and so cannot see an oscillator error; only `.wall` can. A ratio **below** 1.0 is a lost counter wrap in the driver's unwrap; **above** 1.0 cannot be — the counter is ticking faster than `ts_tick_ns` claims, which scales every per-sample `dt` the daemon derives from it. `.wall` is graded at ±2% against `imu.odr`'s ±5%, which is the same ratio measured a second way. That is deliberate, and both notes say so: `imu.odr` asks whether `init()` programmed the rate it was asked for, an encoding question with room to spare, while `.wall` asks whether `ts_tick_ns` describes the counter, and `imu.c` multiplies that constant into every `dt` it hands the filter. A part can legitimately pass the first and warn on the second; when it does, `imu.odr` points at `.wall` rather than leaving two verdicts on one number unexplained. |
| `imu.drdy.edges` | The interrupt line produces edges at a rate matching either the per-sample or the watermark model; the report says which one fits. |
| `imu.fifo.watermark` | **Always SKIPs.** Watermark timing is only observable through `int_gpio`, so the check exists to say so and point at `imu.drdy.edges` rather than leave a reader wondering why the watermark is ungraded. |
| `imu.odr.rounding` | **INFO, not graded.** Records the requested ODR against the rate the driver actually resolved it to. Every driver rounds to its own grid, so a difference is normal — what matters is that the report shows the resolved figure, since every rate check below grades against that and not against what the config asked for. |
| `imu.rest.still` | The rest window really was still: peak per-axis gyro sigma under 0.1 rad/s. WARNs rather than fails when the board moved, and **relaxes the noise and gravity checks below it** — the point is to stop a knocked bench being reported as a bad part. |
| `imu.direct.accel` | The FIFO path measured against the part's own output registers, which are upstream of it. At rest both must see gravity in the same direction; when they do not, the sensor is fine and the **decode** is wrong — a framing offset, a byte order, an axis landing in the wrong slot. No other check here can make that distinction: every one of them reads the FIFO and can only say that what came out looks implausible, never whether the silicon or the driver put it there. This is the comparison that identified the FIFO framing defect fixed in `689133e`, where the direct registers read gravity on Z and the FIFO path read it on X — numbers that are individually plausible on both sides. Compared as a **direction**, not in physical units: scaling the raw counts would mean carrying a second copy of every driver's sensitivity table, and every decode fault worth catching moves the direction anyway (an axis swap is 90°, a sign flip 180°). PASSes under 5°, WARNs under 15°. Output registers stuck at zero FAIL separately, with their own note, since an angle cannot express "there is nothing there". SKIPs where the driver declares no direct window in `imt_regmaps[]`, and where a banked part is not answering for bank 0 — the tool reads the bank register but never writes it. |
| `imu.direct.temp` | The same comparison on temperature, which is the most sensitive framing canary on a still bench: bounded, slowly varying, and one register pair wide, so a single byte of offset moves it by tens of degrees. Graded in degrees at a 3 °C tolerance, which covers both the ST parts batching temperature into the FIFO an order of magnitude below the accel rate and the ICM-42688-P carrying only an 8-bit temperature field in its FIFO packet. **Structurally SKIPs on `mpu9250`, `mpu9255` and `icm20948`**, whose drivers read temperature from the direct register for every sample rather than from the FIFO — comparing there would compare a register against itself and report a canary that cannot fail. |
| `imu.regs.after_run` | The control registers re-read once every phase has finished, diffed against the image `init()` left. Every other register check looks at the part BEFORE it has been driven — `imu.init.regdiff` compares across `init()`, `imu.init.idempotent` compares one `init()` against the next — so a driver that configures the part correctly and then loses that configuration during the run is invisible in a report, and every measurement taken after the change was taken on a differently configured part. Volatile registers are excluded by the same mask the idempotency compare builds, so what remains changed with nothing writing it. Any difference FAILs and the registers are named with their before and after bytes. Skipped on the `sim` driver, which has no registers. |
| `imu.fifo.gravity` | Phase C only. Mean |a| from the FIFO across the commanded turns, so the FIFO's ACCELEROMETER half is checked while the board is moving — nothing else does: `imu.direct.accel` runs at rest and the six faces finish before the first turn. Without it, a FIFO that has stopped carrying measurements while still counting sample-sets at the configured rate is indistinguishable from a gyro-specific decode fault, because both surface only as an integral of about zero. Magnitude, not direction, because the board is moving and the FIFO sample is up to a batch older than the direct read; |a| is invariant to orientation and nearly so to a slow hand turn. The band is wide (g +/- 2 m/s^2) on purpose: a hand turn adds real linear acceleration, and what this must catch is a payload that is not an acceleration at all. The direct registers' |a| is reported beside it in raw counts and is NOT graded — there is no sensitivity table for the direct window — but it says whether the part was still measuring while the FIFO was not. |
| `imu.direct.gyro` | Phase C only. The three `gyro.A.sign` checks grade an integral of FIFO samples, and a turn that integrates to nothing has two very different causes they report identically: the gyro is not responding, or the gyro is fine and the FIFO decode is losing it. This samples the direct gyro registers throughout the commanded turns and reports the peak beside the integral, which separates them — and on the MPU parts it matters particularly, because the register map documents the FIFO as continuing to buffer gyro data while the gyro data path is in standby, so plausible bytes in the FIFO are not evidence that the sensor is live. Graded against 5 °/s of the **configured** full scale rather than an absolute count, since the same hand turn is ~490 counts at ±2000 dps and ~3900 at ±250. Registers moving while the FIFO stays flat FAILs and names the FIFO path; neither moving FAILs and names the gyro data path; the FIFO moving while the registers do not WARNs, since the turn may simply have finished between polls. |
| `imu.fs.accel` | Gravity still reads 9.807 after re-initialising at every advertised accelerometer range — catches one wrong constant in one branch. |
| `imu.fs.restore` | The configured full scale is back in place after the sweep above re-initialised the part at every range. FAILs loudly, because a sweep that leaves the part on the wrong range makes every later check in the run untrustworthy rather than merely wrong. |
| `imu.fs.gyro` | **INFO, not graded.** Records the noise floor at every advertised range. It does *not* assert that sigma scales with full scale: that only holds when quantisation dominates the noise floor, and on a good part it does not — an ISM330DHCX at ±125 dps has a 7.6e-5 rad/s quantisation step against a ~1.9e-3 rad/s measured floor, so sigma is flat across every range and a band around the full-scale ratio grades coin flips. The one direction that still WARNs is sigma *falling* by more than half as the range widens, which has no benign reading. |
| `imu.selftest.accel` / `imu.selftest.gyro` | The part actuates its own sensing elements and the response is graded against the factory window from its datasheet — accel 40–1700 mg and gyro 150–700 dps on the ISM330DHCX, measured at the ±4 g / ±2000 dps ranges those figures are quoted for. This is the **only passive check that proves the data path is live.** Every other one here grades numbers the part produced, and a part producing plausible numbers from a dead sensing element passes all of them: at rest a working gyro and a broken one both read about zero, with the same noise floor and the same `seq` and `chip_ts` behaviour. Until this check the only evidence an element responded at all came from the guided rotation phase, which needs somebody at the bench. Reported per axis, in the datasheet's own units, so a response that is present but wrong is visible rather than merely graded. A run whose `imu.rest.still` warned downgrades a failure to a WARN and says so: the measurement is defined on a stationary part, gravity cancels in the difference between the two averages but a rotation does not, so grading a knocked bench as a defect would send somebody after a part that is fine. A driver that reports no window — `0`/`0` — has its response recorded as INFO instead, which is what the three MPU-925x parts do: TDK defines the measurement but publishes its limits only in an application note, so the number is shown and the judgement is left to whoever reads it. Structurally SKIPs where `imu_ops_t::self_test` is NULL, which today is every driver but `ism330dhcx`, `mpu9250`, `mpu9255` and `mpu6500` — a structural skip does not withhold the `experimental` recommendation, so a part with no built-in self-test stays clearable on the evidence of the rest. |
| `imu.selftest.restore` | The configured setup is back after the self-test, which reconfigures the part for the measurement and hands it back with only self-test itself guaranteed off. Same reasoning as `imu.fs.restore`, and it runs even when the self-test failed. |
| `mag.*` | The magnetometer equivalents, plus `mag.nodata_not_error` (not-ready must return 1, not −1), `mag.field_magnitude` (15–150 µT — deliberately wide: it grades whether the driver decodes and scales at all, not whether the sensor is calibrated, since hard iron and the AMR bridge's own offset move raw `|B|` by tens of µT on any real install), `mag.noise` (per-axis standard deviation at rest; **exactly zero fails** — that axis is stuck and is not being decoded, which is a decode bug and never physics), and `mag.wall_ns`. `mag.rate` splits by direction exactly as `imu.odr` does, and for the same reason — an MMC5983MA reading 130 Hz against a configured 100 Hz was the loudest signal in a bench report and was graded a WARN nobody acted on, because the note only ever excused a *low* reading. `mag.init.regdiff` SKIPs on a part whose control registers are write-only (the MMC5983MA): a readback diff is structurally empty there no matter what `init()` wrote, so grading it would blame the driver for a property of the silicon. `mag.drdy.rate` measures the same rate again over the mag's interrupt line — the way the daemon gets it — and grades it against the polled `mag.rate`. The two can disagree by a large factor: where DRDY is a latched interrupt whose acknowledge write also clears the status bit `read()` gates on, a reader blocked on the edge never sees the bit and waits out its timeout for every sample. Every other mag check here polls, so without this one that defect is invisible to the tool while costing the daemon two thirds of its samples. SKIPs when the part has no interrupt pin or `mag.int_gpio` is 0. `mag.drdy.restore` is its companion and appears **only on failure**: measuring the line requires re-initialising the part interrupt-driven, and every check after it polls, so the part is put back. If that restore fails, the checks below it were read through the wrong path and their numbers mean nothing — an operator reading a set of otherwise plausible figures has no other way to know. A driver may latch the mode before the first bus write that can fail, so the restore runs even when the interrupt-mode init did not succeed. |
| `mag.fuse_rom` | AK8963 only: the factory sensitivity constants ASAX/Y/Z, read from fuse ROM. All-zero or all-ones means there is no fuse ROM to read, which is what a counterfeit or dead magnetometer die returns. It earns a row of its own because every other check on the part can pass while it is dead: such a die answers WHO_AM_I, accepts a CNTL1 mode write, and satisfies `mag.probe`, `mag.reset.rc`, `mag.init.rc` and `mag.init.regdiff`, yet never asserts DRDY and never produces a measurement — and the driver's own adjustment arithmetic turns ASA 0x00 into a plausible-looking 0.5x rather than an error, so even the scaling looks sane. One line of report then separates bad silicon from a driver defect, which is otherwise close to unanswerable from a bug report. The part is walked power-down -> fuse-ROM -> power-down and its configured mode written back, because every AKM mode change must pass through power-down. The AK09916 has fixed sensitivity and no fuse ROM, so it does not carry this check. |
| `mag.set_reset` | The degauss pulse is issued, and it is issued **before** the measurements above rather than after them. Ordering is the substance: run last, it left every field and noise figure graded in whatever magnetisation state the part happened to arrive in, which is not a property of the driver. |
| `mag.burst_framing` | The driver's output window read as one burst, and again one register at a time, land on the same bytes. `bus_burst_read()` passes the part's `spi_inc_mask` only when `len > 1`, so the two take different paths through the command byte — the burst asserts the auto-increment bit, the singles do not — and a wrong mask makes them disagree, or makes the burst repeat one register. This is the on-hardware half of what `test_drivers` proves against the mock. It does **not** see a fault inside `spi_burst_read()` itself: `bus_reg_read()` is `bus_burst_read(len=1)`, so both sides share it, and a bench comparison against a single full-duplex transfer settled that separately. The window is live, so the singles are bracketed between two bursts that must match — otherwise a measurement landed mid-read and the comparison means nothing. SKIPs where no output window is declared in `imt_regmaps[]`, and where no quiet window turns up. The raw bytes go in §5.7, because a verdict cannot tell an off-by-one shift from a pointer that never moved. Like `mag.degauss.differential`, deliberately **not** in the `experimental`-clearing set: only two drivers declare an output window today, so requiring it would block every other part for a reason that has nothing to do with that part. |
| `mag.degauss.differential` | Splits one reading into the field it measured and the bridge offset it carried, by measuring once after SET and once after RESET. SET and RESET magnetise the AMR film opposite ways, so between them the field term changes sign and the offset does not: `field = (vS − vR)/2`, `offset = (vS + vR)/2`. Graded on the **field** (25–65 µT); the offset is always reported. This is what tells a part looking at real iron apart from a part failing to remove its own bias — one symptom, opposite causes, and no other check on a single-transport bench can separate them. SKIPs where `mag_ops_t::degauss` is NULL, which is every part but the MMC5983MA today. Deliberately **not** in the `experimental`-clearing set: it is a diagnostic, and requiring it would block every driver that cannot drive RESET. |

### Phases B–D — guided

| id | Asserts |
| --- | --- |
| `face.N.sign` | The dominant axis and its sign match the expected value for that orientation. A wrong sign is diagnosed as a missing flip; a wrong axis as a swap. SKIPs when that face's `.mag` failed — the axis of a vector that is not gravity is not evidence about the frame. |
| `faces.frame` | Rollup: the board frame is NED (X forward, Y starboard, Z down). SKIPs when any face came back at a magnitude that is not gravity: a remap permutes and negates axes and cannot change \|a\|, so such a face is not a rotation of gravity and can say nothing about the remap. Blaming one there would send a reader to rewrite correct code. SKIPs equally when a face collected too few samples to judge — a rollup that speaks for six faces must have measured six. |
| `faces.symmetry` | **INFO, not graded.** SKIPs when any face failed its magnitude check, since a scale fitted through readings that never saw gravity is meaningless, and when a face was never measured at all. Derives per-axis offset and scale from the six faces — the same model `imud-cal accel` fits. Reported rather than graded because it is a calibration reading, not a pass/fail property of the driver; a scale far from 1.000 points at a sensitivity constant, an offset at a mounting. |
| `gyro.A.sign` | A commanded positive turn integrates positive, per the right-hand rule. **Fails outright below a quarter of the commanded angle** rather than grading the sign there: the sign of an integral near zero is whichever way the noise fell, so without the floor a gyro producing nothing PASSes on all three axes — and all three are in the required set below, so the run then recommends clearing `experimental` on a dead data path. The floor sits under `.scale`'s FAIL band, so a gyro that responds but is badly scaled still has its sign graded. |
| `gyro.A.scale` | The integrated angle is within 20% of commanded. Deliberately loose — it catches factor errors (57.3× for deg/s, 1/57.3 for a double conversion), not sensitivity. |
| `spin.frame_agreement` | The magnetometer heading and the gyro Z integral agree in direction and magnitude. Disagreement in direction means a mag axis is inverted relative to the IMU — the most common magnetometer-driver defect. The heading is taken about the centre of the swept locus, not the origin: hard iron offsets that locus, and once the offset exceeds the field radius a heading measured about the origin stops winding altogether, so the total comes out near zero however far the board turned. SKIPs when the heading moved less than half the gyro's angle — that is a failure to measure, not an inverted axis, and the inversion diagnosis is only printed when the heading actually tracked the turn and ran backwards. |
| `spin.magnitude` | Mean \|B\| while turning is 15–150 µT, the same band `mag.field_magnitude` uses at rest — and the width matters more here, because averaging `|R·B + offset|` across headings is biased upward by hard iron, so a turning sensor reads higher than a stationary one on the same rig. Measured across the whole turn, so unlike the resting figure it cannot be flattered by one lucky orientation. |
| `spin.coverage` | The turn visited the whole heading circle (24 sectors of 15°). |
| `spin.axes_vary` | X and Y each swept at least half the horizontal field through the turn, and Z is not frozen. The asymmetry is deliberate: a **level** spin barely changes the dip component, so requiring Z to vary would fail a correct turn — Z must merely not be *constant*, which only a stuck axis is. A stuck Z FAILs; X or Y barely moving WARNs, since an incomplete or tilted turn produces the same reading as a defect. |
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

One class of miss cannot be fixed by reading harder, and those registers *are*
listed. A **saturated counter** reads its maximum on every pass, so the
experiment concludes it never moves. `FIFO_STATUS1`/`FIFO_STATUS2` on the ST
parts carry `DIFF_FIFO`, and above roughly 833 Hz the FIFO refills to capacity
between passes: the counter reads identically every time, is classified static,
and then `init()` flushes the FIFO and `imu.init.idempotent` reports "2
registers differ". That is a question about whether the FIFO was emptied
wearing the costume of a question about `init()`, and it cost a bench
investigation chasing a driver defect that did not exist. The TDK parts carry
the same counter in a different register file: `FIFO_COUNTH`/`FIFO_COUNTL` at
0x72–0x73 on the MPU-925x and 0x70–0x71 on the ICM-20948, whose drivers both
flush the FIFO inside `init()`. Registers in that class are declared per part,
because the defining property is that inference cannot reach them.

A **data-ready register** is the other one, and there the sweep's own read is
what hides it. `STATUS_REG` on the ST parts clears when the output registers
are read, which every pass does; the TDK `INT_STATUS` words are marked
read-to-clear and the pass clears them directly. Either way the next sample
sets the bit again, so all four passes read the same byte, the register is
classified static, and then `init()` reconfigures the part and the second
snapshot catches it on the far side of a sample. That surfaces as an
*intermittent* `imu.init.idempotent` WARN naming a register no `init()` ever
writes — intermittent because whether a sample is pending at that instant is a
race. The RM3100 arrives at the same place from the opposite direction: its
`DRDY` clears only on a read of the measurement registers, which the
destructive-window exclusion keeps out of the sweep, so the bit latches high
and never moves at all. Only the register the part sets *per sample* is
declared; the event sources beside it — free-fall, wake-up, tap, 6D, WOM —
belong to functions no driver here enables, so they hold 0, the experiment can
see them perfectly well, and they stay in the compare.

The list is deliberately tiny and is not a general volatile table — everything
the experiment *can* find, it still finds. **Check the addresses against the
part's own register map**: the FIFO count and the FIFO data port sit next to
each other and swap places across this family, and declaring the port volatile
instead would leave the counter in the compare and put a destructive read in
the sweep.

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
