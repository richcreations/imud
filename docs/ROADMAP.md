# imud — Roadmap & Deferred Work

Items identified from 2026-07-06 onward that remain open. Completed work is
tracked in git history, NEWS, and spec.md §6, not here.

Sections 1–9 predate §10 and §11, which were added 2026-07-25 and carry the
filter-mathematics and aerospace items left open after 1.7, plus findings from
measuring them. Where the two overlap, the older section is authoritative and
the newer one cross-references it.

## 0. Direction for the next 12 months  *(2026-08 → 2027-08)*

Sections 1–11 are the full backlog. This section is the forward plan: what the
project intends to do over the coming year, and — just as deliberately — what
it intends *not* to do. Items name the section that carries the detail.

**Measurements live in those sections, not here.** This one says what is left
to do and points at where the numbers are. It used to restate them, and every
time a section below advanced, its summary here went quietly stale — §0 claimed
SPI latency was unmeasured for as long as it took someone to notice, and read
"ten drivers" against §1's eleven. A forward plan that carries figures is a
second copy of them with nothing keeping it honest.

**Planned.**

- **Clear the hardware validation backlog (§1).** Eleven drivers ship marked
  `experimental = true` and have never met real silicon. `imud-imutest` reduces
  each one to a single command plus three physical checks, so this is bench
  time rather than design work. The reference pair is validated; the rest clear
  opportunistically as boards become available.
- **Confirm the 1.7/1.8 changes on hardware (§1).** The alignment window and
  the measured sample clock are confirmed, and the `m33_inv` accel-update duty
  is now confirmed too — 4 refusals in 2,381,189 accel updates, 0.0%
  (2026-08-25). Figures in §1.
- **Fit real gyro temperature coefficients (§2).** The mechanism shipped in
  1.5; what is missing is a cold-boot-to-warm capture from real hardware.
- **Finish measuring sample latency on silicon and fix spec §14 (§3.1).**
  All three budgeted terms are now measured, on both transports; `spec.md` §14
  marks the two it does not meet. What remains is not a number but a model:
  FIFO residence is bounded by the drain cadence rather than by `fifo_wm`, and
  the watermark's effect on it inverts between two otherwise identical runs, so
  residence is not yet predictable *from configuration* — which is what a
  rewritten budget would have to be derived from. Ties to §1.1's DRDY item.
  Same bench session as §1 and §2. Figures in §3.1.
- **Confirm 1.9.0's timestamp sourcing on silicon (§1.1).** The measured
  clock and the post-drain forward guard have both now been seen working in a
  running daemon. Two follow-ons still rest on datasheet behaviour no hardware
  has confirmed: the ST tag-`0x04` payload layout, and whether `TAG_CNT`
  participates for it. Both are cross-checked at runtime and degrade rather
  than corrupt, but a bench pass is what turns "degrades safely" into "works".
- **Build the two missing benchmark scenarios (§10.3, §10.5).** Both remaining
  filter items are blocked on scenarios that do not exist — a vibration case
  and a calm-water case. The scenario *is* the work: 1.7 refuted two
  plausible-looking filter changes by measuring them properly, and neither
  remaining change will be attempted against nothing.
- **Finish the consolidation items that are not hardware-blocked (§8).**
- **Further output bridges as demand appears (§5).** WebSocket/SSE, Foxglove,
  and OSC are the shortlist — all additive, none touching the core daemon.

**Explicitly not planned in this window.**

- **A direct NMEA 2000 bridge (§5).** Covered through Signal K, which owns the
  device-level work a C bridge would have to reimplement. Revisit only if an
  appliance install with no Signal K server actually materialises.
- **ROS2 inside this repository (§5).** It needs ament/colcon and cannot build
  under this Makefile. It belongs in its own project reusing the client
  library, as `imud-arduino` does.
- **The aerospace generalization (§11).** Scoped and recorded, deliberately not
  scheduled: marine and robotics are the paths with users today. §11.1 blocks
  the rest and would be picked up against a concrete need, not speculatively.
- **Multi-IMU redundancy / federated Kalman filtering (§6, §11.5).** The
  largest item in the backlog. Not undertaken without an explicit redundancy or
  certification requirement, and a design document first.
- **Barometer and air-data aiding (§11.3, §11.4).** Each needs a whole new
  driver class or input channel, not a new fusion update step.
- **Anything already measured and refuted.** Retuning `Ra` (§10.1), the Huber
  covariance variants (§10.2), and GNSS dual-antenna heading (§10, "Reviewed,
  no action") are closed with recorded measurements, not open questions. They
  reopen only against new evidence.
- **Geometric refinement of the magnetometer fits (§10.4).** Skipped unless
  coverage-gated algebraic fits stop meeting accuracy targets in the field.

**Beyond this window.** The one dated commitment is the WMM2030 data refresh
(§7), expected from NOAA/NCEI around December 2029.

**Cadence.** Versions are `MAJOR.MINOR.PATCH` from 1.9.0 onward
(`docs/RELEASING.md`). There is no date-driven release schedule: a release goes
out when there is something worth shipping, and the hardware-blocked items
above move on bench availability rather than on a calendar.

## 1. Hardware validation matrix  *(bench task — requires the Pi and real silicon)*

Eleven drivers are implemented but marked `experimental = true` and have never run on
real hardware: `icm20948`, `ak09916`, `icm42688p`, `lsm6dso`, `lsm6dsox`, `lis3mdl`,
`lis2mdl`, (added 2026-07-28) `mpu9250`, `mpu9255`, `ak8963`, and (added 2026-08-10)
`rm3100`. Each needs a bench
pass (probe/WHO_AM_I, init, ODR verification, FIFO/DRDY behavior, sane values in all
orientations) before clearing its flag.

**The bench pass is now a single command.** `imud-imutest` (shipped in
`imud-utils`, 2026-07-28) runs exactly that checklist against any registered
driver and writes a Markdown report designed to be pasted into an issue:

```
sudo systemctl stop imud
imud-imutest --imu-driver icm20948 --mag-driver ak09916 --mag-addr 0x0C --all
```

It covers the passive half automatically (probe, reset timing, register
readback, measured ODR, FIFO depth/overflow/recovery, `seq` monotonicity, the
error-return contract, noise floor, temperature, `chip_ts`, DRDY edges,
full-scale sweep) and walks the operator through the three physical checks a
mock cannot do: six-face accelerometer signs, gyro rotation direction, and a
magnetometer spin that cross-checks the mag frame against the gyro. Clearing a
flag means reading one of those reports, not re-deriving the checklist. The
driver contract in the driver guide in `docs/manual.md` remains the reference
for what each check is asserting. Two of
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

**Still owed from 1.7, and now overdue.** 1.7 changed startup and the
accelerometer update path in ways no hardware has seen: `align_window_sec`
moved the alignment window ~1 s → 5 s, and the `m33_inv` fix took accel-update
duty from ~13% to ~100% (both §10.5). 1.8 then changed the sample clock itself
— the chip timer's period is now measured rather than taken from
`ts_tick_ns` — which only real silicon can confirm. `test_ring`,
`test_concurrency`, `test_drivers`, `test_drivers_registry` and `test_imutest`
cannot build on the macOS dev host, so CI and the Pi are the only places they
have ever run.

**`align_window_sec` is confirmed** *(2026-08-18, over SPI, 6/6 runs)*. The
daemon logs `[fusion] settling 5 s — discarding 4165 samples` and then
`aligned from 4165 accel + 200 mag samples`. 4165 is 5 x 833 exactly; the
hardcoded window 1.7 replaced would have read 833. That closes the alignment
half of this item.

**A 10-minute stationary run is clean on resources and diverging on heading**
*(2026-08-19, SPI, capture off, uncalibrated)*. Resources first, because they
are the boring half and they are perfect:

| | |
|---|---|
| open fds | **9, constant** across all ten minutes |
| RSS | **2512 kB, constant** — not one page of growth |
| `ovf` | 0 |
| warnings in 10 min | **one**, a single batched-timestamp fallback |
| rates | 866 Hz IMU, 105 Hz mag, sustained |

The filter is the other half. The attitude covariance trace grows
**monotonically** — `3.8e-02, 4.0e-02, 6.5e-02, 1.2e-01, 2.0e-01, 3.1e-01,
4.5e-01, 6.1e-01, 7.7e-01, 9.0e-01` rad² at one-minute marks — the
`converged` flag never sets in ten minutes, and heading slides 310.0° → 295.0°,
about 1.5°/min, on a board that did not move. Reproduced in a second run
(3.8e-02 → 1.4e-01 over 240 s).

This is the divergence this section already predicts for an uncalibrated
magnetometer, now measured rather than reasoned about: with no cal.json the
heading updates cannot hold yaw, yaw is unobservable from anything else, and
its variance grows without bound. **The daemon is honest about it** — it never
claims `FLAG_FUSION_CONVERGED`, so a consumer checking the flag is not misled.

Two things follow. The magnetometer swing below is not a nicety, it is the
precondition for any heading number from this rig being worth reading. And the
`m33_inv` duty check being blocked behind it is the same fact from the other
side, not a separate obstacle.

**The `m33_inv` accel-update duty is confirmed on hardware** *(2026-08-25)*:

```
accel updates   2381185 used, 4 skipped (0.0%)
```

**4 refusals in 2,381,189 updates.** The instrument is `imud-cal fit-ra`, whose
probe runs the same `m33_inv`, and the prediction was ~0% fixed against ~87%
broken. Measured 0.00017%.

It did not need the swing after all, and the reason is worth keeping. The
refusal to run without a magnetometer calibration is not about heading
*accuracy* — it is about the covariance diverging when yaw is unobservable, so
that every innovation statistic measures the divergence instead. Any calibration
that makes `has_mag` true satisfies that, because the yaw update then executes
and the covariance stays bounded; the bench calibration of 2026-08-24 does, even
though its absolute heading is wrong (see §1.4). `fit_ra` gates on
`n_mag_used == 0`, which is exactly the right condition, and it passed.

A dead-calm berth turned out to be the correct instrument rather than a
compromise. `n_skip` counts three refusal paths, and in a seaway the `|a|`
amplitude band alone discards 66–95% of samples, which would swamp the
`m33_inv` contribution completely. On flat calm the band contributed 4 samples
in 2.4 M, leaving the singularity test as the only thing that could have
refused. It refused nothing.

Two figures worth carrying from the same run: the gravity-direction residual is
0.044° of tilt with a correlation time of 0.001 s — white, so the wave state has
absorbed the correlation — and mean NIS is 0.01 against a target of 1. The
latter is the shipped tuning being conservative for a flat berth rather than a
defect: `mekf_accel_noise` is sized for a seaway, and `fit-ra` says as much
itself ("well below 1 means calmer"). It is not evidence about §10.1, which
concerns the opposite failure.

**1.8's measured sample clock is confirmed in production, twice over.**

- The part's declared trim reaches the daemon: `ism330dhcx declares a 24027 ns
  timer tick against the 25000 ns typical (-3.89%); using the part's own
  value`. This is the *declared* figure being honoured, not `ts_anchor_t`
  measuring the period at runtime — that half is still unverified.
- The forward guard added in this cycle fired on real silicon, once in a 90 s
  run: `ism330dhcx: 1 post-drain timestamp read(s) implausibly far ahead;
  extrapolating from the previous burst`. First sighting outside imutest, and
  it is the corrupt-counter-read case reaching the daemon in production rather
  than a synthesised one.

### 1.1 Timestamp sourcing, from the 1.9.0 RC bench run  *(defects confirmed on silicon 2026-08-14; fixes confirmed 2026-08-15)*

The 2026-08-10 Pi 5 session on the reference pair produced four findings. Two
were shipped fixes (the ARM seccomp filter, the gpio udev trigger); the rest
are below.

**All three are now confirmed on hardware** by the 2026-08-14 run: `chip_ts`
monotonic with 0 reversals, the batched FIFO timestamp **accepted** on this die
with no fallback logged, `imu.chipts.wall` **0.9998** against 1.041 before the
part's own `FREQ_FINE +27` trim was applied, and the header reading
`ts_tick_ns = 25000 typical, 24027 from this part`. The bench notes each item
still owes are kept below for the next part that is not this one.

**`chip_ts` monotonicity depends on what paces the drain** *(measured
2026-08-19 on the reference pair over SPI, and the reason an acceptance run can
FAIL a part that is clean)*.

| drain paced by | reversals |
|---|---|
| FIFO watermark interrupt — `[imu] int_gpio = 17`, the shipped configuration | **0 / 53,708 samples** |
| 10 ms timer — `[imu] int_gpio = 0` | 1 / 53,707 |
| `imud-imutest --passive` (5 ms poll loop) | 2, 5 and 6 per ~4,300-sample window across three runs; a fourth was 0 |

Every timer-paced run logs the same pair, and neither appears in the
interrupt-paced one:

    ism330dhcx: N burst(s) whose batched FIFO timestamp failed its check
                — using the post-drain TIMESTAMP0 read for chip_ts
    ism330dhcx: N post-drain timestamp read(s) implausibly far ahead;
                extrapolating from the previous burst

So the chain is: a drain paced by a timer rather than by the FIFO's own
watermark arrives at an arbitrary phase; the batched timestamp fails its
plausibility check; the driver falls back to the post-drain `TIMESTAMP0` read,
which assumes the last sample in the burst was taken at the instant of the
read; and when that assumption is wrong by more than a burst, the next burst
lands before the previous one ended. **A poll cannot know how old the sample it
just read is** — that is the whole of it.

Three consequences, and they point in different directions:

1. **The shipped configuration is clean.** 0 reversals in 53,708 samples, with
   neither fallback logged. The item this section owes is *confirmed* for any
   install with an IMU interrupt line.
2. **An interrupt-less install is exposed**, rarely — 1 in 53,707 here. Both
   guards fire and degrade rather than corrupt, so this is a quality-of-anchor
   issue and not a correctness hole, but it is real and it is not imutest-only.
3. **`imud-imutest` grades the driver on evidence its own drain manufactures.**
   Its drain is `drain_once(); sleep_s(0.005);` with no edge wait, so
   `imu.chipts.monotonic` can FAIL a part that the daemon is timestamping
   perfectly at that same moment. This is the mirror of the magnetometer defect
   this release fixed: there the tool polled and so could not *see* a defect;
   here the tool polls and so *invents* one. Open question — whether the check
   should pace its window from `[imu] int_gpio` when one is configured (measure
   it the way the daemon gets it, as `mag.drdy.rate` does), or keep polling and
   grade a timer-paced reversal as WARN with the mechanism named.

**That run also found two defects in what happens to a timestamp after the
driver produces it** — both reaching `ts_wall_ns` and neither reaching the
filter, so neither was visible in attitude:

- **The anchor paired the newest sample's `chip_ts` with the midpoint of the
  burst read.** Correct while `chip_ts` meant "the counter as read *after* the
  drain"; batching made it mean "when the sample was taken", which is earlier
  than the read began, so every reconstructed timestamp landed late by about
  half a read — 2–3 ms at 833 Hz over I²C, more at deeper watermarks or slower
  buses. `imu.chipts.wall` cannot see it: that check grades the chip/wall
  *ratio*, which a constant offset leaves at 1.0000. Fixed by pairing with the
  pre-read instant (`anchor_wall_ns`).
- **`chip_to_wall` took the tick delta as unsigned**, so a sample *older* than
  the anchor became a ~2³²-tick jump forward — 29.8 hours at 25 µs/tick. Not a
  corner case: the reader anchors on the newest sample of a burst and pushes
  the burst afterwards, so roughly eight samples a minute at 833 Hz shipped a
  timestamp a day in the future. The dt clamp and the anchor-generation check
  reject the discontinuity before the filter, and the latency histogram drops a
  sample whose `wall` is after its own read stamp — so it reached the wire and
  nowhere else. **Pre-existing, not a 1.9.0 regression.** Fixed by taking the
  delta signed.

The bench proposed the first from its effect on sample latency and had the
direction inverted — an offset of this shape *deflates* the FIFO-residence term
rather than inflating it. What actually explained their headline number was
§3.1's model and the histogram's bucket resolution, both documentation.

**Both fixes confirmed on silicon by the 2026-08-15 run**, by a measurement the
2026-08-14 session had called impossible from outside the daemon — sample age
on the wire, `now - ts_wall_ns` over the TCP stream:

- The unsigned-delta defect is **absent**: 0 grossly mis-timed packets in
  99,961 over 120 s, where a deliberately broken build produces one in 45 s. A
  real negative, not a blind instrument.
- The late-anchor defect is **fixed**: age `p50 = 33.79 ms` against that run's
  `fifo` p50 + `pipe` p50 of 32.93 ms — the 0.86 ms socket hop, and *above* the
  sum rather than below, which is the direction that distinguishes a corrected
  anchor from the old one.

The deflation that hid the first defect also explains an anomaly §3.1 had
recorded as unexplained; see the retraction there.

- **Batch the FIFO's own timestamp instead of back-calculating** — *done*.
  `ism330dhcx`, `lsm6dso` and `icm42688p` timed a burst by reading the live
  counter *after* the drain and stepping back one sample period per sample.
  That register reads "now", and the lag to the newest sample moves with bus
  and scheduler jitter, so bursts overlap — measured at 2–4 `chip_ts` reversals
  per 5 s window. `src/drivers/chip_ts.h` enforced monotonicity; the estimate
  itself is now fixed at source.

  ST parts batch the counter into the FIFO as tag-`0x04` words
  (`src/drivers/st_fifo_ts.h`) and anchor the burst on one. The ICM-42688-P
  uses bytes `[14:15]` of each packet, which it was already reading and
  discarding. Both fall back to the old path, whole, when their checks fail.

  Two of the three original blockers dissolved on a closer read, and the third
  was designed around:

  - *Ordering.* DS13012 never says whether the timestamp word precedes or
    follows its sample set — and never has to. `FIFO_DATA_OUT_TAG` carries
    `TAG_CNT`, "a 2-bit counter which identifies sensor time slot"
    (Table 158), so the word is matched to its slot rather than its position.
  - *Watermark arithmetic.* Real only at `DEC_TS_BATCH = 01` (+50% words). At
    `11` (÷32) it is **+1.6%**, so a 128-word watermark holds ~63 sample-sets
    instead of 64 and §3.1's figures stay comparable. Below `fifo_wm = 8`
    nothing is batched at all.
  - *The ICM's 16-bit stamp.* Unwrapped in-burst against the 32-bit `TMSTVAL`
    read, walking newest to oldest. Only *adjacent* samples need to be under
    one repeat apart, which is a property of the ODR — so 12 Hz (78125 ticks
    against a 65536-tick repeat) keeps the back-calculated path, decided once
    at init.

  **Owed to the bench.** The tag-`0x04` payload layout (LE-32 in the X/Y
  bytes) is ST's and Linux's convention, not a documented one, so it is
  cross-checked against the post-drain register read at runtime rather than
  trusted; a rejection counter logs at 1, 10, 100. Confirm it reads zero.
  Confirm `TAG_CNT` actually participates for tag `0x04` — if it does not, the
  match degrades to off-by-one-sample (1.2 ms at 833 Hz, constant across a
  burst, so `dt` is unaffected). And confirm `imu.chipts.monotonic` still
  reports zero reversals, now for the right reason.

- **Derive `ts_tick_ns` from `INTERNAL_FREQ_FINE`** — *done*. The ST parts
  report their own timebase error in 0.15% steps at register `0x63`, with
  `TS_Res = 1/(40000 + 0.0015·FREQ_FINE·40000)` (DS13012 §9.41). `imu_ops_t`
  gained an optional `ts_tick_ns_actual` hook, resolved once after `init()`;
  `src/drivers/st_freq_fine.h` does the arithmetic for all three ST
  descriptors. The bench part's +27 becomes a 24027 ns tick against the 25000
  typical.

  This turned out not to be the principle-only item it was filed as. §3.1's
  latency instrument produced no data at all in the 2026-08-11 matrix because
  `chip_to_wall()` falls back to the declared tick until `ts_anchor_t` has two
  anchors 20 s apart, and at 4% fast the extrapolated sample time outruns the
  read stamp within seconds. Getting the declared value right is what makes
  the first minute behave like the rest of the run.

  Also found and fixed: the **ICM-42688-P's declared tick was wrong by 6.7%**.
  DS-000347 §12.7 scales the timestamp counter by 32/30 whenever the part is
  not clocked from CLKIN, which is every configuration this driver programs —
  so `ts_tick_ns` is 1067, not 1000, and `ticks_per_sample` divides 937500.

  **Owed to the bench.** That `FREQ_FINE` reads about +27 on the reference
  part, and that `imu.chipts.wall` now sits at ~1.0000 rather than 1.041.
  Whether the ICM's `TMSTVAL` shares the FIFO field's 32/30 scaling is not
  stated in DS-000347; `imu.chipts.wall` prints the implied tick, so one run
  settles it.

  **Deliberately not done.** `FREQ_FINE` scales the *effective ODR* by the
  same factor — `ODR_Actual = (6667 + 0.0015·FREQ_FINE·6667)/ODR_Coeff`
  (Table 139) — which is why the bench measured 866.7 Hz against a nominal
  833. Correcting `actual_odr_hz` for it is defensible but much wider: that
  value is also the requested-rate contract that config validation, the MEKF
  tuning, the latency publish gates and the generated documentation tables all
  key off, and it is resolved before the bus is open. `ts_anchor_t` measures
  the true sample interval at runtime regardless.

- ~~**The DRDY edge rate fits no model**~~ — **answered 2026-08-16, and the
  question was malformed.** `imu.drdy.edges` had reported ~18.3 Hz on the
  reference IMU at 833 Hz with `fifo_wm = 64`, against word accounting that
  predicts about 13.4 Hz (2 words per sample-set at the measured 866.7 Hz,
  plus temperature at 12.5 Hz and the ÷32 timestamp words' 1.6%, against a
  128-word watermark). Three candidates were ruled out by inspection: it is
  not a stalled FIFO, because the tool drains while counting; it is not
  double-counting, because tool and daemon both request rising edges only; and
  the remaining hypothesis was that `INT1_FIFO_TH` is a *level* condition
  whose level oscillates across the threshold during a drain.

  `imud-imutest` counts twice over the same window — once draining on every
  edge, once not — which is what separates those: a level condition asserts
  once and stays asserted with nothing emptying the FIFO, while an
  edge-per-sample line keeps pulsing either way. The second number, identical
  across three consecutive runs at that configuration:

  | | edges | rate |
  |---|---|---|
  | draining on every edge | 41 in 3.0 s | 13.7 Hz |
  | **not draining** | **1 in 3.0 s** | **0.3 Hz** |

  One edge in three seconds with nothing emptying the FIFO is a level
  condition, and it refutes the edge-per-sample reading outright — that line
  would have pulsed ~2500 times in the same window. The tool classifies this
  itself (`src/imutest.c`, the `idle <= 2` branch) and prints "level
  condition, so the drained count is drain-paced, not a rate", so the next
  part to be tested says which model it fits without anyone re-deriving this.

  **That is why no model fitted: the drained count was never a rate.** It
  counts how often the drain loop took the level back under the threshold and
  the FIFO re-crossed it, which is a property of the loop's cadence against
  the fill rate, not of the interrupt. Asking which interrupt model produces
  18.3 Hz had no answer because the premise was wrong.

  The drained figure now sits at 13.7 Hz, one edge per fill-to-watermark
  cycle, which is what a level condition with a complete drain gives — hence
  its agreement with the word accounting. **What moved it from 18.3 is not
  determined**: the tool changed (it now drains differently and counts twice)
  and so did the driver (batched timestamps add words), and separating those
  would need the old build back. It is not worth doing. A partial drain
  re-crossing the threshold early is the expected way to get a number above
  the fill cycle, the mechanism is understood, and the check no longer grades
  a part down for it — 0 edges still FAILs, and a rate above the part's own
  sample rate still WARNs.

### 1.2 The MMC5983MA over SPI — one root cause, not five  *(resolved 2026-08-19)*

**The SPI mode was wrong.** This is a MEMSIC part and wants mode 0; it was
driven in mode 3, the ST convention `src/drivers/bus_io.h` is built around —
whose own comment warns "A part that ever disagrees cannot use these helpers
unchanged." Mode 3 corrupts this part's **writes** while leaving most reads
intact, so it never looked like a bus fault. It looked like working silicon
with a list of odd habits, and that list was written down here as five
separate defects over four days.

The section title was the clue nobody read as a cause: **over SPI**. SPI for
this pair landed 2026-08-08 (`7a76bdb`); every "quirk" below was recorded on
2026-08-15 or later. The part had run on I²C since the project began with none
of them — and I²C has no clock polarity or phase to get wrong.

Measured with a standalone probe, same rig, same wiring, only the mode changed:

| CM_Freq | 001 | 010 | 011 | 100 | 101 | 110 | 111 |
|---|---|---|---|---|---|---|---|
| **mode 0** | 1 | **11** | 21 | **53** | 106 | **211** | 1205 |
| mode 3 | 1 | 130 | 130 | 130 | 130 | 256 | 1690 |
| nominal | 1 | 10 | 20 | 50 | 100 | 200 | 1000 |

What each recorded "defect" actually was:

| | recorded as | what it was |
|---|---|---|
| **B** | a `CTRL0` write also lands in `CTRL1`, setting X-inhibit | mode 3. In mode 0, `CTRL0 = INT_EN` leaves X measuring at full sigma. The `g_ctrl1` shadow and "write CTRL1 last" rule are gone |
| **D** | `CM_Freq` 010/100/110 ignored, part free-runs | mode 3. All seven codes work; the three deleted rates are restored |
| **1.2.1** | the status gate and the DRDY edge are mutually exclusive | mode 3. `Meas_M_Done` returns **10/10** on a quiet bus 14 ms after acknowledgement in mode 0, and **0/10** in mode 3 |
| — | a SET as one 3-byte write drops the part to 1 change/s | mode 3. In mode 0 it measures 106/s, same as the two-write form |

**A** (the continuous-mode settle) is not disproved and the 100 ms settle stays.

Two things came out of it that are real and are kept: `actual_odr_hz`, because
even in mode 0 the rates run 6–10% above nominal and `CM_Freq 111` gives 1205
against 1000; and the staleness guard, which protects against a dead INT line
regardless of mode.

**A second, separate defect fell out of the fix.** The IMU and the
magnetometer commonly share one SPI controller (`spidev0.0` and `0.1` here),
and a controller has one clock: mode 0 idles it low, mode 3 high. Leaving the
ISM330DHCX on mode 3 beside a mode-0 magnetometer broke the bus outright — the
daemon opened both parts, settled, and never produced a sample, with nothing in
the log to say why. Both ST parts document "compatible with SPI modes 0 and 3"
(DS13012 §5.1), so both now declare mode 0, and `imu.c` refuses a mixed-mode
pair on one controller at startup rather than letting it corrupt silently.

| | what it is |
|---|---|
| **A — continuous-mode settle** | a control write within ~40 ms of enabling continuous mode leaves the bridge saturated |
| **B — `CTRL0` write alias** | a `CTRL0` write also lands in `CTRL1`, so `init()`'s own `INT_en` sets X-inhibit and the X axis stops measuring |
| **C — bandwidth after the mode started** | `CM_Freq`'s meaning depends on BW, so starting continuous mode first intermittently fails to start it at all at 1000 Hz |
| **D — three advertised ODRs unhonoured** | `CM_Freq` 010/100/110 are ignored; the part free-runs while fusion sizes noise for the requested rate |

B was invisible underneath A, and would have survived A's fix as a green
`mag.field_magnitude` hiding a dead axis — which is what the "fixed" run of
2026-08-15 actually reported.

**B — a `CTRL0` write also lands in `CTRL1`.** Three confirmations, each
reproducing a `CTRL1` semantic from a `CTRL0` write:

| `CTRL0` write | observed | matches `CTRL1` |
|---|---|---|
| `0x04` (INT_en) | X freezes, sigma exactly 0.000 | X-inhibit |
| `0x18` | Y and Z freeze, X live | YZ-inhibit |
| `0x80` | continuous mode stops | SW_RST |

`CTRL0` still receives the write — SET and RESET fire and the field term inverts
— so it is both registers, not the wrong one. **Specific to `CTRL0`, so it is a
register quirk and not something about what imud puts on the wire:** a `CTRL1`
write does not disturb `CTRL2` (106 output changes/s straight after one, which
clearing `Cmm_en` would have stopped), a `CTRL2` write does not fire `CTRL3`'s
self-test coil, and a `STATUS` write does not set `TM_M`. Not bus timing either —
identical at 10 MHz, 1 MHz and 100 kHz. No published erratum exists and Rev A
pp.6-7 describe the write framing exactly as the tree implements it, so the
measurement itself is recorded in the driver. Fixed by writing `CTRL1` last,
in `init()` and after every degauss pulse; X sigma went 0.000 → 5.18 counts and
the differential field 20.1 → 37.0 µT on all three axes.

**Do not replace that ordering rule with one paired 3-byte write.** It looks
strictly better — the address does walk, so byte 2 reaches `CTRL1` and lands
after the aliased copy, and byte 2 reproduces `CTRL1`'s semantics exactly
(`0x04|bw` inhibits X, `0x18|bw` inhibits Y and Z) with `init()` running at a
healthy 105 changes/s. It cannot be used for the degauss, which is where it
would matter most: with a Set or Reset bit in byte 1 the part stops measuring.

| degauss form | rate after the pulse |
|---|---|
| SET / RESET as two writes | 106 / 105 changes/s |
| SET / RESET as one 3-byte write | 1 / 1 changes/s |

The ~500 ns coil current does not tolerate a second byte inside the same
chip-select assertion. Pairing in `init()` alone would leave two idioms and an
unstated landmine, so the rule stays uniformly.

**A — quiet after enabling continuous mode.** Five runs per point:

| post-`CTRL2` quiet | 0 | 10 | 20 | 30 | 40 | 50 | 60 | 80 | 100 ms |
|---|---|---|---|---|---|---|---|---|---|
| healthy runs | 0/5 | 0/5 | 0/5 | 4/5 | 5/5 | 5/5 | 5/5 | 5/5 | 5/5 |

Fixed with a 100 ms wait after the `CTRL2` write, ~2.5× the boundary. Three
things it is **not**: not the clock (same threshold at 10 MHz, 1 MHz and
100 kHz, so clamping the mag's SPI clock is not the remedy); not general write
spacing (a delay after `CTRL0` or `CTRL1` alone does not help — only after
`CTRL2`, and writes issued once the window has passed are harmless); and not
warm-up (writing back-to-back and then waiting up to 2 s before measuring does
not help at all). Writing `CTRL2` last does **not** retire the wait, which is
what the reorder was tried for: `read()`'s per-sample `STATUS` write enters the
window as soon as the mag reader starts. The wait therefore lives inside
`init()`, not in the caller.

**C — bandwidth before `CM_Freq`.** Rev A p.15 makes BW an input to what
`CM_Freq` means: the frequency table is stated "based on the assumption that
BW[1:0] = 00", and two rows carry a prerequisite in the row itself — `110` needs
BW=01, `111` needs BW=11. The old order enabled continuous mode while `CTRL1`
still held the reset default. At `odr_hz = 1000` the part then intermittently
fails to start at all, the first read waiting 500 ms for `Meas_M_Done`:

| | failed to start |
|---|---|
| `CTRL2` before `CTRL1` (old) | 7 of 20 |
| `CTRL2` after `CTRL1` (new) | 0 of 20 |

12 of 33 against 0 of 33 across every run of that session. In the daemon the
shape of the failure is `init()` returning success — every write ACKed — and a
magnetometer that never produces a sample.

**D — three of the seven advertised ODRs are not honoured.** Counted as distinct
output images per second with **no `STATUS` write**, because a rate counted by
polling `Meas_M_Done` may be the loop's own speed (Rev A p.13 says that bit
"will remain '1' till next measurement" while a write "will clear the
corresponding interrupt", which does not settle whether the write clears the
bit). Every code against every bandwidth:

| cfg Hz | `CM_Freq` | BW=00 | BW=01 | BW=10 | BW=11 |
|---|---|---|---|---|---|
| 1 | 001 | **1** | 1 | 1 | 1 |
| 10 | 010 | 130 | 256 | 497 | 1689 |
| 20 | 011 | **22** | 21 | 21 | 21 |
| 50 | 100 | 130 | 256 | 497 | 1689 |
| 100 | 101 | 106 | **105** | 105 | 105 |
| 200 | 110 | 130 | 256 | 497 | 1690 |
| 1000 | 111 | 120 | 241 | 402 | **1206** |

`010`, `100` and `110` do not take at **any** bandwidth — those rows are each
BW's conversion ceiling (8 ms → 125, 4 ms → 250, 2 ms → 500, 0.5 ms → 2000).
`001`, `011` and `101` take at every bandwidth; `111` only at BW=11, exactly as
p.15 annotates, which is the mechanism behind C. **Bandwidth is therefore not
the constraint**, so do not "fix" 200 Hz by moving it to BW=10 on the strength
of p.4's max-output-rate table (which does contradict p.15's `110 (BW=01)`
annotation) — `110` free-runs at BW=10 too. Nor is it how `CTRL2` is written:
splitting the frequency from the enable, with or without a delay, and stopping
the mode first, all give the identical table.

Fixed by advertising only what the part delivers: `supported_odr_hz` is
`{1, 20, 100, 1000}` and `odr_encode()` emits only those codes, so
`snap_odr_up()` rounds a request across the gap — 10 → 20, 50 → 100,
200 → 1000 — instead of programming a code the part ignores. `test_drivers`
pins the rounding and sweeps 1..1000 Hz asserting no request reaches an
unhonoured code. **This is one die**, which cannot separate a part-wide
limitation from this sample's; if a part turns up that honours the even codes,
`odr_encode()` and `supported_odr_hz` are what to revisit, and
`imud-imutest`'s `mag.rate` is the check that would show it.

**The field magnitude is not a defect** *(resolved 2026-08-17)*. It read
36.6 µT differential where the WMM gives 47.42 µT for the bench's own
coordinates. A `.imucap` capture of ~45 s of hand tumbling, fitted as a sphere:

| | |
|---|---|
| radius (true field) | **45.18 µT** vs WMM **47.42** — **−4.7%** |
| centre (board-fixed offset) | `[22.45, 1.48, 11.26]` µT, **\|c\| = 25.16** |
| residual RMS | 1.96 µT, 8 of 8 octants, 2106 samples |

−4.7% is inside the part's own 5% sensitivity tolerance (Rev A p.4), so the
16384 counts/G constant is right and there is no scale error. The shortfall is
the 25 µT offset, which at the bolted-down orientation opposed the field:
47.4 − 25 ≈ 36.6. Direction agrees independently — the angle between the field
and the accelerometer, offset-corrected and so orientation-free, implies an
inclination of 61.35° against the WMM's 61.15°. A magnetised mounting screw was
the largest contributor, found by unmounting; a magnetic screwdriver magnetises
a steel screw in one insertion, and A2/304 stainless goes weakly ferromagnetic
when cold-worked, which a thread is. Brass or nylon avoids it. `imud-cal`
removes the remainder. That also retires the owed yaw-rotation check, more
thoroughly than a yaw sweep would: a stuck or mis-scaled axis cannot fit a
sphere to a 1.96 µT residual.

**Compare differentials to differentials.** Raw `|B|` includes the bridge
offset, which is a property of the part's magnetisation at that moment and is
not comparable across sessions — an earlier attempt to reconcile a 43 µT SPI
reading with the 64.8 µT I²C baseline was meaningless for that reason.

**Ruled out by measurement, not argument**, and not to be re-run: external
field; SPI read framing (two-transfer and single full-duplex, burst and
bytewise, byte-identical over five static single-shot images — so `bus_io.h` is
not at fault and there is no shared-path defect affecting every SPI driver);
register encoding and the 18-bit output layout; the `STATUS` write-to-clear;
`SW_RST` settle from 10 ms to 1 s; bus crosstalk; GPIO; thermal damage
(documented for reflowed boards of this part, but that damage is permanent and
this one recovers fully); and a missing CAP capacitor (the schematic has 10 µF
where the vendor asks for it). `Auto_SR_en` was tested and rejected: it holds a
recovered bridge in isolation but oscillates under measurement, because the part
alternates SET/RESET per sample and the driver averages them blind.

### 1.2.2 The full ODR ladder, both parts, over SPI  *(2026-08-19, no human)*

`imud-imutest --passive` at every advertised rate of both parts — ten for the
`ism330dhcx`, four for the `mmc5983ma` — plus daemon runs at each. Four
findings, and three defects in the TOOL that this was the first thing ever to
expose (fixed in `af5ddac`, `7e8a039`, `a000b44`).

**The ISM330DHCX runs 4.1% fast, at every rate on its ladder.**

| configured | 12 | 26 | 52 | 104 | 208 | 416 | 833 | 1660 | 3332 | 6664 |
|---|---|---|---|---|---|---|---|---|---|---|
| measured (Hz) | 13.6 | 27.1 | 53.8 | 108.2 | 216.6 | 433.0 | 867.1 | 1733.1 | 3468.4 | 6935.6 |
| ratio | 1.133 | 1.042 | 1.035 | 1.040 | 1.041 | 1.041 | 1.041 | 1.044 | 1.041 | 1.041 |

That is the same oscillator error the part declares through `FREQ_FINE`
(24027 ns against a 25000 ns nominal, −3.89%) seen from the other side, and the
two agree to three figures: at 833 Hz the part delivers 867.1 Hz and 48.00
ticks per sample, and 867.1 × 48 gives a 24.03 µs tick. **Nothing is wrong
here** — but it is why `imu.chipts.rate` was dividing by the wrong number, and
`odr_hz = 12` is a genuine outlier at +13.3% that the rest of the ladder is not.

**The MMC5983MA at `odr_hz = 1000` is still the odd one.** It delivers
**1196.8–1204.4 Hz**, 20% above the configured rate, across every run. The
driver advertises 1000 for `CM_Freq = 0x7`, and fusion sizes the magnetometer's
noise variance from what is advertised — which is the exact failure the driver
comment describes for the three rates it deliberately stopped advertising. Its
DRDY behaviour there is erratic too: one run counted 3614 samples from 3614
edges, the next counted **zero edges** in the same window with the same
configuration. The daemon runs the mag at 100 Hz, where everything is clean
(105.7 Hz over the interrupt against 105.4 polled, 317 samples from 317 edges),
so this is not urgent — but `supported_odr_hz` advertising 1000 for a setting
that yields 1200 is the same class of defect that entry was written to fix.

**Two smaller things, each seen once and not yet reproduced:**

- `imu.fs.a8` FAILed at `odr_hz = 26`: the accelerometer read **11.19 m/s²**
  against 9.807 at ±8 g, 14% high, while the other three ranges passed. The
  full-scale sweep re-inits at each range, and at 26 Hz a sample takes 38 ms,
  so this may be the sweep reading before the part has settled rather than a
  scale error. Worth reproducing at low ODR before believing either.
- ~~`imu.init.idempotent` WARNed at 208 Hz (1 register) and 6664 Hz (2)~~ —
  **fixed 2026-08-20**, and the rates above are wrong. A sweep of the whole
  ladder that evening had the check passing at 208, 1660 and 6664 Hz; the case
  that still reproduced was the *magnetometer* sweep at `mag odr_hz = 1`, where
  it read "2 registers differ" across 103 non-volatile registers.

  The cause was `ism_init()` writing FIFO Continuous mode straight over
  Continuous. DS13012 §6.5.1/§6.5.2 make Bypass the only thing that clears FIFO
  content, so only the first `init()` after a reset — the one transitioning
  from the `000` the reset left — actually flushed. Both ST drivers now park
  the FIFO in Bypass first, which is what the other three FIFO drivers here
  already did. Same case afterwards: PASS, 0 registers differ, over the same
  103-register comparison set, and 19 runs across ten IMU rates and four mag
  rates without a recurrence.

**Everything else was clean at every rate**, including 3332 and 6664 Hz, which
were the two cleanest runs of the ladder.

### 1.2.1 ~~The status gate and the DRDY edge cannot both be used~~ — **refuted 2026-08-19**, it was SPI mode 3 (see §1.2)

The daemon was reading the magnetometer at **35 Hz from a 105.5 Hz part** — two
of every three samples discarded, so heading got a third of the information it
should. Invisible to `imud-imutest`, which polls, and therefore measured a rate
the daemon could not reach.

`mmc_read()` gated on `Meas_M_Done`. That bit is also the interrupt
acknowledge: writing 1 to it is the only way to re-arm the latched INT, and the
same write takes the bit away. It then re-asserts **only while the bus is being
actively polled** — after a clear, with the bus quiet, a single status read found
it clear in 10 of 10 trials at every delay from 2 ms to 25 ms, nearly three
conversion periods. Blocking on the edge is exactly when the bus is quiet:

```
read succeeds → clears M_DONE, INT drops
  │ 9.4 ms
INT rises → reader wakes → STATUS 0x10, no measurement flag → "no data"
  │ latched: no second rising edge
20 ms timeout → read finally succeeds        9.4 + 20 ≈ 29 ms → 35 Hz
```

Per-iteration: 20 edges, 20 timeouts, 20 reads, 20 not-ready. The gate is the
**one-shot idiom in a part running continuous** — Rev A says the bit "turns to 0
when the new measurement command is occurred", and continuous mode issues no
command.

Fixed with `mag_cfg_t.int_driven`, which the daemon sets from the same condition
that requests the mag GPIO line. In that mode `read()` trusts the edge, still
performs the acknowledge write, and rejects a sample whose output registers have
not advanced — that guard is what makes a dead INT line degrade to ~50 Hz of
genuine data instead of an endless duplicate. Measured side by side on healthy
silicon:

| | rate | \|B\| mean | sigma |
|---|---|---|---|
| gated tight poll (2 ms) | 105.3 Hz | 62.334 µT | 0.053 |
| **ungated, edge-driven** | **106.0 Hz** | **62.329 µT** | **0.051** |

Same field, same noise, full rate; 0 repeats and 0 timeouts over 1200
consecutive edges.

**Ruled out by measurement — do not re-run.** The instrument (0 byte-identical
repeats at 2 ms poll, so imutest's 105.4 Hz was correct); the INT connector
(reseated, identical 34.94 vs 35.06 Hz); the DRDY line itself; the periodic
degauss; IMU bus contention; the read/clear **ordering** (burst-then-clear,
clear-then-burst and clear-alone all return the bit in an identical 9.4 ms); and
a bounded re-poll after the edge, which cannot work because the bit needs a
further full conversion period of polling to appear.

**The other three interrupt-capable mag drivers cannot have this defect**, per
their datasheets: LIS3MDL's `ZYXDA` is a plain status flag with no acknowledge
write, LIS2MDL drives the `Zyxda` bit straight onto the pin, and the RM3100's
DRDY "is set LOW when the Measurement Result registers are read" (V11.0, pin 23).
None has a write that could destroy its own gate. They are documented in the
driver guide, not modified.

`imud-imutest` gained `mag.drdy.rate`, which measures over the interrupt line and
grades against the polled figure — the blind spot that let this survive.

**Still open from this.** At `odr_hz = 1000` the edge-driven read reports
**1687 Hz**, above any rate the datasheet allows, so the INT there fires faster
than conversions complete. Its own investigation; the daemon runs the mag at
100 Hz.

**A saturated bridge is not recoverable by the driver.** `reset()` does not cure
it — `SW_RST` clears registers, not magnetisation — and the 500 ns SET pulse is
too weak. Recovery took a sustained self-test coil drive (`CTRL3`
`St_enp`/`St_enm`, ~150 ms each way, three times). Nothing in imud can do that,
so a part saturated in the field by a strong magnet stays saturated. Worth
deciding whether `reset()` should attempt a coil-based recovery, which is its
own change with its own risks.

**Acceptance.** `imud-imutest --passive` against the fixed driver, from a
recovered bridge: `mag.noise` non-zero on all three axes,
`mag.degauss.differential` PASS with all three axes inverting,
`mag.field_magnitude` PASS, `mag.burst_framing` PASS, `mag.rate` WARN at
105.5 Hz (the die runs ~5% fast on both transports; the datasheet gives its
continuous-mode frequencies as typical). Run it with the daemon **stopped** —
imud and imud-imutest open the same device and drain the same FIFO, and a run
that races the daemon produces plausible-looking nonsense.

**Open.**

- **No mechanism for the 40 ms.** Nothing in Rev A explains why the part needs
  that long after `Cmm_en` before it will accept another write. The first
  conversion at BW = 00 takes 8 ms, the right order of magnitude and not the
  number. Until there is a mechanism the delay is empirical.
- **Guided-phase acceptance is owed.** The passive phase runs clean, but
  `--faces`, `--gyro` and `--spin` need an operator, and `spin.*` is where the
  magnetometer's frame agreement with the gyro is established.
- ~~`imu.chipts.monotonic` FAILs on the ISM330DHCX over SPI.~~ **Cause found
  2026-08-17 and fixed**, from a 94,539-sample `.imucap` rather than another
  imutest run: exactly **one** reversal, 0 repeats, 0 zero-stamped. It is not a
  seam-correction failure. One burst of 9 samples was stamped **2,163,509 ticks
  (54 s) AHEAD**, `seq` continuous across it, and the burst after was correct —
  so the reversal the check reports is the *return* to real time, and the
  backward guard recovered one burst too late.

  The post-drain `TIMESTAMP0` read is taken as the newest sample's time, and the
  backward guard only corrects *overlaps*, so a read that comes back garbage in
  the **forward** direction was never checked. The sharp edge: `st_fifo_ts.h`
  already rejects its batched anchor when `now_ts` fails its cross-check, and
  the fallback then used that same `now_ts` as ground truth. Fixed with
  `chip_ts_guard_forward_ok()`, applied in all three drivers that share the
  fallback (`ism330dhcx`, `lsm6dso`, `icm42688p`): an implausible forward read
  is refused and the burst extrapolates one sample period from the previous one,
  counted and logged rate-limited. A plausible advance is still taken from the
  counter, so one bad read does not stop the driver tracking the chip.

  Nothing reached the filter — `imu.c` rejects a non-increasing timestamp and
  falls back to the nominal period — but `ts_wall_ns` carried it to the wire.

  **Confirmed on the reference part.** `imu.chipts.monotonic` PASSes,
  `0 reversals / 0 repeats`, in a run of **40 PASS · 1 WARN · 0 FAIL · 4 SKIP**
  — no FAIL left on this pair. The sole WARN is `mag.rate` at 105.4 Hz against
  a configured 100, the die's own oscillator running ~5% fast on both
  transports, which §1.2 records above as the part rather than the driver.

  Judge a reversal count only against a run the daemon is not in. The earlier
  "12 reversals" reading is void, measured while the daemon was racing imutest
  for the same FIFO; at 1 event per 94,539 samples, 12 inside a 5 s window was
  contention, not this defect.

### 1.4 Long stability run  *(2026-08-25, 44 min, SPI, `ism330dhcx` + `mmc5983ma`)*

833 Hz / `fifo_wm = 64`, mag 100 Hz, capture on, sampled once a minute from
`/proc` and the `[stats]` line.

| | |
|---|---|
| RSS | **2,544 kB for the whole run** — not one sample differed |
| open fds | **10**, constant |
| threads | **7**, constant |
| `ovf` | **0** |
| warnings | **0** |
| drains | 37,332 edge / **1** timeout |
| batched-timestamp rejects | **0** |

No leak, no descriptor growth, no overflow, and the interrupt line carried every
drain but one. The two `[E]` lines are a non-root bench run failing to write
`/run/imud/imud.pid` and bind the health socket, not a fault.

**Heading moved 56° with both of its inputs held still, because the yaw update
never ran.** The vessel was tied at all four corners and could not yaw.

Neither input moved. Gyro RMS over the capture is **0.32 / 0.17 / 0.10 °/s** per
axis (p99 0.63 / 0.42 / 0.28; per-axis max 1.5–2.7) — wind roll, nothing more.
The raw magnetic vector held within **0.31° of its starting direction across
46 minutes**, |B| between 52.3 and 52.9 µT.

The cause is in two lines. `src/imu.c` sets a mag sample's validity from whether
a *calibration* exists — `s.valid = ctx->cal.has_mag` — and `src/fusion.c`'s
magnetometer update returns immediately on `!m->valid`. **With no `cal.json`,
every mag sample is marked invalid and the yaw update never executes**, so yaw
is open-loop gyro integration. Confirmed live on the wire rather than inferred:
`innov_reject` 0.0000, `innov_weight` 1.0000, `nis_mag` exactly 1.000 and
`mag_residual` exactly 0.000° across every packet — the update is not being
rejected or capped, it is not running. Alignment still consults the mag once at
startup, which is why the initial heading looks right and then walks away from
it; the log's "uncalibrated mag — heading approximate" is a considerable
understatement for a heading with no measurement behind it at all.

The baseline drift matches: roughly 1 °/min, the size of the startup gyro Z bias
estimate (−0.0003 rad/s ≈ −1 °/min).

**The covariance collapse is explained, and it is a false-confidence bug.**
Reproduced 2026-08-25 and caught at 1 Hz:

```
11:34:26  hdg=359.4  pitch= 0.6  roll=92.2  cov=1.0e+00
11:34:27  hdg=338.9  pitch= 0.2  roll=92.3  cov=8.2e-01
11:34:28  hdg=  0.9  pitch=-1.1  roll=92.3  cov=4.2e-01
11:34:29  hdg= 35.6  pitch=-1.7  roll=91.3  cov=1.4e-01
```

Pitch and roll move through the transition — pitch 0.6° to −1.7°, roll 92.3° to
91.2° in three seconds. A physical tilt disturbance is the trigger, and the
mechanism is the regime it lands in:

1. No `cal.json`, so the yaw update never runs and yaw error grows unbounded.
2. Attitude covariance reaches ~1 rad² — σ ≈ 57°, far outside the small-angle
   linearisation an error-state filter assumes.
3. A tilt disturbance makes the gravity update do real work.
4. Because the estimated and true frames now differ by order 1 rad, that
   correction is **no longer orthogonal to the heading direction**. It couples
   into yaw.
5. Out comes a ~50° heading correction and an eightfold covariance drop.

**The covariance drop is false confidence.** Yaw is not known to 0.14 rad² after
that event; it is still wrong by tens of degrees. A consumer reading `cov` to
decide whether to trust heading would be misled precisely when heading is at its
worst. That is the part worth fixing, independently of the calibration.

It needs the disturbance: a companion run with no calibration and no capture
held the covariance ceiling for **ten minutes with no collapse**, and its
platform never moved — pitch span 0.212°, roll span 0.344° across 24 minutes,
maximum single-sample step 0.18°. The run that collapsed moved 2.3° of pitch in
three seconds. A first A/B appeared to implicate the capture path, since the
collapsing runs had capture enabled and the quiet one did not; that was
coincidence at n = 1 per side, and the 1 Hz trace shows the real trigger.

This whole regime exists only because the yaw update is disabled. With a
magnetometer calibration present, yaw variance never approaches 1 rad² and the
linearisation stays valid — so `imud-cal mag` is the prerequisite for judging
any of it, and the false-confidence behaviour is what to fix if the regime can
be entered at all.

**Confirmed by the other half of the experiment** *(2026-08-25, same rig, same
static berth)*. Pointing `[calibration] file` at a bench calibration — any
calibration, since what matters is that `has_mag` becomes true and the yaw
update executes — changes the behaviour completely:

| over ~22 min | no calibration | bench calibration |
|---|---|---|
| heading | **220° of drift** | **0.20° range** (251.9–252.1) |
| `cov` trace | 0.03 → 1.0 rad², then collapse | **1.3e-03, flat** |
| filter state | "converging" indefinitely | "converged" within ~4 s |
| warnings | 0 | 0 |

About 1100× less drift and 770× lower covariance, and the collapse regime never
arises because the covariance never approaches 1 rad².

Read the 0.20° as an **upper bound on drift, not a measurement of filter
noise**. It is at the level where real movement cannot be separated from filter
noise without an independent heading reference: the same berth showed a pitch
span of 0.212° and a roll span of 0.344° over 24 minutes, so a couple of tenths
of a degree of genuine yaw in the slip is entirely plausible and some or all of
the 0.20° may simply be the boat. The comparison against 220° is unaffected
either way. That is the diagnosis
closed from both ends: the drift and the collapse are both consequences of the
yaw update not running, not of anything the filter does wrong when it does.

Two things this does NOT show. The heading is not *accurate* — the bench cal was
taken in a different magnetic environment and applying it drives |B| from 46.0
to 78.2 µT against an Earth field of ~48–50, so the value is wrong, merely
constant. And a fixed wrong calibration always looks stable on a static
platform: stability here is evidence the update runs, never that the calibration
is right. Only a swing separates those.

Worth recording from the same measurement: the hard iron largely **transfers**
between the bench and the boat. `|h|` is 37.0 µT and subtracting it lands |B| at
54.1 against ~49 expected, so the offset is right to within roughly 5 µT — which
only makes sense if the dominant source moves with the sensor. On this rig that
is the Pi, mounted in the same relative position both times. The soft iron does
not transfer: it takes |B| from 54.1 to 78.2, and most of that is the Z scale of
2.074 that the coverage guard now refuses to write.

**Two earlier readings of this run were wrong and are retracted.** The first
recorded the platform as moving, citing `fit-temp`'s peak-to-peak of
0.144 rad/s — a misuse of an extremum over 2.26 M samples, the exact error §3.1
warns about when it says to prefer p50/p99 over a lone `max`. The second
attributed the wander to magnetometer updates being gated by hard iron; the
gate counters above show nothing was ever gated, and hard iron on a *fixed*
orientation is a constant offset that cannot produce drift in any case. The
measurements, not the narratives, are what stand.

## 2. Gyro bias temperature compensation  *(code shipped 1.5 — needs Pi thermal data)*

The mechanism shipped in 1.5: cal.json `gyro_temp` per-axis linear
coefficients applied per-sample before fusion, fitted offline with
`imud-cal fit-temp --from <warm-up capture>`, fed by 12.5 Hz FIFO
temperature batching. **Remaining: record a real cold-boot→warm capture on
the Pi and fit actual coefficients** — pair with the first hardware
sessions. The ISM330 self-heats only ~1 °C, so a usable fit needs a real
ambient swing of several °C (leave it across a garage day/night cycle, or
warm it gently mid-capture).

**Attempted 2026-08-25 and the self-heating estimate is confirmed, low.** A
44-minute capture from a cold boot — 2.26 M IMU samples at 833 Hz — moved the
die across **17.59 to 18.17 °C, a span of 0.58 °C**, and it *tracked ambient*
rather than climbing: it peaked near t+18 min and fell back by the end. On this
rig the part is ambient-dominated, so a passive soak of any length will not
produce a fit, and running one longer is not the answer.

`imud-cal fit-temp` refuses it correctly rather than fitting noise —
`temperature span too small (0.7 degC) — capture a cold-boot warm-up (span >= a
few degC)` — so the guard in `src/cal_main.c` is doing its job on real data.

What this needs is a deliberate thermal excursion, not a longer run: the
day/night cycle already suggested above, or gentle external warming partway
through a capture. It is an environmental prerequisite, not a software one.

## 3. Pi 5 interrupt latency re-profiling  *(pre-existing spec §16 item, bench)*

Pi 5 routes GPIO through the RP1; gpiod is the right abstraction but edge-interrupt
latency should be measured against the Pi 4 baseline once hardware testing starts.

### 3.1 spec §14's latency budgets — measured across the whole ladder  *(2026-08-25: the model is settled and spec §14 is rewritten; the material below 2026-08-25 is kept for provenance and is partly superseded)*

`spec.md` §14 budgets FIFO read jitter at 5 ms p99, fusion latency at 1.5 ms and
end-to-end at 3 ms. Nothing measured any of them for a long time, and the
end-to-end row looked impossible against the shipped config — `fifo_wm = 64` at
833 Hz reads as 77 ms of buffering on its own, against a 3 ms budget.

**The instrument exists** (`lat_hist_t` in `imu_math.c`, wired in `imu.c`,
reported on the `[stats]` line). It splits the chain into the two terms only ever
discussed as a sum:

    FIFO residence   sample taken -> I2C read complete   bounded by the drain cadence
    pipeline         read complete -> state fused        imud's own cost

**Sim cannot supply the first term**: the sim driver synthesises `chip_ts` as
`seq*ticks_per`, advancing at exactly nominal rate rather than tracking elapsed
time, so its FIFO column reports the sim's own batching. Reading that as FIFO
residence would be an instrument artifact of exactly the kind §10.9 was written
to avoid.

**Measured twice on a Pi 5**, 120 s per combination, `ism330dhcx` on I²C, all
six combinations at `ovf = 0` in both runs. The two runs straddle `51b0267`,
which fixed an anchor that timestamped every sample about half a burst-read
late — so the second run is the trustworthy one, and the pair together is the
more interesting reading.

| `odr_hz` | `fifo_wm` | `fifo` p50 (08-14 → 08-15) | `fifo` exact max | `pipe` p99 |
|---|---|---|---|---|
| 104 | 8  | 16.4 → **32.8** ms | 12.7 → 49.6 ms | 0.03 ms |
| 104 | 32 | 8.2 → **8.2** ms | 26.0 → 25.6 ms | 0.06 ms |
| 104 | 64 | 0.1 → **8.2** ms | 25.5 → 25.9 ms | 0.03 ms |
| 833 | 8  | 32.8 → **32.8** ms | 56.8 → 34.7 ms | **0.26 ms** |
| 833 | 32 | 32.8 → **32.8** ms | 55.5 → 56.2 ms | **0.26 ms** |
| 833 | 64 | 16.4 → **32.8** ms | 24.4 → 54.1 ms | **0.26 ms** |

**`pipe` — the term imud controls — is 0.26 ms p99 at 833 Hz in both runs**,
six times under §14's 1.5 ms budget and identical across every combination.
Reproduced across a code change and two boots, this is the solid number here,
and the one §14's fusion-latency row was always about.

`fifo` says something other than what this section assumed:

- **It does not scale with `fifo_wm`.** *(SUPERSEDED 2026-08-25 — true of the
  code of the time, and no longer. `6ac3c66` replaced the flat timeout with a
  fallback of `fifo_wm + int_grace` sample periods, which is always later than
  the watermark it backs up, so the watermark now triggers every drain and
  residence tracks `fifo_wm / odr_hz`. See the ladder at the end of this
  section.)* The watermark is not what triggers a
  drain: the reader waits with a **10 ms timeout** (`src/imu.c`) and drains
  whatever is there when it expires, so residence is bounded by that cadence
  plus the read itself. `imud-imutest` measured the same thing independently
  as `max read-loop gap 10.2 ms`. §1.1's finding that `INT1_FIFO_TH` is a
  *level* condition is the same fact from the other side.
- **`wm/odr` was never the right model**, and this section said it was. It is
  the residence of the oldest sample in a burst *drained at the watermark*,
  which is not the configuration anyone runs.
- **Reported percentiles are log₂-bucket upper edges.** A `p99` of `16.4 ms`
  means the true value is in `[8.2, 16.4)`. The 08-14 run's own data proves
  it: at 104/8 the exact `max = 12.7 ms` sat *below* the reported
  `p99 = 16.4 ms`. Conservative by design (`imu_math.h`), but a percentile
  compared against a point model overstates by up to 2×. Use `max=` for a
  number — while reading the next paragraph first.

**Retracted: "a shallower watermark measures worse."** This section used to
record that as the one result no model predicted, on the strength of 08-14's
833 Hz maxima — 56.8 and 55.5 ms at `wm = 8`/`32` against `wm = 64`'s 24.4 ms.
The 08-15 run inverts it exactly: 34.7 / 56.2 / **54.1**, with `wm = 64` now
the worst of the three. The claim does not survive, and the reason it looked
true is worth keeping:

- **The old anchor deflated `fifo`, and deflated it most where reads are
  longest.** A sample timestamped late by ~half a read reports a shorter
  residence than it had, and the read is longest at the deepest watermark.
  08-14's `wm = 64` was therefore the most under-reported row in the matrix,
  which is precisely what made it look best. Every `fifo` p50 in the table is
  now equal or higher, and the two rows that moved a whole bucket — 833/64 and
  104/64 — are the two deepest watermarks. **08-14's `fifo` column measured
  the defect as much as it measured the FIFO.**
- **Exact `max` is one sample in 120 s and moves run to run** — 104/8 went
  12.7 → 49.6 ms with no change that should affect it. Prefer the p50/p99
  pair, and treat a single run's maximum as an anecdote. The earlier wording
  leaned a whole finding on one.

**Still open, stated more carefully this time.** The watermark's effect on
`fifo` is not settled, and 08-15 is not self-consistent about its direction
either: at 104 Hz the shallow setting is worse (p50 32.8 ms against 8.2 ms at
`wm = 32`/`64`), while at 833 Hz it is better at the tail (p99 32.8 ms against
65.5 ms). No model here predicts a sign that flips with ODR. The measurement
that would settle it is unchanged: log per-drain sample counts and the wake
reason (edge vs timeout) at `wm = 8` against `wm = 64`, which distinguishes a
longer cycle from deeper bursts.

**Independently cross-checked.** The 08-15 run measured sample age on the wire
— `now - ts_wall_ns` at the TCP stream, 99,961 packets — at `p50 = 33.79 ms`,
against that run's `fifo` p50 + `pipe` p50 of 32.93 ms for the same
configuration. The 0.86 ms difference is the socket hop, and it lands *above*
the sum rather than below, which is the signature of an anchor that is no
longer late. So the raised `fifo` figures are a truer residence, not a
regression.

**The SPI pair is in** *(measured 2026-08-18, from the daemon's own `[stats]`
line at 833 Hz / `fifo_wm = 64`, `ism330dhcx` over SPI on a Pi 5)*:

| transport | `fifo` p50/p99 | `pipe` p50/p99 |
|---|---|---|
| I²C, same config (08-15) | 32.8 / 32.8 ms | — / 0.26 ms |
| **SPI** | **4.1 / 8.2 ms** | **0.06 / 0.13 ms** |

An eightfold drop in FIFO residence and a halved pipeline term. *(SUPERSEDED
2026-08-25: this row was taken before `6ac3c66` and does not describe the
current reader. The same 833 Hz / `wm = 64` configuration now measures 75.8 ms,
not 4.1 — not a regression but the watermark finally deciding the drain instead
of being pre-empted. The ladder below replaces it.)*

**The whole ladder, and the model that finally fits** *(2026-08-25, SPI,
`ism330dhcx` + `mmc5983ma` on a Pi 5, one run per row, measured rate within 10%
of configured on every row)*.

`6ac3c66` changed what this section was measuring. The reader no longer holds a
flat 10 ms timeout; it waits on the watermark with a fallback of
`fifo_wm + int_grace` sample periods, which is by construction *later* than the
watermark it backs up. So the watermark now decides every drain, at every rate —
`E/T` is 24/0 at 13 Hz and 6510/1 at 6664 Hz — and `fifo_wm / odr_hz`, the model
this section spent two revisions rejecting, is now the right one.

|`odr_hz`|mean/drain|`fifo` p50/p99|`fifo` max|`fifo_wm / odr_hz`|`pipe` p50/p99|E/T|
|---|---|---|---|---|---|---|
|13.016|42.7|1048.6 / 1048.6|3102.9 ms|4917 ms|0.26 / 0.51|24 / 0|
|26.031|51.0|1048.6 / 1048.6|1846.2 ms|2459 ms|0.26 / 0.26|31 / 0|
|52.063|56.3|1048.6 / 1048.6|1034.5 ms|1229 ms|0.26 / 0.51|57 / 0|
|104.125|60.0|262.1 / 1048.6|552.5 ms|615 ms|0.26 / 0.51|108 / 0|
|208.25|61.3|262.1 / 524.3|282.5 ms|307 ms|0.26 / 0.51|212 / 0|
|416.5|62.1|131.1 / 262.1|144.2 ms|154 ms|0.26 / 0.51|419 / 0|
|833|63.1|65.5 / 131.1|**75.8 ms**|**76.8 ms**|0.26 / 0.51|823 / 1|
|1666|63.1|32.8 / 65.5|41.8 ms|38.4 ms|0.26 / 0.51|1647 / 1|
|3332|63.1|16.4 / 32.8|23.3 ms|19.2 ms|0.26 / 0.51|3295 / 1|
|6664|63.8|16.4 / 16.4|14.6 ms|9.6 ms|0.26 / 0.51|6510 / 1|

All at `fifo_wm = 64`. `ovf` was 0 up to 833 Hz and exactly 1 per run above it.

- **`max` tracks `fifo_wm / odr_hz` from 13 Hz to 833 Hz** — 75.8 against a
  predicted 76.8 at 833. Above 1666 Hz it runs *over* the prediction (14.6
  against 9.6 at 6664), because the read itself stops being negligible against
  a shrinking budget. That is the honest upper end of the model.
- **Mean samples per drain is the watermark**, exactly, at `wm = 8` and
  `wm = 32`. At `wm = 64` it is 63.1 above 833 Hz and falls to 42.7 at 13 Hz:
  the ST parts batch the chip timestamp into the same word budget, and at low
  ODR those words are a far larger share of it. The registry's "about 63 rather
  than 64" note holds at high rate and understates the effect at low rate.
- **`pipe` is 0.26 / 0.51 ms at every rate on the ladder** and drops to
  0.06 / 0.13 at `wm = 8`. It scales with burst depth, not with ODR — which is
  what it should do, since it is the cost of fusing a burst. Well inside
  §14's 1.5 ms budget everywhere.

The watermark is the lever, and it works at both ends:

|`odr_hz`|`fifo_wm`|mean/drain|`fifo` max|`pipe` p50/p99|
|---|---|---|---|---|
|833|8|8.0|11.8 ms|0.06 / 0.13|
|833|32|32.1|39.6 ms|0.26 / 0.26|
|833|64|63.1|75.8 ms|0.26 / 0.51|
|6664|8|8.0|6.6 ms|0.06 / 0.06|
|6664|32|32.0|10.7 ms|0.26 / 0.26|
|6664|64|63.8|14.6 ms|0.26 / 0.51|

**This closes the section's last two owed items** — the pair above 833 Hz, and
the watermark question the 2026-08-19 subsection got wrong. spec §14 has been
rewritten against these numbers, and the stale "10 ms timer" wording it and
`docs/manual.md` carried is gone.

*Method note.* The first attempt at this ladder was contaminated and thrown
away: the previous sweep's script was still running when the new one started, so
two daemons drained one FIFO and the 13 Hz cell reported 371,433 samples in 65 s
against a configured 13 Hz, with the filter diverged to `cov = 6.9e+19`. The
runner now refuses to start if `imud` is already up, holds a lockfile, and
prints measured-versus-configured rate per cell so contention shows in the table
rather than only in the logs.

### The watermark question is answered  *(2026-08-19)* — **SUPERSEDED 2026-08-24**

> **RETRACTED, and read the retraction before the table below.**
>
> The measurements here are sound and the rule derived from them was correct
> *for the code of the time*. What was wrong is the conclusion drawn: the 10 ms
> fallback was treated as a property of the design, and it was a defect.
>
> A LEVEL watermark asserts once the FIFO holds `fifo_wm` sets and de-asserts as
> soon as a drain empties it. A fallback shorter than the watermark period
> therefore drains first, holds the FIFO permanently below the threshold, and
> the line never asserts **at all** — so `fifo_wm` was not "ineffective above a
> threshold", it was *suppressed by its own safety net*. Two further defects
> compounded it: the drain buffer could not empty the FIFO (128 samples against
> a 256-set FIFO), and the driver issued one ioctl per 7-byte FIFO word, so a
> drain took 3.1 ms of blocked syscall time and could never get ahead.
>
> Fixed in `6ac3c66` (wait = `depth + grace` **samples**, not a flat 10 ms),
> `c1e81a0` (`IMU_DRAIN_MAX` 256) and `95a3c77` (burst FIFO reads).
> Re-measured on the same rig afterwards:
>
> | `odr_hz` | `fifo_wm` | drains edge/timeout | mean n | note |
> |---|---|---|---|---|
> | 104 | 8   | 377 / 1      | 8.0   | was `0 / 4068` |
> | 104 | 64  | 50 / 0       | 59.9  | was `0 / 4069` |
> | 833 | 64  | 314 / 1      | 63.3  | was `0 / 3764` |
> | 3332 | 128 | 766 / 1     | 126.2 | was `0 / 554`, 24.6 % of samples lost |
> | 6664 | 8   | 24179 / 2   | 8.0   | was `0 / 4493`; CPU 18.2 % → 5.1 % |
>
> **`n` now tracks `fifo_wm` exactly at every rate**, which is what the key was
> always documented to mean. The rule below — "`fifo_wm` has an effect only
> while `wm / odr_hz < 10 ms`" — no longer holds and must not be cited.
> `config/imud.conf` and `docs/config-keys.toml` carried the same claim and
> were corrected in `6ac3c66`.
>
> What is still owed here is the spec §14 rewrite itself, and one caveat found
> while re-measuring: the `lat_hist_t` buckets are **log₂ and saturate at
> ~1.05 s**, so `fifo=` p50/p95 are bucket boundaries. §14 cannot be written to
> better than a factor of two from that instrument without changing it.

*Original 2026-08-19 measurement follows, for the record.*

`16451c2` made the daemon report what wakes the reader and how much it finds.
The answer is a threshold, and it is sharp:

| `odr_hz` | `fifo_wm` | drains edge/timeout | mean n | `fifo` p50/p99 | time to fill `wm` |
|---|---|---|---|---|---|
| 104 | 8  | **0 / 4068** | 1.1 | 2.0 / 16.4 ms | 77 ms |
| 104 | 64 | **0 / 4069** | 1.1 | 4.1 / 16.4 ms | 615 ms |
| 833 | 8  | **4533 / 2** | 8.0 | 16.4 / 16.4 ms | **9.6 ms** |
| 833 | 64 | **0 / 3764** | 9.7 | 16.4 / 32.8 ms | 77 ms |

**The watermark interrupt fires in exactly one of the four cells.** Everywhere
else every single drain is a timeout drain. The rule:

> The reader takes whichever of the watermark and the 10 ms timeout comes
> first, so `fifo_wm` has an effect **only while `wm / odr_hz < 10 ms`**.
> Above that it has none at all, and mean burst depth is `odr_hz x 0.010`.

Mean depth predicted as `min(wm, odr x 0.010)` against measurement: 1.04 vs
1.1, 1.04 vs 1.1, 8.0 vs 8.0, 8.33 vs 9.7.

Run twice. The first pass changed `odr_hz` in `[mag]` as well as `[imu]`, so
the magnetometer ran at 1000 Hz throughout it; the second pinned the mag at
100 Hz and reproduced every cell — `0/4069`, `0/4071`, `4540/2 n=8.0`,
`0/3786 n=9.6`. The IMU drain split is a race between the FIFO fill rate and
the reader's timeout and does not care what the other chip select is doing,
which is what the pair of runs demonstrates rather than assumes. The one loose cell is `833/64`,
where samples keep arriving during the read itself.

**This explains the retraction above rather than merely repeating it.** At
`833/8` the two wakeups are 9.6 ms and 10 ms apart — within 4% — so which one
wins is decided by scheduling noise, and the 4533/2 split here is not
guaranteed to reproduce on a busier host. That is precisely the cell whose
maxima inverted between the 08-14 and 08-15 runs. It was never a watermark
effect; it was a race between two nearly-equal timers, and no model that
ignores the timeout could have predicted either result.

**So residence IS now predictable from configuration**, which is what §14's
rewrite was waiting for.

One outlier to keep: `104/64` recorded a `fifo` max of **1569 ms**. `max_ever`
survives the window reset, so this is a single excursion since start rather
than a sustained state, and the p50/p99 either side of it are unremarkable.
Unexplained; worth a second look before anyone quotes a worst case.

**§14 is fixed** *(2026-08-19, `b839e3f`)*. It states the chain, publishes all
three terms, gives residence as a formula an operator can apply, and labels
every unmeasured row as unmeasured. The reasoning that produced it is kept
below because it is why the rewrite took three bench sessions rather than one.

The likely
defect is the label rather than the number — "I2C sample" probably meant the
sample as delivered by the read, making the row a budget for the daemon's own
pipeline with FIFO residence excluded. Three things support that reading: the
adjacent 1.5 ms fusion row has the same structure and would be equally
impossible under the broad reading; the jitter row's own parenthetical calls the
FIFO a jitter absorber, so the author knew it buffers; and `manual.md` used to
present `fifo_wm` as a chosen latency cost. But that is a reconstruction, and as
written the row is wrong under its plain meaning.

**spec §14 is rewritten** *(2026-08-19)*. It states the chain rather than a
single end-to-end row, publishes all three terms, gives the residence rule as a
formula an operator can apply (`min(fifo_wm, odr_hz x 0.010)`), and labels every
still-unmeasured row as unmeasured instead of leaving silence to read as
agreement. This item is closed.

The instruction it was written against, kept because it is the reason the
rewrite took three bench sessions rather than one:

Do not simply relabel it. The number a control-loop or camera-sync consumer needs
is total sample age, and that appears nowhere. Define the chain and publish both
terms — which the `[stats]` clause now does, with `pipe` at 0.26 ms p99 measured
against the 1.5 ms row. What still blocks a rewrite of the end-to-end row is the
open `wm = 8` question above: total sample age is `fifo + pipe`, and `fifo` is
not yet predictable from configuration.

`manual.md`'s `fifo_wm` entry and the 77 ms arithmetic no longer contradict the
budget across two files — that entry now says what the key actually bounds.

### 3.2 Transport support matrix — which rates a bus can actually carry  *(measured 2026-08-24, SPI)*

Measured on the reference rig (Pi 5, ism330dhcx on `/dev/spidev0.0`,
mmc5983ma on `0.1`, `fifo_wm = 8`, mag 100 Hz), **after** the drain and burst
fixes. Delivered IMU rate from the daemon's own `[stats]` counters, differenced
over a window; "edges" is whether the watermark interrupt drove the reader or
the fallback timer did.

| IMU `odr_hz` | 1 MHz | 2 MHz | 4 MHz | 8 MHz | 10 MHz |
|---|---|---|---|---|---|
| 13.016 | 13.6 | 13.6 | 13.6 | 13.6 | 13.6 |
| 52.063 | 54.4 | 54.1 | 54.1 | 54.1 | 54.1 |
| 208.25 | 214.4 | 214.4 | 214.4 | 214.4 | 214.4 |
| 833 | 855.2 | 855.2 | 855.2 | 855.2 | 855.2 |
| 3332 | 3415 | 3419 | 3405 | 3418 | 3405 |
| **6664** | **0.0** | 6822 | 6812 | 6814 | 6821 |

Delivered rates run ~2.7 % above nominal throughout: that is the part's own
`FREQ_FINE` trim (+27 steps, a 24027 ns tick against the 25000 ns typical), not
an error.

**Two hard results:**

- **6664 Hz delivers nothing at all below 2 MHz.** Not degraded — zero. The
  payload alone is 6664 sets/s x 14 B = 93 kB/s = 746 kbit/s, roughly 75 % of a
  1 MHz bus before per-transaction overhead.
- **Edge recovery has its own, higher floor.** At 3332 Hz the reader was
  timer-driven at 1 and 2 MHz (`drains=0/1335`, `0/3763`) and interrupt-driven
  at 4 MHz and above (`10544/16`, `10241/29`, `10186/26`). A faster clock drains
  faster, the FIFO falls below the watermark, and the level line de-asserts.

**Caveat that bounds this table's usefulness**: cells where `fifo_wm / odr`
approaches the measurement window cannot be measured this way at all. At
`wm = 128` and 13 Hz a batch is 9.8 s, so a 20 s window holds two or three of
them and the counter differencing is meaningless — an early run reported 9.4
against 13.3 and that number is an artifact, not a loss.

**I2C is NOT measured.** This rig is SPI-only, so an I2C column would have to be
computed rather than observed, and the two must not be mixed in one table. The
arithmetic is straightforward — 400 kHz Fast-mode carries ~40 kB/s after
addressing overhead, so ~2800 sample-sets/s before any headroom — but it is a
calculation, and everything calculated on this project this month that went
unmeasured turned out to be wrong. Owed: rewire for I2C and measure, or publish
the column explicitly labelled as calculated.

## 4. ISM330DHCX MLC engine detection  *(pre-existing spec §16 item, optional)*

The current engine-vibration detection is software (EMA of (|a|−g)², with ×4 accel
noise inflation while active). The ISM330's on-chip Machine Learning Core could
assert a GPIO on engine-on instead. Only worth it if the software detector proves
finicky at sea.

1.7 fixed the detector's time constant (it previously sped up with FIFO burst
depth) and gave the threshold hysteresis, so the "finicky" bar is now higher
than it was. Scope this together with §10.5 (continuous rather than on/off
deweighting) — they are the detection and response halves of one problem.

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

## 6. Ideas beyond the roadmap above  *(brainstorm 2026-07-11, roughly by leverage)*

- **Multi-IMU.** Two sensor pairs fused, or at minimum hot-failover with
  cross-checking — the vessel-grade redundancy story (gpsd's multi-receiver
  support is the precedent the name invokes). See §11.5 for the architecture
  name (federated Kalman filter) and a scope estimate.
- **Ecosystem gravity on libimud.** The ABI-stable .so makes bindings nearly
  free: Rust -sys crate, Go package, Python cffi → crates.io/PyPI. Plus the
  shareable browser demo: live 3-D horizon over the WebSocket bridge. First
  satellite client shipped 2026-07-19: **imud-arduino**
  (github.com/richcreations/imud-arduino), the Arduino/ESP32 wire client in
  its own repository. It tracks the wire: updated to v17 (276 B) on
  2026-07-26, so it decodes a 1.7+ daemon. Its parser rejects any other
  version word outright, which is the point — but it means a wire bump has to
  reach that repository too, and it is not part of an imud tag. Keep it in the
  wire-change checklist.
- **SPI transport — SHIPPED.** `[imu] bus` / `[mag] bus` select `"i2c"` or
  `"spi"` per sensor, over a `bus_caps_t` each driver declares from its
  datasheet. ism330dhcx and mmc5983ma are exercised on both transports against
  a mock device; lsm6dso/lsm6dsox, icm42688p and lis3mdl are wired up and
  remain `experimental`. Three follow-ons remain:

  - **Hardware validation.** The claim that both transports produce identical
    register traffic is proven only against a mock. It needs an
    `imud-imutest --all` report on each bus from the same board, and the
    FIFO-drain latency comparison (§3.1's split makes it visible) that would
    substantiate the speed argument with a measurement rather than an
    arithmetic estimate. The sharpest test is **6664 Hz on the ISM330DHCX over
    SPI**: I²C at 400 kHz cannot carry that FIFO drain, so it is the rate the
    transport exists for and the one where a measured-ODR check either
    confirms the part runs there and the FIFO keeps up, or does not. (Those
    rates were only advertised by the driver from the ODR-coverage work that
    followed — before it, `supported_odr_hz` stopped at 1660 Hz and the
    transport could not deliver its own argument.)
  - **AKM compasses behind an InvenSense host.** icm20948 and mpu925x reach
    their on-die AK09916/AK8963 through I²C *bypass*, which only an I²C host
    can use. SPI needs the aux-I²C-master path — `I2C_SLV0_ADDR/REG/CTRL` plus
    `EXT_SENS_DATA_*` shadow polling — a genuinely different read path, so
    those boards run on I²C today.
  - **LIS2MDL.** Its SPI defaults to three wires, and 4-wire mode (CFG_REG_C
    bit 2) disables the data-ready line the driver reads it from. Either
    half-duplex `SPI_3WIRE` support in the bus layer, or a polled read path
    for the part — both change how it is read, not just how it is addressed.
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

## 8. Code consolidation  *(next-release candidates — deferred 2026-07-15, pre-1.5)*

Duplication clusters found in a pre-tag pass over the whole repo, deferred so
a refactor just before the release tag would not invalidate the testing 1.5
had already had. None blocks anything; each is a mechanical extraction plus
a re-test.

**Most of this section shipped in 1.6.2**, in a dedicated refactor commit
whose whole point was that it landed *after* a tag rather than before one —
zero behaviour change, −835 net lines, output verified byte-identical. What
remains is listed as open below; the rest is struck through with the release
that did it.

The standing rule still holds for the items that are left: don't refactor
immediately before a tag. The one item actioned in 1.7 was the Signal K
UDP-open drift, and only because it is a *user-visible bug* that happens to
delete a duplicate, not a refactor undertaken for tidiness.

- ~~**`sd_notify_msg()` ×6**~~ — **DONE 1.6.2.** `include/sdnotify.h` +
  `src/sdnotify.c`; the six byte-identical copies in `main.c` and the five
  bridge mains are gone. The name was kept, so no call site changed.
- ~~**Driver I2C register wrappers ×7**~~ — **DONE 1.6.2.**
  `src/drivers/i2c_io.h` (header-only, driver-private) supplies
  `i2c_burst_read`/`i2c_reg_write`/`i2c_reg_read` plus `i2c_s16le`/`i2c_s16be`;
  all ten hardware drivers include it. `static inline` on purpose: each
  driver still issues its own `ioctl(fd, I2C_RDWR, …)`, so `test_drivers`'
  `--wrap` mock keeps working and no Makefile source list changed. `lis3mdl`'s
  auto-increment became `reg | 0x80` at the call sites.
- ~~**Bridge stream-loop skeleton ×4–5**~~ — **DONE 1.6.2.**
  `include/bridge.h` + `src/bridge.c` — a helper library, not a framework:
  CLI parsing, config load/reload, signal install, tick math, stream
  connect/drop, `open_udp`, `write_all`. All five bridges converted. Each
  bridge keeps its own reload middle and protocol specifics.
- **`ism330dhcx.c` ↔ `lsm6dso.c` near-twin drivers** — *still open.* FIFO-tag
  drain loop, axis remap, and timestamp back-calculation are copy-paste; only
  WHO_AM_I and a CTRL9_XL write differ by design. No drift observed yet, but a
  bug fixed in one silently won't propagate to the other. **Deliberately held
  until `lsm6dso` is hardware-verified (§1)** — folding an unvalidated driver
  onto a validated one's code path would make the reference driver's behaviour
  depend on a merge nobody has tested on silicon.
- ~~**UDP-open drift**~~ — **FIXED 1.7.** `signalk_main.c`'s `open_udp_dest()`
  is deleted; the bridge calls the existing `bridge_open_udp()` helper, so
  `dest_addr` now accepts hostnames and IPv6 like every other bridge. Its
  systemd unit gained `AF_INET6` in the same change — see the refuted note
  below for why that was a *consequence* rather than a pre-existing bug.
  (Daemon *bind* and *send* addresses stay `inet_pton`-only by design — no DNS
  in the daemon's path.)
- **`crc32_ieee` ×3 in-tree C copies** — packet.c, libimud.c, mon_main.c
  could share one internal helper. The copies in `lib/imud_client.h`
  (self-contained header) and the Python client (zlib) stay by design.
- ~~**`imud.service` omits `AF_INET6`**~~ — **REFUTED 1.7, no action needed.**
  The concern was that a `localhost` resolving to `::1` would be blocked. It
  cannot: `src/position.c` pins `hints.ai_family = AF_INET`, so `getaddrinfo`
  never returns an IPv6 address for the gpsd/Signal K position clients. Every
  unit already matched its own code — the four bridges that resolve with
  `AF_UNSPEC` (`bridge.c`, influx, prom) all listed `AF_INET6`, and the two
  that pinned IPv4 did not. What *did* need it was `imud-signalk.service`,
  and only because the fix above moved that bridge to `AF_UNSPEC`. Recorded
  so the note is not "fixed" again on the daemon side, where it would loosen
  the sandbox for nothing.
- ~~**`FLAG_MOTION` (bit 6)**~~ — **RETIRED 1.7.** Decided rather than deferred
  again: everything a motion bit could report is already on the wire at higher
  fidelity — `accel_quiescence` as a continuous float, plus `FLAG_ENGINE_ON`
  (bit 13) — so a boolean restatement would only lose resolution. The define
  stays in all four headers so existing consumers compile; the bit will never
  be set and will not be reused, since a stale consumer would read a new
  meaning through the old name. Note this never needed a wire bump to decide:
  `flags` is a `uint16_t` and bits 14–15 are still free, and `types.h`'s rule
  is to bump only when the *layout* changes, so flag bits are additive.

## 9. Small items

- ~~`ctx->stop` in imu.c stays `volatile int`~~ — **DONE.** It is `_Atomic int`
  (`src/imu.c`), along with every other cross-thread stop flag; `test_concurrency`
  under TSan is the guard. Never add a plain or `volatile` cross-thread flag back.
- Heave settling: ~10·τ (≈2 min) after boot before heave is trustworthy. Could be
  shortened by initializing the integrators from the first seconds of data.
  Considered for 1.7 and deferred: it changes a settled estimator's startup
  behaviour, which is the same class of change as `align_window_sec` (§10.5) —
  and that one turned out to need a benchmark to get right, having been
  catastrophically wrong at its original value without anyone noticing.
- Centripetal correction models `v_body = [speed, 0, 0]` (no leeway); a leeway-angle
  estimate could refine it, but the residual is second-order for typical vessels.
  §11.1 replaces this approximation wholesale for the aerospace path, and would
  subsume the marine leeway question if pursued.
- ~~Validate capture → replay on real captures~~ — **DONE 2026-08-19.** A 24.8 s
  bench recording (21,645 IMU + 2,612 mag records, `ism330dhcx` + `mmc5983ma`
  over SPI) replayed in 27 s of wall clock, exited **0** on its own, and the
  daemon's counters landed on `imu=21645 mag=2612` — every record played, none
  lost at the tail. That is the first time this path has run on anything but
  synthetic data.

  **Confirmed again 2026-08-25 on a second, longer capture**, and this one
  exercised the fusion rather than stopping short: 6,502 IMU + 12,635 mag
  records over 119.9 s (`odr_hz = 52`, mag 105/s), replayed in the container
  with no hardware at all. The daemon consumed exactly those counts, reached
  alignment, and produced plausible attitude throughout. Note the first run
  above could not have shown this — a replay only emits `[stats]` once fusion
  is aligned, and the default 5 s settle plus the gyro-bias window swallows a
  short capture whole. `startup_settle_sec` has to be shortened for a replay to
  report anything.

  **It was NOT bit-deterministic; it is now.** *(fixed 2026-08-25.)* The
  original finding stands as recorded: the same file replayed twice gave an
  identical gyro bias and identical attitude at every shared sample count, but
  alignment consumed **2,521 mag samples one run and 2,539 the next**. Three
  separate causes were found by tracing the filter state per frame — quaternion,
  bias and `dt` as raw bits — and diffing whole runs rather than the 20 Hz
  `[stats]` cadence, which is too coarse to see any of this:

  1. **The aligner's mag average had no target count.** Its drain is nested
     inside the accel loop, so both averages always covered the same span —
     the earlier reading of this as a span mismatch was wrong. What varied was
     the sampling *density* of that span: 2,521, 2,539 or **47** samples out of
     roughly 8,000 available, decided by scheduling. Now bounded by the window,
     from the measured mag rate.
  2. **`dt` came off an anchor built on the replay machine's clock.**
     `chip_to_wall()` derives its slope from the (chip_ts, host time) pairs the
     reader collects, so the filter was a function of when the run happened.
     Three realtime replays diverged from frame 1081 and differed in **82.6%**
     of frames, in `dt` alone, by 1.2e-10 growing to 8.9e-08. The capture header
     carries the recording host's monotonic origin and matching realtime
     instant; anchoring on those reproduces the `dt` the live daemon computed.
  3. **Burst boundaries were a function of the clock.** Playback handed back
     whatever was due when the reader was scheduled, and the anchor pins to the
     LAST sample of a burst — so a boundary one sample either way moved the
     anchor, the slope, and every `dt` after it. This was the residue: with (2)
     fixed, three runs still split three ways at frame 4382 (**29.3%** of
     frames). The writer stamps one delivery instant per burst, so the recorded
     grouping is recoverable, and playback now replays it.

  With all three fixed, **three realtime replays of the same capture are
  bit-identical across all 6,201 frames**. `test_playback_replays_recorded_bursts`
  pins (3) against a fixture of known burst sizes.

  Two cautions for anyone diffing replays. `sim_speed = 0` is deterministic but
  **not faithful**: with pacing off the two streams lose their recorded
  relationship, the IMU outruns the 256-deep ring (723 of 6,201 frames dropped,
  `drains n=240.8 max=256`), and the aligner got **0** mag samples where the
  paced run got 500 — a heading of 3.8° against 315.4°. Diff at `sim_speed = 1`.
  And sample *counters* are not part of the guarantee; the fused output is.

  **The "replays slower than real time" trap is fixed too.** It was not
  inherent: both reader threads sleep a cadence sized for a part filling a FIFO
  in real time — `(depth + int_grace)` sample periods — at the top of every
  poll iteration, and `sim_speed` could not reach it because the sleep is on
  the daemon's side of the driver. A capture recorded with a ~105 Hz
  magnetometer was metered out at **33 Hz**, so a 125 s file took 372 s and
  fused a third of the field samples the live daemon had, arriving further
  behind the IMU stream as it went. Replay now bypasses both cadences: the same
  file replays in **120 s**, with `mag=12635` intact and the aligner seeing its
  full 500.

  The earlier note that `fifo=0.0/0.0ms` throughout a replay applied to the sim
  driver's *synthesis* mode. In playback `chip_ts` is the recorded one, so FIFO
  residence is measurable — and on the capture's timebase it is the residence
  the **live** daemon saw (16.4/32.8 ms on the bench file, against the
  1048.6/1048.6 ms the old batching reported, which was the backlog being swept
  into one read and called latency). `pipe=` stays on the replay machine's own
  clock, which is why `imu_sample_t` carries both instants.

## 10. Filter mathematics  *(opened 2026-07-25, with measurements taken while acting on it)*

Filter-mathematics items that were **not** actioned in 1.7, plus two findings
that came out of measuring the ones that were. 1.7 shipped the error-state reset Jacobian, the
reconfigure/engine-detector fixes, the mount-config validation, and the
update-gate health metrics; those are in NEWS and not repeated here.

Ordered by leverage. 10.1, 10.2 and the seaway half of 10.5 were resolved in
1.7 and are recorded below with their outcomes. What remains in this section is
the vibration half of 10.5 and the 3-D-mode NEES(strict) item it exposed.

### 10.1 The measurement model is over-confident  *(investigated 1.7 — premise confirmed, prescribed remedy refuted)*

**Status: partially resolved.** The investigation found a different and larger
problem first, fixed it, and established that the remedy this item proposed
cannot work. Instruments now exist in-tree; the remaining work moved to 10.5.

**What was actually wrong.** The gross-outlier reject gate sat at 9γ, which is
not far enough out to be a fault gate: over the 12-seed benchmark it discarded
**26% of 3-D accel updates** — routine wave-orbital motion, not outliers.
Starving the filter made it drift, the drift produced larger innovations, and
those tripped the gate more often still. Widening it to 25γ
(`GROSS_REJECT_MULT`) improves every measured quantity in both mag modes at no
cost, with `Ra` unchanged:

| | 3-D att | 3-D hdg | yaw att | yaw hdg | NEES(tr) 3-D/yaw | reject 3-D/yaw |
|---|---|---|---|---|---|---|
| 9γ (before) | 6.85° | 4.27° | 3.57° | 3.10° | 28.9 / 21.1 | .262 / .043 |
| 25γ (after) | 5.65° | 3.07° | 2.31° | 1.96° | 18.3 / 7.8 | .007 / .000 |

It also collapsed the worst-draw spread (3-D 13.7° → 7.0°, yaw 9.6° → 2.3°),
which is most of 10.8 — see there.

**A second real bug, found by cross-validating the two instruments.** The
`innov_weight` / `innov_reject` / `nis_*` EMAs used a per-update gain sized
from the IMU ODR, but the |a| skip band discards 85–95% of samples in a
seaway, so their true time constant was ten minutes, not 30 s, and they were
reporting their seed values rather than the data. The accel path now derives
the gain from elapsed time (`docs/math.md` §4.7). Every consistency figure
recorded before 1.7 was measured with the broken gain.

**The premise is confirmed.** With honest instruments, mean NIS over the
benchmark is **19 (3-D) / 25 (yaw-only)** where a consistent model reads 1 —
`Ra` really is over-optimistic by ~4.5× in σ, as this item claimed.

**The prescribed remedy is refuted.** Retuning `mekf_accel_noise` cannot fix
it. Sweeping it across four orders of magnitude (full table in `docs/math.md`
§4.7):

- NIS reaches 1.0 at `Na` ≈ 0.03, but that costs the **marine default**
  2.31° → 8.58° of attitude RMS and drives NEES(trace) 7.8 → 156. Weakening
  the gravity correction makes the filter lean on gyro integration, so the
  *state* error grows as the *innovations* start to look honest.
- NIS and NEES move in **opposite directions** with `Na`; no scalar satisfies
  both.
- NEES(strict) for 3-D stays pinned in the **44–64 band across the whole
  sweep**, so P's *shape* is wrong. An isotropic scalar cannot fix that.
- `Na ∈ [0.003, 0.03]` is markedly worse than either side of it, so the
  obvious hand-tune lands in the worst available region. Now documented in
  `imud.conf` and `man 5 imud.conf`.

The cause is that the seaway residual is wave-orbital — correlated with wave
phase, measured correlation time τ ≈ 0.3–0.9 s (`imud-cal fit-ra`) — and a
white isotropic R cannot describe a coloured disturbance. Getting its variance
right necessarily gets its spectrum wrong.

**Remaining work → 10.5 — since DONE.** Modelling the correlation was the
actual fix, and 10.5 implemented it as a Gauss–Markov augmented state: accel
NIS 19.3/25.2 → 1.01/0.69 with attitude RMS improved, not traded away. See
10.5 for the outcome, and for the `m33_inv` bug that turned out to underlie
much of what this item measured.

**Instruments shipped by this work** (all in-tree, no field data needed to
re-run):
- NEES in the wave benchmark, both scaled-trace and strict forms
  (`test_fusion.c`); the historical 28.4 figure is reproduced to within 2%.
- `nis_accel` / `nis_mag` on wire v17, via libimud, the Python client, and the
  Prometheus and InfluxDB bridges — this is 10.6's NIS half, delivered.
- `imud-cal fit-ra`: replays a real `.imucap` and reports the residual, its
  correlation time, the d² distribution against both the old and new gates,
  and the `Na` that *would* give NIS = 1. Writes nothing.
- `-DBENCH_SWEEP_RA` and `-DHUBER_VARIANT=n` rebuild switches, so both sweeps
  are re-runnable rather than folklore.

### 10.2 Covariance consistency under Huber capping  *(fix measured and rejected; re-measured 1.7)*

**The finding is real.** `eskf_update()` scales the innovation when the Huber
cap fires but leaves `K` — and therefore the Joseph covariance update — at
full confidence, so P contracts as though the measurement had been fully
trusted even when the correction was attenuated.

**The prescribed fix does not work, and this was measured, not argued.**
Re-measured in 1.7 after the 10.1 gate fix, since the earlier numbers were
taken while the reject gate was corrupting every run. Rebuild with
`-DHUBER_VARIANT=n`:

| n | variant | 3-D att | yaw att | NEES(tr) 3-D/yaw | NIS 3-D/yaw |
|---|---|---|---|---|---|
| 0 | **current** (ν-capping, shipped) | **5.65°** | **2.31°** | **18.3 / 7.8** | 19.3 / 25.2 |
| 1 | K ← w·K — exact Joseph for a suboptimal gain | 8.85° | 3.09° | 39.9 / 12.4 | 17.0 / 28.5 |
| 2 | R → R/w — IRLS-consistent inflation | 7.79° | 5.57° | 31.5 / 40.5 | 15.1 / 43.3 |
| 3 | R → R/w² — as prescribed | 8.48° | 4.74° | 35.0 / 27.9 | 18.2 / 36.3 |

Every variant is **both less accurate and less self-consistent** than what
ships — by a wider margin than before — and the prescribed R/w² variant
remains among the worst.

**This item is now closed, and the earlier reasoning was wrong.** It was
blocked on 10.1 in the belief that a corrected `Ra` would make `K ← w·K` a
straight improvement. 10.1 showed `Ra` cannot be corrected by retuning, and
the variants stay worse across the whole `Na` sweep, so the block was never
going to lift. The real reason they fail is that the cap is a *robustness*
device, not a statistical one: contracting P by the attenuated gain faithfully
records having learned less from that sample, but the consequence is a larger
P and hence a larger gain on the **following** samples — which in a seaway
carry the same wave contamination. The inconsistency does useful work, holding
the gain down exactly when measurements are least trustworthy.

Reasoning and measurements are recorded in-code at `eskf_update()` and in
`docs/math.md` §4.5, with the variants kept behind `HUBER_VARIANT` so this is
not re-attempted blind.

### 10.3 Quiescence-gated gyro process noise

`mekf_gyro_noise = 0.007` rad/s/√Hz against a raw sensor floor of ≈ 1.2e-4 is
a **~58× pad applied statically, at all times**, standing in for wave-induced
angular dynamics the smooth-rotation-plus-bias process model does not capture.
The proposal is to make it conditional, mirroring the `Ra_scale` pattern the
codebase already trusts for the accelerometer: sensor-realistic Qb when the
quiescence EMA (`acc_quiet_ema`, already computed and already gating m_ref
adaptation) says the platform is calm, inflating toward the current value when
it is not — the 58× pad becomes the *ceiling* for rough conditions rather than
the *constant* for all conditions.

Benefit is confined to calm stretches, where gyro bias could be tracked far
more tightly than the pad allows and where there is no wave dynamics for the
pad to be protecting against.

**Note the standing counter-argument:** `main.c` documents a deliberate
decision not to drive Q from measured sensor noise, because doing so made the
filter too stiff and roughly doubled benchmark attitude RMS. The conditional
scheme is compatible with that finding (it only lowers Qb when quiescent) but
must be validated against it — add a calm-water benchmark case measuring
gyro-bias settling time, and keep the existing rough-water case unchanged.

Smaller and more auditable than 10.7 (general adaptive Q), and the natural
first step toward it.

**Deferred from 1.7.** The counter-argument above is not hand-waving — it is a
recorded measurement — and validating against it needs a calm-water benchmark
case measuring gyro-bias settling time, which does not exist yet. Same shape as
the 10.5 vibration half: the missing scenario is the work.

### 10.4 Geometric refinement of magnetometer fits  *(low priority)*

`sphere_fit()` and `ellipse_fit()` minimise algebraic, not geometric, residual
— a known bounded bias for sparse or uneven calibration swings, already
mitigated operationally by `cal_cov_mark()` coverage tracking. Adding 2–3
Gauss-Newton refinement steps seeded from the existing algebraic solution
would remove most of the remainder without disturbing the incremental
accumulation structure. `gauss4()` is reusable but fixed at 4×4, so a 6–9
parameter refinement needs a `gaussN` or block padding.

Skip unless coverage-gated algebraic fits stop meeting accuracy targets in the
field — nice-to-have, not correctness-critical for heading-only marine use.

### 10.5 Correlated measurement noise / continuous deweighting  *(seaway half RESOLVED 1.7 via candidate 2; vibration half still open)*

Two problems that turned out to be the same problem. One is now solved.

**The seaway half (from 10.1) — RESOLVED.** Implemented as candidate 2, the
augmented state: a first-order Gauss–Markov wave-acceleration component
appended to the error state (δx is now 9-D), tuned by `mekf_wave_accel` (σ,
m/s², default 0.8) and `mekf_wave_accel_tau_s` (τ, s, default 0.5). Either at 0
disables it and returns the pre-1.7 6-state filter bit-for-bit. Full derivation
and outcome table in `docs/math.md` §4.1.1, §4.5.1 and §4.7.1.

Measured over the 12-seed wave benchmark against the baseline this item named
as the bar to beat:

| | 3-D att | 3-D hdg | yaw att | yaw hdg | NEES(tr) | NIS_a | reject |
|---|---|---|---|---|---|---|---|
| baseline (1.6) | 5.653° | 3.065° | 2.309° | 1.961° | 18.3 / 7.8 | 19.3 / 25.2 | .007/.000 |
| **1.7** | **4.452°** | **0.828°** | **2.308°** | **1.016°** | **3.47 / 0.99** | **1.01 / 0.69** | .000/.000 |

Success criterion met: NIS → 1 with **no** accuracy regression — 3-D attitude
−21%, 3-D heading −73%, yaw heading −48%, yaw attitude a dead heat — and both
the Huber cap and the gross-reject gate now completely idle in an ordinary
seaway (weight 1.000, reject 0.0000).

Candidate 1 (wave-phase-aware `Ra_scale`) was **not** implemented and is not
needed for the seaway: it modulates the *variance* of a white model, which
10.1 established cannot describe a coloured disturbance whatever the scale.

**A larger bug found on the way.** `m33_inv` tested for singularity on an
absolute `|det| < 1e-12`, but S for the gravity update carries physical units
and its determinant sits near 1e-13 at perfectly ordinary conditioning
(condition number ≈ 3). **87% of accel updates in the wave benchmark were
being silently discarded**, and the health EMAs were fed only by the 13% that
survived — so every number recorded in 10.1, 10.2 and the gate tables was
measured through a filter that was mostly not using its accelerometer. The
accidental decimation was doing real work, crudely decorrelating the
wave-contaminated samples; with it removed and no wave state the filter
diverges outright (9.5°/11.4° attitude, NIS ≈ 56, 15% rejects). The test is now
relative to the matrix scale. This is the strongest single argument for the
augmented state: the filter was only ever surviving the seaway by accident.

**3-D NEES(strict) — RESOLVED, and the first diagnosis was wrong.** This was
recorded as "the swing-circle mag calibration is structurally 2-D, so the dip
channel pulls on roll and pitch". That cannot have been it: the benchmark
synthesises magnetometer data from the true attitude with nothing but white
noise, so there is no calibration error in it at all. Two real causes:

1. **A benchmark-fidelity bug (~96% of it).** The scenario aligned from one
   instantaneous sample taken mid-roll and commented that this was "like the
   daemon". It is not — the daemon averages a window. That single sample baked
   a −4.38° DIP error into `m_ref`, which in 3-D mode is a permanent roll/pitch
   bias P has no term for. Aligning as the daemon does: 3-D attitude RMS
   4.452° → 1.204°, NEES(strict) 335 → 12.8.

2. **The daemon's alignment window was too short.** Fixing the benchmark
   exposed that the window was a hardcoded ~1 s — about a fifth of a roll
   period, so in a seaway it aligns to an arbitrary point in the cycle.
   Measured attitude RMS in the marine default: **47.7° at 1 s**, 2.28° at 2 s,
   2.19° at 5 s, flat thereafter. Now `align_window_sec`, default 5 s. This is
   a real field fix, not a benchmark artefact — any install that starts up
   underway was getting it.

The residual dip error (+0.86° after a 5 s window) is **not observable from
seaway data**: the in-run m_ref healing is gated on quiescence, and raising
that gate is refuted over 30 minutes — `m_ref` walks past truth and NEES(strict)
goes to 42. It is removed at the source by WMM invariants, or admitted into P,
which is what `mekf_mag_dip_sigma_deg` now does: a rank-1 anisotropic term in
the 3-D mag update's R along the one direction a dip error acts on. At the
shipped 1.0° (the measured residual, rounded up) NEES(strict) goes 12.8 → 5.74
with attitude and heading both slightly better and yaw-only bit-identical.
Isotropic inflation reaches the same NEES but collapses `nis_mag` to 0.01,
destroying the wire's magnetometer-health instrument — hence rank-1.

It does not reach 1 and cannot: a dip error is a *bias*, and a covariance term
only partly stands in for one. The best configuration is 3-D **with** a
position source: attitude 0.841°, NEES(strict) 0.22 — now a printed benchmark
row, previously unmeasured. Full derivation and sweeps in `docs/math.md`
§4.3 and §4.8.1.

**The vibration half — still open, deferred from 1.7.** 1.7 fixed
the detector's time constant and added threshold hysteresis, but the response
is still a hard 4× on/off (`src/imu.c`, `f.Ra_scale = ctx->engine_on ? 4.0f :
1.0f`): over-deweighted at low vibration, under-deweighted at high. A
continuous mapping from the existing EMA (e.g. `Ra_scale = 1 + k·e`, clamped)
reuses state that already exists. Note this is now a genuinely separate
problem from the seaway half: engine vibration is high-frequency and
near-zero-mean, so it really is closer to white and really is a variance-
scaling problem — which is why candidate 1 remains the right shape for it.

**Why it did not ride along in 1.7, since the code change looks small:** the
12-seed benchmark is seaway-only. There is no vibration scenario in it, so any
continuous mapping would have been tuned against nothing and asserted against
nothing. Building that scenario is the actual work, and it is a prerequisite —
1.7 is a release in which two separate plausible-looking filter changes (a
retuned `Ra`, and opening the `m_ref` quiescence gate) were each refuted only
by measuring them properly. Do the scenario first.

Scope the vibration half together with the MLC hardware-detection option in §4
— they are the detection and response halves of one problem.

### 10.6 Filter self-monitoring: NEES/NIS  *(NIS shipped 1.7; NEES remains bench-only)*

**Shipped.** Rolling NIS is on wire v17 as `nis_accel` / `nis_mag` (τ ≈ 30 s,
normalised by effective dof so 1.0 = consistent), accumulated pre-cap and
including rejected updates, and exported through libimud, the Python client
and the Prometheus and InfluxDB bridges. `imud-cal fit-ra` computes the same
statistic offline from a capture. Derivation in `docs/math.md` §8.2.

It did its job immediately: it is what confirmed 10.1's premise, and
cross-validating it against the offline tool exposed the health-EMA gain bug
described there.

**Still open.** NEES exists only in the benchmark (`test_fusion.c`, both
scaled-trace and strict forms) because it needs ground truth, which the daemon
does not have at runtime. A runtime proxy — e.g. comparing P against the
observed spread of the state estimate itself — would be a genuinely new thing
and is the remaining part of this item. Lower priority now that NIS is live.

### 10.7 Online adaptive process noise  *(depends on 10.6, follows 10.3)*

The general form of 10.3: adjust `Qg`/`Qb` (and possibly `Ra`) continuously
from the live innovation sequence rather than switching between fixed levels
— useful when conditions shift within a run at intensities a discrete
quiescence gate cannot capture. Needs care to avoid the classic adaptive-KF
failure mode of noise estimates chasing transient disturbances. Land 10.3
first: it is smaller, reuses a proven in-tree pattern, and its calm/disturbed
split will produce real data on how much benefit adaptive Q actually delivers.

### 10.8 Benchmark tail spread  *(new — observed while adding the multi-seed benchmark)*

**Largely explained by 10.1 — it was a filter weakness, not scenario luck.**
The spread was: 3-D attitude RMS mean 6.85° against a **13.68°** worst draw,
yaw-only 3.57° against 9.59°. The suspicion recorded here — that this deserved
one investigation because a filter weakness would be a more important finding
than anything else in the section — was correct.

The cause was the 9γ gross-reject gate (10.1). Unlucky draws pushed the filter
into the rejection-feedback regime, where lost corrections cause drift, drift
causes larger innovations, and those cause more rejections. Widening the gate
to 25γ collapsed the tail:

| | 3-D mean / worst | yaw mean / worst |
|---|---|---|
| 9γ | 6.85° / 13.68° | 3.57° / 9.59° |
| 25γ | 5.65° / 7.02° | 2.31° / 2.34° |

The yaw-only tail is now essentially gone (worst draw within 1.5% of the
mean). The 3-D tail is reduced but still ~1.24× the mean, which is a plausible
amount of genuine scenario luck. The benchmark still asserts on the mean only.
**Remaining work:** none urgent; revisit only if the 3-D tail grows again.

### 10.9 The filter across every supported sample rate  *(new — measured, mostly resolved)*

Every accuracy and consistency figure in this tree existed at exactly one point
in a two-dimensional space: 833 Hz IMU, ~104 Hz mag. The drivers advertise **27
IMU rates from 12 Hz to 8 kHz and 13 mag rates from 1 Hz to 1 kHz** — 351
pairings, each a config `imud.conf` accepts, of which one had ever been
measured. `odr_hz = 12` was, on paper, as supported as `odr_hz = 833`.

Now instrumented. `test_rate_derivations` walks all 351 on every build (derived
tuning correct, filter numerically sane — no accuracy claim, since `Ra ∝ odr`
retunes the filter at every rung). `BENCH_SWEEP_ODR` measures accuracy along two
axes. Full tables and reproduction commands in `docs/math.md` §4.7.2.

**Resolved.**

- **Accuracy is very nearly rate-independent.** 3-D attitude RMS is
  1.156°–1.252° across the whole 12 Hz–8 kHz range, ±4% over 667×. A low-power
  board at 104 Hz gives up essentially nothing — which was the question that
  prompted this, and the answer is yes.
- **Heave and sea-state are clean end to end.** Heave recovers a known sinusoid
  within 1.2% at every rate (worst at 12 Hz, not 8 kHz); sea-state holds its
  historical 200 Hz bounds unchanged, worst Hs error 1.0%. The float-accumulation
  risk predicted at the top of the ladder does not materialise.
- **The wave state is load-bearing at every rate**, and more so as the rate
  falls — not a fix specific to 833 Hz.

**The floor: 12 Hz, yaw-only mode only, and it is the gate.** yaw attitude RMS
goes 1.980° (52 Hz) → 3.949° (14 Hz) → **7.673° (12 Hz)**, with the gross-outlier
gate going from idle to rejecting 10.5% exactly at the cliff. That is the
rejection-feedback regime §10.1 identified. 3-D at 12 Hz is unaffected (1.210°).
Note yaw-only is *best* around 40–52 Hz, not at the 833 Hz default.

*Action, if any:* a documented minimum for yaw-only rather than a code change.
Nothing is retuned on the strength of this.

**Still open — the one that matters.** NIS_a varies monotonically with rate and
crosses 1.0 near 150–200 Hz: 15.66 at 12 Hz, 2.11 at 104, 0.63 at 833, 0.42 at
8 kHz. Retuning does not fix it: no point in the (σ, τ) grid at 104 Hz brings it
near 1, and — decisively — `SCEN_GM`, the scenario whose right answer is known in
advance, reports NIS_a = 0.87 at 833 Hz and **≈3.4 at 104 Hz** with the knobs set
to the truth. This is not a tuning choice, not a scenario artifact, and not the
harness (`Ra` over-estimates the injected per-sample variance by 4.480× at both
rates, identical to four figures). Something in how S is composed is
rate-dependent. **This is the successor to §10.1** and wants the same treatment:
measure before prescribing.

**Also still open — §10.3 is now half-answered.** The RMS-optimal
`mekf_gyro_noise` is 0.002 at 833 Hz and 0.004 at 104 Hz, roughly √Δt, which
supports intra-sample rotation nonlinearity as part of what the 58× pad absorbs
rather than wave dynamics alone. But RMS and NEES(strict) move in opposite
directions exactly as they do for `Ra` (§10.1), so no scalar is right and the
default stands. The shipped 0.007 turns out to be near-optimal for yaw-only and
~9% off for 3-D — a compromise favouring the marine default.

**Not tested, and it should be said plainly.** The first-order `Φ = I + Fc·dt`
and `docs/math.md`'s claim that `‖ω‖dt ≪ 1 at supported ODRs` remain unmeasured.
The wave scenario peaks at 18.8 °/s, so `‖ω‖dt` is 0.027 rad even at 12 Hz —
nowhere near the limit, where 2000 °/s at 12 Hz would be 2.9 rad. Probing it
needs a high-rate-rotation scenario that does not exist. Same shape as §10.3 and
§10.5: **the missing scenario is the work.**

### Reviewed, no action

- **Heave DC-gain trade-off** — the structural high-pass zero at
  DC trading long-period swell accuracy for drift immunity is already
  documented as deliberate. If long-period swell accuracy ever becomes a
  requirement, the upgrade path is a model-based/complementary estimator (and
  a design doc), not a patch to the leaky integrator. See also 11.3.
- **Timestamp inter-anchor drift** — linear interpolation between
  60 s re-anchors carries no oscillator drift model. Below the gyro noise
  floor in practice; revisit only if a specific chip's drift spec says
  otherwise.
- **GNSS dual-antenna heading** — considered and dropped, and worth recording
  as an anti-pattern: it needs hardware not on the target fleet, and
  single-antenna GNSS "heading" is actually COG, so conflating the two would
  silently bias attitude by the leeway/crab angle.

## 11. Aerospace generalization  *(additive; the marine path stays the default)*

`spec.md`'s opening already describes imud as general-purpose across marine,
robotics, machine vision, and gimbal/pointing use. This section closes the gap
between that stated scope and the marine-specific defaults actually
implemented (2-D mag swing, scalar-speed centripetal correction, static-only
alignment, Euler-first output framing) — it is not a new-domain pivot.

**Hard constraint on every item:** the marine configuration must remain the
default and must not regress. Each item is a new code path selected by config,
never a replacement. Regression-test against marine captures before merge.

### 11.1 Full 3-D velocity-aided specific-force compensation  *(B1 — blocking for everything else here)*

`mekf_update_accel()` assumes body velocity ≈ [v, 0, 0] and corrects only the
yaw-plane centripetal term. Correct for a surface vessel; systematically wrong
for an aircraft in a coordinated turn, climb, or descent, where specific force
departs from gravity on all three axes for sustained periods — the documented
failure mode is attitude "levelling out" mid-turn as the accelerometer takes
the resultant for down.

Needs the full `a_platform = v̇ + ω × v` correction from 3-D velocity, gated on
a config flag, falling back to the existing scalar-speed path unchanged.

**Smaller than it looks:** the `position` thread already holds a live gpsd
connection, and gpsd's TPV messages already carry 3-D velocity and climb rate.
This parses more fields out of a connection that is already open — not a new
integration. Related: the leeway note in §9 is the marine end of the same
approximation.

### 11.2 Consequence and follow-on items

- **Accel gate/skip behaviour under sustained manoeuvring** *(B2)* —
  verification only, no structural change expected: once 11.1 recovers true
  gravity direction, the existing skip band and χ² gate should behave. Confirm
  on sustained turn/climb logs; retune only the aerospace profile if not.
- **3-D magnetometer calibration** *(B3)* — the hard-iron sphere fit is already
  3-D and reusable; the soft-iron fit is 2-D-only (matched to a boat's
  necessarily-horizontal swing), so its Z-couplings come from config rather
  than measurement. Add an `ellipsoid_fit()` alongside `ellipse_fit()` —
  same algebraic-fit family, selected by a swing-type flag (planar vs tumble).
  The fusion math needs no change: `mag_yaw_only=false` and the existing
  full-vector update already exist.
- **In-motion / velocity-aided alignment** *(B4)* — `mekf_align()` requires a
  static accel+mag reading. Add a GPS-velocity-aided path for filters starting
  while already taxiing or airborne, selected automatically by whether a valid
  still window was found (not a config choice that could regress the marine
  case). Same gpsd data as 11.1; follow the existing ~1 s averaging pattern
  rather than a single-instant fix. Established technique, worth adopting its
  name ("align in motion") in the docs.
- ~~**Non-Euler attitude output** *(B5)*~~ — **SHIPPED 1.7.** Documentation
  only, as scoped: the packet already carried the quaternion and its
  covariance. `spec.md` gains a "Near-vertical pitch" subsection stating that
  the Euler fields are ZYX intrinsic and singular at pitch ±90° by
  construction — roll and yaw stop being separately determined approaching
  vertical, taking `heading_deg` and `rate_of_turn` with them — and that
  consumers operating there must read `quat_*`, which is never singular, as is
  `cov[9]` in its tangent space. Cross-referenced from `docs/libimud/spec.md`
  (with the affected fields flagged in the field table) and `docs/manual.md`,
  where it also notes the NMEA sentences are Euler by definition.
- **Aerospace config profile** *(B6)* — once the above are validated, a second
  named config file with aircraft-appropriate Qg/Qb/Ra, revisited gate
  constants, and `mag_yaw_only=false`. No code change beyond the items above.
- **Not in scope** *(B7)* — heave, sea-state statistics, and engine-vibration
  detection are marine-specific and simply would not be enabled. No conflict.

### 11.3 Vertical-channel aiding  *(splits into two very differently-sized tickets)*

- **GPS-altitude aiding** *(small)* — reuses 11.1's gpsd plumbing (TPV carries
  altitude). For marine this lets the heave estimator reference an absolute
  vertical measurement instead of relying solely on the high-pass to bound
  drift, partially addressing the A7 long-swell limitation noted in §10.
- **Barometer aiding** *(much larger)* — the hardware profile and driver
  registry contain no barometric-pressure chip at all, so this needs a new
  driver *class* alongside the existing IMU and magnetometer abstractions, not
  just a new fusion update step.

For aerospace, absolute altitude/vertical-rate is a baseline expected input
that the filter currently has no path for at all.

### 11.4 Airspeed / air-data aiding  *(depends on 11.1, aerospace only)*

Once 11.1 lands, feeding the same 3-D velocity-aided path from airspeed plus a
wind estimate would keep the filter accurate through GPS-denied segments — a
more routine concern in aviation than at sea. Unlike 11.1 and 11.4's
predecessors, there is **no existing ingestion path** for air data of any kind:
no driver class, no thread, no config section. This needs a new input channel
built from scratch, so it is a larger first step than "depends on 11.1"
suggests. Gate entirely behind the aerospace profile; no marine analog.

### 11.5 Multi-IMU redundancy and voting

Tracked in §6 as "Multi-IMU". Two things worth recording: the standard name
for this architecture is the **federated Kalman filter**
(Carlson, NAECON 1988), and it is the largest item in this document
— the entire daemon is built around exactly one IMU and one
magnetometer, so this is running the whole per-sensor pipeline N times plus
new arbitration/voting logic that does not exist anywhere today, not a config
change. Only pursue against an explicit redundancy or certification
requirement, and write a design doc first.

---
*Compiled 2026-07-06; output-bridges section added 2026-07-08; ideas section
+ N2K demotion added 2026-07-11; pruned of shipped items 2026-07-19 (1.6);
filter-mathematics and aerospace sections (§10, §11) added 2026-07-25, pruned
of what 1.7 shipped; triaged 2026-07-26 at the close of
1.7 — §8's UDP drift fixed, its `AF_INET6` note refuted, `FLAG_MOTION` retired,
B5 shipped, and the items 1.7 deliberately left (§8 consolidation, §9 heave
init, §10.3, §10.5 vibration half) annotated with why; §0 forward plan added
2026-08-05, synthesised from the sections below rather than from new decisions.
Not yet triaged against 1.8 — §§1–11 still read as they did at the close of
1.7; §10.9 added 2026-08-08 from the sample-rate sweep, §3.1 the same
day when the latency instrument shipped without its measurement.*
