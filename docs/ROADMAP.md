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

**Planned.**

- **Clear the hardware validation backlog (§1).** Ten drivers ship marked
  `experimental = true` and have never met real silicon. `imud-imutest` reduces
  each one to a single command plus three physical checks, so this is bench
  time rather than design work. The reference pair is validated; the rest clear
  opportunistically as boards become available.
- **Confirm the 1.7/1.8 changes on hardware (§1).** The alignment window, the
  accel-update duty fix, and the measured sample clock have run only in CI and
  simulation. This gates confidence in most of §10.
- **Fit real gyro temperature coefficients (§2).** The mechanism shipped in
  1.5; what is missing is a cold-boot-to-warm capture from real hardware.
- **Finish measuring sample latency on silicon and fix spec §14 (§3.1).** The
  pipeline term is now measured — 0.26 ms p99 on a Pi 5, against a 1.5 ms
  budget. FIFO residence is measured on I²C but is drain-rate-bound rather
  than watermark-bound, which ties it to §1.1's DRDY item; SPI is unmeasured
  entirely. One of the three budgeted numbers still looks wrong as written.
  Same bench session as §1 and §2.
- **Confirm 1.9.0's timestamp sourcing on silicon (§1.1).** All three
  follow-ons from the 1.9.0 RC bench run are implemented and unit-tested, and
  two of them rest on datasheet behaviour no hardware has confirmed: the ST
  tag-`0x04` payload layout and whether `TAG_CNT` participates for it. Both
  are cross-checked at runtime and degrade rather than corrupt, but a bench
  pass is what turns "degrades safely" into "works".
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

### 1.1 Timestamp sourcing, from the 1.9.0 RC bench run  *(confirmed on silicon 2026-08-14)*

The 2026-08-10 Pi 5 session on the reference pair produced four findings. Two
were shipped fixes (the ARM seccomp filter, the gpio udev trigger); the rest
are below.

**All three are now confirmed on hardware** by the 2026-08-14 run: `chip_ts`
monotonic with 0 reversals, the batched FIFO timestamp **accepted** on this die
with no fallback logged, `imu.chipts.wall` **0.9998** against 1.041 before the
part's own `FREQ_FINE +27` trim was applied, and the header reading
`ts_tick_ns = 25000 typical, 24027 from this part`. The bench notes each item
still owes are kept below for the next part that is not this one.

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

- **The DRDY edge rate fits no model** *(instrument shipped — measurement
  still owed)*. `imu.drdy.edges` reported ~18.3 Hz on the reference IMU at
  833 Hz with `fifo_wm = 64`. Ruled out by inspection: word accounting
  predicts 13.6 Hz (2 words per sample-set at the measured 866.7 Hz, plus
  temperature at 12.5 Hz, against a 128-word watermark); the tool drains while
  counting, so it is not a stalled FIFO; and both the tool and the daemon
  request rising edges only, so it is not double-counting.

  Remaining hypothesis: `INT1_FIFO_TH` is a *level* condition and the level
  oscillates across the threshold during the drain, as words are consumed
  while new ones arrive, with each crossing producing another rising edge.

  `imud-imutest` now counts twice over the same window — once draining on
  every edge, once not — which separates the candidates: a level condition
  asserts once and stays asserted with nothing emptying the FIFO, while an
  edge-per-sample line keeps pulsing. Both counts are in the report appendix.
  The check no longer grades a part down for fitting neither model, because an
  edge count never could identify one; 0 edges still FAILs, and a rate above
  the part's own sample rate still WARNs.

  **Owed to the bench.** The second number. Note the word accounting above
  rests on `FIFO_CTRL4 = 0x26`, which is now `0xE6` at the shipped watermark —
  the ÷32 timestamp words add 1.6%, moving the prediction from 13.6 to about
  13.4 Hz, which does not change the conclusion that 18.3 fits neither model.

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

### 3.1 spec §14's latency budgets, one of them now measured  *(the pipeline term is in; FIFO residence and SPI still owed)*

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

**Measured, on the 2026-08-14 Pi 5 run** — the full matrix, 120 s per
combination, `ism330dhcx` on I²C, six combinations all at `ovf = 0`. Worst
case (`max=`, which is exact; see the caution below):

| `odr_hz` | `fifo_wm` | `fifo` worst | `pipe` p99 | `wm/odr` says |
|---|---|---|---|---|
| 104 | 8 / 32 / 64 | 12.7 / 26.0 / 25.5 ms | 0.06 ms | 77 / 308 / 615 ms |
| 833 | 8 / 32 / 64 | 56.8 / 55.5 / 24.4 ms | **0.26 ms** | 9.6 / 38 / 77 ms |

**`pipe` — the term imud controls — is 0.26 ms p99, six times under §14's
1.5 ms budget**, and identical across every combination. That is the number
§14's fusion-latency row was always about, and §3.1's original purpose.

`fifo` is the interesting one, and it says something other than what this
section assumed:

- **It does not scale with `fifo_wm` at all**, and the shallow settings
  measured *worse*. That is because the watermark is not what triggers a
  drain: the reader waits with a **10 ms timeout** (`src/imu.c`) and drains
  whatever is there when it expires, so residence is bounded by that cadence
  plus the read itself. `imud-imutest` measured the same thing independently
  as `max read-loop gap 10.2 ms`. At 833/64 that predicts ~10 ms of
  accumulation plus ~3 ms of bus time, against a measured 24.4 ms worst case;
  at 104 Hz roughly one sample per drain, against 12.7 ms. §1.1's finding that
  `INT1_FIFO_TH` is a *level* condition is the same fact from the other side.
- **`wm/odr` was never the right model**, and this section said it was. It is
  the residence of the oldest sample in a burst *drained at the watermark*,
  which is not the configuration anyone runs.
- **Reported percentiles are log₂-bucket upper edges.** A `p99` of `16.4 ms`
  means the true value is in `[8.2, 16.4)`. The 2026-08-14 run's own data
  proves it: at 104/8 the exact `max = 12.7 ms` sits *below* the reported
  `p99 = 16.4 ms`. Conservative by design (`imu_math.h`), but it means a
  percentile compared against a point model overstates by up to 2×. Use
  `max=` for a number.

**Open, and the one thing here that no model predicts.** At 833 Hz, `wm = 8`
and `wm = 32` measured 56.8 and 55.5 ms worst case against `wm = 64`'s
24.4 ms — 2.3× *worse* at a shallower watermark. Everything above says the
drain cadence should dominate and the watermark should barely matter. To
settle it: log per-drain sample counts and the wake reason (edge vs timeout)
at `wm = 8` against `wm = 64`, which distinguishes a longer cycle from deeper
bursts.

Still owed: the same pair on SPI, and at the rates only SPI can carry.

**Then fix §14** *(still owed — the numbers to do it with now exist)*. The likely
defect is the label rather than the number — "I2C sample" probably meant the
sample as delivered by the read, making the row a budget for the daemon's own
pipeline with FIFO residence excluded. Three things support that reading: the
adjacent 1.5 ms fusion row has the same structure and would be equally
impossible under the broad reading; the jitter row's own parenthetical calls the
FIFO a jitter absorber, so the author knew it buffers; and `manual.md` used to
present `fifo_wm` as a chosen latency cost. But that is a reconstruction, and as
written the row is wrong under its plain meaning.

Do not simply relabel it. The number a control-loop or camera-sync consumer needs
is total sample age, and that appears nowhere. Define the chain and publish both
terms — which the `[stats]` clause now does, with `pipe` at 0.26 ms p99 measured
against the 1.5 ms row. What still blocks a rewrite of the end-to-end row is the
open `wm = 8` question above: total sample age is `fifo + pipe`, and `fifo` is
not yet predictable from configuration.

`manual.md`'s `fifo_wm` entry and the 77 ms arithmetic no longer contradict the
budget across two files — that entry now says what the key actually bounds.

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
- Validate capture → replay on real boat captures (the synthetic path is
  regression-tested); the §2 thermal capture can double as the test file.
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
