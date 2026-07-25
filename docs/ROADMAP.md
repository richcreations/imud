# imud — Deferred & Future Work

Items identified during the 2026-07-06 audit and later passes that remain
open. Completed work is tracked in git history, NEWS, and spec.md §6, not
here.

Sections 1–9 predate the 2026-07-25 external audit
(`imud-audit-and-aerospace-roadmap.md`); §10 and §11 carry what that audit
left open after 1.7, plus findings from measuring it. Where the two overlap,
the older section is authoritative and the newer one cross-references it.

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

## 6. Ideas beyond the audit roadmap  *(brainstorm 2026-07-11, roughly by leverage)*

- **Multi-IMU.** Two sensor pairs fused, or at minimum hot-failover with
  cross-checking — the vessel-grade redundancy story (gpsd's multi-receiver
  support is the precedent the name invokes). See §11.5 for the architecture
  name (federated Kalman filter) and a scope estimate.
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
  §11.1 replaces this approximation wholesale for the aerospace path, and would
  subsume the marine leeway question if pursued.
- Validate capture → replay on real boat captures (the synthetic path is
  regression-tested); the §2 thermal capture can double as the test file.
## 10. Filter mathematics  *(from the 2026-07-25 external audit + measurements taken while acting on it)*

Findings from `imud-audit-and-aerospace-roadmap.md` (external document) that
were **not** actioned in 1.7, plus two findings that came out of measuring the
ones that were. 1.7 shipped the error-state reset Jacobian, the
reconfigure/engine-detector fixes, the mount-config validation, and the
update-gate health metrics; those are in NEWS and not repeated here.

Ordered by leverage. 10.1 and 10.2 were investigated in 1.7 and are recorded
below with their outcomes; **10.5 is now the most valuable open item in this
section**, having inherited the real remaining work from 10.1.

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

**Remaining work → 10.5.** Modelling the correlation is the actual fix: an
augmented state for the wave-acceleration component, or the continuous
`Ra_scale` of 10.5. That is now the highest-value item in this section.

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

### 10.2 Covariance consistency under Huber capping  *(audit A1 — fix measured and rejected; re-measured 1.7)*

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
| 3 | R → R/w² — as the audit specifies | 8.48° | 4.74° | 35.0 / 27.9 | 18.2 / 36.3 |

Every variant is **both less accurate and less self-consistent** than what
ships — by a wider margin than before — and the audit's own R/w² prescription
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

### 10.3 Quiescence-gated gyro process noise  *(audit A3)*

`mekf_gyro_noise = 0.007` rad/s/√Hz against a raw sensor floor of ≈ 1.2e-4 is
a **~58× pad applied statically, at all times**, standing in for wave-induced
angular dynamics the smooth-rotation-plus-bias process model does not capture.
The audit proposes making it conditional, mirroring the `Ra_scale` pattern the
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

### 10.4 Geometric refinement of magnetometer fits  *(audit A4 — low priority)*

`sphere_fit()` and `ellipse_fit()` minimise algebraic, not geometric, residual
— a known bounded bias for sparse or uneven calibration swings, already
mitigated operationally by `cal_cov_mark()` coverage tracking. Adding 2–3
Gauss-Newton refinement steps seeded from the existing algebraic solution
would remove most of the remainder without disturbing the incremental
accumulation structure. `gauss4()` is reusable but fixed at 4×4, so a 6–9
parameter refinement needs a `gaussN` or block padding.

Skip unless coverage-gated algebraic fits stop meeting accuracy targets in the
field — nice-to-have, not correctness-critical for heading-only marine use.

### 10.5 Correlated measurement noise / continuous deweighting  *(audit A6 + the residue of 10.1 — now the highest-value item here)*

Two problems that turn out to be the same problem.

**The vibration half (audit A6).** 1.7 fixed the detector's time constant and
added threshold hysteresis, but the response is still a hard 4× on/off:
over-deweighted at low vibration, under-deweighted at high. A continuous
mapping from the existing EMA (e.g. `Ra_scale = 1 + k·e`, clamped) reuses
state that already exists.

**The seaway half (from 10.1).** The gravity residual in a seaway is
wave-orbital: correlated with wave phase, measured correlation time τ ≈ 0.3–0.9 s
(`imud-cal fit-ra`), not white. A white isotropic `Ra` cannot describe it — 10.1
established by sweep that *no* scalar value makes the filter consistent, and
that the value which makes NIS = 1 costs the marine default 3.7× in attitude
RMS. This is the whole of the remaining measurement-model inconsistency
(NIS ≈ 19–25, NEES(strict) ≈ 44–64).

Candidate approaches, in increasing order of ambition:
1. **Wave-phase-aware `Ra_scale`.** The sea-state estimator (§7 of math.md)
   already knows the wave period, and `acc_quiet_ema` already tracks
   disturbance. Modulating `Ra` continuously from those is cheap and reuses
   existing state — the same mechanism as the vibration half, which is why
   these are one item.
2. **Augmented state.** Add the wave-acceleration component (or a first-order
   Gauss–Markov approximation of it) to the error state, so the correlation is
   modelled rather than absorbed into R. Principled, and the standard answer
   for coloured measurement noise; costs filter dimension and tuning.

Measure with the instruments 10.1 shipped: benchmark NEES/NIS, `nis_accel` on
the wire, and `imud-cal fit-ra` on real captures. Success is NIS → 1 *without*
the accuracy regression the naive `Ra` retune caused — that is the specific
thing to beat, and the sweep table in `docs/math.md` §4.7 is the baseline.

Scope the vibration half together with the MLC hardware-detection option in §4
— they are the detection and response halves of one problem.

### 10.6 Filter self-monitoring: NEES/NIS  *(audit C1 — NIS shipped 1.7; NEES remains bench-only)*

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

### 10.7 Online adaptive process noise  *(audit C2 — depends on 10.6, follows 10.3)*

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

### Reviewed, no action

- **Heave DC-gain trade-off** *(audit A7)* — the structural high-pass zero at
  DC trading long-period swell accuracy for drift immunity is already
  documented as deliberate. If long-period swell accuracy ever becomes a
  requirement, the upgrade path is a model-based/complementary estimator (and
  a design doc), not a patch to the leaky integrator. See also 11.3.
- **Timestamp inter-anchor drift** *(audit A8)* — linear interpolation between
  60 s re-anchors carries no oscillator drift model. Below the gyro noise
  floor in practice; revisit only if a specific chip's drift spec says
  otherwise.
- **GNSS dual-antenna heading** — explicitly dropped by the audit and worth
  recording as an anti-pattern: it needs hardware not on the target fleet, and
  single-antenna GNSS "heading" is actually COG, so conflating the two would
  silently bias attitude by the leeway/crab angle.

## 11. Aerospace generalization  *(audit Part B — additive; the marine path stays the default)*

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
- **Non-Euler attitude output** *(B5 — documentation only)* — the wire packet
  already carries the full quaternion and attitude covariance, so there is
  nothing to build. What is missing is the guidance: state explicitly that
  consumers operating near vertical pitch must read `quat_*` rather than
  `pitch`/`roll`/`yaw`/`rate_of_turn`, which are degenerate near gimbal lock
  by construction. Can land immediately, no dependencies.
- **Aerospace config profile** *(B6)* — once the above are validated, a second
  named config file with aircraft-appropriate Qg/Qb/Ra, revisited gate
  constants, and `mag_yaw_only=false`. No code change beyond the items above.
- **Not in scope** *(B7)* — heave, sea-state statistics, and engine-vibration
  detection are marine-specific and simply would not be enabled. No conflict.

### 11.3 Vertical-channel aiding  *(audit C3 — splits into two very differently-sized tickets)*

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

### 11.4 Airspeed / air-data aiding  *(audit C5 — depends on 11.1, aerospace only)*

Once 11.1 lands, feeding the same 3-D velocity-aided path from airspeed plus a
wind estimate would keep the filter accurate through GPS-denied segments — a
more routine concern in aviation than at sea. Unlike 11.1 and 11.4's
predecessors, there is **no existing ingestion path** for air data of any kind:
no driver class, no thread, no config section. This needs a new input channel
built from scratch, so it is a larger first step than "depends on 11.1"
suggests. Gate entirely behind the aerospace profile; no marine analog.

### 11.5 Multi-IMU redundancy and voting  *(audit C4)*

Tracked in §6 as "Multi-IMU". The audit adds two things worth recording: the
standard name for this architecture is the **federated Kalman filter**
(Carlson, NAECON 1988), and the scope is confirmed as the largest item in the
audit — the entire daemon is built around exactly one IMU and one
magnetometer, so this is running the whole per-sensor pipeline N times plus
new arbitration/voting logic that does not exist anywhere today, not a config
change. Only pursue against an explicit redundancy or certification
requirement, and write a design doc first.

---
*Compiled 2026-07-06; output-bridges section added 2026-07-08; ideas section
+ N2K demotion added 2026-07-11; pruned of shipped items 2026-07-19 (1.6);
filter-mathematics and aerospace sections (§10, §11) added 2026-07-25 from the
external audit, pruned of what 1.7 shipped.*
