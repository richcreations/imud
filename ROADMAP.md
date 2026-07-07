# imud — Deferred & Future Work

Items identified during the 2026-07-06 audit and the fusion accuracy pass that were
**deliberately not implemented** — either report-only by decision, hardware-dependent,
or wire-format-affecting. Ordered roughly by expected value.

## 1. Hardware validation matrix  *(bench task — requires the Pi and real silicon)*

Seven drivers are implemented but marked `experimental = true` and have never run on
real hardware: `icm20948`, `ak09916`, `icm42688p`, `lsm6dso`, `lsm6dsox`, `lis3mdl`,
`lis2mdl`. Each needs a bench pass (probe/WHO_AM_I, init, ODR verification, FIFO/DRDY
behavior, sane sensor values in all orientations) before clearing its flag. The
driver contract in `docs/driver-guide.md` makes this mechanical. Note the reference
pair (`ism330dhcx`, `mmc5983ma`) is also untested on real silicon as of this date —
the first Pi session should start there.

## 2. WMM-informed magnetic reference  *(accuracy, medium effort)*

`wmm.c` computes the full field vector internally but only exposes declination.
Exposing magnitude + dip (e.g. `wmm_field()`) and plumbing them like declination
(startup + position thread) would let the MEKF set `m_ref` analytically from lat/lon
instead of capturing it at alignment. This removes the alignment bake-in error
entirely and makes the quiescence-gated self-healing EMA mostly redundant. Convention
note: keep declination out of `m_ref` (heading stays magnetic; declination applies
downstream as today).

## 3. Speed-aided centripetal correction  *(accuracy in turns, medium effort)*

In a sustained turn, the accelerometer measures gravity + centripetal acceleration
ω×v, which biases the tilt reference exactly when the autopilot most needs good roll.
gpsd TPV already carries `speed`; plumbing it into the fusion context (like
declination) and subtracting ω×v_body before the gravity update is the standard
marine/aviation fix. The χ² gate currently *rejects* those samples instead — the
correction would let them be *used*.

## 4. Gyro bias temperature compensation  *(warm-up + engine-room drift, medium)*

Die temperature is already in every IMU sample (`temp_c`). Learning a per-axis linear
bias/temperature coefficient (persisted in cal.json across runs) would cut warm-up
drift and day/night / engine-heat bias walk. The MEKF tracks slow drift online
already, so this mainly speeds convergence after temperature steps.

## 5. Binary packet v11 with `heave_m`  *(wire-format bump — do deliberately)*

The 192-byte packet is full; adding heave means a version bump (v10 → v11), a new
layout, and coordinated updates to `types.h`, `packet.c`, both client libraries,
`test_client`, and the docs. Heave is currently in NMEA (`$PASHR`) and JSON only.
Batch this with any other wire-format wishes rather than bumping for one field.

## 6. Sim driver: non-zero start heading  *(test hygiene, small)*

The sim's yaw sweep starts at 0°, which is precisely why the `mekf_align` heading
mirror bug (fixed in the fusion pass) was invisible to the end-to-end sim tests — the
mirror has a fixed point at north. Giving the sim a configurable (or just non-zero,
e.g. 60°) initial heading would make convention bugs of this class impossible to
hide. Cheap insurance.

## 7. `LOG_*` macro migration  *(pre-existing spec §16 item, cosmetic)*

`include/log.h` defines level-gated `LOG_D/I/W/E`, but most call sites still use bare
`fprintf(stderr, ...)`. Finishing the migration makes `[logging] level` meaningful
across the whole daemon. Related: SIGHUP does not re-apply `log_level` (documented as
restart-only) — could become hot during the same sweep.

## 8. Pi 5 interrupt latency re-profiling  *(pre-existing spec §16 item, bench)*

Pi 5 routes GPIO through the RP1; gpiod is the right abstraction but edge-interrupt
latency should be measured against the Pi 4 baseline once hardware testing starts.

## 9. ISM330DHCX MLC engine detection  *(pre-existing spec §16 item, optional)*

The current engine-vibration detection is software (EMA of (|a|−g)², now with ×4
noise inflation). The ISM330's on-chip Machine Learning Core could assert a GPIO on
engine-on instead, freeing the threshold tuning. Only worth it if the software
detector proves finicky at sea.

## 10. Small items

- `ctx->stop` in imu.c stays `volatile int` (not `_Atomic`) because its address feeds
  the `imu_ring_pop()` API; changing it means touching ring.h/ring.c/test_ring.
  Cosmetic C11-cleanliness only.
- Heave settling: ~10·τ (≈2 min) after boot before heave is trustworthy. Could be
  shortened by initializing the integrators from the first seconds of data.
- `imud-status` display could show heave and declination validity at a glance.

---
*Compiled 2026-07-06. Completed work from the same passes is documented in the git
history and spec.md §6 ("Rough-sea mechanics"); the original audit report (audit.md)
was retired after its findings were fixed.*
