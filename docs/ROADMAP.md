# imud — Deferred & Future Work

Items identified during the 2026-07-06 audit and fusion passes that remain open.
Completed work is tracked in git history and spec.md §6, not here.

## 1. Hardware validation matrix  *(bench task — requires the Pi and real silicon)*

Seven drivers are implemented but marked `experimental = true` and have never run on
real hardware: `icm20948`, `ak09916`, `icm42688p`, `lsm6dso`, `lsm6dsox`, `lis3mdl`,
`lis2mdl`. Each needs a bench pass (probe/WHO_AM_I, init, ODR verification, FIFO/DRDY
behavior, sane values in all orientations) before clearing its flag. The driver
contract in the driver guide in `docs/manual.md` makes this mechanical. The reference pair
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

## 2. Gyro bias temperature compensation  *(warm-up + engine-room drift, medium)*

Die temperature is already in every IMU sample (`temp_c`). Learning a per-axis linear
bias/temperature coefficient (persisted in cal.json across runs) would cut warm-up
drift and day/night / engine-heat bias walk. Needs real thermal data to fit — pair it
with the first hardware sessions.

## 3. Pi 5 interrupt latency re-profiling  *(pre-existing spec §16 item, bench)*

Pi 5 routes GPIO through the RP1; gpiod is the right abstraction but edge-interrupt
latency should be measured against the Pi 4 baseline once hardware testing starts.

## 4. ISM330DHCX MLC engine detection  *(pre-existing spec §16 item, optional)*

The current engine-vibration detection is software (EMA of (|a|−g)², with ×4 accel
noise inflation while active). The ISM330's on-chip Machine Learning Core could
assert a GPIO on engine-on instead. Only worth it if the software detector proves
finicky at sea.

## 5. Small items

- `ctx->stop` in imu.c stays `volatile int` (not `_Atomic`) because its address feeds
  the `imu_ring_pop()` API; changing it means touching ring.h/ring.c/test_ring.
  Cosmetic C11-cleanliness only.
- Heave settling: ~10·τ (≈2 min) after boot before heave is trustworthy. Could be
  shortened by initializing the integrators from the first seconds of data.
- Centripetal correction models `v_body = [speed, 0, 0]` (no leeway); a leeway-angle
  estimate could refine it, but the residual is second-order for typical vessels.

---
*Compiled 2026-07-06, updated same day after the software roadmap items landed.*
