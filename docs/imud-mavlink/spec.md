# imud-mavlink — output spec

`imud-mavlink` consumes imud's AF_UNIX stream and emits MAVLink **v1** (`0xFE`) or
**v2** (`0xFD`) frames with a configurable `system_id` / `component_id`. MAVLink's
body frame is **FRD/NED — the same as imud**, so attitude, body rates, and the
quaternion pass straight through in SI (radians, rad/s) with no sign flips.

## Messages

- **HEARTBEAT (#0)** — 1 Hz. `type = MAV_TYPE_GENERIC`,
  `autopilot = MAV_AUTOPILOT_INVALID`, `system_status = MAV_STATE_ACTIVE`. Lets a
  GCS/autopilot discover the node.
- **SYS_STATUS (#1)** — 1 Hz, alongside the heartbeat. Carries the
  `onboard_control_sensors_present` / `_enabled` / `_health` bitmasks:
  `3D_GYRO`, `3D_ACCEL` and `AHRS` are present and healthy once packets are
  flowing; `3D_MAG` is **present** when a magnetometer is fitted and
  **healthy** only while it is actually being fused. Battery, load and
  comm-error fields are the "unknown" sentinels — imud measures none of
  them.

  This is how a receiver learns that `ATTITUDE.yaw` is not referenced to
  north. `yaw` itself cannot say so: it is the estimator's own yaw, and a
  compass-less autopilot reports exactly that, relative to where it
  started — correct, and useful for heading hold. The two failure cases
  are distinguishable: no magnetometer fitted clears `3D_MAG` in
  *present*, one that has stopped answering clears it in *health* only.
- **ATTITUDE (#30)** — at `rate_hz` when `send_attitude`. `time_boot_ms`;
  `roll`/`pitch`/`yaw` ← imud roll/pitch/yaw (rad); `rollspeed`/`pitchspeed`/
  `yawspeed` ← gyro x/y/z (rad/s).
- **ATTITUDE_QUATERNION (#31)** — at `rate_hz` when `send_attitude_quaternion`.
  `time_boot_ms`; `q1..q4` ← quaternion w,x,y,z; rates ← gyro.

`time_boot_ms` is the bridge's monotonic uptime in milliseconds.

## Quaternion sign

`ATTITUDE_QUATERNION` `q` is passed through from imud's body→NED quaternion. If a
GCS shows it conjugated relative to `ATTITUDE`, flip the quaternion sign
convention (verify on a live display).

## Transports

- **UDP** — datagrams to `udp_addr:udp_port` (default 127.0.0.1:14550,
  QGroundControl's listen port).
- **Serial** — raw 8N1 at `serial_baud` on `serial_device`.
- **TCP listener** — GCS clients connect to `tcp_bind_addr:tcp_port`
  (default 0.0.0.0:5760, the de-facto MAVLink TCP port); all connected
  clients (≤ 8) receive the same frames, and a slow client drops frames
  rather than stalling the bridge.

All can run at once; each transport keeps its own MAVLink sequence counter
(the TCP listener's clients share one — they see the identical stream).

## Wire format

The encoder is hand-rolled: CRC-16/MCRF4XX plus the per-message `CRC_EXTRA`
(HEARTBEAT 50, SYS_STATUS 124, ATTITUDE 39, ATTITUDE_QUATERNION 246). v2 frames use
incompat/compat flags = 0 (no signing) and full-length payloads. The output is
validated against pymavlink (see `test/test_mavlink.c`).

## See also

`imud-mavlink(8)`, `imud-mavlink.conf(5)`, and the imud protocol spec
(`/usr/share/doc/imud/spec.md`) for the input binary packet.
