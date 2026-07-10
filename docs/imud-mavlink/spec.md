# imud-mavlink — output spec

`imud-mavlink` consumes imud's AF_UNIX stream and emits MAVLink **v1** (`0xFE`) or
**v2** (`0xFD`) frames with a configurable `system_id` / `component_id`. MAVLink's
body frame is **FRD/NED — the same as imud**, so attitude, body rates, and the
quaternion pass straight through in SI (radians, rad/s) with no sign flips.

## Messages

- **HEARTBEAT (#0)** — 1 Hz. `type = MAV_TYPE_GENERIC`,
  `autopilot = MAV_AUTOPILOT_INVALID`, `system_status = MAV_STATE_ACTIVE`. Lets a
  GCS/autopilot discover the node.
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

Both can run at once; each link keeps its own MAVLink sequence counter.

## Wire format

The encoder is hand-rolled: CRC-16/MCRF4XX plus the per-message `CRC_EXTRA`
(HEARTBEAT 50, ATTITUDE 39, ATTITUDE_QUATERNION 246). v2 frames use
incompat/compat flags = 0 (no signing) and full-length payloads. The output is
validated against pymavlink (see `test/test_mavlink.c`).

## See also

`imud-mavlink(8)`, `imud-mavlink.conf(5)`, and the imud protocol spec
(`/usr/share/doc/imud/spec.md`) for the input binary packet.
