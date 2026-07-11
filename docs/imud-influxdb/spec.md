# imud-influxdb — output spec

`imud-influxdb` consumes imud's AF_UNIX stream and writes one InfluxDB
line-protocol point per tick:

```
<measurement>,source=<label> <field_set> <timestamp_ns>
```

The measurement defaults to `imud`; the one tag is `source=<source_label>`. The
timestamp is the packet's wall-clock time in **nanoseconds** — set the listener
(UDP) or write endpoint (`precision=ns`) to match. Attitude uses imud's native
NED convention (no sign flip).

## Fields

| Field | Meaning | Units (deg / rad) | When |
|---|---|---|---|
| `qw` `qx` `qy` `qz` | orientation quaternion | unitless | always |
| `roll` `pitch` `yaw` | Euler attitude | ° / rad | always |
| `heading` | magnetic heading | ° / rad | always |
| `heading_true` | true heading | ° / rad | declination known |
| `variation` | magnetic declination | ° / rad | declination known |
| `rate_of_turn` | rate of turn | °/min / rad·s⁻¹ | always |
| `heave` | heave | m | `publish_heave` |
| `heave_rate` | vertical velocity, +up | m·s⁻¹ | `publish_heave` |
| `heave_valid` | heave estimator settled (~10·τ) | boolean (`t`/`f`) | `publish_heave` |
| `gbias_x` `gbias_y` `gbias_z` | gyro-bias estimate (MEKF) | rad·s⁻¹ | always |
| `gbias_var_x` `gbias_var_y` `gbias_var_z` | gyro-bias variance (MEKF `P` diagonal) | (rad·s⁻¹)² | always |
| `quiescence` | accel-quiescence / platform-disturbance metric | unitless | always |
| `wave_height` | significant wave height Hs = 4σ(heave) | m | always |
| `wave_period` | mean zero-crossing wave period Tz | s | always |
| `roll_period` | vessel roll period | s | always |
| `roll_amplitude` | significant single roll amplitude | rad | always |
| `pitch_period` | vessel pitch period | s | always |
| `pitch_amplitude` | significant single pitch amplitude | rad | always |
| `mag_anomaly` | EMA of \|\|B\|−\|B_ref\|\|/\|B_ref\| — interference / iron-cal drift | unitless | always |
| `mag_residual` | EMA of \|heading innovation\| — compass cal health | rad | always |
| `wave_valid` | sea-state statistics settled | boolean (`t`/`f`) | always |
| `temp` | die temperature | °C | always |
| `seq` | imu sample counter | integer (`i` suffix) | always |

Being the diagnostics sink, `imud-influxdb` emits `heave`/`heave_rate` from t=0
(unlike the user-facing bridges, which withhold heave until settled) and exposes
`heave_valid` so the pre-settle transient can be filtered downstream. The
diagnostic fields (`heave_rate`, `gbias_*`, `gbias_var_*`, `quiescence`) and
the sea-state fields (`wave_height`, `wave_period`, `roll_period`,
`roll_amplitude`, `pitch_period`, `pitch_amplitude`, wire v14; 0.0 with
`wave_valid=f` until settled) and the compass-health metrics (`mag_anomaly`,
`mag_residual`) are frame-neutral SI and are **not** affected by the `units`
(deg/rad) setting.

## Example point

```
imud,source=imud qw=1.000000,qx=0.000000,qy=0.000000,qz=0.000000,roll=5.72958,pitch=-2.86479,yaw=70.47381,heading=90.00000,heading_true=103.20001,variation=13.20000,rate_of_turn=60.000000,heave=0.4200,heave_rate=0.2500,heave_valid=t,gbias_x=0.001000,gbias_y=-0.002000,gbias_z=0.003000,gbias_var_x=1.000e-06,gbias_var_y=2.000e-06,gbias_var_z=3.000e-06,quiescence=0.010000,temp=31.40,seq=7i 1620307999123000000
```

## Transports

- **UDP** — datagrams to InfluxDB 1.x's UDP listener or Telegraf's
  `socket_listener` (the usual path into InfluxDB 2.x/3.x).
- **HTTP** — a plaintext POST per point to `/write` (1.x) or `/api/v2/write`
  (2.x), with an optional `Authorization: Token` header. No TLS — front a
  cloud/TLS endpoint with a local proxy.

## See also

`imud-influxdb(8)`, `imud-influxdb.conf(5)`, and the imud protocol spec
(`/usr/share/doc/imud/spec.md`) for the input binary packet.
