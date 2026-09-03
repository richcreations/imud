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

The **Level** column is the lowest `detail` setting that emits the field.
Levels are cumulative, so `detail = "health"` — the default — emits every
row up to and including `health`, and `"full"` emits the table. The sensor
vectors are never unit-converted: `units` governs the angle fields, and
reporting a gyro rate in °/s here would make `gyro_x` disagree with the
`gbias_x` you compare it against.

| Field | Meaning | Units (deg / rad) | Level |
|---|---|---|---|
| `qw` `qx` `qy` `qz` | orientation quaternion | unitless | attitude |
| `roll` `pitch` `yaw` | Euler attitude | ° / rad | attitude |
| `heading` | magnetic heading — see `heading_ref` | ° / rad | attitude |
| `heading_ref` | the magnetometer is being fused, so `heading` really is magnetic | boolean (`t`/`f`) | attitude |
| `mag_absent` | no magnetometer is configured at all | boolean (`t`/`f`) | navigation |
| `heading_true` | true heading | ° / rad | navigation, declination known |
| `variation` | magnetic declination | ° / rad | navigation, declination known |
| `rate_of_turn` | rate of turn | °/min / rad·s⁻¹ | attitude |
| `heave` | heave | m | navigation, `publish_heave` |
| `heave_rate` | vertical velocity, +up | m·s⁻¹ | navigation, `publish_heave` |
| `heave_valid` | heave estimator settled (~10·τ) | boolean (`t`/`f`) | navigation, `publish_heave` |
| `gbias_x` `gbias_y` `gbias_z` | gyro-bias estimate (MEKF) | rad·s⁻¹ | health |
| `gbias_var_x` `gbias_var_y` `gbias_var_z` | gyro-bias variance (MEKF `P` diagonal) | (rad·s⁻¹)² | health |
| `quiescence` | accel-quiescence / platform-disturbance metric | unitless | health |
| `wave_height` | significant wave height Hs = 4σ(heave) | m | seastate |
| `wave_period` | mean zero-crossing wave period Tz | s | seastate |
| `roll_period` | vessel roll period | s | seastate |
| `roll_amplitude` | significant single roll amplitude | rad | seastate |
| `pitch_period` | vessel pitch period | s | seastate |
| `pitch_amplitude` | significant single pitch amplitude | rad | seastate |
| `nis_accel` | EMA of normalised innovation squared, accel update (d²/2); 1 = covariance matches innovations, higher = over-confident | unitless | health |
| `nis_mag` | EMA of normalised innovation squared, mag update (d²/dof); 1 = consistent, higher = over-confident | unitless | health |
| `innov_weight` | EMA of the Huber weight applied to MEKF updates; 1 = no capping, lower = the filter is persistently distrusting its sensors | unitless | health |
| `innov_reject` | EMA of the fraction of MEKF updates rejected by the innovation gate | unitless | health |
| `mag_anomaly` | EMA of \|\|B\|−\|B_ref\|\|/\|B_ref\| — interference / iron-cal drift | unitless | health |
| `mag_residual` | EMA of \|heading innovation\| — compass cal health | rad | health |
| `wave_valid` | sea-state statistics settled | boolean (`t`/`f`) | seastate |
| `temp` | die temperature | °C | attitude |
| `seq` | imu sample counter | integer (`i` suffix) | attitude |
| `accel_x` `accel_y` `accel_z` | calibrated acceleration, body frame | m·s⁻² | full |
| `accel_raw_x` `accel_raw_y` `accel_raw_z` | acceleration before calibration, after mount rotation | m·s⁻² | full |
| `gyro_x` `gyro_y` `gyro_z` | bias-corrected angular rate, body frame | rad·s⁻¹ | full |
| `gyro_raw_x` `gyro_raw_y` `gyro_raw_z` | angular rate before bias correction | rad·s⁻¹ | full |
| `mag_x` `mag_y` `mag_z` | calibrated magnetic field, body frame | µT | full |
| `mag_raw_x` `mag_raw_y` `mag_raw_z` | magnetic field before hard/soft-iron correction | µT | full |
| `ts_tai_ns` | sample instant on CLOCK_TAI | integer ns (`i` suffix) | full |
| `ts_chip_ticks` | the IMU counter the instant came from | integer (`i` suffix) | full |
| `anchor_gen` | increments on each wall-clock re-anchor | integer (`i` suffix) | full |

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
imud,source=imud qw=1.000000,qx=0.000000,qy=0.000000,qz=0.000000,roll=5.72958,pitch=-2.86479,yaw=70.47381,heading=90.00000,heading_true=103.20001,variation=13.20000,rate_of_turn=60.000000,heave=0.4200,heave_rate=0.2500,heave_valid=t,gbias_x=0.001000,gbias_y=-0.002000,gbias_z=0.003000,gbias_var_x=1.000e-06,gbias_var_y=2.000e-06,gbias_var_z=3.000e-06,quiescence=0.010000,mag_anomaly=0.01200,mag_residual=0.00480,wave_height=0.310,wave_period=4.20,roll_period=6.10,roll_amplitude=0.0700,pitch_period=8.00,pitch_amplitude=0.0350,wave_valid=t,temp=31.40,seq=7i 1620307999123000000
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
