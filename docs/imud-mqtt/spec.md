# imud-mqtt — output spec

`imud-mqtt` consumes imud's AF_UNIX stream and publishes one scalar value per
topic under a prefix (default `imud`), at the configured rate. Attitude uses
imud's **native NED convention** (roll + = starboard up, pitch + = bow up) — no
sign flip. Values are degrees / °/min / m / °C by default (`units = deg`), or SI
radians / rad/s (`units = rad`).

## Topics (default prefix `imud`)

| Topic | Value | When |
|---|---|---|
| `imud/navigation/headingMagnetic` | magnetic heading | always |
| `imud/navigation/headingTrue` | true heading | declination known |
| `imud/navigation/magneticVariation` | declination | declination known |
| `imud/navigation/rateOfTurn` | rate of turn | always |
| `imud/attitude/roll` · `/pitch` · `/yaw` | attitude | always |
| `imud/environment/heave` | heave (m) | `publish_heave` **and** settled |
| `imud/environment/heaveRate` | vertical velocity (m/s, +up) | `publish_heave` **and** settled |
| `imud/environment/waveHeight` | significant wave height Hs (m) | `publish_heave` **and** sea state settled |
| `imud/environment/wavePeriod` | mean zero-crossing wave period Tz (s) | `publish_heave` **and** sea state settled |
| `imud/environment/rollPeriod` | vessel roll period (s) | `publish_heave` **and** sea state settled |
| `imud/environment/rollAmplitude` | significant single roll amplitude (deg/rad per `units`) | `publish_heave` **and** sea state settled |
| `imud/environment/pitchPeriod` | vessel pitch period (s) | `publish_heave` **and** sea state settled |
| `imud/environment/pitchAmplitude` | significant single pitch amplitude (deg/rad per `units`) | `publish_heave` **and** sea state settled |
| `imud/imu/temperature` | die temperature (°C) | always |
| `imud/engine/running` | `ON`/`OFF` from the engine-vibration detector | always (HA binary_sensor, device_class `running`) |
| `imud/status/online` | `online` / `offline` (retained) | availability |

`heave` and `heaveRate` are withheld until the heave estimator has settled
(~10·τ, the packet's `HEAVE_VALID` flag), so subscribers never see the startup
transient; `heaveRate` is always m/s regardless of `units`. Home Assistant
discovery for both is still advertised whenever `publish_heave` is set, so the
entity exists and simply reads *unavailable* until heave settles.

The sea-state topics (`waveHeight`, `wavePeriod`, `rollPeriod`,
`rollAmplitude`, `pitchPeriod`, `pitchAmplitude`, wire v14) ride the same
`publish_heave` key (they derive from heave) and are gated on the packet's
`WAVE_VALID` flag — withheld until the wave statistics settle
(~2·`wave_tau_s` after heave settles). Periods publish `0.0` when becalmed /
not rolling; that is the measurement, not an error. Heights and periods are
SI (m, s) regardless of `units`; the amplitude topics are angles and follow
`units` like roll/pitch. `engine/running` mirrors the daemon's
engine-vibration detector (`FLAG_ENGINE_ON`) and is advertised to Home
Assistant as a binary_sensor — `OFF` whenever detection is disabled.

Raw high-rate accel/gyro/mag/quaternion and the gyro-bias / variance / quiescence
diagnostics are **not** published over MQTT (wrong transport / not HA-relevant) —
consume the binary stream or the InfluxDB bridge for those.

## Availability

`imud/status/online` is **retained**: `online` is published on connect, and
`offline` is set both as the MQTT **Last-Will** (delivered by the broker if the
bridge dies) and explicitly on clean shutdown.

## Home Assistant discovery

When `ha_discovery = true`, one retained config message per sensor is published to
`<ha_prefix>/sensor/<client_id>_<sensor>/config`, grouping all sensors under one
HA device and pointing their availability at `imud/status/online`. Example (roll,
degrees):

```json
{"name":"Roll","uniq_id":"imud_roll","stat_t":"imud/attitude/roll",
"unit_of_meas":"°","avty_t":"imud/status/online",
"dev":{"ids":["imud_imud"],"name":"imud","mf":"imud","mdl":"IMU daemon"}}
```

## See also

`imud-mqtt(8)`, `imud-mqtt.conf(5)`, and the imud protocol spec
(`/usr/share/doc/imud/spec.md`) for the input binary packet.
