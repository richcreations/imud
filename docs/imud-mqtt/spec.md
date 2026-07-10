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
| `imud/environment/heave` | heave (m) | `publish_heave` |
| `imud/imu/temperature` | die temperature (°C) | always |
| `imud/status/online` | `online` / `offline` (retained) | availability |

Raw high-rate accel/gyro/mag/quaternion are **not** published over MQTT (wrong
transport) — consume the binary stream directly for those.

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
