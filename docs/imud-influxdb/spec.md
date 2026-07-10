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
| `temp` | die temperature | °C | always |
| `seq` | imu sample counter | integer (`i` suffix) | always |

## Example point

```
imud,source=imud qw=1.000000,qx=0.000000,qy=0.000000,qz=0.000000,roll=5.72958,pitch=-2.86479,yaw=70.47381,heading=90.00000,heading_true=103.20001,variation=13.20000,rate_of_turn=60.000000,heave=0.4200,temp=31.40,seq=7i 1620307999123000000
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
