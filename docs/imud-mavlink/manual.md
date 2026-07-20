# imud-mavlink — manual

`imud-mavlink` connects to imud's `[stream]` socket and sends **HEARTBEAT** at
1 Hz plus **ATTITUDE** and/or **ATTITUDE_QUATERNION** at `rate_hz` (default
10 Hz), as MAVLink **v1 or v2**. Output goes to **UDP, serial, and/or a TCP
listener simultaneously** — ground stations connect to the TCP listener as
clients (`tcp://<host>:5760` in QGroundControl / Mission Planner). Pure C
with a hand-rolled encoder — no dependencies. It holds
no hardware, reconnects to imud automatically, and requires imud's
`[stream] enabled = true`.

The messages, field mapping, and wire format are in [spec.md](spec.md).

## Build and install

Optional — not built by `make` or installed by `sudo make install`. Build and
install it separately:

```sh
make imud-mavlink            # or: make bridges
sudo make install-mavlink    # binary + service + /etc/imud/imud-mavlink.conf
```

## Setup (UDP → QGroundControl example)

1. In `imud.conf`, set `[stream] enabled = true`.
2. In `/etc/imud/imud-mavlink.conf`, set `enabled = true`, `udp_enabled = true`,
   and `udp_addr`/`udp_port` for your GCS.
3. Enable the service:
   ```sh
   sudo systemctl enable --now imud-mavlink
   ```

For **serial** output (e.g. a telemetry radio on `/dev/serial0`), set
`serial_enabled = true`, `serial_device`, and `serial_baud`. The shipped unit
grants the service the `dialout` group and tty-device access. On a vehicle bus
alongside a real autopilot, raise `component_id` (e.g. 191) to avoid an id clash.

For **TCP** output, set `tcp_enabled = true` and point the ground station at
`tcp://<host>:5760` (QGroundControl: Comm Links → Add → TCP; Mission Planner:
TCP connect). Up to 8 clients connect simultaneously and each receives the
same frames; a slow client drops frames rather than stalling the bridge.

## Configuration

The bridge reads its own file, `/etc/imud/imud-mavlink.conf` (the `[imud-mavlink]`
section). `SIGHUP` reloads `version`, `rate_hz`, `send_attitude`,
`send_attitude_quaternion`, and the UDP destination live; the ids and which
transports are enabled require a restart.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `true` | Run the bridge daemon. With it true and every transport off the daemon runs but sends nothing; set false to not run the bridge at all (it exits cleanly, so systemd does not restart it). |
| `socket` | string | `"/run/imud/imud-stream.sock"` | imud stream socket to read. |
| `version` | int | `2` | MAVLink protocol version (1 or 2). |
| `system_id` | int | `1` | MAVLink system id. |
| `component_id` | int | `1` | MAVLink component id (raise, e.g. 191, on a shared autopilot bus). |
| `rate_hz` | int | `10` | ATTITUDE/quaternion rate (heartbeat is fixed 1 Hz). |
| `send_attitude` | bool | `true` | Emit ATTITUDE (#30). |
| `send_attitude_quaternion` | bool | `true` | Emit ATTITUDE_QUATERNION (#31). |
| `udp_enabled` | bool | `false` | Enable UDP output. |
| `udp_addr` | string | `"127.0.0.1"` | UDP destination host. |
| `udp_port` | int | `14550` | UDP destination port (QGC default). |
| `serial_enabled` | bool | `false` | Enable serial output. |
| `serial_device` | string | `"/dev/serial0"` | Serial device. |
| `serial_baud` | int | `57600` | Serial baud (9600–921600). |
| `tcp_enabled` | bool | `false` | Enable the TCP listener (GCS connects as a client). |
| `tcp_bind_addr` | string | `"0.0.0.0"` | Listener bind address (numeric IPv4); `127.0.0.1` keeps it host-local. |
| `tcp_port` | int | `5760` | Listener TCP port (the de-facto MAVLink TCP port). |

See also `imud-mavlink(8)` and `imud-mavlink.conf(5)`.
