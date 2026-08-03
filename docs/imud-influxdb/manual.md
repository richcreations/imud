# imud-influxdb — manual

`imud-influxdb` connects to imud's `[stream]` socket and emits one InfluxDB
line-protocol point per tick (default 10 Hz) over **UDP** (default) or **HTTP**.
It is pure C with no external dependencies. It holds no hardware, reconnects to
imud automatically, and requires imud's `[stream] enabled = true`.

The exact measurement, tags, and field layout are in [spec.md](spec.md).

## Build and install

Optional — not built by `make` or installed by `sudo make install`. Build and
install it separately:

```sh
make imud-influxdb            # or: make bridges
sudo make install-influxdb    # binary + service + /etc/imud/imud-influxdb.conf
```

## Setup (UDP → InfluxDB 1.x example)

1. In `imud.conf`, set `[stream] enabled = true`.
2. In `/etc/imud/imud-influxdb.conf` the daemon is enabled by default; turn on
   an output: set `udp_enabled = true` with the `udp_addr` / `udp_port` of your
   InfluxDB UDP listener (or Telegraf). Configure the listener precision as `ns`.
3. Enable the service:
   ```sh
   sudo systemctl enable --now imud-influxdb
   ```

For InfluxDB 2.x/3.x, set `http_enabled = true`, point `http_path` at
`/api/v2/write?org=<org>&bucket=<bucket>&precision=ns`, and set `http_token`.
UDP and HTTP are independent — enable either or both.

## Configuration

The bridge reads its own file, `/etc/imud/imud-influxdb.conf` (the
`[imud-influxdb]` section). The daemon runs whenever `enabled = true` (the
default) and stays healthy under systemd; each output (`udp_enabled`,
`http_enabled`) is independent and off by default, so nothing is written until
you turn one on. `SIGHUP` reloads `rate_hz`, `measurement`, `source_label`,
`units`, `publish_heave`, and the UDP destination live; the output enables and
HTTP target need a restart.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `true` | Run the bridge daemon. With it true and every output off the daemon runs but writes nothing; set false to not run the bridge at all (it exits cleanly, so systemd does not restart it). |
| `socket` | string | `"/run/imud/imud-stream.sock"` | imud stream socket to read. |
| `transport` | string | `""` | **Deprecated** — legacy `"udp"`/`"http"` selector, mapped to `udp_enabled`/`http_enabled` when neither is set. Prefer the enables. |
| `rate_hz` | int | `10` | Point emit rate in Hz; must be greater than zero. |
| `measurement` | string | `"imud"` | Line-protocol measurement name. |
| `source_label` | string | `"imud"` | Value of the `source=` tag. |
| `units` | string | `"deg"` | `"deg"` (degrees, °/min) or `"rad"` (SI). |
| `publish_heave` | bool | `true` | Include the `heave` field. |
| `udp_enabled` | bool | `false` | Write line-protocol points over UDP. |
| `udp_addr` | string | `"127.0.0.1"` | UDP destination host (InfluxDB 1.x / Telegraf). |
| `udp_port` | int | `8089` | UDP port. |
| `http_enabled` | bool | `false` | Write line-protocol points over HTTP (may run alongside `udp_enabled`). |
| `http_host` | string | `"127.0.0.1"` | HTTP host. |
| `http_port` | int | `8086` | HTTP port. |
| `http_path` | string | `"/write?db=imud&precision=ns"` | Write path (1.x `db`, or 2.x `/api/v2/write?org=&bucket=`). |
| `http_token` | string | `""` | InfluxDB 2.x API token. Stored in plaintext, so `/etc/imud/imud-influxdb.conf` installs mode 0640 owned `root:imud` — keep it that way if you replace the file. |

See also `imud-influxdb(8)` and `imud-influxdb.conf(5)`.
