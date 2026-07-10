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
2. In `/etc/imud/imud-influxdb.conf`, set `enabled = true` and the `udp_addr` /
   `udp_port` of your InfluxDB UDP listener (or Telegraf). Configure the listener
   precision as `ns`.
3. Enable the service:
   ```sh
   sudo systemctl enable --now imud-influxdb
   ```

For InfluxDB 2.x/3.x, set `transport = http`, point `http_path` at
`/api/v2/write?org=<org>&bucket=<bucket>&precision=ns`, and set `http_token`.

## Configuration

The bridge reads its own file, `/etc/imud/imud-influxdb.conf` (the
`[imud-influxdb]` section). `SIGHUP` reloads `rate_hz`, `measurement`,
`source_label`, `units`, `publish_heave`, and the UDP destination live; the
transport and HTTP target need a restart.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable the InfluxDB bridge. |
| `socket` | string | `"/run/imud/imud-stream.sock"` | imud stream socket to read. |
| `transport` | string | `"udp"` | `"udp"` or `"http"`. |
| `rate_hz` | int | `10` | Point emit rate in Hz. |
| `measurement` | string | `"imud"` | Line-protocol measurement name. |
| `source_label` | string | `"imud"` | Value of the `source=` tag. |
| `units` | string | `"deg"` | `"deg"` (degrees, °/min) or `"rad"` (SI). |
| `publish_heave` | bool | `true` | Include the `heave` field. |
| `udp_addr` | string | `"127.0.0.1"` | UDP destination host (InfluxDB 1.x / Telegraf). |
| `udp_port` | int | `8089` | UDP port. |
| `http_host` | string | `"127.0.0.1"` | HTTP host (when `transport = http`). |
| `http_port` | int | `8086` | HTTP port. |
| `http_path` | string | `"/write?db=imud&precision=ns"` | Write path (1.x `db`, or 2.x `/api/v2/write?org=&bucket=`). |
| `http_token` | string | `""` | InfluxDB 2.x API token (plaintext — protect the file). |

See also `imud-influxdb(8)` and `imud-influxdb.conf(5)`.
