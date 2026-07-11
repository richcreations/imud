# imud-signalk — manual

`imud-signalk` connects to imud's `[stream]` socket (the same 260-byte binary
packets) and emits Signal K **delta** messages (JSON) over UDP — one per datagram
at `rate_hz` (default 10 Hz) — for every imud value that has a standard Signal K
path. imud's NMEA output is unchanged; this is an alternative path for Signal K,
which does not reliably parse all of imud's NMEA fields.

It holds no hardware, runs as a separate process, and reconnects automatically if
imud restarts. It requires imud's `[stream] enabled = true`.

The exact field → Signal K path mapping and units are in [spec.md](spec.md).

## Build and install

The bridge is optional — not built by `make` or installed by `sudo make install`.
Build and install it separately:

```sh
make bridges                 # builds imud-signalk
sudo make install-signalk    # binary + service + /etc/imud/imud-signalk.conf
```

## Setup

1. In `imud.conf`, set `[stream] enabled = true` (the bridge reads that socket).
2. In its own file `/etc/imud/imud-signalk.conf`, set `enabled = true` and the
   Signal K server's `dest_addr`/`dest_port`. The bridge reads only this file.
3. On the Signal K server, add a **UDP** connection (Server → Connections → Add)
   listening on that port.
4. Enable the service:
   ```sh
   sudo systemctl enable --now imud-signalk
   ```

Run it in the foreground to check output:

```sh
imud-signalk --config /etc/imud/imud-signalk.conf
nc -u -l 10113        # watch the raw deltas
```

## Configuration

The bridge reads its own file, `/etc/imud/imud-signalk.conf` (the `[imud-signalk]`
section). `SIGHUP` reloads `dest_addr`, `dest_port`, `rate_hz`, `source_label`,
`publish_heave`, and the log level live; `enabled` and `socket` require a restart.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable the Signal K bridge. |
| `socket` | string | `"/run/imud/imud-stream.sock"` | imud stream socket to read (match `imud.conf` if you changed it there). |
| `dest_addr` | string | `"127.0.0.1"` | Signal K server host (numeric IPv4). |
| `dest_port` | int | `10113` | UDP port — must match the Signal K server's UDP input connection. |
| `rate_hz` | int | `10` | Delta emit rate in Hz. |
| `source_label` | string | `"imud"` | Signal K delta `source.label` value. |
| `publish_heave` | bool | `true` | Emit `environment.heave`, withheld until the estimator settles (~10·τ) so no startup transient is sent (set false if imud's heave estimator is off). |

See also `imud-signalk(8)` and `imud-signalk.conf(5)`.
