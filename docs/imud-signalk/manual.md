# imud-signalk — manual

> Getting imud's data onto an **NMEA 2000** backbone? This bridge is step one —
> see the recipe in the main manual (§7, "NMEA 2000 — via Signal K").

`imud-signalk` connects to imud's `[stream]` socket (the same 276-byte binary
packets) and emits Signal K **delta** messages (JSON) over UDP — one per datagram
at `rate_hz` (default 10 Hz) — for every imud value that has a standard Signal K
path. A TCP listener (`tcp_enabled`, default port 10113) can serve the same
deltas newline-framed to connecting clients — the Signal K server consumes it
as a TCP client data connection, handy when the server runs on another
machine. imud's NMEA output is unchanged; this is an alternative path for
Signal K, which does not reliably parse all of imud's NMEA fields.

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
2. In its own file `/etc/imud/imud-signalk.conf` the daemon is enabled by
   default; turn on an output: set `udp_enabled = true` with the Signal K
   server's `dest_addr`/`dest_port`, and/or `tcp_enabled = true`. The bridge
   reads only this file.
3. On the Signal K server, add a **UDP** connection (Server → Connections → Add)
   listening on that port — or set `tcp_enabled = true` in the bridge config
   and add a **TCP client** data connection pointing at the bridge host,
   port 10113, instead.
4. Enable the service:
   ```sh
   sudo systemctl enable --now imud-signalk
   ```

Run it in the foreground to check output:

```sh
imud-signalk --config /etc/imud/imud-signalk.conf
nc -u -l 10113        # watch the raw deltas (UDP)
nc HOST 10113         # or connect to the TCP listener (tcp_enabled = true)
```

## Configuration

The bridge reads its own file, `/etc/imud/imud-signalk.conf` (the `[imud-signalk]`
section). The daemon runs whenever `enabled = true` (the default) and stays
healthy under systemd; each output (`udp_enabled`, `tcp_enabled`) is independent
and off by default, so nothing is sent until you turn one on. `SIGHUP` reloads
`dest_addr`, `dest_port`, `rate_hz`, `source_label`, `publish_heave`, and the log
level live; `enabled`, `socket`, and the `udp_*`/`tcp_*` keys require a restart.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `true` | Run the bridge daemon. With it true and every output off the daemon runs but sends nothing; set false to not run the bridge at all (it exits cleanly, so systemd does not restart it). |
| `socket` | string | `"/run/imud/imud-stream.sock"` | imud stream socket to read (match `imud.conf` if you changed it there). |
| `udp_enabled` | bool | `false` | Emit deltas over UDP to `dest_addr`:`dest_port`. |
| `dest_addr` | string | `"127.0.0.1"` | Signal K server host: a hostname, a numeric IPv4 or IPv6 address, or a broadcast address. Resolved with `getaddrinfo(3)`, as in the other bridges. Before 1.7 this key accepted numeric IPv4 only. |
| `dest_port` | int | `10113` | UDP port — must match the Signal K server's UDP input connection. |
| `rate_hz` | int | `10` | Delta emit rate in Hz. |
| `source_label` | string | `"imud"` | Signal K delta `source.label` value. |
| `publish_heave` | bool | `true` | Emit `environment.heave`, withheld until the estimator settles (~10·τ) so no startup transient is sent (set false if imud's heave estimator is off). |
| `tcp_enabled` | bool | `false` | Serve the same deltas newline-framed on a TCP listener (the SK server connects as a TCP client). Up to 8 clients; independent of `udp_enabled`. |
| `tcp_bind_addr` | string | `"0.0.0.0"` | Listener bind address (numeric IPv4); `127.0.0.1` keeps it host-local. |
| `tcp_port` | int | `10113` | Listener TCP port (TCP namespace of the UDP default). |

See also `imud-signalk(8)` and `imud-signalk.conf(5)`.
