# imud-mqtt — manual

`imud-mqtt` connects to imud's `[stream]` socket and publishes one value per topic
at `rate_hz` (default 5 Hz), plus **Home Assistant MQTT discovery** configs so the
sensors auto-register as a single `imud` device. It uses **libmosquitto**.

Values are published in dashboard-friendly units by default (`units = deg`:
degrees, °/min, metres, °C); set `units = rad` for SI. It holds no hardware and
reconnects to both imud and the broker automatically. Requires imud's
`[stream] enabled = true`.

The exact topic tree, units, and discovery payload are in [spec.md](spec.md).

## Build and install

The bridge needs **libmosquitto** and is not built by `make` or installed by
`sudo make install`. Build and install it separately:

```sh
sudo apt install libmosquitto-dev     # once
make imud-mqtt                        # or: make bridges
sudo make install-mqtt                # binary + service + /etc/imud/imud-mqtt.conf
```

## Setup

1. In `imud.conf`, set `[stream] enabled = true` (the bridge reads that socket).
2. In `/etc/imud/imud-mqtt.conf` the daemon is enabled by default; set
   `broker_enabled = true` and the broker `broker_addr`/`broker_port` (and
   `username`/`password`/`tls` if needed).
3. Enable the service:
   ```sh
   sudo systemctl enable --now imud-mqtt
   ```

Check the topics with any MQTT client:

```sh
mosquitto_sub -t 'imud/#' -v
```

In Home Assistant (with the MQTT integration configured) the `imud` device and
its sensors appear automatically and go *unavailable* when the bridge stops.

## Configuration

The bridge reads its own file, `/etc/imud/imud-mqtt.conf` (the `[imud-mqtt]`
section). The daemon runs whenever `enabled = true` (the default) and stays
healthy under systemd; it does not connect to a broker or publish until
`broker_enabled = true` (off by default). `SIGHUP` reloads `rate_hz`, `qos`,
`retain`, `units`, `publish_heave`, and the log level live; `broker_enabled` and
broker/client/topic changes need a restart.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
<!-- BEGIN GENERATED: config-keys imud-mqtt.1 -->
| `enabled` | bool | `true` | Run the bridge daemon. With it true and `broker_enabled` false the daemon runs but publishes nothing; set false to not run the bridge at all (it exits cleanly, so systemd does not restart it). |
| `broker_enabled` | bool | `false` | Connect to the broker and publish (the bridge's only output). |
| `socket` | string | `"/run/imud/imud-stream.sock"` | imud stream socket to read. |
| `broker_addr` | string | `"127.0.0.1"` | Broker host (name or IP). |
| `broker_port` | int | `1883` | Broker TCP port. |
| `client_id` | string | `"imud"` | MQTT client id; also the Home Assistant device/node id. |
| `keepalive_s` | int | `30` | MQTT keepalive interval in seconds. |
| `topic_prefix` | string | `"imud"` | Prefix for all published topics. |
| `rate_hz` | int | `5` | Publish rate in Hz; must be greater than zero. |
| `qos` | int | `0` | Publish QoS (0/1/2). |
| `retain` | bool | `true` | Retain values so late subscribers / HA see current state. |
| `units` | string | `"deg"` | `"deg"` (degrees, °/min, m, °C) or `"rad"` (SI). |
| `publish_heave` | bool | `true` | Publish the heave family: `environment/heave`, `environment/heaveRate` (m/s), plus the sea-state topics `environment/waveHeight` (m), `wavePeriod`/`rollPeriod`/`pitchPeriod` (s), and `rollAmplitude`/`pitchAmplitude` (angles, follow `units`). State topics are withheld until the respective estimator settles (heave ~10·τ; sea state ~2·`wave_tau_s` after that); HA discovery is still advertised on this flag alone. |
| `ha_discovery` | bool | `true` | Publish Home Assistant discovery configs. |
| `ha_prefix` | string | `"homeassistant"` | HA discovery topic prefix. |
| `username` / `password` | string | `""` | Broker auth. The password is stored in plaintext, so `/etc/imud/imud-mqtt.conf` installs mode 0640 owned `root:imud` — keep it that way if you replace the file. |
| `tls` | bool | `false` | Enable TLS (empty `tls_cafile` = system CA store). |
| `tls_cafile` | string | `""` | CA certificate path for TLS. |
<!-- END GENERATED: config-keys imud-mqtt.1 -->

See also `imud-mqtt(8)` and `imud-mqtt.conf(5)`.
