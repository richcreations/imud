# imud-mqtt

**Publish imud's telemetry to MQTT — Home Assistant, Node-RED, Grafana, and more.**

`imud-mqtt` is an optional daemon that connects to imud's local stream and
publishes heading, attitude, rate of turn, heave, and temperature as MQTT topics.
It also emits Home Assistant MQTT-discovery messages, so the values show up
automatically as a single "imud" device — no manual dashboard wiring.

It's for anyone who already runs an MQTT broker: home-automation setups, boat or
robot telemetry, and dashboards. It holds no hardware, runs as its own service,
and reconnects to both imud and the broker on its own.

Once imud's stream is on and the broker details are set:

```sh
sudo systemctl enable --now imud-mqtt
```

## Documentation

- **[manual.md](manual.md)** — build, install, configuration, and setup
  (also `man imud-mqtt` and `man imud-mqtt.conf`).
- **[spec.md](spec.md)** — the exact topic tree, value units, and the Home
  Assistant discovery payload (for anyone consuming the output).
