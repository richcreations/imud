# imud-influxdb

**Log imud's telemetry to InfluxDB for Grafana dashboards and analysis.**

`imud-influxdb` is an optional daemon that connects to imud's local stream and
writes it to InfluxDB as line-protocol points — over UDP or HTTP — for time-series
storage and **Grafana** dashboards. It's ideal for fusion tuning and sea-trial
logging, where you want every field graphed over time. Pure C, no dependencies.

It's for anyone running InfluxDB (1.x, 2.x, or 3.x) or Telegraf and Grafana. It
holds no hardware, runs as its own service, and reconnects to imud on its own.

Once imud's stream is on and an output is enabled (`udp_enabled` and/or
`http_enabled`) with its destination set:

```sh
sudo systemctl enable --now imud-influxdb
```

## Documentation

- **[manual.md](manual.md)** — build, install, configuration, and setup
  (also `man imud-influxdb` and `man imud-influxdb.conf`).
- **[spec.md](spec.md)** — the exact line-protocol measurement, tags, field
  names, units, and precision (for anyone consuming the output).
