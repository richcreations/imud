# imud-prometheus

**Serve imud's telemetry to Prometheus for alerting and Grafana dashboards.**

`imud-prometheus` is an optional daemon that connects to imud's local stream
and serves the latest fused state as text-format gauges on a `/metrics`
endpoint — attitude, heading, heave, the sea-state statistics, and the
fusion/compass-health diagnostics, plus the flag bits as 0/1 gauges for
alerting ("compass residual high", "engine running", "heave not settled").
Pure C, no dependencies; built on the ABI-stable libimud, so it never needs
a rebuild across imud wire revisions.

It holds no hardware, runs as its own service, and reconnects to imud on its
own (answering scrapes with `imud_up 0` while imud is away).

Once imud's stream is on and the exporter is enabled:

```sh
sudo systemctl enable --now imud-prometheus
curl localhost:9815/metrics
```

## Documentation

- [manual.md](manual.md) — install, configure, run
- [spec.md](spec.md) — exported metrics reference
