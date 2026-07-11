# imud-prometheus manual

`imud-prometheus` serves imud's fused state as Prometheus gauges. It is a
standalone daemon with its own config file, service unit, and man pages;
it reads imud's `[stream]` socket and never touches imud.conf.

## Install

```sh
make imud-prometheus
sudo make install-prometheus
```

Requires imud's stream output (`imud.conf`: `[stream] enabled = true`).

## Configure

`/etc/imud/imud-prometheus.conf`:

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `enabled` | bool | `false` | Start the exporter. [restart] |
| `socket` | string | `/run/imud/imud-stream.sock` | imud stream socket to read. [restart] |
| `listen_addr` | string | `127.0.0.1` | HTTP bind address; `0.0.0.0` to allow scrapes from another host. [restart] |
| `listen_port` | int | `9815` | HTTP port. [restart] |

`[logging] level` is shared with the other daemons. SIGHUP
(`systemctl reload imud-prometheus`) re-reads the log level; the listen
address needs a restart.

## Run

```sh
sudo systemctl enable --now imud-prometheus
curl localhost:9815/metrics
```

Prometheus scrape config:

```yaml
scrape_configs:
  - job_name: imud
    static_configs:
      - targets: ["boat.local:9815"]
```

See [spec.md](spec.md) for the full metric list and alerting examples.
