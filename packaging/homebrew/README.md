# homebrew-imud

Homebrew tap for [imud](https://github.com/richcreations/imud) — a
general-purpose IMU daemon for Linux and macOS.

```sh
brew tap richcreations/imud
brew install richcreations/imud/imud richcreations/imud/imud-wmm-data
brew services start richcreations/imud/imud
```

One formula per Debian package, so a host installs only what it needs:

| Formula | What it installs |
|---|---|
| `imud` | the daemon, `imud-cal`, `imud-status`, libimud and its header |
| `imud-utils` | `imud-mon` and `imud-imutest` |
| `imud-wmm-data` | the World Magnetic Model coefficients |
| `imud-signalk` | the Signal K bridge |
| `imud-mqtt` | the MQTT bridge |
| `imud-influxdb` | the InfluxDB bridge |
| `imud-mavlink` | the MAVLink bridge |
| `imud-prometheus` | the Prometheus exporter |

Every other formula depends on `imud`, so installing a bridge installs the
daemon with it. `libimud0` and `libimud-dev` have no formula of their own:
Homebrew does not split a library from its headers, and both ship inside
`imud`.

The formulae track **releases only** — they declare no `head`, so there is no
route here to an untagged tree. Each release rewrites them, automatically, on
the same trigger that promotes the `.deb`s into the apt repository.

Configuration lives in `$(brew --prefix)/etc/imud/` and an edited file
survives an upgrade.

This whole repository is generated from `packaging/homebrew/` in the
[imud repository](https://github.com/richcreations/imud), README included —
send changes there, not here.

Issues and pull requests: <https://github.com/richcreations/imud/issues>.
