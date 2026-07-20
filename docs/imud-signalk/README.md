# imud-signalk

**Feed imud's heading and attitude to a Signal K server, natively.**

`imud-signalk` is a small optional daemon that connects to imud's local stream
and republishes it as Signal K — so a Signal K server (and everything downstream
of it: chartplotters, instruments, apps) gets imud's magnetic and true heading,
rate of turn, attitude, and heave directly in the Signal K data model, without
relying on NMEA 0183 parsing.

It's for marine users running Signal K who want imud's data in Signal K without
the rough edges of NMEA. It holds no hardware, runs as its own service, and
reconnects on its own if imud restarts. Deltas go out over UDP, a TCP
listener the server connects to, or both.

Once imud's stream output is enabled, turn it on with one command:

```sh
sudo systemctl enable --now imud-signalk
```

## Documentation

- **[manual.md](manual.md)** — build, install, configuration, and setup
  (also `man imud-signalk` and `man imud-signalk.conf`).
- **[spec.md](spec.md)** — the exact Signal K paths, units, and conventions it
  emits (for anyone consuming the output).
