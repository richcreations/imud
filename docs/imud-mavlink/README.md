# imud-mavlink

**Stream imud's attitude to ArduPilot, PX4, or a ground station as MAVLink.**

`imud-mavlink` is an optional daemon that connects to imud's local stream and
emits MAVLink — a HEARTBEAT plus ATTITUDE / ATTITUDE_QUATERNION — over UDP,
serial, and/or a TCP listener ground stations connect to
(`tcp://<host>:5760`). It lets an autopilot or a ground station
(QGroundControl, Mission Planner)
see imud as a MAVLink attitude source. Pure C, no dependencies, MAVLink v1 or v2.

It's for drones, rovers, and boats running an autopilot, or anyone feeding a GCS —
e.g. a telemetry radio on serial and a local GCS on UDP at the same time. It holds
no hardware, runs as its own service, and reconnects to imud on its own.

Once imud's stream is on and a transport is enabled:

```sh
sudo systemctl enable --now imud-mavlink
```

## Documentation

- **[manual.md](manual.md)** — build, install, configuration, and setup
  (also `man imud-mavlink` and `man imud-mavlink.conf`).
- **[spec.md](spec.md)** — the messages, field mapping, frame convention, and
  wire format (for anyone consuming the output).
