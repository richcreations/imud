# imud-signalk — output spec

`imud-signalk` consumes imud's AF_UNIX stream (the 276-byte binary packet) and
emits one Signal K **delta** JSON message per UDP datagram at the configured rate.
With `tcp_enabled` the identical deltas are also served on a TCP listener,
newline-framed (one JSON delta per `\n`-terminated line — the Signal K TCP
data-connection format). Values use Signal K SI units (radians, rad/s, metres).
`context` is omitted, so
the server applies the delta to `vessels.self`; `source.label` is the configured
`source_label`; the delta `timestamp` is ISO-8601 UTC with milliseconds, from the
packet's wall-clock time.

## Field mapping

| imud value | Signal K path | Units | When |
|---|---|---|---|
| magnetic heading | `navigation.headingMagnetic` | rad | always |
| true heading | `navigation.headingTrue` | rad | declination known |
| declination | `navigation.magneticVariation` | rad (Easterly +) | declination known |
| rate of turn | `navigation.rateOfTurn` | rad/s (+ = starboard) | always |
| roll / pitch / yaw | `navigation.attitude` `{roll,pitch,yaw}` | rad | always |
| heave | `environment.heave` | m (+ up) | `publish_heave` **and** settled (~10·τ) |

`navigation.headingTrue` and `navigation.magneticVariation` are emitted only when
imud has a valid magnetic declination (a configured value or a WMM computation).
`environment.heave` is withheld until the heave estimator has settled (the packet's
`HEAVE_VALID` flag), so subscribers never see the startup transient. Heave rate,
the gyro-bias / variance / quiescence / compass-health diagnostics, and the
wire-v14 sea-state fields (wave height/period, roll/pitch period and
amplitude) have no standard Signal K path — the spec's `environment` group
defines no `wave` object — and are not emitted (consume the binary stream,
the MQTT bridge, or the InfluxDB bridge for those). The v14 `engine_on` flag
is likewise not mapped; Signal K's `propulsion.<id>.state` is a plausible
future home but requires an engine identity imud does not have.

## Conventions

Signal K's magnetic variation is Easterly-positive and rate of turn is
starboard-positive, both matching imud directly. Signal K's schema does not state
the attitude sign directions; imud's **roll is negated** into the widely used
Signal K convention (roll + = starboard-down), while **pitch and yaw pass
through**. Verify against a live Signal K display and flip if the horizon reads
inverted.

## Example delta

```json
{"updates":[{"source":{"label":"imud"},"timestamp":"2026-07-10T12:34:56.789Z",
"values":[{"path":"navigation.headingMagnetic","value":1.5708},
{"path":"navigation.attitude","value":{"roll":-0.05,"pitch":-0.03,"yaw":1.23}},
{"path":"navigation.rateOfTurn","value":-0.0021}]}]}
```

## See also

`imud-signalk(8)`, `imud-signalk.conf(5)`, and the imud protocol spec
(`/usr/share/doc/imud/spec.md`) for the input binary packet.
