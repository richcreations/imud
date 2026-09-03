# imud-prometheus — exported metrics

`imud-prometheus` consumes imud's AF_UNIX stream via **libimud** (the
ABI-stable `imud_data_t` view — no wire pinning) and serves the newest
sample as Prometheus text exposition format (`version=0.0.4`) on any GET.
All values are base SI units; heading is additionally conventional degrees.

## Metrics

| Metric | Meaning | Unit |
| --- | --- | --- |
| `imud_up` | 1 when a live packet is held | 0/1 |
| `imud_packets_total` | packets received from the stream socket (counter) | count |
| `imud_heading_degrees` | magnetic heading | ° (0–360) |
| `imud_heading_true_degrees` | true heading (only while declination known) | ° (0–360) |
| `imud_roll_radians` / `imud_pitch_radians` / `imud_yaw_radians` | attitude, imud NED signs | rad |
| `imud_rate_of_turn_radians_per_second` | rate of turn (+ = right) | rad/s |
| `imud_heave_meters` / `imud_heave_rate_meters_per_second` | heave (+ up) | m, m/s |
| `imud_wave_height_meters` | significant wave height Hs = 4σ(heave) | m |
| `imud_wave_period_seconds` | mean zero-crossing wave period Tz | s |
| `imud_roll_period_seconds` / `imud_pitch_period_seconds` | vessel roll/pitch period | s |
| `imud_roll_amplitude_radians` / `imud_pitch_amplitude_radians` | significant single amplitude (2σ) | rad |
| `imud_mag_anomaly_ratio` | EMA of \|\|B\|−\|B_ref\|\|/\|B_ref\| — interference / iron-cal drift | unitless |
| `imud_mag_residual_radians` | EMA of \|heading innovation\| — compass cal health | rad |
| `imud_accel_quiescence_ratio` | EMA of (\|a\|/g−1)² — platform disturbance | unitless |
| `imud_gyro_bias_{x,y,z}_radians_per_second` | estimated gyro bias, body frame | rad/s |
| `imud_temperature_celsius` | IMU die temperature | °C |
| `imud_nis_accel_ratio` | EMA of normalised innovation squared, accel update (d²/2) — 1 = covariance matches innovations, higher = over-confident | unitless |
| `imud_nis_mag_ratio` | EMA of normalised innovation squared, mag update (d²/dof) — 1 = consistent, higher = over-confident | unitless |
| `imud_innov_weight_ratio` | EMA of the Huber weight applied to MEKF updates — 1 = no capping, lower = the filter is persistently distrusting its sensors | unitless |
| `imud_innov_reject_ratio` | EMA of the fraction of MEKF updates rejected by the innovation gate | unitless |
| `imud_state_reset` | MEKF reset itself after a non-finite state; clears when it re-converges | 0/1 |
| `imud_mag_valid` `imud_mag_uncal` `imud_mag_absent` `imud_converged` `imud_heave_valid` `imud_wave_valid` `imud_engine_on` | packet flag bits | 0/1 |

Sea-state gauges read 0.0 until the statistics settle (`imud_wave_valid` = 0);
periods also read 0.0 when becalmed / not rolling — that is the measurement.
`imud_heading_true_degrees` is omitted entirely while declination is unknown.

## Alerting ideas

```yaml
- alert: CompassCalDegraded
  expr: imud_mag_residual_radians > 0.05          # ≈ 3° sustained disagreement
- alert: MagneticInterference
  expr: imud_mag_anomaly_ratio > 0.1
- alert: ImudDown
  expr: imud_up == 0
```

## Transport

Minimal embedded HTTP/1.1 responder: any GET returns the metrics page,
`Connection: close`. Bind address/port via `imud-prometheus.conf` (default
`127.0.0.1:9815`).

A stalled scraper cannot wedge the stream reader. The request is read
non-blocking from inside the same `poll()` that watches the imud stream
socket, so no scraper can delay a frame; a client that connects and then goes
quiet is dropped 2 s later; and the page is served from the cached newest
sample, so answering never touches the stream socket at all. The response
itself is written once under a 2 s `SO_SNDTIMEO`.

One scrape is served at a time — Prometheus scrapes are serial, and the
listener leaves the poll set while a request is in flight, so a second
concurrent scraper waits in the listen backlog rather than being accepted and
starved.
