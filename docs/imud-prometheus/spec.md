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
| `imud_mag_valid` `imud_converged` `imud_heave_valid` `imud_wave_valid` `imud_engine_on` | packet flag bits | 0/1 |

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
`Connection: close`, 2 s socket timeouts (a stalled scraper cannot wedge the
stream reader — scrapes are served from the cached newest sample). Bind
address/port via `imud-prometheus.conf` (default `127.0.0.1:9815`).
