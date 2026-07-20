# libimud — API and ABI reference

`libimud` decodes imud's binary telemetry behind an **ABI-stable** surface: the
opaque handle `imud_t` and the append-only view struct `imud_data_t`. It
validates magic, wire version, and CRC32 before returning a packet; invalid
packets are discarded silently.

The wire format itself is specified in `spec.md §8` (source root). Wire
revisions are independent of the library release — the whole point of the ABI
contract below is that consumers do not track them.

- Header: `imud.h` · Link: `-limud` (`pkg-config libimud`) · SONAME: `libimud.so.0`
- Exported symbols are pinned in `lib/libimud.map` (version node `IMUD_0`)

## Connection API

| Function | Returns | Notes |
| --- | --- | --- |
| `imud_t *imud_connect_stream(const char *path)` | handle, or NULL (errno set) | Local AF_UNIX stream — the recommended, lossless same-host path. NULL/"" uses `/run/imud/imud-stream.sock`. |
| `imud_t *imud_connect_udp(int port, const char *group)` | handle, or NULL | High-rate binary UDP stream. port 0 = 10111. A multicast `group` (224.0.0.0/4) is joined; NULL/"" receives unicast/broadcast. |
| `imud_t *imud_connect_tcp(const char *host, int port)` | handle, or NULL | A daemon's `[stream]` TCP listener (`tcp_enabled` in imud.conf) — the remote equivalent of `imud_connect_stream`, same lossless framing. host NULL/"" = 127.0.0.1 (hostnames resolved, IPv4); port 0 = 10112. Added in 1.6 (symbol version `IMUD_1`). |
| `int imud_read(imud_t *h, int timeout_ms)` | 0 data · 1 timeout · -1 lost | Waits up to `timeout_ms` for the next **valid** packet; < 0 blocks indefinitely. After -1, call `imud_reconnect()` or `imud_free()`. |
| `int imud_reconnect(imud_t *h)` | 0 / -1 (errno set) | Tear down and re-dial the same endpoint. Safe to retry with backoff. |
| `const imud_data_t *imud_data(const imud_t *h)` | library-owned pointer | Latest decoded packet. Valid until `imud_free()`; updated in place by each successful `imud_read()`. Meaningful only after the first `imud_read() == 0`. |
| `int imud_fd(const imud_t *h)` | fd, or -1 | Underlying descriptor for select/poll/epoll. When readable, call `imud_read(h, 0)`. |
| `void imud_free(imud_t *h)` | — | Close and release. NULL is a no-op. |

## Introspection

| Function | Notes |
| --- | --- |
| `const char *imud_lib_version(void)` | Library release string, e.g. `"1.5"`. |
| `unsigned imud_wire_version(const imud_t *h)` | Wire version of the most recent packet; 0 before the first. The library accepts exactly the wire version it was built with. |
| `const imud_packet_t *imud_wire(const imud_t *h)` | **Co-versioned tools only** (imud's own bridges). `imud_packet_t` is pinned to the wire version, so this is **not** ABI-stable. Declared only when `imud_client.h` is included *before* `imud.h` — that include is the explicit opt-in to a wire-pinned build. |

## `imud_data_t`

Body-frame XYZ; NED convention; angles in radians unless the name says
degrees. Fields read 0.0 when the producing estimator is disabled or has not
settled — check the corresponding flag.

| Field | Type | Meaning |
| --- | --- | --- |
| `ts_wall_ns` | uint64 | CLOCK_REALTIME, nanoseconds |
| `ts_tai_ns` | uint64 | CLOCK_TAI, nanoseconds |
| `ts_chip_ticks` | uint32 | IMU hardware counter |
| `anchor_gen` | uint32 | increments on wall-clock re-anchor |
| `flags` | uint32 | `IMUD_FLAG_*` bitmask (below) |
| `imu_seq` | uint32 | monotonic sample counter (gaps = dropped packets) |
| `accel[3]` | float | m/s², calibrated |
| `accel_raw[3]` | float | m/s², pre-calibration |
| `gyro[3]` | float | rad/s, bias-corrected |
| `gyro_raw[3]` | float | rad/s, before bias correction |
| `mag[3]` | float | µT, calibrated |
| `mag_raw[3]` | float | µT, pre-calibration |
| `quat[4]` | float | unit quaternion [w, x, y, z], body→NED |
| `pitch` | float | rad, NED (+bow up) |
| `roll` | float | rad, NED (+starboard up) |
| `yaw` | float | rad, NED magnetic |
| `heading_deg` | float | 0–360° magnetic |
| `heading_true_deg` | float | 0–360° true; **-1.0** until declination known |
| `rate_of_turn` | float | deg/min, + = turning right |
| `temp_c` | float | IMU die temperature, °C |
| `cov[9]` | float | 3×3 attitude error covariance, row-major (rad²) |
| `declination_deg` | float | °E+; valid with `IMUD_FLAG_DECLINATION_VALID` |
| `heave_m` | float | vertical displacement, m, + up |
| `heave_rate` | float | vertical velocity, m/s, + up |
| `gyro_bias[3]` | float | estimated gyro bias, rad/s |
| `gyro_bias_var[3]` | float | gyro-bias variance, (rad/s)² |
| `accel_quiescence` | float | EMA of (\|a\|/g − 1)²; disturbance metric |
| `wave_height_m` | float | significant wave height Hs, m (wire v14) |
| `wave_period_s` | float | mean zero-crossing wave period Tz, s; 0 = n/a |
| `roll_period_s` | float | vessel roll period, s; 0 = not rolling |
| `roll_amplitude` | float | significant single amplitude 2σ(roll), rad |
| `pitch_period_s` | float | vessel pitch period, s; 0 = not pitching |
| `pitch_amplitude` | float | significant single amplitude 2σ(pitch), rad |
| `mag_anomaly` | float | EMA of \|\|B\|−\|B_ref\|\|/\|B_ref\| (unitless) |
| `mag_residual` | float | EMA of \|heading innovation\|, rad |

## Flags (`imud_data_t.flags`)

Values are fixed by the wire protocol.

| Flag | Bit | Meaning |
| --- | --- | --- |
| `IMUD_FLAG_MAG_VALID` | 0 | mag healthy and calibrated |
| `IMUD_FLAG_MAG_SET_RESET` | 1 | SET pulse within last read |
| `IMUD_FLAG_FUSION_CONVERGED` | 2 | MEKF covariance settled |
| `IMUD_FLAG_ACCEL_CAL` | 3 | accel calibration applied |
| `IMUD_FLAG_GYRO_CAL` | 4 | gyro bias applied |
| `IMUD_FLAG_MAG_CAL` | 5 | mag hard/soft-iron cal applied |
| `IMUD_FLAG_MOTION` | 6 | reserved — never set as of wire v14 |
| `IMUD_FLAG_FIFO_OVERFLOW` | 7 | sample gap (FIFO overflow) |
| `IMUD_FLAG_STARTUP` | 8 | gyro bias estimation in progress |
| `IMUD_FLAG_SHUTDOWN` | 9 | final packet before clean exit |
| `IMUD_FLAG_DECLINATION_VALID` | 10 | declination known |
| `IMUD_FLAG_HEAVE_VALID` | 11 | heave estimator settled |
| `IMUD_FLAG_WAVE_VALID` | 12 | sea-state stats settled |
| `IMUD_FLAG_ENGINE_ON` | 13 | engine-vibration detector asserting |

## ABI contract

`imud_data_t` is library-owned and **append-only**:

- new quantities are only ever added **after** the existing members; existing
  members never move, change type, or disappear;
- always access it through the pointer `imud_data()` returns — do **not** copy
  the struct by value, and do not bake `sizeof(imud_data_t)` into anything that
  must survive a library upgrade.

Under those rules a binary built against an older `imud.h` reads its fields
correctly from any newer `libimud.so.0`.

### Maintainer discipline

- Adding a wire field = append a member to `imud_data_t` (after the last one),
  fill it in `fill_data()` (`lib/libimud.c`), and extend the offset asserts in
  `test/test_libimud.c`. SONAME stays `libimud.so.0`.
- New functions are appended to `lib/libimud.map`. SONAME stays.
- **Never** reorder, retype, or remove existing `imud_data_t` members or
  exported functions. If that is ever unavoidable (it should not be), bump the
  SONAME (`libimud.so.1`) and the runtime package name (`libimud0` → `libimud1`).

## Python field names

`imud_client.py` exposes the same quantities under Pythonic names. Where they
differ from the C view: `ts_unix` (float seconds), `cov_trace` (trace of the
covariance), `flags_str` (e.g. `"CVM"`), and `true_heading_deg` (float, or
`None` when declination is unknown — the C view uses -1.0 in
`heading_true_deg`). Vector quantities are tuples (`accel`, `gyro`, `mag`,
`quat`); gyro bias is exposed per axis (`gyro_bias_x/y/z`).

## See also

- `man 3 libimud` — the API at a glance
- [manual.md](manual.md) — install, build/link, usage, Python
- `spec.md §8` (source root) — the wire packet format
- `docs/RELEASING.md` — release process (the library ships with the daemon)
