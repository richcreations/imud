# imud client libraries

Three ways to consume imud's binary telemetry — pick by how you deploy:

| Library | File(s) | Pick it when |
| --- | --- | --- |
| **libimud** (shared library) | `imud.h` + `libimud.so`, `pkg-config libimud` | You install imud from packages / `make install`. **ABI-stable**: your binary survives imud upgrades without recompiling. |
| **C single-header** | `imud_client.h` | You vendor one file into an embedded/static project and *want* to pin the wire version (recompile per wire revision). |
| **Python** | `imud_client.py` | Python 3.8+, standard library only (wire-pinned, like the single-header). |

All three validate the CRC32 on every packet and silently discard anything
malformed. `sudo make install` installs all of them; the daemon's own bridges
link `libimud.so`, and `make test` cross-checks every packet definition
against the daemon's encoder on each run.

---

## C — libimud (`-limud`), the system path

```c
#include <imud.h>
#include <stdio.h>

int main(void) {
    imud_t *h = imud_connect_stream(NULL);   /* local socket (recommended) */
    /* or: imud_connect_udp(10111, "239.255.0.1") for the UDP stream */
    if (!h) { perror("imud_connect_stream"); return 1; }

    while (imud_read(h, 1000) >= 0) {        /* 0 = data, 1 = timeout */
        const imud_data_t *d = imud_data(h);
        printf("hdg=%.1f  heave=%.2f\n", d->heading_deg, d->heave_m);
    }
    imud_free(h);
    return 0;
}
```

Build: `cc app.c $(pkg-config --cflags --libs libimud)`. The `imud_data_t`
struct is **append-only** — access it through the returned pointer and your
binary keeps working across imud updates (see `man 3 libimud` for the full
API and the ABI contract). For select/poll event loops use `imud_fd()`.

---

## C — `imud_client.h`, the vendoring path

Single-header library. No build system required — drop the file into your
project. Note it is **wire-version-pinned**: it validates against the exact
packet version it was compiled with, so rebuild your app when you upgrade
imud across a wire revision (use libimud above to avoid that).

**In exactly one `.c` file:**

```c
#define IMUD_CLIENT_IMPLEMENTATION
#include "imud_client.h"
```

**In all other files (if any):**

```c
#include "imud_client.h"
```

**Basic example:**

```c
#define IMUD_CLIENT_IMPLEMENTATION
#include "imud_client.h"
#include <stdio.h>

int main(void) {
    int fd = imud_open(10111, NULL);   /* NULL = unicast/broadcast */
    if (fd < 0) { perror("imud_open"); return 1; }

    imud_packet_t pkt;
    while (imud_recv(fd, &pkt) == 0) {
        printf("hdg=%6.1f°  pitch=%+5.1f°  roll=%+5.1f°  rot=%+7.1f dpm  seq=%u\n",
               pkt.heading_deg,
               pkt.pitch * (180.0f / 3.14159f),
               pkt.roll  * (180.0f / 3.14159f),
               pkt.rate_of_turn,
               pkt.imu_seq);
    }
    imud_close(fd);
}
```

**Multicast (default hi-rate address):**

```c
int fd = imud_open(10111, "239.255.0.1");
```

**Compile:**

```sh
gcc -o consumer consumer.c -lm
```

**Validate a raw buffer you already received yourself:**

```c
uint8_t buf[IMUD_PACKET_SIZE];
ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
if (imud_packet_valid(buf, (size_t)n)) {
    imud_packet_t *p = (imud_packet_t *)buf;
    /* use p->heading_deg etc. */
}
```

---

## Python — `imud_client.py`

Python 3.8+, standard library only.

**Basic example:**

```python
from imud_client import ImudClient

with ImudClient() as client:
    for pkt in client:
        print(f"hdg={pkt.heading_deg:.1f}°  rot={pkt.rate_of_turn:.1f} dpm  "
              f"flags=[{pkt.flags_str}]")
```

**Multicast:**

```python
with ImudClient(port=10111, addr="239.255.0.1") as client:
    for pkt in client:
        ...
```

**With timeout (returns None when no packet within `timeout` seconds):**

```python
client = ImudClient(timeout=2.0)
client.open()
pkt = client.recv()   # returns None after 2 s silence
client.close()
```

**Run as a command-line monitor:**

```sh
python3 imud_client.py --port 10111
python3 imud_client.py --port 10111 --addr 239.255.0.1
```

**Available packet fields:**

| Field | Type | Notes |
|---|---|---|
| `heading_deg` | float | 0–360° magnetic |
| `pitch` | float | rad, NED (+bow up) |
| `roll` | float | rad, NED (+starboard up) |
| `yaw` | float | rad, magnetic |
| `rate_of_turn` | float | deg/min, + = turning right |
| `temp_c` | float | IMU die temperature |
| `accel` | (x, y, z) | m/s², calibrated |
| `accel_raw` | (x, y, z) | m/s², pre-calibration |
| `gyro` | (x, y, z) | rad/s, bias-corrected |
| `gyro_raw` | (x, y, z) | rad/s, before bias correction |
| `mag` | (x, y, z) | µT, calibrated |
| `mag_raw` | (x, y, z) | µT, pre-calibration |
| `quat` | (w, x, y, z) | unit quaternion, body→NED |
| `cov` | 9-tuple | 3×3 attitude error covariance (rad²), row-major |
| `cov_trace` | float | trace of covariance (sum of diagonal) |
| `ts_unix` | float | UNIX timestamp (seconds) |
| `ts_wall_ns` | int | CLOCK_REALTIME nanoseconds |
| `flags` | int | bitmask — see `Flags` class |
| `flags_str` | str | e.g. `"CVM"` (converged, mag-valid, mag-cal) |
| `imu_seq` | int | monotonic sample counter |
| `declination_deg` | float | °E+; 0.0 when DECLINATION_VALID flag not set |
| `heave_m` | float | vertical displacement, m, + up; 0.0 when heave disabled (v1.1) |
| `gyro_bias_x/y/z` | float | estimated gyro bias, rad/s (v1.2, body frame) |
| `gyro_bias_var_x/y/z` | float | gyro-bias variance, (rad/s)² (v1.2) |
| `heave_rate` | float | vertical velocity, m/s, + up; 0.0 when heave disabled (v1.2) |
| `accel_quiescence` | float | EMA of (\|a\|/g−1)²; platform-disturbance metric (v1.2) |
| `true_heading_deg` | float or None | heading + declination, or None when declination unknown |

---

## Stream B packet layout (v1.2, 228 bytes)

See `spec.md §8` for the complete wire format. The stream is little-endian,
228 bytes per packet (version field = 12, i.e. v1.2), with an IEEE 802.3
CRC32 over the first 224 bytes (0–223). Both libraries validate magic,
version, and CRC before returning a packet.
