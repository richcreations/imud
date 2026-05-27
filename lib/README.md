# imud client libraries

Two libraries for consuming the imud high-rate binary stream (Stream B, UDP port 10111):

| Library | File | Requirements |
| --- | --- | --- |
| **C** | `imud_client.h` | Single-header drop-in, POSIX sockets, no build system needed |
| **Python** | `imud_client.py` | Python 3.8+, standard library only |

Both libraries validate the CRC32 on every packet and silently discard any
datagram that is the wrong size, has the wrong magic/version, or fails CRC.

---

## C — `imud_client.h`

Single-header library. No build system required — drop the file into your project.

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

---

## Stream B packet layout (v6, 188 bytes)

See `spec.md §8` for the complete wire format. The stream is little-endian
with an IEEE 802.3 CRC32 over the first 184 bytes. Both libraries validate
magic, version, and CRC before returning a packet.
