# libimud manual

Install, build against, and use the imud client libraries. For the API and ABI
reference see [spec.md](spec.md); for the wire format itself see `spec.md §8`
in the source root.

Two supported ways to consume imud's binary telemetry:

| Library | File(s) | Pick it when |
| --- | --- | --- |
| **libimud** (shared library) | `imud.h` + `libimud.so`, `pkg-config libimud` | The C client. **ABI-stable**: your binary survives imud upgrades without recompiling. |
| **Python** | `imud_client.py` | Python 3.8+, standard library only (wire-pinned: revalidate per wire revision). |

(A third, `imud_client.h`, is **deprecated** — see the last section.)

Both validate the CRC32 on every packet and silently discard anything
malformed. `make test` cross-checks every packet definition against the
daemon's encoder on each run.

## Install

From packages, `libimud0` provides the runtime library and `libimud-dev` the
header and pkg-config file:

```sh
sudo apt install libimud-dev     # pulls in libimud0
```

From source, `sudo make install` installs `libimud.so`, `imud.h`,
`libimud.pc`, and `man 3 libimud`. The Python client lands in
`/usr/local/share/imud/imud_client.py`.

libimud is built and shipped with the daemon — **keep the two installed
together**. Third-party consumers do not need rebuilding across an imud
upgrade (that is the point of the library), but the installed `libimud.so`
must be upgraded alongside the daemon.

## Build and link

```sh
cc app.c $(pkg-config --cflags --libs libimud)
```

## Use

Connect, then read in a loop. `imud_read()` returns 0 when a new packet is
available, 1 on timeout, and -1 when the connection is lost:

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

Prefer `imud_connect_stream()` on the same host: it is the lossless AF_UNIX
path. `imud_connect_udp()` is for remote consumers of the high-rate stream.

After `imud_read()` returns -1 the connection is gone — call
`imud_reconnect()` (safe to retry with backoff) or `imud_free()`.

**Event loops.** Use `imud_fd()` to integrate with select/poll/epoll; when the
fd is readable, call `imud_read(h, 0)`.

**Do not copy `imud_data_t` by value** or bake `sizeof(imud_data_t)` into
anything that must survive a library upgrade — always read through the pointer
`imud_data()` returns. See the ABI contract in [spec.md](spec.md).

## Python — `imud_client.py`

Python 3.8+, standard library only.

```python
from imud_client import ImudClient

with ImudClient() as client:
    for pkt in client:
        print(f"hdg={pkt.heading_deg:.1f}°  rot={pkt.rate_of_turn:.1f} dpm  "
              f"flags=[{pkt.flags_str}]")
```

Multicast:

```python
with ImudClient(port=10111, addr="239.255.0.1") as client:
    for pkt in client:
        ...
```

With a timeout (returns `None` when no packet arrives within `timeout`
seconds):

```python
client = ImudClient(timeout=2.0)
client.open()
pkt = client.recv()   # None after 2 s silence
client.close()
```

As a command-line monitor:

```sh
python3 imud_client.py --port 10111
python3 imud_client.py --port 10111 --addr 239.255.0.1
```

The Python client is **wire-pinned** — revalidate it against each wire
revision. Its field names are listed in [spec.md](spec.md).

## C — `imud_client.h`, the vendoring path (DEPRECATED)

**Deprecated since the libimud release — use libimud for all new C code.**

The header stays in the source tree because it is the deliberate wire-pinning
mechanism behind libimud's `imud_wire()` opt-in and imud's own bridges, and
existing vendored copies keep working (it still tracks every wire revision).
It is no longer installed by `make install` and gains no new API. Including it
prints a compile-time notice; define `IMUD_CLIENT_ALLOW_DEPRECATED` before the
include to silence it.

Single-header library, no build system required — vendor the file from the
source tree. It is **wire-version-pinned**: it validates against the exact
packet version it was compiled with, so rebuild your app when you upgrade imud
across a wire revision (libimud avoids that).

In exactly one `.c` file:

```c
#define IMUD_CLIENT_IMPLEMENTATION
#include "imud_client.h"
```

In all other files:

```c
#include "imud_client.h"
```

Basic example:

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

Multicast (default hi-rate address):

```c
int fd = imud_open(10111, "239.255.0.1");
```

Compile:

```sh
gcc -o consumer consumer.c -lm
```

Validate a raw buffer you received yourself:

```c
uint8_t buf[IMUD_PACKET_SIZE];
ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
if (imud_packet_valid(buf, (size_t)n)) {
    imud_packet_t *p = (imud_packet_t *)buf;
    /* use p->heading_deg etc. */
}
```
