# libimud

**Read imud's telemetry from C without pinning yourself to the wire format.**

`libimud` is the ABI-stable client library for imud. It dials the daemon's
local stream socket, the high-rate UDP stream, or — since 1.6 — a remote
daemon's `[stream]` TCP listener (`imud_connect_tcp`), validates every packet
(magic, version, CRC32), and hands you the newest fused state as a plain
struct — attitude, heading, heave, sea state, and the fusion/compass-health
diagnostics.

The point of the library is the **ABI contract**: `imud_data_t` is append-only,
so a binary compiled against an older `imud.h` keeps working against any newer
`libimud.so.0` — no recompile when imud's wire format revs. imud's own bridges
are built on it, which is what keeps them wire-agnostic.

```c
#include <imud.h>
#include <stdio.h>

int main(void) {
    imud_t *h = imud_connect_stream(NULL);   /* /run/imud/imud-stream.sock */
    if (!h) { perror("imud_connect_stream"); return 1; }

    while (imud_read(h, 1000) >= 0) {        /* 0 = data, 1 = timeout */
        const imud_data_t *d = imud_data(h);
        printf("hdg=%.1f  heave=%.2f\n", d->heading_deg, d->heave_m);
    }
    imud_free(h);
    return 0;
}
```

```sh
cc app.c $(pkg-config --cflags --libs libimud)
```

A Python client (`imud_client.py`, standard library only) ships alongside it,
and a deprecated single-header C path (`imud_client.h`) remains for the
wire-pinned cases — both are covered in the manual. For microcontrollers,
[imud-arduino](https://github.com/richcreations/imud-arduino) (library name
`ImudClient`) receives the same stream over TCP or UDP on Arduino/ESP32
boards; it is maintained in its own repository.

## Documentation

- [manual.md](manual.md) — install, build/link, use, Python, the deprecated header
- [spec.md](spec.md) — API reference, `imud_data_t` fields, flags, ABI contract
- `man 3 libimud` — the API at a glance
