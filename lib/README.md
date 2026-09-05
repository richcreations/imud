# imud client libraries

The docs for these live with every other component's, under
[docs/libimud/](../docs/libimud/) (and are installed to
`/usr/share/doc/libimud/`):

- **[README](../docs/libimud/README.md)** — what libimud is, and a quick start
- **[manual](../docs/libimud/manual.md)** — install, build/link, use, the Python
  client, and the deprecated `imud_client.h` vendoring path
- **[spec](../docs/libimud/spec.md)** — API reference, `imud_data_t` fields,
  flags, and the ABI contract

Also: `man 3 libimud`, and `spec.md §8` in the source root for the wire format.

## What is in this directory

| File | Role |
| --- | --- |
| `imud.h` | Public API of libimud — **ABI-stable**; the header you install and include |
| `libimud.c` | Library implementation |
| `libimud.map` | Exported symbol list (version nodes `IMUD_0`, `IMUD_1`) |
| `imud_client.h` | Single-header wire-pinned client — **DEPRECATED**, kept for `imud-mon` and existing vendored copies |
| `imud_client.py` | Python client (3.8+, standard library only) |

The Arduino/ESP32 client,
[imud-arduino](https://github.com/richcreations/imud-arduino) (library name
`ImudClient`), is maintained in its own repository. It is wire-pinned — sync
it whenever the wire format changes.
