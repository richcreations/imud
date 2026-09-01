# Security policy

## Reporting a vulnerability

Please report security issues **privately**, not as a public issue.

Use GitHub's private vulnerability reporting — the **Security** tab of this
repository → **Report a vulnerability**. If that is unavailable to you, email
<richcreations@gmail.com> with `imud security` in the subject.

Please include the imud version (`imud --version`), the platform (Raspberry Pi
OS bookworm/trixie, 32- or 64-bit), and enough detail to reproduce — a config
file, a capture, or a packet dump is ideal. If you have a reproducer that fits
one of the fuzz harnesses below, that is the fastest possible report.

This is a single-maintainer project, so please allow a few days for an initial
response. You will get an acknowledgement, an assessment, and credit in the
`NEWS` entry for the fix unless you would rather not be named.

## Supported versions

Fixes land in the next release from `main`. Only the **latest release** is
supported; there are no maintenance branches for older versions.

The apt repository at <https://richcreations.github.io/imud/apt/> carries the
three most recent releases per Debian suite, so an upgrade path is always
available — but only the newest of those receives fixes.

## What is in scope

imud runs as an unprivileged system user (`imud`) and reads several inputs it
does not control. Those parsers are the interesting attack surface. Each of
them — and the command line, which is not one, for the reason below — has a
dedicated fuzz harness under `fuzz/` that runs for an hour a night:

| Input | Source | Harness |
| --- | --- | --- |
| Config file | `/etc/imud/*.conf` | `fuzz_config` |
| JSON | gpsd / Signal K over the network | `fuzz_json` |
| Wire packets | the binary UDP/stream protocol | `fuzz_packet` |
| `.imucap` captures | replayed capture files | `fuzz_capture` |
| Calibration | `/etc/imud/cal.json` | `fuzz_cal` |
| Magnetic model | `WMM.COF` coefficient file | `fuzz_wmm` |
| Command line | `argv`, from whoever starts the process | `fuzz_argv` |

The last two go beyond the parse: values that survive validation are fed into
the code that consumes them — `apply_imu_cal` / `apply_mag_cal` on the sample
hot path, and the spherical-harmonic field evaluation at the poles and far
from the model epoch — because a non-finite value quietly poisoning the filter
is a worse outcome than a rejected file.

Command-line arguments cross no trust boundary: `argv` is supplied by whoever
launches the process, at that user's privilege. `fuzz_argv` drives them anyway,
because the five front-ends in `src/cli.c` do `strtol`/`strtod` conversions and
fixed-buffer copies over arbitrary strings, and `parse_int`'s `ERANGE` guard is
ABI-conditional (`#if LONG_MAX > INT_MAX`), so 64- and 32-bit builds take
different paths through it.

It runs alongside the unit tests rather than in place of them — `test_cli` for
the five tool front-ends and `test_bridge` for the bridges' shared parser.
Every flag that consumes the argument after it is driven as the final
argument, so each `i + 1 < argc` guard is asserted individually.

Also in scope: the `libimud` client library (a bad packet must never
compromise a consuming application), privilege or permission errors in the
packaging (the daemon must not need root at runtime), and anything that lets a
network peer affect the daemon beyond the documented outputs.

Two bridge configs can hold credentials — an MQTT broker `password` in
`/etc/imud/imud-mqtt.conf` and an InfluxDB `http_token` in
`/etc/imud/imud-influxdb.conf`. Both install **mode 0640 owned `root:imud`**,
readable only by root and by the `imud` user the bridges run as; the staged
install layout is checked in CI so it cannot regress to 0644. Every other file
under `/etc/imud/` holds no secrets and is world-readable by design. imud has
no other credential storage: it accepts no passwords of its own and writes none
to its logs or captures.

## What is out of scope

- The binary UDP broadcast and the AF_UNIX stream are **unauthenticated by
  design** — they are local/LAN telemetry, like gpsd's. Anyone who can reach
  the socket can read the attitude data. Restrict this at the network layer;
  it is not a vulnerability.
- Physical I²C bus access, and anything requiring root on the host.
- Denial of service by a local user who can already exhaust the machine's
  CPU, memory, or file descriptors.
- Inaccurate attitude output. Fusion bugs are ordinary bugs — please file
  them publicly, with a capture if you can.

## Verifying what you install

Packages from the apt repository are covered by two independent mechanisms:

- **The repository index is GPG-signed.** `apt` verifies it against
  `/usr/share/keyrings/imud.gpg`; see the
  [apt repository page](https://richcreations.github.io/imud/apt/).
- **Release artifacts carry build provenance.** Every `.deb` and source
  tarball attached to a GitHub Release is attested to this repository's
  release workflow, so you can confirm a package really was built here:

  ```sh
  gh attestation verify imud_1.7-1~trixie1_arm64.deb --repo richcreations/imud
  ```

If a signature or attestation ever fails to verify, treat that as a security
report and use the private channel above.
