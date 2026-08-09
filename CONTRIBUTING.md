# Contributing to imud

Thanks for your interest in improving imud. This guide is for people who want to
build, hack on, and contribute to the project. For who decides what lands and
how, see [GOVERNANCE.md](GOVERNANCE.md).

## Get the source

Fork the repository on your Git host, then clone your fork:

```sh
git clone https://github.com/<you>/imud
cd imud
```

## Prerequisites

- A C11 compiler (gcc or clang) and `make`.
- `libgpiod-dev` — the daemon links libgpiod, and the Makefile auto-detects
  which API to build against from `pkg-config`: v1 (Debian bookworm ships 1.6)
  or v2 (trixie ships 2.x). The target is Linux generally — anything with I²C
  and a GPIO character device. Packages are built for arm64 and armhf on both
  bookworm and trixie, and Raspberry Pi OS is the most exercised host, not a
  requirement.
- `libmosquitto-dev` — only for building the MQTT bridge.

## Build and test

```sh
make            # core: imud, imud-cal, imud-status, imud-mon
make bridges    # the optional bridge daemons
make test       # the test suite — run from the repo root
```

Please keep the tree **warning-clean**: the build uses `-Wall -Wextra`, and CI
runs `make && make bridges && make test`. Add or extend a test for any new
behaviour.

**Not developing on Linux?** `devbox/run make -j4 test` runs the whole suite in
a throwaway Debian container matching CI's environment. macOS builds only 29 of
the 34 suites, and it can actively prove the *wrong* thing — a macOS-only
`fcntl()` once passed the local gate and turned every Linux job in CI red — so
this is the check to run before pushing. Setup, and the recipes macOS cannot run
at all (sanitizers, coverage, `.deb` builds, systemd unit verification), are in
[devbox/README.md](devbox/README.md). On Linux it is optional, but it is the
quick way to test the other Debian release (`--dist bookworm`, the libgpiod v1
path Raspberry Pi OS still ships) without touching your own toolchain.

## Coding conventions

- C11 and POSIX. Match the surrounding style. The **core** takes no new external
  dependencies; an optional bridge may take one, confined to that bridge's own
  package.
- Every source file carries the copyright header:
  ```c
  /*
   * imud — IMU daemon
   * Copyright (c) 2026 Richard Simpson
   * SPDX-License-Identifier: MIT
   */
  ```
- Units are SI unless a name ends in `_deg`; the sensor frame is NED-compatible
  (X forward, Y starboard, Z down). See [docs/manual.md](docs/manual.md).
- **Keep the docs in sync with the code, in the same change.** imud
  deliberately keeps the same facts in several places (man pages, the manual,
  the protocol spec, the client libraries, and tests); update the parallel
  surfaces together. Release-level notes go in `NEWS` and the per-package
  changelogs — the Git history is the detailed changelog, so there is no
  separate `ChangeLog` file. Cutting a release follows
  [docs/RELEASING.md](docs/RELEASING.md) (the canonical version is
  `include/version.h`).

## Doc-sync discipline

This project states the same fact in several places on purpose — a config key
lives in the parser, the shipped template, the man page and the manual, and a
reader of any one of them should get the truth. A code change is not done
until its parallel surfaces move with it, **in the same commit**.

Most of that is now machine-enforced. `make check-generated-text` proves:

| Checker | What it will not let you break |
|---|---|
| `check-docs` | every key `src/config.c` parses appears in its template, man page and manual; and `docs/config-keys.toml` names the same keys, fields and `NEED_*` macros `apply_kv()` really uses |
| `check-config-docs` | the man5 entries, the manual tables and `test/test_config_defaults.gen.c` are what the registry renders — one key, one home, four surfaces |
| `check-cli-docs` | every flag a parser accepts is in that tool's `--help` **and** its man page, manual, spec and the website |
| `check-packet-docs` | `spec.md`'s offset table and flags bitmask **are** `imu_packet_t` and the `FLAG_*` defines — generated, so an inserted field renumbers every row beneath it and the wire version cannot go stale |
| `check-driver-docs` | `docs/manual.md` §5 **is** `src/drivers.c` — name, type, SPI mode and clock, and the experimental marker, with the parts and notes in `docs/driver-notes.toml`; every documented rate list comes from the `.supported_*` array it describes |
| `check-nmea` | the documented sentence set and count match `nmea_encode()` |
| `check-mqtt-topics`, `check-bridge-outputs` | each bridge's spec lists exactly what its encoder emits, **and nothing it no longer emits**; each MQTT topic's documented condition matches its `GATE_*` |
| `check-libimud-api` | `libimud.map`, `imud.h`, `libimud.3` and the libimud spec describe one API |
| `check-links` | every link resolves — in the repo **and** in the tree `make install` lays down |
| `check-flags`, `check-devices` | the flags word agrees across all four definitions; the config's device nodes are ones the unit permits |

Run it before you commit; CI runs it on every push. `make test-tools` then
checks the checkers themselves still detect drift, by breaking one fact at a
time in a copy of the tree.

**Adding a driver** used to mean editing six files. It is now: write
`src/drivers/<part>.c`, register it in `src/drivers.c`, add an entry to
`docs/driver-notes.toml` (the part, its address range, the reference GPIO, why
it is not on SPI if it is not, and the notes), and run `make docs-tables`. The
driver table, the type column, the SPI column and every supported-rate list
follow from the code. The one hand edit left is the driver-name sentence in
`docs/config-keys.toml`'s `[imu] driver` / `[mag] driver` entry — prose, not a
table, so it is checked rather than generated, and forgetting it fails.

What is still on you, because no checker covers it:

- **Prose.** The checkers compare names, numbers and structure, never wording.
  If you change what a setting *does*, every surface that explains it needs
  rewriting — they are deliberately different lengths, not copies.
- **`NEWS`**, and the Debian changelogs under `packaging/`. Release notes are
  written for an operator deciding whether an upgrade will bite them; nothing
  can generate that. CI only checks the version numbers agree.
- **A `[hot]` key** must also reach `HOT_FIELDS()` in `test/test_config.c` (the
  two lists are independent on purpose and `test_apply_hot_partition` fails if
  they disagree), and `imu_ctx_update_config()` in `src/imu.c` if the fusion
  thread reads it. That it reaches `config_apply_hot()` at all, and that the
  man page marks it the same way, `check-docs` now proves.
- **A new config key** needs a `fill_distinct()` line in `test/test_config.c` —
  that helper is written from the struct, and without the line the partition
  test cannot see the field at all. Its documented default no longer needs a
  hand-written assertion: describe the key once in `docs/config-keys.toml`, run
  `make docs-config`, and commit what it writes.
- **Installed docs must be self-contained.** Anything a shipped document links
  to has to ship too, and the doc tree mirrors the source tree so one relative
  path is correct in both. `check-links` enforces both, but deciding whether a
  new document belongs in the package is a judgement call — contributor-facing
  material that deep-links into `src/` should stay repo-only.

## Submitting a change

1. **Fork** the repository and create a branch for your work.
2. Keep each pull request **bite-sized — one self-contained feature or fix**.
   Small, focused changes are far easier to review and land than large ones.
3. Make sure `make && make bridges && make test` passes and the tree is
   warning-clean.
4. **Sign off every commit** with `git commit -s` — see below.
5. Open a **pull request** describing what changed and how you tested it.

## Developer Certificate of Origin

imud uses the [Developer Certificate of Origin][dco] (DCO) — the same
lightweight mechanism the Linux kernel uses — rather than a Contributor
License Agreement. There is nothing to sign up for and no paperwork to mail:
you certify it per commit, with a line Git adds for you.

```sh
git commit -s -m "ism330dhcx: handle a short FIFO read"
```

`-s` appends a trailer built from your `user.name` and `user.email`:

```
Signed-off-by: Jane Developer <jane@example.com>
```

Adding it certifies the [DCO](DCO), reproduced verbatim in this repository —
in short: you wrote the patch or otherwise hold the right to submit it under
the project's license, and you understand that the contribution and your
sign-off are public and kept indefinitely. Use a real name; a sign-off is a
legal assertion and an anonymous one cannot serve that purpose.

Forgot one? `git commit -s --amend` fixes the tip commit, and
`git rebase --signoff origin/main` fixes a whole branch. CI checks every
non-merge commit in a pull request, so a missing sign-off fails the build
rather than surfacing during review.

Contributions are licensed under the project's MIT License (see
[LICENSE](LICENSE)). The sign-off certifies that you are entitled to license
them that way — it assigns nothing, and you keep your copyright.

[dco]: https://developercertificate.org/
