# Contributing to imud

Thanks for your interest in improving imud. This guide is for people who want to
build, hack on, and contribute to the project.

## Get the source

Fork the repository on your Git host, then clone your fork:

```sh
git clone https://github.com/<you>/imud
cd imud
```

## Prerequisites

- A C11 compiler (gcc or clang) and `make`.
- `libgpiod-dev` — the daemon links libgpiod. The target platform is Raspberry
  Pi OS / Debian Bookworm (or a similar Linux with I²C).
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
  separate `ChangeLog` file.

## Submitting a change

1. **Fork** the repository and create a branch for your work.
2. Keep each pull request **bite-sized — one self-contained feature or fix**.
   Small, focused changes are far easier to review and land than large ones.
3. Make sure `make && make bridges && make test` passes and the tree is
   warning-clean.
4. Open a **pull request** describing what changed and how you tested it.

By contributing, you agree that your contributions are licensed under the
project's MIT License (see [LICENSE](LICENSE)).
