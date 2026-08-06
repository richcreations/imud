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
