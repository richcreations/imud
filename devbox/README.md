# devbox — the Debian build/test box

A throwaway Debian container that runs the full test suite, the sanitizers, the
packaging and the systemd checks the way CI does. Nothing in the build, the
packaging or CI refers to anything in here, and none of it ships in a package —
it is a contributor convenience, not part of the product.

Two audiences:

- **Developing on macOS (or any non-Linux host).** This is the only way to run
  the whole gate before pushing. See "Why" below — the host build does not just
  cover less, it can actively prove the wrong thing.
- **Developing on Linux.** You do not need it for `make test`, but it is the
  quick way to check the *other* Debian release (`--dist bookworm` builds
  against libgpiod v1, which Raspberry Pi OS still ships), to build the `.deb`s
  without installing debhelper and lintian on your own machine, or to reproduce
  a CI failure in CI's actual environment.

## Why

`make test` runs 37 suites. macOS builds 31 of them. It cannot link `imud`,
`imud-cal` or `imud-imutest` at all (no libgpiod, no `<linux/i2c.h>`, no
`pthread_condattr_setclock`), cannot build `test_ring`, `test_concurrency`,
`test_drivers`, `test_drivers_registry`, `test_imutest` or `test_hwtools_e2e`
(the last four need GNU ld's `--wrap`), and `make coverage` there is blind to
`imu.c`, `ring.c` and all twelve drivers.

It can also prove the wrong thing. `APPLY_CLOEXEC` is a real `fcntl()` on macOS
and `((void)0)` on Linux, so `39df6b4` passed the local gate and turned every
Linux job in CI red. That is the bug this box exists to catch before a push.

## Setup (once)

Any Docker-compatible engine works; on Linux, a native Docker or Podman install
is all you need and you can skip straight to "Use". The rest of this section is
the macOS path.

```sh
brew install colima docker-buildx
mkdir -p ~/.docker/cli-plugins
ln -sfn "$(brew --prefix)/opt/docker-buildx/bin/docker-buildx" \
        ~/.docker/cli-plugins/docker-buildx
colima start --cpu 4 --memory 4 --disk 60 --vm-type vz --mount-type virtiofs
```

`brew install docker` alone gives only the client — macOS has no container
engine, and `docker build` on CLI v29 needs the buildx plugin, hence both lines.
`colima stop` gives the 4 GB back; `colima status` says what is running. The
first `devbox/run` builds the image (~1 GB of Debian packages, several minutes);
every run after that starts instantly.

## Use

```sh
devbox/run                        # interactive shell, at the repo root
devbox/run make -j4 test          # the full 34-suite gate
devbox/run --dist bookworm make -j4 test   # the libgpiod v1 path (Pi OS today)
devbox/run --rebuild true         # rebuild the image after editing the Dockerfile
```

The repo is bind-mounted at `/work/src`; edits on either side are the same files.
Build products that would otherwise litter the repo's parent — `.deb`, `.changes`
— land in `~/.cache/imud-devbox/out/` on the host, because that directory is
mounted as `/work`.

### The recipes macOS cannot run

```sh
devbox/run make coverage           # lcov now LISTS imu.c, ring.c, drivers/*
devbox/run make check-docs

# `make clean` first on both: make compares timestamps, not flags, so without it
# the existing binaries are simply re-run un-instrumented.
devbox/run sh -c 'make clean && make -j4 test \
    CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Wall -Wextra" \
    LDFLAGS="-fsanitize=address,undefined"'
devbox/run sh -c 'make clean && make -j4 test \
    CFLAGS="-O1 -g -fsanitize=thread -Wall -Wextra" LDFLAGS="-fsanitize=thread"'

devbox/run cppcheck --enable=warning,performance,portability --inline-suppr \
                    --std=c11 -Iinclude --suppress=missingIncludeSystem \
                    --error-exitcode=1 --quiet src lib
devbox/run scan-build --status-bugs --keep-going make all bridges

# CI blocks on this one and `make test` does not run it, so it is easy to
# discover only after a push.
devbox/run pyflakes3 lib/imud_client.py tools/*.py test/test_checkers.py

devbox/run dpkg-buildpackage -us -uc -b     # → ~/.cache/imud-devbox/out/
devbox/run sh -c 'lintian --info /work/*.changes'
devbox/run devbox/unitcheck.sh              # verify + the six exposure scores
```

### Running the daemon without a sensor

`driver = "sim"` with `int_gpio = 0` skips the GPIO chip entirely, but
`src/imu.c` opens `i2c_bus` unconditionally even in sim mode — point it at
`/dev/null`, which opens `O_RDWR` fine and which the sim driver's
`probe`/`reset`/`init` ignore:

```sh
devbox/run bash
  make && make bridges
  mkdir -p /run/imud
  sed -e 's|^i2c_bus.*|i2c_bus = "/dev/null"|' \
      -e 's|^driver .*|driver = "sim"|' -e 's|^int_gpio.*|int_gpio = 0|' \
      config/imud.conf > /work/sim.conf
  ./imud --config /work/sim.conf &
  lsof +f g -p $(pidof imud)                   # every fd must be close-on-exec

  sed -e 's|^http_enabled.*|http_enabled = true|' \
      config/imud-prometheus.conf > /work/prom.conf
  ./imud-prometheus --config /work/prom.conf &
  nc 127.0.0.1 9815 &                          # connect, send nothing
  curl -s 127.0.0.1:9815/metrics | head        # must still answer immediately
```

The exporter listens on **9815** (the `listen_port` default), not the 9100 you
might assume from node_exporter.

A unit *starting* under `systemctl` needs systemd as PID 1, which this container
is not — `unitcheck.sh` verifies the unit files offline instead.

## Gotchas

- **`chmod()` on an AF_UNIX socket fails with EINVAL on the mounted repo.**
  virtiofs does not support it, and imud chmods every socket it binds to 0660
  (`bind_unix_mode`, in `include/fileio.h`) — so a daemon
  whose `[stream] socket` path is inside `/work/src` dies at startup with
  `[output] stream bind/chmod(...): Invalid argument`. Put runtime sockets in
  `/tmp` (the container's own filesystem), which is what `test_stream.c` and
  `test_daemon.c` do. Binding works fine there; only `chmod` is refused.
- **After a container build, a plain `make` on macOS needs `make clean` first**
  — the tree holds ELF objects. The reverse is handled automatically: the
  entrypoint checks every `.o` in the tree against the container's own
  architecture and cleans when any disagrees.
- **Never filter a test run through `grep` for the good lines.** TSan aborted
  every binary here with `FATAL: ThreadSanitizer: unexpected memory mapping`,
  and a recipe of the form `make test … | grep -E "passed|FAIL" | tail` printed
  *nothing at all* — the abort matched no pattern, and the pipe to `tail` threw
  away make's non-zero exit. Silence read exactly like success. Redirect to a
  file and check `$?`.
- **Do not run the fuzzers next to an emulated build.** libFuzzer's watchdog
  measures wall time, so on a 2-core box a `-timeout=10` unit reported
  `ERROR: libFuzzer: timeout after 48 seconds` purely because an emulated armhf
  install was saturating both cores. The saved `timeout-*` artifact ran in
  **0.7 s** on its own, and a quiet re-run did 378k units with
  `slowest_unit_time_sec: 0`. **Time the artifact before believing a timeout**
  — and run the harnesses from `/tmp`, since libFuzzer drops `crash-*` /
  `timeout-*` reproducers in the CWD and the repo root is a mounted volume.
- **TSan under colima's kernel** needs `vm.mmap_rnd_bits=28`; the VM's default
  entropy is higher than the sanitizer runtime supports. Fix it persistently
  with a `provision:` entry in `~/.colima/default/colima.yaml` — a bare
  `colima ssh -- sudo sysctl -w vm.mmap_rnd_bits=28` is lost on the next VM
  boot. If TSan aborts with `unexpected memory mapping`, that is this.
- **`--arch armhf`/`arm64` are emulated** on an x86 host and are slow. They
  need binfmt registered in the VM first:
  `docker run --privileged --rm tonistiigi/binfmt --install arm`.

  **The symptom when it is missing does not mention binfmt or qemu.** The image
  build dies on its first `RUN`, whatever that happens to be, with

  ```
  #6 0.363 exec /bin/sh: exec format error
  ```

  — the container cannot execute *any* ARM binary, so it fails before reaching
  anything of yours. Run the binfmt line and retry.

  **The registration is VM state, not host state**: it survives `colima stop`
  and `colima start`, but not `colima delete`, and it is not in
  `~/.colima/default/colima.yaml`'s `provision:` block the way the TSan sysctl
  is. Expect to re-run it after rebuilding the VM.

  Budget ~7 minutes for the first armhf run — the emulated apt install of the
  toolchain is nearly all of it. The image is cached afterwards, so later runs
  are the build and test only.

  CI already builds and tests armhf natively, so this is only for chasing a
  32-bit-specific bug locally. There is a live one: `parse_int` in
  `src/config.c` carries `#if LONG_MAX > INT_MAX`, and on armhf that branch
  compiles out, leaving the `ERANGE` test as the sole guard. A mutation of it
  fails nothing on amd64 and two assertions on armhf, so amd64 alone cannot
  tell you whether that guard works. Same shape as `APPLY_CLOEXEC`.
- **`vz` needs macOS 13+.** If `colima start --vm-type vz` fails, drop both VM
  flags and take the qemu + sshfs default; everything still works, more slowly.

## Files

| | |
|---|---|
| `Dockerfile` | the image; package list mirrors `.github/workflows/ci.yml` |
| `entrypoint.sh` | cleans objects left by another platform, then execs |
| `run` | the wrapper — engine check, image build, mounts, flags |
| `unitcheck.sh` | staged install → `systemd-analyze verify` + exposure scores |
