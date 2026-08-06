# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
# The Debian build/test box (devbox/README.md).  Mainly for non-Linux hosts.
#
# macOS cannot link imud, imud-cal or imud-imutest at all (libgpiod, <linux/i2c.h>,
# pthread_condattr_setclock) and cannot build 5 of the 34 test suites, so a plain
# `make test` on the host proves 29.  Worse, it can prove the WRONG thing:
# APPLY_CLOEXEC is a real fcntl() on macOS and ((void)0) on Linux, so 39df6b4
# passed locally and failed every Linux job in CI.  This image is the local Linux
# gate that would have caught it.
#
# The package list mirrors .github/workflows/ci.yml — that file is the source of
# truth for what CI needs; this one only has to be a superset of it.  Nothing here
# feeds a release: a contributor tool, never referenced by the packaging.
#
# Three install layers rather than one, so a slow or unreliable connection is
# survivable: a dropped download only invalidates its own layer, and
# the ~200 MB clang/llvm group is last so the build/test box is usable even if
# the analysis group has to be retried.

# Deliberately NOT digest-pinned, unlike every action in .github/workflows and
# unlike the actionlint tarball.  The whole point of this image is to be built
# twice from one file — `--dist bookworm` is the libgpiod v1 path — and a digest
# names exactly one image, so pinning would mean two hand-maintained digests for
# a tool that builds no release artifact and that nothing in the packaging, the
# build or CI depends on.  A stale dev container is a slow afternoon; a stale
# release dependency is a supply-chain problem, and only the latter is pinned.
# Expect Scorecard's Pinned-Dependencies to count this file: that is the honest
# reading, and the trade is intentional.
ARG DIST=trixie
FROM debian:${DIST}

ARG DEBIAN_FRONTEND=noninteractive

# Survive a flaky link rather than failing the whole layer on one dropped GET.
RUN printf '%s\n' \
        'Acquire::Retries "5";' \
        'Acquire::http::Timeout "60";' \
        'Acquire::https::Timeout "60";' \
        > /etc/apt/apt.conf.d/99devbox-retries

# 1. build-and-test: make, make bridges, make test, the Python client self-check.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential pkgconf libgpiod-dev libmosquitto-dev python3 \
        lsof curl netcat-openbsd procps file git ca-certificates
# lsof/nc/curl are not padding: they are the tools audit.md §2 names for the
# L3 (close-on-exec on a live daemon) and L6 (silent scraper) runtime checks.

# 2. packaging + units: dpkg-buildpackage, dch, hardening-check, lintian, and
#    systemd-analyze verify/security (systemd is never PID 1 in here).
RUN apt-get install -y --no-install-recommends \
        debhelper devscripts fakeroot lintian systemd

# 3. analysis: cppcheck, scan-build, gcov/lcov, man lint — the biggest download
#    by far (clang/llvm is most of it).
#
#    The libFuzzer runtime is a version-suffixed package (libclang-rt-14-dev on
#    bookworm, -19- on trixie) with no unversioned alias, so resolve it from the
#    package lists rather than pinning a number that rots at the next release.
#
#    Keep the comments ABOVE the RUN, not interleaved as `# …` command
#    substitutions: those expand to nothing but still occupy the command's first
#    word, so the `rt=` that follows stops being an assignment and the shell
#    tries to execute it ("rt=libclang-rt-19-dev: not found").  That cost a full
#    re-download of this layer once already.
#
#    The cache mount plus Keep-Downloaded-Packages hold the .debs across
#    rebuilds OF THIS LAYER, so editing the analysis set is free after the first
#    time.  Both are set here rather than in layer 1 because touching layer 1
#    would invalidate everything below it.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
        > /etc/apt/apt.conf.d/99devbox-keep-debs && \
    apt-get install -y --no-install-recommends \
        cppcheck clang clang-tools lcov mandoc && \
    rt=$(apt-cache --names-only search '^libclang-rt-[0-9]+-dev$' \
         | awk '{print $1}' | sort -V | tail -1) && \
    if [ -n "$rt" ]; then \
        echo "fuzzer runtime: $rt"; \
        apt-get install -y --no-install-recommends "$rt"; \
    else \
        echo "WARNING: no libclang-rt-*-dev found; -fsanitize=fuzzer will fail"; \
    fi && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /usr/local/bin/devbox-entry
RUN chmod +x /usr/local/bin/devbox-entry

WORKDIR /work/src
ENTRYPOINT ["/usr/local/bin/devbox-entry"]
CMD ["bash"]
