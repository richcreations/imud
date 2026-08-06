#!/bin/sh
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
# Container entrypoint: guard against objects from another platform, then exec.
#
# /work/src is a bind mount of the host repo, and the host is macOS.  make
# compares timestamps only — it has no idea a .o is Mach-O rather than ELF, so
# without this it hands the wrong object format to GNU ld and the error names a
# linker problem instead of the real cause.  Checking the artifact (rather than a
# stamp file the host build would never update) stays correct no matter which
# side built last, and catches an amd64 <-> armhf switch by the same test.
set -e

if [ -d /work/src ]; then
    # /usr/bin/make is this container's own architecture, by definition.
    want=$(readelf -h /usr/bin/make 2>/dev/null | sed -n 's/^ *Machine: *//p')

    # EVERY object, not just the first one found: a partial build on the other
    # platform (say `make test_bridge_e2e`, which produces one .entry.o and
    # nothing else) leaves a single mismatched object among correct ones, and
    # sampling one file would walk straight past it into a link error whose
    # message says nothing about the real cause.
    for obj in /work/src/src/*.o /work/src/src/drivers/*.o /work/src/lib/*.o; do
        [ -e "$obj" ] || continue
        have=$(readelf -h "$obj" 2>/dev/null | sed -n 's/^ *Machine: *//p')
        if [ "$have" != "$want" ]; then
            echo "devbox: ${obj#/work/src/} is '${have:-not ELF}', this box builds '$want' — make clean" >&2
            ( cd /work/src && make clean >/dev/null 2>&1 ) || true
            break
        fi
    done
fi

exec "$@"
