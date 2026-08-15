#!/bin/sh
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
# systemd unit verification + hardening scores for all six units.  Run INSIDE the
# container:  devbox/run devbox/unitcheck.sh
#
# ci.yml runs `systemd-analyze verify` as a hard gate and `security` with
# `|| true`, discarding the scores — so in practice they have never actually
# been read.  This prints every one of them and keeps the full tables.
#
# `verify` matters because systemd only WARNS about a misspelled directive, at
# runtime, in a log nobody reads: a typo in a hardening block silently un-hardens
# the unit.  `security` is informational — it mis-scores units carrying more than
# one SystemCallFilter= line (systemd#23663), which is exactly imud.service's
# shape, so read the number as a trend, not a grade.
set -eu

STAGE=/work/stage
ROOT=/work/unitlint
OUT=/work/unitcheck

rm -rf "$STAGE" "$ROOT" "$OUT"
mkdir -p "$OUT"

echo "=== staged install ==="
make install install-utils install-wmm-data install-signalk install-mqtt \
     install-influxdb install-mavlink install-prometheus \
     DESTDIR="$STAGE" PREFIX=/usr SVCDIR=/lib/systemd/system \
     UDEVDIR=/usr/lib/udev/rules.d >/dev/null
echo "staged to $STAGE"

# systemd-analyze --root= searches the compiled-in unit path
# (/usr/lib/systemd/system).  The packages install to /lib/systemd/system, which
# on a real system is a symlink to /usr/lib — but DESTDIR has no such symlink, so
# assemble a root that looks like an installed system.  ExecStart= is resolved
# under --root too, so the binaries come along; /bin/kill (ExecReload=) is a base
# system utility this repo does not ship, so a stub is enough — a typo'd path
# still fails, because it would not match the stub's path either.
mkdir -p "$ROOT/usr/lib/systemd" "$ROOT/usr/bin" "$ROOT/bin"
cp -r "$STAGE/lib/systemd/system" "$ROOT/usr/lib/systemd/system"
cp    "$STAGE/usr/bin/"*          "$ROOT/usr/bin/"
printf '#!/bin/sh\n' > "$ROOT/bin/kill"
chmod +x "$ROOT/bin/kill"

units=$(cd "$ROOT/usr/lib/systemd/system" && echo *.service)
echo
echo "=== systemd-analyze verify ==="
echo "units: $units"
# --recursive-errors=no keeps this about our units rather than
# network-online.target being absent; --man=no skips Documentation= lookups,
# whose pages are staged under DESTDIR rather than installed.
# shellcheck disable=SC2086  # $units is a list of unit names and MUST word-split
systemd-analyze verify --root="$ROOT" --recursive-errors=no --man=no $units
echo "verify OK"

echo
echo "=== systemd-analyze security (exposure levels) ==="
for u in $units; do
    systemd-analyze security --offline=true --root="$ROOT" "$u" > "$OUT/$u.txt" 2>&1 || true
    if grep -h 'Overall exposure level' "$OUT/$u.txt"; then :; else
        echo "  $u: NO SCORE — see $OUT/$u.txt"
    fi
done
echo
echo "full tables: $OUT/  (host: ~/.cache/imud-devbox/out/unitcheck/)"
