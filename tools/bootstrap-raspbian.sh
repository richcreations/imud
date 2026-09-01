#!/bin/sh
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
# bootstrap-raspbian.sh — build a Raspbian armhf rootfs tarball.
#
# Usage: tools/bootstrap-raspbian.sh <suite> <output.tar> [pkg,pkg,...]
#
# Why this exists at all: "armhf" names two different architectures.  Debian's
# armhf port is ARMv7-A/VFPv3-D16.  Raspberry Pi OS 32-bit is a SEPARATE port
# (Raspbian) rebuilt for ARMv6/VFPv2, so that it still runs on the Pi 1 and Pi
# Zero/Zero W.  Both report the identical string from `dpkg
# --print-architecture`, so apt cannot distinguish them: a package built to the
# Debian baseline installs without complaint on an ARMv6 board and then dies on
# an illegal instruction the first time it runs.  ARMv6 code executes on every
# later core, so building on Raspbian is what makes ONE armhf package correct
# on every Pi.
#
# Both the release build (build-debs.yml) and the 32-bit test job (ci.yml)
# bootstrap through this script, so the rootfs that ships and the rootfs the
# suite is tested in cannot drift apart.
#
# Runs as root or unprivileged: mmdebstrap's default mode picks --mode=root
# when euid is 0 and --mode=unshare otherwise, and the unshare path needs only
# user namespaces plus a subuid range.

set -eu

SUITE="${1:?usage: bootstrap-raspbian.sh <suite> <output.tar> [pkg,pkg,...]}"
OUT="${2:?usage: bootstrap-raspbian.sh <suite> <output.tar> [pkg,pkg,...]}"
INCLUDE="${3:-}"

# Overridable for a mirror closer to the builder; the default is the archive
# the key below actually signs.
RASPBIAN_MIRROR="${RASPBIAN_MIRROR:-http://raspbian.raspberrypi.com/raspbian}"

# The Raspbian archive signing key.  There is no image digest to pin — the
# rootfs is built here, from whatever the archive serves — so this fingerprint
# is the trust anchor for the whole 32-bit build.  tools/check-arch-claims.py
# keeps a second copy, which makes swapping the key a change that has to be
# made twice.
RASPBIAN_KEY_FPR="A0DA38D0D76E8B5D638872819165938D90FDDD2E"

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

# The key comes from the raspbian-archive-keyring PACKAGE, never from the
# raspbian.public.key file the website serves.  That export dates from 2012 and
# carries ONLY SHA-1 binding signatures; Sequoia sqv — which apt uses from
# trixie onward — rejects those outright, and the failure then reads "the
# repository is not signed", pointing at the archive rather than at the key.
# The packaged keyring carries SHA-256 bindings alongside the originals.
pool="$RASPBIAN_MIRROR/pool/main/r/raspbian-archive-keyring"
keydeb="$(curl -fsSL --retry 5 "$pool/" \
          | grep -oE 'raspbian-archive-keyring_[^"<]*_all\.deb' \
          | sort -V | tail -1)"
[ -n "$keydeb" ] || { echo "$0: no raspbian-archive-keyring in $pool" >&2; exit 1; }

curl -fsSL --retry 5 -o "$work/keyring.deb" "$pool/$keydeb"
dpkg-deb -x "$work/keyring.deb" "$work/keyring-root"
key="$work/keyring-root/usr/share/keyrings/raspbian-archive-keyring.gpg"
[ -f "$key" ] || { echo "$0: $keydeb contains no keyring" >&2; exit 1; }

# Fetched over plain HTTP from the very archive we are about to trust, so this
# fingerprint comparison is what makes the key an anchor rather than a circle.
got="$(gpg --show-keys --with-colons "$key" | awk -F: '/^fpr:/{print $10; exit}')"
if [ "$got" != "$RASPBIAN_KEY_FPR" ]; then
    echo "$0: keyring fingerprint $got != pinned $RASPBIAN_KEY_FPR" >&2
    exit 1
fi

# apt inside the bootstrap drops to a sandbox user that cannot read a key held
# under a private temporary directory, so stage it world-readable.
keyfile=/tmp/imud-raspbian-keyring.gpg
install -m 0644 "$key" "$keyfile"

# The merged-usr hook is REQUIRED and must not be left to autodetection.
# Raspbian is merged-usr exactly as Debian is — its base-files ships
# `/lib -> usr/lib` — but mmdebstrap's detection does not fire for this
# vendor, and without the symlinks the first `chroot ... dpkg --install`
# exits 127: every binary in the essential set names
# /lib/ld-linux-armhf.so.3 as its ELF interpreter, and until base-files is
# unpacked that path does not exist.  A missing interpreter is indistinguish-
# able from a missing command, so the error says only "127" and points
# nowhere near the cause.
#
# Only main is enabled: every Build-Depends lives there, and the other
# components would add index downloads for nothing.
set -- --architectures=armhf --variant=apt --components=main \
       --hook-dir=/usr/share/mmdebstrap/hooks/merged-usr \
       --aptopt='Acquire::Retries "5"' \
       --aptopt='Acquire::http::Timeout "60"' \
       --aptopt='APT::Sandbox::User "root"'
[ -n "$INCLUDE" ] && set -- "$@" --include="$INCLUDE"

mmdebstrap "$@" "$SUITE" "$OUT" \
    "deb [signed-by=$keyfile] $RASPBIAN_MIRROR $SUITE main"
