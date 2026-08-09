#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-devices.py — every device node the shipped config names must be one the
shipped unit allows.

Audit N2: config/imud.conf shipped `gpio_chip = "gpiochip4"` (right for a Pi 5)
while etc/imud.service.in allowed only /dev/gpiochip0, with the gpiochip4 line
commented out under DevicePolicy=closed. So the daemon was forbidden from the
very chip its own default config asked for, and no combination of the two
shipped files was correct on any board. It read that way since the initial
commit, because nothing compared them.

The failure is invisible to every existing check: the config parses, the unit
passes `systemd-analyze verify`, and the mismatch only bites at device-open
time on real hardware. Hence a text check — cheap, and it runs anywhere
python3 does.

Both conventions have to be normalised: `i2c_bus` carries a full path
(/dev/i2c-1) while `gpio_chip` is a bare name (gpiochip4) that src/imu.c
turns into /dev/%s.

Run as `make check-devices`.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CONF = "config/imud.conf"
UNIT = "etc/imud.service.in"

# Keys that name a /dev node, per section, and how to turn the value into a
# path. The bare-name form is not a quirk to paper over: include/config.h
# stores gpio_chip without the prefix and src/imu.c builds "/dev/%s" from it.
#
# spi_dev is per-sensor rather than global, because the IMU and the
# magnetometer sit on separate chip selects — so it is looked for in [imu] and
# [mag], not [device].
SECTION_KEYS = {
    "device": {
        "i2c_bus":   lambda v: v,
        "gpio_chip": lambda v: v if v.startswith("/dev/") else "/dev/" + v,
    },
    "imu": {"spi_dev": lambda v: v},
    "mag": {"spi_dev": lambda v: v},
}

# The section whose absence means the file has been restructured. The sensor
# sections are optional here only in the sense that a config need not select
# SPI; [device] always exists.
REQUIRED_SECTION = "device"


def read(rel):
    path = os.path.join(ROOT, rel)
    if not os.path.exists(path):
        sys.exit(f"{rel}: not found — has it moved?")
    return open(path, encoding="utf-8").read()


def conf_section(text, name):
    """The body of [name] in a .conf, up to the next section header."""
    m = re.search(r"^\[" + re.escape(name) + r"\]\s*$", text, re.M)
    if not m:
        return None
    rest = text[m.end():]
    nxt = re.search(r"^\[[a-z0-9_-]+\]\s*$", rest, re.M)
    return rest[:nxt.start()] if nxt else rest


def configured_devices(conf_text):
    """[(where, key, value, path)] for every active device-valued key."""
    out = []
    for section, keys in SECTION_KEYS.items():
        body = conf_section(conf_text, section)
        if body is None:
            if section == REQUIRED_SECTION:
                sys.exit(f"{CONF}: no [{section}] section — has it been renamed?")
            continue
        for key, to_path in keys.items():
            # Active lines only. A commented-out alternative is a suggestion,
            # and the operator who uncomments it owns the matching unit edit.
            m = re.search(r'^\s*' + re.escape(key) + r'\s*=\s*"?([^"#\s]+)"?',
                          body, re.M)
            if m:
                out.append((f"[{section}] {key}", key, m.group(1),
                            to_path(m.group(1))))
    return out


def allowed_devices(unit_text):
    """Device paths on active DeviceAllow= lines, and any malformed lines.

    The rights token is validated because systemd parses everything after the
    path as the rights: a trailing comment makes them invalid, and the whole
    line is then IGNORED rather than rejected. Under DevicePolicy=closed that
    silently allows nothing — strictly worse than the mismatch this script was
    written for, and visible only to `systemd-analyze verify`, which needs a
    container. Cheap to catch here instead.
    """
    paths, malformed = set(), []
    for m in re.finditer(r'^\s*DeviceAllow=(.*)$', unit_text, re.M):
        value = m.group(1).rstrip()
        parts = value.split()
        # "char-tty rw" and similar device-class forms are legal too; only the
        # rights token is checked, since that is what the comment corrupts.
        if len(parts) != 2 or not re.fullmatch(r"[rwm]+", parts[1]):
            malformed.append(value)
            continue
        paths.add(parts[0])
    return paths, malformed


def main():
    conf = read(CONF)
    unit = read(UNIT)

    devices = configured_devices(conf)
    allows, malformed = allowed_devices(unit)

    # An extractor that silently finds nothing must fail, not pass. Both of
    # these would otherwise turn a renamed key or directive into a green run.
    if not devices:
        sys.exit(f"{CONF}: no device keys found — expected at least one of "
                 f"{sorted(SECTION_KEYS[REQUIRED_SECTION])} in "
                 f"[{REQUIRED_SECTION}]")
    if not allows:
        sys.exit(f"{UNIT}: no DeviceAllow= lines found — "
                 f"has the hardening block been removed?")

    failures = []
    for value in malformed:
        failures.append(
            f"{UNIT}: DeviceAllow={value} — the rights token must be one of "
            f"r/w/m with nothing after it. systemd would ignore this line "
            f"entirely, allowing nothing under DevicePolicy=closed. "
            f"(A trailing comment does this; put it on its own line.)")

    for where, _key, value, path in devices:
        if path not in allows:
            failures.append(
                f"{CONF} {where} = {value} resolves to {path}, "
                f"which {UNIT} does not allow. Under DevicePolicy=closed the "
                f"open is refused. Allowed: {', '.join(sorted(allows))}")

    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(f"\n{len(failures)} device mismatch(es)", file=sys.stderr)
        return 1

    print(f"check-devices: {len(devices)} configured device(s), "
          f"{len(allows)} allowed by the unit, all reachable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
