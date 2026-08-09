#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
driverlib.py — read the driver registries and the *_ops initialisers.

Every fact about a driver that the documentation restates is a literal in one
of those initialisers: the name a config file writes, whether it is
experimental, whether SPI is offered and at what mode and clock, and the rate
and range lists.  Two generators need them — gen-drivers.py for the driver
tables, gen-config-docs.py for the `supports:` lists in the [imu] and [mag]
key documentation — so the extraction lives here rather than in either.

man/man5/imud.conf.5 once told operators

    ISM330DHCX supports: 12, 26, 52, 104, 208, 416, 833, 1660.

while ism330dhcx.c had `.supported_odr_hz = { ..., 1660, 3332, 6664, 0 }`.
config/imud.conf and docs/manual.md both listed all ten.  That is the drift
this exists to make impossible.

Import from a sibling in tools/:

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from driverlib import drivers
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, must_read                            # noqa: E402

REGISTRY = "src/drivers.c"
DRIVER_DIR = "src/drivers"

ARRAYS = ("supported_odr_hz", "supported_accel_g", "supported_gyro_dps")


def _registries(rep):
    """{'imu': [ops symbol...], 'mag': [...]} in registry order."""
    src = must_read(REGISTRY, "the driver registries")
    out = {}
    for kind in ("imu", "mag"):
        m = re.search(kind + r"_registry\[\]\s*=\s*\{(.*?)\n\};", src, re.S)
        if not m:
            sys.exit(f"{REGISTRY}: cannot find {kind}_registry[]")
        out[kind] = re.findall(r"&(\w+_ops)\b", m.group(1))
        rep.expect(out[kind], f"{kind}_registry entries")
    return out


def _ops_blocks():
    """{ops symbol: (initialiser text, file)} across src/drivers/*.c."""
    blocks = {}
    d = os.path.join(ROOT, DRIVER_DIR)
    for name in sorted(os.listdir(d)):
        if not name.endswith(".c"):
            continue
        src = open(os.path.join(d, name), encoding="utf-8").read()
        for m in re.finditer(r"\b(\w+_ops)\s*=\s*\{(.*?)\n\};", src, re.S):
            blocks[m.group(1)] = (m.group(2), f"{DRIVER_DIR}/{name}")
    return blocks


def _scalar(text, name):
    m = re.search(r"\." + re.escape(name) + r"\s*=\s*([^,\n}]+)", text)
    return m.group(1).strip() if m else None


def _int_array(text, name):
    """The 0-terminated .<name> array as a list of ints, or None."""
    m = re.search(r"\." + re.escape(name) + r"\s*=\s*\{(.*?)\}", text, re.S)
    if not m:
        return None
    nums = [int(n) for n in re.findall(r"\b(\d+)\b", m.group(1))]
    while nums and nums[-1] == 0:            # drop the terminator
        nums.pop()
    return nums


def drivers(rep):
    """[driver dict] in registry order, imu first.

    Keyed downstream by (kind, name), never by name alone: sim_imu_ops and
    sim_mag_ops are BOTH .name = "sim", and a name-keyed dict silently drops
    one of them along with everything it was supposed to carry.
    """
    reg = _registries(rep)
    blocks = _ops_blocks()

    out = []
    for kind in ("imu", "mag"):
        for sym in reg[kind]:
            if sym not in blocks:
                rep.fail(f"{REGISTRY}: {kind}_registry names {sym}, but no "
                         f"initialiser for it exists under {DRIVER_DIR}/")
                continue
            text, where = blocks[sym]
            body = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
            raw = _scalar(body, "name")
            if not raw or not raw.startswith('"'):
                rep.fail(f"{where}: {sym} has no .name string literal")
                continue

            caps = re.search(r"\.bus_caps\s*=\s*\{(.*?)\}", body, re.S)
            caps = caps.group(1) if caps else ""
            out.append({
                "kind": kind,
                "sym": sym,
                "name": raw.strip('"'),
                "where": where,
                "experimental": _scalar(body, "experimental") == "true",
                # No .bus_caps at all means zero-initialised, which is
                # .spi_capable = false — the AKM compasses and the 9-axis
                # parts whose mag hangs off an auxiliary I2C bus.
                "spi_capable": _scalar(caps, "spi_capable") == "true",
                "spi_mode": _scalar(caps, "spi_mode"),
                "spi_max_hz": _scalar(caps, "spi_max_hz"),
                **{a: _int_array(body, a) for a in ARRAYS},
            })
    rep.expect(out, "driver ops initialisers")
    return out


def by_key(rep):
    """{(kind, name): driver}."""
    return {(d["kind"], d["name"]): d for d in drivers(rep)}


def names(ds, kind):
    """Driver names of one kind, in registry order, deduplicated."""
    seen, out = set(), []
    for d in ds:
        if d["kind"] == kind and d["name"] not in seen:
            seen.add(d["name"])
            out.append(d["name"])
    return out
