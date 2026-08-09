#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-drivers.py — the driver tables and the rate lists must be the drivers.

man/man5/imud.conf.5 told operators

    ISM330DHCX supports: 12, 26, 52, 104, 208, 416, 833, 1660.

while ism330dhcx.c had `.supported_odr_hz = { ..., 1660, 3332, 6664, 0 }`.
config/imud.conf and docs/manual.md both listed all ten.  So a reader of the
man page believed 1660 was the ceiling, and a reader of the manual did not —
for a setting whose whole point is picking a rate the chip can produce.

Every fact here is a literal in a `*_ops` initialiser, which is why this
checks rather than trusts:

  - .name             the driver a config file names
  - .experimental     whether the docs must say so
  - .supported_odr_hz / _accel_g / _gyro_dps   the rate and range lists

Adding a driver currently means hand-editing six files.  This does not remove
that work — S5b's generator does — but it makes forgetting one loud.

Run as `make check-drivers`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, read, must_read, Report               # noqa: E402

REGISTRY = "src/drivers.c"
DRIVER_DIR = "src/drivers"

# Surfaces that must name every registered driver.
#
# docs/datasheets.md is deliberately absent: it is a table of parts with
# manufacturer links, and it says in its own text that it covers parts
# researched *without* drivers too.  Its membership is a superset by design.
NAME_SURFACES = [
    "docs/manual.md",
    "man/man5/imud.conf.5",
]

ARRAYS = ("supported_odr_hz", "supported_accel_g", "supported_gyro_dps")


def registered(rep):
    """{'imu': [ops symbol...], 'mag': [...]} from the two registries."""
    src = must_read(REGISTRY, "the driver registries")
    out = {}
    for kind in ("imu", "mag"):
        m = re.search(kind + r"_registry\[\]\s*=\s*\{(.*?)\n\};", src, re.S)
        if not m:
            sys.exit(f"{REGISTRY}: cannot find {kind}_registry[]")
        out[kind] = re.findall(r"&(\w+_ops)\b", m.group(1))
        rep.expect(out[kind], f"{kind}_registry entries")
    return out


def ops_blocks():
    """{ops symbol: initialiser text} across src/drivers/*.c."""
    blocks = {}
    d = os.path.join(ROOT, DRIVER_DIR)
    for name in sorted(os.listdir(d)):
        if not name.endswith(".c"):
            continue
        src = open(os.path.join(d, name), encoding="utf-8").read()
        for m in re.finditer(r"\b(\w+_ops)\s*=\s*\{(.*?)\n\};", src, re.S):
            blocks[m.group(1)] = (m.group(2), f"{DRIVER_DIR}/{name}")
    return blocks


def field(text, name):
    """The literal assigned to .<name>, or None."""
    m = re.search(r"\." + re.escape(name) + r"\s*=\s*([^,\n]+)", text)
    return m.group(1).strip() if m else None


def int_array(text, name):
    """The 0-terminated .<name> array as a list of ints, or None."""
    m = re.search(r"\." + re.escape(name) + r"\s*=\s*\{(.*?)\}", text, re.S)
    if not m:
        return None
    nums = [int(n) for n in re.findall(r"\b(\d+)\b", m.group(1))]
    while nums and nums[-1] == 0:            # drop the terminator
        nums.pop()
    return nums


def main():
    rep = Report("check-drivers")

    reg = registered(rep)
    blocks = ops_blocks()

    drivers = {}                              # name -> {arrays, experimental}
    for kind, syms in reg.items():
        for sym in syms:
            if sym not in blocks:
                rep.fail(f"{REGISTRY}: {kind}_registry names {sym}, but no "
                         f"initialiser for it exists under {DRIVER_DIR}/")
                continue
            text, where = blocks[sym]
            raw = field(text, "name")
            if not raw or not raw.startswith('"'):
                rep.fail(f"{where}: {sym} has no .name string literal")
                continue
            name = raw.strip('"')
            # Keyed by (kind, name), not name: sim_imu_ops and sim_mag_ops are
            # both .name = "sim", and a name-keyed dict silently drops one of
            # them along with whatever it was supposed to assert.
            drivers[(kind, name)] = {
                "kind": kind,
                "name": name,
                "where": where,
                "experimental": field(text, "experimental") == "true",
                **{a: int_array(text, a) for a in ARRAYS},
            }

    rep.expect(drivers, "driver ops initialisers")

    names = sorted({d["name"] for d in drivers.values()})

    # ── 1. every registered driver is named on every naming surface ──────────
    for rel in NAME_SURFACES:
        text = read(rel)
        if text is None:
            rep.fail(f"missing file {rel}")
            continue
        for name in names:
            rep.check(re.search(r"\b" + re.escape(name) + r"\b", text) is not None,
                      f"{rel}: driver '{name}' is registered but not named here")

    # ── 2. experimental drivers are flagged as such ──────────────────────────
    # Match the driver TABLE row — `| \`ak09916\` | ... |` — not merely a line
    # mentioning the name.  ak09916 and ak8963 are both named in the [mag] bus
    # description hundreds of lines above their rows, and taking the first
    # match reported two correctly-flagged drivers as unflagged.
    manual = read("docs/manual.md") or ""
    for d in sorted(drivers.values(), key=lambda x: x["name"]):
        if not d["experimental"]:
            continue
        name = d["name"]
        row = next((l for l in manual.split("\n")
                    if re.match(r"\s*\|\s*`" + re.escape(name) + r"`\s*\|", l)),
                   None)
        rep.check(row is not None and "xperimental" in row,
                  f"docs/manual.md: '{name}' is .experimental = true in "
                  f"{d['where']}, but its driver-table row does not say so"
                  if row is not None else
                  f"docs/manual.md: '{name}' is registered but has no row in "
                  f"the driver table")

    # ── 3. documented rate / range lists match the ops arrays ────────────────
    # Two phrasings, both in use, and both must be read or half the lists go
    # unchecked:
    #
    #   odr_hz    "ISM330DHCX supports: 12, 26, 52, ..."
    #   accel_g   "ISM330DHCX: 2, 4, 8, 16."
    #
    # The bare-colon form needs two guards.  At least two comma-separated
    # decimals, and no 0x anywhere in the match — otherwise the i2c_addr entry
    # ("ISM330DHCX: 0x6B (SA0 high) or 0x6A") parses as a rate list of 6 and 6.
    #
    # Compare against whichever array the claim is closest to, so the message
    # can name the field that actually disagrees.
    claim = re.compile(r"\b([A-Za-z][\w-]{3,})\s*(?:supports)?:\s*"
                       r"((?:`?\d+`?\s*,\s*)+`?\d+`?)")
    for rel in NAME_SURFACES + ["config/imud.conf"]:
        text = read(rel)
        if text is None:
            continue
        for m in claim.finditer(text):
            if "0x" in m.group(0):
                continue
            part, listing = m.group(1).lower(), m.group(2)
            d = next((v for v in drivers.values() if v["name"] == part), None)
            if d is None:
                continue
            claimed = [int(n) for n in re.findall(r"\d+", listing)]
            lineno = text[:m.start()].count("\n") + 1

            candidates = [(a, d[a]) for a in ARRAYS if d[a]]
            if not candidates:
                continue
            best, values = max(
                candidates,
                key=lambda kv: len(set(claimed) & set(kv[1])))
            rep.check(claimed == values,
                      f"{rel}:{lineno}: says {part} supports "
                      f"{', '.join(str(n) for n in claimed)} — "
                      f"{d['where']} has .{best} = "
                      f"{', '.join(str(n) for n in values)}")

    return rep.finish(f"{len(drivers)} drivers "
                      f"({len(reg['imu'])} imu, {len(reg['mag'])} mag)")


if __name__ == "__main__":
    sys.exit(main())
