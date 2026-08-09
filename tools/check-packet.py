#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-packet.py — spec.md's wire table must be the wire.

spec.md §8 prints the 276-byte packet as an offset/size/type/field table.  It
is the document third-party integrators read to write a decoder, and its
version row said

    4      2      uint16    version           = 14  (1.4; encoded as major*10+minor)

while include/types.h said IMUD_VERSION 17 — three wire revisions stale, with
a decoding rule (major*10+minor) that had stopped being true.  spec.md said
"v17" four times elsewhere in the same file.  CI compared the version across
types.h, imud_client.h and imud_client.py; it never read spec.md's table.

Offsets in a packed struct are pure cumulative sizeof, so they are checked
exactly rather than trusted: a field inserted mid-struct silently shifts every
row beneath it, which is the one error a human proof-reader will not catch.

Run as `make check-packet`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import must_read, Report                          # noqa: E402

HDR = "include/types.h"
SPEC = "spec.md"

# C type -> (size, the name spec.md uses for it)
CTYPE = {
    "uint8_t":  (1, "uint8"),
    "int8_t":   (1, "int8"),
    "uint16_t": (2, "uint16"),
    "int16_t":  (2, "int16"),
    "uint32_t": (4, "uint32"),
    "int32_t":  (4, "int32"),
    "uint64_t": (8, "uint64"),
    "int64_t":  (8, "int64"),
    "float":    (4, "float32"),
    "double":   (8, "float64"),
    "char":     (1, "char"),
}


def struct_layout(rep):
    """[(offset, size, spec_type, name)] for imu_packet_t, in declaration order."""
    src = must_read(HDR, "the wire packet definition")

    m = re.search(
        r"typedef struct\s+__attribute__\(\(packed\)\)\s*\{(.*?)\}\s*imu_packet_t;",
        src, re.S)
    if not m:
        sys.exit(f"{HDR}: cannot find the packed imu_packet_t — has it moved?")

    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)     # drop comments
    fields, offset = [], 0
    for line in body.split("\n"):
        fm = re.match(r"\s*(\w+)\s+(\w+)\s*(?:\[(\d+)\])?\s*;", line)
        if not fm:
            continue
        ctype, name, count = fm.group(1), fm.group(2), fm.group(3)
        if ctype not in CTYPE:
            rep.fail(f"{HDR}: field '{name}' has type '{ctype}', which "
                     f"check-packet.py cannot size — add it to CTYPE")
            continue
        size, spec_type = CTYPE[ctype]
        n = int(count) if count else 1
        # An array is one row, named the way spec.md names it (`cov[9]`), not
        # nine rows.  The table documents it as a unit — "144  9×4  float32
        # cov[9]" — and splitting it here would report a correct table as
        # having nine missing fields.
        fields.append((offset, size * n, spec_type,
                       f"{name}[{n}]" if count else name))
        offset += size * n
    rep.expect(fields, "imu_packet_t fields")
    return fields, offset


def declared_size(rep):
    """The size the _Static_assert pins, which is the contract everyone codes to."""
    src = must_read(HDR)
    m = re.search(r"_Static_assert\(sizeof\(imu_packet_t\)\s*==\s*(\d+)", src)
    if not m:
        rep.fail(f"{HDR}: no _Static_assert on sizeof(imu_packet_t)")
        return None
    return int(m.group(1))


def wire_version(rep):
    src = must_read(HDR)
    m = re.search(r"^#define\s+IMUD_VERSION\s+(\d+)", src, re.M)
    if not m:
        rep.fail(f"{HDR}: cannot find #define IMUD_VERSION")
        return None
    return int(m.group(1))


def spec_rows(rep):
    """[(lineno, offset, size, type, field)] from spec.md's Packet Layout block."""
    text = must_read(SPEC)
    m = re.search(r"###\s+Packet Layout.*?```text\n(.*?)```", text, re.S)
    if not m:
        sys.exit(f"{SPEC}: cannot find the '### Packet Layout' text block")
    start = text[:m.start(1)].count("\n") + 1

    rows = []
    for i, line in enumerate(m.group(1).split("\n")):
        # The Bytes column is an integer, or `9×4` for an array — the table
        # spells out element count × element size so a reader can see both.
        rm = re.match(r"\s*(\d+)\s+(\d+)(?:\s*[×x]\s*(\d+))?\s+(\w+)\s+(\w+(?:\[\d+\])?)",
                      line)
        if rm:
            size = int(rm.group(2))
            if rm.group(3):
                size *= int(rm.group(3))
            rows.append((start + i, int(rm.group(1)), size,
                         rm.group(4), rm.group(5), line))
    rep.expect(rows, "spec.md packet table rows")
    return rows


def main():
    rep = Report("check-packet")

    fields, total = struct_layout(rep)
    rows = spec_rows(rep)
    version = wire_version(rep)
    pinned = declared_size(rep)

    if pinned is not None:
        rep.check(total == pinned,
                  f"{HDR}: fields sum to {total} bytes but _Static_assert pins "
                  f"{pinned} — check-packet.py is mis-sizing something")

    by_name = {f[3]: f for f in fields}
    seen = set()

    for lineno, off, size, ctype, name, raw in rows:
        seen.add(name)
        f = by_name.get(name)
        if f is None:
            rep.check(False,
                      f"{SPEC}:{lineno}: documents field '{name}', which is not "
                      f"in imu_packet_t")
            continue
        want_off, want_size, want_type, _ = f
        rep.check(off == want_off,
                  f"{SPEC}:{lineno}: '{name}' documented at offset {off}, "
                  f"actually at {want_off}")
        rep.check(size == want_size,
                  f"{SPEC}:{lineno}: '{name}' documented as {size} bytes, "
                  f"actually {want_size}")
        rep.check(ctype == want_type,
                  f"{SPEC}:{lineno}: '{name}' documented as {ctype}, "
                  f"actually {want_type}")

        # The version row carries its value in the Notes column.
        if name == "version" and version is not None:
            vm = re.search(r"=\s*(\d+)", raw)
            if vm:
                rep.check(int(vm.group(1)) == version,
                          f"{SPEC}:{lineno}: says the wire version is "
                          f"{vm.group(1)}, but {HDR} defines IMUD_VERSION "
                          f"{version}")

    for _, _, _, name in fields:
        rep.check(name in seen,
                  f"{SPEC}: imu_packet_t has field '{name}' but the Packet "
                  f"Layout table does not document it")

    return rep.finish(f"{len(fields)} fields, {total} bytes, wire v{version}")


if __name__ == "__main__":
    sys.exit(main())
