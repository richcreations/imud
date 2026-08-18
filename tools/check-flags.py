#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-flags.py — the packet flag bits must agree across all four copies.

The flags word is wire format, and four independent files define it: the
daemon's own include/types.h, both standalone client headers (which are
standalone by design, so they cannot include the first), and the Python
client. CI already refuses a disagreement on IMUD_VERSION for exactly this
reason — a consumer reading a stale copy misreads every packet — but nothing
compared the flag definitions, so those four could drift silently.

A drifted flag is worse than a drifted version, because it fails quietly: the
packet still parses, the wrong bit is simply read under the right name. That
is indistinguishable from the sensor reporting something it never reported.

Names are compared with the IMUD_ prefix stripped, since include/types.h uses
FLAG_* and everything else uses IMUD_FLAG_*.

Run as `make check-flags`.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT                                  # noqa: E402

# (path, regex capturing (name, bit), description). Each regex is anchored to
# a definition line, so prose mentioning a flag cannot satisfy it.
SOURCES = [
    ("include/types.h",
     r"^#define\s+FLAG_(\w+)\s+\(1u\s*<<\s*(\d+)\)",
     "daemon"),
    ("lib/imud_client.h",
     r"^#define\s+IMUD_FLAG_(\w+)\s+\(1u\s*<<\s*(\d+)\)",
     "single-header client"),
    ("lib/imud.h",
     r"^#define\s+IMUD_FLAG_(\w+)\s+\(1u\s*<<\s*(\d+)\)",
     "shared-library header"),
    ("lib/imud_client.py",
     r"^\s+(\w+)\s*=\s*1\s*<<\s*(\d+)",
     "python client"),
]


def flags_in(rel, pattern):
    path = os.path.join(ROOT, rel)
    if not os.path.exists(path):
        sys.exit(f"{rel}: not found — has it moved?")
    text = open(path, encoding="utf-8").read()
    out = {}
    for m in re.finditer(pattern, text, re.M):
        out[m.group(1)] = int(m.group(2))
    # An extractor that finds nothing must fail, not pass vacuously: a renamed
    # macro or a reformatted define would otherwise turn drift into a green run.
    if not out:
        sys.exit(f"{rel}: no flag definitions matched — "
                 f"has the definition style changed?")
    return out


def main():
    maps = [(rel, desc, flags_in(rel, pat)) for rel, pat, desc in SOURCES]
    ref_rel, ref_desc, ref = maps[0]

    failures = []
    for rel, desc, got in maps[1:]:
        for name, bit in sorted(ref.items()):
            if name not in got:
                failures.append(f"{rel} ({desc}) is missing {name} "
                                f"(bit {bit} in {ref_rel})")
            elif got[name] != bit:
                failures.append(f"{rel} ({desc}) has {name} at bit {got[name]}, "
                                f"but {ref_rel} has it at bit {bit}")
        for name, bit in sorted(got.items()):
            if name not in ref:
                failures.append(f"{rel} ({desc}) defines {name} (bit {bit}), "
                                f"which {ref_rel} does not")

    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(f"\n{len(failures)} flag disagreement(s)", file=sys.stderr)
        return 1

    print(f"check-flags: {len(ref)} flags agree across "
          f"{len(maps)} definitions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
