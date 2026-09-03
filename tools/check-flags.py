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

Wire v18 added a SECOND word, flags_ext, and the two are checked separately.
They have to be: bit 0 of `flags` and bit 0 of `flags_ext` are different
flags, so folding them into one namespace reports a duplicate bit that is
not one, and would let a real collision inside either word hide behind it.
The Python client keeps them in two classes, so its extractor is scoped to
the class rather than to any indented constant.

Run as `make check-flags`.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT                                  # noqa: E402

# (path, regex capturing (name, bit), description). Each regex is anchored to
# a definition line, so prose mentioning a flag cannot satisfy it.
# (path, regex capturing (name, bit), description, python class to scope to).
# The C patterns exclude EXT_ from the base word with a negative lookahead;
# the Python ones are scoped to a class instead, because its constants carry
# no prefix at all.
WORDS = {
    "flags": [
        ("include/types.h",
         r"^#define\s+FLAG_(?!EXT_)(\w+)\s+\(1u\s*<<\s*(\d+)\)",
         "daemon", "Flags"),
        ("lib/imud_client.h",
         r"^#define\s+IMUD_FLAG_(?!EXT_)(\w+)\s+\(1u\s*<<\s*(\d+)\)",
         "single-header client", "Flags"),
        ("lib/imud.h",
         r"^#define\s+IMUD_FLAG_(?!EXT_)(\w+)\s+\(1u\s*<<\s*(\d+)\)",
         "shared-library header", "Flags"),
        ("lib/imud_client.py",
         r"^\s+(\w+)\s*=\s*1\s*<<\s*(\d+)",
         "python client", "Flags"),
    ],
    "flags_ext": [
        ("include/types.h",
         r"^#define\s+FLAG_EXT_(\w+)\s+\(1u\s*<<\s*(\d+)\)",
         "daemon", "FlagsExt"),
        ("lib/imud_client.h",
         r"^#define\s+IMUD_FLAG_EXT_(\w+)\s+\(1u\s*<<\s*(\d+)\)",
         "single-header client", "FlagsExt"),
        ("lib/imud.h",
         r"^#define\s+IMUD_FLAG_EXT_(\w+)\s+\(1u\s*<<\s*(\d+)\)",
         "shared-library header", "FlagsExt"),
        ("lib/imud_client.py",
         r"^\s+(\w+)\s*=\s*1\s*<<\s*(\d+)",
         "python client", "FlagsExt"),
    ],
}


def class_body(text, name):
    """The source of `class <name>:` up to the next top-level statement.

    The Python client's constants carry no prefix, so an unscoped regex over
    the whole file cannot tell Flags.MAG_VALID from FlagsExt.MAG_ABSENT —
    both are an indented NAME = 1 << 0.
    """
    m = re.search(rf"^class {name}\b.*?:$", text, re.M)
    if not m:
        sys.exit(f"class {name} not found in the python client — renamed?")
    rest = text[m.end():]
    nxt = re.search(r"^\S", rest, re.M)
    return rest[:nxt.start()] if nxt else rest


def flags_in(rel, pattern, cls=None):
    path = os.path.join(ROOT, rel)
    if not os.path.exists(path):
        sys.exit(f"{rel}: not found — has it moved?")
    text = open(path, encoding="utf-8").read()
    if rel.endswith(".py") and cls:
        text = class_body(text, cls)
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
    failures, totals = [], {}
    for word, sources in WORDS.items():
        failures += check_word(word, sources, totals)

    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(f"\n{len(failures)} flag disagreement(s)", file=sys.stderr)
        return 1

    counts = ", ".join(f"{n} {word}" for word, n in totals.items())
    print(f"check-flags: {counts} agree across {len(WORDS['flags'])} definitions")
    return 0


def check_word(word, sources, totals):
    maps = [(rel, desc, flags_in(rel, pat, cls))
            for rel, pat, desc, cls in sources]
    ref_rel, ref_desc, ref = maps[0]
    totals[word] = len(ref)

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

    return [f"[{word}] {f}" for f in failures]


if __name__ == "__main__":
    sys.exit(main())
