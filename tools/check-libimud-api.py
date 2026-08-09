#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-libimud-api.py — the exported API, the header and the docs must agree.

libimud is eleven functions, and its documentation risk is not wording but
*completeness*: a symbol reaching libimud.map without a doc entry, a prototype
in imud.h that was never exported, or a new function landing in an existing
version node instead of a new one.  None of those is visible by reading; all
three are set differences.

Three sources:
  lib/libimud.map        what the linker exports, per version node
  lib/imud.h             what an application can call
  man/man3/libimud.3     the reference page
  docs/libimud/spec.md   the API/ABI reference

imud_wire is the reason this is a checker and not a generator: it is exported,
prototyped and in the spec, but deliberately absent from the man page's
SYNOPSIS because it is the wire-pinned opt-in that the ABI-stable API exists
to avoid.  A generator would need to be told about that asymmetry anyway; here
the allowlist *is* the written record of it.

Run as `make check-libimud-api`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import read, must_read, Report                    # noqa: E402

MAP = "lib/libimud.map"
HDR = "lib/imud.h"

SURFACES = ["man/man3/libimud.3", "docs/libimud/spec.md"]

# symbol -> (surface it may be missing from, why)
ALLOW = {
    ("imud_wire", "man/man3/libimud.3"):
        "wire-pinned opt-in: exported and in the spec, deliberately kept out "
        "of the man page SYNOPSIS so the ABI-stable API is what readers find",
}


def exported(rep):
    """{symbol: version node} from the linker script."""
    text = must_read(MAP, "the symbol version script")
    syms, node = {}, None
    for line in text.split("\n"):
        m = re.match(r"\s*(IMUD_\d+)\s*\{", line)
        if m:
            node = m.group(1)
            continue
        if re.match(r"\s*local:", line):
            node = None
            continue
        m = re.match(r"\s*(imud_\w+)\s*;", line)
        if m and node:
            syms[m.group(1)] = node
    rep.expect(syms, "exported symbols")
    return syms


def prototyped(rep):
    """Public function names declared in the installed header."""
    text = must_read(HDR, "the public header")
    names = set(re.findall(r"^\s*(?:const\s+)?[\w ]+\**\s*(imud_\w+)\s*\(",
                           text, re.M))
    rep.expect(names, "header prototypes")
    return names


def main():
    rep = Report("check-libimud-api")

    syms = exported(rep)
    protos = prototyped(rep)

    # ── header and linker script describe the same API ───────────────────────
    for name in sorted(set(syms) - protos):
        rep.check(False,
                  f"{MAP}: exports '{name}', which {HDR} does not declare — "
                  f"an application cannot call it")
    for name in sorted(protos - set(syms)):
        rep.check(False,
                  f"{HDR}: declares '{name}', which {MAP} does not export — "
                  f"it will not link")

    # ── every symbol reaches every reference surface ─────────────────────────
    for rel in SURFACES:
        text = read(rel)
        if text is None:
            rep.fail(f"missing file {rel}")
            continue
        for name in sorted(syms):
            if (name, rel) in ALLOW:
                continue
            rep.check(name in text,
                      f"{rel}: '{name}' is exported ({syms[name]}) but not "
                      f"documented here")

    # ── the allowlist may not outlive its reason ─────────────────────────────
    for (name, rel), why in sorted(ALLOW.items()):
        rep.check(name in syms,
                  f"{MAP}: no longer exports '{name}', but ALLOW still "
                  f"excuses it from {rel} — drop the entry ({why})")

    nodes = sorted(set(syms.values()))
    return rep.finish(f"{len(syms)} symbols across {len(nodes)} version "
                      f"nodes ({', '.join(nodes)}), {len(SURFACES)} surfaces")


if __name__ == "__main__":
    sys.exit(main())
