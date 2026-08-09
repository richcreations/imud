#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-math-citations.py — docs/math.md must cite code that exists.

math.md is the one document here that no tool could ever generate: an
equation-by-equation account of the MEKF, heave, sea state, compass health and
the offline fits, written so an estimation specialist can audit the filter
against its implementation.  Its value rests entirely on the citations being
true — "as implemented in `mekf_update_mag()`" is worthless if that function
was renamed two releases ago.

It cites bare filenames and function names, never file:line, which is a
deliberate and good choice: nothing rots when code moves within a file.  That
also makes it cheap to check.

  1. every cited `src/x.c` / `include/x.h` path exists;
  2. every cited `foo()` is defined or declared somewhere in the tree.

Run as `make check-math-citations`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, must_read, Report                    # noqa: E402

DOC = "docs/math.md"
CODE_DIRS = ("src", "include", "lib")

# Names written as foo() that are not imud functions — maths and libc, which
# the prose cites the same way because that is how they read in a formula.
NOT_OURS = {
    "sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sqrt", "exp",
    "log", "log2", "log10", "pow", "fabs", "fmod", "floor", "ceil", "round",
    "expm1", "log1p", "hypot", "sinf", "cosf", "sqrtf", "fabsf", "expf",
    "atan2f", "asinf", "tanhf", "erf", "min", "max", "abs", "sign", "diag",
    "trace", "det", "exp_map", "vee", "hat", "skew",
}


def source_index():
    """Every function name defined or declared under the code dirs."""
    names = set()
    for d in CODE_DIRS:
        base = os.path.join(ROOT, d)
        if not os.path.isdir(base):
            continue
        for dirpath, _, filenames in os.walk(base):
            for fn in filenames:
                if not fn.endswith((".c", ".h")):
                    continue
                text = open(os.path.join(dirpath, fn), encoding="utf-8",
                            errors="replace").read()
                # A definition or prototype: name( at a plausible position.
                names.update(re.findall(r"\b([a-z_][a-z0-9_]*)\s*\(", text))
    return names


def main():
    rep = Report("check-math-citations")
    text = must_read(DOC, "the mathematical reference")

    # ── 1. cited files exist ─────────────────────────────────────────────────
    files = sorted(set(re.findall(r"`((?:src|include|lib)/[\w/]+\.[ch])`", text)))
    rep.expect(files, "cited source files")
    for rel in files:
        lineno = text[:text.index(f"`{rel}`")].count("\n") + 1
        rep.check(os.path.exists(os.path.join(ROOT, rel)),
                  f"{DOC}:{lineno}: cites `{rel}`, which does not exist")

    # ── 2. cited functions exist ─────────────────────────────────────────────
    defined = source_index()
    rep.expect(defined, "function names in the source tree")

    cited = sorted(set(re.findall(r"`([a-z_][a-z0-9_]*)\(\)`", text)))
    rep.expect(cited, "cited function names")
    for name in cited:
        if name in NOT_OURS:
            continue
        lineno = text[:text.index(f"`{name}()`")].count("\n") + 1
        rep.check(name in defined,
                  f"{DOC}:{lineno}: cites `{name}()`, which is not defined "
                  f"anywhere under {'/, '.join(CODE_DIRS)}/")

    return rep.finish(f"{len(files)} files and {len(cited)} functions cited")


if __name__ == "__main__":
    sys.exit(main())
