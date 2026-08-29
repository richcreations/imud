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

It cites bare filenames, function names AND `file.c:123` line references.  The
line references are the ones that rot: they were written correct and 25 of 26
had drifted by 1.9.0, by as much as 848 lines, while this checker passed —
because it only looked at names.  So all three are checked now:

  1. every cited `src/x.c` / `include/x.h` path exists;
  2. every cited `foo()` is defined or declared somewhere in the tree;
  3. every cited `file.c:123` lands inside the definition it is citing.

Check 3 resolves the line against the C definition that encloses it, not
against an exact line, so a citation stays valid while code moves *within* a
function and fails when the function itself moves.  That is the loosest rule
that still catches the drift, which matters: a checker that fires on every
edit inside a function is one people learn to skip.

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
                names.update(re.findall(r"\b([A-Za-z_]\w*)\s*\(", text))
    return names


# ── C top-level definitions, for check 3 ──────────────────────────────────────
#
# Deliberately a small hand-rolled scan rather than a parser dependency: the
# tree is plain C11 in one consistent style — return type and name on one line
# at column 0, body brace and its closing brace at column 0 — and check 3 only
# needs "which definition encloses line N", not a syntax tree.

DEF_LINE = re.compile(r'^(?!\s)(?!#)(?!//)(?!/\*)(?!\*)'
                      r'(?:static\s+|inline\s+|const\s+|extern\s+)*'
                      r'[A-Za-z_][\w \t\*]*?'
                      r'\b([A-Za-z_]\w*)\s*\(')
MACRO_LINE = re.compile(r'^#\s*define\s+([A-Za-z_]\w*)')
TYPEDEF_END = re.compile(r'^\}\s*([A-Za-z_]\w*)\s*;')


def definitions(path):
    """name -> (first_line, last_line), 1-indexed inclusive, for one C file."""
    lines = open(path, encoding="utf-8", errors="replace").read().split("\n")
    out, i = {}, 0
    while i < len(lines):
        line = lines[i]

        m = MACRO_LINE.match(line)
        if m:                                   # #define, plus continuations
            j = i
            while j < len(lines) and lines[j].rstrip().endswith("\\"):
                j += 1
            out.setdefault(m.group(1), (i + 1, j + 1))
            i = j + 1
            continue

        m = TYPEDEF_END.match(line)             # typedef struct { … } name;
        if m:
            j = i
            while j >= 0 and not lines[j].startswith("typedef"):
                j -= 1
            out.setdefault(m.group(1), ((j if j >= 0 else i) + 1, i + 1))
            i += 1
            continue

        m = DEF_LINE.match(line)
        if m and not line.rstrip().endswith(";"):
            j = i                               # walk to the body brace
            while j < len(lines) and not lines[j].startswith("{"):
                if lines[j].rstrip().endswith(";"):
                    break                       # a prototype, not a body
                j += 1
            if j < len(lines) and lines[j].startswith("{"):
                k = j
                while k < len(lines) and lines[k] != "}":
                    k += 1
                out.setdefault(m.group(1), (i + 1, min(k, len(lines) - 1) + 1))
                i = k + 1
                continue
        i += 1
    return out


def resolve(base):
    """`fusion.c` -> the tracked path that holds it, or None."""
    for d in CODE_DIRS:
        cand = os.path.join(ROOT, d, base)
        if os.path.exists(cand):
            return cand
    for d in CODE_DIRS:                         # src/drivers/ and friends
        for dirpath, _, filenames in os.walk(os.path.join(ROOT, d)):
            if base in filenames:
                return os.path.join(dirpath, base)
    return None


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

    cited = sorted(set(re.findall(r"`([A-Za-z_]\w*)\(\)`", text)))
    rep.expect(cited, "cited function names")
    for name in cited:
        if name in NOT_OURS:
            continue
        lineno = text[:text.index(f"`{name}()`")].count("\n") + 1
        rep.check(name in defined,
                  f"{DOC}:{lineno}: cites `{name}()`, which is not defined "
                  f"anywhere under {'/, '.join(CODE_DIRS)}/")

    # ── 3. cited file:line lands inside the definition it names ──────────────
    #
    # The citation usually names its symbol right beside the line — either
    # `foo()` (`x.c:12`) or (`foo`, `x.c:12`) — and sometimes on the line
    # before, so the lookback spans both.  When a symbol is named, the line
    # must fall inside it.  When none is (a citation pointing at a comment or
    # an expression), the line must at least fall inside SOME definition,
    # which still catches a citation that has drifted off the end or into a
    # gap between functions.
    # `x.c:12`, `x.c:12`–`34`, and the `:12` shorthand that continues
    # whichever file was named last — all three appear in the document.
    cite = re.compile(r"`(?:([a-z0-9_]+\.[ch]))?:(\d+)`(?:–`(\d+)`)?")
    name_in = re.compile(r"`([A-Za-z_]\w*)(?:\(\))?`")
    doc = text.split("\n")
    spans, n_lines, carried = {}, 0, None

    for i, line in enumerate(doc, 1):
        for m in cite.finditer(line):
            base, first = m.group(1), int(m.group(2))
            last = int(m.group(3)) if m.group(3) else None
            n_lines += 1

            if base:
                carried = base
            elif not rep.check(carried, f"{DOC}:{i}: cites `:{first}` before "
                                        f"any file has been named"):
                continue
            base = carried

            src = resolve(base)
            if not rep.check(src, f"{DOC}:{i}: cites `{base}`, which is not "
                                  f"under {'/, '.join(CODE_DIRS)}/"):
                continue
            if base not in spans:
                spans[base] = definitions(src)
            defs = spans[base]

            total = len(open(src, encoding="utf-8",
                             errors="replace").read().split("\n"))
            if not rep.check(first <= total,
                             f"{DOC}:{i}: cites `{base}:{first}`, but that "
                             f"file has {total} lines"):
                continue

            lookback = (doc[i - 2] + " " if i >= 2 else "") + line[:m.start()]
            named = next((c for c in reversed(name_in.findall(lookback))
                          if c in defs), None)

            if named:
                lo, hi = defs[named]
                rep.check(lo <= first <= hi,
                          f"{DOC}:{i}: cites `{base}:{first}` for "
                          f"`{named}`, which is at {base}:{lo}"
                          + (f"–{hi}" if hi != lo else ""))
            else:
                inside = any(lo <= first <= hi for lo, hi in defs.values())
                rep.check(inside,
                          f"{DOC}:{i}: cites `{base}:{first}`, which is not "
                          f"inside any definition in {base}")

            if last is not None:
                rep.check(first < last <= total,
                          f"{DOC}:{i}: cites the range `{base}:{first}`–"
                          f"`{last}`, which does not run forwards inside a "
                          f"file of {total} lines")

    return rep.finish(f"{len(files)} files, {len(cited)} functions and "
                      f"{n_lines} file:line citations")


if __name__ == "__main__":
    sys.exit(main())
