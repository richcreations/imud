#!/usr/bin/env python3
"""
coverage-gaps.py — say which sources are missing from the coverage report, and
why, because "missing" currently means two very different things.

`make coverage` runs `make test`, so a translation unit that no test binary
links emits no .gcno at all: lcov does not list it at 0%, it does not list it.
That is the signal worth chasing — which files have never been executed.

But a file compiled into MORE THAN ONE test binary goes missing too, for an
unrelated reason. The test rules compile sources directly into each binary, so
src/cal.c becomes both src/cal.gcno (the shared object that test_concurrency
links) and test_cal-cal.gcno (test_cal's own copy). Two notes files, two
stamps, one source — lcov calls that a mismatch and `--ignore-errors mismatch`
drops it silently. lib/libimud.c, src/bridge.c, src/cal.c and src/cal_capture.c
disappeared exactly this way: 1,072 lines that ARE covered by passing suites and
were being read as untested.

Ground truth for telling them apart is the notes files themselves: no .gcno
anywhere means nothing compiled it; one or more means it was compiled and lcov
chose not to attribute it.

Usage: tools/coverage-gaps.py [coverage.info]   (run from the repo root)
"""

import glob
import os
import sys


def sources():
    out = []
    for pat in ("src/*.c", "src/drivers/*.c", "lib/*.c"):
        out.extend(sorted(glob.glob(pat)))
    return out


def reported(info_path):
    """Every source file lcov attributed, as repo-relative paths."""
    seen = set()
    if not os.path.exists(info_path):
        return seen
    root = os.path.abspath(".") + os.sep
    with open(info_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("SF:"):
                p = line[3:].strip()
                if p.startswith(root):
                    p = p[len(root):]
                seen.add(p)
    return seen


def notes_for(src):
    """.gcno files that belong to `src`, under any of the naming schemes."""
    base = os.path.splitext(os.path.basename(src))[0]
    d = os.path.dirname(src)
    hits = glob.glob(os.path.join(d, base + ".gcno"))
    hits += glob.glob("*-" + base + ".gcno")
    # The Makefile's src/%.entry.o rule -- a daemon's real main() compiled as
    # <base>_entry for the end-to-end suites -- writes src/<base>.entry.gcno.
    # Without this pattern an .entry.o source that lost lcov attribution would
    # be reported as "in NO test binary" rather than as a stamp mismatch, which
    # is the one answer this tool must not get wrong.
    hits += glob.glob(os.path.join(d, base + ".entry.gcno"))
    return hits


def main():
    info = sys.argv[1] if len(sys.argv) > 1 else "coverage.info"
    have = reported(info)
    if not have:
        print("coverage-gaps: no %s to read — run `make coverage` first" % info)
        return 0

    dark, unattributed = [], []
    for src in sources():
        if src in have:
            continue
        n = len(notes_for(src))
        lines = sum(1 for _ in open(src, encoding="utf-8", errors="replace"))
        (dark if n == 0 else unattributed).append((src, lines, n))

    print()
    print("── coverage gaps ──────────────────────────────────────────────────")

    if dark:
        total = sum(l for _, l, _ in dark)
        print("\nIn NO test binary — never executed: "
              "%d files, %d lines" % (len(dark), total))
        for src, lines, _ in dark:
            print("    %-28s %5d lines" % (src, lines))
    else:
        print("\nIn NO test binary: none — every source is linked by a suite.")

    if unattributed:
        total = sum(l for _, l, _ in unattributed)
        print("\nCovered but NOT in the report — lcov stamp mismatch, not a "
              "test gap: %d files, %d lines" % (len(unattributed), total))
        for src, lines, n in unattributed:
            print("    %-28s %5d lines  (%d .gcno: compiled into %d binaries)"
                  % (src, lines, n, n))
        print("\n    These are exercised by passing suites. The report "
              "understates itself by\n    the line count above; do not read "
              "their absence as untested code.")

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
