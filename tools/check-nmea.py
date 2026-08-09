#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-nmea.py — the documented NMEA output must be the NMEA output.

spec.md said "Four sentences are emitted per update cycle (five when magnetic
declination is known)" while nmea_encode() emitted five and six, and the list
of sentences printed directly beneath that claim already had six entries.  The
prose disagreed with the list one line below it, in the file that defines the
wire format, and nothing noticed.

The emit set is a call *sequence* with a conditional, not a table, so this
checks rather than generates:

  1. every sentence nmea_encode() can emit is named on every doc surface;
  2. no surface advertises a sentence that is no longer emitted;
  3. any "N sentences" claim matches the real unconditional count.

Run as `make check-nmea`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, read                                # noqa: E402

SRC = os.path.join(ROOT, "src", "nmea.c")

# Where the sentence set is described to a reader.
#
# `full` means the surface claims to present the whole output set, so a
# sentence missing from it is a documentation gap.  config/imud.conf is not
# such a surface: it is a settings template whose [position] comment mentions
# only the sentences declination affects, and demanding it enumerate all six
# would be demanding it stop being a config file.  The phantom-sentence and
# count checks still apply there — those catch a template describing output
# the daemon no longer produces.
SURFACES = {
    "spec.md":              {"full": True},
    "docs/manual.md":       {"full": True},
    "man/man5/imud.conf.5": {"full": True},
    "config/imud.conf":     {"full": False},
}

WORD = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
        "seven": 7, "eight": 8, "nine": 9, "ten": 10}


def emitted():
    """[(sentence, gated)] in emission order, from nmea_encode()'s body.

    `gated` marks a sentence emitted only inside a conditional — today that
    is $HCHDT under FLAG_DECLINATION_VALID, which is the difference between
    the "five" and the "six".
    """
    src = open(SRC, encoding="utf-8").read()

    m = re.search(r"^int nmea_encode\([^)]*\)\s*\{", src, re.M)
    if not m:
        sys.exit(f"{SRC}: cannot find nmea_encode() — has it been renamed?")
    body = src[m.end():]
    body = body[:body.index("\n}")]

    # Which builder writes which sentence: the literal in its own body.
    literal = {}
    for fm in re.finditer(r"^static int (build_\w+)\(", src, re.M):
        name = fm.group(1)
        tail = src[fm.end():]
        end = tail.index("\n}") if "\n}" in tail else len(tail)
        lm = re.search(r'"\$?([A-Z]{5})', tail[:end])
        if lm:
            literal[name] = "$" + lm.group(1)

    out, depth_gate = [], 0
    for line in body.split("\n"):
        # Track whether we are inside an `if (...)` block within the function.
        stripped = line.strip()
        if re.match(r"^if\s*\(", stripped) and stripped.endswith("{"):
            depth_gate += 1
        elif stripped == "}" and depth_gate:
            depth_gate -= 1
        for call in re.findall(r"\b(build_\w+)\(", line):
            if call in literal:
                out.append((literal[call], depth_gate > 0))
    if not out:
        sys.exit(f"{SRC}: no build_*() calls found in nmea_encode() — "
                 f"the extractor is not reading the emit sequence")
    return out


def main():
    seq = emitted()
    always = [s for s, gated in seq if not gated]
    gated = [s for s, gated in seq if gated]
    every = [s for s, _ in seq]

    failures, checked = [], 0

    for rel, opts in SURFACES.items():
        text = read(rel)
        if text is None:
            failures.append(f"missing file {rel}")
            continue

        if opts["full"]:
            for sentence in every:
                checked += 1
                if sentence not in text:
                    failures.append(
                        f"{rel}: {sentence} is emitted but not documented")

        # Anything written as a $-prefixed sentence that we do not emit.
        for claimed in sorted(set(re.findall(r"\$([A-Z]{5})\b", text))):
            checked += 1
            if "$" + claimed not in every:
                failures.append(
                    f"{rel}: documents ${claimed}, which nmea_encode() "
                    f"does not emit")

        # "N sentences" must be the unconditional count.
        for m in re.finditer(r"\b([A-Za-z]+|\d+)\s+(?:NMEA\s+)?sentences\b",
                             text, re.I):
            token = m.group(1).lower()
            n = WORD.get(token) or (int(token) if token.isdigit() else None)
            if n is None:
                continue
            checked += 1
            if n != len(always):
                lineno = text[:m.start()].count("\n") + 1
                failures.append(
                    f"{rel}:{lineno}: claims {m.group(0)!r}, but nmea_encode() "
                    f"emits {len(always)} unconditionally "
                    f"({len(always) + len(gated)} with declination)")

    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(f"\n{len(failures)} problem(s); emitted set is "
              f"{' '.join(always)} (+{' '.join(gated)} when gated)",
              file=sys.stderr)
        return 1

    print(f"check-nmea: {len(always)} sentences (+{len(gated)} conditional), "
          f"{checked} assertions across {len(SURFACES)} surfaces")
    return 0


if __name__ == "__main__":
    sys.exit(main())
