#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-fuzz-targets.py — every harness in fuzz/ must be known to everything that
has to know about it.

THE DRIFT THIS EXISTS FOR.  `fuzz_argv` was added in 6bc6a51 and reached two of
the seven places that name a harness.  Five months later SECURITY.md — the
document a reporter reads before deciding what is worth reporting — still said
command-line arguments were "deliberately **not** fuzzed" and listed six
harnesses, `.gitignore` still let a built `fuzz_argv` sit in `git status` for a
wildcard to stage, `make clean` still left it behind, and codeql.yml still said
"six fuzzers".  Every one of those was correct when written.

Adding a harness is a seven-surface edit and nothing counted them.  check-links
reads links; the doc generators do not model CI.  So this compares the files on
disk against each surface that restates them:

  fuzz/fuzz_*.c            the harnesses that exist -- the source of truth
  ci.yml $FZ lines         the push build
  ci.yml `for t in ...`    the 60 s smoke loop; a harness built and not run is
                           the silent failure, since the build still passes
  fuzz-nightly.yml matrix  the 1 h deep run
  fuzz-nightly.yml DEPS    its per-target link line; a missing case links the
                           harness against nothing and fails at the first
                           undefined symbol
  SECURITY.md Harness col  what a reporter is told is covered
  Makefile clean           so a built harness does not survive `make clean`
  .gitignore               so it does not survive `git status`
  test/fuzz/corpus/<t>     seeds: CI passes this path, and libFuzzer starting
                           from nothing rediscovers the file format each run

TWO CHECKS THAT ARE NOT SET COMPARISONS.  Both guard the prose, which is where
the reported defect actually lived — the table was merely wrong alongside it.

  * Spelled or numeric counts ("seven harnesses", "six fuzzers") must equal the
    number of harnesses.  Only the four files in COUNTED are scanned: NEWS
    describes what shipped in a past release and must keep saying six.

  * SECURITY.md may not claim anything is "not fuzzed".  Deliberately blunt —
    every harness on disk is fuzzed, and the document's job is to say what is
    covered.  The alternative is a checker that tries to work out which noun a
    negation attached to, which is judgement, and a checker that fires on
    judgement is one people learn to skip.

Run as `make check-fuzz-targets` (via check-generated-text).
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import Report, must_read, path                    # noqa: E402

FUZZ_DIR  = "fuzz"
CORPUS    = "test/fuzz/corpus"
SECURITY  = "SECURITY.md"
CI        = ".github/workflows/ci.yml"
NIGHTLY   = ".github/workflows/fuzz-nightly.yml"
CODEQL    = ".github/workflows/codeql.yml"
MAKEFILE  = "Makefile"
GITIGNORE = ".gitignore"

# Files whose prose states how many harnesses there are.  An explicit list, not
# a tree sweep: NEWS records what a past release shipped and is frozen.
COUNTED = [SECURITY, CI, NIGHTLY, CODEQL]

NUMBER = {
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12,
}

# A count word IMMEDIATELY before the noun, on the same line.  Allowing an
# intervening word makes "one hour per harness" (fuzz-nightly.yml's own header)
# a false positive; allowing \s to cross a newline makes a line ending in a
# number and a block starting with "fuzz:" into another one.
COUNT_RE = re.compile(
    r"\b(\d+|" + "|".join(NUMBER) + r")[ \t]+(fuzz\w*|harness\w*)\b", re.I)


def on_disk(rep):
    """The harnesses that exist. fuzz/ also holds mkseed_packet.c and
    standalone_main.c, which are tooling, not harnesses -- neither matches."""
    names = {m.group(1)
             for f in sorted(os.listdir(path(FUZZ_DIR)))
             for m in [re.fullmatch(r"fuzz_(\w+)\.c", f)] if m}
    rep.expect(names, f"{FUZZ_DIR}/fuzz_*.c harnesses")
    return names


def ci_built(text, rep):
    names = set(re.findall(r"^\s*\$FZ\s+fuzz/fuzz_(\w+)\.c", text, re.M))
    rep.expect(names, f"{CI} $FZ build lines")
    return names


def ci_run(text, rep):
    m = re.search(r"^\s*for t in ([^;]+); do\s*$", text, re.M)
    if not m:
        rep.expect(None, f"{CI} `for t in ...` smoke loop")
        return set()
    names = set(m.group(1).split())
    rep.expect(names, f"{CI} `for t in ...` smoke loop")
    return names


def nightly_matrix(text, rep):
    m = re.search(r"^\s*target: \[([^\]]+)\]", text, re.M)
    if not m:
        rep.expect(None, f"{NIGHTLY} matrix target list")
        return set()
    names = {t.strip() for t in m.group(1).split(",") if t.strip()}
    rep.expect(names, f"{NIGHTLY} matrix target list")
    return names


def nightly_deps(text, rep):
    names = set(re.findall(r"^\s*(\w+)\)\s+DEPS=", text, re.M))
    rep.expect(names, f"{NIGHTLY} DEPS case arms")
    return names


def security_table(text, rep):
    """The Harness column. Table rows only -- the prose below names harnesses
    too, and a paragraph is not the table this pins."""
    names = set()
    for line in text.split("\n"):
        if line.startswith("|"):
            names.update(re.findall(r"`fuzz_(\w+)`", line))
    rep.expect(names, f"{SECURITY} Harness column")
    return names


def makefile_clean(text, rep):
    """Harness binaries the `clean` recipe removes. The recipe is one
    backslash-continued logical line whose physical lines are indented with
    spaces, not tabs, so it is read as "indented until it is not"."""
    lines = text.split("\n")
    start = next((i for i, l in enumerate(lines) if l.startswith("clean:")),
                 None)
    if start is None:
        rep.expect(None, f"{MAKEFILE} clean recipe")
        return set()
    body = []
    for line in lines[start + 1:]:
        if line and not line[0].isspace():
            break
        body.append(line)
    names = set(re.findall(r"\bfuzz_(\w+)\b", "\n".join(body)))
    rep.expect(names, f"{MAKEFILE} clean recipe")
    return names


def gitignore(text, rep):
    names = set(re.findall(r"^/fuzz_(\w+)$", text, re.M))
    rep.expect(names, f"{GITIGNORE} harness binaries")
    return names


def check_counts(targets, rep):
    """Every stated harness count is the real one."""
    seen = 0
    for rel in COUNTED:
        text = must_read(rel, "a file that states the harness count")
        for m in COUNT_RE.finditer(text):
            word = m.group(1).lower()
            n = NUMBER.get(word, int(word) if word.isdigit() else None)
            if n is None:
                continue
            seen += 1
            rep.check(n == len(targets),
                      f"{rel}: says {m.group(0)!r} but {FUZZ_DIR}/ holds "
                      f"{len(targets)} harnesses ({', '.join(sorted(targets))})")
    rep.expect(seen, f"harness counts stated in {', '.join(COUNTED)}")


def main():
    rep = Report("check-fuzz-targets")
    targets = on_disk(rep)
    if not targets:
        return rep.finish("no harnesses found")

    ci = must_read(CI, "the push CI workflow")
    nightly = must_read(NIGHTLY, "the nightly fuzz workflow")

    surfaces = [
        (f"{CI} $FZ build lines",        ci_built(ci, rep)),
        (f"{CI} 60 s smoke loop",        ci_run(ci, rep)),
        (f"{NIGHTLY} matrix",            nightly_matrix(nightly, rep)),
        (f"{NIGHTLY} DEPS case",         nightly_deps(nightly, rep)),
        (f"{SECURITY} Harness column",
         security_table(must_read(SECURITY, "the security policy"), rep)),
        (f"{MAKEFILE} clean recipe",
         makefile_clean(must_read(MAKEFILE, "the build"), rep)),
        (f"{GITIGNORE} harness binaries",
         gitignore(must_read(GITIGNORE, "the ignore list"), rep)),
    ]

    for label, found in surfaces:
        for t in sorted(targets - found):
            rep.check(False,
                      f"{label}: does not list fuzz_{t}, which exists as "
                      f"{FUZZ_DIR}/fuzz_{t}.c")
        for t in sorted(found - targets):
            rep.check(False,
                      f"{label}: lists fuzz_{t}, but there is no "
                      f"{FUZZ_DIR}/fuzz_{t}.c")

    for t in sorted(targets):
        seeds = os.path.join(path(CORPUS), t)
        rep.check(os.path.isdir(seeds) and os.listdir(seeds),
                  f"{CORPUS}/{t}: no seed corpus for fuzz_{t} — CI passes this "
                  f"path to the harness, and a run from nothing spends its "
                  f"budget rediscovering the input format")

    check_counts(targets, rep)

    # The reported defect: the table was corrected once and the paragraph under
    # it went on saying the opposite.
    text = must_read(SECURITY, "the security policy")
    m = re.search(r"\bnot\b[^\n]{0,6}\bfuzzed\b", text, re.I)
    rep.check(m is None,
              f"{SECURITY}: says {m.group(0)!r} — every harness in {FUZZ_DIR}/ "
              f"is fuzzed, including argv; state why a surface is low risk "
              f"rather than that it is uncovered" if m else "")

    return rep.finish(f"{len(targets)} harnesses across {len(surfaces)} "
                      f"surfaces, seeds, and the counts in "
                      f"{len(COUNTED)} files")


if __name__ == "__main__":
    sys.exit(main())
