#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-portable-tests.py — every suite in TEST_BINS must be known to everything
that has to know about it, and must run on every host that can run it.

THE DRIFT THIS EXISTS FOR.  The macos CI job built the tree and ran
test_host_time.  One suite of thirty-nine.  Everything else — fusion, the
packet layout, the config parser, the drivers, the two end-to-end mains — was
compiled by nobody off Linux and had never been executed there, so a
portability regression in any of them went green.  The job looked thorough:
it probed ./configure's rungs, built both the core and the bridges, and ran a
real daemon.  What it did not do was test.

Nothing counted.  TEST_BINS is a hand-maintained list, and four other places
restate it — the `test` recipe, `clean`, .gitignore, and CI.  A suite absent
from any of them fails silently and in a different way each time: one that
`clean` forgets survives a rebuild, one .gitignore forgets waits in `git
status` for a wildcard to stage, and one the `test` recipe forgets is built on
every run and executed on none.

So this compares them:

  Makefile TEST_BINS            the suites that exist -- the source of truth
  Makefile `test:` recipe       every suite is actually RUN
  Makefile NONPORTABLE_TEST_BINS  names a real suite, and says why
  Makefile PORTABLE_TEST_BINS   is computed, never hand-listed
  Makefile `test-portable:`     runs the computed list
  Makefile clean                so a suite does not survive `make clean`
  .gitignore                    so it does not survive `git status`
  ci.yml macos job              runs the portable target, not a hand-picked suite

PORTABLE IS THE DEFAULT.  PORTABLE_TEST_BINS is TEST_BINS minus an exclusion
list, so a new suite reaches the macos job by existing.  That is the half of
this a checker cannot do; what it can do is refuse an exclusion that names a
suite which is not there (`filter-out` accepts a typo in silence) or one with
no stated reason.

TWO CHECKS THAT ARE NOT SET COMPARISONS.  Both guard prose, which is where the
claim a contributor actually reads lives -- and both were wrong when this was
written: CONTRIBUTING.md and devbox/README.md told anyone developing on a Mac
that it built 31 of 37 suites and could not link imud at all.

  * Stated suite counts must be the real ones.  "<n> suites" is TEST_BINS;
    "<n> portable suites" is what macOS runs.  Only the files in COUNTED are
    scanned: NEWS records what a past release shipped and is frozen.

  * The macos job may not build-and-run a single suite by name.  Deliberately
    blunt -- when the whole list runs, one suite named by hand is either dead
    weight or the beginning of the hand-picked set this replaced.

Run as `make check-portable-tests` (via check-generated-text).
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import Report, must_read                          # noqa: E402

MAKEFILE  = "Makefile"
GITIGNORE = ".gitignore"
CI        = ".github/workflows/ci.yml"
CONTRIB   = "CONTRIBUTING.md"
DEVBOX    = "devbox/README.md"

# Files whose prose states how many suites there are.  An explicit list, not a
# tree sweep: a release note describes what shipped and is frozen.
COUNTED = [CONTRIB, DEVBOX]

# `<n> suites` -> all of them; `<n> portable suites` -> what macOS runs.  The
# two patterns are disjoint: the first requires the noun immediately after the
# number, so "39 portable suites" cannot match it.
COUNT_RE    = re.compile(r"\b(\d+)[ \t]+suites?\b", re.I)
PORTABLE_RE = re.compile(r"\b(\d+)[ \t]+portable[ \t]+suites?\b", re.I)


def make_var(text, name, rep):
    """A make variable's value, following backslash continuations.

    Returns the raw string; the caller splits it.  An empty assignment is a
    legitimate value here (NONPORTABLE_TEST_BINS is empty), so absence and
    emptiness are different answers: None means the variable is gone.
    """
    lines = text.split("\n")
    for i, line in enumerate(lines):
        m = re.match(r"^%s\s*=(.*)$" % re.escape(name), line)
        if not m:
            continue
        value = m.group(1)
        while value.rstrip().endswith("\\"):
            i += 1
            if i >= len(lines):
                break
            value = value.rstrip()[:-1] + " " + lines[i]
        return value
    rep.fail(f"{MAKEFILE}: no {name} assignment")
    return None


def recipe_body(text, target, rep):
    """A make recipe's body: the indented lines under `<target>:`."""
    lines = text.split("\n")
    start = next((i for i, l in enumerate(lines)
                  if re.match(r"^%s:" % re.escape(target), l)), None)
    if start is None:
        rep.fail(f"{MAKEFILE}: no `{target}:` rule")
        return ""
    body = []
    for line in lines[start + 1:]:
        if line and not line[0].isspace():
            break
        body.append(line)
    return "\n".join(body)


def job_block(text, job, rep):
    """One job out of a workflow: from `  <job>:` to the next job at that
    indent.  Reading the whole file instead would let a `test-portable` in some
    other job satisfy the assertion about this one."""
    m = re.search(r"^  %s:\n(.*?)(?=^  \w[\w-]*:\n|\Z)" % re.escape(job),
                  text, re.M | re.S)
    if not m:
        rep.fail(f"{CI}: no `{job}:` job")
        return ""
    # Comments out.  The job's own prose names the target and the suite it used
    # to run, so leaving them in would let a comment satisfy the assertion that
    # the job RUNS the target -- the checker passing on the description of the
    # defect it checks for.
    return "\n".join(l for l in m.group(1).split("\n")
                     if not l.lstrip().startswith("#"))


def check_counts(rep, n_all, n_portable):
    """Every stated suite count is the real one."""
    seen = 0
    for rel in COUNTED:
        text = must_read(rel, "a file that states the suite count")
        for m in PORTABLE_RE.finditer(text):
            seen += 1
            rep.check(int(m.group(1)) == n_portable,
                      f"{rel}: says {m.group(0)!r} but PORTABLE_TEST_BINS "
                      f"holds {n_portable}")
        # The portable matches are a subset of the plain ones -- "39 portable
        # suites" contains no "39 suites", but "39 suites" inside a sentence
        # that also says "portable" elsewhere would be double-counted without
        # matching on the number's own neighbours, which COUNT_RE does.
        for m in COUNT_RE.finditer(text):
            seen += 1
            rep.check(int(m.group(1)) == n_all,
                      f"{rel}: says {m.group(0)!r} but TEST_BINS holds "
                      f"{n_all}")
    rep.expect(seen, f"suite counts stated in {', '.join(COUNTED)}")


def main():
    rep = Report("check-portable-tests")
    mk = must_read(MAKEFILE, "the build")

    raw = make_var(mk, "TEST_BINS", rep)
    if raw is None:
        return rep.finish("TEST_BINS is gone")
    suites = set(re.findall(r"\btest_\w+", raw))
    rep.expect(suites, f"{MAKEFILE} TEST_BINS")
    if not suites:
        return rep.finish("no suites found")

    surfaces = [
        (f"{MAKEFILE} `test:` recipe",
         set(re.findall(r"^\t\./(test_\w+)\s*$",
                        recipe_body(mk, "test", rep), re.M))),
        (f"{MAKEFILE} clean recipe",
         set(re.findall(r"\b(test_\w+)\b", recipe_body(mk, "clean", rep)))),
        (f"{GITIGNORE} suite binaries",
         set(re.findall(r"^/(test_\w+)$",
                        must_read(GITIGNORE, "the ignore list"), re.M))),
    ]

    for label, found in surfaces:
        rep.expect(found, label)
        for t in sorted(suites - found):
            rep.check(False, f"{label}: does not list {t}, which is in "
                             f"TEST_BINS")
        for t in sorted(found - suites):
            rep.check(False, f"{label}: lists {t}, which is not in TEST_BINS")

    # ── the exclusion list ───────────────────────────────────────────────────
    raw_out = make_var(mk, "NONPORTABLE_TEST_BINS", rep)
    excluded = set(re.findall(r"\btest_\w+", raw_out or ""))
    for t in sorted(excluded - suites):
        rep.check(False,
                  f"{MAKEFILE} NONPORTABLE_TEST_BINS: excludes {t}, which is "
                  f"not in TEST_BINS — $(filter-out) accepts a name that does "
                  f"not exist without a word")
    for t in sorted(excluded & suites):
        rep.check(re.search(r"^#\s+%s\s+[-—]\s+\S" % re.escape(t), mk, re.M)
                  is not None,
                  f"{MAKEFILE}: {t} is in NONPORTABLE_TEST_BINS with no "
                  f"`#   {t} — <reason>` line saying what a non-Linux host "
                  f"lacks")

    # Computed, never hand-listed: a literal list here is the drift itself.
    rep.check("PORTABLE_TEST_BINS = $(filter-out $(NONPORTABLE_TEST_BINS),"
              "$(TEST_BINS))" in mk,
              f"{MAKEFILE}: PORTABLE_TEST_BINS must stay "
              f"$(filter-out $(NONPORTABLE_TEST_BINS),$(TEST_BINS)) — spelling "
              f"it out by hand is how a new suite lands outside the macos job")

    portable = recipe_body(mk, "test-portable", rep)
    rep.check("$(PORTABLE_TEST_BINS)" in portable,
              f"{MAKEFILE}: the `test-portable:` recipe does not run "
              f"$(PORTABLE_TEST_BINS)")

    # ── CI ───────────────────────────────────────────────────────────────────
    macos = job_block(must_read(CI, "the push CI workflow"), "macos", rep)
    rep.check(re.search(r"make\s+(?:-\S+\s+)*test-portable\b", macos)
              is not None,
              f"{CI} macos job: does not run `make test-portable` — it ran one "
              f"suite of {len(suites)} until 1.9.2, which is the defect this "
              f"checks for")
    m = re.search(r"make\s+(?:-\S+\s+)*(test_\w+)", macos)
    rep.check(m is None,
              f"{CI} macos job: builds {m.group(1) if m else ''} by name — the "
              f"whole portable list runs there, so a hand-picked suite is "
              f"either dead weight or the start of the set that replaced it")

    check_counts(rep, len(suites), len(suites - excluded))

    return rep.finish(f"{len(suites)} suites across {len(surfaces) + 1} "
                      f"surfaces, {len(excluded)} excluded from the "
                      f"{len(suites - excluded)} the macos job runs, and the "
                      f"counts in {len(COUNTED)} files")


if __name__ == "__main__":
    sys.exit(main())
