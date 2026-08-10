#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-texi.py — docs/imud.texi must still be docs/manual.md.

docs/imud.texi is generated (tools/gen-texi.py) and committed, because
`make dist` is `git archive HEAD` and a packager must not need pandoc.  What
it is NOT is diff-gated: this tree meets pandoc 3.10 on the dev box and
3.1.11 in the container, they do not produce identical Texinfo from the same
markdown, and a regenerate-and-diff would be red on whichever runner did not
match whoever last ran it.

So the committed file is checked on the things that matter and that no pandoc
version changes:

  1. every heading in the manual has a node, and every node has a heading —
     a section added to the manual and never converted is invisible to
     `info imud`, which is the whole failure mode;
  2. @set VERSION matches include/version.h, so a release bump cannot leave
     the Info manual claiming the previous one;
  3. every cross-reference resolves to a node that exists;
  4. the Info directory entry is intact — without @dircategory/@direntry the
     page installs and never appears in `info` or `M-x info`.

All of it is text analysis, so CI runs it without pandoc's 40 MB.  `makeinfo`
is the other half of the gate and runs in build-and-test, where texinfo is
installed anyway to build the .info.

Run as `make check-texi` (via check-generated-text).
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import must_read, Report                          # noqa: E402

MANUAL = "docs/manual.md"
TEXI = "docs/imud.texi"
VERSION_H = "include/version.h"


def key(name):
    """A node/heading name reduced to what both spellings agree on.

    pandoc rewrites a heading on its way to a node name: it drops commas,
    periods and parentheses (`probe(fd, addr)` becomes `probefd addr`) and
    writes an em dash as ---, on top of the markup tools/texi-headings.lua
    has already flattened.  This undoes exactly that much and no more.

    Reducing further — to letters and digits alone — looks safer and is not:
    it merges `[logging]` with `Logging`, which are two different sections of
    this manual and two different nodes.  The check then compares 70 of 71
    headings and passes while one is missing, which is the failure it exists
    to catch.
    """
    name = name.lower().replace("—", "---").replace("–", "--")
    return re.sub(r"\s+", " ", re.sub(r"[`*_(),.@]", "", name)).strip()


def headings(rep):
    """{key: heading text} for every ##/###/#### heading, fences excluded."""
    out, fenced = {}, False
    for line in must_read(MANUAL).split("\n"):
        # A shell comment inside a ```toml block is not a heading, and
        # docs/manual.md has one that looks exactly like an H1.
        if line.startswith("```"):
            fenced = not fenced
            continue
        if fenced:
            continue
        m = re.match(r"^(#{2,4})\s+(.*?)\s*$", line)
        if m:
            text = re.sub(r"[`*_]", "", m.group(2))
            # Two headings reducing to one key would make this compare fewer
            # sections than the manual has and still report success.
            if key(text) in out:
                rep.fail(f"{MANUAL}: '{text}' and '{out[key(text)]}' reduce to "
                         f"the same node key — Info would give them one node")
            out[key(text)] = text
    rep.expect(out, f"{MANUAL} headings")
    return out


def main():
    rep = Report("check-texi")
    texi = must_read(TEXI, "run `make docs-texi`")

    # ── 1. headings <-> nodes ────────────────────────────────────────────────
    heads = headings(rep)
    nodes = {key(n): n.strip()
             for n in re.findall(r"^@node\s+(.+?)\s*$", texi, re.M)}
    rep.expect(nodes, f"{TEXI} nodes")
    nodes.pop("top", None)                     # the @top node has no heading

    for k, text in sorted(heads.items()):
        rep.check(k in nodes,
                  f"{TEXI}: '{text}' is a section of {MANUAL} with no @node — "
                  f"`info imud` cannot reach it; run `make docs-texi`")
    for k, name in sorted(nodes.items()):
        rep.check(k in heads,
                  f"{TEXI}: @node '{name}' has no heading in {MANUAL} — the "
                  f"conversion is stale; run `make docs-texi`")

    # ── 2. version ──────────────────────────────────────────────────────────
    want = re.search(r'#define\s+IMUD_VERSION_STR\s+"([^"]+)"',
                     must_read(VERSION_H))
    got = re.search(r"^@set VERSION\s+(\S+)\s*$", texi, re.M)
    if not want:
        rep.fail(f"{VERSION_H}: cannot find IMUD_VERSION_STR")
    elif not got:
        rep.fail(f"{TEXI}: no @set VERSION line")
    else:
        rep.check(got.group(1) == want.group(1),
                  f"{TEXI}: @set VERSION is {got.group(1)}, {VERSION_H} says "
                  f"{want.group(1)} — run `make docs-texi` after a bump")

    # ── 3. cross-references resolve ─────────────────────────────────────────
    # @ref{NODE,,LABEL}: the node is the first argument, and it is what Info
    # follows.  A stale one is a dead end a reader hits mid-sentence.
    known = set(nodes) | {"top"}
    for target in sorted({t.split(",")[0].strip() for t in
                          re.findall(r"@p?x?ref\{([^}]*)\}", texi)}):
        if not target or target.startswith("("):     # (dir), (info-file)node
            continue
        rep.check(key(target) in known,
                  f"{TEXI}: @ref to '{target}', which is not a node here")

    # ── 4. the Info directory entry ─────────────────────────────────────────
    for what, pattern in (("@dircategory", r"^@dircategory\s+\S"),
                          ("@direntry", r"^@direntry\s*$"),
                          ("the * imud: (imud). entry",
                           r"^\* imud: \(imud\)\.")):
        rep.check(re.search(pattern, texi, re.M) is not None,
                  f"{TEXI}: no {what} — the manual would install without "
                  f"appearing in the Info directory")

    return rep.finish(f"{len(heads)} sections, {len(nodes)} nodes, "
                      f"imud {got.group(1) if got else '?'}")


if __name__ == "__main__":
    sys.exit(main())
