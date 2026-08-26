#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-comment-refs.py — every code reference a comment makes must resolve.

Comments here name functions and cite `file.c:123` freely, and both rot
silently: a rename leaves the comment pointing at nothing, and nothing in the
build notices.  This asserts that a `foo()` named in a comment exists somewhere
in the tree, and that a `file.c:123` names a real file and a line it has.

Both were clean when this was written, which is the point: it costs nothing now
and keeps them that way.

Deliberately narrow.  It does not check prose, length or phrasing — those need
judgement, and a checker that fires on judgement is one people learn to skip.
"""

import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, Report                                # noqa: E402

COMMENT = re.compile(r'^\s*(/\*|\*|//)')
FUNC    = re.compile(r'\b([a-z_][a-z0-9_]{3,})\(\)')
FILELN  = re.compile(r'\b([a-z0-9_]+\.[ch]):(\d+)\b')
DEFINED = re.compile(r'\b([a-z_][a-z0-9_]{3,})\s*\(')

# Named in comments as concepts rather than as functions in this tree.
ALLOW = {
    "etc", "ioctl", "syscall", "sigwait", "select", "poll", "accept",
    "connect", "listen", "recv", "send", "read", "write", "open", "close",
    "fork", "exec", "exit", "main", "malloc", "free", "printf", "sleep",
    "pthread_self", "localtime", "recvfrom",
}


def tracked():
    """Tracked C sources, falling back to a walk when ROOT is not a checkout
    (test_checkers.py runs every checker against a copied tree)."""
    try:
        out = subprocess.run(["git", "-C", ROOT, "ls-files"],
                             capture_output=True, text=True).stdout.split()
    except OSError:
        out = []
    if not out:
        out = []
        for base, dirs, names in os.walk(ROOT):
            dirs[:] = [d for d in dirs if d not in (".git", "notes")]
            for n in names:
                out.append(os.path.relpath(os.path.join(base, n), ROOT))
    return sorted(f for f in out
                  if f.endswith((".c", ".h")) and not f.endswith(".gen.c"))


def main():
    rep = Report("check-comment-refs")
    files = tracked()
    rep.expect(files, "tracked C sources")

    whole = []
    for f in files:
        whole.append(open(os.path.join(ROOT, f), errors="replace").read())
    # Built from CODE lines only: a symbol set taken from whole files would
    # include names that appear nowhere but in comments, so every reference
    # would validate itself.
    symbols = set()
    for src in whole:
        for line in src.split("\n"):
            if not COMMENT.match(line):
                symbols.update(DEFINED.findall(line))

    n_func = n_file = 0
    for f, src in zip(files, whole):
        for lineno, line in enumerate(src.split("\n"), 1):
            if not COMMENT.match(line):
                continue
            for m in FUNC.finditer(line):
                name = m.group(1)
                if name in ALLOW:
                    continue
                n_func += 1
                rep.check(name in symbols,
                          f"{f}:{lineno}: comment names {name}(), which is "
                          f"defined and called nowhere in the tree")
            for m in FILELN.finditer(line):
                base, want = m.group(1), int(m.group(2))
                n_file += 1
                hits = [g for g in files if os.path.basename(g) == base]
                if not rep.check(hits, f"{f}:{lineno}: cites {base}, which is "
                                       f"not a file in the tree"):
                    continue
                have = len(open(os.path.join(ROOT, hits[0]),
                                errors="replace").read().split("\n"))
                rep.check(want <= have,
                          f"{f}:{lineno}: cites {base}:{want}, but that file "
                          f"has {have} lines")

    return rep.finish(f"{n_func} function and {n_file} file:line references "
                      f"in comments across {len(files)} sources")


if __name__ == "__main__":
    sys.exit(main())
