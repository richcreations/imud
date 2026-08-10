#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
gen-texi.py — build docs/imud.texi from docs/manual.md.

`info imud` is the third way to read the manual, after the file itself and
the man pages, and the only one that is navigable: chapters, menus and an
index, in a reader that is on every Debian system already.

The conversion is pandoc's, through two Lua filters that fix what a GFM
source cannot tell it (tools/texi-headings.lua, tools/texi-tables.lua), plus
a preamble pandoc's template does not write: @setfilename, the version, and
the @dircategory/@direntry pair that puts imud in the Info directory.

The preamble is spliced rather than supplied as a custom pandoc template.
Templates are versioned with pandoc and this tree meets two pandocs — 3.10 on
the dev box, 3.1.11 in the container — so a template lifted from one is not
guaranteed to render under the other.  Nine lines of splice are.

WHY THE OUTPUT IS COMMITTED BUT NOT DIFF-GATED.  It is committed because
`make dist` is `git archive HEAD` and a packager must not need pandoc.  It is
not diff-gated because those two pandocs do not produce identical Texinfo
from the same markdown, so a regenerate-and-diff would be red on whichever
runner did not match the last person to run it.  tools/check-texi.py gates it
instead, on the things that matter and that no pandoc version changes: every
heading has a node, the version matches include/version.h, and every
cross-reference resolves.  That check needs no pandoc, which is why CI can
run it without a 40 MB dependency.

CHAPTERS ARE NUMBERED TWICE, and deliberately.  docs/manual.md numbers its own
sections (`## 5. Supported drivers`) and Texinfo numbers chapters itself, so
Info shows "5 5. Supported drivers".  Stripping the manual's numbers would fix
the display and cost more than it is worth: the node name becomes "Supported
drivers", so tools/check-texi.py would need the same stripping rule to match
headings against nodes — a second copy of a transformation, which is the drift
this whole stage exists to remove.  A doubled chapter number is a familiar
artefact of converting a self-numbered document; a rule maintained in two
places is a bug waiting.

Run as `make docs-texi`.  Needs pandoc; maintainer-only.
"""

import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, must_read                            # noqa: E402

SRC = "docs/manual.md"
OUT = "docs/imud.texi"
FILTERS = ("tools/texi-headings.lua", "tools/texi-tables.lua")

# The generated-region markers sit BETWEEN a table's `| --- |` separator and
# its rows, because that is the span gen-config-docs.py owns.  To pandoc's
# GFM reader an HTML comment is a block, and a block in the middle of a pipe
# table ends the table: §4 came out as run-on prose with literal pipes in it.
#
# They are machine plumbing rather than content — invisible on GitHub, and
# meaningless in Info — so they are dropped before conversion.  A Lua filter
# cannot do this: by the time there is an AST the table is already broken.
MARKER = re.compile(r"^<!-- (?:BEGIN|END) GENERATED: .* -->\n", re.M)

# `* imud: (imud).` is the Info directory entry; the padding to column 32 is
# the convention every other entry in /usr/share/info/dir follows.
#
# "System administration" is a standard Debian dir section — inventing one
# would put imud in a category of its own at the bottom of the directory.
PREAMBLE = """\
@c GENERATED from docs/manual.md by tools/gen-texi.py — do not edit.
@c Edit the manual, then run `make docs-texi`.
@setfilename imud.info
@set VERSION %s
@set RELEASEDATE %s

@dircategory System administration
@direntry
* imud: (imud).                 IMU daemon for Raspberry Pi.
@end direntry
"""


def version():
    """(version, release date) from include/version.h — the one source."""
    src = must_read("include/version.h")
    out = []
    for macro in ("IMUD_VERSION_STR", "IMUD_RELEASE_DATE"):
        m = re.search(r'#define\s+%s\s+"([^"]+)"' % macro, src)
        if not m:
            sys.exit(f"include/version.h: cannot find {macro}")
        out.append(m.group(1))
    return out


def main():
    ver, date = version()

    source, n_markers = MARKER.subn("", must_read(SRC))
    if not n_markers:
        sys.exit(f"{SRC}: no generated-region markers found — either they "
                 f"moved, or this is stripping something it should not")

    cmd = ["pandoc", "-s", "-f", "gfm", "-t", "texinfo",
           # docs/manual.md's H1 is the document title.  Without this every
           # `## N. Foo` becomes a @section inside one giant chapter, which
           # is a single flat menu instead of a navigable book.
           "--shift-heading-level-by=-1"]
    for f in FILTERS:
        cmd += ["--lua-filter", os.path.join(ROOT, f)]

    with tempfile.TemporaryDirectory(prefix="imud-texi-") as tmp:
        # Written next to nothing, and named for the real file so any pandoc
        # diagnostic still points at a recognisable path.
        stripped = os.path.join(tmp, "manual.md")
        with open(stripped, "w", encoding="utf-8") as fh:
            fh.write(source)
        try:
            text = subprocess.run(cmd + [stripped], capture_output=True,
                                  text=True, check=True).stdout
        except FileNotFoundError:
            sys.exit("pandoc is not installed — `brew install pandoc`, or run "
                     "this in devbox")
        except subprocess.CalledProcessError as e:
            # A Lua filter that refuses (a node-name collision) reports here,
            # and its message is the useful one.
            sys.exit(e.stderr.strip() or f"pandoc failed with {e.returncode}")

    head = "\\input texinfo"
    if not text.startswith(head):
        sys.exit(f"{OUT}: pandoc no longer starts its output with "
                 f"{head!r} — the preamble splice has nowhere to go")
    i = text.index("\n") + 1
    text = text[:i] + PREAMBLE % (ver, date) + text[i:]

    path = os.path.join(ROOT, OUT)
    old = open(path, encoding="utf-8").read() if os.path.exists(path) else None
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)

    n = text.count("\n@node ")
    print("gen-texi: %s, %d nodes, imud %s%s"
          % (OUT, n, ver, "" if old == text else "  (changed)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
