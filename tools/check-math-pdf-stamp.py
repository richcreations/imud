#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-math-pdf-stamp.py — docs/math.pdf must be the docs/math.md it ships with.

docs/math.pdf is a rendered copy of docs/math.md, committed so a reader can
open the derivations without a LaTeX toolchain.  A copy nothing compares goes
stale, and this one did: it sat three weeks behind its source, because
build-math-pdf.sh hard-required xelatex and failed on the box where math.md
was being edited.

docs/math.pdf.stamp holds the SHA-256 of the math.md the PDF was built from,
written by build-math-pdf.sh only after pandoc succeeds.  This compares it
against math.md as it stands.

Why a hash and not an mtime: git does not preserve mtimes, so in a fresh
clone the PDF is newer or older than its source at random, and any timestamp
rule is a coin toss.  Why not rebuild in CI and diff: the PDF is not
reproducible — pandoc, the engine and the font stack all vary — and CI would
need a TeX toolchain to produce a file it could not meaningfully compare.

The remedy is one command, which is the point: `make math-pdf`.

Run as `make check-math-pdf-stamp` (via check-generated-text).
"""

import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, read, must_read, Report              # noqa: E402

SRC = "docs/math.md"
PDF = "docs/math.pdf"
STAMP = "docs/math.pdf.stamp"


def main():
    rep = Report("check-math-pdf-stamp")

    source = must_read(SRC).encode("utf-8")
    digest = hashlib.sha256(source).hexdigest()

    if not os.path.exists(os.path.join(ROOT, PDF)):
        rep.fail(f"{PDF} is missing — run `make math-pdf`")
        return rep.finish("no PDF to check")

    stamp = read(STAMP)
    if stamp is None:
        rep.fail(f"{STAMP} is missing: nothing records which {SRC} "
                 f"{PDF} was built from — run `make math-pdf`")
    else:
        got = stamp.split()[0] if stamp.split() else ""
        rep.check(got == digest,
                  f"{PDF} was built from a different {SRC} "
                  f"({got[:12] or '(empty)'}... vs {digest[:12]}...) — "
                  f"run `make math-pdf` and commit both")

    return rep.finish(f"{PDF} matches {SRC} ({digest[:12]}...)")


if __name__ == "__main__":
    sys.exit(main())
