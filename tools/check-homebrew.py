#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-homebrew.py — the Homebrew formulae against the packages they mirror.

packaging/homebrew/ holds one formula per Debian binary package, rendered into
the richcreations/homebrew-imud tap by .github/workflows/homebrew-publish.yml
when a draft release is published.  Three things have to stay true, and none of
them is visible from inside a single formula:

  1. NO FORMULA CARRIES A `head` BLOCK.  A tap is a delivery channel, and
     anything a user installs must come from a release — main is allowed to
     carry half-finished work, which is the whole reason releases exist.  A
     `head` line is a supported route straight to the branch, so it is refused
     here rather than left to review.  This shipped once: the formulae were
     HEAD-only for a day because 1.10.1's configure could not take the flags
     they pass, and the docs told people to install with --HEAD.

  2. EVERY FORMULA PINS THE SAME RELEASE.  homebrew-publish.yml rewrites url
     and sha256 across all eight in one pass; a formula left behind would
     install a different imud from its siblings, and imud-signalk against a
     different imud than the one it depends on is a support case nobody wants
     to debug.

  3. THE SPLIT MATCHES THE DEBS.  A new bridge package that never got a
     formula is a package apt users can install and brew users cannot, which
     nothing else would notice.

NO_FORMULA records the deliberate exceptions, so a package cannot join them by
being forgotten.

Run as `make check-homebrew` (via check-generated-text).
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, Report, must_read                     # noqa: E402

FORMULA_DIR = "packaging/homebrew"
CONTROL = "debian/control"

# Debian packages that deliberately have no formula of their own.
#
# Homebrew has no runtime/-dev split: a formula always installs the headers
# beside the library, so libimud0 and libimud-dev would be one formula.  That
# formula would then collide with imud over lib/libimud.0.dylib, because the
# core `make install` target installs the library as part of the daemon.  So
# the library ships inside imud, and the bridges reach it there.
NO_FORMULA = {
    "libimud0": "Homebrew installs headers with the library; both ship in imud",
    "libimud-dev": "same formula as libimud0 would be, so also inside imud",
}

# The tap's README, rendered from packaging/homebrew/README.md at each release.
README = "packaging/homebrew/README.md"


def control_packages(text):
    """The binary package names debian/control builds."""
    return [m.group(1) for m in re.finditer(r"^Package:\s*(\S+)", text, re.M)]


def formulae():
    """{formula name: source} for every .rb in packaging/homebrew."""
    d = os.path.join(ROOT, FORMULA_DIR)
    out = {}
    for fn in sorted(os.listdir(d)):
        if fn.endswith(".rb"):
            out[fn[:-3]] = must_read(os.path.join(FORMULA_DIR, fn))
    return out


def main():
    rep = Report("check-homebrew")

    forms = formulae()
    rep.expect(forms, f"{FORMULA_DIR}/*.rb formulae")

    # ── 1. no route to a branch ──────────────────────────────────────────────
    for name, src in forms.items():
        head = re.search(r"^\s*head\s", src, re.M)
        rep.check(head is None,
                  f"{FORMULA_DIR}/{name}.rb declares a `head` block. A tap "
                  f"serves releases only — delete it, and let "
                  f"homebrew-publish.yml pin url/sha256 at the next release")

    # ── 2. one release across all of them ────────────────────────────────────
    pins = {}
    for name, src in forms.items():
        url = re.search(r'^\s*url\s+"([^"]+)"', src, re.M)
        sha = re.search(r'^\s*sha256\s+"([0-9a-f]{64})"', src, re.M)
        if not url:
            rep.fail(f"{FORMULA_DIR}/{name}.rb has no url")
            continue
        if not sha:
            rep.fail(f"{FORMULA_DIR}/{name}.rb has no sha256 "
                     f"(or it is not 64 hex characters)")
            continue
        pins[name] = (url.group(1), sha.group(1))

        # The tag in the path and the version in the tarball name are written
        # by the same rewrite, so a mismatch means a hand edit went wrong.
        m = re.search(r"/download/v([0-9][^/]*)/imud-([0-9][^/]*)\.tar\.gz$",
                      url.group(1))
        if not m:
            rep.fail(f"{FORMULA_DIR}/{name}.rb url is not a release tarball: "
                     f"{url.group(1)}")
        elif m.group(1) != m.group(2):
            rep.fail(f"{FORMULA_DIR}/{name}.rb url names tag v{m.group(1)} "
                     f"but tarball {m.group(2)}")

    if pins:
        want = pins[sorted(pins)[0]]
        for name in sorted(pins):
            rep.check(pins[name] == want,
                      f"{FORMULA_DIR}/{name}.rb pins a different release from "
                      f"its siblings ({pins[name][0].rsplit('/', 1)[-1]} vs "
                      f"{want[0].rsplit('/', 1)[-1]}) — they are rewritten "
                      f"together and must stay together")

    # ── 3. one formula per binary package ────────────────────────────────────
    packages = control_packages(must_read(CONTROL, "the binary packages"))
    rep.expect(packages, f"{CONTROL} Package: stanzas")

    for pkg in packages:
        if pkg in NO_FORMULA:
            continue
        rep.check(pkg in forms,
                  f"{CONTROL} builds {pkg}, which no formula installs — add "
                  f"{FORMULA_DIR}/{pkg}.rb, or record the exemption in "
                  f"NO_FORMULA in this checker")

    for name in forms:
        rep.check(name in packages,
                  f"{FORMULA_DIR}/{name}.rb installs {name}, which "
                  f"{CONTROL} does not build")

    # ── 4. the README is version-agnostic, and offers no branch either ───────
    #
    # It is copied to the tap verbatim at each release, so anything
    # version-specific in it is stale the moment the next one ships — which is
    # exactly how it came to advertise `--HEAD` for a release that had already
    # been tagged.  Nothing rewrites it, so nothing must need rewriting.
    readme = must_read(README, "the tap README")
    for m in re.finditer(r"\b\d+\.\d+(?:\.\d+)?\b", readme):
        rep.fail(f"{README} names a version ({m.group(0)}). The tap README is "
                 f"copied verbatim at every release and nothing rewrites it, "
                 f"so it has to read the same for all of them")

    for path, text in list(forms.items()) + [(README, readme)]:
        where = README if path == README else f"{FORMULA_DIR}/{path}.rb"
        rep.check("--HEAD" not in text,
                  f"{where} advertises --HEAD. The formulae carry no head "
                  f"block, so that instruction cannot work — and a tap serves "
                  f"releases only")

    version = want[0].rsplit("/imud-", 1)[-1][:-7] if pins else "?"
    return rep.finish(f"{len(forms)} formulae, all pinned to {version}")


if __name__ == "__main__":
    sys.exit(main())
