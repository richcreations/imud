#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-package-descriptions.py — packaging/<pkg>/description must be the
description debian/control actually ships.

Ten binary packages come out of this source tree, and each one's description is
written down twice: once in debian/control, which is what dpkg reads and what
users see in `apt show`, and once in packaging/<pkg>/description, which is the
per-package documentation set the maintainer edits.  Nothing compared them.

That makes this the last cross-surface fact in the tree with no checker, and
the failure mode is the quiet one: the two files drift, `apt show` prints one
description while the repo documents another, and nobody notices because
neither file is wrong on its own terms.  A packager reading packaging/ to
write a downstream spec gets the stale half.

Comparison is exact, over the synopsis AND the extended description, because
a body that has drifted is worse than a synopsis that has: it is longer, it is
what a reader actually reads, and it is where a removed dependency or a
renamed tool goes unmentioned.

Two mapping rules, both explicit rather than inferred, so a new package cannot
join by accident:

  * DIRNAME_TO_PACKAGE maps a packaging/ directory to its control stanza where
    the names differ.  libimud/ documents libimud0 — the SONAME suffix is a
    Debian library-packaging convention, not a different package.

  * NO_DESCRIPTION_DIR lists control packages that deliberately have no
    packaging/ directory.  libimud-dev is the only one: Debian derives a -dev
    package's description from its runtime half, and duplicating it here would
    create a third copy of the same prose to keep in step.

Run as `make check-package-descriptions` (via check-generated-text).
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, Report, must_read                   # noqa: E402

CONTROL = "debian/control"

DIRNAME_TO_PACKAGE = {
    "libimud": "libimud0",
}

NO_DESCRIPTION_DIR = {
    "libimud-dev",
}


def control_descriptions(text):
    """{package: (synopsis, [body lines])} from debian/control."""
    out = {}
    pkg = None
    for stanza in re.split(r"\n(?=Package:)", text):
        m = re.match(r"Package:\s*(\S+)", stanza)
        if not m:
            continue
        pkg = m.group(1)
        d = re.search(r"^Description:[ \t]*(.*)$", stanza, re.M)
        if not d:
            continue
        synopsis = d.group(1).rstrip()
        body = []
        # Continuation lines start with a single space; the stanza ends at the
        # first line that does not.
        for line in stanza[d.end():].split("\n"):
            if line.startswith(" "):
                body.append(line)
            elif line.strip() == "":
                continue
            else:
                break
        out[pkg] = (synopsis, body)
    return out


def file_description(rel):
    """(synopsis, [body lines]) from a packaging/<pkg>/description file."""
    lines = must_read(rel, "per-package description").rstrip("\n").split("\n")
    return lines[0].rstrip(), [l for l in lines[1:]]


def main():
    r = Report("check-package-descriptions")
    control = control_descriptions(must_read(CONTROL, "the binary packages"))
    r.expect(control, f"{CONTROL} Package:/Description: stanzas")

    pkgdir = os.path.join(ROOT, "packaging")
    dirs = sorted(d for d in os.listdir(pkgdir)
                  if os.path.isfile(os.path.join(pkgdir, d, "description")))
    r.expect(dirs, "packaging/*/description files")

    seen = set()
    for d in dirs:
        pkg = DIRNAME_TO_PACKAGE.get(d, d)
        seen.add(pkg)
        if pkg not in control:
            r.fail(f"packaging/{d}/description documents {pkg}, which "
                   f"{CONTROL} does not build. Rename the directory, or add "
                   f"the mapping to DIRNAME_TO_PACKAGE in this checker.")
            continue

        want_syn, want_body = control[pkg]
        got_syn, got_body = file_description(f"packaging/{d}/description")

        r.check(got_syn == want_syn,
                f"packaging/{d}/description synopsis differs from {CONTROL} "
                f"({pkg}):\n       packaging/: {got_syn}\n       control:    "
                f"{want_syn}")

        if not r.check(got_body == want_body,
                       f"packaging/{d}/description body differs from "
                       f"{CONTROL} ({pkg}) — {len(got_body)} line(s) here "
                       f"against {len(want_body)} there"):
            for i, (a, b) in enumerate(zip(got_body, want_body)):
                if a != b:
                    r.fail(f"  first difference at body line {i + 1}:\n"
                           f"       packaging/: {a}\n"
                           f"       control:    {b}")
                    break

    for pkg in sorted(control):
        if pkg not in seen and pkg not in NO_DESCRIPTION_DIR:
            r.fail(f"{CONTROL} builds {pkg}, which has no "
                   f"packaging/{pkg}/description. Every shipped package "
                   f"carries its own doc set; add one, or list it in "
                   f"NO_DESCRIPTION_DIR here with the reason.")

    return r.finish(f"{len(dirs)} package description(s) match {CONTROL}")


if __name__ == "__main__":
    sys.exit(main())
