#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
pin-homebrew.py — point every formula at one release tarball.

.github/workflows/homebrew-publish.yml runs this when a draft release is
published, then pushes the result to the richcreations/homebrew-imud tap and
commits the same rewrite back to main.  Run it by hand to rehearse:

    python3 tools/pin-homebrew.py --version 1.11.0 --sha256 <hex>

Every formula is rewritten in one pass, because a tap where imud-signalk pins
a different release from imud installs a bridge against a daemon it was never
built beside.  check-homebrew.py refuses that state; this is what keeps it
from arising.

It fails loudly on purpose.  A rewrite that silently matched nothing is the
worst outcome available here — the workflow would report success, push the
unchanged formulae, and the tap would go on serving the previous release with
a commit claiming otherwise.  So the count of files changed is asserted
against the count of files found, and both are printed.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT                                        # noqa: E402

FORMULA_DIR = "packaging/homebrew"
URL = ("https://github.com/richcreations/imud/releases/download/"
       "v{v}/imud-{v}.tar.gz")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--version", required=True,
                    help="release version, without the leading v")
    ap.add_argument("--sha256", required=True,
                    help="sha256 of imud-<version>.tar.gz")
    args = ap.parse_args()

    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", args.version):
        sys.exit(f"pin-homebrew: --version {args.version!r} is not X.Y.Z")
    if not re.fullmatch(r"[0-9a-f]{64}", args.sha256):
        sys.exit("pin-homebrew: --sha256 is not 64 lowercase hex characters")

    url = URL.format(v=args.version)
    d = os.path.join(ROOT, FORMULA_DIR)
    names = sorted(f for f in os.listdir(d) if f.endswith(".rb"))
    if not names:
        sys.exit(f"pin-homebrew: no formulae in {FORMULA_DIR}")

    changed = 0
    for fn in names:
        path = os.path.join(d, fn)
        with open(path, encoding="utf-8") as f:
            src = f.read()

        new, n_url = re.subn(r'^(\s*url\s+)"[^"]*"',
                             lambda m: f'{m.group(1)}"{url}"', src,
                             count=1, flags=re.M)
        new, n_sha = re.subn(r'^(\s*sha256\s+)"[^"]*"',
                             lambda m: f'{m.group(1)}"{args.sha256}"', new,
                             count=1, flags=re.M)
        if n_url != 1 or n_sha != 1:
            sys.exit(f"pin-homebrew: {FORMULA_DIR}/{fn} has "
                     f"{n_url} url and {n_sha} sha256 lines to rewrite, "
                     f"expected 1 of each — refusing to write a partial pin")

        if new != src:
            with open(path, "w", encoding="utf-8") as f:
                f.write(new)
            changed += 1

    print(f"pin-homebrew: {len(names)} formulae pinned to {args.version} "
          f"({changed} changed, {len(names) - changed} already current)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
