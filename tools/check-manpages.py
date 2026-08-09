#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-manpages.py — the Makefile and Debian must install the same man pages.

The list of man pages used to exist three times: once in each install-* recipe,
again in uninstall, and a third time across debian/*.install.  The Makefile's
two copies are now one $(MAN_*) variable.  Debian's cannot join them —
dh_install reads debian/<pkg>.install from the SOURCE package before any
Makefile has run, and generating those files inside debian/rules would leave
build-modified files under debian/, which dpkg-source rejects on a 3.0 (quilt)
package.  So the third copy stays, and this compares it to the first.

Nothing else catches a disagreement.  The mandoc lint and the .TH version check
both glob man/, so they pass for a page that is written, linted, versioned and
never installed by anything.  dh_missing would notice an orphan, but only
during a full package build.

Three assertions:
  1. every man path in debian/*.install is one $(MAN_ALL) knows about;
  2. every $(MAN_ALL) entry is claimed by exactly one Debian package — none
     installed twice, none orphaned;
  3. every entry names a file that actually exists.

Run as `make check-manpages`.  Needs `make` to print the variable; no build.
"""

import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, path, Report                         # noqa: E402


def man_all(rep):
    """$(MAN_ALL) as make expands it — the list the install rules really use.

    Read from make rather than re-parsed out of the Makefile: a second parser
    would be a fourth copy of the thing this checker exists to de-duplicate.
    """
    try:
        out = subprocess.run(["make", "-s", "-C", ROOT, "print-MAN_ALL"],
                             capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        sys.exit(f"cannot read $(MAN_ALL) from the Makefile: {e}")
    pages = out.split()
    rep.expect(pages, "$(MAN_ALL) entries")
    return pages


def debian_claims(rep):
    """{man path: [debian package, ...]} from debian/*.install."""
    claims = {}
    d = path("debian")
    if not os.path.isdir(d):
        sys.exit("debian/ is missing — has the packaging moved?")
    for name in sorted(os.listdir(d)):
        if not name.endswith(".install"):
            continue
        pkg = name[:-len(".install")]
        for line in open(os.path.join(d, name), encoding="utf-8"):
            m = re.match(r"\s*usr/share/man/(man\d)/(\S+)\.gz\s*$", line)
            if m:
                claims.setdefault(f"{m.group(1)}/{m.group(2)}", []).append(pkg)
    rep.expect(claims, "debian/*.install man entries")
    return claims


def main():
    rep = Report("check-manpages")

    pages = man_all(rep)
    claims = debian_claims(rep)

    # 1. Debian installs nothing the Makefile does not build into the stage.
    for page, pkgs in sorted(claims.items()):
        rep.check(page in pages,
                  f"debian/{pkgs[0]}.install ships usr/share/man/{page}.gz, "
                  f"but $(MAN_ALL) does not list {page} — nothing installs it "
                  f"into the staging tree, so the build will fail on a missing "
                  f"file")

    # 2. Every page is claimed exactly once.
    for page in pages:
        pkgs = claims.get(page, [])
        rep.check(len(pkgs) == 1,
                  f"man/{page} is in $(MAN_ALL) but claimed by "
                  f"{len(pkgs)} Debian package(s)"
                  + (f" ({', '.join(pkgs)}) — a page installed twice"
                     if len(pkgs) > 1 else
                     " — it would be staged and then dropped from every .deb"))

    # 3. And the file is really there.
    for page in pages:
        rep.check(os.path.exists(path(os.path.join("man", page))),
                  f"$(MAN_ALL) lists {page}, but man/{page} does not exist")

    return rep.finish(f"{len(pages)} man pages across "
                      f"{len(set(p for v in claims.values() for p in v))} "
                      f"Debian packages")


if __name__ == "__main__":
    sys.exit(main())
