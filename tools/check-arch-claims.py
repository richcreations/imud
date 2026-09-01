#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-arch-claims.py — the CPU baseline the docs promise must be the one the
release actually builds.

Debian's armhf port and Raspberry Pi OS's 32-bit port are different
architectures wearing the same name.  Debian armhf is ARMv7-A/VFPv3-D16;
Raspbian is ARMv6/VFPv2, so that it still runs on the Pi 1 and Pi Zero/Zero W.
Both report the identical string from `dpkg --print-architecture`, which means
apt cannot tell them apart: a package built to the Debian baseline installs
without complaint on an ARMv6 board and then dies on an illegal instruction the
first time it runs.

That failure is invisible from inside this repository.  Nothing in the source
mentions a CPU baseline, `make test` cannot see it, and the shipped .deb looks
correct in every way a checker could inspect locally — the only evidence is a
build-attribute tag in an ELF header on a machine nobody here owns.  So the
baseline is a fact that lives in exactly two places and drifts silently between
them: the workflow that picks the build rootfs, and the prose that tells a user
which boards are supported.

This compares them.  Four assertions, none of which can pass vacuously:

  1. build-debs.yml declares an `arm_baseline:` for every matrix leg.  Reading
     the baseline out of the workflow rather than hardcoding it here is the
     whole point — when the armhf leg moves, this checker fails until the
     documentation follows it.

  2. Every document that states the baseline states the one the workflow
     builds.  DOCUMENTED lists them explicitly: a file joins that list by
     someone deciding it should carry the fact, never by accident.

  3. The armhf leg bootstraps from Raspbian and the arm64 leg from Debian.
     A declared baseline that its own archive cannot supply is a lie the
     first assertion would otherwise accept.

  4. The Raspbian archive key is pinned to the expected fingerprint.  Keeping
     the second copy here is deliberate: a silent key swap in the workflow is
     exactly the change that should have to be made twice.

Run as `make check-arch-claims` (via check-generated-text).
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import Report, must_read, read                    # noqa: E402

WORKFLOW = ".github/workflows/build-debs.yml"
BOOTSTRAP = "tools/bootstrap-raspbian.sh"

# Documents that carry the armhf baseline as a stated fact.  README.md is
# deliberately absent: it is the pitch and quick start, and it names the
# architectures without promising a baseline, so there is nothing here to
# contradict.
DOCUMENTED = [
    "docs/manual.md",
    "INSTALL",
    "CONTRIBUTING.md",
    "apt/README.md",
    "web/index.html",
    "docs/RELEASING.md",
]

# The archive each architecture must build against for its baseline to hold.
EXPECTED_ARCHIVE = {
    "armhf": "raspbian",
    "arm64": "debian",
}

# Raspbian archive signing key.  Verified against the InRelease signature and
# against the copy served by raspbian.raspberrypi.com, which is a different
# host from the mirror the build fetches packages from.
RASPBIAN_KEY_FPR = "A0DA38D0D76E8B5D638872819165938D90FDDD2E"


def matrix_baselines(text, rep):
    """{arch: baseline} from the workflow's matrix include: entries."""
    out = {}
    # Each include: entry opens with `- arch: <name>` and carries its keys
    # until the next such line.  Splitting on the arch key keeps this robust
    # against the order of the keys inside an entry.
    for chunk in re.split(r"\n\s*-\s+arch:\s*", text)[1:]:
        arch = chunk.split("\n", 1)[0].strip()
        m = re.search(r"^\s*arm_baseline:\s*(\S+)", chunk, re.M)
        if m:
            out[arch] = m.group(1).strip().strip("'\"")
    return out


def main():
    rep = Report("check-arch-claims")
    wf = must_read(WORKFLOW, "the build matrix defines the CPU baselines")

    baselines = matrix_baselines(wf, rep)
    rep.expect(baselines, "arm_baseline: entries in %s" % WORKFLOW)

    for arch in sorted(EXPECTED_ARCHIVE):
        if arch not in baselines:
            rep.fail("%s: matrix leg %r declares no arm_baseline:"
                     % (WORKFLOW, arch))

    armhf = baselines.get("armhf")
    if armhf:
        # The token as it is written in prose: armv6 -> ARMv6.
        token = armhf.upper().replace("ARMV", "ARMv")
        for rel in DOCUMENTED:
            text = read(rel)
            if text is None:
                rep.fail("%s: listed in DOCUMENTED but missing" % rel)
                continue
            rep.check(token in text,
                      "%s: does not state the armhf baseline %s that %s "
                      "builds — a reader cannot tell which boards are "
                      "supported" % (rel, token, WORKFLOW))

    # The declared baseline has to come from an archive that supplies it.
    for arch, archive in sorted(EXPECTED_ARCHIVE.items()):
        chunks = re.split(r"\n\s*-\s+arch:\s*", wf)[1:]
        leg = next((c for c in chunks if c.split("\n", 1)[0].strip() == arch),
                   None)
        if leg is None:
            rep.fail("%s: no matrix leg for %r" % (WORKFLOW, arch))
            continue
        rep.check(archive in leg.lower(),
                  "%s: the %s leg does not name the %s archive, so its "
                  "declared baseline is unsupported by what it builds against"
                  % (WORKFLOW, arch, archive))

    boot = must_read(BOOTSTRAP, "the armhf rootfs pins its own trust anchor")
    rep.check(RASPBIAN_KEY_FPR in boot,
              "%s: the Raspbian archive key fingerprint %s is not pinned — "
              "the armhf rootfs would trust whatever the archive serves"
              % (BOOTSTRAP, RASPBIAN_KEY_FPR))

    return rep.finish("%d architecture claims checked across %d documents"
                      % (rep.checked, len(DOCUMENTED)))


if __name__ == "__main__":
    sys.exit(main())
