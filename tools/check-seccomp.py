#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-seccomp.py — every syscall the daemon's guarded calls can compile to must
be re-allowed by the unit that sandboxes it.

THE BUG THIS EXISTS FOR.  imud.service ended its filter with

    SystemCallFilter=~@privileged @resources
    SystemCallFilter=adjtimex

which is correct on x86-64 and fatal on ARM.  SystemCallFilter= takes syscall
NAMES, and systemd resolves a name against the running architecture's table —
but glibc's adjtimex() is not one syscall.  On x86-64 it issues adjtimex (159);
on arm64 that syscall does not exist and glibc issues clock_adjtime (266); on
armhf with glibc >= 2.34 it issues the time64 variant clock_adjtime64 (405).
A name that resolves to nothing allows nothing, so on arm64 the line restored a
syscall the daemon never makes while the one it does make stayed blocked by
~@privileged (clock_adjtime is in @clock, hence in @privileged).  The 1.9.0 RC
daemon was SIGSYS-killed — status=31/SYS — on every start on a Raspberry Pi 5,
before fusing a sample.

WHY A TEXT CHECK, GIVEN CI ALREADY RUNS THE BINARY.  Because every existing
test runs it WITHOUT the unit's seccomp filter, and the jobs that do exercise a
filter run on x86, where the wrong list happens to work.  ci.yml now also starts
the daemon under these very lines on an arm64 runner, which is the check with
teeth; this one is the cheap always-on companion that names the invariant in
text and runs anywhere python3 does.

WHAT IT DOES NOT DO.  It cannot tell you which syscall glibc will issue on an
architecture — that is a property of a libc and a kernel ABI, not of this tree.
It asserts the list is complete against a table maintained here, so adding a
guarded privileged call means adding a row.

Run as `make check-seccomp`.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import must_read, Report                          # noqa: E402

UNIT = "etc/imud.service.in"

# A libc call the daemon makes that lands in a filtered systemd syscall set,
# and every syscall NAME it can compile to across the architectures imud ships
# (amd64, arm64, armhf).  If a call is added to the daemon that needs a
# @privileged syscall, it belongs here and in the unit.
GUARDED_CALLS = [
    {
        "libc": "adjtimex(3)",
        "caller": "src/main.c",          # the CLOCK_TAI offset check at startup
        "set": "@clock (via @privileged)",
        "names": ["adjtimex", "clock_adjtime", "clock_adjtime64"],
        "why": "x86-64 issues adjtimex; arm64 has no such syscall and issues "
               "clock_adjtime; armhf (glibc >= 2.34) issues clock_adjtime64",
    },
]


def allowed_names(unit_text, rep):
    """Every syscall name re-allowed by a positive SystemCallFilter= line.

    Lines beginning '~' are the DENY form and are deliberately skipped: a name
    appearing there is being removed, which is the opposite of what this
    checker is looking for.  '@sets' are skipped too — a set can only be the
    thing that removed the syscall, never the thing that restores one name.
    """
    names = set()
    for m in re.finditer(r"^SystemCallFilter=(.+)$", unit_text, re.M):
        spec = m.group(1).strip()
        if spec.startswith("~"):
            continue
        for tok in spec.split():
            if not tok.startswith("@"):
                names.add(tok)
    rep.expect(names, f"{UNIT} positive SystemCallFilter= entries")
    return names


def main():
    rep = Report("check-seccomp")
    unit = must_read(UNIT, "the daemon's systemd unit")

    # The premise: something must actually be removing these, or the whole
    # re-allow list is cargo cult and this checker is guarding nothing.
    rep.check(re.search(r"^SystemCallFilter=~.*@privileged", unit, re.M) is not None,
              f"{UNIT}: no 'SystemCallFilter=~...@privileged' line — the "
              f"re-allow list below it is then pointless, and this checker "
              f"would be asserting a list nothing needs")

    allowed = allowed_names(unit, rep)

    for call in GUARDED_CALLS:
        for name in call["names"]:
            rep.check(name in allowed,
                      f"{UNIT}: {call['libc']} ({call['caller']}) is filtered "
                      f"by {call['set']} but the unit does not re-allow "
                      f"'{name}' — {call['why']}")

    n = sum(len(c["names"]) for c in GUARDED_CALLS)
    return rep.finish(f"{len(GUARDED_CALLS)} guarded call(s), "
                      f"{n} syscall name(s) across amd64/arm64/armhf")


if __name__ == "__main__":
    sys.exit(main())
