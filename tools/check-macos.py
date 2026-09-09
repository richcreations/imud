#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-macos.py — the macOS support surface: the launchd jobs, and the version
range the docs promise.

Two failures this exists for, neither of which any other gate can see.

The first is a launchd job drifting away from the systemd unit beside it.
They describe the same daemon twice, in two formats, with no generator
between them — so a new bridge, a renamed binary or a moved config path
lands in one and not the other, and the miss shows up as a service that
silently never starts on the one host nobody develops on. `make install`
picks the file by uname, so a broken plist is invisible on Linux and a
broken unit is invisible on a Mac.

The second is the stated version range outliving its runners. The docs
promise macOS 14 Sonoma through macOS 26 Tahoe; CI proves it by running the
suite on both ends. GitHub retires runner images on its own schedule — 13
is already gone — so the day macos-14 goes, the promise becomes untestable
prose unless something compares the two. That is the same drift
check-arch-claims.py exists for on the armhf side.

Run as `make check-macos` (via check-generated-text). Pure text analysis
plus plistlib, no build.
"""

import glob
import os
import plistlib
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import Report, must_read, path, read           # noqa: E402

MAKEFILE = "Makefile"
WORKFLOW = ".github/workflows/ci.yml"

# Documents that carry the supported macOS range as a stated fact. A file
# joins this list by someone deciding it should carry the fact, never by
# accident — the same rule as check-arch-claims.py's DOCUMENTED.
DOCUMENTED = ["INSTALL", "docs/manual.md"]

# "macOS 14 Sonoma through macOS 26 Tahoe" — the codenames are required so
# the sentence reads as prose rather than a version tuple, but only the
# numbers are compared.  \s+ throughout because both documents are hard
# wrapped and the sentence spans a line break in each.
RANGE_RE = re.compile(
    r"macOS\s+(\d+)\s+[A-Z][a-z]+\s+through\s+macOS\s+(\d+)\s+[A-Z][a-z]+")


def unit_name(p):
    """imud-signalk, from etc/imud-signalk.service.in or …plist.in."""
    return os.path.basename(p).split(".")[0]


def check_parity(rep):
    """Every systemd unit has a launchd job and back."""
    units = {unit_name(p) for p in glob.glob(path("etc/*.service.in"))}
    jobs = {unit_name(p) for p in glob.glob(path("etc/*.plist.in"))}
    rep.expect(units, "etc/*.service.in")
    rep.expect(jobs, "etc/*.plist.in")

    for name in sorted(units - jobs):
        rep.fail(f"etc/{name}.plist.in: missing — etc/{name}.service.in has "
                 f"no launchd counterpart, so `make install` on macOS "
                 f"installs nothing for it")
    for name in sorted(jobs - units):
        rep.fail(f"etc/{name}.service.in: missing — etc/{name}.plist.in has "
                 f"no systemd counterpart")
    return sorted(units & jobs)


def exec_start(rep, name):
    """(binary, config) from the unit's ExecStart=, or None."""
    text = must_read(f"etc/{name}.service.in")
    m = re.search(r"^ExecStart=(\S+)\s+--config\s+(\S+)\s*$", text, re.M)
    if not m:
        rep.fail(f"etc/{name}.service.in: no ExecStart= of the expected "
                 f"shape (BINARY --config PATH)")
        return None
    return m.group(1), m.group(2)


def check_job(rep, name, prefix):
    """One plist: it parses, and it says what the unit says."""
    raw = read(f"etc/{name}.plist.in")
    try:
        job = plistlib.loads(raw.encode("utf-8"))
    except Exception as exc:                       # noqa: BLE001 — any parse error
        rep.fail(f"etc/{name}.plist.in: not a readable property list ({exc})")
        return
    if not isinstance(job, dict):
        rep.fail(f"etc/{name}.plist.in: top level is "
                 f"{type(job).__name__}, not a dict")
        return

    rep.check(job.get("Label") == f"{prefix}.{name}",
              f"etc/{name}.plist.in: Label is {job.get('Label')!r}, but the "
              f"Makefile installs it as {prefix}.{name}.plist — launchd keys "
              f"the job by Label, so the two must agree")

    # RunAtLoad, or bootstrapping the job does not start it; KeepAlive with
    # SuccessfulExit false, or a clean shutdown respawns for ever.
    rep.check(job.get("RunAtLoad") is True,
              f"etc/{name}.plist.in: RunAtLoad is not true, so the job would "
              f"not start at boot")
    keep = job.get("KeepAlive")
    rep.check(isinstance(keep, dict) and keep.get("SuccessfulExit") is False,
              f"etc/{name}.plist.in: KeepAlive must be a dict with "
              f"SuccessfulExit=false, to match the unit's "
              f"Restart=on-failure; got {keep!r}")

    started = exec_start(rep, name)
    if started is None:
        return
    unit_bin, unit_conf = started
    argv = job.get("ProgramArguments")
    if not isinstance(argv, list) or len(argv) != 3 or argv[1] != "--config":
        rep.fail(f"etc/{name}.plist.in: ProgramArguments is {argv!r}, "
                 f"expected [BINARY, '--config', PATH]")
        return

    rep.check(argv[0] == unit_bin,
              f"etc/{name}.plist.in: runs {argv[0]}, but "
              f"etc/{name}.service.in runs {unit_bin}")
    # The unit hardcodes /etc/imud; the plist takes @ETCDIR@ because launchd
    # has no drop-in and the whole path has to be right at install time. Only
    # the basename can be compared.
    rep.check(os.path.basename(argv[2]) == os.path.basename(unit_conf),
              f"etc/{name}.plist.in: reads {argv[2]}, but "
              f"etc/{name}.service.in reads {unit_conf}")


def check_version_range(rep):
    """The promised range is the one CI actually runs."""
    ci = must_read(WORKFLOW)
    block = re.search(r"^  macos:$.*?^  \w", ci, re.M | re.S)
    labels = re.findall(r"macos-(\d+)(?:-\w+)?",
                        block.group(0) if block else "")
    versions = sorted({int(v) for v in labels})
    if not rep.expect(versions, f"macos-* runner labels in {WORKFLOW}"):
        return

    for rel in DOCUMENTED:
        text = must_read(rel)
        found = RANGE_RE.search(text)
        if not found:
            rep.fail(f"{rel}: states no supported macOS range — expected a "
                     f"sentence of the form 'macOS N Codename through "
                     f"macOS M Codename'")
            continue
        lo, hi = int(found.group(1)), int(found.group(2))
        rep.check(lo == versions[0],
                  f"{rel}: promises macOS {lo} as the floor, but the oldest "
                  f"macOS runner in {WORKFLOW} is {versions[0]} — nothing "
                  f"tests {lo}")
        rep.check(hi == versions[-1],
                  f"{rel}: promises macOS {hi} as the ceiling, but the "
                  f"newest macOS runner in {WORKFLOW} is {versions[-1]}")


def main():
    rep = Report("check-macos")

    mk = must_read(MAKEFILE)
    m = re.search(r"^LAUNCHD_PREFIX\s*=\s*(\S+)$", mk, re.M)
    if not m:
        sys.exit(f"{MAKEFILE}: no LAUNCHD_PREFIX — this checker reads the "
                 f"label prefix from the Makefile that installs it")
    prefix = m.group(1)

    for name in check_parity(rep):
        check_job(rep, name, prefix)
    check_version_range(rep)

    return rep.finish("%d assertions across the launchd jobs and the macOS "
                      "range" % rep.checked)


if __name__ == "__main__":
    sys.exit(main())
