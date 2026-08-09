#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
test_checkers.py — the doc checkers must actually detect drift.

A checker run against a clean tree proves one thing: it does not false-
positive.  It says nothing about whether it can detect the drift it exists
for, and a regex that quietly stopped matching after a refactor looks exactly
like a clean tree.  Every checker in tools/ is a regex over source, so this is
not a hypothetical failure mode — four of them mis-parsed a correct document
during development, and the reverse is just as easy.

Method: copy the tree, break one specific fact, and assert the checker emits a
finding it was NOT emitting before.  Comparing against the pristine run rather
than against zero means these tests work whether or not the tree has been
cleaned up yet, and keeps passing afterwards.

Mutations are deliberately distinctive (99, ZZZZZ) rather than replays of the
drifts that shipped: a mutation that happens to match the current state would
produce no new finding and the test would fail for the wrong reason.

Run as `make test-tools`, or directly.  Needs git and python3, nothing else.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOOLS = os.path.join(ROOT, "tools")


# ── the cases ────────────────────────────────────────────────────────────────
# (checker, file to break, how to break it, what the finding must mention)

def sub(pattern, repl, count=1):
    """Regex substitution, asserting it actually changed something."""
    def apply(text):
        # re.M throughout: every fixture pattern anchors to a line, not to the
        # start of a 10 000-line file.
        new, n = re.subn(pattern, repl, text, count=count, flags=re.M)
        if n == 0:
            raise AssertionError(f"fixture pattern never matched: {pattern!r}")
        return new
    return apply


CASES = [
    ("check-packet", "include/types.h",
     sub(r"#define\s+IMUD_VERSION\s+\d+", "#define IMUD_VERSION  99"),
     "wire version"),

    ("check-packet", "spec.md",
     sub(r"^(\s*8\s+)8(\s+uint64\s+ts_wall_ns)", r"\g<1>4\g<2>"),
     "ts_wall_ns"),

    ("check-nmea", "src/nmea.c",
     sub(r'"\$?PASHR', '"$ZZZZZ'),
     "ZZZZZ"),

    ("check-drivers", "src/drivers/ism330dhcx.c",
     sub(r"\.supported_accel_g\s*=\s*\{[^}]*\}",
         ".supported_accel_g  = { 2, 4, 8, 16, 32, 0 }"),
     "accel_g"),

    ("check-mqtt-topics", "src/mqtt_publish.c",
     sub(r'\{ "imu/temperature"', '{ "imu/zzztemp"'),
     "imu/zzztemp"),

    ("check-cli-docs", "src/cli.c",
     sub(r'strcmp\(argv\[i\], "--no-nmea"\)', 'strcmp(argv[i], "--no-zzz")'),
     "--no-zzz"),

    ("check-bridge-outputs", "src/prom_metrics.c",
     sub(r'GAUGE\("imud_up"', 'GAUGE("imud_zzz_total"'),
     "imud_zzz_total"),

    ("check-libimud-api", "lib/libimud.map",
     sub(r"imud_lib_version;", "imud_lib_version;\n        imud_zzz;"),
     "imud_zzz"),

    ("check-math-citations", "docs/math.md",
     sub(r"`src/fusion\.c`", "`src/no_such_file.c`"),
     "no_such_file"),

    ("check-links", "docs/manual.md",
     sub(r"\]\(#6-calibration\)", "](#no-such-heading)"),
     "no-such-heading"),

    # A page the Makefile stages and no Debian package claims: staged, then
    # dropped from every .deb.  dh_missing only notices during a full package
    # build, which is why this is checked here.
    ("check-manpages", "debian/imud.install",
     sub(r"^usr/share/man/man1/imud-status\.1\.gz\n", ""),
     "imud-status.1"),

    # A config key whose documented default drifted from the registry.
    ("gen-config-docs", "man/man5/imud.conf.5",
     sub(r"^\.RI \(int,\\ default:\\ 833\)$", r".RI (int,\\ default:\\ 999)"),
     "odr_hz"),

    # ...and the same on the markdown surface.
    ("gen-config-docs", "docs/manual.md",
     sub(r"^\| `fifo_wm` \| int \| `64` \|", "| `fifo_wm` | int | `128` |"),
     "fifo_wm"),

    # And the reverse: Debian shipping a page the Makefile never stages.
    ("check-manpages", "debian/imud-utils.install",
     sub(r"^usr/share/man/man1/imud-mon\.1\.gz$",
         "usr/share/man/man1/imud-mon.1.gz\nusr/share/man/man9/imud-zzz.9.gz"),
     "imud-zzz"),
]


def findings(checker, root):
    """The set of FAIL lines a checker emits against `root`."""
    env = dict(os.environ, IMUD_ROOT=root)
    p = subprocess.run([sys.executable, os.path.join(TOOLS, f"{checker}.py")],
                       capture_output=True, text=True, env=env)
    if p.returncode not in (0, 1):
        raise AssertionError(
            f"{checker} exited {p.returncode} (not a pass/fail):\n{p.stderr}")
    return {l for l in p.stderr.split("\n") if l.startswith("FAIL")}


def main():
    with tempfile.TemporaryDirectory(prefix="imud-checkers-") as tmp:
        pristine = os.path.join(tmp, "pristine")
        os.makedirs(pristine)

        # The fixture is the WORKING TREE, not HEAD.  These checkers are the
        # gate for changes that have not been committed yet — a fixture built
        # from `git archive HEAD` cannot see the checker being added or the
        # Makefile variable it reads, so the test silently exercises the old
        # tree and reports a detection failure that is really a fixture bug.
        #
        # --cached --others --exclude-standard is tracked files plus new ones,
        # minus anything gitignored: what a commit would contain, as it stands
        # right now.
        listing = subprocess.run(
            ["git", "-C", ROOT, "ls-files", "-z",
             "--cached", "--others", "--exclude-standard"],
            capture_output=True, check=True).stdout
        for rel in listing.decode().split("\0"):
            if not rel:
                continue
            src = os.path.join(ROOT, rel)
            if not os.path.isfile(src):       # deleted-but-still-indexed
                continue
            dst = os.path.join(pristine, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(src, dst)

        base = {}
        for checker, *_ in CASES:
            if checker not in base:
                base[checker] = findings(checker, pristine)

        failures = 0
        for checker, target, mutate, expect in CASES:
            work = os.path.join(tmp, "work")
            shutil.rmtree(work, ignore_errors=True)
            shutil.copytree(pristine, work, symlinks=True)

            path = os.path.join(work, target)
            text = open(path, encoding="utf-8").read()
            try:
                open(path, "w", encoding="utf-8").write(mutate(text))
            except AssertionError as e:
                print(f"FAIL {checker} [{target}]: {e}", file=sys.stderr)
                failures += 1
                continue

            new = findings(checker, work) - base[checker]
            hit = [l for l in new if expect in l]
            if hit:
                print(f"ok   {checker:22} {target:28} detects {expect!r}")
            else:
                failures += 1
                print(f"FAIL {checker:22} {target:28} did NOT detect "
                      f"{expect!r} after mutation", file=sys.stderr)
                for l in sorted(new)[:3]:
                    print(f"       new finding: {l}", file=sys.stderr)

        if failures:
            print(f"\n{failures} of {len(CASES)} checkers failed to detect "
                  f"their drift", file=sys.stderr)
            return 1
        print(f"\ntest_checkers: {len(CASES)} drifts, all detected")
        return 0


if __name__ == "__main__":
    sys.exit(main())
