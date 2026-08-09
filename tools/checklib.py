#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
checklib.py — shared plumbing for the tools/check-*.py family.

Two jobs, both of which exist because the family grew from three checkers to
eleven and the copies started to differ:

  1. One definition of what a checker's output looks like.  A drift report is
     read by someone who did not write the checker, so "FAIL <where>: <what>"
     and a closing count are a contract, not a style preference.

  2. One definition of where the repo is — honouring $IMUD_ROOT so a checker
     can be pointed at a fixture tree.  test/test_checkers.py copies the real
     tree, reintroduces a drift that actually shipped, and asserts the checker
     catches it.  Without the override, a checker can only ever be tested
     against a clean tree, which proves it does not false-positive and says
     nothing at all about whether it can detect the thing it exists for.

Import from a sibling in tools/:

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from checklib import ROOT, read, must_read, Report
"""

import os
import re
import sys

# $IMUD_ROOT lets the tests aim a checker at a fixture tree; unset it is the
# repo this file lives in.
ROOT = os.environ.get("IMUD_ROOT") or os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))


def path(rel):
    """Absolute path of a repo-relative file."""
    return os.path.join(ROOT, rel)


def read(rel):
    """File contents, or None when it does not exist."""
    p = path(rel)
    if not os.path.exists(p):
        return None
    return open(p, encoding="utf-8").read()


def must_read(rel, why=""):
    """File contents, or exit — for a file the checker cannot work without."""
    text = read(rel)
    if text is None:
        sys.exit(f"{rel}: missing{' — ' + why if why else ''}")
    return text


def expand_shorthands(text):
    """Resolve the ways the docs abbreviate families of sibling names.

    A topic tree or metric list written out in full does not fit in 80 columns
    and reads worse when it does, so the docs group siblings:

        imud/attitude/{roll,pitch,yaw}                 config/imud-mqtt.conf
        `imud_gyro_bias_{x,y,z}_radians_per_second`    prometheus spec
        `imud/attitude/roll` · `/pitch` · `/yaw`       mqtt spec

    A checker that cannot follow these reports correct documentation as
    missing, which is how a checker gets switched off.  Returns the original
    text with the expansions appended, so plain substring search finds both
    the shorthand and what it stands for.
    """
    out = []

    # name{a,b,c}suffix
    for m in re.finditer(r"([\w/.-]*)\{([^}]*)\}([\w/.-]*)", text):
        head, alts, tail = m.group(1), m.group(2), m.group(3)
        for alt in alts.split(","):
            out.append(f"{head}{alt.strip()}{tail}")

    # `full/path` · `/leaf` — the leaf inherits the previous path's parent.
    prev = None
    for m in re.finditer(r"`([^`]+)`", text):
        token = m.group(1).strip()
        if token.startswith("/") and prev and "/" in prev:
            out.append(prev.rsplit("/", 1)[0] + token)
        elif "/" in token:
            prev = token.rstrip("/")

    return text + "\n" + "\n".join(out)


class Report:
    """Failure accumulator with the family's output format.

    A checker that matches nothing must not pass silently: `expect()` records
    that a extractor found something, and finish() fails when one did not.
    check-flags.py already refuses to pass vacuously; this generalises it,
    because a regex that stops matching after a refactor is indistinguishable
    from a clean tree unless something says otherwise.
    """

    def __init__(self, name):
        self.name = name
        self.failures = []
        self.checked = 0
        self.empty = []

    def fail(self, msg):
        self.failures.append(msg)

    def check(self, ok, msg):
        """Count an assertion, record a failure when it does not hold."""
        self.checked += 1
        if not ok:
            self.failures.append(msg)
        return ok

    def expect(self, found, what):
        """Assert an extractor produced something. Guards vacuous passes."""
        if not found:
            self.empty.append(what)
        return found

    def finish(self, summary):
        for what in self.empty:
            self.failures.append(
                f"extracted nothing for {what} — the source moved and this "
                f"checker is now asserting nothing")
        if self.failures:
            for f in self.failures:
                print(f"FAIL {f}", file=sys.stderr)
            print(f"\n{len(self.failures)} problem(s) — {summary}",
                  file=sys.stderr)
            return 1
        print(f"{self.name}: {summary}")
        return 0
