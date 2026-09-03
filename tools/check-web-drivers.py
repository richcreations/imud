#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-web-drivers.py — the website's hardware section against the registry.

THE DRIFT THIS EXISTS FOR.  web/index.html is the first page anyone reads
about which parts imud drives, and nothing checked it.  Its experimental list
named nine parts while eleven were registered -- LSM6DSOX and RM3100 had been
added to the tree and never to the page -- and the sentence beside it said
"Ten of the twelve sensor drivers have never run on physical silicon" against
a real count of twelve of fourteen.

Two drivers and two numbers, wrong on the page a prospective user reads BEFORE
the manual.  check-cli-docs.py already covers the same file, but only for CLI
flags, so it looked guarded and was not.

WHAT IS COMPARED.  Every registered driver has a part designator -- the last
whitespace-separated token of its `chip` field in docs/driver-notes.toml, so
'TDK MPU-6500' is MPU-6500 -- and the page sorts those designators into two
definition-list entries:

  <dt>Reference</dt>              the parts with experimental = false
  <dt>Experimental drivers</dt>   the parts with experimental = true

Membership is checked BOTH ways.  A part missing from its list is the drift
above; a part in the wrong list is the one that comes next, when an
imud-imutest report clears a flag in the ops struct and the page keeps
advertising the part as unproven.

`sim` is excluded: its chip field is an em dash, it has no silicon, and the
page gives it a <dt> of its own.

WORD BOUNDARIES MATTER HERE.  LSM6DSO is a prefix of LSM6DSOX, so a substring
test passes on a page that lists only the longer part -- which is exactly the
pair that drifted.  The designator must not be followed by another
alphanumeric or a hyphen.

THE COUNT SENTENCE is prose, and prose is where the claim a reader believes
lives.  It carries both numbers, so both are checked against the registry.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import tomllib
except ModuleNotFoundError:                                     # pragma: no cover
    sys.exit("docs/driver-notes.toml needs python3.11+ for tomllib")

import driverlib                                                # noqa: E402
from checklib import ROOT, Report, must_read                    # noqa: E402

WEB = "web/index.html"
NOTES = "docs/driver-notes.toml"

# Driver names with no part behind them, so no designator to find on the page.
NO_SILICON = {"sim"}

NUMBER = {
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12,
    "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
    "seventeen": 17, "eighteen": 18, "nineteen": 19, "twenty": 20,
}

# "Twelve of the fourteen sensor drivers have never run on physical silicon."
COUNT_RE = re.compile(
    r"\b(\d+|" + "|".join(NUMBER) + r")\s+of\s+the\s+"
    r"(\d+|" + "|".join(NUMBER) + r")\s+sensor drivers\b", re.I)


def number(tok):
    return int(tok) if tok.isdigit() else NUMBER[tok.lower()]


def dd_for(text, label, rep):
    """The <dd> body following <dt>label</dt>, tags and entities flattened."""
    m = re.search(r"<dt>\s*" + re.escape(label) + r"\s*</dt>\s*<dd>(.*?)</dd>",
                  text, re.S | re.I)
    if not m:
        rep.fail(f"{WEB}: no <dt>{label}</dt> with a <dd> after it — the "
                 f"hardware section moved and this checker cannot see it")
        return ""
    body = re.sub(r"<[^>]+>", " ", m.group(1))
    rep.expect(body.strip(), f"{WEB} <dd> for {label}")
    return body


def listed(body, part):
    """Is `part` named in `body` as a whole designator?"""
    return re.search(re.escape(part) + r"(?![0-9A-Za-z-])", body) is not None


def main():
    rep = Report("check-web-drivers")

    with open(os.path.join(ROOT, NOTES), "rb") as fh:
        notes = tomllib.load(fh)["drivers"]

    # Registry order, deduplicated by name: sim is two ops structs and every
    # other driver is one, and a designator is a property of the part.
    parts, seen = [], set()
    for d in driverlib.drivers(rep):
        if d["name"] in seen or d["name"] in NO_SILICON:
            continue
        seen.add(d["name"])
        note = notes.get(d["name"])
        if not note:
            rep.fail(f"{NOTES}: no entry for driver {d['name']}, so its part "
                     f"designator is unknown and {WEB} cannot be checked "
                     f"against it")
            continue
        parts.append((d["name"], note["chip"].split()[-1], d["experimental"]))
    rep.expect(parts, "registered drivers with a part designator")

    text = must_read(WEB, "the website's hardware section")
    ref = dd_for(text, "Reference", rep)
    exp = dd_for(text, "Experimental drivers", rep)

    for name, part, experimental in parts:
        want, other = (exp, ref) if experimental else (ref, exp)
        wname = "Experimental drivers" if experimental else "Reference"
        oname = "Reference" if experimental else "Experimental drivers"
        rep.check(listed(want, part),
                  f"{WEB}: {part} ({name}) is not named under <dt>{wname}</dt>, "
                  f"but {name}_ops sets experimental = "
                  f"{'true' if experimental else 'false'}")
        rep.check(not listed(other, part),
                  f"{WEB}: {part} ({name}) is named under <dt>{oname}</dt>, "
                  f"which contradicts experimental = "
                  f"{'true' if experimental else 'false'} in {name}_ops")

    n_exp = sum(1 for _, _, e in parts if e)
    m = COUNT_RE.search(text)
    if not m:
        rep.fail(f"{WEB}: no \"N of the M sensor drivers\" sentence — it is "
                 f"the only place the page states how much of the hardware "
                 f"support is unproven, and it is not generated")
    else:
        rep.check(number(m.group(1)) == n_exp,
                  f"{WEB}: says {m.group(1)} sensor drivers have never run on "
                  f"silicon; {n_exp} ops structs set experimental = true")
        rep.check(number(m.group(2)) == len(parts),
                  f"{WEB}: counts {m.group(2)} sensor drivers; the registry "
                  f"has {len(parts)} with a part behind them "
                  f"({', '.join(sorted(NO_SILICON))} excluded)")

    return rep.finish(f"{len(parts)} parts across 2 lists in {WEB}, "
                      f"{n_exp} experimental")


if __name__ == "__main__":
    sys.exit(main())
