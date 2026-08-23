#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-imutest-checks.py — every verdict imud-imutest can print must be
described in docs/imud-utils/spec.md.

A check ID is what an operator reads off a report and has to act on, and the
spec's check tables are the only place saying what each one means. Nothing
compared the two, so a check added to src/imutest.c simply did not appear
there: `mag.drdy.restore` shipped undescribed, and it is the one line in a
report that tells the reader to distrust everything below it. Eight more had
been missing for longer.

The spec tables are editorial — they group siblings and use placeholders,
because a table with one row per ID would be unreadable — so this resolves the
three shorthands they use rather than demanding every ID appear verbatim. A
checker that reports correct documentation as missing is a checker that gets
switched off:

  * dot-leaf      `imu.chipts.monotonic` / `.rate` / `.wall`
  * placeholder   `gyro.A.sign` covers gyro.x/y/z.sign; `face.N.sign` any face
  * mag-equivalent  the `mag.*` row says "the magnetometer equivalents", so a
                    mag.X whose imu.X is documented is covered by it. A mag
                    check with NO imu counterpart is not an equivalent of
                    anything and must be named — which is exactly the class
                    mag.drdy.restore fell into.

Run as `make check-imutest-checks` (via check-generated-text).
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import Report, must_read                        # noqa: E402

SRC = "src/imutest.c"
SPEC = "docs/imud-utils/spec.md"

# add_check(r, "id", ... ) / skip_check(r, "id", ... )
ID_RE = re.compile(r'(?:add_check|skip_check)\s*\(\s*r\s*,\s*"([^"]+)"')

# Single-letter placeholders the guided tables use for an axis or a face index.
PLACEHOLDER = re.compile(r"^(.*\.)[A-Z](\..*)$")


def documented_ids(spec):
    """Every id the spec names, with its shorthands resolved."""
    out = set()
    prev = None
    for m in re.finditer(r"`(\.?[A-Za-z][A-Za-z0-9_.]*)`", spec):
        tok = m.group(1)
        if tok.startswith("."):
            # dot-leaf: inherits the previous id's parent
            if prev and "." in prev:
                out.add(prev.rsplit(".", 1)[0] + tok)
            continue
        out.add(tok)
        if "." in tok:
            prev = tok
    return out


def row_ids(spec):
    """Ids that head a table row — `| `imu.odr` | ... |`.

    The reverse check uses these rather than every backticked token, because
    the prose names files and config keys in the same namespaces and those are
    not checks.
    """
    return set(re.findall(r"^\|\s*`([A-Za-z][A-Za-z0-9_.*]*)`\s*\|",
                          spec, re.M))


def covered(cid, documented, imu_suffixes):
    if cid in documented:
        return True
    # A placeholder row (gyro.A.sign) covers a concrete id (gyro.x.sign).
    for d in documented:
        m = PLACEHOLDER.match(d)
        if m and re.fullmatch(re.escape(m.group(1)) + r"[a-z0-9]+"
                              + re.escape(m.group(2)), cid):
            return True
    # The mag.* row covers a magnetometer equivalent of a documented imu check.
    if cid.startswith("mag.") and cid[4:] in imu_suffixes:
        return True
    return False


def main():
    r = Report("check-imutest-checks")
    src = must_read(SRC, "the check ids imud-imutest can emit")
    spec = must_read(SPEC, "the check tables")

    ids = sorted(set(ID_RE.findall(src)))
    r.expect(ids, f"{SRC} add_check/skip_check call sites")

    documented = documented_ids(spec)
    r.expect(documented, f"{SPEC} check ids")

    imu_suffixes = {i[4:] for i in ids if i.startswith("imu.")
                    and covered(i, documented, set())}

    for cid in ids:
        r.check(covered(cid, documented, imu_suffixes),
                f"{SRC} can report `{cid}`, which {SPEC} does not describe. "
                f"A check id is what an operator reads off the report and acts "
                f"on; add a row, or fold it into a sibling's row by name.")

    # And the other direction.  A row naming a check the tool cannot emit
    # promises the operator something that will never appear in a report --
    # `mag.bus.integrity` was documented as "the mag equivalent" of the IMU
    # check and simply did not exist, which is how a gap in coverage reads as
    # covered.
    #
    # Only ROW ids count, not every backticked token: the prose legitimately
    # names files (`imu.c`) and config keys (`mag.int_gpio`) that share the
    # namespaces and are not checks.
    #
    # The emitted set is wider than ID_RE here, because an id can reach
    # add_check through a helper -- check_burst_framing(r, "mag.burst_framing",
    # ...) is one -- so any check-id-shaped literal in the source counts.
    namespaces = {i.split(".", 1)[0] for i in ids if "." in i}
    literal = re.findall(r'"([a-z][a-z0-9]*\.[a-z0-9_.]+)"', src)
    emitted = set(ids) | {x for x in literal if x.split(".", 1)[0] in namespaces}

    for d in sorted(row_ids(spec)):
        # A check id always has a namespace and a dot; single-token rows
        # come from the status legend and other tables.
        if "." not in d or "*" in d or d in emitted:
            continue
        # A placeholder row (gyro.A.scale) stands for ids built at runtime from
        # an axis or face index, which never appear as literals.  The forward
        # direction already proves every id actually emitted has a row.
        if PLACEHOLDER.match(d):
            continue
        r.check(False,
                f"{SPEC} has a row for `{d}`, which {SRC} never reports. A "
                f"documented check that does not exist is worse than an "
                f"undocumented one: it reads as coverage. Implement it, or "
                f"drop the row.")

    # Rows are not the only way the spec promises a check.  A row's prose can
    # name a sibling -- "The mag equivalent is `mag.bus.integrity`" -- and that
    # sentence is a promise to the operator in exactly the way a row is.  It is
    # also the case that actually happened: the sentence was written, the check
    # was never implemented, and scoping this to rows alone would not have
    # caught it.  Matching the phrasing keeps it precise; prose that merely
    # mentions an id in passing is not making a claim about its existence.
    for m in re.finditer(r"(?:mag|magnetometer) equivalent[^`.]*is\s+"
                         r"`([a-z][a-z0-9]*\.[a-z0-9_.]+)`", spec):
        eq = m.group(1)
        r.check(eq in emitted,
                f"{SPEC} says the mag equivalent is `{eq}`, but {SRC} never "
                f"reports it. Implement it, or stop promising it.")

    return r.finish(f"{len(ids)} check ids, all described in {SPEC}, "
                    f"and every documented row reachable")


if __name__ == "__main__":
    sys.exit(main())
