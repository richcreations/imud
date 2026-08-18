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

    return r.finish(f"{len(ids)} check ids, all described in {SPEC}")


if __name__ == "__main__":
    sys.exit(main())
