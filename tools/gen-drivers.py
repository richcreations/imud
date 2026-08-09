#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
gen-drivers.py — render the driver table in docs/manual.md §5.

man/man5/imud.conf.5 told operators

    ISM330DHCX supports: 12, 26, 52, 104, 208, 416, 833, 1660.

while ism330dhcx.c had `.supported_odr_hz = { ..., 1660, 3332, 6664, 0 }` —
for a setting whose whole point is picking a rate the chip can produce.
check-drivers.py caught that; this makes it unsayable, and retires it.

Adding a driver used to mean hand-editing six files.  It now means writing
src/drivers/<part>.c, registering it in src/drivers.c, adding an entry to
docs/driver-notes.toml, and running `make docs-tables`: the driver table, the
driver-name lists in the [imu]/[mag] key documentation, and the supported-rate
lists all follow from the code.

Four of the table's seven columns come from the ops initialiser — the name,
the type (which registry it is in), the SPI mode and clock, and the
*Experimental.* marker.  The other three are prose about the physical part and
live in the sidecar.  The split is the point: the columns that restate the
code are generated, the columns that describe the world are written.

docs/datasheets.md is deliberately NOT generated.  It merges LSM6DSO with
LSM6DSOX and MPU-9250 with MPU-9255 into one row each, omits `sim` (which
drives no silicon), and says in its own text that it covers parts researched
*without* drivers too.  Its row set is an editorial grouping of parts, not a
projection of the driver list, and a generator would have to be told the
grouping anyway.  check-drivers.py never covered it either.

  --write   update docs/manual.md.
  (default) compare against what is on disk and report.
"""

import difflib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, must_read, splice, Report            # noqa: E402
import driverlib                                                # noqa: E402

try:
    import tomllib
except ModuleNotFoundError:                                     # pragma: no cover
    sys.exit("docs/driver-notes.toml needs python3.11+ for tomllib")

NOTES = "docs/driver-notes.toml"
MANUAL = "docs/manual.md"

BEGIN = "<!-- BEGIN GENERATED: %s -->"
END = "<!-- END GENERATED: %s -->"


def spi_cell(d, override):
    """The SPI column: what `[imu] bus` / `[mag] bus` will accept.

    A part with no SPI port, or one whose SPI mode costs something imud needs,
    carries its reason from the sidecar — that is a fact about silicon.  The
    rest is the ops initialiser read back: mode and clock, in bold when the
    driver is not experimental, which is how the table marks the two that have
    been exercised on both transports.
    """
    if override:
        return override
    if not d["spi_capable"]:
        return "no"
    mhz = int(d["spi_max_hz"]) // 1000000
    yes = "**yes**" if not d["experimental"] else "yes"
    return "%s — mode %s, %d MHz" % (yes, d["spi_mode"], mhz)


def table(ds, notes, rep):
    """The manual's §5 driver table, one row per driver name."""
    kinds = {}
    for d in ds:
        kinds.setdefault(d["name"], set()).add(d["kind"])

    # Registry order within a type, but the types grouped: every IMU, then
    # every magnetometer, then anything registered as both.  That is how the
    # table has always read and how a reader scans it — `sim` sits at the end
    # rather than in the middle because it is not a part you would compare
    # against the others.
    order, seen = [], []
    for want in ({"imu"}, {"mag"}, {"imu", "mag"}):
        for d in ds:
            if d["name"] not in seen and kinds[d["name"]] == want:
                seen.append(d["name"])
                order.append(d)

    lines = ["| Driver name | Chip | Type | I²C address | GPIO interrupt "
             "| SPI | Notes |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for d in order:
        note = notes.get(d["name"])
        if note is None:
            rep.fail(f"{NOTES}: no entry for driver '{d['name']}' "
                     f"(registered in {d['where']})")
            continue
        # Which registries list it, not a field anyone maintains: `sim` is in
        # both, and that IS its type.
        kind = kinds[d["name"]]
        typ = ("IMU + Magnetometer" if kind == {"imu", "mag"}
               else "IMU" if kind == {"imu"} else "Magnetometer")
        body = note["notes"]
        if d["experimental"]:
            body = "*Experimental.* " + body
        lines.append("| `%s` | %s | %s | %s | %s | %s | %s |" % (
            d["name"], note["chip"], typ, note["i2c_addr"], note["gpio"],
            spi_cell(d, note["spi_override"]), body))

    for name in sorted(set(notes) - set(seen)):
        rep.fail(f"{NOTES}: has an entry for '{name}', which src/drivers.c "
                 f"does not register")
    return "\n".join(lines)


REGISTRY = "docs/config-keys.toml"


def check_name_lists(ds, rep):
    """The `[imu] driver` / `[mag] driver` prose must name every driver.

    Not generated, and deliberately.  The roff is a sentence, not a table —
    `.BR a ,` repeated, then `or`, then a last item that carries the closing
    punctuation and, for [imu], a parenthetical about `sim` needing no
    hardware.  Generating it would mean teaching a tool English punctuation to
    save a one-line edit, and the byte-for-byte identity that makes every
    other region in this stage reviewable would go with it.

    So it stays hand-written and is checked instead, in both directions: a
    driver added to src/drivers.c and not to the prose fails here, and so does
    prose naming a driver that no longer exists.  Same guarantee, no
    punctuation engine.
    """
    with open(os.path.join(ROOT, REGISTRY), "rb") as fh:
        reg = tomllib.load(fh)

    known = {d["name"] for d in ds}
    for section in reg["section"]:
        if section["name"] not in ("imu", "mag"):
            continue
        entry = next((k for k in section["key"] if k["names"] == ["driver"]),
                     None)
        if entry is None:
            rep.fail(f"{REGISTRY}: [{section['name']}] has no `driver` key")
            continue
        want = driverlib.names(ds, section["name"])
        # Whole words only: `lsm6dso` is a prefix of `lsm6dsox`, and a
        # substring test would call a page complete that names only one.
        named = set(re.findall(r"\b([a-z][a-z0-9]{2,})\b", entry["man"]))
        for name in want:
            rep.check(name in named,
                      f"{REGISTRY}: [{section['name']}] driver is registered "
                      f"in src/drivers.c but not named in the key's prose: "
                      f"'{name}'")
        for name in sorted(named & known - set(want)):
            rep.fail(f"{REGISTRY}: [{section['name']}] driver names '{name}', "
                     f"which is registered as a {'mag' if section['name'] == 'imu' else 'imu'} "
                     f"driver, not a {section['name']} one")

        # "Drivers other than X and Y are experimental" — the one claim in
        # that prose that is a fact about the ops initialisers.
        reference = [d["name"] for d in ds
                     if d["kind"] == section["name"] and not d["experimental"]]
        m = re.search(r"Drivers other than(.*?)are experimental",
                      entry["man"], re.S)
        if not m:
            rep.fail(f"{REGISTRY}: [{section['name']}] driver no longer says "
                     f"which drivers are experimental")
            continue
        # Intersected with the known names: the sentence contains ordinary
        # English ("and", "are") that a bare word scan would read as drivers.
        claimed = set(re.findall(r"\b([a-z][a-z0-9]{2,})\b", m.group(1))) & known
        rep.check(claimed == set(reference),
                  f"{REGISTRY}: [{section['name']}] driver exempts "
                  f"{sorted(claimed)} from 'experimental', but "
                  f".experimental = false on {sorted(set(reference))}")


def main():
    write = "--write" in sys.argv[1:]
    rep = Report("gen-drivers")

    with open(os.path.join(ROOT, NOTES), "rb") as fh:
        notes = tomllib.load(fh)["drivers"]

    ds = driverlib.drivers(rep)
    text = must_read(MANUAL)
    text = splice(text, BEGIN % "driver-table", END % "driver-table",
                  table(ds, notes, rep), f"{MANUAL} (driver-table)", rep)

    on_disk = must_read(MANUAL)
    if text != on_disk:
        if write:
            with open(os.path.join(ROOT, MANUAL), "w", encoding="utf-8") as fh:
                fh.write(text)
            print(f"  wrote {MANUAL}")
        else:
            rep.fail(f"{MANUAL}'s driver table does not match src/drivers.c — "
                     f"run `make docs-tables`")
            for line in list(difflib.unified_diff(
                    on_disk.split("\n"), text.split("\n"),
                    "on disk", "drivers.c", lineterm="", n=0))[:16]:
                print("    " + line, file=sys.stderr)

    check_name_lists(ds, rep)

    imu = driverlib.names(ds, "imu")
    mag = driverlib.names(ds, "mag")
    return rep.finish(f"{len(set(imu) | set(mag))} drivers "
                      f"({len(imu)} imu, {len(mag)} mag)")


if __name__ == "__main__":
    sys.exit(main())
