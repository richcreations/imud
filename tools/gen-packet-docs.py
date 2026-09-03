#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
gen-packet-docs.py — render spec.md's wire tables from include/types.h.

spec.md §8 prints the 276-byte packet as an offset/size/type/field table.  It
is the document a third-party integrator reads to write a decoder, and its
version row said

    4      2      uint16    version           = 14  (1.4; encoded as major*10+minor)

while include/types.h said IMUD_VERSION 17 — three wire revisions stale, with
a decoding rule that had stopped being true.  spec.md said "v17" four times
elsewhere in the same file.  The flags table beneath it is the FIFTH copy of
the flags word; check-flags.py compares the four in code and never read this.

Offsets in a packed struct are cumulative sizeof, and a field inserted
mid-struct shifts every row beneath it — the one error a proof-reader will
not catch, and now one that cannot be made.  check-packet.py used to check
this table; generating it retires that checker, which is the trade the whole
stage is built on: a checked surface can still be wrong between checks, a
generated one cannot be wrong at all.

What is NOT generated is the Notes column.  It is prose, and it is the reason
the table is usable at all; it lives in docs/packet-notes.toml keyed by field
name, so a field that moves carries its note with it.

  --write   update spec.md.
  (default) compare against what is on disk and report.
"""

import difflib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, must_read, splice, Report            # noqa: E402

try:
    import tomllib
except ModuleNotFoundError:                                     # pragma: no cover
    sys.exit("docs/packet-notes.toml needs python3.11+ for tomllib")

HDR = "include/types.h"
SPEC = "spec.md"
NOTES = "docs/packet-notes.toml"

BEGIN = "<!-- BEGIN GENERATED: %s -->"
END = "<!-- END GENERATED: %s -->"

# C type -> (size, the name spec.md uses for it).  Sizes are the fixed-width
# ones the packed struct is built from; nothing here is platform-dependent,
# which is the point of the struct using them.
CTYPE = {
    "uint8_t":  (1, "uint8"),
    "int8_t":   (1, "int8"),
    "uint16_t": (2, "uint16"),
    "int16_t":  (2, "int16"),
    "uint32_t": (4, "uint32"),
    "int32_t":  (4, "int32"),
    "uint64_t": (8, "uint64"),
    "int64_t":  (8, "int64"),
    "float":    (4, "float32"),
    "double":   (8, "float64"),
    "char":     (1, "char"),
}

# Column starts, measured off the table this replaced.  The Notes column is
# where a note's FIRST line begins; continuation lines carry their own
# indentation verbatim from the sidecar, because the table they came from does
# not indent them consistently (42, 43 and 46 all occur) and re-wrapping prose
# to fix that would be a content change hiding inside a refactor.
COLS = (8, 7, 10, 18)
FLAG_COLS = (8, 19)


def struct_layout(rep):
    """[(offset, size, spec_type, name)] for imu_packet_t, in order."""
    src = must_read(HDR, "the wire packet definition")
    m = re.search(
        r"typedef struct\s+__attribute__\(\(packed\)\)\s*\{(.*?)\}\s*imu_packet_t;",
        src, re.S)
    if not m:
        sys.exit(f"{HDR}: cannot find the packed imu_packet_t — has it moved?")

    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    fields, offset = [], 0
    for line in body.split("\n"):
        fm = re.match(r"\s*(\w+)\s+(\w+)\s*(?:\[(\d+)\])?\s*;", line)
        if not fm:
            continue
        ctype, name, count = fm.group(1), fm.group(2), fm.group(3)
        if ctype not in CTYPE:
            rep.fail(f"{HDR}: field '{name}' has type '{ctype}', which this "
                     f"cannot size — add it to CTYPE")
            continue
        size, spec_type = CTYPE[ctype]
        n = int(count) if count else 1
        # An array is ONE row, sized `9×4` so a reader sees element count and
        # element size both.  The table has always documented it as a unit.
        fields.append((offset, f"{n}×{size}" if count else str(size),
                       spec_type, f"{name}[{n}]" if count else name))
        offset += size * n
    rep.expect(fields, "imu_packet_t fields")
    return fields, offset


def _bits(src, pattern, rep, what, where):
    bits = [(int(b), n.lower()) for n, b in re.findall(pattern, src, re.M)]
    rep.expect(bits, what)
    dupes = {b for b, _ in bits if [x for x, _ in bits].count(b) > 1}
    for b in sorted(dupes):
        rep.fail(f"{HDR}: {where} bit {b} is defined more than once")
    return sorted(bits)


def flag_bits(rep):
    """[(bit, name)] from the FLAG_* defines of the 16-bit `flags` word.

    FLAG_EXT_ is excluded rather than merged: flags_ext is a SEPARATE word,
    so its bit 0 and this word's bit 0 are different flags.  Matching both
    with one pattern reported "bit 0 is defined more than once" and would
    have documented one of them under the other's offset.
    """
    return _bits(must_read(HDR),
                 r"^#define\s+FLAG_(?!EXT_)(\w+)\s+\(1u\s*<<\s*(\d+)\)",
                 rep, "FLAG_* defines", "flags")


def flag_ext_bits(rep):
    """[(bit, name)] from the FLAG_EXT_* defines of the 32-bit flags_ext."""
    return _bits(must_read(HDR),
                 r"^#define\s+FLAG_EXT_(\w+)\s+\(1u\s*<<\s*(\d+)\)",
                 rep, "FLAG_EXT_* defines", "flags_ext")


def declared_size(rep):
    """The size the _Static_assert pins — the contract everyone codes to."""
    src = must_read(HDR)
    m = re.search(r"_Static_assert\(sizeof\(imu_packet_t\)\s*==\s*(\d+)", src)
    if not m:
        rep.fail(f"{HDR}: no _Static_assert on sizeof(imu_packet_t)")
        return None
    return int(m.group(1))


def wire_version(rep):
    src = must_read(HDR)
    m = re.search(r"^#define\s+IMUD_VERSION\s+(\d+)", src, re.M)
    if not m:
        rep.fail(f"{HDR}: cannot find #define IMUD_VERSION")
        return None
    return int(m.group(1))


def row(cells, widths, note, rep, where):
    """One table line plus its verbatim continuations."""
    out = ""
    for cell, width in zip(cells, widths):
        if len(cell) >= width:
            rep.fail(f"{where}: '{cell}' does not fit its {width}-column "
                     f"field — widen COLS rather than let it run into Notes")
        out += cell.ljust(width)
    head, _, rest = note.partition("\n")
    return (out + head).rstrip() + (("\n" + rest) if rest else "")


def layout_block(fields, total, notes, version, rep):
    lines = ["```text",
             "Offset".ljust(COLS[0]) + "Bytes".ljust(COLS[1])
             + "Type".ljust(COLS[2]) + "Field".ljust(COLS[3]) + "Notes",
             "─" * 66]
    for offset, size, ctype, name in fields:
        key = name.replace("[", "_").replace("]", "")
        note = notes.get(key)
        if note is None:
            rep.fail(f"{NOTES}: no note for '{name}' — every packet field "
                     f"needs one, even if it is empty")
            note = ""
        note = note.replace("{version}", str(version))
        lines.append(row((f"{offset:>2}", size, ctype, name),
                         COLS, note, rep, f"{SPEC} packet table"))
    lines += ["─" * 68, f"Total: {total} bytes", "```"]

    for key in sorted(set(notes) - {n.replace("[", "_").replace("]", "")
                                    for _, _, _, n in fields}):
        rep.fail(f"{NOTES}: has a note for '{key}', which imu_packet_t does "
                 f"not contain")
    return "\n".join(lines)


def flags_block(bits, notes, rep, trailer="trailer", label="flags"):
    lines = ["```text"]
    for bit, name in bits:
        note = notes.get(name)
        if note is None:
            rep.fail(f"{NOTES}: no note for {label} flag '{name}'")
            note = ""
        lines.append(row((f"bit {bit}", name), FLAG_COLS, note, rep,
                         f"{SPEC} {label} table"))
    lines.append(notes[trailer])
    lines.append("```")

    known = {n for _, n in bits} | {trailer}
    for key in sorted(set(notes) - known):
        rep.fail(f"{NOTES}: has a note for {label} flag '{key}', which {HDR} "
                 f"does not define")
    return "\n".join(lines)


def main():
    write = "--write" in sys.argv[1:]
    rep = Report("gen-packet-docs")

    with open(os.path.join(ROOT, NOTES), "rb") as fh:
        notes = tomllib.load(fh)

    fields, total = struct_layout(rep)
    version = wire_version(rep)
    pinned = declared_size(rep)
    if pinned is not None:
        rep.check(total == pinned,
                  f"{HDR}: fields sum to {total} bytes but _Static_assert pins "
                  f"{pinned} — this tool is mis-sizing something")

    bits = flag_bits(rep)
    ext_bits = flag_ext_bits(rep)

    text = must_read(SPEC)
    for tag, body in (("packet-layout",
                       layout_block(fields, total, notes["fields"], version, rep)),
                      ("packet-flags",
                       flags_block(bits, notes["flags"], rep)),
                      ("packet-flags-ext",
                       flags_block(ext_bits, notes["flags_ext"], rep,
                                   trailer="ext_trailer",
                                   label="flags_ext"))):
        text = splice(text, BEGIN % tag, END % tag, body,
                      f"{SPEC} ({tag})", rep)

    on_disk = must_read(SPEC)
    if text != on_disk:
        if write:
            with open(os.path.join(ROOT, SPEC), "w", encoding="utf-8") as fh:
                fh.write(text)
            print(f"  wrote {SPEC}")
        else:
            rep.fail(f"{SPEC} does not match {HDR} — run `make docs-tables`")
            for line in list(difflib.unified_diff(
                    on_disk.split("\n"), text.split("\n"),
                    "on disk", "types.h", lineterm="", n=0))[:16]:
                print("    " + line, file=sys.stderr)

    return rep.finish(f"{len(fields)} fields, {total} bytes, wire v{version}, "
                      f"{len(bits)} flags")


if __name__ == "__main__":
    sys.exit(main())
