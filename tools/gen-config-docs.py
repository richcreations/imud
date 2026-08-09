#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
gen-config-docs.py — render the config-key documentation from one registry.

150 keys, each stated on three surfaces: the shipped .conf template, a man5
.TP entry and a docs/manual.md table row.  check-docs.py proves a key REACHED
all three; nothing proved they agreed, and man/man5/imud.conf.5 spent several
releases telling operators the ISM330DHCX topped out at 1660 Hz while the
driver had offered 3332 and 6664 since it shipped.

docs/config-keys.toml is now the one home for the type, default, scope and
prose.  This renders the man5 entries and the manual tables from it.

  --check   render and compare against what is on disk; print a diff and exit
            1 on any difference.  This is what CI runs.
  (default) same comparison, reported as a summary.

The .conf templates are deliberately NOT generated: their value is the
worked-example layout — column-aligned, with commented-out blocks presented as
copy-paste units — and a generator flattens exactly that.  check-docs.py keeps
proving every key is present in them.

The registry's prose was extracted from the pages it now feeds, so the first
run of this tool reproduced them byte for byte.  That identity is the evidence
the migration preserved the documentation rather than paraphrasing it; keep it
true, and a diff here will only ever be the edit you meant to make.

Run as `make check-generated-text` (via check-config-docs) or directly.
"""

import difflib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, must_read, Report                    # noqa: E402

REGISTRY = "docs/config-keys.toml"

# Region markers.  Invisible where they live: .\" is a roff comment, <!-- -->
# is a markdown comment that neither GitHub nor pandoc renders.
BEGIN_ROFF = '.\\" BEGIN GENERATED: config-keys %s'
END_ROFF   = '.\\" END GENERATED: config-keys %s'
BEGIN_MD   = "<!-- BEGIN GENERATED: config-keys %s -->"
END_MD     = "<!-- END GENERATED: config-keys %s -->"


def splice(text, begin, end, body, where, rep):
    """Replace the text between two markers, leaving the rest byte-identical.

    Fails loudly rather than guessing: an unmatched or duplicated marker means
    the file is not in the shape this tool understands, and writing anyway
    would corrupt a document.
    """
    b, e = text.count(begin), text.count(end)
    if b != 1 or e != 1:
        rep.fail("%s: expected exactly one %r and one %r, found %d and %d"
                 % (where, begin, end, b, e))
        return text
    i = text.index(begin) + len(begin)
    j = text.index(end)
    if j < i:
        rep.fail("%s: END marker precedes BEGIN" % where)
        return text
    return text[:i] + "\n" + body + "\n" + text[j:]

try:
    import tomllib
except ModuleNotFoundError:                                     # pragma: no cover
    sys.exit(f"{REGISTRY} needs python3.11+ for tomllib "
             f"(Debian bookworm and newer are fine)")


def load():
    with open(os.path.join(ROOT, REGISTRY), "rb") as f:
        return tomllib.load(f)


def man_entry(key):
    """The .TP block for one registry entry, as roff.

    Two spellings are in use and both are reproduced: the core page writes
    `.RI (int,\\ default:\\ 833)`, the bridge pages
    `.RI ( int ,\\ default:\\ 5 )\\ [hot]`.  Unifying them is a separate change
    that should show up as a diff, not something this quietly does.
    """
    names = key["names"]
    if len(names) == 1:
        head = ".B " + names[0]
    else:
        head = ".BR " + ' ", " '.join(names)

    if key["man_style"] == "spaced":
        ri = ".RI ( %s ,\\ %s )" % (key["type"], key["default"])
        if key["scope"]:
            ri += "\\ [%s]" % key["scope"]
    else:
        ri = ".RI (%s,\\ %s)" % (key["type"], key["default"])

    return "\n".join([".TP", head, ri, key["man"]])


def md_tables(section):
    """The section's entries grouped as the MANUAL lays them out.

    Two things differ from the man page and both are reproduced rather than
    unified — making them agree is a content change and belongs in a commit
    that shows it as a diff:

      order   [mount] is euler/matrix/preset in the man page and
              euler/preset/matrix in the manual;
      shape   [position] is one .SS in the man page but FOUR tables in the
              manual, with prose between them, so it gets four regions.
    """
    by_name = {k["names"][0]: k for k in section["key"]}
    out = []
    for group in section.get("md_groups") or []:
        out.append([by_name[n] for n in group if n in by_name])
    return out


def md_row(key):
    """The manual.md table row for one registry entry."""
    names = " / ".join("`%s`" % n for n in key["names"])
    return "| %s | %s | %s | %s |" % (names, key["md_type"],
                                      key["md_default"], key["desc"])


def regions(reg):
    """(file, begin, end, body, [(key label, rendered)]) per owned region.

    The per-key renderings ride along so a mismatch can name the KEY that
    drifted rather than the file or the region: "docs/manual.md differs" sends
    the reader through 1700 lines, "[imu] fifo_wm" sends them to the row.
    """
    for section in reg["section"]:
        name = section["name"]
        parts = [("[%s] %s" % (name, "/".join(k["names"])), man_entry(k))
                 for k in section["key"]]
        yield (section["man_file"], BEGIN_ROFF % name, END_ROFF % name,
               "\n".join(p for _, p in parts), parts)
        for i, group in enumerate(md_tables(section), 1):
            tag = "%s.%d" % (name, i)
            parts = [("[%s] %s" % (name, "/".join(k["names"])), md_row(k))
                     for k in group]
            yield (section["md_file"], BEGIN_MD % tag, END_MD % tag,
                   "\n".join(p for _, p in parts), parts)


def main():
    write = "--write" in sys.argv[1:]
    rep = Report("gen-config-docs")
    reg = load()
    rep.expect(reg.get("section"), "registry sections")

    # Group by file: a file holds several regions and must be read once and
    # written once, or each splice would undo the last.
    files, order = {}, []
    stale = {}
    for path, begin, end, body, parts in regions(reg):
        if path not in files:
            files[path] = must_read(path)
            order.append(path)
        # Name the region that actually differs, not just the file.  A failure
        # reading "docs/manual.md differs" sends the reader through 1700 lines;
        # naming the keys in the region sends them to the row.
        if body and body not in files[path]:
            for label, rendered in parts:
                if rendered not in files[path]:
                    stale.setdefault(path, []).append((label, rendered))
        files[path] = splice(files[path], begin, end, body,
                             "%s (%s)" % (path, begin), rep)

    changed = 0
    for path in order:
        on_disk = must_read(path)
        if files[path] == on_disk:
            continue
        changed += 1
        if write:
            with open(os.path.join(ROOT, path), "w", encoding="utf-8") as f:
                f.write(files[path])
            print("  wrote %s" % path)
        else:
            for label, rendered in stale.get(path, [("(ordering)", "")]):
                rep.fail("%s: %s does not match the registry" % (path, label))
            for line in list(difflib.unified_diff(
                    on_disk.split("\n"), files[path].split("\n"),
                    "on disk", "registry", lineterm="", n=0))[:12]:
                print("    " + line, file=sys.stderr)

    n_keys = sum(len(k["names"]) for s in reg["section"] for k in s["key"])
    n_regions = sum(1 for _ in regions(reg))
    if write:
        return rep.finish("%d keys into %d regions, %d file(s) updated"
                          % (n_keys, n_regions, changed))
    return rep.finish("%d keys across %d regions match the registry"
                      % (n_keys, n_regions))


if __name__ == "__main__":
    sys.exit(main())
