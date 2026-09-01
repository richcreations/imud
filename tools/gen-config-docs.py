#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
gen-config-docs.py — render the config-key documentation from one registry.

156 keys, each stated on three surfaces: the shipped .conf template, a man5
.TP entry and a docs/manual.md table row.  check-docs.py proves a key REACHED
all three; nothing proved they agreed, and man/man5/imud.conf.5 spent several
releases telling operators the ISM330DHCX topped out at 1660 Hz while the
driver had offered 3332 and 6664 since it shipped.

docs/config-keys.toml is now the one home for the type, default, scope and
prose.  This renders the man5 entries, the manual tables and the compiled
defaults test from it.

  --write   update the files on disk.
  (default) render and compare against what is on disk; print a diff and name
            the key that drifted.  This is what CI runs.

The .conf templates are deliberately NOT generated: their value is the
worked-example layout — column-aligned, with commented-out blocks presented as
copy-paste units — and a generator flattens exactly that.  check-docs.py keeps
proving every key is present in them.

The registry's prose was extracted from the pages it now feeds, so the first
run of this tool reproduced them byte for byte.  That identity is the evidence
the migration preserved the documentation rather than paraphrasing it; keep it
true, and a diff here will only ever be the edit you meant to make.

test/test_config_defaults.gen.c is the fourth surface, and the only one that
is not prose: it turns every documented default into a compiled assertion
against config_defaults().  Rendering the .conf, the man page and the manual
from one registry proves the three AGREE; it cannot prove any of them is TRUE.
That needs the code, and it has to be compiled C rather than a text scan
because config_defaults() opens with memset(cfg, 0, sizeof *cfg) — a deleted
assignment leaves a plausible zero that no scan of the source would question.

Run as `make check-generated-text` (via check-config-docs) or directly.
"""

import difflib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, read, must_read, splice, Report      # noqa: E402
import driverlib                                                # noqa: E402

REGISTRY = "docs/config-keys.toml"

# Region markers.  Invisible where they live: .\" is a roff comment, <!-- -->
# is a markdown comment that neither GitHub nor pandoc renders.
BEGIN_ROFF = '.\\" BEGIN GENERATED: config-keys %s'
END_ROFF   = '.\\" END GENERATED: config-keys %s'
BEGIN_MD   = "<!-- BEGIN GENERATED: config-keys %s -->"
END_MD     = "<!-- END GENERATED: config-keys %s -->"


try:
    import tomllib
except ModuleNotFoundError:                                     # pragma: no cover
    sys.exit(f"{REGISTRY} needs python3.11+ for tomllib "
             f"(Debian bookworm and newer are fine)")


def load():
    with open(os.path.join(ROOT, REGISTRY), "rb") as f:
        return tomllib.load(f)


# The *_ops initialisers, loaded once in main().  A module global rather than
# a parameter threaded through six renderers: only values() reads it, and only
# for the four keys that name a driver's rate list.
DRIVERS = []


def values(key, rep, backticked):
    """The `supports:` list a key documents, read out of the driver.

    A handful of keys name the rates or ranges one specific part accepts —
    `[imu] odr_hz` says "ISM330DHCX supports: ..." — and that list is a copy
    of a .supported_* array in the ops initialiser.  It is the copy that went
    stale: the man page stopped at 1660 for several releases while the driver
    had offered 3332 and 6664 since it shipped.

    `values_from` names the array; the prose keeps a {values} placeholder, so
    the sentence around it stays hand-written and only the enumeration is
    machine-owned.
    """
    src = key.get("values_from")
    if not src:
        return None
    name, _, array = src.partition(".")
    d = next((x for x in DRIVERS if x["name"] == name and x.get(array)), None)
    if d is None:
        rep.fail("%s: values_from = %r names no registered driver with that "
                 "array" % (REGISTRY, src))
        return ""
    def render(v):
        """Milli-Hz arrays print as Hz; everything else prints as-is.

        The ladders are stored in milli-Hz because that is the only unit that
        can hold 12.5 Hz (TDK) or the ST chain's 13.016 Hz bottom rung. Docs
        are for people, so they read Hz, with the fraction only where there is
        one: 833000 -> 833, 12500 -> 12.5, 13016 -> 13.016.
        """
        if not array.endswith("_mhz"):
            return "%d" % v
        whole, frac = divmod(v, 1000)
        return "%d" % whole if frac == 0 else ("%d.%03d" % (whole, frac)).rstrip("0")

    fmt = "`%s`" if backticked else "%s"
    return ", ".join(fmt % render(v) for v in d[array])


def substitute(text, key, rep, backticked):
    """Fill a rendered body's {values} placeholder, if it has one."""
    if "{values}" not in text:
        if key.get("values_from"):
            rep.fail("%s: [%s] has values_from but no {values} placeholder — "
                     "the list it names is not reaching the page"
                     % (REGISTRY, "/".join(key["names"])))
        return text
    got = values(key, rep, backticked)
    if got is None:
        rep.fail("%s: [%s] writes {values} but has no values_from"
                 % (REGISTRY, "/".join(key["names"])))
        return text
    return text.replace("{values}", got)


def man_entry(key, rep):
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

    return "\n".join([".TP", head, ri,
                      substitute(key["man"], key, rep, backticked=False)])


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


def md_row(key, rep):
    """The manual.md table row for one registry entry."""
    names = " / ".join("`%s`" % n for n in key["names"])
    return "| %s | %s | %s | %s |" % (
        names, key["md_type"], key["md_default"],
        substitute(key["desc"], key, rep, backticked=True))


# ── The compiled defaults test ───────────────────────────────────────────────

DEFAULTS_TEST = "test/test_config_defaults.gen.c"

# NEED_* macro -> the CK_* macro test_config.c defines for that C type.  The
# macro is what fixes the comparison: an int is ==, a string is strcmp, and the
# two float widths get their own tolerances.  Reading the type off the macro
# rather than off include/config.h keeps this honest in one specific way — the
# macro is what apply_kv() ACTUALLY applies to the field, so a key whose parser
# and whose struct member disagree is caught by the compiler here rather than
# by a checker that agreed with the wrong one.
CK = {
    "NEED_INT":       "CK_INT",
    "NEED_POS_INT":   "CK_INT",
    "NEED_POS_MHZ":   "CK_INT",
    "NEED_PORT":      "CK_INT",
    "NEED_GPIO":      "CK_INT",
    "NEED_I2C_ADDR":  "CK_INT",
    "NEED_BOOL":      "CK_BOOL",
    "NEED_STR":       "CK_STR",
    "NEED_FLT":       "CK_FLT",
    "NEED_DBL":       "CK_DBL",
    "NEED_POS_DBL":   "CK_DBL",
    "NEED_RANGE_DBL": "CK_DBL",
    "NEED_BUS_KIND":  "CK_ENUM",
}

BUS = {"i2c": "BUS_I2C", "spi": "BUS_SPI"}

# The manual writes a default the way an operator would type it into the file.
# That spelling is close enough to C to reuse, and deliberately so: reading the
# literal back out of the published text is what makes this an assertion about
# the DOCUMENTATION rather than a second copy of the code.
UNSET = "*(unset)*"


def c_literal(macro, shown, where):
    """The C literal for a documented default, or None with a reason.

    Strict on purpose.  Anything this cannot read is raised as a failure
    naming the key, never skipped — a default silently dropped from the
    generated test is a default nothing checks, which is the state this whole
    stage exists to end.
    """
    m = re.search(r"`([^`]*)`", shown)          # first code span; `""` (auto)
    text = m.group(1).strip() if m else shown.strip()
    unset = not m and text == UNSET

    if macro == "NEED_POS_MHZ":
        # Documented in Hz, stored in milli-Hz. The published default is what an
        # operator types; the assertion has to be what config_defaults() sets,
        # so scale here rather than publishing a milli-Hz number nobody writes.
        try:
            hz = float(text)
        except ValueError:
            return None, "expected a number of Hz, got %r" % text
        mhz = int(hz * 1000.0 + 0.5)
        if mhz <= 0:
            return None, "a rate default must be greater than zero (%r)" % text
        return str(mhz), None

    if macro == "NEED_BOOL":
        if text not in ("true", "false"):
            return None, "expected true or false, got %r" % text
        return text, None

    if macro == "NEED_STR":
        if unset:
            return '""', None                   # nothing assigned, so memset's ""
        if text.startswith('"') and text.endswith('"') and len(text) >= 2:
            text = text[1:-1]
        elif re.search(r"[\s`*]", text):
            return None, "not a plain string default: %r" % text
        # Paths, URLs and dotted-quad addresses only; no C escape has ever been
        # needed, and one appearing unnoticed would be a silently wrong test.
        if '"' in text or "\\" in text:
            return None, "string default needs C escaping: %r" % text
        return '"%s"' % text, None

    if macro == "NEED_BUS_KIND":
        name = text.strip('"')
        if name not in BUS:
            return None, "unknown bus kind %r" % text
        return BUS[name], None

    if CK.get(macro) == "CK_INT":
        if not re.fullmatch(r"-?(0[xX][0-9a-fA-F]+|\d+)", text):
            return None, "expected an integer, got %r" % text
        return text, None

    if CK.get(macro) in ("CK_FLT", "CK_DBL"):
        if not re.fullmatch(r"-?\d+(\.\d+)?([eE][-+]?\d+)?", text):
            return None, "expected a number, got %r" % text
        return text, None

    return None, "no C type for macro %r" % macro


def defaults_test(reg, rep):
    """The generated body of test_defaults_registry(), as C."""
    out = [
        "/*",
        " * imud — IMU daemon",
        " * Copyright (c) 2026 Richard Simpson",
        " * SPDX-License-Identifier: MIT",
        " */",
        "",
        "/*",
        " * test_config_defaults.gen.c — GENERATED, do not edit.",
        " *",
        " * Written by tools/gen-config-docs.py --write from",
        " * docs/config-keys.toml; `make check-generated-text` fails if this",
        " * file and the registry disagree.  Edit the registry, not this.",
        " *",
        " * One assertion per documented default: the value on the right is the",
        " * one docs/manual.md prints for that key, read back out of the table.",
        " * So a failure here is never ambiguous — either config_defaults() no",
        " * longer produces what is published, or the published value is wrong.",
        " *",
        " * #included inside a function in test/test_config.c, which defines the",
        " * CK_* macros and holds the config_defaults() call.  __FILE__ resolves",
        " * to THIS file, so a failure reports the generated line, and the label",
        " * on it names the [section] and key.",
        " */",
        "",
    ]

    n, skipped = 0, []
    for section in reg["section"]:
        rows = []
        for key in section["key"]:
            for name, field, macro in zip(key["names"], key["fields"],
                                          key["macros"]):
                label = "[%s] %s" % (section["name"], name)
                if not field:
                    skipped.append(label)
                    continue
                if macro not in CK:
                    rep.fail("%s: %s: unknown macro %r"
                             % (DEFAULTS_TEST, label, macro))
                    continue
                lit, why = c_literal(macro, key["md_default"], label)
                if lit is None:
                    rep.fail("%s: %s: cannot read its documented default (%s)"
                             % (DEFAULTS_TEST, label, why))
                    continue
                rows.append((CK[macro], field, lit, label))
                n += 1
        if not rows:
            continue
        out.append("/* [%s] */" % section["name"])
        # Column-align within a section, not across the file: a 150-line block
        # aligned to its widest member is mostly whitespace, and the sections
        # are what a reader scans.
        w1 = max(len(r[0]) for r in rows)
        w2 = max(len(r[1]) for r in rows) + 1
        w3 = max(len(r[2]) for r in rows) + 1
        for ck, field, lit, label in rows:
            out.append("%-*s(c.%-*s %-*s \"%s\");"
                       % (w1, ck, w2, field + ",", w3, lit + ",", label))
        out.append("")

    # Named, not counted: "3 keys skipped" ages into nobody knowing which.
    out.append("/* Not assertable, and each one deliberately so:")
    for label in skipped:
        out.append(" *   %s" % label)
    out.append(" * they are hand-rolled blocks in apply_kv() that set several")
    out.append(" * members at once, or (preset) match a name and store nothing,")
    out.append(" * so there is no single field carrying the documented default.")
    out.append(" * test_defaults_mount() in test_config.c asserts all three. */")
    return "\n".join(out) + "\n", n, skipped


def regions(reg, rep):
    """(file, begin, end, body, [(key label, rendered)]) per owned region.

    The per-key renderings ride along so a mismatch can name the KEY that
    drifted rather than the file or the region: "docs/manual.md differs" sends
    the reader through 1700 lines, "[imu] fifo_wm" sends them to the row.
    """
    for section in reg["section"]:
        name = section["name"]
        parts = [("[%s] %s" % (name, "/".join(k["names"])), man_entry(k, rep))
                 for k in section["key"]]
        yield (section["man_file"], BEGIN_ROFF % name, END_ROFF % name,
               "\n".join(p for _, p in parts), parts)
        for i, group in enumerate(md_tables(section), 1):
            tag = "%s.%d" % (name, i)
            parts = [("[%s] %s" % (name, "/".join(k["names"])), md_row(k, rep))
                     for k in group]
            yield (section["md_file"], BEGIN_MD % tag, END_MD % tag,
                   "\n".join(p for _, p in parts), parts)


def main():
    write = "--write" in sys.argv[1:]
    rep = Report("gen-config-docs")
    reg = load()
    rep.expect(reg.get("section"), "registry sections")

    global DRIVERS
    DRIVERS = driverlib.drivers(rep)

    # Group by file: a file holds several regions and must be read once and
    # written once, or each splice would undo the last.
    files, order = {}, []
    stale = {}
    for path, begin, end, body, parts in regions(reg, rep):
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

    # The generated test is a whole file rather than a region, so it is
    # compared and written as one — but through the same report, because a
    # drifted default and a drifted table are the same failure to the reader.
    body, n_asserts, skipped = defaults_test(reg, rep)
    on_disk = read(DEFAULTS_TEST)
    if body != on_disk:
        if write:
            with open(os.path.join(ROOT, DEFAULTS_TEST), "w",
                      encoding="utf-8") as f:
                f.write(body)
            print("  wrote %s" % DEFAULTS_TEST)
            changed += 1
        else:
            rep.fail("%s is stale — run `make docs-config`" % DEFAULTS_TEST)
            for line in list(difflib.unified_diff(
                    (on_disk or "").split("\n"), body.split("\n"),
                    "on disk", "registry", lineterm="", n=0))[:12]:
                print("    " + line, file=sys.stderr)

    n_keys = sum(len(k["names"]) for s in reg["section"] for k in s["key"])
    n_regions = sum(1 for _ in regions(reg, rep))
    rep.expect(n_asserts, "defaults-test assertions")
    if write:
        return rep.finish("%d keys into %d regions + %d assertions, "
                          "%d file(s) updated"
                          % (n_keys, n_regions, n_asserts, changed))
    return rep.finish("%d keys across %d regions match the registry; "
                      "%d defaults asserted, %d not assertable"
                      % (n_keys, n_regions, n_asserts, len(skipped)))


if __name__ == "__main__":
    sys.exit(main())
