#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
loc.py — how big is this project, split the way this project is organised.

cloc, tokei and scc all exist and all report by LANGUAGE, which is the wrong
axis here: every .c file lands in one row, so `src/` and `test/` become
indistinguishable and the only interesting number disappears. This repo's test
code slightly EXCEEDS its production code, and that is a fact about the project
worth being able to see at a glance — a per-language table hides it.

So the categories below are roles, not languages: production C is separate from
the suites, the fuzz harnesses are separate from both, the shipped Python client
is separate from the build tools that only gate CI.

Counting rules:

  - Only files git tracks. That drops build output, devbox/ and everything else
    gitignored without an exclusion list here to keep in sync.
  - "code" is lines that are neither blank nor comment.
  - Comment detection is language-aware but deliberately simple: a `#` inside a
    shell or Python string is miscounted as a comment, and so is one inside a
    YAML scalar. cloc has the same limitation. This is a size report, not a
    parser — do not treat the last digit as exact.

Two structural guards, because a counter that is quietly wrong is worse than no
counter at all (both of these come from real mistakes):

  - A category marked required that matches NOTHING is a hard error. Rename
    test/ and this fails naming the category, rather than reporting 0 test lines
    and looking plausible. Same rule tools/check-flags.py applies to its
    extractors.
  - No file may match two categories, and the per-category counts must re-add to
    the total file count. The throwaway script that first produced these numbers
    silently dropped 89 of 300 files; this cannot.

Run as `make loc`. `--format=markdown` emits the table CI appends to the run
summary.
"""

import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _ext(p):
    return os.path.splitext(p)[1]


def _under(p, *prefixes):
    """Path-prefix match that never steals a Markdown file from Docs.

    Markdown is prose wherever it sits, so `apt/README.md` and a hypothetical
    `.github/CONTRIBUTING.md` belong in Docs rather than in the directory's
    category. Without this the two predicates both match and the overlap guard
    in classify() rejects the file — which is how this rule was discovered.
    """
    return p.startswith(prefixes) and _ext(p) != ".md"


# (label, predicate, required, is_code)
#
# Predicates are fully qualified rather than order-dependent — "production C" is
# `.c` outside test/ and fuzz/, not "whatever .c is left over". That way the
# report order below is presentational only, and a file matching two categories
# is a bug this tool can detect instead of a silent first-match-wins.
#
# is_code marks what the CODE subtotal counts: prose, packaging metadata,
# shipped config templates and fuzz corpus seeds are all real parts of the
# release, but they are not code and padding the number with them would be
# dishonest.
CATEGORIES = [
    ("Production C",
     lambda p: _ext(p) == ".c" and not p.startswith(("test/", "fuzz/")),
     True, True),
    ("Headers (C)",
     lambda p: _ext(p) == ".h" and not p.startswith("test/"),
     True, True),
    ("Tests (C)",
     lambda p: p.startswith("test/") and _ext(p) in (".c", ".h"),
     True, True),
    ("Fuzz harnesses (C)",
     lambda p: p.startswith("fuzz/") and _ext(p) == ".c",
     True, True),
    ("Python (shipped client)",
     lambda p: p.startswith("lib/") and _ext(p) == ".py",
     True, True),
    ("Python (build tools)",
     lambda p: p.startswith("tools/") and _ext(p) == ".py",
     True, True),
    # A shell script under .github/ is CI infrastructure, not loose tooling:
    # .github/ci-setup.sh is sourced by the workflow jobs and has no life
    # outside them, so it belongs with the workflows it serves.  Qualified
    # here rather than ordered around, for the reason given above — the
    # overlap guard is what caught this.
    ("Shell",
     lambda p: _ext(p) == ".sh" and not p.startswith(".github/"),
     False, True),
    ("Build system",
     lambda p: p in ("Makefile", "debian/rules"),
     True, True),
    ("CI workflows",
     lambda p: _under(p, ".github/"),
     True, True),
    ("Man pages (roff)",
     lambda p: p.startswith("man/") and _ext(p) in (".1", ".3", ".5", ".8"),
     True, False),
    ("Docs (Markdown)",
     lambda p: _ext(p) == ".md",
     True, False),
    ("Packaging",
     lambda p: _under(p, "debian/", "packaging/") and p != "debian/rules",
     True, False),
    ("Config + units",
     lambda p: _under(p, "config/", "etc/"),
     True, False),
    ("Fuzz corpus",
     lambda p: _under(p, "test/fuzz/corpus/"),
     False, False),
    ("Data",
     lambda p: _under(p, "data/"),
     False, False),
    ("Website",
     lambda p: _under(p, "web/", "apt/"),
     False, False),
]

OTHER = "Other"          # LICENSE, AUTHORS, NEWS, spec.md's siblings, dotfiles


def tracked_files():
    """Every path git tracks, relative to ROOT."""
    try:
        out = subprocess.run(["git", "-C", ROOT, "ls-files", "-z"],
                             capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        sys.exit("loc: needs a git checkout — the file list comes from "
                 "`git ls-files` so that build output is never counted. "
                 "(A `make dist` tarball has no .git; run this in the repo.)")
    return [p.decode() for p in out.split(b"\0") if p]


def is_binary(path):
    """git's own heuristic: a NUL byte anywhere in the first 8 KB."""
    try:
        with open(os.path.join(ROOT, path), "rb") as fh:
            return b"\0" in fh.read(8192)
    except OSError:
        return False


def count(path):
    """(total, blank, comment) for one file; a binary is 0 lines, not 0 bytes.

    Binary files are tracked here (docs/math.pdf, the .bin and .imucap fuzz
    seeds) and they have no lines to count. Two traps, both hit on the way in:

      - `wc -l` reports 844 for docs/math.pdf because it counts newline BYTES.
        That is not a line count of anything; it is a property of the compressed
        stream. Reporting 0 and saying so is the honest answer.
      - Python text mode does universal-newline translation, so a lone \\r in a
        binary decodes as a line break. That inflated math.pdf to 1434 and made
        the whole report 592 lines heavier than reality. newline="" below turns
        the translation off for the text files too, so a CRLF or CR file counts
        the same way wc would.
    """
    ext = _ext(path)
    full = os.path.join(ROOT, path)
    if is_binary(path):
        return (0, 0, 0)
    try:
        with open(full, encoding="utf-8", errors="replace", newline="") as fh:
            lines = fh.read().split("\n")
    except OSError:
        return (0, 0, 0)
    if lines and lines[-1] == "":
        lines.pop()                       # trailing newline is not a line
    total = len(lines)
    blank = sum(1 for ln in lines if not ln.strip())
    comment = 0

    if ext in (".c", ".h"):
        in_block = False
        for ln in lines:
            s = ln.strip()
            if not s:
                continue
            if in_block:
                comment += 1
                if "*/" in s:
                    in_block = False
            elif s.startswith("//"):
                comment += 1
            elif s.startswith("/*"):
                # Only a comment that STARTS the line; `int x; /* n */` is code.
                comment += 1
                if "*/" not in s[2:]:
                    in_block = True
    elif (ext in (".py", ".sh", ".yml", ".yaml", ".conf", ".in")
          or path in ("Makefile", "debian/rules")):
        comment = sum(1 for ln in lines if ln.lstrip().startswith("#"))
    elif ext in (".1", ".3", ".5", ".8"):
        comment = sum(1 for ln in lines
                      if ln.lstrip().startswith(('.\\"', "'\\\"")))

    return (total, blank, comment)


def classify(files):
    """path -> category, failing loudly on an ambiguous or missing rule."""
    labels = {}
    for path in files:
        hits = [label for label, pred, _, _ in CATEGORIES if pred(path)]
        if len(hits) > 1:
            sys.exit(f"loc: {path} matches {len(hits)} categories "
                     f"({', '.join(hits)}) — the predicates overlap")
        labels[path] = hits[0] if hits else OTHER
    return labels


def collect():
    files = tracked_files()
    labels = classify(files)

    rows = {}                             # label -> [files, total, blank, comment]
    for path, label in labels.items():
        t, b, c = count(path)
        r = rows.setdefault(label, [0, 0, 0, 0])
        r[0] += 1
        r[1] += t
        r[2] += b
        r[3] += c

    missing = [label for label, _, required, _ in CATEGORIES
               if required and label not in rows]
    if missing:
        sys.exit("loc: category matched no files: " + ", ".join(missing) +
                 "\n     A renamed or moved directory must fail here rather "
                 "than silently report zero.")

    counted = sum(r[0] for r in rows.values())
    if counted != len(files):
        sys.exit(f"loc: accounted for {counted} files but git tracks "
                 f"{len(files)} — the aggregation dropped some")

    nbinary = sum(1 for p in files if is_binary(p))
    return rows, len(files), nbinary


def totals(rows, code_only):
    """(files, lines, code) summed over categories, optionally code ones only."""
    code_labels = {label for label, _, _, is_code in CATEGORIES if is_code}
    f = ln = cd = 0
    for label, (nf, t, b, c) in rows.items():
        if code_only and label not in code_labels:
            continue
        f += nf
        ln += t
        cd += t - b - c
    return (f, ln, cd)


def ordered(rows):
    for label, _, _, _ in CATEGORIES:
        if label in rows:
            yield label, rows[label]
    if OTHER in rows:
        yield OTHER, rows[OTHER]


def ratio(rows):
    """Tests-to-production-C ratio — the number worth watching over time."""
    prod = rows.get("Production C", [0, 0, 0, 0])
    test = rows.get("Tests (C)", [0, 0, 0, 0])
    p = prod[1] - prod[2] - prod[3]
    t = test[1] - test[2] - test[3]
    return (t / p) if p else 0.0


def render_text(rows, nfiles, nbinary):
    out = [f"{'':26}{'files':>6}{'lines':>9}{'blank':>8}"
           f"{'comment':>9}{'code':>9}", "-" * 67]
    for label, (nf, t, b, c) in ordered(rows):
        out.append(f"{label:26}{nf:6}{t:9}{b:8}{c:9}{t - b - c:9}")
    out.append("-" * 67)
    cf, cl, cc = totals(rows, code_only=True)
    af, al, ac = totals(rows, code_only=False)
    out.append(f"{'CODE (excl. docs, data)':26}{cf:6}{cl:9}{'':8}{'':9}{cc:9}")
    out.append(f"{'ALL TRACKED':26}{af:6}{al:9}{'':8}{'':9}{ac:9}")
    out.append("")
    out.append(f"tests/production C ratio: {ratio(rows):.2f}:1   "
               f"({nfiles} files tracked, of which {nbinary} binary "
               f"and counted as 0 lines)")
    return "\n".join(out)


def render_markdown(rows, nfiles, nbinary):
    out = ["## Line count", "",
           "| | files | lines | blank | comment | code |",
           "|---|---:|---:|---:|---:|---:|"]
    for label, (nf, t, b, c) in ordered(rows):
        out.append(f"| {label} | {nf} | {t:,} | {b:,} | {c:,} | {t - b - c:,} |")
    cf, cl, cc = totals(rows, code_only=True)
    af, al, ac = totals(rows, code_only=False)
    out.append(f"| **CODE** (excl. docs, data) | {cf} | {cl:,} | | | **{cc:,}** |")
    out.append(f"| **ALL TRACKED** | {af} | {al:,} | | | {ac:,} |")
    out.append("")
    out.append(f"Tests-to-production-C ratio **{ratio(rows):.2f}:1** "
               f"over {nfiles} tracked files "
               f"({nbinary} binary, counted as 0 lines).")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="Count lines by role, not language.")
    ap.add_argument("--format", choices=("text", "markdown"), default="text",
                    help="markdown emits the table CI appends to its summary")
    args = ap.parse_args()

    rows, nfiles, nbinary = collect()
    render = render_markdown if args.format == "markdown" else render_text
    print(render(rows, nfiles, nbinary))
    return 0


if __name__ == "__main__":
    sys.exit(main())
