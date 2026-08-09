#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-links.py — every link in a shipped document must lead somewhere.

Three failure modes, all of which have already happened here:

  1. A heading is renamed and the anchors pointing at it are not.
     docs/manual.md linked to `#supported-hardware` for a section actually
     called "5. Supported drivers" — a dead link in the manual's own
     description of odr_hz.

  2. A file is moved or renamed and the relative links to it are not.

  3. A document is installed to /usr/share/doc without the files it links
     to.  The install target already ships GOVERNANCE.md and DCO *solely*
     because README.md and CONTRIBUTING.md reference them by relative path —
     the Makefile says so in a comment — but nothing enforced it, and the
     reasoning was applied to two files out of the several that need it.

Checks 1 and 2 run over every tracked .md.  Check 3 (--installed) models
what `make install` actually lays down: the install lines FLATTEN paths, so
`docs/manual.md` on disk becomes `manual.md` in /usr/share/doc/imud, and a
README.md link that reads `docs/manual.md` dangles once installed even
though it resolves perfectly in the source tree.

Run as `make check-links`.  Pure text analysis, no build.
"""

import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, Report                              # noqa: E402

# Link targets that are deliberately not files: anchors handled separately,
# schemes we do not resolve, and the placeholder used in examples.
SKIP_SCHEME = re.compile(r"^(https?:|mailto:|ftp:|#|data:)")

# A fenced code block: ``` or ~~~ , optionally indented, with any info string.
FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})")

# Inline code spans, so `](foo)` inside backticks is not read as a link.
INLINE_CODE = re.compile(r"`[^`\n]*`")

# [text](target) — target runs to the first unescaped ')'.  Titles are not
# used anywhere in this tree, so the simple form is enough; a title would
# show up as a missing file and get looked at.
LINK = re.compile(r"\[[^\]\n]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")

# Reference-style definitions: [label]: target
REFDEF = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*(\S+)")


def tracked_md():
    """Every .md that ships.

    git is the authority when it is available, because it excludes the
    untracked working notes (AGENTS.md, audit.md) that live in this tree and
    are nobody's published documentation.  A plain walk is the fallback for
    the two places git is not available and the distinction does not arise: a
    fixture tree under test, and an extracted release tarball.
    """
    try:
        out = subprocess.run(["git", "-C", ROOT, "ls-files", "*.md"],
                             capture_output=True, text=True, check=True).stdout
        found = sorted(p for p in out.split("\n") if p)
        if found:
            return found
    except (OSError, subprocess.CalledProcessError):
        pass

    found = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d != ".git"]
        for name in filenames:
            if name.endswith(".md"):
                found.append(os.path.relpath(os.path.join(dirpath, name), ROOT))
    return sorted(found)


def blank_code(text, spans=True):
    """Same line count, fenced blocks emptied; inline spans too when asked.

    Line numbers have to survive so failures can name one.  manual.md has a
    shell comment `# or equivalently:` inside a fence that a naive heading
    scan reads as an H1, so fences always go.

    Inline spans are a different matter, and the distinction is load-bearing:
    links must have them removed, because `](foo)` inside backticks is prose
    about a link and not a link.  Headings must NOT, because the span content
    is part of the anchor — GitHub slugs "### The `actual_odr_hz` hook" to
    the-actual_odr_hz-hook, and blanking the span first yields the-hook and a
    confident report of a dead link that works perfectly.
    """
    out, fence = [], None
    for line in text.split("\n"):
        m = FENCE.match(line)
        if fence is None and m:
            fence = m.group(1)[0] * 3
            out.append("")
            continue
        if fence is not None:
            if line.strip().startswith(fence):
                fence = None
            out.append("")
            continue
        out.append(INLINE_CODE.sub("", line) if spans else line)
    return "\n".join(out)


def slug(text):
    """GitHub's heading -> anchor rule, as used by the repo's own links."""
    s = re.sub(r"`([^`]*)`", r"\1", text)                 # code spans keep content
    s = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", s)        # links keep their text
    # Asterisk emphasis only.  Underscore is NOT stripped: GitHub keeps it in
    # the anchor, and headings here name C identifiers — stripping it turns
    # "### The `actual_odr_hz` hook" into the-actualodrhz-hook and reports a
    # working link as dead.  Underscore emphasis would mis-slug, but this tree
    # uses asterisks and identifiers are the common case by far.
    s = re.sub(r"\*+", "", s)
    s = s.strip().lower()
    s = re.sub(r"[^\w\s-]", "", s, flags=re.UNICODE)      # drop punctuation
    return re.sub(r"\s+", "-", s)


def headings(text):
    """anchor -> lineno, with GitHub's -1/-2 suffixes for repeated headings."""
    anchors, seen = {}, {}
    for lineno, line in enumerate(text.split("\n"), 1):
        m = re.match(r"^(#{1,6})\s+(.*?)\s*#*\s*$", line)
        if not m:
            continue
        base = slug(m.group(2))
        if not base:
            continue
        n = seen.get(base, 0)
        seen[base] = n + 1
        anchors[base if n == 0 else f"{base}-{n}"] = lineno
    return anchors


def link_targets(text):
    """(lineno, target) for every link, code already blanked out."""
    found = []
    for lineno, line in enumerate(text.split("\n"), 1):
        for m in LINK.finditer(line):
            found.append((lineno, m.group(1)))
        m = REFDEF.match(line)
        if m:
            found.append((lineno, m.group(1)))
    return found


def installed_tree():
    """The layout `make install` lays down under $(DOCDIR).

    Returns (paths, sources):
      paths   — every installed file as "<pkg>/<name>", e.g. "imud/manual.md"
      sources — {"<pkg>/<name>": "<repo path>"} for the ones we can trace back

    Two things matter here and both bite.  The install lines FLATTEN:
    `install -m 644 docs/manual.md $(DOCDIR)/imud/` lands as imud/manual.md,
    so a README.md link reading `docs/manual.md` dangles once installed even
    though it resolves in the source tree.  But $(DOCDIR) is a SHARED parent,
    so `../libimud/README.md` from imud-utils resolves to libimud/README.md
    and is perfectly valid — which is why this models real paths rather than
    checking basenames within one package.
    """
    text = re.sub(r"\\\n\s*", " ",
                  open(os.path.join(ROOT, "Makefile"), encoding="utf-8").read())
    paths, sources = set(), {}

    def add(pkg, name, src=None):
        key = f"{pkg}/{name}"
        paths.add(key)
        if src:
            sources[key] = src

    # `install -m 644 <src>... $(DOCDIR)/<dir>/` — many sources into a dir.
    # <dir> may be nested (imud/docs, imud/devbox): the install deliberately
    # mirrors the source tree so relative links survive installation.
    for m in re.finditer(
            r"^\tinstall -m 644 (.+?) \$\(DESTDIR\)\$\(DOCDIR\)/([\w./-]+?)/\s*$",
            text, re.M):
        for f in m.group(1).split():
            add(m.group(2), os.path.basename(f), f)

    # `install -m 644 <src> $(DOCDIR)/<dir>/<name>` — single source, renamed.
    for m in re.finditer(
            r"^\tinstall -m 644 (\S+) \$\(DESTDIR\)\$\(DOCDIR\)/([\w./-]+)/([\w.-]+)\s*$",
            text, re.M):
        add(m.group(2), m.group(3), m.group(1))

    # `gzip -9nc <src> > $(DOCDIR)/<dir>/<name>.gz`
    for m in re.finditer(
            r"^\tgzip -9nc (\S+) > \$\(DESTDIR\)\$\(DOCDIR\)/([\w./-]+)/([\w.-]+)\s*$",
            text, re.M):
        add(m.group(2), m.group(3), m.group(1))

    return paths, sources


def main():
    argv = sys.argv[1:]
    check_installed = "--installed" in argv or not argv

    docs = tracked_md()
    raw = {p: open(os.path.join(ROOT, p), encoding="utf-8").read() for p in docs}
    text = {p: blank_code(t) for p, t in raw.items()}                 # for links
    anchors = {p: headings(blank_code(t, spans=False))                # for anchors
               for p, t in raw.items()}

    failures, checked = [], 0

    # ── 1 & 2: every link resolves in the source tree ────────────────────────
    for path in docs:
        base = os.path.dirname(path)
        for lineno, target in link_targets(text[path]):
            if SKIP_SCHEME.match(target) and not target.startswith("#"):
                continue
            checked += 1

            if target.startswith("#"):
                anchor = target[1:]
                if anchor not in anchors[path]:
                    failures.append(
                        f"{path}:{lineno}: dead anchor '{target}' "
                        f"— no heading in this file slugs to it")
                continue

            file_part, _, anchor = target.partition("#")
            dest = os.path.normpath(os.path.join(base, file_part))
            abs_dest = os.path.join(ROOT, dest)

            if not os.path.exists(abs_dest):
                failures.append(f"{path}:{lineno}: '{target}' — no such file {dest}")
                continue

            if anchor and dest in anchors:
                if anchor not in anchors[dest]:
                    failures.append(
                        f"{path}:{lineno}: '{target}' — {dest} has no heading "
                        f"slugging to '{anchor}'")

    # ── 3: links inside installed docs resolve in the INSTALLED tree ─────────
    if check_installed:
        paths, sources = installed_tree()
        if not paths:
            failures.append("Makefile: no $(DOCDIR) install lines found — "
                            "has the install target been restructured?")

        for key in sorted(sources):
            src = sources[key]
            if not src.endswith(".md") or src not in text:
                continue
            # Resolve links relative to the directory the doc is installed
            # IN, not to its package root: with docs/ preserved, imud/README.md
            # and imud/docs/manual.md have different neighbours.
            here = os.path.dirname(key)
            for lineno, target in link_targets(text[src]):
                if SKIP_SCHEME.match(target):
                    continue
                checked += 1
                file_part = target.partition("#")[0]
                if not file_part:
                    continue
                # Resolve against the package dir, the way a reader would:
                # $(DOCDIR) is shared, so ../<pkg>/x.md is legitimate.
                dest = os.path.normpath(os.path.join(here, file_part))
                if dest in paths:
                    continue
                if file_part.endswith("/"):
                    failures.append(
                        f"{src}:{lineno}: installed as {key}, links to directory "
                        f"'{target}' — /usr/share/doc/{dest} is not a doc dir")
                else:
                    # Name the fix precisely: resolve the link in the SOURCE
                    # tree, find where that exact file gets installed, and
                    # report the path the link would need to use.  Matching on
                    # basename instead would happily suggest imud-influxdb's
                    # manual.md for a link to the core manual.
                    repo_target = os.path.normpath(
                        os.path.join(os.path.dirname(src), file_part))
                    landed = [p for p, s in sources.items() if s == repo_target]
                    hint = ""
                    if landed:
                        rel = os.path.relpath(landed[0], here)
                        hint = (f" ({repo_target} installs to "
                                f"/usr/share/doc/{landed[0]}, so the link would "
                                f"need to read '{rel}')")
                    elif os.path.exists(os.path.join(ROOT, repo_target)):
                        hint = f" ({repo_target} is not installed by any package)"
                    failures.append(
                        f"{src}:{lineno}: installed as {key}, links to '{target}' "
                        f"but /usr/share/doc/{dest} is not installed{hint}")

    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(f"\n{len(failures)} problem(s) across {len(docs)} documents",
              file=sys.stderr)
        return 1

    print(f"check-links: {len(docs)} documents, {checked} links, all resolve")
    return 0


if __name__ == "__main__":
    sys.exit(main())
