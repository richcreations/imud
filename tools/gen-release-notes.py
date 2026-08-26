#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
gen-release-notes.py — render NEWS and the Debian changelogs from one registry.

A release is described in three places: NEWS for users upgrading, the
per-package changelog for packagers, and debian/changelog for the build.  Only
the last was ever machine-checked, and then only for its version number — so
NEWS drifted from terse feature notes into a record of committed churn, and by
1.9.0 it carried 2.6x the words of the same release's changelog for 1.3x the
items.

docs/release-notes.toml is now the one home for what a release changed.  This
renders the NEWS entries from GENERATED_FLOOR onward and the changelog stanza
for the version in include/version.h.

  --write   update the files on disk.
  (default) render and compare against what is on disk; print what drifted.
            This is what CI runs.

Two things are deliberately NOT generated:

  * NEWS entries below GENERATED_FLOOR.  Those releases shipped and users read
    them; rewriting them from a registry transcribed years later would alter
    the record rather than preserve it.
  * Changelog stanzas for versions already released.  The generator only ever
    touches the stanza for the CURRENT version, so shipped notes and their
    Debian trailers survive byte for byte.  Line 1 of every changelog stays
    `name (version) dist; urgency=...`, which ci.yml's version-consistency job
    and tools/bump-version.sh both parse.

The word cap is the point of the whole exercise: a change that cannot be said
in WORD_CAP words is either several changes or a story about how it was found,
and the second belongs in the commit message.
"""

import os
import re
import subprocess
import sys
import textwrap

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, Report                                # noqa: E402

REGISTRY   = os.path.join(ROOT, "docs", "release-notes.toml")
NEWS       = os.path.join(ROOT, "NEWS")
VERSION_H  = os.path.join(ROOT, "include", "version.h")
PACKAGING  = os.path.join(ROOT, "packaging")
DEB_CHANGE = os.path.join(ROOT, "debian", "changelog")

# NEWS entries from this version up are generated; older ones are hand-kept.
GENERATED_FLOOR = "1.7"
WORD_CAP        = 40
WIDTH           = 78

KINDS   = ("feature", "behaviour", "fix")
HEADING = {"feature": "Added", "behaviour": "Changed", "fix": "Fixed"}

# Tracks the WMM epoch, not the imud version — never gets an imud stanza.
EXEMPT_PACKAGES = {"imud-wmm-data"}

try:
    import tomllib
except ModuleNotFoundError:
    sys.exit(f"{REGISTRY} needs python3.11+ for tomllib")


def die(msg):
    sys.exit(f"gen-release-notes: {msg}")


def load_registry(rep):
    with open(REGISTRY, "rb") as f:
        reg = tomllib.load(f)
    releases = reg.get("release", [])
    rep.expect(releases, "releases in docs/release-notes.toml")

    known = {d for d in os.listdir(PACKAGING)
             if os.path.isdir(os.path.join(PACKAGING, d))}
    for rel in releases:
        v = rel.get("version", "?")
        for field in ("version", "date", "summary"):
            rep.check(field in rel, f"release {v} has no {field}")
        for ch in rel.get("change", []):
            rep.check(ch.get("kind") in KINDS,
                      f"{v}: kind must be one of {KINDS}, "
                      f"got {ch.get('kind')!r}")
            pkgs = ch.get("packages") or []
            rep.check(bool(pkgs), f"{v}: every change needs a package")
            for p in pkgs:
                rep.check(p not in EXEMPT_PACKAGES,
                          f"{v}: {p} tracks its own epoch and takes no stanza")
                rep.check(p in known or p in EXEMPT_PACKAGES,
                          f"{v}: no such package {p!r} under packaging/")
            text = (ch.get("text") or "").strip()
            rep.check(bool(text), f"{v}: a change has no text")
            n = len(text.split())
            rep.check(n <= WORD_CAP,
                      f"{v}: a change is {n} words, cap is {WORD_CAP} — say it "
                      f"shorter or split it; the detail belongs in the commit "
                      f"message: {text[:70]}...")
        rep.check(bool(rel.get("change")), f"release {v} lists no changes")
    return releases


def current_version():
    src = open(VERSION_H, encoding="utf-8").read()
    m = re.search(r'#define\s+IMUD_VERSION_STR\s+"([^"]+)"', src)
    if not m:
        die(f"{VERSION_H}: no IMUD_VERSION_STR")
    return m.group(1)


def vkey(v):
    return tuple(int(x) for x in re.findall(r"\d+", v))


# ── NEWS ────────────────────────────────────────────────────────────────────

def render_news_release(rel):
    out = [rel["version"], "-" * max(3, len(rel["version"]))]
    summary = " ".join(rel["summary"].split())
    out += textwrap.wrap(summary, WIDTH)

    for kind in KINDS:
        items = [c for c in rel["change"] if c["kind"] == kind]
        if not items:
            continue
        out += ["", HEADING[kind] + ":", ""]
        for ch in items:
            body = " ".join(ch["text"].split())
            out += textwrap.wrap(body, WIDTH, initial_indent="* ",
                                 subsequent_indent="  ")
            if ch.get("impact"):
                out += textwrap.wrap(" ".join(ch["impact"].split()), WIDTH,
                                     initial_indent="  ",
                                     subsequent_indent="  ")
            if ch.get("action"):
                out += textwrap.wrap("ACTION: " + " ".join(ch["action"].split()),
                                     WIDTH, initial_indent="  ",
                                     subsequent_indent="  ")
    return out


def render_news(releases):
    src = open(NEWS, encoding="utf-8").read().split("\n")

    # The header is everything before the first release heading.
    first = None
    for i, line in enumerate(src):
        if i + 1 < len(src) and re.fullmatch(r"\d+\.\d+(\.\d+)?", line.strip()) \
                and set(src[i + 1].strip()) == {"-"}:
            first = i
            break
    if first is None:
        die("NEWS: no release heading found")

    # The tail starts at the first release below GENERATED_FLOOR.
    floor = vkey(GENERATED_FLOOR)
    tail = None
    for i in range(first, len(src)):
        line = src[i].strip()
        if i + 1 < len(src) and re.fullmatch(r"\d+\.\d+(\.\d+)?", line) \
                and set(src[i + 1].strip()) == {"-"}:
            if vkey(line) < floor:
                tail = i
                break
    if tail is None:
        die(f"NEWS: no release below {GENERATED_FLOOR} — "
            f"the hand-kept tail is missing")

    body = []
    for rel in sorted(releases, key=lambda r: vkey(r["version"]), reverse=True):
        if vkey(rel["version"]) < floor:
            die(f"{rel['version']} is below the generated floor "
                f"{GENERATED_FLOOR}; move the floor or drop the entry")
        body += render_news_release(rel) + ["", ""]

    return "\n".join(src[:first] + body + src[tail:])


# ── Debian changelogs ───────────────────────────────────────────────────────

def render_bullets(rel, package):
    """The `  * ...` block for one package's stanza."""
    out = []
    for kind in KINDS:
        for ch in rel["change"]:
            if ch["kind"] != kind or package not in ch["packages"]:
                continue
            body = " ".join(ch["text"].split())
            if ch.get("impact"):
                body += "  " + " ".join(ch["impact"].split())
            if ch.get("action"):
                body += "  ACTION: " + " ".join(ch["action"].split())
            out += textwrap.wrap(body, 74, initial_indent="  * ",
                                 subsequent_indent="    ")
    return out


def split_top_stanza(path):
    """(header_line, bullet_lines, rest_lines) for a changelog."""
    lines = open(path, encoding="utf-8").read().split("\n")
    if not lines or "(" not in lines[0]:
        die(f"{path}: line 1 is not `name (version) dist; urgency=...`")
    end = None
    for i in range(1, len(lines)):
        if lines[i].startswith(" -- "):
            end = i
            break
    if end is None:
        die(f"{path}: top stanza has no ` -- name <email>  date` trailer")
    bullets = [l for l in lines[1:end] if l.strip()]
    return lines[0], bullets, lines[end:]


def stanza_version(header):
    m = re.match(r"^\S+\s+\(([^)]+)\)", header)
    return m.group(1) if m else None


def packages_for(rel):
    pkgs = {"imud"}
    for ch in rel["change"]:
        pkgs.update(ch["packages"])
    return sorted(pkgs)


def trailer_now():
    def cfg(k, default):
        try:
            v = subprocess.run(["git", "config", "--get", k], cwd=ROOT,
                               capture_output=True, text=True).stdout.strip()
            return v or default
        except OSError:
            return default
    name = cfg("user.name", "Richard Simpson")
    mail = cfg("user.email", "richcreations@gmail.com")
    date = subprocess.run(["date", "-R"], capture_output=True,
                          text=True).stdout.strip()
    return f" -- {name} <{mail}>  {date}"


# ── main ────────────────────────────────────────────────────────────────────

def main():
    write = "--write" in sys.argv[1:]
    rep = Report("gen-release-notes")
    releases = load_registry(rep)
    if rep.failures or rep.empty:
        return rep.finish("registry is not valid")
    ver = current_version()

    rel = next((r for r in releases if r["version"] == ver), None)
    if rel is None:
        rep.fail(f"include/version.h says {ver}, which "
                 f"docs/release-notes.toml does not describe")
        return rep.finish("registry does not describe the current version")

    # NEWS
    want = render_news(releases)
    have = open(NEWS, encoding="utf-8").read()
    if want != have:
        if write:
            open(NEWS, "w", encoding="utf-8").write(want)
            print(f"  wrote {os.path.relpath(NEWS, ROOT)}")
        else:
            rep.fail("NEWS does not match the registry")

    # Changelogs
    targets = [(p, os.path.join(PACKAGING, p, "changelog"))
               for p in packages_for(rel)]
    targets.append(("imud", DEB_CHANGE))

    for pkg, path in targets:
        if not os.path.exists(path):
            die(f"{path}: no such changelog")
        header, bullets, rest = split_top_stanza(path)
        want_bullets = render_bullets(rel, pkg)
        if not want_bullets:
            continue
        sv = stanza_version(header)
        # debian/changelog carries a -N revision; compare the upstream part.
        if (sv or "").split("-")[0] != ver:
            if write:
                name = os.path.basename(os.path.dirname(path))
                pv = ver + "-1" if path == DEB_CHANGE else ver
                new = [f"{pkg if path != DEB_CHANGE else 'imud'} ({pv}) "
                       f"unstable; urgency=medium", ""] + want_bullets + \
                      ["", trailer_now(), ""]
                old = open(path, encoding="utf-8").read()
                open(path, "w", encoding="utf-8").write(
                    "\n".join(new) + "\n" + old)
                print(f"  wrote {os.path.relpath(path, ROOT)} ({name})")
            else:
                rep.fail(f"{os.path.relpath(path, ROOT)}: top stanza is {sv}, "
                         f"registry describes {ver}")
            continue
        if bullets != want_bullets:
            if write:
                out = [header, ""] + want_bullets + [""] + rest
                open(path, "w", encoding="utf-8").write("\n".join(out))
                print(f"  wrote {os.path.relpath(path, ROOT)}")
            else:
                rep.fail(f"{os.path.relpath(path, ROOT)}: top stanza does not "
                         f"match the registry")

    n = sum(len(r["change"]) for r in releases)
    if write:
        rep.failures = []
    return rep.finish(f"{len(releases)} releases, {n} changes, NEWS from "
                      f"{GENERATED_FLOOR}, cap {WORD_CAP} words "
                      f"(edit docs/release-notes.toml, not the output)")


if __name__ == "__main__":
    sys.exit(main())
