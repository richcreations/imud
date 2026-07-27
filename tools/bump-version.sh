#!/bin/sh
#
# Propagate a release version across the files that carry it.
#
# The canonical version lives in ONE place — include/version.h — but ~20
# other files repeat it, and doing that by hand is how a release ends up
# half-bumped.  ci.yml's version-consistency job fails on exactly that; this
# script is the other half, the thing that makes it pass.
#
# Usage:  make bump-version VERSION=1.8 [DATE=2026-08-01]
#     or: tools/bump-version.sh 1.8 [2026-08-01]
#
# What it does NOT do: write changelog prose.  debian/changelog and
# packaging/<pkg>/changelog need a human-written stanza describing what
# actually changed, so the script reports which ones are stale and leaves
# them to you.  It also does not touch NEWS, commit, or tag.
#
# Portable /bin/sh + POSIX sed: the maintainer works on macOS and releases
# on Debian, and `sed -i` is spelled differently on the two.  Every edit
# goes through subst(), which writes a temp file and moves it into place.
set -eu

VERSION="${1:-}"
DATE="${2:-$(date +%Y-%m-%d)}"

if [ -z "$VERSION" ]; then
    echo "usage: $0 <version> [YYYY-MM-DD]" >&2
    echo "   e.g. $0 1.8" >&2
    exit 1
fi

case "$VERSION" in
    [0-9]*.[0-9]*) ;;
    *) echo "error: '$VERSION' does not look like a version (expected X.Y or X.Y.Z)" >&2; exit 1 ;;
esac
case "$DATE" in
    [0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]) ;;
    *) echo "error: '$DATE' is not an ISO date (YYYY-MM-DD)" >&2; exit 1 ;;
esac

cd "$(dirname "$0")/.."

changed=0

# Apply a sed expression to a file, reporting whether it altered anything.
subst() {
    _expr=$1
    _file=$2
    [ -f "$_file" ] || { echo "  ?? missing: $_file" >&2; return 0; }
    sed "$_expr" "$_file" > "$_file.tmp$$"
    if cmp -s "$_file" "$_file.tmp$$"; then
        rm -f "$_file.tmp$$"
    else
        mv "$_file.tmp$$" "$_file"
        echo "  updated $_file"
        changed=$((changed + 1))
    fi
}

echo "Bumping to $VERSION ($DATE)"

# ── The canonical definition ─────────────────────────────────────────────
subst "s/^#define IMUD_VERSION_STR .*/#define IMUD_VERSION_STR \"$VERSION\"/" \
      include/version.h

# ── Man pages: the .TH line carries both version and release date ────────
# Not always line 1 ('\" t preambles), so match the .TH line itself.
for f in man/man1/*.1 man/man3/*.3 man/man5/*.5 man/man8/*.8; do
    [ -e "$f" ] || continue
    subst "/^\.TH/{
             s/\"imud [0-9][0-9.]*\"/\"imud $VERSION\"/
             s/\"[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]\"/\"$DATE\"/
           }" "$f"
done

# ── The published site ───────────────────────────────────────────────────
# Both pages carry a "imud X.Y" / date footer bar; web/index.html also has
# a JSON-LD softwareVersion.  These patterns are deliberately narrow — the
# pages are full of other numbers.
for f in web/index.html apt/index.html; do
    subst "s|<span>imud [0-9][0-9.]*</span>|<span>imud $VERSION</span>|
           s|<span>[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]</span>|<span>$DATE</span>|" "$f"
done
subst "s|\"softwareVersion\": \"[0-9][0-9.]*\"|\"softwareVersion\": \"$VERSION\"|" \
      web/index.html
subst "s|<lastmod>[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]</lastmod>|<lastmod>$DATE</lastmod>|g" \
      web/sitemap.xml

# ── spec.md ──────────────────────────────────────────────────────────────
# ONLY the header field.  spec.md discusses version numbers throughout
# ("the pre-1.7 6-state filter"), and a looser pattern would rewrite prose.
subst "s|^\*\*Version:\*\* [0-9][0-9.]*|**Version:** $VERSION|" spec.md

echo "$changed file(s) changed."

# ── Report what still needs a human ──────────────────────────────────────
# Changelog stanzas describe what changed; only you can write those.
echo
echo "Still needs a hand-written stanza (ci.yml's version-consistency job checks these):"
stale=0

head1=$(sed -n '1s/^[^(]*(\([^)]*\)).*/\1/p' debian/changelog 2>/dev/null || true)
if [ "${head1%%-*}" != "$VERSION" ]; then
    echo "  debian/changelog          — at ${head1:-?}, needs an 'imud ($VERSION-1) ...' stanza"
    stale=$((stale + 1))
fi

for f in packaging/*/changelog; do
    [ -e "$f" ] || continue
    case "$f" in */imud-wmm-data/*) continue ;;   # tracks the WMM epoch, not imud
    esac
    v=$(sed -n '1s/^[^(]*(\([^)]*\)).*/\1/p' "$f" 2>/dev/null || true)
    if [ "$v" != "$VERSION" ]; then
        pkg=$(basename "$(dirname "$f")")
        # Only packaging/imud must always be current; bridges get a stanza
        # only when they actually changed.
        if [ "$pkg" = "imud" ]; then
            echo "  $f — at ${v:-?}, REQUIRED"
        else
            echo "  $f — at ${v:-?}, only if $pkg changed"
        fi
        stale=$((stale + 1))
    fi
done

[ "$stale" -eq 0 ] && echo "  (none — all changelogs already at $VERSION)"

echo
echo "Also by hand: NEWS (user-visible changes for $VERSION)."
echo "Then: make test && mandoc -Tlint -Wwarning man/man*/*  before tagging."
