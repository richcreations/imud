#!/bin/sh
# Regenerate docs/math.pdf from docs/math.md.
#
# Lives in tools/ with the other generators, not in docs/ beside its output:
# docs/ is a published tree — most of it installs to /usr/share/doc/imud/ —
# and a maintainer script that needs pandoc and a TeX engine is not
# documentation. It was the last generator outside tools/.
#
# math.md is the canonical source; the PDF is a rendered convenience copy for
# reading the derivations away from a terminal. Rebuild it after editing
# math.md — `make math-pdf`, or run this directly.
#
# ENGINE. tectonic first, xelatex second. This used to hard-require xelatex,
# which is TeX Live or MacTeX — about 4 GB for one PDF — and the result was
# predictable: the script failed silently on any box without it and math.pdf
# sat three weeks behind math.md. tectonic is a single ~30 MB binary that
# fetches only the packages a document actually uses, so the PDF can be
# rebuilt on the machine where math.md is edited. xelatex is kept because a
# box that already has TeX Live should not need a second toolchain.
#
# A preprocessing pass maps the handful of prose Unicode glyphs that the
# Latin Modern text font lacks (⁻¹ ⁻² ↔ ⇒ ≤ ≥ σ ν) to inline-math
# equivalents, so no extra LaTeX package (e.g. newunicodechar) is needed on a
# minimal install. All math/tables/code pass straight through.
#
# The Greek pair are there because the engine DROPPED them: "Gauss–Markov
# wave σ (m·s⁻²)" printed as "Gauss–Markov wave  (m·s⁻²)", losing the symbol
# the row is about. Each occurs exactly once, in a table cell, outside any
# $...$ — so a blind substitution is safe here and would not be if either
# were ever written inside math, where \sigma and \nu are what the source
# uses anyway.
#
# STAMP. On success this writes docs/math.pdf.stamp, holding the SHA-256 of
# the math.md the PDF was built from. tools/check-math-pdf-stamp.py compares
# it against math.md and fails when they differ, which is what makes a stale
# PDF loud. A timestamp comparison cannot do this job: git does not preserve
# mtimes, so every fresh clone has a math.pdf that looks either newer or
# older than its source at random.
set -e

root=$(cd "$(dirname "$0")/.." && pwd)
src="$root/docs/math.md"
out="$root/docs/math.pdf"
stamp="$root/docs/math.pdf.stamp"
tmp=$(mktemp -t imud_math.XXXXXX.md)
trap 'rm -f "$tmp"' EXIT

command -v pandoc >/dev/null 2>&1 || { echo "error: pandoc not found" >&2; exit 1; }

if command -v tectonic >/dev/null 2>&1; then
    engine=tectonic
elif command -v xelatex >/dev/null 2>&1; then
    engine=xelatex
else
    echo "error: no PDF engine found — install tectonic (brew install tectonic," >&2
    echo "       a single binary) or a XeLaTeX from TeX Live / MacTeX" >&2
    exit 1
fi
echo "engine: $engine"

perl -CSD -pe '
  s/\x{207B}\x{00B9}/\$^{-1}\$/g;      # rad·s⁻¹ etc.
  s/\x{207B}\x{00B2}/\$^{-2}\$/g;      # m·s⁻²
  s/\x{2194}/\$\\leftrightarrow\$/g;   # ↔
  s/\x{21D2}/\$\\Rightarrow\$/g;       # ⇒
  s/\x{2264}/\$\\le\$/g;               # ≤
  s/\x{2265}/\$\\ge\$/g;               # ≥
  s/\x{2248}/\$\\approx\$/g;           # ≈
  s/\x{03C3}/\$\\sigma\$/g;            # σ in prose
  s/\x{03BD}/\$\\nu\$/g;               # ν in prose
' "$src" > "$tmp"

# Nothing may be dropped silently. The engine warns per occurrence when a
# glyph is missing from the text font and then prints NOTHING in its place —
# "NIS ≈ 56" became "NIS  56" — so the warning is a correctness failure, not
# a cosmetic one, and it is caught here rather than left in the scrollback.
check_glyphs() {
    missing=$(grep -oE 'could not represent character "[^"]+" \(0x[0-9a-f]+\)' \
              "$1" | sort -u || true)
    [ -z "$missing" ] && return 0
    echo "error: the PDF would silently DROP these characters:" >&2
    echo "$missing" | sed 's/^/       /' >&2
    echo "       Add each to the substitution pass above." >&2
    return 1
}

log=$(mktemp -t imud_math_log.XXXXXX)
trap 'rm -f "$tmp" "$log"' EXIT

pandoc "$tmp" -o "$out" \
  --from gfm+tex_math_dollars \
  --pdf-engine="$engine" \
  --toc --toc-depth=2 \
  -V geometry:margin=1in -V fontsize=10pt \
  -V colorlinks=true -V linkcolor=RoyalBlue -V urlcolor=RoyalBlue -V toccolor=black \
  2>&1 | tee "$log"

check_glyphs "$log"

# Written only after pandoc succeeds: a stamp for a PDF that was never
# produced would report freshness that does not exist.
if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$src" | cut -d' ' -f1 > "$stamp"
else
    shasum -a 256 "$src" | cut -d' ' -f1 > "$stamp"   # macOS
fi

echo "wrote $out"
echo "wrote $stamp ($(cat "$stamp"))"
