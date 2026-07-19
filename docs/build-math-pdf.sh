#!/bin/sh
# Regenerate docs/math.pdf from docs/math.md.
#
# Requires pandoc and a XeLaTeX engine (TeX Live / MacTeX). math.md is the
# canonical source; the PDF is a rendered convenience copy — rebuild it after
# editing math.md.
#
# A preprocessing pass maps the handful of prose Unicode glyphs that the
# Latin Modern text font lacks (⁻¹ ⁻² ↔ ⇒ ≤ ≥) to inline-math equivalents,
# so no extra LaTeX package (e.g. newunicodechar) is needed on a minimal
# BasicTeX install. All math/tables/code pass straight through.
set -e

here=$(cd "$(dirname "$0")" && pwd)
src="$here/math.md"
out="$here/math.pdf"
tmp=$(mktemp -t imud_math.XXXXXX.md)
trap 'rm -f "$tmp"' EXIT

command -v pandoc   >/dev/null 2>&1 || { echo "error: pandoc not found" >&2; exit 1; }
command -v xelatex  >/dev/null 2>&1 || { echo "error: xelatex not found (install MacTeX/TeX Live)" >&2; exit 1; }

perl -CSD -pe '
  s/\x{207B}\x{00B9}/\$^{-1}\$/g;      # rad·s⁻¹ etc.
  s/\x{207B}\x{00B2}/\$^{-2}\$/g;      # m·s⁻²
  s/\x{2194}/\$\\leftrightarrow\$/g;   # ↔
  s/\x{21D2}/\$\\Rightarrow\$/g;       # ⇒
  s/\x{2264}/\$\\le\$/g;               # ≤
  s/\x{2265}/\$\\ge\$/g;               # ≥
' "$src" > "$tmp"

pandoc "$tmp" -o "$out" \
  --from gfm+tex_math_dollars \
  --pdf-engine=xelatex \
  --toc --toc-depth=2 \
  -V geometry:margin=1in -V fontsize=10pt \
  -V colorlinks=true -V linkcolor=RoyalBlue -V urlcolor=RoyalBlue -V toccolor=black

echo "wrote $out"
