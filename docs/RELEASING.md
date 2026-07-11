# Releasing imud

Checklist for cutting release X.Y. The canonical version lives in ONE place —
`include/version.h` — everything else follows it.

1. **Bump the version**: `include/version.h` → `#define IMUD_VERSION_STR "X.Y"`.
   (All five daemons — imud + the four bridges — report this via `--version`.
   The *wire* version `IMUD_VERSION` in `include/types.h` / `lib/imud_client.h`
   is separate and changes only when the packet layout changes.)
2. **NEWS**: add an `X.Y` section at the top — user-visible changes only.
3. **Changelogs**: add an `imud (X.Y)` stanza to `packaging/imud/changelog`,
   and to each `packaging/imud-<bridge>/changelog` whose package changed.
   Trailer format is Debian's: `-- Name <email>  Day, DD Mon YYYY HH:MM:SS ±ZZZZ`.
4. **Man pages**: update every `.TH` line to `"imud X.Y"` and the release date
   in ISO form, e.g. `"2026-07-10"`:
   `sed -i 's/"imud [0-9.]*"/"imud X.Y"/; s/"[0-9]\{4\}-[0-9]\{2\}-[0-9]\{2\}"/"YYYY-MM-DD"/' man/man*/*`
   then `mandoc -Tlint man/man*/*` must be clean.
5. **Verify**: `make clean && make && make bridges && make test` (on the Pi or
   CI for the full set), and `mandoc -Tlint man/man*/*`.
6. **Commit and tag**: commit the release, then `git tag vX.Y` and push the tag.
7. **Tarball**: `make dist` → `imud-X.Y.tar.gz` (built from git HEAD, so tag
   first or at the same commit). This is the upstream release artifact — for
   Debian packaging it later becomes `imud_X.Y.orig.tar.gz`.

Deployment reminder: the daemon, all bridges, and imud-mon validate the wire
version and must be rebuilt/deployed together when `IMUD_VERSION` changes.
