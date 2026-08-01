# Releasing imud

Checklist for cutting release X.Y. The canonical version lives in ONE place —
`include/version.h` — everything else follows it.

CI enforces this: the **version-consistency** job in `.github/workflows/ci.yml`
fails if `include/version.h`, `debian/changelog`, `packaging/imud/changelog` and
the man page `.TH` lines disagree, and the **Release** workflow refuses to build
if the tag does not match `include/version.h`. So a half-finished bump cannot
reach the apt repository.

## 1. Bump the version strings

```sh
make bump-version VERSION=X.Y            # DATE=YYYY-MM-DD to override "today"
```

`tools/bump-version.sh` rewrites `include/version.h`, every man page `.TH` line
(version + date), the `imud X.Y` / date footers and JSON-LD `softwareVersion` in
`web/index.html` and `apt/index.html`, `web/sitemap.xml`'s `lastmod`, and the
`**Version:**` header in `spec.md`. It then lists the changelogs that still need
a stanza. It is idempotent, so re-running it is safe.

(All six daemons — imud + the five bridges — report this version via
`--version`. The *wire* version `IMUD_VERSION` in `include/types.h` /
`lib/imud_client.h` is separate and changes only when the packet layout does.)

## 2. Write the prose — the part no script can do

- **`NEWS`**: add an `X.Y` section at the top, user-visible changes only.
- **`debian/changelog`**: add an `imud (X.Y-1) unstable; urgency=medium` stanza.
  This drives the .deb version and **must** match `include/version.h`.
  (`dch -v X.Y-1` writes the trailer for you.) The per-dist suffix
  (`X.Y-1~bookworm1`) is added by CI, not committed.
- **`packaging/imud/changelog`**: an `imud (X.Y)` stanza — always required.
- **`packaging/imud-<bridge>/changelog`**: a stanza only for packages that
  actually changed. `packaging/imud-wmm-data/changelog` is exempt — it tracks
  the WMM epoch (2025.0), not the imud version.

Trailer format is Debian's:
`-- Name <email>  Day, DD Mon YYYY HH:MM:SS ±ZZZZ`

## 3. Verify

```sh
make clean && make && make bridges && make test
mandoc -Tlint -Wwarning man/man*/*
```

Push the branch and let CI do the rest — it covers amd64, arm64 and 32-bit
armhf, the sanitizers, the fuzzers, static analysis, and a full .deb build for
both suites.

## 4. Tag

```sh
git tag vX.Y && git push origin vX.Y
```

That fires `.github/workflows/release.yml`, which checks the tag against
`include/version.h`, builds .debs for bookworm + trixie × arm64 + armhf, records
build provenance, and opens a **draft** GitHub Release with the debs and the
`make dist` tarball attached.

## 5. Publish — this is the gate

Review the draft release, write the notes, and click **Publish**.

Publishing is what promotes the packages: it fires
`.github/workflows/apt-publish.yml`, which routes the debs into
`pool/<suite>/` on the orphan **`apt-pool`** branch, prunes to the three most
recent versions per suite, rebuilds that branch as a single root commit,
force-pushes it, and triggers `apt-repo.yml` to rebuild and GPG-sign the index
and redeploy Pages. Nothing reaches users before this click.

The pool is deliberately **not** on `main`: binaries there are permanent, and
by 1.7 they had made the repository 75 MB and the source tarball 6.1 MB. See
`apt/README.md` for the full rationale and the rehearsal procedure.

Automatic `-dbgsym` packages stay attached to the Release and are deliberately
kept **out** of the apt pool — the same split Debian makes with its separate
`debian-debug` archive. Download them from the Release when you need to
symbolize a crash report.

If anything goes wrong, `apt-publish.yml` can be re-run by hand from the
Actions tab (**Run workflow** → tag), and `apt/README.md` documents the fully
manual fallback.

## libimud ABI / soname discipline

The public ABI is `imud.h`: the `imud_*` functions (kept complete in
`lib/libimud.map`) and the **append-only** `imud_data_t`. The rules — how to
add a wire field, when the SONAME may move, and what must never change — are
specified once in **[libimud/spec.md](libimud/spec.md)** ("ABI contract" and
"Maintainer discipline"); follow them there rather than duplicating them here.

Release-relevant summary: appending a member to `imud_data_t` or a function to
`lib/libimud.map` keeps SONAME `libimud.so.0`, so a normal release never
touches it. Only a reorder/retype/removal would force `libimud.so.1` and a
runtime package rename (libimud0 → libimud1) — that should not happen.

Deployment reminder: the daemon, all bridges, and imud-mon validate the wire
version and must be rebuilt/deployed together when `IMUD_VERSION` changes.
Third-party libimud consumers do NOT need rebuilding — that is the point of
the library — but the installed libimud.so must be upgraded with the daemon.
