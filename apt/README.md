# imud apt repository

Signed Debian packages served from GitHub Pages at
<https://richcreations.github.io/imud/apt/>. The
`.github/workflows/apt-repo.yml` workflow assembles and signs the published
repository.

**The `.deb` files are not in this directory, and not on `main`.** They live on
the orphan branch **`apt-pool`**, under `pool/<suite>/`. Everything else — the
key, the landing page, this file — stays on `main`:

| where | what |
|---|---|
| `main:apt/KEY.gpg` | the **public** signing key, served for users to trust |
| `main:apt/index.html` | the landing/instructions page (served at `/apt/`) |
| `main:web/` | the site root (`/imud/`); apt is namespaced under `/apt/` so a project page can own the root |
| `apt-pool:pool/<suite>/` | the `.deb` files |
| *generated* | `dists/` and `site/` — built by CI, never committed |

## Why the pool is on its own branch

Binaries on the default branch are permanent: git keeps every version of every
file forever, so each release added ~1.5 MB that could never be reclaimed. By
1.7 that had put **54 MB of removed datasheets and 6.7 MB of debs** into a
75 MB repository, pushed the `make dist` **source** tarball to 6.1 MB (it was
shipping the previous three releases' binaries inside itself), and scored 0 on
OpenSSF Scorecard's Binary-Artifacts check with 114 findings.

`apt-pool` is rebuilt as a **single root commit and force-pushed** on every
publish, so it holds today's pool and no history of it. The repository is now
5.5 MB and the tarball 0.8 MB, and neither grows with releases.

The consequence to know about: **there is no git history of the pool.** A deb
pruned from it is gone from the branch — but every deb ever built is still
attached to its GitHub Release, which is now the archive of record.

## Publishing a package (the routine)

**Publish the draft GitHub Release.** That is the whole routine.

Tagging `vX.Y` makes `release.yml` build the debs and open a *draft* release.
When you review it and click **Publish**, `.github/workflows/apt-publish.yml`
takes over: it checks out `apt-pool`, downloads the release's `.deb` assets,
routes each into `pool/bookworm/` or `pool/trixie/` by its `~bookworm1` /
`~trixie1` suffix, prunes to the three most recent versions per suite, rebuilds
the branch as a fresh root commit, force-pushes it, and then triggers
`apt-repo.yml` to regenerate and GPG-sign the index and redeploy Pages.

It refuses to publish anything whose version disagrees with the tag, refuses to
push an empty pool, and skips pre-releases entirely.

Two deliberate exclusions:

- **`-dbgsym` packages never enter the pool.** `dh_strip` builds one per binary
  package and they ride along as Release assets, which is where they stay —
  the same split Debian makes with its separate `debian-debug` archive. Fetch
  them from the Release when you need to symbolize a crash.
- **Only the newest three versions stay installable.** Older debs are pruned on
  each publish, keeping the Pages payload and the generated index bounded.
  Anyone pinned to a pruned version loses their install source; the debs
  themselves remain on their GitHub Release.

### Re-running or doing it by hand

`apt-publish.yml` can be re-run from the Actions tab (**Run workflow**) with a
tag, a pool branch, and a keep-count.

**To rehearse**, give it a `pool_branch` other than `apt-pool` — say
`apt-pool-rehearsal`. It rebuilds *that* branch, and `apt-repo.yml` will build
and GPG-sign the index from it but **refuse to deploy**: its deploy gate
requires both `github.ref == main` *and* `pool_ref == apt-pool`. The live site
is untouched. Delete the rehearsal branch afterwards.

The manual path still works, it just targets the other branch: check out
`apt-pool`, drop `.deb` files into `pool/<suite>/`, commit, push, then run
**apt repo** from the Actions tab (a push to `apt-pool` does not trigger it —
only `apt/**` and `web/**` on `main` do).

Only arm64/armhf + arch-`all` debs belong here — the packages target the Pi
(64- and 32-bit Raspberry Pi OS). Both arches share one pool directory; CI
splits the per-arch indexes by filename, and each arch index carries the
arch-`all` debs.

There is exactly **one** armhf deb per version, built on Raspbian at the ARMv6
baseline so it serves every 32-bit Pi. Never add a second armhf build at
Debian's ARMv7 baseline: both ports report `armhf`, so apt would see two
packages distinguished only by version and would hand some machines a binary
their CPU cannot execute.

## One-time setup (do this once to bring the repo live)

### 1. Generate a dedicated signing key

A fresh key **only** for signing this repo — not your personal GPG identity.
Ed25519, no passphrase (an unattended CI signer can't type one; the GitHub
Actions secret store is what protects it).

```sh
cat > /tmp/imud-key.batch <<'EOF'
%no-protection
Key-Type: eddsa
Key-Curve: ed25519
Name-Real: imud package signing
Name-Email: richcreations@gmail.com
Expire-Date: 0
%commit
EOF
gpg --batch --gen-key /tmp/imud-key.batch
rm /tmp/imud-key.batch

# grab the fingerprint
FPR=$(gpg --list-secret-keys --with-colons \
      | awk -F: '/^fpr:/{print $10; exit}')
echo "$FPR"
```

### 2. Commit the public key

```sh
gpg --armor --export "$FPR" > apt/KEY.gpg
# commit apt/KEY.gpg
```

### 3. Add the private key as a repository secret

```sh
gpg --armor --export-secret-keys "$FPR"
```

Copy the entire block (including the `-----BEGIN/END PGP PRIVATE KEY BLOCK-----`
lines) into **Settings → Secrets and variables → Actions → New repository
secret**, named exactly `APT_GPG_PRIVATE_KEY`.

### 4. Turn on Pages with the Actions source

**Settings → Pages → Build and deployment → Source: GitHub Actions.**

### 5. Create the pool branch and publish

The pool branch is an orphan — no shared history with `main`:

```sh
git checkout --orphan apt-pool
git rm -rf --cached . && rm -rf ./*
mkdir -p pool/bookworm pool/trixie
# drop the .deb files in, then:
git add pool && git commit -m "apt pool @ X.Y"
git push --force origin apt-pool
git checkout main
```

Then run **apt repo** from the Actions tab. When it deploys, the repository is
live and installable per the instructions on the landing page. After this,
`apt-publish.yml` maintains the branch for you and you should not need to
touch it by hand.

## Rotating / revoking the key

If the signing key is ever exposed: generate a new one (steps 1–3), replace
`apt/KEY.gpg` and the `APT_GPG_PRIVATE_KEY` secret, and re-run the workflow.
Users re-import the new `KEY.gpg`. The key signs only package indexes, so the
blast radius is limited to that.
