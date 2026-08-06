# Governance

imud is a **single-maintainer project**, run on what is usually called the
benevolent-dictator model: one person holds final say on every technical and
scope decision, works in the open, and explains the reasoning. This document
says who that is, what the roles are, how decisions actually get made, and what
would happen to the project if the maintainer stopped.

It is deliberately short. A project this size does not need a constitution; it
needs an honest description of how things really work, so that nobody has to
guess.

## Roles

### Maintainer

**Richard Simpson** (<richcreations@gmail.com>, `@richcreations`) — sole
maintainer, and author of the daemon, the fusion filter, the drivers, the
bridges, the client libraries, and the documentation. See [AUTHORS](AUTHORS).

The role carries these tasks, and there is nobody else to hand them to:

- **Decide scope.** What belongs in imud and what does not (see below).
- **Review and merge.** Every pull request is reviewed and merged by the
  maintainer. Nothing lands on `main` without that.
- **Triage.** Read and answer issues, reproduce bugs, and decide what is a
  defect, what is deferred to [docs/ROADMAP.md](docs/ROADMAP.md), and what is
  out of scope.
- **Release.** Cut versions per [docs/RELEASING.md](docs/RELEASING.md), and
  own the apt repository and its signing key.
- **Handle security reports.** Run the private disclosure process in
  [SECURITY.md](SECURITY.md): acknowledge, assess, fix, credit.
- **Enforce the Code of Conduct.** See
  [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md); the maintainer is the point of
  contact and the one who acts on a report.

In GitHub terms this is literal rather than nominal: `@richcreations` is the
only account with write or administrative access to the repository, to the
`apt-pool` branch that holds the published `.deb` files, and to the Pages
deployment. There are no other collaborators, and the repository is owned by a
personal account rather than by an organization that could grant access to
someone else. See "Continuity, and the bus factor" below for what that means.

### Contributor

Anyone who opens an issue or a pull request. No application, invitation, or
prior involvement is needed, and a first-time contributor's patch is judged the
same way as any other.

Contributors are expected to follow [CONTRIBUTING.md](CONTRIBUTING.md): keep
changes bite-sized, keep the tree warning-clean, add or extend a test, update
the parallel documentation in the same change, and sign off every commit under
the [DCO](DCO). Contributors keep their copyright; the sign-off certifies the
right to submit, and assigns nothing.

### Security reporter

Anyone reporting a vulnerability through the private channel in
[SECURITY.md](SECURITY.md). They are credited in the `NEWS` entry for the fix
unless they ask not to be.

There are no other roles. There is no committee, no vote, no formal membership,
and no seat that is merely unfilled — one person does all of it.

## How decisions are made

The maintainer decides, after discussion in the open on the issue or pull
request. Disagreement is resolved by argument and evidence rather than by
seniority: a benchmark, a failing test, a capture, or a citation moves a
decision far more reliably than a strong opinion, and a contributor who brings
one has changed decisions here before.

If a disagreement does not resolve, the maintainer's decision stands and the
reasoning is recorded — in the pull request, and in the commit message if the
change lands. Anyone who disagrees with the outcome is free to fork; the MIT
license exists precisely so that a disagreement never has to become a
negotiation.

Some standing decisions govern what is accepted, and a change that violates one
will be turned down regardless of how good the code is:

- **imud is general-purpose.** It is *gpsd for IMUs*, a Linux daemon that owns
  an inertial sensor and publishes a fused estimate on standard interfaces.
  Marine navigation is the most exercised use case, not the project's identity;
  robotics, drones, camera stabilization, and pointing rigs are equal citizens.
- **The core takes no new external dependencies.** It is C11 and POSIX against
  `libgpiod` and libc. An optional bridge may take one dependency, confined to
  that bridge's own package.
- **The wire format and the libimud ABI are contracts.** `imud_data_t` is
  append-only, `lib/libimud.map` only grows, and the packet version is bumped
  when the layout changes. Consumers survive daemon upgrades without
  recompiling; that promise is not traded away for convenience.
- **Documentation is part of the change, not a follow-up.** The same fact lives
  in the man pages, the manual, the protocol spec, the client libraries, and
  the tests; they move together or the change is incomplete.

Decisions are recorded where they can be found later: the commit message for
why a change is the way it is, `NEWS` for what a release changed, and
[docs/ROADMAP.md](docs/ROADMAP.md) for what was deliberately deferred.

## How a change gets in

1. A contributor opens a pull request (see [CONTRIBUTING.md](CONTRIBUTING.md)).
2. CI must be green. The gates are not advisory and the maintainer does not
   merge around them: build and test on amd64 and arm64, a 32-bit armhf run
   under UBSan, ASan/UBSan and TSan suites, CodeQL, cppcheck and scan-build,
   the fuzzers, the workflow linter, a DCO sign-off check, a staged-install
   check, and a native install of the built `.deb`s.
3. The maintainer reviews and merges, or explains why not.

Coverage is measured and reported but deliberately not gated on a threshold, so
a number never substitutes for judgment about what a test is worth.

## Continuity, and the bus factor

**The bus factor is 1.** This is stated plainly rather than managed, because
the project's own [OpenSSF Best Practices](https://www.bestpractices.dev/projects/13917)
entry records it as unmet and pretending otherwise would be worse than the
gap itself.

What that does and does not put at risk:

- **The code is safe.** It is MIT-licensed, and its full history is on GitHub
  and in every clone. Anyone can fork it and continue, with no permission
  required from anyone.
- **Releases and the apt repository are the fragile part.** They depend on
  access to the GitHub account and repository. The signing and publishing
  machinery, however, does *not* live on any one laptop: releases are built,
  attested, and published by GitHub Actions, the Pages deployment uses an OIDC
  token minted per run, and the apt repository is signed inside CI from the
  `APT_GPG_PRIVATE_KEY` repository secret. A successor with administrative
  access to the repository can therefore cut a release and publish the apt
  repository without recovering anything from the maintainer's machine.
- **A fork's users can be migrated.** The apt repository is a static tree
  served from GitHub Pages, and every release is also downloadable from the
  GitHub Releases page with build provenance attached, so nothing about the
  distribution path is locked to one host.

**A co-maintainer would be welcome, and is the intended fix.** There is no
quota and no waiting period; the path is the ordinary one — contribute
substantially over a few releases, review other people's changes, and show the
judgment the role needs about what *not* to accept. Ask, or just start; the
maintainer will raise it with anyone who is obviously doing the work.

## Changing this document

The maintainer amends it, in a pull request like any other change, when the way
the project actually runs stops matching what is written here. If a second
maintainer joins, this document changes first.
