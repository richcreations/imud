# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
# Shared apt policy and dependency set for the Debian containers CI runs its
# gates in.  Sourced, never executed:
#
#     . .github/ci-setup.sh
#     apt_setup lcov          # base deps, plus whatever this job needs
#
# Two different failures need two different guards.  Acquire::Retries covers a
# refused or dropped CONNECTION; it does not cover a download that completes
# short, which surfaces as a hash mismatch and fails the whole apt-get.  The
# apt_retry wrapper covers that second case by re-running the command.  The
# measurements behind both are in build-debs.yml.
#
# The dependency set lives here for the same reason the retry policy does: it
# was stated in six places with drifting contents, so a new dependency could be
# added to five gates and missed in the sixth.

printf '%s\n' 'Acquire::Retries "5";' \
              'Acquire::http::Timeout "60";' \
              'Acquire::https::Timeout "60";' \
    > /etc/apt/apt.conf.d/99retries

apt_retry() {
    for i in 1 2 3; do
        "$@" && return 0
        echo "apt attempt $i failed; retrying in $((i * 10))s" >&2
        sleep $((i * 10))
    done
    return 1
}

# What every gate needs to build imud and run the suite.  python3 is here
# because `make test` runs the generated-text checkers and the Python client
# self-check, not merely because the tools are written in it.
IMUD_BUILD_DEPS=(build-essential pkgconf libgpiod-dev libmosquitto-dev python3)

# apt_setup [extra packages...]
apt_setup() {
    apt_retry apt-get update
    apt_retry apt-get install -y --no-install-recommends \
        "${IMUD_BUILD_DEPS[@]}" "$@"
}

# The libFuzzer runtime is a version-suffixed package (libclang-rt-19-dev on
# trixie) with no unversioned alias, so resolve it from the package lists by
# the major version of the clang that will link against it.  Never take the
# newest match instead: trixie's archive carries a runtime newer than `clang`
# resolves to, and clang looks only in its own resource directory, so that
# links against nothing.  Same query as devbox/Dockerfile.  Run this only
# after apt_setup clang, which installs the compiler and refreshes the lists.
clang_rt_package() {
    maj=$(clang -dumpversion | cut -d. -f1) || return 1
    apt-cache --names-only search '^libclang-rt-[0-9]+-dev$' \
        | awk -v m="libclang-rt-$maj-dev" '$1 == m {print $1}'
}
