# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudUtils < Formula
  desc "Diagnostic tools for the imud IMU daemon"
  homepage "https://github.com/richcreations/imud"
  license "MIT"

  # HEAD-only until 1.11.0: every flag the install block passes to configure
  # was added after 1.10.1, so the newest release tarball cannot build it.
  # release.yml writes the stable url and sha256 at that tag — imud issue #80.
  head "https://github.com/richcreations/imud.git", branch: "main"

  depends_on "richcreations/imud/imud"

  def install
    system "./configure", "--prefix=#{HOMEBREW_PREFIX}",
                          "--etcdir=#{etc}/imud",
                          "--rundir=#{var}/run",
                          "--statedir=#{var}/imud",
                          "--with-service=none"
    system "make", "imud-mon", "imud-imutest"
    system "make", "install-utils", "DESTDIR=#{buildpath}/stage"

    prefix.install Dir["#{buildpath}/stage#{HOMEBREW_PREFIX}/*"]
  end

  test do
    assert_match "imud-mon", shell_output("#{bin}/imud-mon --version")
    assert_match "imud-imutest", shell_output("#{bin}/imud-imutest --version")
  end
end
