# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudUtils < Formula
  desc "Diagnostic tools for the imud IMU daemon"
  homepage "https://github.com/richcreations/imud"
  # url and sha256 are rewritten at every release by homebrew-publish.yml, on
  # the gate apt already uses: publishing the draft release.  There is no head
  # block by design — a tap serves releases, never a branch.
  url "https://github.com/richcreations/imud/releases/download/v1.11.3/imud-1.11.3.tar.gz"
  sha256 "9c1cb8dd93fb49a82fd926b0ae1e7cf679f212bc74ebd951f6b9b7106593f8a8"
  license "MIT"

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
