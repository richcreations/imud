# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudUtils < Formula
  desc "Diagnostic tools for the imud IMU daemon"
  homepage "https://github.com/richcreations/imud"
  # url and sha256 are rewritten at every release by homebrew-publish.yml, on
  # the gate apt already uses: publishing the draft release.  There is no head
  # block by design — a tap serves releases, never a branch.
  url "https://github.com/richcreations/imud/releases/download/v1.11.0/imud-1.11.0.tar.gz"
  sha256 "3d4793f407b64dbb4574dd67e8552f9ced4acdc017dfcf09a70957ca8b94cb10"
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
