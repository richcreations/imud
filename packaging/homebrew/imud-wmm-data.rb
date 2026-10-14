# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudWmmData < Formula
  desc "World Magnetic Model coefficient data for imud"
  homepage "https://github.com/richcreations/imud"
  # url and sha256 are rewritten at every release by homebrew-publish.yml, on
  # the gate apt already uses: publishing the draft release.  There is no head
  # block by design — a tap serves releases, never a branch.
  url "https://github.com/richcreations/imud/releases/download/v1.11.2/imud-1.11.2.tar.gz"
  sha256 "0b39f7f41f9d49b3c03936671e32c2ae47a809874aca1d922ef1d32a703ddd5d"
  license "MIT"

  def install
    # Data only — no compile.  The prefix still has to be HOMEBREW_PREFIX so
    # the file lands where imud's compiled-in DATADIR looks for it.
    system "./configure", "--prefix=#{HOMEBREW_PREFIX}",
                          "--etcdir=#{etc}/imud",
                          "--with-service=none"
    system "make", "install-wmm-data", "DESTDIR=#{buildpath}/stage"

    prefix.install Dir["#{buildpath}/stage#{HOMEBREW_PREFIX}/*"]
  end

  def caveats
    <<~EOS
      The model is valid to 2030.0.  An installation can drop a newer file at
      #{etc}/imud/WMM.COF, which imud prefers over this one.
    EOS
  end

  test do
    assert_predicate share/"imud/WMM.COF", :exist?
    assert_match "2025.0", (share/"imud/WMM.COF").read
  end
end
