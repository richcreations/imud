# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudWmmData < Formula
  desc "World Magnetic Model coefficient data for imud"
  homepage "https://github.com/richcreations/imud"
  license "MIT"

  # HEAD-only until 1.11.0: every flag the install block passes to configure
  # was added after 1.10.1, so the newest release tarball cannot build it.
  # release.yml writes the stable url and sha256 at that tag — imud issue #80.
  head "https://github.com/richcreations/imud.git", branch: "main"

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
