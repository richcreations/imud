# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudMavlink < Formula
  desc "MAVLink bridge for the imud IMU daemon"
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
    # One bridge, not `make bridges`: each is its own package here, and
    # building all five would drag every bridge's dependencies into each.
    system "make", "imud-mavlink"
    system "make", "install-mavlink", "DESTDIR=#{buildpath}/stage"

    staged = buildpath/"stage#{HOMEBREW_PREFIX}"

    # The bridge's own config, kept across upgrades once edited.
    (etc/"imud").mkpath
    Dir["#{staged}/etc/imud/*"].each do |src|
      (etc/"imud").install src unless (etc/"imud"/File.basename(src)).exist?
    end
    rm_r staged/"etc" if (staged/"etc").exist?

    prefix.install Dir["#{staged}/*"]
  end

  service do
    run [opt_bin/"imud-mavlink", "--config", etc/"imud/imud-mavlink.conf"]
    keep_alive true
    log_path var/"log/imud-mavlink.log"
    error_log_path var/"log/imud-mavlink.log"
  end

  test do
    assert_match "imud-mavlink", shell_output("#{bin}/imud-mavlink --version")
    assert_match "--config", shell_output("#{bin}/imud-mavlink --help")
  end
end
