# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudPrometheus < Formula
  desc "Prometheus exporter for the imud IMU daemon"
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
    system "make", "imud-prometheus"
    system "make", "install-prometheus", "DESTDIR=#{buildpath}/stage"

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
    run [opt_bin/"imud-prometheus", "--config", etc/"imud/imud-prometheus.conf"]
    keep_alive true
    log_path var/"log/imud-prometheus.log"
    error_log_path var/"log/imud-prometheus.log"
  end

  test do
    assert_match "imud-prometheus", shell_output("#{bin}/imud-prometheus --version")
    assert_match "--config", shell_output("#{bin}/imud-prometheus --help")
  end
end
