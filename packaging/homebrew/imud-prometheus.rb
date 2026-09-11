# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudPrometheus < Formula
  desc "Prometheus exporter for the imud IMU daemon"
  homepage "https://github.com/richcreations/imud"
  # url and sha256 are rewritten at every release by homebrew-publish.yml, on
  # the gate apt already uses: publishing the draft release.  There is no head
  # block by design — a tap serves releases, never a branch.
  url "https://github.com/richcreations/imud/releases/download/v1.10.1/imud-1.10.1.tar.gz"
  sha256 "d402928b1501637063cc8a7014dfe8e4758dd8e4dd697fea06b7bf6236f912d0"
  license "MIT"

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
