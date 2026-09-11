# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
class ImudSignalk < Formula
  desc "Signal K bridge for the imud IMU daemon"
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
    # One bridge, not `make bridges`: each is its own package here, and
    # building all five would drag every bridge's dependencies into each.
    system "make", "imud-signalk"
    system "make", "install-signalk", "DESTDIR=#{buildpath}/stage"

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
    run [opt_bin/"imud-signalk", "--config", etc/"imud/imud-signalk.conf"]
    keep_alive true
    log_path var/"log/imud-signalk.log"
    error_log_path var/"log/imud-signalk.log"
  end

  test do
    assert_match "imud-signalk", shell_output("#{bin}/imud-signalk --version")
    assert_match "--config", shell_output("#{bin}/imud-signalk --help")
  end
end
