# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
# The tap's copy of this file is what `brew install` reads; this one is the
# source it is rendered from, so the configure flags stay beside the configure
# script that has to accept them.  test_configure.c reads them back out.
class Imud < Formula
  desc "IMU daemon publishing attitude, heading and heave over NMEA and UDP"
  homepage "https://github.com/richcreations/imud"
  license "MIT"

  # HEAD-only until 1.11.0: every flag the install block passes to configure
  # was added after 1.10.1, so the newest release tarball cannot build it.
  # release.yml writes the stable url and sha256 at that tag — imud issue #80.
  head "https://github.com/richcreations/imud.git", branch: "main"

  def install
    # imud compiles its paths in (include/paths.h), so a keg-relative prefix
    # would send the running daemon looking inside its own Cellar directory.
    # configure names the FINAL prefix and the install is staged into the keg
    # instead, which is what makes every compiled-in path resolve through
    # Homebrew's symlink farm — including WMM.COF, which arrives from the
    # separate imud-wmm-data keg.
    system "./configure", "--prefix=#{HOMEBREW_PREFIX}",
                          "--etcdir=#{etc}/imud",
                          "--rundir=#{var}/run",
                          "--statedir=#{var}/imud",
                          "--with-service=none"
    system "make"
    system "make", "install", "DESTDIR=#{buildpath}/stage"

    staged = buildpath/"stage#{HOMEBREW_PREFIX}"

    # Config is Homebrew's to preserve across upgrades, so it lands in etc
    # directly and an edited file is never replaced.
    (etc/"imud").mkpath
    Dir["#{staged}/etc/imud/*"].each do |src|
      (etc/"imud").install src unless (etc/"imud"/File.basename(src)).exist?
    end
    rm_r staged/"etc" if (staged/"etc").exist?

    prefix.install Dir["#{staged}/*"]

    (var/"run").mkpath
    (var/"imud").mkpath
  end

  service do
    run [opt_bin/"imud", "--config", etc/"imud/imud.conf"]
    keep_alive true
    log_path var/"log/imud.log"
    error_log_path var/"log/imud.log"
  end

  def caveats
    <<~EOS
      True heading needs the World Magnetic Model:
        brew install richcreations/imud/imud-wmm-data

      Review #{etc}/imud/imud.conf before starting — i2c_bus, gpio_chip and
      rotation_euler_deg.  macOS has no I2C or SPI on a header, so the sensor
      comes in over an FT232H USB bridge: set i2c_bus = "ftdi:" or
      spi_dev = "ftdi:/cs0" and int_gpio = 0 for both sensors.
    EOS
  end

  test do
    assert_match "imud", shell_output("#{bin}/imud --version")
    assert_match "imud-status", shell_output("#{bin}/imud-status --version")

    # The sim driver runs the whole pipeline with no hardware, so this asserts
    # the daemon fuses and shuts down rather than merely linking.
    (testpath/"imud.conf").write <<~CONF
      [device]
      i2c_bus = "/dev/null"
      [runtime]
      pid_file      = "#{testpath}/imud.pid"
      status_socket = "#{testpath}/imud.sock"
      [stream]
      socket = "#{testpath}/stream.sock"
      [imu]
      driver   = "sim"
      int_gpio = 0
      odr_hz   = 833
      [mag]
      driver   = "sim"
      int_gpio = 0
      odr_hz   = 100
    CONF

    # A redirect target has to be a String, not the Pathname testpath hands
    # back: spawn raises ArgumentError, "wrong exec redirect action", on one.
    log = testpath/"imud.log"
    pid = spawn bin/"imud", "--config", testpath/"imud.conf",
                [:out, :err] => [log.to_s, "w"]
    sleep 10
    Process.kill "TERM", pid
    Process.wait pid
    assert_match "running", log.read
  end
end
