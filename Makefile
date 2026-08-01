# Toolchain and flags. Externally-set CC/CFLAGS/CPPFLAGS/LDFLAGS are respected
# (e.g. dpkg-buildflags hardening injection); the `override +=` lines below add
# the flags the build cannot work without, even past a command-line override.
ifeq ($(origin CC),default)
CC = gcc
endif
CFLAGS   ?= -O2 -Wall -Wextra
override CFLAGS   += -std=c11 -pthread -Iinclude
override CPPFLAGS += -D_GNU_SOURCE
# imud_client.h is deprecated for third parties but is imud's own wire-pinning
# header (bridges, libimud, tests); silence its deprecation notice in-tree.
override CPPFLAGS += -DIMUD_CLIENT_ALLOW_DEPRECATED
# LDFLAGS carries linker flags only (relro/PIE/...); libraries are per-target.

# Canonical release version — single source is include/version.h.
VERSION := $(shell sed -n 's/^\#define IMUD_VERSION_STR *"\(.*\)"/\1/p' include/version.h)

# Auto-detect libgpiod major version; default to v1 (Bookworm ships 1.x).
# Pass -DGPIOD_V2 when pkg-config reports version 2.x or newer.
GPIOD_MAJ := $(shell pkg-config --modversion libgpiod 2>/dev/null | cut -d. -f1)
ifeq ($(GPIOD_MAJ),2)
    override CPPFLAGS += -DGPIOD_V2
endif

# ── libimud — the public client shared library ───────────────────────────────
# Linux builds the versioned .so (SONAME libimud.so.0) and the bridges link it
# (they are its first consumers). Darwin has no .so here: bridges link the
# object directly so the macOS dev/test workflow keeps working. In-tree bridge
# runs on Linux need LD_LIBRARY_PATH=. (the installed copy is found via
# ldconfig).
UNAME_S := $(shell uname -s)
SONAME   = libimud.so.0
SHLIB    = libimud.so.0.0
ifeq ($(UNAME_S),Linux)
    LIBIMUD = $(SHLIB)
else
    LIBIMUD = lib/libimud.o
endif

# ── Source lists (no main() in any of these) ─────────────────────────────────

DRIVER_SRCS = src/drivers/ism330dhcx.c \
              src/drivers/mmc5983ma.c \
              src/drivers/icm20948.c \
              src/drivers/ak09916.c \
              src/drivers/sim.c \
              src/drivers/lsm6dso.c \
              src/drivers/icm42688p.c \
              src/drivers/lis3mdl.c \
              src/drivers/lis2mdl.c \
              src/drivers/mpu925x.c \
              src/drivers/ak8963.c

# Full daemon: every module
IMUD_SRCS   = src/cal.c \
              src/capture.c \
              src/log.c \
              src/config.c \
              src/ring.c \
              src/fusion.c \
              src/imu.c \
              src/imu_math.c \
              src/nmea.c \
              src/netserv.c \
              src/output.c \
              src/packet.c \
              src/position.c \
              src/sdnotify.c \
              src/wmm.c \
              src/drivers.c \
              $(DRIVER_SRCS)

# Calibration tool: hardware access + config; no threads or output
CAL_SRCS    = src/cal.c \
              src/capture.c \
              src/log.c \
              src/cal_math.c \
              src/config.c \
              src/drivers.c \
              src/fusion.c \
              src/imu_math.c \
              src/fit_ra.c \
              $(DRIVER_SRCS)

# Driver-validation tool: hardware access, config, the driver registry, and the
# swing-coverage helper shared with imud-cal.  src/capture.c is here only
# because the sim driver's .imucap playback needs it.
IMUTEST_SRCS = src/config.c \
               src/log.c \
               src/capture.c \
               src/cal_math.c \
               src/imu_math.c \
               src/drivers.c \
               src/imutest.c \
               src/imutest_open.c \
               src/imutest_gpio.c \
               src/imutest_report.c \
               $(DRIVER_SRCS)

IMUD_OBJS    = $(IMUD_SRCS:.c=.o)
CAL_OBJS     = $(CAL_SRCS:.c=.o)
IMUTEST_OBJS = $(IMUTEST_SRCS:.c=.o)

.PHONY: all bridges libimud clean test check coverage dist install install-utils install-wmm-data install-signalk install-mqtt install-influxdb install-mavlink install-prometheus uninstall .FORCE

all: imud imud-cal imud-imutest imud-status imud-mon

# ── Binaries ──────────────────────────────────────────────────────────────────

imud: $(IMUD_OBJS) src/main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lgpiod -lm

# imud-cal requires src/cal_main.c
imud-cal: $(CAL_OBJS) src/cal_main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lgpiod -lm

# imud-imutest exercises any registered driver against real silicon and writes
# a Markdown validation report (ROADMAP §1).  Ships in imud-utils alongside
# imud-mon, but unlike imud-mon it must run on the box with the sensor.
# -lgpiod (the interrupt edge-count check) makes it Linux-only, like imud.
imud-imutest: $(IMUTEST_OBJS) src/imutest_main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lgpiod -lm

# imud-status is a plain socket client: no hardware libs
imud-status: src/status_main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

# imud-mon is a plain UDP consumer: needs config parsing and math
imud-mon: src/config.o src/log.o src/mon_main.o
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# imud-signalk bridges the AF_UNIX stream to Signal K delta JSON over UDP.
# Stream access + validation come from libimud ($(LIBIMUD) in $^ is either the
# versioned .so — linked directly, embedding its SONAME — or, on Darwin, the
# plain object).
imud-signalk: src/sk_delta.o src/config.o src/log.o src/netserv.o src/bridge.o src/sdnotify.o src/signalk_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# imud-mqtt bridges the AF_UNIX stream to MQTT: scalar telemetry topics plus
# Home Assistant discovery, via libmosquitto.  Needs libmosquitto-dev.
imud-mqtt: src/mqtt_publish.o src/config.o src/log.o src/bridge.o src/sdnotify.o src/mqtt_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lmosquitto -lm

# imud-influxdb bridges the AF_UNIX stream to InfluxDB line protocol over UDP or
# HTTP.  Pure C — no external dependencies beyond libimud.
imud-influxdb: src/influx_line.o src/config.o src/log.o src/bridge.o src/sdnotify.o src/influx_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# imud-mavlink bridges the AF_UNIX stream to MAVLink (v1/v2) over UDP and/or
# serial.  Pure C — hand-rolled encoder, no external dependencies beyond libimud.
imud-mavlink: src/mavlink_encode.o src/config.o src/log.o src/netserv.o src/bridge.o src/sdnotify.o src/mavlink_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Optional bridge daemons — each has its own config file, service, and man page,
# and installs via its own `install-*` target (prep for per-bridge packaging).
# Kept out of `all` so a core build / CI never needs a bridge's dependencies.
# imud-prometheus serves the fused state as Prometheus /metrics gauges. Pure
# C — no external dependencies beyond libimud; the first bridge built purely
# on the ABI-stable imud_data_t (no wire pinning).
imud-prometheus: src/prom_metrics.o src/config.o src/log.o src/bridge.o src/sdnotify.o src/prom_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

bridges: imud-signalk imud-mqtt imud-influxdb imud-mavlink imud-prometheus

# ── libimud shared library ────────────────────────────────────────────────────

# PIC object (also linked directly into the bridges on Darwin).
lib/libimud.o: lib/libimud.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -fPIC -c -o $@ $<

# Versioned shared library (Linux): SONAME libimud.so.0; only imud_* exported.
$(SHLIB): lib/libimud.o lib/libimud.map
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -Wl,-soname,$(SONAME) \
	      -Wl,--version-script=lib/libimud.map -o $@ lib/libimud.o
	ln -sf $(SHLIB) $(SONAME)
	ln -sf $(SONAME) libimud.so

libimud: $(LIBIMUD)

# ── Compilation rules ─────────────────────────────────────────────────────────

# -MMD -MP writes a .d makefile fragment per object so header edits rebuild
# their dependents; the -include pulls in whichever fragments exist so far.
DEPFLAGS = -MMD -MP

src/%.o: src/%.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

src/drivers/%.o: src/drivers/%.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

-include $(wildcard src/*.d src/drivers/*.d lib/*.d)

# ── Tests ─────────────────────────────────────────────────────────────────────

test_fusion: src/fusion.c test/test_fusion.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_fit_ra: src/fit_ra.c src/fusion.c src/imu_math.c src/capture.c test/test_fit_ra.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_config: src/config.c src/log.c test/test_config.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_nmea: src/nmea.c test/test_nmea.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_capture: src/capture.c src/drivers/sim.c src/fusion.c src/log.c test/test_capture.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_packet: src/packet.c test/test_packet.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_concurrency: $(IMUD_OBJS) test/test_concurrency.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lgpiod -lm

test_ring: src/ring.c test/test_ring.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_mount: src/config.c src/log.c test/test_mount.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_cal: src/cal.c src/log.c test/test_cal.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_cal_math: src/cal_math.c test/test_cal_math.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_wmm: src/wmm.c src/log.c test/test_wmm.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_position: src/position.c src/wmm.c src/log.c test/test_position.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Wire-format compatibility: daemon packet_build vs lib/imud_client.h.
# test_client_impl.c compiles the client header in its own translation unit,
# exactly as a third-party consumer would.
test_client: src/packet.c test/test_client.c test/test_client_impl.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# End-to-end AF_UNIX + TCP subscription stream: real output.c, stubbed imu accessors
test_stream: src/output.c src/nmea.c src/netserv.c src/packet.c src/config.c src/log.c test/test_stream.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# netserv TCP broadcast server (pure sockets; macOS-buildable)
test_netserv: src/netserv.c src/log.c test/test_netserv.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_log: src/log.c test/test_log.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Signal K delta encoder (pure function; reuses lib/imud_client.h for the struct)
test_signalk: src/sk_delta.c test/test_signalk.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# MQTT message builders (pure functions; no libmosquitto needed)
test_mqtt: src/mqtt_publish.c test/test_mqtt.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Prometheus metrics encoder (pure function)
test_prometheus: src/prom_metrics.c test/test_prometheus.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

test_influxdb: src/influx_line.c test/test_influxdb.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# MAVLink encoder (pure function; golden frames from a pymavlink cross-check)
test_mavlink: src/mavlink_encode.c test/test_mavlink.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# libimud public API: end-to-end over a local AF_UNIX server + UDP loopback,
# packets built by the daemon's real encoder (src/packet.c).
test_libimud: lib/libimud.c src/packet.c test/test_libimud.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Bridge scaffolding (src/bridge.c + src/sdnotify.c): CLI matrix, emit-tick
# timespec math (period/wait/due/advance/earlier), config load / reload /
# disabled flows, and sd_notify delivery over a test-bound NOTIFY_SOCKET.
# Links libimud directly, like test_libimud.
test_bridge: src/bridge.c src/sdnotify.c src/config.c src/log.c lib/libimud.c test/test_bridge.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Driver registry + ops-table sanity for every registered chip (no hardware).
# Links every driver: sim.c pulls in capture.c for its .imucap playback path,
# and the drivers reference log_emit (LOG_E) on their error branches.
# Linux-only (the drivers include <linux/i2c.h>), like the rest of `test`.
test_drivers_registry: src/drivers.c $(DRIVER_SRCS) src/capture.c src/log.c test/test_drivers_registry.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# imu.c pure math: ODR rounding, timestamp reconstruction, mount rotation,
# calibration application — the helpers factored into src/imu_math.c.  Pure
# (no <linux/*> or CLOCK_MONOTONIC), so it also builds on the macOS dev box.
test_imu_math: src/imu_math.c test/test_imu_math.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Per-driver register decode/encode over a mock I2C bus (test/i2c_mock.c wraps
# ioctl with --wrap — GNU ld only).  Covers the two hardware-validated drivers
# (ism330dhcx IMU, mmc5983ma mag) plus the MPU-925x pair, whose fuse-ROM
# correction and rotated magnetometer axes are worth pinning down off-hardware.
# Also wrap __ioctl_time64: on 32-bit glibc with -D_TIME_BITS=64 (Debian armhf)
# ioctl() is redirected to that symbol, so wrapping plain ioctl alone misses
# every call there.
test_drivers: src/drivers/ism330dhcx.c src/drivers/mmc5983ma.c \
              src/drivers/mpu925x.c src/drivers/ak8963.c src/log.c \
              test/i2c_mock.c test/test_drivers.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) \
	    -Wl,--wrap=ioctl -Wl,--wrap=__ioctl_time64 -o $@ $^ -lm

# The imud-imutest checker logic over the mock I2C bus, with a scripted
# imt_ui_t standing in for the operator so the guided phases are covered too.
# Links the driver ops structs directly (not src/drivers.c) and stubs
# imt_gpio_count_edges, so it needs neither the registry nor -lgpiod.
# Same --wrap pair as test_drivers; Linux/GNU-ld only.
# -Isrc is for test/test_imutest.c alone: it checks the mock's FIFO-port window
# through drivers/i2c_io.h, the same single-byte read path imud-imutest's
# register sweep uses, and a quoted include only searches the including file's
# own directory — which works from src/imutest.c and not from test/. src/ holds
# exactly one header (drivers/i2c_io.h) and none of include/'s names, so this
# widens the search path without shadowing anything.
test_imutest: src/imutest.c src/imutest_report.c \
              src/drivers/ism330dhcx.c src/drivers/mmc5983ma.c \
              src/config.c src/log.c src/imu_math.c src/cal_math.c \
              test/i2c_mock.c test/test_imutest.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -Isrc $(LDFLAGS) \
	    -Wl,--wrap=ioctl -Wl,--wrap=__ioctl_time64 -o $@ $^ -lm

test: test_fusion test_fit_ra test_config test_nmea test_packet test_capture test_ring \
      test_concurrency \
      test_mount test_cal test_cal_math test_wmm test_position test_client \
      test_stream test_netserv test_log test_signalk test_mqtt test_influxdb \
      test_mavlink test_libimud test_bridge test_prometheus \
      test_drivers_registry test_imu_math test_drivers test_imutest
	./test_fusion
	./test_fit_ra
	./test_config
	./test_nmea
	./test_packet
	./test_capture
	./test_ring
	./test_concurrency
	./test_mount
	./test_cal
	./test_cal_math
	./test_wmm
	./test_position
	./test_client
	./test_stream
	./test_netserv
	./test_log
	./test_signalk
	./test_mqtt
	./test_influxdb
	./test_prometheus
	./test_mavlink
	./test_libimud
	./test_bridge
	./test_drivers_registry
	./test_imu_math
	./test_drivers
	./test_imutest

# ── Release tarball ───────────────────────────────────────────────────────────
# The upstream release artifact (later renamed imud_$(VERSION).orig.tar.gz for
# Debian packaging). Contents come from git HEAD; .gitattributes export-ignore
# keeps repo-only files (.github, .git*) out.

dist:
	git archive --format=tar.gz --prefix=imud-$(VERSION)/ \
	    -o imud-$(VERSION).tar.gz HEAD
	@echo "Wrote imud-$(VERSION).tar.gz"

# ── Version bump ──────────────────────────────────────────────────────────────
# Propagate a new release version across include/version.h, the man page .TH
# lines, the published pages and spec.md, then report which changelogs still
# need a hand-written stanza. See docs/RELEASING.md.
#   make bump-version VERSION=1.9.0 [DATE=2026-08-01]
# VERSION already holds the CURRENT version (parsed from version.h above), so
# require it to come from the command line — otherwise a bare `make
# bump-version` would silently "bump" to the version already in the tree.
.PHONY: bump-version
bump-version:
	@test "$(origin VERSION)" = "command line" || { \
	    echo "usage: make bump-version VERSION=X.Y.Z [DATE=YYYY-MM-DD]" >&2; \
	    echo "       (tree is currently at $(VERSION))" >&2; exit 1; }
	@tools/bump-version.sh "$(VERSION)" "$(DATE)"

# Regenerate the packet fuzz seed for the CURRENT wire revision. The seed is
# committed (CI's fuzz smoke run reads it), so this is only run when the wire
# format changes — test_packet fails with a pointer here when it must be.
.PHONY: fuzz-seeds
fuzz-seeds:
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o mkseed_packet \
	    src/packet.c fuzz/mkseed_packet.c -lm
	@rm -f test/fuzz/corpus/packet/valid_v*.bin
	./mkseed_packet test/fuzz/corpus/packet/valid_v$(shell sed -n 's/^\#define IMUD_VERSION *\([0-9]*\).*/\1/p' include/types.h).bin
	@rm -f mkseed_packet
	@echo "Regenerated test/fuzz/corpus/packet/ — commit the new seed."

# GNU-convention alias
check: test

# ── Coverage ──────────────────────────────────────────────────────────────────
# Rebuild and run the whole host test suite instrumented with gcov, then
# summarise with lcov.  A measuring tool for finding residual line/branch gaps
# in the already-tested modules — not part of `test` or CI's required jobs.
# Linux only (like `test`: the daemon-linking tests need <linux/*> + gpiod).
#
# Caveat: the per-test targets compile each source directly, so a module built
# into several tests (log.c, config.c, packet.c) gets a fresh .gcno per build
# and lcov skips the stamp-mismatched runs — hence --ignore-errors.  Every
# single-consumer module (all drivers, fusion, imu_math, position, output, the
# bridge encoders, …) reports accurately, which is where the gaps were.
coverage:
	$(MAKE) clean
	$(MAKE) test CFLAGS="-O0 -g --coverage" LDFLAGS="--coverage"
	@if command -v lcov >/dev/null 2>&1; then \
	    lcov --capture --directory . --output-file coverage.info \
	         --ignore-errors mismatch,source,graph,empty,unused \
	         --exclude '*/test/*' --exclude '*/fuzz/*' --exclude '/usr/*'; \
	    lcov --list coverage.info; \
	    if command -v genhtml >/dev/null 2>&1; then \
	        genhtml coverage.info --output-directory coverage-html \
	            --ignore-errors source >/dev/null 2>&1 && \
	        echo "HTML report: coverage-html/index.html"; \
	    fi; \
	else \
	    echo "lcov not installed; raw gcov summary for src/:"; \
	    gcov -n -o . src/*.c src/drivers/*.c 2>/dev/null | \
	        grep -E "File '(src|lib)/|Lines executed" || true; \
	    echo "(install lcov for an aggregated, HTML report)"; \
	fi

# ── Install ───────────────────────────────────────────────────────────────────
# Packagers: pass PREFIX=/usr SVCDIR=/usr/lib/systemd/system DESTDIR=<stage>.
# When DESTDIR is set the install is a pure file copy: no useradd, no
# systemctl — those belong to the package's maintainer scripts.

PREFIX  ?= /usr/local
ETCDIR  ?= /etc/imud
SVCDIR  ?= /etc/systemd/system
LIBDIR  ?= $(PREFIX)/lib
MANDIR  ?= $(PREFIX)/share/man
DOCDIR  ?= $(PREFIX)/share/doc

# pkg-config metadata, generated with the configured paths/version.
libimud.pc: lib/libimud.pc.in .FORCE
	sed -e 's|@PREFIX@|$(PREFIX)|g' -e 's|@LIBDIR@|$(LIBDIR)|g' \
	    -e 's|@VERSION@|$(VERSION)|g' $< > $@

# systemd units are generated from etc/*.service.in with the real bin dir.
# .FORCE regenerates on every make so a changed PREFIX can't leave stale units.
etc/%.service: etc/%.service.in .FORCE
	sed 's|@BINDIR@|$(PREFIX)/bin|g' $< > $@

.FORCE:

install: imud imud-cal imud-status etc/imud.service $(SHLIB) libimud.pc
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(SVCDIR)
	install -m 755 imud imud-cal imud-status $(DESTDIR)$(PREFIX)/bin/
	# ── System user (skipped for staged/packaged installs: DESTDIR set) ────
	@if [ -z "$(DESTDIR)" ] && ! id -u imud >/dev/null 2>&1; then \
	    useradd --system --no-create-home --shell /usr/sbin/nologin imud; \
	    usermod -aG gpio imud 2>/dev/null || true; \
	    usermod -aG i2c  imud 2>/dev/null || true; \
	    echo "Created system user 'imud'"; \
	fi
	# ── Config + calibration (all in /etc/imud) ────────────────────────────
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud.conf" ]; then \
	    install -m 644 config/imud.conf $(DESTDIR)$(ETCDIR)/imud.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud.conf"; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud.conf"; \
	fi
	@if [ -f "config/cal.json" ]; then \
	    install -m 644 config/cal.json $(DESTDIR)$(ETCDIR)/cal.json; \
	    echo "Installed calibration:  $(DESTDIR)$(ETCDIR)/cal.json"; \
	else \
	    echo "No config/cal.json found — run 'imud-cal' after install to calibrate."; \
	fi
	# ── Systemd service ────────────────────────────────────────────────────
	install -m 644 etc/imud.service         $(DESTDIR)$(SVCDIR)/imud.service
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	# ── Client libraries ───────────────────────────────────────────────────
	# imud_client.h is DEPRECATED and no longer installed (vendor from the
	# source tree if you must); the C client is libimud below.
	install -d -m 0755 $(DESTDIR)$(PREFIX)/include $(DESTDIR)$(PREFIX)/share/imud
	install -m 644 lib/imud_client.py $(DESTDIR)$(PREFIX)/share/imud/imud_client.py
	@echo "Installed client libs:  $(DESTDIR)$(PREFIX)/share/imud/imud_client.py"
	# ── libimud shared library + public header + pkg-config ────────────────
	install -d -m 0755 $(DESTDIR)$(LIBDIR)/pkgconfig
	install -m 644 $(SHLIB) $(DESTDIR)$(LIBDIR)/$(SHLIB)
	ln -sf $(SHLIB) $(DESTDIR)$(LIBDIR)/$(SONAME)
	ln -sf $(SONAME) $(DESTDIR)$(LIBDIR)/libimud.so
	install -m 644 lib/imud.h $(DESTDIR)$(PREFIX)/include/imud.h
	install -m 644 libimud.pc $(DESTDIR)$(LIBDIR)/pkgconfig/libimud.pc
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man3
	gzip -9nc man/man3/libimud.3 > $(DESTDIR)$(MANDIR)/man3/libimud.3.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/libimud
	install -m 644 docs/libimud/README.md docs/libimud/manual.md \
	               docs/libimud/spec.md $(DESTDIR)$(DOCDIR)/libimud/
	install -m 644 packaging/libimud/copyright $(DESTDIR)$(DOCDIR)/libimud/copyright
	gzip -9nc packaging/libimud/changelog > $(DESTDIR)$(DOCDIR)/libimud/changelog.gz
	@if [ -z "$(DESTDIR)" ] && command -v ldconfig >/dev/null 2>&1; then \
	    ldconfig; \
	fi
	@echo "Installed libimud:      $(DESTDIR)$(LIBDIR)/$(SHLIB) (+ imud.h, libimud.pc, libimud.3)"
	# ── Man pages ──────────────────────────────────────────────────────────
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man1 \
	                   $(DESTDIR)$(MANDIR)/man5 \
	                   $(DESTDIR)$(MANDIR)/man8
	gzip -9nc man/man1/imud-status.1 > $(DESTDIR)$(MANDIR)/man1/imud-status.1.gz
	gzip -9nc man/man5/imud.conf.5   > $(DESTDIR)$(MANDIR)/man5/imud.conf.5.gz
	gzip -9nc man/man8/imud.8        > $(DESTDIR)$(MANDIR)/man8/imud.8.gz
	gzip -9nc man/man8/imud-cal.8    > $(DESTDIR)$(MANDIR)/man8/imud-cal.8.gz
	@echo "Installed man pages to $(DESTDIR)$(MANDIR)"
	# ── Documentation (/usr/share/doc/imud) ────────────────────────────────
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud/examples
	# INSTALL is build-from-source guidance: it ships in the tarball, not in
	# the installed docs (Debian: package-contains-upstream-installation-documentation).
	install -m 644 AUTHORS NEWS README.md CONTRIBUTING.md spec.md \
	               docs/manual.md docs/ROADMAP.md $(DESTDIR)$(DOCDIR)/imud/
	install -m 644 packaging/imud/copyright $(DESTDIR)$(DOCDIR)/imud/copyright
	gzip -9nc packaging/imud/changelog > $(DESTDIR)$(DOCDIR)/imud/changelog.gz
	install -m 644 config/imud.conf $(DESTDIR)$(DOCDIR)/imud/examples/imud.conf
	@echo "Installed docs to       $(DESTDIR)$(DOCDIR)/imud"
	@echo ""
	@echo "Next steps:"
	@echo "  sudo systemctl enable --now imud"
	@echo "  review $(ETCDIR)/imud.conf  (i2c_bus, gpio_chip, rotation_euler_deg)"

# ── Install the Signal K bridge (optional) ─────────────────────────────────────
# Run after `make bridges`.  Installs the binary, service, man page, and its own
# config file (non-clobbering).  Prep for a standalone imud-signalk package.
# Diagnostic tools kept out of the core package — the imud-utils package.
# imud-mon listens to the UDP broadcast and can run from any machine on the
# network; imud-imutest talks to the sensor over I2C and must run on the
# daemon's own box.  Both are operator tools rather than part of the running
# system, which is why they share a package.
install-utils: imud-mon imud-imutest
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(MANDIR)/man1 \
	                   $(DESTDIR)$(MANDIR)/man8
	install -m 755 imud-mon imud-imutest $(DESTDIR)$(PREFIX)/bin/
	gzip -9nc man/man1/imud-mon.1 > $(DESTDIR)$(MANDIR)/man1/imud-mon.1.gz
	gzip -9nc man/man8/imud-imutest.8 > $(DESTDIR)$(MANDIR)/man8/imud-imutest.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-utils
	install -m 644 docs/imud-utils/README.md docs/imud-utils/manual.md \
	               docs/imud-utils/spec.md $(DESTDIR)$(DOCDIR)/imud-utils/
	install -m 644 packaging/imud-utils/copyright $(DESTDIR)$(DOCDIR)/imud-utils/copyright
	gzip -9nc packaging/imud-utils/changelog > $(DESTDIR)$(DOCDIR)/imud-utils/changelog.gz
	@echo "Installed imud-utils (imud-mon, imud-imutest)."

# WMM coefficient data — separate target so it can be packaged on its own
# (tzdata pattern: imud-wmm-data updates independently of the daemon).
# imud auto-resolves /etc/imud/WMM.COF (operator override) then this path.
install-wmm-data:
	install -d -m 0755 $(DESTDIR)$(PREFIX)/share/imud
	install -m 644 data/WMM.COF $(DESTDIR)$(PREFIX)/share/imud/WMM.COF
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-wmm-data
	install -m 644 packaging/imud-wmm-data/copyright $(DESTDIR)$(DOCDIR)/imud-wmm-data/copyright
	gzip -9nc packaging/imud-wmm-data/changelog > $(DESTDIR)$(DOCDIR)/imud-wmm-data/changelog.gz
	@echo "Installed WMM2025 coefficients: $(DESTDIR)$(PREFIX)/share/imud/WMM.COF"
	@echo "  (drop a newer model at $(ETCDIR)/WMM.COF to override; imud prefers it)"

install-signalk: imud-signalk etc/imud-signalk.service
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(SVCDIR)
	install -m 755 imud-signalk $(DESTDIR)$(PREFIX)/bin/
	install -m 644 etc/imud-signalk.service $(DESTDIR)$(SVCDIR)/imud-signalk.service
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-signalk.conf" ]; then \
	    install -m 644 config/imud-signalk.conf $(DESTDIR)$(ETCDIR)/imud-signalk.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-signalk.conf"; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-signalk.conf"; \
	fi
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man5 $(DESTDIR)$(MANDIR)/man8
	gzip -9nc man/man5/imud-signalk.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-signalk.conf.5.gz
	gzip -9nc man/man8/imud-signalk.8 > $(DESTDIR)$(MANDIR)/man8/imud-signalk.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-signalk/examples
	install -m 644 docs/imud-signalk/README.md docs/imud-signalk/manual.md \
	               docs/imud-signalk/spec.md $(DESTDIR)$(DOCDIR)/imud-signalk/
	install -m 644 packaging/imud-signalk/copyright $(DESTDIR)$(DOCDIR)/imud-signalk/copyright
	gzip -9nc packaging/imud-signalk/changelog > $(DESTDIR)$(DOCDIR)/imud-signalk/changelog.gz
	install -m 644 config/imud-signalk.conf $(DESTDIR)$(DOCDIR)/imud-signalk/examples/imud-signalk.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-signalk.  Enable with: sudo systemctl enable --now imud-signalk"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-signalk.conf)"

# ── Install the MQTT bridge (optional) ─────────────────────────────────────────
# Run after `make imud-mqtt` (needs libmosquitto-dev).  Installs the binary,
# service, man page, and its own config file (non-clobbering).
install-mqtt: imud-mqtt etc/imud-mqtt.service
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(SVCDIR)
	install -m 755 imud-mqtt $(DESTDIR)$(PREFIX)/bin/
	install -m 644 etc/imud-mqtt.service $(DESTDIR)$(SVCDIR)/imud-mqtt.service
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-mqtt.conf" ]; then \
	    install -m 644 config/imud-mqtt.conf $(DESTDIR)$(ETCDIR)/imud-mqtt.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-mqtt.conf"; \
	    echo "  NOTE: if you set a broker password, restrict it (chmod 640 + service group)."; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-mqtt.conf"; \
	fi
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man5 $(DESTDIR)$(MANDIR)/man8
	gzip -9nc man/man5/imud-mqtt.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-mqtt.conf.5.gz
	gzip -9nc man/man8/imud-mqtt.8 > $(DESTDIR)$(MANDIR)/man8/imud-mqtt.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-mqtt/examples
	install -m 644 docs/imud-mqtt/README.md docs/imud-mqtt/manual.md \
	               docs/imud-mqtt/spec.md $(DESTDIR)$(DOCDIR)/imud-mqtt/
	install -m 644 packaging/imud-mqtt/copyright $(DESTDIR)$(DOCDIR)/imud-mqtt/copyright
	gzip -9nc packaging/imud-mqtt/changelog > $(DESTDIR)$(DOCDIR)/imud-mqtt/changelog.gz
	install -m 644 config/imud-mqtt.conf $(DESTDIR)$(DOCDIR)/imud-mqtt/examples/imud-mqtt.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-mqtt.  Enable with: sudo systemctl enable --now imud-mqtt"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-mqtt.conf)"

# ── Install the InfluxDB bridge (optional) ─────────────────────────────────────
# Run after `make imud-influxdb`.  Installs the binary, service, man page, and
# its own config file (non-clobbering).
install-influxdb: imud-influxdb etc/imud-influxdb.service
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(SVCDIR)
	install -m 755 imud-influxdb $(DESTDIR)$(PREFIX)/bin/
	install -m 644 etc/imud-influxdb.service $(DESTDIR)$(SVCDIR)/imud-influxdb.service
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-influxdb.conf" ]; then \
	    install -m 644 config/imud-influxdb.conf $(DESTDIR)$(ETCDIR)/imud-influxdb.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-influxdb.conf"; \
	    echo "  NOTE: if you set an HTTP token, restrict it (chmod 640 + service group)."; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-influxdb.conf"; \
	fi
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man5 $(DESTDIR)$(MANDIR)/man8
	gzip -9nc man/man5/imud-influxdb.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-influxdb.conf.5.gz
	gzip -9nc man/man8/imud-influxdb.8 > $(DESTDIR)$(MANDIR)/man8/imud-influxdb.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-influxdb/examples
	install -m 644 docs/imud-influxdb/README.md docs/imud-influxdb/manual.md \
	               docs/imud-influxdb/spec.md $(DESTDIR)$(DOCDIR)/imud-influxdb/
	install -m 644 packaging/imud-influxdb/copyright $(DESTDIR)$(DOCDIR)/imud-influxdb/copyright
	gzip -9nc packaging/imud-influxdb/changelog > $(DESTDIR)$(DOCDIR)/imud-influxdb/changelog.gz
	install -m 644 config/imud-influxdb.conf $(DESTDIR)$(DOCDIR)/imud-influxdb/examples/imud-influxdb.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-influxdb.  Enable with: sudo systemctl enable --now imud-influxdb"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-influxdb.conf)"

# ── Install the Prometheus exporter (optional) ─────────────────────────────────
# Run after `make imud-prometheus`.  Installs the binary, service, man pages,
# and its own config file (non-clobbering).
install-prometheus: imud-prometheus etc/imud-prometheus.service
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(SVCDIR)
	install -m 755 imud-prometheus $(DESTDIR)$(PREFIX)/bin/
	install -m 644 etc/imud-prometheus.service $(DESTDIR)$(SVCDIR)/imud-prometheus.service
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-prometheus.conf" ]; then \
	    install -m 644 config/imud-prometheus.conf $(DESTDIR)$(ETCDIR)/imud-prometheus.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-prometheus.conf"; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-prometheus.conf"; \
	fi
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man5 $(DESTDIR)$(MANDIR)/man8
	gzip -9nc man/man5/imud-prometheus.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-prometheus.conf.5.gz
	gzip -9nc man/man8/imud-prometheus.8 > $(DESTDIR)$(MANDIR)/man8/imud-prometheus.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-prometheus/examples
	install -m 644 docs/imud-prometheus/README.md docs/imud-prometheus/manual.md \
	               docs/imud-prometheus/spec.md $(DESTDIR)$(DOCDIR)/imud-prometheus/
	install -m 644 packaging/imud-prometheus/copyright $(DESTDIR)$(DOCDIR)/imud-prometheus/copyright
	gzip -9nc packaging/imud-prometheus/changelog > $(DESTDIR)$(DOCDIR)/imud-prometheus/changelog.gz
	install -m 644 config/imud-prometheus.conf $(DESTDIR)$(DOCDIR)/imud-prometheus/examples/imud-prometheus.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-prometheus.  Enable with: sudo systemctl enable --now imud-prometheus"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-prometheus.conf)"

# ── Install the MAVLink bridge (optional) ──────────────────────────────────────
# Run after `make imud-mavlink`.  Installs the binary, service, man page, and its
# own config file (non-clobbering).
install-mavlink: imud-mavlink etc/imud-mavlink.service
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(SVCDIR)
	install -m 755 imud-mavlink $(DESTDIR)$(PREFIX)/bin/
	install -m 644 etc/imud-mavlink.service $(DESTDIR)$(SVCDIR)/imud-mavlink.service
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-mavlink.conf" ]; then \
	    install -m 644 config/imud-mavlink.conf $(DESTDIR)$(ETCDIR)/imud-mavlink.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-mavlink.conf"; \
	    echo "  NOTE: for serial output, grant the imud user serial access (e.g. dialout group)."; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-mavlink.conf"; \
	fi
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man5 $(DESTDIR)$(MANDIR)/man8
	gzip -9nc man/man5/imud-mavlink.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-mavlink.conf.5.gz
	gzip -9nc man/man8/imud-mavlink.8 > $(DESTDIR)$(MANDIR)/man8/imud-mavlink.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-mavlink/examples
	install -m 644 docs/imud-mavlink/README.md docs/imud-mavlink/manual.md \
	               docs/imud-mavlink/spec.md $(DESTDIR)$(DOCDIR)/imud-mavlink/
	install -m 644 packaging/imud-mavlink/copyright $(DESTDIR)$(DOCDIR)/imud-mavlink/copyright
	gzip -9nc packaging/imud-mavlink/changelog > $(DESTDIR)$(DOCDIR)/imud-mavlink/changelog.gz
	install -m 644 config/imud-mavlink.conf $(DESTDIR)$(DOCDIR)/imud-mavlink/examples/imud-mavlink.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-mavlink.  Enable with: sudo systemctl enable --now imud-mavlink"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-mavlink.conf)"

uninstall:
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl disable --now imud 2>/dev/null || true; \
	    systemctl disable --now imud-signalk 2>/dev/null || true; \
	    systemctl disable --now imud-mqtt 2>/dev/null || true; \
	    systemctl disable --now imud-influxdb 2>/dev/null || true; \
	    systemctl disable --now imud-prometheus 2>/dev/null || true; \
	    systemctl disable --now imud-mavlink 2>/dev/null || true; \
	fi
	rm -f $(DESTDIR)$(PREFIX)/bin/imud \
	      $(DESTDIR)$(PREFIX)/bin/imud-cal \
	      $(DESTDIR)$(PREFIX)/bin/imud-status \
	      $(DESTDIR)$(PREFIX)/bin/imud-mon \
	      $(DESTDIR)$(PREFIX)/bin/imud-imutest \
	      $(DESTDIR)$(PREFIX)/bin/imud-signalk \
	      $(DESTDIR)$(PREFIX)/bin/imud-mqtt \
	      $(DESTDIR)$(PREFIX)/bin/imud-influxdb \
	      $(DESTDIR)$(PREFIX)/bin/imud-prometheus \
	      $(DESTDIR)$(PREFIX)/bin/imud-mavlink \
	      $(DESTDIR)$(PREFIX)/include/imud_client.h \
	      $(DESTDIR)$(PREFIX)/include/imud.h \
	      $(DESTDIR)$(LIBDIR)/$(SHLIB) \
	      $(DESTDIR)$(LIBDIR)/$(SONAME) \
	      $(DESTDIR)$(LIBDIR)/libimud.so \
	      $(DESTDIR)$(LIBDIR)/pkgconfig/libimud.pc \
	      $(DESTDIR)$(PREFIX)/share/imud/imud_client.py \
	      $(DESTDIR)$(PREFIX)/share/imud/WMM.COF \
	      $(DESTDIR)$(SVCDIR)/imud.service \
	      $(DESTDIR)$(SVCDIR)/imud-signalk.service \
	      $(DESTDIR)$(SVCDIR)/imud-mqtt.service \
	      $(DESTDIR)$(SVCDIR)/imud-influxdb.service \
	      $(DESTDIR)$(SVCDIR)/imud-prometheus.service \
	      $(DESTDIR)$(SVCDIR)/imud-mavlink.service \
	      $(DESTDIR)$(MANDIR)/man3/libimud.3.gz \
	      $(DESTDIR)$(MANDIR)/man1/imud-status.1.gz \
	      $(DESTDIR)$(MANDIR)/man1/imud-mon.1.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-cal.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-imutest.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-signalk.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-mqtt.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-influxdb.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-prometheus.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-mavlink.8.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-signalk.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-mqtt.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-influxdb.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-prometheus.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-mavlink.conf.5.gz
	rm -rf $(DESTDIR)$(DOCDIR)/imud $(DESTDIR)$(DOCDIR)/imud-signalk \
	       $(DESTDIR)$(DOCDIR)/imud-mqtt $(DESTDIR)$(DOCDIR)/imud-influxdb \
	       $(DESTDIR)$(DOCDIR)/imud-mavlink $(DESTDIR)$(DOCDIR)/imud-prometheus \
	       $(DESTDIR)$(DOCDIR)/imud-wmm-data $(DESTDIR)$(DOCDIR)/imud-utils \
	       $(DESTDIR)$(DOCDIR)/libimud
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Config and calibration in $(ETCDIR) were NOT removed — delete manually if no longer needed."

# ── Clean ─────────────────────────────────────────────────────────────────────

clean:
	rm -f src/*.o src/drivers/*.o src/*.d src/drivers/*.d lib/*.o lib/*.d \
	      imud imud-cal imud-imutest imud-status imud-mon imud-signalk imud-mqtt imud-influxdb imud-mavlink \
      imud-prometheus \
	      libimud.so libimud.so.* libimud.pc \
	      test_fusion test_fit_ra test_config test_nmea test_packet test_ring test_mount \
	      test_cal test_cal_math test_wmm test_position test_client test_stream \
	      test_netserv test_log test_signalk test_mqtt test_influxdb test_mavlink \
      test_libimud test_bridge test_prometheus test_capture test_concurrency \
	      test_drivers_registry test_imu_math test_drivers test_imutest \
	      fuzz_config fuzz_json fuzz_packet fuzz_capture fuzz_wmm fuzz_cal \
	      mkseed_packet \
	      src/*.gcda src/*.gcno src/drivers/*.gcda src/drivers/*.gcno \
	      lib/*.gcda lib/*.gcno *.gcda *.gcno coverage.info coverage-html \
	      etc/*.service imud-*.tar.gz
