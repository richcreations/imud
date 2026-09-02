# Optional ./configure output, included FIRST so every `?=` default below and
# every $(origin ...) guard defers to it. Absent by default: `make` alone
# probes the same things, which is what CI, debian/rules and a packager's
# build all do. `make distclean` removes it.
-include config.mk

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
#
# $(origin) rather than ?=, here and for ATOMIC_LIB and UNAME_S below: config.mk
# answers each of these, and one of the answers is legitimately EMPTY (no
# libatomic needed, no libgpiod found). ?= and ifndef both treat an empty value
# as unset and would re-probe past it.
ifeq ($(origin GPIOD_MAJ),undefined)
GPIOD_MAJ := $(shell pkg-config --modversion libgpiod 2>/dev/null | cut -d. -f1)
endif

# The GPIO backend behind include/imu_gpio.h.  src/imu_gpio.c is libgpiod;
# src/imu_gpio_null.c is the same three entry points failing with ENOSYS, which
# leaves the reader threads on the rate-sized timer they already fall back to
# when no interrupt is wired.  Empty GPIOD_LIB then drops -lgpiod from every
# link line, which is what lets imud build where the library does not exist.
#
# Off automatically when pkg-config finds no libgpiod, so a host without it
# builds rather than failing at the link; NO_GPIOD=1 forces it off where the
# library IS present.  `./configure --without-gpiod` writes the same 1 into
# config.mk, and --with-gpiod refuses rather than downgrading silently.
ifeq ($(GPIOD_MAJ),)
    NO_GPIOD ?= 1
endif
NO_GPIOD ?= 0

ifeq ($(NO_GPIOD),0)
    GPIO_SRC  = src/imu_gpio.c
    GPIOD_LIB = -lgpiod
    # Pass -DGPIOD_V2 when pkg-config reports version 2.x or newer.  Only
    # meaningful with a libgpiod backend, so it lives here rather than beside
    # the probe: with no backend there is no v1/v2 to choose between.
    ifeq ($(GPIOD_MAJ),2)
        override CPPFLAGS += -DGPIOD_V2
    endif
else
    GPIO_SRC  = src/imu_gpio_null.c
    GPIOD_LIB =
    override CPPFLAGS += -DIMUD_NO_GPIOD
endif

# The bus backend behind include/bus_backend.h.  src/bus_linux.c is the
# i2c-dev and spidev ioctls; src/bus_null.c is the same five entry points with
# open() and close() real and every transfer failing ENOSYS, which leaves a
# `driver = sim` build running the whole pipeline on a host that has neither.
# That is the state a port starts from — a BSD or a Mac builds and runs before
# it has a backend of its own.
#
# Detected by ./configure, which writes NO_LINUX_BUS into config.mk; the
# default here is the Linux backend, so a plain `make` on the target platform
# needs no configure step.
NO_LINUX_BUS ?= 0

ifeq ($(NO_LINUX_BUS),0)
    BUS_SRC = src/bus_linux.c
else
    BUS_SRC = src/bus_null.c
    override CPPFLAGS += -DIMUD_NO_LINUX_BUS
endif

# Auto-detect whether 64-bit atomics need libatomic.
#
# The cross-thread counters and timestamps are _Atomic on 64-bit types. Those
# are lock-free on aarch64 and on ARMv7, which have LDREXD/STREXD, so the
# compiler inlines them and nothing extra is linked. ARMv6 has no 64-bit
# exclusive load/store at all, so gcc emits calls to __atomic_load_8,
# __atomic_store_8, __atomic_fetch_add_8 and __atomic_exchange_8 instead, and
# those live in libatomic. The armhf packages are built for ARMv6 (see
# tools/bootstrap-raspbian.sh), which is where this bites.
#
# Probed by linking, not matched against a machine name: the requirement
# follows the instruction set the compiler is targeting, which no uname string
# reliably reports. Empty on every architecture that does not need it, so the
# link lines below are unchanged there.
#
# It has to come AFTER the objects on the link line, never in LDFLAGS: Debian
# links with --as-needed, which drops a library that precedes the objects
# referencing it, leaving exactly the undefined references it was added for.
ifeq ($(origin ATOMIC_LIB),undefined)
ATOMIC_LIB := $(shell printf '#include <stdatomic.h>\n_Atomic unsigned long long v;\nint main(void){atomic_fetch_add(&v,1);return (int)atomic_load(&v);}\n' \
                      | $(CC) -x c - -o /dev/null 2>/dev/null || echo -latomic)
endif

# ── libimud — the public client shared library ───────────────────────────────
# Linux builds the versioned .so (SONAME libimud.so.0) and the bridges link it
# (they are its first consumers). Darwin has no .so here: bridges link the
# object directly so the macOS dev/test workflow keeps working. In-tree bridge
# runs on Linux need LD_LIBRARY_PATH=. (the installed copy is found via
# ldconfig).
ifeq ($(origin UNAME_S),undefined)
UNAME_S := $(shell uname -s)
endif
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
              src/drivers/ak8963.c \
              src/drivers/rm3100.c

# Full daemon: every module
IMUD_SRCS   = src/cal.c \
              src/capture.c \
              src/cli.c \
              src/log.c \
              src/config.c \
              src/ring.c \
              src/fusion.c \
              src/imu.c \
              $(GPIO_SRC) \
              src/imu_math.c \
              src/nmea.c \
              src/netserv.c \
              src/output.c \
              src/packet.c \
              src/position.c \
              src/sdnotify.c \
              src/status_fmt.c \
              src/wmm.c \
              src/bus.c \
              $(BUS_SRC) \
              src/drivers.c \
              $(DRIVER_SRCS)

# Calibration tool: hardware access + config; no threads or output
CAL_SRCS    = src/cal.c \
              src/cal_capture.c \
              src/capture.c \
              src/cli.c \
              src/log.c \
              src/cal_math.c \
              src/config.c \
              src/bus.c \
              $(BUS_SRC) \
              src/drivers.c \
              src/fusion.c \
              src/imu_math.c \
              src/fit_ra.c \
              $(DRIVER_SRCS)

# Driver-validation tool: hardware access, config, the driver registry, and the
# swing-coverage helper shared with imud-cal.  src/capture.c is here only
# because the sim driver's .imucap playback needs it.
# src/imu.c, src/ring.c and src/fusion.c are linked deliberately: imud-imutest
# tests the daemon's driver and transport layer, so it must USE that layer
# rather than reimplement it.  It used to reimplement the GPIO edge wait, and
# every way the copy differed from the daemon was reported as a driver defect.
IMUTEST_SRCS = src/cli.c \
               src/config.c \
               src/log.c \
               src/capture.c \
               src/imu.c \
               $(GPIO_SRC) \
               src/ring.c \
               src/fusion.c \
               src/cal_math.c \
               src/imu_math.c \
               src/bus.c \
               $(BUS_SRC) \
               src/drivers.c \
               src/imutest.c \
               src/imutest_open.c \
               src/imutest_gpio.c \
               src/imutest_report.c \
               $(DRIVER_SRCS)

IMUD_OBJS    = $(IMUD_SRCS:.c=.o)
CAL_OBJS     = $(CAL_SRCS:.c=.o)
IMUTEST_OBJS = $(IMUTEST_SRCS:.c=.o)

.PHONY: all bridges libimud clean test check check-docs coverage dist install install-utils install-wmm-data install-signalk install-mqtt install-influxdb install-mavlink install-prometheus uninstall .FORCE

all: imud imud-cal imud-imutest imud-status imud-mon

# ── Binaries ──────────────────────────────────────────────────────────────────

imud: $(IMUD_OBJS) src/main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(GPIOD_LIB) -lm $(ATOMIC_LIB)

# imud-cal requires src/cal_main.c.  No GPIO library: CAL_SRCS has no imu.c and
# no driver touches a line, so imud-cal takes its samples by polling.
imud-cal: $(CAL_OBJS) src/cal_main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm $(ATOMIC_LIB)

# imud-imutest exercises any registered driver against real silicon and writes
# a Markdown validation report (ROADMAP §1).  Ships in imud-utils alongside
# imud-mon, but unlike imud-mon it must run on the box with the sensor.
# It links the GPIO backend for the interrupt edge-count check, so it follows
# imud's NO_GPIOD switch too.
imud-imutest: $(IMUTEST_OBJS) src/imutest_main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(GPIOD_LIB) -lm $(ATOMIC_LIB)

# imud-status is a plain socket client: no hardware libs.  src/cli.c stays off
# log.c precisely so this link line does not grow a pthread dependency.
imud-status: src/cli.o src/status_main.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(ATOMIC_LIB)

# imud-mon is a plain UDP consumer: needs config parsing and math
imud-mon: src/cli.o src/config.o src/log.o src/mon_parse.o src/packet.o src/mon_main.o
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm $(ATOMIC_LIB)

# imud-signalk bridges the AF_UNIX stream to Signal K delta JSON over UDP.
# Stream access + validation come from libimud ($(LIBIMUD) in $^ is either the
# versioned .so — linked directly, embedding its SONAME — or, on Darwin, the
# plain object).
imud-signalk: src/sk_delta.o src/config.o src/log.o src/netserv.o src/bridge.o src/sdnotify.o src/signalk_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm $(ATOMIC_LIB)

# imud-mqtt bridges the AF_UNIX stream to MQTT: scalar telemetry topics plus
# Home Assistant discovery, via libmosquitto.  Needs libmosquitto-dev.
imud-mqtt: src/mqtt_publish.o src/config.o src/log.o src/bridge.o src/sdnotify.o src/mqtt_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lmosquitto -lm $(ATOMIC_LIB)

# imud-influxdb bridges the AF_UNIX stream to InfluxDB line protocol over UDP or
# HTTP.  Pure C — no external dependencies beyond libimud.
imud-influxdb: src/influx_line.o src/config.o src/log.o src/bridge.o src/sdnotify.o src/influx_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm $(ATOMIC_LIB)

# imud-mavlink bridges the AF_UNIX stream to MAVLink (v1/v2) over UDP and/or
# serial.  Pure C — hand-rolled encoder, no external dependencies beyond libimud.
imud-mavlink: src/mavlink_encode.o src/config.o src/log.o src/netserv.o src/bridge.o src/sdnotify.o src/mavlink_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm $(ATOMIC_LIB)

# Optional bridge daemons — each has its own config file, service, and man page,
# and installs via its own `install-*` target (prep for per-bridge packaging).
# Kept out of `all` so a core build / CI never needs a bridge's dependencies.
# imud-prometheus serves the fused state as Prometheus /metrics gauges. Pure
# C — no external dependencies beyond libimud; the first bridge built purely
# on the ABI-stable imud_data_t (no wire pinning).
imud-prometheus: src/prom_metrics.o src/prom_http.o src/config.o src/log.o src/bridge.o src/sdnotify.o src/prom_main.o $(LIBIMUD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm $(ATOMIC_LIB)

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

# Entry points compiled as callable functions, for the end-to-end suites: the
# real main() of a daemon under the name <base>_entry.  The remaining untested
# code is the WIRING inside these main()s — config → sockets → poll →
# reload → emit → reconnect — every ingredient of which is unit-tested already
# while the joining of them never was.  Renaming at compile time tests exactly
# what ships, with no seam invented to hold it.
#
# It needs its own object rule rather than -Dmain= on the test's link line,
# because the test file has a main() of its own that must stay main().
src/%.entry.o: src/%.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -Dmain=$*_entry -c -o $@ $<

-include $(wildcard src/*.d src/drivers/*.d lib/*.d)

# ── Tests ─────────────────────────────────────────────────────────────────────

# test/rate_ladder.h is a prerequisite, not just an include: adding a rung to
# it must rebuild this suite, or the rate sweeps keep walking the old ladder
# and report a pass for rates they never ran.  $(filter %.c,$^) keeps the
# header off the compiler command line.
test_fusion: src/fusion.c test/test_fusion.c test/rate_ladder.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_fit_ra: src/fit_ra.c src/fusion.c src/imu_math.c src/capture.c test/test_fit_ra.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# The .gen.c is a prerequisite but not a translation unit: test_config.c
# #includes it inside a function.  Listed so regenerating it rebuilds the
# suite, filtered out so the compiler is not handed it twice.
test_config: src/config.c src/log.c test/test_config.c \
             test/test_config_defaults.gen.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter-out %.gen.c,$(filter %.c %.o,$^)) -lm $(ATOMIC_LIB)

# imud-mon's stream decoding (src/mon_parse.c): packet CRC, NMEA field
# extraction, flag summary.  Pure — no sockets, no config.
test_mon: src/mon_parse.c test/test_mon.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# imud-status report text (src/status_fmt.c): the gated lines, and the WS()
# truncation bound at every buffer size.  Pure formatter — config.c is here
# only for config_defaults() in the fixtures.
test_status: src/status_fmt.c src/config.c src/log.c test/test_status.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# argv parsing for the five non-bridge entry points (src/cli.c).  This is what
# SECURITY.md means by "the CLI parsers are covered by unit tests instead" —
# the bridges' shared parser is covered by test_bridge.  cli.c deliberately
# depends on nothing but libc, so this link line is one source file.
test_cli: src/cli.c test/test_cli.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) $(ATOMIC_LIB)

test_nmea: src/nmea.c test/test_nmea.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_capture: src/capture.c src/drivers/sim.c src/fusion.c src/log.c test/test_capture.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_packet: src/packet.c test/test_packet.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_concurrency: $(IMUD_OBJS) test/test_concurrency.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) $(GPIOD_LIB) -lm $(ATOMIC_LIB)

# ./configure itself.  `configure` is a prerequisite, not a translation unit:
# the suite runs the script, so an edit to it must rebuild nothing but must
# re-run this — and it must be listed, or a changed script is tested by a
# binary make believes is up to date.  $(filter %.c) keeps it off the compiler
# command line.  No sources from src/: the subject is the script.
test_configure: test/test_configure.c configure
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) $(ATOMIC_LIB)

# The no-libgpiod GPIO backend, linked on its own: it is what imud carries
# where the library is absent, so this must not depend on $(GPIO_SRC) — it
# builds and runs identically whichever backend the tree is configured for.
test_imu_gpio_null: src/imu_gpio_null.c test/test_imu_gpio_null.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) $(ATOMIC_LIB)

# The no-i2c-dev bus backend, on the same terms as test_imu_gpio_null above:
# linked on its own, never through $(BUS_SRC), so it builds and runs
# identically whichever backend the tree is configured for.
test_bus_null: src/bus_null.c test/test_bus_null.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) $(ATOMIC_LIB)

test_ring: src/ring.c test/test_ring.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_mount: src/config.c src/log.c test/test_mount.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_cal: src/cal.c src/cal_capture.c src/capture.c src/log.c test/test_cal.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# include/cal_math.h is a prerequisite, not just an include: CAL_SI_MIN_SPAN
# lives there, and a header-only edit to it left this suite stale AND green --
# a mutation of the coverage guard fired nothing until the binary was deleted
# by hand.  Same reasoning, and same $(filter %.c,$^), as test_fusion above.
test_cal_math: src/cal_math.c test/test_cal_math.c include/cal_math.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_wmm: src/wmm.c src/log.c test/test_wmm.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_position: src/position.c src/wmm.c src/config.c src/log.c test/test_position.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# Wire-format compatibility: daemon packet_build vs lib/imud_client.h.
# test_client_impl.c compiles the client header in its own translation unit,
# exactly as a third-party consumer would.
test_client: src/packet.c test/test_client.c test/test_client_impl.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# End-to-end AF_UNIX + TCP subscription stream: real output.c, stubbed imu accessors
test_stream: src/output.c src/nmea.c src/netserv.c src/packet.c src/config.c src/log.c test/test_stream.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# netserv TCP broadcast server (pure sockets; macOS-buildable)
test_netserv: src/netserv.c src/log.c test/test_netserv.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_log: src/log.c test/test_log.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# Signal K delta encoder (pure function; reuses lib/imud_client.h for the struct)
test_signalk: src/sk_delta.c test/test_signalk.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# MQTT message builders (pure functions; no libmosquitto needed)
test_mqtt: src/mqtt_publish.c test/test_mqtt.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# Prometheus metrics encoder (pure function)
test_prometheus: src/prom_metrics.c src/prom_http.c test/test_prometheus.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

test_influxdb: src/influx_line.c test/test_influxdb.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# MAVLink encoder (pure function; golden frames from a pymavlink cross-check)
test_mavlink: src/mavlink_encode.c test/test_mavlink.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# libimud public API: end-to-end over a local AF_UNIX server + UDP loopback,
# packets built by the daemon's real encoder (src/packet.c).
test_libimud: lib/libimud.c src/packet.c test/test_libimud.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# The bridge daemons end to end, main() included.  Links each real
# entry point renamed to <base>_entry by the src/%.entry.o rule, and drives it
# against test/fakestream.h — so the config→sockets→poll→reload→emit→reconnect
# wiring, which exists nowhere but inside main(), is finally executed.  Portable:
# AF_UNIX, UDP and threads only.
# All five in one binary rather than five: each shared source is then compiled
# exactly once for the whole suite, which is what keeps lcov able to attribute
# it (a source compiled into several test binaries hits lcov's stamp-mismatch
# skip and vanishes from the report).
test_bridge_e2e: src/signalk_main.entry.o src/influx_main.entry.o \
                 src/mavlink_main.entry.o src/prom_main.entry.o \
                 src/mqtt_main.entry.o \
                 src/sk_delta.c src/influx_line.c src/mavlink_encode.c \
                 src/prom_metrics.c src/prom_http.c src/mqtt_publish.c \
                 src/config.c src/log.c src/netserv.c \
                 src/bridge.c src/sdnotify.c src/packet.c lib/libimud.c \
                 test/test_bridge_e2e.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lmosquitto -lm $(ATOMIC_LIB)

# The daemon end to end, main() included: startup with no
# sensor (driver = "sim"), the stream and status sockets, SIGHUP's hot-vs-
# restart contract, and shutdown ordering.  Linux-only, like test_concurrency —
# it links the daemon objects and the GPIO backend.
#
# PID_FILE/STATUS_SOCK are redirected into the build directory for this copy of
# main.c only: /run/imud/imud.sock is a process-wide singleton, and the suite
# has to be safe to run on a Pi that is currently serving from it.
#
# SYS_CONF is redirected for a different reason: an explicit --config
# now skips the $HOME fallback, so the ONLY way a test can reach that branch is
# to run the daemon with no --config at all — which means the system config has
# to be a path the suite can rely on being absent.  /etc/imud/imud.conf is not:
# it exists on any machine where `make install` has been run, including the Pi.
src/main.entry.o: CPPFLAGS += -DPID_FILE='"/tmp/imud_e2e_daemon.pid"' \
                              -DSTATUS_SOCK='"/tmp/imud_e2e_daemon.sock"' \
                              -DSYS_CONF='"/tmp/imud_e2e_sysconf.conf"'

# The Makefile is a prerequisite of the object, not decoration: those two -D
# values live here, and make does not consider a target-specific flag change a
# reason to recompile.  Without this line, editing a path above silently leaves
# the previous one compiled in — which cost a debugging round already.
src/main.entry.o: Makefile

# --wrap=pthread_create is the seam for the thread-failure paths: main()'s four
# warn-and-continue output threads (and the one fatal one) are otherwise
# unreachable, since nothing a test can do from outside makes pthread_create
# fail.  The wrapper in test_daemon.c passes every thread through untouched
# unless the test has named its entry point, so the other cases in the suite
# are unaffected.  GNU ld only,
# like test_drivers — this suite is already Linux-only.
test_daemon: $(IMUD_OBJS) src/main.entry.o test/test_daemon.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) \
	    -Wl,--wrap=pthread_create -o $@ $(filter %.c %.o,$^) $(GPIOD_LIB) -lm $(ATOMIC_LIB)

# imud-status and imud-mon end to end, main() included.  Their pure
# halves are already covered by test_status/test_mon (status_fmt.c, mon_parse.c);
# what only main() holds is the socket work and the render loop, so this drives
# the real entry points against sockets the test binds and captures stdout.
test_tools_e2e: src/status_main.entry.o src/mon_main.entry.o \
                src/cli.c src/config.c src/log.c src/mon_parse.c src/packet.c \
                test/test_tools_e2e.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# Bridge scaffolding (src/bridge.c + src/sdnotify.c): CLI matrix, emit-tick
# timespec math (period/wait/due/advance/earlier), config load / reload /
# disabled flows, and sd_notify delivery over a test-bound NOTIFY_SOCKET.
# Links libimud directly, like test_libimud.
test_bridge: src/bridge.c src/sdnotify.c src/config.c src/log.c lib/libimud.c test/test_bridge.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# cal_capture.c is here for the .imucap loading behind `imud-cal characterize`
# and `fit-temp`; capture.c is its reader.
# Driver registry + ops-table sanity for every registered chip (no hardware).
# Links every driver: sim.c pulls in capture.c for its .imucap playback path,
# and the drivers reference log_emit (LOG_E) on their error branches.
# imu_math.c is here for odr_actual_imu/odr_actual_mag — the ODR resolution
# invariant has to hold for every registered driver, which is exactly this
# suite's job.  Linux-only (the drivers include <linux/i2c.h>), like `test`.
# rate_ladder.h listed for the same reason as test_fusion: this suite is the
# guard that the ladder covers the registry, and a stale binary would compare
# the new tables against the old ladder — or the reverse — and pass.
# src/bus_null.c rather than $(BUS_SRC): the subject is the registry — names,
# counts, ODR ladders — and it issues no transfer.  Pinning the null backend
# keeps the suite identical on every host, and makes a stray transfer fail
# with ENOSYS rather than reach whatever fd it was handed.
test_drivers_registry: src/drivers.c $(DRIVER_SRCS) src/capture.c src/log.c \
                       src/imu_math.c src/bus_null.c \
                       test/test_drivers_registry.c \
                       test/rate_ladder.h \
                       src/drivers/bus_io.h src/drivers/chip_ts.h \
                       src/drivers/st_freq_fine.h src/drivers/st_fifo_ts.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# imu.c pure math: ODR rounding, timestamp reconstruction, mount rotation,
# calibration application — the helpers factored into src/imu_math.c.  Pure
# (no <linux/*> or CLOCK_MONOTONIC), so it also builds on the macOS dev box.
test_imu_math: src/imu_math.c test/test_imu_math.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# Per-driver register decode/encode over a mock bus.  test/bus_mock.c is a
# third implementation of include/bus_backend.h and is linked in place of
# $(BUS_SRC), so this needs no --wrap and no GNU ld.  Covers the two
# hardware-validated drivers (ism330dhcx IMU, mmc5983ma mag) plus the MPU-925x
# pair, whose fuse-ROM correction and rotated magnetometer axes are worth
# pinning down off-hardware.
# src/imu_math.c is linked for snap_odr_up / odr_actual_*: test_odr_agreement
# checks each driver's own ODR encoding against the shared default rule, which
# is the invariant that lets imu.c hand the resolved rate back to the driver.
# src/bus.c is linked for test_bus_open_policy: the transport policy (SPI on a
# driver with no SPI port, clock clamping) is shared by all three tools, so it
# is asserted once here rather than in each.  -Isrc is for the quoted include
# of drivers/bus_io.h, the framing test_spi_framing exercises directly — the
# same reason test_imutest carries it.
test_drivers: src/drivers/ism330dhcx.c src/drivers/mmc5983ma.c \
              src/drivers/mpu925x.c src/drivers/ak8963.c \
              src/drivers/lsm6dso.c src/drivers/icm42688p.c \
              src/drivers/icm20948.c src/drivers/ak09916.c \
              src/drivers/lis3mdl.c src/drivers/lis2mdl.c \
              src/drivers/rm3100.c src/log.c \
              src/bus.c src/imu_math.c test/bus_mock.c test/test_drivers.c \
              src/drivers/bus_io.h src/drivers/chip_ts.h \
              src/drivers/st_freq_fine.h src/drivers/st_fifo_ts.h \
              include/bus.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -Isrc $(LDFLAGS) \
	    -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# The imud-imutest checker logic over the mock bus, with a scripted
# imt_ui_t standing in for the operator so the guided phases are covered too.
# Links the driver ops structs directly (not src/drivers.c) and stubs
# imt_gpio_count_edges, so it needs neither the registry nor -lgpiod.
# test/bus_mock.c is the backend, as in test_drivers.
# -Isrc is for test/test_imutest.c alone: it checks the mock's FIFO-port window
# through drivers/i2c_io.h, the same single-byte read path imud-imutest's
# register sweep uses, and a quoted include only searches the including file's
# own directory — which works from src/imutest.c and not from test/. src/ holds
# exactly one header (drivers/i2c_io.h) and none of include/'s names, so this
# widens the search path without shadowing anything.
test_imutest: src/imutest.c src/imutest_report.c \
              src/drivers/ism330dhcx.c src/drivers/mmc5983ma.c \
              src/config.c src/log.c src/imu_math.c src/cal_math.c \
              test/bus_mock.c test/test_imutest.c \
              src/drivers/bus_io.h src/drivers/chip_ts.h \
              src/drivers/st_freq_fine.h src/drivers/st_fifo_ts.h \
              include/bus.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -Isrc $(LDFLAGS) \
	    -o $@ $(filter %.c %.o,$^) -lm $(ATOMIC_LIB)

# imud-cal and imud-imutest end to end, main() included.  These two tools held
# the last sources that no test binary compiled at all -- cal_main.c,
# imutest_main.c, imutest_open.c, imutest_gpio.c -- so lcov did not report them
# as 0%, it did not report them.
#
# One binary rather than two, for test_bridge_e2e's stated reason: each shared
# source is then compiled exactly once for the whole suite, which is what keeps
# lcov able to attribute it.
#
# The source list is the UNION of what the two tools link, so a file added to
# either of them joins this suite without a second edit here.  $(sort) is for
# the dedupe, not the ordering -- the overlap between the two lists is most of
# both.
HWTOOLS_SRCS = $(sort $(CAL_SRCS) $(IMUTEST_SRCS))

# test/bus_mock.c replaces $(BUS_SRC) here, serving the real mmc5983ma driver
# for the --degauss path exactly as test_imutest does — hence the filter-out,
# since HWTOOLS_SRCS carries the real backend.
#
# --wrap on the three imu_gpio_* entry points is the seam for
# imutest_gpio.c: its counting POLICY is pure logic over injectable callbacks,
# and only imu_gpio_* (in $(GPIO_SRC), which IMUTEST_SRCS links deliberately)
# touch a line -- so --wrap rather than stubs, which would collide with the
# real definitions.  Same shape as --wrap=pthread_create in test_daemon.
# Linux/GNU-ld only, and the GPIO backend, like it.
test_hwtools_e2e: src/cal_main.entry.o src/imutest_main.entry.o \
                  $(filter-out $(BUS_SRC),$(HWTOOLS_SRCS)) \
                  test/bus_mock.c test/test_hwtools_e2e.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) \
	    -Wl,--wrap=imu_gpio_open -Wl,--wrap=imu_gpio_wait_edge \
	    -Wl,--wrap=imu_gpio_close \
	    -o $@ $(filter %.c %.o,$^) $(GPIOD_LIB) -lm $(ATOMIC_LIB)

TEST_BINS =test_fusion test_fit_ra test_config test_cli test_status test_mon test_nmea test_packet test_capture test_ring \
      test_concurrency \
      test_mount test_cal test_cal_math test_wmm test_position test_client \
      test_stream test_netserv test_log test_signalk test_mqtt test_influxdb \
      test_mavlink test_libimud test_bridge test_prometheus test_bridge_e2e test_tools_e2e test_daemon \
      test_drivers_registry test_imu_math test_imu_gpio_null test_bus_null test_drivers test_imutest test_hwtools_e2e \
      test_configure

# Every test binary depends on every project header.
#
# Blunt on purpose.  The suites are built by compiling their sources in ONE
# command rather than through .o files, so the -MMD fragments that keep the
# daemon honest cannot be used here: each cc1 writes the same <output>.d and the
# last source wins, leaving the other sources' headers untracked.  Checked, and
# it really does -- test_mount.d listed test/test_mount.c's includes and nothing
# from src/config.c or src/log.c.
#
# Listing headers per rule was the alternative and it rots: 29 of 34 rules had
# none at all, and the two that did were added only after a header edit left a
# suite stale.  That failure is silent and it inverts the meaning of a run --
# a mutation of CAL_SI_MIN_SPAN fired NOTHING until the binary was deleted by
# hand, i.e. the suite reported a pass for code it had not compiled.
#
# So: rebuild every suite whenever any header changes.  Over-rebuilding costs
# seconds; a stale test binary costs a wrong answer about whether the tree works.
TEST_BINS_HEADERS = $(wildcard include/*.h src/drivers/*.h test/*.h lib/*.h)
$(TEST_BINS): $(TEST_BINS_HEADERS)

test: $(TEST_BINS)
	./test_fusion
	./test_fit_ra
	./test_config
	./test_cli
	./test_status
	./test_mon
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
	./test_bridge_e2e
	./test_tools_e2e
	./test_daemon
	./test_drivers_registry
	./test_imu_math
	./test_imu_gpio_null
	./test_bus_null
	./test_drivers
	./test_imutest
	./test_hwtools_e2e
	./test_configure

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
	    src/packet.c fuzz/mkseed_packet.c -lm $(ATOMIC_LIB)
	@rm -f test/fuzz/corpus/packet/valid_v*.bin
	./mkseed_packet test/fuzz/corpus/packet/valid_v$(shell sed -n 's/^\#define IMUD_VERSION *\([0-9]*\).*/\1/p' include/types.h).bin
	@rm -f mkseed_packet
	@echo "Regenerated test/fuzz/corpus/packet/ — commit the new seed."

# GNU-convention alias
check: test

# ── Documentation cross-check ─────────────────────────────────────────────────
# Every config key src/config.c parses must appear in the shipped template,
# the man page and the manual — and nothing a template still sets may have
# lost its parser.  Pure text analysis, no build, so it runs anywhere python3
# does.  See tools/check-docs.py for the surfaces and the exception list.
.PHONY: check-docs
check-docs:
	@python3 tools/check-docs.py

# ── Config ↔ systemd unit device cross-check ─────────────────────────────────
# Every /dev node config/imud.conf names must be one etc/imud.service.in
# allows, or DevicePolicy=closed refuses the open.  This was a real defect: the
# template asked for gpiochip4 while the unit allowed only gpiochip0, and nothing
# compared them — systemd-analyze cannot see it, since it only bites at
# device-open time.  Text only, like check-docs.
.PHONY: check-devices
check-devices:
	@python3 tools/check-devices.py

# ── Packet flag cross-check ──────────────────────────────────────────────────
# The flags word is wire format with four independent definitions (the daemon,
# both standalone client headers and the Python client).  CI already refuses a
# disagreement on IMUD_VERSION; a drifted flag is worse, because the packet
# still parses and the wrong bit is read under the right name.
.PHONY: check-flags
check-flags:
	@python3 tools/check-flags.py

# ── Documentation drift checks ───────────────────────────────────────────────
# check-docs above proves every config key REACHED its doc surfaces.  These
# prove the surfaces say the right thing: that the wire table matches the
# struct, the rate lists match the driver, the topic tree matches the
# publisher, the flags match the parser, and every link leads somewhere.
#
# Written before the generators (S4/S5) deliberately: run against the tree as
# it stood they found 81 disagreements, which is the audit no amount of
# proof-reading was going to produce.  Each stays as the regression gate for
# the surface it covers, and retires when a generator makes that surface
# incapable of drifting.
#
# All text-only — no build, no network — so they run anywhere python3 does,
# including the version-consistency CI job that never compiles anything.
CHECK_DOC_TOOLS = check-links check-cli-docs check-nmea \
                  check-mqtt-topics check-bridge-outputs \
                  check-libimud-api check-math-citations check-manpages \
                  check-seccomp check-package-descriptions \
                  check-imutest-checks check-comment-refs check-arch-claims \
                  check-fuzz-targets

.PHONY: $(CHECK_DOC_TOOLS) check-generated-text test-tools check-config-docs \
        check-packet-docs check-driver-docs check-texi check-math-pdf-stamp \
        docs-config docs-tables docs-texi docs-man math-pdf \
        check-release-notes docs-release-notes \
        check-generated-man install-info-doc uninstall-info-doc
$(CHECK_DOC_TOOLS):
	@python3 tools/$@.py

# Everything text-only, in one target: what CI runs, and what to run before
# touching a document.
check-generated-text: check-docs check-devices check-flags $(CHECK_DOC_TOOLS) \
                     check-config-docs check-packet-docs check-driver-docs \
                     check-release-notes check-texi check-math-pdf-stamp

# The 150 config keys have ONE home now (docs/config-keys.toml); this asserts
# the man5 entries, the manual tables and the generated defaults test on disk
# are what it renders.  Its prose was extracted from those pages, so the first
# run matched byte for byte — a diff here is an edit that reached one surface
# and not the other.
check-config-docs:
	@python3 tools/gen-config-docs.py

# Write them.  python3 only: no build, no network, so it runs on any box.
# Run this after editing docs/config-keys.toml, then commit what it changes —
# test/test_config_defaults.gen.c is compiled into test_config, and the man5
# and manual regions ship in the packages.
docs-config:
	@python3 tools/gen-config-docs.py --write

# NEWS from 1.7 onward and the current release's changelog stanzas, from
# docs/release-notes.toml.  The registry is what keeps the three surfaces
# saying the same thing, and its word cap is what keeps a NEWS entry a
# user-visible change rather than an account of how the defect was found.
check-release-notes:
	@python3 tools/gen-release-notes.py

# Write them.  Run after editing docs/release-notes.toml.  Older NEWS entries
# and already-released changelog stanzas are never rewritten: only the stanza
# for the version in include/version.h is touched.
docs-release-notes:
	@python3 tools/gen-release-notes.py --write

# spec.md's packet table and flags bitmask, from include/types.h.  Offsets in
# a packed struct are cumulative sizeof: insert a field and every row below it
# moves, which is exactly what a human proof-reader does not catch.  This
# replaced check-packet.py — a checked table can still be wrong between
# checks, a generated one cannot be wrong at all.
check-packet-docs:
	@python3 tools/gen-packet-docs.py

# docs/manual.md §5, from src/drivers.c and the *_ops initialisers.  The four
# generated columns are the ones that restate the code — name, type, SPI mode
# and clock, and the experimental marker; the parts, addresses and notes are
# prose in docs/driver-notes.toml.  This replaced check-drivers.py, and also
# absorbed its check on the driver-name lists in the [imu]/[mag] key prose,
# which stay hand-written because they are a sentence rather than a table.
check-driver-docs:
	@python3 tools/gen-drivers.py

docs-tables:
	@python3 tools/gen-packet-docs.py --write
	@python3 tools/gen-drivers.py --write

# ── Info manual ──────────────────────────────────────────────────────────────
# docs/manual.md -> docs/imud.texi -> imud.info.  `info imud` is the third way
# to read the manual and the only navigable one, in a reader already on every
# Debian system.
#
# docs/imud.texi is COMMITTED (make dist is `git archive HEAD`, so a packager
# must not need pandoc) but not diff-gated: this tree meets pandoc 3.10 and
# 3.1.11, they disagree on the Texinfo they emit, and a regenerate-and-diff
# would be red on whichever runner did not match.  check-texi is the gate
# instead, and needs no pandoc.
docs-texi:
	@python3 tools/gen-texi.py

check-texi:
	@python3 tools/check-texi.py

imud.info: docs/imud.texi
	makeinfo --no-split -o $@ $<

# NOT a prerequisite of `install`: makeinfo is not a build dependency, and a
# source install must keep working on a box without texinfo.  Debian's
# dh_installinfo calls this via debian/imud.info.
#
# Installed UNCOMPRESSED.  dh_compress handles /usr/share/info/*.info itself
# and runs after dh_install, so compressing here would leave it a .info.gz.gz.
install-info-doc: imud.info
	install -d -m 0755 $(DESTDIR)$(INFODIR)
	install -m 0644 imud.info $(DESTDIR)$(INFODIR)/imud.info
	@# Register with the Info directory when installing for real.  Debian
	@# packages get this from dh_installinfo's trigger instead, which is why
	@# a staged install (DESTDIR set) must not touch the system dir file.
	@if [ -z "$(DESTDIR)" ] && command -v install-info >/dev/null 2>&1; then \
	  install-info --quiet $(INFODIR)/imud.info $(INFODIR)/dir || true; \
	fi

# ── math.pdf ─────────────────────────────────────────────────────────────────
# A rendered copy of docs/math.md for reading the derivations away from a
# terminal.  Committed, because the alternative is a LaTeX toolchain as a
# prerequisite for reading the maths.
#
# tectonic (one ~30 MB binary) before xelatex (TeX Live, ~4 GB): the script
# used to hard-require xelatex, so it failed on the box where math.md was
# being edited and the PDF sat three weeks behind its source.
math-pdf:
	@sh tools/build-math-pdf.sh

# The PDF is not reproducible — pandoc, the engine and the fonts all vary —
# so it cannot be rebuilt-and-diffed.  docs/math.pdf.stamp holds the sha256
# of the math.md it was built from, and this compares that against math.md.
# An mtime rule could not work: git does not preserve mtimes.
check-math-pdf-stamp:
	@python3 tools/check-math-pdf-stamp.py

uninstall-info-doc:
	@if [ -z "$(DESTDIR)" ] && command -v install-info >/dev/null 2>&1; then \
	  install-info --quiet --delete $(INFODIR)/imud.info $(INFODIR)/dir || true; \
	fi
	rm -f $(DESTDIR)$(INFODIR)/imud.info

# The checkers are regexes over source, so a checker that has quietly stopped
# matching looks exactly like a clean tree.  This breaks one fact at a time in
# a copy of the tree and asserts the relevant checker notices.
test-tools:
	@python3 test/test_checkers.py

# ── Generated man pages (help2man) ───────────────────────────────────────────
# The man1/man8 pages are generated from each tool's own --help, so the flag
# list in the page and the flag list the tool prints cannot disagree — they are
# the same text.  Everything that is not a flag list (DESCRIPTION, SIGNALS,
# FILES, EXAMPLES, ...) lives in man/h2m/<page>.h2m and is spliced in.
#
# man5 and man3 are NOT here: help2man documents a command's options, and a
# config-file format is not that.  Those stay hand-written.
#
# This is the one docs target that needs a BUILD — help2man runs the binary.
# On macOS imud, imud-cal and imud-imutest do not link at all, so regenerating
# is a devbox/Linux operation.  The generated pages are committed, so a
# packager never runs help2man and `make dist` (git archive HEAD) still ships
# a complete tree.
MAN_GENERATED = $(addprefix man/,$(filter man1/% man8/%,$(MAN_ALL)))

# $(call run-help2man,<binary>,<section>,<manual>,<output>)
# LD_LIBRARY_PATH: the bridges link libimud, which is not installed yet.
define run-help2man
	@TZ=UTC0 LC_ALL=C LD_LIBRARY_PATH=. help2man \
	    --output=$(4) --section=$(2) --manual="$(3)" \
	    --source="imud $(VERSION)" --info-page=imud \
	    --include=man/h2m/$(notdir $(4)).h2m ./$(1)
	@python3 tools/man-postprocess.py $(4) >/dev/null
	@echo "  generated $(4)"
endef

man/man1/%.1: % man/h2m/%.1.h2m tools/man-postprocess.py include/version.h
	$(call run-help2man,$*,1,User Commands,$@)

man/man8/%.8: % man/h2m/%.8.h2m tools/man-postprocess.py include/version.h
	$(call run-help2man,$*,8,System Manager's Manual,$@)

# -B: regenerate unconditionally.  Without it make skips a page whose mtime is
# newer than its prerequisites, so a hand-edit that got committed would never
# be overwritten and check-generated-man would pass on it — the exact thing the
# gate exists to prevent.  Ten help2man runs cost about a second.
docs-man: all bridges
	@$(MAKE) --no-print-directory -B $(MAN_GENERATED)

# What CI runs: regenerating must change nothing that is committed.
check-generated-man: docs-man
	@git diff --quiet -- man/ || { \
	    echo "man pages are stale — run 'make docs-man' and commit" >&2; \
	    git --no-pager diff --stat -- man/ >&2; \
	    exit 1; }
	@echo "check-generated-man: $(words $(MAN_GENERATED)) pages regenerate unchanged"

# ── Line count ────────────────────────────────────────────────────────────────
# Size of the tree, split by ROLE rather than by language: production C apart
# from the suites, the fuzz harnesses apart from both.  cloc/tokei/scc report
# per language, which puts every .c in one row and hides the number actually
# worth watching here — the test suite is slightly larger than the code it
# tests.  A report, deliberately not a gate: it is not in `check`, and CI prints
# it to the run summary without ever failing on it.
.PHONY: loc
loc:
	@python3 tools/loc.py

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
	    python3 tools/coverage-gaps.py coverage.info; \
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
# Packagers: pass PREFIX=/usr SVCDIR=/usr/lib/systemd/system
# UDEVDIR=/usr/lib/udev/rules.d DESTDIR=<stage>.
# When DESTDIR is set the install is a pure file copy: no useradd, no
# systemctl — those belong to the package's maintainer scripts.

PREFIX  ?= /usr/local
ETCDIR  ?= /etc/imud
SVCDIR  ?= /etc/systemd/system
LIBDIR  ?= $(PREFIX)/lib
MANDIR  ?= $(PREFIX)/share/man
DOCDIR  ?= $(PREFIX)/share/doc
INFODIR ?= $(PREFIX)/share/info
# udev reads rules from /etc/udev/rules.d, /run/udev/rules.d and
# /usr/lib/udev/rules.d only — never from $(PREFIX) — so a source install must
# land in /etc or the rule is inert.  Packagers override this to
# /usr/lib/udev/rules.d (/etc is reserved for the admin's own overrides).
UDEVDIR ?= /etc/udev/rules.d

# ── Man pages, one list per installed package ────────────────────────────────
# These existed three times over — once in each install-* recipe, again in
# uninstall, and a third time in debian/*.install — with nothing comparing
# them.  A page that exists and is never installed passes CI, because the
# mandoc lint and the .TH version check both glob man/ and neither knows what
# ships.
#
# install-* and uninstall now read these variables, so a new page is added in
# one place.  debian/*.install cannot: dh_install reads it from the source
# package before any Makefile runs, and generating it in debian/rules would
# leave build-modified files under debian/, which dpkg-source rejects on a
# 3.0 (quilt) package.  tools/check-manpages.py closes that third copy instead.
MAN_imud       = man8/imud.8 man8/imud-cal.8 man5/imud.conf.5 man1/imud-status.1
MAN_libimud    = man3/libimud.3
MAN_utils      = man1/imud-mon.1 man8/imud-imutest.8
MAN_signalk    = man8/imud-signalk.8    man5/imud-signalk.conf.5
MAN_mqtt       = man8/imud-mqtt.8       man5/imud-mqtt.conf.5
MAN_influxdb   = man8/imud-influxdb.8   man5/imud-influxdb.conf.5
MAN_prometheus = man8/imud-prometheus.8 man5/imud-prometheus.conf.5
MAN_mavlink    = man8/imud-mavlink.8    man5/imud-mavlink.conf.5

MAN_ALL = $(MAN_imud) $(MAN_libimud) $(MAN_utils) $(MAN_signalk) $(MAN_mqtt) \
          $(MAN_influxdb) $(MAN_prometheus) $(MAN_mavlink)

# $(call install-man,<list>) — gzip man/<sec>/<page> into $(MANDIR)/<sec>/.
# ${p%%/*} is the section directory, ${p##*/} the page: POSIX parameter
# expansion, so it behaves the same in the macOS and Debian shells this
# project builds under.
define install-man
	@for p in $(1); do \
	    d=$(DESTDIR)$(MANDIR)/$${p%%/*}; \
	    install -d -m 0755 "$$d"; \
	    gzip -9nc man/$$p > "$$d/$${p##*/}.gz"; \
	done
endef

# Printing a variable for tools/check-manpages.py, which has to see the same
# list the install rules use rather than a second copy of it.
print-%:
	@echo "$($*)"

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
	# ── System user + hardware groups (skipped for staged installs: DESTDIR set) ─
	# The groups are created, not just joined: Raspberry Pi OS ships gpio, i2c
	# and spi, stock Debian ships none, and imud.service's SupplementaryGroups=
	# refuses to start without them.  Done on every install, not only when the
	# user is created, so an upgrade repairs a host that was missing them.
	@if [ -z "$(DESTDIR)" ]; then \
	    if ! id -u imud >/dev/null 2>&1; then \
	        useradd --system --no-create-home --shell /usr/sbin/nologin imud; \
	        echo "Created system user 'imud'"; \
	    fi; \
	    for grp in gpio i2c spi; do \
	        getent group "$$grp" >/dev/null 2>&1 || groupadd --system "$$grp" 2>/dev/null || true; \
	        usermod -aG "$$grp" imud 2>/dev/null || true; \
	    done; \
	fi
	# ── Config + calibration (all in /etc/imud) ────────────────────────────
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud.conf" ]; then \
	    install -m 644 config/imud.conf $(DESTDIR)$(ETCDIR)/imud.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud.conf"; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud.conf"; \
	fi
	# Never into a staged (packaging) root: cal.json is one machine's
	# calibration, and capturing a developer's copy into a .deb would ship it
	# to every user.  It also breaks the build outright — nothing in
	# debian/*.install claims etc/imud/cal.json, so dh_missing aborts with
	# "missing files".  CI never sees that because a fresh checkout has no
	# cal.json; anyone who runs imud-cal before dpkg-buildpackage does.
	@if [ -n "$(DESTDIR)" ]; then \
	    echo "Staged install: skipping config/cal.json (machine-specific)"; \
	elif [ -f "config/cal.json" ]; then \
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
	# ── udev rule: group access to /dev/i2c-* and /dev/gpiochip* ───────────
	#
	# NOTE THE MISSING --subsystem-match=gpio.  It is not an oversight, and
	# putting it back breaks installs on a Raspberry Pi 5.  Pi OS ships a
	# compatibility rule that aliases the header GPIO chip to its old name:
	#
	#   SUBSYSTEM=="gpio", KERNEL=="gpiochip0", \
	#     PROGRAM="/usr/bin/test ! -c /dev/gpiochip4", SYMLINK+="gpiochip4"
	#
	# (Pi 5's RP1 is on PCIe and used to enumerate late as gpiochip4; kernels
	# from ~2024-07 renumber it to gpiochip0 like every other Pi.)  That guard
	# OSCILLATES: with the symlink present, `test ! -c` fails, no SYMLINK+= is
	# applied, and udev recomputes the device's symlink set WITHOUT it — so the
	# trigger deletes it.  Trigger again and it comes back.  A 1.9.0 RC bench
	# run on a Pi 5 caught it: after `make install`, /dev/gpiochip4 was gone and
	# the daemon died with "cannot open /dev/gpiochip4" until the next reboot.
	#
	# i2c-dev and spidev have no such third-party rule, so they still trigger.
	# The gpio nodes get the same outcome from the direct chgrp/chmod below,
	# without re-running anybody else's rules; 60-imud.rules covers every boot
	# and hotplug after this one.
	install -d -m 0755 $(DESTDIR)$(UDEVDIR)
	install -m 644 etc/60-imud.rules $(DESTDIR)$(UDEVDIR)/60-imud.rules
	@if [ -z "$(DESTDIR)" ] && command -v udevadm >/dev/null 2>&1; then \
	    udevadm control --reload-rules || true; \
	    udevadm trigger --subsystem-match=i2c-dev --subsystem-match=spidev || true; \
	fi
	@if [ -z "$(DESTDIR)" ]; then \
	    for n in /dev/gpiochip*; do \
	        [ -e "$$n" ] || continue; \
	        chgrp gpio "$$n" 2>/dev/null || true; \
	        chmod 0660 "$$n" 2>/dev/null || true; \
	    done; \
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
	$(call install-man,$(MAN_libimud))
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
	$(call install-man,$(MAN_imud))
	@echo "Installed man pages to $(DESTDIR)$(MANDIR)"
	# ── Documentation (/usr/share/doc/imud) ────────────────────────────────
	#
	# The installed tree MIRRORS the source tree: repo-root files land at the
	# top and docs/ keeps its directory.  That is not cosmetic — every one of
	# these documents links to the others by relative path, and flattening
	# docs/manual.md to manual.md broke every `docs/...` link the moment it
	# was installed while leaving it perfectly correct on GitHub.  Preserving
	# the prefix is the only arrangement where one relative path is right in
	# both trees.  tools/check-links.py enforces it.
	#
	# Everything a shipped document links to must ship too, for the same
	# reason: a link to a file no package installs is a dangling link.  That
	# is why DCO, GOVERNANCE.md, LICENSE, SECURITY.md and CODE_OF_CONDUCT.md
	# are here — README.md and CONTRIBUTING.md reference them.
	#
	# INSTALL is the one deliberate exception: it is build-from-source
	# guidance, ships in the tarball, and Debian rejects it in a binary
	# package (package-contains-upstream-installation-documentation).  Nothing
	# installed links to it.
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud/examples \
	                   $(DESTDIR)$(DOCDIR)/imud/docs \
	                   $(DESTDIR)$(DOCDIR)/imud/devbox
	install -m 644 AUTHORS NEWS README.md CONTRIBUTING.md GOVERNANCE.md DCO \
	               LICENSE SECURITY.md CODE_OF_CONDUCT.md spec.md \
	               $(DESTDIR)$(DOCDIR)/imud/
	install -m 644 docs/manual.md docs/ROADMAP.md docs/RELEASING.md \
	               docs/capture.md docs/datasheets.md docs/math.md \
	               $(DESTDIR)$(DOCDIR)/imud/docs/
	# CONTRIBUTING.md points at the dev container guide by relative path.
	install -m 644 devbox/README.md $(DESTDIR)$(DOCDIR)/imud/devbox/
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
	$(call install-man,$(MAN_utils))
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
	$(call install-man,$(MAN_signalk))
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
	# 0640, not 0644: this file can hold a plaintext broker password.
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-mqtt.conf" ]; then \
	    install -m 640 config/imud-mqtt.conf $(DESTDIR)$(ETCDIR)/imud-mqtt.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-mqtt.conf (0640)"; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-mqtt.conf"; \
	fi
	# Outside the block above on purpose: an upgrade over an existing 0644 file
	# is the common case and must be repaired too.  Never leave the file
	# unreadable by the daemon — without the group, fall back to 0644 and say so.
	@if [ -z "$(DESTDIR)" ] && [ -f "$(ETCDIR)/imud-mqtt.conf" ]; then \
	    if getent group imud >/dev/null 2>&1; then \
	        chgrp imud "$(ETCDIR)/imud-mqtt.conf" && chmod 640 "$(ETCDIR)/imud-mqtt.conf"; \
	    else \
	        chmod 644 "$(ETCDIR)/imud-mqtt.conf"; \
	        echo "WARNING: group 'imud' missing — left $(ETCDIR)/imud-mqtt.conf 0644."; \
	        echo "         Install imud itself first, then re-run, or the bridge cannot read it."; \
	    fi; \
	fi
	$(call install-man,$(MAN_mqtt))
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
	# 0640, not 0644: this file can hold a plaintext InfluxDB API token.
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-influxdb.conf" ]; then \
	    install -m 640 config/imud-influxdb.conf $(DESTDIR)$(ETCDIR)/imud-influxdb.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-influxdb.conf (0640)"; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-influxdb.conf"; \
	fi
	# Outside the block above on purpose: an upgrade over an existing 0644 file
	# is the common case and must be repaired too.  Never leave the file
	# unreadable by the daemon — without the group, fall back to 0644 and say so.
	@if [ -z "$(DESTDIR)" ] && [ -f "$(ETCDIR)/imud-influxdb.conf" ]; then \
	    if getent group imud >/dev/null 2>&1; then \
	        chgrp imud "$(ETCDIR)/imud-influxdb.conf" && chmod 640 "$(ETCDIR)/imud-influxdb.conf"; \
	    else \
	        chmod 644 "$(ETCDIR)/imud-influxdb.conf"; \
	        echo "WARNING: group 'imud' missing — left $(ETCDIR)/imud-influxdb.conf 0644."; \
	        echo "         Install imud itself first, then re-run, or the bridge cannot read it."; \
	    fi; \
	fi
	$(call install-man,$(MAN_influxdb))
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
	$(call install-man,$(MAN_prometheus))
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
	$(call install-man,$(MAN_mavlink))
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
	      $(DESTDIR)$(UDEVDIR)/60-imud.rules \
	      $(DESTDIR)$(SVCDIR)/imud.service \
	      $(DESTDIR)$(SVCDIR)/imud-signalk.service \
	      $(DESTDIR)$(SVCDIR)/imud-mqtt.service \
	      $(DESTDIR)$(SVCDIR)/imud-influxdb.service \
	      $(DESTDIR)$(SVCDIR)/imud-prometheus.service \
	      $(DESTDIR)$(SVCDIR)/imud-mavlink.service
	# Same list the install rules use, so a page cannot be installed and then
	# left behind by uninstall.
	@for p in $(MAN_ALL); do \
	    rm -f $(DESTDIR)$(MANDIR)/$${p%%/*}/$${p##*/}.gz; \
	done
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

# The second rm needs -r: rm -f silently skips directories. coverage-html is
# genhtml's output tree; the .dSYM bundles are debug symbols that the macOS
# clang driver produces by running dsymutil whenever a compile-and-link
# invocation carries -g — the shape of every test recipe here, so `make
# coverage` leaves one per suite. The glob matches nothing on Linux.
clean:
	rm -f src/*.o src/drivers/*.o src/*.d src/drivers/*.d lib/*.o lib/*.d \
	      imud imud-cal imud-imutest imud-status imud-mon imud-signalk imud-mqtt imud-influxdb imud-mavlink \
      imud-prometheus \
	      libimud.so libimud.so.* libimud.pc \
	      test_fusion test_fit_ra test_config test_cli test_status test_mon test_nmea test_packet test_ring test_mount \
	      test_cal test_cal_math test_wmm test_position test_client test_stream \
	      test_netserv test_log test_signalk test_mqtt test_influxdb test_mavlink \
      test_libimud test_bridge test_prometheus test_bridge_e2e test_tools_e2e test_daemon test_capture test_concurrency \
	      test_drivers_registry test_imu_math test_imu_gpio_null test_bus_null test_drivers test_imutest \
      test_hwtools_e2e test_configure \
	      fuzz_config fuzz_json fuzz_packet fuzz_capture fuzz_wmm fuzz_cal \
	      fuzz_argv \
	      mkseed_packet imud.info \
	      src/*.gcda src/*.gcno src/drivers/*.gcda src/drivers/*.gcno \
	      lib/*.gcda lib/*.gcno *.gcda *.gcno coverage.info \
	      etc/*.service imud-*.tar.gz
	rm -rf coverage-html *.dSYM

# GNU convention: clean removes what make built, distclean also removes what
# configure wrote.  Deleting config.mk returns the build to the Makefile's own
# probes, which is the state CI and debian/rules always build in.
.PHONY: distclean
distclean: clean
	rm -f config.mk
