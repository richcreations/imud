CC      = gcc
CFLAGS  = -std=c11 -O2 -Wall -Wextra -pthread -Iinclude -D_GNU_SOURCE
LDFLAGS = -lgpiod -lm

# Auto-detect libgpiod major version; default to v1 (Bookworm ships 1.x).
# Pass -DGPIOD_V2 when pkg-config reports version 2.x or newer.
GPIOD_MAJ := $(shell pkg-config --modversion libgpiod 2>/dev/null | cut -d. -f1)
ifeq ($(GPIOD_MAJ),2)
    CFLAGS += -DGPIOD_V2
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
              src/drivers/lis2mdl.c

# Full daemon: every module
IMUD_SRCS   = src/cal.c \
              src/log.c \
              src/config.c \
              src/ring.c \
              src/fusion.c \
              src/imu.c \
              src/nmea.c \
              src/output.c \
              src/packet.c \
              src/position.c \
              src/wmm.c \
              src/drivers.c \
              $(DRIVER_SRCS)

# Calibration tool: hardware access + config; no threads or output
CAL_SRCS    = src/cal.c \
              src/log.c \
              src/cal_math.c \
              src/config.c \
              src/drivers.c \
              $(DRIVER_SRCS)

IMUD_OBJS   = $(IMUD_SRCS:.c=.o)
CAL_OBJS    = $(CAL_SRCS:.c=.o)

.PHONY: all bridges clean test install install-signalk install-mqtt install-influxdb install-mavlink uninstall

all: imud imud-cal imud-status imud-mon

# ── Binaries ──────────────────────────────────────────────────────────────────

imud: $(IMUD_OBJS) src/main.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# imud-cal requires src/cal_main.c
imud-cal: $(CAL_OBJS) src/cal_main.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# imud-status is a plain socket client: no hardware libs
imud-status: src/status_main.o
	$(CC) $(CFLAGS) -o $@  $^

# imud-mon is a plain UDP consumer: needs config parsing and math
imud-mon: src/config.o src/log.o src/mon_main.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

# imud-signalk bridges the AF_UNIX stream to Signal K delta JSON over UDP.
# Reuses the public client header (lib/imud_client.h) for packet validation.
imud-signalk: src/sk_delta.o src/config.o src/log.o src/signalk_main.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

# imud-mqtt bridges the AF_UNIX stream to MQTT: scalar telemetry topics plus
# Home Assistant discovery, via libmosquitto.  Needs libmosquitto-dev.
imud-mqtt: src/mqtt_publish.o src/config.o src/log.o src/mqtt_main.o
	$(CC) $(CFLAGS) -o $@ $^ -lmosquitto -lm

# imud-influxdb bridges the AF_UNIX stream to InfluxDB line protocol over UDP or
# HTTP.  Pure C — no external dependencies.
imud-influxdb: src/influx_line.o src/config.o src/log.o src/influx_main.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

# imud-mavlink bridges the AF_UNIX stream to MAVLink (v1/v2) over UDP and/or
# serial.  Pure C — hand-rolled encoder, no external dependencies.
imud-mavlink: src/mavlink_encode.o src/config.o src/log.o src/mavlink_main.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

# Optional bridge daemons — each has its own config file, service, and man page,
# and installs via its own `install-*` target (prep for per-bridge packaging).
# Kept out of `all` so a core build / CI never needs a bridge's dependencies.
bridges: imud-signalk imud-mqtt imud-influxdb imud-mavlink

# ── Compilation rules ─────────────────────────────────────────────────────────

# -MMD -MP writes a .d makefile fragment per object so header edits rebuild
# their dependents; the -include pulls in whichever fragments exist so far.
DEPFLAGS = -MMD -MP

src/%.o: src/%.c
	$(CC) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

src/drivers/%.o: src/drivers/%.c
	$(CC) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

-include $(wildcard src/*.d src/drivers/*.d)

# ── Tests ─────────────────────────────────────────────────────────────────────

test_fusion: src/fusion.c test/test_fusion.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_config: src/config.c src/log.c test/test_config.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_nmea: src/nmea.c test/test_nmea.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_packet: src/packet.c test/test_packet.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_ring: src/ring.c test/test_ring.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_mount: src/config.c src/log.c test/test_mount.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_cal: src/cal.c src/log.c test/test_cal.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_cal_math: src/cal_math.c test/test_cal_math.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_wmm: src/wmm.c src/log.c test/test_wmm.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_position: src/position.c src/wmm.c src/log.c test/test_position.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

# Wire-format compatibility: daemon packet_build vs lib/imud_client.h.
# test_client_impl.c compiles the client header in its own translation unit,
# exactly as a third-party consumer would.
test_client: src/packet.c test/test_client.c test/test_client_impl.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

# End-to-end AF_UNIX subscription stream: real output.c, stubbed imu accessors
test_stream: src/output.c src/nmea.c src/packet.c src/config.c src/log.c test/test_stream.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_log: src/log.c test/test_log.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

# Signal K delta encoder (pure function; reuses lib/imud_client.h for the struct)
test_signalk: src/sk_delta.c test/test_signalk.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

# MQTT message builders (pure functions; no libmosquitto needed)
test_mqtt: src/mqtt_publish.c test/test_mqtt.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

# InfluxDB line-protocol encoder (pure function)
test_influxdb: src/influx_line.c test/test_influxdb.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

# MAVLink encoder (pure function; golden frames from a pymavlink cross-check)
test_mavlink: src/mavlink_encode.c test/test_mavlink.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test: test_fusion test_config test_nmea test_packet test_ring test_mount \
      test_cal test_cal_math test_wmm test_position test_client test_stream \
      test_log test_signalk test_mqtt test_influxdb test_mavlink
	./test_fusion
	./test_config
	./test_nmea
	./test_packet
	./test_ring
	./test_mount
	./test_cal
	./test_cal_math
	./test_wmm
	./test_position
	./test_client
	./test_stream
	./test_log
	./test_signalk
	./test_mqtt
	./test_influxdb
	./test_mavlink

# ── Install ───────────────────────────────────────────────────────────────────

PREFIX  ?= /usr/local
ETCDIR  ?= /etc/imud
SVCDIR  ?= /etc/systemd/system
MANDIR  ?= $(PREFIX)/share/man
DOCDIR  ?= $(PREFIX)/share/doc

install: imud imud-cal imud-status imud-mon
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(SVCDIR)
	install -m 755 imud imud-cal imud-status imud-mon $(DESTDIR)$(PREFIX)/bin/
	# ── System user ────────────────────────────────────────────────────────
	@if ! id -u imud >/dev/null 2>&1; then \
	    useradd --system --no-create-home --shell /usr/sbin/nologin imud; \
	    usermod -aG gpio imud 2>/dev/null || true; \
	    usermod -aG i2c  imud 2>/dev/null || true; \
	    echo "Created system user 'imud'"; \
	fi
	# ── Config + calibration (all in /etc/imud) ────────────────────────────
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud.conf" ]; then \
	    sed 's|"config/cal.json"|"$(ETCDIR)/cal.json"|' \
	        config/imud.conf > $(DESTDIR)$(ETCDIR)/imud.conf; \
	    chmod 644 $(DESTDIR)$(ETCDIR)/imud.conf; \
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
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/WMM.COF" ]; then \
	    install -m 644 data/WMM.COF $(DESTDIR)$(ETCDIR)/WMM.COF; \
	    echo "Installed WMM2025 coefficients: $(DESTDIR)$(ETCDIR)/WMM.COF"; \
	else \
	    echo "WMM.COF already present — skipping (preserving operator-installed model)"; \
	fi
	# ── Systemd service ────────────────────────────────────────────────────
	install -m 644 etc/imud.service         $(DESTDIR)$(SVCDIR)/imud.service
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	# ── Client libraries ───────────────────────────────────────────────────
	install -d -m 0755 $(DESTDIR)$(PREFIX)/include $(DESTDIR)$(PREFIX)/share/imud
	install -m 644 lib/imud_client.h  $(DESTDIR)$(PREFIX)/include/imud_client.h
	install -m 644 lib/imud_client.py $(DESTDIR)$(PREFIX)/share/imud/imud_client.py
	@echo "Installed client libs:  $(DESTDIR)$(PREFIX)/include/imud_client.h, $(DESTDIR)$(PREFIX)/share/imud/imud_client.py"
	# ── Man pages ──────────────────────────────────────────────────────────
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man1 \
	                   $(DESTDIR)$(MANDIR)/man5 \
	                   $(DESTDIR)$(MANDIR)/man8
	gzip -9c man/man1/imud-status.1 > $(DESTDIR)$(MANDIR)/man1/imud-status.1.gz
	gzip -9c man/man1/imud-mon.1    > $(DESTDIR)$(MANDIR)/man1/imud-mon.1.gz
	gzip -9c man/man5/imud.conf.5   > $(DESTDIR)$(MANDIR)/man5/imud.conf.5.gz
	gzip -9c man/man8/imud.8        > $(DESTDIR)$(MANDIR)/man8/imud.8.gz
	gzip -9c man/man8/imud-cal.8    > $(DESTDIR)$(MANDIR)/man8/imud-cal.8.gz
	@echo "Installed man pages to $(DESTDIR)$(MANDIR)"
	# ── Documentation (/usr/share/doc/imud) ────────────────────────────────
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud/examples
	install -m 644 AUTHORS NEWS INSTALL README.md CONTRIBUTING.md spec.md \
	               docs/manual.md docs/ROADMAP.md $(DESTDIR)$(DOCDIR)/imud/
	install -m 644 packaging/imud/copyright $(DESTDIR)$(DOCDIR)/imud/copyright
	gzip -9c packaging/imud/changelog > $(DESTDIR)$(DOCDIR)/imud/changelog.gz
	install -m 644 config/imud.conf $(DESTDIR)$(DOCDIR)/imud/examples/imud.conf
	@echo "Installed docs to       $(DESTDIR)$(DOCDIR)/imud"
	@echo ""
	@echo "Next steps:"
	@echo "  sudo systemctl enable --now imud"
	@echo "  review $(ETCDIR)/imud.conf  (i2c_bus, gpio_chip, rotation_euler_deg)"

# ── Install the Signal K bridge (optional) ─────────────────────────────────────
# Run after `make bridges`.  Installs the binary, service, man page, and its own
# config file (non-clobbering).  Prep for a standalone imud-signalk package.
install-signalk: imud-signalk
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
	gzip -9c man/man5/imud-signalk.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-signalk.conf.5.gz
	gzip -9c man/man8/imud-signalk.8 > $(DESTDIR)$(MANDIR)/man8/imud-signalk.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-signalk/examples
	install -m 644 docs/imud-signalk/README.md docs/imud-signalk/manual.md \
	               docs/imud-signalk/spec.md $(DESTDIR)$(DOCDIR)/imud-signalk/
	install -m 644 packaging/imud-signalk/copyright $(DESTDIR)$(DOCDIR)/imud-signalk/copyright
	gzip -9c packaging/imud-signalk/changelog > $(DESTDIR)$(DOCDIR)/imud-signalk/changelog.gz
	install -m 644 config/imud-signalk.conf $(DESTDIR)$(DOCDIR)/imud-signalk/examples/imud-signalk.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-signalk.  Enable with: sudo systemctl enable --now imud-signalk"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-signalk.conf)"

# ── Install the MQTT bridge (optional) ─────────────────────────────────────────
# Run after `make imud-mqtt` (needs libmosquitto-dev).  Installs the binary,
# service, man page, and its own config file (non-clobbering).
install-mqtt: imud-mqtt
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
	gzip -9c man/man5/imud-mqtt.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-mqtt.conf.5.gz
	gzip -9c man/man8/imud-mqtt.8 > $(DESTDIR)$(MANDIR)/man8/imud-mqtt.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-mqtt/examples
	install -m 644 docs/imud-mqtt/README.md docs/imud-mqtt/manual.md \
	               docs/imud-mqtt/spec.md $(DESTDIR)$(DOCDIR)/imud-mqtt/
	install -m 644 packaging/imud-mqtt/copyright $(DESTDIR)$(DOCDIR)/imud-mqtt/copyright
	gzip -9c packaging/imud-mqtt/changelog > $(DESTDIR)$(DOCDIR)/imud-mqtt/changelog.gz
	install -m 644 config/imud-mqtt.conf $(DESTDIR)$(DOCDIR)/imud-mqtt/examples/imud-mqtt.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-mqtt.  Enable with: sudo systemctl enable --now imud-mqtt"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-mqtt.conf)"

# ── Install the InfluxDB bridge (optional) ─────────────────────────────────────
# Run after `make imud-influxdb`.  Installs the binary, service, man page, and
# its own config file (non-clobbering).
install-influxdb: imud-influxdb
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
	gzip -9c man/man5/imud-influxdb.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-influxdb.conf.5.gz
	gzip -9c man/man8/imud-influxdb.8 > $(DESTDIR)$(MANDIR)/man8/imud-influxdb.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-influxdb/examples
	install -m 644 docs/imud-influxdb/README.md docs/imud-influxdb/manual.md \
	               docs/imud-influxdb/spec.md $(DESTDIR)$(DOCDIR)/imud-influxdb/
	install -m 644 packaging/imud-influxdb/copyright $(DESTDIR)$(DOCDIR)/imud-influxdb/copyright
	gzip -9c packaging/imud-influxdb/changelog > $(DESTDIR)$(DOCDIR)/imud-influxdb/changelog.gz
	install -m 644 config/imud-influxdb.conf $(DESTDIR)$(DOCDIR)/imud-influxdb/examples/imud-influxdb.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-influxdb.  Enable with: sudo systemctl enable --now imud-influxdb"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-influxdb.conf)"

# ── Install the MAVLink bridge (optional) ──────────────────────────────────────
# Run after `make imud-mavlink`.  Installs the binary, service, man page, and its
# own config file (non-clobbering).
install-mavlink: imud-mavlink
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
	gzip -9c man/man5/imud-mavlink.conf.5 > $(DESTDIR)$(MANDIR)/man5/imud-mavlink.conf.5.gz
	gzip -9c man/man8/imud-mavlink.8 > $(DESTDIR)$(MANDIR)/man8/imud-mavlink.8.gz
	install -d -m 0755 $(DESTDIR)$(DOCDIR)/imud-mavlink/examples
	install -m 644 docs/imud-mavlink/README.md docs/imud-mavlink/manual.md \
	               docs/imud-mavlink/spec.md $(DESTDIR)$(DOCDIR)/imud-mavlink/
	install -m 644 packaging/imud-mavlink/copyright $(DESTDIR)$(DOCDIR)/imud-mavlink/copyright
	gzip -9c packaging/imud-mavlink/changelog > $(DESTDIR)$(DOCDIR)/imud-mavlink/changelog.gz
	install -m 644 config/imud-mavlink.conf $(DESTDIR)$(DOCDIR)/imud-mavlink/examples/imud-mavlink.conf
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-mavlink.  Enable with: sudo systemctl enable --now imud-mavlink"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-mavlink.conf)"

uninstall:
	@if command -v systemctl >/dev/null 2>&1; then \
	    systemctl disable --now imud 2>/dev/null || true; \
	    systemctl disable --now imud-signalk 2>/dev/null || true; \
	    systemctl disable --now imud-mqtt 2>/dev/null || true; \
	    systemctl disable --now imud-influxdb 2>/dev/null || true; \
	    systemctl disable --now imud-mavlink 2>/dev/null || true; \
	fi
	rm -f $(DESTDIR)$(PREFIX)/bin/imud \
	      $(DESTDIR)$(PREFIX)/bin/imud-cal \
	      $(DESTDIR)$(PREFIX)/bin/imud-status \
	      $(DESTDIR)$(PREFIX)/bin/imud-mon \
	      $(DESTDIR)$(PREFIX)/bin/imud-signalk \
	      $(DESTDIR)$(PREFIX)/bin/imud-mqtt \
	      $(DESTDIR)$(PREFIX)/bin/imud-influxdb \
	      $(DESTDIR)$(PREFIX)/bin/imud-mavlink \
	      $(DESTDIR)$(PREFIX)/include/imud_client.h \
	      $(DESTDIR)$(PREFIX)/share/imud/imud_client.py \
	      $(DESTDIR)$(SVCDIR)/imud.service \
	      $(DESTDIR)$(SVCDIR)/imud-signalk.service \
	      $(DESTDIR)$(SVCDIR)/imud-mqtt.service \
	      $(DESTDIR)$(SVCDIR)/imud-influxdb.service \
	      $(DESTDIR)$(SVCDIR)/imud-mavlink.service \
	      $(DESTDIR)$(MANDIR)/man1/imud-status.1.gz \
	      $(DESTDIR)$(MANDIR)/man1/imud-mon.1.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-cal.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-signalk.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-mqtt.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-influxdb.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-mavlink.8.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-signalk.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-mqtt.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-influxdb.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud-mavlink.conf.5.gz
	rm -rf $(DESTDIR)$(DOCDIR)/imud $(DESTDIR)$(DOCDIR)/imud-signalk \
	       $(DESTDIR)$(DOCDIR)/imud-mqtt $(DESTDIR)$(DOCDIR)/imud-influxdb \
	       $(DESTDIR)$(DOCDIR)/imud-mavlink
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Config and calibration in $(ETCDIR) were NOT removed — delete manually if no longer needed."

# ── Clean ─────────────────────────────────────────────────────────────────────

clean:
	rm -f src/*.o src/drivers/*.o src/*.d src/drivers/*.d \
	      imud imud-cal imud-status imud-mon imud-signalk imud-mqtt imud-influxdb imud-mavlink \
	      test_fusion test_config test_nmea test_packet test_ring test_mount \
	      test_cal test_cal_math test_wmm test_position test_client test_stream \
	      test_log test_signalk test_mqtt test_influxdb test_mavlink
