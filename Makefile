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

.PHONY: all bridges clean test install install-signalk install-mqtt uninstall

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

# Optional bridge daemons — each has its own config file, service, and man page,
# and installs via its own `install-*` target (prep for per-bridge packaging).
# Kept out of `all` so a core build / CI never needs a bridge's dependencies.
bridges: imud-signalk imud-mqtt

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

test: test_fusion test_config test_nmea test_packet test_ring test_mount \
      test_cal test_cal_math test_wmm test_position test_client test_stream \
      test_log test_signalk test_mqtt
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

# ── Install ───────────────────────────────────────────────────────────────────

PREFIX  ?= /usr/local
ETCDIR  ?= /etc/imud
SVCDIR  ?= /etc/systemd/system
MANDIR  ?= $(PREFIX)/share/man

install: imud imud-cal imud-status imud-mon
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
	@echo ""
	@echo "Next steps:"
	@echo "  sudo systemctl enable --now imud"
	@echo "  review $(ETCDIR)/imud.conf  (i2c_bus, gpio_chip, rotation_euler_deg)"

# ── Install the Signal K bridge (optional) ─────────────────────────────────────
# Run after `make bridges`.  Installs the binary, service, man page, and its own
# config file (non-clobbering).  Prep for a standalone imud-signalk package.
install-signalk: imud-signalk
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin
	install -m 755 imud-signalk $(DESTDIR)$(PREFIX)/bin/
	install -m 644 etc/imud-signalk.service $(DESTDIR)$(SVCDIR)/imud-signalk.service
	install -d -m 0755 $(DESTDIR)$(ETCDIR)
	@if [ ! -f "$(DESTDIR)$(ETCDIR)/imud-signalk.conf" ]; then \
	    install -m 644 config/imud-signalk.conf $(DESTDIR)$(ETCDIR)/imud-signalk.conf; \
	    echo "Installed config:       $(DESTDIR)$(ETCDIR)/imud-signalk.conf"; \
	else \
	    echo "Config already exists, skipping: $(DESTDIR)$(ETCDIR)/imud-signalk.conf"; \
	fi
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man8
	gzip -9c man/man8/imud-signalk.8 > $(DESTDIR)$(MANDIR)/man8/imud-signalk.8.gz
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-signalk.  Enable with: sudo systemctl enable --now imud-signalk"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-signalk.conf)"

# ── Install the MQTT bridge (optional) ─────────────────────────────────────────
# Run after `make imud-mqtt` (needs libmosquitto-dev).  Installs the binary,
# service, man page, and its own config file (non-clobbering).
install-mqtt: imud-mqtt
	install -d -m 0755 $(DESTDIR)$(PREFIX)/bin
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
	install -d -m 0755 $(DESTDIR)$(MANDIR)/man8
	gzip -9c man/man8/imud-mqtt.8 > $(DESTDIR)$(MANDIR)/man8/imud-mqtt.8.gz
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Installed imud-mqtt.  Enable with: sudo systemctl enable --now imud-mqtt"
	@echo "  (requires imud's [stream] output enabled; see $(ETCDIR)/imud-mqtt.conf)"

uninstall:
	@if command -v systemctl >/dev/null 2>&1; then \
	    systemctl disable --now imud 2>/dev/null || true; \
	    systemctl disable --now imud-signalk 2>/dev/null || true; \
	    systemctl disable --now imud-mqtt 2>/dev/null || true; \
	fi
	rm -f $(DESTDIR)$(PREFIX)/bin/imud \
	      $(DESTDIR)$(PREFIX)/bin/imud-cal \
	      $(DESTDIR)$(PREFIX)/bin/imud-status \
	      $(DESTDIR)$(PREFIX)/bin/imud-mon \
	      $(DESTDIR)$(PREFIX)/bin/imud-signalk \
	      $(DESTDIR)$(PREFIX)/bin/imud-mqtt \
	      $(DESTDIR)$(PREFIX)/include/imud_client.h \
	      $(DESTDIR)$(PREFIX)/share/imud/imud_client.py \
	      $(DESTDIR)$(SVCDIR)/imud.service \
	      $(DESTDIR)$(SVCDIR)/imud-signalk.service \
	      $(DESTDIR)$(SVCDIR)/imud-mqtt.service \
	      $(DESTDIR)$(MANDIR)/man1/imud-status.1.gz \
	      $(DESTDIR)$(MANDIR)/man1/imud-mon.1.gz \
	      $(DESTDIR)$(MANDIR)/man5/imud.conf.5.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-cal.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-signalk.8.gz \
	      $(DESTDIR)$(MANDIR)/man8/imud-mqtt.8.gz
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Config and calibration in $(ETCDIR) were NOT removed — delete manually if no longer needed."

# ── Clean ─────────────────────────────────────────────────────────────────────

clean:
	rm -f src/*.o src/drivers/*.o src/*.d src/drivers/*.d \
	      imud imud-cal imud-status imud-mon imud-signalk imud-mqtt \
	      test_fusion test_config test_nmea test_packet test_ring test_mount \
	      test_cal test_cal_math test_wmm test_position test_client test_stream \
	      test_log test_signalk test_mqtt
