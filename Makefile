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
              src/drivers/sim.c

# Full daemon: every module
IMUD_SRCS   = src/cal.c \
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
              src/cal_math.c \
              src/config.c \
              src/drivers.c \
              $(DRIVER_SRCS)

IMUD_OBJS   = $(IMUD_SRCS:.c=.o)
CAL_OBJS    = $(CAL_SRCS:.c=.o)

.PHONY: all clean test install uninstall

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
imud-mon: src/config.o src/mon_main.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

# ── Compilation rules ─────────────────────────────────────────────────────────

src/%.o: src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

src/drivers/%.o: src/drivers/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

# ── Tests ─────────────────────────────────────────────────────────────────────

test_fusion: src/fusion.c test/test_fusion.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_config: src/config.c test/test_config.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_nmea: src/nmea.c test/test_nmea.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_packet: src/packet.c test/test_packet.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_ring: src/ring.c test/test_ring.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_mount: src/config.c test/test_mount.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_cal: src/cal.c test/test_cal.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_cal_math: src/cal_math.c test/test_cal_math.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_wmm: src/wmm.c test/test_wmm.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test_position: src/position.c src/wmm.c test/test_position.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test: test_fusion test_config test_nmea test_packet test_ring test_mount \
      test_cal test_cal_math test_wmm test_position
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

# ── Install ───────────────────────────────────────────────────────────────────

PREFIX  ?= /usr/local
ETCDIR  ?= /etc/imud
SVCDIR  ?= /etc/systemd/system

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
	install -m 644 etc/imud.service $(DESTDIR)$(SVCDIR)/imud.service
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	    echo ""; \
	    echo "Next steps:"; \
	    echo "  sudo systemctl enable --now imud"; \
	    echo "  review $(ETCDIR)/imud.conf  (i2c_bus, gpio_chip, rotation_euler_deg)"; \
	fi

uninstall:
	@if command -v systemctl >/dev/null 2>&1; then \
	    systemctl disable --now imud 2>/dev/null || true; \
	fi
	rm -f $(DESTDIR)$(PREFIX)/bin/imud \
	      $(DESTDIR)$(PREFIX)/bin/imud-cal \
	      $(DESTDIR)$(PREFIX)/bin/imud-status \
	      $(DESTDIR)$(PREFIX)/bin/imud-mon \
	      $(DESTDIR)$(SVCDIR)/imud.service
	@if [ -z "$(DESTDIR)" ] && command -v systemctl >/dev/null 2>&1; then \
	    systemctl daemon-reload; \
	fi
	@echo "Config and calibration in $(ETCDIR) were NOT removed — delete manually if no longer needed."

# ── Clean ─────────────────────────────────────────────────────────────────────

clean:
	rm -f src/*.o src/drivers/*.o \
	      imud imud-cal imud-status imud-mon \
	      test_fusion test_config test_nmea test_packet test_ring test_mount \
	      test_cal test_cal_math test_wmm test_position
