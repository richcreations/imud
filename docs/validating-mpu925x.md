# Validating an MPU-9250 / MPU-9255 on your bench

The `mpu9250`, `mpu9255` and `ak8963` drivers are marked **experimental**: their
register maps were checked against the datasheets and their encode/decode paths
are covered by unit tests against a mock I²C bus, but they have never run on
physical silicon. Nothing in a mock can tell us whether the real chip hits its
output rate, whether the FIFO and interrupt behave, or — the one that matters
most — whether the chip-to-board axis remap is right.

`imud-imutest` closes that gap in about ten minutes. It exercises the whole
driver contract against your hardware, walks you through the three physical
checks a mock cannot do, and writes a Markdown report. **Post that report to a
GitHub issue and the experimental flag comes off for everyone.**

You do not need to know anything about the driver internals. You need the
board, a Pi, six jumper wires, and a table that isn't made of steel.

**Everything below happens on the Raspberry Pi itself** — at its own desktop,
or over SSH into it. `imud-imutest` reaches the sensor over I²C, so it can only
run on the machine the board is wired to. The one step you may prefer to do
elsewhere is posting the report at the end, and [§9](#9-post-the-report) covers
both ways.

- [1. What you need](#1-what-you-need)
- [2. Wiring](#2-wiring)
- [3. Enable I2C at 400 kHz](#3-enable-i2c-at-400-khz)
- [4. Install imud](#4-install-imud)
- [5. Which chip do you actually have?](#5-which-chip-do-you-actually-have)
- [6. Configure imud](#6-configure-imud)
- [7. Run the test](#7-run-the-test)
- [8. Read the result](#8-read-the-result)
- [9. Post the report](#9-post-the-report)
- [10. Troubleshooting](#10-troubleshooting)

---

## 1. What you need

- A Raspberry Pi (any model with the 40-pin header) running Raspberry Pi OS or
  Debian, arm64 or armhf.
- The MPU-9250 or MPU-9255 breakout, and six jumper wires. Longer is better —
  30 cm or so lets you turn the board well away from the Pi during the
  magnetometer spin.
- Something to mount the board to. Two of the test phases ask you to stand the
  board on each of its four edges and to turn it by hand; a scrap of wood or a
  small cardboard box with the board taped to it makes this far easier than
  balancing a bare PCB on edge.
- A level, solid, **non-magnetic** work surface. A wooden table is ideal. A
  steel bench, a desk with a steel frame, or anywhere near speakers, motors, or
  a switching power supply will produce a magnetically cluttered spin phase.
- About 10 minutes of hands-on time.

> **A warning about the parts.** Boards sold as MPU-9250 are very often
> relabelled **MPU-6500s**, which have no magnetometer at all. `probe()`
> detects this and says so by name, so if that is what you have, you will find
> out in the first second of the run rather than after ten minutes of turning
> a board over. Please report it anyway — see
> [§10](#10-troubleshooting).

---

## 2. Wiring

Six wires. Physical pin numbers are the ones printed on a Pi pinout diagram;
BCM numbers are what goes in the config file.

| Breakout pin | Pi header pin | Pi signal | Notes |
| --- | --- | --- | --- |
| `VCC` / `3V3` | pin 1 | 3.3 V | See the voltage note below. |
| `GND` | pin 6 | GND | Any GND pin works; pin 6 is next to pin 1. |
| `SDA` | pin 3 | BCM 2 / SDA1 | |
| `SCL` | pin 5 | BCM 3 / SCL1 | |
| `INT` | pin 11 | BCM 17 | Data-ready interrupt. Optional — see below. |
| `AD0` / `ADO` / `SDO` | pin 6 (GND) | — | Address select: GND → `0x68`, 3.3 V → `0x69`. |

**Voltage.** The MPU-925x die is a 3.3 V part and the Pi's I²C lines are **not**
5 V tolerant. Most GY-9250 / GY-91-style breakouts carry a regulator and level
shifting and will accept 5 V on `VCC`, but if you are not certain yours does,
use 3.3 V (pin 1). It works either way, and it cannot damage anything.

**Pull-ups.** None needed. The Pi has 1.8 kΩ pull-ups on BCM 2 and 3, and most
breakouts add their own.

**`NCS`.** If your board exposes an `NCS` pin, it selects I²C when high. Nearly
all breakouts pull it high on the PCB, so leave it alone; only if the chip does
not answer at all is it worth tying `NCS` to 3.3 V.

**`FSYNC`.** Leave it unconnected or tie it to GND. The driver does not use it.

**The `INT` wire is optional but worth doing.** Without it, one check
(`imu.drdy.edges`) is skipped and the reader falls back to a polling timer. The
report is still valid and can still clear the experimental flag — it just
covers one less thing. If you skip the wire, set `int_gpio = 0` in the config.

---

## 3. Enable I2C at 400 kHz

The default bus speed is 100 kHz, which is not enough headroom for this part at
its higher rates: a sample-set is 12 bytes, so 1000 Hz is 12 kB/s before
protocol overhead, plus the magnetometer on the same bus. Set 400 kHz.

Edit the firmware config:

```sh
sudo nano /boot/firmware/config.txt     # Bookworm and later
# sudo nano /boot/config.txt            # Bullseye and earlier
```

Find the existing `dtparam=i2c_arm` line — `raspi-config` adds
`dtparam=i2c_arm=on` — and make it read:

```ini
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

If there is no such line, add it. **Edit the existing line rather than adding a
second one**; two `dtparam=i2c_arm=...` lines is the usual reason a baudrate
change appears to do nothing.

Reboot, then confirm the bus is there and the chip answers:

```sh
sudo reboot
# after it comes back:
sudo apt install -y i2c-tools
i2cdetect -y 1
```

You should see `68` (or `69` if you tied `AD0` high) in the grid.

**`0c` will not appear**, and that is expected — the AK8963 magnetometer sits
on the MPU's private auxiliary bus and only becomes visible on the host bus
once the IMU driver opens the I²C bypass during init. `imud-imutest` does that
for you.

Optionally, confirm the clock really changed:

```sh
for f in $(find /proc/device-tree -path '*i2c*' -name clock-frequency 2>/dev/null); do
    printf '%s = %s Hz\n' "$f" "$(od -An -tu4 --endian=big "$f" | tr -d ' ')"
done
```

---

## 4. Install imud

From the apt repository. The suite is read from `/etc/os-release`, so these are
the same commands on bookworm and trixie — nothing to substitute:

```sh
# 1. Trust the signing key
curl -fsSL https://richcreations.github.io/imud/apt/KEY.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/imud.gpg

# 2. Add the repository
sudo tee /etc/apt/sources.list.d/imud.sources >/dev/null <<EOF
Types: deb
URIs: https://richcreations.github.io/imud/apt
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: main
Signed-By: /usr/share/keyrings/imud.gpg
EOF

# 3. Install the daemon, the World Magnetic Model data, and the test tool
sudo apt update
sudo apt install imud imud-wmm-data imud-utils
```

`imud-imutest` ships in **`imud-utils`** — that is the package that matters
here. Installing `imud` too gives you `/etc/imud/imud.conf` to edit, the
`imud`/`gpio`/`i2c` group setup, and a daemon to run once the board is
validated.

Check it landed:

```sh
imud-imutest --version
man imud-imutest        # the full reference, if you want it
```

**Permissions.** `imud-imutest` talks to `/dev/i2c-1` and to the GPIO chip
directly. On Raspberry Pi OS the default user is already in the `i2c` and
`gpio` groups, so **no `sudo` is needed** — and running without it keeps the
report file owned by you. If you get "permission denied", add yourself and log
out and back in:

```sh
sudo usermod -aG i2c,gpio "$USER"
```

---

## 5. Which chip do you actually have?

The MPU-9250 and MPU-9255 are the same silicon with different `WHO_AM_I`
values, and each has its own driver name. Read the register directly:

```sh
i2cget -y 1 0x68 0x75        # use 0x69 if AD0 is tied high
```

| Reads | Part | Driver name |
| --- | --- | --- |
| `0x71` | MPU-9250 | `mpu9250` |
| `0x73` | MPU-9255 | `mpu9255` |
| `0x70` | **MPU-6500** — 6-axis only, no magnetometer | *(not supported; see [§10](#10-troubleshooting))* |

If you would rather not bother: pick either driver and run the tool. Choosing
the wrong one of the two fails immediately with a message naming the one you
should have used.

---

## 6. Configure imud

Edit `/etc/imud/imud.conf`:

```sh
sudo nano /etc/imud/imud.conf
```

Replace the `[device]`, `[imu]` and `[mag]` sections with these values. Leave
every other section at its shipped default — `imud-imutest` reads the driver
directly and none of the fusion, output, or mount settings affect it.

```ini
[device]
i2c_bus        = "/dev/i2c-1"
gpio_chip      = "gpiochip0"     # Pi 4 / 3 / Zero. Pi 5: usually "gpiochip4" — see below.

[imu]
driver         = "mpu9250"       # or "mpu9255" — whichever §5 told you
i2c_addr       = 0x68            # 0x69 if AD0 is tied high
int_gpio       = 17              # BCM 17 = header pin 11; use 0 if you skipped the INT wire
odr_hz         = 200             # on this chip's 1000/(1+SMPLRT_DIV) grid
accel_g        = 8
gyro_dps       = 2000
fifo_wm        = 16              # this chip's FIFO holds only ~42 sample-sets

[mag]
driver         = "ak8963"
i2c_addr       = 0x0C            # fixed; reached through the MPU's I2C bypass
int_gpio       = 0               # no interrupt pin is available in bypass mode
odr_hz         = 100             # this part supports 8 or 100 only
set_period_s   = 0.0             # no degauss coil on an AK8963
```

Three of those values are chosen for this chip specifically, and are worth
understanding if you go on to run the daemon:

- **`odr_hz = 200`** — the MPU-925x runs at `1000 / (1 + SMPLRT_DIV)`, giving
  1000 / 500 / 333 / 250 / 200 / 125 / 100 Hz. imud's shipped default of 833 Hz
  is **not** on that grid and rounds up to 1000 Hz. 200 Hz is comfortable for
  the bench and well above the chip's 41 Hz internal filter.
- **`fifo_wm = 16`** — the FIFO is 512 bytes, or about 42 accel+gyro
  sample-sets: the smallest of any part imud supports, and smaller than imud's
  default watermark of 64. Keeping the watermark well under capacity lets the
  FIFO-depth check see the queue actually growing.
- **`gyro_dps` / `accel_g`** — only the starting point. The test sweeps every
  advertised full-scale range on its own.

**Pi 5 GPIO chip.** The chip name moved between kernel releases. Rather than
guess, ask:

```sh
sudo apt install -y gpiod
gpiodetect
```

Use the chip whose label is `pinctrl-rp1` on a Pi 5, or `pinctrl-bcm2711` on a
Pi 4. (If you set `int_gpio = 0`, none of this matters.)

**Optional, one extra check.** Setting your position enables `spin.dip`, which
verifies the magnetometer's vertical axis sign against the hemisphere you are
in. Two decimal places is plenty — this only picks a hemisphere:

```ini
[position]
lat_deg = 47.6                   # +N / -S
lon_deg = -122.3                 # +E / -W
```

---

## 7. Run the test

**Stop the daemon first.** `imud` and `imud-imutest` both open the same I²C
device and both drain the same FIFO, so each would see about half the samples:
the measured output rate would read low and the sequence-gap check would fail
for a reason that has nothing to do with the driver. The tool refuses to start
if it can reach a running daemon — please don't reach for `--force`, because a
report produced that way cannot clear the flag.

```sh
sudo systemctl stop imud
cd ~
imud-imutest --all
```

If you would rather not edit `/etc/imud/imud.conf` at all, pass everything on
the command line instead:

```sh
imud-imutest --imu-driver mpu9250 --imu-addr 0x68 \
             --mag-driver ak8963  --mag-addr 0x0C \
             --gpio-chip gpiochip0 --int-gpio 17 \
             --odr 200 --fifo-wm 16 --all
```

### What it asks of you

**First, about a minute of passive checks.** Put the board flat on the table,
component side up, and **don't touch it or bump the table** until it starts
prompting. It is measuring the noise floor and the output rate; a nudge shows
up as a warning.

**Then six orientations.** Each prompt names the position and prints the
expected reading. Put the board in position, press Enter, and hold still for
about three seconds while it samples.

Orient yourself once and the rest follows: board flat, components facing up,
the **+X arrow on the silkscreen pointing away from you**. That is the bow.
Your right-hand edge is starboard, your left is port.

| Face | What to do |
| --- | --- |
| 1 | Flat on the bench, component side up. |
| 2 | Turned over, components facing the bench. |
| 3 | Standing on its **+X (bow)** edge, so the +X arrow points at the floor. |
| 4 | Standing on its opposite edge, +X arrow pointing at the ceiling. |
| 5 | Standing on its **right-hand (starboard)** edge. |
| 6 | Standing on its **left-hand (port)** edge. |

This is the phase that catches an inverted or swapped axis — the single most
likely defect in a new driver — so it is worth being careful that the board is
really on the edge the prompt asked for. If you fumble one, press `s` then
Enter to skip it and re-run the phase afterwards with `imud-imutest --faces`.

**Then three rotations.** Here the rhythm is **Enter, turn, Enter**: press
Enter to start, make the turn by hand, then press Enter again once you have
stopped. About 90°, smooth, over two or three seconds. Each turn has a 30-second
timeout.

| Turn | What to do |
| --- | --- |
| Gyro X | **Roll to starboard** — rotate about the bow-stern axis so the right-hand edge goes down. |
| Gyro Y | **Pitch bow up** — rotate about the port-starboard axis so the +X edge rises. |
| Gyro Z | **Yaw clockwise seen from above** — keep it flat and swing the bow to the right. |

Accuracy does not matter much; a hand turn is ±10° at best, and the check
exists to catch sign errors and factor-of-57.3 unit mistakes, not to measure
sensitivity.

**Finally, the magnetometer spin.** Hold the board **level** and turn it slowly
through at least two full circles, clockwise seen from above — roughly ten
seconds per circle. A coverage bar fills in as you go and tells you when you
have the full circle; press Enter then.

Get the board away from steel, motors, speakers — and from the Pi itself and
its power supply, which are both magnetic sources. This is what the long jumper
wires are for. The phase times out after three minutes.

**Total: roughly ten minutes.** Ctrl-C aborts cleanly at any point (the report
is then marked partial).

---

## 8. Read the result

The tool prints a digest and writes the full report to the current directory:

```
imud-imutest-mpu9250-20260802-143015.md
```

Each check is graded `PASS`, `FAIL`, `WARN`, `SKIP` or `INFO`, and the report
prints every threshold inline, so nobody needs the source to know what a check
asserted. A `WARN` never blocks anything — it means a number was out of band
but has a plausible physical explanation (an unlevel table, a moving board,
magnetic clutter), and asks a human to look at it.

The exit code summarises the run:

| Code | Meaning |
| --- | --- |
| 0 | Everything passed. |
| 2 | At least one check **failed**. |
| 3 | Warnings only. |
| 1 | Usage, config or I/O error — nothing was measured. |
| 130 | You aborted it. |

The last lines of the digest tell you plainly whether the report supports
clearing the experimental flag, and if not, exactly which check stood in the
way.

**A `FAIL` is a good outcome too.** It means you found a real bug in the driver
before it reached anyone's boat, and the report contains everything needed to
fix it. Please post it.

---

## 9. Post the report

Open an issue at **<https://github.com/richcreations/imud/issues/new>**.

Suggested title:

```
imutest report: mpu9250 + ak8963 on Raspberry Pi 4 (Pi OS bookworm)
```

Attach the report file, or paste its contents into the issue body — it is
already Markdown and renders as-is. Either way is fine; see
[below](#getting-the-report-into-the-issue) for how to do it.

Alongside the report, please add:

- **The board** — vendor, model, and a link or photo if you have one. Given how
  common relabelled parts are, knowing which boards produce clean reports is
  genuinely useful to the next person.
- **`WHO_AM_I`** — the byte you read in [§5](#5-which-chip-do-you-actually-have),
  so it is on the record next to the result.
- **Anything odd during the run** — a face you had to skip, a wobbly table, a
  fan or a motor nearby. Context turns a puzzling `WARN` into an explained one.

### Getting the report into the issue

The report is sitting in your home directory on the Pi. Pick whichever of these
matches how you are working.

**If you are using the Pi with a monitor and keyboard** (Raspberry Pi OS
Desktop) — the simplest route:

1. Open **Chromium** and go to
   <https://github.com/richcreations/imud/issues/new>.
2. Open **File Manager** on your home folder and find
   `imud-imutest-mpu9250-*.md`.
3. **Drag the file straight into the issue's comment box.** GitHub attaches it
   and inserts a link.

Or, to paste it as text instead: double-click the file to open it in Text
Editor, `Ctrl+A`, `Ctrl+C`, and paste into the comment box.

**If your Pi is headless and you are on it over SSH**, either copy the file to
the machine you are sitting at and attach it from there:

```sh
# run this on the other machine, not on the Pi;
# substitute your own username and the Pi's hostname or IP:
scp you@raspberrypi.local:imud-imutest-mpu9250-\*.md .
```

…or print it and copy it straight out of the terminal window:

```sh
cat ~/imud-imutest-mpu9250-*.md
```

Select all of the output, copy, and paste into the issue. Your terminal's
scrollback needs to reach the top of it — if it doesn't, use `scp` above, or
`less` the file and copy a screen at a time.

> **One privacy note.** The report's environment section records your hostname
> and kernel version. If you would rather not publish those, edit those two
> lines out before posting — nothing else in the file identifies you or your
> machine.

Once a clean report lands, `experimental = true` comes off `mpu9250`, `mpu9255`
and `ak8963`, imud stops warning at startup for everyone using them, and your
run is credited in the release notes.

---

## 10. Troubleshooting

| Symptom | What it means |
| --- | --- |
| `imud appears to be running` | Stop it: `sudo systemctl stop imud`. Don't use `--force` — the FIFO would be shared and the timing figures meaningless. |
| `probe() failed at 0x68` | Wrong address, wrong driver, or nothing on the bus. Check `i2cdetect -y 1` shows `68` or `69`, and that `i2c_addr` matches. |
| `WHO_AM_I = 0x70 — this is an MPU-6500` | A relabelled 6-axis part with no magnetometer. Not something imud can drive as a 9-axis device. Please open an issue anyway with the board's vendor and listing — that data is useful. |
| `WHO_AM_I = 0x73, expected 0x71` | You have the other part. Switch `driver` to `mpu9255` (or `mpu9250`) as the message says. |
| `no AK8963 magnetometer found at 0x0C` | Also usually a relabelled MPU-6500. If `WHO_AM_I` really did read 0x71/0x73, that combination is unusual and worth an issue on its own. |
| `0c` never shows in `i2cdetect` | Expected. The magnetometer is only visible once the IMU driver opens the I²C bypass. |
| `imu.drdy.edges`: *no edges on BCM 17* | The `INT` wire is not connected, or is on a different pin. Fix the wiring, or set `int_gpio = 0` and re-run — the report stays valid. |
| `GPIO is held by another process` | The daemon is still running, or a previous run is stuck. `sudo systemctl stop imud`. |
| `GPIO chip not found` | Wrong `gpio_chip`. Run `gpiodetect` and use the `pinctrl-*` chip for your Pi model. |
| Permission denied on `/dev/i2c-1` | `sudo usermod -aG i2c,gpio "$USER"`, then log out and back in. |
| Lots of `WARN`s on noise or gravity | The bench moved. Use a solid, level, non-magnetic surface and don't lean on the table during the first minute. |
| `spin` warnings | Magnetic clutter. Move away from steel, motors, speakers, and the Pi itself. |

Re-running a single phase is cheap while you are diagnosing something:

```sh
imud-imutest --faces      # just the six orientations
imud-imutest --spin       # just the magnetometer
```

Those partial reports are useful for diagnosis but cannot clear the
experimental flag — that needs one complete `--all` run with no failures.

---

## Afterwards

The board is now wired and configured. Calibrate before first real use —
`imud-cal` talks to the hardware directly, so like `imud-imutest` it needs the
daemon stopped:

```sh
sudo systemctl stop imud       # it should still be stopped from the test
imud-cal gyro
imud-cal accel
imud-cal mag                   # in situ, in the vessel, not on the bench

sudo systemctl enable --now imud
imud-status
```

Two things specific to this part are worth reading before you rely on it: the
[manual's *Fitting an MPU-9250 or MPU-9255*](manual.md#fitting-an-mpu-9250-or-mpu-9255)
section, and in particular its warning **not** to copy the MPU-9250's datasheet
noise figure into `mekf_accel_noise`. That value is a tuned filter parameter,
not a per-chip transcription, and the naive conversion lands inside a region
where the filter measurably diverges.

Thank you for doing this — a bench report is the only thing that can turn an
experimental driver into a supported one.
