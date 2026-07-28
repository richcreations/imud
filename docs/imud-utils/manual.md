# imud-utils manual

Install and run imud's operator tools: `imud-mon`, which watches a running
daemon, and `imud-imutest`, which validates a sensor driver against real
hardware. For what the monitor display shows field by field and what each
driver check asserts, see [spec.md](spec.md); for options see
`man 1 imud-mon` and `man 8 imud-imutest`.

## Install

```sh
sudo apt install imud-utils
```

From source: `sudo make install-utils`. Neither tool is part of the core
package's install; they build and install on their own.

`imud-mon` is **wire-pinned** — it decodes the binary packet directly rather
than through libimud, so keep `imud-utils` at the same release as the daemon
it watches.

## Run

```sh
imud-mon                              # both streams
imud-mon nmea                         # NMEA only
imud-mon binary                       # binary only
imud-mon --config config/sim.conf     # non-default config (e.g. sim mode)
```

The display refreshes once per second. Ctrl-C to stop.

`imud-mon` needs no config of its own: it reads `imud.conf` purely to learn
which UDP ports and addresses to listen on (`--config PATH`, else
`/etc/imud/imud.conf`, then `~/.config/imud/imud.conf`).

## Watching a remote daemon

`imud-mon` listens to imud's UDP streams, so it does not need to run on the
Pi. Point it at a config carrying the same ports/addresses the daemon
broadcasts to, and run it from any machine on the network. For a multicast
high-rate stream the machine must be able to join the group.

If nothing appears:

- Confirm the streams are actually enabled on the daemon — `[nmea] enabled`
  and `[highrate] enabled` in `imud.conf`. Both are separate from the local
  `[stream]` socket that the bridges use.
- Check the ports and addresses in the config you passed match the daemon's.
- `imud-status` (on the daemon's own host) reports whether each output is
  enabled and whether the filter is producing data at all.

# Validating a driver — `imud-imutest`

`imud-imutest` drives a registered IMU and magnetometer through the whole
driver contract on real silicon and writes a Markdown report. It works with
any registered driver, not just the one in your config — use the device
overrides to point it at whatever is wired up.

**Stop the daemon first.** Both processes open the same I²C device and, worse,
both drain the same FIFO, so each would see about half the samples: the
measured rate reads low and the sequence-gap check fails for a reason that has
nothing to do with the driver. The tool refuses to start if it can reach the
daemon's status socket; `--force` overrides that and marks the report.

```sh
sudo systemctl stop imud
imud-imutest --imu-driver mpu9250 --mag-driver ak8963 --mag-addr 0x0C --all
sudo systemctl start imud
```

## The four phases

**Passive** needs no operator action: chip identification, reset timing,
control-register readback, measured output rate, FIFO depth and deliberate
overflow, sequence-number continuity, the error-return contract, noise floor,
gravity magnitude, temperature, hardware-timestamp rate and wrap handling,
interrupt edges, and a sweep that re-initialises the part at every advertised
full scale.

The three guided phases are where the axis conventions actually get proved,
and no mock can substitute for them:

- **Six-face** — you place the board on each of six faces. The rule is that
  the axis pointing *down* reads −g, in the NED board frame (X forward,
  Y starboard, Z down). Flat and component-side up must read
  `[0, 0, −9.807]`.
- **Gyro rotation** — you turn the board about each axis in turn; the sign
  must follow the right-hand rule.
- **Magnetometer spin** — two slow level circles. The substantive check is
  that the magnetometer's heading turns the same way, by the same amount, as
  the gyro's Z integral. If it does not, a mag axis is inverted or swapped
  relative to the IMU.

Answer a prompt with `s` to skip that item; skipped checks are reported as
SKIP rather than silently omitted, and they suppress the recommendation.

## Reading the report

Every threshold is printed inline, so you never need the source to know what a
check asserted. `FAIL` means the driver broke the contract on evidence the
bench cannot explain away. `WARN` means out of band but with a plausible
physical cause — an unlevel surface, a moving board, magnetic clutter, a busy
machine — and never blocks clearing the experimental flag; it asks you to read
the number. `SKIP` means the capability is absent or the phase did not run.

The report also states two limits explicitly: the control-register diff is raw
and undecoded by design, and the rule that `-1` is returned *only* on a genuine
I²C fault is proved by the in-tree unit tests, not by a hardware run.

Exit status is 0 for a clean run, 2 if anything failed, 3 for warnings only,
1 for a usage or configuration error, and 130 if you abort.

## Trying it without hardware

The `sim` driver has a real FIFO, sequence counter and timestamp, so a passive
run against it is a genuine smoke test of the tool:

```sh
imud-imutest --imu-driver sim --mag-driver sim --passive
```

The report says plainly that it exercised the tool rather than any hardware,
and never recommends clearing a flag.

## See also

- `man 1 imud-mon`, `man 8 imud-imutest`, `man 8 imud`, `man 5 imud.conf`
- [libimud](../libimud/README.md) — the ABI-stable path for tools that must
  survive wire revisions
