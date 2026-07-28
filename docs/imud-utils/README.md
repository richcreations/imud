# imud-utils

**Operator tools for imud: watch a running daemon, and validate a sensor
driver against real hardware.**

`imud-utils` holds the tools that are not part of the running system.

## `imud-mon` — watch a daemon live

A terminal monitor that listens on imud's UDP output streams and prints a
live, once-per-second display: heading, pitch, roll and rate of turn, the
full binary packet (quaternion, calibrated and raw gyro/accel/mag,
covariance, declination), and the status flags as a compact string.

It touches no hardware and needs no configuration of its own — it reads
`imud.conf` only to learn which ports and addresses to listen on. Because it
listens to the broadcast/multicast streams rather than the local socket, it
runs happily on a laptop pointed at the boat's imud.

`imud-mon` decodes the wire packet directly, so it is **wire-pinned**: keep
`imud-utils` and `imud` at the same release. (Tools that must survive wire
revisions should use libimud instead — see [libimud](../libimud/README.md).)

```sh
sudo apt install imud-utils
imud-mon                 # both streams
imud-mon binary          # binary stream only
```

## `imud-imutest` — validate a driver on real hardware

Drives a registered IMU and magnetometer driver through the whole driver
contract on real silicon and writes a Markdown report you can paste into a
bug report.

imud ships drivers for ten parts that have never run on physical hardware.
They are marked `experimental`, and the daemon warns at startup when one is
selected. Their register maps are verified against the datasheet and their
encoding and decoding are unit-tested against a mock I²C bus — but none of
that says anything about timing, real output rate, FIFO or interrupt
behaviour, or whether the chip-to-board axis remap is right. `imud-imutest`
covers exactly that gap, and a clean report is what clears the flag.

Unlike `imud-mon`, it talks to the sensor over I²C, so it runs on the machine
the sensor is attached to, with the daemon stopped.

```sh
sudo systemctl stop imud
imud-imutest --imu-driver icm20948 --mag-driver ak09916 --mag-addr 0x0C --all
sudo systemctl start imud
```

It runs four phases: a fully automatic pass, then three guided ones that
prompt you to place and turn the board — which is the only way to prove the
axis conventions are correct.

If you have one of the experimental parts, running this and opening an issue
with the report is the single most useful thing you can contribute.

## Documentation

- [manual.md](manual.md) — install, run, read the output
- [spec.md](spec.md) — what each stream section shows, the flag letters, and
  what each driver check asserts
- `man 1 imud-mon`, `man 8 imud-imutest` — options and examples
