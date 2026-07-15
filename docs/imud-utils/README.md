# imud-utils

**Watch an imud daemon live, from any machine on the network.**

`imud-utils` holds imud's diagnostic tools. Currently that is `imud-mon`, a
terminal monitor that listens on imud's UDP output streams and prints a
live, once-per-second display: heading, pitch, roll and rate of turn, the
full binary packet (quaternion, calibrated and raw gyro/accel/mag,
covariance, declination), and the status flags as a compact string.

It holds no hardware and needs no configuration of its own — it reads
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

## Documentation

- [manual.md](manual.md) — install, run, read the display
- [spec.md](spec.md) — what each stream section shows, and the flag letters
- `man 1 imud-mon` — options and examples
