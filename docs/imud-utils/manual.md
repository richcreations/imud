# imud-utils manual

Install and run imud's diagnostic tools. Currently `imud-mon`. For what the
display shows field by field, see [spec.md](spec.md); for the options see
`man 1 imud-mon`.

## Install

```sh
sudo apt install imud-utils
```

From source: `sudo make install-utils` (imud-mon is not part of `make all`'s
default install; it builds and installs on its own).

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

## See also

- `man 1 imud-mon`, `man 8 imud`, `man 5 imud.conf`
- [libimud](../libimud/README.md) — the ABI-stable path for tools that must
  survive wire revisions
