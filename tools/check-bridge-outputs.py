#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-bridge-outputs.py — each bridge's spec must list what its encoder emits.

Four bridges, one shape: an encoder holds the output vocabulary as string
literals, and docs/imud-<name>/spec.md restates it as a table for whoever is
writing the consumer.  Nothing compared them.  (The MQTT bridge has the same
shape but a much richer table — object ids, friendly names, gates — so it gets
its own checker; see check-mqtt-topics.py.)

  signalk     src/sk_delta.c        {"path":"navigation.headingTrue",...}
  influxdb    src/influx_line.c     key=%f pairs in the line-protocol format
  mavlink     src/mavlink_encode.c  mav_pack_<message>() per message type
  prometheus  src/prom_metrics.c    GAUGE("imud_roll_radians", "HELP", ...)

These are checked rather than generated because the vocabulary is embedded in
hand-formatted builders — a JSON template with conditionals and a nested
attitude object, one long line-protocol snprintf — and extracting a table from
them means parsing the template.  The failure that matters is an output added
to the encoder and never written down, and set comparison catches that for a
fraction of the code a generator would need.

Run as `make check-bridge-outputs`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import read, must_read, expand_shorthands, Report  # noqa: E402

BRIDGES = {
    "signalk": {
        "src": "src/sk_delta.c",
        # Escaped quotes: the literal in C reads \"path\":\"navigation.x\".
        "pattern": r'\\"path\\":\\"([A-Za-z][\w.]*)\\"',
        "what": "Signal K paths",
    },
    "influxdb": {
        "src": "src/influx_line.c",
        # Line-protocol field keys: `key=%…` inside the format string.
        "pattern": r"\b([a-z][a-z0-9_]*)=%",
        "what": "line-protocol field keys",
    },
    "mavlink": {
        "src": "src/mavlink_encode.c",
        "pattern": r"\bint mav_pack_(\w+)\(",
        "what": "MAVLink message packers",
        # The docs name messages the way MAVLink does.
        "transform": lambda s: s.upper(),
    },
    "prometheus": {
        "src": "src/prom_metrics.c",
        "pattern": r'GAUGE\(\s*"([a-z][a-z0-9_]*)"',
        "what": "exported gauge names",
        # Documented names look like metric names and nothing else does, so
        # the spec can be read back and each name checked against the code.
        # The counter is not a GAUGE — it is APPENDed with its own HELP/TYPE
        # because Prometheus types it differently — so it is added by hand.
        "documented": r"`(imud_[a-z0-9_]+)`",
        "extra": ["imud_packets_total"],
    },
}

# Emitted names a spec need not list, with the reason.
ALLOW = {
    # Influx tags, not fields: `source` identifies the writer and `seq` is the
    # packet counter used for de-duplication. Both are documented in the
    # bridge's line-protocol prose rather than as measurement fields.
    ("influxdb", "source"),
    ("influxdb", "seq"),
}


def main():
    rep = Report("check-bridge-outputs")
    total = 0

    for name, spec in sorted(BRIDGES.items()):
        src = must_read(spec["src"], f"the {name} encoder")
        found = sorted(set(re.findall(spec["pattern"], src)))
        transform = spec.get("transform", lambda s: s)
        emitted = sorted({transform(f) for f in found})
        rep.expect(emitted, f"{name} {spec['what']}")
        total += len(emitted)

        rel = f"docs/{name}/spec.md" if name.startswith("imud") else \
              f"docs/imud-{name}/spec.md"
        text = read(rel)
        if text is None:
            rep.fail(f"missing file {rel}")
            continue

        haystack = expand_shorthands(text)
        for item in emitted:
            if (name, item.lower()) in ALLOW:
                continue
            rep.check(item in haystack,
                      f"{rel}: {spec['src']} emits '{item}' but the spec does "
                      f"not document it")

        # And the reverse, where the spec's names are unambiguous enough to
        # read back.  A metric that stops being exported leaves every
        # dashboard and alert rule built from the spec silently empty — the
        # same failure as an undocumented one, from the other end.
        if "documented" in spec:
            known = set(emitted) | set(spec.get("extra", ()))
            for item in sorted(set(re.findall(spec["documented"], haystack))):
                # `imud_data_t` and friends: the spec talks about the libimud
                # types it decodes as well as the metrics it exports, and C's
                # _t suffix is what tells the two apart.  No Prometheus metric
                # name ends there — they end in a unit.
                if item.endswith("_t"):
                    continue
                rep.check(item in known,
                          f"{rel}: documents '{item}', which {spec['src']} no "
                          f"longer exports")

    return rep.finish(f"{total} outputs across {len(BRIDGES)} bridges")


if __name__ == "__main__":
    sys.exit(main())
