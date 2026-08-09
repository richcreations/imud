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

    return rep.finish(f"{total} outputs across {len(BRIDGES)} bridges")


if __name__ == "__main__":
    sys.exit(main())
