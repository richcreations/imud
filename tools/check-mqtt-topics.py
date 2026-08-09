#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-mqtt-topics.py — the advertised MQTT topic tree must be the real one.

config/imud-mqtt.conf opens with a "Topics (default prefix ...)" block that
enumerates what the bridge publishes.  It listed ten topics while
mqtt_publish.c published seventeen: every wave and period topic added with sea
state (waveHeight, wavePeriod, rollPeriod, rollAmplitude, pitchPeriod,
pitchAmplitude) and engine/running were missing.  An operator reading the
shipped template to find out what to subscribe to was a release behind, and
the bridge's own spec.md had them all along.

Sources of truth in src/mqtt_publish.c:
  - FIELDS[].sub          — the scalar telemetry subpaths, one per topic
  - snprintf(...topic...) — everything outside FIELDS: engine/running, the
                            retained availability topic, and the two Home
                            Assistant discovery topics

Run as `make check-mqtt-topics`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import read, must_read, expand_shorthands, Report  # noqa: E402

SRC = "src/mqtt_publish.c"

# Surfaces that present themselves as the topic list.  Each must name every
# published subpath.
#
# man/man5/imud-mqtt.conf.5 is NOT one of them, and the distinction is worth
# stating: it is a configuration reference, and its single topic mention
# (environment/heave) is there to explain what publish_heave gates.  Demanding
# a config man page enumerate the topic tree would be demanding it become the
# bridge spec.
#
# The two Home Assistant discovery topics are excluded from the obligation
# too: they are addressed to Home Assistant rather than to a subscriber, their
# prefix is a separate setting, and they belong in the spec's discovery
# section rather than a "what can I subscribe to" list.
SURFACES = [
    "config/imud-mqtt.conf",
    "docs/imud-mqtt/spec.md",
]

# Topics whose subpath is built outside FIELDS[], with the reason they are
# not in the table.  Listing them here rather than regexing every snprintf
# keeps the checker honest about what it knows: see verify_extras().
EXTRA = {
    "engine/running":  "binary ON/OFF from FLAG_ENGINE_ON, not a scalar",
    "status/online":   "retained availability topic (LWT)",
}


def expand_shorthands(text):
    """Resolve the two ways the docs abbreviate sibling topics.

    Both exist because a topic tree written out in full does not fit in 80
    columns and reads worse when it does.  A checker that cannot follow them
    reports working documentation as missing, which is how a checker gets
    switched off — so it follows them:

      config/imud-mqtt.conf   imud/attitude/{roll,pitch,yaw}
      docs/imud-mqtt/spec.md  `imud/attitude/roll` · `/pitch` · `/yaw`

    The second form shares the leading path with the previous full topic, so
    it needs the running context rather than a local rewrite.
    """
    out = []

    # {a,b,c} alternation
    for m in re.finditer(r"([\w/.-]*)\{([^}]*)\}([\w/.-]*)", text):
        head, alts, tail = m.group(1), m.group(2), m.group(3)
        for alt in alts.split(","):
            out.append(f"{head}{alt.strip()}{tail}")

    # `full/path` · `/leaf` — the leaf inherits the previous topic's parent.
    prev = None
    for m in re.finditer(r"`([^`]+)`", text):
        token = m.group(1).strip()
        if token.startswith("/") and prev and "/" in prev:
            out.append(prev.rsplit("/", 1)[0] + token)
        elif "/" in token:
            prev = token.rstrip("/")

    return text + "\n" + "\n".join(out)


def published():
    """The scalar topic subpaths, in FIELDS[] order."""
    src = must_read(SRC, "the MQTT bridge's publish table")
    m = re.search(r"\}\s*FIELDS\[\]\s*=\s*\{(.*?)\n\};", src, re.S)
    if not m:
        sys.exit(f"{SRC}: cannot find FIELDS[] — has the publish table moved?")
    return re.findall(r'\{\s*"([^"]+)"', m.group(1))


def verify_extras(rep):
    """Every topic built outside FIELDS[] must be one this checker knows.

    A new snprintf(out[n].topic, ...) with a literal subpath is a new topic;
    if EXTRA does not name it, the surfaces are not being checked for it and
    this checker would silently under-report.
    """
    src = must_read(SRC)
    literals = set()
    for m in re.finditer(r'out\[n\]\.topic[^;]*?"%s/([^"%]+)"', src, re.S):
        literals.add(m.group(1).rstrip("/"))
    rep.expect(literals, "topic literals outside FIELDS[]")
    for lit in sorted(literals):
        rep.check(lit in EXTRA,
                  f"{SRC}: publishes '<prefix>/{lit}' but check-mqtt-topics.py "
                  f"does not know about it — add it to EXTRA with a reason, or "
                  f"it will never be checked against the docs")
    return literals


def main():
    rep = Report("check-mqtt-topics")

    subs = published()
    rep.expect(subs, "FIELDS[] subpaths")
    verify_extras(rep)

    expected = list(subs) + sorted(EXTRA)

    for rel in SURFACES:
        text = read(rel)
        if text is None:
            rep.fail(f"missing file {rel}")
            continue
        haystack = expand_shorthands(text)
        for topic in expected:
            rep.check(topic in haystack,
                      f"{rel}: publishes '{topic}' but it is not documented here")

    return rep.finish(f"{len(subs)} scalar topics + {len(EXTRA)} others "
                      f"across {len(SURFACES)} surfaces")


if __name__ == "__main__":
    sys.exit(main())
