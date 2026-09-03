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
    "navigation/headingReferenced":
        "binary ON/OFF saying whether the heading topics are being published, "
        "not a scalar",
}


def fields():
    """[(subpath, GATE_*)] in FIELDS[] order."""
    src = must_read(SRC, "the MQTT bridge's publish table")
    m = re.search(r"\}\s*FIELDS\[\]\s*=\s*\{(.*?)\n\};", src, re.S)
    if not m:
        sys.exit(f"{SRC}: cannot find FIELDS[] — has the publish table moved?")
    return re.findall(r'\{\s*"([^"]+)".*?(GATE_\w+)\s*\}', m.group(1))


def published():
    """The scalar topic subpaths, in FIELDS[] order."""
    return [t for t, _ in fields()]


# The spec's condition column, per gate.  It is prose, so it is matched on the
# distinguishing phrase rather than compared whole — but WHICH phrase a row
# carries is a fact about FIELDS[], and one nothing read until now: a topic
# moved from GATE_ALWAYS to GATE_WAVE still appeared in the table, still
# appeared in the config template, and still said "always".
GATE_PHRASE = {
    "GATE_ALWAYS": "always",
    "GATE_DECL":   "declination known",
    "GATE_HEAVE":  "settled",
    "GATE_WAVE":   "sea state settled",
    "GATE_MAG":    "magnetometer fused",
}


def check_gates(rep, rel, text):
    """Each documented topic's condition must match its gate in FIELDS[]."""
    rows = {}
    for line in text.split("\n"):
        m = re.match(r"^\|\s*(.+?)\s*\|.*\|\s*(.+?)\s*\|\s*$", line)
        if m and "imud/" in m.group(1):
            for topic in re.findall(r"`imud/([^`]+)`", expand_shorthands(m.group(1))):
                rows[topic] = m.group(2)
    rep.expect(rows, f"{rel} topic table rows")

    for topic, gate in fields():
        cond = rows.get(topic)
        if cond is None:
            continue                      # absence is reported by main()
        want = GATE_PHRASE.get(gate)
        if want is None:
            rep.fail(f"{SRC}: '{topic}' has gate {gate}, which this checker "
                     f"has no documented condition for — add it to GATE_PHRASE")
            continue
        # GATE_HEAVE's phrase is a substring of GATE_WAVE's, so a heave topic
        # must NOT read as a wave one.
        ok = want in cond and not (gate == "GATE_HEAVE"
                                   and GATE_PHRASE["GATE_WAVE"] in cond)
        rep.check(ok,
                  f"{rel}: '{topic}' is published under {gate}, but the table "
                  f"gives its condition as '{cond}'")


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

        # And the reverse.  A topic that stops being published leaves a
        # subscriber waiting forever on a path the documentation still
        # promises — the same failure as an undocumented one, read from the
        # other end, and nothing checked it until now.
        for topic in sorted(set(re.findall(r"`imud/([a-zA-Z/]+)`", haystack))):
            rep.check(topic in expected,
                      f"{rel}: documents 'imud/{topic}', which {SRC} no "
                      f"longer publishes")

    check_gates(rep, "docs/imud-mqtt/spec.md",
                read("docs/imud-mqtt/spec.md") or "")

    return rep.finish(f"{len(subs)} scalar topics + {len(EXTRA)} others "
                      f"across {len(SURFACES)} surfaces")


if __name__ == "__main__":
    sys.exit(main())
