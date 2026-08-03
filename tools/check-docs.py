#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-docs.py — every config key imud parses must reach every doc surface.

Audit D1 and D3 were the same bug twice: a key that parses, works, and is
documented in its man page, but never made it into the shipped .conf
template or the bridge manual. Nothing checked, so nothing complained.

This walks src/config.c's apply_kv() — `case SEC_*:` labels delimit the
sections, `strcmp(key, "...")` names the keys — and asserts each key appears
on all three surfaces:

  1. a `key =` line (commented out is fine; the templates present optional
     keys that way) under the matching [section] of the matching
     config/*.conf;
  2. the matching man/man5/*.5;
  3. docs/manual.md for core sections, docs/imud-<name>/manual.md for bridges.

Then the reverse: every *active* (uncommented) `key =` line in a shipped
config must have a parser, so a template outliving its key fails too.

Run as `make check-docs`. Adding a config key without documenting it should
fail here rather than reaching an operator.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Bridge sections keep their keys in their own conf/man/docs tree.
BRIDGE = {
    "SEC_SIGNALK": "imud-signalk",
    "SEC_MQTT":    "imud-mqtt",
    "SEC_INFLUX":  "imud-influxdb",
    "SEC_PROM":    "imud-prometheus",
    "SEC_MAVLINK": "imud-mavlink",
}

# Core sections: SEC_* -> the [name] it appears under in config/imud.conf.
CORE = {
    "SEC_MOUNT":       "mount",
    "SEC_DEVICE":      "device",
    "SEC_CAPTURE":     "capture",
    "SEC_IMU":         "imu",
    "SEC_MAG":         "mag",
    "SEC_FUSION":      "fusion",
    "SEC_CALIBRATION": "calibration",
    "SEC_NMEA":        "nmea",
    "SEC_HIGHRATE":    "highrate",
    "SEC_STREAM":      "stream",
    "SEC_LOGGING":     "logging",
    "SEC_POSITION":    "position",
}

# Keys deliberately absent from a surface, with the reason. Keep this list
# short and argued — it is the record of every exception.
ALLOW = {
    # Deprecated in favour of udp_enabled/http_enabled. Explained in a comment
    # in the template but intentionally not offered as a settable line: a new
    # install should never start out using it.
    ("SEC_INFLUX", "transport", "conf"),
}


def parsed_keys(path):
    """section -> [key], read from apply_kv's case labels and strcmp chain."""
    src = open(path, encoding="utf-8").read()
    try:
        body = src[src.index("static int apply_kv"):]
    except ValueError:
        sys.exit(f"{path}: cannot find apply_kv() — has the parser been renamed?")

    out, cur = {}, None
    for line in body.splitlines():
        m = re.search(r"case (SEC_\w+):", line)
        if m:
            cur = m.group(1)
        for key in re.findall(r'strcmp\(key,\s*"([^"]+)"\)', line):
            if cur:
                out.setdefault(cur, []).append(key)
    return out


def conf_section(text, name):
    """The body of [name] in a .conf, up to the next section header."""
    m = re.search(r"^\[" + re.escape(name) + r"\]\s*$", text, re.M)
    if not m:
        return None
    rest = text[m.end():]
    nxt = re.search(r"^\[[a-z0-9_-]+\]\s*$", rest, re.M)
    return rest[:nxt.start()] if nxt else rest


def read(rel):
    path = os.path.join(ROOT, rel)
    if not os.path.exists(path):
        return None
    return open(path, encoding="utf-8").read()


def main():
    keys = parsed_keys(os.path.join(ROOT, "src", "config.c"))
    if not keys:
        sys.exit("no config keys found — the extractor is not reading config.c")

    failures = []
    checked = 0

    for sec, klist in sorted(keys.items()):
        bridge = BRIDGE.get(sec)
        if bridge:
            conf_rel, sec_name = f"config/{bridge}.conf", bridge
            man_rel = f"man/man5/{bridge}.conf.5"
            doc_rel = f"docs/{bridge}/manual.md"
        elif sec in CORE:
            conf_rel, sec_name = "config/imud.conf", CORE[sec]
            man_rel = "man/man5/imud.conf.5"
            doc_rel = "docs/manual.md"
        else:
            failures.append(f"{sec}: unknown section — add it to CORE or BRIDGE")
            continue

        surfaces = {"conf": conf_rel, "man": man_rel, "doc": doc_rel}
        texts = {}
        for kind, rel in surfaces.items():
            texts[kind] = read(rel)
            if texts[kind] is None:
                failures.append(f"{sec}: missing file {rel}")

        block = conf_section(texts["conf"], sec_name) if texts["conf"] else None
        if texts["conf"] is not None and block is None:
            failures.append(f"{sec}: no [{sec_name}] section in {conf_rel}")

        for key in klist:
            word = re.compile(r"\b" + re.escape(key) + r"\b")

            if block is not None and ("conf" not in [a[2] for a in ALLOW
                                                     if a[0] == sec and a[1] == key]):
                checked += 1
                # Commented-out entries count: templates present optional keys
                # that way, and `#   preset = "yaw_180"` is documentation.
                if not re.search(r"^\s*#?\s*" + re.escape(key) + r"\s*=", block, re.M):
                    failures.append(
                        f"[{sec_name}] {key}: no '{key} =' line in {conf_rel}")

            for kind in ("man", "doc"):
                if texts[kind] is None:
                    continue
                if (sec, key, kind) in ALLOW:
                    continue
                checked += 1
                if not word.search(texts[kind]):
                    failures.append(
                        f"[{sec_name}] {key}: not documented in {surfaces[kind]}")

    # Reverse: an active setting in a shipped template with no parser behind it.
    everything = {k for ks in keys.values() for k in ks}
    conf_dir = os.path.join(ROOT, "config")
    for name in sorted(os.listdir(conf_dir)):
        if not name.endswith(".conf"):
            continue
        with open(os.path.join(conf_dir, name), encoding="utf-8") as fh:
            for lineno, line in enumerate(fh, 1):
                m = re.match(r"([a-z][a-z0-9_]*)\s*=", line)   # uncommented only
                if m:
                    checked += 1
                    if m.group(1) not in everything:
                        failures.append(
                            f"config/{name}:{lineno}: '{m.group(1)}' is set but "
                            f"no longer parsed by config.c")

    total = sum(len(v) for v in keys.values())
    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(f"\n{len(failures)} problem(s) across {total} config keys",
              file=sys.stderr)
        return 1

    print(f"check-docs: {total} config keys, {checked} assertions, all documented")
    return 0


if __name__ == "__main__":
    sys.exit(main())
