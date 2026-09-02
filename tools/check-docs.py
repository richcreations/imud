#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-docs.py — every config key imud parses must reach every doc surface,
and docs/config-keys.toml must describe the parser that actually exists.

The same bug turned up twice: a key that parses, works, and is
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

Since the registry landed, four more things are checkable, and they are what
keeps it honest — a registry that has quietly stopped describing the parser
generates confident, wrong documentation:

  4. registry keys == apply_kv keys, per section, in both directions;
  5. each key's `fields` and `macros` are what apply_kv really applies, and
     each field really exists in imud_config_t;
  6. a default restated OUTSIDE a generated region agrees with the registry
     (the generated regions are proven equal by gen-config-docs.py; a
     hand-written restatement elsewhere is the only place one can still
     drift, and one had — imud-prometheus.conf.5 gave [logging] level a
     default of info against the code's warn);
  7. the [hot]/[restart] markers in imud.conf.5 partition the core keys the
     same way config_apply_hot() does.  Nothing has ever checked this, and a
     key documented [hot] that the function does not copy is silently
     ignored on every SIGHUP.

Run as `make check-docs`. Adding a config key without documenting it should
fail here rather than reaching an operator.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import ROOT, read as _read, must_read            # noqa: E402

try:
    import tomllib
except ModuleNotFoundError:                                     # pragma: no cover
    sys.exit("docs/config-keys.toml needs python3.11+ for tomllib")

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
    "SEC_RUNTIME":     "runtime",
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


def apply_kv_body(path):
    src = open(path, encoding="utf-8").read()
    try:
        return src[src.index("static int apply_kv"):]
    except ValueError:
        sys.exit(f"{path}: cannot find apply_kv() — has the parser been renamed?")


def parsed_keys(path):
    """section -> [key], read from apply_kv's case labels and strcmp chain."""
    out, cur = {}, None
    for line in apply_kv_body(path).splitlines():
        m = re.search(r"case (SEC_\w+):", line)
        if m:
            cur = m.group(1)
        for key in re.findall(r'strcmp\(key,\s*"([^"]+)"\)', line):
            if cur:
                out.setdefault(cur, []).append(key)
    return out


def parsed_fields(path):
    """(section, key) -> (macro, field), read from the same strcmp chain.

    The macro follows the strcmp on the same line for every key but [mount]'s
    three, which are hand-rolled blocks setting several members at once; those
    simply do not appear here, and the registry spells them '' to match.
    """
    out, cur, pending = {}, None, []
    for line in apply_kv_body(path).splitlines():
        m = re.search(r"case (SEC_\w+):", line)
        if m:
            cur, pending = m.group(1), []
            continue
        keys = re.findall(r'strcmp\(key,\s*"([^"]+)"\)', line)
        macros = re.findall(r"\b(NEED_\w+)\s*\(\s*cfg->([A-Za-z0-9_]+)", line)
        if keys:
            pending = keys
        if macros and pending:
            for key in pending:
                out[(cur, key)] = macros[0]
            pending = []
    return out


def struct_members(path):
    """Every member name declared in imud_config_t."""
    text = open(path, encoding="utf-8").read()
    m = re.search(r"typedef struct\s*\{(.*?)\}\s*imud_config_t\s*;", text, re.S)
    if not m:
        sys.exit(f"{path}: cannot find imud_config_t")
    # Comments first: every member carries one, and a prose semicolon inside
    # would enter the set as a member that does not exist — turning "the field
    # is declared" into a check that can pass for a field that is not.
    body = re.sub(r"/\*.*?\*/", " ", m.group(1), flags=re.S)
    return set(re.findall(r"\b([a-z_][a-z0-9_]*)\s*(?:\[[^\]]*\])?\s*;", body))


def hot_fields(path):
    """The imud_config_t members config_apply_hot() copies."""
    src = open(path, encoding="utf-8").read()
    try:
        body = src[src.index("void config_apply_hot"):]
    except ValueError:
        sys.exit(f"{path}: cannot find config_apply_hot()")
    return set(re.findall(r"dst->([A-Za-z0-9_]+)", body[:body.index("\n}\n")]))


# ── The registry ─────────────────────────────────────────────────────────────

REGISTRY = "docs/config-keys.toml"

# config_apply_hot() copies these, and no config key writes them: they are
# RESULTS, recomputed from the freshly parsed config before the call. Listing
# them here rather than loosening the check keeps the reason on the record.
DERIVED_HOT = {
    "pos_declination_valid": "computed from declination_deg or a live source",
    "pos_mref_h_gauss":      "WMM field reference, recomputed from lat/lon",
    "pos_mref_z_gauss":      "WMM field reference, recomputed from lat/lon",
    "pos_mref_valid":        "set with the pos_mref_* pair above",
}

# Documented [hot] but deliberately not copied. main.c runs the WMM
# recomputation over the freshly parsed config and copies only the derived
# results, so the live pos_lat_deg / pos_lon_deg have no reader — a reload
# does honour them, just not by copying them. See config.c's comment in
# config_apply_hot() and test_apply_hot_partition.
HOT_NOT_COPIED = {("position", "lat_deg"), ("position", "lon_deg")}


def load_registry():
    with open(os.path.join(ROOT, REGISTRY), "rb") as fh:
        return tomllib.load(fh)


def check_registry(reg, keys, failures):
    """Registry vs apply_kv: same keys, same fields, same macros."""
    sec_of = {name: sec for sec, name in
              list(CORE.items()) + list(BRIDGE.items())}
    fields = parsed_fields(os.path.join(ROOT, "src", "config.c"))
    members = struct_members(os.path.join(ROOT, "include", "config.h"))
    checked = 0

    for section in reg["section"]:
        name = section["name"]
        sec = sec_of.get(name)
        if not sec:
            failures.append(f"{REGISTRY}: section [{name}] matches no SEC_* "
                            f"in apply_kv()")
            continue

        listed = [n for k in section["key"] for n in k["names"]]
        parsed = keys.get(sec, [])
        checked += 2
        for key in sorted(set(parsed) - set(listed)):
            failures.append(f"[{name}] {key}: parsed by apply_kv() but absent "
                            f"from {REGISTRY} — it documents itself nowhere")
        for key in sorted(set(listed) - set(parsed)):
            failures.append(f"[{name}] {key}: in {REGISTRY} but apply_kv() no "
                            f"longer parses it — the docs offer a dead key")

        for entry in section["key"]:
            for key, field, macro in zip(entry["names"], entry["fields"],
                                         entry["macros"]):
                got = fields.get((sec, key))
                checked += 1
                if got is None:
                    # Hand-rolled in apply_kv(); the registry says so with ''.
                    if field or macro:
                        failures.append(
                            f"[{name}] {key}: {REGISTRY} claims "
                            f"{macro}(cfg->{field}), but apply_kv() applies no "
                            f"NEED_* macro to it")
                    continue
                if (macro, field) != got:
                    failures.append(
                        f"[{name}] {key}: {REGISTRY} says "
                        f"{macro or '(none)'}(cfg->{field or '(none)'}), "
                        f"apply_kv() applies {got[0]}(cfg->{got[1]})")
                if field:
                    checked += 1
                    if field not in members:
                        failures.append(
                            f"[{name}] {key}: {REGISTRY} names field "
                            f"'{field}', which is not a member of "
                            f"imud_config_t")
    return checked


SECTION_HEAD = re.compile(r'^\.S[SH]\s+"?(?:SECTION\s+)?\[([a-z0-9_-]+)\]"?\s*$')
DEFAULT_LINE = re.compile(r"^\.RI\s*\(.*?(default:.*?)\s*\)")
NAME_LINE = re.compile(r"^\.BR?\s+(.+)$")


def man_sections(text, implicit):
    """Walk a man5 page yielding (section, key names, line), tracking whether
    the line sits inside a generated region.

    `implicit` is the section a bridge page documents without ever naming it
    in a heading; imud.conf.5 names each with .SS [name] and passes None.
    """
    sec, names, generated = implicit, [], False
    for line in text.split("\n"):
        if line.startswith('.\\" BEGIN GENERATED'):
            generated = True
            continue
        if line.startswith('.\\" END GENERATED'):
            generated = False
            continue
        m = SECTION_HEAD.match(line)
        if m:
            sec, names = m.group(1), []
            continue
        m = NAME_LINE.match(line)
        if m:
            names = [n.strip().strip(",").strip('"')
                     for n in re.split(r'\s*",\s*"\s*|\s+', m.group(1))]
            names = [n for n in names if re.fullmatch(r"[a-z][a-z0-9_]*", n)]
            continue
        yield sec, names, line, generated


def check_restated_defaults(reg, failures):
    """A default written outside a generated region must match the registry.

    Inside a region the generator already proves it; outside is where one can
    still rot unnoticed, and one had.
    """
    want = {(s["name"], n): k["default"]
            for s in reg["section"] for k in s["key"] for n in k["names"]}
    checked = 0
    man_dir = os.path.join(ROOT, "man", "man5")
    for base in sorted(os.listdir(man_dir)):
        if not base.endswith(".conf.5"):
            continue
        implicit = base[:-len(".conf.5")] if base.startswith("imud-") else None
        text = open(os.path.join(man_dir, base), encoding="utf-8").read()
        for sec, names, line, generated in man_sections(text, implicit):
            if generated:
                continue
            m = DEFAULT_LINE.match(line)
            if not m or not names:
                continue
            for key in names:
                if (sec, key) not in want:
                    continue
                checked += 1
                if m.group(1) != want[(sec, key)]:
                    failures.append(
                        f"man/man5/{base}: [{sec}] {key} restates its default "
                        f"as '{m.group(1)}', {REGISTRY} says "
                        f"'{want[(sec, key)]}'")
    return checked


def documented_hot(text):
    """section -> set of keys the page marks [hot], from imud.conf.5.

    Two spellings, both in use:
      .B [hot]     alone   — the whole section is hot ([fusion])
      .B [hot]:    a list  — either .BR roff lines ([nmea], [stream]) or a
                             plain comma list ([position]), running until the
                             prose that explains it starts.
    """
    out, sec, mode = {}, None, None
    for line in text.split("\n"):
        m = SECTION_HEAD.match(line)
        if m:
            sec, mode = m.group(1), None
            continue
        if re.match(r"^\.B\s+\[hot\]\s*$", line):
            out[sec] = None                       # None = every key in it
            continue
        if re.match(r"^\.B\s+\[hot\]:\s*$", line):
            mode = out.setdefault(sec, set())
            continue
        if mode is None:
            continue
        # The list ends where its explanation begins.
        if (line.startswith(".B [restart]") or line.startswith(".PP")
                or line.startswith(".TP") or line.startswith('.\\"')
                or not line.strip()):
            mode = None
            continue
        mode.update(re.findall(r"[a-z][a-z0-9_]*", line.split("(")[0]
                               .replace(".BR", " ").replace(".B ", " ")))
        if line.split("(")[0].rstrip().endswith("."):
            mode = None
    return out


def check_hot_partition(reg, failures):
    """imud.conf.5's [hot] markers vs what config_apply_hot() copies."""
    copied = hot_fields(os.path.join(ROOT, "src", "config.c"))
    by_field = {}
    for section in reg["section"]:
        for entry in section["key"]:
            for key, field in zip(entry["names"], entry["fields"]):
                if field:
                    by_field.setdefault(field, []).append((section["name"], key))

    checked = 0
    for field in sorted(copied):
        checked += 1
        if field not in by_field and field not in DERIVED_HOT:
            failures.append(
                f"config_apply_hot() copies '{field}', which no config key "
                f"writes and DERIVED_HOT does not explain")

    claimed = documented_hot(must_read("man/man5/imud.conf.5"))
    for section in reg["section"]:
        name = section["name"]
        if name not in CORE.values():
            continue                # bridges reload whole-file; see bridge.c
        keys = [n for k in section["key"] for n in k["names"]]
        hot = {n for k in section["key"]
               for n, f in zip(k["names"], k["fields"]) if f in copied}
        want = claimed.get(name)
        want = set(keys) if want is None and name in claimed else want
        if want is None:
            want = set()
        want = {k for k in want if k in keys}
        checked += 1
        for key in sorted(hot - want - {k for _, k in HOT_NOT_COPIED}):
            failures.append(
                f"[{name}] {key}: config_apply_hot() copies it, but "
                f"man/man5/imud.conf.5 does not mark it [hot]")
        for key in sorted(want - hot):
            if (name, key) in HOT_NOT_COPIED:
                continue
            failures.append(
                f"[{name}] {key}: man/man5/imud.conf.5 marks it [hot], but "
                f"config_apply_hot() does not copy it — every SIGHUP silently "
                f"ignores it")
    return checked


def conf_section(text, name):
    """The body of [name] in a .conf, up to the next section header."""
    m = re.search(r"^\[" + re.escape(name) + r"\]\s*$", text, re.M)
    if not m:
        return None
    rest = text[m.end():]
    nxt = re.search(r"^\[[a-z0-9_-]+\]\s*$", rest, re.M)
    return rest[:nxt.start()] if nxt else rest


read = _read            # checklib's, so $IMUD_ROOT aims this at a fixture tree


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

    reg = load_registry()
    checked += check_registry(reg, keys, failures)
    checked += check_restated_defaults(reg, failures)
    checked += check_hot_partition(reg, failures)

    total = sum(len(v) for v in keys.values())
    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(f"\n{len(failures)} problem(s) across {total} config keys",
              file=sys.stderr)
        return 1

    print(f"check-docs: {total} config keys, {checked} assertions, all "
          f"documented and all matching the registry")
    return 0


if __name__ == "__main__":
    sys.exit(main())
