#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
check-cli-docs.py — an option the daemon accepts must be an option it admits to.

The flag list lives in four places for imud alone (src/cli.c's usage string,
imud.8's OPTIONS, docs/manual.md §3, spec.md §12) and a fifth on the website,
with three different wordings and nothing comparing them.  web/index.html's
synopsis advertised

    imud [--config PATH] [--replay FILE] [--version]

omitting --skip-bias-cal, --no-nmea, --no-highrate and --foreground.

Three axes, because the failures are different in kind:

  1. parser -> usage text.  imud-status accepts --help and its usage string
     never mentioned it, so `imud-status --help` printed a usage message that
     did not list the flag the reader had just typed.
  2. usage text -> man page and the other reference surfaces.
  3. surfaces -> parser.  A flag documented after it was removed.

Axis 1 and 2 are what help2man makes structural in S3: once the man page is
generated from --help, they cannot disagree.  Until then, and for the surfaces
help2man does not own (manual.md, spec.md, the website), this is the gate.

Run as `make check-cli-docs`.  Pure text analysis, no build.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checklib import read, must_read, Report                    # noqa: E402

CLI = "src/cli.c"
BRIDGE = "src/bridge.c"

# tool -> usage function, parser function, and the surfaces that must list
# every one of its options.
TOOLS = {
    "imud": {
        "usage": "usage_imud", "parse": "cli_parse_imud",
        "surfaces": ["man/man8/imud.8", "docs/manual.md", "spec.md",
                     "web/index.html"],
    },
    "imud-cal": {
        "usage": "usage_cal", "parse": "cli_parse_cal",
        "surfaces": ["man/man8/imud-cal.8"],
    },
    "imud-mon": {
        "usage": "usage_mon", "parse": "cli_parse_mon",
        "surfaces": ["man/man1/imud-mon.1"],
    },
    "imud-status": {
        "usage": "usage_status", "parse": "cli_parse_status",
        "surfaces": ["man/man1/imud-status.1"],
    },
    "imud-imutest": {
        "usage": "usage_imutest", "parse": "cli_parse_imutest",
        "surfaces": ["man/man8/imud-imutest.8"],
    },
}

# All five bridges share one parser and one usage function in bridge.c.
BRIDGES = ["signalk", "mqtt", "influxdb", "mavlink", "prometheus"]

# Options a parser accepts but no surface is required to advertise, with the
# reason.  Keep this short and argued — it is the record of every exception.
HIDDEN = {
    # -h is the conventional short form of --help; man pages document the
    # long form and readers do not need both spelled out on every surface.
    "-h",
}


def body_of(src, name):
    """The text of a C function, brace-matched from its opening line."""
    m = re.search(r"^\w[\w \*]*\b" + re.escape(name) + r"\s*\([^)]*\)\s*\{",
                  src, re.M)
    if not m:
        return None
    depth, i = 0, m.end() - 1
    for j in range(i, len(src)):
        if src[j] == "{":
            depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0:
                return src[m.end():j]
    return None


def options_in(text):
    """Every --long-option mentioned, minus the hidden ones."""
    return {o for o in re.findall(r"--[a-z][a-z0-9-]*", text)} - HIDDEN


def main():
    rep = Report("check-cli-docs")
    cli = must_read(CLI, "the CLI parsers")
    bridge = must_read(BRIDGE, "the shared bridge parser")

    plan = dict(TOOLS)
    for name in BRIDGES:
        plan[f"imud-{name}"] = {
            "usage": "usage", "parse": "bridge_parse_cli", "src": bridge,
            "surfaces": [f"man/man8/imud-{name}.8"],
        }

    for tool, spec in sorted(plan.items()):
        src = spec.get("src", cli)

        usage_body = body_of(src, spec["usage"])
        parse_body = body_of(src, spec["parse"])
        if usage_body is None or parse_body is None:
            rep.fail(f"{tool}: cannot find {spec['usage']}() or "
                     f"{spec['parse']}() — have they been renamed?")
            continue

        # The parser's options are the strcmp'd literals, not every -- token:
        # a usage string quoted inside the parser would otherwise count.
        #
        # Both binding styles have to match.  Most parsers compare argv[i]
        # directly; cli_parse_imutest binds `const char *s = argv[i]` first
        # and compares s, so a regex pinned to argv[i] reads its thirty-odd
        # flags as unaccepted and buries the real findings.
        accepted = {o for o in re.findall(
            r'strcmp\(\s*(?:argv\[i\]|\w+)\s*,\s*"(--[a-z0-9-]+)"', parse_body)
        } - HIDDEN
        advertised = options_in(usage_body)

        rep.expect(accepted, f"{tool} parser options")
        rep.expect(advertised, f"{tool} usage-string options")

        # 1. Everything the parser accepts appears in its own --help output.
        for opt in sorted(accepted - advertised):
            rep.check(False,
                      f"{CLI}: {tool} accepts {opt} but {spec['usage']}() does "
                      f"not list it — `{tool} --help` hides a working flag")

        # 3. Nothing is advertised that the parser rejects.
        for opt in sorted(advertised - accepted):
            rep.check(False,
                      f"{CLI}: {spec['usage']}() advertises {opt} for {tool}, "
                      f"but {spec['parse']}() does not accept it")

        # 2. Every accepted option reaches every reference surface.
        for rel in spec["surfaces"]:
            text = read(rel)
            if text is None:
                rep.fail(f"{tool}: missing surface {rel}")
                continue
            # roff splits options across escape sequences (\fB\-\-config\fR),
            # so compare on a de-escaped copy.
            flat = text.replace("\\fB", "").replace("\\fR", "")
            flat = flat.replace("\\fI", "").replace("\\-", "-")
            for opt in sorted(accepted):
                rep.check(opt in flat,
                          f"{rel}: {tool} accepts {opt}, not documented here")

    return rep.finish(f"{len(plan)} tools across "
                      f"{sum(len(s['surfaces']) for s in plan.values())} surfaces")


if __name__ == "__main__":
    sys.exit(main())
