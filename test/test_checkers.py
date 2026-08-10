#!/usr/bin/env python3
#
# imud — IMU daemon
# Copyright (c) 2026 Richard Simpson
# SPDX-License-Identifier: MIT
#
"""
test_checkers.py — the doc checkers must actually detect drift.

A checker run against a clean tree proves one thing: it does not false-
positive.  It says nothing about whether it can detect the drift it exists
for, and a regex that quietly stopped matching after a refactor looks exactly
like a clean tree.  Every checker in tools/ is a regex over source, so this is
not a hypothetical failure mode — four of them mis-parsed a correct document
during development, and the reverse is just as easy.

Method: copy the tree, break one specific fact, and assert the checker emits a
finding it was NOT emitting before.  Comparing against the pristine run rather
than against zero means these tests work whether or not the tree has been
cleaned up yet, and keeps passing afterwards.

Mutations are deliberately distinctive (99, ZZZZZ) rather than replays of the
drifts that shipped: a mutation that happens to match the current state would
produce no new finding and the test would fail for the wrong reason.

Run as `make test-tools`, or directly.  Needs git and python3, nothing else.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOOLS = os.path.join(ROOT, "tools")


# ── the cases ────────────────────────────────────────────────────────────────
# (checker, file to break, how to break it, what the finding must mention)

def sub(pattern, repl, count=1):
    """Regex substitution, asserting it actually changed something."""
    def apply(text):
        # re.M throughout: every fixture pattern anchors to a line, not to the
        # start of a 10 000-line file.
        new, n = re.subn(pattern, repl, text, count=count, flags=re.M)
        if n == 0:
            raise AssertionError(f"fixture pattern never matched: {pattern!r}")
        return new
    return apply


CASES = [
    # The wire version, which spent three revisions reading 14 in this table
    # while the header said 17.
    ("gen-packet-docs", "include/types.h",
     sub(r"#define\s+IMUD_VERSION\s+\d+", "#define IMUD_VERSION  99"),
     "spec.md does not match"),

    # A field added to the packet: the offsets beneath it move and the total
    # changes, while nothing about the rows themselves looks wrong.  Anchored
    # on the crc32 DECLARATION — several field names in types.h appear in more
    # than one struct, and a mutation that lands in the wrong one silently
    # tests nothing.
    ("gen-packet-docs", "include/types.h",
     sub(r"^    uint32_t crc32;", "    uint32_t zzz_new;\n    uint32_t crc32;"),
     "_Static_assert pins"),

    # A flag renamed in the header, leaving the table describing a bit by a
    # name no longer defined.
    ("gen-packet-docs", "include/types.h",
     sub(r"^#define FLAG_ENGINE_ON(\s+)\(1u << 13\)",
         r"#define FLAG_ZZZ_ON\1(1u << 13)"),
     "zzz_on"),

    # A new packet field with no Notes prose.  The table would still be
    # correct and still be useless, so it is a failure.
    ("gen-packet-docs", "docs/packet-notes.toml",
     sub(r"^heave_m = .*$", ""),
     "no note for 'heave_m'"),

    ("check-nmea", "src/nmea.c",
     sub(r'"\$?PASHR', '"$ZZZZZ'),
     "ZZZZZ"),

    # A rate list that grew in the driver.  This is the drift that shipped:
    # the man page stopped at 1660 while ism330dhcx.c offered 3332 and 6664.
    ("gen-config-docs", "src/drivers/ism330dhcx.c",
     sub(r"\.supported_accel_g\s*=\s*\{[^}]*\}",
         ".supported_accel_g  = { 2, 4, 8, 16, 32, 0 }"),
     "[imu] accel_g"),

    # A driver's SPI clock changed without the driver table following.
    ("gen-drivers", "src/drivers/ism330dhcx.c",
     sub(r"\.spi_max_hz = 10000000", ".spi_max_hz = 20000000"),
     "driver table does not match"),

    # A driver registered and never documented.
    ("gen-drivers", "src/drivers.c",
     sub(r"^(\s*)&lis2mdl_ops,", r"\1&zzz_new_ops,\n\1&lis2mdl_ops,"),
     "zzz_new_ops"),

    # A driver promoted out of experimental in the code only.
    ("gen-drivers", "src/drivers/lis3mdl.c",
     sub(r"\.experimental\s*=\s*true", ".experimental     = false"),
     "driver table does not match"),

    ("check-mqtt-topics", "src/mqtt_publish.c",
     sub(r'\{ "imu/temperature"', '{ "imu/zzztemp"'),
     "imu/zzztemp"),

    # A topic whose publish CONDITION changed.  It stays in the table, stays
    # in the config template, and every subscriber that reads "always" waits
    # for a message that now needs a settled sea state.
    ("check-mqtt-topics", "src/mqtt_publish.c",
     sub(r'"Temperature",        U_TEMP,     GATE_ALWAYS',
         '"Temperature",        U_TEMP,     GATE_WAVE'),
     "published under GATE_WAVE"),

    # A topic dropped from the code and left in the docs.
    ("check-mqtt-topics", "src/mqtt_publish.c",
     sub(r'^\s*\{ "navigation/rateOfTurn".*\n', ""),
     "no longer publishes"),

    # The same, for a Prometheus metric.
    ("check-bridge-outputs", "src/prom_metrics.c",
     sub(r'GAUGE\("imud_wave_height_meters"', 'GAUGE("imud_zzz_height_meters"'),
     "no longer exports"),

    ("check-cli-docs", "src/cli.c",
     sub(r'strcmp\(argv\[i\], "--no-nmea"\)', 'strcmp(argv[i], "--no-zzz")'),
     "--no-zzz"),

    ("check-bridge-outputs", "src/prom_metrics.c",
     sub(r'GAUGE\("imud_up"', 'GAUGE("imud_zzz_total"'),
     "imud_zzz_total"),

    ("check-libimud-api", "lib/libimud.map",
     sub(r"imud_lib_version;", "imud_lib_version;\n        imud_zzz;"),
     "imud_zzz"),

    ("check-math-citations", "docs/math.md",
     sub(r"`src/fusion\.c`", "`src/no_such_file.c`"),
     "no_such_file"),

    ("check-links", "docs/manual.md",
     sub(r"\]\(#6-calibration\)", "](#no-such-heading)"),
     "no-such-heading"),

    # A page the Makefile stages and no Debian package claims: staged, then
    # dropped from every .deb.  dh_missing only notices during a full package
    # build, which is why this is checked here.
    ("check-manpages", "debian/imud.install",
     sub(r"^usr/share/man/man1/imud-status\.1\.gz\n", ""),
     "imud-status.1"),

    # A config key whose documented default drifted from the registry.
    ("gen-config-docs", "man/man5/imud.conf.5",
     sub(r"^\.RI \(int,\\ default:\\ 833\)$", r".RI (int,\\ default:\\ 999)"),
     "odr_hz"),

    # ...and the same on the markdown surface.
    ("gen-config-docs", "docs/manual.md",
     sub(r"^\| `fifo_wm` \| int \| `64` \|", "| `fifo_wm` | int | `128` |"),
     "fifo_wm"),

    # And the reverse: Debian shipping a page the Makefile never stages.
    ("check-manpages", "debian/imud-utils.install",
     sub(r"^usr/share/man/man1/imud-mon\.1\.gz$",
         "usr/share/man/man1/imud-mon.1.gz\nusr/share/man/man9/imud-zzz.9.gz"),
     "imud-zzz"),

    # ── The registry describing a parser that no longer exists ───────────────
    # Every generated surface is rendered from docs/config-keys.toml, so a
    # registry that has drifted from apply_kv() does not fail — it publishes.
    # These are the ways it can drift.

    # The field a key writes was renamed in the registry but not in the code
    # (or, just as likely, the other way round).
    ("check-docs", "docs/config-keys.toml",
     sub(r'fields = \["imu_odr_hz"\]', 'fields = ["imu_odr_zzz"]'),
     "imu_odr_zzz"),

    # The macro drifted: NEED_POS_INT rejects zero, NEED_INT accepts it, and
    # the generated defaults test picks its comparison from this.
    ("check-docs", "docs/config-keys.toml",
     sub(r'macros = \["NEED_POS_INT"\]', 'macros = ["NEED_INT"]'),
     "odr_hz"),

    # A key the parser gained and nobody documented.
    ("check-docs", "src/config.c",
     sub(r'(else if \(strcmp\(key, "fifo_wm"\)  == 0\) NEED_INT\(cfg->imu_fifo_wm\);)',
         r'\1\n        else if (strcmp(key, "zzz_new")  == 0) '
         r'NEED_INT(cfg->imu_accel_g);'),
     "zzz_new"),

    # A default restated outside a generated region — the one place a
    # documented default can still rot, and where one had.
    ("check-docs", "man/man5/imud-prometheus.conf.5",
     sub(r"^\.RI \(string,\\ default:\\ \\\(dqwarn\\\(dq\)$",
         r".RI (string,\\ default:\\ info)"),
     "level restates its default"),

    # A [hot] key config_apply_hot() does not copy: documented as live, and
    # silently ignored on every SIGHUP.
    ("check-docs", "src/config.c",
     sub(r"^    dst->stream_rate_hz   = src->stream_rate_hz;\n", ""),
     "[stream] rate_hz"),

    # ...and the reverse, a field copied live that the page calls [restart].
    ("check-docs", "src/config.c",
     sub(r"^(    dst->log_stats_hz     = src->log_stats_hz;)$",
         r"\1\n    dst->nmea_dest_port   = src->nmea_dest_port;"),
     "[nmea] dest_port"),

    # The generated defaults test going stale against the registry.  It is
    # committed, so nothing rebuilds it on checkout — only this notices.
    ("gen-config-docs", "test/test_config_defaults.gen.c",
     sub(r"^CK_INT \(c\.imu_odr_hz,       833,", "CK_INT (c.imu_odr_hz,       999,"),
     "test_config_defaults.gen.c is stale"),

    # ── The Info manual ──────────────────────────────────────────────────────
    # docs/imud.texi is committed and NOT diff-gated (two pandocs, two
    # outputs), so these are the whole guarantee that it still is the manual.

    # A section added to the manual and never converted: invisible to
    # `info imud`, and the failure mode the conversion exists to prevent.
    ("check-texi", "docs/manual.md",
     sub(r"^## 10\. Troubleshooting", "## 9z. Zzz new section\n\n"
         "Body.\n\n## 10. Troubleshooting"),
     "Zzz new section"),

    # A section renamed in the manual, leaving the .texi node behind.
    ("check-texi", "docs/manual.md",
     sub(r"^### Signals$", "### Zzz signals"),
     "Zzz signals"),

    # A version bump that never reached `make docs-texi`.
    ("check-texi", "include/version.h",
     sub(r'#define IMUD_VERSION_STR\s+"[^"]+"',
         '#define IMUD_VERSION_STR "9.9.9"'),
     "@set VERSION"),

    # The Info directory entry deleted: the manual installs and never appears
    # in `info` or `M-x info`, which no other check would notice.
    ("check-texi", "docs/imud.texi",
     sub(r"^@dircategory .*$", ""),
     "@dircategory"),

    # docs/math.md edited without rebuilding the PDF — the state the tree was
    # already in when this was written, three weeks deep.
    ("check-math-pdf-stamp", "docs/math.md",
     sub(r"^# ", "# Zzz "),
     "built from a different"),
]


def findings(checker, root):
    """The set of FAIL lines a checker emits against `root`."""
    env = dict(os.environ, IMUD_ROOT=root)
    p = subprocess.run([sys.executable, os.path.join(TOOLS, f"{checker}.py")],
                       capture_output=True, text=True, env=env)
    if p.returncode not in (0, 1):
        raise AssertionError(
            f"{checker} exited {p.returncode} (not a pass/fail):\n{p.stderr}")
    return {l for l in p.stderr.split("\n") if l.startswith("FAIL")}


def main():
    with tempfile.TemporaryDirectory(prefix="imud-checkers-") as tmp:
        pristine = os.path.join(tmp, "pristine")
        os.makedirs(pristine)

        # The fixture is the WORKING TREE, not HEAD.  These checkers are the
        # gate for changes that have not been committed yet — a fixture built
        # from `git archive HEAD` cannot see the checker being added or the
        # Makefile variable it reads, so the test silently exercises the old
        # tree and reports a detection failure that is really a fixture bug.
        #
        # --cached --others --exclude-standard is tracked files plus new ones,
        # minus anything gitignored: what a commit would contain, as it stands
        # right now.
        listing = subprocess.run(
            ["git", "-C", ROOT, "ls-files", "-z",
             "--cached", "--others", "--exclude-standard"],
            capture_output=True, check=True).stdout
        for rel in listing.decode().split("\0"):
            if not rel:
                continue
            src = os.path.join(ROOT, rel)
            if not os.path.isfile(src):       # deleted-but-still-indexed
                continue
            dst = os.path.join(pristine, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(src, dst)

        base = {}
        for checker, *_ in CASES:
            if checker not in base:
                base[checker] = findings(checker, pristine)

        failures = 0
        for checker, target, mutate, expect in CASES:
            work = os.path.join(tmp, "work")
            shutil.rmtree(work, ignore_errors=True)
            shutil.copytree(pristine, work, symlinks=True)

            path = os.path.join(work, target)
            text = open(path, encoding="utf-8").read()
            try:
                open(path, "w", encoding="utf-8").write(mutate(text))
            except AssertionError as e:
                print(f"FAIL {checker} [{target}]: {e}", file=sys.stderr)
                failures += 1
                continue

            new = findings(checker, work) - base[checker]
            hit = [l for l in new if expect in l]
            if hit:
                print(f"ok   {checker:22} {target:28} detects {expect!r}")
            else:
                failures += 1
                print(f"FAIL {checker:22} {target:28} did NOT detect "
                      f"{expect!r} after mutation", file=sys.stderr)
                for l in sorted(new)[:3]:
                    print(f"       new finding: {l}", file=sys.stderr)

        if failures:
            print(f"\n{failures} of {len(CASES)} checkers failed to detect "
                  f"their drift", file=sys.stderr)
            return 1
        print(f"\ntest_checkers: {len(CASES)} drifts, all detected")
        return 0


if __name__ == "__main__":
    sys.exit(main())
