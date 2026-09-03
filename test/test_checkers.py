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


def chain(*fns):
    """Apply several substitutions in order, as one fixture."""
    def apply(text):
        for f in fns:
            text = f(text)
        return text
    return apply


def top_bullet(prefix="Zzz "):
    """Prefix the first `  * ` bullet of a Debian changelog's TOP stanza.

    gen-release-notes only ever compares the stanza for the version in
    include/version.h, which is the one on top; every stanza below it shipped
    and is frozen byte for byte, so a hand-edit down there is invisible.  The
    top stanza is therefore the only place a fixture can break.

    Finding that bullet structurally -- line 1 is the header, the bullets run
    to the first ` -- name <mail>  date` trailer -- is what makes this survive
    a release.  Naming the bullet's text instead pins the fixture to whichever
    version was current when it was written, and it then stops detecting at
    the next bump.
    """
    def apply(text):
        lines = text.split("\n")
        end = next((i for i, l in enumerate(lines) if l.startswith(" -- ")),
                   None)
        if end is None:
            raise AssertionError("no ` -- ` trailer: not a Debian changelog")
        for i in range(1, end):
            if lines[i].startswith("  * "):
                lines[i] = "  * " + prefix + lines[i][4:]
                return "\n".join(lines)
        raise AssertionError("top stanza has no `  * ` bullet to break")
    return apply


CASES = [
    # ── check-imutest-checks ─────────────────────────────────────────────────
    # A new check added to the tool and not to the spec. mag.drdy.restore
    # shipped exactly this way -- the one report line telling a reader to
    # distrust everything below it, undescribed.
    ("check-imutest-checks", "src/imutest.c",
     sub(r'add_check\(r, "mag\.set_reset"',
         'add_check(r, "mag.zzz_new"'),
     "mag.zzz_new"),

    # A documented id renamed in the tool, so the spec now describes a verdict
    # that can never print and says nothing about the one that can.
    ("check-imutest-checks", "src/imutest.c",
     sub(r'"imu\.rest\.gravity"', '"imu.rest.gravity2"'),
     "imu.rest.gravity2"),

    # A check the spec PROMISES and the tool does not implement. This is the
    # reverse direction, and it is the one that actually shipped: the spec said
    # "The mag equivalent is `mag.bus.integrity`" while no such check existed,
    # so a gap in coverage read as coverage. Note the mutation renames the id to
    # another that is itself documented and emitted -- renaming it to something
    # novel would trip the forward direction instead and prove nothing here.
    # count=0 because the id appears three times (one skip_check, two
    # add_check): replacing only the first leaves the check still emitted and
    # the mutation proves nothing.
    ("check-imutest-checks", "src/imutest.c",
     sub(r'"mag\.bus\.integrity"', '"mag.probe"', 0),   # count=0: ALL of them
     "mag equivalent"),

    # ── check-package-descriptions ───────────────────────────────────────────
    # Each package's description lives in debian/control (what apt shows) and
    # in packaging/<pkg>/description (what the repo documents). This found a
    # real drift on its first run: control required a little-endian host and
    # the packaging copy had lost the paragraph.
    ("check-package-descriptions", "packaging/imud-influxdb/description",
     sub(r"^InfluxDB bridge for the imud IMU daemon$",
         "InfluxDB exporter for the imud IMU daemon"),
     "synopsis differs"),

    # Body drift is the one that matters more: it is longer, it is what a
    # reader actually reads, and a removed dependency goes unmentioned there.
    ("check-package-descriptions", "packaging/imud-mqtt/description",
     sub(r"^ QoS, retained values, TLS.*$",
         " QoS and retained values."),
     "body differs"),

    # ── check-flags ──────────────────────────────────────────────────────────
    # The flags word has four independent definitions and no generator. A
    # drifted bit is the worst kind of drift here: the packet still parses and
    # the wrong bit is read under the right name, so nothing downstream errors.
    ("check-flags", "lib/imud_client.h",
     sub(r"^#define IMUD_FLAG_ENGINE_ON(\s+)\(1u << 13\)",
         r"#define IMUD_FLAG_ENGINE_ON\1(1u << 12)"),
     "at bit 12"),

    # A flag that never reached one of the clients. The C headers are edited
    # together often enough to stay in step; the Python client is the one that
    # gets forgotten.
    ("check-flags", "lib/imud_client.py",
     sub(r"^    ENGINE_ON(\s+)= 1 << 13.*$", r"    # removed"),
     "missing ENGINE_ON"),

    # ── check-devices ────────────────────────────────────────────────────────
    # The defect this checker was written for: the shipped config names a
    # device node the shipped unit does not allow. DevicePolicy=closed refuses
    # the open, and nothing else in the tree compares the two files —
    # systemd-analyze passes, because the unit is valid; it is just wrong.
    ("check-devices", "config/imud.conf",
     sub(r'^gpio_chip(\s+)= "gpiochip4"', r'gpio_chip\1= "gpiochip9"'),
     "does not allow"),

    # A trailing comment on a DeviceAllow= line. systemd parses the rights
    # token strictly and discards the whole line if it does not match r/w/m,
    # so this silently allows NOTHING — which looks like a working unit right
    # up until the device open fails on hardware.
    ("check-devices", "etc/imud.service.in",
     sub(r"^DeviceAllow=/dev/i2c-1 rw$",
         "DeviceAllow=/dev/i2c-1 rw   # the IMU bus"),
     "rights token"),

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

    # The drift that shipped: 25 of 26 line citations pointed at the wrong
    # line by 1.9.0 while the checker passed, because it only read names.
    # A line outside the function it names must fail.
    ("check-math-citations", "docs/math.md",
     sub(r"`eskf_update\(\)` \(`fusion\.c:\d+`", "`eskf_update()` (`fusion.c:999`"),
     "fusion.c:999"),

    # A citation that names no symbol (they point at a comment or an
    # expression) still has to land inside SOME definition — 453 is a gap
    # between two functions.
    ("check-math-citations", "docs/math.md",
     sub(r"the in-code comment at `fusion\.c:\d+`",
         "the in-code comment at `fusion.c:453`"),
     "fusion.c:453"),

    # A line past the end of the file.
    ("check-math-citations", "docs/math.md",
     sub(r"`gate_health\(\)` \(`fusion\.c:\d+`", "`gate_health()` (`fusion.c:99999`"),
     "99999"),

    # A range that runs backwards.
    ("check-math-citations", "docs/math.md",
     sub(r"`fusion\.c:1829`–`\d+`", "`fusion.c:1829`–`999`"),
     "run forwards"),

    # The `:449` shorthand, which continues whichever file was named last,
    # rots exactly like a full citation and is resolved the same way.
    ("check-math-citations", "docs/math.md",
     sub(r"`f->Rm` \(`:\d+`\)", "`f->Rm` (`:450`)"),
     "fusion.c:450"),

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

    # ── The seccomp re-allow list losing an architecture ─────────────────────
    # This is the 1.9.0 RC bug, restaged. Reverting to the x86-only name is
    # invisible to every x86 test and to the whole doc family; on arm64 it is
    # SIGSYS before the first sample. The mutation has to be the REAL one —
    # dropping the two ARM names while leaving a plausible line behind — or it
    # would prove only that the checker notices an empty list.
    ("check-seccomp", "etc/imud.service.in",
     sub(r"^SystemCallFilter=adjtimex clock_adjtime clock_adjtime64$",
         "SystemCallFilter=adjtimex"),
     "clock_adjtime"),

    # The deny line going away entirely. Then nothing needs re-allowing, the
    # re-allow list is cargo cult, and the checker should say so rather than
    # pass because every name it wanted is present.
    ("check-seccomp", "etc/imud.service.in",
     sub(r"^SystemCallFilter=~@privileged @resources$", ""),
     "@privileged"),

    # ── The registry describing a parser that no longer exists ───────────────
    # Every generated surface is rendered from docs/config-keys.toml, so a
    # registry that has drifted from apply_kv() does not fail — it publishes.
    # These are the ways it can drift.

    # The field a key writes was renamed in the registry but not in the code
    # (or, just as likely, the other way round).
    ("check-docs", "docs/config-keys.toml",
     sub(r'fields = \["imu_odr_mhz"\]', 'fields = ["imu_odr_zzz"]'),
     "imu_odr_zzz"),

    # The macro drifted: NEED_POS_MHZ scales Hz into milli-Hz and NEED_INT does
    # not, so the generated defaults test picks a different comparison.
    ("check-docs", "docs/config-keys.toml",
     sub(r'macros = \["NEED_POS_MHZ"\]', 'macros = ["NEED_INT"]'),
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
     sub(r"^CK_INT \(c\.imu_odr_mhz,      833000,",
         "CK_INT (c.imu_odr_mhz,      999000,"),
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

    # ── gen-release-notes ────────────────────────────────────────────────────
    # NEWS edited by hand instead of the registry. This is the drift the whole
    # registry exists to stop: NEWS said one thing and the changelog another
    # for the same release, and nothing compared them.
    ("gen-release-notes", "NEWS",
     sub(r'^\* SPI transport for the IMU and the magnetometer\.',
         '* SPI transport for the CPU and the magnetometer.'),
     "NEWS"),

    # A changelog stanza edited by hand. Packagers read this one; a user reads
    # NEWS. They describe the same release and must not diverge.
    #
    # The bullet is found rather than named, because only the top stanza is
    # compared and which release is on top changes at every version bump. A
    # fixture naming its bullet detects nothing from the next release onward.
    ("gen-release-notes", "packaging/imud/changelog",
     top_bullet(),
     "packaging/imud/changelog"),

    # A change that outgrows the word cap. Every NEWS entry in this project
    # drifted this way -- 32 words per bullet at 1.4, 217 at 1.8 with one of
    # 587 -- one reasonable-looking sentence at a time.
    ("gen-release-notes", "docs/release-notes.toml",
     sub(r'^  text     = "The manual installs as an Info page: info imud\."',
         '  text     = "The manual installs as an Info page, which is worth '
         'recording at length because the investigation behind it ran for '
         'several days and produced a number of measurements that seemed '
         'important at the time and are set down here in full detail."'),
     "cap"),

    # A package that does not exist, so the stanza would be written nowhere.
    ("gen-release-notes", "docs/release-notes.toml",
     sub(r'^  packages = \["imud-utils"\]', '  packages = ["imud-nope"]'),
     "imud-nope"),

    # A kind outside feature/behaviour/fix, which would silently vanish from
    # the rendered output rather than being reported.
    ("gen-release-notes", "docs/release-notes.toml",
     sub(r'^  kind     = "behaviour"', '  kind     = "improvement"'),
     "improvement"),

    # A release that shipped with its date never stamped, leaving two stanzas
    # claiming to be the one still accumulating. The older then keeps taking
    # entries that render into the wrong NEWS section. 1.9.0 shipped this way.
    #
    # Both fixtures below are written against a tree that DOES carry an
    # unreleased stanza, because between releases it does -- that is the state
    # the marker exists for. Unstamping a second date is therefore enough to
    # reach two, and reaching "is newer" needs the real one stamped first.
    ("gen-release-notes", "docs/release-notes.toml",
     sub(r'^date    = "2026-08-31"', 'date    = "unreleased"'),
     "2 releases"),

    # The date in a form nothing downstream can order or parse.
    ("gen-release-notes", "docs/release-notes.toml",
     sub(r'^date    = "2026-08-28"', 'date    = "Aug 28 2026"'),
     "YYYY-MM-DD"),

    # The accumulating stanza left behind a newer one, so changes land in a
    # section that has already shipped. Reached by stamping the real
    # accumulating stanza and unstamping an older one, rather than renaming the
    # newest, which would only trip the separate check that include/version.h
    # has a stanza at all.
    ("gen-release-notes", "docs/release-notes.toml",
     chain(sub(r'^date    = "unreleased"', 'date    = "2026-09-01"'),
           sub(r'^date    = "2026-08-28"', 'date    = "unreleased"')),
     "is newer"),


    # ── check-comment-refs ───────────────────────────────────────────────────
    # A comment naming a function that does not exist -- what a rename leaves
    # behind once the code has been updated and the prose has not.
    ("check-comment-refs", "src/imu.c",
     sub(r'\bimu_finalise_sample\(\)', 'imu_finalise_sample_gone()'),
     "imu_finalise_sample_gone()"),

    # A file:line citation pointing past the end of the file it names.
    ("check-comment-refs", "test/test_config.c",
     sub(r'imu_math\.c:265', 'imu_math.c:999999'),
     "imu_math.c:999999"),

    # ── check-arch-claims ────────────────────────────────────────────────────
    # The build moves to another CPU baseline and the documentation does not
    # follow.  This is the direction that actually ships a broken package: the
    # docs keep promising ARMv6 while the release is built for something a Pi 1
    # cannot execute.  Every documented surface must report, so the fixture
    # asserts on the one carrying the full explanation.
    ("check-arch-claims", ".github/workflows/build-debs.yml",
     sub(r'^(\s*)arm_baseline: armv6$', r'\1arm_baseline: armv9-a'),
     "docs/manual.md"),

    # The reverse: the build is right and a document lost the fact.  count=0
    # because docs/manual.md states ARMv6 three times -- replacing only the
    # first leaves the other two satisfying the check, and the mutation would
    # prove nothing.
    ("check-arch-claims", "docs/manual.md",
     sub(r'ARMv6', 'ARMv9', 0),                        # count=0: ALL of them
     "docs/manual.md"),

    # The archive key silently swapped for another.  The rootfs has no image
    # digest to pin, so the fingerprint is the only trust anchor the armhf
    # build has -- an unpinned one would trust whatever the mirror served.
    ("check-arch-claims", "tools/bootstrap-raspbian.sh",
     sub(r'A0DA38D0D76E8B5D638872819165938D90FDDD2E',
         'ZZZZ38D0D76E8B5D638872819165938D90FDDD2E'),
     "is not pinned"),

    # ── check-fuzz-targets ───────────────────────────────────────────────────
    # A harness reaching some of the seven surfaces that name it and not the
    # rest.  This is fuzz_argv's history: added in 6bc6a51, present in CI and
    # nowhere else, so SECURITY.md went on telling reporters argv was not
    # fuzzed.  Each surface fails for its own reason, so each gets a case.

    # Built but never run.  The build still passes, which is what makes this
    # the quiet one.
    ("check-fuzz-targets", ".github/workflows/ci.yml",
     sub(r'^(\s*for t in .*) argv; do$', r'\1; do'),
     "smoke loop"),

    # ...and the reverse: dropped from the build while the loop still runs it.
    ("check-fuzz-targets", ".github/workflows/ci.yml",
     sub(r'^\s*\$FZ fuzz/fuzz_cal\.c.*\n', ''),
     "$FZ build lines"),

    # The nightly's per-target link line.  A missing case arm links the
    # harness against no sources at all.
    ("check-fuzz-targets", ".github/workflows/fuzz-nightly.yml",
     sub(r'^\s*cal\)\s+DEPS=.*\n', ''),
     "DEPS case"),

    # The document a reporter actually reads.
    ("check-fuzz-targets", "SECURITY.md",
     sub(r'`fuzz_wmm`', '`fuzz_zzz`'),
     "fuzz_zzz"),

    # Left ungitignored, a built harness sits in `git status` waiting to be
    # staged by a wildcard -- the same trap as the crash-* reproducers.
    ("check-fuzz-targets", ".gitignore",
     sub(r'^/fuzz_packet$', ''),
     ".gitignore"),

    # A stated count going stale.  The table was corrected once already and
    # the prose around it was not.
    ("check-fuzz-targets", ".github/workflows/codeql.yml",
     sub(r'seven fuzzers', 'three fuzzers'),
     "three fuzzers"),

    # ── check-portable-tests ─────────────────────────────────────────────────
    # A suite built by every run and executed by none.  Silent: the binary is
    # up to date, `make test` succeeds, and nothing it asserts was ever asked.
    ("check-portable-tests", "Makefile",
     sub(r'^\t\./test_ring$', ''),
     "test_ring"),

    # Left out of `clean`, a suite survives it and is then reused against the
    # next set of flags -- the __gcov_init link failure after `make coverage`.
    ("check-portable-tests", "Makefile",
     sub(r'test_hwtools_e2e test_configure', 'test_configure'),
     "clean recipe"),

    # Left ungitignored, a built suite waits in `git status` for a wildcard.
    ("check-portable-tests", ".gitignore",
     sub(r'^/test_daemon$', ''),
     "test_daemon"),

    # An exclusion with no reason beside it.  The list is where a suite goes to
    # stop being tested off Linux, so "why" is the whole of its content.
    ("check-portable-tests", "Makefile",
     sub(r'^NONPORTABLE_TEST_BINS =$', 'NONPORTABLE_TEST_BINS = test_nmea'),
     "NONPORTABLE_TEST_BINS with no"),

    # An exclusion naming a suite that does not exist. $(filter-out) takes a
    # typo without a word, so the suite it was meant to exclude still runs --
    # or, read the other way, the list stops meaning anything.
    ("check-portable-tests", "Makefile",
     sub(r'^NONPORTABLE_TEST_BINS =$', 'NONPORTABLE_TEST_BINS = test_zzzzz'),
     "test_zzzzz"),

    # PORTABLE_TEST_BINS spelled out by hand. That is the drift itself: the
    # filter-out is what makes a new suite portable by default.
    ("check-portable-tests", "Makefile",
     sub(r'^PORTABLE_TEST_BINS = \$\(filter-out.*$',
         'PORTABLE_TEST_BINS = test_host_time'),
     "PORTABLE_TEST_BINS must stay"),

    # The reported defect, put back: the macos job running one named suite
    # instead of the list. The comment above it still says test-portable, which
    # is why the checker reads the job with its comments stripped.
    ("check-portable-tests", ".github/workflows/ci.yml",
     sub(r'^          make -j3 test-portable.*$',
         '          make test_host_time && ./test_host_time'),
     "does not run"),

    # A stated count going stale -- both spellings, since the two are checked
    # against different lists.
    ("check-portable-tests", "CONTRIBUTING.md",
     sub(r'the 39 portable suites', 'the 3 portable suites'),
     "3 portable suites"),

    # ── check-web-drivers ────────────────────────────────────────────────────
    # The reported defect: a driver added to the tree and not to the page.
    # LSM6DSOX specifically, because LSM6DSO is a prefix of it -- a substring
    # test passes on this mutation and the checker must not.
    ("check-web-drivers", "web/index.html",
     sub(r'ST LSM6DSO and LSM6DSOX,', 'ST LSM6DSO,'),
     "LSM6DSOX"),

    # A validated part dropped from the Reference entry. The page then makes
    # no claim about the one combination that has actually been run.
    ("check-web-drivers", "web/index.html",
     sub(r'<strong>ISM330DHCX</strong>', '<strong>ISM330DHCZ</strong>'),
     "ISM330DHCX"),

    # The other direction, and the one that comes next: an imutest report
    # clears a flag in the ops struct and the page keeps selling the part as
    # unproven.
    ("check-web-drivers", "src/drivers/lis3mdl.c",
     sub(r'\.experimental     = true,', '.experimental     = false,'),
     "LIS3MDL"),

    # Both halves of the count sentence, checked against different lists.
    ("check-web-drivers", "web/index.html",
     sub(r'Twelve of the fourteen', 'Eleven of the fourteen'),
     "Eleven"),

    ("check-web-drivers", "web/index.html",
     sub(r'Twelve of the fourteen', 'Twelve of the thirteen'),
     "thirteen"),

    # The same figure restated where a would-be tester reads it. It said ten
    # while twelve were unproven, in the file whose whole subject is running
    # the tool that clears the flag.
    ("check-web-drivers", "docs/imud-utils/README.md",
     sub(r'drivers for twelve parts', 'drivers for ten parts'),
     "ten parts have never run"),

    ("check-portable-tests", "devbox/README.md",
     sub(r'runs 39 suites', 'runs 99 suites'),
     "99 suites"),

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
