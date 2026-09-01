/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_configure.c — the ./configure script: what it refuses, and what it
 * writes into config.mk.
 *
 * The script's answers come from exactly two programs it runs — the compiler
 * and pkg-config — so this puts a stub of each on an otherwise empty PATH and
 * dictates their replies.  Every probe outcome is then reachable here: a host
 * with no libgpiod, one that needs -latomic, a big-endian one, one missing the
 * Linux bus headers.  None of those exist on this bench, and waiting for one
 * is how a configure script rots.
 *
 * Stubbing the two programs rather than the probe results is deliberate: the
 * real script runs, with its real argument parsing, its real link-order rules
 * (a -l after the source, or --as-needed drops it) and its real output
 * writing.  The stub compiler decides only whether a given program links.
 *
 * The two real utilities symlinked onto that PATH — cat and uname — are not
 * under test.  Withholding them would prove nothing about the host and only
 * make the script contort around its own heredocs.
 */

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

static int g_fail;
static int g_checks;

#define EXPECT(c, msg) do {                                       \
        g_checks++;                                               \
        if (!(c)) { printf("  FAIL: %s\n", (msg)); g_fail++; }     \
    } while (0)

static char g_configure[PATH_MAX];   /* absolute path to ./configure */
static char g_work[]  = "/tmp/imud-conf-XXXXXX";
/* Half of PATH_MAX so that "$g_stub/pkg-config" is provably short enough for a
 * PATH_MAX buffer; g_work is a fixed /tmp template, so this is never tight. */
static char g_stub[PATH_MAX / 2];    /* $g_work/bin — the whole PATH */
static char g_real_path[8192];       /* the PATH this suite was started with */

/* ── Fixture ─────────────────────────────────────────────────────────────── */

static void write_exec(const char *path, const char *body)
{
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); exit(2); }
    fputs(body, f);
    fclose(f);
    if (chmod(path, 0755) != 0) { perror(path); exit(2); }
}

/*
 * The stub compiler.  It reads the probe program on stdin and matches it
 * against the header or symbol that identifies which probe is running, then
 * consults one environment variable per answer.  Everything defaults to
 * "links fine", so a case sets only the variable it is about.
 *
 * The -latomic and -lmosquitto arms look at the ARGUMENTS, not the program:
 * that is what pins configure's link order.  A -l placed before the source
 * would still reach this stub, so the ordering is asserted separately, in
 * test_link_order.
 */
static const char *STUB_CC =
    "#!/bin/sh\n"
    "prog=$(cat)\n"
    "args=\" $* \"\n"
    "fail() { echo \"stub cc: $1\" >&2; exit 1; }\n"
    "[ \"${STUB_CC_BROKEN:-0}\" = 0 ] || fail broken\n"
    "case $args in *\" -lmosquitto \"*)\n"
    "    [ \"${STUB_MOSQUITTO:-1}\" = 1 ] || fail mosquitto ;;\n"
    "esac\n"
    "case $prog in\n"
    "  *_Static_assert*)   [ \"${STUB_C11:-1}\" = 1 ] || fail c11 ;;\n"
    "  *pthread.h*)        [ \"${STUB_PTHREAD:-1}\" = 1 ] || fail pthread ;;\n"
    "  *math.h*)           [ \"${STUB_LIBM:-1}\" = 1 ] || fail libm ;;\n"
    "  *linux/i2c-dev.h*)  [ \"${STUB_BUS:-1}\" = 1 ] || fail bus ;;\n"
    "  *__ORDER_BIG_ENDIAN__*)\n"
    "      [ \"${STUB_ENDIAN:-little}\" = little ] || fail bigendian ;;\n"
    "  *stdatomic.h*)\n"
    "      case $args in\n"
    "        *\" -latomic \"*) [ \"${STUB_LATOMIC:-1}\" = 1 ] || fail latomic ;;\n"
    "        *)               [ \"${STUB_INLINE_ATOMIC:-1}\" = 1 ] || fail inline ;;\n"
    "      esac ;;\n"
    "  *clock_nanosleep*)  [ \"${STUB_CLOCKNS:-1}\" = 1 ] || fail clockns ;;\n"
    "  *adjtimex*)         [ \"${STUB_ADJTIMEX:-1}\" = 1 ] || fail adjtimex ;;\n"
    "  *accept4*)          [ \"${STUB_ACCEPT4:-1}\" = 1 ] || fail accept4 ;;\n"
    "esac\n"
    "echo \"$args\" >> \"$STUB_CC_LOG\"\n"
    "exit 0\n";

/* pkg-config answers for libgpiod only, and only when the case names a
 * version.  An empty STUB_GPIOD_VERSION is the host that does not have it. */
static const char *STUB_PKGCONFIG =
    "#!/bin/sh\n"
    "case \"$*\" in\n"
    "  \"--modversion libgpiod\")\n"
    "     [ -n \"${STUB_GPIOD_VERSION:-}\" ] || exit 1\n"
    "     echo \"$STUB_GPIOD_VERSION\" ;;\n"
    "  *) exit 1 ;;\n"
    "esac\n";

static void link_real(const char *tool)
{
    const char *dirs[] = { "/bin/", "/usr/bin/" };
    char dst[PATH_MAX], src[PATH_MAX];

    snprintf(dst, sizeof dst, "%s/%s", g_stub, tool);
    for (size_t i = 0; i < sizeof dirs / sizeof dirs[0]; i++) {
        snprintf(src, sizeof src, "%s%s", dirs[i], tool);
        if (access(src, X_OK) == 0) {
            if (symlink(src, dst) != 0 && errno != EEXIST) {
                perror(dst);
                exit(2);
            }
            return;
        }
    }
    fprintf(stderr, "test_configure: no %s on this host\n", tool);
    exit(2);
}

static void fixture_init(void)
{
    char path[PATH_MAX];

    if (!realpath("configure", g_configure)) {
        fprintf(stderr, "test_configure: no ./configure — run from the repo root\n");
        exit(2);
    }
    if (!mkdtemp(g_work)) { perror("mkdtemp"); exit(2); }

    snprintf(g_stub, sizeof g_stub, "%s/bin", g_work);
    if (mkdir(g_stub, 0755) != 0) { perror(g_stub); exit(2); }

    snprintf(path, sizeof path, "%s/cc", g_stub);
    write_exec(path, STUB_CC);
    snprintf(path, sizeof path, "%s/pkg-config", g_stub);
    write_exec(path, STUB_PKGCONFIG);

    link_real("cat");
    link_real("uname");

    snprintf(g_real_path, sizeof g_real_path, "%s", getenv("PATH") ? getenv("PATH") : "");

    /* The stub PATH is the whole environment configure gets, so every
     * optional tool it probes with `command -v` is absent unless a case adds
     * one.  That makes "nothing but a compiler" the default fixture. */
    setenv("PATH", g_stub, 1);
    setenv("CC", "cc", 1);
}

static void fixture_reset(void)
{
    static const char *vars[] = {
        "STUB_CC_BROKEN", "STUB_C11", "STUB_PTHREAD", "STUB_LIBM", "STUB_BUS",
        "STUB_ENDIAN", "STUB_INLINE_ATOMIC", "STUB_LATOMIC", "STUB_MOSQUITTO",
        "STUB_CLOCKNS", "STUB_ADJTIMEX", "STUB_ACCEPT4", "STUB_GPIOD_VERSION",
    };
    for (size_t i = 0; i < sizeof vars / sizeof vars[0]; i++)
        unsetenv(vars[i]);
}

/* ── Running it ──────────────────────────────────────────────────────────── */

/* Runs configure in the scratch directory with the current stub environment.
 * Returns its exit status; config.mk, stdout and stderr are left behind for
 * cfg()/out() to read. */
static int run(const char *args)
{
    char cmd[PATH_MAX * 2 + 256];
    char path[PATH_MAX];

    snprintf(path, sizeof path, "%s/cc.log", g_work);
    setenv("STUB_CC_LOG", path, 1);
    unlink(path);

    /* Removed here rather than in the shell: the stub PATH has no rm, and
     * every assertion below that config.mk was NOT written depends on a stale
     * one being gone first. */
    snprintf(path, sizeof path, "%s/config.mk", g_work);
    unlink(path);

    snprintf(cmd, sizeof cmd,
             "cd '%s' && '%s' %s > stdout.txt 2> stderr.txt",
             g_work, g_configure, args);

    int rc = system(cmd);
    return (rc == -1) ? -1 : WEXITSTATUS(rc);
}

/* Reads <name> back out of the generated config.mk.  Returns NULL when the
 * file or the variable is absent; an empty string when it is set to nothing,
 * which is a real answer here (ATOMIC_LIB, GPIOD_MAJ). */
static const char *cfg(const char *name)
{
    static char value[512];
    char path[PATH_MAX], line[512], key[128];
    FILE *f;

    snprintf(path, sizeof path, "%s/config.mk", g_work);
    if (!(f = fopen(path, "r"))) return NULL;

    /* Trailing whitespace is stripped before matching, so `NAME =` (set to
     * nothing) and `NAME = value` are read by the same key. */
    snprintf(key, sizeof key, "%s =", name);
    size_t klen = strlen(key);

    value[0] = '\0';
    const char *found = NULL;
    while (fgets(line, sizeof line, f)) {
        size_t n = strlen(line);
        while (n && (line[n - 1] == '\n' || line[n - 1] == ' ' ||
                     line[n - 1] == '\t'))
            line[--n] = '\0';
        if (strncmp(line, key, klen) != 0) continue;
        const char *v = line + klen;
        while (*v == ' ') v++;
        snprintf(value, sizeof value, "%s", v);
        found = value;
    }
    fclose(f);
    return found;
}

static int cfg_is(const char *name, const char *want)
{
    const char *got = cfg(name);
    return got && strcmp(got, want) == 0;
}

/* Slurps configure's stdout or stderr from the last run. */
static const char *out(const char *which)
{
    static char buf[16384];
    char path[PATH_MAX];
    FILE *f;

    snprintf(path, sizeof path, "%s/%s.txt", g_work, which);
    buf[0] = '\0';
    if ((f = fopen(path, "r"))) {
        size_t n = fread(buf, 1, sizeof buf - 1, f);
        buf[n] = '\0';
        fclose(f);
    }
    return buf;
}

static int wrote_config_mk(void)
{
    char path[PATH_MAX];
    snprintf(path, sizeof path, "%s/config.mk", g_work);
    return access(path, F_OK) == 0;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

/* help to stdout with exit 0, a bad option to stderr with exit 1 — the same
 * contract src/cli.c holds for the five front-ends.  help2man depends on it
 * there; here it is what keeps `./configure --help | less` from being a
 * failure the caller has to notice. */
static void test_usage_contract(void)
{
    printf("test_usage_contract\n");
    fixture_reset();

    EXPECT(run("--help") == 0, "--help exits 0");
    EXPECT(strstr(out("stdout"), "--without-gpiod") != NULL,
           "--help lists the options on stdout");
    EXPECT(out("stderr")[0] == '\0', "--help writes nothing to stderr");
    EXPECT(!wrote_config_mk(), "--help writes no config.mk");

    EXPECT(run("--version") == 0, "--version exits 0");
    EXPECT(strncmp(out("stdout"), "imud ", 5) == 0, "--version names the project");

    EXPECT(run("--nonsense") == 1, "an unknown option exits 1");
    EXPECT(strstr(out("stderr"), "--nonsense") != NULL,
           "and names it on stderr");
    EXPECT(out("stdout")[0] == '\0', "with nothing on stdout");
    EXPECT(!wrote_config_mk(), "and writes no config.mk");
}

static void test_gpiod_v2(void)
{
    printf("test_gpiod_v2\n");
    fixture_reset();
    setenv("STUB_GPIOD_VERSION", "2.2.1", 1);

    EXPECT(run("") == 0, "a host with libgpiod 2.x configures");
    EXPECT(cfg_is("NO_GPIOD", "0"), "NO_GPIOD = 0");
    EXPECT(cfg_is("GPIOD_MAJ", "2"), "GPIOD_MAJ = 2 (the -DGPIOD_V2 arm)");
    EXPECT(strstr(out("stdout"), "libgpiod 2.2.1") != NULL,
           "the summary names the version pkg-config reported");
}

static void test_gpiod_v1(void)
{
    printf("test_gpiod_v1\n");
    fixture_reset();
    setenv("STUB_GPIOD_VERSION", "1.6.3", 1);

    EXPECT(run("") == 0, "a bookworm host configures");
    EXPECT(cfg_is("NO_GPIOD", "0"), "NO_GPIOD = 0");
    EXPECT(cfg_is("GPIOD_MAJ", "1"), "GPIOD_MAJ = 1, not 1.6.3");
}

/* The case a porter actually meets: no libgpiod at all.  It must configure,
 * not fail — the interrupt line is optional at run time, and the null backend
 * leaves both readers on their rate-sized timer. */
static void test_gpiod_absent(void)
{
    printf("test_gpiod_absent\n");
    fixture_reset();

    EXPECT(run("") == 0, "a host with no libgpiod still configures");
    EXPECT(cfg_is("NO_GPIOD", "1"), "NO_GPIOD = 1 selects the null backend");
    EXPECT(cfg_is("GPIOD_MAJ", ""), "GPIOD_MAJ is empty, not stale");
    EXPECT(strstr(out("stdout"), "int_gpio = 0") != NULL,
           "the summary says what the operator must now set");
}

static void test_gpiod_forced(void)
{
    printf("test_gpiod_forced\n");
    fixture_reset();
    setenv("STUB_GPIOD_VERSION", "2.2.1", 1);

    EXPECT(run("--without-gpiod") == 0, "--without-gpiod configures");
    EXPECT(cfg_is("NO_GPIOD", "1"), "and takes the null backend anyway");
    EXPECT(cfg_is("GPIOD_MAJ", ""),
           "clearing GPIOD_MAJ too, so no -DGPIOD_V2 rides along");

    /* The asymmetry is the point: --without-gpiod is a choice, --with-gpiod is
     * a requirement.  Silently downgrading it would hand back a build that is
     * missing the interrupts the caller just asked for. */
    fixture_reset();
    EXPECT(run("--with-gpiod") == 1, "--with-gpiod fails when it is absent");
    EXPECT(strstr(out("stderr"), "libgpiod") != NULL, "saying so on stderr");
    EXPECT(!wrote_config_mk(), "and writes no config.mk");

    setenv("STUB_GPIOD_VERSION", "2.2.1", 1);
    EXPECT(run("--with-gpiod") == 0, "--with-gpiod succeeds when it is present");
    EXPECT(cfg_is("NO_GPIOD", "0"), "with the libgpiod backend");
}

/* Only the daemon's own dependencies are fatal.  Each of these is one of them,
 * and each must name itself rather than failing generically. */
static void test_required_failures(void)
{
    printf("test_required_failures\n");

    fixture_reset();
    setenv("STUB_CC_BROKEN", "1", 1);
    EXPECT(run("") == 1, "a compiler that cannot link anything fails");
    EXPECT(strstr(out("stderr"), "working C compiler") != NULL, "naming itself");
    EXPECT(!wrote_config_mk(), "and writes no config.mk");

    fixture_reset();
    setenv("STUB_C11", "0", 1);
    EXPECT(run("") == 1, "no -std=c11 fails");
    EXPECT(strstr(out("stderr"), "C11") != NULL, "naming C11");

    fixture_reset();
    setenv("STUB_PTHREAD", "0", 1);
    EXPECT(run("") == 1, "no pthreads fails");
    EXPECT(strstr(out("stderr"), "POSIX threads") != NULL, "naming threads");

    fixture_reset();
    setenv("STUB_LIBM", "0", 1);
    EXPECT(run("") == 1, "no libm fails");
    EXPECT(strstr(out("stderr"), "libm") != NULL, "naming libm");

    /* The one Linux-specific requirement, and the reason imud does not build
     * on a BSD or a Mac yet. */
    fixture_reset();
    setenv("STUB_BUS", "0", 1);
    EXPECT(run("") == 1, "no i2c-dev/spidev headers fails");
    EXPECT(strstr(out("stderr"), "i2c-dev") != NULL, "naming them");

    /* include/types.h refuses to compile on a big-endian host.  Answering it
     * here turns a #error deep in a header into a named configure result. */
    fixture_reset();
    setenv("STUB_ENDIAN", "big", 1);
    EXPECT(run("") == 1, "a big-endian host fails");
    EXPECT(strstr(out("stderr"), "little-endian") != NULL, "naming byte order");

    /* Several at once must all be reported, not just the first: a porter who
     * has to re-run configure once per missing header learns nothing. */
    fixture_reset();
    setenv("STUB_LIBM", "0", 1);
    setenv("STUB_BUS", "0", 1);
    EXPECT(run("") == 1, "two missing dependencies fail");
    EXPECT(strstr(out("stderr"), "libm") != NULL &&
           strstr(out("stderr"), "i2c-dev") != NULL,
           "and BOTH are listed");
}

/* ARMv6 has no 64-bit exclusive load/store, so gcc emits calls that live in
 * libatomic.  The answer is a link probe because no uname string reports it. */
static void test_atomics(void)
{
    printf("test_atomics\n");

    fixture_reset();
    EXPECT(run("") == 0, "inline atomics configure");
    EXPECT(cfg_is("ATOMIC_LIB", ""), "ATOMIC_LIB is empty");
    EXPECT(strstr(out("stdout"), "inline") != NULL, "the summary says inline");

    fixture_reset();
    setenv("STUB_INLINE_ATOMIC", "0", 1);
    EXPECT(run("") == 0, "an ARMv6-shaped host configures");
    EXPECT(cfg_is("ATOMIC_LIB", "-latomic"), "ATOMIC_LIB = -latomic");

    fixture_reset();
    setenv("STUB_INLINE_ATOMIC", "0", 1);
    setenv("STUB_LATOMIC", "0", 1);
    EXPECT(run("") == 1, "neither inline nor -latomic is fatal");
    EXPECT(strstr(out("stderr"), "atomics") != NULL, "naming atomics");
}

/*
 * Every -l goes AFTER the source on the probe command line.  Debian links with
 * --as-needed, which drops a library that precedes the object referencing it —
 * so a probe with the flag first reports every optional library absent on a
 * host that has it.  That is not hypothetical: the first draft of configure
 * did exactly this and reported libmosquitto missing on this bench, where it
 * is installed and links fine.
 */
static void test_link_order(void)
{
    printf("test_link_order\n");
    fixture_reset();

    EXPECT(run("") == 0, "a plain run configures");

    char path[PATH_MAX], line[1024];
    snprintf(path, sizeof path, "%s/cc.log", g_work);
    FILE *f = fopen(path, "r");
    EXPECT(f != NULL, "the stub compiler logged its arguments");
    if (!f) return;

    int seen = 0, ordered = 0;
    while (fgets(line, sizeof line, f)) {
        const char *src = strstr(line, " -x c - ");
        const char *lib = strstr(line, " -l");
        if (!lib) continue;
        seen++;
        if (src && lib > src) ordered++;
    }
    fclose(f);

    EXPECT(seen >= 2, "at least two probes passed a -l");
    EXPECT(seen == ordered, "every -l came after the source, not before it");
}

/* Nothing in this tier may fail the run: a missing pandoc costs `make
 * docs-texi` and nothing else, and a porter with a bare toolchain must still
 * get a config.mk. */
static void test_optional_never_fatal(void)
{
    printf("test_optional_never_fatal\n");
    fixture_reset();
    setenv("STUB_MOSQUITTO", "0", 1);

    EXPECT(run("") == 0, "a host with nothing but a compiler configures");
    EXPECT(cfg_is("HAVE_MOSQUITTO", "0"), "HAVE_MOSQUITTO = 0");
    EXPECT(cfg_is("HAVE_PANDOC", "0"), "HAVE_PANDOC = 0");
    EXPECT(cfg_is("HAVE_PYTHON3", "0"), "HAVE_PYTHON3 = 0");
    EXPECT(cfg_is("HAVE_TEXENGINE", "0"), "HAVE_TEXENGINE = 0");
    EXPECT(strstr(out("stdout"), "make imud-mqtt") != NULL,
           "the summary says what each miss costs");

    /* Present is reported as present: a stub on the PATH is enough, since the
     * probe is `command -v`. */
    char path[PATH_MAX];
    snprintf(path, sizeof path, "%s/pandoc", g_stub);
    write_exec(path, "#!/bin/sh\nexit 0\n");
    EXPECT(run("") == 0, "adding pandoc changes nothing fatal");
    EXPECT(cfg_is("HAVE_PANDOC", "1"), "HAVE_PANDOC = 1");
    unlink(path);

    fixture_reset();
    EXPECT(run("") == 0, "libmosquitto present configures");
    EXPECT(cfg_is("HAVE_MOSQUITTO", "1"), "HAVE_MOSQUITTO = 1");
}

/* The host runtime issue #45 works through.  Each has a fallback in the tree,
 * so a 0 names a degraded path rather than a broken build. */
static void test_host_runtime(void)
{
    printf("test_host_runtime\n");

    fixture_reset();
    EXPECT(run("") == 0, "a Linux-shaped host configures");
    EXPECT(cfg_is("HAVE_CLOCK_NANOSLEEP", "1"), "clock_nanosleep found");
    EXPECT(cfg_is("HAVE_ADJTIMEX", "1"), "adjtimex found");
    EXPECT(cfg_is("HAVE_ACCEPT4", "1"), "accept4 found");
    EXPECT(cfg_is("HOST_ENDIAN", "little"), "HOST_ENDIAN = little");

    fixture_reset();
    setenv("STUB_CLOCKNS", "0", 1);
    setenv("STUB_ADJTIMEX", "0", 1);
    setenv("STUB_ACCEPT4", "0", 1);
    EXPECT(run("") == 0, "a host missing all three still configures");
    EXPECT(cfg_is("HAVE_CLOCK_NANOSLEEP", "0"), "clock_nanosleep reported absent");
    EXPECT(cfg_is("HAVE_ADJTIMEX", "0"), "adjtimex reported absent");
    EXPECT(cfg_is("HAVE_ACCEPT4", "0"), "accept4 reported absent");

    /* accept4 is the one with teeth: POSIX accept() does not carry
     * FD_CLOEXEC, so a host without it leaks every accepted fd across an
     * exec.  The summary must say so rather than printing a bare 0. */
    EXPECT(strstr(out("stdout"), "close-on-exec") != NULL,
           "the summary says what accept4 buys");
}

/* Install paths: the Makefile's own ?= defaults must lose to config.mk, and a
 * --prefix must reach the four directories derived from it. */
static void test_install_paths(void)
{
    printf("test_install_paths\n");
    fixture_reset();

    EXPECT(run("") == 0, "a default run configures");
    EXPECT(cfg_is("PREFIX", "/usr/local"), "PREFIX defaults to /usr/local");
    EXPECT(cfg_is("LIBDIR", "/usr/local/lib"), "LIBDIR follows it");

    EXPECT(run("--prefix=/usr") == 0, "--prefix configures");
    EXPECT(cfg_is("PREFIX", "/usr"), "PREFIX = /usr");
    EXPECT(cfg_is("LIBDIR", "/usr/lib"), "LIBDIR derived");
    EXPECT(cfg_is("MANDIR", "/usr/share/man"), "MANDIR derived");
    EXPECT(cfg_is("DOCDIR", "/usr/share/doc"), "DOCDIR derived");
    EXPECT(cfg_is("INFODIR", "/usr/share/info"), "INFODIR derived");

    /* /etc is not under the prefix and must not move with it: a package that
     * installs into /usr still reads its config from /etc/imud, and udev
     * never reads rules from a prefix at all. */
    EXPECT(cfg_is("ETCDIR", "/etc/imud"), "ETCDIR ignores the prefix");
    EXPECT(cfg_is("UDEVDIR", "/etc/udev/rules.d"), "UDEVDIR ignores it too");

    EXPECT(run("--prefix=/usr --libdir=/usr/lib/aarch64-linux-gnu") == 0,
           "an explicit --libdir configures");
    EXPECT(cfg_is("LIBDIR", "/usr/lib/aarch64-linux-gnu"),
           "and beats the derived value (the multiarch case debian/rules needs)");

    EXPECT(run("--svcdir=/usr/lib/systemd/system --udevdir=/usr/lib/udev/rules.d "
               "--etcdir=/opt/imud --docdir=/opt/doc --infodir=/opt/info "
               "--mandir=/opt/man") == 0, "every path option configures");
    EXPECT(cfg_is("SVCDIR", "/usr/lib/systemd/system"), "SVCDIR set");
    EXPECT(cfg_is("UDEVDIR", "/usr/lib/udev/rules.d"), "UDEVDIR set");
    EXPECT(cfg_is("ETCDIR", "/opt/imud"), "ETCDIR set");
    EXPECT(cfg_is("DOCDIR", "/opt/doc"), "DOCDIR set");
    EXPECT(cfg_is("INFODIR", "/opt/info"), "INFODIR set");
    EXPECT(cfg_is("MANDIR", "/opt/man"), "MANDIR set");
}

/*
 * config.mk has to be a makefile, not just a text file that looks like one.
 * The Makefile -includes it ahead of everything, so a syntax error there
 * breaks every target at once — including the ones that never compile
 * anything.
 */
static void test_config_mk_is_valid_make(void)
{
    printf("test_config_mk_is_valid_make\n");
    fixture_reset();
    setenv("STUB_GPIOD_VERSION", "2.2.1", 1);

    EXPECT(run("") == 0, "a run to parse the output of");

    /* make itself is not one of configure's dependencies, so it is not on the
     * stub PATH.  Hand this one step the real environment back. */
    setenv("PATH", g_real_path, 1);

    char cmd[PATH_MAX * 2 + 512];
    snprintf(cmd, sizeof cmd,
             "cd '%s' && printf 'include config.mk\\nall:\\n\\t@echo "
             "\"[$(NO_GPIOD)][$(GPIOD_MAJ)][$(ATOMIC_LIB)][$(PREFIX)]\"\\n'"
             " > Makefile.probe && make -f Makefile.probe > make.txt 2>&1",
             g_work);
    EXPECT(system(cmd) == 0, "make parses config.mk without error");

    char path[PATH_MAX], buf[1024] = "";
    snprintf(path, sizeof path, "%s/make.txt", g_work);
    FILE *f = fopen(path, "r");
    if (f) { size_t n = fread(buf, 1, sizeof buf - 1, f); buf[n] = '\0'; fclose(f); }

    EXPECT(strstr(buf, "[0][2][][/usr/local]") != NULL,
           "and every variable reaches a recipe with the value configure wrote");

    /* An empty ATOMIC_LIB is a real answer, not a missing one — which is why
     * the Makefile guards its probes with $(origin), not ?= or ifndef. */
    snprintf(cmd, sizeof cmd,
             "cd '%s' && printf 'include config.mk\\nall:\\n\\t@echo "
             "\"$(origin ATOMIC_LIB)/$(origin GPIOD_MAJ)\"\\n'"
             " > Makefile.probe && make -f Makefile.probe > make.txt 2>&1",
             g_work);
    EXPECT(system(cmd) == 0, "a second probe parses");
    f = fopen(path, "r");
    buf[0] = '\0';
    if (f) { size_t n = fread(buf, 1, sizeof buf - 1, f); buf[n] = '\0'; fclose(f); }
    EXPECT(strstr(buf, "file/file") != NULL,
           "both report origin `file`, so the Makefile skips its own probes");

    setenv("PATH", g_stub, 1);
}

int main(void)
{
    printf("=== test_configure ===\n");

    fixture_init();

    test_usage_contract();
    test_gpiod_v2();
    test_gpiod_v1();
    test_gpiod_absent();
    test_gpiod_forced();
    test_required_failures();
    test_atomics();
    test_link_order();
    test_optional_never_fatal();
    test_host_runtime();
    test_install_paths();
    test_config_mk_is_valid_make();

    printf("\n%d passed, %d failed\n", g_checks - g_fail, g_fail);
    return g_fail ? 1 : 0;
}
