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
 * cat is symlinked from the real PATH and is not under test.  uname IS, and
 * so gets a stub of its own: which init system configure installs a unit for
 * is the one answer here that comes from the host kernel rather than from a
 * link probe, and stubbing it is what makes both the systemd and the launchd
 * branch reachable from one box.
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
static char g_mkbin[PATH_MAX / 2];   /* holds one symlink: make -> GNU make */
/* Sized from its two inputs rather than guessed, so the join provably fits and
 * -Wformat-truncation stays quiet. */
static char g_make_path[sizeof g_real_path + sizeof g_mkbin + 2];

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
    "  *usbdevice_fs.h*)   [ \"${STUB_USBFS:-1}\" = 1 ] || fail usbfs ;;\n"
    "  *IOUSBLib.h*)       [ \"${STUB_IOKIT:-0}\" = 1 ] || fail iokit ;;\n"
    "  *libusb20.h*)       [ \"${STUB_LIBUSB20:-0}\" = 1 ] || fail libusb20 ;;\n"
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

/*
 * The stub uname.  configure asks it only for -s and -m, both of which it
 * writes into config.mk and prints in the summary; -s additionally chooses
 * the init system.  Defaults are a Linux host, so a case sets a variable only
 * when it is about the other one.
 */
static const char *STUB_UNAME =
    "#!/bin/sh\n"
    "for a in \"$@\"; do\n"
    "  case $a in\n"
    "    -s) echo \"${STUB_UNAME_S:-Linux}\" ;;\n"
    "    -m) echo \"${STUB_UNAME_M:-x86_64}\" ;;\n"
    "  esac\n"
    "done\n"
    "[ $# -gt 0 ] || echo \"${STUB_UNAME_S:-Linux}\"\n";

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

/*
 * The recipes under test are GNU make's, and `make` is not GNU make everywhere
 * -- on a BSD it is the host's own, which cannot parse this Makefile at all.
 * So the make-driven cases run with a one-entry directory ahead of the real
 * PATH, holding a `make` that points at the GNU one.  Identical on Linux,
 * where they are already the same binary.
 */
static int find_in_real_path(const char *tool, char *out, size_t n)
{
    if (strchr(tool, '/')) {                   /* already a path */
        if (access(tool, X_OK) != 0) return 0;
        snprintf(out, n, "%s", tool);
        return 1;
    }
    char dirs[sizeof g_real_path];
    snprintf(dirs, sizeof dirs, "%s", g_real_path);
    for (char *save = NULL, *d = strtok_r(dirs, ":", &save); d;
         d = strtok_r(NULL, ":", &save)) {
        char cand[PATH_MAX];
        snprintf(cand, sizeof cand, "%s/%s", d, tool);
        if (access(cand, X_OK) == 0) {
            snprintf(out, n, "%s", cand);
            return 1;
        }
    }
    return 0;
}

/* MAKE is exported by GNU make itself, so `gmake test` names the right one;
 * a suite run by hand falls back to gmake, then to make. */
static void link_gnu_make(void)
{
    const char *env = getenv("MAKE");
    const char *tries[3];
    size_t n = 0;
    if (env && *env) tries[n++] = env;
    tries[n++] = "gmake";
    tries[n++] = "make";

    char src[PATH_MAX], dst[PATH_MAX];
    for (size_t i = 0; i < n; i++) {
        if (!find_in_real_path(tries[i], src, sizeof src)) continue;
        snprintf(dst, sizeof dst, "%s/make", g_mkbin);
        if (symlink(src, dst) != 0 && errno != EEXIST) { perror(dst); exit(2); }
        return;
    }
    fprintf(stderr, "test_configure: no make on this host\n");
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
    snprintf(path, sizeof path, "%s/uname", g_stub);
    write_exec(path, STUB_UNAME);

    link_real("cat");

    snprintf(g_real_path, sizeof g_real_path, "%s", getenv("PATH") ? getenv("PATH") : "");

    snprintf(g_mkbin, sizeof g_mkbin, "%s/mkbin", g_work);
    if (mkdir(g_mkbin, 0755) != 0) { perror(g_mkbin); exit(2); }
    link_gnu_make();
    snprintf(g_make_path, sizeof g_make_path, "%s:%s", g_mkbin, g_real_path);

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
        "STUB_USBFS", "STUB_IOKIT", "STUB_LIBUSB20",
        "STUB_ENDIAN", "STUB_INLINE_ATOMIC", "STUB_LATOMIC", "STUB_MOSQUITTO",
        "STUB_CLOCKNS", "STUB_ADJTIMEX", "STUB_ACCEPT4", "STUB_GPIOD_VERSION",
        "STUB_UNAME_S", "STUB_UNAME_M",
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

/*
 * The whole of `path`, NUL-terminated, in storage that lives until the next
 * call — "" if it cannot be read, so a missing file fails an assertion instead
 * of crashing the suite.
 *
 * Sized from the file rather than from a constant, because `make -n install`
 * prints every build recipe still outstanding ahead of the install rules and
 * so has no bound: 10 KB with the tree built, 32 KB with nothing built.  A
 * fixed buffer truncated it, which failed the positive assertions in whichever
 * job had built the least and passed the negative ones from any build state.
 */
static const char *slurp(const char *path)
{
    static char *buf;
    struct stat st;
    FILE *f;
    size_t n;

    free(buf);
    buf = NULL;
    if (!(f = fopen(path, "r")))
        return "";
    if (fstat(fileno(f), &st) != 0 || st.st_size < 0
        || !(buf = malloc((size_t)st.st_size + 1))) {
        fclose(f);
        return "";
    }
    n = fread(buf, 1, (size_t)st.st_size, f);
    buf[n] = '\0';
    fclose(f);
    return buf;
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

/*
 * The bus backend, on the same terms as the GPIO one above.
 *
 * A host with no i2c-dev and no spidev must CONFIGURE, not fail: src/bus_null.c
 * keeps open() and close() and fails only the transfers, which still runs the
 * sim driver end to end.  That is the state a BSD or a Mac port starts from,
 * and configure refusing it would be the thing standing in the way.
 */
static void test_bus_absent(void)
{
    printf("test_bus_absent\n");
    fixture_reset();
    setenv("STUB_BUS", "0", 1);

    EXPECT(run("") == 0, "a host with no i2c-dev/spidev still configures");
    EXPECT(cfg_is("NO_LINUX_BUS", "1"), "NO_LINUX_BUS = 1 drops the Linux backend");

    /* usbfs alone is still a usable bus, so the null backend is only reached
     * when BOTH are gone -- which is the case the sim line is about. */
    EXPECT(cfg_is("NO_FT232H", "0"), "the FT232H backend carries the build");
    EXPECT(strstr(out("stdout"), "ftdi:") != NULL,
           "and the summary says every node must be an ftdi: one");

    fixture_reset();
    setenv("STUB_BUS", "0", 1);
    setenv("STUB_USBFS", "0", 1);
    EXPECT(run("") == 0, "a host with neither still configures");
    EXPECT(cfg_is("NO_FT232H", "1"), "NO_FT232H = 1 too");
    EXPECT(strstr(out("stdout"), "null") != NULL,
           "and the summary says the backend is null");
    EXPECT(strstr(out("stdout"), "sim") != NULL,
           "naming the one driver that still runs");

    /* Present is the ordinary case, and must stay the default. */
    fixture_reset();
    EXPECT(run("") == 0, "a host with them configures");
    EXPECT(cfg_is("NO_LINUX_BUS", "0"), "with the Linux backend");
    EXPECT(cfg_is("NO_FT232H", "0"), "and the FT232H beside it");
}

/*
 * Which include/ft_usb.h rung the bridge is built on is decided by the same
 * probe that decides whether to build it at all, because nothing else
 * distinguishes the two: usbfs where the kernel header is, IOKit where the
 * framework is.  A macOS host has the second and not the first.
 */
static void test_ft232h_rung_follows_the_host(void)
{
    printf("test_ft232h_rung_follows_the_host\n");

    fixture_reset();
    EXPECT(run("") == 0, "a Linux-shaped host configures");
    EXPECT(cfg_is("FT_USB_SRC", "src/ft_usb_linux.c"), "and takes usbfs");
    EXPECT(cfg_is("FT_USB_LIB", ""), "which needs no library");

    fixture_reset();
    setenv("STUB_BUS", "0", 1);
    setenv("STUB_USBFS", "0", 1);
    setenv("STUB_IOKIT", "1", 1);
    EXPECT(run("") == 0, "a macOS-shaped host configures");
    EXPECT(cfg_is("NO_FT232H", "0"), "with the bridge built");
    EXPECT(cfg_is("FT_USB_SRC", "src/ft_usb_darwin.c"), "on the IOKit rung");
    /* The frameworks are not optional there: without them the link fails on
     * every IOKit symbol, and nothing else in the build would supply them. */
    EXPECT(cfg_is("FT_USB_LIB", "-framework IOKit -framework CoreFoundation"),
           "naming the frameworks that rung has to link");
    EXPECT(strstr(out("stdout"), "IOKit") != NULL, "and the summary says so");

    /* --with-ft232h must accept EITHER rung, not just the Linux one. */
    fixture_reset();
    setenv("STUB_USBFS", "0", 1);
    setenv("STUB_IOKIT", "1", 1);
    EXPECT(run("--with-ft232h") == 0, "--with-ft232h is satisfied by IOKit");

    fixture_reset();
    setenv("STUB_BUS", "0", 1);
    setenv("STUB_USBFS", "0", 1);
    setenv("STUB_LIBUSB20", "1", 1);
    EXPECT(run("") == 0, "a FreeBSD-shaped host configures");
    EXPECT(cfg_is("NO_FT232H", "0"), "with the bridge built");
    EXPECT(cfg_is("FT_USB_SRC", "src/ft_usb_freebsd.c"), "on the libusb20 rung");
    /* libusb20 is in FreeBSD base, but it is still a library to link: the
     * usbfs rung is the only one that needs nothing. */
    EXPECT(cfg_is("FT_USB_LIB", "-lusb"), "naming the library that rung links");
    EXPECT(strstr(out("stdout"), "libusb20") != NULL, "and the summary says so");

    fixture_reset();
    setenv("STUB_USBFS", "0", 1);
    setenv("STUB_LIBUSB20", "1", 1);
    EXPECT(run("--with-ft232h") == 0, "--with-ft232h is satisfied by libusb20");

    fixture_reset();
    setenv("STUB_USBFS", "0", 1);
    EXPECT(run("--with-ft232h") == 1, "and fails when none of the three is there");
}

/*
 * Byte order is reported, never required.  The wire packet and .imucap are
 * converted field by field, so a big-endian host builds and emits the same
 * little-endian bytes — configure must say which host it found and carry on.
 */
static void test_endianness_is_reported(void)
{
    printf("test_endianness_is_reported\n");

    fixture_reset();
    setenv("STUB_ENDIAN", "big", 1);
    EXPECT(run("") == 0, "a big-endian host configures");
    EXPECT(cfg_is("HOST_ENDIAN", "big"), "HOST_ENDIAN = big in config.mk");
    EXPECT(strstr(out("stdout"), "big-endian") != NULL,
           "and the summary names the host byte order");
    EXPECT(strstr(out("stderr"), "little-endian") == NULL,
           "with nothing on stderr demanding little-endian");

    fixture_reset();
    EXPECT(run("") == 0, "a little-endian host configures");
    EXPECT(cfg_is("HOST_ENDIAN", "little"), "HOST_ENDIAN = little in config.mk");
}

static void test_bus_forced(void)
{
    printf("test_bus_forced\n");
    fixture_reset();

    EXPECT(run("--without-linux-bus") == 0, "--without-linux-bus configures");
    EXPECT(cfg_is("NO_LINUX_BUS", "1"), "and takes the null backend anyway");

    /* Same asymmetry as --with-gpiod: asking for the Linux bus and silently
     * getting the null one would hand back a build that cannot reach a sensor. */
    fixture_reset();
    setenv("STUB_BUS", "0", 1);
    EXPECT(run("--with-linux-bus") == 1, "--with-linux-bus fails when absent");
    EXPECT(strstr(out("stderr"), "i2c-dev") != NULL, "saying so on stderr");
    EXPECT(!wrote_config_mk(), "and writes no config.mk");

    fixture_reset();
    EXPECT(run("--with-linux-bus") == 0, "--with-linux-bus succeeds when present");
    EXPECT(cfg_is("NO_LINUX_BUS", "0"), "with the Linux backend");
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

    /* NOT here any more: i2c-dev and spidev are a backend CHOICE, not a
     * requirement — see test_bus_absent below. */

    /* Several at once must all be reported, not just the first: a porter who
     * has to re-run configure once per missing header learns nothing. */
    fixture_reset();
    setenv("STUB_LIBM", "0", 1);
    setenv("STUB_PTHREAD", "0", 1);
    EXPECT(run("") == 1, "two missing dependencies fail");
    EXPECT(strstr(out("stderr"), "libm") != NULL &&
           strstr(out("stderr"), "POSIX threads") != NULL,
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
    EXPECT(cfg_is("HOST_TIME_SRC", "src/host_time_linux.c"),
           "the top rung on a host with both");

    fixture_reset();
    setenv("STUB_CLOCKNS", "0", 1);
    setenv("STUB_ADJTIMEX", "0", 1);
    setenv("STUB_ACCEPT4", "0", 1);
    EXPECT(run("") == 0, "a host missing all three still configures");
    EXPECT(cfg_is("HAVE_CLOCK_NANOSLEEP", "0"), "clock_nanosleep reported absent");
    EXPECT(cfg_is("HAVE_ADJTIMEX", "0"), "adjtimex reported absent");
    EXPECT(cfg_is("HAVE_ACCEPT4", "0"), "accept4 reported absent");
    EXPECT(cfg_is("HOST_TIME_SRC", "src/host_time_fallback.c"),
           "the bottom rung with no clock selection");

    /* accept4 is the one with teeth: POSIX accept() does not carry
     * FD_CLOEXEC, so a host without it leaks every accepted fd across an
     * exec.  The summary must say so rather than printing a bare 0.
     * Read here, against the run that has all three missing. */
    EXPECT(strstr(out("stdout"), "close-on-exec") != NULL,
           "the summary says what accept4 buys");

    /* The middle rung — clock selection but no TAI, which is every BSD. It is
     * reachable only from this combination, so a script that collapsed the two
     * probes into one would pass everything above and still never build it. */
    fixture_reset();
    setenv("STUB_ADJTIMEX", "0", 1);
    EXPECT(run("") == 0, "a BSD-shaped host configures");
    EXPECT(cfg_is("HAVE_CLOCK_NANOSLEEP", "1"), "clock_nanosleep found");
    EXPECT(cfg_is("HAVE_ADJTIMEX", "0"), "adjtimex absent");
    EXPECT(cfg_is("HOST_TIME_SRC", "src/host_time_posix.c"),
           "the middle rung with clock selection but no TAI");

    /* --with-host-time is the affordance for a host none of the three fits:
     * one file against include/host_time.h, named here, nothing in the script
     * to patch first. It must not have to be a rung this script knows. */
    fixture_reset();
    EXPECT(run("--with-host-time=src/host_time_fallback.c") == 0,
           "--with-host-time configures");
    EXPECT(cfg_is("HOST_TIME_SRC", "src/host_time_fallback.c"),
           "and overrides what the probes chose");

    fixture_reset();
    EXPECT(run("--with-host-time=src/host_time_nonexistent.c") != 0,
           "--with-host-time refuses a file that is not there");
    EXPECT(strstr(out("stderr"), "no such file") != NULL,
           "saying so, rather than failing later at the link");
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

    /* DATADIR is where install-wmm-data writes and where the daemon's WMM
     * auto-resolver looks, so it has to follow the prefix — it used to be a
     * literal /usr/share on both sides, which a default source install at
     * PREFIX=/usr/local never reached. */
    fixture_reset();
    EXPECT(run("--prefix=/opt/imud") == 0, "--prefix configures");
    EXPECT(cfg_is("DATADIR", "/opt/imud/share"), "DATADIR follows the prefix");
    EXPECT(run("--datadir=/opt/shared") == 0, "--datadir configures");
    EXPECT(cfg_is("DATADIR", "/opt/shared"), "and beats the derived value");

    /* RUNDIR and STATEDIR are the Makefile's to derive from the init system,
     * so config.mk names them ONLY when asked: an assignment here would beat
     * the ?= there for every build that never passed the option. */
    fixture_reset();
    EXPECT(run("") == 0, "a default run configures");
    EXPECT(cfg("RUNDIR") == NULL, "RUNDIR unset unless named");
    EXPECT(cfg("STATEDIR") == NULL, "STATEDIR unset unless named");
    EXPECT(run("--rundir=/opt/imud/run --statedir=/opt/imud/var") == 0,
           "both configure");
    EXPECT(cfg_is("RUNDIR", "/opt/imud/run"), "RUNDIR set");
    EXPECT(cfg_is("STATEDIR", "/opt/imud/var"), "STATEDIR set");
}

/*
 * The five directories reach the BINARY, not just the install rules.
 *
 * Every default path the tree names used to be a literal, so a build under a
 * prefix that is not /usr looked for its config, calibration, sockets and WMM
 * data where a Debian package would have put them — issue #78, and #77 for
 * the WMM half.  include/paths.h derives all of them from the -D block the
 * Makefile writes.
 *
 * Compiled, not preprocessed: adjacent string literals are joined after cpp
 * has run, so `IMUD_ETCDIR "/imud.conf"` is still two tokens in -E output.
 * The object's own strings are the constants the daemon would actually use.
 */
static void test_prefix_reaches_the_binary(void)
{
    printf("test_prefix_reaches_the_binary\n");
    fixture_reset();
    setenv("PATH", g_make_path, 1);

    char root[PATH_MAX], cmd[PATH_MAX * 3 + 1024];
    snprintf(root, sizeof root, "%s", g_configure);
    char *slash = strrchr(root, '/');
    if (slash) *slash = '\0';

    static const char *SRCS[] = { "src/cli.c", "src/config.c", "src/main.c",
                                  "src/signalk_main.c", "lib/libimud.c" };
    for (size_t i = 0; i < sizeof SRCS / sizeof SRCS[0]; i++) {
        const char *base = strrchr(SRCS[i], '/') + 1;
        snprintf(cmd, sizeof cmd,
                 "cd '%s' && cc -c -std=c11 -pthread -D_GNU_SOURCE "
                 "-Iinclude -Ilib "
                 "-DIMUD_PREFIX='\"/opt/imud\"' -DIMUD_ETCDIR='\"/opt/imud/etc\"' "
                 "-DIMUD_DATADIR='\"/opt/imud/share\"' "
                 "-DIMUD_RUNDIR='\"/opt/imud/run\"' "
                 "-DIMUD_STATEDIR='\"/opt/imud/var\"' "
                 "-o '%s/%.*so' %s 2>/dev/null",
                 root, g_work, (int)(strlen(base) - 1), base, SRCS[i]);
        EXPECT(system(cmd) == 0, "the prefixed object compiles");
    }

    /* Present in <obj>'s constants. */
    #define OBJ_HAS(obj, pat)                                                 \
        (snprintf(cmd, sizeof cmd,                                            \
                  "strings '%s/%s.o' | grep -qF -- '%s'", g_work, (obj), (pat)), \
         system(cmd) == 0)
    /* A stock path that is NOT part of a /opt/imud one — the substring trap:
     * "/opt/imud/etc/imud.conf" contains "/etc/imud" outright. */
    #define OBJ_HAS_STOCK(obj, pat)                                           \
        (snprintf(cmd, sizeof cmd,                                            \
                  "strings '%s/%s.o' | grep -F -- '%s' | grep -qvF /opt/imud", \
                  g_work, (obj), (pat)),                                      \
         system(cmd) == 0)

    EXPECT(OBJ_HAS("cli", "/opt/imud/etc/imud.conf"),
           "cli.c: the four tools' default config follows ETCDIR");
    EXPECT(!OBJ_HAS_STOCK("cli", "/etc/imud/imud.conf"),
           "and no /etc/imud literal survives, in a default or a usage line");
    EXPECT(OBJ_HAS("cli", "/opt/imud/run/imud.sock"),
           "imud-status's default socket follows RUNDIR — it and the daemon "
           "were both literals, so a launchd install's /var/run reached neither");

    EXPECT(OBJ_HAS("config", "/opt/imud/etc/cal.json"),
           "config.c: cal.json follows ETCDIR");
    EXPECT(OBJ_HAS("config", "/opt/imud/share/imud/WMM.COF"),
           "WMM package data follows DATADIR");
    EXPECT(!OBJ_HAS_STOCK("config", "/usr/share/imud/WMM.COF"),
           "and the /usr/share literal is gone — that is issue #77");
    EXPECT(OBJ_HAS("config", "/opt/imud/run/imud-stream.sock"),
           "the stream socket follows RUNDIR");
    EXPECT(OBJ_HAS("config", "/opt/imud/var"),
           "and the capture directory follows STATEDIR");

    EXPECT(OBJ_HAS("main", "/opt/imud/run/imud.pid"),
           "main.c: the PID file follows RUNDIR");
    EXPECT(OBJ_HAS("main", "/opt/imud/run/imud.sock"),
           "and so does the status socket the daemon binds");

    EXPECT(OBJ_HAS("signalk_main", "/opt/imud/etc/imud-signalk.conf"),
           "each bridge's own config follows ETCDIR");
    EXPECT(OBJ_HAS("libimud", "/opt/imud/run/imud-stream.sock"),
           "libimud's default socket follows it too");

    /* Vendored out of the tree there is no -D and no include/paths.h, so the
     * fallback has to still be the Linux path. */
    snprintf(cmd, sizeof cmd,
             "cd '%s' && cc -c -std=c11 -D_GNU_SOURCE -Iinclude -Ilib "
             "-o '%s/vendored.o' lib/libimud.c 2>/dev/null", root, g_work);
    EXPECT(system(cmd) == 0, "libimud compiles with no -D at all");
    EXPECT(OBJ_HAS("vendored", "/run/imud/imud-stream.sock"),
           "and falls back to the Linux default, as a vendored copy must");

    #undef OBJ_HAS
    #undef OBJ_HAS_STOCK
    setenv("PATH", g_stub, 1);
}

/*
 * --with-service=none: no unit, no state directory.  Homebrew defines the
 * service in the formula and refuses writes outside its prefix, so an install
 * that insists on /Library/LaunchDaemons and /var/db/imud cannot be packaged.
 */
static void test_service_none(void)
{
    printf("test_service_none\n");

    fixture_reset();
    EXPECT(run("--with-service=none") == 0, "--with-service=none configures");
    EXPECT(cfg_is("SVC_KIND", "none"), "and records it");
    EXPECT(strstr(out("stdout"), "none — no unit, no state directory") != NULL,
           "with the summary saying what that costs");

    /* Nothing else moves: a prefixed install still needs its own etc. */
    fixture_reset();
    EXPECT(run("--prefix=/opt/imud --etcdir=/opt/imud/etc --with-service=none "
               "--rundir=/opt/imud/run --statedir=/opt/imud/var") == 0,
           "the whole Homebrew-shaped invocation configures");
    EXPECT(cfg_is("SVC_KIND", "none"), "SVC_KIND none");
    EXPECT(cfg_is("PREFIX", "/opt/imud"), "PREFIX set");
    EXPECT(cfg_is("ETCDIR", "/opt/imud/etc"), "ETCDIR set");
    EXPECT(cfg_is("DATADIR", "/opt/imud/share"), "DATADIR derived");

    /* ── the Makefile's half ─────────────────────────────────────────────── */
    setenv("PATH", g_make_path, 1);

    char root[PATH_MAX], cmd[PATH_MAX * 2 + 512], path[PATH_MAX];
    snprintf(root, sizeof root, "%s", g_configure);
    char *slash = strrchr(root, '/');
    if (slash) *slash = '\0';

    snprintf(cmd, sizeof cmd,
             "cd '%s' && make -n install SVC_KIND=none SVCDIR=/tmp/svc "
             "DESTDIR=/tmp/imud-nonexistent > '%s/mk.txt' 2>&1",
             root, g_work);
    EXPECT(system(cmd) == 0, "make -n install parses under SVC_KIND=none");

    /* grep rather than a fixed buffer: these are absence checks, and a read
     * that truncated the recipe would pass every one of them for free. */
    snprintf(path, sizeof path, "%s/mk.txt", g_work);
    #define MK_HAS(pat)                                                       \
        (snprintf(cmd, sizeof cmd, "grep -qF -- '%s' '%s'", (pat), path),     \
         system(cmd) == 0)

    /* On the DIRECTORY, not on a unit filename: with svc-dst empty a partial
     * regression still emits `install -m 644  /tmp/svc/`, which every check
     * for "imud.service" passes for the wrong reason. */
    EXPECT(!MK_HAS("/tmp/svc"), "SVCDIR is not written to at all");
    EXPECT(!MK_HAS("install -d -m 0750 /tmp/imud-nonexistent/var"),
           "and no state directory");
    EXPECT(MK_HAS("No service unit installed"), "the recipe says so");
    EXPECT(MK_HAS("/tmp/imud-nonexistent/usr/local/bin"),
           "while the binaries still install");

    /* The five bridges take the same arm — one of them stands for all five,
     * since install-svc is a single definition. */
    snprintf(cmd, sizeof cmd,
             "cd '%s' && make -n install-signalk SVC_KIND=none SVCDIR=/tmp/svc "
             "DESTDIR=/tmp/imud-nonexistent > '%s/mk.txt' 2>&1",
             root, g_work);
    EXPECT(system(cmd) == 0, "make -n install-signalk parses too");
    EXPECT(!MK_HAS("/tmp/svc"), "and installs no unit either");

    /* install-wmm-data is the other half of #77: it wrote $(PREFIX)/share
     * while the daemon looked in /usr/share.  DATADIR is passed rather than
     * derived from PREFIX, for the same reason SVCDIR is above — configure
     * writes it into config.mk as a plain assignment, which the repo may or
     * may not have, and that beats the Makefile's ?= for a later command-line
     * PREFIX.  That DATADIR itself follows the prefix is test_install_paths'. */
    snprintf(cmd, sizeof cmd,
             "cd '%s' && make -n install-wmm-data DATADIR=/opt/imud/share "
             "DESTDIR=/tmp/imud-nonexistent > '%s/mk.txt' 2>&1",
             root, g_work);
    EXPECT(system(cmd) == 0, "make -n install-wmm-data parses");
    /* The whole install command, not the path: the recipe echoes the
     * destination as well, and matching that passes while the copy itself
     * still goes to $(PREFIX)/share. */
    EXPECT(MK_HAS("install -m 644 data/WMM.COF "
                  "/tmp/imud-nonexistent/opt/imud/share/imud/WMM.COF"),
           "WMM.COF lands in DATADIR, where the resolver now looks");

    #undef MK_HAS
    setenv("PATH", g_stub, 1);
}

/*
 * The Homebrew formulae, held against the configure they drive.
 *
 * packaging/homebrew/imud.rb is READ rather than restated here, so a flag
 * renamed on one side and not the other fails in this suite rather than on
 * someone's Mac, where nothing in this tree can see it.
 *
 * The second half is issue #88: under --with-service=none the install must
 * reach for neither systemctl nor ldconfig.  Neither works unprivileged, and
 * the failure aborted `make install` rather than being skipped — which is
 * every prefix install from source, Homebrew's included.
 */
#define BREW_PREFIX "/opt/brewtest"

/* The formula's Ruby interpolations, resolved to a prefix a test can name. */
static void brew_expand(char *dst, size_t n, const char *src)
{
    static const struct { const char *from, *to; } SUB[] = {
        { "#{HOMEBREW_PREFIX}", BREW_PREFIX },
        { "#{etc}",             BREW_PREFIX "/etc" },
        { "#{var}",             BREW_PREFIX "/var" },
    };
    size_t o = 0, nsub = sizeof SUB / sizeof SUB[0];

    for (size_t i = 0; src[i] && o + 1 < n; ) {
        size_t k;
        for (k = 0; k < nsub; k++) {
            size_t fl = strlen(SUB[k].from);
            if (strncmp(src + i, SUB[k].from, fl) != 0) continue;
            for (const char *t = SUB[k].to; *t && o + 1 < n; t++) dst[o++] = *t;
            i += fl;
            break;
        }
        if (k == nsub) dst[o++] = src[i++];
    }
    dst[o] = '\0';
}

/*
 * The --flags of the formula's ./configure call, expanded and space-joined.
 * Line-wise and bounded to that one statement: "--config" appears again in
 * the service and test blocks, and a scan of the whole file would collect it.
 */
static int brew_configure_args(const char *rb, char *args, size_t n)
{
    FILE *f = fopen(rb, "r");
    if (!f) return 0;

    char line[1024];
    int started = 0;
    size_t o = 0;

    args[0] = '\0';
    while (fgets(line, sizeof line, f)) {
        if (!started) {
            if (!strstr(line, "\"./configure\"")) continue;
            started = 1;
        }
        const char *p = strstr(line, "\"--");
        if (!p) break;                  /* the statement ended with the last flag */
        const char *q = strchr(++p, '"');
        if (!q) break;

        char raw[512], exp[1024];
        size_t len = (size_t)(q - p);
        if (len >= sizeof raw) break;
        memcpy(raw, p, len);
        raw[len] = '\0';
        brew_expand(exp, sizeof exp, raw);

        int w = snprintf(args + o, n - o, "%s%s", o ? " " : "", exp);
        if (w < 0 || (size_t)w >= n - o) break;
        o += (size_t)w;
    }
    fclose(f);
    return started && o > 0;
}

static void test_homebrew_formula(void)
{
    printf("test_homebrew_formula\n");
    fixture_reset();

    char root[PATH_MAX], rb[PATH_MAX], args[2048];
    snprintf(root, sizeof root, "%s", g_configure);
    char *slash = strrchr(root, '/');
    if (slash) *slash = '\0';
    snprintf(rb, sizeof rb, "%s/packaging/homebrew/imud.rb", root);

    EXPECT(brew_configure_args(rb, args, sizeof args),
           "imud.rb names a ./configure call with flags");
    EXPECT(strstr(args, "--with-service=none") != NULL,
           "and one of them is --with-service=none");

    EXPECT(run(args) == 0,
           "every flag the formula passes is one configure takes");
    EXPECT(cfg_is("PREFIX", BREW_PREFIX), "PREFIX is Homebrew's own prefix");
    EXPECT(cfg_is("ETCDIR", BREW_PREFIX "/etc/imud"), "ETCDIR under it");
    EXPECT(cfg_is("RUNDIR", BREW_PREFIX "/var/run"), "RUNDIR under it");
    EXPECT(cfg_is("STATEDIR", BREW_PREFIX "/var/imud"), "STATEDIR under it");
    EXPECT(cfg_is("SVC_KIND", "none"), "and no init system is claimed");

    /* ── what that install actually runs ─────────────────────────────────── */
    setenv("PATH", g_make_path, 1);

    char cmd[PATH_MAX * 3 + 1024], path[PATH_MAX], want[PATH_MAX];
    snprintf(path, sizeof path, "%s/mk.txt", g_work);

    #define MK(target, extra)                                                 \
        (snprintf(cmd, sizeof cmd,                                            \
                  "cd '%s' && make -n %s PREFIX=" BREW_PREFIX                 \
                  " ETCDIR=" BREW_PREFIX "/etc/imud %s"                       \
                  " DESTDIR='%s/stage' > '%s' 2>&1",                          \
                  root, (target), (extra), g_work, path),                     \
         system(cmd) == 0)
    #define MK_HAS(pat)                                                       \
        (snprintf(cmd, sizeof cmd, "grep -qF -- '%s' '%s'", (pat), path),     \
         system(cmd) == 0)

    EXPECT(MK("install", "SVC_KIND=none"),
           "make -n install parses with the formula's paths");

    /* Issue #88, both directions.  The absence alone would also pass if the
     * grep were simply looking in the wrong place, so the systemd run below
     * proves `none` is what removes the call. */
    /* The commands, not the words: `make -n` echoes a recipe's comments too,
     * and the block below is introduced by one naming useradd. */
    EXPECT(!MK_HAS("systemctl"), "no systemctl under --with-service=none");
    EXPECT(!MK_HAS("useradd --system"), "and no system user is created");
    EXPECT(!MK_HAS("/etc/udev"), "and no udev rule is written");
    EXPECT(MK_HAS("[ \"$(id -u)\" = 0 ]"),
           "while ldconfig is gated on being root");

    snprintf(want, sizeof want, "%s/stage" BREW_PREFIX "/bin", g_work);
    EXPECT(MK_HAS(want), "the binaries still install, under the staged prefix");
    snprintf(want, sizeof want,
             "%s/stage" BREW_PREFIX "/etc/imud/imud.conf", g_work);
    EXPECT(MK_HAS(want), "and the config lands in the formula's etcdir");

    EXPECT(MK("install-signalk", "SVC_KIND=none"),
           "make -n install-signalk parses too");
    EXPECT(!MK_HAS("systemctl"), "and reaches for no systemctl either");

    EXPECT(MK("install", "SVC_KIND=systemd SVCDIR=/tmp/svc"),
           "make -n install parses under systemd");
    EXPECT(MK_HAS("systemctl"),
           "where the reload IS the right thing, so `none` is what removed it");

    #undef MK
    #undef MK_HAS
    setenv("PATH", g_stub, 1);
}

/*
 * Which service unit gets installed is the host's answer, not something the
 * caller has to remember: a Mac gets a launchd job in /Library/LaunchDaemons,
 * everything else a systemd unit.  The stub uname above is what makes both
 * branches reachable from one box.
 *
 * The second half asserts the MAKEFILE consumes the answer.  configure
 * writing SVC_KIND and the install rules reading it are separate facts, and
 * the failure that matters is a correct config.mk installing the wrong file —
 * which no assertion about config.mk alone can see.
 */
static void test_service_unit_follows_the_host(void)
{
    printf("test_service_unit_follows_the_host\n");

    fixture_reset();
    EXPECT(run("") == 0, "a Linux host configures");
    EXPECT(cfg_is("SVC_KIND", "systemd"), "and takes systemd");
    EXPECT(cfg_is("SVCDIR", "/etc/systemd/system"), "into /etc/systemd/system");
    EXPECT(strstr(out("stdout"), "systemd — /etc/systemd/system") != NULL,
           "with the summary naming the kind and the directory");
    EXPECT(strstr(out("stdout"), "udev rules") != NULL,
           "and offering a udev directory");

    fixture_reset();
    setenv("STUB_UNAME_S", "Darwin", 1);
    EXPECT(run("") == 0, "a Darwin host configures");
    EXPECT(cfg_is("SVC_KIND", "launchd"), "and takes launchd unaided");
    EXPECT(cfg_is("SVCDIR", "/Library/LaunchDaemons"),
           "into /Library/LaunchDaemons");
    EXPECT(strstr(out("stdout"), "launchd — /Library/LaunchDaemons") != NULL,
           "with the summary saying so");
    /* udev is Linux's.  A Mac has no equivalent and `make install` writes no
     * rule there, so naming a directory for one is a promise nothing keeps. */
    EXPECT(strstr(out("stdout"), "udev rules") == NULL,
           "and no udev directory at all");

    /* An explicit --with-service moves the default directory with it ... */
    fixture_reset();
    EXPECT(run("--with-service=launchd") == 0, "--with-service=launchd configures");
    EXPECT(cfg_is("SVC_KIND", "launchd"), "overriding the detected systemd");
    EXPECT(cfg_is("SVCDIR", "/Library/LaunchDaemons"), "and moving SVCDIR with it");

    fixture_reset();
    setenv("STUB_UNAME_S", "Darwin", 1);
    EXPECT(run("--with-service=systemd") == 0, "a Mac can be told systemd");
    EXPECT(cfg_is("SVC_KIND", "systemd"), "and takes it");
    EXPECT(cfg_is("SVCDIR", "/etc/systemd/system"), "with the systemd directory");

    /* ... but never past an explicit --svcdir.  A packager naming both means
     * both, and this is the pair debian/rules would pass. */
    fixture_reset();
    EXPECT(run("--with-service=launchd --svcdir=/opt/units") == 0,
           "both options together configure");
    EXPECT(cfg_is("SVC_KIND", "launchd"), "kind from --with-service");
    EXPECT(cfg_is("SVCDIR", "/opt/units"), "directory from --svcdir");

    fixture_reset();
    EXPECT(run("--with-service=upstart") == 1, "an unknown init system fails");
    EXPECT(strstr(out("stderr"), "upstart") != NULL, "naming it on stderr");
    EXPECT(!wrote_config_mk(), "and writes no config.mk");

    /* ── the Makefile's half ─────────────────────────────────────────────── */
    setenv("PATH", g_make_path, 1);

    char root[PATH_MAX], cmd[PATH_MAX * 2 + 512], path[PATH_MAX];
    snprintf(root, sizeof root, "%s", g_configure);
    char *slash = strrchr(root, '/');
    if (slash) *slash = '\0';

    /* -n prints the recipe and builds nothing, so this reads the install
     * rules without touching a file.  SVCDIR is passed rather than left to
     * config.mk, which the repo may or may not have. */
    for (int launchd = 0; launchd < 2; launchd++) {
        snprintf(cmd, sizeof cmd,
                 "cd '%s' && make -n install SVC_KIND=%s SVCDIR=/tmp/svc "
                 "DESTDIR=/tmp/imud-nonexistent > '%s/mk.txt' 2>&1",
                 root, launchd ? "launchd" : "systemd", g_work);
        EXPECT(system(cmd) == 0, "make -n install parses");

        snprintf(path, sizeof path, "%s/mk.txt", g_work);
        const char *buf = slurp(path);

        if (launchd) {
            EXPECT(strstr(buf, "/tmp/svc/io.github.richcreations.imud.plist")
                   != NULL,
                   "SVC_KIND=launchd installs the plist under its label");
            EXPECT(strstr(buf, "/tmp/svc/imud.service") == NULL,
                   "and not the systemd unit");
            /* RUNDIR follows the same switch: macOS has no /run, so the
             * installed config's AF_UNIX paths have to move with it. */
            EXPECT(strstr(buf, "s|/run/imud|/var/run|g") != NULL,
                   "and rewrites the config's runtime paths to /var/run");
        } else {
            EXPECT(strstr(buf, "/tmp/svc/imud.service") != NULL,
                   "SVC_KIND=systemd installs the unit");
            /* Matched on the installed path, not on ".plist": the recipe's
             * own comments name etc/imud.plist.in, and make -n echoes them. */
            EXPECT(strstr(buf, "/tmp/svc/io.github") == NULL,
                   "and no launchd job");
            EXPECT(strstr(buf, "s|/run/imud|/run/imud|g") != NULL,
                   "leaving the config's runtime paths alone");
        }
    }

    setenv("PATH", g_stub, 1);
}

/*
 * uninstall has to STOP a service before it removes the unit file, or the init
 * system is left holding a job whose file is gone.  The launchd half was
 * missing entirely: the plist was deleted while the job stayed bootstrapped.
 *
 * Answerable here rather than only on a Mac, because `make -n` echoes an
 * @-prefixed recipe -- so the branch a Linux box never runs is still readable.
 */
static void test_uninstall_stops_the_service(void)
{
    printf("test_uninstall_stops_the_service\n");

    setenv("PATH", g_make_path, 1);

    char root[PATH_MAX], cmd[PATH_MAX * 2 + 512], path[PATH_MAX];
    snprintf(root, sizeof root, "%s", g_configure);
    char *slash = strrchr(root, '/');
    if (slash) *slash = '\0';

    static const char *kinds[] = { "launchd", "systemd", "none" };

    for (unsigned i = 0; i < sizeof kinds / sizeof kinds[0]; i++) {
        snprintf(cmd, sizeof cmd,
                 "cd '%s' && make -n uninstall SVC_KIND=%s SVCDIR=/tmp/svc "
                 "DESTDIR=/tmp/imud-nonexistent > '%s/mk.txt' 2>&1",
                 root, kinds[i], g_work);
        EXPECT(system(cmd) == 0, "make -n uninstall parses");

        snprintf(path, sizeof path, "%s/mk.txt", g_work);
        const char *buf = slurp(path);

        int boots_out = strstr(buf, "launchctl bootout") != NULL;
        int disables  = strstr(buf, "systemctl disable") != NULL;

        if (strcmp(kinds[i], "launchd") == 0) {
            EXPECT(boots_out, "SVC_KIND=launchd boots the job out");
            EXPECT(!disables, "and does not reach for systemctl");
            EXPECT(strstr(buf, "bootout system/io.github.richcreations.$n")
                   != NULL, "addressing it by label");
            /*
             * The trap this test exists for: bootout takes the LABEL, and
             * svc-dst is the FILENAME -- the same string plus ".plist".
             * Passing the filename is refused, and the 2>/dev/null that lets
             * an unloaded job pass would swallow that too.
             */
            EXPECT(strstr(buf, "bootout system/io.github.richcreations.$n.plist")
                   == NULL, "never the .plist filename");
            /*
             * Over every name in SVC_NAMES, not just imud.  Pinned in full so
             * a seventh bridge added to that list fails here rather than being
             * installed and then never stopped.
             */
            EXPECT(strstr(buf, "for n in imud imud-signalk imud-mqtt "
                               "imud-influxdb imud-prometheus imud-mavlink")
                   != NULL, "for every service, not just the daemon");
            /*
             * bootout returns while the job is still exiting, so without a
             * wait the job is still listed the instant uninstall returns --
             * which is the state issue #86 reports.  Measured on macOS: still
             * there immediately after, gone moments later.
             */
            EXPECT(strstr(buf, "launchctl print system/$(LAUNCHD_PREFIX)")
                   != NULL
                   || strstr(buf, "launchctl print system/io.github") != NULL,
                   "then waits for the job to actually go");
        } else if (strcmp(kinds[i], "systemd") == 0) {
            EXPECT(disables, "SVC_KIND=systemd disables the units");
            EXPECT(!boots_out, "and does not reach for launchctl");
            EXPECT(strstr(buf, "for n in imud imud-signalk imud-mqtt "
                               "imud-influxdb imud-prometheus imud-mavlink")
                   != NULL, "over the same list");
        } else {
            EXPECT(!boots_out && !disables,
                   "SVC_KIND=none stops nothing, having installed nothing");
        }

        /* Whichever kind: removing the files is still uninstall's job. */
        EXPECT(strstr(buf, "rm -f") != NULL, "and the files are still removed");
    }

    setenv("PATH", g_stub, 1);
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
    setenv("PATH", g_make_path, 1);

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
    test_bus_absent();
    test_ft232h_rung_follows_the_host();
    test_endianness_is_reported();
    test_bus_forced();
    test_required_failures();
    test_atomics();
    test_link_order();
    test_optional_never_fatal();
    test_host_runtime();
    test_install_paths();
    test_prefix_reaches_the_binary();
    test_service_unit_follows_the_host();
    test_service_none();
    test_homebrew_formula();
    test_uninstall_stops_the_service();
    test_config_mk_is_valid_make();

    printf("\n%d passed, %d failed\n", g_checks - g_fail, g_fail);
    return g_fail ? 1 : 0;
}
