/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_hwtools_e2e.c — imud-cal and imud-imutest end to end, main() included
 *
 * Four sources were in no test binary at all: src/cal_main.c (921 lines),
 * src/imutest_main.c (319), src/imutest_open.c (201) and src/imutest_gpio.c
 * (165).  `make test` never compiled them, so lcov did not list them as 0% —
 * it did not list them.  Both real main()s are compiled as <base>_entry by the
 * Makefile's src/%.entry.o rule and called here.
 *
 * What stands in for the hardware:
 *
 *   - the `sim` driver, over a config this suite writes into /tmp.  Its bus is
 *     /dev/null: open_i2c() is a bare open() with no I2C_SLAVE ioctl, and the
 *     sim ops never look at the handle.
 *   - sim playback (sim_set_playback) for the magnetometer swing.  The
 *     synthetic scenario yaws 6 deg/s, so a real swing takes two minutes;
 *     replaying a capture of that same scenario at speed 0, looped, walks the
 *     whole heading circle as fast as the loop can read it.  do_mag then ends
 *     on its own MAX_MAG_SAMPLES bound rather than on a signal — which matters,
 *     because cal_main.c's g_stop is a file static that on_sigint sets and
 *     nothing ever clears, so one SIGINT-terminated case would poison every
 *     later one in this process.
 *   - test/bus_mock.c for the mmc5983ma degauss path, the same __wrap_ioctl
 *     register model test_imutest uses.
 *   - --wrap on imu_gpio_open/wait_edge/close for imutest_gpio.c.  Its counting
 *     POLICY — the 200 ms wait cap, the latched-vs-level drain rule, the
 *     imt_gpio_why_t classification — is pure logic over injectable callbacks;
 *     only imu_gpio_* (in the GPIO backend, linked here for real) touch a line.
 *     Unlike test_daemon's --wrap=pthread_create, these wrappers never fall
 *     through to __real_.  A passthrough would request a real interrupt line
 *     on any machine that has one — and imud is developed on the bench Pi,
 *     where gpiochip4 line 17 is the IMU's own.
 *
 * Deliberately dark, and named here so each gap is a decision rather than an
 * oversight:
 *
 *   - imutest_main.c's term_prompt, term_poll_done and term_coverage, which
 *     belong to the three guided phases: those need a tty (--non-interactive
 *     is implied when stdin is not one) AND a driver with a physical
 *     orientation, and the core skips all three outright for `sim`.
 *     term_progress and clear_line DO run — they are gated only on --quiet.
 *   - both on_sigint handlers, and everything downstream of them.  Neither
 *     tool's flag is ever cleared (cal_main.c's g_stop, imutest_main.c's
 *     g_sigint), so a case that raised SIGINT would decide the outcome of
 *     every case after it in this process.
 *   - do_accel's successful-fit arm: the sim reads one orientation for all six
 *     faces, so cal_accel_fit refuses — which is the arm worth testing, and
 *     the fit itself has six cases in test_cal_math.
 *   - imt_degauss's pulse-failure arms, which would need a write to fail after
 *     init() succeeded on the same register.
 *
 * Every transient file lives in /tmp rather than the build directory — see the
 * note in test_bridge_e2e.c.  Linux/GNU-ld only, like test_daemon.
 */

#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
#endif

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include "bus_mock.h"
#include "cal.h"
#include "capture.h"
#include "config.h"
#include "drivers.h"
#include "imu_gpio.h"
#include "imu_math.h"
#include "imutest.h"

int cal_main_entry(int argc, char **argv);
int imutest_main_entry(int argc, char **argv);

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── stdout/stderr capture ───────────────────────────────────────────────── */

/* Lifted from test_tools_e2e.c, and with its discipline: cap_end() runs before
 * any EXPECT, or a failure message is swallowed by the capture it reports on. */
typedef struct { int saved_out, saved_err, tmp; char path[64]; } cap_t;

static void cap_begin(cap_t *c, const char *path)
{
    fflush(stdout);
    fflush(stderr);
    snprintf(c->path, sizeof c->path, "%s", path);
    c->saved_out = dup(STDOUT_FILENO);
    c->saved_err = dup(STDERR_FILENO);
    c->tmp       = open(path, O_RDWR | O_CREAT | O_TRUNC, 0600);
    dup2(c->tmp, STDOUT_FILENO);
    dup2(c->tmp, STDERR_FILENO);
}

static void cap_end(cap_t *c, char *out, size_t outsz)
{
    fflush(stdout);
    fflush(stderr);
    dup2(c->saved_out, STDOUT_FILENO);
    dup2(c->saved_err, STDERR_FILENO);
    close(c->saved_out);
    close(c->saved_err);
    lseek(c->tmp, 0, SEEK_SET);
    ssize_t n = read(c->tmp, out, outsz - 1);
    out[n > 0 ? (size_t)n : 0] = '\0';
    close(c->tmp);
    unlink(c->path);
}

#define CAPFILE "/tmp/imud_hwtools_cap.txt"

/* Most cases print a few kB.  do_mag prints a progress line per sample and is
 * bounded by MAX_MAG_SAMPLES, so it gets its own heap buffer. */
static char g_out[64 * 1024];

/* ── Paths ───────────────────────────────────────────────────────────────── */

static char g_conf[128];        /* the sim config this suite writes */
static char g_cal[128];         /* --output target, one per case via suffix */
/* Sized to fit sockaddr_un::sun_path (108), not to the other paths here: this
 * one is bound as well as written into the config. */
static char g_sock[100];
static char g_cap_still[128];   /* stationary .imucap, temperature ramp */
static char g_cap_move[128];    /* the same, but turning: trips the motion gate */
static char g_cap_wave[128];    /* moving .imucap with mag records, for fit-ra */
static char g_cap_swing[128];   /* the sim's own mag field over a full circle */
static char g_report[128];

/* Two more configs for the arms the sim cannot reach: a driver name nothing
 * resolves, and real drivers whose bus either will not open or answers with
 * nothing.  All three sensor modes share those three branches. */
static char g_conf_baddrv[128];
static char g_conf_badbus[128];
static char g_conf_hw[128];

static const char *tmppath(char *buf, size_t sz, const char *tail)
{
    snprintf(buf, sz, "/tmp/imud_hwtools_%d_%s", (int)getpid(), tail);
    return buf;
}

/* A fresh cal.json path per case, into the caller's own buffer: cal_main loads
 * the existing file and preserves the sections it is not writing, so sharing
 * one would make each case depend on the ones before it. */
static char *calpath(char *buf, size_t sz, int id)
{
    snprintf(buf, sz, "/tmp/imud_hwtools_%d_cal%d.json", (int)getpid(), id);
    remove(buf);
    return buf;
}

static bool file_exists(const char *p) { return access(p, F_OK) == 0; }

static char *slurp(const char *path, char *buf, size_t sz)
{
    buf[0] = '\0';
    FILE *f = fopen(path, "r");
    if (!f) return buf;
    size_t n = fread(buf, 1, sz - 1, f);
    buf[n] = '\0';
    fclose(f);
    return buf;
}

/* ── stdin scripting ─────────────────────────────────────────────────────── */

/*
 * freopen rather than dup2 onto fd 0: both tools read stdin through the FILE*
 * (fgets for the [y/N] prompts, getchar for the accel Enters), and swapping
 * the descriptor underneath leaves whatever stdio had already buffered, plus a
 * latched EOF flag from the case before.  freopen resets both.
 */
static void stdin_from(const char *text)
{
    static char path[128];
    tmppath(path, sizeof path, "stdin.txt");
    FILE *f = fopen(path, "w");
    if (f) { fputs(text, f); fclose(f); }
    if (!freopen(path, "r", stdin)) perror("freopen stdin");
}

/* ── GPIO seam ───────────────────────────────────────────────────────────── */

/*
 * The three imu_gpio_* entry points, scripted.  A wait is answered from
 * `script` while it lasts; after that the wrapper sleeps the timeout it was
 * given and reports one, which is what a quiet line does and what lets
 * imt_gpio_count_edges' own window close it rather than the speed of this
 * loop deciding the count.
 */
typedef struct {
    int         open_errno;     /* non-zero: open fails, setting this errno */
    const int  *script;         /* wait_edge answers: 1 edge, 0 timeout, -1 err */
    int         nscript, next;
    int         opens, closes, waits, drains, primes;
    long        max_timeout_ms;
} gpio_sim_t;

static gpio_sim_t g_gpio;
static bool       g_gpio_armed;      /* a case has set the script up */
static int        g_gpio_unscripted; /* calls made outside a gpio case */

static int g_gpio_line_token;        /* address handed back as the line handle */

imu_gpio_line_t *__wrap_imu_gpio_open(const char *chip_name, unsigned int offset,
                                      const char *consumer);
int  __wrap_imu_gpio_wait_edge(imu_gpio_line_t *line, long timeout_ms);
void __wrap_imu_gpio_close(imu_gpio_line_t *line);

imu_gpio_line_t *__wrap_imu_gpio_open(const char *chip_name, unsigned int offset,
                                      const char *consumer)
{
    (void)chip_name; (void)offset; (void)consumer;
    if (!g_gpio_armed) { g_gpio_unscripted++; errno = ENODEV; return NULL; }
    g_gpio.opens++;
    if (g_gpio.open_errno) { errno = g_gpio.open_errno; return NULL; }
    return (imu_gpio_line_t *)&g_gpio_line_token;
}

int __wrap_imu_gpio_wait_edge(imu_gpio_line_t *line, long timeout_ms)
{
    (void)line;
    if (!g_gpio_armed) { g_gpio_unscripted++; return -1; }
    g_gpio.waits++;
    if (timeout_ms > g_gpio.max_timeout_ms) g_gpio.max_timeout_ms = timeout_ms;
    if (g_gpio.next < g_gpio.nscript) return g_gpio.script[g_gpio.next++];
    /* Not a poll interval: this IS the timeout under test, so it is spent. */
    if (timeout_ms > 0) usleep((useconds_t)timeout_ms * 1000);
    return 0;
}

void __wrap_imu_gpio_close(imu_gpio_line_t *line)
{
    (void)line;
    if (!g_gpio_armed) { g_gpio_unscripted++; return; }
    g_gpio.closes++;
}

static void gpio_arm(const int *script, int n, int open_errno)
{
    memset(&g_gpio, 0, sizeof g_gpio);
    g_gpio.script     = script;
    g_gpio.nscript    = n;
    g_gpio.open_errno = open_errno;
    g_gpio_armed      = true;
}

static void gpio_disarm(void) { g_gpio_armed = false; }

static void gpio_drain_cb(void *u) { (void)u; g_gpio.drains++; }
static void gpio_prime_cb(void *u) { (void)u; g_gpio.primes++; }

/* ── Fixtures ────────────────────────────────────────────────────────────── */

/*
 * startup_settle_sec is small but NOT zero: the settle window is its own
 * branch in three places (settle_imu, do_mag's inline loop, and the offline
 * modes' skipped-samples report), and zero is the one value that skips all
 * three.  0.2 s costs a fifth of a second per sensor mode.
 */
static void write_one_config(const char *path, const char *bus,
                             const char *imu_drv, const char *mag_drv)
{
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); exit(1); }
    fprintf(f,
        "# written by test_hwtools_e2e\n"
        "[device]\n"
        "i2c_bus   = \"%s\"\n"
        "gpio_chip = \"gpiochip0\"\n"
        "\n"
        "[imu]\n"
        "driver   = \"%s\"\n"
        "i2c_addr = 0x6A\n"
        "int_gpio = 0\n"
        "odr_hz   = 833\n"
        "\n"
        "[mag]\n"
        "driver       = \"%s\"\n"
        "i2c_addr     = 0x30\n"
        "int_gpio     = 0\n"
        "odr_hz       = 100\n"
        "set_period_s = 0.0\n"
        "\n"
        "[calibration]\n"
        "file               = \"%s\"\n"
        "startup_settle_sec = 0.2\n"
        "\n"
        "[stream]\n"
        "socket = \"%s\"\n",
        bus, imu_drv, mag_drv, g_cal, g_sock);
    fclose(f);
}

static void write_configs(void)
{
    write_one_config(g_conf,        "/dev/null", "sim", "sim");
    write_one_config(g_conf_baddrv, "/dev/null", "nosuchimu", "nosuchmag");
    /* Real drivers, a node that does not exist: open_sensor_bus fails, and for
     * anything but sim that is fatal. */
    write_one_config(g_conf_badbus, "/tmp/imud_hwtools_no_such_bus",
                     "ism330dhcx", "mmc5983ma");
    /* Real drivers on a node that opens, answering through an empty bus_mock
     * register file: probe() reads a WHO_AM_I of 0 and refuses. */
    write_one_config(g_conf_hw,     "/dev/null", "ism330dhcx", "mmc5983ma");
}

/* A deterministic LCG: a perfectly noiseless capture makes the Allan
 * deviation and the fit-ra residual variance identically zero, which is not a
 * shape either analysis is meant to handle. */
static uint32_t rng = 0x2545F491u;
static double noise(double amp)
{
    rng = rng * 1664525u + 1013904223u;
    return ((double)(rng >> 8) / 8388608.0 - 1.0) * amp;
}

/*
 * A stationary record with a die-temperature ramp: `characterize` reads the
 * Allan deviation off it and `fit-temp` reads the gyro's temperature slope.
 *
 * `turn_rps` breaks the stationarity both analyses assume, which is what the
 * motion gate exists to notice; 0 for a record that should pass it.
 */
static void write_still_capture(const char *path, int n, double hz,
                                double temp_span_c, double turn_rps)
{
    cap_writer_t w;
    if (cap_writer_open(&w, path, (uint32_t)hz, (uint32_t)(hz * 1000.0),
                        "sim", "sim", "1.9", 0, 0) != 0) {
        fprintf(stderr, "cannot write %s\n", path);
        exit(1);
    }
    rng = 0x2545F491u;
    for (int i = 0; i < n; i++) {
        imu_sample_t s;
        memset(&s, 0, sizeof s);
        double frac = (double)i / (double)(n - 1);
        double turn = turn_rps * sin(2.0 * M_PI * 0.5 * (double)i / hz);
        for (int a = 0; a < 3; a++) {
            /* A temperature-proportional gyro bias, so fit-temp has a slope to
             * find rather than only noise. */
            s.gyro[a]  = (float)(0.002 * frac + turn + noise(0.0006));
            s.accel[a] = (float)noise(0.01);
        }
        s.accel[2] = (float)(9.80665 + noise(0.01));
        s.temp_c   = (float)(20.0 + temp_span_c * frac);
        s.seq      = (uint32_t)i;
        cap_writer_imu(&w, &s, (uint64_t)((double)i / hz * 1e9));
    }
    cap_writer_close(&w);
}

/*
 * A level, gently noisy record with magnetometer samples: what `fit-ra` needs
 * to replay the MEKF and report on the accelerometer measurement model.  Level
 * and slow on purpose — the |a| skip band in [fusion] discards anything that
 * moves, and fitra_run refuses a record with fewer than 1000 accepted updates.
 */
static void write_wave_capture(const char *path, int odr, double dur_s)
{
    cap_writer_t w;
    if (cap_writer_open(&w, path, (uint32_t)odr, (uint32_t)(odr * 1000),
                        "sim", "sim", "1.9", 0, 0) != 0) {
        fprintf(stderr, "cannot write %s\n", path);
        exit(1);
    }
    rng = 0x9E3779B9u;
    const int    n       = (int)(odr * dur_s);
    const double dt      = 1.0 / odr;
    const int    mag_div = 8;
    for (int i = 0; i < n; i++) {
        double t = i * dt;
        imu_sample_t s;
        memset(&s, 0, sizeof s);
        s.accel[0] = (float)(0.05 * sin(2.0 * M_PI * 0.2 * t) + noise(0.02));
        s.accel[1] = (float)(0.05 * cos(2.0 * M_PI * 0.15 * t) + noise(0.02));
        s.accel[2] = (float)(-9.80665 + noise(0.02));
        for (int a = 0; a < 3; a++) s.gyro[a] = (float)noise(0.002);
        s.temp_c   = 25.0f;
        s.seq      = (uint32_t)i;
        /* 25 us/tick, the ISM330DHCX timer fit_ra.c reconstructs dt from. */
        s.chip_ts  = (uint32_t)llround(t / 25e-6);
        cap_writer_imu(&w, &s, (uint64_t)(t * 1e9));

        if (i % mag_div == 0) {
            mag_sample_t m;
            memset(&m, 0, sizeof m);
            m.field[0] = (float)(22.06 + noise(0.05));
            m.field[1] = (float)(0.0   + noise(0.05));
            m.field[2] = (float)(41.49 + noise(0.05));
            m.valid    = true;
            m.wall_ns  = (uint64_t)(t * 1e9);
            cap_writer_mag(&w, &m, (uint64_t)(t * 1e9));
        }
    }
    cap_writer_close(&w);
}

/*
 * The sim's own magnetometer output over one full heading circle, sampled from
 * sim_synth_mag() directly rather than waiting on the driver's real-time
 * pacing.  60 s covers 360 deg at the scenario's 6 deg/s.
 */
static void write_swing_capture(const char *path, double hz, double dur_s)
{
    cap_writer_t w;
    /* t0_mono_ns non-zero: pb_fetch's loop rebasing refuses to wrap a stream
     * whose newest record time is still 0. */
    const uint64_t t0 = 1000000000ULL;
    if (cap_writer_open(&w, path, 100, 100000, "sim", "sim", "1.9", t0, t0) != 0) {
        fprintf(stderr, "cannot write %s\n", path);
        exit(1);
    }
    const int n = (int)(hz * dur_s);
    for (int i = 0; i < n; i++) {
        double t = (double)i / hz;
        mag_sample_t m;
        memset(&m, 0, sizeof m);
        sim_synth_mag(t, &m);
        m.wall_ns = t0 + (uint64_t)(t * 1e9);
        cap_writer_mag(&w, &m, t0 + (uint64_t)(t * 1e9));
    }
    cap_writer_close(&w);
}

/* A pass-through mag calibration on disk: fit-ra's replay mirrors the daemon,
 * which marks mag samples invalid when cal.json has no mag section — and with
 * no heading update yaw is unobservable and the covariance diverges. */
static void write_mag_cal(const char *path)
{
    imud_cal_t c;
    memset(&c, 0, sizeof c);
    for (int i = 0; i < 3; i++) {
        c.accel_scale[i]     = 1.0f;
        c.mag_soft_iron[i][i] = 1.0f;
    }
    c.has_mag = true;
    if (cal_write(path, &c) < 0) fprintf(stderr, "cannot write %s\n", path);
}

/* ── imud-cal: argv and dispatch ─────────────────────────────────────────── */

static void test_cal_version_and_help(void)
{
    begin("test_cal_version_and_help");
    int fb = g_fail;

    char *av_v[] = { (char *)"imud-cal", (char *)"--version", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc_v = cal_main_entry(2, av_v);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_v == 0, "--version exits 0");
    EXPECT(strstr(g_out, "imud-cal ") != NULL, "--version names the tool");

    char *av_h[] = { (char *)"imud-cal", (char *)"--help", NULL };
    cap_begin(&c, CAPFILE);
    int rc_h = cal_main_entry(2, av_h);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_h == 0, "--help exits 0");
    EXPECT(strstr(g_out, "Usage:") != NULL, "--help prints usage");

    end(fb);
}

static void test_cal_bad_argv(void)
{
    begin("test_cal_bad_argv");
    int fb = g_fail;

    char *av_mode[] = { (char *)"imud-cal", (char *)"wobble", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc_mode = cal_main_entry(2, av_mode);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_mode == 1, "unknown mode exits 1");
    EXPECT(strstr(g_out, "unknown mode") != NULL, "unknown mode is named");

    char *av_none[] = { (char *)"imud-cal", NULL };
    cap_begin(&c, CAPFILE);
    int rc_none = cal_main_entry(1, av_none);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_none == 1, "no mode exits 1");

    /* An offline mode with no --from is refused before any file is touched. */
    char *av_from[] = { (char *)"imud-cal", (char *)"characterize", NULL };
    cap_begin(&c, CAPFILE);
    int rc_from = cal_main_entry(2, av_from);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_from == 1, "characterize without --from exits 1");
    EXPECT(strstr(g_out, "--from") != NULL, "the missing flag is named");

    end(fb);
}

/*
 * A sensor mode with an unreadable config is refused; an offline mode with the
 * same missing file runs on defaults, because all it needs the config for is
 * the cal.json path.
 */
static void test_cal_missing_config(void)
{
    begin("test_cal_missing_config");
    int fb = g_fail;

    const char *missing = "/tmp/imud_hwtools_no_such_config.conf";
    char *av[] = { (char *)"imud-cal", (char *)"--config", (char *)missing,
                   (char *)"gyro", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(4, av);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc == 1, "gyro with a missing config exits 1");
    EXPECT(strstr(g_out, "cannot load config") != NULL, "the reason is printed");

    end(fb);
}

/* ── imud-cal: the offline analysis modes ────────────────────────────────── */

static void test_cal_characterize(void)
{
    begin("test_cal_characterize");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 1);
    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out,
                   (char *)"--from", g_cap_still,
                   (char *)"characterize", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(8, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 0, "characterize exits 0");
    EXPECT(strstr(g_out, "Allan deviation") != NULL, "reports the Allan curve");
    EXPECT(strstr(g_out, "noise density") != NULL, "reports noise density");
    EXPECT(strstr(g_out, "never feed the filter") != NULL,
           "says the numbers are informational");

    char json[8192];
    slurp(out, json, sizeof json);
    EXPECT(strstr(json, "noise") != NULL, "cal.json gained a noise section");

    imud_cal_t cal;
    EXPECT(cal_load(out, &cal) == 0, "the written cal.json loads back");
    EXPECT(cal.has_noise, "has_noise survives the round trip");
    EXPECT(cal.gyro_noise_density[0] > 0.0f, "a gyro noise density was fitted");

    remove(out);
    end(fb);
}

static void test_cal_fit_temp(void)
{
    begin("test_cal_fit_temp");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 2);
    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out,
                   (char *)"--from", g_cap_still,
                   (char *)"fit-temp", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(8, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 0, "fit-temp exits 0");
    EXPECT(strstr(g_out, "Die temperature span") != NULL, "reports the span");
    EXPECT(strstr(g_out, "per degC") != NULL, "reports a slope per axis");

    imud_cal_t cal;
    EXPECT(cal_load(out, &cal) == 0, "the written cal.json loads back");
    EXPECT(cal.has_gyro_temp, "has_gyro_temp survives the round trip");
    EXPECT(fabsf(cal.gyro_temp_ref_c - 25.0f) < 0.01f, "the reference is 25 degC");

    remove(out);
    end(fb);
}

/* Too small a temperature span is the documented refusal, and it writes
 * nothing — the mode has no other output. */
static void test_cal_fit_temp_flat(void)
{
    begin("test_cal_fit_temp_flat");
    int fb = g_fail;

    char flat[128];
    tmppath(flat, sizeof flat, "flat.imucap");
    write_still_capture(flat, 2000, 100.0, 0.0, 0.0);

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 3);
    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out,
                   (char *)"--from", flat,
                   (char *)"fit-temp", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(8, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 1, "a flat temperature record exits 1");
    EXPECT(strstr(g_out, "temperature span too small") != NULL,
           "the reason is printed");
    EXPECT(!file_exists(out), "nothing was written");

    remove(flat);
    end(fb);
}

static void test_cal_offline_bad_capture(void)
{
    begin("test_cal_offline_bad_capture");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 4);
    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out,
                   (char *)"--from", (char *)"/tmp/imud_hwtools_absent.imucap",
                   (char *)"characterize", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(8, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 1, "a missing capture exits 1");
    EXPECT(strstr(g_out, "cannot open") != NULL, "the path is reported");
    EXPECT(!file_exists(out), "nothing was written");

    /* A file that opens but holds too few IMU records is a different arm. */
    char tiny[128];
    tmppath(tiny, sizeof tiny, "tiny.imucap");
    write_still_capture(tiny, 10, 100.0, 4.0, 0.0);
    char *av2[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                    (char *)"--output", (char *)out,
                    (char *)"--from", tiny,
                    (char *)"characterize", NULL };
    cap_begin(&c, CAPFILE);
    int rc2 = cal_main_entry(8, av2);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc2 == 1, "a too-short capture exits 1");
    remove(tiny);

    end(fb);
}

/*
 * fit-ra is the one mode that writes nothing: the answer belongs in imud.conf's
 * [fusion] section, not in cal.json.
 */
static void test_cal_fit_ra(void)
{
    begin("test_cal_fit_ra");
    int fb = g_fail;

    /* The failure arm first: an absent capture, and an empty cal.json that is
     * still absent afterwards. */
    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 5);
    char *av_bad[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                       (char *)"--output", (char *)out,
                       (char *)"--from", (char *)"/tmp/imud_hwtools_absent.imucap",
                       (char *)"fit-ra", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc_bad = cal_main_entry(8, av_bad);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_bad == 1, "fit-ra on a missing capture exits 1");
    EXPECT(strstr(g_out, "fit-ra:") != NULL, "the failure is attributed");
    EXPECT(!file_exists(out), "fit-ra wrote nothing");

    /* The success arm needs a mag-calibrated cal.json, exactly as the daemon
     * does: without one the replay marks every mag sample invalid and refuses. */
    char out2buf[128];
    const char *out2 = calpath(out2buf, sizeof out2buf, 6);
    write_mag_cal(out2);
    struct stat before, after;
    bool have_before = stat(out2, &before) == 0;

    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out2,
                   (char *)"--from", g_cap_wave,
                   (char *)"fit-ra", NULL };
    cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(8, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 0, "fit-ra exits 0 on a usable capture");
    EXPECT(strstr(g_out, "fit-ra: ") != NULL, "names the capture");
    EXPECT(strstr(g_out, "Gravity-direction residual") != NULL,
           "prints the residual report");
    EXPECT(strstr(g_out, "mean NIS") != NULL, "prints innovation consistency");

    /* fit-ra adds no section of its own; the file is only rewritten because
     * the mag section it was handed is still set. */
    bool have_after = stat(out2, &after) == 0;
    imud_cal_t cal;
    EXPECT(cal_load(out2, &cal) == 0, "cal.json still loads");
    EXPECT(cal.has_mag && !cal.has_noise && !cal.has_gyro && !cal.has_accel,
           "fit-ra added no section of its own");
    EXPECT(have_before && have_after && before.st_size == after.st_size,
           "the file did not grow");

    remove(out2);
    end(fb);
}

/* The gate that says the record was not the stationary one both analyses
 * assume — a warning, not a refusal: the numbers are still produced, and are
 * pessimistic rather than wrong. */
static void test_cal_motion_gate(void)
{
    begin("test_cal_motion_gate");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 11);
    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out,
                   (char *)"--from", g_cap_move,
                   (char *)"characterize", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(8, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 0, "a moving record still produces numbers");
    EXPECT(strstr(g_out, "does not look stationary") != NULL, "but says so");
    /* startup_settle_sec is non-zero, so the skipped-samples line runs too. */
    EXPECT(strstr(g_out, "Skipped first") != NULL,
           "and reports the settle window it dropped");

    remove(out);
    end(fb);
}

/* ── imud-cal: the sensor modes' three refusals ──────────────────────────── */

/*
 * Every sensor mode opens with the same three guards — resolve the driver
 * name, open its bus, bring the part up — and all three are unreachable with
 * `sim`, which resolves, ignores the handle and always answers.  So: a config
 * naming drivers that do not exist, one naming real drivers on a node that
 * does not open, and one naming real drivers on a node that does open and
 * answers with an empty register file.
 */
static void run_cal_mode(const char *conf, const char *mode, int id,
                         char *out, size_t outsz, int *rc_out)
{
    char outbuf[128];
    const char *cal = calpath(outbuf, sizeof outbuf, id);
    char *av[] = { (char *)"imud-cal", (char *)"--config", (char *)conf,
                   (char *)"--output", (char *)cal, (char *)mode, NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    *rc_out = cal_main_entry(6, av);
    cap_end(&c, out, outsz);
    remove(cal);
}

static void test_cal_unknown_driver(void)
{
    begin("test_cal_unknown_driver");
    int fb = g_fail;
    int rc;

    run_cal_mode(g_conf_baddrv, "mag", 20, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "mag with an unknown driver exits 1");
    EXPECT(strstr(g_out, "unknown mag driver") != NULL, "and names it");

    run_cal_mode(g_conf_baddrv, "gyro", 21, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "gyro with an unknown driver exits 1");
    EXPECT(strstr(g_out, "unknown IMU driver") != NULL, "and names it");

    run_cal_mode(g_conf_baddrv, "accel", 22, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "accel with an unknown driver exits 1");
    EXPECT(strstr(g_out, "unknown IMU driver") != NULL, "and names it");

    end(fb);
}

static void test_cal_bus_open_failure(void)
{
    begin("test_cal_bus_open_failure");
    int fb = g_fail;
    int rc;

    /* Fatal for a real driver, and only for a real driver: the sim never looks
     * at the handle, so the same failure is survivable there. */
    run_cal_mode(g_conf_badbus, "mag", 23, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "mag on an unopenable bus exits 1");
    EXPECT(strstr(g_out, "cannot open") != NULL, "and names the node");

    run_cal_mode(g_conf_badbus, "gyro", 24, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "gyro on an unopenable bus exits 1");

    run_cal_mode(g_conf_badbus, "accel", 25, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "accel on an unopenable bus exits 1");

    end(fb);
}

static void test_cal_sensor_bringup_failure(void)
{
    begin("test_cal_sensor_bringup_failure");
    int fb = g_fail;
    int rc;

    i2cmock_reset();          /* every register 0, so no WHO_AM_I matches */

    run_cal_mode(g_conf_hw, "mag", 26, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "mag against a silent part exits 1");
    EXPECT(strstr(g_out, "mag sensor init failed") != NULL, "and says so");

    run_cal_mode(g_conf_hw, "gyro", 27, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "gyro against a silent part exits 1");
    EXPECT(strstr(g_out, "IMU sensor init failed") != NULL, "and says so");

    run_cal_mode(g_conf_hw, "accel", 28, g_out, sizeof g_out, &rc);
    EXPECT(rc == 1, "accel against a silent part exits 1");
    EXPECT(strstr(g_out, "IMU sensor init failed") != NULL, "and says so");

    end(fb);
}

/* ── imud-cal: the sensor modes, against sim ─────────────────────────────── */

static void test_cal_gyro(void)
{
    begin("test_cal_gyro (5 s)");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 7);
    stdin_from("y\n");

    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out, (char *)"gyro", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(6, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 0, "gyro exits 0");
    EXPECT(strstr(g_out, "gyroscope bias calibration") != NULL, "prints the banner");
    EXPECT(strstr(g_out, "Gyro bias (rad/s)") != NULL, "prints the bias");
    /* The scenario is a boat under way at 6 deg/s, which is 0.105 rad/s — well
     * past the 0.05 rad/s "was the sensor moving?" threshold. */
    EXPECT(strstr(g_out, "bias magnitude") != NULL,
           "warns that the sensor was moving");
    EXPECT(strstr(g_out, "Saved to") != NULL, "reports the save");

    imud_cal_t cal;
    EXPECT(cal_load(out, &cal) == 0, "the written cal.json loads back");
    EXPECT(cal.has_gyro, "has_gyro survives the round trip");
    double mag = sqrt((double)cal.gyro_bias[0] * cal.gyro_bias[0] +
                      (double)cal.gyro_bias[1] * cal.gyro_bias[1] +
                      (double)cal.gyro_bias[2] * cal.gyro_bias[2]);
    EXPECT(mag > 0.05 && mag < 0.5, "the recorded bias is the scenario's yaw rate");

    remove(out);
    end(fb);
}

/* Answering anything but y at the prompt leaves the file alone. */
static void test_cal_gyro_declined(void)
{
    begin("test_cal_gyro_declined (5 s)");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 8);
    stdin_from("n\n");

    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out, (char *)"gyro", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(6, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 0, "declining exits 0");
    EXPECT(strstr(g_out, "Calibration not saved") != NULL, "says so");
    EXPECT(!file_exists(out), "and wrote nothing");

    end(fb);
}

/*
 * The safety-critical arm.  The sim scenario is a boat under way, so all six
 * "positions" read the same orientation; cal_accel_fit refuses rather than
 * fitting, because a calibration taken from a misplaced board is worse than no
 * calibration at all.
 */
static void test_cal_accel_geometry_refused(void)
{
    begin("test_cal_accel_geometry_refused (17 s)");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 9);
    stdin_from("\n\n\n\n\n\n");

    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out, (char *)"accel", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(6, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 1, "the geometry check fails the run");
    EXPECT(strstr(g_out, "6-position calibration") != NULL, "prints the banner");
    EXPECT(strstr(g_out, "Position 6/6") != NULL, "walked all six positions");
    EXPECT(strstr(g_out, "gravity landed on") != NULL,
           "names the axis gravity actually landed on");
    EXPECT(strstr(g_out, "geometry check failed") != NULL, "refuses the fit");
    EXPECT(strstr(g_out, "nothing written") != NULL, "says nothing was written");
    EXPECT(!file_exists(out), "and nothing was");

    end(fb);
}

/*
 * The magnetometer swing.  Replayed rather than synthesised: the scenario yaws
 * 6 deg/s, so a live swing is two minutes of wall clock, while the same
 * scenario replayed at speed 0 and looped walks the circle as fast as do_mag
 * can read it and ends on MAX_MAG_SAMPLES.
 *
 * Last of the cal cases, and the only one that leaves sim state behind — the
 * teardown puts playback back.
 */
static void test_cal_mag_swing(void)
{
    begin("test_cal_mag_swing");
    int fb = g_fail;

    char outbuf[128];
    const char *out = calpath(outbuf, sizeof outbuf, 10);
    stdin_from("y\n");
    sim_set_playback(g_cap_swing, true, 0.0f);

    /* One progress line of about 140 bytes per sample, bounded by
     * MAX_MAG_SAMPLES = 16000 — so around 2.3 MB, and the assertions that
     * matter are at the far end of it. */
    size_t bufsz = 8u * 1024 * 1024;
    char *buf = malloc(bufsz);
    if (!buf) { puts("SKIP (out of memory)"); return; }

    char *av[] = { (char *)"imud-cal", (char *)"--config", g_conf,
                   (char *)"--output", (char *)out, (char *)"mag", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = cal_main_entry(6, av);
    cap_end(&c, buf, bufsz);

    sim_set_playback(NULL, false, 1.0f);

    EXPECT(rc == 0, "mag exits 0");
    EXPECT(strstr(buf, "magnetometer calibration") != NULL, "prints the banner");
    EXPECT(strstr(buf, "24/24") != NULL, "the swing covered every sector");
    EXPECT(strstr(buf, "Hard iron") != NULL, "reports hard iron");
    EXPECT(strstr(buf, "Field radius") != NULL, "reports the field radius");
    EXPECT(strstr(buf, "RMS residual") != NULL, "reports the residual");
    EXPECT(strstr(buf, "Saved to") != NULL, "reports the save");

    imud_cal_t cal;
    EXPECT(cal_load(out, &cal) == 0, "the written cal.json loads back");
    EXPECT(cal.has_mag, "has_mag survives the round trip");
    /* The sim field is 25 uT north and 40 uT down with no hard iron, so the
     * fitted radius is hypot(25, 40) = 47.2 uT and the centre is the origin. */
    double r = sqrt((double)cal.mag_hard_iron[0] * cal.mag_hard_iron[0] +
                    (double)cal.mag_hard_iron[1] * cal.mag_hard_iron[1] +
                    (double)cal.mag_hard_iron[2] * cal.mag_hard_iron[2]);
    EXPECT(r < 2.0, "the fitted hard iron is near zero, as the sim has none");

    free(buf);
    remove(out);
    end(fb);
}

/* ── imud-imutest: argv and the guards ───────────────────────────────────── */

static void test_imutest_version_and_bad_option(void)
{
    begin("test_imutest_version_and_bad_option");
    int fb = g_fail;

    char *av_v[] = { (char *)"imud-imutest", (char *)"--version", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc_v = imutest_main_entry(2, av_v);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_v == 0, "--version exits 0");
    EXPECT(strstr(g_out, "imud-imutest ") != NULL, "--version names the tool");

    char *av_b[] = { (char *)"imud-imutest", (char *)"--nope", NULL };
    cap_begin(&c, CAPFILE);
    int rc_b = imutest_main_entry(2, av_b);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_b == 1, "a bad option exits 1");
    EXPECT(strstr(g_out, "unknown option") != NULL, "and says which");

    end(fb);
}

/* An explicit --config that cannot be read is fatal; without the flag the tool
 * runs on defaults. */
static void test_imutest_missing_config(void)
{
    begin("test_imutest_missing_config");
    int fb = g_fail;

    char *av[] = { (char *)"imud-imutest", (char *)"--config",
                   (char *)"/tmp/imud_hwtools_no_such_config.conf",
                   (char *)"--non-interactive", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(4, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 1, "an unreadable --config exits 1");
    EXPECT(strstr(g_out, "cannot read") != NULL, "the path is reported");

    end(fb);
}

/* Bind a listener where [stream] socket points and the daemon-conflict guard
 * must fire — that is the whole of its evidence that imud is up. */
static int bind_stream_socket(void)
{
    struct sockaddr_un a;
    memset(&a, 0, sizeof a);
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", g_sock);
    unlink(g_sock);
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0 || listen(fd, 4) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void test_imutest_daemon_guard(void)
{
    begin("test_imutest_daemon_guard");
    int fb = g_fail;

    int srv = bind_stream_socket();
    if (srv < 0) { puts("SKIP (cannot bind)"); return; }

    char *av[] = { (char *)"imud-imutest", (char *)"--config", g_conf,
                   (char *)"--non-interactive", NULL };
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(4, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 1, "a running daemon exits 1 without --force");
    EXPECT(strstr(g_out, "appears to be running") != NULL, "says why");
    EXPECT(strstr(g_out, g_sock) != NULL, "names the socket it probed");
    EXPECT(strstr(g_out, "--force") != NULL, "offers the override");

    close(srv);
    unlink(g_sock);
    end(fb);
}

/* ── imud-imutest: --degauss ─────────────────────────────────────────────── */

#define MMC_ADDR 0x30

/* Split a field into the MMC5983MA's seven output registers, as the driver
 * decodes them: value = (raw - 131072) * 100/16384 uT, with Y flipped. */
static void mmc_set_field(double ut_x, double ut_y, double ut_z)
{
    const double lsb = 100.0 / 16384.0;
    uint32_t rx = (uint32_t)lrint(131072.0 +  ut_x / lsb);
    uint32_t ry = (uint32_t)lrint(131072.0 + -ut_y / lsb);
    uint32_t rz = (uint32_t)lrint(131072.0 +  ut_z / lsb);
    uint8_t raw[7] = {
        (uint8_t)((rx >> 10) & 0xFF), (uint8_t)((rx >> 2) & 0xFF),
        (uint8_t)((ry >> 10) & 0xFF), (uint8_t)((ry >> 2) & 0xFF),
        (uint8_t)((rz >> 10) & 0xFF), (uint8_t)((rz >> 2) & 0xFF),
        (uint8_t)(((rx & 3) << 6) | ((ry & 3) << 4) | ((rz & 3) << 2)),
    };
    i2cmock_set_regs(MMC_ADDR, 0x00, raw, 7);
    i2cmock_set_reg(MMC_ADDR, 0x08, 0x01);       /* STATUS: M_DONE */
}

static void mmc_mock_up(double ut_x, double ut_y, double ut_z)
{
    i2cmock_reset();
    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x30);       /* PRODUCT_ID */
    mmc_set_field(ut_x, ut_y, ut_z);
}

/*
 * --force on every run but the daemon-guard case: the conflict probe also
 * tries the compiled-in default socket, so without it this suite would pass or
 * fail depending on whether the machine it runs on happens to be serving imud.
 */
static int run_degauss(const char *mag_driver, char *out, size_t outsz)
{
    char addr[8];
    snprintf(addr, sizeof addr, "%d", MMC_ADDR);
    char *av[] = { (char *)"imud-imutest", (char *)"--config", g_conf,
                   (char *)"--mag-driver", (char *)mag_driver,
                   (char *)"--mag-addr", addr, (char *)"--force",
                   (char *)"--degauss", (char *)"--non-interactive", NULL };
    int ac = 0; while (av[ac]) ac++;
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(ac, av);
    cap_end(&c, out, outsz);
    return rc;
}

/* sim_mag_ops declares no .degauss, which is the validation arm. */
static void test_imutest_degauss_unsupported(void)
{
    begin("test_imutest_degauss_unsupported");
    int fb = g_fail;

    int rc = run_degauss("sim", g_out, sizeof g_out);
    EXPECT(rc == 1, "a part with no coil exits 1");
    EXPECT(strstr(g_out, "no degauss coil") != NULL, "says why");

    int rc2 = run_degauss("nosuchmag", g_out, sizeof g_out);
    EXPECT(rc2 == 1, "an unknown mag driver exits 1");
    EXPECT(strstr(g_out, "unknown mag driver") != NULL, "names the driver");

    /* An IMU-only board: "none" is how that is spelled, and there is nothing
     * to degauss. */
    int rc3 = run_degauss("none", g_out, sizeof g_out);
    EXPECT(rc3 == 1, "a board with no magnetometer exits 1");
    EXPECT(strstr(g_out, "no magnetometer configured") != NULL, "says so");

    end(fb);
}

/* imt_degauss opens the bus itself, so its open failure is its own arm. */
static void test_imutest_degauss_bus_open_failure(void)
{
    begin("test_imutest_degauss_bus_open_failure");
    int fb = g_fail;

    char *av[] = { (char *)"imud-imutest", (char *)"--config", g_conf_badbus,
                   (char *)"--force", (char *)"--degauss",
                   (char *)"--non-interactive", NULL };
    int ac = 0; while (av[ac]) ac++;
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(ac, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 1, "an unopenable bus exits 1");
    EXPECT(strstr(g_out, "cannot open") != NULL &&
           strstr(g_out, "magnetometer") != NULL, "and says which sensor");

    end(fb);
}

/* imt_run's own failure, reported through main() rather than returned. */
static void test_imutest_run_failure(void)
{
    begin("test_imutest_run_failure");
    int fb = g_fail;

    char *av[] = { (char *)"imud-imutest", (char *)"--config", g_conf,
                   (char *)"--imu-driver", (char *)"nosuchimu",
                   (char *)"--force", (char *)"--non-interactive", NULL };
    int ac = 0; while (av[ac]) ac++;
    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(ac, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc == 1, "a driver that does not resolve exits 1");
    EXPECT(strstr(g_out, "imud-imutest: unknown IMU driver") != NULL,
           "and the tool attributes the failure");
    EXPECT(strstr(g_out, "Report written to") == NULL,
           "with no report for a run that never happened");

    end(fb);
}

static void test_imutest_degauss_in_range(void)
{
    begin("test_imutest_degauss_in_range");
    int fb = g_fail;

    mmc_mock_up(22.0, 4.0, 42.0);          /* |B| ~ 47.6 uT */
    int rc = run_degauss("mmc5983ma", g_out, sizeof g_out);

    EXPECT(rc == 0, "a plausible field exits 0");
    EXPECT(strstr(g_out, "before        |B|") != NULL, "reports the field before");
    EXPECT(strstr(g_out, "after RESET") != NULL, "reports the RESET pass");
    EXPECT(strstr(g_out, "after SET") != NULL, "reports the SET pass");
    EXPECT(strstr(g_out, "Nothing further to do") != NULL, "declares it healthy");

    end(fb);
}

static void test_imutest_degauss_still_saturated(void)
{
    begin("test_imutest_degauss_still_saturated");
    int fb = g_fail;

    /* The MMC5983MA anomaly this exists for: a saturated bridge sits in the
     * hundreds and does not move when the coil is pulsed. */
    mmc_mock_up(1100.0, 40.0, 60.0);
    int rc = run_degauss("mmc5983ma", g_out, sizeof g_out);

    EXPECT(rc == 2, "a field the coil cannot clear exits 2");
    EXPECT(strstr(g_out, "after a full RESET/SET pass") != NULL,
           "says the pass completed");
    EXPECT(strstr(g_out, "The coil is not the fault") != NULL,
           "points at the installation");

    end(fb);
}

static void test_imutest_degauss_bus_errors(void)
{
    begin("test_imutest_degauss_bus_errors");
    int fb = g_fail;

    /* Wrong PRODUCT_ID: probe() refuses before anything is written. */
    i2cmock_reset();
    i2cmock_set_reg(MMC_ADDR, 0x2F, 0x00);
    int rc = run_degauss("mmc5983ma", g_out, sizeof g_out);
    EXPECT(rc == 1, "a part that does not answer probe exits 1");
    EXPECT(strstr(g_out, "did not answer probe") != NULL, "says so");

    /*
     * CTRL2 (0x0B) is the last write mmc_init makes — the one that starts
     * continuous mode — so failing it leaves probe() and reset() intact and
     * lands on the "did not initialise" arm rather than on probe again.
     */
    mmc_mock_up(22.0, 4.0, 42.0);
    i2cmock_fail_write_to(MMC_ADDR, 0x0B, -1);
    int rc2 = run_degauss("mmc5983ma", g_out, sizeof g_out);
    EXPECT(rc2 == 1, "a part that will not start exits 1");
    EXPECT(strstr(g_out, "did not initialise") != NULL, "says which stage");

    /* A wedged bus: every transfer fails, from probe onward. */
    mmc_mock_up(22.0, 4.0, 42.0);
    i2cmock_fail_all(1);
    int rc3 = run_degauss("mmc5983ma", g_out, sizeof g_out);
    i2cmock_fail_all(0);
    EXPECT(rc3 == 1, "a wedged bus exits 1");

    i2cmock_reset();
    end(fb);
}

/* ── imud-imutest: imt_run's resolution and open layer ───────────────────── */

/*
 * The error arms of imt_run(), called directly.  Reaching them through main()
 * would mean a full check run per arm for four lines apiece; the entry point's
 * own wiring is covered by the passive runs below.
 */
static void test_imt_run_open_errors(void)
{
    begin("test_imt_run_open_errors");
    int fb = g_fail;

    imud_config_t cfg;
    config_defaults(&cfg);
    snprintf(cfg.i2c_bus, sizeof cfg.i2c_bus, "/dev/null");
    cfg.imu_int_gpio = 0;
    cfg.mag_int_gpio = 0;

    imt_opts_t o;
    imt_opts_defaults(&o);
    o.phases = 0;                       /* resolve and open only */

    imt_report_t *rep = calloc(1, sizeof *rep);
    if (!rep) { puts("SKIP (out of memory)"); return; }
    char err[256];
    cap_t c;

    snprintf(cfg.imu_driver, sizeof cfg.imu_driver, "nosuchimu");
    err[0] = '\0';
    cap_begin(&c, CAPFILE);
    int rc_imu = imt_run(&cfg, &o, rep, err, sizeof err);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_imu < 0, "an unknown IMU driver fails");
    EXPECT(strstr(err, "unknown IMU driver") != NULL, "and names it");

    snprintf(cfg.imu_driver, sizeof cfg.imu_driver, "sim");
    snprintf(cfg.mag_driver, sizeof cfg.mag_driver, "nosuchmag");
    err[0] = '\0';
    cap_begin(&c, CAPFILE);
    int rc_mag = imt_run(&cfg, &o, rep, err, sizeof err);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_mag < 0, "an unknown mag driver fails");
    EXPECT(strstr(err, "unknown mag driver") != NULL, "and names it");

    /* A bus node that cannot be opened, for each sensor in turn. */
    snprintf(cfg.mag_driver, sizeof cfg.mag_driver, "sim");
    snprintf(cfg.i2c_bus, sizeof cfg.i2c_bus, "/tmp/imud_hwtools_no_such_bus");
    err[0] = '\0';
    cap_begin(&c, CAPFILE);
    int rc_bus = imt_run(&cfg, &o, rep, err, sizeof err);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_bus < 0, "an unopenable bus fails");
    EXPECT(strstr(err, "cannot open") != NULL && strstr(err, "IMU") != NULL,
           "and says which sensor");

    /*
     * The magnetometer's own open: the IMU is on a node that opens and the mag
     * on one that does not, which is only reachable when the two differ.  SPI
     * gives that — mag_spi_dev is a separate node — and bus_open must still
     * close the IMU handle it already took.
     */
    snprintf(cfg.i2c_bus, sizeof cfg.i2c_bus, "/dev/null");
    cfg.mag_bus_kind = BUS_SPI;
    snprintf(cfg.mag_spi_dev, sizeof cfg.mag_spi_dev,
             "/tmp/imud_hwtools_no_such_spidev");
    err[0] = '\0';
    cap_begin(&c, CAPFILE);
    int rc_mbus = imt_run(&cfg, &o, rep, err, sizeof err);
    cap_end(&c, g_out, sizeof g_out);
    EXPECT(rc_mbus < 0, "an unopenable mag bus fails");
    EXPECT(strstr(err, "magnetometer") != NULL, "and says which sensor");

    free(rep);
    end(fb);
}

/* ── imud-imutest: the passive run, through main() ───────────────────────── */

/*
 * The tool end to end against sim: the config-override block, the daemon
 * marker, the banner, imt_run's bus_open for both sensors, the digest and the
 * report file.  --force with a listener bound covers the other side of the
 * daemon guard, and the report has to say the daemon was up.
 */
static void test_imutest_passive_run(void)
{
    begin("test_imutest_passive_run (~13 s)");
    int fb = g_fail;

    int srv = bind_stream_socket();
    remove(g_report);

    char *av[] = { (char *)"imud-imutest", (char *)"--config", g_conf,
                   (char *)"--imu-driver", (char *)"sim",
                   (char *)"--mag-driver", (char *)"sim",
                   (char *)"--i2c-bus", (char *)"/dev/null",
                   (char *)"--int-gpio", (char *)"0",
                   (char *)"--odr", (char *)"833",
                   (char *)"--accel-g", (char *)"4",
                   (char *)"--gyro-dps", (char *)"500",
                   (char *)"--fifo-wm", (char *)"8",
                   (char *)"--odr-window", (char *)"3",
                   (char *)"--noise-window", (char *)"1",
                   (char *)"--drdy-window", (char *)"1",
                   (char *)"--turn-deg", (char *)"90",
                   (char *)"--grav-tol", (char *)"0.85",
                   (char *)"--odr-tol", (char *)"10",
                   (char *)"--no-fs-sweep", (char *)"--no-overflow",
                   (char *)"--no-regdiff",
                   (char *)"--force", (char *)"--non-interactive",
                   (char *)"--all", (char *)"--report", g_report, NULL };
    int ac = 0; while (av[ac]) ac++;

    /* Not g_out: term_progress rewrites a line per update for the whole run,
     * and the digest this asserts on is at the far end of it. */
    size_t bufsz = 4u * 1024 * 1024;
    char  *buf   = malloc(bufsz);
    if (!buf) { puts("SKIP (out of memory)"); return; }

    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(ac, av);
    cap_end(&c, buf, bufsz);

    /* 0 / 2 / 3 are graded outcomes; 1 is "the run could not happen" and 130 is
     * an abort, and this run must be neither. */
    EXPECT(rc == 0 || rc == 2 || rc == 3, "the run produced a graded verdict");
    EXPECT(strstr(buf, "imud-imutest ") != NULL, "prints the banner");
    EXPECT(strstr(buf, "on I2C /dev/null") != NULL,
           "names the transport and node actually opened");
    EXPECT(strstr(buf, "passive checks only") != NULL,
           "says the guided phases were skipped");
    EXPECT(strstr(buf, "PASS") != NULL && strstr(buf, "FAIL") != NULL,
           "prints the digest tally");
    EXPECT(strstr(buf, "the `sim` driver exercises this tool") != NULL,
           "marks the run as sim rather than hardware");
    EXPECT(strstr(buf, "imud was running") != NULL,
           "marks the report because --force was used over a live socket");
    EXPECT(strstr(buf, "Report written to") != NULL, "reports the file");

    EXPECT(file_exists(g_report), "the report file exists");
    char *md = malloc(512 * 1024);
    if (md) {
        slurp(g_report, md, 512 * 1024);
        EXPECT(strstr(md, "# imud driver validation") != NULL, "report: title");
        EXPECT(strstr(md, "## 1. Environment") != NULL, "report: environment");
        EXPECT(strstr(md, "## 2. Device under test") != NULL, "report: subject");
        EXPECT(strstr(md, "## 3. Results") != NULL, "report: results");
        EXPECT(strstr(md, "## 4. Scope of this run") != NULL, "report: scope");
        EXPECT(strstr(md, "## 5. Appendix") != NULL, "report: appendix");
        EXPECT(strstr(md, "imud-imutest --config") != NULL,
               "report: records the invocation");
        free(md);
    }

    free(buf);
    if (srv >= 0) close(srv);
    unlink(g_sock);
    remove(g_report);
    end(fb);
}

/*
 * The default report name.  --report omitted puts
 * imud-imutest-<driver>-%Y%m%d-%H%M%S.md in the working directory — the site of
 * a past truncation bug, where sizeof on the char* alias cut it to "imud-im" —
 * so the run happens from /tmp and the name is reconstructed rather than
 * globbed.  `mag_driver = "none"` is the second thing under test: an IMU-only
 * board is a real deployment, and imt_run leaves mbus closed for it.
 */
static void test_imutest_default_report_name(void)
{
    begin("test_imutest_default_report_name (~6 s)");
    int fb = g_fail;

    char cwd[512];
    if (!getcwd(cwd, sizeof cwd)) { puts("SKIP (no cwd)"); return; }
    if (chdir("/tmp") != 0)       { puts("SKIP (cannot chdir)"); return; }

    time_t t_before = time(NULL);

    char *av[] = { (char *)"imud-imutest", (char *)"--config", g_conf,
                   (char *)"--imu-driver", (char *)"sim",
                   (char *)"--mag-driver", (char *)"none",
                   (char *)"--odr-window", (char *)"3",
                   (char *)"--noise-window", (char *)"1",
                   (char *)"--drdy-window", (char *)"1",
                   (char *)"--no-fs-sweep", (char *)"--no-overflow",
                   (char *)"--no-regdiff", (char *)"--force",
                   (char *)"--quiet", (char *)"--non-interactive", NULL };
    int ac = 0; while (av[ac]) ac++;

    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(ac, av);
    cap_end(&c, g_out, sizeof g_out);

    time_t t_after = time(NULL);
    (void)rc;

    /* The stamp is localtime at the moment main() built it, so accept any
     * second in the window the run spanned. */
    char found[128] = "";
    for (time_t t = t_before; t <= t_after && !found[0]; t++) {
        struct tm tm;
        localtime_r(&t, &tm);
        char stamp[32], name[128];
        strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", &tm);
        snprintf(name, sizeof name, "/tmp/imud-imutest-sim-%s.md", stamp);
        if (file_exists(name)) snprintf(found, sizeof found, "%s", name);
    }

    EXPECT(found[0] != '\0', "wrote imud-imutest-<driver>-<stamp>.md");
    EXPECT(strstr(g_out, "Report written to imud-imutest-sim-") != NULL,
           "and reported the untruncated name");
    EXPECT(strstr(g_out, "sim + ") == NULL,
           "no magnetometer is named when mag_driver is none");

    if (found[0]) {
        char *md = malloc(512 * 1024);
        if (md) {
            slurp(found, md, 512 * 1024);
            EXPECT(strstr(md, "None configured for this run") != NULL,
                   "the report says there was no magnetometer");
            free(md);
        }
        remove(found);
    }

    if (chdir(cwd) != 0) fprintf(stderr, "  (could not return to %s)\n", cwd);
    end(fb);
}

/* An unwritable report path is reported, and the run's own verdict survives. */
static void test_imutest_report_write_failure(void)
{
    begin("test_imutest_report_write_failure (~6 s)");
    int fb = g_fail;

    char *av[] = { (char *)"imud-imutest", (char *)"--config", g_conf,
                   (char *)"--imu-driver", (char *)"sim",
                   (char *)"--mag-driver", (char *)"none",
                   (char *)"--odr-window", (char *)"3",
                   (char *)"--noise-window", (char *)"1",
                   (char *)"--drdy-window", (char *)"1",
                   (char *)"--no-fs-sweep", (char *)"--no-overflow",
                   (char *)"--no-regdiff", (char *)"--quiet",
                   (char *)"--force", (char *)"--non-interactive",
                   (char *)"--report",
                   (char *)"/tmp/imud_hwtools_no_such_dir/report.md", NULL };
    int ac = 0; while (av[ac]) ac++;

    cap_t c; cap_begin(&c, CAPFILE);
    int rc = imutest_main_entry(ac, av);
    cap_end(&c, g_out, sizeof g_out);

    EXPECT(rc != 0, "an unwritable report is a non-zero exit");
    EXPECT(strstr(g_out, "imud-imutest: ") != NULL, "the failure is attributed");

    end(fb);
}

/* ── imutest_gpio.c: the counting policy ─────────────────────────────────── */

static void test_gpio_disabled_and_open_errors(void)
{
    begin("test_gpio_disabled_and_open_errors");
    int fb = g_fail;

    imt_gpio_why_t why = IMT_GPIO_OK;

    /* int_gpio 0 is a real deployment, not a fault: the reader polls instead. */
    gpio_arm(NULL, 0, 0);
    int rc = imt_gpio_count_edges("gpiochip0", 0, 50, gpio_drain_cb, NULL,
                                  &why, NULL, 100000);
    EXPECT(rc == -1, "a disabled line returns -1");
    EXPECT(why == IMT_GPIO_DISABLED, "and says it is disabled");
    EXPECT(g_gpio.opens == 0, "without requesting a line");

    gpio_arm(NULL, 0, ENOENT);
    why = IMT_GPIO_OK;
    rc = imt_gpio_count_edges("gpiochip9", 17, 50, NULL, NULL, &why, NULL, 100000);
    EXPECT(rc == -1 && why == IMT_GPIO_ENOCHIP, "ENOENT is a missing chip");

    /* EBUSY is the daemon holding the line: a reason to skip the check, never
     * to fail the driver. */
    gpio_arm(NULL, 0, EBUSY);
    why = IMT_GPIO_OK;
    rc = imt_gpio_count_edges("gpiochip0", 17, 50, NULL, NULL, &why, NULL, 100000);
    EXPECT(rc == -1 && why == IMT_GPIO_EBUSY, "EBUSY is someone else holding it");

    /* ENOSYS is src/imu_gpio_null.c: no GPIO backend in this build.  Nothing
     * to measure, and nothing wrong with the part — reporting it as EIO would
     * fail a driver over a build option. */
    gpio_arm(NULL, 0, ENOSYS);
    why = IMT_GPIO_OK;
    rc = imt_gpio_count_edges("gpiochip0", 17, 50, NULL, NULL, &why, NULL, 100000);
    EXPECT(rc == -1 && why == IMT_GPIO_UNSUPPORTED,
           "ENOSYS is a build with no GPIO backend");

    gpio_arm(NULL, 0, EACCES);
    why = IMT_GPIO_OK;
    rc = imt_gpio_count_edges("gpiochip0", 17, 50, NULL, NULL, &why, NULL, 100000);
    EXPECT(rc == -1 && why == IMT_GPIO_EIO, "anything else is a real fault");

    gpio_disarm();
    end(fb);
}

/*
 * A LEVEL watermark (the IMU FIFO) passes no prime, and must be drained only on
 * an edge: draining on a timeout too keeps the FIFO below the threshold and the
 * watermark never asserts at all.
 */
static void test_gpio_level_line(void)
{
    begin("test_gpio_level_line");
    int fb = g_fail;

    static const int script[] = { 1, 1, 1 };
    gpio_arm(script, 3, 0);
    imt_gpio_why_t why = IMT_GPIO_OK;

    /* Window past the 200 ms cap, so the cap itself is what is measured. */
    int edges = imt_gpio_count_edges("gpiochip0", 17, 260,
                                     gpio_drain_cb, NULL, &why, NULL, 833000);

    EXPECT(edges == 3, "every scripted edge is counted");
    EXPECT(why == IMT_GPIO_OK, "and the line is not faulted");
    EXPECT(g_gpio.drains == 3, "drained once per edge, never on a timeout");
    EXPECT(g_gpio.primes == 0, "a level line has nothing to prime");
    EXPECT(g_gpio.max_timeout_ms == 200, "no single wait exceeds 200 ms");
    EXPECT(g_gpio.opens == 1 && g_gpio.closes == 1, "the line is released");

    gpio_disarm();
    end(fb);
}

/*
 * A LATCHED data-ready (the magnetometer) is re-armed only by the acknowledge a
 * read performs, so it is primed once from inside the window and drained on a
 * timeout as well as on an edge — and its waits are cut to the daemon's own
 * fallback interval for the rate, because waiting longer there produces FEWER
 * edges, not later ones.
 */
static void test_gpio_latched_line(void)
{
    begin("test_gpio_latched_line");
    int fb = g_fail;

    static const int script[] = { 1, 0, 1 };
    gpio_arm(script, 3, 0);
    imt_gpio_why_t why = IMT_GPIO_OK;

    const int  odr      = 100000;                          /* milli-Hz */
    const long fallback = imu_int_fallback_ms(odr, 1, 1);

    int edges = imt_gpio_count_edges("gpiochip0", 27, 150,
                                     gpio_drain_cb, NULL, &why,
                                     gpio_prime_cb, odr);

    EXPECT(edges == 2, "only the edges are counted, not the timeouts");
    EXPECT(why == IMT_GPIO_OK, "and the line is not faulted");
    EXPECT(g_gpio.primes == 1, "primed exactly once, from inside the window");
    EXPECT(g_gpio.drains > edges, "a latched line is drained on timeouts too");
    EXPECT(g_gpio.max_timeout_ms == fallback,
           "waits are cut to the daemon's fallback for this rate");
    EXPECT(g_gpio.opens == 1 && g_gpio.closes == 1, "the line is released");

    gpio_disarm();
    end(fb);
}

static void test_gpio_no_edges_and_wait_error(void)
{
    begin("test_gpio_no_edges_and_wait_error");
    int fb = g_fail;

    /* A line that never moves: 0 edges is a real and damning measurement, so
     * it has its own reason code rather than an empty OK. */
    gpio_arm(NULL, 0, 0);
    imt_gpio_why_t why = IMT_GPIO_OK;
    int edges = imt_gpio_count_edges("gpiochip0", 17, 80,
                                     gpio_drain_cb, NULL, &why, NULL, 833000);
    EXPECT(edges == 0, "a silent line counts nothing");
    EXPECT(why == IMT_GPIO_NOEDGES, "and is reported as silent");
    EXPECT(g_gpio.drains == 0, "with no drain on a level line's timeouts");

    /* An error mid-window keeps what was already counted and stops. */
    static const int script[] = { 1, 1, -1 };
    gpio_arm(script, 3, 0);
    why = IMT_GPIO_OK;
    edges = imt_gpio_count_edges("gpiochip0", 17, 400,
                                 gpio_drain_cb, NULL, &why, NULL, 833000);
    EXPECT(edges == 2, "the edges seen before the error are kept");
    EXPECT(why == IMT_GPIO_EIO, "and the failure is reported");
    EXPECT(g_gpio.closes == 1, "the line is still released");

    gpio_disarm();
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("\n=== test_hwtools_e2e: imud-cal and imud-imutest, main() included "
           "===\n\n");

    tmppath(g_conf,      sizeof g_conf,      "sim.conf");
    tmppath(g_cal,       sizeof g_cal,       "cal.json");
    tmppath(g_sock,      sizeof g_sock,      "stream.sock");
    tmppath(g_cap_still, sizeof g_cap_still, "still.imucap");
    tmppath(g_cap_move,  sizeof g_cap_move,  "move.imucap");
    tmppath(g_conf_baddrv, sizeof g_conf_baddrv, "baddrv.conf");
    tmppath(g_conf_badbus, sizeof g_conf_badbus, "badbus.conf");
    tmppath(g_conf_hw,     sizeof g_conf_hw,     "hw.conf");
    tmppath(g_cap_wave,  sizeof g_cap_wave,  "wave.imucap");
    tmppath(g_cap_swing, sizeof g_cap_swing, "swing.imucap");
    tmppath(g_report,    sizeof g_report,    "report.md");

    write_configs();
    write_still_capture(g_cap_still, 4000, 100.0, 12.0, 0.0);
    write_still_capture(g_cap_move,  4000, 100.0, 12.0, 0.5);
    write_wave_capture(g_cap_wave, 833, 3.0);
    write_swing_capture(g_cap_swing, 20.0, 65.0);

    test_cal_version_and_help();
    test_cal_bad_argv();
    test_cal_missing_config();
    test_cal_characterize();
    test_cal_fit_temp();
    test_cal_fit_temp_flat();
    test_cal_offline_bad_capture();
    test_cal_fit_ra();
    test_cal_motion_gate();
    test_cal_unknown_driver();
    test_cal_bus_open_failure();
    test_cal_sensor_bringup_failure();
    test_cal_gyro();
    test_cal_gyro_declined();
    test_cal_accel_geometry_refused();
    test_cal_mag_swing();

    test_imutest_version_and_bad_option();
    test_imutest_missing_config();
    test_imutest_daemon_guard();
    test_imutest_degauss_unsupported();
    test_imutest_degauss_bus_open_failure();
    test_imutest_degauss_in_range();
    test_imutest_degauss_still_saturated();
    test_imutest_degauss_bus_errors();
    test_imt_run_open_errors();
    test_imutest_run_failure();
    test_imutest_passive_run();
    test_imutest_default_report_name();
    test_imutest_report_write_failure();

    test_gpio_disabled_and_open_errors();
    test_gpio_level_line();
    test_gpio_latched_line();
    test_gpio_no_edges_and_wait_error();

    /* Nothing outside a gpio case may reach a real line: this box is the bench
     * Pi, and gpiochip4 line 17 is the IMU's interrupt. */
    begin("test_no_stray_gpio_use");
    int fb = g_fail;
    EXPECT(g_gpio_unscripted == 0, "no unscripted imu_gpio_* call was made");
    end(fb);

    char stdin_path[128];
    remove(tmppath(stdin_path, sizeof stdin_path, "stdin.txt"));
    remove(g_conf);
    remove(g_conf_baddrv);
    remove(g_conf_badbus);
    remove(g_conf_hw);
    remove(g_cal);
    remove(g_cap_still);
    remove(g_cap_move);
    remove(g_cap_wave);
    remove(g_cap_swing);
    unlink(g_sock);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
