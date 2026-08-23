/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imutest.c — the driver-validation core (see include/imutest.h).
 *
 * Nothing here writes to a terminal or reads a keyboard: the operator is
 * reached only through imt_ui_t, which is what lets test/test_imutest.c drive
 * every phase — including the guided ones — against the mock I2C bus.
 *
 * Check ordering is deliberate and load-bearing:
 *   - bring-up mirrors imu_ctx_open() exactly (IMU probe/reset/init before the
 *     mag), because the bypass magnetometers only appear on the host bus once
 *     the IMU's init() has opened the bypass;
 *   - the seq checks run before the deliberate FIFO overflow, since an
 *     overflow produces a legitimate seq gap;
 *   - the full-scale sweep runs last of the passive checks, because every
 *     init() resets the driver's seq counter.
 */

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/utsname.h>

#include "imutest.h"
#include "cloexec.h"
#include "imu_gpio.h"
#include "imu_math.h"
#include "cal_math.h"
#include "imu_math.h"
#include "log.h"
#include "version.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* An I2C address nothing sane answers on: 0x7B is in the reserved range. */
#define IMT_BOGUS_ADDR 0x7B

/* ── Abort plumbing ────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_abort;

void imt_request_abort(void) { g_abort = 1; }

/* ── Small helpers ─────────────────────────────────────────────────────────── */

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void sleep_s(double s)
{
    if (s <= 0) return;
    struct timespec ts = { (time_t)s, (long)((s - (double)(time_t)s) * 1e9) };
    nanosleep(&ts, NULL);
}

static int tab_len(const int *t, int cap)
{
    int n = 0;
    while (n < cap && t[n] != 0) n++;
    return n;
}

const char *imt_status_str(imt_status_t s)
{
    switch (s) {
    case IMT_PASS: return "PASS";
    case IMT_WARN: return "WARN";
    case IMT_FAIL: return "FAIL";
    case IMT_INFO: return "INFO";
    default:       return "SKIP";
    }
}

const imt_check_t *imt_find(const imt_report_t *r, const char *id)
{
    for (int i = 0; i < r->n_checks; i++)
        if (strcmp(r->check[i].id, id) == 0) return &r->check[i];
    return NULL;
}

int imt_exit_code(const imt_report_t *r)
{
    if (r->aborted)    return 130;
    if (r->n_fail > 0) return 2;
    if (r->n_warn > 0) return 3;
    return 0;
}

void imt_opts_defaults(imt_opts_t *o)
{
    memset(o, 0, sizeof *o);
    o->phases          = IMT_PHASE_ALL;
    o->odr_window_s    = 5.0;
    o->noise_window_s  = 10.0;
    o->drdy_window_s   = 3.0;
    o->mag_window_s    = 5.0;
    o->face_settle_s   = 0.7;
    o->face_collect_s  = 2.0;
    o->turn_deg        = 90.0;
    o->turn_timeout_s  = 30.0;
    o->spin_timeout_s  = 180.0;
    /*
     * Gravity tolerance, from the PART's spec rather than from what a good
     * reading looks like.  ISM330DHCX DS13012: linear acceleration sensitivity
     * LA_So is -2%/+2%, and the zero-g level offset LA_TyOff is +/-65 mg --
     * 0.64 m/s^2 on its own, before any sensitivity error.  So an uncalibrated
     * part that is entirely within specification can read |a| anywhere in
     * roughly 9.0 to 10.6 m/s^2.
     *
     * The old +/-0.25 was tighter than that, and duly failed one: |a| at rest
     * measured 10.1, and the full-scale sweep FAILed or WARNed a different
     * range on nearly every run, the range moving with the ODR and between
     * runs -- scatter inside the part's own tolerance, read as a defect.
     *
     * Widening does not blunt the check.  What it exists to catch is a wrong
     * sensitivity CONSTANT, which is a factor of two or four -- the note
     * prints the ratio for exactly that -- and 0.85 still separates 9.8 from
     * 4.9 or 19.6 by a wide margin. Calibration is what closes the gap between
     * this band and a good reading, and imud-cal is where that belongs.
     */
    o->grav_tol_warn   = 0.85;
    o->grav_tol_fail   = 1.60;
    o->odr_tol_warn    = 0.05;
    o->odr_tol_fail    = 0.15;
    o->fs_sweep        = true;
    o->induce_overflow = true;
    o->regdiff         = true;
}

/* ── Check recording ───────────────────────────────────────────────────────── */

/*
 * All five fields go in at once so a check is never half-written, and every
 * caller is forced to say what it measured and what it expected — that pair is
 * what makes the report reviewable by someone without the hardware.
 *
 * The printf attribute is not decoration: nearly every note here is built from
 * a ternary that picks one of two format strings, and it is easy to give the
 * two branches different conversion counts while passing the argument list of
 * only one.  That reads off the end of the va_list and printed a garbage
 * "ratio to true g is 0.000" in a shipped report before this was added.  Keep
 * both branches of every ternary taking the same arguments.
 */
static void add_check(imt_report_t *r, const char *id, const char *name,
                      imt_status_t st, const char *measured,
                      const char *expected, const char *fmt, ...)
    __attribute__((format(printf, 7, 8)));

static void add_check(imt_report_t *r, const char *id, const char *name,
                      imt_status_t st, const char *measured,
                      const char *expected, const char *fmt, ...)
{
    if (r->n_checks >= IMT_MAX_CHECKS) return;
    imt_check_t *c = &r->check[r->n_checks++];

    snprintf(c->id,   sizeof c->id,   "%s", id);
    snprintf(c->name, sizeof c->name, "%s", name);
    c->status = st;
    snprintf(c->measured, sizeof c->measured, "%s", measured ? measured : "");
    snprintf(c->expected, sizeof c->expected, "%s", expected ? expected : "");

    if (fmt) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(c->note, sizeof c->note, fmt, ap);
        va_end(ap);
    }

    switch (st) {
    case IMT_PASS: r->n_pass++; break;
    case IMT_WARN: r->n_warn++; break;
    case IMT_FAIL: r->n_fail++; break;
    case IMT_INFO: r->n_info++; break;
    default:       r->n_skip++; break;
    }
}

static void skip_check(imt_report_t *r, const char *id, const char *name,
                       const char *why)
{
    add_check(r, id, name, IMT_SKIP, "-", "-", "%s", why);
}

/* Format helper for the measured/expected columns. */
static const char *fmtbuf(char *buf, size_t sz, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static const char *fmtbuf(char *buf, size_t sz, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sz, fmt, ap);
    va_end(ap);
    return buf;
}

/* ── UI shims (a NULL callback is always legal) ────────────────────────────── */

static int ui_prompt(const imt_opts_t *o, const char *id,
                     const char *title, const char *body)
{
    if (!o->ui.prompt) return 1;   /* no operator available -> skip the item */
    return o->ui.prompt(o->ui.user, id, title, body);
}

static int ui_poll_done(const imt_opts_t *o)
{
    return o->ui.poll_done ? o->ui.poll_done(o->ui.user) : 0;
}

static void ui_progress(const imt_opts_t *o, const char *id, double frac,
                        const char *detail)
{
    if (o->ui.progress) o->ui.progress(o->ui.user, id, frac, detail);
}

static void ui_coverage(const imt_opts_t *o, const int *sectors, int nsec,
                        int cur, int n, double radius)
{
    if (o->ui.coverage) o->ui.coverage(o->ui.user, sectors, nsec, cur, n, radius);
    else                ui_progress(o, "spin", -1.0, NULL);
}

/* ── Welford accumulation ──────────────────────────────────────────────────── */

typedef struct {
    uint64_t n;
    double   mean[3], m2[3], min[3], max[3];
} welford3_t;

static void w3_init(welford3_t *w)
{
    memset(w, 0, sizeof *w);
    for (int k = 0; k < 3; k++) { w->min[k] = 1e30; w->max[k] = -1e30; }
}

static void w3_add(welford3_t *w, const float v[3])
{
    w->n++;
    for (int k = 0; k < 3; k++) {
        double x = v[k];
        double d = x - w->mean[k];
        w->mean[k] += d / (double)w->n;
        w->m2[k]   += d * (x - w->mean[k]);
        if (x < w->min[k]) w->min[k] = x;
        if (x > w->max[k]) w->max[k] = x;
    }
}

static void w3_finish(const welford3_t *w, imt_stats3_t *out)
{
    out->n = w->n;
    for (int k = 0; k < 3; k++) {
        out->mean[k]  = w->n ? w->mean[k] : 0.0;
        out->sigma[k] = (w->n > 1) ? sqrt(w->m2[k] / (double)(w->n - 1)) : 0.0;
        out->min[k]   = w->n ? w->min[k] : 0.0;
        out->max[k]   = w->n ? w->max[k] : 0.0;
    }
}

/* ── Safe control-register ranges, per driver ─────────────────────────────── */

/*
 * Drivers are opaque, so the only chip-agnostic readback available is a
 * snapshot diff around init().  That is only safe with a per-driver map: a
 * blind scan would read a FIFO port (popping data) or a read-to-clear status
 * register.  Reads are single-byte for the same reason — a burst would
 * auto-increment straight into the data port.
 *
 * An unmapped driver SKIPs the regdiff check rather than guessing.
 *
 * Two kinds of register have to be kept out of the sweep, and they need
 * different mechanisms:
 *
 *   Destructive to read — a FIFO port, a read-to-clear status word.  These
 *   cannot be discovered by experiment without corrupting the run, so they
 *   are listed here.  `skip[]` names individual registers; `nrd_lo..nrd_hi`
 *   covers a contiguous window, which is what a FIFO port usually is.  The
 *   ST 6-axis parts are the reason the range exists: FIFO_DATA_OUT is seven
 *   registers wide (tag at 0x78, then X/Y/Z low/high through 0x7E), and
 *   listing only the first two left the sweep single-byte-reading five FIFO
 *   words per snapshot.
 *
 *   Volatile — sensor output, status, FIFO level, the timestamp counter.
 *   Safe to read, but they change on their own, so a diff across init() is
 *   dominated by them and an idempotency compare is meaningless.  These are
 *   NOT listed: reg_volatile_scan() finds them by reading the mapped range
 *   several times with the part running and no writes in between.  That is
 *   self-calibrating, needs no datasheet, and stays correct for drivers that
 *   do not exist yet — which matters, because this tree ships no datasheets
 *   and a hand-written volatile table would be exactly the kind of unverified
 *   register knowledge it avoids.
 */
typedef struct {
    const char *driver;
    uint8_t     lo, hi;
    uint8_t     skip[8];
    int         nskip;
    /*
     * Address ranges the datasheet marks RESERVED, or does not list at all.
     *
     * NEVER READ THESE.  The sweep used to walk lo..hi blind, skipping only
     * the FIFO port, which meant ~60 reserved addresses per snapshot and about
     * 420 reserved reads per run once the volatile scan and the idempotency
     * compare are counted.  On the reference ISM330DHCX that is not harmless:
     * the part is clean at power-up, and after ONE run roughly 1 register read
     * in 100 comes back with the wrong byte, persistently, across processes.
     * A power cycle clears it; nothing in software does.
     *
     * Reading an undefined address is not a read of nothing -- on this family
     * 0x60-0x62 are marked RESERVED *RW*, and the embedded-function bank hides
     * behind FUNC_CFG_ACCESS at 0x01.  Poking at either is how a diagnostic
     * tool becomes the fault it is looking for.
     */
    struct { uint8_t lo, hi; } resv[10];
    int         nresv;
    uint8_t     bank_reg;        /* 0x00 = not banked */
    uint8_t     nrd_lo, nrd_hi;  /* destructive window; lo > hi = none */
    /*
     * The part's measurement-output window, if it has a contiguous one.
     * lo > hi = not declared, and the framing check below SKIPs.
     *
     * Used only by check_burst_framing(), which reads it two ways and compares.
     * It has to be a window the driver itself bursts, because the claim under
     * test is that this part's multi-byte read lands where the driver assumes.
     */
    uint8_t     out_lo, out_hi;
    bool        ctrl_writeonly;  /* control registers do not read back at all */
    /*
     * Register reporting the part's own timebase error, if it has one.
     * 0x00 = none.
     *
     * The ST 6-axis parts carry INTERNAL_FREQ_FINE (0x63): an 8-bit two's
     * complement count of 0.15% steps by which this individual part's ODR and
     * timestamp rate differ from typical (DS13012 §9.36, Table 139), with
     *
     *     TS_Res = 1 / (40000 + 0.0015 * FREQ_FINE * 40000)
     *
     * That turns imu.chipts.wall's implied tick from an inference into a
     * cross-check: the bench measured 1.041 on the reference ISM330DHCX, and
     * if the part also DECLARES roughly +27 steps, the fast oscillator is
     * confirmed by the chip rather than deduced from a ratio.  If the two
     * disagree, something other than the oscillator is moving the timebase,
     * which is a materially different finding.
     *
     * Read-only and side-effect-free, so it costs one byte on the wire.
     */
    uint8_t     freq_fine_reg;

    /*
     * The part's identity register and the value it must always hold.
     *
     * This is what imu.bus.integrity hammers, and picking it correctly is the
     * whole check.  It used to read freq_fine_reg instead, on the reasoning
     * that a factory trim is something nothing writes -- but "nothing writes
     * it" is not the same as "it cannot change".  Measured on the reference
     * ISM330DHCX: INTERNAL_FREQ_FINE reads 0x1B while the part is running and
     * 0x1A with the sensors powered down, because it reports a trim of an
     * oscillator that is switched off.  The check counted that transition as
     * bus corruption, which is how it produced a FAIL on a part whose every
     * other check passed.
     *
     * WHO_AM_I has no such state: it is hard-wired, identical in every power
     * mode, and the one byte on these parts that genuinely cannot change.
     * 0 means the part has none the sweep can reach, and the check falls back
     * to probe().  Two parts land on that by accident rather than by lacking
     * an identity -- the AK8963's WIA and the ICM-20948's WHO_AM_I are both
     * register 0x00 -- so they take the fallback too.  That costs the
     * wrong-value/io-error split on those parts and nothing else; inventing a
     * second "is it set" flag to recover it would be more machinery than the
     * distinction is worth.
     *
     * The RM3100 genuinely has none: PNI documents no fixed value for REVID,
     * so there is nothing to compare against.
     */
    uint8_t     whoami_reg, whoami_val;

    /*
     * Registers that ARE volatile but that the experiment cannot prove are.
     *
     * reg_volatile_scan() decides by reading the map several times and marking
     * whatever moved.  That misses any register which is changing but happens
     * to read the same value on every pass -- and a counter that has SATURATED
     * is exactly that.  The ST FIFO status pair is the case in hand: at
     * 6664 Hz the FIFO refills to capacity between passes, so DIFF_FIFO reads
     * its maximum every time and is classified static.  init() then flushes
     * the FIFO, the value drops, and imu.init.idempotent reports "2 registers
     * differ" -- which asks whether the FIFO was emptied, not whether init()
     * is idempotent.  That false positive is what sent a bench investigation
     * after a driver bug that was never there.
     *
     * Declared per part rather than inferred, because the whole point is that
     * inference cannot reach them.
     */
    uint8_t     vol_reg[4];
    int         nvol_reg;
} imt_regmap_t;

static const imt_regmap_t imt_regmaps[] = {
    /* ST: FIFO_DATA_OUT is 0x78 (tag) through 0x7E (Z high). */
    { .driver = "ism330dhcx", .lo = 0x00, .hi = 0x7F,
      .nrd_lo = 0x78, .nrd_hi = 0x7E, .freq_fine_reg = 0x63,
      .whoami_reg = 0x0F, .whoami_val = 0x6B,
      /* FIFO_STATUS1/2: DIFF_FIFO saturates, so it can read static. */
      .vol_reg = { 0x3A, 0x3B }, .nvol_reg = 2,
      .resv = { {0x00,0x00}, {0x03,0x06}, {0x1F,0x1F}, {0x2E,0x34},
                {0x3C,0x3F}, {0x44,0x55}, {0x60,0x62}, {0x64,0x6E},
                {0x76,0x77} }, .nresv = 9 },
    { .driver = "lsm6dso",    .lo = 0x00, .hi = 0x7F,
      .nrd_lo = 0x78, .nrd_hi = 0x7E, .freq_fine_reg = 0x63,
      .whoami_reg = 0x0F, .whoami_val = 0x6C,
      /* FIFO_STATUS1/2: DIFF_FIFO saturates, so it can read static. */
      .vol_reg = { 0x3A, 0x3B }, .nvol_reg = 2,
      .resv = { {0x00,0x00}, {0x03,0x06}, {0x1F,0x1F}, {0x2E,0x34},
                {0x3C,0x3F}, {0x44,0x55}, {0x60,0x62}, {0x64,0x6E},
                {0x76,0x77} }, .nresv = 9 },
    { .driver = "lsm6dsox",   .lo = 0x00, .hi = 0x7F,
      .nrd_lo = 0x78, .nrd_hi = 0x7E, .freq_fine_reg = 0x63,
      .whoami_reg = 0x0F, .whoami_val = 0x6D,
      /* FIFO_STATUS1/2: DIFF_FIFO saturates, so it can read static. */
      .vol_reg = { 0x3A, 0x3B }, .nvol_reg = 2,
      .resv = { {0x00,0x00}, {0x03,0x06}, {0x1F,0x1F}, {0x2E,0x34},
                {0x3C,0x3F}, {0x44,0x55}, {0x60,0x62}, {0x64,0x6E},
                {0x76,0x77} }, .nresv = 9 },
    /* TDK: FIFO ports and banked register files. */
    { .driver = "icm42688p",  .lo = 0x00, .hi = 0x7F,
      .skip = { 0x2E, 0x2F, 0x30 }, .nskip = 3, .bank_reg = 0x76,
      .whoami_reg = 0x75, .whoami_val = 0x47,
      .nrd_lo = 1, .nrd_hi = 0 },
        /* icm20948 WHO_AM_I is bank-0 register 0x00, and 0 is this field's
     * "no identity register" sentinel, so it uses the probe() fallback. */
    { .driver = "icm20948",   .lo = 0x00, .hi = 0x7F,
      .skip = { 0x72, 0x73, 0x74 }, .nskip = 3, .bank_reg = 0x7F,
      .nrd_lo = 1, .nrd_hi = 0 },
    { .driver = "mpu9250",    .lo = 0x00, .hi = 0x7F,
      .whoami_reg = 0x75, .whoami_val = 0x71,
      .skip = { 0x74 }, .nskip = 1, .nrd_lo = 1, .nrd_hi = 0 },
    { .driver = "mpu9255",    .lo = 0x00, .hi = 0x7F,
      .whoami_reg = 0x75, .whoami_val = 0x73,
      .skip = { 0x74 }, .nskip = 1, .nrd_lo = 1, .nrd_hi = 0 },
    /*
     * Mags.  The MMC5983MA's readable file ends at 0x08, which is read-to-clear
     * status; its FOUR control registers (CTRL0 0x09, CTRL1 0x0A, CTRL2 0x0B,
     * CTRL3 0x0C) are write-only — Rev A gives them as Mode W, and
     * src/drivers/mmc5983ma.c marks them so.  A readback diff across init() is
     * therefore structurally empty on this part, which is a fact about the
     * silicon and not a finding about the driver; ctrl_writeonly makes the
     * check SKIP and say that.
     *
     * hi used to be 0x1F, which named a range this part does not have: it
     * covered all four write-only registers and 0x0D-0x1F of reserved space.
     * Nothing read them, because ctrl_writeonly skips the snapshot outright —
     * but a map that describes a register file the silicon does not have is a
     * loaded gun for whoever clears that flag or copies the entry for a part
     * with a mixed control file.  0x08 is the last readable register, and it
     * is itself read-to-clear, so the swept set is 0x00-0x07.
     */
    { .driver = "mmc5983ma",  .lo = 0x00, .hi = 0x08,
      .whoami_reg = 0x2F, .whoami_val = 0x30,
      .skip = { 0x08 }, .nskip = 1, .nrd_lo = 1, .nrd_hi = 0,
      .out_lo = 0x00, .out_hi = 0x06,   /* XOUT0..XYZOUT2, the driver's burst */
      .ctrl_writeonly = true },
    /*
     * AKM parts, both of which need two things the old entries got wrong.
     *
     * First, DRDY: the datasheet says the bit "returns to 0 when any one of
     * ST2 register or the measurement data registers (HXL to HZH) is read".
     * Naming only the first data register left the rest of the burst in the
     * sweep, so the snapshot completed the measurement the next check was
     * waiting on.  nrd_lo..nrd_hi now covers the whole data block plus ST2.
     *
     * Second, and worse: both parts carry read/WRITE test registers that the
     * vendor marks DO NOT ACCESS — TS1/TS2 at 0x0D-0x0E on the AK8963
     * ("0DH and 0EH are reserved addresses. Do not access to those
     * addresses."), at 0x33-0x34 on the AK09916 ("test registers for shipment
     * test. Do not access these registers.").  hi was 0x1F and 0x3F, so the
     * sweep read straight across them, and across 0x13 RSV on the AK8963,
     * likewise DO NOT ACCESS.  hi now stops at the last register each part
     * actually documents, and the interior holes are marked reserved.
     *
     * The AK8963's file is 0x00-0x0C plus 0x10-0x12; 0x10-0x12 (the fuse-ROM
     * sensitivity values) read correctly only in fuse-ROM access mode, so they
     * are swept but mean nothing outside it.  The AK09916's is 0x00-0x01,
     * 0x10-0x18 and 0x30-0x32.
     */
    { .driver = "ak09916",    .lo = 0x00, .hi = 0x32,
      .whoami_reg = 0x01, .whoami_val = 0x09,   /* WIA2; WIA1 is 0x00 */
      .skip = { 0x10 }, .nskip = 1, .nrd_lo = 0x11, .nrd_hi = 0x18,
      .resv = { {0x02,0x0F}, {0x19,0x2F} }, .nresv = 2 },
    { .driver = "ak8963",     .lo = 0x00, .hi = 0x12,
      .whoami_reg = 0x00, .whoami_val = 0x48,
      .skip = { 0x02 }, .nskip = 1, .nrd_lo = 0x03, .nrd_hi = 0x09,
      .resv = { {0x0D,0x0E} }, .nresv = 1 },
    /* lis3mdl is the one part with a real auto-increment bit, so it is the one
     * where the framing check has something to catch: OUT_X_L..OUT_Z_H. */
    { .driver = "lis3mdl",    .lo = 0x00, .hi = 0x3F, .nrd_lo = 1, .nrd_hi = 0,
      .whoami_reg = 0x0F, .whoami_val = 0x3D,
      .out_lo = 0x28, .out_hi = 0x2D },
    { .driver = "lis2mdl",    .lo = 0x00, .hi = 0x3F, .nrd_lo = 1, .nrd_hi = 0,
      .whoami_reg = 0x4F, .whoami_val = 0x40 },
    /* PNI: reading the measurement results (0x24-0x2C) is what CLEARS DRDY,
     * so a sweep through them would consume the sample the next check is
     * waiting for. */
    { .driver = "rm3100",     .lo = 0x00, .hi = 0x3F,
      .nrd_lo = 0x24, .nrd_hi = 0x2C },
};

static const imt_regmap_t *regmap_for(const char *driver)
{
    for (size_t i = 0; i < sizeof imt_regmaps / sizeof imt_regmaps[0]; i++)
        if (strcmp(imt_regmaps[i].driver, driver) == 0) return &imt_regmaps[i];
    return NULL;
}

static bool regmap_skips(const imt_regmap_t *m, uint8_t reg)
{
    for (int i = 0; i < m->nskip; i++)
        if (m->skip[i] == reg) return true;
    for (int i = 0; i < m->nresv; i++)
        if (reg >= m->resv[i].lo && reg <= m->resv[i].hi) return true;
    if (m->nrd_lo <= m->nrd_hi && reg >= m->nrd_lo && reg <= m->nrd_hi)
        return true;
    /* Never read the bank selector itself as part of the sweep. */
    return m->bank_reg && reg == m->bank_reg;
}

/* ── Daemon-conflict probe ────────────────────────────────────────────────── */

/* True if something is listening on this AF_UNIX path. */
static bool socket_answers(const char *path)
{
    struct sockaddr_un addr;
    size_t plen = path ? strlen(path) : 0;
    if (plen == 0 || plen >= sizeof addr.sun_path) return false;

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;
    APPLY_CLOEXEC(fd);

    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, path, plen);

    bool up = connect(fd, (struct sockaddr *)&addr, sizeof addr) == 0;
    close(fd);
    return up;
}

/*
 * Is a daemon holding the sensors?
 *
 * Opening the bus succeeds even when another process has it, so the reliable
 * signal is the daemon's own stream socket. Both processes would drain the same
 * FIFO, so each sees about half the samples: the measured ODR reads low, the
 * mag rate reads low, and seq.gapless fails -- all at once, on both sensors, at
 * every rate. Recognising that signature costs a bench session if the guard
 * does not fire first.
 *
 * TWO paths, which is the whole point. Probing only the CONFIGURED socket
 * means a bench config naming a private path -- `socket = "/tmp/bench.sock"` --
 * connects to nothing, concludes no daemon is running, and waves the run
 * through, while the installed daemon sits on the default path draining the
 * FIFO. That is not hypothetical: it voided a full day of measurements, and the
 * guard's own comment described the exact failure it had just permitted.
 *
 * `fallback` is a parameter rather than a constant so a test can drive both
 * paths without binding a socket under /run.
 */
bool imt_daemon_conflict(const char *configured, const char *fallback)
{
    if (socket_answers(configured)) return true;
    if (fallback && configured && strcmp(fallback, configured) == 0) return false;
    return socket_answers(fallback);
}

bool imt_daemon_running(const imud_config_t *cfg)
{
    /* The default comes from config_defaults() rather than a second copy of the
     * path literal, so this cannot drift away from what the daemon listens on. */
    static imud_config_t dflt;
    config_defaults(&dflt);
    return imt_daemon_conflict(cfg->stream_socket, dflt.stream_socket);
}

/*
 * Test seam: would the control-register sweep read `reg` on `driver`?
 *
 * Exposed because the reserved and DO-NOT-ACCESS exclusions are table data,
 * and the bus mock can only stand in for one part pair — this lets the suites
 * pin the exclusions for every registered driver, including the AKM
 * magnetometers whose vendor test registers must never be touched.
 */
/*
 * Test seam: the identity register and value imu.bus.integrity compares
 * against, for `driver`.  Exposed because choosing this wrongly is what made
 * the check report bus corruption on a healthy part -- it read
 * INTERNAL_FREQ_FINE, which changes with power state -- and a registry entry
 * is data that no mock run can validate.
 */
/*
 * Test seam: is `reg` declared known-volatile for `driver`?
 *
 * Exposed because the failure this guards against is invisible to a mock run:
 * a saturated counter reads identical every pass, so a test that only checks
 * "the scan found the volatile registers" passes whether or not the
 * declaration exists.
 */
bool imt_regmap_known_volatile(const char *driver, uint8_t reg)
{
    const imt_regmap_t *m = regmap_for(driver);
    if (!m) return false;
    for (int i = 0; i < m->nvol_reg; i++)
        if (m->vol_reg[i] == reg) return true;
    return false;
}

bool imt_regmap_identity(const char *driver, uint8_t *reg, uint8_t *val)
{
    const imt_regmap_t *m = regmap_for(driver);
    if (!m || !m->whoami_reg) return false;
    if (reg) *reg = m->whoami_reg;
    if (val) *val = m->whoami_val;
    return true;
}

bool imt_regmap_reads(const char *driver, uint8_t reg)
{
    const imt_regmap_t *m = regmap_for(driver);
    if (!m) return false;
    if (reg < m->lo || reg > m->hi) return false;
    return !regmap_skips(m, reg);
}

/*
 * The register snapshot needs raw bus access, so this file includes the
 * drivers' private bus header.  That is the same single-ioctl path every
 * driver uses, which keeps the snapshot visible to test/bus_mock.c, and it is
 * why the tool is Linux-only exactly like the drivers are.
 */
#include "drivers/bus_io.h"

/* Single-byte snapshot of a device's safe control registers. */
static int reg_snapshot(const imud_bus_t *bus, const imt_regmap_t *m, uint8_t *out)
{
    for (int reg = m->lo; reg <= (int)m->hi; reg++) {
        if (regmap_skips(m, (uint8_t)reg)) { out[reg] = 0; continue; }
        uint8_t v = 0;
        if (bus_reg_read(bus, (uint8_t)reg, &v) < 0) return -1;
        out[reg] = v;
    }
    return 0;
}

/*
 * Mark every register in the mapped range that changes on its own.
 *
 * Called with the part running and configured, and with no writes in between
 * passes, so anything that moves is sensor output, a status word, a FIFO
 * level, or a timestamp counter — never a control register.  `ref` is the
 * post-init snapshot the caller already holds; each pass is compared against
 * it and the differences are accumulated, so a register only has to move once
 * across the whole probe to be caught.
 *
 * Passes are spaced so slowly-batched words get a chance to move too: on the
 * ISM330DHCX the temperature is batched at 12.5 Hz, an order of magnitude
 * below the 833 Hz accel and gyro.
 *
 * Returns the number marked, or -1 if the bus failed.  A register that holds
 * the same value through every pass stays unmarked — a stationary board's
 * accelerometer high byte is the usual case — so the filter reduces the noise
 * in the diff rather than eliminating it.  Section 4 of the report says so.
 */
#define IMT_VOLATILE_PASSES 4

static int reg_volatile_scan(const imud_bus_t *bus, const imt_regmap_t *m,
                             const uint8_t *ref, bool *vol)
{
    static uint8_t probe[256];

    memset(vol, 0, 256 * sizeof *vol);

    /* Known-volatile first: a saturated counter reads identical on every pass
     * and the loop below would call it static. */
    for (int i = 0; i < m->nvol_reg; i++)
        if (m->vol_reg[i] >= m->lo && m->vol_reg[i] <= m->hi)
            vol[m->vol_reg[i]] = true;

    for (int pass = 0; pass < IMT_VOLATILE_PASSES; pass++) {
        if (pass) sleep_s(0.03);
        memset(probe, 0, sizeof probe);
        if (reg_snapshot(bus, m, probe) < 0) return -1;
        for (int reg = m->lo; reg <= (int)m->hi; reg++) {
            if (regmap_skips(m, (uint8_t)reg)) continue;
            if (probe[reg] != ref[reg]) vol[reg] = true;
        }
    }

    int n = 0;
    for (int reg = m->lo; reg <= (int)m->hi; reg++) if (vol[reg]) n++;
    return n;
}

/* `vol` may be NULL when no volatile scan was possible. */
static int reg_diff(const uint8_t *before, const uint8_t *after,
                    const imt_regmap_t *m, const bool *vol,
                    imt_regdiff_t *out, int cap)
{
    int n = 0;
    for (int reg = m->lo; reg <= (int)m->hi && n < cap; reg++) {
        if (regmap_skips(m, (uint8_t)reg)) continue;
        if (vol && vol[reg]) continue;
        if (before[reg] != after[reg]) {
            out[n].reg    = (uint8_t)reg;
            out[n].before = before[reg];
            out[n].after  = after[reg];
            n++;
        }
    }
    return n;
}

/* ── IMU drain helper ──────────────────────────────────────────────────────── */

typedef struct {
    const imu_ops_t *ops;
    const imud_bus_t *bus;
    /*
     * The tick period to grade against: what imu.c will actually use, which is
     * the part's own declared period where it has one.  Grading against the
     * datasheet typical instead would report a healthy part as a defect —
     * the reference ISM330DHCX is 4% off typical and entirely in spec.
     */
    uint32_t         tick_ns;
    /* Running contract observations, accumulated across every drain. */
    bool             have_seq;
    uint32_t         last_seq;
    uint64_t         gaps, backwards, max_gap;
    int              rc1, rcneg, last_errno;
    uint64_t         total;

    /*
     * The interrupt line the DAEMON would wait on, held for the whole run.
     *
     * Every check here drains through drain_pace(), which waits on this line
     * with the daemon's cadence instead of a timer of imutest's own. That
     * difference used to be invisible and expensive: paced on a 5 ms sleep,
     * chip_ts reversals appeared at burst seams that the daemon -- woken by
     * the watermark -- never produces, and the report blamed the driver. The
     * daemon scored 0 reversals in 53,708 samples while this tool scored
     * several per window on the same part minutes apart.
     *
     * NULL when no line is configured, which is a real deployment: the reader
     * then paces itself, and so does this.
     */
    imu_gpio_line_t *line;
} drain_ctx_t;

/*
 * One pacing step, exactly as ism_reader performs it: wait on the watermark
 * with the daemon's timeout, or sleep that long where no line is configured.
 * Never a shorter timer -- what this tool measures has to be what the daemon
 * gets, and the pacing is most of that.
 */
static void drain_pace(drain_ctx_t *d)
{
    /*
     * Used by the checks that GRADE THE READ PATH -- check_odr_seq_ts, which
     * owns imu.odr, imu.seq.* and every imu.chipts.* verdict, and check_rest.
     * Those must traverse the path the way the daemon does, because the thing
     * they measure is a property of the pacing: a timer-paced drain arrives at
     * an arbitrary phase, the batched FIFO timestamp is rejected, and the
     * post-drain anchor can place a burst before the previous one ended. The
     * daemon, woken by the watermark, scored 0 reversals in 53,708 samples
     * while this tool scored several per window on the same part.
     *
     * Deliberately NOT used by collect_stats() or the guided phases. Those
     * grade physical orientation and noise while an operator holds the board
     * still; the drain cadence is not what they measure, and pacing them at it
     * only collects fewer samples per face than the sign check needs.
     */
    if (d->line) {
        (void)imu_gpio_wait_edge(d->line, IMU_DRAIN_WAIT_MS);
        return;
    }
    sleep_s(IMU_DRAIN_WAIT_MS / 1000.0);
}

static void drain_init(drain_ctx_t *d, const imu_ops_t *ops,
                       const imud_bus_t *bus, const imud_config_t *cfg)
{
    memset(d, 0, sizeof *d);
    d->ops = ops; d->bus = bus;
    if (cfg && cfg->imu_int_gpio > 0)
        d->line = imu_gpio_open(cfg->gpio_chip, (unsigned)cfg->imu_int_gpio,
                                "imud-imutest");
    d->tick_ns = ops->ts_tick_ns;
    if (ops->ts_tick_ns_actual) {
        uint32_t part = ops->ts_tick_ns_actual(bus);
        if (part != 0) d->tick_ns = part;
    }
}

/*
 * One read() call, with the seq contract checked on every sample.  Unsigned
 * deltas make the 32-bit wrap correct for free.  Returns the driver's rc.
 */
static int drain_once(drain_ctx_t *d, imu_sample_t *buf, int max, int *n)
{
    int rc = d->ops->read(d->bus, buf, max, n);
    if (rc < 0) { d->rcneg++; d->last_errno = errno; *n = 0; return rc; }
    if (rc > 0) d->rc1++;

    for (int i = 0; i < *n; i++) {
        if (d->have_seq) {
            uint32_t delta = buf[i].seq - d->last_seq;   /* wrap-safe */
            if (delta == 0 || delta >= 0x80000000u) {
                d->backwards++;
            } else if (delta > 1) {
                d->gaps++;
                if (delta - 1 > d->max_gap) d->max_gap = delta - 1;
            }
        }
        d->last_seq = buf[i].seq;
        d->have_seq = true;
        d->total++;
    }
    return rc;
}

/* Drain until empty (bounded), discarding samples. */
static void drain_flush(drain_ctx_t *d)
{
    imu_sample_t buf[128];
    int n = 0;
    for (int i = 0; i < 64; i++) {
        if (drain_once(d, buf, 128, &n) < 0) return;
        if (n == 0) return;
    }
}

/* ── Phase A: probe / reset / init ─────────────────────────────────────────── */

/* Returns 0 if the device came up, -1 if a prerequisite failed. */
static int check_bringup(imt_report_t *r, const imt_opts_t *o,
                         const imud_bus_t *ibus, const imud_bus_t *mbus,
                         const imu_ops_t *imu, const mag_ops_t *mag,
                         const imud_config_t *cfg, const imu_cfg_t *icfg,
                         const mag_cfg_t *mcfg, bool *mag_ok)
{
    char mb[56];

    /* ── probe ───────────────────────────────────────────────────────────── */
    if (imu->probe(ibus) < 0) {
        add_check(r, "imu.probe", "IMU probe() / chip identification", IMT_FAIL,
                  "rejected", "accepted",
                  "probe() failed at 0x%02X. Wrong address, wrong driver, or "
                  "the part is not present. Check `i2cdetect -y` for the bus.",
                  cfg->imu_addr);
        return -1;
    }
    add_check(r, "imu.probe", "IMU probe() / chip identification", IMT_PASS,
              fmtbuf(mb, sizeof mb, "accepted at 0x%02X", cfg->imu_addr),
              "accepted", "driver '%s' recognised the part", imu->name);


    /*
     * A probe that ignores WHO_AM_I, or swallows the ioctl error, passes the
     * check above for the wrong reason.  Nothing answers at the reserved
     * address, so a driver that accepts it is not identifying anything.
     *
     * Only on I2C.  The lever is i2c_addr, and on SPI the chip select does the
     * addressing — the field is not on the wire at all, so the "bogus" probe
     * reads the same part, gets the right WHO_AM_I back, and returns 0.  That
     * is the check misfiring, not the driver failing it, and left as a FAIL it
     * put two phantom rows and a nonzero exit on every SPI report forever.
     */
    if (ibus->kind == BUS_SPI) {
        skip_check(r, "imu.probe.reject", "IMU probe() rejects a bogus address",
                   "chip select addresses the part on SPI, so i2c_addr never "
                   "reaches the wire and a bogus one reads the same chip; "
                   "probing a neighbouring chip select could disturb it");
    } else {
        imud_bus_t ibogus = *ibus;
        ibogus.i2c_addr = IMT_BOGUS_ADDR;
        if (imu->probe(&ibogus) == 0)
            add_check(r, "imu.probe.reject", "IMU probe() rejects a bogus address",
                      IMT_FAIL, "accepted", "rejected",
                      "probe() returned 0 at unused address 0x%02X — it is not "
                      "checking WHO_AM_I, or it is discarding the bus error.",
                      IMT_BOGUS_ADDR);
        else
            add_check(r, "imu.probe.reject", "IMU probe() rejects a bogus address",
                      IMT_PASS, "rejected", "rejected",
                      "no false positive at unused address 0x%02X", IMT_BOGUS_ADDR);
    }

    /* ── reset ───────────────────────────────────────────────────────────── */
    double t0 = now_s();
    int rc = imu->reset(ibus);
    double ms = (now_s() - t0) * 1e3;
    r->raw.reset_ms = ms;

    if (rc < 0) {
        add_check(r, "imu.reset.rc", "IMU reset()", IMT_FAIL, "failed", "0",
                  "reset() returned -1 after %.1f ms — the reset bit never "
                  "self-cleared, or the bus dropped.", ms);
        return -1;
    }
    add_check(r, "imu.reset.rc", "IMU reset()", IMT_PASS, "0", "0",
              "completed in %.1f ms", ms);

    if (ms < 1.0)
        add_check(r, "imu.reset.ms", "IMU reset() settle time", IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%.2f ms", ms), "> 1 ms",
                  "no measurable settle — check reset() both polls the "
                  "self-clearing bit and waits the datasheet turn-on time.");
    else if (ms > 500.0)
        add_check(r, "imu.reset.ms", "IMU reset() settle time", IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%.0f ms", ms), "< 500 ms",
                  "unusually long — close to a poll timeout.");
    else
        add_check(r, "imu.reset.ms", "IMU reset() settle time", IMT_INFO,
                  fmtbuf(mb, sizeof mb, "%.1f ms", ms), "informational",
                  "time from reset() entry to return.");

    /* ── init, with the register snapshot around it ──────────────────────── */
    char ob[16];             /* MHZ_STR scratch */
    const imt_regmap_t *rm = o->regdiff ? regmap_for(imu->name) : NULL;
    static uint8_t before[256], after[256], again[256];
    static bool    volatile_imu[256];
    bool snapped = false;

    if (rm) {
        memset(before, 0, sizeof before);
        snapped = (reg_snapshot(ibus, rm, before) == 0);
    }

    if (imu->init(ibus, icfg) < 0) {
        add_check(r, "imu.init.rc", "IMU init()", IMT_FAIL, "failed", "0",
                  "init() returned -1 for ODR %s Hz, %d g, %d dps, wm %d.",
                  MHZ_STR(ob, icfg->odr_mhz), icfg->accel_g, icfg->gyro_dps, icfg->fifo_wm);
        return -1;
    }
    add_check(r, "imu.init.rc", "IMU init()", IMT_PASS, "0", "0",
              "%s Hz, +/-%d g, +/-%d dps, watermark %d sample-sets",
              MHZ_STR(ob, icfg->odr_mhz), icfg->accel_g, icfg->gyro_dps,
              icfg->fifo_wm);

    /*
     * BUS INTEGRITY.  probe() reads a register whose value cannot change --
     * the part's identity -- and compares it, so a failure at the REAL address
     * is a corrupted transfer rather than a wrong part.  Repeating it counts
     * corruption directly instead of leaving it to be inferred.
     *
     * It has been inferred twice today, and expensively.  A post-drain
     * TIMESTAMP0 read came back 65,706 ticks (1.58 s) high and poisoned the
     * timestamp chain for eleven bursts; INTERNAL_FREQ_FINE (0x63), a
     * factory-trim register nothing writes, read differently across two
     * init()s and surfaced as imu.init.idempotent. Both were single corrupted
     * reads, and neither was reported as one -- they arrived disguised as a
     * clock defect and a state-dependent init.
     *
     * Cheap enough to hammer: one identity byte per call.
     *
     * Placed AFTER reset() and init() deliberately.  It used to run straight
     * after probe(), which measured the part in whatever state the previous
     * process happened to leave it in -- not a state the daemon ever operates
     * in, and on these parts a materially different one: an ISM330DHCX left
     * with its sensors powered down answers reads far less reliably than the
     * same part configured and running.  The question this check exists to
     * answer is whether the bus is sound while the daemon is using it, so it
     * is measured on a part the driver has just brought up.
     */
    {
        /*
         * Two failure modes, counted apart, because they call for different
         * things: a transfer that ERRORED never delivered a byte (a driver or
         * kernel-level fault), while one that SUCCEEDED and delivered the
         * wrong byte is corruption on the wire.  Reporting them as one number
         * is how a bus problem gets mistaken for a driver problem -- which is
         * exactly what happened to the TIMESTAMP0 read and to 0x63.
         *
         * Where the part has an identity register the check reads that
         * directly, so the two are separable; probe() collapses them into one
         * -1 and is the fallback for parts without one.
         *
         * The expected byte comes from the registry, NOT from the first read.
         * Seeding a reference from read #1 means a single corrupt read at the
         * start inverts the whole result -- every good read afterwards counts
         * as bad -- and the check cannot tell which case it is in.  A
         * hard-wired identity has a known answer, so it is simply compared.
         */
        int bad = 0, bad2 = 0, ioerr = 0;
        const imt_regmap_t *irm = regmap_for(imu->name);
        const uint8_t idreg = (irm && irm->whoami_reg) ? irm->whoami_reg : 0;
        const uint8_t idval = irm ? irm->whoami_val : 0;
        for (int i = 0; i < IMT_BUS_INTEGRITY_READS && !g_abort; i++) {
            if (idreg) {
                uint8_t v;
                if (bus_reg_read(ibus, idreg, &v) < 0) { ioerr++; continue; }
                if (v != idval) {
                    /* Record WHAT came back, not just that it differed: a
                     * shifted copy of the right value says clocking, 0x00 or
                     * 0xFF says the part never drove the line, and anything
                     * else says the read landed somewhere it should not. */
                    if (r->raw.n_bus_bad < IMT_MAX_BUS_BAD) {
                        r->raw.bus_bad_val[r->raw.n_bus_bad] = v;
                        r->raw.bus_bad_at[r->raw.n_bus_bad]  = i;
                        r->raw.n_bus_bad++;
                    }
                    r->raw.bus_ref_imu = idval;
                    bad++;
                }
            } else if (imu->probe(ibus) < 0) {
                bad++;
            }
            /*
             * Cross-check a SECOND fixed register through probe(), so the
             * count cannot be blamed on the one register chosen.  One register
             * misreading is a question about that register; two is the bus.
             */
            if (idreg && imu->probe(ibus) < 0) bad2++;
        }
        double pct = 100.0 * (double)bad / (double)IMT_BUS_INTEGRITY_READS;
        r->raw.bus_bad_imu = bad;
        r->raw.bus_ioerr_imu = ioerr;
        const char *val = fmtbuf(mb, sizeof mb,
                                 "%d+%d of %d bad, %d io-err",
                                 bad, bad2, IMT_BUS_INTEGRITY_READS,
                                 ioerr);
        if (bad == 0)
            add_check(r, "imu.bus.integrity", "Bus reads are not corrupted",
                      IMT_PASS, val, "0 bad",
                      "%d reads of a register that cannot change all came back "
                      "with the value it cannot change from",
                      IMT_BUS_INTEGRITY_READS);
        else
            add_check(r, "imu.bus.integrity", "Bus reads are not corrupted",
                      imt_bus_integrity_status(bad,
                                              IMT_BUS_INTEGRITY_READS),
                      val, "0 bad",
                      "the identity register read back wrong %.2f%% of the "
                      "time, against a value that is hard-wired and cannot "
                      "change. The bytes that came back are in the appendix; "
                      "read them before assuming a cause. Candidates, roughly "
                      "in order of how often they are the answer: the part is "
                      "in a state the driver did not put it in (power-cycle it "
                      "and re-run before anything else), spi_speed_hz landing "
                      "on a clock the controller cannot generate cleanly, or "
                      "the wiring and ground return. Do not trust any timing "
                      "figure in this report while this is failing", pct);
    }

    if (!o->regdiff) {
        skip_check(r, "imu.init.regdiff", "IMU control-register diff",
                   "disabled with --no-regdiff");
        skip_check(r, "imu.init.idempotent", "IMU init() is idempotent",
                   "needs the register diff");
    } else if (!rm) {
        skip_check(r, "imu.init.regdiff", "IMU control-register diff",
                   "no safe-register map for this driver; add one to "
                   "imt_regmaps[] in src/imutest.c rather than scanning blind");
        skip_check(r, "imu.init.idempotent", "IMU init() is idempotent",
                   "needs a safe-register map");
    } else if (!snapped) {
        skip_check(r, "imu.init.regdiff", "IMU control-register diff",
                   "the post-reset snapshot could not be read");
        skip_check(r, "imu.init.idempotent", "IMU init() is idempotent",
                   "the post-reset snapshot could not be read");
    } else {
        memset(after, 0, sizeof after);
        if (reg_snapshot(ibus, rm, after) < 0) {
            skip_check(r, "imu.init.regdiff", "IMU control-register diff",
                       "the post-init snapshot could not be read");
            skip_check(r, "imu.init.idempotent", "IMU init() is idempotent",
                       "the post-init snapshot could not be read");
        } else {
            /* Find the registers that move on their own before diffing
             * anything, so neither the diff nor the idempotency compare is
             * dominated by sensor output, FIFO level and the timestamp. */
            int nvol = reg_volatile_scan(ibus, rm, after, volatile_imu);
            const bool *vol = nvol >= 0 ? volatile_imu : NULL;
            r->raw.n_volatile_imu = nvol > 0 ? nvol : 0;

            /* How many registers the idempotency compare actually covers. */
            int nscan = 0;
            for (int reg = rm->lo; reg <= (int)rm->hi; reg++)
                if (!regmap_skips(rm, (uint8_t)reg) && !(vol && vol[reg]))
                    nscan++;
            r->raw.n_scanned_imu = nscan;

            int n = reg_diff(before, after, rm, vol, r->raw.regdiff_imu,
                             IMT_MAX_REGDIFF);
            r->raw.n_regdiff_imu     = n;
            r->raw.regdiff_imu_mapped = true;

            add_check(r, "imu.init.regdiff", "IMU control-register diff",
                      n > 0 ? IMT_PASS : IMT_WARN,
                      fmtbuf(mb, sizeof mb, "%d register%s changed",
                             n, n == 1 ? "" : "s"),
                      "> 0",
                      n > 0
                      ? "raw values are in the appendix; decoding them against "
                        "the datasheet is the reviewer's job, not the tool's."
                      : "init() changed nothing in the mapped range — either "
                        "the map is wrong or init() is not configuring the part.");

            /* A second init must land on the same image: catches a bank left
             * selected, a latched enable, or any state-dependent branch. */
            if (imu->init(ibus, icfg) < 0) {
                add_check(r, "imu.init.idempotent", "IMU init() is idempotent",
                          IMT_FAIL, "second init failed", "0",
                          "init() succeeded once but failed when repeated.");
            } else {
                memset(again, 0, sizeof again);
                if (reg_snapshot(ibus, rm, again) < 0) {
                    skip_check(r, "imu.init.idempotent",
                               "IMU init() is idempotent",
                               "the second snapshot could not be read");
                } else if (!vol) {
                    skip_check(r, "imu.init.idempotent",
                               "IMU init() is idempotent",
                               "the volatile-register scan could not be read, "
                               "so a compare would be dominated by live data");
                } else {
                    /* Record WHICH registers moved, not just how many.
                     * Volatile registers are excluded — without that this
                     * counts live sensor output and reads as a state-dependent
                     * init().  reg_diff() applies exactly the same skip and
                     * volatile filtering the reset-to-init diff above uses, so
                     * the two cannot drift apart the way two hand-rolled loops
                     * would. */
                    int ndiff = reg_diff(after, again, rm, vol,
                                         r->raw.idem_imu, IMT_MAX_REGDIFF);
                    r->raw.n_idem_imu = ndiff;

                    /* Name them in the finding itself.  The appendix carries
                     * the before/after bytes, but a reader scanning the check
                     * list needs enough to start on without turning the page. */
                    char rl[160];
                    size_t rn = 0;
                    rl[0] = '\0';
                    for (int i = 0; i < ndiff; i++) {
                        int w = snprintf(rl + rn, sizeof rl - rn, "%s0x%02X",
                                         i ? ", " : "", r->raw.idem_imu[i].reg);
                        if (w < 0 || (size_t)w >= sizeof rl - rn) {
                            snprintf(rl + rn, sizeof rl - rn, ", ...");
                            break;
                        }
                        rn += (size_t)w;
                    }
                    add_check(r, "imu.init.idempotent",
                              "IMU init() is idempotent",
                              ndiff == 0 ? IMT_PASS : IMT_WARN,
                              fmtbuf(mb, sizeof mb, "%d register%s differ",
                                     ndiff, ndiff == 1 ? "" : "s"),
                              "0",
                              ndiff == 0
                              ? "two consecutive inits leave an identical image "
                                "across %d non-volatile register%s"
                              /* The register list goes near the front: note[]
                               * is 192 bytes and truncates, so the one part a
                               * reader cannot reconstruct must not be the part
                               * that gets cut.  The causes this used to
                               * enumerate live in the spec and the appendix. */
                              : "%d non-volatile register%s compared; %s differ "
                                "after a second init — init() depends on the "
                                "state it was called in.",
                              nscan, nscan == 1 ? "" : "s", rl);
                }
            }
        }
    }

    /* ── magnetometer, always after the IMU ──────────────────────────────── */
    *mag_ok = false;
    if (!mag) {
        skip_check(r, "mag.probe", "Mag probe() / chip identification",
                   "no magnetometer configured (--mag-driver none)");
        return 0;
    }

    if (mag->probe(mbus) < 0) {
        add_check(r, "mag.probe", "Mag probe() / chip identification", IMT_FAIL,
                  "rejected", "accepted",
                  "probe() failed at 0x%02X. For a compass behind an IMU's I2C "
                  "bypass (ak09916, ak8963) the address is 0x0C and the IMU "
                  "must have initialised first — it did here.", cfg->mag_addr);
        return 0;
    }
    add_check(r, "mag.probe", "Mag probe() / chip identification", IMT_PASS,
              fmtbuf(mb, sizeof mb, "accepted at 0x%02X", cfg->mag_addr),
              "accepted", "driver '%s' recognised the part", mag->name);

    /* Same reasoning as imu.probe.reject above: i2c_addr is not on the SPI
     * wire, so there is no bogus address to probe. */
    if (mbus->kind == BUS_SPI) {
        skip_check(r, "mag.probe.reject", "Mag probe() rejects a bogus address",
                   "chip select addresses the part on SPI, so i2c_addr never "
                   "reaches the wire and a bogus one reads the same chip; "
                   "probing a neighbouring chip select could disturb it");
    } else {
        imud_bus_t mbogus = *mbus;
        mbogus.i2c_addr = IMT_BOGUS_ADDR;
        if (mag->probe(&mbogus) == 0)
            add_check(r, "mag.probe.reject", "Mag probe() rejects a bogus address",
                      IMT_FAIL, "accepted", "rejected",
                      "probe() returned 0 at unused address 0x%02X.", IMT_BOGUS_ADDR);
        else
            add_check(r, "mag.probe.reject", "Mag probe() rejects a bogus address",
                      IMT_PASS, "rejected", "rejected",
                      "no false positive at unused address 0x%02X", IMT_BOGUS_ADDR);
    }

    t0 = now_s();
    rc = mag->reset(mbus);
    ms = (now_s() - t0) * 1e3;
    r->raw.mag_reset_ms = ms;
    if (rc < 0) {
        add_check(r, "mag.reset.rc", "Mag reset()", IMT_FAIL, "failed", "0",
                  "reset() returned -1 after %.1f ms.", ms);
        return 0;
    }
    add_check(r, "mag.reset.rc", "Mag reset()", IMT_PASS, "0", "0",
              "completed in %.1f ms", ms);

    const imt_regmap_t *mrm = o->regdiff ? regmap_for(mag->name) : NULL;
    static uint8_t mbefore[256], mafter[256];
    static bool    volatile_mag[256];
    bool msnapped = false;
    if (mrm && !mrm->ctrl_writeonly) {
        memset(mbefore, 0, sizeof mbefore);
        msnapped = (reg_snapshot(mbus, mrm, mbefore) == 0);
    }

    if (mag->init(mbus, mcfg) < 0) {
        add_check(r, "mag.init.rc", "Mag init()", IMT_FAIL, "failed", "0",
                  "init() returned -1 for ODR %s Hz.", MHZ_STR(ob, mcfg->odr_mhz));
        return 0;
    }
    add_check(r, "mag.init.rc", "Mag init()", IMT_PASS, "0", "0",
              "%s Hz requested", MHZ_STR(ob, mcfg->odr_mhz));

    if (mrm && mrm->ctrl_writeonly) {
        /*
         * Not a finding about the driver: on this part the control registers
         * are write-only, so a readback diff is empty no matter what init()
         * wrote.  Grading it WARN blamed the driver for a property of the
         * silicon, which is worse than not running the check.
         */
        r->raw.regdiff_mag_writeonly = true;
        skip_check(r, "mag.init.regdiff", "Mag control-register diff",
                   "this part's control registers are write-only, so a "
                   "readback diff is structurally empty — the register writes "
                   "are covered off-hardware by test_drivers instead");
    } else if (mrm && msnapped) {
        memset(mafter, 0, sizeof mafter);
        if (reg_snapshot(mbus, mrm, mafter) == 0) {
            int nvol = reg_volatile_scan(mbus, mrm, mafter, volatile_mag);
            const bool *mvol = nvol >= 0 ? volatile_mag : NULL;
            r->raw.n_volatile_mag = nvol > 0 ? nvol : 0;

            int n = reg_diff(mbefore, mafter, mrm, mvol, r->raw.regdiff_mag,
                             IMT_MAX_REGDIFF);
            r->raw.n_regdiff_mag      = n;
            r->raw.regdiff_mag_mapped = true;
            add_check(r, "mag.init.regdiff", "Mag control-register diff",
                      n > 0 ? IMT_PASS : IMT_WARN,
                      fmtbuf(mb, sizeof mb, "%d register%s changed",
                             n, n == 1 ? "" : "s"),
                      "> 0",
                      n > 0 ? "raw values are in the appendix."
                            : "init() changed nothing in the mapped range.");
        } else {
            skip_check(r, "mag.init.regdiff", "Mag control-register diff",
                       "the post-init snapshot could not be read");
        }
    } else {
        skip_check(r, "mag.init.regdiff", "Mag control-register diff",
                   mrm ? "the post-reset snapshot could not be read"
                       : "no safe-register map for this driver");
    }

    /*
     * AK8963 fuse ROM.  ASAX/Y/Z are factory-burned sensitivity constants,
     * readable only in FUSE_ROM_ACCESS mode.  All-zero or all-ones means
     * there is no fuse ROM to read — a counterfeit or dead magnetometer die.
     *
     * This earns its own check because nothing else here catches it.  A fake
     * part answers WHO_AM_I, accepts CNTL1, and passes probe, reset, init and
     * the register diff, while never asserting DRDY and never producing a
     * measurement; and the driver's adjustment arithmetic turns ASA 0x00 into
     * a plausible-looking 0.5x, so even the scaling looks sane.  One line of
     * report then decides "bad silicon" against "our software", which is
     * otherwise an unanswerable question from a bug report.
     *
     * AK8963-only: the AK09916 has fixed sensitivity and no fuse ROM.
     *
     * Every AKM mode change must pass through power-down, so the part is
     * walked power-down -> fuse-ROM -> power-down and the saved CNTL1 written
     * back, leaving the configured mode exactly as init() set it.
     */
    if (strcmp(mag->name, "ak8963") == 0) {
        uint8_t saved = 0, asa[3] = { 0, 0, 0 };
        bool ok = bus_reg_read(mbus, 0x0A, &saved) == 0;
        if (ok) { ok = bus_reg_write(mbus, 0x0A, 0x00) == 0; usleep(1000); }
        if (ok) { ok = bus_reg_write(mbus, 0x0A, 0x0F) == 0; usleep(1000); }
        if (ok) { ok = bus_burst_read(mbus, 0x10, asa, 3) == 0; }
        if (ok) {
            (void)bus_reg_write(mbus, 0x0A, 0x00); usleep(1000);
            (void)bus_reg_write(mbus, 0x0A, saved); usleep(1000);
        }

        if (!ok) {
            skip_check(r, "mag.fuse_rom", "Magnetometer fuse-ROM identity",
                       "the fuse-ROM read did not complete");
        } else {
            bool none = (asa[0] == 0x00 && asa[1] == 0x00 && asa[2] == 0x00)
                     || (asa[0] == 0xFF && asa[1] == 0xFF && asa[2] == 0xFF);
            add_check(r, "mag.fuse_rom", "Magnetometer fuse-ROM identity",
                      none ? IMT_FAIL : IMT_PASS,
                      fmtbuf(mb, sizeof mb, "ASA %u/%u/%u",
                             asa[0], asa[1], asa[2]),
                      "not 0/0/0 or 255/255/255",
                      none ? "no genuine AK8963 returns that, so this die very "
                             "likely has no fuse ROM to read: counterfeit or "
                             "dead. Every other check here can still pass on "
                             "such a part, including probe and init."
                           : "factory sensitivity constants are present, so "
                             "the die carries a programmed fuse ROM.");
        }
    }

    /*
     * BUS INTEGRITY, magnetometer.  The same measurement as imu.bus.integrity
     * and for the same reason, on the other part.
     *
     * It earns its place beyond symmetry: the two parts share SCK and MOSI and
     * differ only in chip select, so running the check on both separates a
     * fault on the shared clock net from one on a single part's branch -- its
     * own pull-up pad, its own chip select, its own stub.  One part corrupting
     * while the other stays clean is a materially different finding from both
     * corrupting, and without this check the report cannot tell them apart.
     *
     * Note the two are NOT clocked alike: an MMC5983MA declares a 2 MHz
     * maximum against the ISM330DHCX's 10 MHz, so a clean mag beside a dirty
     * IMU narrows the search rather than settling it, while a dirty mag
     * implicates the shared wiring outright.
     *
     * Runs after the mag's own init() for the reason imu.bus.integrity does:
     * a part answers reads differently depending on the state it is in, and
     * the state worth measuring is the one the daemon uses.
     */
    {
        int bad = 0, ioerr = 0;
        const imt_regmap_t *mrm2 = regmap_for(mag->name);
        const uint8_t idreg = (mrm2 && mrm2->whoami_reg) ? mrm2->whoami_reg : 0;
        const uint8_t idval = mrm2 ? mrm2->whoami_val : 0;

        if (!idreg) {
            skip_check(r, "mag.bus.integrity", "Mag bus reads are not corrupted",
                       "this part has no identity register to compare against");
        } else {
            for (int i = 0; i < IMT_BUS_INTEGRITY_READS && !g_abort; i++) {
                uint8_t v;
                if (bus_reg_read(mbus, idreg, &v) < 0) { ioerr++; continue; }
                if (v != idval) bad++;
            }
            double pct = 100.0 * (double)bad / (double)IMT_BUS_INTEGRITY_READS;
            const char *val = fmtbuf(mb, sizeof mb, "%d of %d bad, %d io-err",
                                     bad, IMT_BUS_INTEGRITY_READS, ioerr);
            if (bad == 0)
                add_check(r, "mag.bus.integrity",
                          "Mag bus reads are not corrupted", IMT_PASS,
                          val, "0 bad",
                          "%d reads of register 0x%02X all returned 0x%02X, the "
                          "value it is hard-wired to hold",
                          IMT_BUS_INTEGRITY_READS, idreg, idval);
            else
                add_check(r, "mag.bus.integrity",
                          "Mag bus reads are not corrupted",
                          imt_bus_integrity_status(bad, IMT_BUS_INTEGRITY_READS),
                          val, "0 bad",
                          "the magnetometer's identity register read back wrong "
                          "%.2f%% of the time. Compare against imu.bus.integrity "
                          "before assuming a cause: both parts corrupting points "
                          "at the shared clock and data lines, one part alone "
                          "points at that part's own branch -- its pull-ups, its "
                          "chip select, its wiring", pct);
        }
    }

    *mag_ok = true;
    return 0;
}

/* ── Phase A: measured ODR, seq, chip_ts ──────────────────────────────────── */

static void check_odr_seq_ts(imt_report_t *r, const imt_opts_t *o,
                             drain_ctx_t *d, const imu_ops_t *imu,
                             double eff_odr)
{
    char mb[56], eb[56];
    imu_sample_t buf[128];
    int n = 0;

    /* Discard setup residue so the window measures steady state. */
    double t_end = now_s() + 0.3;
    while (now_s() < t_end && !g_abort) { drain_once(d, buf, 128, &n); drain_pace(d); }
    drain_flush(d);

    /* Reset the contract counters so the ODR window is measured cleanly. */
    d->gaps = d->backwards = d->max_gap = 0;
    d->rc1 = d->rcneg = 0;
    d->total = 0;
    d->have_seq = false;

    imt_ts_acc_t tsa = { 0 };
    double   ts_deltas[512];
    int      n_deltas = 0;
    uint32_t seq_first = 0;
    bool     have_first = false;

    double t0        = now_s();
    double first_t   = 0, last_t = 0;
    double prev_loop = t0, max_gap_ms = 0;
    double deadline  = t0 + o->odr_window_s;

    while (now_s() < deadline && !g_abort) {
        double t = now_s();
        double gap_ms = (t - prev_loop) * 1e3;
        if (gap_ms > max_gap_ms) max_gap_ms = gap_ms;
        prev_loop = t;

        int rc = drain_once(d, buf, 128, &n);
        if (rc < 0) { drain_pace(d); continue; }

        if (n > 0) {
            if (!have_first) { first_t = t; seq_first = buf[0].seq; have_first = true; }
            last_t = t;

            /* Each drain_once() is one burst, so its first sample is the
             * seam where the driver re-derives its anchor.  The seam delta is
             * excluded from the tick estimate -- it measures this loop's drain
             * cadence, not the part.  See imt_ts_collect_burst(). */
            uint32_t tsbuf[128];
            int nts = n > 128 ? 128 : n;
            for (int i = 0; i < nts; i++) tsbuf[i] = buf[i].chip_ts;
            n_deltas = imt_ts_collect_burst(&tsa, tsbuf, nts,
                                            ts_deltas, 512, n_deltas);
        }
        ui_progress(o, "imu.odr", (now_s() - t0) / o->odr_window_s, NULL);
        drain_pace(d);
    }

    double span = (last_t > first_t) ? (last_t - first_t) : 0.0;
    double hz   = span > 0 ? (double)(d->total - 1) / span : 0.0;

    r->raw.odr_measured_hz     = hz;
    r->raw.odr_window_s        = span;
    r->raw.odr_n               = d->total;
    r->raw.odr_max_loop_gap_ms = max_gap_ms;
    r->raw.seq_first           = seq_first;
    r->raw.seq_last            = d->last_seq;
    r->raw.seq_gaps            = d->gaps;
    r->raw.seq_backwards       = d->backwards;
    r->raw.seq_max_gap         = d->max_gap;
    r->raw.rc1_count           = d->rc1;
    r->raw.rcneg_count         = d->rcneg;
    r->raw.last_errno          = d->last_errno;

    /*
     * Which supported_odr_mhz entry does the measurement actually match?
     * `hz` is Hz and the table is milli-Hz, so the comparison scales the table
     * rather than the measurement — mixing the two silently picked the lowest
     * rung every time, and no compiler warns about it because both are numbers.
     */
    int best = 0;
    double bestdiff = 1e30;
    int ntab = tab_len(imu->supported_odr_mhz, 16);
    for (int i = 0; i < ntab; i++) {
        double diff = fabs(hz - (double)imu->supported_odr_mhz[i] * 1e-3);
        if (diff < bestdiff) { bestdiff = diff; best = imu->supported_odr_mhz[i]; }
    }
    r->raw.odr_best_table_mhz = best;

    if (d->total < 10) {
        add_check(r, "imu.odr", "Measured ODR against the rate the driver reports", IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%llu samples",
                         (unsigned long long)d->total),
                  fmtbuf(eb, sizeof eb, "~%.6g Hz", eff_odr),
                  "almost no data in %.1f s. The chip is configured but not "
                  "producing samples — check the FIFO enable in init().",
                  o->odr_window_s);
    } else {
        double err = fabs(hz - eff_odr) / eff_odr;
        bool starved = max_gap_ms > 0.20 * o->odr_window_s * 1e3;
        bool over    = hz > eff_odr * (1.0 + o->odr_tol_warn);
        imt_status_t st = (err <= o->odr_tol_warn) ? IMT_PASS
                        : (err <= o->odr_tol_fail) ? IMT_WARN : IMT_FAIL;
        /*
         * Direction decides which excuses apply, and they only run one way.
         * A stalled reader and a missing FIFO both LOSE samples, so they can
         * explain a measurement below the configured rate — scheduling, not
         * the driver. Neither can explain one above it: nothing in the read
         * path invents samples the part did not produce, so an over-rate
         * reading means init()'s rate write did not land or is encoded wrong.
         * An over-rate reading therefore keeps whatever the tolerance ladder
         * gave it: WARN in the warn band, FAIL past odr_tol_fail. It is NOT
         * promoted straight to FAIL — the part's own oscillator tolerance can
         * exceed odr_tol_warn on a perfectly good die. The reference
         * MMC5983MA measures 105.5 Hz against a configured 100 on both
         * transports, and a check that fires on expected silicon behaviour is
         * one people learn to skip, which costs more than the row is worth.
         */
        if (!over && st == IMT_FAIL && (starved || !imu->has_fifo))
            st = IMT_WARN;

        if (st == IMT_PASS)
            /*
             * A pass inside the band can still be a real oscillator error, and
             * imu.chipts.wall grades the same ratio against a tighter bound
             * because the daemon's dt depends on it.  Point at it whenever the
             * measurement is far enough off to matter there, so the two rows
             * do not read as contradicting each other.
             */
            add_check(r, "imu.odr", "Measured ODR against the rate the driver reports",
                      IMT_PASS, fmtbuf(mb, sizeof mb, "%.1f Hz", hz),
                      fmtbuf(eb, sizeof eb, "%.6g Hz +/-%.0f%%",
                             eff_odr, o->odr_tol_warn * 100),
                      "%llu samples in %.3f s%s",
                      (unsigned long long)d->total, span,
                      err > 0.02 && imu->has_hw_timestamp
                      ? ", but off by more than 2% — see imu.chipts.wall, which "
                        "grades the same timebase against what ts_tick_ns claims"
                      : "");
        else if (best && (double)best * 1e-3 != eff_odr &&
                 fabs(hz - (double)best * 1e-3) < fabs(hz - eff_odr))
            add_check(r, "imu.odr", "Measured ODR against the rate the driver reports", st,
                      fmtbuf(mb, sizeof mb, "%.1f Hz", hz),
                      fmtbuf(eb, sizeof eb, "%.6g Hz +/-%.0f%%",
                             eff_odr, o->odr_tol_warn * 100),
                      "off by %.1f%%, and it matches supported_odr_mhz entry "
                      "%.6g Hz instead — check the rate encoding in init().",
                      err * 100, (double)best * 1e-3);
        else
            add_check(r, "imu.odr", "Measured ODR against the rate the driver reports", st,
                      fmtbuf(mb, sizeof mb, "%.1f Hz", hz),
                      fmtbuf(eb, sizeof eb, "%.6g Hz +/-%.0f%%",
                             eff_odr, o->odr_tol_warn * 100),
                      "off by %.1f%%%s", err * 100,
                      over ? " — ABOVE the configured rate; the read loop cannot "
                             "cause that, so init()'s rate write did not land "
                             "or is encoded wrong"
                      : starved ? "; the read loop stalled for up to "
                                "20%+ of the window, so this may be scheduling "
                                "rather than the driver" : "");
    }

    /* seq contract */
    if (d->backwards > 0)
        add_check(r, "imu.seq.monotonic", "seq is monotonic", IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%llu non-advancing",
                         (unsigned long long)d->backwards),
                  "0",
                  "seq repeated or went backwards. Fusion uses seq deltas to "
                  "detect dropped samples, so this corrupts the timeline.");
    else
        add_check(r, "imu.seq.monotonic", "seq is monotonic", IMT_PASS, "0", "0",
                  "%llu samples, no repeats or reversals",
                  (unsigned long long)d->total);

    if (d->gaps == 0)
        add_check(r, "imu.seq.gapless", "seq has no unexplained gaps", IMT_PASS,
                  "0 gaps", "0",
                  "every consecutive delta was exactly 1");
    else {
        /* Gaps are legitimate only where the driver reported an overflow. */
        bool explained = (d->rc1 > 0);
        double frac = (double)d->gaps / (double)(d->total ? d->total : 1);
        add_check(r, "imu.seq.gapless", "seq has no unexplained gaps",
                  (explained && frac < 0.001) ? IMT_WARN : IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%llu gaps, max %llu",
                         (unsigned long long)d->gaps,
                         (unsigned long long)d->max_gap),
                  "0",
                  explained
                  ? "%d read(s) reported a FIFO overflow in this window, which "
                    "legitimately drops samples."
                  : "no overflow was reported, so samples are being lost "
                    "silently — the FIFO drain is dropping data.",
                  d->rc1);
    }

    /* chip_ts contract */
    if (!imu->has_hw_timestamp) {
        if (tsa.zeros == (int)d->total)
            add_check(r, "imu.chipts.presence", "chip_ts matches has_hw_timestamp",
                      IMT_PASS, "always 0", "0",
                      "has_hw_timestamp is false and every chip_ts was 0, as "
                      "the contract requires.");
        else
            add_check(r, "imu.chipts.presence", "chip_ts matches has_hw_timestamp",
                      IMT_FAIL,
                      fmtbuf(mb, sizeof mb, "%d of %llu nonzero",
                             (int)d->total - tsa.zeros, (unsigned long long)d->total),
                      "always 0",
                      "has_hw_timestamp is false, so chip_ts must be 0 on every "
                      "sample — imu.c re-anchors wall-clock time when it is.");
        skip_check(r, "imu.chipts.monotonic", "chip_ts is monotonic",
                   "part has no sample timer");
        skip_check(r, "imu.chipts.rate", "chip_ts tick period",
                   "part has no sample timer");
        skip_check(r, "imu.chipts.wall", "chip_ts against wall clock",
                   "part has no sample timer");
        return;
    }

    if (!tsa.have) {
        add_check(r, "imu.chipts.presence", "chip_ts matches has_hw_timestamp",
                  IMT_FAIL, "always 0", "nonzero",
                  "has_hw_timestamp is true but chip_ts stayed 0 — the "
                  "timestamp read is failing silently inside read().");
        skip_check(r, "imu.chipts.monotonic", "chip_ts is monotonic",
                   "prerequisite imu.chipts.presence failed");
        skip_check(r, "imu.chipts.rate", "chip_ts tick period",
                   "prerequisite imu.chipts.presence failed");
        skip_check(r, "imu.chipts.wall", "chip_ts against wall clock",
                   "prerequisite imu.chipts.presence failed");
        return;
    }

    add_check(r, "imu.chipts.presence", "chip_ts matches has_hw_timestamp",
              IMT_PASS, "nonzero", "nonzero",
              "tick in use %u ns", d->tick_ns);

    r->raw.ts_first     = tsa.first;
    r->raw.ts_last      = tsa.last;
    r->raw.ts_backwards = tsa.reversals;
    r->raw.ts_seam_backwards = tsa.seam_reversals;
    r->raw.n_ts_rev     = tsa.n_rev;
    for (int i = 0; i < tsa.n_rev; i++) r->raw.ts_rev[i] = tsa.rev[i];
    r->raw.ts_repeats   = tsa.repeats;
    r->raw.ts_zero_count = tsa.zeros;
    r->raw.ts_wraps     = tsa.wraps;

    /*
     * Where a reversal happened decides what it means, so they are graded
     * apart.  Inside a burst the driver stamps every sample from one anchor
     * and time can only go forwards, so a reversal there is a decode or unwrap
     * defect and FAILs.  At a seam the anchor is re-derived from a post-drain
     * timestamp read, and this loop is paced by a 5 ms sleep rather than by
     * the FIFO watermark -- a poll cannot know how old the sample it just read
     * is, so the new burst can land before the old one ended.
     *
     * Measured on the reference part: the daemon, woken by the watermark
     * interrupt, scored 0 reversals in 53,708 samples while this check scored
     * 2 to 7 per window on the same part minutes apart.  Grading those as a
     * driver fault reported a healthy timestamp chain as broken on every run.
     * They still WARN, because an interrupt-less install really does drain on
     * a timer and really does see them.
     */
    int inner_rev = tsa.reversals - tsa.seam_reversals;
    imt_status_t mono_st = (tsa.reversals == 0 && tsa.repeats == 0) ? IMT_PASS
                         : (inner_rev > 0 || tsa.repeats != 0)      ? IMT_FAIL
                                                                   : IMT_WARN;
    add_check(r, "imu.chipts.monotonic", "chip_ts is monotonic", mono_st,
              fmtbuf(mb, sizeof mb, "%d reversals (%d at a burst seam) / %d repeats",
                     tsa.reversals, tsa.seam_reversals, tsa.repeats), "0 / 0",
              inner_rev > 0
              ? "chip_ts went backwards WITHIN a burst, where every sample is "
                "stamped from one anchor and time can only move forwards. For "
                "a counter narrower than 32 bits that is usually a missing "
                "unwrap in the driver (%d wrap%s, %d zero-stamped sample%s "
                "skipped)"
              : tsa.reversals != 0 && tsa.repeats == 0
              ? "chip_ts went backwards only at burst SEAMS, which this check "
                "drains on a timer rather than on the watermark interrupt — a "
                "poll cannot know how old the sample it just read is, so the "
                "post-drain anchor can place a burst before the previous one "
                "ended. The daemon, woken by the interrupt, does not do this. "
                "An install with imu.int_gpio = 0 does (%d wrap%s, %d "
                "zero-stamped sample%s skipped)"
              : tsa.repeats != 0
              ? "chip_ts repeated: consecutive samples carry the identical "
                "tick, so a burst is being stamped from one reading rather "
                "than per sample (%d wrap%s, %d zero-stamped sample%s skipped)"
              : "unsigned deltas advanced on every sample (%d counter wrap%s "
                "seen and handled, %d zero-stamped sample%s skipped)",
              tsa.wraps, tsa.wraps == 1 ? "" : "s",
              tsa.zeros,  tsa.zeros  == 1 ? "" : "s");

    if (n_deltas < 4) {
        /*
         * Only within-burst deltas count, so this is reachable at a rate slow
         * enough that a drain seldom returns two samples: there is then no
         * per-sample tick period to measure, only the drain cadence, and
         * saying so is better than grading the cadence as if it were the part.
         */
        skip_check(r, "imu.chipts.rate", "chip_ts tick period",
                   "fewer than 4 within-burst timestamp deltas at this rate — "
                   "a drain returned two samples too rarely to measure a "
                   "per-sample tick period");
    } else {
        /* Median over within-burst deltas only; see the collection loop. */
        for (int i = 1; i < n_deltas; i++) {
            double v = ts_deltas[i];
            int j = i - 1;
            while (j >= 0 && ts_deltas[j] > v) { ts_deltas[j + 1] = ts_deltas[j]; j--; }
            ts_deltas[j + 1] = v;
        }
        double med = ts_deltas[n_deltas / 2];

        /*
         * Both sides of this ratio must come from the CHIP, or it stops being
         * a tick check.  Use the rate just measured, never the configured one:
         * a part whose oscillator runs fast produces proportionally fewer
         * ticks per sample against nominal, and dividing by the nominal rate
         * reports that as a tick error when imu.odr above already owns it.
         *
         * Measured on the reference ISM330DHCX 2026-08-19, which runs 4.1%
         * fast at every rate on its ladder: at a configured 833 Hz it delivers
         * 867.1 Hz and 48.00 ticks/sample.  Against nominal that "expects"
         * 49.96 and reads as a 4% tick error; against the measured rate it
         * expects 48.00 and matches exactly.  The nominal form put this check
         * into WARN at 104, 208 and 416 Hz on a part that was fine.
         *
         * Falls back to nominal only if the rate measurement did not produce
         * one, which cannot normally happen — check_odr_seq_ts fills it from
         * the same window these deltas came from.
         */
        double rate = r->raw.odr_measured_hz > 0 ? r->raw.odr_measured_hz
                                                 : eff_odr;
        double expect = 1e9 / (rate * (double)d->tick_ns);
        double implied = med > 0 ? 1e9 / (rate * med) : 0.0;
        r->raw.ts_median_delta    = med;
        r->raw.ts_implied_tick_ns = implied;

        double err = fabs(med - expect) / expect;
        imt_status_t st = err <= 0.05 ? IMT_PASS : err <= 0.20 ? IMT_WARN : IMT_FAIL;
        add_check(r, "imu.chipts.rate", "chip_ts tick period", st,
                  fmtbuf(mb, sizeof mb, "%.2f ticks/sample", med),
                  fmtbuf(eb, sizeof eb, "%.2f +/-5%%", expect),
                  "implied ts_tick_ns = %.0f against the %u in use, at the "
                  "%.1f Hz measured — chip against chip, so see "
                  "imu.chipts.wall for the timebase itself",
                  implied, d->tick_ns, rate);

        double elapsed_chip = (double)(uint32_t)(tsa.last - tsa.first)
                            * (double)d->tick_ns * 1e-9;
        double ratio = span > 0 ? elapsed_chip / span : 0.0;
        r->raw.ts_wall_ratio = ratio;
        imt_status_t wst = imt_chipts_wall_status(ratio);

        /*
         * Report the part's own declared timebase error where it has one.  The
         * driver now APPLIES this (ts_tick_ns_actual), so it is no longer a
         * cross-check against a constant the daemon was about to get wrong —
         * it says which trim produced the tick the ratio above was graded
         * with, and a ratio still off 1.0 with a plausible FREQ_FINE means
         * something other than the oscillator is moving the timebase.
         */
        char fine[80];
        fine[0] = '\0';
        const imt_regmap_t *fm = regmap_for(imu->name);
        uint8_t ff = 0;
        if (fm && fm->freq_fine_reg &&
            bus_reg_read(d->bus, fm->freq_fine_reg, &ff) == 0)
            /* DS13012 Table 139: 8-bit two's complement, 0.15% per step. */
            snprintf(fine, sizeof fine,
                     " Part declares FREQ_FINE %+d.", (int)(int8_t)ff);
        /*
         * Direction matters for the diagnosis and the two causes are opposite.
         * Short: chip time is missing, so the driver dropped a counter wrap.
         * Long: chip time is running ahead of the wall, which a wrap cannot
         * cause — the counter is ticking faster than the tick in use, and
         * imu.c scales every per-sample dt by that number.
         */
        double implied_tick = ratio > 0 ? (double)d->tick_ns / ratio : 0.0;
        add_check(r, "imu.chipts.wall", "chip_ts against wall clock", wst,
                  fmtbuf(mb, sizeof mb, "%.4f", ratio),
                  "1.0000 (+/-2% exact; long is tolerated, short is not)",
                  /* Both must fit imt_check_t.note (192 bytes) once expanded;
                   * test_imutest asserts no note is truncated. */
                  ratio < 1.0
                  ? "%.3f s chip vs %.3f s wall over %d wrap%s; implied tick "
                    "%.0f ns, not %u. Chip time missing — a dropped wrap.%s"
                  : "%.3f s chip vs %.3f s wall over %d wrap%s; implied tick "
                    "%.0f ns, not %u. Fast oscillator; imu.c measures the "
                    "real period.%s",
                  elapsed_chip, span, tsa.wraps, tsa.wraps == 1 ? "" : "s",
                  implied_tick, d->tick_ns, fine);
    }
}

/* ── Phase A: error-return contract ───────────────────────────────────────── */

static void check_error_contract(imt_report_t *r, drain_ctx_t *d)
{
    char mb[56];
    imu_sample_t buf[128];
    int n = 0;

    drain_flush(d);

    /* An empty FIFO is not an error: 0 with n == 0.  This is the single most
     * commonly violated line of the contract. */
    int rc = drain_once(d, buf, 128, &n);
    if (rc < 0)
        add_check(r, "imu.err.nodata_not_error",
                  "Empty FIFO returns 0, not -1", IMT_FAIL, "-1", "0",
                  "read() returned -1 on an empty FIFO (errno %d). -1 is "
                  "reserved for bus errors; imu.c counts it toward the "
                  "reset threshold and will restart the chip.", errno);
    else
        add_check(r, "imu.err.nodata_not_error",
                  "Empty FIFO returns 0, not -1", IMT_PASS,
                  fmtbuf(mb, sizeof mb, "%d, n=%d", rc, n), "0",
                  "no data is reported as success with zero samples");

    /* Sustained reads on a healthy bus must never produce -1. */
    int neg = 0, saved_errno = 0;
    for (int i = 0; i < 200 && !g_abort; i++) {
        if (drain_once(d, buf, 128, &n) < 0) { neg++; saved_errno = errno; }
    }
    if (neg == 0)
        add_check(r, "imu.err.no_spurious", "No spurious -1 on a healthy bus",
                  IMT_PASS, "0 of 200", "0",
                  "200 back-to-back reads, no bus errors reported");
    else
        add_check(r, "imu.err.no_spurious", "No spurious -1 on a healthy bus",
                  IMT_FAIL, fmtbuf(mb, sizeof mb, "%d of 200", neg), "0",
                  "read() reported bus errors on an otherwise working bus "
                  "(last errno %d: %s).", saved_errno, strerror(saved_errno));
}

/* ── Phase A: FIFO behaviour ──────────────────────────────────────────────── */

static void check_fifo(imt_report_t *r, const imt_opts_t *o, drain_ctx_t *d,
                       const imu_ops_t *imu, double eff_odr, int fifo_wm)
{
    char mb[56], eb[56];
    imu_sample_t buf[128];
    int n = 0;

    if (!imu->has_fifo) {
        skip_check(r, "imu.fifo.depth", "FIFO accumulates between reads",
                   "driver declares has_fifo = false");
        skip_check(r, "imu.fifo.watermark", "FIFO watermark timing",
                   "driver declares has_fifo = false");
        skip_check(r, "imu.fifo.overflow", "FIFO overflow is reported as rc 1",
                   "driver declares has_fifo = false");
        skip_check(r, "imu.fifo.recovers", "FIFO recovers after overflow",
                   "driver declares has_fifo = false");
        return;
    }

    double period = 1.0 / eff_odr;
    double wait   = period * (fifo_wm > 0 ? fifo_wm : 32);
    if (wait < 0.02) wait = 0.02;
    if (wait > 0.5)  wait = 0.5;

    /*
     * Depth should grow roughly linearly with the wait.  The probe buffer is
     * deliberately much larger than the one used everywhere else: read() stops
     * at `max`, so a 128-sample buffer made every step past the first report
     * exactly 128 and the table measured the caller's array instead of the
     * FIFO.  512 clears the deepest FIFO in the tree.
     */
    static imu_sample_t deep[512];
    int depth[3] = { 0, 0, 0 };
    for (int step = 0; step < 3 && !g_abort; step++) {
        drain_flush(d);
        double w = wait * (step + 1);
        sleep_s(w);
        if (drain_once(d, deep, (int)(sizeof deep / sizeof deep[0]), &n) < 0) {
            depth[step] = -1;
            continue;
        }
        depth[step] = n;
        if (r->raw.fifo_steps < IMT_MAX_FIFO_STEPS) {
            r->raw.fifo_wait_s[r->raw.fifo_steps] = w;
            r->raw.fifo_depth[r->raw.fifo_steps]  = n;
            r->raw.fifo_steps++;
        }
        ui_progress(o, "imu.fifo", (step + 1) / 3.0, NULL);
    }

    int expect0 = (int)(wait * eff_odr);
    if (depth[0] <= 0)
        add_check(r, "imu.fifo.depth", "FIFO accumulates between reads",
                  IMT_FAIL, "0 samples",
                  fmtbuf(eb, sizeof eb, "~%d", expect0),
                  "nothing accumulated in %.0f ms — the FIFO is not filling, "
                  "or read() returns only the newest sample.", wait * 1e3);
    else if (depth[1] > depth[0])
        add_check(r, "imu.fifo.depth", "FIFO accumulates between reads",
                  IMT_PASS,
                  fmtbuf(mb, sizeof mb, "%d / %d / %d", depth[0], depth[1], depth[2]),
                  fmtbuf(eb, sizeof eb, "rising, ~%d first", expect0),
                  "depth grows with the wait, so read() drains a real queue "
                  "rather than a single sample register");
    else
        add_check(r, "imu.fifo.depth", "FIFO accumulates between reads",
                  IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%d / %d / %d", depth[0], depth[1], depth[2]),
                  fmtbuf(eb, sizeof eb, "rising, ~%d first", expect0),
                  "depth did not grow with a longer wait — the FIFO may "
                  "already be at capacity, or the caller's buffer is the limit.");

    /* Watermark timing is only visible on the interrupt line. */
    skip_check(r, "imu.fifo.watermark", "FIFO watermark timing",
               "only observable through int_gpio; see imu.drdy.edges, which "
               "reports which interrupt model the measured edge rate fits");

    if (!o->induce_overflow) {
        skip_check(r, "imu.fifo.overflow", "FIFO overflow is reported as rc 1",
                   "disabled with --no-overflow");
        skip_check(r, "imu.fifo.recovers", "FIFO recovers after overflow",
                   "disabled with --no-overflow");
        return;
    }

    /*
     * Fill the FIFO by not draining it.  Depth is chip-specific and not
     * knowable generically, so search with a doubling sleep and give up
     * politely.  The overflow flag is latched and cleared by the read that
     * reports it, so it must be taken from that read's return value.
     */
    drain_flush(d);
    double waited = 0, w = wait * 2;
    int rc = 0, prev_n = -1;
    bool growing = true;
    for (int i = 0; i < 12 && !g_abort; i++) {
        sleep_s(w);
        waited += w;
        /*
         * The deep buffer, not the 128-sample one: read() stops at `max`, so a
         * short buffer reports the caller's array size and saturation could
         * never be told from a full FIFO.
         */
        rc = drain_once(d, deep, (int)(sizeof deep / sizeof deep[0]), &n);
        if (rc != 0) break;
        /*
         * Each iteration drains, so what fills the FIFO is the LAST sleep, not
         * the accumulated time.  While the count still rises with a longer
         * sleep the FIFO has not reached capacity, and a missing overflow says
         * nothing about the driver -- it says the window was too short.  That
         * distinction is the whole verdict: at 12 Hz the part needs about 20 s
         * to fill and this check will not wait that long, which was being
         * reported as a driver fault.
         */
        growing = (prev_n < 0) || (n > prev_n);
        prev_n  = n;
        if (!growing) break;              /* at capacity, and still rc 0 */
        if (waited > 8.0) break;
        w *= 1.6;
        ui_progress(o, "imu.fifo.overflow", waited / 8.0, NULL);
    }
    r->raw.overflow_after_s = waited;

    imt_status_t ost = imt_overflow_status(rc, growing);
    if (ost == IMT_PASS)
        add_check(r, "imu.fifo.overflow", "FIFO overflow is reported as rc 1",
                  IMT_PASS, "1",
                  "1 after the FIFO fills",
                  "overflow signalled after %.2f s without draining", waited);
    else if (ost == IMT_FAIL)
        add_check(r, "imu.fifo.overflow", "FIFO overflow is reported as rc 1",
                  IMT_FAIL, "-1", "1",
                  "read() returned a bus error while the FIFO was full "
                  "instead of reporting the overflow.");
    else if (ost == IMT_WARN)
        add_check(r, "imu.fifo.overflow", "FIFO overflow is reported as rc 1",
                  IMT_WARN, "0", "1",
                  "the FIFO stopped accepting samples at %d per drain, so it "
                  "is at capacity, and read() still returned 0 — the driver is "
                  "not surfacing the overflow bit.", prev_n);
    else
        skip_check(r, "imu.fifo.overflow", "FIFO overflow is reported as rc 1",
                   "the FIFO was still filling when the check ran out of "
                   "patience, so nothing was learned about the overflow bit; "
                   "at this rate it needs a longer wait than this check "
                   "allows");

    /* Whatever happened, reads must return to normal afterwards. */
    drain_flush(d);
    sleep_s(wait);
    rc = drain_once(d, buf, 128, &n);
    add_check(r, "imu.fifo.recovers", "FIFO recovers after overflow",
              rc == 0 ? IMT_PASS : rc > 0 ? IMT_WARN : IMT_FAIL,
              fmtbuf(mb, sizeof mb, "rc %d, %d samples", rc, n), "0",
              rc == 0 ? "normal reads resumed after the overflow"
                      : "reads did not return to rc 0 after draining.");
}

/* ── Phase A: noise floor, gravity, temperature ───────────────────────────── */

static void check_rest(imt_report_t *r, const imt_opts_t *o, drain_ctx_t *d)
{
    char mb[56], eb[56];
    imu_sample_t buf[128];
    int n = 0;

    welford3_t wa, wg;
    w3_init(&wa); w3_init(&wg);
    double gsum = 0, g2sum = 0;
    uint64_t gn = 0;
    double tmin = 1e30, tmax = -1e30, tsum = 0;
    uint64_t tn = 0;
    /* Cheap distinct-value estimate: count changes in the quantised reading. */
    int tdistinct = 0;
    float tprev = 0;
    bool  have_tprev = false;

    drain_flush(d);
    double t0 = now_s(), deadline = t0 + o->noise_window_s;
    while (now_s() < deadline && !g_abort) {
        if (drain_once(d, buf, 128, &n) < 0) { drain_pace(d); continue; }
        for (int i = 0; i < n; i++) {
            w3_add(&wa, buf[i].accel);
            w3_add(&wg, buf[i].gyro);
            double mag = sqrt((double)buf[i].accel[0] * buf[i].accel[0] +
                              (double)buf[i].accel[1] * buf[i].accel[1] +
                              (double)buf[i].accel[2] * buf[i].accel[2]);
            gsum += mag; g2sum += mag * mag; gn++;

            float t = buf[i].temp_c;
            if (t < tmin) tmin = t;
            if (t > tmax) tmax = t;
            tsum += t; tn++;
            if (!have_tprev || fabsf(t - tprev) > 1e-6f) { tdistinct++; tprev = t; have_tprev = true; }
        }
        ui_progress(o, "imu.noise", (now_s() - t0) / o->noise_window_s, NULL);
        drain_pace(d);
    }

    w3_finish(&wa, &r->raw.accel);
    w3_finish(&wg, &r->raw.gyro);
    r->raw.grav_mean  = gn ? gsum / (double)gn : 0.0;
    r->raw.grav_sigma = gn > 1 ? sqrt(fabs(g2sum / (double)gn -
                                     (gsum / (double)gn) * (gsum / (double)gn))) : 0.0;
    r->raw.temp_min = tn ? tmin : 0.0;
    r->raw.temp_max = tn ? tmax : 0.0;
    r->raw.temp_mean = tn ? tsum / (double)tn : 0.0;
    r->raw.temp_distinct = tdistinct;

    if (gn < 10) {
        skip_check(r, "imu.rest.still", "Platform was still", "no samples collected");
        skip_check(r, "imu.noise.gyro", "Gyro noise floor", "no samples collected");
        skip_check(r, "imu.noise.accel", "Accel noise floor", "no samples collected");
        skip_check(r, "imu.rest.gravity", "Gravity magnitude at rest", "no samples collected");
        skip_check(r, "imu.temp.plausible", "Temperature is plausible", "no samples collected");
        skip_check(r, "imu.temp.varies", "Temperature is not stuck", "no samples collected");
        return;
    }

    /* Was the board actually still?  Everything below is graded on that. */
    double gmax = fmax(fmax(r->raw.gyro.sigma[0], r->raw.gyro.sigma[1]),
                       r->raw.gyro.sigma[2]);
    bool moved = gmax > 0.1;
    add_check(r, "imu.rest.still", "Platform was still", moved ? IMT_WARN : IMT_PASS,
              fmtbuf(mb, sizeof mb, "gyro sigma %.4f rad/s", gmax),
              "< 0.1 rad/s",
              moved ? "the board moved during the rest window; the noise and "
                      "gravity checks below are graded leniently as a result."
                    : "the rest window was quiet enough to grade noise and "
                      "gravity strictly.");

    /* A stuck axis reads a perfect constant.  That is a decode bug, never
     * physics, so it fails regardless of how still the platform was. */
    bool gstuck = false, astuck = false;
    for (int k = 0; k < 3; k++) {
        if (r->raw.gyro.sigma[k]  == 0.0 || !isfinite(r->raw.gyro.sigma[k]))  gstuck = true;
        if (r->raw.accel.sigma[k] == 0.0 || !isfinite(r->raw.accel.sigma[k])) astuck = true;
    }

    if (gstuck)
        add_check(r, "imu.noise.gyro", "Gyro noise floor", IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%.2g / %.2g / %.2g",
                         r->raw.gyro.sigma[0], r->raw.gyro.sigma[1],
                         r->raw.gyro.sigma[2]),
                  "5e-4 .. 5e-2 rad/s",
                  "an axis has exactly zero variance — that axis is stuck, "
                  "which means it is not being decoded from the sample.");
    else {
        bool ok = true;
        for (int k = 0; k < 3; k++)
            if (r->raw.gyro.sigma[k] < 5e-4 || r->raw.gyro.sigma[k] > 5e-2) ok = false;
        add_check(r, "imu.noise.gyro", "Gyro noise floor",
                  ok ? IMT_PASS : IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%.2g / %.2g / %.2g",
                         r->raw.gyro.sigma[0], r->raw.gyro.sigma[1],
                         r->raw.gyro.sigma[2]),
                  "5e-4 .. 5e-2 rad/s",
                  ok ? "per-axis standard deviation over %.0f s at rest"
                     : "outside the usual band over %.0f s — plausible if the "
                       "bench is not still, otherwise check the gyro scaling.",
                  o->noise_window_s);
    }

    if (astuck)
        add_check(r, "imu.noise.accel", "Accel noise floor", IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%.2g / %.2g / %.2g",
                         r->raw.accel.sigma[0], r->raw.accel.sigma[1],
                         r->raw.accel.sigma[2]),
                  "2e-3 .. 0.5 m/s^2",
                  "an axis has exactly zero variance — that axis is stuck.");
    else {
        bool ok = true;
        for (int k = 0; k < 3; k++)
            if (r->raw.accel.sigma[k] < 2e-3 || r->raw.accel.sigma[k] > 0.5) ok = false;
        add_check(r, "imu.noise.accel", "Accel noise floor",
                  ok ? IMT_PASS : IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%.2g / %.2g / %.2g",
                         r->raw.accel.sigma[0], r->raw.accel.sigma[1],
                         r->raw.accel.sigma[2]),
                  "2e-3 .. 0.5 m/s^2",
                  ok ? "per-axis standard deviation over %.0f s at rest"
                     : "outside the usual band over %.0f s.", o->noise_window_s);
    }

    /* Gravity magnitude is orientation-independent, so it is the one accel
     * check that works without asking the operator to do anything. */
    double err = fabs(r->raw.grav_mean - IMT_G_MS2);
    imt_status_t st = err <= o->grav_tol_warn ? IMT_PASS
                    : err <= o->grav_tol_fail ? IMT_WARN : IMT_FAIL;
    if (moved && st == IMT_FAIL) st = IMT_WARN;
    double ratio = r->raw.grav_mean > 0.01 ? IMT_G_MS2 / r->raw.grav_mean : 0.0;
    bool pow2 = ratio > 0.1 && fabs(ratio - round(ratio)) < 0.05 &&
                (fabs(ratio - 2.0) < 0.05 || fabs(ratio - 4.0) < 0.05 ||
                 fabs(ratio - 8.0) < 0.05 || fabs(ratio - 0.5) < 0.05);
    /* Both branches take (window, ratio) so the argument list matches whichever
     * format string the ternary picks — see the note on add_check(). */
    add_check(r, "imu.rest.gravity", "Gravity magnitude at rest", st,
              fmtbuf(mb, sizeof mb, "%.4f m/s^2", r->raw.grav_mean),
              fmtbuf(eb, sizeof eb, "9.8066 +/-%.2f", o->grav_tol_warn),
              pow2 ? "|a| averaged over %.0f s; ratio to true g is %.3f — a "
                     "power of two, so this full-scale branch's sensitivity "
                     "constant is wrong."
                   : "|a| averaged over %.0f s; ratio to true g is %.3f.",
              o->noise_window_s, ratio);

    /* Temperature */
    bool pinned = (tdistinct <= 1 && fabs(r->raw.temp_mean - 25.0) < 1e-3);
    if (pinned)
        add_check(r, "imu.temp.plausible", "Temperature is plausible", IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%.3f C, 1 value", r->raw.temp_mean),
                  "-20 .. 85 C, varying",
                  "pinned at exactly 25.000 C with no variation — that is the "
                  "placeholder, so the temperature word is never decoded.");
    else if (r->raw.temp_mean < -20.0 || r->raw.temp_mean > 85.0)
        add_check(r, "imu.temp.plausible", "Temperature is plausible", IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%.2f C", r->raw.temp_mean),
                  "-20 .. 85 C",
                  "outside any plausible die temperature — check the "
                  "temperature scaling and offset.");
    else
        add_check(r, "imu.temp.plausible", "Temperature is plausible", IMT_PASS,
                  fmtbuf(mb, sizeof mb, "%.2f C", r->raw.temp_mean),
                  "-20 .. 85 C",
                  "range %.2f .. %.2f C over %.0f s",
                  r->raw.temp_min, r->raw.temp_max, o->noise_window_s);

    add_check(r, "imu.temp.varies", "Temperature is not stuck",
              tdistinct > 1 ? IMT_PASS : IMT_WARN,
              fmtbuf(mb, sizeof mb, "%d distinct, range %.2f C",
                     tdistinct, r->raw.temp_max - r->raw.temp_min),
              "> 1 distinct value",
              tdistinct > 1
              ? "the reading moves, so it is being read live"
              : "no variation in %.0f s. Expected for a coarse 1 C/LSB word "
                "(the ICM-42688-P decodes an int8); confirm against the "
                "datasheet before treating this as a defect.",
              o->noise_window_s);
}

/* ── Phase A: full-scale sweep ────────────────────────────────────────────── */

/*
 * Re-init at every entry in supported_accel_g / supported_gyro_dps.  This is
 * the check that catches one wrong sensitivity constant in one branch of an
 * encode function — a very common driver bug that no single-configuration test
 * can see.  Runs last, because each init() resets the driver's seq counter.
 */
/* Collect for `secs`, returning per-axis accel/gyro stats and mean |a|. */
static uint64_t collect_stats(const imt_opts_t *o, drain_ctx_t *d, double secs,
                              const char *prog_id,
                              imt_stats3_t *accel, imt_stats3_t *gyro,
                              double *grav_mean)
{
    imu_sample_t buf[128];
    int n = 0;
    welford3_t wa, wg;
    w3_init(&wa); w3_init(&wg);
    double gsum = 0;
    uint64_t gn = 0;

    double t0 = now_s(), deadline = t0 + secs;
    while (now_s() < deadline && !g_abort) {
        if (drain_once(d, buf, 128, &n) < 0) { sleep_s(0.002); continue; }
        for (int i = 0; i < n; i++) {
            w3_add(&wa, buf[i].accel);
            w3_add(&wg, buf[i].gyro);
            gsum += sqrt((double)buf[i].accel[0] * buf[i].accel[0] +
                         (double)buf[i].accel[1] * buf[i].accel[1] +
                         (double)buf[i].accel[2] * buf[i].accel[2]);
            gn++;
        }
        if (prog_id) ui_progress(o, prog_id, (now_s() - t0) / secs, NULL);
        sleep_s(0.002);
    }

    if (accel) w3_finish(&wa, accel);
    if (gyro)  w3_finish(&wg, gyro);
    if (grav_mean) *grav_mean = gn ? gsum / (double)gn : 0.0;
    return gn;
}

static void check_fs_sweep(imt_report_t *r, const imt_opts_t *o, drain_ctx_t *d,
                           const imu_ops_t *imu, const imud_bus_t *bus,
                           const imu_cfg_t *base)
{
    char mb[56], eb[56], id[32], nm[64];

    if (!o->fs_sweep) {
        skip_check(r, "imu.fs.accel", "Full-scale sweep, accelerometer",
                   "disabled with --no-fs-sweep");
        skip_check(r, "imu.fs.gyro", "Full-scale sweep, gyroscope",
                   "disabled with --no-fs-sweep");
        skip_check(r, "imu.fs.restore", "Configured full scale restored",
                   "disabled with --no-fs-sweep");
        return;
    }

    /* ── Accelerometer: gravity is the reference at every full scale ─────── */
    /*
     * Settle and collect in SAMPLES, not in seconds.
     *
     * A fixed 300 ms settle is 250 samples at 833 Hz and under four at 12 Hz,
     * and a fixed 1 s window is 833 samples against twelve.  The low-rate end
     * of the ladder was therefore reading the part mid-transient after each
     * re-init: at 12 and 26 Hz gravity came back 7% out at one range while
     * every other range passed, and the Z noise floor read 1.6-2.5 m/s^2
     * against its usual 0.05.  Which range got caught moved between runs,
     * which is the signature of a transient rather than a scale error.
     *
     * Bounded at both ends: the fast rates keep the timings they had, and the
     * slow ones cannot stretch a four-range sweep past a bench's patience.
     */
    double fs_per    = base->odr_mhz > 0
                     ? 1000.0 / (double)base->odr_mhz : 0.001;
    double fs_settle = 20.0 * fs_per;
    double fs_window = 50.0 * fs_per;
    if (fs_settle < 0.3) fs_settle = 0.3;
    if (fs_settle > 2.0) fs_settle = 2.0;
    if (fs_window < 1.0) fs_window = 1.0;
    if (fs_window > 4.0) fs_window = 4.0;

    int na = tab_len(imu->supported_accel_g, 8);
    int failed = 0, worst = 0;
    for (int i = 0; i < na && !g_abort; i++) {
        imu_cfg_t c = *base;
        c.accel_g = imu->supported_accel_g[i];

        if (imu->reset(bus) < 0 || imu->init(bus, &c) < 0) {
            snprintf(id, sizeof id, "imu.fs.a%d", c.accel_g);
            snprintf(nm, sizeof nm, "Accel full scale +/-%d g", c.accel_g);
            add_check(r, id, nm, IMT_FAIL, "init failed", "0",
                      "could not configure +/-%d g, which supported_accel_g[] "
                      "advertises.", c.accel_g);
            failed++;
            continue;
        }
        drain_flush(d);
        sleep_s(fs_settle);
        drain_flush(d);

        imt_stats3_t acc;
        double grav = 0;
        uint64_t gn = collect_stats(o, d, fs_window, "imu.fs", &acc, NULL, &grav);

        if (r->raw.n_fs_accel >= IMT_MAX_FS_ROWS) continue;
        imt_fs_row_t *row = &r->raw.fs_accel[r->raw.n_fs_accel++];
        row->fs        = c.accel_g;
        row->grav_mean = grav;
        row->ratio     = grav > 0.01 ? IMT_G_MS2 / grav : 0.0;
        row->n         = (int)gn;
        for (int k = 0; k < 3; k++) row->sigma[k] = acc.sigma[k];

        double err = fabs(grav - IMT_G_MS2);
        row->status = gn < 10 ? IMT_SKIP
                    : err <= o->grav_tol_warn ? IMT_PASS
                    : err <= o->grav_tol_fail ? IMT_WARN : IMT_FAIL;

        if (row->status == IMT_FAIL) { failed++; worst = c.accel_g; }

        snprintf(id, sizeof id, "imu.fs.a%d", c.accel_g);
        snprintf(nm, sizeof nm, "Accel full scale +/-%d g", c.accel_g);
        add_check(r, id, nm, row->status,
                  fmtbuf(mb, sizeof mb, "%.4f m/s^2", grav),
                  fmtbuf(eb, sizeof eb, "9.8066 +/-%.2f", o->grav_tol_warn),
                  "ratio to true g %.3f%s", row->ratio,
                  (row->status == IMT_FAIL && fabs(row->ratio - 2.0) < 0.05)
                      ? " — exactly 2x, so this branch's sensitivity constant "
                        "is off by a factor of two" : "");
    }
    add_check(r, "imu.fs.accel", "Full-scale sweep, accelerometer",
              failed ? IMT_FAIL : IMT_PASS,
              fmtbuf(mb, sizeof mb, "%d of %d ranges bad", failed, na),
              "0 bad",
              failed ? "gravity does not read 9.807 at every advertised full "
                       "scale (worst: +/-%d g) — one sensitivity constant is "
                       "wrong."
                     : "gravity reads correctly at all %d advertised ranges",
              failed ? worst : na);

    /*
     * Gyro has no static reference: a stationary gyro reads about zero at
     * every full scale.  The usable bench proxy is that the ADC range, and so
     * the noise standard deviation, scales with the full scale — a branch with
     * a wrong constant breaks the ratio between adjacent entries.  Absolute
     * gyro scale is only verified by the guided rotation phase, and only at
     * the configured full scale; the report says so.
     */
    int ng = tab_len(imu->supported_gyro_dps, 8);
    int gbad = 0;
    double prev_sigma = 0;
    int prev_fs = 0;
    for (int i = 0; i < ng && !g_abort; i++) {
        imu_cfg_t c = *base;
        c.gyro_dps = imu->supported_gyro_dps[i];

        if (imu->reset(bus) < 0 || imu->init(bus, &c) < 0) {
            gbad++;
            continue;
        }
        drain_flush(d);
        sleep_s(fs_settle);
        drain_flush(d);

        imt_stats3_t gy;
        collect_stats(o, d, fs_window, "imu.fs", NULL, &gy, NULL);
        double s = (gy.sigma[0] + gy.sigma[1] + gy.sigma[2]) / 3.0;

        if (r->raw.n_fs_gyro >= IMT_MAX_FS_ROWS) continue;
        imt_fs_row_t *row = &r->raw.fs_gyro[r->raw.n_fs_gyro++];
        row->fs = c.gyro_dps;
        for (int k = 0; k < 3; k++) row->sigma[k] = gy.sigma[k];
        row->status = IMT_INFO;

        /*
         * One-sided, and only for a large effect.  Sigma rising more slowly
         * than the full scale is not evidence of anything: the proxy assumes
         * quantisation dominates the noise floor, and on a good part it does
         * not.  On an ISM330DHCX the +/-125 dps LSB is 4.375 mdps = 7.6e-5
         * rad/s against a measured floor near 1.9e-3 — analogue noise is ~25x
         * the quantisation step, so sigma stays flat across every range and a
         * two-sided band around fsratio grades coin flips.  Sigma *falling*
         * as the full scale rises has no benign reading, so that is what is
         * graded; the numbers go in the appendix either way.
         */
        /* Ratio to the previous step is recorded for the appendix.  It is
         * NOT what grades the step -- see the median test below. */
        if (prev_fs > 0 && prev_sigma > 0 && s > 0)
            row->ratio = s / prev_sigma;
        prev_sigma = s;
        prev_fs    = c.gyro_dps;
    }

    /*
     * Grade each step against the MEDIAN of all of them, not against its
     * neighbour.
     *
     * The pairwise form graded a transient.  Sigma here is dominated by
     * ambient movement rather than by the part -- the comment above works out
     * that analogue noise is ~25x the quantisation step, so the expectation is
     * a FLAT sigma across every range -- and one step inflated by a knock on
     * the bench made the NEXT step read as a halving, with the innocent step
     * carrying the WARN.  On the reference part it fired at 52, 104 and
     * 208 Hz in one sweep and at 12 and 52 in another, moving between runs,
     * which is the signature of grading noise rather than silicon.
     *
     * The median is robust to one bad step, and it matches the defect being
     * hunted more directly: a range whose sensitivity constant does not track
     * its register reads LOW against every other range at once, not merely
     * against the one before it.
     */
    /*
     * Grade only where the proxy holds.  See imt_fs_scales_with_range(): on a
     * part whose analogue noise dwarfs its quantisation step, sigma is flat
     * across every range and grading it grades the bench, not the silicon.
     */
    bool fs_graded = imt_fs_scales_with_range(r->raw.fs_gyro, r->raw.n_fs_gyro);
    if (fs_graded)
        gbad += imt_fs_grade_median(r->raw.fs_gyro, r->raw.n_fs_gyro, 0.5);

    add_check(r, "imu.fs.gyro", "Full-scale sweep, gyroscope",
              ng < 2 ? IMT_SKIP : gbad ? IMT_WARN : IMT_INFO,
              fmtbuf(mb, sizeof mb, "%d of %d steps dropped", gbad,
                     ng ? ng - 1 : 0),
              "sigma must not fall as full scale rises",
              gbad ? "a range's noise floor is below half the median of all "
                     "ranges — one branch's sensitivity constant does not "
                     "track its register. Rotation phase grades absolute "
                     "scale, at +/-%d dps."
                   : fs_graded
                     ? "each range compared with the median of all; sigma "
                       "tracks full scale here, so the proxy holds. Absolute "
                       "scale: rotation phase, +/-%d dps."
                     : "recorded, NOT graded: sigma is flat across the ranges, "
                       "so analogue noise dominates quantisation and the proxy "
                       "carries no information. Absolute scale: rotation "
                       "phase, +/-%d dps.", base->gyro_dps);

    /* Everything after this depends on the configured setup being back. */
    bool ok = (imu->reset(bus) == 0) && (imu->init(bus, base) == 0);
    add_check(r, "imu.fs.restore", "Configured full scale restored",
              ok ? IMT_PASS : IMT_FAIL,
              ok ? "restored" : "failed",
              fmtbuf(eb, sizeof eb, "%d g / %d dps", base->accel_g, base->gyro_dps),
              ok ? "back to the configured full scale after the sweep"
                 : "could not restore the configured full scale — later "
                   "checks in this run are not trustworthy.");
    drain_flush(d);
}

/* ── Phase A: DRDY / watermark interrupt line ─────────────────────────────── */

static void drain_cb(void *user)
{
    drain_ctx_t *d = user;
    imu_sample_t buf[128];
    int n = 0;
    drain_once(d, buf, 128, &n);
}

static void check_drdy(imt_report_t *r, const imt_opts_t *o, drain_ctx_t *d,
                       const imud_config_t *cfg, double eff_odr, int fifo_wm)
{
    char mb[56], eb[56];

    if (cfg->imu_int_gpio <= 0) {
        skip_check(r, "imu.drdy.edges", "IMU interrupt line produces edges",
                   "imu.int_gpio is 0 — the reader uses a polling timer");
        r->raw.gpio_why = IMT_GPIO_DISABLED;
        return;
    }

    /*
     * Can an edge even happen in this window?
     *
     * The watermark asserts once `fifo_wm` sample-sets have accumulated, so it
     * first fires at `fifo_wm / odr` seconds. At 12 Hz with the shipped
     * `fifo_wm = 64` that is 5.3 s against a 3 s window: no edge is possible,
     * and the check was reporting "the line is not wired" about a line that is
     * wired and working. Grading an impossibility is worse than not grading.
     */
    double first_edge_s = eff_odr > 0 ? (double)fifo_wm / eff_odr : 0.0;
    if (first_edge_s >= o->drdy_window_s) {
        skip_check(r, "imu.drdy.edges", "IMU interrupt line produces edges",
                   "the watermark cannot fill inside this window at this ODR — "
                   "raise --drdy-window, or lower fifo_wm");
        r->raw.gpio_why = IMT_GPIO_OK;
        return;
    }

    imt_gpio_why_t why = IMT_GPIO_OK;

    /*
     * The operator's window, NOT widened by imt_rate_window_s().
     *
     * That helper sizes a window to resolve a ±tolerance, and this check does
     * not grade one: it asks which interrupt model the edge rate fits, per
     * sample or per watermark, and reports which. A model fit needs enough
     * edges to tell two rates that differ by orders of magnitude apart, not
     * enough to resolve 5%. The feasibility gate above already refuses a
     * window too short for the watermark to fill in at all, which is the only
     * way this one can be too short.
     */
    double win = o->drdy_window_s;
    long ms = (long)(win * 1e3);

    /* Pass 1: drain on every edge, which is what the daemon does. */
    drain_flush(d);
    int edges = imt_gpio_count_edges(cfg->gpio_chip, cfg->imu_int_gpio, ms,
                                     drain_cb, d, &why, NULL, eff_odr);
    r->raw.gpio_why      = why;
    r->raw.gpio_edges    = edges;
    r->raw.gpio_window_s = win;
    r->raw.gpio_rate_hz  = edges > 0 ? edges / win : 0.0;

    if (edges < 0) {
        const char *reason =
            why == IMT_GPIO_EBUSY  ? "GPIO is held by another process — is imud running?"
          : why == IMT_GPIO_ENOCHIP ? "GPIO chip not found; check device.gpio_chip"
          : why == IMT_GPIO_UNSUPPORTED ? "built without libgpiod"
          : "GPIO request failed";
        skip_check(r, "imu.drdy.edges", "IMU interrupt line produces edges", reason);
        return;
    }

    /*
     * Pass 2: the same window with the FIFO left alone.
     *
     * This is the experiment ROADMAP section 1.1 asked for.  The 2026-08-10
     * bench measured ~18.3 Hz on the reference part at 833 Hz with fifo_wm 64,
     * which fits neither the per-sample model (833) nor the watermark model
     * (13), and an edge count alone cannot say why.  The surviving hypothesis
     * was that INT1_FIFO_TH is a LEVEL condition oscillating across the
     * threshold during the drain — words consumed while new ones arrive, each
     * crossing producing another rising edge — in which case the number is an
     * artifact of draining and not a rate at all.
     *
     * Stop draining and the two candidates separate cleanly.  A level
     * condition latches: it asserts once, stays asserted because nothing
     * empties the FIFO, and yields about one rising edge for the whole window.
     * A genuine per-sample data-ready keeps pulsing at the ODR regardless.
     *
     * Deliberately left until after check_fifo, which already provokes an
     * overflow: this pass fills the FIFO on purpose, and the flush afterwards
     * is what keeps that out of the checks that follow.
     */
    drain_flush(d);
    imt_gpio_why_t why_idle = IMT_GPIO_OK;
    int idle = imt_gpio_count_edges(cfg->gpio_chip, cfg->imu_int_gpio, ms,
                                    NULL, NULL, &why_idle, NULL, eff_odr);
    drain_flush(d);

    r->raw.gpio_idle_valid   = (idle >= 0);
    r->raw.gpio_edges_idle   = idle;
    r->raw.gpio_rate_idle_hz = idle > 0 ? idle / o->drdy_window_s : 0.0;

    double per_sample = eff_odr;
    double watermark  = fifo_wm > 0 ? eff_odr / fifo_wm : 0.0;
    double rate = r->raw.gpio_rate_hz;

    if (edges == 0) {
        add_check(r, "imu.drdy.edges", "IMU interrupt line produces edges",
                  IMT_FAIL, "0 edges", "> 0",
                  "no edges on BCM %d in %.1f s. The line is not wired, the "
                  "interrupt is not enabled in init(), or the BCM number is "
                  "wrong.", cfg->imu_int_gpio, win);
        return;
    }

    /*
     * ONE format string, ONE argument list.  The verdict varies, the prose
     * does not: imt_check_t.note is 192 bytes and test_imutest fails a
     * truncated one, and a format that forks mid-call would have to consume
     * identical arguments on both sides to be safe at all.  Building the two
     * variable clauses separately sidesteps the whole problem.
     *
     * Budget: base ~47 + lead ~47 + idle ~88 = ~182 against the 192.  The
     * idle buffer is sized for an int of any width even though the branch
     * that prints one bounds it at 2, because the compiler checks the format
     * and not the branch, and a -Wformat-truncation warning is not worth
     * arguing with over eight bytes.
     */
    char idle_note[96];
    if (!r->raw.gpio_idle_valid)
        snprintf(idle_note, sizeof idle_note, "Second pass could not run.");
    else if (idle <= 2)
        snprintf(idle_note, sizeof idle_note,
                 "%d undrained: level condition, so the drained count is "
                 "drain-paced, not a rate.", idle);
    else if (per_sample > 0 && fabs(idle / win - per_sample)
                                   / per_sample < 0.20)
        snprintf(idle_note, sizeof idle_note,
                 "%.0f Hz undrained too: an edge per sample.",
                 idle / win);
    else
        snprintf(idle_note, sizeof idle_note,
                 "%.0f Hz undrained: fits no model.", idle / o->drdy_window_s);

    /*
     * More edges than the part has samples cannot be an interrupt it raised,
     * so that is still worth flagging.  Everything below it passes.  The tool
     * deliberately no longer grades a part down for fitting neither model:
     * the reference part does exactly that and is healthy, which is what made
     * the old WARN noise rather than a finding, and an edge count on its own
     * was never able to identify a model in the first place.
     */
    bool too_fast = per_sample > 0 && rate > per_sample * 1.2;
    char lead[56];
    lead[0] = '\0';
    if (too_fast)
        snprintf(lead, sizeof lead, "Above the sample rate: floating or "
                                    "shared line. ");

    add_check(r, "imu.drdy.edges", "IMU interrupt line produces edges",
              too_fast ? IMT_WARN : IMT_PASS,
              fmtbuf(mb, sizeof mb, "%.1f Hz drained, %.1f Hz idle",
                     rate, r->raw.gpio_rate_idle_hz),
              fmtbuf(eb, sizeof eb, "%.0f or %.1f Hz", per_sample, watermark),
              "BCM %d: %.1f Hz vs %.0f/%.1f (sample/wm). %s%s",
              cfg->imu_int_gpio, rate, per_sample, watermark, lead, idle_note);
}

/* ── Phase A: magnetometer ────────────────────────────────────────────────── */

static double norm3d(const double v[3])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/*
 * Average `secs` worth of magnetometer samples into out[3].
 *
 * The first two completed samples are discarded, because a degauss pulse only
 * affects the NEXT measurement to start: at 100 Hz a measurement takes several
 * ms and the driver's settling sleep is 1 ms, so the first sample out of the
 * part after a pulse was very likely already in flight when the pulse landed.
 * Averaging it in would drag both halves of a differential toward each other
 * and understate the very thing the differential exists to measure.
 *
 * Returns the number of samples averaged; 0 means the caller must not use out.
 */
static uint64_t mag_collect_mean(const mag_ops_t *mag, const imud_bus_t *bus,
                                 double secs, double poll, double out[3])
{
    double sum[3] = { 0, 0, 0 };
    uint64_t n = 0, seen = 0;
    double deadline = now_s() + secs;

    for (int k = 0; k < 3; k++) out[k] = 0.0;

    while (now_s() < deadline && !g_abort) {
        mag_sample_t s;
        memset(&s, 0, sizeof s);
        if (mag->read(bus, &s) == 0 && s.valid) {
            if (++seen > 2) {
                for (int k = 0; k < 3; k++) sum[k] += s.field[k];
                n++;
            }
        }
        sleep_s(poll);
    }
    if (n) for (int k = 0; k < 3; k++) out[k] = sum[k] / (double)n;
    return n;
}

/*
 * Does a multi-byte read land where a sequence of single reads does?
 *
 * This is the on-hardware half of the spi_inc_mask claim. bus_burst_read()
 * passes the mask only when len > 1, so an N-byte burst and N one-byte reads
 * take genuinely different paths through the command byte: the burst asserts
 * the part's auto-increment bit, the singles do not. If the declared mask is
 * wrong the burst walks somewhere else — or does not walk at all and returns
 * one register N times — and the two disagree. On the LIS3MDL, the one part
 * here with a real increment bit, that is the whole question.
 *
 * What it does NOT cover, and the report should not be read as saying it does:
 * bus_reg_read() is bus_burst_read(len=1), so both sides share
 * spi_burst_read(). This cannot see a fault in that helper's two-transfer
 * framing — a bench comparison against a single full-duplex transfer settled
 * that separately, byte-identical over five single-shot images.
 *
 * The window is live, so a bare comparison would fail whenever a measurement
 * lands mid-read. Bracketing the singles between two bursts and requiring
 * those to match means the sample did not move while they were taken; only
 * then does a difference mean anything. If no quiet window turns up, that is a
 * SKIP rather than a guess.
 */
static void check_burst_framing(imt_report_t *r, const char *id, const char *name,
                                const imud_bus_t *bus, const imt_regmap_t *m,
                                uint8_t *hex_out, int *hex_n)
{
    if (!m || m->out_lo > m->out_hi) {
        skip_check(r, id, name,
                   "no measurement-output window declared for this driver in "
                   "imt_regmaps[], so there is nothing to read two ways");
        return;
    }

    int n = m->out_hi - m->out_lo + 1;
    uint8_t burst[32], again[32], single[32];
    if (n > (int)sizeof burst) n = (int)sizeof burst;

    for (int attempt = 0; attempt < 8; attempt++) {
        if (bus_burst_read(bus, m->out_lo, burst, (uint16_t)n) < 0 ||
            bus_burst_read(bus, m->out_lo, again, (uint16_t)n) < 0) {
            add_check(r, id, name, IMT_FAIL, "read failed", "matching bytes",
                      "the output window could not be read.");
            return;
        }
        if (memcmp(burst, again, (size_t)n) != 0) continue;  /* sample moved */

        for (int i = 0; i < n; i++) {
            if (bus_reg_read(bus, (uint8_t)(m->out_lo + i), &single[i]) < 0) {
                add_check(r, id, name, IMT_FAIL, "read failed", "matching bytes",
                          "a single-register read of the output window failed.");
                return;
            }
        }
        /* Confirm nothing moved underneath the singles either. */
        if (bus_burst_read(bus, m->out_lo, again, (uint16_t)n) < 0) continue;
        if (memcmp(burst, again, (size_t)n) != 0) continue;

        if (hex_out && hex_n) {
            memcpy(hex_out, burst, (size_t)n);
            memcpy(hex_out + n, single, (size_t)n);
            *hex_n = n;
        }

        int bad = 0, first = -1;
        for (int i = 0; i < n; i++)
            if (burst[i] != single[i]) { bad++; if (first < 0) first = i; }

        char mb[64], eb[64];
        if (bad == 0)
            add_check(r, id, name, IMT_PASS,
                      fmtbuf(mb, sizeof mb, "%d bytes agree", n),
                      "identical",
                      "0x%02X-0x%02X reads the same whether bursted or taken one "
                      "register at a time", m->out_lo, m->out_hi);
        else
            add_check(r, id, name, IMT_FAIL,
                      fmtbuf(mb, sizeof mb, "%d of %d bytes differ", bad, n),
                      fmtbuf(eb, sizeof eb, "%d identical", n),
                      "first at 0x%02X: burst 0x%02X, single 0x%02X. The burst "
                      "is not landing where single reads do — check "
                      "spi_inc_mask against the datasheet.",
                      (unsigned)(m->out_lo + first), burst[first], single[first]);
        return;
    }

    skip_check(r, id, name,
               "the output registers changed under every attempt, so burst and "
               "single reads could not be compared on the same sample");
}

/*
 * mag.degauss.differential — is a high reading a field, or an offset?
 *
 * SET and RESET magnetise the AMR film opposite ways, so the field term of a
 * reading flips sign between them and the bridge's own offset does not.  One
 * measurement each way therefore separates the two (imt_degauss_split), and
 * that is the only way to make the distinction on a bench with one transport
 * and no reference field.
 *
 * Both numbers are reported whatever the verdict.  The offset is not a
 * secondary detail here: a part reading far outside Earth's range with a SMALL
 * offset is looking at real iron, and the same part with a LARGE offset is
 * failing to remove its own bias — the same symptom, opposite causes, and only
 * this check tells them apart.
 */
static void check_mag_degauss(imt_report_t *r, const imt_opts_t *o,
                              const mag_ops_t *mag, const imud_bus_t *bus,
                              double eff_odr)
{
    char mb[96], eb[56];

    if (!mag->degauss) {
        skip_check(r, "mag.degauss.differential",
                   "True field vs bridge offset",
                   mag->has_set_reset
                   ? "this driver pulses SET but cannot drive RESET, so the "
                     "two directions cannot be compared"
                   : "this part has no degauss coil");
        return;
    }

    double period = eff_odr > 0 ? 1.0 / eff_odr : 0.01;
    double poll   = period / 4;
    if (poll > 0.002) poll = 0.002;
    double half = o->mag_window_s / 2.0;
    if (half < 1.0) half = 1.0;

    double vS[3] = { 0, 0, 0 }, vR[3] = { 0, 0, 0 };
    uint64_t nS = 0, nR = 0;
    int rcS = mag->degauss(bus, MAG_DEGAUSS_SET);
    if (rcS == 0) {
        ui_progress(o, "mag.degauss.differential", 0.25, "SET");
        nS = mag_collect_mean(mag, bus, half, poll, vS);
    }
    int rcR = rcS == 0 ? mag->degauss(bus, MAG_DEGAUSS_RESET) : 0;
    if (rcS == 0 && rcR == 0) {
        ui_progress(o, "mag.degauss.differential", 0.75, "RESET");
        nR = mag_collect_mean(mag, bus, half, poll, vR);
    }

    /*
     * Leave the part SET however this went.  RESET inverts the field term, so
     * a run that ended there would hand every later check — and the daemon, if
     * one is started straight afterwards — a sign-flipped magnetometer.
     */
    (void)mag->degauss(bus, MAG_DEGAUSS_SET);

    if (rcS != 0 || rcR != 0) {
        add_check(r, "mag.degauss.differential", "True field vs bridge offset",
                  IMT_FAIL, "pulse failed", "0",
                  "degauss() returned %d for SET and %d for RESET; the coil "
                  "could not be driven, so nothing was measured.", rcS, rcR);
        return;
    }
    if (nS < 3 || nR < 3) {
        skip_check(r, "mag.degauss.differential", "True field vs bridge offset",
                   "too few samples between the pulses to average");
        return;
    }

    double field[3], offset[3];
    imt_degauss_split(vS, vR, field, offset);
    double fn = norm3d(field), on = norm3d(offset);

    for (int k = 0; k < 3; k++) {
        r->raw.mag_dg_set[k]    = vS[k];
        r->raw.mag_dg_reset[k]  = vR[k];
        r->raw.mag_dg_field[k]  = field[k];
        r->raw.mag_dg_offset[k] = offset[k];
    }
    r->raw.mag_dg_n           = nS < nR ? nS : nR;
    r->raw.mag_dg_field_norm  = fn;
    r->raw.mag_dg_offset_norm = on;

    imt_status_t st = (fn >= 25.0 && fn <= 65.0) ? IMT_PASS
                    : (fn >= 15.0 && fn <= 100.0) ? IMT_WARN : IMT_FAIL;

    /*
     * The interpretation is the payload, so spell it out rather than leaving a
     * reader to work out which of the two numbers matters.
     */
    const char *reading =
        st == IMT_PASS
        ? (on > 100.0
           ? "the part measures Earth's field correctly; the reading is high "
             "because a large bridge offset is riding on it and is not being "
             "removed"
           : "field and offset are both where they should be")
        : (on > fn
           ? "the offset dominates the field, so a high |B| is the bridge, "
             "not the environment"
           : "the field itself is out of range, so the part really is seeing "
             "this much flux — look for iron on or near the board");

    add_check(r, "mag.degauss.differential", "True field vs bridge offset", st,
              fmtbuf(mb, sizeof mb, "field %.1f uT, offset %.1f uT", fn, on),
              fmtbuf(eb, sizeof eb, "field 25 .. 65 uT"),
              "SET [%.1f %.1f %.1f] vs RESET [%.1f %.1f %.1f] over %llu/%llu "
              "samples: %s.",
              vS[0], vS[1], vS[2], vR[0], vR[1], vR[2],
              (unsigned long long)nS, (unsigned long long)nR, reading);
}

static void check_mag_passive(imt_report_t *r, const imt_opts_t *o,
                              const mag_ops_t *mag, const imud_bus_t *bus,
                              double eff_odr)
{
    char mb[56], eb[56];
    welford3_t w;
    w3_init(&w);

    /*
     * Degauss BEFORE measuring, not after.
     *
     * This check used to run at the end of the phase, which meant every
     * number above it — field magnitude, noise, the whole of §5.7 — graded the
     * bridge in whatever magnetisation state the part happened to be left in
     * by whatever last touched it.  That is not a property of the driver, and
     * on 2026-08-15 it produced a 1124.7 uT field reading that nothing in the
     * report could attribute.  Pulsing first makes the state a known one.
     */
    if (mag->has_set_reset) {
        if (!mag->set_reset) {
            add_check(r, "mag.set_reset", "SET/RESET degauss pulse", IMT_FAIL,
                      "NULL", "non-NULL",
                      "has_set_reset is true but set_reset is NULL — the "
                      "daemon would call through a null pointer.");
        } else {
            int rc = mag->set_reset(bus);
            add_check(r, "mag.set_reset", "SET/RESET degauss pulse",
                      rc == 0 ? IMT_PASS : IMT_FAIL,
                      fmtbuf(mb, sizeof mb, "%d", rc), "0",
                      rc == 0 ? "the degauss pulse was issued, before the "
                                "measurements below rather than after them"
                              : "set_reset() failed.");
        }
    } else {
        add_check(r, "mag.set_reset", "SET/RESET degauss pulse",
                  mag->set_reset == NULL ? IMT_PASS : IMT_FAIL,
                  mag->set_reset ? "non-NULL" : "NULL", "NULL",
                  mag->set_reset == NULL
                  ? "no coil declared and none exposed, which is consistent"
                  : "has_set_reset is false but set_reset is not NULL.");
    }

    double period = eff_odr > 0 ? 1.0 / eff_odr : 0.01;
    double poll   = period / 4;
    if (poll > 0.002) poll = 0.002;

    uint64_t got = 0;
    int rc1 = 0, rcneg = 0, zero_wall = 0, backwards_wall = 0;
    uint64_t prev_wall = 0;
    double nsum = 0, nmin = 1e30, nmax = -1e30;

    double t0 = now_s(), deadline = t0 + o->mag_window_s;
    while (now_s() < deadline && !g_abort) {
        mag_sample_t s;
        memset(&s, 0, sizeof s);
        int rc = mag->read(bus, &s);
        if (rc < 0)      rcneg++;
        else if (rc > 0) rc1++;
        else {
            w3_add(&w, s.field);
            double norm = sqrt((double)s.field[0] * s.field[0] +
                               (double)s.field[1] * s.field[1] +
                               (double)s.field[2] * s.field[2]);
            nsum += norm;
            if (norm < nmin) nmin = norm;
            if (norm > nmax) nmax = norm;
            if (s.wall_ns == 0)              zero_wall++;
            else if (s.wall_ns < prev_wall)  backwards_wall++;
            if (s.wall_ns) prev_wall = s.wall_ns;
            got++;
        }
        ui_progress(o, "mag.rate", (now_s() - t0) / o->mag_window_s, NULL);
        sleep_s(poll);
    }
    double span = now_s() - t0;

    w3_finish(&w, &r->raw.magf);
    r->raw.mag_n        = got;
    r->raw.mag_rc1      = rc1;
    r->raw.mag_rcneg    = rcneg;
    r->raw.mag_window_s = span;
    r->raw.mag_rate_hz  = span > 0 ? got / span : 0.0;
    r->raw.mag_norm_mean = got ? nsum / (double)got : 0.0;
    r->raw.mag_norm_min  = got ? nmin : 0.0;
    r->raw.mag_norm_max  = got ? nmax : 0.0;

    /* Rate */
    if (got < 5) {
        add_check(r, "mag.rate", "Measured mag rate", IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%llu samples", (unsigned long long)got),
                  fmtbuf(eb, sizeof eb, "~%.6g Hz", eff_odr),
                  "almost no samples in %.1f s (%d not-ready, %d bus errors).",
                  span, rc1, rcneg);
    } else {
        double err = fabs(r->raw.mag_rate_hz - eff_odr) / (double)eff_odr;
        /*
         * Low and high are not the same finding, and grading them alike is how
         * the MMC5983MA's 130 Hz-against-a-configured-100 Hz reached a report
         * as a WARN nobody read.  The poll loop can only ever undercount, so a
         * LOW reading may be pacing and stays a WARN.  Nothing in the sampling
         * path can invent samples the part did not produce, so a HIGH reading
         * says the part is not running at the rate init() asked for.  Past
         * odr_tol_fail that is a driver defect and FAILs; inside the warn band
         * it stays a WARN, because a part's continuous-mode oscillator is
         * specified as typical and the reference die runs 5.4% fast.
         */
        /*
         * Two different questions, so two different booleans. `high` is the
         * DIRECTION and decides the wording; `out` is the tolerance and
         * decides the grade. One boolean served both, so any reading above
         * nominal but inside the band was described as "low" -- the bench saw
         * 21.0 Hz against a configured 20 reported as "5.0% low", which sends
         * a reader looking for a pacing problem when the part is running fast.
         */
        /*
         * A count over a fixed window resolves to one sample, +/-1/N, and at a
         * low ODR that step is wider than the tolerance: at 1 Hz over 5 s the
         * only readings possible are 1.0 and 1.2 Hz, and 1.2 reads as "20%
         * ABOVE" -- which FAILed a part running at exactly its rate.
         *
         * Skip only when quantisation could BE the whole error, not merely
         * when the window is short. A reading hundreds of percent high is
         * unambiguous however few samples were expected -- a poll loop cannot
         * invent conversions -- and that case must still fail.
         */
        double q = imt_rate_quantum(eff_odr, span);
        if (err > o->odr_tol_warn && err <= o->odr_tol_warn + q) {
            skip_check(r, "mag.rate", "Measured mag rate",
                       "the miss is no larger than one sample in this window — "
                       "raise --odr-window to resolve it at a low mag ODR");
        } else {

        int  dir  = imt_rate_dir(r->raw.mag_rate_hz, (double)eff_odr,
                                 o->odr_tol_warn);
        bool high = dir >= 0;      /* direction: decides the wording */
        bool over = dir > 0;       /* and outside tolerance: decides the grade */
        add_check(r, "mag.rate", "Measured mag rate",
                  err <= o->odr_tol_warn      ? IMT_PASS
                : (over && err > o->odr_tol_fail) ? IMT_FAIL : IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%.1f Hz", r->raw.mag_rate_hz),
                  fmtbuf(eb, sizeof eb, "%.6g Hz +/-%.0f%%", eff_odr,
                         o->odr_tol_warn * 100),
                  (over && err > o->odr_tol_fail)
                       ? "%llu samples in %.2f s, %.1f%% ABOVE the configured "
                         "rate — the poll loop cannot outrun the part, so "
                         "init()'s rate write did not land or is encoded wrong."
                  : over ? "%llu samples in %.2f s, %.1f%% ABOVE the configured "
                         "rate. The poll loop cannot cause that, so it is the "
                         "part's own oscillator unless the margin grows."
                  : high ? "%llu samples in %.2f s, %.1f%% above the configured "
                         "rate but inside tolerance."
                       : "%llu samples in %.2f s (%.1f%% low); the poll loop "
                         "bounds this from above, so a low reading can be "
                         "pacing rather than the chip.",
                  (unsigned long long)got, span, err * 100);
        }
    }

    /*
     * The contract line that most often gets broken: a not-ready sensor must
     * report 1, not -1.  imu.c counts -1 toward the error-reset threshold, so
     * a driver that returns it for "no data yet" makes the daemon reset a
     * perfectly healthy chip.
     */
    add_check(r, "mag.nodata_not_error", "Not-ready returns 1, not -1",
              rcneg == 0 ? IMT_PASS : IMT_FAIL,
              fmtbuf(mb, sizeof mb, "%d not-ready, %d errors", rc1, rcneg),
              "0 errors",
              rcneg == 0
              ? "polling faster than the ODR produced %d not-ready returns "
                "and no bus errors, which is exactly right"
              : "read() returned -1 on a healthy bus. -1 is reserved for I2C "
                "faults; DRDY-not-set and overflow must return 1.", rc1);

    if (got == 0) {
        skip_check(r, "mag.field_magnitude", "Field magnitude", "no samples");
        skip_check(r, "mag.noise", "Mag noise floor", "no samples");
        skip_check(r, "mag.wall_ns", "wall_ns is stamped and monotonic", "no samples");
    } else {
        double n = r->raw.mag_norm_mean;
        imt_status_t st = (n >= 25.0 && n <= 65.0) ? IMT_PASS
                        : (n >= 15.0 && n <= 100.0) ? IMT_WARN : IMT_FAIL;
        add_check(r, "mag.field_magnitude", "Field magnitude", st,
                  fmtbuf(mb, sizeof mb, "%.1f uT", n), "25 .. 65 uT",
                  "range %.1f .. %.1f uT. Earth's field is 25-65 uT; well "
                  "outside that is a scaling error or heavy local iron.",
                  r->raw.mag_norm_min, r->raw.mag_norm_max);

        bool stuck = false;
        for (int k = 0; k < 3; k++)
            if (r->raw.magf.sigma[k] == 0.0 || !isfinite(r->raw.magf.sigma[k]))
                stuck = true;
        add_check(r, "mag.noise", "Mag noise floor", stuck ? IMT_FAIL : IMT_PASS,
                  fmtbuf(mb, sizeof mb, "%.3f / %.3f / %.3f uT",
                         r->raw.magf.sigma[0], r->raw.magf.sigma[1],
                         r->raw.magf.sigma[2]),
                  "all axes > 0",
                  stuck ? "an axis has exactly zero variance — that axis is "
                          "stuck and is not being decoded."
                        : "per-axis standard deviation at rest");

        add_check(r, "mag.wall_ns", "wall_ns is stamped and monotonic",
                  (zero_wall == 0 && backwards_wall == 0) ? IMT_PASS : IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%d zero, %d backwards",
                         zero_wall, backwards_wall), "0 / 0",
                  (zero_wall == 0 && backwards_wall == 0)
                  ? "every sample carried a non-decreasing CLOCK_REALTIME stamp"
                  : "wall_ns must be set on every valid sample and never go "
                    "backwards — fusion timestamps mag updates with it.");
    }

    /*
     * Framing, before the degauss: it only reads, but it needs a window where
     * the sample holds still, and the coil pulses below deliberately move it.
     */
    {
        uint8_t hex[64];
        int hexn = 0;
        check_burst_framing(r, "mag.burst_framing",
                            "Burst read lands where single reads do",
                            bus, regmap_for(mag->name), hex, &hexn);
        if (hexn > 0) {
            r->raw.mag_bf_n = hexn;
            memcpy(r->raw.mag_bf_burst,  hex,        (size_t)hexn);
            memcpy(r->raw.mag_bf_single, hex + hexn, (size_t)hexn);
        }
    }

    /*
     * Now split the reading into the field and the offset.  Last, because it
     * drives the coil both ways and leaves the part re-SET behind it — the
     * numbers above are measured under one steady magnetisation, this one
     * deliberately changes it twice.
     */
    check_mag_degauss(r, o, mag, bus, eff_odr);
}

/* ── The mag rate the DAEMON gets, not the one a poller can see ───────────── */

/*
 * Everything above polls the magnetometer.  The daemon does not: it blocks on
 * the mag interrupt.  On 2026-08-18 that difference was worth a factor of three
 * — a polled 105.4 Hz against 35 Hz in the daemon — and nothing in this tool
 * could see it, because every check here measured the path the daemon does not
 * use.  A validation tool that cannot measure the production path will keep
 * certifying drivers that do not work in it.
 *
 * The mechanism is in mmc5983ma.c: when DRDY is a latched interrupt whose
 * acknowledge write also clears the status bit read() gates on, the gate and
 * the edge are mutually exclusive, and a reader blocked on the edge never sees
 * the bit.  That is a property of the part, so this check states the symptom
 * — the two rates disagree — rather than assuming the cause.
 */
typedef struct {
    const mag_ops_t   *mag;
    const imud_bus_t  *bus;
    int                samples;   /* read() == 0 */
    int                nodata;    /* read() == 1 */
    int                errors;    /* read() <  0 */
} mag_drdy_ctx_t;

static void mag_drdy_cb(void *user)
{
    mag_drdy_ctx_t *m = user;
    mag_sample_t s;
    memset(&s, 0, sizeof s);
    int rc = m->mag->read(m->bus, &s);
    if      (rc == 0) m->samples++;
    else if (rc > 0)  m->nodata++;
    else              m->errors++;
}

/*
 * Put the magnetometer back into the polled mode the rest of the report needs.
 *
 * Failure here is worth a line of its own: every mag check that follows would
 * be measuring the wrong thing, and without this the operator would see a set
 * of plausible numbers with nothing to say they are untrustworthy.
 */
static void mag_restore_polled(imt_report_t *r, const mag_ops_t *mag,
                               const imud_bus_t *bus, double eff_odr)
{
    mag_cfg_t polled = { .odr_mhz = (int)(eff_odr * 1000.0 + 0.5), .set_period_s = 0.0f,
                         .int_driven = false };
    if (mag->init(bus, &polled) < 0)
        add_check(r, "mag.drdy.restore", "Mag returned to polled mode",
                  IMT_FAIL, "init() failed", "ok",
                  "the magnetometer could not be put back into polled mode "
                  "after the interrupt-line measurement. Any mag result below "
                  "this line was read the wrong way and cannot be trusted.");
}

static void measure_mag_drdy(imt_report_t *r, const imt_opts_t *o,
                             const mag_ops_t *mag, const imud_bus_t *bus,
                             const imud_config_t *cfg, double eff_odr);

/*
 * Acknowledge whatever the interrupt is currently asserting, from inside the
 * edge window.  This part's DRDY is LATCHED: it goes high when a conversion
 * completes and is re-armed only by the write that clears Meas_M_Done, which
 * read() performs.  If it went high while the check was still setting up, the
 * rising edge is already spent and nothing else will make another.
 *
 * Deliberately NOT counted: it is arming, not a measurement, and counting it
 * would report one more sample than there were edges.
 *
 * Return value ignored: 1 (no new measurement) is the ordinary case, and a
 * bus error shows up immediately in the counters below rather than here.
 */
static void mag_drdy_prime(void *user)
{
    mag_drdy_ctx_t *m = user;
    mag_sample_t s;
    memset(&s, 0, sizeof s);
    (void)m->mag->read(m->bus, &s);
}

static void check_mag_drdy(imt_report_t *r, const imt_opts_t *o,
                           const mag_ops_t *mag, const imud_bus_t *bus,
                           const imud_config_t *cfg, int eff_odr)
{
    r->raw.mag_drdy_edges = -1;

    /*
     * `mag` cannot actually be NULL here — the caller runs this only when
     * check_bringup() set mag_ok, and that is unreachable on the !mag path.
     * But the correlation travels through an out-parameter in another
     * function, so it is invisible to a reader and to the static analyzer,
     * which reports the dereference as a null deref.  State the precondition
     * where it is used rather than leaving it to be re-derived.
     */
    if (!mag || !mag->has_interrupt || cfg->mag_int_gpio <= 0) {
        skip_check(r, "mag.drdy.rate", "Mag rate over its interrupt line",
                   (mag && mag->has_interrupt)
                   ? "mag.int_gpio is 0 — the reader uses a polling timer"
                   : "this part has no interrupt pin; the reader polls it");
        return;
    }

    /*
     * Re-init as the daemon does, so read() answers "is there new data?" the
     * way it will in production, then put the part back — everything that runs
     * later in the report polls.
     *
     * The restore is ONE unconditional statement below rather than a step on
     * each exit path, because a driver may latch the mode before the first bus
     * write that can fail.  mmc5983ma.c does, deliberately: a half-completed
     * init must not leave the staleness guard holding a sample taken in the
     * other mode.  So even a FAILED init() can leave the driver
     * interrupt-driven, and any early return that skipped the restore would
     * hand every later mag check a read() that bypasses its status gate,
     * silently changing what mag.rate, mag.noise and mag.field measure.
     * Structure it so there is no path to get this wrong on.
     */
    mag_cfg_t irq = { .odr_mhz = (int)(eff_odr * 1000.0 + 0.5), .set_period_s = 0.0f,
                      .int_driven = true };
    if (mag->init(bus, &irq) == 0)
        measure_mag_drdy(r, o, mag, bus, cfg, eff_odr);
    else
        skip_check(r, "mag.drdy.rate", "Mag rate over its interrupt line",
                   "re-init in interrupt mode failed");

    mag_restore_polled(r, mag, bus, eff_odr);
}

/*
 * The measurement itself: count edges on the mag's interrupt line for the DRDY
 * window and read behind each one, then grade the rate that produces.
 *
 * Split out of check_mag_drdy so that function's init/restore pair brackets a
 * single call and cannot be escaped by an early return.  Grades into *r and
 * touches nothing else.
 */
static void measure_mag_drdy(imt_report_t *r, const imt_opts_t *o,
                             const mag_ops_t *mag, const imud_bus_t *bus,
                             const imud_config_t *cfg, double eff_odr)
{
    char mb[56], eb[56];

    mag_drdy_ctx_t m = { .mag = mag, .bus = bus };
    imt_gpio_why_t why = IMT_GPIO_OK;
    /* Long enough to resolve the tolerance at THIS mag rate. */
    double win = o->drdy_window_s;
    double need = imt_rate_window_s((int)(eff_odr + 0.5), o->odr_tol_warn);
    if (need > win) win = need;
    long ms = (long)(win * 1e3);
    int edges = imt_gpio_count_edges(cfg->gpio_chip, cfg->mag_int_gpio, ms,
                                     mag_drdy_cb, &m, &why, mag_drdy_prime,
                                     r->mag_eff_odr_mhz);  /* milli-Hz: fallback timer */

    if (edges < 0) {
        const char *reason =
            why == IMT_GPIO_EBUSY   ? "mag GPIO is held by another process — is imud running?"
          : why == IMT_GPIO_ENOCHIP ? "GPIO chip not found; check device.gpio_chip"
          : why == IMT_GPIO_UNSUPPORTED ? "built without libgpiod"
          : "GPIO request failed";
        skip_check(r, "mag.drdy.rate", "Mag rate over its interrupt line", reason);
        return;
    }

    double rate = m.samples / win;
    r->raw.mag_drdy_edges    = edges;
    r->raw.mag_drdy_samples  = m.samples;
    r->raw.mag_drdy_window_s = win;
    r->raw.mag_drdy_rate_hz  = rate;

    /*
     * Can this window resolve the tolerance at all?  Decide from the CONFIGURED
     * rate, not from the count observed -- otherwise a genuine zero-sample
     * defect excuses itself as "too few samples", which is the failure this
     * check exists to catch.  Not from the polled rate either: that is itself
     * a measurement, and it reads ~0 on exactly the broken part this check is
     * for, which would turn the defect into a SKIP.
     *
     * The rate is a count over a fixed window, so its resolution is one sample,
     * +/-1/N.  Grading at +/-5% needs N >= 20, and at 1 Hz a 3 s window holds
     * three.  The bench reported 0.7 Hz against 1.0 Hz polled and FAILed a
     * working part on two samples where the arithmetic wanted three -- a
     * rounding boundary, not a defect.
     */
    /*
     * Can this window conclude anything at all?  One sample of resolution over
     * `eff_odr * window` expected, so below about 1/tol expected samples the
     * measurement cannot separate a working part from a broken one -- at 1 Hz
     * over 3 s it expects three edges, and catching one late is a boundary
     * rather than a fault.  This gates the zero-sample verdict too: zero is
     * damning only when many were expected.  At 105 Hz that is 316, so the
     * defect this check exists for still fails loudly.
     */
    if (imt_rate_quantum(eff_odr, win) > o->odr_tol_warn) {
        skip_check(r, "mag.drdy.rate", "Mag rate over its interrupt line",
                   "too few edges expected in this window to conclude anything "
                   "— raise --drdy-window for a low mag ODR");
        return;
    }

    if (m.samples == 0) {
        add_check(r, "mag.drdy.rate", "Mag rate over its interrupt line",
                  IMT_FAIL, fmtbuf(mb, sizeof mb, "0 Hz"),
                  fmtbuf(eb, sizeof eb, "~%.1f Hz", r->raw.mag_rate_hz),
                  "%d edge(s) arrived but read() returned data on none of them "
                  "(%d not-ready, %d errors). The daemon would get no "
                  "magnetometer at all.", edges, m.nodata, m.errors);
        return;
    }

    /*
     * Graded against the POLLED rate rather than the configured ODR, because
     * that is the comparison that isolates the wait: both figures come from the
     * same part in the same state moments apart, so a gap between them is the
     * path, not the oscillator.  (mag.rate above already grades against the ODR.)
     */
    double ref = r->raw.mag_rate_hz;
    double err = ref > 0 ? fabs(rate - ref) / ref : 0.0;

    /* As for mag.rate: one sample of resolution, so a miss no larger than that
     * cannot be told from a rounding boundary. A zero-sample run is handled
     * above and still fails, because no window makes that ambiguous. */
    double q = imt_rate_quantum(ref, o->drdy_window_s);
    if (err > o->odr_tol_warn && err <= o->odr_tol_warn + q) {
        skip_check(r, "mag.drdy.rate", "Mag rate over its interrupt line",
                   "the miss is no larger than one sample in this window — "
                   "raise --drdy-window to resolve it at a low mag ODR");
        return;
    }

    add_check(r, "mag.drdy.rate", "Mag rate over its interrupt line",
              err <= o->odr_tol_warn ? IMT_PASS
            : err >  o->odr_tol_fail ? IMT_FAIL : IMT_WARN,
              fmtbuf(mb, sizeof mb, "%.1f Hz", rate),
              fmtbuf(eb, sizeof eb, "%.1f Hz polled +/-%.0f%%", ref,
                     o->odr_tol_warn * 100),
              err <= o->odr_tol_warn
              ? "%d samples from %d edge(s) in %.2f s, matching the polled "
                "rate — the daemon gets what this report measures."
              : "%d samples from %d edge(s) in %.2f s, %.0f%% off the polled "
                "rate. The daemon waits on this line, so THIS is the rate it "
                "gets. A driver that gates read() on a status bit the interrupt "
                "acknowledge clears will stall here until its timeout.",
              m.samples, edges, win, err * 100);
}

/* ── Phase B: guided six-face accelerometer / axis-sign test ──────────────── */

/*
 * The rule, in the NED-compatible board frame (X bow, Y starboard, Z DOWN):
 * the axis pointing UP reads +g, the axis pointing DOWN reads -g.  Face 1 is
 * the cross-check on the whole convention and matches the driver guide: flat,
 * component side up, must read [0, 0, -9.807].
 *
 * This is the check that catches a wrong axis flip, which is the single most
 * likely defect in a new driver.
 */
static const struct {
    const char *id;
    const char *label;
    const char *instruction;
    int axis, sign;
} imt_faces[6] = {
    { "face.1", "Flat, component side up",
      "Lay the board flat on the bench the normal way up.", 2, -1 },
    { "face.2", "Flat, upside down",
      "Turn the board over so the components face the bench.", 2, +1 },
    { "face.3", "Nose down",
      "Stand the board on its front edge, the bow arrow (chip +X) "
      "pointing at the floor.", 0, -1 },
    { "face.4", "Nose up",
      "Stand the board on its back edge, the bow arrow (chip +X) "
      "pointing at the ceiling.", 0, +1 },
    { "face.5", "Starboard side down",
      "Stand the board on its right-hand (starboard) edge.", 1, -1 },
    { "face.6", "Port side down",
      "Stand the board on its left-hand (port) edge.", 1, +1 },
};

static int dominant_axis(const double a[3])
{
    int k = 0;
    for (int i = 1; i < 3; i++) if (fabs(a[i]) > fabs(a[k])) k = i;
    return k;
}

static void phase_faces(imt_report_t *r, const imt_opts_t *o, drain_ctx_t *d)
{
    char mb[64], eb[64], id[32], nm[64], body[512];
    static const char axis_name[3] = { 'X', 'Y', 'Z' };
    int frame_bad = 0;

    for (int f = 0; f < 6 && !g_abort; f++) {
        double want[3] = { 0, 0, 0 };
        want[imt_faces[f].axis] = imt_faces[f].sign * IMT_G_MS2;

        snprintf(body, sizeof body,
                 "%s\n"
                 "  Expected: accel ~ [ %+.2f, %+.2f, %+.2f ] m/s^2\n"
                 "  (board %c points %s, so %c reads %sg)",
                 imt_faces[f].instruction, want[0], want[1], want[2],
                 axis_name[imt_faces[f].axis],
                 imt_faces[f].sign > 0 ? "up" : "down",
                 axis_name[imt_faces[f].axis],
                 imt_faces[f].sign > 0 ? "+" : "-");

        snprintf(nm, sizeof nm, "Face %d/6: %s", f + 1, imt_faces[f].label);
        int pr = ui_prompt(o, imt_faces[f].id, nm, body);
        if (pr < 0) { r->aborted = true; return; }
        if (pr > 0) {
            snprintf(id, sizeof id, "%s.sign", imt_faces[f].id);
            skip_check(r, id, nm, "skipped by the operator");
            continue;
        }

        drain_flush(d);
        sleep_s(o->face_settle_s);
        drain_flush(d);

        imt_stats3_t acc;
        double grav = 0;
        uint64_t n = collect_stats(o, d, o->face_collect_s, imt_faces[f].id,
                                   &acc, NULL, &grav);

        imt_face_row_t *row = &r->raw.face[r->raw.n_faces];
        row->idx      = f;
        row->label    = imt_faces[f].label;
        row->exp_axis = imt_faces[f].axis;
        row->exp_sign = imt_faces[f].sign;
        for (int k = 0; k < 3; k++) row->a[k] = acc.mean[k];
        row->norm     = grav;
        row->n        = (int)n;
        row->got_axis = dominant_axis(acc.mean);
        row->got_sign = acc.mean[row->got_axis] >= 0 ? +1 : -1;

        if (n < 10) {
            row->status = IMT_SKIP;
            snprintf(id, sizeof id, "%s.sign", imt_faces[f].id);
            skip_check(r, id, nm, "too few samples collected at this face");
            if (r->raw.n_faces < 5) r->raw.n_faces++;
            continue;
        }
        if (r->raw.n_faces < 6) r->raw.n_faces++;

        /* Magnitude */
        double err = fabs(grav - IMT_G_MS2);
        snprintf(id, sizeof id, "%s.mag", imt_faces[f].id);
        snprintf(nm, sizeof nm, "Face %d magnitude", f + 1);
        add_check(r, id, nm,
                  err <= o->grav_tol_warn ? IMT_PASS
                : err <= o->grav_tol_fail ? IMT_WARN : IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%.3f m/s^2", grav),
                  fmtbuf(eb, sizeof eb, "9.807 +/-%.2f", o->grav_tol_warn),
                  "|a| at face %d", f + 1);

        /* Sign — the substantive check.  Diagnose, do not just report. */
        snprintf(id, sizeof id, "%s.sign", imt_faces[f].id);
        snprintf(nm, sizeof nm, "Face %d axis and sign", f + 1);
        bool axis_ok = (row->got_axis == row->exp_axis);
        bool sign_ok = (row->got_sign == row->exp_sign);
        row->status  = (axis_ok && sign_ok) ? IMT_PASS : IMT_FAIL;

        fmtbuf(mb, sizeof mb, "%c%c (%+.2f)",
               row->got_sign > 0 ? '+' : '-', axis_name[row->got_axis],
               acc.mean[row->got_axis]);
        fmtbuf(eb, sizeof eb, "%c%c (%+.2f)",
               row->exp_sign > 0 ? '+' : '-', axis_name[row->exp_axis],
               want[row->exp_axis]);

        if (axis_ok && sign_ok)
            add_check(r, id, nm, IMT_PASS, mb, eb,
                      "measured [%+.2f, %+.2f, %+.2f] m/s^2",
                      acc.mean[0], acc.mean[1], acc.mean[2]);
        else if (axis_ok)
            add_check(r, id, nm, IMT_FAIL, mb, eb,
                      "right axis, wrong sign: the %c sign flip is missing "
                      "from (or spurious in) the driver's chip-to-board remap.",
                      axis_name[row->exp_axis]);
        else
            add_check(r, id, nm, IMT_FAIL, mb, eb,
                      "gravity landed on %c but should be on %c — those two "
                      "axes appear swapped in the chip-to-board remap.",
                      axis_name[row->got_axis], axis_name[row->exp_axis]);

        if (row->status != IMT_PASS) frame_bad++;

        /* Cross-axis leakage grades the bench, not the driver: hand placement
         * on a bench edge is good to a few degrees at best. */
        double cross = 0;
        for (int k = 0; k < 3; k++)
            if (k != row->exp_axis && fabs(acc.mean[k]) > cross)
                cross = fabs(acc.mean[k]);
        snprintf(id, sizeof id, "%s.cross", imt_faces[f].id);
        snprintf(nm, sizeof nm, "Face %d off-axis components", f + 1);
        add_check(r, id, nm, cross <= 1.5 ? IMT_PASS : IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%.2f m/s^2", cross), "< 1.5 m/s^2",
                  cross <= 1.5 ? "the board was close to square with gravity"
                               : "about %.0f degrees off square — this grades "
                                 "how the board was held, not the driver.",
                  asin(fmin(1.0, cross / IMT_G_MS2)) * 180.0 / M_PI);
    }

    /* Rollup */
    if (r->raw.n_faces < 6) {
        skip_check(r, "faces.frame", "Board frame matches NED",
                   "not all six faces were measured");
    } else {
        add_check(r, "faces.frame", "Board frame matches NED",
                  frame_bad ? IMT_FAIL : IMT_PASS,
                  fmtbuf(mb, sizeof mb, "%d of 6 faces wrong", frame_bad),
                  "0 wrong",
                  frame_bad ? "the chip-to-board axis remap is wrong; see the "
                              "per-face diagnoses above."
                            : "X forward, Y starboard, Z down confirmed on all "
                              "six faces, signs included.");
    }

    /*
     * The calibration model imud-cal accel would derive from these same six
     * readings.  Recorded for information: a sane offset/scale pair here means
     * a calibration run on this board would produce something usable.
     */
    if (r->raw.n_faces == 6) {
        bool have[3] = { false, false, false };
        double plus[3] = { 0, 0, 0 }, minus[3] = { 0, 0, 0 };
        for (int i = 0; i < 6; i++) {
            imt_face_row_t *row = &r->raw.face[i];
            if (row->exp_sign > 0) plus[row->exp_axis]  = row->a[row->exp_axis];
            else                   minus[row->exp_axis] = row->a[row->exp_axis];
            have[row->exp_axis] = true;
        }
        char buf[64] = "";
        for (int k = 0; k < 3; k++) {
            if (!have[k]) continue;
            r->raw.face_offset[k] = (plus[k] + minus[k]) / 2.0;
            double half = (plus[k] - minus[k]) / 2.0;
            r->raw.face_scale[k]  = fabs(half) > 0.1 ? IMT_G_MS2 / half : 1.0;
        }
        add_check(r, "faces.symmetry", "Derived accel offset and scale", IMT_INFO,
                  fmtbuf(buf, sizeof buf, "scale %.3f/%.3f/%.3f",
                         r->raw.face_scale[0], r->raw.face_scale[1],
                         r->raw.face_scale[2]),
                  "~1.000 each",
                  "offsets %+.3f/%+.3f/%+.3f m/s^2. This is the model "
                  "`imud-cal accel` fits; scales far from 1.0 mean a "
                  "sensitivity error rather than a mounting one.",
                  r->raw.face_offset[0], r->raw.face_offset[1],
                  r->raw.face_offset[2]);
    }
}

/* ── Phase C: guided gyro rotation ────────────────────────────────────────── */

/*
 * Right-hand rule in the board frame: +X roll puts starboard down, +Y pitch
 * puts the bow up, +Z yaw turns clockwise seen from above.
 *
 * The scale tolerance is deliberately wide.  A hand turn to "90 degrees" is
 * +/-10 degrees at best, so this cannot measure sensitivity — it exists to
 * catch factor errors, and the two that matter are 57.30 (returned deg/s where
 * the contract says rad/s) and 0.01745 (converted twice).
 */
static const struct {
    const char *id;
    const char *cmd;
    int axis;
} imt_turns[3] = {
    { "gyro.x", "ROLL the board to starboard: rotate about the bow-stern axis "
                "so the right-hand edge goes DOWN", 0 },
    { "gyro.y", "PITCH the bow UP: rotate about the port-starboard axis so the "
                "bow (chip +X) rises", 1 },
    { "gyro.z", "YAW CLOCKWISE seen from above: keep the board flat and swing "
                "the bow to starboard", 2 },
};

static void phase_gyro(imt_report_t *r, const imt_opts_t *o, drain_ctx_t *d,
                       const imu_ops_t *imu, int eff_odr)
{
    char mb[64], eb[64], id[32], nm[64], body[512];
    static const char axis_name[3] = { 'X', 'Y', 'Z' };
    imu_sample_t buf[128];
    int n = 0;

    bool use_ts = imu->has_hw_timestamp && d->tick_ns != 0;
    double dt_nominal = 1.0 / (double)eff_odr;

    for (int t = 0; t < 3 && !g_abort; t++) {
        int axis = imt_turns[t].axis;
        snprintf(body, sizeof body,
                 "%s,\n"
                 "  by about %.0f degrees, slowly and steadily.\n"
                 "  Signal when the turn is finished.\n"
                 "  Expected: theta%c ~ %+.0f deg, the other two near zero.",
                 imt_turns[t].cmd, o->turn_deg, axis_name[axis], o->turn_deg);
        snprintf(nm, sizeof nm, "Gyro %c rotation", axis_name[axis]);

        int pr = ui_prompt(o, imt_turns[t].id, nm, body);
        if (pr < 0) { r->aborted = true; return; }
        if (pr > 0) {
            snprintf(id, sizeof id, "%s.sign", imt_turns[t].id);
            skip_check(r, id, nm, "skipped by the operator");
            continue;
        }

        drain_flush(d);

        double theta[3] = { 0, 0, 0 };
        uint64_t count = 0;
        bool have_prev_ts = false;
        uint32_t prev_ts = 0;
        double t0 = now_s(), deadline = t0 + o->turn_timeout_s;
        bool done = false;

        while (now_s() < deadline && !g_abort && !done) {
            if (drain_once(d, buf, 128, &n) < 0) { sleep_s(0.002); continue; }
            for (int i = 0; i < n; i++) {
                double dt = dt_nominal;
                if (use_ts && buf[i].chip_ts) {
                    if (have_prev_ts) {
                        uint32_t delta = buf[i].chip_ts - prev_ts;
                        if (delta > 0 && delta < 0x80000000u)
                            dt = (double)delta * (double)d->tick_ns * 1e-9;
                    }
                    prev_ts = buf[i].chip_ts;
                    have_prev_ts = true;
                }
                for (int k = 0; k < 3; k++)
                    theta[k] += (double)buf[i].gyro[k] * dt * 180.0 / M_PI;
                count++;
            }
            fmtbuf(mb, sizeof mb, "%+.1f / %+.1f / %+.1f deg",
                   theta[0], theta[1], theta[2]);
            ui_progress(o, imt_turns[t].id, -1.0, mb);

            int pd = ui_poll_done(o);
            if (pd < 0) { r->aborted = true; return; }
            if (pd > 0) done = true;
            sleep_s(0.002);
        }

        /* One row per commanded turn.  The index is clamped before it is used,
         * not after: a cap that trailed the write dropped the Z row out of the
         * appendix while still letting phase C grade it. */
        const int turn_cap = (int)(sizeof r->raw.turn / sizeof r->raw.turn[0]);
        if (r->raw.n_turns >= turn_cap) continue;
        imt_turn_row_t *row = &r->raw.turn[r->raw.n_turns++];
        row->axis    = axis;
        row->cmd_deg = o->turn_deg;
        row->dur_s   = now_s() - t0;
        row->n       = (int)count;
        row->used_chip_ts = use_ts;
        for (int k = 0; k < 3; k++) row->theta[k] = theta[k];

        if (count < 10) {
            snprintf(id, sizeof id, "%s.sign", imt_turns[t].id);
            skip_check(r, id, nm, "too few samples during the turn");
            row->status = IMT_SKIP;
            continue;
        }

        double got = theta[axis];

        /* Sign */
        snprintf(id, sizeof id, "%s.sign", imt_turns[t].id);
        row->status = got > 0 ? IMT_PASS : IMT_FAIL;
        fmtbuf(mb, sizeof mb, "%+.1f deg", got);
        fmtbuf(eb, sizeof eb, "%+.0f deg", o->turn_deg);
        if (got > 0)
            add_check(r, id, nm, IMT_PASS, mb, eb,
                      "integrated the right way for a commanded +%.0f degrees "
                      "about %c", o->turn_deg, axis_name[axis]);
        else
            add_check(r, id, nm, IMT_FAIL, mb, eb,
                      "the %c gyro sign is inverted: a commanded +%.0f degree "
                      "turn integrated to %+.1f.",
                      axis_name[axis], o->turn_deg, got);

        /* Scale */
        snprintf(id, sizeof id, "%s.scale", imt_turns[t].id);
        snprintf(nm, sizeof nm, "Gyro %c scale factor", axis_name[axis]);
        double ratio = o->turn_deg > 0 ? fabs(got) / o->turn_deg : 0.0;
        imt_status_t st = (ratio > 0.8 && ratio < 1.2) ? IMT_PASS
                        : (ratio > 0.6 && ratio < 1.4) ? IMT_WARN : IMT_FAIL;
        const char *hint = "";
        if (st == IMT_FAIL) {
            if (ratio > 40.0 && ratio < 80.0)
                hint = " — about 57.3x, so read() is returning deg/s where the "
                       "contract says rad/s";
            else if (ratio > 0.01 && ratio < 0.03)
                hint = " — about 1/57.3, so the deg-to-rad conversion is "
                       "being applied twice";
        }
        add_check(r, id, nm, st,
                  fmtbuf(mb, sizeof mb, "%.2fx commanded", ratio),
                  "1.0 +/-20%",
                  "integrated %+.1f deg against a commanded %.0f%s. A hand "
                  "turn is only good to about +/-10%%, so this catches factor "
                  "errors, not sensitivity.", got, o->turn_deg, hint);

        /* Cross-axis */
        double cross = 0;
        for (int k = 0; k < 3; k++)
            if (k != axis && fabs(theta[k]) > cross) cross = fabs(theta[k]);
        snprintf(id, sizeof id, "%s.cross", imt_turns[t].id);
        snprintf(nm, sizeof nm, "Gyro %c cross-axis", axis_name[axis]);
        add_check(r, id, nm,
                  cross <= 0.30 * fabs(got) ? IMT_PASS : IMT_WARN,
                  fmtbuf(mb, sizeof mb, "%.1f deg", cross),
                  "< 30% of the commanded axis",
                  "other axes integrated to %+.1f / %+.1f / %+.1f deg; a large "
                  "value means swapped axes or a sloppy turn.",
                  theta[0], theta[1], theta[2]);
    }
}

/* ── Phase D: guided magnetometer spin ────────────────────────────────────── */

static void phase_spin(imt_report_t *r, const imt_opts_t *o, drain_ctx_t *d,
                       const mag_ops_t *mag, const imud_bus_t *bus,
                       const imud_config_t *cfg, const imu_ops_t *imu,
                       int eff_odr)
{
    char mb[64], eb[64];
    imu_sample_t ibuf[128];
    int in = 0;

    int pr = ui_prompt(o, "spin", "Magnetometer spin",
                       "Hold the board LEVEL and turn it slowly through at "
                       "least two full circles, clockwise seen from above.\n"
                       "  Keep it away from steel, motors, and speakers.\n"
                       "  Signal when the circles are complete.");
    if (pr < 0) { r->aborted = true; return; }
    if (pr > 0) {
        skip_check(r, "spin.magnitude", "Spin field magnitude", "skipped by the operator");
        skip_check(r, "spin.coverage", "Spin heading coverage", "skipped by the operator");
        skip_check(r, "spin.frame_agreement", "Mag frame agrees with the gyro",
                   "skipped by the operator");
        return;
    }

    int sectors[IMT_MAG_SECTORS] = { 0 };
    double bx_min = 1e30, bx_max = -1e30, by_min = 1e30, by_max = -1e30;
    double bz_min = 1e30, bz_max = -1e30, bz_sum = 0, norm_sum = 0;
    uint64_t got = 0;
    double gyro_z_deg = 0, heading_unwrapped = 0, prev_heading = 0;
    bool have_heading = false;
    bool use_ts = imu->has_hw_timestamp && d->tick_ns != 0;
    double dt_nominal = 1.0 / (double)eff_odr;
    bool have_prev_ts = false;
    uint32_t prev_ts = 0;

    drain_flush(d);
    double t0 = now_s(), deadline = t0 + o->spin_timeout_s;
    bool done = false;

    while (now_s() < deadline && !g_abort && !done) {
        /* Gyro Z, integrated over the same interval as the mag heading. */
        if (drain_once(d, ibuf, 128, &in) >= 0) {
            for (int i = 0; i < in; i++) {
                double dt = dt_nominal;
                if (use_ts && ibuf[i].chip_ts) {
                    if (have_prev_ts) {
                        uint32_t delta = ibuf[i].chip_ts - prev_ts;
                        if (delta > 0 && delta < 0x80000000u)
                            dt = (double)delta * (double)d->tick_ns * 1e-9;
                    }
                    prev_ts = ibuf[i].chip_ts;
                    have_prev_ts = true;
                }
                gyro_z_deg += (double)ibuf[i].gyro[2] * dt * 180.0 / M_PI;
            }
        }

        mag_sample_t s;
        memset(&s, 0, sizeof s);
        if (mag->read(bus, &s) == 0 && s.valid) {
            double bx = s.field[0], by = s.field[1], bz = s.field[2];
            if (bx < bx_min) bx_min = bx;
            if (bx > bx_max) bx_max = bx;
            if (by < by_min) by_min = by;
            if (by > by_max) by_max = by;
            if (bz < bz_min) bz_min = bz;
            if (bz > bz_max) bz_max = bz;
            bz_sum   += bz;
            norm_sum += sqrt(bx * bx + by * by + bz * bz);
            got++;

            /*
             * Heading from the horizontal field.  Bow north puts the field
             * along +X (0 deg); bow east puts magnetic north to port, so
             * By < 0, giving +90 deg.  Unwrapped so the total swing over the
             * spin can be compared against the gyro.
             */
            double heading = atan2(-by, bx) * 180.0 / M_PI;
            if (have_heading) {
                double dh = heading - prev_heading;
                while (dh >  180.0) dh -= 360.0;
                while (dh < -180.0) dh += 360.0;
                heading_unwrapped += dh;
            }
            prev_heading = heading;
            have_heading = true;

            double cx = (bx_min + bx_max) / 2.0, cy = (by_min + by_max) / 2.0;
            int cur = cal_cov_mark(sectors, IMT_MAG_SECTORS, bx, by, cx, cy);
            ui_coverage(o, sectors, IMT_MAG_SECTORS, cur, (int)got,
                        got ? norm_sum / (double)got : 0.0);
        }

        int pd = ui_poll_done(o);
        if (pd < 0) { r->aborted = true; return; }
        if (pd > 0) done = true;
        sleep_s(0.002);
    }

    memcpy(r->raw.spin_sectors, sectors, sizeof sectors);
    r->raw.spin_covered = cal_cov_count(sectors, IMT_MAG_SECTORS);
    r->raw.spin_n       = got;
    r->raw.spin_norm_mean = got ? norm_sum / (double)got : 0.0;
    r->raw.spin_bz_mean   = got ? bz_sum / (double)got : 0.0;
    r->raw.spin_range[0]  = got ? bx_max - bx_min : 0.0;
    r->raw.spin_range[1]  = got ? by_max - by_min : 0.0;
    r->raw.spin_range[2]  = got ? bz_max - bz_min : 0.0;
    r->raw.spin_heading_delta_deg = heading_unwrapped;
    r->raw.spin_gyro_z_deg        = gyro_z_deg;

    if (got < 20) {
        skip_check(r, "spin.magnitude", "Spin field magnitude", "too few mag samples");
        skip_check(r, "spin.axes_vary", "All mag axes respond", "too few mag samples");
        skip_check(r, "spin.coverage", "Spin heading coverage", "too few mag samples");
        skip_check(r, "spin.frame_agreement", "Mag frame agrees with the gyro",
                   "too few mag samples");
        skip_check(r, "spin.dip", "Vertical field sign", "too few mag samples");
        return;
    }

    double nm_ = r->raw.spin_norm_mean;
    add_check(r, "spin.magnitude", "Spin field magnitude",
              (nm_ >= 25.0 && nm_ <= 65.0) ? IMT_PASS
            : (nm_ >= 15.0 && nm_ <= 100.0) ? IMT_WARN : IMT_FAIL,
              fmtbuf(mb, sizeof mb, "%.1f uT", nm_), "25 .. 65 uT",
              "mean |B| over %llu samples while turning",
              (unsigned long long)got);

    /*
     * X and Y must swing through the spin.  Z must merely not be constant: a
     * level spin barely changes the dip component, so requiring Z to vary
     * would be a false failure.
     */
    double bh = sqrt(r->raw.magf.mean[0] * r->raw.magf.mean[0] +
                     r->raw.magf.mean[1] * r->raw.magf.mean[1]);
    double need = 0.5 * (bh > 1.0 ? bh : nm_ * 0.5);
    bool xy_ok = r->raw.spin_range[0] >= need && r->raw.spin_range[1] >= need;
    bool z_stuck = r->raw.spin_range[2] == 0.0;
    add_check(r, "spin.axes_vary", "All mag axes respond",
              z_stuck ? IMT_FAIL : xy_ok ? IMT_PASS : IMT_WARN,
              fmtbuf(mb, sizeof mb, "%.1f / %.1f / %.1f uT",
                     r->raw.spin_range[0], r->raw.spin_range[1],
                     r->raw.spin_range[2]),
              fmtbuf(eb, sizeof eb, "X,Y > %.0f uT; Z nonzero", need),
              z_stuck ? "the Z axis never changed at all — it is stuck."
                      : xy_ok ? "X and Y both swept through the turn, as a "
                                "level spin requires"
                              : "X or Y barely moved; the turn may not have "
                                "been level or complete.");

    add_check(r, "spin.coverage", "Spin heading coverage",
              r->raw.spin_covered >= IMT_MAG_SECTORS ? IMT_PASS
            : r->raw.spin_covered >= 18 ? IMT_WARN : IMT_FAIL,
              fmtbuf(mb, sizeof mb, "%d/%d sectors", r->raw.spin_covered,
                     IMT_MAG_SECTORS),
              fmtbuf(eb, sizeof eb, "%d/%d", IMT_MAG_SECTORS, IMT_MAG_SECTORS),
              "how much of the heading circle the turn actually visited");

    /*
     * The check that makes this phase worth doing.  Two independent sensors
     * measure the same rotation; if the mag's axes are swapped or a sign is
     * inverted relative to the IMU, the heading runs backwards against the
     * gyro.  That is the most common magnetometer-driver defect.
     */
    if (fabs(gyro_z_deg) < 45.0) {
        skip_check(r, "spin.frame_agreement", "Mag frame agrees with the gyro",
                   "the gyro saw less than 45 degrees of turn — spin further");
    } else {
        double ratio = heading_unwrapped / gyro_z_deg;
        bool same_sign = ratio > 0;
        bool magnitude_ok = fabs(ratio - 1.0) < 0.25;
        add_check(r, "spin.frame_agreement", "Mag frame agrees with the gyro",
                  (same_sign && magnitude_ok) ? IMT_PASS
                : same_sign ? IMT_WARN : IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%+.0f deg mag", heading_unwrapped),
                  fmtbuf(eb, sizeof eb, "%+.0f deg gyro", gyro_z_deg),
                  !same_sign
                  ? "the mag heading turns the OPPOSITE way to the gyro: an "
                    "X or Y sign is inverted in the magnetometer's remap "
                    "relative to the IMU frame."
                  : magnitude_ok
                    ? "both sensors agree on the direction and amount of turn"
                    : "same direction but %.0f%% of the gyro's angle — likely "
                      "hard iron distorting the circle, or an incomplete turn.",
                  ratio * 100.0);
    }

    /* Dip sign needs to know which hemisphere the bench is in. */
    if (cfg->pos_lat_deg == 0.0) {
        skip_check(r, "spin.dip", "Vertical field sign",
                   "position.latitude is not set, so the expected dip sign is "
                   "unknown");
    } else {
        bool north = cfg->pos_lat_deg > 0;
        bool ok = north ? (r->raw.spin_bz_mean > 0) : (r->raw.spin_bz_mean < 0);
        add_check(r, "spin.dip", "Vertical field sign", ok ? IMT_PASS : IMT_FAIL,
                  fmtbuf(mb, sizeof mb, "%+.1f uT", r->raw.spin_bz_mean),
                  north ? "positive (northern hemisphere)"
                        : "negative (southern hemisphere)",
                  ok ? "Z points into the earth as expected at latitude %.1f"
                     : "the vertical component has the wrong sign for latitude "
                       "%.1f — the mag Z axis is inverted.",
                  cfg->pos_lat_deg);
    }
}

/* ── Recommendation ───────────────────────────────────────────────────────── */

/*
 * The named subset that has to PASS before a driver's `experimental` flag can
 * come off.  Everything else in the report is context; these are the claims.
 * A SKIP here suppresses the recommendation and says which one.
 */
static const char *imt_required_imu[] = {
    "imu.probe", "imu.probe.reject", "imu.reset.rc", "imu.init.rc",
    "imu.odr", "imu.seq.monotonic", "imu.seq.gapless",
    "imu.err.nodata_not_error", "imu.err.no_spurious",
    "imu.noise.accel", "imu.noise.gyro", "imu.rest.gravity",
    "imu.temp.plausible", "imu.chipts.presence", "imu.fs.accel",
    "faces.frame", "gyro.x.sign", "gyro.y.sign", "gyro.z.sign",
    NULL
};

static const char *imt_required_mag[] = {
    "mag.probe", "mag.init.rc", "mag.rate", "mag.nodata_not_error",
    "mag.field_magnitude", "mag.noise", "mag.wall_ns",
    "spin.magnitude", "spin.frame_agreement",
    NULL
};

/*
 * chip-time / wall-time ratio → a grade.  Declared in imutest.h; see there for
 * why it is exposed rather than static.
 *
 * This used to warn on any deviation past 2%, reasoning that imu.c multiplies
 * ts_tick_ns into every per-sample dt.  1.8 made that false: ts_anchor_t
 * measures the counter's real period across consecutive anchors and
 * chip_to_wall applies THAT (see "Timestamps + per-sample dt" in src/imu.c),
 * so an oscillator a few percent off is absorbed instead of scaling integrated
 * rotation.  A Pi 5 bench run on the reference ISM330DHCX reported 1.041 — a
 * ~4% fast part, well inside its tolerance — and the row read as a defect.  A
 * check that fires on expected silicon behaviour is one people learn to skip,
 * which costs more than the row is worth.
 *
 * Direction carries the meaning, so the bands are asymmetric:
 *
 *   |err| <= 2%     PASS   the tick describes the counter
 *   ratio > 1       INFO   part runs fast; the anchor absorbs it
 *   ratio < 1       WARN   chip time MISSING — a dropped wrap, unrecoverable
 *   |err| >  10%    FAIL   either way, ts_tick_ns is not this counter's period
 */
double imt_rate_window_s(int odr_hz, double tol)
{
    if (odr_hz <= 0 || tol <= 0.0) return 0.0;
    double need = 2.0 / (tol * (double)odr_hz);
    return need > IMT_RATE_WINDOW_CAP_S ? IMT_RATE_WINDOW_CAP_S : need;
}

double imt_rate_quantum(double nominal, double window_s)
{
    double n = nominal * window_s;
    /*
     * TWO samples, not one. A window that is not synchronised to the sample
     * clock can catch a partial period at each end, so a count of an f Hz
     * stream over T seconds lands anywhere in floor(fT) .. ceil(fT)+1. At 1 Hz
     * over 5 s that is 5, 6 or 7 -- 1.0, 1.2 or 1.4 Hz -- and reading the top
     * of that range as "39.9% ABOVE the configured rate" failed a part running
     * at its exact rate.
     */
    return n > 0.0 ? 2.0 / n : 1.0;
}

int imt_rate_dir(double measured, double nominal, double tol)
{
    if (measured <= nominal) return -1;
    return measured > nominal * (1.0 + tol) ? 1 : 0;
}

imt_status_t imt_chipts_wall_status(double ratio)
{
    double werr = fabs(ratio - 1.0);
    if (werr <= 0.02) return IMT_PASS;
    if (werr >  0.10) return IMT_FAIL;
    return ratio > 1.0 ? IMT_INFO : IMT_WARN;
}

uint32_t imt_ts_acc_step(imt_ts_acc_t *a, uint32_t ts)
{
    /* Absent, not early: skip it entirely rather than comparing it. See the
     * header for what comparing one costs the rate estimate. */
    if (ts == 0) { a->zeros++; return 0; }

    if (!a->have) {
        a->first = a->prev = a->last = ts;
        a->have  = true;
        a->n_seen = 1;
        return 0;                       /* no predecessor to measure against */
    }
    a->n_seen++;

    uint32_t delta = ts - a->prev;       /* modular, so a wrap reads forward */
    uint32_t out   = 0;
    if (delta == 0)                a->repeats++;
    else if (delta >= 0x80000000u) {
        a->reversals++;
        /* At a seam the anchor was re-derived; inside a burst it was not. */
        if (a->seam) a->seam_reversals++;
        /* Record the values, not just the count -- see the header. */
        if (a->n_rev < IMT_MAX_TS_REV) {
            a->rev[a->n_rev].idx  = a->n_seen - 1;
            a->rev[a->n_rev].prev = a->prev;
            a->rev[a->n_rev].cur  = ts;
            a->rev[a->n_rev].seam = a->seam;
            a->n_rev++;
        }
    }
    else {
        out = delta;
        if (ts < a->prev) a->wraps++;    /* forward across the 32-bit end */
    }
    a->seam = false;
    a->prev = a->last = ts;
    return out;
}

void imt_ts_acc_seam(imt_ts_acc_t *a)
{
    a->seam = true;
}

/*
 * Any corrupted read is a failure.  There is no rate low enough to be
 * acceptable: the value compared against is hard-wired and cannot change, so a
 * single wrong byte means a byte on this bus can come back wrong -- and the
 * driver reads WHO_AM_I exactly once in probe(), a chip timestamp once per
 * burst, and a full-scale setting once per init().  A rate of "only" 0.2% is
 * one probe() in five hundred rejecting a part that is present, which is
 * precisely how this was first seen.
 *
 * This used to WARN below 0.5%, which let a real defect read as a tolerance
 * band and get waved past.
 */
imt_status_t imt_bus_integrity_status(int bad, int total)
{
    if (total <= 0) return IMT_PASS;
    return bad > 0 ? IMT_FAIL : IMT_PASS;
}

imt_status_t imt_overflow_status(int rc, bool growing)
{
    if (rc == 1)  return IMT_PASS;        /* overflow reported, as required */
    if (rc < 0)   return IMT_FAIL;        /* a bus error instead of a verdict */
    return growing ? IMT_SKIP             /* never filled it; nothing learned */
                   : IMT_WARN;            /* at capacity and still rc 0 */
}

/* Median of a short series, sorted in place.  n >= 1. */
static double med_of(double *v, int n)
{
    for (int i = 1; i < n; i++) {
        double t = v[i]; int k = i - 1;
        while (k >= 0 && v[k] > t) { v[k + 1] = v[k]; k--; }
        v[k + 1] = t;
    }
    return v[n / 2];
}

bool imt_fs_scales_with_range(const imt_fs_row_t *rows, int n)
{
    if (n < 4) return false;              /* need two halves worth splitting */

    /* Rows with a measurement, in ascending full scale (the sweep builds them
     * that way; sorting is not worth it for <= 8 entries). */
    double sig[IMT_MAX_FS_ROWS], fs[IMT_MAX_FS_ROWS];
    int m = 0;
    for (int i = 0; i < n && m < IMT_MAX_FS_ROWS; i++) {
        double v = (rows[i].sigma[0] + rows[i].sigma[1] + rows[i].sigma[2]) / 3.0;
        if (v <= 0 || rows[i].fs <= 0) continue;
        sig[m] = v; fs[m] = (double)rows[i].fs; m++;
    }
    if (m < 4) return false;

    /*
     * PROPORTIONALITY, judged between the medians of the bottom and top halves.
     *
     * If quantisation dominates, sigma tracks full scale, so a 32x span of
     * range produces roughly a 32x span of sigma.  If analogue noise dominates
     * sigma is flat and the ratio is about 1.  Those are separated by more than
     * an order of magnitude, which is what makes this decidable on noisy data;
     * the requirement is only that sigma covers half the span in log terms.
     *
     * Medians of halves rather than max/min or a spread statistic, because both
     * of those are decided by a single row.  Two earlier attempts failed on
     * exactly that: comparing CV(sigma) with CV(sigma/fs) let one near-zero
     * step OPEN the gate (and that same row then tripped the median test), and
     * counting rises admitted noise 6 times in 32 -- with 6 ranges there are
     * only 5 steps, so "4 of 5 rose" happens by chance about 19% of the time,
     * which is why imu.fs.gyro still fired on 2 of 10 bench rungs after it.
     *
     * Measured on the reference ISM330DHCX at 104.125 Hz: 0.0061, 0.0028,
     * 0.0060, 0.0053, 0.0039, 0.0040 rad/s over 125..4000 dps.  Half-medians
     * 0.0060 against 0.0040 -- a ratio of 0.67 where the quantisation model
     * needs 2.83.  Not close, and not a coin flip.
     */
    int half = m / 2;
    double lo_s[IMT_MAX_FS_ROWS], hi_s[IMT_MAX_FS_ROWS];
    double lo_f[IMT_MAX_FS_ROWS], hi_f[IMT_MAX_FS_ROWS];
    for (int i = 0; i < half; i++)      { lo_s[i] = sig[i];        lo_f[i] = fs[i]; }
    for (int i = half; i < m; i++)      { hi_s[i - half] = sig[i]; hi_f[i - half] = fs[i]; }

    double lo_sig = med_of(lo_s, half), hi_sig = med_of(hi_s, m - half);
    double lo_fs  = med_of(lo_f, half), hi_fs  = med_of(hi_f, m - half);
    if (lo_sig <= 0 || lo_fs <= 0 || hi_fs <= lo_fs) return false;

    /* Half the span in log terms: sqrt of the full-scale ratio. */
    return (hi_sig / lo_sig) >= sqrt(hi_fs / lo_fs);
}

int imt_fs_grade_median(imt_fs_row_t *rows, int n, double frac)
{
    if (n < 3) return 0;                 /* no median worth the name */

    double mid[IMT_MAX_FS_ROWS];
    int nm = 0;
    for (int i = 0; i < n && nm < IMT_MAX_FS_ROWS; i++) {
        double m = (rows[i].sigma[0] + rows[i].sigma[1] + rows[i].sigma[2]) / 3.0;
        if (m > 0) mid[nm++] = m;
    }
    if (nm < 3) return 0;

    for (int i = 1; i < nm; i++) {       /* insertion sort; nm <= IMT_MAX_FS_ROWS */
        double v = mid[i];
        int j = i - 1;
        while (j >= 0 && mid[j] > v) { mid[j + 1] = mid[j]; j--; }
        mid[j + 1] = v;
    }
    double med = mid[nm / 2];
    if (med <= 0) return 0;

    int bad = 0;
    for (int i = 0; i < n; i++) {
        double m = (rows[i].sigma[0] + rows[i].sigma[1] + rows[i].sigma[2]) / 3.0;
        if (m > 0 && m < frac * med) { rows[i].status = IMT_WARN; bad++; }
    }
    return bad;
}

int imt_ts_collect_burst(imt_ts_acc_t *a, const uint32_t *ts, int n,
                         double *out, int cap, int have)
{
    imt_ts_acc_seam(a);
    for (int i = 0; i < n; i++) {
        uint32_t d = imt_ts_acc_step(a, ts[i]);
        /* i == 0 is the seam: see the header for why it cannot inform a
         * per-sample tick period. */
        if (d && i > 0 && have < cap) out[have++] = d;
    }
    return have;
}

void imt_degauss_split(const double vS[3], const double vR[3],
                       double field[3], double offset[3])
{
    for (int k = 0; k < 3; k++) {
        if (field)  field[k]  = (vS[k] - vR[k]) / 2.0;
        if (offset) offset[k] = (vS[k] + vR[k]) / 2.0;
    }
}

void imt_decide_verdict(imt_report_t *r)
{
    const char *blocker = NULL;

    bool complete = !r->aborted
                 && r->n_fail == 0
                 && r->phases_requested == IMT_PHASE_ALL
                 && r->phases_run == IMT_PHASE_ALL
                 && !r->is_sim
                 && !r->daemon_was_running;

    if (complete) {
        for (const char **p = imt_required_imu; *p && !blocker; p++) {
            const imt_check_t *c = imt_find(r, *p);
            if (!c || c->status == IMT_SKIP || c->status == IMT_FAIL) blocker = *p;
        }
        if (r->have_mag)
            for (const char **p = imt_required_mag; *p && !blocker; p++) {
                const imt_check_t *c = imt_find(r, *p);
                if (!c || c->status == IMT_SKIP || c->status == IMT_FAIL) blocker = *p;
            }
    }

    r->recommend_clear_experimental = complete && !blocker;

    if (r->is_sim)
        snprintf(r->verdict, sizeof r->verdict,
                 "This run used the `sim` driver: it exercises imud-imutest "
                 "itself, not any hardware. No conclusion about a driver can "
                 "be drawn from it.");
    else if (r->aborted)
        snprintf(r->verdict, sizeof r->verdict,
                 "Run ABORTED before completing — the results below are partial.");
    else if (r->n_fail > 0)
        snprintf(r->verdict, sizeof r->verdict,
                 "%d check%s FAILED: `%s` is not ready to have its "
                 "`experimental` flag cleared.",
                 r->n_fail, r->n_fail == 1 ? "" : "s", r->imu_driver);
    else if (r->daemon_was_running)
        snprintf(r->verdict, sizeof r->verdict,
                 "No failures, but imud was running during this run and both "
                 "processes drain the same FIFO — the timing figures are not "
                 "trustworthy. Re-run with the daemon stopped.");
    else if (blocker)
        snprintf(r->verdict, sizeof r->verdict,
                 "No failures, but `%s` did not run — see its row for why — so "
                 "this report does not yet support clearing `experimental` "
                 "for `%s`.",
                 blocker, r->imu_driver);
    else if (r->phases_requested != IMT_PHASE_ALL)
        snprintf(r->verdict, sizeof r->verdict,
                 "No failures, but only some phases were selected. Run with "
                 "--all to produce a report that can clear `experimental`.");
    else {
        /*
         * Recommend clearing the flag only where it is actually set.  A report
         * that tells the reader to clear a flag already clear reads as stale
         * advice and undercuts the rest of it — and this report is written to
         * be pasted into an issue, where that costs a round trip.
         */
        bool imu_exp = r->imu_experimental;
        bool mag_exp = r->have_mag && r->mag_experimental;
        if (imu_exp || mag_exp)
            snprintf(r->verdict, sizeof r->verdict,
                     "All required checks passed. RECOMMEND clearing "
                     "`experimental` for `%s`%s%s.",
                     imu_exp ? r->imu_driver : r->mag_driver,
                     (imu_exp && mag_exp) ? " and " : "",
                     (imu_exp && mag_exp) ? r->mag_driver : "");
        else
            snprintf(r->verdict, sizeof r->verdict,
                     "All required checks passed. `%s`%s%s already "
                     "%s `experimental` clear; this run re-confirms it on "
                     "this hardware.",
                     r->imu_driver,
                     r->have_mag ? " and " : "",
                     r->have_mag ? r->mag_driver : "",
                     r->have_mag ? "have" : "has");
    }
}

/* ── Entry points ─────────────────────────────────────────────────────────── */

static void fill_environment(imt_report_t *r, const imud_config_t *cfg)
{
    snprintf(r->imud_version, sizeof r->imud_version, "%s", IMUD_VERSION_STR);

    struct utsname u;
    if (uname(&u) == 0) {
        snprintf(r->sysname, sizeof r->sysname, "%s", u.sysname);
        snprintf(r->release, sizeof r->release, "%s", u.release);
        snprintf(r->machine, sizeof r->machine, "%s", u.machine);
        snprintf(r->hostname, sizeof r->hostname, "%s", u.nodename);
    }
#ifdef GPIOD_V2
    r->gpiod_v2 = true;
#endif

    time_t now = time(NULL);
    struct tm tm;
    gmtime_r(&now, &tm);
    strftime(r->started_utc, sizeof r->started_utc, "%Y-%m-%dT%H:%M:%SZ", &tm);

    snprintf(r->i2c_bus,   sizeof r->i2c_bus,   "%s", cfg->i2c_bus);
    snprintf(r->gpio_chip, sizeof r->gpio_chip, "%s", cfg->gpio_chip);
}

int imt_run_ops(const imud_bus_t *ibus, const imud_bus_t *mbus,
                const imu_ops_t *imu, const mag_ops_t *mag,
                const imud_config_t *cfg, const imt_opts_t *opts,
                imt_report_t *r, char *errbuf, size_t errbufsz)
{
    if (!imu) {
        snprintf(errbuf, errbufsz, "no IMU driver supplied");
        return -1;
    }

    g_abort = 0;
    double t_start = now_s();

    /* -1 = "the check did not run", which a zeroed report cannot say: 0 edges
     * is a real and damning measurement, so it must not be the default. */
    r->raw.mag_drdy_edges = -1;

    fill_environment(r, cfg);

    /* Subject under test */
    snprintf(r->imu_driver, sizeof r->imu_driver, "%s", imu->name);
    r->imu_addr         = cfg->imu_addr;
    r->imu_int_gpio     = cfg->imu_int_gpio;
    r->imu_experimental = imu->experimental;
    r->imu_has_fifo     = imu->has_fifo;
    r->imu_has_hw_ts    = imu->has_hw_timestamp;
    r->imu_ts_tick_ns   = imu->ts_tick_ns;
    memcpy(r->imu_odr_tab,   imu->supported_odr_mhz,   sizeof r->imu_odr_tab);
    memcpy(r->imu_accel_tab, imu->supported_accel_g,  sizeof r->imu_accel_tab);
    memcpy(r->imu_gyro_tab,  imu->supported_gyro_dps, sizeof r->imu_gyro_tab);
    r->is_sim = (strcmp(imu->name, "sim") == 0);

    r->have_mag = (mag != NULL);
    if (mag) {
        snprintf(r->mag_driver, sizeof r->mag_driver, "%s", mag->name);
        r->mag_addr              = cfg->mag_addr;
        r->mag_int_gpio          = cfg->mag_int_gpio;
        r->mag_experimental      = mag->experimental;
        r->mag_has_interrupt     = mag->has_interrupt;
        r->mag_has_set_reset     = mag->has_set_reset;
        r->mag_set_reset_nonnull = (mag->set_reset != NULL);
        r->mag_set_period_s      = cfg->mag_set_period_s;
        memcpy(r->mag_odr_tab, mag->supported_odr_mhz, sizeof r->mag_odr_tab);
    }

    /*
     * Resolve exactly as imud does (odr_actual_*, not nearest_odr), so this
     * tool programs the rate the daemon would program and measures against
     * that rate. Using nearest_odr here would have it check the hardware
     * against a rate the daemon never selects.
     */
    r->req_odr_mhz = cfg->imu_odr_mhz;
    r->eff_odr_mhz = odr_actual_imu(imu, cfg->imu_odr_mhz);
    r->accel_g    = cfg->imu_accel_g;
    r->gyro_dps   = cfg->imu_gyro_dps;
    r->fifo_wm    = cfg->imu_fifo_wm;
    if (mag) {
        r->mag_req_odr_mhz = cfg->mag_odr_mhz;
        r->mag_eff_odr_mhz = odr_actual_mag(mag, cfg->mag_odr_mhz);
    }
    r->phases_requested = opts->phases;

    imu_cfg_t icfg = {
        .odr_mhz  = r->eff_odr_mhz,
        .accel_g  = cfg->imu_accel_g,
        .gyro_dps = cfg->imu_gyro_dps,
        .fifo_wm  = cfg->imu_fifo_wm,
    };
    mag_cfg_t mcfg = {
        .odr_mhz      = mag ? r->mag_eff_odr_mhz : 0,
        .set_period_s = 0.0f,   /* the SET pulse is exercised explicitly */
    };

    {
        char rb[56], qb[16], eb2[16];
        add_check(r, "imu.odr.rounding",
                  "Requested ODR resolves to a programmable rate", IMT_INFO,
                  /* Milli-Hz in the field, Hz on the page. */
                  fmtbuf(rb, sizeof rb, "%s -> %s Hz",
                         MHZ_STR(qb, r->req_odr_mhz),
                         MHZ_STR(eb2, r->eff_odr_mhz)),
                  "-",
                  r->req_odr_mhz == r->eff_odr_mhz
                  ? "the configured rate is on this chip's grid"
                  : "the configured rate is not reachable; the driver reports "
                    "it will program this rate instead, and imud tunes the "
                    "filter for it");
        if (mag)
            add_check(r, "mag.odr.rounding",
                      "Requested mag ODR resolves to a programmable rate",
                      IMT_INFO,
                      fmtbuf(rb, sizeof rb, "%s -> %s Hz",
                             MHZ_STR(qb, r->mag_req_odr_mhz),
                             MHZ_STR(eb2, r->mag_eff_odr_mhz)),
                      "-",
                      r->mag_req_odr_mhz == r->mag_eff_odr_mhz
                      ? "the configured rate is on this chip's grid"
                      : "the configured rate is not reachable; the driver "
                        "reports it will program this rate instead, and imud "
                        "sizes the mag noise variance for it");
    }

    bool mag_ok = false;
    if (check_bringup(r, opts, ibus, mbus, imu, mag, cfg, &icfg, &mcfg, &mag_ok) < 0) {
        r->wall_duration_s = now_s() - t_start;
        imt_decide_verdict(r);
        return 0;   /* the checks carry the verdict */
    }

    drain_ctx_t d;
    drain_init(&d, imu, ibus, cfg);
    /* Resolved after bringup, so the header records what the checks below
     * actually graded against rather than the descriptor's typical. */
    r->imu_ts_tick_actual_ns = d.tick_ns;

    if (opts->phases & IMT_PHASE_PASSIVE) {
        /* The check functions take Hz as a double; the report field is
         * milli-Hz.  Convert once, here, rather than at every use inside. */
        const double eff_hz = (double)r->eff_odr_mhz * 1e-3;
        check_odr_seq_ts(r, opts, &d, imu, eff_hz);
        check_error_contract(r, &d);
        /* seq before the deliberate overflow: an overflow is a legitimate gap */
        check_fifo(r, opts, &d, imu, eff_hz, cfg->imu_fifo_wm);
        check_rest(r, opts, &d);
        check_drdy(r, opts, &d, cfg, eff_hz, cfg->imu_fifo_wm);
        if (mag_ok) {
            const double mag_eff_hz = (double)r->mag_eff_odr_mhz * 1e-3;
            check_mag_passive(r, opts, mag, mbus, mag_eff_hz);
            /* After the passive sweep: it needs mag.rate as its reference, and
             * it re-inits the part twice. */
            check_mag_drdy(r, opts, mag, mbus, cfg, mag_eff_hz);
        }
        /* last: every init() in the sweep resets the driver's seq counter */
        check_fs_sweep(r, opts, &d, imu, ibus, &icfg);
        r->phases_run |= IMT_PHASE_PASSIVE;
    }

    /*
     * The sim driver has a real FIFO, seq and chip_ts, so the passive checks
     * above are a genuine smoke test of this tool.  It has no physical
     * orientation, so the guided phases would be meaningless.
     */
    if (r->is_sim) {
        if (opts->phases & IMT_PHASE_FACES)
            skip_check(r, "faces.frame", "Board frame matches NED",
                       "the sim driver has no physical orientation");
        if (opts->phases & IMT_PHASE_GYRO)
            skip_check(r, "gyro.x.sign", "Gyro rotation",
                       "the sim driver has no physical orientation");
        if (opts->phases & IMT_PHASE_SPIN)
            skip_check(r, "spin.frame_agreement", "Mag frame agrees with the gyro",
                       "the sim driver has no physical orientation");
    } else {
        if ((opts->phases & IMT_PHASE_FACES) && !g_abort) {
            phase_faces(r, opts, &d);
            r->phases_run |= IMT_PHASE_FACES;
        }
        if ((opts->phases & IMT_PHASE_GYRO) && !g_abort && !r->aborted) {
            phase_gyro(r, opts, &d, imu, r->eff_odr_mhz);
            r->phases_run |= IMT_PHASE_GYRO;
        }
        if ((opts->phases & IMT_PHASE_SPIN) && !g_abort && !r->aborted) {
            if (mag_ok) {
                phase_spin(r, opts, &d, mag, mbus,
                           cfg, imu, r->eff_odr_mhz);
                r->phases_run |= IMT_PHASE_SPIN;
            } else {
                skip_check(r, "spin.frame_agreement",
                           "Mag frame agrees with the gyro",
                           "no working magnetometer in this run");
            }
        }
    }

    if (g_abort) r->aborted = true;
    r->wall_duration_s = now_s() - t_start;
    imt_decide_verdict(r);
    return 0;
}
