/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu.c — reader threads, ring buffers, and fusion thread (§3)
 *
 * Thread model:
 *   ism_reader  wakes on GPIO FIFO-watermark edge (fallback: fifo_wm +
 *               int_grace sample periods), drains
 *               IMU FIFO into imu_ring, updates wall-clock anchor.
 *   mag_reader  wakes on GPIO measurement-done edge (20 ms fallback), reads
 *               magnetometer, applies hard/soft-iron cal, issues periodic SET.
 *   fusion      consumes imu_ring (blocking), injects mag updates, runs MEKF,
 *               writes shared_state for output threads to snapshot.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdatomic.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <gpiod.h>

/* libgpiod v1 exposes struct gpiod_line *; v2 uses struct gpiod_line_request *. */
#ifdef GPIOD_V2
typedef struct gpiod_line_request gpio_line_h;
#else
typedef struct gpiod_line         gpio_line_h;
#endif

/* CLOCK_TAI is Linux ≥ 3.10; fall back to CLOCK_REALTIME on other platforms. */
#ifndef CLOCK_TAI
#define CLOCK_TAI CLOCK_REALTIME
#endif

#include "capture.h"
#include "imu.h"
#include "imu_gpio.h"
#include "imu_math.h"   /* ts_anchor_t + pure helpers factored out of this file */
#include "version.h"
#include "ring.h"
#include "drivers.h"
#include "fusion.h"
#include "log.h"

/* Engine-vibration detector: EMA time constant (s) and the ratio of the
 * assert threshold at which the detector releases (Schmitt hysteresis). */
#define VIB_TAU_S       1.0f
#define VIB_HYST_RATIO  0.7f

/* ── Full context (opaque to callers via imu.h) ─────────────────────────── */

struct imu_ctx {
    imud_config_t    cfg;              /* copy at open; hot-reload fields updated via imu_ctx_update_config */
    imud_cal_t       cal;              /* copy at open, read-only afterwards */

    /* One handle per sensor, not one bus shared by both: the two can sit on
     * different nodes, and each driver addresses only its own. */
    imud_bus_t       imu_bus;
    imud_bus_t       mag_bus;
    const imu_ops_t *imu_ops;
    const mag_ops_t *mag_ops;
    imu_cfg_t        imu_hw_cfg;
    mag_cfg_t        mag_hw_cfg;
    /* Rates the drivers said they would really program for the configured
     * requests (odr_actual_imu / odr_actual_mag). These, not the raw config
     * values, are what the drivers are handed and what the filter is tuned
     * for — see the resolution comment in imu_ctx_open. */
    int              actual_odr_mhz;      /* milli-Hz */
    int              actual_mag_odr_mhz;  /* milli-Hz */
    /*
     * Chip-timer period actually in force: the driver's declared ts_tick_ns,
     * or what ts_tick_ns_actual() said this individual part's timer runs at.
     * Resolved once in imu_ctx_open after init() and read-only afterwards, so
     * every consumer of the tick agrees.  0 when the part has no timer.
     */
    uint32_t         ts_tick_ns;

    struct gpiod_chip  *gpio_chip;
    gpio_line_h    *imu_line;   /* GPIO for IMU FIFO watermark interrupt */
    gpio_line_h    *mag_line;   /* GPIO for magnetometer measurement-done interrupt */

    imu_ring_t       imu_ring;
    mag_ring_t       mag_ring;
    shared_state_t   shared;
    ts_anchor_t      anchor;

    /* Guards every post-open write to `cfg` (hot-reload params + live
     * position values from imu_ctx_update_config / set_declination /
     * set_mag_ref / set_speed). Reader/fusion threads take a snapshot copy
     * under this lock (cfg_snapshot) rather than reading cfg fields
     * directly — plain shared reads/writes are C11 data races even where a
     * single word is "naturally atomic" on the target CPU. */
    pthread_mutex_t  live_lock;

    /* Black-box capture ([capture] enabled): reader threads tap raw samples
     * into cap_ring right after the driver read() (pre-mount, pre-cal);
     * capture_thread drains it into rotating .imucap files.  Status fields
     * are the writer's snapshot for imud-status, under shared.lock. */
    cap_ring_t       cap_ring;
    char             cap_path[320];    /* shared.lock */
    uint64_t         cap_bytes;        /* shared.lock */
    uint64_t         cap_drops;        /* shared.lock */
    bool             cap_active;       /* shared.lock */

    /* IMU samples for the binary output packet, written by fusion_thread.
     * latest_imu: calibrated accel + bias-corrected gyro (fused output).
     * raw_imu:    pre-calibration accel (accel_raw[]) + pre-bias gyro[].
     * Protected by shared.lock (same lock as shared.state). */
    imu_sample_t     latest_imu;
    imu_sample_t     raw_imu;

    /* Stats — each field written by exactly one thread. _Atomic so the
     * cross-thread reads in imu_get_stats() are tear-free even on a
     * 32-bit build (free on 64-bit: plain loads/stores). */
    _Atomic uint64_t imu_sample_count;
    _Atomic uint64_t mag_sample_count;
    _Atomic uint64_t fifo_overflow_count;
    _Atomic uint64_t imu_error_count;
    _Atomic uint64_t mag_error_count;

    /*
     * Sample latency.  The histograms are touched ONLY by fusion_thread, so
     * they need no lock; what crosses threads is the derived p50/p99/max,
     * published as _Atomic like every other stat.  Deriving costs a 20-bucket
     * walk and is done about once a second rather than per sample.
     */
    lat_hist_t       lat_fifo;      /* fusion_thread only */
    lat_hist_t       lat_pipe;      /* fusion_thread only */
    _Atomic uint64_t lat_fifo_p50, lat_fifo_p99, lat_fifo_max;
    _Atomic uint64_t lat_pipe_p50, lat_pipe_p99, lat_pipe_max;

    /*
     * Drain accounting.  Written by ism_reader only, read by whoever calls
     * imu_get_stats(), so _Atomic directly rather than the accumulate-then-
     * publish shape the histograms above use: these are four increments per
     * DRAIN (~100/s), not per sample, so the cost is nil and the simpler form
     * cannot drift out of step with a publisher.
     */
    _Atomic uint64_t drains_edge, drains_timeout;
    _Atomic uint64_t drain_samples, drain_max;

    float            vib_ema;       /* EMA of (|a|-g)² for engine detection, ism_reader only */

    /* Cross-thread signal flags: _Atomic int rather than volatile so the
     * single-writer/single-reader handshakes are C11-clean, not just
     * "works on ARM". Syntax of all existing reads/writes is unchanged.
     * `stop` became _Atomic with the 1.5 TSan job (the ring API takes
     * _Atomic int * now) — plain volatile is a C11 data race. */
    _Atomic int      stop;
    _Atomic int      settled;      /* release-written by fusion_thread at Phase 4 entry;
                                    * acquire-read by output/health threads to suppress
                                    * output until settle + bias estimation + alignment done */
    _Atomic int      reconfigure;  /* set by main on SIGHUP; cleared by fusion_thread */
    _Atomic int      mref_update;  /* set by position thread; cleared by fusion_thread */
    _Atomic int      mag_set_flag; /* set by mag_reader after SET pulse; cleared by fusion_thread */
    _Atomic int      engine_on;    /* set by ism_reader when vibration EMA exceeds threshold */
};

/* ── Calibration, timestamp, and utility helpers ─────────────────────────────
 * apply_imu_cal / apply_mag_cal / ts_ns / anchor_update / chip_to_wall /
 * nearest_odr / apply_mount_rot_if_set moved to src/imu_math.c (imu_math.h) so
 * they can be unit tested off-thread; imu.c calls them unchanged. */

/* ── GPIO helpers ────────────────────────────────────────────────────────── */

/*
 * wait_gpio_edge — wait up to timeout_ms ms for a rising-edge event.
 * Drains the event so the next call sees a fresh edge.
 * Returns 1 on event, 0 on timeout, -1 on error.
 */
static int wait_gpio_edge(gpio_line_h *line, long timeout_ms)
{
#ifdef GPIOD_V2
    struct gpiod_edge_event_buffer *evbuf = gpiod_edge_event_buffer_new(1);
    if (!evbuf) return -1;
    int r = gpiod_line_request_wait_edge_events(line,
                (int64_t)timeout_ms * 1000000LL);
    if (r == 1)
        gpiod_line_request_read_edge_events(line, evbuf, 1);
    gpiod_edge_event_buffer_free(evbuf);
    return r;
#else
    struct timespec ts = {
        .tv_sec  = timeout_ms / 1000,
        .tv_nsec = (timeout_ms % 1000) * 1000000L,
    };
    int r = gpiod_line_event_wait(line, &ts);
    if (r == 1) {
        struct gpiod_line_event ev;
        gpiod_line_event_read(line, &ev);
    }
    return r;
#endif
}

/*
 * open_gpio_line — request one GPIO line for rising-edge detection.
 * Returns an opaque handle on success, NULL on error.
 */
static gpio_line_h *open_gpio_line_as(struct gpiod_chip *chip,
                                      unsigned int offset,
                                      const char *consumer)
{
#ifdef GPIOD_V2
    struct gpiod_line_settings  *ls = gpiod_line_settings_new();
    struct gpiod_line_config    *lc = gpiod_line_config_new();
    struct gpiod_request_config *rc = gpiod_request_config_new();
    if (!ls || !lc || !rc) goto fail;
    gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(ls, GPIOD_LINE_EDGE_RISING);
    gpiod_line_config_add_line_settings(lc, &offset, 1, ls);
    gpiod_request_config_set_consumer(rc, consumer);
    gpio_line_h *req = gpiod_chip_request_lines(chip, rc, lc);
    gpiod_request_config_free(rc);
    gpiod_line_config_free(lc);
    gpiod_line_settings_free(ls);
    return req;
fail:
    if (rc) gpiod_request_config_free(rc);
    if (lc) gpiod_line_config_free(lc);
    if (ls) gpiod_line_settings_free(ls);
    return NULL;
#else
    gpio_line_h *line = gpiod_chip_get_line(chip, offset);
    if (!line) return NULL;
    if (gpiod_line_request_rising_edge_events(line, consumer) < 0)
        return NULL;
    return line;
#endif
}

static void release_gpio_line(gpio_line_h *line)
{
#ifdef GPIOD_V2
    gpiod_line_request_release(line);
#else
    gpiod_line_release(line);
#endif
}

/* ── The same edge wait, exposed — see include/imu_gpio.h ────────────────── */

/*
 * imud-imutest waits on an interrupt by calling these, rather than carrying a
 * second copy of the libgpiod v1/v2 split.  It used to carry one, and every
 * way the copy differed from the daemon showed up as a defect reported against
 * the driver.
 *
 * The handle owns its chip because libgpiod v1 hands back a line that BORROWS
 * the chip -- closing the chip there is a use-after-free -- while v2's request
 * owns what it needs.  Keeping both in one allocation makes that difference
 * invisible to a caller and impossible to get wrong at the call site.
 */
struct imu_gpio_line {
    struct gpiod_chip *chip;
    gpio_line_h       *line;
};

imu_gpio_line_t *imu_gpio_open(const char *chip_name, unsigned int offset,
                               const char *consumer)
{
    char path[80];
    snprintf(path, sizeof path, "/dev/%s", chip_name);

    imu_gpio_line_t *h = calloc(1, sizeof *h);
    if (!h) return NULL;

    h->chip = gpiod_chip_open(path);
    if (!h->chip) { int e = errno; free(h); errno = e; return NULL; }

    h->line = open_gpio_line_as(h->chip, offset, consumer);
    if (!h->line) {
        int e = errno;
        gpiod_chip_close(h->chip);
        free(h);
        errno = e;
        return NULL;
    }
    return h;
}

int imu_gpio_wait_edge(imu_gpio_line_t *h, long timeout_ms)
{
    return h ? wait_gpio_edge(h->line, timeout_ms) : -1;
}

void imu_gpio_close(imu_gpio_line_t *h)
{
    if (!h) return;
    release_gpio_line(h->line);
    gpiod_chip_close(h->chip);
    free(h);
}

/* ── Config snapshot ─────────────────────────────────────────────────────── */

/*
 * Copy the live config under live_lock.  All worker threads read config
 * through a local snapshot taken by this helper — never ctx->cfg directly —
 * so a concurrent imu_ctx_update_config / set_* write is never a data race.
 * The struct is ~4 KB; the copy is well under a microsecond and writers are
 * rare (SIGHUP, GPS fixes), so there is no contention on the hot path.
 */
static void cfg_snapshot(imu_ctx_t *ctx, imud_config_t *out)
{
    pthread_mutex_lock(&ctx->live_lock);
    *out = ctx->cfg;
    pthread_mutex_unlock(&ctx->live_lock);
}

/* ── ism_reader_thread ───────────────────────────────────────────────────── */

void *ism_reader_thread(void *arg)
{
    imu_ctx_t *ctx = arg;
    imu_sample_t buf[IMU_DRAIN_MAX];
    int n;
    int consec_errors = 0;
    int reset_failures = 0;
    bool anchor_valid = false;
    struct timespec anchor_last = {0, 0};
    /* Latches once ts_anchor_t has a real tick period.  Drives the re-anchor
     * interval below and gates the one-shot log of what was measured. */
    bool tick_measured = false;
    imud_config_t cfg;
    /*
     * Wait sized to what the line is waiting FOR: fifo_wm sample-sets plus a
     * grace, both in samples.  A flat 10 ms drained before the watermark could
     * assert at nine of the ISM330DHCX's ten rates, which does not merely
     * ignore the interrupt -- a LEVEL watermark deasserts when a drain empties
     * the FIFO, so an early fallback holds it below the threshold and the line
     * never asserts at all.  Measured as drains=0/1261 e/t on a working line.
     *
     * Recomputed after every cfg_snapshot() so a SIGHUP that changes fifo_wm,
     * int_grace or poll_ms takes effect without a restart.
     */
    cfg_snapshot(ctx, &cfg);
    long imu_wait_ms = imu_int_fallback_ms(ctx->actual_odr_mhz,
                                           cfg.imu_fifo_wm, cfg.imu_int_grace);
    /*
     * Interrupt-less installs poll.  That cadence must be adaptive too: a flat
     * interval under-polls at high ODR -- the FIFO overflows and the effective
     * rate drops below the configured one -- and burns reads at low ODR.  So
     * it defaults to the same batch period the interrupt path waits for.
     * poll_ms > 0 forces a fixed cadence for anyone who wants one; it is
     * IGNORED entirely when an interrupt line exists.
     */
    long imu_poll_ms = cfg.imu_poll_ms > 0 ? cfg.imu_poll_ms : imu_wait_ms;

    while (!ctx->stop) {
        if (ctx->imu_line) {
            int gr = wait_gpio_edge(ctx->imu_line, imu_wait_ms);
            if (gr < 0) {
                if (ctx->stop) break;
                LOG_E("[ism_reader] GPIO error: %s\n", strerror(errno));
                usleep(10000);
                continue;
            }
            /* gr == 0: the line was late — fall through to read anyway */
            atomic_fetch_add(gr > 0 ? &ctx->drains_edge : &ctx->drains_timeout, 1);
        } else {
            struct timespec t = { .tv_sec = imu_poll_ms / 1000,
                                  .tv_nsec = (imu_poll_ms % 1000) * 1000000L };
            nanosleep(&t, NULL);
            if (ctx->stop) break;
            /* Every drain is a timer drain here, by construction. */
            atomic_fetch_add(&ctx->drains_timeout, 1);
        }


        cfg_snapshot(ctx, &cfg);
        imu_wait_ms = imu_int_fallback_ms(ctx->actual_odr_mhz,
                                          cfg.imu_fifo_wm, cfg.imu_int_grace);
        imu_poll_ms = cfg.imu_poll_ms > 0 ? cfg.imu_poll_ms : imu_wait_ms;

        struct timespec t_before, t_after, t_tai;
        clock_gettime(CLOCK_REALTIME, &t_before);

        int rc = ctx->imu_ops->read(&ctx->imu_bus, buf, IMU_DRAIN_MAX, &n);

        clock_gettime(CLOCK_REALTIME, &t_after);
        clock_gettime(CLOCK_TAI,      &t_tai);

        if (rc < 0) {
            ctx->imu_error_count++;
            if (++consec_errors >= 10) {
                LOG_E("[ism_reader] 10 consecutive errors — resetting chip\n");
                int rok = (ctx->imu_ops->reset(&ctx->imu_bus) == 0)
                       && (ctx->imu_ops->init (&ctx->imu_bus,
                                               &ctx->imu_hw_cfg) == 0);
                consec_errors = 0;
                if (!rok) {
                    LOG_E("[ism_reader] reset failed (%d/3)\n",
                            ++reset_failures);
                    if (reset_failures >= 3) {
                        LOG_E("[ism_reader] 3 reset failures — "
                              "signalling shutdown\n");
                        /*
                         * PROCESS-directed, deliberately.  raise() is
                         * thread-directed (POSIX defines it as
                         * pthread_kill(pthread_self(), sig)), and main.c blocks
                         * SIGTERM before creating any thread, so this thread
                         * inherits the block: a raise() here would leave the
                         * signal pending on THIS thread, where main's sigwait
                         * cannot see it, and the break below would discard it
                         * with the thread.  The daemon would then keep
                         * reporting active with a dead reader.
                         */
                        kill(getpid(), SIGTERM);
                        break;
                    }
                } else {
                    reset_failures = 0;
                    ctx->vib_ema  = 0.0f;  /* clear stale EMA after chip recovery */
                }
            }
            continue;
        }
        consec_errors = 0;

        /*
         * Burst depth, recorded before the n == 0 early-out so an empty drain
         * still counts as a drain.  Dropping those would flatter the mean: a
         * timer that fires before the part has produced anything is exactly
         * the case worth seeing, and it is invisible in a per-sample total.
         */
        atomic_fetch_add(&ctx->drain_samples, (uint64_t)n);
        if ((uint64_t)n > atomic_load(&ctx->drain_max))
            atomic_store(&ctx->drain_max, (uint64_t)n);

        if (n == 0) continue;

        /* Black-box tap: raw samples exactly as the driver delivered them
         * (pre-mount, pre-cal) so replay traverses the identical pipeline.
         * One delivery timestamp per burst; per-sample time is chip_ts. */
        if (cfg.capture_enabled) {
            struct timespec tm;
            clock_gettime(CLOCK_MONOTONIC, &tm);
            uint64_t mono = ts_ns(&tm);
            for (int i = 0; i < n; i++)
                cap_ring_push_imu(&ctx->cap_ring, &buf[i], mono);
        }

        /* When this burst's I2C read finished — the boundary between FIFO
         * residence (the operator's fifo_wm/odr) and the daemon's own
         * pipeline.  t_after is already CLOCK_REALTIME immediately after the
         * read, so this costs a store per sample and no extra syscall. */
        const uint64_t read_done = ts_ns(&t_after);
        for (int i = 0; i < n; i++) buf[i].read_done_ns = read_done;

        /* Apply mount rotation (board->body), then accel calibration */
        for (int i = 0; i < n; i++) {
            apply_mount_rot_if_set(&cfg, buf[i].accel);
            apply_mount_rot_if_set(&cfg, buf[i].gyro);
            buf[i].accel_raw[0] = buf[i].accel[0];
            buf[i].accel_raw[1] = buf[i].accel[1];
            buf[i].accel_raw[2] = buf[i].accel[2];
            apply_imu_cal(&ctx->cal, &buf[i]);
        }

        /* Update timestamp anchor at startup and every 60 s thereafter.  The
         * newest sample's chip_ts is paired with the host instant that sample
         * was taken at — see anchor_wall_ns, which owns that choice and the
         * reason it is not simply the middle of the read. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = anchor_valid
            ? (double)(now.tv_sec  - anchor_last.tv_sec)
              + (double)(now.tv_nsec - anchor_last.tv_nsec) * 1e-9
            : 1e9;

        /* Update anchor:
         *   - Always on first read.
         *   - Every 25 s until the tick period has been measured, 60 s after.
         *   - Every burst when no hw timestamp (chip_ts=0 always, so anchor
         *     IS the per-sample timestamp; must stay current).
         *
         * The two intervals do different jobs.  60 s tracks oscillator drift,
         * which is slow, and there is no reason to pay for it more often.  The
         * shorter one exists because chip_to_wall() has no measured period at
         * all until the SECOND anchor lands — anchor_update needs two, 20 s
         * apart (ANCHOR_MIN_INTERVAL_NS) — and until then it extrapolates from
         * the declared tick.  A part a percent or two off nominal drifts far
         * enough during that window to stop the sample-latency histogram
         * recording, which is how the 2026-08-11 bench got no latency data out
         * of six 40 s runs.  25 s clears the 20 s guard with margin, so the
         * measurement is exactly as trustworthy as it was, and costs one extra
         * anchor per daemon lifetime.
         *
         * ts_tick_ns_actual (see imu_ctx_open) shrinks the error inside that
         * window rather than the window itself; the two are complementary, and
         * this half is what covers parts with no trim register to read. */
        double want = tick_measured ? 60.0 : 25.0;
        bool do_anchor = !anchor_valid
                         || !ctx->imu_ops->has_hw_timestamp
                         || elapsed >= want;
        if (do_anchor) {
            uint64_t wall_at = anchor_wall_ns(ts_ns(&t_before), ts_ns(&t_after),
                                              ctx->imu_ops->has_hw_timestamp);
            /* `now` is CLOCK_MONOTONIC and is what the tick-period measurement
             * runs on; the REALTIME pair is only the absolute reference, and
             * an NTP step in it must not be read as the oscillator drifting. */
            anchor_update(&ctx->anchor, buf[n - 1].chip_ts, wall_at,
                          ts_ns(&t_tai), ts_ns(&now),
                          ctx->imu_ops->has_hw_timestamp
                              ? ctx->ts_tick_ns : 0);
            anchor_last  = now;
            anchor_valid = true;

            /*
             * Say so once when the part's timer is materially off its declared
             * period.  It is corrected from here on, but it is also the kind of
             * thing worth knowing about a board: imud-imutest's imu.chipts.wall
             * grades the same number, and a reading this far out is a hardware
             * characteristic rather than a driver defect.
             */
            if (!tick_measured && ctx->imu_ops->has_hw_timestamp) {
                double meas = anchor_measured_tick_ns(&ctx->anchor);
                double nom  = (double)ctx->ts_tick_ns;
                if (meas > 0.0 && nom > 0.0) {
                    tick_measured = true;
                    if (fabs(meas - nom) / nom > 0.01)
                        LOG_I("[imu] chip timer measured at %.1f ns/tick "
                              "against the declared %.0f (%+.1f%%); using the "
                              "measured value for sample timestamps and dt\n",
                              meas, nom, (meas - nom) / nom * 100.0);
                }
            }
        }

        /* Engine vibration detection: exponential moving average of (|a|-g)².
         *
         * Alpha is derived from the resolved sample rate, NOT hardcoded: the
         * EMA is stepped once per SAMPLE, so a fixed alpha made the time
         * constant scale with FIFO burst depth (a deeper burst = more steps
         * per read = a faster EMA) rather than being the ~1 s the detector
         * wants. 1/(τ·odr) gives τ = VIB_TAU_S regardless of how the driver
         * happens to batch. */
        if (cfg.engine_vibration_g2 > 0.0 && n > 0) {
            const float alpha =
            1.0f / (VIB_TAU_S * (float)ctx->actual_odr_mhz * 1e-3f);
            const float g = 9.80665f;
            for (int i = 0; i < n; i++) {
                float a = sqrtf(buf[i].accel[0]*buf[i].accel[0]
                              + buf[i].accel[1]*buf[i].accel[1]
                              + buf[i].accel[2]*buf[i].accel[2]);
                float dev = a - g;
                ctx->vib_ema += alpha * (dev*dev - ctx->vib_ema);
            }
            /* Schmitt trigger: assert above the threshold, clear only once the
             * EMA falls to VIB_HYST_RATIO of it. A single threshold chattered
             * at the boundary, and each toggle steps Ra_scale, rewrites the
             * skip window, emits a log line and flips FLAG_ENGINE_ON on the
             * wire — all externally visible. */
            float on_thresh  = (float)cfg.engine_vibration_g2;
            float off_thresh = on_thresh * VIB_HYST_RATIO;
            if (ctx->engine_on) {
                if (ctx->vib_ema < off_thresh) ctx->engine_on = false;
            } else {
                if (ctx->vib_ema > on_thresh)  ctx->engine_on = true;
            }
        }

        /* Track hardware FIFO overflow (rc == 1) and software ring overflow. */
        if (rc == 1) ctx->fifo_overflow_count++;
        int dropped = imu_ring_push(&ctx->imu_ring, buf, n);
        ctx->fifo_overflow_count += (uint64_t)dropped;
        ctx->imu_sample_count    += (uint64_t)n;
    }

    return NULL;
}

/* ── mag_reader_thread ───────────────────────────────────────────────────── */

void *mag_reader_thread(void *arg)
{
    imu_ctx_t *ctx = arg;
    mag_sample_t s;
    int consec_errors = 0;
    int reset_failures = 0;
    struct timespec last_set;
    imud_config_t cfg;
    clock_gettime(CLOCK_MONOTONIC, &last_set);

    /*
     * Missed-interrupt recovery, sized to the rate.
     *
     * A latched data-ready is re-armed only by the acknowledge that read()
     * performs, so it yields exactly one rising edge per acknowledge -- and a
     * single missed edge leaves the line asserted for ever with nothing to
     * clear it.  Measured on an MMC5983MA: after one acknowledge and then a
     * silent bus, 0 further edges in 3 s at 20 Hz and 1 at 100 Hz.  So this
     * timeout is not a safety net, it is the only way back.
     *
     * It was a flat 20 ms, which is fifty reads per conversion at 1 Hz and
     * twenty-four conversions of latency at 1204.  Two sample periods bounds a
     * missed edge to about one sample at any rate on the ladder.
     */
    cfg_snapshot(ctx, &cfg);
    /* depth 1: these magnetometers have no FIFO, so the line signals one
     * finished conversion.  Recomputed below on SIGHUP. */
    long mag_wait_ms = imu_int_fallback_ms(ctx->actual_mag_odr_mhz, 1,
                                           cfg.mag_int_grace);
    long mag_poll_ms = cfg.mag_poll_ms > 0 ? cfg.mag_poll_ms : mag_wait_ms;

    /* Stall watch: read() returning 1 is normal, but only for a while. */
    struct timespec last_good;
    clock_gettime(CLOCK_MONOTONIC, &last_good);
    bool stall_warned = false;

    while (!ctx->stop) {
        if (ctx->mag_line) {
            /* Interrupt-driven, with a rate-sized fallback: see above. */
            int gr = wait_gpio_edge(ctx->mag_line, mag_wait_ms);
            if (gr < 0) {
                if (ctx->stop) break;
                usleep(5000);
                continue;
            }
            /* gr == 0: the fallback expired — read anyway */
        } else {
            /* No interrupt pin, so no expected arrival to be late against:
             * the fixed poll cadence applies here and nowhere else.  The
             * driver read() polls DRDY internally, so no stale data. */
            struct timespec t = { .tv_sec = mag_poll_ms / 1000,
                                  .tv_nsec = (mag_poll_ms % 1000) * 1000000L };
            nanosleep(&t, NULL);
            if (ctx->stop) break;
        }

        cfg_snapshot(ctx, &cfg);
        mag_wait_ms = imu_int_fallback_ms(ctx->actual_mag_odr_mhz, 1,
                                          cfg.mag_int_grace);
        mag_poll_ms = cfg.mag_poll_ms > 0 ? cfg.mag_poll_ms : mag_wait_ms;

    /* Periodic SET/RESET (degauss) — skip this read cycle; wait for next edge. */
        if (cfg.mag_set_period_s > 0.0f && ctx->mag_ops->set_reset) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (double)(now.tv_sec  - last_set.tv_sec)
                           + (double)(now.tv_nsec - last_set.tv_nsec) * 1e-9;
            if (elapsed >= (double)cfg.mag_set_period_s) {
                ctx->mag_ops->set_reset(&ctx->mag_bus);
                ctx->mag_set_flag = 1;
                last_set = now;
                usleep(1000); /* 1 ms settling — no read this cycle */
                continue;
            }
        }

        int mrc = ctx->mag_ops->read(&ctx->mag_bus, &s);
        if (mrc > 0) {
            /*
             * No new measurement, so nothing is pushed -- re-fusing a stale
             * sample would be worse.  That is normal in isolation, because the
             * poll can outrun the part at any rate.  A magnetometer that has
             * STOPPED returns the same 1 for ever, though, and used to do so in
             * total silence: 800 s with no mag samples produced one line in the
             * log, from the fusion aligner at t+5 s, and nothing afterwards.
             * Say it once per outage, and say when it comes back.
             */
            if (!stall_warned) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                long ms = (long)((now.tv_sec  - last_good.tv_sec)  * 1000L
                               + (now.tv_nsec - last_good.tv_nsec) / 1000000L);
                if (ms >= imu_mag_stall_ms(ctx->actual_mag_odr_mhz)) {
                    LOG_W("[mag_reader] no magnetometer sample for %.1f s "
                          "(%.3f Hz configured) -- the part has stopped "
                          "delivering; heading will not update\n",
                          (double)ms / 1000.0,
                          (double)ctx->actual_mag_odr_mhz / 1000.0);
                    stall_warned = true;
                }
            }
            continue;
        }
        if (stall_warned) {
            LOG_I("[mag_reader] magnetometer delivering again\n");
            stall_warned = false;
        }
        clock_gettime(CLOCK_MONOTONIC, &last_good);
        if (mrc < 0) {
            ctx->mag_error_count++;
            if (++consec_errors >= 10) {
                LOG_E("[mag_reader] 10 consecutive errors — resetting chip\n");
                int rok = (ctx->mag_ops->reset(&ctx->mag_bus) == 0)
                       && (ctx->mag_ops->init (&ctx->mag_bus,
                                               &ctx->mag_hw_cfg) == 0);
                consec_errors = 0;
                if (!rok) {
                    LOG_E("[mag_reader] reset failed (%d/3)\n",
                            ++reset_failures);
                    if (reset_failures >= 3) {
                        LOG_E("[mag_reader] 3 reset failures — "
                              "signalling shutdown\n");
                        /* Process-directed — see the ism_reader twin above. */
                        kill(getpid(), SIGTERM);
                        break;
                    }
                } else {
                    reset_failures = 0;
                }
            }
            continue;
        }
        consec_errors = 0;

        /* Black-box tap: raw field as the driver delivered it (pre-mount,
         * pre-cal). */
        if (cfg.capture_enabled) {
            struct timespec tm;
            clock_gettime(CLOCK_MONOTONIC, &tm);
            cap_ring_push_mag(&ctx->cap_ring, &s, ts_ns(&tm));
        }

        /* Rotate mag into body frame, then apply mag calibration */
        apply_mount_rot_if_set(&cfg, s.field);
        s.field_raw[0] = s.field[0];
        s.field_raw[1] = s.field[1];
        s.field_raw[2] = s.field[2];
        apply_mag_cal(&ctx->cal, &s);
        s.valid = ctx->cal.has_mag;

        mag_ring_push(&ctx->mag_ring, &s);
        ctx->mag_sample_count++;
    }

    return NULL;
}

/* ── capture_thread ──────────────────────────────────────────────────────── */

/*
 * Black-box writer: drains the tap ring into rotating .imucap files under
 * cfg.capture_dir.  Never in the reader threads' path — they push into the
 * ring and move on; a slow SD card costs dropped capture records (counted),
 * never sensor latency.  A write error stops capture with an error log but
 * leaves the daemon running.
 */
void *capture_thread(void *arg)
{
    imu_ctx_t *ctx = arg;
    imud_config_t cfg;

    cfg_snapshot(ctx, &cfg);

    cap_rotator_t rot;
    if (cap_rot_open(&rot, cfg.capture_dir,
                     (uint32_t)(cfg.capture_max_mb    > 0 ? cfg.capture_max_mb    : 0),
                     (uint32_t)(cfg.capture_max_files > 0 ? cfg.capture_max_files : 0),
                     (uint32_t)((ctx->actual_odr_mhz + 500) / 1000),
                     (uint32_t)ctx->actual_odr_mhz,
                     ctx->imu_ops->name, ctx->mag_ops->name,
                     IMUD_VERSION_STR) != 0) {
        LOG_E("[capture] cannot open capture file in %s: %s\n",
              cfg.capture_dir, strerror(errno));
        return NULL;
    }
    LOG_I("[capture] recording to %s\n", cap_rot_path(&rot));

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t last_flush   = ts_ns(&now);
    uint64_t last_drops   = 0;
    uint64_t flush_ns     = (uint64_t)(cfg.capture_flush_s > 0
                                       ? cfg.capture_flush_s : 5)
                            * 1000000000ULL;
    char     prev_path[320];
    snprintf(prev_path, sizeof(prev_path), "%s", cap_rot_path(&rot));
    bool     failed = false;

    while (!ctx->stop && !failed) {
        cap_ring_rec_t recs[64];
        int n = cap_ring_pop(&ctx->cap_ring, recs, 64);
        if (n == 0) {
            struct timespec t = { .tv_sec = 0, .tv_nsec = 20 * 1000000L };
            nanosleep(&t, NULL);
        }
        for (int i = 0; i < n; i++) {
            if (cap_rot_write(&rot, &recs[i]) != 0) {
                LOG_E("[capture] write failed (%s) — capture stopped\n",
                      strerror(errno));
                failed = true;
                break;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t tnow = ts_ns(&now);
        if (!failed && tnow - last_flush >= flush_ns) {
            cap_rot_flush(&rot);
            last_flush = tnow;
        }

        if (strcmp(prev_path, cap_rot_path(&rot)) != 0) {
            snprintf(prev_path, sizeof(prev_path), "%s", cap_rot_path(&rot));
            LOG_I("[capture] rotated to %s\n", prev_path);
        }

        uint64_t drops = cap_ring_dropped(&ctx->cap_ring);
        if (drops != last_drops) {
            LOG_W("[capture] %llu records dropped (slow storage?)\n",
                  (unsigned long long)(drops - last_drops));
            last_drops = drops;
        }

        pthread_mutex_lock(&ctx->shared.lock);
        snprintf(ctx->cap_path, sizeof(ctx->cap_path), "%s", cap_rot_path(&rot));
        ctx->cap_bytes  = cap_rot_bytes(&rot);
        ctx->cap_drops  = drops;
        ctx->cap_active = !failed;
        pthread_mutex_unlock(&ctx->shared.lock);
    }

    /* Drain what the readers pushed before stop, then close cleanly.  The
     * inner break only leaves the current batch, so `failed` has to gate the
     * outer loop too — otherwise a write error keeps popping and retrying
     * against storage that has already said no. */
    if (!failed) {
        cap_ring_rec_t recs[64];
        int n;
        while (!failed && (n = cap_ring_pop(&ctx->cap_ring, recs, 64)) > 0)
            for (int i = 0; i < n; i++)
                if (cap_rot_write(&rot, &recs[i]) != 0) { failed = true; break; }
    }
    cap_rot_close(&rot);

    pthread_mutex_lock(&ctx->shared.lock);
    ctx->cap_active = false;
    pthread_mutex_unlock(&ctx->shared.lock);

    LOG_I("[capture] stopped\n");
    return NULL;
}

void imu_get_capture_status(imu_ctx_t *ctx, char *path, size_t path_sz,
                            uint64_t *bytes, uint64_t *drops, bool *active)
{
    pthread_mutex_lock(&ctx->shared.lock);
    if (path && path_sz) snprintf(path, path_sz, "%s", ctx->cap_path);
    if (bytes)  *bytes  = ctx->cap_bytes;
    if (drops)  *drops  = ctx->cap_drops;
    if (active) *active = ctx->cap_active;
    pthread_mutex_unlock(&ctx->shared.lock);
}

/* ── fusion_thread ───────────────────────────────────────────────────────── */

void *fusion_thread(void *arg)
{
    imu_ctx_t *ctx = arg;
    /* Rates the drivers actually programmed, not the cfg requests. */
    float odr_hz     = (float)ctx->actual_odr_mhz     * 1e-3f;
    float mag_odr_hz = (float)ctx->actual_mag_odr_mhz * 1e-3f;
    imud_config_t cfg;
    cfg_snapshot(ctx, &cfg);   /* startup phase reads this snapshot */

    /* ── Phase 0: startup settle — drain stale samples ─────────────────── */

    if (cfg.startup_settle_sec > 0.0) {
        int n_settle = (int)(cfg.startup_settle_sec * odr_hz);
        if (n_settle < 1) n_settle = 1;
        LOG_I("[fusion] settling %g s — discarding %d samples\n",
                cfg.startup_settle_sec, n_settle);
        imu_sample_t tmp;
        int discarded = 0;
        while (discarded < n_settle && !ctx->stop) {
            if (imu_ring_pop(&ctx->imu_ring, &tmp, &ctx->stop) == 0)
                discarded++;
        }
        /* Also flush the mag ring so alignment uses only post-settle mag data. */
        mag_sample_t mtmp;
        while (mag_ring_try_pop(&ctx->mag_ring, &mtmp) == 0) {}
    }

    if (ctx->stop) return NULL;

    /* ── Phase 1: gyro bias estimation ───────────────────────────────────── */

    float init_bias[3] = {0.0f, 0.0f, 0.0f};
    uint16_t cal_flags = 0;

    if (ctx->cal.has_accel) cal_flags |= FLAG_ACCEL_CAL;
    if (ctx->cal.has_mag)   cal_flags |= FLAG_MAG_CAL;

    if (ctx->cal.has_gyro) {
        memcpy(init_bias, ctx->cal.gyro_bias, sizeof(init_bias));
        cal_flags |= FLAG_GYRO_CAL;
    } else if (cfg.gyro_bias_sec > 0.0) {
        int n_needed = (int)(cfg.gyro_bias_sec * odr_hz);
        if (n_needed < 1) n_needed = 1;

        double sum[3] = {0}, sumsq[3] = {0};
        int  count = 0, target = n_needed;
        bool extended = false;
        imu_sample_t s;

        LOG_I("[fusion] gyro bias estimation: collecting %d samples\n",
                n_needed);

        /* Motion check: on a vessel rolling at anchor, the mean over a
         * short window is contaminated by wave rotation. If the per-axis
         * std exceeds ~0.5 °/s, warn and double the window once — a longer
         * average spans more wave cycles and cancels better. The MEKF
         * refines the bias online either way. */
        while (count < target && !ctx->stop) {
            if (imu_ring_pop(&ctx->imu_ring, &s, &ctx->stop) != 0)
                continue;
            for (int k = 0; k < 3; k++) {
                sum[k]   += s.gyro[k];
                sumsq[k] += (double)s.gyro[k] * s.gyro[k];
            }
            count++;

            if (count == target && !extended) {
                float std_max = 0.0f;
                for (int k = 0; k < 3; k++) {
                    /*
                     * E[x²] − mean², the cancellation-prone variance form,
                     * and safe here for a reason worth writing down rather
                     * than rediscovering.  Gyro readings are ~1e-3 rad/s, so
                     * E[x²] and mean² are both ~1e-6 and their difference is
                     * the ~1e-8 variance — three digits lost out of double's
                     * sixteen.  The window is bounded (gyro_bias_sec, at most
                     * doubled once), so the sums cannot grow without limit
                     * either.  In float32 this same expression would keep
                     * about four digits and the 0.5 °/s threshold below would
                     * be reading noise; the accumulators are double for
                     * exactly that reason and must stay that way.
                     */
                    double mean = sum[k] / count;
                    double var  = sumsq[k] / count - mean * mean;
                    if (var > 0 && (float)sqrt(var) > std_max)
                        std_max = (float)sqrt(var);
                }
                if (std_max > 0.00873f) {   /* 0.5 °/s */
                    extended = true;
                    target *= 2;
                    LOG_W("[fusion] gyro std %.2f °/s during bias "
                            "window — vessel moving; extending capture to "
                            "%.1f s\n", std_max * (float)(180.0 / M_PI),
                            2.0 * cfg.gyro_bias_sec);
                }
            }
        }
        if (count > 0) {
            init_bias[0] = (float)(sum[0] / count);
            init_bias[1] = (float)(sum[1] / count);
            init_bias[2] = (float)(sum[2] / count);
            cal_flags |= FLAG_GYRO_CAL;
            LOG_I("[fusion] gyro bias = [%.5f, %.5f, %.5f] rad/s"
                    "%s\n", init_bias[0], init_bias[1], init_bias[2],
                    extended ? "  (motion during capture — treat as rough)" : "");
        }
    }

    if (ctx->stop) return NULL;

    /* ── Phase 2: MEKF init ───────────────────────────────────────────────── */

    mekf_t f;
    mekf_init(&f, &cfg, odr_hz, mag_odr_hz, init_bias);

    heave_t heave;
    heave_init(&heave, cfg.heave_tau_s, f.dt);
    if (heave.enabled)
        LOG_I("[fusion] heave estimator on (tau=%.0f s)\n",
                (double)cfg.heave_tau_s);

    /* Sea state rides on heave: wave_tau_s > 0 needs a running heave estimator. */
    seastate_t wave;
    seastate_init(&wave, heave.enabled ? cfg.wave_tau_s : 0.0f, f.dt);
    if (wave.enabled)
        LOG_I("[fusion] sea-state estimator on (tau=%.0f s)\n",
                (double)cfg.wave_tau_s);
    else if (cfg.wave_tau_s > 0.0f)
        LOG_W("[fusion] wave_tau_s set but heave_tau_s = 0 — sea state disabled\n");

    /* ── Phase 3: initial alignment (tilt from accel, heading from mag) ─────
     *
     * Average ~1 s of accel and every mag sample seen in that window rather
     * than aligning from one instantaneous reading: a single mid-swing
     * sample in waves can be ~10° off, and any alignment error is baked
     * into m_ref (the heading reference), so averaging directly improves
     * the long-term estimate. Wave-orbital accelerations largely cancel
     * over the window. */

    {
        double acc_sum[3] = {0}, mag_sum[3] = {0};
        int    acc_n = 0, mag_n = 0;
        /*
         * Averaging window for the one-shot alignment (align_window_sec).
         *
         * This used to be a hardcoded ~1 s. That is fine at a dock and poor at
         * sea: one second is a fifth of a typical roll period, so the mean is
         * taken over an arbitrary fraction of the cycle and the tilt estimate
         * — hence m_ref's dip and heading anchor — is left wherever the wave
         * happened to be. Measured over the 12-seed wave benchmark, attitude
         * RMS in the marine (yaw-only) default: 47.7° at 1 s, 2.28° at 2 s,
         * 2.19° at 5 s, flat thereafter. The default is now 5 s.
         */
        double aw_s = ctx->cfg.align_window_sec;
        if (!(aw_s > 0.0)) aw_s = 1.0;
        int    n_avg = (int)(aw_s * odr_hz);
        if (n_avg < 8) n_avg = 8;

        /* The mag-starvation fallback must outlast the averaging window. */
        double mag_wait_s = (aw_s > 5.0) ? aw_s : 5.0;

        struct timespec align_start;
        clock_gettime(CLOCK_MONOTONIC, &align_start);

        imu_sample_t isample;
        mag_sample_t msample;

        /* Uncalibrated mag: the reader marks every sample invalid so the
         * hard-iron error never steers the filter, but for the one-shot
         * initial alignment a raw field is still a far better heading
         * reference than the forward-is-North fallback.  Accept invalid
         * samples here only; Phase 4 fusion stays gated on msample.valid. */
        bool mag_uncal = !ctx->cal.has_mag;

        while (acc_n < n_avg && !ctx->stop) {
            if (imu_ring_pop(&ctx->imu_ring, &isample, &ctx->stop) == 0) {
                acc_sum[0] += isample.accel[0];
                acc_sum[1] += isample.accel[1];
                acc_sum[2] += isample.accel[2];
                acc_n++;
            }
            while (mag_ring_try_pop(&ctx->mag_ring, &msample) == 0) {
                if (msample.valid || mag_uncal) {
                    mag_sum[0] += msample.field[0];
                    mag_sum[1] += msample.field[1];
                    mag_sum[2] += msample.field[2];
                    mag_n++;
                }
            }
        }

        /* Keep waiting (up to mag_wait_s total) for at least one mag sample.
         * Drain the IMU ring while waiting: it holds only ~0.3 s at 833 Hz,
         * and an undrained wait counts thousands of ring drops as FIFO
         * overflows. */
        while (mag_n == 0 && !ctx->stop) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (double)(now.tv_sec  - align_start.tv_sec)
                           + (double)(now.tv_nsec - align_start.tv_nsec) * 1e-9;
            if (elapsed > mag_wait_s) break;
            if (mag_ring_try_pop(&ctx->mag_ring, &msample) == 0) {
                if (msample.valid || mag_uncal) {
                    mag_sum[0] += msample.field[0];
                    mag_sum[1] += msample.field[1];
                    mag_sum[2] += msample.field[2];
                    mag_n++;
                }
            } else {
                /* Discard; the pop's 100 ms timeout paces the loop. */
                imu_ring_pop(&ctx->imu_ring, &isample, &ctx->stop);
            }
        }

        if (!ctx->stop && acc_n > 0) {
            float acc_avg[3], mag_avg[3];
            for (int k = 0; k < 3; k++)
                acc_avg[k] = (float)(acc_sum[k] / acc_n);
            if (mag_n > 0) {
                for (int k = 0; k < 3; k++)
                    mag_avg[k] = (float)(mag_sum[k] / mag_n);
            } else {
                /* Timeout: align without heading (assume forward = North). */
                LOG_W("[fusion] no mag sample after %.0f s; "
                        "aligning without heading\n", mag_wait_s);
                mag_avg[0] = 1.0f; mag_avg[1] = 0.0f; mag_avg[2] = 0.0f;
            }
            mekf_align(&f, acc_avg, mag_avg);
            /* WMM-known field invariants remove the residual alignment-tilt
             * error from the magnetic reference at the source. */
            if (cfg.pos_mref_valid)
                mekf_set_mref_invariants(&f, cfg.pos_mref_h_gauss,
                                         cfg.pos_mref_z_gauss);
            LOG_I("[fusion] aligned from %d accel + %d mag samples%s%s\n",
                    acc_n, mag_n,
                    (mag_n > 0 && mag_uncal)
                        ? " (uncalibrated mag — heading approximate)" : "",
                    cfg.pos_mref_valid ? " (m_ref invariants from WMM)" : "");
        }
    }

    if (ctx->stop) return NULL;

    /* ── Phase 4: main prediction + update loop ───────────────────────────── */

    /* All startup phases complete (settle → bias → MEKF init → alignment).
     * Release-store so output/health threads see all prior writes. */
    atomic_store_explicit(&ctx->settled, 1, memory_order_release);

    imu_sample_t s;
    mag_sample_t m;
    int prev_engine_on = -1;   /* track transitions for logging */
    bool mag_healthy   = false; /* set on first valid mag update; drives FLAG_MAG_VALID */
    uint64_t prev_wall_ns = 0;  /* previous sample time for per-sample dt */
    uint32_t prev_gen     = 0;

    while (!ctx->stop) {
        if (imu_ring_pop(&ctx->imu_ring, &s, &ctx->stop) != 0)
            continue;

        cfg_snapshot(ctx, &cfg);   /* fresh live config for this sample */

        if (ctx->reconfigure) {
            mekf_reconfigure(&f, &cfg);
            /* heave_tau_s is [hot]: re-derive constants, keep filter state
             * unless toggling between enabled/disabled. */
            if ((cfg.heave_tau_s > 0.0f) != heave.enabled)
                heave_init(&heave, cfg.heave_tau_s, f.dt);
            else
                heave.tau = cfg.heave_tau_s;
            /* wave_tau_s is [hot] too; sea state needs heave running. */
            {
                float wtau = heave.enabled ? cfg.wave_tau_s : 0.0f;
                if ((wtau > 0.0f) != wave.enabled)
                    seastate_init(&wave, wtau, f.dt);
                else
                    wave.tau = wtau > 0.0f ? wtau : wave.tau;
            }
            ctx->reconfigure = 0;
        }

        /* Live WMM field invariants pushed by the position thread. */
        if (ctx->mref_update) {
            mekf_set_mref_invariants(&f, cfg.pos_mref_h_gauss,
                                     cfg.pos_mref_z_gauss);
            ctx->mref_update = 0;
        }

        /* Speed over ground for the centripetal correction (0 = off). */
        f.speed_mps = cfg.pos_speed_valid ? cfg.pos_speed_mps : 0.0f;

        /*
         * Timestamps + per-sample dt.  For hw-timestamp chips chip_ts encodes
         * the exact sample time, and the interval between two of them is what
         * the predict step wants.
         *
         * That interval is only as good as the tick period it is scaled by.
         * A datasheet typical did NOT remove the chip's oscillator tolerance —
         * it preserved it exactly, because the interval is ticks times that
         * constant, so a counter running a few percent fast made every dt a few
         * percent long and scaled all integrated rotation by the same factor.
         * ts_anchor_t measures the real period across consecutive anchors and
         * chip_to_wall applies it, which is what makes this paragraph true
         * rather than aspirational.  ctx->ts_tick_ns is what it falls back to
         * before two anchors exist, and is the part's own declared period where
         * the driver could ask for it (imu_ops_t.ts_tick_ns_actual) rather than
         * the typical — so the first minute is close too, not just the rest.
         *
         * Clamped to [0.5x, 2x] nominal so FIFO-overflow gaps and anchor
         * resets fall back to the nominal period.
         */
        uint64_t wall = 0, tai = 0;
        uint32_t gen  = 0;
        bool have_ts  = (s.chip_ts != 0 || !ctx->imu_ops->has_hw_timestamp);
        float dt      = f.dt;
        if (have_ts) {
            chip_to_wall(&ctx->anchor, s.chip_ts, ctx->ts_tick_ns,
                         &wall, &tai, &gen);
            if (ctx->imu_ops->has_hw_timestamp && s.chip_ts != 0 &&
                prev_wall_ns != 0 && gen == prev_gen && wall > prev_wall_ns) {
                float d = (float)((double)(wall - prev_wall_ns) * 1e-9);
                if (d > 0.5f * f.dt && d < 2.0f * f.dt)
                    dt = d;
            }
            prev_wall_ns = wall;
            prev_gen     = gen;
        }

        mekf_predict(&f, &s, dt);

        /* Drain all pending mag samples before the accel update. */
        while (mag_ring_try_pop(&ctx->mag_ring, &m) == 0) {
            if (m.valid) {
                mekf_update_mag(&f, &m);
                mag_healthy = true;
            }

            /* Always update latest_mag regardless of valid flag. */
            pthread_mutex_lock(&ctx->shared.lock);
            ctx->shared.latest_mag = m;
            pthread_mutex_unlock(&ctx->shared.lock);
        }

        /* Switch accel gating when engine vibration is detected: widen the
         * skip window so the filter isn't starved, but also inflate the
         * accel noise ×4 so the vibration-contaminated samples that do pass
         * are trusted proportionally less.
         *
         * Applied on EVERY sample, not just on a transition: mekf_reconfigure
         * resets accel_skip_lo/hi from the non-engine threshold, so a SIGHUP
         * while the engine was running used to leave the skip window narrow
         * with Ra_scale still at 4.0 — a half-engine state that persisted
         * until the engine next stopped. Re-asserting is four float stores.
         * Only the log stays edge-triggered. */
        if (cfg.engine_vibration_g2 > 0.0) {
            float sk = ctx->engine_on
                       ? (float)cfg.engine_accel_skip_thresh
                       : (float)cfg.accel_skip_thresh;
            f.accel_skip_lo = 1.0f - sk;
            f.accel_skip_hi = 1.0f + sk;
            f.Ra_scale      = ctx->engine_on ? 4.0f : 1.0f;
            if (ctx->engine_on != prev_engine_on) {
                prev_engine_on = ctx->engine_on;
                LOG_I("[fusion] engine vibration %s — accel_skip=%.2f "
                        "Ra×%.0f\n",
                        ctx->engine_on ? "detected" : "cleared", sk,
                        (double)f.Ra_scale);
            }
        }

        mekf_update_accel(&f, &s);

        /*
         * Last gate before anything is published.  It sits here, rather than
         * inside mekf_predict, because all three producers of filter state
         * have now run — predict, the mag updates and the accel update — so a
         * non-finite value from any of them is caught in the same sample it
         * appeared, and no packet can carry one.  It is also ahead of
         * heave_update below, which reads f.q directly.
         *
         * Rate-limited because at 833 Hz arithmetic that produced one
         * non-finite value will likely produce the next few thousand too, and
         * the first message — the one that says when it started — is the one
         * worth keeping.
         */
        if (mekf_sanitize(&f) &&
            (f.reset_count == 1 || (f.reset_count % 1000) == 0))
            LOG_E("[fusion] non-finite filter state — reset and re-aligning "
                    "(occurrence %u)\n", f.reset_count);

        /* Extract fused state and fill fields mekf_get_state leaves as stubs. */
        fused_state_t state;
        mekf_get_state(&f, &state, cal_flags);

        state.heave_m    = heave_update(&heave, f.q, s.accel);
        /* heave.vel is double (see heave_t); the wire field is float. */
        state.heave_rate = (float)(-heave.vel);  /* NED down → up, m/s */
        if (heave.enabled && heave.settled)
            state.flags |= FLAG_HEAVE_VALID;

        /* Sea state: feed only settled heave (the settling ramp is a huge
         * low-frequency transient that would poison the variances). Roll
         * rate is the Euler rate φ̇ = ω_x + tanθ·(ω_y·sinφ + ω_z·cosφ),
         * clamped back to body ω_x near ±90° pitch like rate_of_turn;
         * pitch rate θ̇ = ω_y·cosφ − ω_z·sinφ has no singularity. */
        if (wave.enabled && heave.settled) {
            float wx = s.gyro[0] - f.bias[0];
            float wy = s.gyro[1] - f.bias[1];
            float wz = s.gyro[2] - f.bias[2];
            float sf = sinf(state.roll), cf = cosf(state.roll);
            float ct = cosf(state.pitch);
            float phi_dot = (fabsf(ct) > 0.2f)
                ? wx + (sinf(state.pitch)/ct) * (wy * sf + wz * cf)
                : wx;
            float theta_dot = wy * cf - wz * sf;
            seastate_update(&wave, state.heave_m, state.heave_rate,
                            state.roll, phi_dot, state.pitch, theta_dot);
        }
        state.wave_height_m   = seastate_wave_height(&wave);
        state.wave_period_s   = seastate_wave_period(&wave);
        state.roll_period_s   = seastate_roll_period(&wave);
        state.roll_amplitude  = seastate_roll_amplitude(&wave);
        state.pitch_period_s  = seastate_pitch_period(&wave);
        state.pitch_amplitude = seastate_pitch_amplitude(&wave);
        if (wave.enabled && wave.settled)
            state.flags |= FLAG_WAVE_VALID;

        if (ctx->engine_on)
            state.flags |= FLAG_ENGINE_ON;

        if (mag_healthy)
            state.flags |= FLAG_MAG_VALID;

        if (ctx->mag_set_flag) {
            state.flags |= FLAG_MAG_SET_RESET;
            ctx->mag_set_flag = 0;
        }

        /* rate_of_turn: true heading rate (Euler yaw rate), not raw body-Z.
         * ψ̇ = (ω_y·sinφ + ω_z·cosφ)/cosθ — with 20° of roll the raw body-Z
         * gyro under-reads ROT and couples in pitch rate, which autopilots
         * notice. Falls back to body-Z near ±90° pitch (cosθ → 0). */
        {
            float wy = s.gyro[1] - f.bias[1];
            float wz = s.gyro[2] - f.bias[2];
            float ct = cosf(state.pitch);
            float psi_dot = (fabsf(ct) > 0.2f)
                ? (wy * sinf(state.roll) + wz * cosf(state.roll)) / ct
                : wz;
            state.rate_of_turn = psi_dot * (float)(180.0 / M_PI) * 60.0f;
        }

        /* Timestamps computed once above (also feed the per-sample dt). */
        if (have_ts) {
            state.ts_wall_ns    = wall;
            state.ts_tai_ns     = tai;
            state.ts_chip_ticks = s.chip_ts;
            state.anchor_gen    = gen;
        }
        state.imu_seq = s.seq;

        /*
         * Sample latency, both terms.
         *
         * Only measurable with a hardware timestamp: without one, chip_to_wall
         * degenerates to the burst's own read midpoint, so `wall` and
         * read_done_ns are the same clock reading and their difference is
         * noise rather than FIFO residence.  Reporting the pipeline alone
         * would invite reading it as the total, so the whole thing stays zero.
         */
        if (have_ts && ctx->imu_ops->has_hw_timestamp && s.read_done_ns) {
            struct timespec t_now;
            clock_gettime(CLOCK_REALTIME, &t_now);
            uint64_t now_ns = ts_ns(&t_now);

            /*
             * Record and publish in one step, so the recording clamps and the
             * two independent window gates live together where they can be
             * tested — see lat_step in imu_math.h for both.  The window is
             * `actual_odr_mhz` samples, which self-paces to about a second at
             * any ODR.  main.c prints the clause when either p99 is nonzero,
             * so a split publish still reads as one line.
             */
            lat_pub_t pf, pp;
            lat_step(&ctx->lat_fifo, &ctx->lat_pipe,
                     (uint64_t)((ctx->actual_odr_mhz + 500) / 1000),
                     wall, s.read_done_ns, now_ns, &pf, &pp);

            if (pf.valid) {
                atomic_store(&ctx->lat_fifo_p50, pf.p50);
                atomic_store(&ctx->lat_fifo_p99, pf.p99);
                atomic_store(&ctx->lat_fifo_max, pf.max_ever);
            }
            if (pp.valid) {
                atomic_store(&ctx->lat_pipe_p50, pp.p50);
                atomic_store(&ctx->lat_pipe_p99, pp.p99);
                atomic_store(&ctx->lat_pipe_max, pp.max_ever);
            }
        }

        /* Declination validity is an explicit flag, not a 0.0 sentinel, so a
         * WMM-computed 0.0° on the agonic line still yields true heading. */
        if (cfg.pos_declination_valid) {
            state.declination_deg = cfg.pos_declination_deg;
            state.flags |= FLAG_DECLINATION_VALID;
        }

        /* Bias-corrected IMU sample for binary packet fused gyro fields. */
        imu_sample_t imu_pkt = s;
        for (int i = 0; i < 3; i++)
            imu_pkt.gyro[i] -= f.bias[i];

        pthread_mutex_lock(&ctx->shared.lock);
        ctx->shared.state = state;
        ctx->latest_imu   = imu_pkt;  /* calibrated accel + bias-corrected gyro */
        ctx->raw_imu      = s;        /* pre-cal accel (accel_raw) + pre-bias gyro */
        pthread_mutex_unlock(&ctx->shared.lock);
    }

    return NULL;
}

/* ── imu_ctx_open ────────────────────────────────────────────────────────── */

int imu_ctx_open(imu_ctx_t **ctx_out,
                 const imud_config_t *cfg,
                 const imud_cal_t *cal)
{
    imu_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) { LOG_E("[imu] calloc: %s\n", strerror(errno)); return -1; }
    /* calloc zeros both handles to fd 0; bus_init makes them -1 so a fail:
     * before the opens does not close stdin. */
    bus_init(&ctx->imu_bus);
    bus_init(&ctx->mag_bus);

    ctx->cfg = *cfg;
    if (cal) {
        ctx->cal = *cal;
    } else {
        /* No cal: identity defaults so apply_*_cal is a no-op. */
        ctx->cal.accel_scale[0] = ctx->cal.accel_scale[1] = ctx->cal.accel_scale[2] = 1.0f;
        ctx->cal.mag_soft_iron[0][0] = ctx->cal.mag_soft_iron[1][1]
            = ctx->cal.mag_soft_iron[2][2] = 1.0f;
    }

    imu_ring_init(&ctx->imu_ring);
    mag_ring_init(&ctx->mag_ring);
    cap_ring_init(&ctx->cap_ring);
    pthread_mutex_init(&ctx->shared.lock, NULL);
    pthread_mutex_init(&ctx->anchor.mtx, NULL);
    pthread_mutex_init(&ctx->live_lock, NULL);

    /* ── Locate drivers ──────────────────────────────────────────────────── */

    /* Arm sim-driver playback before init ([device] sim_file / --replay). */
    sim_set_playback(cfg->sim_file, cfg->sim_loop, cfg->sim_speed);

    ctx->imu_ops = imu_driver_find(cfg->imu_driver);
    if (!ctx->imu_ops) {
        LOG_E("[imu] unknown IMU driver '%s'\n", cfg->imu_driver);
        goto fail;
    }
    if (ctx->imu_ops->experimental)
        LOG_W("[imu] WARNING: driver '%s' is EXPERIMENTAL — "
                "not yet validated on hardware\n", cfg->imu_driver);
    if (ctx->imu_ops->has_hw_timestamp && ctx->imu_ops->ts_tick_ns == 0)
        LOG_W("[imu] driver '%s' sets has_hw_timestamp but no ts_tick_ns — "
                "hardware timestamps will not advance\n", cfg->imu_driver);

    ctx->mag_ops = mag_driver_find(cfg->mag_driver);
    if (!ctx->mag_ops) {
        LOG_E("[imu] unknown mag driver '%s'\n", cfg->mag_driver);
        goto fail;
    }
    if (ctx->mag_ops->experimental)
        LOG_W("[mag] WARNING: driver '%s' is EXPERIMENTAL — "
                "not yet validated on hardware\n", cfg->mag_driver);

    /*
     * Resolve both requested rates to what the drivers will really program,
     * BEFORE anything else uses them. The resolved values — not the raw config
     * requests — go to the drivers below and into mekf_init, so the hardware
     * and the filter tuning cannot disagree about the sample rate. Asking the
     * driver rather than snapping here is what lets the divider-based parts
     * (mpu925x, icm20948) answer for their own grid.
     */
    ctx->actual_odr_mhz     = odr_actual_imu(ctx->imu_ops, cfg->imu_odr_mhz);
    ctx->actual_mag_odr_mhz = odr_actual_mag(ctx->mag_ops, cfg->mag_odr_mhz);

    /* ── Open the sensor buses ───────────────────────────────────────────── */

    /* One handle each. On I2C that opens the same node twice, which costs a
     * descriptor and buys a uniform model: no shared-fd ownership to track,
     * and the two sensors are free to live on different buses. */
    bus_spec_t spec;

    /*
     * Two SPI devices on ONE controller must agree about the clock mode.
     *
     * spidev sets the mode per chip select, but the controller has a single
     * SCLK and can only idle it at one level -- mode 0 idles low, mode 3 high.
     * Put a mode-3 part on spidev0.0 beside a mode-0 part on spidev0.1 and the
     * bus corrupts: measured on the reference rig, the daemon opened both
     * parts, settled, and then never produced a sample. Nothing in the log
     * said why, because each device was individually configured exactly as its
     * driver asked.
     *
     * Refuse it at startup instead. This is a wiring-and-driver combination
     * question, not an operator mistake, so the message names both parts and
     * the modes they want rather than blaming the config.
     */
    if (cfg->imu_bus_kind == BUS_SPI && cfg->mag_bus_kind == BUS_SPI &&
        ctx->imu_ops->bus_caps.spi_mode != ctx->mag_ops->bus_caps.spi_mode &&
        bus_spi_same_controller(cfg->imu_spi_dev, cfg->mag_spi_dev)) {
        LOG_E("[imu] %s wants SPI mode %u and %s wants mode %u, but %s and %s "
              "share one SPI controller, which has a single clock. Move one "
              "part to another SPI bus.\n",
              ctx->imu_ops->name, ctx->imu_ops->bus_caps.spi_mode,
              ctx->mag_ops->name, ctx->mag_ops->bus_caps.spi_mode,
              cfg->imu_spi_dev, cfg->mag_spi_dev);
        goto fail;
    }

    config_imu_bus_spec(cfg, &spec);
    if (bus_open(&ctx->imu_bus, &spec, &ctx->imu_ops->bus_caps, "imu") < 0)
        goto fail;

    config_mag_bus_spec(cfg, &spec);
    if (bus_open(&ctx->mag_bus, &spec, &ctx->mag_ops->bus_caps, "mag") < 0)
        goto fail;

    /* ── Build hw config structs ─────────────────────────────────────────── */

    /* Resolved rates, not cfg->*_odr_mhz: the driver's own rounding is then a
     * no-op and it programs exactly what the filter was tuned for. */
    ctx->imu_hw_cfg.odr_mhz  = ctx->actual_odr_mhz;
    ctx->imu_hw_cfg.accel_g  = cfg->imu_accel_g;
    ctx->imu_hw_cfg.gyro_dps = cfg->imu_gyro_dps;
    ctx->imu_hw_cfg.fifo_wm  = cfg->imu_fifo_wm;

    ctx->mag_hw_cfg.odr_mhz      = ctx->actual_mag_odr_mhz;
    ctx->mag_hw_cfg.set_period_s = cfg->mag_set_period_s;
    /*
     * Deliberately the SAME condition that requests the line further down, so
     * the driver's idea of how it is being read cannot drift from what the
     * reader thread actually does.  On a part whose gate and interrupt are
     * mutually exclusive that disagreement is not cosmetic — it is the
     * difference between every sample and one in three.  See mag_cfg_t.
     */
    ctx->mag_hw_cfg.int_driven   = ctx->mag_ops->has_interrupt
                                && cfg->mag_int_gpio > 0;

    char hb1[16], hb2[16];   /* MHZ_STR scratch for the startup lines */

    /* ── Probe + reset + init IMU ────────────────────────────────────────── */

    if (ctx->imu_ops->probe(&ctx->imu_bus) < 0) {
        LOG_E("[imu] %s probe failed at 0x%02X\n",
                ctx->imu_ops->name, cfg->imu_addr);
        goto fail;
    }
    if (ctx->imu_ops->reset(&ctx->imu_bus) < 0) {
        LOG_E("[imu] %s reset failed\n", ctx->imu_ops->name);
        goto fail;
    }
    if (ctx->imu_ops->init(&ctx->imu_bus, &ctx->imu_hw_cfg) < 0) {
        LOG_E("[imu] %s init failed\n", ctx->imu_ops->name);
        goto fail;
    }
    /*
     * Ask the part what its timer period really is, now that the bus is up and
     * init() has run.  Resolved once and never again: the parts that answer do
     * so from a factory trim register, which a reset cannot change, so the
     * error-recovery re-init path above deliberately does not re-read it —
     * that is the least healthy moment to add a bus transfer for a value that
     * cannot have moved.  A driver with no hook, or one whose read fails,
     * leaves the declared constant in place, which is what shipped before.
     */
    ctx->ts_tick_ns = ctx->imu_ops->ts_tick_ns;
    if (ctx->imu_ops->ts_tick_ns_actual) {
        uint32_t part = ctx->imu_ops->ts_tick_ns_actual(&ctx->imu_bus);
        if (part != 0 && part != ctx->ts_tick_ns) {
            LOG_I("[imu] %s declares a %u ns timer tick against the %u ns "
                  "typical (%+.2f%%); using the part's own value\n",
                  ctx->imu_ops->name, part, ctx->ts_tick_ns,
                  ((double)part - (double)ctx->ts_tick_ns)
                      / (double)ctx->ts_tick_ns * 100.0);
            ctx->ts_tick_ns = part;
        }
    }

    if (ctx->actual_odr_mhz != cfg->imu_odr_mhz)
        LOG_I("[imu] %s 0x%02X OK — ODR %s Hz requested, %s Hz actual\n",
                ctx->imu_ops->name, cfg->imu_addr,
                MHZ_STR(hb1, cfg->imu_odr_mhz),
                MHZ_STR(hb2, ctx->actual_odr_mhz));
    else
        LOG_I("[imu] %s 0x%02X OK — actual ODR %s Hz\n",
                ctx->imu_ops->name, cfg->imu_addr,
                MHZ_STR(hb1, ctx->actual_odr_mhz));

    /* ── Probe + reset + init mag ────────────────────────────────────────── */

    if (ctx->mag_ops->probe(&ctx->mag_bus) < 0) {
        LOG_E("[imu] %s probe failed at 0x%02X\n",
                ctx->mag_ops->name, cfg->mag_addr);
        goto fail;
    }
    if (ctx->mag_ops->reset(&ctx->mag_bus) < 0) {
        LOG_E("[imu] %s reset failed\n", ctx->mag_ops->name);
        goto fail;
    }
    if (ctx->mag_ops->init(&ctx->mag_bus, &ctx->mag_hw_cfg) < 0) {
        LOG_E("[imu] %s init failed\n", ctx->mag_ops->name);
        goto fail;
    }
    if (ctx->actual_mag_odr_mhz != cfg->mag_odr_mhz)
        LOG_I("[imu] %s 0x%02X OK — ODR %s Hz requested, %s Hz actual\n",
                ctx->mag_ops->name, cfg->mag_addr,
                MHZ_STR(hb1, cfg->mag_odr_mhz),
                MHZ_STR(hb2, ctx->actual_mag_odr_mhz));
    else
        LOG_I("[imu] %s 0x%02X OK — actual ODR %s Hz\n",
                ctx->mag_ops->name, cfg->mag_addr,
                MHZ_STR(hb1, ctx->actual_mag_odr_mhz));

    /* ── Configure GPIO lines ────────────────────────────────────────────── */

    /* Only open the GPIO chip if at least one interrupt line is configured.
     * Sim mode sets int_gpio = 0 for both and skips this entirely. */
    bool need_gpio = cfg->imu_int_gpio > 0 ||
                     (ctx->mag_ops->has_interrupt && cfg->mag_int_gpio > 0);
    if (need_gpio) {
        char chip_path[80];
        snprintf(chip_path, sizeof(chip_path), "/dev/%s", cfg->gpio_chip);
        ctx->gpio_chip = gpiod_chip_open(chip_path);
        if (!ctx->gpio_chip) {
            LOG_E("[imu] cannot open /dev/%s: %s\n",
                    cfg->gpio_chip, strerror(errno));
            goto fail;
        }
    }

    /* IMU interrupt line — optional (timer fallback used when absent). */
    if (cfg->imu_int_gpio > 0) {
        ctx->imu_line = open_gpio_line_as(ctx->gpio_chip, (unsigned)cfg->imu_int_gpio, "imud");
        if (!ctx->imu_line) {
            LOG_E("[imu] cannot request GPIO%d: %s\n",
                    cfg->imu_int_gpio, strerror(errno));
            goto fail;
        }
    }

    /* Mag interrupt line — only requested when the driver has an external pin. */
    if (ctx->mag_ops->has_interrupt && cfg->mag_int_gpio > 0) {
        ctx->mag_line = open_gpio_line_as(ctx->gpio_chip, (unsigned)cfg->mag_int_gpio, "imud");
        if (!ctx->mag_line) {
            LOG_E("[imu] cannot request GPIO%d: %s\n",
                    cfg->mag_int_gpio, strerror(errno));
            goto fail;
        }
    }

    *ctx_out = ctx;
    return 0;

fail:
    bus_close(&ctx->imu_bus);
    bus_close(&ctx->mag_bus);
    if (ctx->imu_line)     release_gpio_line(ctx->imu_line);
    if (ctx->mag_line)     release_gpio_line(ctx->mag_line);
    if (ctx->gpio_chip)    gpiod_chip_close(ctx->gpio_chip);
    pthread_mutex_destroy(&ctx->shared.lock);
    pthread_mutex_destroy(&ctx->anchor.mtx);
    pthread_mutex_destroy(&ctx->live_lock);
    pthread_mutex_destroy(&ctx->imu_ring.lock);
    pthread_cond_destroy(&ctx->imu_ring.ready);
    pthread_mutex_destroy(&ctx->mag_ring.lock);
    pthread_cond_destroy(&ctx->mag_ring.ready);
    cap_ring_destroy(&ctx->cap_ring);
    free(ctx);
    return -1;
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/*
 * All four setters below write ctx->cfg under live_lock; the worker threads
 * read a snapshot under the same lock (cfg_snapshot), so there is no data
 * race. The _Atomic reconfigure/mref_update flags are set last, after the
 * data is committed, telling the fusion loop that expensive re-derivation
 * (mekf_reconfigure, heave/wave re-init, m_ref invariants) is due.
 */
void imu_ctx_update_config(imu_ctx_t *ctx, const imud_config_t *new_cfg)
{
    pthread_mutex_lock(&ctx->live_lock);
    ctx->cfg.mekf_gyro_noise           = new_cfg->mekf_gyro_noise;
    ctx->cfg.mekf_gyro_bias            = new_cfg->mekf_gyro_bias;
    ctx->cfg.mekf_accel_noise          = new_cfg->mekf_accel_noise;
    ctx->cfg.mekf_mag_noise            = new_cfg->mekf_mag_noise;
    ctx->cfg.mekf_wave_accel           = new_cfg->mekf_wave_accel;
    ctx->cfg.mekf_wave_accel_tau_s     = new_cfg->mekf_wave_accel_tau_s;
    ctx->cfg.mekf_mag_dip_sigma_deg    = new_cfg->mekf_mag_dip_sigma_deg;
    ctx->cfg.mag_reject_gauss          = new_cfg->mag_reject_gauss;
    ctx->cfg.accel_skip_thresh         = new_cfg->accel_skip_thresh;
    ctx->cfg.engine_vibration_g2       = new_cfg->engine_vibration_g2;
    ctx->cfg.engine_accel_skip_thresh  = new_cfg->engine_accel_skip_thresh;
    /* [hot] fusion params the reconfigure path re-derives — these must be
     * published here too or a SIGHUP change to them never reaches the
     * running filter (mekf_reconfigure reads mag_yaw_only; the fusion loop
     * reads heave_tau_s/wave_tau_s). */
    ctx->cfg.mag_yaw_only              = new_cfg->mag_yaw_only;
    ctx->cfg.heave_tau_s               = new_cfg->heave_tau_s;
    ctx->cfg.wave_tau_s                = new_cfg->wave_tau_s;
    /* Static declination applies only when no live position source is
     * configured. With gpsd/SignalK enabled the position thread owns this
     * field via imu_ctx_set_declination(); copying main's static value
     * (typically 0.0) here would clobber the live GPS-derived declination
     * on every SIGHUP and drop true-heading output until the next ≥5 km
     * position change. */
    if (!new_cfg->pos_gpsd_enabled && !new_cfg->pos_signalk_enabled) {
        ctx->cfg.pos_declination_deg   = new_cfg->pos_declination_deg;
        ctx->cfg.pos_declination_valid = new_cfg->pos_declination_valid;
        if (new_cfg->pos_mref_valid) {
            ctx->cfg.pos_mref_h_gauss = new_cfg->pos_mref_h_gauss;
            ctx->cfg.pos_mref_z_gauss = new_cfg->pos_mref_z_gauss;
            ctx->cfg.pos_mref_valid   = true;
            ctx->mref_update = 1;
        }
    }
    ctx->reconfigure = 1;
    pthread_mutex_unlock(&ctx->live_lock);
}

void imu_ctx_set_mag_ref(imu_ctx_t *ctx, float h_gauss, float z_gauss)
{
    pthread_mutex_lock(&ctx->live_lock);
    ctx->cfg.pos_mref_h_gauss = h_gauss;
    ctx->cfg.pos_mref_z_gauss = z_gauss;
    ctx->cfg.pos_mref_valid   = true;
    ctx->mref_update = 1;
    pthread_mutex_unlock(&ctx->live_lock);
}

void imu_ctx_set_speed(imu_ctx_t *ctx, float speed_mps, bool valid)
{
    pthread_mutex_lock(&ctx->live_lock);
    ctx->cfg.pos_speed_mps   = speed_mps;
    ctx->cfg.pos_speed_valid = valid;
    pthread_mutex_unlock(&ctx->live_lock);
}

void imu_ctx_set_declination(imu_ctx_t *ctx, float decl_deg, bool valid)
{
    /* The fusion thread reads these each predict step (via its snapshot);
     * a new value takes effect within one IMU sample period (~1.2 ms at
     * 833 Hz). */
    pthread_mutex_lock(&ctx->live_lock);
    ctx->cfg.pos_declination_deg   = decl_deg;
    ctx->cfg.pos_declination_valid = valid;
    pthread_mutex_unlock(&ctx->live_lock);
}

void imu_ctx_stop(imu_ctx_t *ctx)
{
    ctx->stop = 1;
    /* Wake threads blocked in imu_ring_pop. */
    pthread_cond_broadcast(&ctx->imu_ring.ready);
}

int imu_ctx_ring_backlog(imu_ctx_t *ctx)
{
    /*
     * The context is opaque, so a caller outside this file cannot reach the
     * ring itself.  Only the IMU ring is reported: the mag ring is
     * overwrite-on-full by design and carries corrections rather than the
     * sample stream, so samples left in it are not work owed to anyone.
     */
    return ctx ? imu_ring_count(&ctx->imu_ring) : 0;
}

void imu_ctx_free(imu_ctx_t *ctx)
{
    if (!ctx) return;
    bus_close(&ctx->imu_bus);
    bus_close(&ctx->mag_bus);
    if (ctx->imu_line)  release_gpio_line(ctx->imu_line);
    if (ctx->mag_line)  release_gpio_line(ctx->mag_line);
    if (ctx->gpio_chip) gpiod_chip_close(ctx->gpio_chip);
    pthread_mutex_destroy(&ctx->shared.lock);
    pthread_mutex_destroy(&ctx->anchor.mtx);
    pthread_mutex_destroy(&ctx->live_lock);
    pthread_mutex_destroy(&ctx->imu_ring.lock);
    pthread_cond_destroy(&ctx->imu_ring.ready);
    pthread_mutex_destroy(&ctx->mag_ring.lock);
    pthread_cond_destroy(&ctx->mag_ring.ready);
    cap_ring_destroy(&ctx->cap_ring);
    free(ctx);
}

/* ── State accessors ─────────────────────────────────────────────────────── */

void imu_get_state(imu_ctx_t *ctx, fused_state_t *state_out,
                   mag_sample_t *mag_out, imu_sample_t *imu_out,
                   imu_sample_t *raw_imu_out)
{
    pthread_mutex_lock(&ctx->shared.lock);
    if (state_out)    *state_out    = ctx->shared.state;
    if (mag_out)      *mag_out      = ctx->shared.latest_mag;
    if (imu_out)      *imu_out      = ctx->latest_imu;
    if (raw_imu_out)  *raw_imu_out  = ctx->raw_imu;
    pthread_mutex_unlock(&ctx->shared.lock);
}

bool imu_ctx_is_settled(imu_ctx_t *ctx)
{
    return atomic_load_explicit(&ctx->settled, memory_order_acquire) != 0;
}

void imu_get_stats(imu_ctx_t *ctx, imu_stats_t *out)
{
    out->imu_samples    = ctx->imu_sample_count;
    out->mag_samples    = ctx->mag_sample_count;
    out->fifo_overflows = ctx->fifo_overflow_count;
    out->imu_errors     = ctx->imu_error_count;
    out->mag_errors     = ctx->mag_error_count;

    out->drains_edge    = atomic_load(&ctx->drains_edge);
    out->drains_timeout = atomic_load(&ctx->drains_timeout);
    out->drain_samples  = atomic_load(&ctx->drain_samples);
    out->drain_max      = atomic_load(&ctx->drain_max);

    out->lat_fifo_p50_ns = ctx->lat_fifo_p50;
    out->lat_fifo_p99_ns = ctx->lat_fifo_p99;
    out->lat_fifo_max_ns = ctx->lat_fifo_max;
    out->lat_pipe_p50_ns = ctx->lat_pipe_p50;
    out->lat_pipe_p99_ns = ctx->lat_pipe_p99;
    out->lat_pipe_max_ns = ctx->lat_pipe_max;
}
