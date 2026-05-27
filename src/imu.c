/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu.c — reader threads, ring buffers, and fusion thread (§3)
 *
 * Thread model:
 *   ism_reader  wakes on GPIO FIFO-watermark edge (10 ms fallback), drains
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
typedef struct gpiod_line_request imu_gpio_line_t;
#else
typedef struct gpiod_line         imu_gpio_line_t;
#endif

/* CLOCK_TAI is Linux ≥ 3.10; fall back to CLOCK_REALTIME on other platforms. */
#ifndef CLOCK_TAI
#define CLOCK_TAI CLOCK_REALTIME
#endif

#include "imu.h"
#include "ring.h"
#include "drivers.h"
#include "fusion.h"

/* ── Timestamp anchor ───────────────────────────────────────────────────── */

typedef struct {
    uint64_t        wall_ns;    /* CLOCK_REALTIME at anchor point, ns */
    uint64_t        tai_ns;     /* CLOCK_TAI at anchor point, ns */
    uint32_t        chip_ticks; /* IMU hardware counter at anchor point */
    uint32_t        gen;        /* incremented on each update */
    pthread_mutex_t mtx;
} ts_anchor_t;

/* ── Full context (opaque to callers via imu.h) ─────────────────────────── */

struct imu_ctx {
    imud_config_t    cfg;              /* copy at open; hot-reload fields updated via imu_ctx_update_config */
    imud_cal_t       cal;              /* copy at open, read-only afterwards */

    int              i2c_fd;
    const imu_ops_t *imu_ops;
    const mag_ops_t *mag_ops;
    imu_cfg_t        imu_hw_cfg;
    mag_cfg_t        mag_hw_cfg;
    int              actual_odr_hz;    /* nearest supported to cfg.imu_odr_hz */

    struct gpiod_chip  *gpio_chip;
    imu_gpio_line_t    *imu_line;   /* GPIO for IMU FIFO watermark interrupt */
    imu_gpio_line_t    *mag_line;   /* GPIO for magnetometer measurement-done interrupt */

    imu_ring_t       imu_ring;
    mag_ring_t       mag_ring;
    shared_state_t   shared;
    ts_anchor_t      anchor;

    /* IMU samples for the binary output packet, written by fusion_thread.
     * latest_imu: calibrated accel + bias-corrected gyro (fused output).
     * raw_imu:    pre-calibration accel (accel_raw[]) + pre-bias gyro[].
     * Protected by shared.lock (same lock as shared.state). */
    imu_sample_t     latest_imu;
    imu_sample_t     raw_imu;

    /* Stats — each field written by exactly one thread; no extra lock needed. */
    uint64_t         imu_sample_count;
    uint64_t         mag_sample_count;
    uint64_t         fifo_overflow_count;
    uint64_t         imu_error_count;
    uint64_t         mag_error_count;

    float            vib_ema;       /* EMA of (|a|-g)² for engine detection, ism_reader only */

    volatile int     stop;
    _Atomic int      settled;      /* release-written by fusion_thread at Phase 4 entry;
                                    * acquire-read by output/health threads to suppress
                                    * output until settle + bias estimation + alignment done */
    volatile int     reconfigure;  /* set by main on SIGHUP; cleared by fusion_thread */
    volatile int     mag_set_flag; /* set by mag_reader after SET pulse; cleared by fusion_thread */
    volatile int     engine_on;    /* set by ism_reader when vibration EMA exceeds threshold */
};

/* ── Calibration helpers ─────────────────────────────────────────────────── */

/*
 * Apply accel cal from cal.json on top of the driver's chip-level scaling.
 * Gyro bias is NOT subtracted here — the MEKF subtracts it during predict.
 */
static void apply_imu_cal(const imud_cal_t *cal, imu_sample_t *s)
{
    if (!cal->has_accel) return;
    for (int i = 0; i < 3; i++)
        s->accel[i] = (s->accel[i] - cal->accel_offset[i]) * cal->accel_scale[i];
}

/* Apply hard/soft-iron correction: m_cal = soft_iron × (m_raw − hard_iron). */
static void apply_mag_cal(const imud_cal_t *cal, mag_sample_t *s)
{
    if (!cal->has_mag) return;
    float tmp[3];
    for (int i = 0; i < 3; i++)
        tmp[i] = s->field[i] - cal->mag_hard_iron[i];
    for (int i = 0; i < 3; i++)
        s->field[i] = cal->mag_soft_iron[i][0] * tmp[0]
                    + cal->mag_soft_iron[i][1] * tmp[1]
                    + cal->mag_soft_iron[i][2] * tmp[2];
}

/* ── Timestamp helpers ───────────────────────────────────────────────────── */

static uint64_t ts_ns(const struct timespec *t)
{
    return (uint64_t)t->tv_sec * 1000000000ULL + (uint64_t)t->tv_nsec;
}

static void anchor_update(ts_anchor_t *a, uint32_t chip_ts,
                          uint64_t wall_ns, uint64_t tai_ns)
{
    pthread_mutex_lock(&a->mtx);
    a->chip_ticks = chip_ts;
    a->wall_ns    = wall_ns;
    a->tai_ns     = tai_ns;
    a->gen++;
    pthread_mutex_unlock(&a->mtx);
}

/*
 * Convert a chip counter value to wall + TAI timestamps.
 * Uses 32-bit wrapping arithmetic — safe up to ~29.8 h between anchors.
 */
static void chip_to_wall(ts_anchor_t *a, uint32_t chip_ts,
                         uint64_t *wall_out, uint64_t *tai_out,
                         uint32_t *gen_out)
{
    pthread_mutex_lock(&a->mtx);
    uint64_t offset = (uint64_t)(chip_ts - a->chip_ticks) * 25000ULL; /* 25 µs → ns */
    if (wall_out) *wall_out = a->wall_ns + offset;
    if (tai_out)  *tai_out  = a->tai_ns  + offset;
    if (gen_out)  *gen_out  = a->gen;
    pthread_mutex_unlock(&a->mtx);
}

/* ── Utilities ───────────────────────────────────────────────────────────── */

static int nearest_odr(const int supported[], int requested)
{
    int best = supported[0], best_diff = abs(supported[0] - requested);
    for (int i = 1; supported[i] != 0; i++) {
        int d = abs(supported[i] - requested);
        if (d < best_diff) { best = supported[i]; best_diff = d; }
    }
    return best;
}

/* Apply mount rotation (board -> body) if configured. In-place on v. */
static void apply_mount_rot_if_set(const imud_config_t *cfg, float v[3])
{
    if (!cfg->mount_set) return;
    double out0 = cfg->mount_rot[0][0] * v[0]
                + cfg->mount_rot[0][1] * v[1]
                + cfg->mount_rot[0][2] * v[2];
    double out1 = cfg->mount_rot[1][0] * v[0]
                + cfg->mount_rot[1][1] * v[1]
                + cfg->mount_rot[1][2] * v[2];
    double out2 = cfg->mount_rot[2][0] * v[0]
                + cfg->mount_rot[2][1] * v[1]
                + cfg->mount_rot[2][2] * v[2];
    v[0] = (float)out0; v[1] = (float)out1; v[2] = (float)out2;
}

/*
 * wait_gpio_edge — wait up to timeout_ms ms for a rising-edge event.
 * Drains the event so the next call sees a fresh edge.
 * Returns 1 on event, 0 on timeout, -1 on error.
 */
static int wait_gpio_edge(imu_gpio_line_t *line, long timeout_ms)
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
static imu_gpio_line_t *open_gpio_line(struct gpiod_chip *chip,
                                        unsigned int offset)
{
#ifdef GPIOD_V2
    struct gpiod_line_settings  *ls = gpiod_line_settings_new();
    struct gpiod_line_config    *lc = gpiod_line_config_new();
    struct gpiod_request_config *rc = gpiod_request_config_new();
    if (!ls || !lc || !rc) goto fail;
    gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(ls, GPIOD_LINE_EDGE_RISING);
    gpiod_line_config_add_line_settings(lc, &offset, 1, ls);
    gpiod_request_config_set_consumer(rc, "imud");
    imu_gpio_line_t *req = gpiod_chip_request_lines(chip, rc, lc);
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
    imu_gpio_line_t *line = gpiod_chip_get_line(chip, offset);
    if (!line) return NULL;
    if (gpiod_line_request_rising_edge_events(line, "imud") < 0)
        return NULL;
    return line;
#endif
}

static void release_gpio_line(imu_gpio_line_t *line)
{
#ifdef GPIOD_V2
    gpiod_line_request_release(line);
#else
    gpiod_line_release(line);
#endif
}

/* ── ism_reader_thread ───────────────────────────────────────────────────── */

void *ism_reader_thread(void *arg)
{
    imu_ctx_t *ctx = arg;
    imu_sample_t buf[128];
    int n;
    int consec_errors = 0;
    int reset_failures = 0;
    bool anchor_valid = false;
    struct timespec anchor_last = {0, 0};
    while (!ctx->stop) {
        if (ctx->imu_line) {
            int gr = wait_gpio_edge(ctx->imu_line, 10);
            if (gr < 0) {
                if (ctx->stop) break;
                fprintf(stderr, "[ism_reader] GPIO error: %s\n", strerror(errno));
                usleep(10000);
                continue;
            }
            /* gr == 0: 10 ms timeout — fall through to read anyway */
        } else {
            /* No interrupt line: pace by sleeping 10 ms between FIFO drains. */
            struct timespec t = { .tv_sec = 0, .tv_nsec = 10 * 1000000L };
            nanosleep(&t, NULL);
            if (ctx->stop) break;
        }


        struct timespec t_before, t_after, t_tai;
        clock_gettime(CLOCK_REALTIME, &t_before);

        int rc = ctx->imu_ops->read(ctx->i2c_fd, (uint8_t)ctx->cfg.imu_addr,
                                    buf, 128, &n);

        clock_gettime(CLOCK_REALTIME, &t_after);
        clock_gettime(CLOCK_TAI,      &t_tai);

        if (rc < 0) {
            ctx->imu_error_count++;
            if (++consec_errors >= 10) {
                fprintf(stderr, "[ism_reader] 10 consecutive errors — resetting chip\n");
                int rok = (ctx->imu_ops->reset(ctx->i2c_fd, (uint8_t)ctx->cfg.imu_addr) == 0)
                       && (ctx->imu_ops->init (ctx->i2c_fd, (uint8_t)ctx->cfg.imu_addr,
                                               &ctx->imu_hw_cfg) == 0);
                consec_errors = 0;
                if (!rok) {
                    fprintf(stderr, "[ism_reader] reset failed (%d/3)\n",
                            ++reset_failures);
                    if (reset_failures >= 3) {
                        fprintf(stderr, "[ism_reader] 3 reset failures — raising SIGTERM\n");
                        raise(SIGTERM);
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
        if (n == 0) continue;

        /* Apply mount rotation (board->body), then accel calibration */
        for (int i = 0; i < n; i++) {
            apply_mount_rot_if_set(&ctx->cfg, buf[i].accel);
            apply_mount_rot_if_set(&ctx->cfg, buf[i].gyro);
            buf[i].accel_raw[0] = buf[i].accel[0];
            buf[i].accel_raw[1] = buf[i].accel[1];
            buf[i].accel_raw[2] = buf[i].accel[2];
            apply_imu_cal(&ctx->cal, &buf[i]);
        }

        /* Update timestamp anchor at startup and every 60 s thereafter.
         * Midpoint of the I2C read is paired with the newest sample's chip_ts. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = anchor_valid
            ? (double)(now.tv_sec  - anchor_last.tv_sec)
              + (double)(now.tv_nsec - anchor_last.tv_nsec) * 1e-9
            : 1e9;

        /* Update anchor:
         *   - Always on first read.
         *   - Every 60 s when hw timestamp is available (chip counter drift).
         *   - Every burst when no hw timestamp (chip_ts=0 always, so anchor
         *     IS the per-sample timestamp; must stay current). */
        bool do_anchor = !anchor_valid
                         || !ctx->imu_ops->has_hw_timestamp
                         || elapsed >= 60.0;
        if (do_anchor) {
            uint64_t wall_mid = (ts_ns(&t_before) + ts_ns(&t_after)) / 2;
            anchor_update(&ctx->anchor, buf[n - 1].chip_ts, wall_mid, ts_ns(&t_tai));
            anchor_last  = now;
            anchor_valid = true;
        }

        /* Engine vibration detection: exponential moving average of (|a|-g)².
         * Alpha = 0.01 gives a ~1 s time constant at 100 Hz burst rate. */
        if (ctx->cfg.engine_vibration_g2 > 0.0 && n > 0) {
            const float alpha = 0.01f;
            const float g = 9.80665f;
            for (int i = 0; i < n; i++) {
                float a = sqrtf(buf[i].accel[0]*buf[i].accel[0]
                              + buf[i].accel[1]*buf[i].accel[1]
                              + buf[i].accel[2]*buf[i].accel[2]);
                float dev = a - g;
                ctx->vib_ema += alpha * (dev*dev - ctx->vib_ema);
            }
            ctx->engine_on = (ctx->vib_ema > (float)ctx->cfg.engine_vibration_g2);
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
    clock_gettime(CLOCK_MONOTONIC, &last_set);
    while (!ctx->stop) {
        if (ctx->mag_line) {
            /* Interrupt-driven: wait for GPIO edge, 20 ms fallback. */
            int gr = wait_gpio_edge(ctx->mag_line, 20);
            if (gr < 0) {
                if (ctx->stop) break;
                usleep(5000);
                continue;
            }
            /* gr == 0: 20 ms timeout — fall through */
        } else {
            /* No interrupt pin: pace at ~100 Hz.  The driver read() polls
             * DRDY internally so we won't process stale data. */
            struct timespec t = { .tv_sec = 0, .tv_nsec = 10 * 1000000L };
            nanosleep(&t, NULL);
            if (ctx->stop) break;
        }

        /* Periodic SET/RESET (degauss) — skip this read cycle; wait for next edge. */
        if (ctx->cfg.mag_set_period_s > 0.0f && ctx->mag_ops->set_reset) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (double)(now.tv_sec  - last_set.tv_sec)
                           + (double)(now.tv_nsec - last_set.tv_nsec) * 1e-9;
            if (elapsed >= (double)ctx->cfg.mag_set_period_s) {
                ctx->mag_ops->set_reset(ctx->i2c_fd, (uint8_t)ctx->cfg.mag_addr);
                ctx->mag_set_flag = 1;
                last_set = now;
                usleep(1000); /* 1 ms settling — no read this cycle */
                continue;
            }
        }

        if (ctx->mag_ops->read(ctx->i2c_fd, (uint8_t)ctx->cfg.mag_addr, &s) < 0) {
            ctx->mag_error_count++;
            if (++consec_errors >= 10) {
                fprintf(stderr, "[mag_reader] 10 consecutive errors — resetting chip\n");
                int rok = (ctx->mag_ops->reset(ctx->i2c_fd, (uint8_t)ctx->cfg.mag_addr) == 0)
                       && (ctx->mag_ops->init (ctx->i2c_fd, (uint8_t)ctx->cfg.mag_addr,
                                               &ctx->mag_hw_cfg) == 0);
                consec_errors = 0;
                if (!rok) {
                    fprintf(stderr, "[mag_reader] reset failed (%d/3)\n",
                            ++reset_failures);
                    if (reset_failures >= 3) {
                        fprintf(stderr, "[mag_reader] 3 reset failures — raising SIGTERM\n");
                        raise(SIGTERM);
                        break;
                    }
                } else {
                    reset_failures = 0;
                }
            }
            continue;
        }
        consec_errors = 0;

        /* Rotate mag into body frame, then apply mag calibration */
        apply_mount_rot_if_set(&ctx->cfg, s.field);
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

/* ── fusion_thread ───────────────────────────────────────────────────────── */

void *fusion_thread(void *arg)
{
    imu_ctx_t *ctx = arg;
    float odr_hz = (float)ctx->actual_odr_hz;

    /* ── Phase 0: startup settle — drain stale samples ─────────────────── */

    if (ctx->cfg.startup_settle_sec > 0.0) {
        int n_settle = (int)(ctx->cfg.startup_settle_sec * odr_hz);
        if (n_settle < 1) n_settle = 1;
        fprintf(stderr, "[fusion] settling %g s — discarding %d samples\n",
                ctx->cfg.startup_settle_sec, n_settle);
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
    } else if (ctx->cfg.gyro_bias_sec > 0.0) {
        int n_needed = (int)(ctx->cfg.gyro_bias_sec * odr_hz);
        if (n_needed < 1) n_needed = 1;

        float acc[3] = {0.0f, 0.0f, 0.0f};
        int count = 0;
        imu_sample_t s;

        fprintf(stderr, "[fusion] gyro bias estimation: collecting %d samples\n",
                n_needed);

        while (count < n_needed && !ctx->stop) {
            if (imu_ring_pop(&ctx->imu_ring, &s, &ctx->stop) == 0) {
                acc[0] += s.gyro[0];
                acc[1] += s.gyro[1];
                acc[2] += s.gyro[2];
                count++;
            }
        }
        if (count > 0) {
            init_bias[0] = acc[0] / count;
            init_bias[1] = acc[1] / count;
            init_bias[2] = acc[2] / count;
            cal_flags |= FLAG_GYRO_CAL;
            fprintf(stderr, "[fusion] gyro bias = [%.5f, %.5f, %.5f] rad/s\n",
                    init_bias[0], init_bias[1], init_bias[2]);
        }
    }

    if (ctx->stop) return NULL;

    /* ── Phase 2: MEKF init ───────────────────────────────────────────────── */

    mekf_t f;
    mekf_init(&f, &ctx->cfg, odr_hz, init_bias);

    /* ── Phase 3: initial alignment (tilt from accel, heading from mag) ───── */

    {
        imu_sample_t isample;  memset(&isample, 0, sizeof(isample));
        mag_sample_t msample;  memset(&msample, 0, sizeof(msample));
        bool have_imu = false, have_mag = false;

        struct timespec align_start;
        clock_gettime(CLOCK_MONOTONIC, &align_start);

        while ((!have_imu || !have_mag) && !ctx->stop) {
            if (!have_imu && imu_ring_pop(&ctx->imu_ring, &isample, &ctx->stop) == 0)
                have_imu = true;
            if (!have_mag && mag_ring_try_pop(&ctx->mag_ring, &msample) == 0)
                have_mag = true;

            if (!have_mag) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                double elapsed = (double)(now.tv_sec  - align_start.tv_sec)
                               + (double)(now.tv_nsec - align_start.tv_nsec) * 1e-9;
                if (elapsed > 5.0 && have_imu) {
                    /* Timeout: align without heading (assume forward = North). */
                    fprintf(stderr, "[fusion] no mag sample after 5 s; "
                            "aligning without heading\n");
                    msample.field[0] = 1.0f;
                    msample.field[1] = 0.0f;
                    msample.field[2] = 0.0f;
                    have_mag = true;
                } else {
                    usleep(1000);
                }
            }
        }

        if (!ctx->stop)
            mekf_align(&f, isample.accel, msample.field);
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

    while (!ctx->stop) {
        if (imu_ring_pop(&ctx->imu_ring, &s, &ctx->stop) != 0)
            continue;

        if (ctx->reconfigure) {
            mekf_reconfigure(&f, &ctx->cfg);
            ctx->reconfigure = 0;
        }

        mekf_predict(&f, &s);

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

        /* Switch accel skip threshold when engine vibration is detected. */
        if (ctx->cfg.engine_vibration_g2 > 0.0 &&
            ctx->engine_on != prev_engine_on) {
            prev_engine_on = ctx->engine_on;
            float sk = ctx->engine_on
                       ? (float)ctx->cfg.engine_accel_skip_thresh
                       : (float)ctx->cfg.accel_skip_thresh;
            f.accel_skip_lo = 1.0f - sk;
            f.accel_skip_hi = 1.0f + sk;
            fprintf(stderr, "[fusion] engine vibration %s — accel_skip=%.2f\n",
                    ctx->engine_on ? "detected" : "cleared", sk);
        }

        mekf_update_accel(&f, &s);

        /* Extract fused state and fill fields mekf_get_state leaves as stubs. */
        fused_state_t state;
        mekf_get_state(&f, &state, cal_flags);

        if (mag_healthy)
            state.flags |= FLAG_MAG_VALID;

        if (ctx->mag_set_flag) {
            state.flags |= FLAG_MAG_SET_RESET;
            ctx->mag_set_flag = 0;
        }

        /* rate_of_turn: bias-corrected yaw rate (NED Z), rad/s → deg/min */
        state.rate_of_turn = (s.gyro[2] - f.bias[2])
                             * (float)(180.0 / M_PI) * 60.0f;

        /* For hw-timestamp chips: chip_ts encodes exact sample time.
         * For chips without hw timestamp: chip_ts=0 always; anchor_wall_ns
         * is updated per-burst so it tracks current time adequately. */
        if (s.chip_ts != 0 || !ctx->imu_ops->has_hw_timestamp) {
            uint64_t wall, tai;
            uint32_t gen;
            chip_to_wall(&ctx->anchor, s.chip_ts, &wall, &tai, &gen);
            state.ts_wall_ns    = wall;
            state.ts_tai_ns     = tai;
            state.ts_chip_ticks = s.chip_ts;
            state.anchor_gen    = gen;
        }
        state.imu_seq = s.seq;

        /* Static declination: opt-in; zero means feature disabled. */
        if (ctx->cfg.pos_declination_deg != 0.0f) {
            state.declination_deg = ctx->cfg.pos_declination_deg;
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
    if (!ctx) { perror("calloc"); return -1; }
    ctx->i2c_fd = -1;   /* calloc zeros to 0; must be -1 so fail: doesn't close stdin */

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
    pthread_mutex_init(&ctx->shared.lock, NULL);
    pthread_mutex_init(&ctx->anchor.mtx, NULL);

    /* ── Locate drivers ──────────────────────────────────────────────────── */

    ctx->imu_ops = imu_driver_find(cfg->imu_driver);
    if (!ctx->imu_ops) {
        fprintf(stderr, "[imu] unknown IMU driver '%s'\n", cfg->imu_driver);
        goto fail;
    }
    ctx->mag_ops = mag_driver_find(cfg->mag_driver);
    if (!ctx->mag_ops) {
        fprintf(stderr, "[imu] unknown mag driver '%s'\n", cfg->mag_driver);
        goto fail;
    }

    ctx->actual_odr_hz = nearest_odr(ctx->imu_ops->supported_odr_hz, cfg->imu_odr_hz);

    /* ── Open I2C bus ────────────────────────────────────────────────────── */

    ctx->i2c_fd = open(cfg->i2c_bus, O_RDWR);
    if (ctx->i2c_fd < 0) {
        fprintf(stderr, "[imu] cannot open %s: %s\n", cfg->i2c_bus, strerror(errno));
        goto fail;
    }

    /* ── Build hw config structs ─────────────────────────────────────────── */

    ctx->imu_hw_cfg.odr_hz   = cfg->imu_odr_hz;
    ctx->imu_hw_cfg.accel_g  = cfg->imu_accel_g;
    ctx->imu_hw_cfg.gyro_dps = cfg->imu_gyro_dps;
    ctx->imu_hw_cfg.fifo_wm  = cfg->imu_fifo_wm;

    ctx->mag_hw_cfg.odr_hz       = cfg->mag_odr_hz;
    ctx->mag_hw_cfg.set_period_s = cfg->mag_set_period_s;

    /* ── Probe + reset + init IMU ────────────────────────────────────────── */

    if (ctx->imu_ops->probe(ctx->i2c_fd, (uint8_t)cfg->imu_addr) < 0) {
        fprintf(stderr, "[imu] %s probe failed at 0x%02X\n",
                ctx->imu_ops->name, cfg->imu_addr);
        goto fail;
    }
    if (ctx->imu_ops->reset(ctx->i2c_fd, (uint8_t)cfg->imu_addr) < 0) {
        fprintf(stderr, "[imu] %s reset failed\n", ctx->imu_ops->name);
        goto fail;
    }
    if (ctx->imu_ops->init(ctx->i2c_fd, (uint8_t)cfg->imu_addr, &ctx->imu_hw_cfg) < 0) {
        fprintf(stderr, "[imu] %s init failed\n", ctx->imu_ops->name);
        goto fail;
    }
    fprintf(stderr, "[imu] %s 0x%02X OK — actual ODR %d Hz\n",
            ctx->imu_ops->name, cfg->imu_addr, ctx->actual_odr_hz);

    /* ── Probe + reset + init mag ────────────────────────────────────────── */

    if (ctx->mag_ops->probe(ctx->i2c_fd, (uint8_t)cfg->mag_addr) < 0) {
        fprintf(stderr, "[imu] %s probe failed at 0x%02X\n",
                ctx->mag_ops->name, cfg->mag_addr);
        goto fail;
    }
    if (ctx->mag_ops->reset(ctx->i2c_fd, (uint8_t)cfg->mag_addr) < 0) {
        fprintf(stderr, "[imu] %s reset failed\n", ctx->mag_ops->name);
        goto fail;
    }
    if (ctx->mag_ops->init(ctx->i2c_fd, (uint8_t)cfg->mag_addr, &ctx->mag_hw_cfg) < 0) {
        fprintf(stderr, "[imu] %s init failed\n", ctx->mag_ops->name);
        goto fail;
    }
    fprintf(stderr, "[imu] %s 0x%02X OK\n", ctx->mag_ops->name, cfg->mag_addr);

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
            fprintf(stderr, "[imu] cannot open /dev/%s: %s\n",
                    cfg->gpio_chip, strerror(errno));
            goto fail;
        }
    }

    /* IMU interrupt line — optional (timer fallback used when absent). */
    if (cfg->imu_int_gpio > 0) {
        ctx->imu_line = open_gpio_line(ctx->gpio_chip, (unsigned)cfg->imu_int_gpio);
        if (!ctx->imu_line) {
            fprintf(stderr, "[imu] cannot request GPIO%d: %s\n",
                    cfg->imu_int_gpio, strerror(errno));
            goto fail;
        }
    }

    /* Mag interrupt line — only requested when the driver has an external pin. */
    if (ctx->mag_ops->has_interrupt && cfg->mag_int_gpio > 0) {
        ctx->mag_line = open_gpio_line(ctx->gpio_chip, (unsigned)cfg->mag_int_gpio);
        if (!ctx->mag_line) {
            fprintf(stderr, "[imu] cannot request GPIO%d: %s\n",
                    cfg->mag_int_gpio, strerror(errno));
            goto fail;
        }
    }

    *ctx_out = ctx;
    return 0;

fail:
    if (ctx->i2c_fd >= 0)  close(ctx->i2c_fd);
    if (ctx->imu_line)     release_gpio_line(ctx->imu_line);
    if (ctx->mag_line)     release_gpio_line(ctx->mag_line);
    if (ctx->gpio_chip)    gpiod_chip_close(ctx->gpio_chip);
    pthread_mutex_destroy(&ctx->shared.lock);
    pthread_mutex_destroy(&ctx->anchor.mtx);
    pthread_mutex_destroy(&ctx->imu_ring.lock);
    pthread_cond_destroy(&ctx->imu_ring.ready);
    pthread_mutex_destroy(&ctx->mag_ring.lock);
    pthread_cond_destroy(&ctx->mag_ring.ready);
    free(ctx);
    return -1;
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void imu_ctx_update_config(imu_ctx_t *ctx, const imud_config_t *new_cfg)
{
    ctx->cfg.mekf_gyro_noise           = new_cfg->mekf_gyro_noise;
    ctx->cfg.mekf_gyro_bias            = new_cfg->mekf_gyro_bias;
    ctx->cfg.mekf_accel_noise          = new_cfg->mekf_accel_noise;
    ctx->cfg.mekf_mag_noise            = new_cfg->mekf_mag_noise;
    ctx->cfg.mag_reject_gauss          = new_cfg->mag_reject_gauss;
    ctx->cfg.accel_skip_thresh         = new_cfg->accel_skip_thresh;
    ctx->cfg.engine_vibration_g2       = new_cfg->engine_vibration_g2;
    ctx->cfg.engine_accel_skip_thresh  = new_cfg->engine_accel_skip_thresh;
    ctx->cfg.pos_declination_deg       = new_cfg->pos_declination_deg;
    ctx->reconfigure = 1;
}

void imu_ctx_set_declination(imu_ctx_t *ctx, float decl_deg)
{
    /*
     * Write a single aligned float — naturally atomic on ARM and x86.
     * The fusion thread reads pos_declination_deg each predict step; the
     * new value takes effect within one IMU sample period (~1.2 ms at 833 Hz).
     * Follows the same lockless pattern as imu_ctx_update_config().
     */
    ctx->cfg.pos_declination_deg = decl_deg;
}

void imu_ctx_stop(imu_ctx_t *ctx)
{
    ctx->stop = 1;
    /* Wake threads blocked in imu_ring_pop. */
    pthread_cond_broadcast(&ctx->imu_ring.ready);
}

void imu_ctx_free(imu_ctx_t *ctx)
{
    if (!ctx) return;
    close(ctx->i2c_fd);
    if (ctx->imu_line)  release_gpio_line(ctx->imu_line);
    if (ctx->mag_line)  release_gpio_line(ctx->mag_line);
    if (ctx->gpio_chip) gpiod_chip_close(ctx->gpio_chip);
    pthread_mutex_destroy(&ctx->shared.lock);
    pthread_mutex_destroy(&ctx->anchor.mtx);
    pthread_mutex_destroy(&ctx->imu_ring.lock);
    pthread_cond_destroy(&ctx->imu_ring.ready);
    pthread_mutex_destroy(&ctx->mag_ring.lock);
    pthread_cond_destroy(&ctx->mag_ring.ready);
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
}
