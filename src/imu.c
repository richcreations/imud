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

#include "capture.h"
#include "imu.h"
#include "version.h"
#include "ring.h"
#include "drivers.h"
#include "fusion.h"
#include "log.h"

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

/* ── Calibration helpers ─────────────────────────────────────────────────── */

/*
 * Apply accel cal from cal.json on top of the driver's chip-level scaling.
 * Gyro bias is NOT subtracted here — the MEKF subtracts it during predict.
 */
static void apply_imu_cal(const imud_cal_t *cal, imu_sample_t *s)
{
    /* Gyro bias/temperature compensation (imud-cal fit-temp): remove the
     * temperature-tracking bias component here so the MEKF's random-walk
     * estimator only has to follow the residual. */
    if (cal->has_gyro_temp) {
        float dT = s->temp_c - cal->gyro_temp_ref_c;
        s->gyro[0] -= cal->gyro_temp_coeff[0] * dT;
        s->gyro[1] -= cal->gyro_temp_coeff[1] * dT;
        s->gyro[2] -= cal->gyro_temp_coeff[2] * dT;
    }

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
                LOG_E("[ism_reader] GPIO error: %s\n", strerror(errno));
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
                LOG_E("[ism_reader] 10 consecutive errors — resetting chip\n");
                int rok = (ctx->imu_ops->reset(ctx->i2c_fd, (uint8_t)ctx->cfg.imu_addr) == 0)
                       && (ctx->imu_ops->init (ctx->i2c_fd, (uint8_t)ctx->cfg.imu_addr,
                                               &ctx->imu_hw_cfg) == 0);
                consec_errors = 0;
                if (!rok) {
                    LOG_E("[ism_reader] reset failed (%d/3)\n",
                            ++reset_failures);
                    if (reset_failures >= 3) {
                        LOG_E("[ism_reader] 3 reset failures — raising SIGTERM\n");
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

        /* Black-box tap: raw samples exactly as the driver delivered them
         * (pre-mount, pre-cal) so replay traverses the identical pipeline.
         * One delivery timestamp per burst; per-sample time is chip_ts. */
        if (ctx->cfg.capture_enabled) {
            struct timespec tm;
            clock_gettime(CLOCK_MONOTONIC, &tm);
            uint64_t mono = ts_ns(&tm);
            for (int i = 0; i < n; i++)
                cap_ring_push_imu(&ctx->cap_ring, &buf[i], mono);
        }

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

        int mrc = ctx->mag_ops->read(ctx->i2c_fd, (uint8_t)ctx->cfg.mag_addr, &s);
        if (mrc > 0) continue;   /* no data yet (DRDY not set / playback idle) —
                                  * pushing here would re-fuse a stale sample */
        if (mrc < 0) {
            ctx->mag_error_count++;
            if (++consec_errors >= 10) {
                LOG_E("[mag_reader] 10 consecutive errors — resetting chip\n");
                int rok = (ctx->mag_ops->reset(ctx->i2c_fd, (uint8_t)ctx->cfg.mag_addr) == 0)
                       && (ctx->mag_ops->init (ctx->i2c_fd, (uint8_t)ctx->cfg.mag_addr,
                                               &ctx->mag_hw_cfg) == 0);
                consec_errors = 0;
                if (!rok) {
                    LOG_E("[mag_reader] reset failed (%d/3)\n",
                            ++reset_failures);
                    if (reset_failures >= 3) {
                        LOG_E("[mag_reader] 3 reset failures — raising SIGTERM\n");
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

        /* Black-box tap: raw field as the driver delivered it (pre-mount,
         * pre-cal). */
        if (ctx->cfg.capture_enabled) {
            struct timespec tm;
            clock_gettime(CLOCK_MONOTONIC, &tm);
            cap_ring_push_mag(&ctx->cap_ring, &s, ts_ns(&tm));
        }

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

    cap_rotator_t rot;
    if (cap_rot_open(&rot, ctx->cfg.capture_dir,
                     (uint32_t)(ctx->cfg.capture_max_mb    > 0 ? ctx->cfg.capture_max_mb    : 0),
                     (uint32_t)(ctx->cfg.capture_max_files > 0 ? ctx->cfg.capture_max_files : 0),
                     (uint32_t)ctx->actual_odr_hz,
                     ctx->imu_ops->name, ctx->mag_ops->name,
                     IMUD_VERSION_STR) != 0) {
        LOG_E("[capture] cannot open capture file in %s: %s\n",
              ctx->cfg.capture_dir, strerror(errno));
        return NULL;
    }
    LOG_I("[capture] recording to %s\n", cap_rot_path(&rot));

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t last_flush   = ts_ns(&now);
    uint64_t last_drops   = 0;
    uint64_t flush_ns     = (uint64_t)(ctx->cfg.capture_flush_s > 0
                                       ? ctx->cfg.capture_flush_s : 5)
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

    /* Drain what the readers pushed before stop, then close cleanly. */
    if (!failed) {
        cap_ring_rec_t recs[64];
        int n;
        while ((n = cap_ring_pop(&ctx->cap_ring, recs, 64)) > 0)
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
    float odr_hz = (float)ctx->actual_odr_hz;

    /* ── Phase 0: startup settle — drain stale samples ─────────────────── */

    if (ctx->cfg.startup_settle_sec > 0.0) {
        int n_settle = (int)(ctx->cfg.startup_settle_sec * odr_hz);
        if (n_settle < 1) n_settle = 1;
        LOG_I("[fusion] settling %g s — discarding %d samples\n",
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
                            2.0 * ctx->cfg.gyro_bias_sec);
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
    mekf_init(&f, &ctx->cfg, odr_hz, init_bias);

    heave_t heave;
    heave_init(&heave, ctx->cfg.heave_tau_s, f.dt);
    if (heave.enabled)
        LOG_I("[fusion] heave estimator on (tau=%.0f s)\n",
                (double)ctx->cfg.heave_tau_s);

    /* Sea state rides on heave: wave_tau_s > 0 needs a running heave estimator. */
    seastate_t wave;
    seastate_init(&wave, heave.enabled ? ctx->cfg.wave_tau_s : 0.0f, f.dt);
    if (wave.enabled)
        LOG_I("[fusion] sea-state estimator on (tau=%.0f s)\n",
                (double)ctx->cfg.wave_tau_s);
    else if (ctx->cfg.wave_tau_s > 0.0f)
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
        int    n_avg = (int)odr_hz;      /* ~1 s of IMU samples */
        if (n_avg < 8) n_avg = 8;

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

        /* Keep waiting (up to 5 s total) for at least one mag sample.
         * Drain the IMU ring while waiting: it holds only ~0.3 s at 833 Hz,
         * and an undrained wait counts thousands of ring drops as FIFO
         * overflows. */
        while (mag_n == 0 && !ctx->stop) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (double)(now.tv_sec  - align_start.tv_sec)
                           + (double)(now.tv_nsec - align_start.tv_nsec) * 1e-9;
            if (elapsed > 5.0) break;
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
                LOG_W("[fusion] no mag sample after 5 s; "
                        "aligning without heading\n");
                mag_avg[0] = 1.0f; mag_avg[1] = 0.0f; mag_avg[2] = 0.0f;
            }
            mekf_align(&f, acc_avg, mag_avg);
            /* WMM-known field invariants remove the residual alignment-tilt
             * error from the magnetic reference at the source. */
            if (ctx->cfg.pos_mref_valid)
                mekf_set_mref_invariants(&f, ctx->cfg.pos_mref_h_gauss,
                                         ctx->cfg.pos_mref_z_gauss);
            LOG_I("[fusion] aligned from %d accel + %d mag samples%s%s\n",
                    acc_n, mag_n,
                    (mag_n > 0 && mag_uncal)
                        ? " (uncalibrated mag — heading approximate)" : "",
                    ctx->cfg.pos_mref_valid ? " (m_ref invariants from WMM)" : "");
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

        if (ctx->reconfigure) {
            mekf_reconfigure(&f, &ctx->cfg);
            /* heave_tau_s is [hot]: re-derive constants, keep filter state
             * unless toggling between enabled/disabled. */
            if ((ctx->cfg.heave_tau_s > 0.0f) != heave.enabled)
                heave_init(&heave, ctx->cfg.heave_tau_s, f.dt);
            else
                heave.tau = ctx->cfg.heave_tau_s;
            /* wave_tau_s is [hot] too; sea state needs heave running. */
            {
                float wtau = heave.enabled ? ctx->cfg.wave_tau_s : 0.0f;
                if ((wtau > 0.0f) != wave.enabled)
                    seastate_init(&wave, wtau, f.dt);
                else
                    wave.tau = wtau > 0.0f ? wtau : wave.tau;
            }
            ctx->reconfigure = 0;
        }

        /* Live WMM field invariants pushed by the position thread. */
        if (ctx->mref_update) {
            mekf_set_mref_invariants(&f, ctx->cfg.pos_mref_h_gauss,
                                     ctx->cfg.pos_mref_z_gauss);
            ctx->mref_update = 0;
        }

        /* Speed over ground for the centripetal correction (0 = off). */
        f.speed_mps = ctx->cfg.pos_speed_valid ? ctx->cfg.pos_speed_mps : 0.0f;

        /* Timestamps + per-sample dt. For hw-timestamp chips, chip_ts
         * encodes the exact sample time; feeding the measured interval to
         * the predict step keeps oscillator tolerance (a few % on the
         * ISM330's internal clock) from scaling all integrated rotation.
         * Clamped to [0.5×, 2×] nominal so FIFO-overflow gaps and anchor
         * resets fall back to the nominal period. */
        uint64_t wall = 0, tai = 0;
        uint32_t gen  = 0;
        bool have_ts  = (s.chip_ts != 0 || !ctx->imu_ops->has_hw_timestamp);
        float dt      = f.dt;
        if (have_ts) {
            chip_to_wall(&ctx->anchor, s.chip_ts, &wall, &tai, &gen);
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
         * are trusted proportionally less. */
        if (ctx->cfg.engine_vibration_g2 > 0.0 &&
            ctx->engine_on != prev_engine_on) {
            prev_engine_on = ctx->engine_on;
            float sk = ctx->engine_on
                       ? (float)ctx->cfg.engine_accel_skip_thresh
                       : (float)ctx->cfg.accel_skip_thresh;
            f.accel_skip_lo = 1.0f - sk;
            f.accel_skip_hi = 1.0f + sk;
            f.Ra_scale      = ctx->engine_on ? 4.0f : 1.0f;
            LOG_I("[fusion] engine vibration %s — accel_skip=%.2f "
                    "Ra×%.0f\n",
                    ctx->engine_on ? "detected" : "cleared", sk,
                    (double)f.Ra_scale);
        }

        mekf_update_accel(&f, &s);

        /* Extract fused state and fill fields mekf_get_state leaves as stubs. */
        fused_state_t state;
        mekf_get_state(&f, &state, cal_flags);

        state.heave_m    = heave_update(&heave, f.q, s.accel);
        state.heave_rate = -heave.vel;   /* NED down → heave positive up, m/s */
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

        /* Declination validity is an explicit flag, not a 0.0 sentinel, so a
         * WMM-computed 0.0° on the agonic line still yields true heading. */
        if (ctx->cfg.pos_declination_valid) {
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
    cap_ring_init(&ctx->cap_ring);
    pthread_mutex_init(&ctx->shared.lock, NULL);
    pthread_mutex_init(&ctx->anchor.mtx, NULL);

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

    ctx->mag_ops = mag_driver_find(cfg->mag_driver);
    if (!ctx->mag_ops) {
        LOG_E("[imu] unknown mag driver '%s'\n", cfg->mag_driver);
        goto fail;
    }
    if (ctx->mag_ops->experimental)
        LOG_W("[mag] WARNING: driver '%s' is EXPERIMENTAL — "
                "not yet validated on hardware\n", cfg->mag_driver);

    ctx->actual_odr_hz = nearest_odr(ctx->imu_ops->supported_odr_hz, cfg->imu_odr_hz);

    /* ── Open I2C bus ────────────────────────────────────────────────────── */

    ctx->i2c_fd = open(cfg->i2c_bus, O_RDWR);
    if (ctx->i2c_fd < 0) {
        LOG_E("[imu] cannot open %s: %s\n", cfg->i2c_bus, strerror(errno));
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
        LOG_E("[imu] %s probe failed at 0x%02X\n",
                ctx->imu_ops->name, cfg->imu_addr);
        goto fail;
    }
    if (ctx->imu_ops->reset(ctx->i2c_fd, (uint8_t)cfg->imu_addr) < 0) {
        LOG_E("[imu] %s reset failed\n", ctx->imu_ops->name);
        goto fail;
    }
    if (ctx->imu_ops->init(ctx->i2c_fd, (uint8_t)cfg->imu_addr, &ctx->imu_hw_cfg) < 0) {
        LOG_E("[imu] %s init failed\n", ctx->imu_ops->name);
        goto fail;
    }
    LOG_I("[imu] %s 0x%02X OK — actual ODR %d Hz\n",
            ctx->imu_ops->name, cfg->imu_addr, ctx->actual_odr_hz);

    /* ── Probe + reset + init mag ────────────────────────────────────────── */

    if (ctx->mag_ops->probe(ctx->i2c_fd, (uint8_t)cfg->mag_addr) < 0) {
        LOG_E("[imu] %s probe failed at 0x%02X\n",
                ctx->mag_ops->name, cfg->mag_addr);
        goto fail;
    }
    if (ctx->mag_ops->reset(ctx->i2c_fd, (uint8_t)cfg->mag_addr) < 0) {
        LOG_E("[imu] %s reset failed\n", ctx->mag_ops->name);
        goto fail;
    }
    if (ctx->mag_ops->init(ctx->i2c_fd, (uint8_t)cfg->mag_addr, &ctx->mag_hw_cfg) < 0) {
        LOG_E("[imu] %s init failed\n", ctx->mag_ops->name);
        goto fail;
    }
    LOG_I("[imu] %s 0x%02X OK\n", ctx->mag_ops->name, cfg->mag_addr);

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
        ctx->imu_line = open_gpio_line(ctx->gpio_chip, (unsigned)cfg->imu_int_gpio);
        if (!ctx->imu_line) {
            LOG_E("[imu] cannot request GPIO%d: %s\n",
                    cfg->imu_int_gpio, strerror(errno));
            goto fail;
        }
    }

    /* Mag interrupt line — only requested when the driver has an external pin. */
    if (ctx->mag_ops->has_interrupt && cfg->mag_int_gpio > 0) {
        ctx->mag_line = open_gpio_line(ctx->gpio_chip, (unsigned)cfg->mag_int_gpio);
        if (!ctx->mag_line) {
            LOG_E("[imu] cannot request GPIO%d: %s\n",
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
}

void imu_ctx_set_mag_ref(imu_ctx_t *ctx, float h_gauss, float z_gauss)
{
    /* Same single-word lockless pattern as imu_ctx_set_declination:
     * values first, then the flag the fusion loop polls. */
    ctx->cfg.pos_mref_h_gauss = h_gauss;
    ctx->cfg.pos_mref_z_gauss = z_gauss;
    ctx->cfg.pos_mref_valid   = true;
    ctx->mref_update = 1;
}

void imu_ctx_set_speed(imu_ctx_t *ctx, float speed_mps, bool valid)
{
    ctx->cfg.pos_speed_mps   = speed_mps;
    ctx->cfg.pos_speed_valid = valid;
}

void imu_ctx_set_declination(imu_ctx_t *ctx, float decl_deg, bool valid)
{
    /*
     * Two single-word writes — each naturally atomic on ARM and x86.
     * The fusion thread reads them each predict step; the new value takes
     * effect within one IMU sample period (~1.2 ms at 833 Hz).  Value is
     * written before the flag when validating (and the flag first when
     * clearing) so a torn pair can at worst apply one stale-but-sane sample.
     * Follows the same lockless pattern as imu_ctx_update_config().
     */
    if (valid) {
        ctx->cfg.pos_declination_deg   = decl_deg;
        ctx->cfg.pos_declination_valid = true;
    } else {
        ctx->cfg.pos_declination_valid = false;
        ctx->cfg.pos_declination_deg   = decl_deg;
    }
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
