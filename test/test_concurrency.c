/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_concurrency.c — drives the real daemon threads under concurrent
 * config reload and stream shutdown, so ThreadSanitizer can prove the
 * hot-reload and shutdown paths are C11-race-free (the 1.5.1 race fixes).
 *
 * The normal unit suite never runs the reader/fusion/output threads while
 * another thread mutates config, which is exactly why the data races went
 * unseen. This test closes that gap. It uses the sim driver (no hardware):
 * i2c_bus = /dev/null and int_gpio = 0, so the I2C fd is never touched and
 * no GPIO lines are opened.
 *
 * Build/run under TSan in CI's tsan job (make test with -fsanitize=thread).
 * Links the daemon objects, like test_ring, and runs anywhere they build —
 * configure picks the bus, GPIO and clock backends per host.
 */

#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <math.h>

#include "capture.h"
#include "config.h"
#include "imu.h"
#include "output.h"

static int g_pass, g_fail;
#define EXPECT(c, m) do { if (c) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s\n", (m)); } } while (0)

static double now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

static void sim_cfg(imud_config_t *cfg)
{
    config_defaults(cfg);
    snprintf(cfg->imu_driver, sizeof cfg->imu_driver, "sim");
    snprintf(cfg->mag_driver, sizeof cfg->mag_driver, "sim");
    snprintf(cfg->i2c_bus,    sizeof cfg->i2c_bus,    "/dev/null");
    cfg->imu_int_gpio = 0;
    cfg->mag_int_gpio = 0;
    cfg->imu_odr_mhz  = 200000;
    cfg->mag_odr_mhz  = 50000;
    cfg->startup_settle_sec = 0.0;   /* settle fast so the test is quick */
    cfg->gyro_bias_sec      = 0.0;
    cfg->nmea_enabled     = false;
    cfg->highrate_enabled = false;
    cfg->stream_enabled   = false;
}

/* ── Part A: config-reload storm against the fusion/reader threads ────────── */

typedef struct { imu_ctx_t *imu; imud_config_t base; _Atomic int stop; } hammer_t;

static void *hammer_fn(void *arg)
{
    hammer_t *h = arg;
    unsigned x = 2463534242u;
    while (!atomic_load(&h->stop)) {
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        float r = (float)(x & 0xffff) / 65535.0f;
        imu_ctx_set_declination(h->imu, r * 30.0f - 15.0f, (x & 1));
        imu_ctx_set_speed(h->imu, r * 10.0f, (x & 2) != 0);
        imu_ctx_set_mag_ref(h->imu, 0.18f + r * 0.06f, 0.38f + r * 0.06f);
        imud_config_t c = h->base;
        c.mekf_gyro_noise     = 0.005 + r * 0.004;
        c.mekf_accel_noise    = 0.002 + r * 0.001;
        c.engine_vibration_g2 = (x & 4) ? 0.02 : 0.0;
        c.heave_tau_s         = 8.0f + r * 8.0f;
        c.wave_tau_s          = 40.0f + r * 40.0f;
        imu_ctx_update_config(h->imu, &c);
    }
    return NULL;
}

static void test_reload_race(void)
{
    printf("%-40s", "test_reload_race");
    int fb = g_fail;

    imud_config_t cfg;
    sim_cfg(&cfg);
    /* Capture on for this test only: it exercises the tap ring and
     * capture_thread under TSan, which is where a race between the readers
     * pushing and the drain popping would surface. */
    cfg.capture_enabled   = true;
    cfg.capture_max_mb    = 1;
    cfg.capture_max_files = 2;
    cfg.capture_flush_s   = 1;
    snprintf(cfg.capture_dir, sizeof cfg.capture_dir,
             "/tmp/imud_conc_cap_%d", (int)getpid());
    mkdir(cfg.capture_dir, 0755);

    imu_ctx_t *imu = NULL;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) == 0, "imu_ctx_open (sim)");
    if (!imu) { puts("FAIL"); return; }

    pthread_t ism, mag, fus, cap;
    pthread_create(&ism, NULL, ism_reader_thread, imu);
    pthread_create(&mag, NULL, mag_reader_thread, imu);
    pthread_create(&fus, NULL, fusion_thread, imu);
    /* The black box runs alongside: it drains the tap ring the reader threads
     * push into and publishes cap_* under shared.lock, which is exactly what
     * the status reads in the loop below contend with. */
    pthread_create(&cap, NULL, capture_thread, imu);

    hammer_t h = { .imu = imu, .base = cfg, .stop = 0 };
    pthread_t ham;
    pthread_create(&ham, NULL, hammer_fn, &h);

    /* Concurrently read fused state + stats (exercises shared.lock too). */
    double t0 = now_s();
    while (now_s() - t0 < 0.7) {
        fused_state_t st; mag_sample_t m;
        imu_get_state(imu, &st, &m, NULL, NULL);
        imu_stats_t s; imu_get_stats(imu, &s);
        /* Capture status reads the same shared.lock the reader/fusion threads
         * write under, so it belongs in this loop rather than in a quiet
         * single-threaded call — the contention is the thing worth testing. */
        char cpath[256]; uint64_t cbytes = 0, cdrops = 0; bool cactive = true;
        imu_get_capture_status(imu, cpath, sizeof cpath, &cbytes, &cdrops, &cactive);
        usleep(500);
    }

    atomic_store(&h.stop, 1);
    pthread_join(ham, NULL);
    imu_ctx_stop(imu);
    pthread_join(fus, NULL);
    pthread_join(mag, NULL);
    pthread_join(ism, NULL);
    pthread_join(cap, NULL);

    imu_stats_t s;
    imu_get_stats(imu, &s);
    EXPECT(s.imu_samples > 0, "fusion consumed samples under the reload storm");

    /*
     * Drain accounting. This config has no IMU interrupt line, so the reader
     * paces itself and EVERY drain must be a timer drain — the edge counter
     * staying at zero is what says the split is wired to the right branch
     * rather than to whichever one happens to run.
     */
    EXPECT(s.drains_timeout > 0, "timer-paced drains were counted");
    EXPECT(s.drains_edge == 0,
           "no interrupt line configured, so no drain is attributed to an edge");
    EXPECT(s.drain_samples >= s.imu_samples,
           "every fused sample came from a counted drain");
    EXPECT(s.drain_max > 0, "the deepest burst was recorded");

    char cpath[256]; uint64_t cbytes = 0, cdrops = 0; bool cactive = true;
    imu_get_capture_status(imu, cpath, sizeof cpath, &cbytes, &cdrops, &cactive);
    EXPECT(!cactive, "capture reports inactive once its thread has joined");
    EXPECT(cpath[0] != '\0', "capture published the file it wrote");

    /* Don't leave a directory per run in /tmp. */
    {
        char cmd[320];
        snprintf(cmd, sizeof cmd, "rm -rf '%s'", cfg.capture_dir);
        if (system(cmd) != 0) { /* best effort */ }
    }

    imu_ctx_free(imu);
    puts(g_fail == fb ? "OK" : "FAIL");
}

/* ── Part B: stream shutdown + reload against connecting/disconnecting clients */

typedef struct { char path[108]; _Atomic int stop; } churn_t;

static void *churn_fn(void *arg)
{
    churn_t *c = arg;
    while (!atomic_load(&c->stop)) {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) continue;
        struct sockaddr_un a;
        memset(&a, 0, sizeof a);
        a.sun_family = AF_UNIX;
        snprintf(a.sun_path, sizeof a.sun_path, "%s", c->path);
        if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) {
            char buf[64];
            (void)!read(fd, buf, sizeof buf);   /* maybe get a packet, maybe not */
        }
        close(fd);                              /* force the server to reap us */
        usleep(200);
    }
    return NULL;
}

typedef struct { out_ctx_t *out; imud_config_t cfg; _Atomic int stop; } reloader_t;

static void *reloader_fn(void *arg)
{
    reloader_t *r = arg;
    int hz = 20;
    while (!atomic_load(&r->stop)) {
        imud_config_t c = r->cfg;
        c.stream_rate_hz   = hz;
        c.nmea_rate_hz     = hz;
        c.highrate_rate_hz = hz * 10;
        out_ctx_reload(r->out, &c);
        hz = (hz == 20) ? 100 : 20;
        usleep(1000);
    }
    return NULL;
}

static void test_stream_shutdown_race(void)
{
    printf("%-40s", "test_stream_shutdown_race");
    int fb = g_fail;

    imud_config_t cfg;
    sim_cfg(&cfg);
    cfg.stream_enabled = true;
    snprintf(cfg.stream_socket, sizeof cfg.stream_socket,
             "/tmp/imud_tc_%d.sock", (int)getpid());
    cfg.stream_rate_hz = 200;
    unlink(cfg.stream_socket);

    imu_ctx_t *imu = NULL;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) == 0, "imu_ctx_open (stream)");
    if (!imu) { puts("FAIL"); return; }

    pthread_t ism, mag, fus;
    pthread_create(&ism, NULL, ism_reader_thread, imu);
    pthread_create(&mag, NULL, mag_reader_thread, imu);
    pthread_create(&fus, NULL, fusion_thread, imu);

    out_ctx_t *out = NULL;
    EXPECT(out_ctx_open(&out, &cfg, imu) == 0, "out_ctx_open (stream)");
    if (!out) { imu_ctx_stop(imu); pthread_join(fus,NULL); pthread_join(mag,NULL);
                pthread_join(ism,NULL); imu_ctx_free(imu); puts("FAIL"); return; }

    pthread_t stream;
    pthread_create(&stream, NULL, stream_out_thread, out);

    churn_t churn = { .stop = 0 };
    snprintf(churn.path, sizeof churn.path, "%s", cfg.stream_socket);
    pthread_t churn_tid;
    pthread_create(&churn_tid, NULL, churn_fn, &churn);

    reloader_t rel = { .out = out, .cfg = cfg, .stop = 0 };
    pthread_t rel_tid;
    pthread_create(&rel_tid, NULL, reloader_fn, &rel);

    /* let clients churn + rates reload against the running stream thread */
    double t0 = now_s();
    while (now_s() - t0 < 0.6) usleep(2000);

    /* Shutdown sequence in the daemon's order: final packet (hirate only —
     * disabled here, so a no-op), then stop + join. The stream thread emits
     * its own subscribers' final packet on exit; no other thread touches
     * the client array — that is the race this proves gone. */
    atomic_store(&churn.stop, 1);
    pthread_join(churn_tid, NULL);
    out_ctx_send_shutdown(out);
    out_ctx_stop(out);
    pthread_join(stream, NULL);
    atomic_store(&rel.stop, 1);
    pthread_join(rel_tid, NULL);

    out_ctx_free(out);
    imu_ctx_stop(imu);
    pthread_join(fus, NULL);
    pthread_join(mag, NULL);
    pthread_join(ism, NULL);
    imu_ctx_free(imu);
    unlink(cfg.stream_socket);

    EXPECT(1, "stream shutdown completed without a TSan report");
    puts(g_fail == fb ? "OK" : "FAIL");
}

/* ── Part C: a magnetometer that starts after alignment has given up ─────── */

/*
 * The mag reader is started only once the fusion thread has aligned without
 * it, which is what a magnetometer that is slow to deliver looks like from
 * there.  Heading must then start being corrected rather than dead-reckon for
 * the rest of the run.
 */
static void test_late_magnetometer(void)
{
    printf("%-40s", "test_late_magnetometer");
    int fb = g_fail;

    imud_config_t cfg;
    sim_cfg(&cfg);

    imu_ctx_t *imu = NULL;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) == 0, "imu_ctx_open (late mag)");
    if (!imu) { puts("FAIL"); return; }

    pthread_t ism, mag, fus;
    pthread_create(&ism, NULL, ism_reader_thread, imu);
    pthread_create(&fus, NULL, fusion_thread, imu);

    /* Alignment waits out its window for a first mag sample, then goes on. */
    double t0 = now_s();
    while (!imu_ctx_is_settled(imu) && now_s() - t0 < 20.0) usleep(10000);
    EXPECT(imu_ctx_is_settled(imu), "aligned without the magnetometer");

    fused_state_t st;
    imu_get_state(imu, &st, NULL, NULL, NULL);
    EXPECT(!(st.flags & (FLAG_MAG_VALID | FLAG_MAG_UNCAL)),
           "no heading correction before it delivers");

    pthread_create(&mag, NULL, mag_reader_thread, imu);

    bool fused = false;
    t0 = now_s();
    while (!fused && now_s() - t0 < 5.0) {
        usleep(20000);
        imu_get_state(imu, &st, NULL, NULL, NULL);
        fused = (st.flags & (FLAG_MAG_VALID | FLAG_MAG_UNCAL)) != 0;
    }
    EXPECT(fused, "heading is corrected once the magnetometer delivers");

    imu_ctx_stop(imu);
    pthread_join(fus, NULL);
    pthread_join(mag, NULL);
    pthread_join(ism, NULL);
    imu_ctx_free(imu);
    puts(g_fail == fb ? "OK" : "FAIL");
}

/* ── Part D: the filter resets mid-run and has to align again ────────────── */

/*
 * Six seconds of a level board pointing north, with one sample three seconds
 * in whose gyro is not a number.  Nothing on the sample path refuses that, so
 * it reaches the filter and mekf_sanitize resets it.
 */
static int write_reset_capture(const char *path)
{
    const uint64_t t0_mono = 1000000000ULL;
    const uint64_t t0_wall = 1700000000000000000ULL;
    cap_writer_t w;
    if (cap_writer_open(&w, path, 200, 200000, "sim", "sim", "test",
                        t0_wall, t0_mono) != 0)
        return -1;

    for (int i = 0; i < 200 * 6; i++) {
        uint64_t dt = (uint64_t)i * 5000000ULL;        /* 200 Hz */
        imu_sample_t s;
        memset(&s, 0, sizeof s);
        s.accel[2] = -9.80665f;
        s.temp_c   = 25.0f;
        s.seq      = (uint32_t)i;
        s.chip_ts  = (uint32_t)i * 200u;               /* 25 µs ticks */
        if (i == 200 * 3) s.gyro[0] = NAN;
        if (cap_writer_imu(&w, &s, t0_mono + dt) != 0) return -1;

        if (i % 4 == 0) {                              /* 50 Hz */
            mag_sample_t m;
            memset(&m, 0, sizeof m);
            m.field[0] = 25.0f;
            m.field[2] = 40.0f;
            m.wall_ns  = t0_wall + dt;
            m.valid    = true;
            if (cap_writer_mag(&w, &m, t0_mono + dt) != 0) return -1;
        }
    }
    cap_writer_close(&w);
    return 0;
}

/*
 * After a reset the filter has no attitude.  It must take one again from the
 * samples that follow, with the outputs held in between, rather than publish
 * identity for the rest of the run.
 */
static void test_realign_after_reset(void)
{
    printf("%-40s", "test_realign_after_reset");
    int fb = g_fail;

    char path[96];
    snprintf(path, sizeof path, "/tmp/imud_conc_reset_%d.imucap", (int)getpid());
    EXPECT(write_reset_capture(path) == 0, "capture written");

    imud_config_t cfg;
    sim_cfg(&cfg);
    cfg.align_window_sec = 0.5;
    snprintf(cfg.sim_file, sizeof cfg.sim_file, "%s", path);

    imu_ctx_t *imu = NULL;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) == 0, "imu_ctx_open (replay)");
    if (!imu) { unlink(path); puts("FAIL"); return; }

    pthread_t ism, mag, fus;
    pthread_create(&ism, NULL, ism_reader_thread, imu);
    pthread_create(&mag, NULL, mag_reader_thread, imu);
    pthread_create(&fus, NULL, fusion_thread, imu);

    /* Aligned, then held by the reset, then aligned again. */
    bool aligned = false, held = false, realigned = false;
    double t0 = now_s();
    while (!realigned && now_s() - t0 < 15.0) {
        usleep(2000);
        bool up = imu_ctx_is_settled(imu);
        if      (!aligned) aligned   = up;
        else if (!held)    held      = !up;
        else               realigned = up;
    }
    EXPECT(aligned,   "aligned at startup");
    EXPECT(held,      "the non-finite sample reset the filter and held the outputs");
    EXPECT(realigned, "and the filter aligned again");

    /* Half a second on: an attitude, the right one, and heading corrected. */
    usleep(500000);
    fused_state_t st;
    imu_get_state(imu, &st, NULL, NULL, NULL);
    EXPECT(!(st.flags & FLAG_STARTUP), "the filter has an attitude again");
    EXPECT(fabsf(st.roll) < 0.05f && fabsf(st.pitch) < 0.05f,
           "and it is the level one");
    EXPECT((st.flags & (FLAG_MAG_VALID | FLAG_MAG_UNCAL)) != 0,
           "with heading being corrected");

    imu_ctx_stop(imu);
    pthread_join(fus, NULL);
    pthread_join(mag, NULL);
    pthread_join(ism, NULL);
    imu_ctx_free(imu);
    unlink(path);
    puts(g_fail == fb ? "OK" : "FAIL");
}

/* A full-scale range the part does not have stops the daemon at open, rather
 * than running at whatever the driver falls back to. */
static void test_open_refuses_unsupported_range(void)
{
    printf("%-40s", "test_open_refuses_unsupported_range");
    int fb = g_fail;

    imud_config_t cfg;
    sim_cfg(&cfg);
    imu_ctx_t *imu = NULL;

    cfg.imu_gyro_dps = 200;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) < 0 && imu == NULL,
           "200 dps is refused");
    cfg.imu_gyro_dps = 2000;
    cfg.imu_accel_g  = 3;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) < 0 && imu == NULL,
           "3 g is refused");
    cfg.imu_accel_g  = 16;
    cfg.imu_gyro_dps = 4000;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) == 0 && imu != NULL,
           "a range the part has opens");
    if (imu) imu_ctx_free(imu);

    puts(g_fail == fb ? "OK" : "FAIL");
}

/* A FIFO overflow that delivered nothing still lost samples, so the reader
 * counts it like any other. */
static void test_empty_overflow_is_counted(void)
{
    printf("%-40s", "test_empty_overflow_is_counted");
    int fb = g_fail;

    imud_config_t cfg;
    sim_cfg(&cfg);
    imu_ctx_t *imu = NULL;
    EXPECT(imu_ctx_open(&imu, &cfg, NULL) == 0, "imu_ctx_open (sim)");
    if (!imu) { puts("FAIL"); return; }

    sim_inject_overflow(3);
    pthread_t ism;
    pthread_create(&ism, NULL, ism_reader_thread, imu);

    imu_stats_t st = {0};
    for (int i = 0; i < 100; i++) {
        imu_get_stats(imu, &st);
        if (st.fifo_overflows >= 3) break;
        usleep(10000);
    }
    EXPECT(st.fifo_overflows == 3, "three empty overflows, three counted");

    imu_ctx_stop(imu);
    pthread_join(ism, NULL);
    imu_ctx_free(imu);
    puts(g_fail == fb ? "OK" : "FAIL");
}

int main(void)
{
    puts("=== imud concurrency tests (run under TSan) ===");
    test_empty_overflow_is_counted();
    test_open_refuses_unsupported_range();
    test_reload_race();
    test_stream_shutdown_race();
    test_late_magnetometer();
    test_realign_after_reset();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
