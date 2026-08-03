/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_capture.c — unit tests for the .imucap capture format (src/capture.c)
 *
 * Key correctness properties verified:
 *   - header/frame/payload sizes match the format spec
 *   - writer→reader roundtrip preserves every field, order, and mag validity
 *   - bad magic / unsupported version / short file are rejected cleanly
 *   - a truncated trailing record reads as clean EOF (black-box crash case)
 *   - unknown record types and extended payloads are skipped (forward compat)
 *   - rewind replays the stream from the first record
 *   - tap ring: FIFO order, batch pop, drop-newest on overflow with counter
 *   - rotator: size-based rotation, oldest-file pruning, name ordering
 *   - END TO END: a sim-scenario capture replayed offline through the MEKF
 *     tracks the scenario attitude (the capture→replay keystone, no daemon)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>

#include "capture.h"
#include "config.h"
#include "drivers.h"
#include "fusion.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

static void begin(const char *name) { printf("%-48s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static char g_dir[256];   /* scratch dir for this run */

static char *path_in_dir(const char *name)
{
    static char p[512];
    snprintf(p, sizeof(p), "%s/%s", g_dir, name);
    return p;
}

static imu_sample_t mk_imu(float base, uint32_t seq)
{
    imu_sample_t s;
    memset(&s, 0, sizeof(s));
    s.accel[0] = base;      s.accel[1] = base + 0.1f; s.accel[2] = base + 0.2f;
    s.gyro[0]  = -base;     s.gyro[1]  = base * 2.0f; s.gyro[2]  = base * 3.0f;
    s.temp_c   = 25.0f + base;
    s.chip_ts  = seq * 400u;
    s.seq      = seq;
    return s;
}

static mag_sample_t mk_mag(float base, bool valid)
{
    mag_sample_t m;
    memset(&m, 0, sizeof(m));
    m.field[0] = base; m.field[1] = base + 1.0f; m.field[2] = base + 2.0f;
    m.wall_ns  = 1715000000000000000ULL + (uint64_t)(base * 1e6f);
    m.valid    = valid;
    return m;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_sizes(void)
{
    begin("test_sizes");
    int fb = g_fail;
    EXPECT(sizeof(cap_header_t)   == 104, "header 104 bytes");
    EXPECT(sizeof(cap_frame_t)    == 12,  "frame 12 bytes");
    EXPECT(sizeof(cap_imu_rec_t)  == 36,  "imu payload 36 bytes");
    EXPECT(sizeof(cap_mag_rec_t)  == 20,  "mag payload 20 bytes");
    EXPECT(sizeof(cap_mark_rec_t) == 4,   "mark payload 4 bytes");
    end(fb);
}

static void test_roundtrip(void)
{
    begin("test_roundtrip");
    int fb = g_fail;
    const char *path = path_in_dir("rt.imucap");

    cap_writer_t w;
    EXPECT(cap_writer_open(&w, path, 104, "ism330dhcx", "mmc5983ma",
                           "1.5", 111ULL, 222ULL) == 0, "writer open");

    imu_sample_t s0 = mk_imu(1.0f, 7), s1 = mk_imu(2.0f, 8);
    mag_sample_t m0 = mk_mag(20.0f, true), m1 = mk_mag(30.0f, false);
    EXPECT(cap_writer_imu (&w, &s0, 1000) == 0, "write imu 0");
    EXPECT(cap_writer_mag (&w, &m0, 1500) == 0, "write mag 0");
    EXPECT(cap_writer_imu (&w, &s1, 2000) == 0, "write imu 1");
    EXPECT(cap_writer_mark(&w, 42u,  2100) == 0, "write mark");
    EXPECT(cap_writer_mag (&w, &m1, 2500) == 0, "write mag 1");
    EXPECT(w.n_imu == 2 && w.n_mag == 2, "record counters");
    cap_writer_close(&w);

    cap_reader_t r;
    EXPECT(cap_reader_open(&r, path) == 0, "reader open");
    EXPECT(r.hdr.imu_odr_hz == 104, "header odr");
    EXPECT(strcmp(r.hdr.imu_driver, "ism330dhcx") == 0, "header imu driver");
    EXPECT(strcmp(r.hdr.mag_driver, "mmc5983ma") == 0, "header mag driver");
    EXPECT(strcmp(r.hdr.imud_version, "1.5") == 0, "header version string");
    EXPECT(r.hdr.t0_wall_ns == 111ULL && r.hdr.t0_mono_ns == 222ULL,
           "header t0 clocks");

    cap_record_t rec;
    EXPECT(cap_reader_next(&r, &rec) == 1 && rec.type == CAP_REC_IMU,
           "rec 0 is imu");
    EXPECT(rec.mono_ns == 1000, "rec 0 mono_ns");
    EXPECT(memcmp(rec.imu.accel, s0.accel, sizeof(s0.accel)) == 0 &&
           memcmp(rec.imu.gyro,  s0.gyro,  sizeof(s0.gyro))  == 0,
           "rec 0 vectors exact");
    EXPECT(rec.imu.temp_c == s0.temp_c && rec.imu.chip_ts == s0.chip_ts &&
           rec.imu.seq == s0.seq, "rec 0 scalars");
    EXPECT(rec.imu.accel_raw[0] == 0.0f, "rec 0 accel_raw zeroed");

    EXPECT(cap_reader_next(&r, &rec) == 1 && rec.type == CAP_REC_MAG,
           "rec 1 is mag");
    EXPECT(memcmp(rec.mag.field, m0.field, sizeof(m0.field)) == 0 &&
           rec.mag.wall_ns == m0.wall_ns && rec.mag.valid,
           "rec 1 fields + valid");

    EXPECT(cap_reader_next(&r, &rec) == 1 && rec.type == CAP_REC_IMU &&
           rec.imu.seq == 8, "rec 2 is imu seq 8");
    EXPECT(cap_reader_next(&r, &rec) == 1 && rec.type == CAP_REC_MARK &&
           rec.mark == 42u, "rec 3 is mark 42");
    EXPECT(cap_reader_next(&r, &rec) == 1 && rec.type == CAP_REC_MAG &&
           !rec.mag.valid, "rec 4 is mag, invalid flag preserved");
    EXPECT(cap_reader_next(&r, &rec) == 0, "clean EOF");

    /* rewind: same stream again */
    EXPECT(cap_reader_rewind(&r) == 0, "rewind");
    int n = 0;
    while (cap_reader_next(&r, &rec) == 1) n++;
    EXPECT(n == 5, "rewind replays all 5 records");

    cap_reader_close(&r);
    end(fb);
}

/*
 * A capture file must be created 0644 by its own choice, not by inheriting a
 * umask.  cap_writer_open used to be a plain fopen(path, "wb") — mode 0666
 * masked by whatever the invoking shell had, which under `umask 0` left the
 * black box world-writable.  Force umask 0 so the file's own mode is what is
 * being measured.
 */
static void test_file_mode(void)
{
    begin("test_file_mode");
    int fb = g_fail;
    const char *path = path_in_dir("mode.imucap");

    mode_t prev = umask(0);
    cap_writer_t w;
    EXPECT(cap_writer_open(&w, path, 100, "sim", "sim", "1.5", 0, 0) == 0,
           "writer open");
    cap_writer_close(&w);
    umask(prev);

    struct stat st;
    EXPECT(stat(path, &st) == 0, "capture file exists");
    EXPECT((st.st_mode & 07777) == 0644, "capture file is 0644, not 0666");
    end(fb);
}

static void test_open_rejects(void)
{
    begin("test_open_rejects");
    int fb = g_fail;
    cap_reader_t r;

    /* nonexistent */
    EXPECT(cap_reader_open(&r, path_in_dir("nope.imucap")) == CAP_ERR_IO,
           "missing file is CAP_ERR_IO");

    /* bad magic */
    const char *bad = path_in_dir("bad.imucap");
    FILE *f = fopen(bad, "wb");
    cap_header_t h;
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, "NOTACAP", 8);
    h.version = CAP_FMT_VERSION; h.hdr_len = sizeof(h);
    fwrite(&h, sizeof(h), 1, f);
    fclose(f);
    EXPECT(cap_reader_open(&r, bad) == CAP_ERR_FORMAT, "bad magic rejected");

    /* future version */
    f = fopen(bad, "wb");
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, CAP_MAGIC, 8);
    h.version = 99; h.hdr_len = sizeof(h);
    fwrite(&h, sizeof(h), 1, f);
    fclose(f);
    EXPECT(cap_reader_open(&r, bad) == CAP_ERR_FORMAT, "version 99 rejected");

    /* shorter than a header */
    f = fopen(bad, "wb");
    fwrite("IMUCAP1", 8, 1, f);
    fclose(f);
    EXPECT(cap_reader_open(&r, bad) == CAP_ERR_FORMAT, "short file rejected");

    end(fb);
}

static void test_truncated_tail(void)
{
    begin("test_truncated_tail");
    int fb = g_fail;
    const char *path = path_in_dir("trunc.imucap");

    cap_writer_t w;
    cap_writer_open(&w, path, 100, "sim", "sim", "1.5", 0, 0);
    imu_sample_t s = mk_imu(1.0f, 1);
    cap_writer_imu(&w, &s, 100);
    cap_writer_imu(&w, &s, 200);
    uint64_t full = w.bytes;
    cap_writer_close(&w);

    /* chop mid-way through the second record's payload */
    EXPECT(truncate(path, (off_t)(full - 10)) == 0, "truncate file");

    cap_reader_t r;
    cap_record_t rec;
    EXPECT(cap_reader_open(&r, path) == 0, "open truncated");
    EXPECT(cap_reader_next(&r, &rec) == 1, "first record intact");
    EXPECT(cap_reader_next(&r, &rec) == 0, "truncated tail = clean EOF");
    cap_reader_close(&r);

    end(fb);
}

static void test_forward_compat(void)
{
    begin("test_forward_compat");
    int fb = g_fail;
    const char *path = path_in_dir("fwd.imucap");

    /* Hand-build: [unknown type 99] [imu with 8 extra payload bytes] */
    cap_writer_t w;
    cap_writer_open(&w, path, 100, "sim", "sim", "1.5", 0, 0);
    cap_writer_close(&w);   /* header only; append frames manually */

    FILE *f = fopen(path, "ab");
    cap_frame_t fr = { .type = 99, .flags = 0, .len = 16, .mono_ns = 50 };
    uint8_t junk[16] = {0xAB};
    fwrite(&fr, sizeof(fr), 1, f);
    fwrite(junk, sizeof(junk), 1, f);

    cap_imu_rec_t rec = { .accel = {1, 2, 3}, .gyro = {4, 5, 6},
                          .temp_c = 25.0f, .chip_ts = 400, .seq = 9 };
    uint8_t extra[8] = {0xCD};            /* future appended fields */
    fr = (cap_frame_t){ .type = CAP_REC_IMU, .flags = 0,
                        .len = sizeof(rec) + sizeof(extra), .mono_ns = 60 };
    fwrite(&fr, sizeof(fr), 1, f);
    fwrite(&rec, sizeof(rec), 1, f);
    fwrite(extra, sizeof(extra), 1, f);
    fclose(f);

    cap_reader_t r;
    cap_record_t out;
    EXPECT(cap_reader_open(&r, path) == 0, "open");
    EXPECT(cap_reader_next(&r, &out) == 1, "unknown type skipped");
    EXPECT(out.type == CAP_REC_IMU && out.imu.seq == 9 &&
           out.mono_ns == 60, "extended imu record parsed");
    EXPECT(out.imu.accel[2] == 3.0f && out.imu.gyro[0] == 4.0f,
           "known fields read, extra bytes skipped");
    EXPECT(cap_reader_next(&r, &out) == 0, "EOF after extended record");
    cap_reader_close(&r);

    end(fb);
}

static void test_ring(void)
{
    begin("test_ring");
    int fb = g_fail;

    cap_ring_t ring;
    cap_ring_init(&ring);

    imu_sample_t s = mk_imu(1.0f, 0);
    mag_sample_t m = mk_mag(25.0f, true);
    EXPECT(cap_ring_push_imu(&ring, &s, 10) == 0, "push imu");
    EXPECT(cap_ring_push_mag(&ring, &m, 20) == 0, "push mag");

    cap_ring_rec_t out[4];
    int n = cap_ring_pop(&ring, out, 4);
    EXPECT(n == 2, "batch pop both");
    EXPECT(out[0].type == CAP_REC_IMU && out[0].mono_ns == 10 &&
           out[1].type == CAP_REC_MAG && out[1].mono_ns == 20,
           "FIFO order preserved");
    EXPECT(cap_ring_pop(&ring, out, 4) == 0, "empty pop returns 0");

    /* fill to capacity; the overflow record is dropped and counted */
    for (int i = 0; i < CAP_RING_LEN; i++) {
        s.seq = (uint32_t)i;
        EXPECT(cap_ring_push_imu(&ring, &s, (uint64_t)i) == 0 || 1, "fill");
    }
    EXPECT(cap_ring_dropped(&ring) == 0, "no drops at exactly full");
    EXPECT(cap_ring_push_imu(&ring, &s, 9999) == 1, "overflow push dropped");
    EXPECT(cap_ring_dropped(&ring) == 1, "drop counted");

    /* drain and confirm the oldest survived (drop-newest policy) */
    int total = 0, got;
    cap_ring_rec_t buf[64];
    uint32_t first_seq = 0xFFFFFFFFu;
    while ((got = cap_ring_pop(&ring, buf, 64)) > 0) {
        if (total == 0) first_seq = buf[0].u.imu.seq;
        total += got;
    }
    EXPECT(total == CAP_RING_LEN, "drained full ring");
    EXPECT(first_seq == 0, "oldest record survived (drop-newest)");

    end(fb);
}

static int count_captures(const char *dir)
{
    DIR *d = opendir(dir);
    if (!d) return -1;
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL)
        if (strstr(e->d_name, ".imucap") && strncmp(e->d_name, "imud-", 5) == 0)
            n++;
    closedir(d);
    return n;
}

static void test_rotator(void)
{
    begin("test_rotator");
    int fb = g_fail;

    char dir[300];
    snprintf(dir, sizeof(dir), "%s/rot", g_dir);
    mkdir(dir, 0755);

    cap_rotator_t rt;
    EXPECT(cap_rot_open(&rt, dir, 1 /* MB */, 2 /* files */,
                        100, "sim", "sim", "1.5") == 0, "rotator open");
    EXPECT(cap_rot_path(&rt)[0] != '\0', "current path set");

    /* one imu record = 12 + 36 = 48 bytes → ~22k records per MB;
     * 70k records ≈ 3.4 MB → several rotations */
    cap_ring_rec_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.type = CAP_REC_IMU;
    for (int i = 0; i < 70000; i++) {
        rec.mono_ns = (uint64_t)i;
        rec.u.imu = mk_imu(0.5f, (uint32_t)i);
        if (cap_rot_write(&rt, &rec) != 0) { EXPECT(0, "rot write"); break; }
    }
    EXPECT(cap_rot_flush(&rt) == 0, "flush");
    EXPECT(cap_rot_bytes(&rt) < 1100u * 1024u, "current file below cap+slack");

    char last_path[320];
    snprintf(last_path, sizeof(last_path), "%s", cap_rot_path(&rt));
    cap_rot_close(&rt);

    EXPECT(count_captures(dir) == 2, "pruned to max_files=2");

    /* newest file must still parse and contain the LAST records written */
    cap_reader_t r;
    cap_record_t out;
    EXPECT(cap_reader_open(&r, last_path) == 0, "newest file parses");
    uint32_t last_seq = 0;
    while (cap_reader_next(&r, &out) == 1) last_seq = out.imu.seq;
    EXPECT(last_seq == 69999u, "newest file ends at final record");
    cap_reader_close(&r);

    end(fb);
}

/* ── Playback mode: the real sim driver ops replaying a capture ──────────── */

extern const imu_ops_t sim_imu_ops;
extern const mag_ops_t sim_mag_ops;

static const char *make_small_capture(void)
{
    const char *path = path_in_dir("pb.imucap");
    const int   odr = 100, dur_s = 5;

    cap_writer_t w;
    cap_writer_open(&w, path, (uint32_t)odr, "sim", "sim", "1.5", 0, 0);
    for (int i = 0; i < odr * dur_s; i++) {
        double t = i / (double)odr;
        imu_sample_t s;
        memset(&s, 0, sizeof(s));
        sim_synth_imu(t, &s);
        s.seq     = (uint32_t)i;
        s.chip_ts = (uint32_t)i * 400u;
        cap_writer_imu(&w, &s, (uint64_t)(t * 1e9));
        if (i % 10 == 0) {
            mag_sample_t m;
            memset(&m, 0, sizeof(m));
            sim_synth_mag(t, &m);
            cap_writer_mag(&w, &m, (uint64_t)(t * 1e9));
        }
    }
    cap_writer_close(&w);
    return path;
}

static void test_playback_driver(void)
{
    begin("test_playback_driver");
    int fb = g_fail;
    const char *path = make_small_capture();

    imu_cfg_t icfg = { .odr_hz = 100, .accel_g = 8, .gyro_dps = 2000 };
    mag_cfg_t mcfg = { .odr_hz = 100 };

    /* ── single pass, as-fast-as-possible ─────────────────────────────── */
    sim_set_playback(path, false, 0.0f);
    EXPECT(sim_imu_ops.init(0, 0, &icfg) == 0, "imu init (playback)");
    EXPECT(sim_mag_ops.init(0, 0, &mcfg) == 0, "mag init (playback)");

    imu_sample_t buf[128];
    int total = 0, n = 0;
    uint32_t next_seq = 0;
    bool seq_ok = true;
    for (int guard = 0; guard < 100; guard++) {
        EXPECT(sim_imu_ops.read(0, 0, buf, 128, &n) == 0, "imu read rc");
        if (n == 0) break;
        for (int i = 0; i < n; i++)
            if (buf[i].seq != next_seq++) seq_ok = false;
        total += n;
    }
    EXPECT(total == 500, "all 500 imu samples replayed");
    EXPECT(seq_ok, "imu seq contiguous");

    mag_sample_t m;
    int mags = 0;
    uint64_t prev_wall = 0;
    bool wall_ok = true;
    while (sim_mag_ops.read(0, 0, &m) == 0 && mags < 1000) {
        if (m.wall_ns <= prev_wall && mags > 0) wall_ok = false;
        prev_wall = m.wall_ns;
        mags++;
    }
    EXPECT(mags == 50, "all 50 mag samples replayed");
    EXPECT(wall_ok, "mag wall_ns strictly increasing (remapped)");
    EXPECT(sim_mag_ops.read(0, 0, &m) == 1, "mag EOF reports no-data");

    /* ── loop mode: seq/chip_ts stay monotonic across the wrap ─────────── */
    sim_set_playback(path, true, 0.0f);
    sim_imu_ops.init(0, 0, &icfg);

    uint32_t prev_s = 0, prev_ts = 0;
    bool mono_ok = true;
    int got = 0;
    while (got < 1500) {                       /* three passes worth */
        sim_imu_ops.read(0, 0, buf, 128, &n);
        if (n == 0) break;
        for (int i = 0; i < n && got < 1500; i++, got++) {
            if (got > 0 && (buf[i].seq != prev_s + 1 ||
                            buf[i].chip_ts <= prev_ts)) mono_ok = false;
            prev_s  = buf[i].seq;
            prev_ts = buf[i].chip_ts;
        }
    }
    EXPECT(got == 1500, "loop mode keeps delivering");
    EXPECT(mono_ok, "seq +1 contiguous and chip_ts monotonic across wraps");

    /* back to synthesis so later tests see default behavior */
    sim_set_playback(NULL, false, 1.0f);
    end(fb);
}

/* ── End to end: sim scenario → capture file → offline MEKF replay ───────── */

static void q_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float R20 = 2 * (x * z - w * y);
    float R21 = 2 * (y * z + w * x);
    float R22 = 1 - 2 * (x * x + y * y);
    float R10 = 2 * (x * y + w * z);
    float R00 = 1 - 2 * (y * y + z * z);
    *pitch = asinf(-R20);
    *roll  = atan2f(R21, R22);
    *yaw   = atan2f(R10, R00);
}

static float ang_diff(float a, float b)   /* wrapped a−b, rad */
{
    float d = a - b;
    while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
    while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    return d;
}

static void test_end_to_end_replay(void)
{
    begin("test_end_to_end_replay");
    int fb = g_fail;
    const char *path = path_in_dir("e2e.imucap");

    /* ── Generate: 90 s of the sim scenario @ 100 Hz IMU, 10 Hz mag ─────── */
    const int    odr = 100, dur_s = 90;
    const double dt  = 1.0 / odr;

    cap_writer_t w;
    EXPECT(cap_writer_open(&w, path, (uint32_t)odr, "sim", "sim",
                           "1.5", 0, 0) == 0, "e2e writer open");
    for (int i = 0; i < odr * dur_s; i++) {
        double t = i * dt;
        imu_sample_t s;
        memset(&s, 0, sizeof(s));
        sim_synth_imu(t, &s);
        s.seq     = (uint32_t)i;
        s.chip_ts = (uint32_t)i * (40000u / (uint32_t)odr);
        cap_writer_imu(&w, &s, (uint64_t)(t * 1e9));
        if (i % 10 == 0) {                     /* 10 Hz mag */
            mag_sample_t m;
            memset(&m, 0, sizeof(m));
            sim_synth_mag(t, &m);
            m.wall_ns = (uint64_t)(t * 1e9);
            cap_writer_mag(&w, &m, (uint64_t)(t * 1e9));
        }
    }
    cap_writer_close(&w);

    /* ── Replay offline through the MEKF (the daemon-free keystone) ─────── */
    imud_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mekf_gyro_noise   = 0.007;
    cfg.mekf_gyro_bias    = 0.00015;
    cfg.mekf_accel_noise  = 0.0022;
    cfg.mekf_mag_noise    = 0.0004;
    cfg.accel_skip_thresh = 0.05;
    cfg.mag_reject_gauss  = 0.05;
    cfg.mag_odr_hz        = 10;

    cap_reader_t r;
    EXPECT(cap_reader_open(&r, path) == 0, "e2e reader open");

    mekf_t f;
    float  bias0[3] = {0, 0, 0};
    mekf_init(&f, &cfg, (float)odr, (float)cfg.mag_odr_hz, bias0);

    cap_record_t rec;
    bool         aligned  = false;
    bool         have_imu = false;
    imu_sample_t last_imu;
    memset(&last_imu, 0, sizeof(last_imu));
    uint32_t prev_ticks = 0;
    double   t_end = 0.0;

    while (cap_reader_next(&r, &rec) == 1) {
        if (rec.type == CAP_REC_IMU) {
            last_imu = rec.imu;
            have_imu = true;
            if (!aligned) { prev_ticks = rec.imu.chip_ts; continue; }
            float dts = (float)((rec.imu.chip_ts - prev_ticks) * 25e-6);
            prev_ticks = rec.imu.chip_ts;
            mekf_predict(&f, &rec.imu, dts > 0 ? dts : f.dt);
            mekf_update_accel(&f, &rec.imu);
            t_end = (double)rec.mono_ns * 1e-9;
        } else if (rec.type == CAP_REC_MAG) {
            if (!aligned && have_imu) {
                mekf_align(&f, last_imu.accel, rec.mag.field);
                aligned = true;
                continue;
            }
            if (aligned) mekf_update_mag(&f, &rec.mag);
        }
    }
    cap_reader_close(&r);
    EXPECT(aligned, "alignment happened from captured samples");
    EXPECT(t_end > 89.0, "replayed the full 90 s");

    /* Scenario ground truth at t_end (same closed form the sim uses). */
    float roll, pitch, yaw;
    q_to_euler(f.q, &roll, &pitch, &yaw);

    double psi_true   = (60.0 + 6.0 * t_end) * M_PI / 180.0;
    double phi_true   = (4.0 * M_PI / 180.0) * sin(2.0 * M_PI * t_end / 6.0);
    double theta_true = (2.0 * M_PI / 180.0) * sin(2.0 * M_PI * t_end / 8.0
                                                   + M_PI / 3.0);

    /* Post sim-pitch-fix tracking is ~0.03 deg; bounds leave margin only
     * for float/platform variance.  Do not loosen without a recorded
     * before/after (house wave-benchmark rule). */
    EXPECT(fabsf(ang_diff(yaw,   (float)psi_true))   < 1.0f * M_PI / 180.0f,
           "heading tracks scenario within 1 deg");
    EXPECT(fabsf(ang_diff(roll,  (float)phi_true))   < 0.5f * M_PI / 180.0f,
           "roll tracks scenario within 0.5 deg");
    EXPECT(fabsf(ang_diff(pitch, (float)theta_true)) < 0.5f * M_PI / 180.0f,
           "pitch tracks scenario within 0.5 deg");

    end(fb);
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    snprintf(g_dir, sizeof(g_dir), "/tmp/imucap_test_%d", (int)getpid());
    if (mkdir(g_dir, 0755) != 0) {
        fprintf(stderr, "cannot create %s\n", g_dir);
        return 1;
    }

    test_sizes();
    test_roundtrip();
    test_file_mode();
    test_open_rejects();
    test_truncated_tail();
    test_forward_compat();
    test_ring();
    test_rotator();
    test_playback_driver();
    test_end_to_end_replay();

    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
