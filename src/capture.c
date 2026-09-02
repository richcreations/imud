/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * capture.c — .imucap raw-sample capture format + black-box plumbing
 *
 * See capture.h for the format contract.  This TU is pure: no logging and
 * no daemon dependencies, so the daemon, imud-cal, tests, and fuzzers all
 * link it directly.  Callers log failures themselves.
 */

#include <dirent.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "capture.h"
#include "fileio.h"
#include "wire.h"

/* ── Wire serialisation ──────────────────────────────────────────────────── */

/*
 * The file is little-endian on every host, so each struct is converted through
 * a byte buffer rather than written as its in-memory image.  Sizes come from
 * the _Static_asserts in capture.h; the char arrays are copied verbatim.
 */

static void hdr_encode(uint8_t b[104], const cap_header_t *h)
{
    memset(b, 0, 104);
    memcpy(b + 0, h->magic, 8);
    wire_put_u16(b +  8, h->version);
    wire_put_u16(b + 10, h->hdr_len);
    wire_put_u32(b + 12, h->imu_odr_hz);
    memcpy(b + 16, h->imu_driver,   16);
    memcpy(b + 32, h->mag_driver,   16);
    memcpy(b + 48, h->imud_version, 16);
    wire_put_u64(b + 64, h->t0_wall_ns);
    wire_put_u64(b + 72, h->t0_mono_ns);
    wire_put_u32(b + 80, h->imu_odr_mhz);
    /* b[84..103] stays the zeroed `reserved` tail. */
}

static void hdr_decode(cap_header_t *h, const uint8_t b[104])
{
    memset(h, 0, sizeof(*h));
    memcpy(h->magic, b + 0, 8);
    h->version      = wire_get_u16(b +  8);
    h->hdr_len      = wire_get_u16(b + 10);
    h->imu_odr_hz   = wire_get_u32(b + 12);
    memcpy(h->imu_driver,   b + 16, 16);
    memcpy(h->mag_driver,   b + 32, 16);
    memcpy(h->imud_version, b + 48, 16);
    h->t0_wall_ns   = wire_get_u64(b + 64);
    h->t0_mono_ns   = wire_get_u64(b + 72);
    h->imu_odr_mhz  = wire_get_u32(b + 80);
}

static void frame_encode(uint8_t b[12], const cap_frame_t *f)
{
    wire_put_u8 (b + 0, f->type);
    wire_put_u8 (b + 1, f->flags);
    wire_put_u16(b + 2, f->len);
    wire_put_u64(b + 4, f->mono_ns);
}

static void frame_decode(cap_frame_t *f, const uint8_t b[12])
{
    f->type    = wire_get_u8 (b + 0);
    f->flags   = wire_get_u8 (b + 1);
    f->len     = wire_get_u16(b + 2);
    f->mono_ns = wire_get_u64(b + 4);
}

static void imu_rec_encode(uint8_t b[36], const cap_imu_rec_t *r)
{
    for (int i = 0; i < 3; i++) wire_put_f32(b +  0 + 4 * i, r->accel[i]);
    for (int i = 0; i < 3; i++) wire_put_f32(b + 12 + 4 * i, r->gyro[i]);
    wire_put_f32(b + 24, r->temp_c);
    wire_put_u32(b + 28, r->chip_ts);
    wire_put_u32(b + 32, r->seq);
}

static void imu_rec_decode(cap_imu_rec_t *r, const uint8_t b[36])
{
    for (int i = 0; i < 3; i++) r->accel[i] = wire_get_f32(b +  0 + 4 * i);
    for (int i = 0; i < 3; i++) r->gyro[i]  = wire_get_f32(b + 12 + 4 * i);
    r->temp_c  = wire_get_f32(b + 24);
    r->chip_ts = wire_get_u32(b + 28);
    r->seq     = wire_get_u32(b + 32);
}

static void mag_rec_encode(uint8_t b[20], const cap_mag_rec_t *r)
{
    for (int i = 0; i < 3; i++) wire_put_f32(b + 4 * i, r->field[i]);
    wire_put_u64(b + 12, r->wall_ns);
}

static void mag_rec_decode(cap_mag_rec_t *r, const uint8_t b[20])
{
    for (int i = 0; i < 3; i++) r->field[i] = wire_get_f32(b + 4 * i);
    r->wall_ns = wire_get_u64(b + 12);
}

/* ── Writer ──────────────────────────────────────────────────────────────── */

/* Buffer file I/O in 64 KiB chunks — fewer, larger SD-card writes. */
#define CAP_WRITE_BUF (64 * 1024)

int cap_writer_open(cap_writer_t *w, const char *path,
                    uint32_t imu_odr_hz, uint32_t imu_odr_mhz,
                    const char *imu_driver, const char *mag_driver,
                    const char *imud_version,
                    uint64_t t0_wall_ns, uint64_t t0_mono_ns)
{
    memset(w, 0, sizeof(*w));

    w->f = fcreate(path, "wb", IMUD_FILE_MODE);
    if (!w->f) return -1;
    setvbuf(w->f, NULL, _IOFBF, CAP_WRITE_BUF);

    cap_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, CAP_MAGIC, sizeof(hdr.magic));   /* "IMUCAP1\0" */
    hdr.version    = CAP_FMT_VERSION;
    hdr.hdr_len    = (uint16_t)sizeof(cap_header_t);
    hdr.imu_odr_hz  = imu_odr_hz;
    hdr.imu_odr_mhz = imu_odr_mhz;
    snprintf(hdr.imu_driver,   sizeof(hdr.imu_driver),   "%s", imu_driver   ? imu_driver   : "");
    snprintf(hdr.mag_driver,   sizeof(hdr.mag_driver),   "%s", mag_driver   ? mag_driver   : "");
    snprintf(hdr.imud_version, sizeof(hdr.imud_version), "%s", imud_version ? imud_version : "");
    hdr.t0_wall_ns = t0_wall_ns;
    hdr.t0_mono_ns = t0_mono_ns;

    uint8_t hb[sizeof(cap_header_t)];
    hdr_encode(hb, &hdr);
    if (fwrite(hb, sizeof(hb), 1, w->f) != 1) {
        fclose(w->f);
        w->f = NULL;
        return -1;
    }
    w->bytes = sizeof(hdr);
    return 0;
}

/* payload is already in wire order. */
static int cap_write_rec(cap_writer_t *w, uint8_t type, uint8_t flags,
                         const uint8_t *payload, uint16_t len, uint64_t mono_ns)
{
    if (!w->f) return -1;

    cap_frame_t fr = { .type = type, .flags = flags,
                       .len = len, .mono_ns = mono_ns };
    uint8_t fb[sizeof(cap_frame_t)];
    frame_encode(fb, &fr);
    if (fwrite(fb, sizeof(fb), 1, w->f) != 1)       return -1;
    if (len && fwrite(payload, len, 1, w->f) != 1)  return -1;
    w->bytes += sizeof(fr) + len;
    return 0;
}

int cap_writer_imu(cap_writer_t *w, const imu_sample_t *s, uint64_t mono_ns)
{
    cap_imu_rec_t rec = {
        .accel   = { s->accel[0], s->accel[1], s->accel[2] },
        .gyro    = { s->gyro[0],  s->gyro[1],  s->gyro[2]  },
        .temp_c  = s->temp_c,
        .chip_ts = s->chip_ts,
        .seq     = s->seq,
    };
    uint8_t rb[sizeof(cap_imu_rec_t)];
    imu_rec_encode(rb, &rec);
    if (cap_write_rec(w, CAP_REC_IMU, 0, rb, sizeof(rb), mono_ns) != 0)
        return -1;
    w->n_imu++;
    return 0;
}

int cap_writer_mag(cap_writer_t *w, const mag_sample_t *m, uint64_t mono_ns)
{
    cap_mag_rec_t rec = {
        .field   = { m->field[0], m->field[1], m->field[2] },
        .wall_ns = m->wall_ns,
    };
    uint8_t rb[sizeof(cap_mag_rec_t)];
    mag_rec_encode(rb, &rec);
    uint8_t flags = m->valid ? CAP_MAGF_VALID : 0;
    if (cap_write_rec(w, CAP_REC_MAG, flags, rb, sizeof(rb), mono_ns) != 0)
        return -1;
    w->n_mag++;
    return 0;
}

int cap_writer_mark(cap_writer_t *w, uint32_t code, uint64_t mono_ns)
{
    uint8_t rb[sizeof(cap_mark_rec_t)];
    wire_put_u32(rb, code);
    return cap_write_rec(w, CAP_REC_MARK, 0, rb, sizeof(rb), mono_ns);
}

int cap_writer_flush(cap_writer_t *w)
{
    if (!w->f) return -1;
    return fflush(w->f) == 0 ? 0 : -1;
}

void cap_writer_close(cap_writer_t *w)
{
    if (w->f) {
        fclose(w->f);
        w->f = NULL;
    }
}

/* ── Reader ──────────────────────────────────────────────────────────────── */

int cap_reader_open(cap_reader_t *r, const char *path)
{
    memset(r, 0, sizeof(*r));

    r->f = fopen(path, "rb");
    if (!r->f) return CAP_ERR_IO;

    uint8_t hb[sizeof(cap_header_t)];
    if (fread(hb, sizeof(hb), 1, r->f) != 1) {
        fclose(r->f); r->f = NULL;
        return CAP_ERR_FORMAT;                 /* shorter than a header */
    }
    hdr_decode(&r->hdr, hb);
    if (memcmp(r->hdr.magic, CAP_MAGIC, sizeof(r->hdr.magic)) != 0 ||
        r->hdr.version != CAP_FMT_VERSION ||
        r->hdr.hdr_len < sizeof(cap_header_t)) {
        fclose(r->f); r->f = NULL;
        return CAP_ERR_FORMAT;
    }
    /* A future writer may extend the header; hdr_len says where records start. */
    if (fseek(r->f, (long)r->hdr.hdr_len, SEEK_SET) != 0) {
        fclose(r->f); r->f = NULL;
        return CAP_ERR_IO;
    }
    return 0;
}

/*
 * Read exactly len payload bytes into dst (dst_sz max), skipping any excess
 * — a future writer may have appended payload fields.  dst receives raw wire
 * bytes; the caller decodes.  Returns 1 ok, 0 truncated (treat as EOF),
 * -1 on read error.
 */
static int read_payload(FILE *f, void *dst, size_t dst_sz, uint16_t len)
{
    size_t want = len < dst_sz ? len : dst_sz;
    if (want && fread(dst, want, 1, f) != 1)
        return ferror(f) ? -1 : 0;
    if (len > want && fseek(f, (long)(len - want), SEEK_CUR) != 0)
        return ferror(f) ? -1 : 0;
    return 1;
}

int cap_reader_next(cap_reader_t *r, cap_record_t *out)
{
    if (!r->f) return CAP_ERR_IO;

    for (;;) {
        cap_frame_t fr;
        uint8_t     fb[sizeof(cap_frame_t)];
        if (fread(fb, sizeof(fb), 1, r->f) != 1)
            return ferror(r->f) ? CAP_ERR_IO : 0;      /* clean EOF */
        frame_decode(&fr, fb);

        memset(out, 0, sizeof(*out));
        out->type    = fr.type;
        out->mono_ns = fr.mono_ns;

        switch (fr.type) {
        case CAP_REC_IMU: {
            cap_imu_rec_t rec;
            uint8_t       rb[sizeof(cap_imu_rec_t)];
            memset(rb, 0, sizeof(rb));
            if (fr.len < sizeof(rb)) return 0;         /* corrupt tail */
            int rc = read_payload(r->f, rb, sizeof(rb), fr.len);
            if (rc <= 0) return rc < 0 ? CAP_ERR_IO : 0;
            imu_rec_decode(&rec, rb);
            out->imu.accel[0] = rec.accel[0];
            out->imu.accel[1] = rec.accel[1];
            out->imu.accel[2] = rec.accel[2];
            out->imu.gyro[0]  = rec.gyro[0];
            out->imu.gyro[1]  = rec.gyro[1];
            out->imu.gyro[2]  = rec.gyro[2];
            out->imu.temp_c   = rec.temp_c;
            out->imu.chip_ts  = rec.chip_ts;
            out->imu.seq      = rec.seq;
            return 1;
        }
        case CAP_REC_MAG: {
            cap_mag_rec_t rec;
            uint8_t       rb[sizeof(cap_mag_rec_t)];
            memset(rb, 0, sizeof(rb));
            if (fr.len < sizeof(rb)) return 0;
            int rc = read_payload(r->f, rb, sizeof(rb), fr.len);
            if (rc <= 0) return rc < 0 ? CAP_ERR_IO : 0;
            mag_rec_decode(&rec, rb);
            out->mag.field[0] = rec.field[0];
            out->mag.field[1] = rec.field[1];
            out->mag.field[2] = rec.field[2];
            out->mag.wall_ns  = rec.wall_ns;
            out->mag.valid    = (fr.flags & CAP_MAGF_VALID) != 0;
            /* The tap records the field pre-calibration, so a replayed sample
             * is uncalibrated until the replay path applies a cal of its own. */
            out->mag.calibrated = false;
            return 1;
        }
        case CAP_REC_MARK: {
            uint8_t rb[sizeof(cap_mark_rec_t)];
            memset(rb, 0, sizeof(rb));
            if (fr.len < sizeof(rb)) return 0;
            int rc = read_payload(r->f, rb, sizeof(rb), fr.len);
            if (rc <= 0) return rc < 0 ? CAP_ERR_IO : 0;
            out->mark = wire_get_u32(rb);
            return 1;
        }
        default:
            /* Unknown record type from a newer writer: skip and continue. */
            if (fr.len && fseek(r->f, (long)fr.len, SEEK_CUR) != 0)
                return ferror(r->f) ? CAP_ERR_IO : 0;
            break;
        }
    }
}

int cap_reader_rewind(cap_reader_t *r)
{
    if (!r->f) return -1;
    return fseek(r->f, (long)r->hdr.hdr_len, SEEK_SET) == 0 ? 0 : -1;
}

void cap_reader_close(cap_reader_t *r)
{
    if (r->f) {
        fclose(r->f);
        r->f = NULL;
    }
}

/* ── Tap ring ────────────────────────────────────────────────────────────── */

void cap_ring_init(cap_ring_t *r)
{
    memset(r, 0, sizeof(*r));
    pthread_mutex_init(&r->mtx, NULL);
}

void cap_ring_destroy(cap_ring_t *r)
{
    pthread_mutex_destroy(&r->mtx);
}

static int ring_push(cap_ring_t *r, const cap_ring_rec_t *rec)
{
    int dropped = 0;
    pthread_mutex_lock(&r->mtx);
    if (r->count == CAP_RING_LEN) {
        /* Drop the NEWEST record: history stays contiguous, the gap sits at
         * the stall and is visible in seq. */
        r->dropped++;
        dropped = 1;
    } else {
        r->buf[r->head] = *rec;
        r->head = (r->head + 1) % CAP_RING_LEN;
        r->count++;
    }
    pthread_mutex_unlock(&r->mtx);
    return dropped;
}

int cap_ring_push_imu(cap_ring_t *r, const imu_sample_t *s, uint64_t mono_ns)
{
    cap_ring_rec_t rec = { .type = CAP_REC_IMU, .mono_ns = mono_ns };
    rec.u.imu = *s;
    return ring_push(r, &rec);
}

int cap_ring_push_mag(cap_ring_t *r, const mag_sample_t *m, uint64_t mono_ns)
{
    cap_ring_rec_t rec = { .type = CAP_REC_MAG, .mono_ns = mono_ns };
    rec.u.mag = *m;
    return ring_push(r, &rec);
}

int cap_ring_pop(cap_ring_t *r, cap_ring_rec_t *out, int max)
{
    int n = 0;
    pthread_mutex_lock(&r->mtx);
    while (n < max && r->count > 0) {
        out[n++] = r->buf[r->tail];
        r->tail = (r->tail + 1) % CAP_RING_LEN;
        r->count--;
    }
    pthread_mutex_unlock(&r->mtx);
    return n;
}

uint64_t cap_ring_dropped(cap_ring_t *r)
{
    pthread_mutex_lock(&r->mtx);
    uint64_t d = r->dropped;
    pthread_mutex_unlock(&r->mtx);
    return d;
}

/* ── Rotating black-box writer ───────────────────────────────────────────── */

#define CAP_FILE_PREFIX "imud-"
#define CAP_FILE_SUFFIX ".imucap"

static bool is_capture_name(const char *name)
{
    size_t n = strlen(name), p = strlen(CAP_FILE_PREFIX),
           s = strlen(CAP_FILE_SUFFIX);
    return n > p + s &&
           strncmp(name, CAP_FILE_PREFIX, p) == 0 &&
           strcmp(name + n - s, CAP_FILE_SUFFIX) == 0;
}

static int name_cmp(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

/*
 * Delete the oldest imud-*.imucap files so at most max_files remain
 * (UTC timestamp names sort lexicographically = chronologically).
 * Best-effort: unlink failures are ignored.
 */
static void cap_rot_prune(const cap_rotator_t *rt)
{
    if (rt->max_files == 0) return;

    DIR *d = opendir(rt->dir);
    if (!d) return;

    char  *names[256];
    int    n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL && n < 256) {
        if (!is_capture_name(e->d_name)) continue;
        names[n] = strdup(e->d_name);
        if (names[n]) n++;
    }
    closedir(d);

    /* cppcheck-suppress uninitvar
     * False positive: cppcheck flags names[] as uninitialized on the path
     * where readdir() matched nothing, but that path leaves n == 0 and
     * qsort() reads no elements for a zero count. */
    qsort(names, (size_t)n, sizeof(names[0]), name_cmp);

    const char *slash = strrchr(rt->cur_path, '/');
    const char *cur   = slash ? slash + 1 : rt->cur_path;

    for (int i = 0; i < n; i++) {
        if (i < n - (int)rt->max_files && strcmp(names[i], cur) != 0) {
            char path[512];
            snprintf(path, sizeof(path), "%s/%s", rt->dir, names[i]);
            unlink(path);
        }
        free(names[i]);
    }
}

static uint64_t now_ns(clockid_t clk)
{
    struct timespec t;
    clock_gettime(clk, &t);
    return (uint64_t)t.tv_sec * 1000000000ULL + (uint64_t)t.tv_nsec;
}

/*
 * Open a fresh imud-YYYYMMDD-HHMMSS.imucap (UTC).  Stamps are strictly
 * increasing within a rotator instance (bumped forward a second on
 * collision or fast rotation) so lexicographic order == chronological —
 * pruning relies on that, and a pruned old name must never be reused for
 * a NEWER file (it would then be the next prune's "oldest" victim).
 */
static int cap_rot_open_file(cap_rotator_t *rt)
{
    time_t t = time(NULL);
    if (t <= rt->last_stamp) t = rt->last_stamp + 1;

    for (int bump = 0; bump < 600; bump++, t++) {
        struct tm tm_utc;
        gmtime_r(&t, &tm_utc);

        char stamp[32];
        strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_utc);
        snprintf(rt->cur_path, sizeof(rt->cur_path),
                 "%s/" CAP_FILE_PREFIX "%s" CAP_FILE_SUFFIX, rt->dir, stamp);
        if (access(rt->cur_path, F_OK) != 0) break;
    }
    rt->last_stamp = t;

    if (cap_writer_open(&rt->w, rt->cur_path, rt->imu_odr_hz, rt->imu_odr_mhz,
                        rt->imu_driver, rt->mag_driver, rt->imud_version,
                        now_ns(CLOCK_REALTIME), now_ns(CLOCK_MONOTONIC)) != 0)
        return -1;
    rt->open = true;
    cap_rot_prune(rt);
    return 0;
}

int cap_rot_open(cap_rotator_t *rt, const char *dir,
                 uint32_t max_mb, uint32_t max_files,
                 uint32_t imu_odr_hz, uint32_t imu_odr_mhz,
                 const char *imu_driver, const char *mag_driver,
                 const char *imud_version)
{
    memset(rt, 0, sizeof(*rt));
    snprintf(rt->dir, sizeof(rt->dir), "%s", dir);
    rt->max_mb     = max_mb;
    rt->max_files  = max_files;
    rt->imu_odr_hz  = imu_odr_hz;
    rt->imu_odr_mhz = imu_odr_mhz;
    snprintf(rt->imu_driver,   sizeof(rt->imu_driver),   "%s", imu_driver   ? imu_driver   : "");
    snprintf(rt->mag_driver,   sizeof(rt->mag_driver),   "%s", mag_driver   ? mag_driver   : "");
    snprintf(rt->imud_version, sizeof(rt->imud_version), "%s", imud_version ? imud_version : "");
    return cap_rot_open_file(rt);
}

int cap_rot_write(cap_rotator_t *rt, const cap_ring_rec_t *rec)
{
    if (!rt->open) return -1;

    if (rt->max_mb > 0 &&
        rt->w.bytes >= (uint64_t)rt->max_mb * 1024u * 1024u) {
        cap_writer_close(&rt->w);
        rt->open = false;
        if (cap_rot_open_file(rt) != 0) return -1;
    }

    switch (rec->type) {
    case CAP_REC_IMU:
        return cap_writer_imu(&rt->w, &rec->u.imu, rec->mono_ns);
    case CAP_REC_MAG:
        return cap_writer_mag(&rt->w, &rec->u.mag, rec->mono_ns);
    default:
        return 0;   /* ignore types the ring shouldn't carry */
    }
}

int cap_rot_flush(cap_rotator_t *rt)
{
    return rt->open ? cap_writer_flush(&rt->w) : -1;
}

void cap_rot_close(cap_rotator_t *rt)
{
    if (rt->open) {
        cap_writer_close(&rt->w);
        rt->open = false;
    }
}

const char *cap_rot_path(const cap_rotator_t *rt)
{
    return rt->open ? rt->cur_path : "";
}

uint64_t cap_rot_bytes(const cap_rotator_t *rt)
{
    return rt->open ? rt->w.bytes : 0;
}
