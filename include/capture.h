/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * capture.h — raw-sample capture file format (.imucap) + black-box plumbing
 *
 * The capture file is imud's regression corpus format: every raw sample from
 * both sensors, EXACTLY as the driver's read() delivered it — before mount
 * rotation, before calibration, before bias correction.  Replaying a capture
 * (sim driver, [device] sim_file) therefore traverses the identical daemon
 * pipeline, and a capture can be re-run against a different cal.json, mount,
 * or filter tuning.
 *
 * Layout (little-endian, packed; the LE-only build guard lives in types.h):
 *
 *   cap_header_t                  104 bytes, magic "IMUCAP1\0"
 *   repeated records:
 *     cap_frame_t                 12 bytes  {type, flags, len, mono_ns}
 *     payload                     len bytes (cap_imu_rec_t / cap_mag_rec_t /
 *                                            cap_mark_rec_t / future types)
 *
 * Robustness rules:
 *   - a truncated tail (power loss mid-write) reads as clean EOF
 *   - unknown record types are skipped via len (forward compatible)
 *   - a payload longer than the struct we know is read then skipped past
 *     (a future writer may append fields — same append-only discipline as
 *     the wire packet)
 *
 * This TU is pure (no logging, no daemon deps) so tools and tests link it
 * standalone; callers do their own logging.
 */

#ifndef IMUD_CAPTURE_H
#define IMUD_CAPTURE_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "types.h"

/* ── File format ─────────────────────────────────────────────────────────── */

#define CAP_MAGIC        "IMUCAP1"      /* 7 chars + NUL = 8 bytes on disk */
#define CAP_FMT_VERSION  1u

typedef struct __attribute__((packed)) {
    char     magic[8];        /* "IMUCAP1\0" */
    uint16_t version;         /* CAP_FMT_VERSION */
    uint16_t hdr_len;         /* byte offset of the first record */
    uint32_t imu_odr_hz;      /* configured IMU output data rate */
    char     imu_driver[16];  /* driver names, NUL-terminated */
    char     mag_driver[16];
    char     imud_version[16];/* IMUD_VERSION_STR of the writer */
    uint64_t t0_wall_ns;      /* CLOCK_REALTIME at capture start */
    uint64_t t0_mono_ns;      /* CLOCK_MONOTONIC at capture start; record
                               * mono_ns values are absolute on that clock */
    uint8_t  reserved[24];    /* zeroed; room for future header fields */
} cap_header_t;

_Static_assert(sizeof(cap_header_t) == 104, "cap_header_t must be 104 bytes");

/* Record types */
#define CAP_REC_IMU   1u
#define CAP_REC_MAG   2u
#define CAP_REC_MARK  3u

/* Frame flag bits */
#define CAP_MAGF_VALID 0x01u  /* CAP_REC_MAG: mag_sample_t.valid */

typedef struct __attribute__((packed)) {
    uint8_t  type;            /* CAP_REC_* */
    uint8_t  flags;           /* per-type flag bits */
    uint16_t len;             /* payload bytes following this frame */
    uint64_t mono_ns;         /* CLOCK_MONOTONIC when the driver delivered it */
} cap_frame_t;

_Static_assert(sizeof(cap_frame_t) == 12, "cap_frame_t must be 12 bytes");

typedef struct __attribute__((packed)) {  /* CAP_REC_IMU payload */
    float    accel[3];        /* m/s², as the driver delivered (pre-mount/cal) */
    float    gyro[3];         /* rad/s, as the driver delivered */
    float    temp_c;
    uint32_t chip_ts;         /* hw timestamp ticks (25 µs on ISM330; 0 = none) */
    uint32_t seq;             /* driver's monotonic sample counter */
} cap_imu_rec_t;

_Static_assert(sizeof(cap_imu_rec_t) == 36, "cap_imu_rec_t must be 36 bytes");

typedef struct __attribute__((packed)) {  /* CAP_REC_MAG payload */
    float    field[3];        /* µT, as the driver delivered (pre-mount/cal) */
    uint64_t wall_ns;         /* CLOCK_REALTIME the driver stamped */
} cap_mag_rec_t;

_Static_assert(sizeof(cap_mag_rec_t) == 20, "cap_mag_rec_t must be 20 bytes");

typedef struct __attribute__((packed)) {  /* CAP_REC_MARK payload */
    uint32_t code;            /* annotation code; reserved for future use */
} cap_mark_rec_t;

/* ── Writer ──────────────────────────────────────────────────────────────── */

typedef struct {
    FILE    *f;
    uint64_t bytes;           /* total bytes written incl. header */
    uint64_t n_imu;
    uint64_t n_mag;
} cap_writer_t;

/*
 * Create path and write the header.  Returns 0, or -1 on I/O error (errno
 * set).  Driver/version strings are truncated to fit their fields.
 */
int cap_writer_open(cap_writer_t *w, const char *path,
                    uint32_t imu_odr_hz,
                    const char *imu_driver, const char *mag_driver,
                    const char *imud_version,
                    uint64_t t0_wall_ns, uint64_t t0_mono_ns);

/* Append one record.  Return 0, or -1 on I/O error. */
int cap_writer_imu (cap_writer_t *w, const imu_sample_t *s, uint64_t mono_ns);
int cap_writer_mag (cap_writer_t *w, const mag_sample_t *m, uint64_t mono_ns);
int cap_writer_mark(cap_writer_t *w, uint32_t code,        uint64_t mono_ns);

int  cap_writer_flush(cap_writer_t *w);   /* fflush; 0 / -1 */
void cap_writer_close(cap_writer_t *w);

/* ── Reader ──────────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t      type;        /* CAP_REC_* */
    uint64_t     mono_ns;
    imu_sample_t imu;         /* filled for CAP_REC_IMU (accel_raw zeroed —
                               * the daemon's reader thread rebuilds it) */
    mag_sample_t mag;         /* filled for CAP_REC_MAG (field_raw zeroed) */
    uint32_t     mark;        /* filled for CAP_REC_MARK */
} cap_record_t;

typedef struct {
    FILE        *f;
    cap_header_t hdr;         /* validated header, available to the caller */
} cap_reader_t;

#define CAP_ERR_IO     (-1)   /* open/read failure (errno set) */
#define CAP_ERR_FORMAT (-2)   /* bad magic, unsupported version, bad header */

/* Open and validate.  0 ok, CAP_ERR_IO or CAP_ERR_FORMAT otherwise. */
int cap_reader_open(cap_reader_t *r, const char *path);

/*
 * Read the next known record.  Unknown types are skipped silently.
 * Returns 1 with *out filled, 0 on end of file (a truncated trailing
 * record also ends the stream cleanly), CAP_ERR_IO on read error.
 */
int cap_reader_next(cap_reader_t *r, cap_record_t *out);

int  cap_reader_rewind(cap_reader_t *r);  /* back to first record; 0 / -1 */
void cap_reader_close(cap_reader_t *r);

/* ── Tap ring (daemon black box) ─────────────────────────────────────────── */

/*
 * Reader threads push raw samples here right after the driver read();
 * the capture writer thread drains in batches.  Push NEVER blocks on I/O:
 * when full, the newest record is dropped and counted (history stays
 * contiguous; gaps are visible in seq).  No condvar — the writer thread
 * polls every few tens of ms, which is far below the ring's capacity at
 * any supported ODR and keeps this portable (macOS tests).
 */
#define CAP_RING_LEN 1024

typedef struct {
    uint8_t  type;            /* CAP_REC_IMU / CAP_REC_MAG */
    uint64_t mono_ns;
    union {
        imu_sample_t imu;
        mag_sample_t mag;
    } u;
} cap_ring_rec_t;

typedef struct {
    cap_ring_rec_t  buf[CAP_RING_LEN];
    int             head, tail, count;
    uint64_t        dropped;
    pthread_mutex_t mtx;
} cap_ring_t;

void cap_ring_init(cap_ring_t *r);
void cap_ring_destroy(cap_ring_t *r);
/* Return 0 stored, 1 dropped (ring full). */
int  cap_ring_push_imu(cap_ring_t *r, const imu_sample_t *s, uint64_t mono_ns);
int  cap_ring_push_mag(cap_ring_t *r, const mag_sample_t *m, uint64_t mono_ns);
/* Drain up to max records; returns the number popped (0 = empty). */
int  cap_ring_pop(cap_ring_t *r, cap_ring_rec_t *out, int max);
uint64_t cap_ring_dropped(cap_ring_t *r);

/* ── Rotating black-box writer ───────────────────────────────────────────── */

/*
 * Manages capture files in a directory: imud-YYYYMMDD-HHMMSS.imucap (UTC,
 * so names sort chronologically).  A new file is started when the current
 * one exceeds max_mb; the oldest imud-*.imucap files are deleted so at most
 * max_files remain.  0 disables the respective limit.
 */
typedef struct {
    char         dir[256];
    uint32_t     max_mb;
    uint32_t     max_files;
    uint32_t     imu_odr_hz;
    char         imu_driver[16];
    char         mag_driver[16];
    char         imud_version[16];
    cap_writer_t w;
    bool         open;
    char         cur_path[320];
    time_t       last_stamp;  /* newest stamp used; keeps names monotonic */
} cap_rotator_t;

/* Opens the first file immediately.  0 ok, -1 on error (errno set). */
int cap_rot_open(cap_rotator_t *rt, const char *dir,
                 uint32_t max_mb, uint32_t max_files,
                 uint32_t imu_odr_hz,
                 const char *imu_driver, const char *mag_driver,
                 const char *imud_version);

/* Write one tap-ring record, rotating when the size cap is hit.  0 / -1. */
int cap_rot_write(cap_rotator_t *rt, const cap_ring_rec_t *rec);

int  cap_rot_flush(cap_rotator_t *rt);
void cap_rot_close(cap_rotator_t *rt);

/* Current file path and its size so far (status reporting). */
const char *cap_rot_path (const cap_rotator_t *rt);
uint64_t    cap_rot_bytes(const cap_rotator_t *rt);

#endif /* IMUD_CAPTURE_H */
