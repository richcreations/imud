/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_capture.c — libFuzzer harness for the .imucap reader
 * (src/capture.c).  Capture files are exactly the artifact users share for
 * bug reports ("send me your capture"), so the reader must survive
 * arbitrary bytes.  Also exercises rewind + a second pass.
 */

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "capture.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static char path[64];
    if (!path[0])
        snprintf(path, sizeof(path), "/tmp/imud_fuzz_cap_%d", (int)getpid());

    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    if (size) fwrite(data, 1, size, f);
    fclose(f);

    cap_reader_t r;
    if (cap_reader_open(&r, path) != 0) return 0;

    cap_record_t rec;
    int guard = 0;
    while (cap_reader_next(&r, &rec) == 1 && ++guard < 100000) { }
    cap_reader_rewind(&r);
    guard = 0;
    while (cap_reader_next(&r, &rec) == 1 && ++guard < 100000) { }
    cap_reader_close(&r);
    return 0;
}
