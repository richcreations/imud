/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_config.c — libFuzzer harness for the TOML config parser
 * (src/config.c).  config_load takes a path, so each input lands in a
 * per-process temp file first.  Run with -close_fd_mask=3 to silence the
 * parser's per-line error logging.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "config.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static char path[64];
    if (!path[0])
        snprintf(path, sizeof(path), "/tmp/imud_fuzz_cfg_%d", (int)getpid());

    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    if (size) fwrite(data, 1, size, f);
    fclose(f);

    imud_config_t cfg;
    config_defaults(&cfg);
    config_load(path, &cfg);
    return 0;
}
