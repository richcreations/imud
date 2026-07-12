/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * standalone_main.c — corpus driver for building the fuzz harnesses
 * WITHOUT libFuzzer (e.g. Apple clang): feeds each file named on the
 * command line to LLVMFuzzerTestOneInput once.  CI links the real fuzzer
 * instead (clang -fsanitize=fuzzer) and never compiles this file.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        FILE *f = fopen(argv[i], "rb");
        if (!f) { perror(argv[i]); return 1; }
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        uint8_t *buf = malloc(sz > 0 ? (size_t)sz : 1);
        if (sz > 0 && fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
            fclose(f); free(buf); return 1;
        }
        fclose(f);
        LLVMFuzzerTestOneInput(buf, (size_t)sz);
        free(buf);
        printf("ok %s (%ld bytes)\n", argv[i], sz);
    }
    return 0;
}
