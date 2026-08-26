/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_argv.c — libFuzzer harness for the five CLI front-ends in src/cli.c.
 *
 * argv crosses no trust boundary: it is the operator's own input, not a file
 * or a socket.  It is fuzzed anyway because the parsers do real work on
 * arbitrary strings — strtol/strtod conversions with range checks,
 * fixed-buffer snprintf copies, and a positional-mode lookup — across five
 * entry points that were written at different times.  parse_int's ERANGE
 * guard is also ABI-conditional (`#if LONG_MAX > INT_MAX`), so amd64 and
 * armhf take different paths through it.
 *
 * The input is split on NUL into tokens, which is what an argv already is.
 * argv[0] is supplied rather than taken from the input: no parser inspects it,
 * and letting the fuzzer spend entropy there would waste it.
 *
 * All five parsers see the same argv.  They keep `const char *` members
 * pointing into it (imud-cal's --output, imutest's driver overrides), so the
 * buffer has to outlive every call in the iteration — it does, and nothing
 * dereferences those after it is freed.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cli.h"

/* argv is a command line, not a stream: past a few dozen tokens the fuzzer is
 * exploring length rather than logic. */
#define MAX_ARGS  48
#define MAX_INPUT 4096

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (size > MAX_INPUT)
        return 0;

    char *buf = malloc(size + 1);
    if (!buf)
        return 0;
    memcpy(buf, data, size);
    buf[size] = '\0';          /* terminates a trailing token with no NUL */

    char *argv[MAX_ARGS + 2];
    int   argc = 0;
    argv[argc++] = (char *)"imud";

    size_t off = 0;
    while (off < size && argc < MAX_ARGS)
        argv[argc++] = buf + off, off += strlen(buf + off) + 1;
    argv[argc] = NULL;

    /* A fresh output struct per parser: each memsets its own, but a shared one
     * would let one parser's leftovers decide another's branches. */
    cli_imud_t    a_imud;
    cli_cal_t     a_cal;
    cli_mon_t     a_mon;
    cli_status_t  a_status;
    cli_imutest_t a_imutest;

    cli_parse_imud   (argc, argv, &a_imud);
    cli_parse_cal    (argc, argv, &a_cal);
    cli_parse_mon    (argc, argv, &a_mon);
    cli_parse_status (argc, argv, &a_status);
    cli_parse_imutest(argc, argv, &a_imutest);

    free(buf);
    return 0;
}
