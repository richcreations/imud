/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_cal.c — libFuzzer harness for the cal.json parser (src/cal.c).
 * cal_load takes a path, so each input lands in a per-process temp file
 * first, exactly like fuzz_config.
 *
 * The parser is hand-rolled — parse_float_array and parse_scalar walk the
 * buffer with pointer arithmetic looking for "key": [a, b, c] — which is
 * the shape that rewards fuzzing.  But a file that parses cleanly can still
 * hold NaN, Inf, or absurd scale factors, and those go straight into
 * apply_imu_cal / apply_mag_cal on the sample hot path.  So a successful
 * load is followed by applying the calibration to sample values, including
 * non-finite ones: a NaN soft-iron matrix silently poisoning the filter is
 * a worse outcome than a rejected file.
 *
 * Run with -close_fd_mask=3 to silence the parser's error logging.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "cal.h"
#include "imu_math.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static char path[64];
    if (!path[0])
        snprintf(path, sizeof(path), "/tmp/imud_fuzz_cal_%d", (int)getpid());

    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    if (size) fwrite(data, 1, size, f);
    fclose(f);

    imud_cal_t cal;
    if (cal_load(path, &cal) < 0)
        return 0;               /* rejected — that is the parser working */

    /*
     * Feed the loaded calibration the values a real driver can produce,
     * including the non-finite ones a wedged sensor yields.
     *
     * The plausible ones carry an oracle: a calibration that cal_load
     * ACCEPTED must never turn one of them into a non-finite value.  Without
     * it the harness detected only sanitizer traps, and a NaN calibration
     * quietly poisoning the filter — the outcome the header comment names —
     * is not a trap.  It is a silently wrong number that never washes out.
     *
     * `oracle` deliberately stops at physically meaningful magnitudes.  The
     * extremes below are NOT held to it, for a reason worth recording: a
     * soft-iron element of 1e9 (finite, so cal_load accepts it) times a
     * sample of 1e30 (finite, so this loop would feed it) is 1e39, which
     * simply overflows a float.  Asserting on that tests IEEE 754, not imud —
     * neither operand is a value any magnetometer or calibration can hold.
     * Bounding the plausible RANGE of a calibration is a separate question
     * from bounding it to the finite ones, and this file only claims the
     * latter.
     */
    static const float oracle[]  = { 0.0f, 1.0f, -9.81f, 100.0f };
    static const float extreme[] = { 1e30f, (float)NAN, (float)INFINITY };

    for (size_t i = 0;
         i < sizeof(oracle) / sizeof(oracle[0])
             + sizeof(extreme) / sizeof(extreme[0]); i++) {
        bool  checked = i < sizeof(oracle) / sizeof(oracle[0]);
        float v = checked ? oracle[i]
                          : extreme[i - sizeof(oracle) / sizeof(oracle[0])];

        imu_sample_t s = { 0 };
        mag_sample_t m = { 0 };

        for (int k = 0; k < 3; k++) {
            s.accel[k] = s.accel_raw[k] = v;
            s.gyro[k]  = v;
            m.field[k] = m.field_raw[k] = v;
        }
        s.temp_c = v;

        apply_imu_cal(&cal, &s);
        apply_mag_cal(&cal, &m);

        if (!checked)
            continue;

        for (int k = 0; k < 3; k++) {
            if (isfinite(s.accel[k]) && isfinite(s.gyro[k]) &&
                isfinite(m.field[k]))
                continue;
            fprintf(stderr, "fuzz_cal: an accepted calibration turned the "
                            "plausible sample %g into a non-finite one "
                            "(axis %d)\n", (double)v, k);
            abort();
        }
    }
    return 0;
}
