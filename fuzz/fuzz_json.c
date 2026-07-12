/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_json.c — libFuzzer harness for the position-source JSON field
 * extractor (pos_json_double in src/position.c), which parses untrusted
 * gpsd / Signal K responses off the network.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "imu.h"
#include "position.h"

/* position.c references the daemon's setters; the parser under test never
 * calls them from this harness — stub them out (same trick as
 * test/test_position.c). */
void imu_ctx_set_declination(imu_ctx_t *ctx, float deg, bool valid)
{ (void)ctx; (void)deg; (void)valid; }
void imu_ctx_set_mag_ref(imu_ctx_t *ctx, float h, float z)
{ (void)ctx; (void)h; (void)z; }
void imu_ctx_set_speed(imu_ctx_t *ctx, float mps, bool valid)
{ (void)ctx; (void)mps; (void)valid; }

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    char *json = malloc(size + 1);
    if (!json) return 0;
    memcpy(json, data, size);
    json[size] = '\0';

    double v;
    pos_json_double(json, "lat",    &v);
    pos_json_double(json, "lon",    &v);
    pos_json_double(json, "alt",    &v);
    pos_json_double(json, "altMSL", &v);
    pos_json_double(json, "speed",  &v);
    pos_json_double(json, "value",  &v);

    free(json);
    return 0;
}
