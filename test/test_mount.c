/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/* test_mount.c — unit tests for rotation parsing: [mount] and the
 * per-sensor [imu]/[mag] overrides */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "config.h"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR_D(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

static const char *write_tmpconf(int id, const char *content)
{
    static char path[64];
    snprintf(path, sizeof(path), "/tmp/imud_test_mount_%d.conf", id);
    FILE *f = fopen(path, "w");
    if (!f) { perror("fopen tmp"); exit(1); }
    fputs(content, f);
    fclose(f);
    return path;
}

static void test_euler_identity(void)
{
    puts("test_euler_identity");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(1, "[mount]\nrotation_euler_deg = [0.0, 0.0, 0.0]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load euler identity returns 0");
    EXPECT(cfg.mount_set == true, "mount_set true");
    EXPECT_NEAR_D(cfg.mount_euler_deg[0], 0.0, 1e-9, "roll 0");
    EXPECT_NEAR_D(cfg.mount_euler_deg[1], 0.0, 1e-9, "pitch 0");
    EXPECT_NEAR_D(cfg.mount_euler_deg[2], 0.0, 1e-9, "yaw 0");
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        double expect = (i==j) ? 1.0 : 0.0;
        char msg[80]; snprintf(msg, sizeof(msg), "identity rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], expect, 1e-9, msg);
    }
    remove(path);
    (void)fb;
}

static void test_euler_yaw_90(void)
{
    puts("test_euler_yaw_90");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(2, "[mount]\nrotation_euler_deg = [0.0, 0.0, 90.0]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load euler yaw 90 returns 0");
    EXPECT(cfg.mount_set == true, "mount_set true");
    EXPECT_NEAR_D(cfg.mount_euler_deg[2], 90.0, 1e-9, "yaw 90");
    /* Expected Rz(90) */
    double expect[3][3] = { {0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0} };
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        char msg[80]; snprintf(msg, sizeof(msg), "yaw90 rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], expect[i][j], 1e-9, msg);
    }
    remove(path);
    (void)fb;
}

static void test_euler_parse(void)
{
    puts("test_euler_parse");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(3, "[mount]\nrotation_euler_deg = [10.0, 20.0, 30.0]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load euler returns 0");
    EXPECT(cfg.mount_set == true, "mount_set true");
    EXPECT_NEAR_D(cfg.mount_euler_deg[0], 10.0, 1e-9, "roll 10");
    EXPECT_NEAR_D(cfg.mount_euler_deg[1], 20.0, 1e-9, "pitch 20");
    EXPECT_NEAR_D(cfg.mount_euler_deg[2], 30.0, 1e-9, "yaw 30");

    /* Compute reference matrix using same math as config.c */
    double roll = 10.0 * (M_PI / 180.0);
    double pitch = 20.0 * (M_PI / 180.0);
    double yaw = 30.0 * (M_PI / 180.0);
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);
    double Rx[3][3] = { {1,0,0}, {0, cr, -sr}, {0, sr, cr} };
    double Ry[3][3] = { {cp,0,sp}, {0,1,0}, {-sp,0,cp} };
    double Rz[3][3] = { {cy,-sy,0}, {sy,cy,0}, {0,0,1} };
    double tmp[3][3], R[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        tmp[i][j] = 0.0;
        for (int k=0;k<3;k++) tmp[i][j] += Ry[i][k] * Rx[k][j];
    }
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        R[i][j] = 0.0;
        for (int k=0;k<3;k++) R[i][j] += Rz[i][k] * tmp[k][j];
    }
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        char msg[80]; snprintf(msg, sizeof(msg), "euler rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], R[i][j], 1e-9, msg);
    }

    remove(path);
    (void)fb;
}

static void test_euler_yaw_180(void)
{
    puts("test_euler_yaw_180");
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(4, "[mount]\nrotation_euler_deg = [0.0, 0.0, 180.0]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load euler yaw 180 returns 0");
    EXPECT_NEAR_D(cfg.mount_euler_deg[2], 180.0, 1e-9, "yaw 180");
    /* Rz(180°) = [[-1,0,0],[0,-1,0],[0,0,1]] */
    double expect[3][3] = { {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0} };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        char msg[80]; snprintf(msg, sizeof(msg), "yaw180 rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], expect[i][j], 1e-9, msg);
    }
    remove(path);
}

static void test_euler_roll_90(void)
{
    puts("test_euler_roll_90");
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(5, "[mount]\nrotation_euler_deg = [90.0, 0.0, 0.0]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load euler roll 90 returns 0");
    EXPECT_NEAR_D(cfg.mount_euler_deg[0], 90.0, 1e-9, "roll 90");
    /* Rx(90°) = [[1,0,0],[0,0,-1],[0,1,0]] */
    double expect[3][3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0} };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        char msg[80]; snprintf(msg, sizeof(msg), "roll90 rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], expect[i][j], 1e-9, msg);
    }
    remove(path);
}

static void test_euler_pitch_90(void)
{
    puts("test_euler_pitch_90");
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(6, "[mount]\nrotation_euler_deg = [0.0, 90.0, 0.0]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load euler pitch 90 returns 0");
    EXPECT_NEAR_D(cfg.mount_euler_deg[1], 90.0, 1e-9, "pitch 90");
    /* Ry(90°) = [[0,0,1],[0,1,0],[-1,0,0]] */
    double expect[3][3] = { {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0} };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        char msg[80]; snprintf(msg, sizeof(msg), "pitch90 rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], expect[i][j], 1e-9, msg);
    }
    remove(path);
}

/*
 * preset is gone.  Left to the unknown-key warning it would start the daemon
 * with the samples unrotated, which is the same silent bias an unrecognised
 * preset name was made fatal to prevent.
 */
static void test_preset_retired_is_fatal(void)
{
    puts("test_preset_retired_is_fatal");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(10, "[mount]\npreset = \"yaw_180\"\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE, "a retired preset key is a parse error");
    EXPECT(cfg.mount_set == false, "and does not set the mount");
    remove(path);
    (void)fb;
}

/*
 * A sensor's own rotation replaces [mount] for that sensor alone.  The whole
 * point is that the two can differ: a compass on a bulkhead needs its own
 * frame, and a heading is wrong by exactly the angle between them if it does
 * not get one.
 */
static void test_per_sensor_rotation(void)
{
    puts("test_per_sensor_rotation");
    int fb = g_fail;
    imud_config_t cfg;

    /* Each section parses into its own members, and leaves the others alone. */
    config_defaults(&cfg);
    const char *path = write_tmpconf(11,
        "[mount]\nrotation_euler_deg = [0.0, 0.0, 180.0]\n"
        "[imu]\nrotation_euler_deg = [0.0, 0.0, 90.0]\n"
        "[mag]\nrotation_euler_deg = [0.0, 0.0, 270.0]\n");
    EXPECT(config_load(path, &cfg) == 0, "three rotations load");
    EXPECT(cfg.mount_set && cfg.imu_rot_set && cfg.mag_rot_set,
           "each section sets its own flag");
    EXPECT_NEAR_D(cfg.mount_euler_deg[2],   180.0, 1e-9, "[mount] keeps 180");
    EXPECT_NEAR_D(cfg.imu_rot_euler_deg[2],  90.0, 1e-9, "[imu] keeps 90");
    EXPECT_NEAR_D(cfg.mag_rot_euler_deg[2], 270.0, 1e-9, "[mag] keeps 270");
    /* Rz(90) into imu_rot, so the matrices are built per target too. */
    EXPECT_NEAR_D(cfg.imu_rot[0][1], -1.0, 1e-9, "[imu] rot is Rz(90)");
    EXPECT_NEAR_D(cfg.imu_rot[1][0],  1.0, 1e-9, "[imu] rot is Rz(90)");
    EXPECT_NEAR_D(cfg.mag_rot[0][1],  1.0, 1e-9, "[mag] rot is Rz(270)");
    EXPECT_NEAR_D(cfg.mag_rot[1][0], -1.0, 1e-9, "[mag] rot is Rz(270)");
    remove(path);

    /* [mount] alone leaves both sensor flags clear — that is what makes the
     * fallback, and every existing config, behave as it did. */
    config_defaults(&cfg);
    path = write_tmpconf(12, "[mount]\nrotation_euler_deg = [0.0, 0.0, 180.0]\n");
    EXPECT(config_load(path, &cfg) == 0, "[mount] alone loads");
    EXPECT(cfg.mount_set, "mount set");
    EXPECT(!cfg.imu_rot_set && !cfg.mag_rot_set,
           "and neither sensor claims a rotation of its own");
    remove(path);

    /* One sensor overriding does not set the other, nor [mount]. */
    config_defaults(&cfg);
    path = write_tmpconf(13, "[mag]\nrotation_euler_deg = [0.0, 0.0, 90.0]\n");
    EXPECT(config_load(path, &cfg) == 0, "[mag] alone loads");
    EXPECT(cfg.mag_rot_set, "[mag] set");
    EXPECT(!cfg.mount_set && !cfg.imu_rot_set,
           "[mag] alone sets neither [mount] nor [imu]");
    remove(path);

    /* The array is validated per section, not only under [mount]. */
    config_defaults(&cfg);
    path = write_tmpconf(14, "[imu]\nrotation_euler_deg = [1.0, 2.0]\n");
    EXPECT(config_load(path, &cfg) == CONFIG_ERR_PARSE,
           "a short array under [imu] is a parse error");
    EXPECT(!cfg.imu_rot_set, "and leaves the sensor unrotated");
    remove(path);

    (void)fb;
}

/*
 * A short array must not be accepted silently (only a completely empty one
 * errored), leaving the missing angles at their defaults — e.g. "[0, 0]"
 * would quietly mean yaw = 0 rather than being flagged as incomplete.
 */
static void test_euler_partial_array_rejected(void)
{
    puts("test_euler_partial_array_rejected");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(11,
        "[mount]\nrotation_euler_deg = [10.0, 20.0]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE, "two-element euler array is a parse error");
    remove(path);

    config_defaults(&cfg);
    path = write_tmpconf(12,
        "[mount]\nrotation_euler_deg = [10.0, 20.0, 30.0, 40.0]\n");
    rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE, "four-element euler array is a parse error");
    remove(path);
    (void)fb;
}

/* A valid 3×3 rotation given directly must be accepted verbatim. */
static void test_rotation_matrix_accepted(void)
{
    puts("test_rotation_matrix_accepted");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    /* Rz(90°): x→y, y→−x */
    const char *path = write_tmpconf(13,
        "[mount]\nrotation_matrix = [0,-1,0, 1,0,0, 0,0,1]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "valid rotation_matrix loads");
    EXPECT(cfg.mount_set == true, "mount_set true for rotation_matrix");
    EXPECT_NEAR_D(cfg.mount_rot[0][1], -1.0, 1e-12, "rot[0][1] = -1");
    EXPECT_NEAR_D(cfg.mount_rot[1][0],  1.0, 1e-12, "rot[1][0] = 1");
    EXPECT_NEAR_D(cfg.mount_rot[2][2],  1.0, 1e-12, "rot[2][2] = 1");
    remove(path);
    (void)fb;
}

/*
 * The orthonormality check is the one A5 asked for. It is only reachable via
 * rotation_matrix — euler_deg_to_rot output is orthonormal by construction.
 * Both failure modes must be caught: a non-orthonormal matrix, and a
 * reflection (det = −1), which an orthogonality-only test would accept.
 */
static void test_rotation_matrix_validated(void)
{
    puts("test_rotation_matrix_validated");
    int fb = g_fail;
    imud_config_t cfg;

    config_defaults(&cfg);
    const char *path = write_tmpconf(14,
        "[mount]\nrotation_matrix = [1,0,0, 0,1,0, 0,0,1.5]\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE, "non-orthonormal matrix rejected");
    remove(path);

    config_defaults(&cfg);
    path = write_tmpconf(15,
        "[mount]\nrotation_matrix = [1,0,0, 0,1,0, 0,0,-1]\n");
    rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE, "reflection (det = -1) rejected");
    remove(path);

    config_defaults(&cfg);
    path = write_tmpconf(16,
        "[mount]\nrotation_matrix = [1,0,0, 0,1,0]\n");
    rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE, "short rotation_matrix rejected");
    remove(path);
    (void)fb;
}

int main(void)
{
    puts("=== imud mount tests ===");
    test_euler_identity();
    test_euler_yaw_90();
    test_euler_yaw_180();
    test_euler_roll_90();
    test_euler_pitch_90();
    test_euler_parse();
    test_preset_retired_is_fatal();
    test_per_sensor_rotation();
    test_euler_partial_array_rejected();
    test_rotation_matrix_accepted();
    test_rotation_matrix_validated();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
