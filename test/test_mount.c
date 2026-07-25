/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/* test_mount.c — unit tests for mount presets and Euler parsing */

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

static void test_preset_identity(void)
{
    puts("test_preset_identity");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(1, "[mount]\npreset = \"identity\"\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load identity returns 0");
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

static void test_preset_yaw_90(void)
{
    puts("test_preset_yaw_90");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(2, "[mount]\npreset = \"yaw_90\"\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load yaw_90 returns 0");
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

static void test_preset_yaw_180(void)
{
    puts("test_preset_yaw_180");
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(4, "[mount]\npreset = \"yaw_180\"\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load yaw_180 returns 0");
    EXPECT_NEAR_D(cfg.mount_euler_deg[2], 180.0, 1e-9, "yaw 180");
    /* Rz(180°) = [[-1,0,0],[0,-1,0],[0,0,1]] */
    double expect[3][3] = { {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0} };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        char msg[80]; snprintf(msg, sizeof(msg), "yaw180 rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], expect[i][j], 1e-9, msg);
    }
    remove(path);
}

static void test_preset_roll_90(void)
{
    puts("test_preset_roll_90");
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(5, "[mount]\npreset = \"roll_90\"\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load roll_90 returns 0");
    EXPECT_NEAR_D(cfg.mount_euler_deg[0], 90.0, 1e-9, "roll 90");
    /* Rx(90°) = [[1,0,0],[0,0,-1],[0,1,0]] */
    double expect[3][3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0} };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        char msg[80]; snprintf(msg, sizeof(msg), "roll90 rot[%d][%d]", i, j);
        EXPECT_NEAR_D(cfg.mount_rot[i][j], expect[i][j], 1e-9, msg);
    }
    remove(path);
}

static void test_preset_pitch_90(void)
{
    puts("test_preset_pitch_90");
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(6, "[mount]\npreset = \"pitch_90\"\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == 0, "config_load pitch_90 returns 0");
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
 * An unrecognised preset name used to be a warning that still left
 * mount_set = true with whatever Euler angles were already in the struct —
 * i.e. the daemon ran with a silently wrong mount rotation biasing every
 * sample. It must now be a hard parse error.
 */
static void test_preset_unknown_is_fatal(void)
{
    puts("test_preset_unknown_is_fatal");
    int fb = g_fail;
    imud_config_t cfg;
    config_defaults(&cfg);
    const char *path = write_tmpconf(10, "[mount]\npreset = \"yaw_45\"\n");
    int rc = config_load(path, &cfg);
    EXPECT(rc == CONFIG_ERR_PARSE, "unknown preset is a parse error");
    remove(path);
    (void)fb;
}

/*
 * A short array used to be accepted silently (only a completely empty one
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
    test_preset_identity();
    test_preset_yaw_90();
    test_preset_yaw_180();
    test_preset_roll_90();
    test_preset_pitch_90();
    test_euler_parse();
    test_preset_unknown_is_fatal();
    test_euler_partial_array_rejected();
    test_rotation_matrix_accepted();
    test_rotation_matrix_validated();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
