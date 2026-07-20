/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imu_math.c — pure helpers factored out of imu.c (see imu_math.h).
 *
 * Behaviour is identical to the former static definitions in imu.c; the move
 * only makes them externally linkable so test/test_imu_math.c can exercise
 * them directly.
 */

#include <stdlib.h>   /* abs */

#include "imu_math.h"

/* ── Calibration helpers ─────────────────────────────────────────────────── */

/*
 * Apply accel cal from cal.json on top of the driver's chip-level scaling.
 * Gyro bias is NOT subtracted here — the MEKF subtracts it during predict.
 */
void apply_imu_cal(const imud_cal_t *cal, imu_sample_t *s)
{
    /* Gyro bias/temperature compensation (imud-cal fit-temp): remove the
     * temperature-tracking bias component here so the MEKF's random-walk
     * estimator only has to follow the residual. */
    if (cal->has_gyro_temp) {
        float dT = s->temp_c - cal->gyro_temp_ref_c;
        s->gyro[0] -= cal->gyro_temp_coeff[0] * dT;
        s->gyro[1] -= cal->gyro_temp_coeff[1] * dT;
        s->gyro[2] -= cal->gyro_temp_coeff[2] * dT;
    }

    if (!cal->has_accel) return;
    for (int i = 0; i < 3; i++)
        s->accel[i] = (s->accel[i] - cal->accel_offset[i]) * cal->accel_scale[i];
}

/* Apply hard/soft-iron correction: m_cal = soft_iron × (m_raw − hard_iron). */
void apply_mag_cal(const imud_cal_t *cal, mag_sample_t *s)
{
    if (!cal->has_mag) return;
    float tmp[3];
    for (int i = 0; i < 3; i++)
        tmp[i] = s->field[i] - cal->mag_hard_iron[i];
    for (int i = 0; i < 3; i++)
        s->field[i] = cal->mag_soft_iron[i][0] * tmp[0]
                    + cal->mag_soft_iron[i][1] * tmp[1]
                    + cal->mag_soft_iron[i][2] * tmp[2];
}

/* ── Timestamp helpers ───────────────────────────────────────────────────── */

uint64_t ts_ns(const struct timespec *t)
{
    return (uint64_t)t->tv_sec * 1000000000ULL + (uint64_t)t->tv_nsec;
}

void anchor_update(ts_anchor_t *a, uint32_t chip_ts,
                   uint64_t wall_ns, uint64_t tai_ns)
{
    pthread_mutex_lock(&a->mtx);
    a->chip_ticks = chip_ts;
    a->wall_ns    = wall_ns;
    a->tai_ns     = tai_ns;
    a->gen++;
    pthread_mutex_unlock(&a->mtx);
}

/*
 * Convert a chip counter value to wall + TAI timestamps.
 * Uses 32-bit wrapping arithmetic — safe up to 2^32 ticks between anchors
 * (~29.8 h at the ST parts' 25 µs/tick, ~71.6 min at the ICM-42688-P's
 * 1 µs/tick); the 60 s anchor refresh keeps deltas far below either bound.
 * tick_ns comes from imu_ops_t.ts_tick_ns (0 when !has_hw_timestamp, where
 * chip_ts is always 0 and the offset degenerates to 0 as intended).
 */
void chip_to_wall(ts_anchor_t *a, uint32_t chip_ts, uint32_t tick_ns,
                  uint64_t *wall_out, uint64_t *tai_out,
                  uint32_t *gen_out)
{
    pthread_mutex_lock(&a->mtx);
    uint64_t offset = (uint64_t)(chip_ts - a->chip_ticks) * tick_ns;
    if (wall_out) *wall_out = a->wall_ns + offset;
    if (tai_out)  *tai_out  = a->tai_ns  + offset;
    if (gen_out)  *gen_out  = a->gen;
    pthread_mutex_unlock(&a->mtx);
}

/* ── Utilities ───────────────────────────────────────────────────────────── */

int nearest_odr(const int supported[], int requested)
{
    int best = supported[0], best_diff = abs(supported[0] - requested);
    for (int i = 1; supported[i] != 0; i++) {
        int d = abs(supported[i] - requested);
        if (d < best_diff) { best = supported[i]; best_diff = d; }
    }
    return best;
}

/* Apply mount rotation (board -> body) if configured. In-place on v. */
void apply_mount_rot_if_set(const imud_config_t *cfg, float v[3])
{
    if (!cfg->mount_set) return;
    double out0 = cfg->mount_rot[0][0] * v[0]
                + cfg->mount_rot[0][1] * v[1]
                + cfg->mount_rot[0][2] * v[2];
    double out1 = cfg->mount_rot[1][0] * v[0]
                + cfg->mount_rot[1][1] * v[1]
                + cfg->mount_rot[1][2] * v[2];
    double out2 = cfg->mount_rot[2][0] * v[0]
                + cfg->mount_rot[2][1] * v[1]
                + cfg->mount_rot[2][2] * v[2];
    v[0] = (float)out0; v[1] = (float)out1; v[2] = (float)out2;
}
