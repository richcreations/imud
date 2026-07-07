/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * config.h — runtime configuration loaded from TOML file (§9)
 *
 * config_load() parses the file; config_defaults() fills safe defaults.
 * SIGHUP reloads the fields marked [hot]: fusion noise/threshold params,
 * output rates, stats heartbeat rate, and static declination (unless a live
 * position source owns it). Log level is applied once at startup.
 * Fields marked [restart] require a full daemon restart to take effect.
 */
#ifndef IMUD_CONFIG_H
#define IMUD_CONFIG_H

#include <stdbool.h>

typedef struct {

    /* [device] */
    char  i2c_bus[64];          /* e.g. "/dev/i2c-1" */
    char  gpio_chip[32];        /* gpiochip name: "gpiochip0" Pi 4, "gpiochip4" Pi 5 */

    /* [imu]  [restart] */
    char  imu_driver[32];       /* "ism330dhcx" */
    int   imu_addr;             /* 0x6B default, 0x6A via jumper */
    int   imu_int_gpio;         /* BCM GPIO for FIFO watermark interrupt */
    int   imu_odr_hz;           /* 833; driver rounds to nearest supported */
    int   imu_accel_g;          /* 2 | 4 | 8 | 16 */
    int   imu_gyro_dps;         /* 125 | 250 | 500 | 1000 | 2000 | 4000 */
    int   imu_fifo_wm;          /* watermark in sample-sets */

    /* [mag]  [restart] */
    char  mag_driver[32];       /* "mmc5983ma" */
    int   mag_addr;             /* 0x30 fixed */
    int   mag_int_gpio;         /* BCM GPIO for measurement-done interrupt */
    int   mag_odr_hz;           /* 100 */
    float mag_set_period_s;     /* degauss interval seconds; 0 = disable */

    /* [fusion]  [hot]: gains and thresholds */
    bool   mag_yaw_only;          /* [hot] heading-only mag fusion (marine default) */
    float  heave_tau_s;           /* [hot] heave filter time constant, s; 0 = off */
    double mekf_gyro_noise;      /* rad/s/√Hz — from datasheet */
    double mekf_gyro_bias;       /* rad/s — in-run bias instability */
    double mekf_accel_noise;     /* m/s²/√Hz — from datasheet */
    double mekf_mag_noise;       /* Gauss/√Hz — from datasheet */
    double mag_reject_gauss;         /* reject mag if residual > this (post-cal) */
    double accel_skip_thresh;        /* skip accel update if ||a|-1g| > this */
    double engine_vibration_g2;      /* EMA of (|a|-g)² threshold for engine-on (0=disabled) */
    double engine_accel_skip_thresh; /* accel_skip_thresh override when engine detected */

    /* [calibration] */
    char   cal_file[256];
    double startup_settle_sec;   /* discard sensor data for this long after chip init */
    double gyro_bias_sec;        /* stationary window at startup */

    /* [nmea]  [restart]: enabled, port, addr; [hot]: rate */
    bool  nmea_enabled;
    int   nmea_rate_hz;
    char  nmea_dest_addr[64];
    int   nmea_dest_port;

    /* [highrate]  [restart]: enabled, port, addr; [hot]: rate */
    bool  highrate_enabled;
    int   highrate_rate_hz;
    char  highrate_dest_addr[64];
    int   highrate_dest_port;
    char  highrate_coord_frame[8]; /* "NED" | "ENU" */

    /* [json]  [restart]: enabled, port, addr; [hot]: rate */
    bool  json_enabled;
    int   json_rate_hz;
    char  json_dest_addr[64];
    int   json_dest_port;

    /* [stream]  local AF_UNIX subscription stream (binary packets) */
    bool  stream_enabled;         /* [restart] */
    char  stream_socket[108];     /* [restart] listen path; sized to sun_path */
    int   stream_rate_hz;         /* [hot] per-subscriber packet rate */

    /* [logging]  [hot] */
    char  log_level[16];           /* "debug" | "info" | "warn" | "error" */
    char  log_file[256];
    int   log_stats_hz;

    /* [position]  [restart] */
    float  pos_declination_deg;   /* static magnetic declination °E+; 0 = disabled */
    bool   pos_declination_valid; /* derived, not a config key: true when declination
                                   * came from an explicit non-zero declination_deg or
                                   * a WMM computation (which may legitimately be 0.0
                                   * on the agonic line) */
    double pos_lat_deg;           /* geodetic latitude  (+N / -S); 0 = WMM disabled */
    double pos_lon_deg;           /* geodetic longitude (+E / -W); 0 = WMM disabled */
    char   pos_wmm_file[256];     /* path to WMM.COF; default /etc/imud/WMM.COF */

    /* Live position sources — gpsd (preferred) and/or SignalK HTTP poll */
    bool  pos_gpsd_enabled;       /* connect to gpsd for live lat/lon */
    char  pos_gpsd_host[64];      /* gpsd host; default "localhost" */
    int   pos_gpsd_port;          /* gpsd port; default 2947 */

    bool  pos_signalk_enabled;    /* poll SignalK REST API for lat/lon */
    char  pos_signalk_host[64];   /* SignalK host; default "localhost" */
    int   pos_signalk_port;       /* SignalK port; default 3000 */
    char  pos_signalk_path[128];  /* REST path; default /signalk/v1/api/…/position */

    float pos_fix_max_age_h;      /* hours to keep a GPS-derived declination without a new fix;
                                   * 0 = never expire; default 24.0 */

    /* [mount] — Euler angles in degrees: [roll, pitch, yaw] (ZYX order)
     * If `mount_set` is true the 3x3 rotation matrix `mount_rot` maps
     * board-frame vectors into the configured body frame: v_body = R * v_board
     */
    bool  mount_set;
    double mount_euler_deg[3];
    double mount_rot[3][3];
    char   mount_preset[32];   /* optional named preset, e.g. "identity", "yaw_90" */

} imud_config_t;

/* config_load return codes */
#define CONFIG_ERR_OPEN  (-1)  /* file could not be opened; cfg untouched */
#define CONFIG_ERR_PARSE (-2)  /* file read, but one or more values were bad */

/*
 * config_load: parse TOML file at path into cfg.
 * Returns 0 on success, CONFIG_ERR_OPEN or CONFIG_ERR_PARSE on failure.
 * A bad value does not stop the parse: every error in the file is logged
 * to stderr and all valid lines are applied, but CONFIG_ERR_PARSE is
 * returned so the caller can treat the file as unfit to run on.
 * Unknown keys/sections are warned but not fatal.
 */
int  config_load(const char *path, imud_config_t *cfg);

/*
 * config_defaults: fill cfg with the values from the spec (§9).
 * Call before config_load so unset keys get the right fallback.
 */
void config_defaults(imud_config_t *cfg);

#endif /* IMUD_CONFIG_H */
