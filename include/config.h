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
 * output rates, stats heartbeat rate, log level, and static declination
 * (unless a live position source owns it); the log file is also reopened
 * so logrotate can rotate it.
 * Fields marked [restart] require a full daemon restart to take effect.
 */
#ifndef IMUD_CONFIG_H
#define IMUD_CONFIG_H

#include <stdbool.h>

typedef struct {

    /* [device] */
    char  i2c_bus[64];          /* e.g. "/dev/i2c-1" */
    char  gpio_chip[32];        /* gpiochip name: "gpiochip0" Pi 4, "gpiochip4" Pi 5 */
    char  sim_file[256];        /* .imucap for sim-driver playback; "" = synthesis */
    bool  sim_loop;             /* repeat the capture (seq/time rebased) */
    float sim_speed;            /* playback pacing; 1.0 real time, 0 = fastest */

    /* [capture]  [restart]: raw-sample black box (.imucap files) */
    bool  capture_enabled;      /* record every raw sample from both sensors */
    char  capture_dir[256];     /* where imud-YYYYMMDD-HHMMSS.imucap files go */
    int   capture_max_mb;       /* rotate file at this size; 0 = unlimited */
    int   capture_max_files;    /* keep at most N files (ring); 0 = unlimited */
    int   capture_flush_s;      /* flush interval, seconds */

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
    bool   use_measured_noise;    /* [restart] prefer cal.json "noise" (imud-cal
                                   * characterize) over the mekf_* keys below */
    float  heave_tau_s;           /* [hot] heave filter time constant, s; 0 = off */
    float  wave_tau_s;            /* [hot] sea-state averaging window, s; 0 = off;
                                   * needs heave_tau_s > 0 */
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

    /* [stream]  local AF_UNIX subscription stream (binary packets) */
    bool  stream_enabled;         /* [restart] */
    char  stream_socket[108];     /* [restart] listen path; sized to sun_path */
    int   stream_rate_hz;         /* [hot] per-subscriber packet rate */

    /* [imud-signalk]  Signal K bridge daemon — reads its own
     * /etc/imud/imud-signalk.conf (the [imud-signalk] section, plus the shared
     * `socket` and `publish_heave` keys below), connects to the stream socket,
     * and emits Signal K delta JSON over UDP. Consumed only by the imud-signalk
     * binary; unrelated to the pos_signalk_* input keys in [position]. */
    bool  sk_enabled;             /* [restart] */
    char  sk_dest_addr[64];       /* [hot] Signal K server host */
    int   sk_dest_port;           /* [hot] SK server UDP input port */
    int   sk_rate_hz;             /* [hot] delta emit rate */
    char  sk_source_label[32];    /* [hot] Signal K delta source.label */

    /* Bridge-shared keys — the imud-signalk / imud-mqtt daemons read these from
     * their own config file (so they never read imud.conf). The `socket` key
     * reuses stream_socket above; default is the well-known stream path. */
    bool  publish_heave;          /* [hot] emit heave (env.heave / imud/environment/heave) */

    /* [imud-mqtt]  MQTT bridge daemon — reads its own /etc/imud/imud-mqtt.conf
     * ([imud-mqtt] section, plus the shared `socket`/`publish_heave` keys above),
     * connects to the stream socket, and publishes scalar telemetry topics
     * (+ Home Assistant discovery) to an MQTT broker via libmosquitto.
     * Consumed only by the imud-mqtt binary. */
    bool  mqtt_enabled;           /* [restart] */
    char  mqtt_broker_addr[64];   /* [restart] broker host (hostname or IP) */
    int   mqtt_broker_port;       /* [restart] broker TCP port */
    char  mqtt_client_id[64];     /* [restart] MQTT client id; also HA node id */
    char  mqtt_topic_prefix[64];  /* [restart] topic prefix, e.g. "imud" */
    int   mqtt_rate_hz;           /* [hot] publish rate */
    int   mqtt_qos;               /* [hot] publish QoS (0|1|2) */
    bool  mqtt_retain;            /* [hot] retain published values */
    char  mqtt_units[8];          /* [hot] "deg" | "rad" */
    char  mqtt_username[64];      /* [restart] broker auth username ("" = none) */
    char  mqtt_password[128];     /* [restart] broker auth password ("" = none) */
    bool  mqtt_tls;               /* [restart] enable TLS */
    char  mqtt_tls_cafile[256];   /* [restart] CA cert path ("" = system store) */
    int   mqtt_keepalive_s;       /* [restart] MQTT keepalive interval, s */
    bool  mqtt_ha_discovery;      /* [restart] publish Home Assistant discovery */
    char  mqtt_ha_prefix[64];     /* [restart] HA discovery prefix (e.g. "homeassistant") */

    /* [imud-influxdb]  InfluxDB bridge daemon — reads its own
     * /etc/imud/imud-influxdb.conf ([imud-influxdb] section, plus the shared
     * `socket`/`publish_heave` keys), connects to the stream socket, and writes
     * InfluxDB line-protocol points over UDP (default) or HTTP. Pure C, no
     * external deps. Consumed only by the imud-influxdb binary. */
    bool  influx_enabled;          /* [restart] */
    char  influx_transport[8];     /* [restart] "udp" | "http" */
    int   influx_rate_hz;          /* [hot] point emit rate */
    char  influx_measurement[32];  /* [hot] line-protocol measurement name */
    char  influx_source_label[32]; /* [hot] value of the source= tag */
    char  influx_units[8];         /* [hot] "deg" | "rad" */
    /* UDP transport */
    char  influx_udp_addr[64];     /* [hot] InfluxDB/Telegraf UDP host */
    int   influx_udp_port;         /* [hot] UDP port (InfluxDB default 8089) */
    /* HTTP transport */
    char  influx_http_host[64];    /* [restart] HTTP host */
    int   influx_http_port;        /* [restart] HTTP port (InfluxDB default 8086) */
    char  influx_http_path[192];   /* [restart] write path incl. query (db / org+bucket / precision) */
    char  influx_http_token[128];  /* [restart] InfluxDB 2.x API token ("" = none) */

    /*
     * [imud-prometheus] — Prometheus exporter bridge (imud-prometheus daemon;
     * shared `socket` key above). Serves the latest fused state as Prometheus
     * text-format gauges on GET /metrics. Base SI units only (no deg/rad key).
     */
    bool  prom_enabled;            /* [restart] */
    char  prom_listen_addr[64];    /* [restart] HTTP bind address */
    int   prom_listen_port;        /* [restart] HTTP port (default 9815) */

    /* [imud-mavlink]  MAVLink bridge daemon — reads its own
     * /etc/imud/imud-mavlink.conf, connects to the stream socket, and emits
     * MAVLink (v1 or v2) HEARTBEAT/ATTITUDE/ATTITUDE_QUATERNION over UDP and/or
     * serial. Pure C, no external deps. Consumed only by the imud-mavlink binary. */
    bool  mav_enabled;             /* [restart] */
    int   mav_version;             /* [hot] MAVLink protocol version: 1 or 2 */
    int   mav_system_id;           /* [restart] MAVLink system id */
    int   mav_component_id;        /* [restart] MAVLink component id */
    int   mav_rate_hz;             /* [hot] data-message rate (heartbeat is 1 Hz) */
    bool  mav_send_attitude;       /* [hot] emit ATTITUDE (#30) */
    bool  mav_send_attitude_quaternion; /* [hot] emit ATTITUDE_QUATERNION (#31) */
    /* UDP transport */
    bool  mav_udp_enabled;         /* [restart] */
    char  mav_udp_addr[64];        /* [hot] destination host (e.g. a GCS) */
    int   mav_udp_port;            /* [hot] destination UDP port (QGC default 14550) */
    /* Serial transport */
    bool  mav_serial_enabled;      /* [restart] */
    char  mav_serial_device[64];   /* [restart] e.g. /dev/serial0 */
    int   mav_serial_baud;         /* [restart] e.g. 57600 */

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
    float  pos_mref_h_gauss;      /* derived: WMM horizontal field magnitude, Gauss */
    float  pos_mref_z_gauss;      /* derived: WMM vertical field (down +), Gauss */
    bool   pos_mref_valid;        /* derived: true when WMM field was computed */
    float  pos_speed_mps;         /* derived: live speed over ground from gpsd */
    bool   pos_speed_valid;       /* derived: cleared when the GPS fix expires */
    double pos_lat_deg;           /* geodetic latitude  (+N / -S); 0 = WMM disabled */
    double pos_lon_deg;           /* geodetic longitude (+E / -W); 0 = WMM disabled */
    char   pos_wmm_file[256];     /* path to WMM.COF; "" (default) auto-resolves:
                                   * /etc/imud (override) then /usr/share/imud (data) */

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
