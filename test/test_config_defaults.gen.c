/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_config_defaults.gen.c — GENERATED, do not edit.
 *
 * Written by tools/gen-config-docs.py --write from
 * docs/config-keys.toml; `make check-generated-text` fails if this
 * file and the registry disagree.  Edit the registry, not this.
 *
 * One assertion per documented default: the value on the right is the
 * one docs/manual.md prints for that key, read back out of the table.
 * So a failure here is never ambiguous — either config_defaults() no
 * longer produces what is published, or the published value is wrong.
 *
 * #included inside a function in test/test_config.c, which defines the
 * CK_* macros and holds the config_defaults() call.  __FILE__ resolves
 * to THIS file, so a failure reports the generated line, and the label
 * on it names the [section] and key.
 */

/* [device] */
CK_STR (c.i2c_bus,   "/dev/i2c-1", "[device] i2c_bus");
CK_STR (c.gpio_chip, "gpiochip0",  "[device] gpio_chip");
CK_STR (c.sim_file,  "",           "[device] sim_file");
CK_BOOL(c.sim_loop,  false,        "[device] sim_loop");
CK_FLT (c.sim_speed, 1.0,          "[device] sim_speed");

/* [capture] */
CK_BOOL(c.capture_enabled,   false,           "[capture] enabled");
CK_STR (c.capture_dir,       "/var/lib/imud", "[capture] dir");
CK_INT (c.capture_max_mb,    256,             "[capture] max_mb");
CK_INT (c.capture_max_files, 8,               "[capture] max_files");
CK_INT (c.capture_flush_s,   5,               "[capture] flush_s");

/* [imu] */
CK_STR (c.imu_driver,       "ism330dhcx", "[imu] driver");
CK_ENUM(c.imu_bus_kind,     BUS_I2C,      "[imu] bus");
CK_STR (c.imu_spi_dev,      "",           "[imu] spi_dev");
CK_INT (c.imu_spi_speed_hz, 0,            "[imu] spi_speed_hz");
CK_INT (c.imu_addr,         0x6B,         "[imu] i2c_addr");
CK_INT (c.imu_int_gpio,     17,           "[imu] int_gpio");
CK_INT (c.imu_odr_hz,       833,          "[imu] odr_hz");
CK_INT (c.imu_accel_g,      8,            "[imu] accel_g");
CK_INT (c.imu_gyro_dps,     2000,         "[imu] gyro_dps");
CK_INT (c.imu_fifo_wm,      64,           "[imu] fifo_wm");

/* [mag] */
CK_STR (c.mag_driver,       "mmc5983ma", "[mag] driver");
CK_ENUM(c.mag_bus_kind,     BUS_I2C,     "[mag] bus");
CK_STR (c.mag_spi_dev,      "",          "[mag] spi_dev");
CK_INT (c.mag_spi_speed_hz, 0,           "[mag] spi_speed_hz");
CK_INT (c.mag_addr,         0x30,        "[mag] i2c_addr");
CK_INT (c.mag_int_gpio,     27,          "[mag] int_gpio");
CK_INT (c.mag_odr_hz,       100,         "[mag] odr_hz");
CK_FLT (c.mag_set_period_s, 5.0,         "[mag] set_period_s");

/* [fusion] */
CK_DBL (c.mekf_gyro_noise,          0.007,   "[fusion] mekf_gyro_noise");
CK_DBL (c.mekf_gyro_bias,           0.00015, "[fusion] mekf_gyro_bias");
CK_DBL (c.mekf_accel_noise,         0.0022,  "[fusion] mekf_accel_noise");
CK_DBL (c.mekf_mag_noise,           0.0004,  "[fusion] mekf_mag_noise");
CK_DBL (c.mekf_wave_accel,          0.8,     "[fusion] mekf_wave_accel");
CK_DBL (c.mekf_wave_accel_tau_s,    0.5,     "[fusion] mekf_wave_accel_tau_s");
CK_DBL (c.mekf_mag_dip_sigma_deg,   1.0,     "[fusion] mekf_mag_dip_sigma_deg");
CK_DBL (c.mag_reject_gauss,         0.05,    "[fusion] mag_reject_gauss");
CK_DBL (c.accel_skip_thresh,        0.05,    "[fusion] accel_skip_thresh");
CK_BOOL(c.mag_yaw_only,             true,    "[fusion] mag_yaw_only");
CK_FLT (c.heave_tau_s,              12.0,    "[fusion] heave_tau_s");
CK_FLT (c.wave_tau_s,               120.0,   "[fusion] wave_tau_s");
CK_DBL (c.engine_vibration_g2,      0.0,     "[fusion] engine_vibration_g2");
CK_DBL (c.engine_accel_skip_thresh, 0.20,    "[fusion] engine_accel_skip_thresh");

/* [calibration] */
CK_STR(c.cal_file,           "/etc/imud/cal.json", "[calibration] file");
CK_DBL(c.startup_settle_sec, 5.0,                  "[calibration] startup_settle_sec");
CK_DBL(c.gyro_bias_sec,      2.0,                  "[calibration] gyro_bias_sec");
CK_DBL(c.align_window_sec,   5.0,                  "[calibration] align_window_sec");

/* [nmea] */
CK_BOOL(c.nmea_enabled,       false,             "[nmea] enabled");
CK_INT (c.nmea_rate_hz,       10,                "[nmea] rate_hz");
CK_STR (c.nmea_dest_addr,     "255.255.255.255", "[nmea] dest_addr");
CK_INT (c.nmea_dest_port,     10110,             "[nmea] dest_port");
CK_BOOL(c.nmea_tcp_enabled,   false,             "[nmea] tcp_enabled");
CK_STR (c.nmea_tcp_bind_addr, "0.0.0.0",         "[nmea] tcp_bind_addr");
CK_INT (c.nmea_tcp_port,      10110,             "[nmea] tcp_port");

/* [highrate] */
CK_BOOL(c.highrate_enabled,     false,         "[highrate] enabled");
CK_INT (c.highrate_rate_hz,     500,           "[highrate] rate_hz");
CK_STR (c.highrate_dest_addr,   "239.255.0.1", "[highrate] dest_addr");
CK_INT (c.highrate_dest_port,   10111,         "[highrate] dest_port");
CK_STR (c.highrate_coord_frame, "NED",         "[highrate] coord_frame");

/* [stream] */
CK_BOOL(c.stream_enabled,       true,                         "[stream] enabled");
CK_STR (c.stream_socket,        "/run/imud/imud-stream.sock", "[stream] socket");
CK_INT (c.stream_rate_hz,       100,                          "[stream] rate_hz");
CK_BOOL(c.stream_tcp_enabled,   false,                        "[stream] tcp_enabled");
CK_STR (c.stream_tcp_bind_addr, "0.0.0.0",                    "[stream] tcp_bind_addr");
CK_INT (c.stream_tcp_port,      10112,                        "[stream] tcp_port");

/* [logging] */
CK_STR(c.log_level,    "warn", "[logging] level");
CK_STR(c.log_file,     "",     "[logging] file");
CK_INT(c.log_stats_hz, 1,      "[logging] stats_hz");

/* [position] */
CK_FLT (c.pos_declination_deg, 0.0,                                                "[position] declination_deg");
CK_DBL (c.pos_lat_deg,         0.0,                                                "[position] lat_deg");
CK_DBL (c.pos_lon_deg,         0.0,                                                "[position] lon_deg");
CK_STR (c.pos_wmm_file,        "",                                                 "[position] wmm_file");
CK_BOOL(c.pos_gpsd_enabled,    false,                                              "[position] gpsd_enabled");
CK_STR (c.pos_gpsd_host,       "localhost",                                        "[position] gpsd_host");
CK_INT (c.pos_gpsd_port,       2947,                                               "[position] gpsd_port");
CK_BOOL(c.pos_signalk_enabled, false,                                              "[position] signalk_enabled");
CK_STR (c.pos_signalk_host,    "localhost",                                        "[position] signalk_host");
CK_INT (c.pos_signalk_port,    3000,                                               "[position] signalk_port");
CK_STR (c.pos_signalk_path,    "/signalk/v1/api/vessels/self/navigation/position", "[position] signalk_path");
CK_FLT (c.pos_fix_max_age_h,   24.0,                                               "[position] fix_max_age_h");

/* [imud-signalk] */
CK_BOOL(c.sk_enabled,       false,                        "[imud-signalk] enabled");
CK_STR (c.stream_socket,    "/run/imud/imud-stream.sock", "[imud-signalk] socket");
CK_BOOL(c.sk_udp_enabled,   false,                        "[imud-signalk] udp_enabled");
CK_STR (c.sk_dest_addr,     "127.0.0.1",                  "[imud-signalk] dest_addr");
CK_INT (c.sk_dest_port,     10113,                        "[imud-signalk] dest_port");
CK_INT (c.sk_rate_hz,       10,                           "[imud-signalk] rate_hz");
CK_STR (c.sk_source_label,  "imud",                       "[imud-signalk] source_label");
CK_BOOL(c.publish_heave,    true,                         "[imud-signalk] publish_heave");
CK_BOOL(c.sk_tcp_enabled,   false,                        "[imud-signalk] tcp_enabled");
CK_STR (c.sk_tcp_bind_addr, "0.0.0.0",                    "[imud-signalk] tcp_bind_addr");
CK_INT (c.sk_tcp_port,      10113,                        "[imud-signalk] tcp_port");

/* [imud-mqtt] */
CK_BOOL(c.mqtt_enabled,        false,                        "[imud-mqtt] enabled");
CK_BOOL(c.mqtt_broker_enabled, false,                        "[imud-mqtt] broker_enabled");
CK_STR (c.stream_socket,       "/run/imud/imud-stream.sock", "[imud-mqtt] socket");
CK_STR (c.mqtt_broker_addr,    "127.0.0.1",                  "[imud-mqtt] broker_addr");
CK_INT (c.mqtt_broker_port,    1883,                         "[imud-mqtt] broker_port");
CK_STR (c.mqtt_client_id,      "imud",                       "[imud-mqtt] client_id");
CK_INT (c.mqtt_keepalive_s,    30,                           "[imud-mqtt] keepalive_s");
CK_STR (c.mqtt_topic_prefix,   "imud",                       "[imud-mqtt] topic_prefix");
CK_INT (c.mqtt_rate_hz,        5,                            "[imud-mqtt] rate_hz");
CK_INT (c.mqtt_qos,            0,                            "[imud-mqtt] qos");
CK_BOOL(c.mqtt_retain,         true,                         "[imud-mqtt] retain");
CK_STR (c.mqtt_units,          "deg",                        "[imud-mqtt] units");
CK_BOOL(c.publish_heave,       true,                         "[imud-mqtt] publish_heave");
CK_BOOL(c.mqtt_ha_discovery,   true,                         "[imud-mqtt] ha_discovery");
CK_STR (c.mqtt_ha_prefix,      "homeassistant",              "[imud-mqtt] ha_prefix");
CK_STR (c.mqtt_username,       "",                           "[imud-mqtt] username");
CK_STR (c.mqtt_password,       "",                           "[imud-mqtt] password");
CK_BOOL(c.mqtt_tls,            false,                        "[imud-mqtt] tls");
CK_STR (c.mqtt_tls_cafile,     "",                           "[imud-mqtt] tls_cafile");

/* [imud-influxdb] */
CK_BOOL(c.influx_enabled,      false,                         "[imud-influxdb] enabled");
CK_STR (c.stream_socket,       "/run/imud/imud-stream.sock",  "[imud-influxdb] socket");
CK_STR (c.influx_transport,    "",                            "[imud-influxdb] transport");
CK_INT (c.influx_rate_hz,      10,                            "[imud-influxdb] rate_hz");
CK_STR (c.influx_measurement,  "imud",                        "[imud-influxdb] measurement");
CK_STR (c.influx_source_label, "imud",                        "[imud-influxdb] source_label");
CK_STR (c.influx_units,        "deg",                         "[imud-influxdb] units");
CK_BOOL(c.publish_heave,       true,                          "[imud-influxdb] publish_heave");
CK_BOOL(c.influx_udp_enabled,  false,                         "[imud-influxdb] udp_enabled");
CK_STR (c.influx_udp_addr,     "127.0.0.1",                   "[imud-influxdb] udp_addr");
CK_INT (c.influx_udp_port,     8089,                          "[imud-influxdb] udp_port");
CK_BOOL(c.influx_http_enabled, false,                         "[imud-influxdb] http_enabled");
CK_STR (c.influx_http_host,    "127.0.0.1",                   "[imud-influxdb] http_host");
CK_INT (c.influx_http_port,    8086,                          "[imud-influxdb] http_port");
CK_STR (c.influx_http_path,    "/write?db=imud&precision=ns", "[imud-influxdb] http_path");
CK_STR (c.influx_http_token,   "",                            "[imud-influxdb] http_token");

/* [imud-prometheus] */
CK_BOOL(c.prom_enabled,      false,                        "[imud-prometheus] enabled");
CK_BOOL(c.prom_http_enabled, false,                        "[imud-prometheus] http_enabled");
CK_STR (c.stream_socket,     "/run/imud/imud-stream.sock", "[imud-prometheus] socket");
CK_STR (c.prom_listen_addr,  "127.0.0.1",                  "[imud-prometheus] listen_addr");
CK_INT (c.prom_listen_port,  9815,                         "[imud-prometheus] listen_port");

/* [imud-mavlink] */
CK_BOOL(c.mav_enabled,                  false,                        "[imud-mavlink] enabled");
CK_STR (c.stream_socket,                "/run/imud/imud-stream.sock", "[imud-mavlink] socket");
CK_INT (c.mav_version,                  2,                            "[imud-mavlink] version");
CK_INT (c.mav_system_id,                1,                            "[imud-mavlink] system_id");
CK_INT (c.mav_component_id,             1,                            "[imud-mavlink] component_id");
CK_INT (c.mav_rate_hz,                  10,                           "[imud-mavlink] rate_hz");
CK_BOOL(c.mav_send_attitude,            true,                         "[imud-mavlink] send_attitude");
CK_BOOL(c.mav_send_attitude_quaternion, true,                         "[imud-mavlink] send_attitude_quaternion");
CK_BOOL(c.mav_udp_enabled,              false,                        "[imud-mavlink] udp_enabled");
CK_STR (c.mav_udp_addr,                 "127.0.0.1",                  "[imud-mavlink] udp_addr");
CK_INT (c.mav_udp_port,                 14550,                        "[imud-mavlink] udp_port");
CK_BOOL(c.mav_serial_enabled,           false,                        "[imud-mavlink] serial_enabled");
CK_STR (c.mav_serial_device,            "/dev/serial0",               "[imud-mavlink] serial_device");
CK_INT (c.mav_serial_baud,              57600,                        "[imud-mavlink] serial_baud");
CK_BOOL(c.mav_tcp_enabled,              false,                        "[imud-mavlink] tcp_enabled");
CK_STR (c.mav_tcp_bind_addr,            "0.0.0.0",                    "[imud-mavlink] tcp_bind_addr");
CK_INT (c.mav_tcp_port,                 5760,                         "[imud-mavlink] tcp_port");

/* Not assertable, and each one deliberately so:
 *   [mount] rotation_euler_deg
 *   [mount] rotation_matrix
 *   [mount] preset
 * they are hand-rolled blocks in apply_kv() that set several
 * members at once, or (preset) match a name and store nothing,
 * so there is no single field carrying the documented default.
 * test_defaults_mount() in test_config.c asserts all three. */
