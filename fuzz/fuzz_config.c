/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fuzz_config.c — libFuzzer harness for the TOML config parser
 * (src/config.c).  config_load takes a path, so each input lands in a
 * per-process temp file first.  Run with -close_fd_mask=3 to silence the
 * parser's per-line error logging.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "config.h"

/*
 * The oracle: a config the parser ACCEPTED must not contain an out-of-range
 * value.  Without it this harness could only ever detect a sanitizer trap, and
 * the bug it is aimed at traps nothing — "dest_port = 4294977414" wrapped to
 * 10118 and config_load returned success, so the daemon listened on a port
 * nobody asked for. That is invisible to ASan and to a crash-only fuzzer.
 *
 * Only checked on rc == 0. A rejected config is allowed to hold anything: the
 * parser applies valid lines as it goes and reports at the end, so a partially
 * applied struct is expected and is exactly what the daemon refuses to run on.
 *
 * WHAT THIS DOES AND DOES NOT GUARD — verified by mutation, not assumed:
 *
 *   It guards the semantic bounds (NEED_PORT / NEED_GPIO / NEED_I2C_ADDR).
 *   Reverting one key to plain NEED_INT makes it fire on "dest_port = 70000".
 *
 *   It CANNOT guard parse_int's own range check, and no oracle over the
 *   resulting struct could. A value that wraps to something inside the bound
 *   ("dest_port = 4294977414" -> 10118) is by then indistinguishable from a
 *   port the operator typed; a value that wraps to something outside it is
 *   rejected by the bound, so rc != 0 and this never runs. Removing the ERANGE
 *   and INT_MIN/INT_MAX tests leaves this harness silent.
 *
 * That half lives in test_config's test_load_rejects_out_of_range_int, which
 * asserts the rejection directly. Do not read a green fuzz run as covering it.
 */
static void check_ranges(const imud_config_t *c)
{
    const int ports[] = {
        c->nmea_dest_port, c->nmea_tcp_port, c->highrate_dest_port,
        c->stream_tcp_port, c->sk_dest_port, c->sk_tcp_port,
        c->mqtt_broker_port, c->influx_udp_port, c->influx_http_port,
        c->prom_listen_port, c->mav_udp_port, c->mav_tcp_port,
        c->pos_gpsd_port, c->pos_signalk_port,
    };
    for (unsigned i = 0; i < sizeof ports / sizeof ports[0]; i++)
        assert(ports[i] >= 1 && ports[i] <= 65535);

    assert(c->imu_int_gpio >= 0 && c->imu_int_gpio <= 255);
    assert(c->mag_int_gpio >= 0 && c->mag_int_gpio <= 255);
    assert(c->imu_addr >= 0x00 && c->imu_addr <= 0x7F);
    assert(c->mag_addr >= 0x00 && c->mag_addr <= 0x7F);

    /* The rates NEED_POS_MHZ guards. A wrapped value can land positive
     * here and satisfy it, which is how odr_hz = 4294968129 became 833.
     * Carried in milli-Hz since the rate rework, so odr_hz = 12.5 is a real
     * configuration rather than something that has to round. */
    assert(c->imu_odr_mhz > 0);
    assert(c->mag_odr_mhz > 0);
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static char path[64];
    if (!path[0])
        snprintf(path, sizeof(path), "/tmp/imud_fuzz_cfg_%d", (int)getpid());

    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    if (size) fwrite(data, 1, size, f);
    fclose(f);

    imud_config_t cfg;
    config_defaults(&cfg);
    if (config_load(path, &cfg) == 0)
        check_ranges(&cfg);
    return 0;
}
