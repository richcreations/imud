/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mqtt_main.c — imud-mqtt: MQTT bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket) via
 * libimud, and publishes scalar telemetry topics to an MQTT
 * broker (via libmosquitto) at the configured rate — one value per topic under
 * a prefix — plus Home Assistant MQTT-discovery configs so the sensors self-
 * register. See mqtt_publish.c for the topic/unit mapping.
 *
 * Like imud-signalk it holds no hardware, takes libimud's ABI-stable
 * imud_data_t view, reconnects to the stream if imud
 * restarts, and reads its own config file (/etc/imud/imud-mqtt.conf).
 * libmosquitto's background loop owns the broker connection (auto-reconnect,
 * keepalive), so a broker outage never stalls the stream reader.
 */

/* The Makefile also passes -D_GNU_SOURCE; guard so a standalone compile
 * still works without redefining it. */
#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
#endif

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include "mqtt_publish.h"        /* the builders; pull in ../lib/imud.h */
#include "../lib/imud.h"         /* libimud: stream connect/read/validate */
#include "bridge.h"              /* shared bridge scaffolding */
#include "sdnotify.h"
#include "config.h"
#include "log.h"

#include <mosquitto.h>

#define MAX_MSGS    24           /* upper bound on state/discovery message counts */

static const bridge_info_t BI = {
    .prog         = "imud-mqtt",
    .tag          = "mqtt",
    .section      = "imud-mqtt",
    .default_conf = "/etc/imud/imud-mqtt.conf",
    .usage_desc   =
        "  MQTT bridge: reads imud's stream socket and publishes scalar\n"
        "  telemetry topics (+ Home Assistant discovery) to an MQTT broker.\n"
        "  Configured by [imud-mqtt] in its own file.\n",
};

/* Discovery/availability context handed to libmosquitto callbacks. Its [restart]
 * fields are fixed at startup; the [hot] ones (deg/emit_heave/qos) are written
 * by the main thread on SIGHUP and read by the mosquitto network thread in
 * on_connect() — _Atomic so the cross-thread access is well-defined. */
typedef struct {
    char prefix[64];
    char ha_prefix[64];
    char node_id[64];
    char avail_topic[128];   /* "<prefix>/status/online" */
    bool ha_discovery;
    _Atomic bool deg;
    _Atomic bool emit_heave;
    _Atomic int  qos;
} mqtt_ctx_t;

/* ── libmosquitto callbacks ──────────────────────────────────────────────── */

static void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
    mqtt_ctx_t *c = obj;
    if (rc != 0) {
        LOG_W("[mqtt] broker refused connection (rc=%d)\n", rc);
        return;
    }
    LOG_I("[mqtt] connected to broker\n");

    /* Availability: retained "online" (the LWT sets "offline" on a crash). */
    mosquitto_publish(mosq, NULL, c->avail_topic, 6, "online", c->qos, true);

    if (c->ha_discovery) {
        mqtt_msg_t d[MAX_MSGS];
        int n = mqtt_build_discovery(d, MAX_MSGS, c->prefix, c->ha_prefix,
                                     c->node_id, c->emit_heave, c->deg);
        for (int i = 0; i < n; i++)
            mosquitto_publish(mosq, NULL, d[i].topic, (int)strlen(d[i].payload),
                              d[i].payload, c->qos, true);   /* retained */
        LOG_I("[mqtt] published %d Home Assistant discovery configs\n", n);
    }
}

static void on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    (void)mosq; (void)obj;
    if (rc != 0) LOG_W("[mqtt] broker connection lost (rc=%d) — reconnecting\n", rc);
}

int main(int argc, char **argv)
{
    char config_path[256];
    int rc = bridge_parse_cli(argc, argv, &BI, config_path, sizeof config_path);
    if (rc != 0) return rc < 0 ? 1 : 0;

    imud_config_t cfg;
    if (bridge_load_config(&BI, config_path, &cfg) < 0) return 1;

    if (!cfg.mqtt_enabled) {
        bridge_exit_disabled(&BI);
        return 0;
    }

    bridge_install_signals();

    bool deg        = (strcmp(cfg.mqtt_units, "rad") != 0);
    bool emit_heave = cfg.publish_heave;
    long period_ns  = bridge_period_ns(cfg.mqtt_rate_hz, 200000000L);

    mqtt_ctx_t ctx;
    memset(&ctx, 0, sizeof ctx);
    snprintf(ctx.prefix,    sizeof ctx.prefix,    "%s", cfg.mqtt_topic_prefix);
    snprintf(ctx.ha_prefix, sizeof ctx.ha_prefix, "%s", cfg.mqtt_ha_prefix);
    snprintf(ctx.node_id,   sizeof ctx.node_id,   "%s", cfg.mqtt_client_id);
    snprintf(ctx.avail_topic, sizeof ctx.avail_topic, "%s/status/online",
             cfg.mqtt_topic_prefix);
    ctx.ha_discovery = cfg.mqtt_ha_discovery;
    ctx.deg          = deg;
    ctx.emit_heave   = emit_heave;
    ctx.qos          = cfg.mqtt_qos;

    /* ── libmosquitto setup (only when the broker output is enabled) ─────── */
    struct mosquitto *mosq = NULL;
    if (cfg.mqtt_broker_enabled) {
        mosquitto_lib_init();
        mosq = mosquitto_new(cfg.mqtt_client_id, true, &ctx);
        if (!mosq) {
            LOG_E("[mqtt] mosquitto_new failed: %s\n", strerror(errno));
            mosquitto_lib_cleanup();
            return 1;
        }
        if (cfg.mqtt_username[0])
            mosquitto_username_pw_set(mosq, cfg.mqtt_username,
                                      cfg.mqtt_password[0] ? cfg.mqtt_password : NULL);
        if (cfg.mqtt_tls) {
            const char *ca     = cfg.mqtt_tls_cafile[0] ? cfg.mqtt_tls_cafile : NULL;
            const char *capath = ca ? NULL : "/etc/ssl/certs";   /* system store */
            if (mosquitto_tls_set(mosq, ca, capath, NULL, NULL, NULL) != MOSQ_ERR_SUCCESS)
                LOG_W("[mqtt] tls_set failed — continuing without TLS verification\n");
        }
        /* Last will: if the bridge dies, the broker marks us offline (retained). */
        mosquitto_will_set(mosq, ctx.avail_topic, 7, "offline", cfg.mqtt_qos, true);
        mosquitto_connect_callback_set(mosq, on_connect);
        mosquitto_disconnect_callback_set(mosq, on_disconnect);
        mosquitto_reconnect_delay_set(mosq, 2, 30, true);

        rc = mosquitto_connect_async(mosq, cfg.mqtt_broker_addr, cfg.mqtt_broker_port,
                                     cfg.mqtt_keepalive_s);
        if (rc != MOSQ_ERR_SUCCESS)
            LOG_W("[mqtt] initial connect to %s:%d deferred (%s) — will retry\n",
                  cfg.mqtt_broker_addr, cfg.mqtt_broker_port, mosquitto_strerror(rc));
        mosquitto_loop_start(mosq);   /* background network thread */

        LOG_I("[mqtt] bridge → %s:%d @ %d Hz (client '%s', prefix '%s', %s), reading %s\n",
              cfg.mqtt_broker_addr, cfg.mqtt_broker_port, cfg.mqtt_rate_hz,
              cfg.mqtt_client_id, cfg.mqtt_topic_prefix, deg ? "deg" : "rad",
              cfg.stream_socket);
    } else {
        LOG_W("[mqtt] no output enabled (broker_enabled = false) — nothing will be published\n");
    }
    sd_notify_msg("READY=1");

    imud_t      *stream = NULL;
    imud_data_t  latest;
    bool have_pkt = false;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!bridge_stop) {
        sd_notify_msg("WATCHDOG=1");

        imud_config_t nc;
        if (bridge_reload_begin(&BI, config_path, &nc)) {
            deg        = (strcmp(nc.mqtt_units, "rad") != 0);
            emit_heave = nc.publish_heave;
            period_ns  = bridge_period_ns(nc.mqtt_rate_hz, 200000000L);
            ctx.deg = deg; ctx.emit_heave = emit_heave; ctx.qos = nc.mqtt_qos;
            if (nc.mqtt_broker_enabled != cfg.mqtt_broker_enabled)
                LOG_W("[mqtt] broker_enabled change needs a restart to apply\n");
            if (strcmp(nc.mqtt_broker_addr, cfg.mqtt_broker_addr) != 0 ||
                nc.mqtt_broker_port != cfg.mqtt_broker_port ||
                strcmp(nc.mqtt_client_id, cfg.mqtt_client_id) != 0 ||
                strcmp(nc.mqtt_topic_prefix, cfg.mqtt_topic_prefix) != 0)
                LOG_W("[mqtt] broker/client/prefix change needs a restart to apply\n");
            cfg = nc;
            bridge_reload_done(&BI);
        }

        int cs = bridge_stream_ensure(&stream, cfg.stream_socket, BI.tag, 2);
        if (cs == 0) { have_pkt = false; continue; }
        if (cs == 2) clock_gettime(CLOCK_MONOTONIC, &next);

        /* Wait until the next publish tick, draining frames so the packet we
         * publish is always the most recent. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int r = imud_read(stream, bridge_wait_ms(&now, &next));
        if (r < 0) {
            bridge_stream_drop(&stream, BI.tag);
            have_pkt = false;
            continue;
        }
        if (r == 0) {
            latest   = *imud_data(stream);
            have_pkt = true;
        }
        /* r == 1: tick deadline (or a signal) — fall through to the publisher. */

        clock_gettime(CLOCK_MONOTONIC, &now);
        if (bridge_due(&now, &next)) {
            if (have_pkt && mosq) {
                mqtt_msg_t st[MAX_MSGS];
                int n = mqtt_build_state(st, MAX_MSGS, &latest,
                                         cfg.mqtt_topic_prefix, emit_heave, deg);
                for (int i = 0; i < n; i++)
                    mosquitto_publish(mosq, NULL, st[i].topic,
                                      (int)strlen(st[i].payload), st[i].payload,
                                      cfg.mqtt_qos, cfg.mqtt_retain);
            }
            bridge_advance(&next, &now, period_ns);
        }
    }

    LOG_I("[mqtt] shutting down\n");
    if (mosq) {
        /* Clean disconnect suppresses the LWT, so announce offline explicitly. */
        mosquitto_publish(mosq, NULL, ctx.avail_topic, 7, "offline", cfg.mqtt_qos, true);
        mosquitto_disconnect(mosq);
        mosquitto_loop_stop(mosq, false);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
    }
    imud_free(stream);
    return 0;
}
