/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mqtt_main.c — imud-mqtt: MQTT bridge daemon
 *
 * Connects to imud's AF_UNIX binary subscription stream ([stream] socket),
 * reads the 196-byte packets, and publishes scalar telemetry topics to an MQTT
 * broker (via libmosquitto) at the configured rate — one value per topic under
 * a prefix — plus Home Assistant MQTT-discovery configs so the sensors self-
 * register. See mqtt_publish.c for the topic/unit mapping.
 *
 * Like imud-signalk it holds no hardware, reuses the public client header
 * (lib/imud_client.h) for packet validation, reconnects to the stream if imud
 * restarts, and reads its own config file (/etc/imud/imud-mqtt.conf).
 * libmosquitto's background loop owns the broker connection (auto-reconnect,
 * keepalive), so a broker outage never stalls the stream reader.
 */

#ifdef __linux__
# define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>

#define IMUD_CLIENT_IMPLEMENTATION
#include "mqtt_publish.h"        /* pulls in ../lib/imud_client.h (with impl) */
#include "config.h"
#include "log.h"

#include <mosquitto.h>

#ifndef SOCK_CLOEXEC
# define SOCK_CLOEXEC 0
#endif

#define VERSION_STR "1.1"
#define MAX_MSGS    16           /* upper bound on state/discovery message counts */

static volatile sig_atomic_t g_stop;
static volatile sig_atomic_t g_reload;

static void on_signal(int sig)
{
    if (sig == SIGHUP) g_reload = 1;
    else               g_stop   = 1;
}

/* Discovery/availability context handed to libmosquitto callbacks. Its [restart]
 * fields are fixed at startup; the [hot] ones (deg/emit_heave/qos/retain) are
 * word-sized and only refreshed on SIGHUP — a benign race with the net thread. */
typedef struct {
    char prefix[64];
    char ha_prefix[64];
    char node_id[64];
    char avail_topic[128];   /* "<prefix>/status/online" */
    bool ha_discovery;
    bool deg;
    bool emit_heave;
    int  qos;
} mqtt_ctx_t;

/* ── sd_notify (mirrors src/signalk_main.c) ──────────────────────────────── */

static void sd_notify_msg(const char *msg)
{
    const char *sock = getenv("NOTIFY_SOCKET");
    if (!sock) return;
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    if (sock[0] == '@') {
        addr.sun_path[0] = '\0';
        strncpy(addr.sun_path + 1, sock + 1, sizeof(addr.sun_path) - 2);
    } else {
        strncpy(addr.sun_path, sock, sizeof(addr.sun_path) - 1);
    }
    int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return;
    sendto(fd, msg, strlen(msg), MSG_NOSIGNAL, (struct sockaddr *)&addr, sizeof addr);
    close(fd);
}

/* ── AF_UNIX stream connect + framed read (mirrors src/signalk_main.c) ────── */

static int connect_stream(const char *path)
{
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        close(fd);
        return -1;
    }
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return fd;
}

/* Read exactly one IMUD_PACKET_SIZE frame; 0 on success, -1 on EOF/error. */
static int read_frame(int fd, unsigned char *frame)
{
    size_t got = 0;
    while (got < IMUD_PACKET_SIZE) {
        ssize_t n = read(fd, frame + got, IMUD_PACKET_SIZE - got);
        if (n <= 0) return -1;
        got += (size_t)n;
    }
    return 0;
}

static void interruptible_sleep(int secs)
{
    for (int i = 0; i < secs && !g_stop && !g_reload; i++) sleep(1);
}

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

/* ── CLI + config ────────────────────────────────────────────────────────── */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--config PATH]\n"
        "\n"
        "  MQTT bridge: reads imud's stream socket and publishes scalar\n"
        "  telemetry topics (+ Home Assistant discovery) to an MQTT broker.\n"
        "  Configured by [imud-mqtt] in its own file.\n"
        "\n"
        "  --config PATH   Config file (default: /etc/imud/imud-mqtt.conf)\n"
        "  --version       Print version and exit\n",
        prog);
}

int main(int argc, char **argv)
{
    char config_path[256];
    snprintf(config_path, sizeof config_path, "/etc/imud/imud-mqtt.conf");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, sizeof config_path, "%s", argv[++i]);
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("imud-mqtt %s\n", VERSION_STR);
            return 0;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            LOG_E("unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    imud_config_t cfg;
    config_defaults(&cfg);
    int rc = config_load(config_path, &cfg);
    if (rc == CONFIG_ERR_PARSE) {
        LOG_E("[mqtt] %s has errors (see above) — refusing to start\n", config_path);
        return 1;
    }

    if (getenv("INVOCATION_ID")) log_set_style(LOG_STYLE_JOURNAL);
    log_set_level_str(cfg.log_level);

    if (!cfg.mqtt_enabled) {
        LOG_I("[mqtt] disabled in config ([imud-mqtt] enabled = false) — exiting\n");
        return 0;
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGHUP,  &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    bool deg        = (strcmp(cfg.mqtt_units, "rad") != 0);
    bool emit_heave = cfg.publish_heave;
    long period_ns  = (cfg.mqtt_rate_hz > 0) ? 1000000000L / cfg.mqtt_rate_hz
                                             : 200000000L;

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

    /* ── libmosquitto setup ─────────────────────────────────────────────── */
    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(cfg.mqtt_client_id, true, &ctx);
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
    sd_notify_msg("READY=1");

    int  stream_fd = -1;
    unsigned char frame[IMUD_PACKET_SIZE];
    imud_packet_t latest;
    bool have_pkt = false;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!g_stop) {
        sd_notify_msg("WATCHDOG=1");

        if (g_reload) {
            g_reload = 0;
            imud_config_t nc;
            config_defaults(&nc);
            if (config_load(config_path, &nc) != CONFIG_ERR_PARSE) {
                log_set_level_str(nc.log_level);
                deg        = (strcmp(nc.mqtt_units, "rad") != 0);
                emit_heave = nc.publish_heave;
                period_ns  = (nc.mqtt_rate_hz > 0) ? 1000000000L / nc.mqtt_rate_hz
                                                   : 200000000L;
                ctx.deg = deg; ctx.emit_heave = emit_heave; ctx.qos = nc.mqtt_qos;
                if (strcmp(nc.mqtt_broker_addr, cfg.mqtt_broker_addr) != 0 ||
                    nc.mqtt_broker_port != cfg.mqtt_broker_port ||
                    strcmp(nc.mqtt_client_id, cfg.mqtt_client_id) != 0 ||
                    strcmp(nc.mqtt_topic_prefix, cfg.mqtt_topic_prefix) != 0)
                    LOG_W("[mqtt] broker/client/prefix change needs a restart to apply\n");
                cfg = nc;
                LOG_I("[mqtt] config reloaded\n");
            } else {
                LOG_W("[mqtt] reload failed — keeping current config\n");
            }
        }

        if (stream_fd < 0) {
            stream_fd = connect_stream(cfg.stream_socket);
            if (stream_fd < 0) {
                LOG_W("[mqtt] stream socket %s unavailable: %s — retrying\n",
                      cfg.stream_socket, strerror(errno));
                have_pkt = false;
                interruptible_sleep(2);
                continue;
            }
            LOG_I("[mqtt] connected to %s\n", cfg.stream_socket);
            clock_gettime(CLOCK_MONOTONIC, &next);
        }

        /* Wait until the next publish tick, draining frames so the packet we
         * publish is always the most recent. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long wait_ns = (next.tv_sec - now.tv_sec) * 1000000000L
                     + (next.tv_nsec - now.tv_nsec);
        if (wait_ns < 0) wait_ns = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(stream_fd, &rfds);
        struct timeval tv = { .tv_sec  = wait_ns / 1000000000L,
                              .tv_usec = (wait_ns % 1000000000L) / 1000 };
        int s = select(stream_fd + 1, &rfds, NULL, NULL, &tv);
        if (s < 0) {
            if (errno == EINTR) continue;
            LOG_W("[mqtt] select: %s\n", strerror(errno));
            close(stream_fd); stream_fd = -1; have_pkt = false;
            continue;
        }
        if (s > 0 && FD_ISSET(stream_fd, &rfds)) {
            if (read_frame(stream_fd, frame) != 0) {
                LOG_I("[mqtt] stream disconnected\n");
                close(stream_fd); stream_fd = -1; have_pkt = false;
                continue;
            }
            if (imud_packet_valid(frame, sizeof frame)) {
                memcpy(&latest, frame, sizeof latest);
                have_pkt = true;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        if ((now.tv_sec > next.tv_sec) ||
            (now.tv_sec == next.tv_sec && now.tv_nsec >= next.tv_nsec)) {
            if (have_pkt) {
                mqtt_msg_t st[MAX_MSGS];
                int n = mqtt_build_state(st, MAX_MSGS, &latest,
                                         cfg.mqtt_topic_prefix, emit_heave, deg);
                for (int i = 0; i < n; i++)
                    mosquitto_publish(mosq, NULL, st[i].topic,
                                      (int)strlen(st[i].payload), st[i].payload,
                                      cfg.mqtt_qos, cfg.mqtt_retain);
            }
            do {
                next.tv_nsec += period_ns;
                while (next.tv_nsec >= 1000000000L) {
                    next.tv_sec++; next.tv_nsec -= 1000000000L;
                }
            } while ((next.tv_sec < now.tv_sec) ||
                     (next.tv_sec == now.tv_sec && next.tv_nsec <= now.tv_nsec));
        }
    }

    LOG_I("[mqtt] shutting down\n");
    /* Clean disconnect suppresses the LWT, so announce offline explicitly. */
    mosquitto_publish(mosq, NULL, ctx.avail_topic, 7, "offline", cfg.mqtt_qos, true);
    mosquitto_disconnect(mosq);
    mosquitto_loop_stop(mosq, false);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    if (stream_fd >= 0) close(stream_fd);
    return 0;
}
