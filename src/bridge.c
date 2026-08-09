/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * bridge.c — shared scaffolding for the bridge daemons (see bridge.h)
 *
 * Every function body here is the verbatim block the five bridges used to
 * carry individually, with the bridge-specific strings routed through
 * bridge_info_t / tag parameters. Behavior and output must stay
 * byte-identical to the pre-extraction bridges.
 */

/* The Makefile also passes -D_GNU_SOURCE; guard so a standalone compile
 * still works without redefining it. */
#ifdef __linux__
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "bridge.h"
#include "sdnotify.h"
#include "log.h"
#include "version.h"                 /* IMUD_VERSION_STR — canonical version */

/* ── Signals ─────────────────────────────────────────────────────────────── */

volatile sig_atomic_t bridge_stop;
volatile sig_atomic_t bridge_reload;

static void on_signal(int sig)
{
    if (sig == SIGHUP) bridge_reload = 1;
    else               bridge_stop   = 1;
}

void bridge_install_signals(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGHUP,  &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
}

void bridge_sleep_interruptible(int secs)
{
    for (int i = 0; i < secs && !bridge_stop && !bridge_reload; i++) sleep(1);
}

/* ── CLI + config ────────────────────────────────────────────────────────── */

/* Help to stdout, diagnostics to stderr — see the stream contract in cli.c. */
static void usage(FILE *o, const char *prog, const bridge_info_t *bi)
{
    fprintf(o,
        "Usage: %s [--config PATH]\n"
        "\n"
        "%s"
        "\n"
        "Options:\n"
        "  --config PATH   Config file (default: %s)\n"
        "  --version       Print version and exit\n"
        "  -h, --help      Print this help and exit\n",
        prog, bi->usage_desc, bi->default_conf);
}

int bridge_parse_cli(int argc, char **argv, const bridge_info_t *bi,
                     char *config_path, size_t pathsz)
{
    snprintf(config_path, pathsz, "%s", bi->default_conf);

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            snprintf(config_path, pathsz, "%s", argv[++i]);
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("%s %s\n", bi->prog, IMUD_VERSION_STR);
            return 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(stdout, argv[0], bi);
            return 1;
        } else {
            LOG_E("unknown option: %s\n", argv[i]);
            usage(stderr, argv[0], bi);
            return -1;
        }
    }
    return 0;
}

int bridge_load_config(const bridge_info_t *bi, const char *path,
                       imud_config_t *cfg)
{
    config_defaults(cfg);
    int rc = config_load(path, cfg);
    if (rc == CONFIG_ERR_PARSE) {
        LOG_E("[%s] %s has errors (see above) — refusing to start\n",
              bi->tag, path);
        return -1;
    }
    /* Not merely "no config, use defaults": the file is there and we cannot
     * read it.  Carrying on would run with every output disabled and exit
     * saying "disabled in config", which points the operator at the wrong
     * thing entirely.  The config installs 0640 root:imud — a bridge that is
     * not in the imud group lands here. */
    if (rc == CONFIG_ERR_PERM) {
        LOG_E("[%s] %s exists but cannot be read (see above) — refusing to "
              "start; check that it is readable by the service user\n",
              bi->tag, path);
        return -1;
    }

    /* Match the daemon's logging behaviour. */
    if (getenv("INVOCATION_ID")) log_set_style(LOG_STYLE_JOURNAL);
    log_set_level_str(cfg->log_level);
    return 0;
}

void bridge_exit_disabled(const bridge_info_t *bi)
{
    LOG_I("[%s] disabled in config ([%s] enabled = false) — exiting\n",
          bi->tag, bi->section);
    sd_notify_msg("READY=1");   /* signal a clean start so systemd stops us, no restart loop */
}

int bridge_reload_begin(const bridge_info_t *bi, const char *path,
                        imud_config_t *nc)
{
    if (!bridge_reload) return 0;
    bridge_reload = 0;

    config_defaults(nc);
    int rc = config_load(path, nc);
    if (rc == CONFIG_ERR_PARSE || rc == CONFIG_ERR_PERM) {
        LOG_W("[%s] reload failed — keeping current config\n", bi->tag);
        return 0;
    }
    log_set_level_str(nc->log_level);
    return 1;
}

void bridge_reload_done(const bridge_info_t *bi)
{
    LOG_I("[%s] config reloaded\n", bi->tag);
}

/* ── Emit-tick timing ────────────────────────────────────────────────────── */

long bridge_period_ns(int rate_hz, long fallback_ns)
{
    return (rate_hz > 0) ? 1000000000L / rate_hz : fallback_ns;
}

int bridge_wait_ms(const struct timespec *now, const struct timespec *next)
{
    long wait_ns = (next->tv_sec - now->tv_sec) * 1000000000L
                 + (next->tv_nsec - now->tv_nsec);
    if (wait_ns < 0) wait_ns = 0;
    return (int)((wait_ns + 999999L) / 1000000L);
}

bool bridge_due(const struct timespec *now, const struct timespec *next)
{
    return (now->tv_sec > next->tv_sec) ||
           (now->tv_sec == next->tv_sec && now->tv_nsec >= next->tv_nsec);
}

void bridge_advance(struct timespec *next, const struct timespec *now,
                    long period_ns)
{
    /* Advance the deadline, skipping missed ticks after a stall. */
    do {
        next->tv_nsec += period_ns;
        while (next->tv_nsec >= 1000000000L) {
            next->tv_sec++; next->tv_nsec -= 1000000000L;
        }
    } while ((next->tv_sec < now->tv_sec) ||
             (next->tv_sec == now->tv_sec && next->tv_nsec <= now->tv_nsec));
}

const struct timespec *bridge_earlier(const struct timespec *a,
                                      const struct timespec *b)
{
    return ((a->tv_sec < b->tv_sec) ||
            (a->tv_sec == b->tv_sec && a->tv_nsec < b->tv_nsec)) ? a : b;
}

/* ── Stream plumbing ─────────────────────────────────────────────────────── */

int bridge_stream_ensure(imud_t **stream, const char *sock_path,
                         const char *tag, int retry_sleep_s)
{
    if (*stream) return 1;

    *stream = imud_connect_stream(sock_path);
    if (!*stream) {
        LOG_W("[%s] stream socket %s unavailable: %s — retrying\n",
              tag, sock_path, strerror(errno));
        if (retry_sleep_s > 0) bridge_sleep_interruptible(retry_sleep_s);
        return 0;
    }
    LOG_I("[%s] connected to %s\n", tag, sock_path);
    return 2;
}

void bridge_stream_drop(imud_t **stream, const char *tag)
{
    LOG_I("[%s] stream disconnected\n", tag);
    imud_free(*stream);
    *stream = NULL;
}

/* ── Output transports ───────────────────────────────────────────────────── */

int bridge_open_udp(const char *host, int port, const char *tag,
                    struct sockaddr_storage *dst, socklen_t *dlen)
{
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) {
        LOG_E("[%s] cannot resolve UDP host '%s'\n", tag, host);
        return -1;
    }
    int fd = socket(res->ai_family, res->ai_socktype | SOCK_CLOEXEC, 0);
    if (fd < 0) { freeaddrinfo(res); return -1; }
    memcpy(dst, res->ai_addr, res->ai_addrlen);
    *dlen = (socklen_t)res->ai_addrlen;
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof yes);  /* harmless for unicast */
    freeaddrinfo(res);
    return fd;
}

int bridge_write_all(int fd, const char *b, int n)
{
    int off = 0;
    while (off < n) {
        ssize_t w = write(fd, b + off, n - off);
        if (w <= 0) { if (w < 0 && errno == EINTR) continue; return -1; }
        off += (int)w;
    }
    return 0;
}
