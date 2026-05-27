/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * position.c — gpsd TCP stream + SignalK HTTP poll for live vessel position
 *
 * Connects to gpsd or polls the SignalK REST API to obtain lat/lon fixes.
 * When the position changes by more than POS_UPDATE_THRESH_DEG (≈ 5 km),
 * recomputes WMM magnetic declination and pushes the new value into the
 * fusion thread via imu_ctx_set_declination().
 *
 * gpsd protocol: JSON streaming over TCP port 2947.  Sends
 *   ?WATCH={"enable":true,"json":true}
 * then reads lines, looking for TPV messages with mode ≥ 2.
 *
 * SignalK protocol: plain HTTP/1.0 GET of the position REST endpoint,
 * polled every POS_SIGNALK_POLL_S seconds.
 *
 * Both sources use getaddrinfo() for host resolution and a non-blocking
 * connect with a 5-second timeout so the thread never hangs indefinitely
 * on an unreachable host.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "position.h"
#include "imu.h"
#include "wmm.h"

/* ── Tunables ────────────────────────────────────────────────────────────── */

/* Minimum position change (degrees) that triggers a WMM recompute. */
#define POS_UPDATE_THRESH_DEG  0.05    /* ≈ 5 km */

/* Seconds between SignalK polls. */
#define POS_SIGNALK_POLL_S     30

/* Seconds to wait before retrying a failed/dropped gpsd connection. */
#define POS_GPSD_RETRY_S        5

/* TCP connect timeout (seconds). */
#define POS_CONNECT_TIMEOUT_S   5

/* ── JSON field extractor ─────────────────────────────────────────────────── */

/*
 * pos_json_double — find "key":VALUE in a JSON string and parse VALUE.
 * Exposed in position.h for unit tests.
 */
bool pos_json_double(const char *json, const char *key, double *out)
{
    char needle[80];
    snprintf(needle, sizeof needle, "\"%s\":", key);
    const char *p = strstr(json, needle);
    if (!p) return false;
    p += strlen(needle);
    while (*p == ' ' || *p == '\t') p++;
    char *end;
    double v = strtod(p, &end);
    if (end == p) return false;   /* not a number (e.g. null, "string") */
    *out = v;
    return true;
}

/* ── TCP helpers ─────────────────────────────────────────────────────────── */

/*
 * tcp_connect_host — open a non-blocking TCP connection with a timeout.
 * Sets SO_RCVTIMEO = 10s on the resulting socket.
 * Returns fd on success, -1 on failure.
 */
static int tcp_connect_host(const char *host, int port)
{
    struct addrinfo hints = {0}, *res;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    if (getaddrinfo(host, portstr, &hints, &res) != 0) return -1;

    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) { freeaddrinfo(res); return -1; }

    /* Non-blocking connect so we can impose a timeout. */
    fcntl(fd, F_SETFL, O_NONBLOCK);
    int r = connect(fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);

    if (r < 0 && errno == EINPROGRESS) {
        fd_set wfds; FD_ZERO(&wfds); FD_SET(fd, &wfds);
        struct timeval tv = { .tv_sec = POS_CONNECT_TIMEOUT_S, .tv_usec = 0 };
        if (select(fd + 1, NULL, &wfds, NULL, &tv) > 0) {
            int err = 0; socklen_t sl = sizeof(err);
            getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &sl);
            if (err != 0) { close(fd); return -1; }
        } else {
            /* Timeout or select error. */
            close(fd); return -1;
        }
    } else if (r < 0) {
        close(fd); return -1;
    }

    /* Restore blocking mode for normal I/O. */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    /* Receive timeout: prevents recv() from blocking forever on stalled servers. */
    struct timeval rtv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof rtv);

    return fd;
}

/* Sleep for `secs` seconds in 1-second increments, checking *stop each tick. */
static void interruptible_sleep(int secs, volatile sig_atomic_t *stop)
{
    for (int i = 0; i < secs && !*stop; i++) sleep(1);
}

/* ── Line reader (gpsd) ──────────────────────────────────────────────────── */

typedef struct {
    int  fd;
    char buf[4096];
    int  pos;
    int  len;
} line_reader_t;

static void line_reader_init(line_reader_t *r, int fd)
{
    r->fd = fd; r->pos = 0; r->len = 0;
}

/*
 * read_line — read one newline-terminated line from a TCP socket.
 * Uses select() with a 1-second timeout to remain responsive to ctx->stop.
 * Returns the line length (≥ 0) on success, -1 on disconnect or stop.
 */
static int read_line(line_reader_t *r, char *out, int outsz,
                     volatile sig_atomic_t *stop)
{
    int n = 0;
    while (!*stop) {
        if (r->pos >= r->len) {
            fd_set fds; FD_ZERO(&fds); FD_SET(r->fd, &fds);
            struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
            int s = select(r->fd + 1, &fds, NULL, NULL, &tv);
            if (s < 0) return -1;
            if (s == 0) continue;   /* timeout — check stop and retry */
            r->len = (int)recv(r->fd, r->buf, sizeof(r->buf), 0);
            if (r->len <= 0) return -1;   /* EOF or error */
            r->pos = 0;
        }
        char c = r->buf[r->pos++];
        if (c == '\n') {
            if (n > 0 && out[n - 1] == '\r') n--;   /* strip \r */
            out[n] = '\0';
            return n;
        }
        if (n < outsz - 1) out[n++] = c;
    }
    return -1;
}

/* ── WMM declination update ──────────────────────────────────────────────── */

/*
 * maybe_update_decl — record a valid GPS fix and, if the position has shifted
 * by more than POS_UPDATE_THRESH_DEG degrees in lat or lon, recompute WMM
 * declination and push the new value into the fusion thread.
 *
 * *last_fix_time is updated on every valid fix arrival, regardless of whether
 * the position has moved or WMM is available.  This ensures the TTL watchdog
 * in position_thread() reflects live GPS reception, not just threshold crossings
 * — an anchored vessel receiving steady fixes never trips the watchdog.
 */
static void maybe_update_decl(pos_ctx_t *ctx, const wmm_t *wmm,
                               double lat, double lon,
                               double *last_lat, double *last_lon,
                               time_t *last_fix_time)
{
    /* Record valid fix regardless of position change or WMM availability. */
    *last_fix_time = time(NULL);

    if (!wmm) return;
    if (fabs(lat - *last_lat) < POS_UPDATE_THRESH_DEG &&
        fabs(lon - *last_lon) < POS_UPDATE_THRESH_DEG)
        return;

    double year = wmm_decimal_year();
    double decl = wmm_declination(lat, lon, 0.0, year, wmm);
    imu_ctx_set_declination(ctx->imu, (float)decl);
    fprintf(stderr, "[pos] fix (%.4f°N, %.4f°E) → decl=%.2f°E\n",
            lat, lon, decl);
    *last_lat = lat;
    *last_lon = lon;
}

/* ── gpsd streaming ──────────────────────────────────────────────────────── */

/*
 * run_gpsd — connect to gpsd, subscribe to JSON, read TPV messages.
 * Blocks until the connection drops or ctx->stop is set.
 */
static void run_gpsd(pos_ctx_t *ctx, const wmm_t *wmm,
                     double *last_lat, double *last_lon,
                     time_t *last_fix_time)
{
    int fd = tcp_connect_host(ctx->cfg->pos_gpsd_host, ctx->cfg->pos_gpsd_port);
    if (fd < 0) {
        fprintf(stderr, "[pos] gpsd %s:%d unavailable: %s\n",
                ctx->cfg->pos_gpsd_host, ctx->cfg->pos_gpsd_port,
                strerror(errno));
        return;
    }
    fprintf(stderr, "[pos] gpsd connected %s:%d\n",
            ctx->cfg->pos_gpsd_host, ctx->cfg->pos_gpsd_port);

    const char *watch = "?WATCH={\"enable\":true,\"json\":true}\n";
    (void)write(fd, watch, strlen(watch));

    line_reader_t rdr;
    line_reader_init(&rdr, fd);
    char line[4096];

    while (read_line(&rdr, line, sizeof line, &ctx->stop) >= 0) {
        if (!strstr(line, "\"class\":\"TPV\"")) continue;
        double mode = 0;
        pos_json_double(line, "mode", &mode);
        if (mode < 2.0) continue;   /* no fix yet */
        double lat, lon;
        if (!pos_json_double(line, "lat", &lat)) continue;
        if (!pos_json_double(line, "lon", &lon)) continue;
        maybe_update_decl(ctx, wmm, lat, lon, last_lat, last_lon, last_fix_time);
    }

    fprintf(stderr, "[pos] gpsd disconnected\n");
    close(fd);
}

/* ── SignalK HTTP poll ────────────────────────────────────────────────────── */

/*
 * poll_signalk — perform a single HTTP GET of the SignalK position endpoint
 * and call maybe_update_decl() if a valid lat/lon is returned.
 */
static void poll_signalk(pos_ctx_t *ctx, const wmm_t *wmm,
                         double *last_lat, double *last_lon,
                         time_t *last_fix_time)
{
    int fd = tcp_connect_host(ctx->cfg->pos_signalk_host,
                              ctx->cfg->pos_signalk_port);
    if (fd < 0) {
        fprintf(stderr, "[pos] SignalK %s:%d unavailable: %s\n",
                ctx->cfg->pos_signalk_host, ctx->cfg->pos_signalk_port,
                strerror(errno));
        return;
    }

    char req[512];
    snprintf(req, sizeof req,
             "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
             ctx->cfg->pos_signalk_path, ctx->cfg->pos_signalk_host);
    if (write(fd, req, strlen(req)) < 0) { close(fd); return; }

    /* Read the full response (server closes on EOF for HTTP/1.0). */
    char resp[8192]; int total = 0; ssize_t n;
    while ((n = recv(fd, resp + total,
                     (size_t)(sizeof(resp) - 1 - total), 0)) > 0)
        total += (int)n;
    close(fd);
    resp[total] = '\0';

    /* Skip HTTP response headers. */
    const char *body = strstr(resp, "\r\n\r\n");
    if (!body) return;
    body += 4;

    /*
     * SignalK position response:
     *   {"value":{"longitude":-122.315,"latitude":37.870},...}
     */
    double lat, lon;
    if (!pos_json_double(body, "latitude",  &lat)) return;
    if (!pos_json_double(body, "longitude", &lon)) return;
    maybe_update_decl(ctx, wmm, lat, lon, last_lat, last_lon, last_fix_time);
}

/* ── Thread entry point ──────────────────────────────────────────────────── */

void *position_thread(void *arg)
{
    pos_ctx_t *ctx = arg;

    if (!ctx->cfg->pos_gpsd_enabled && !ctx->cfg->pos_signalk_enabled)
        return NULL;

    /*
     * Load a private copy of the WMM coefficients.  The position thread owns
     * this struct; it is never shared with the main thread.
     */
    wmm_t wmm;
    bool wmm_ok = (wmm_load(ctx->cfg->pos_wmm_file, &wmm) == 0);
    if (!wmm_ok)
        fprintf(stderr, "[pos] WMM file '%s' unavailable — "
                "position fixes will not update declination\n",
                ctx->cfg->pos_wmm_file);

    fprintf(stderr, "[pos] position thread started");
    if (ctx->cfg->pos_gpsd_enabled)
        fprintf(stderr, "  gpsd=%s:%d",
                ctx->cfg->pos_gpsd_host, ctx->cfg->pos_gpsd_port);
    if (ctx->cfg->pos_signalk_enabled)
        fprintf(stderr, "  signalk=%s:%d%s",
                ctx->cfg->pos_signalk_host, ctx->cfg->pos_signalk_port,
                ctx->cfg->pos_signalk_path);
    fputc('\n', stderr);

    /* Force first update regardless of position change. */
    double last_lat = 1000.0, last_lon = 1000.0;

    /* Timestamp of the last valid GPS fix received (any source).
     * Updated by maybe_update_decl() on every fix, regardless of threshold.
     * 0 = no fix obtained yet this session. */
    time_t last_fix_time = 0;

    /* Timestamp of the last SignalK poll — used to enforce the 30s minimum
     * interval when SignalK runs as a fallback in the gpsd-enabled path.
     * In SignalK-only mode the interruptible_sleep already provides the delay. */
    time_t last_signalk_time = 0;

    while (!ctx->stop) {
        /*
         * TTL expiry watchdog — if a fix was previously obtained but hasn't
         * been refreshed within pos_fix_max_age_h hours, clear the declination
         * so FLAG_DECLINATION_VALID drops and true-heading output stops.
         * Set fix_max_age_h = 0 to disable (declination persists indefinitely).
         *
         * Note: when gpsd is connected and streaming, run_gpsd() below blocks
         * until the connection drops, so this check runs each reconnect cycle
         * (typically every POS_GPSD_RETRY_S seconds on failure).  A live gpsd
         * session with valid fixes keeps last_fix_time current, so the TTL only
         * fires during a multi-hour GPS outage.
         */
        if (ctx->cfg->pos_fix_max_age_h > 0.0f &&
            last_fix_time > 0 &&
            difftime(time(NULL), last_fix_time) >
                    (double)(ctx->cfg->pos_fix_max_age_h * 3600.0f)) {
            fprintf(stderr, "[pos] GPS fix expired (>%.1f h old) — clearing declination\n",
                    (double)ctx->cfg->pos_fix_max_age_h);
            imu_ctx_set_declination(ctx->imu, 0.0f);
            last_fix_time = 0;   /* prevent repeated clears */
        }

        if (ctx->cfg->pos_gpsd_enabled) {
            /*
             * run_gpsd() blocks until the connection drops.  When it returns
             * (whether from disconnect or immediate connect failure), do a
             * SignalK fallback poll — if enabled — but only if POS_SIGNALK_POLL_S
             * seconds have elapsed since the last poll.  Without this guard,
             * a repeatedly-failing gpsd connection would hammer SignalK every
             * POS_GPSD_RETRY_S (~5 s) instead of the intended 30 s.
             */
            run_gpsd(ctx, wmm_ok ? &wmm : NULL,
                     &last_lat, &last_lon, &last_fix_time);
            if (!ctx->stop && ctx->cfg->pos_signalk_enabled &&
                difftime(time(NULL), last_signalk_time) >= POS_SIGNALK_POLL_S) {
                poll_signalk(ctx, wmm_ok ? &wmm : NULL,
                             &last_lat, &last_lon, &last_fix_time);
                last_signalk_time = time(NULL);
            }
            interruptible_sleep(POS_GPSD_RETRY_S, &ctx->stop);
        } else {
            /* SignalK only — poll, then sleep. */
            poll_signalk(ctx, wmm_ok ? &wmm : NULL,
                         &last_lat, &last_lon, &last_fix_time);
            last_signalk_time = time(NULL);
            interruptible_sleep(POS_SIGNALK_POLL_S, &ctx->stop);
        }
    }

    return NULL;
}
