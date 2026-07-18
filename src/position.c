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
#include "log.h"

/* ── Tunables ────────────────────────────────────────────────────────────── */

/* Minimum position change (degrees) that triggers a WMM recompute. */
#define POS_UPDATE_THRESH_DEG  0.05    /* ≈ 5 km */

/* Seconds between SignalK polls. */
#define POS_SIGNALK_POLL_S     30

/* Seconds to wait before retrying a failed/dropped gpsd connection. */
#define POS_GPSD_RETRY_S        5

/* TCP connect timeout (seconds). */
#define POS_CONNECT_TIMEOUT_S   5

/* Sentinel for last_lat/last_lon guaranteed to exceed the update threshold,
 * forcing a declination recompute on the next valid fix. */
#define POS_FORCE_UPDATE_DEG 1000.0

/* read_line() return code: select() timed out with no complete line buffered.
 * The connection is still up; the caller may run periodic housekeeping. */
#define POS_READ_IDLE (-2)

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
    /* strtod also parses "nan"/"inf(inity)" — not valid JSON numbers, and a
     * NaN/Inf here would poison the WMM/atan2 path and the MEKF. Reject. */
    if (!isfinite(v)) return false;
    /* The value must be terminated by a JSON delimiter, not stray text —
     * rejects a numeric prefix like "lat":12abc. */
    if (*end != '\0' && !strchr(" \t\r\n,}]", *end)) return false;
    *out = v;
    return true;
}

/*
 * Range/finiteness gate for a position fix before it feeds declination and
 * the magnetic reference. Both the gpsd and SignalK paths pass through here.
 */
bool pos_fix_valid(double lat, double lon, double alt)
{
    return isfinite(lat) && lat >= -90.0  && lat <= 90.0
        && isfinite(lon) && lon >= -180.0 && lon <= 180.0
        && isfinite(alt);
}

/* ── TCP helpers ─────────────────────────────────────────────────────────── */

/*
 * tcp_connect_host — open a TCP connection with a per-address timeout.
 * Tries every address getaddrinfo returns (a multi-homed host whose first
 * address is down still connects).  Sets SO_RCVTIMEO = 10s on the result.
 * Returns fd on success; -1 on failure with errno describing the last
 * real connect error, so callers' strerror(errno) messages are accurate.
 */
static int tcp_connect_host(const char *host, int port)
{
    struct addrinfo hints = {0}, *res;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    if (getaddrinfo(host, portstr, &hints, &res) != 0) {
        errno = EHOSTUNREACH;   /* getaddrinfo failure doesn't set errno */
        return -1;
    }

    int fd = -1, err = EHOSTUNREACH;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) { err = errno; continue; }

        /* Non-blocking connect so we can impose a timeout. */
        fcntl(fd, F_SETFL, O_NONBLOCK);
        int r = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (r == 0) break;                       /* immediate connect */
        if (r < 0 && errno == EINPROGRESS) {
            fd_set wfds; FD_ZERO(&wfds); FD_SET(fd, &wfds);
            struct timeval tv = { .tv_sec = POS_CONNECT_TIMEOUT_S, .tv_usec = 0 };
            if (select(fd + 1, NULL, &wfds, NULL, &tv) > 0) {
                int soerr = 0; socklen_t sl = sizeof(soerr);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &sl);
                if (soerr == 0) break;           /* connected */
                err = soerr;
            } else {
                err = ETIMEDOUT;
            }
        } else {
            err = errno;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    if (fd < 0) { errno = err; return -1; }

    /* Restore blocking mode for normal I/O. */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    /* Receive timeout: prevents recv() from blocking forever on stalled servers. */
    struct timeval rtv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof rtv);

    return fd;
}

/* Sleep for `secs` seconds in 1-second increments, checking *stop each tick. */
static void interruptible_sleep(int secs, _Atomic sig_atomic_t *stop)
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
 * Returns the line length (≥ 0) on success, -1 on disconnect or stop, or
 * POS_READ_IDLE when a full second passes with no data and no partial line
 * pending — so the caller can run housekeeping while a quiet connection
 * stays up.
 */
static int read_line(line_reader_t *r, char *out, int outsz,
                     _Atomic sig_atomic_t *stop)
{
    int n = 0;
    while (!*stop) {
        if (r->pos >= r->len) {
            fd_set fds; FD_ZERO(&fds); FD_SET(r->fd, &fds);
            struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
            int s = select(r->fd + 1, &fds, NULL, NULL, &tv);
            if (s < 0) return -1;
            if (s == 0) {
                /* Mid-line data would be lost by returning here, so only
                 * report idle between lines; gpsd writes whole lines. */
                if (n == 0) return POS_READ_IDLE;
                continue;
            }
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
                               double lat, double lon, double alt_m,
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
    double ned[3];
    wmm_field_ned(lat, lon, alt_m, year, wmm, ned);
    double decl = atan2(ned[1], ned[0]) * (180.0 / M_PI);
    imu_ctx_set_declination(ctx->imu, (float)decl, true);
    /* Field invariants (nT → Gauss) for the MEKF magnetic reference. */
    imu_ctx_set_mag_ref(ctx->imu,
                        (float)(sqrt(ned[0]*ned[0] + ned[1]*ned[1]) * 1e-5),
                        (float)(ned[2] * 1e-5));
    LOG_I("[pos] fix (%.4f°N, %.4f°E) → decl=%.2f°E\n",
            lat, lon, decl);
    *last_lat = lat;
    *last_lon = lon;
}

/*
 * check_fix_ttl — clear declination when the last GPS fix is older than
 * pos_fix_max_age_h hours, so FLAG_DECLINATION_VALID drops and true-heading
 * output stops rather than steering on stale variation.
 * fix_max_age_h = 0 disables the watchdog (declination persists forever).
 *
 * Called from the outer thread loop (SignalK mode, gpsd reconnect cycles)
 * AND from inside run_gpsd's read loop, so it also fires while a gpsd
 * session stays connected but stops delivering fixes (antenna failure,
 * device removed).
 *
 * On expiry, last_lat/last_lon are reset to the force-update sentinel so
 * the next valid fix — even at an unchanged anchorage — recomputes
 * declination immediately instead of waiting for a ≥ POS_UPDATE_THRESH_DEG
 * move that an anchored vessel may never make.
 *
 * Non-static: exposed in position.h for unit tests (like pos_json_double).
 */
void check_fix_ttl(pos_ctx_t *ctx, time_t *last_fix_time,
                   double *last_lat, double *last_lon)
{
    if (ctx->cfg->pos_fix_max_age_h <= 0.0f || *last_fix_time == 0)
        return;
    if (difftime(time(NULL), *last_fix_time) <=
            (double)(ctx->cfg->pos_fix_max_age_h * 3600.0f))
        return;

    LOG_W("[pos] GPS fix expired (>%.1f h old) — clearing declination\n",
            (double)ctx->cfg->pos_fix_max_age_h);
    imu_ctx_set_declination(ctx->imu, 0.0f, false);
    imu_ctx_set_speed(ctx->imu, 0.0f, false);   /* stale speed must not steer */
    *last_fix_time = 0;   /* prevent repeated clears */
    *last_lat = POS_FORCE_UPDATE_DEG;
    *last_lon = POS_FORCE_UPDATE_DEG;
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
        LOG_W("[pos] gpsd %s:%d unavailable: %s\n",
                ctx->cfg->pos_gpsd_host, ctx->cfg->pos_gpsd_port,
                strerror(errno));
        return;
    }
    LOG_I("[pos] gpsd connected %s:%d\n",
            ctx->cfg->pos_gpsd_host, ctx->cfg->pos_gpsd_port);

    const char *watch = "?WATCH={\"enable\":true,\"json\":true}\n";
    (void)write(fd, watch, strlen(watch));

    line_reader_t rdr;
    line_reader_init(&rdr, fd);
    char line[4096];

    for (;;) {
        /* Run the fix-TTL watchdog per iteration: this loop can hold the
         * thread for hours (streaming fixless TPVs, or a silent-but-open
         * session), and the outer-loop check only runs between connects. */
        check_fix_ttl(ctx, last_fix_time, last_lat, last_lon);

        int r = read_line(&rdr, line, sizeof line, &ctx->stop);
        if (r == POS_READ_IDLE) continue;   /* quiet second — watchdog ran */
        if (r < 0) break;                   /* disconnect or stop */

        if (!strstr(line, "\"class\":\"TPV\"")) continue;
        double mode = 0;
        pos_json_double(line, "mode", &mode);
        if (mode < 2.0) continue;   /* no fix yet */
        double lat, lon;
        if (!pos_json_double(line, "lat", &lat)) continue;
        if (!pos_json_double(line, "lon", &lon)) continue;
        /* Altitude barely moves marine declination but costs nothing to use:
         * newer gpsd emits altMSL, older builds plain alt. Optional. */
        double alt = 0.0;
        if (!pos_json_double(line, "altMSL", &alt))
            pos_json_double(line, "alt", &alt);
        /* Speed over ground feeds the centripetal correction; refresh it on
         * every fix (not just threshold crossings). */
        double spd;
        if (pos_json_double(line, "speed", &spd) && isfinite(spd) && spd >= 0.0)
            imu_ctx_set_speed(ctx->imu, (float)spd, true);
        if (!pos_fix_valid(lat, lon, alt)) continue;
        maybe_update_decl(ctx, wmm, lat, lon, alt,
                          last_lat, last_lon, last_fix_time);
    }

    LOG_I("[pos] gpsd disconnected\n");
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
        LOG_W("[pos] SignalK %s:%d unavailable: %s\n",
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

    /* The position endpoint returns a few hundred bytes; a full buffer means
     * the path points at something bigger and the tail was dropped. */
    if (total >= (int)sizeof(resp) - 1)
        LOG_W("[pos] SignalK response truncated at %zu bytes — "
                "check signalk_path points at the position endpoint\n",
                sizeof(resp) - 1);

    /* Skip HTTP response headers. */
    const char *body = strstr(resp, "\r\n\r\n");
    if (!body) return;
    body += 4;

    /*
     * SignalK position response:
     *   {"value":{"longitude":-122.315,"latitude":37.870},...}
     */
    double lat, lon, alt = 0.0;
    if (!pos_json_double(body, "latitude",  &lat)) return;
    if (!pos_json_double(body, "longitude", &lon)) return;
    pos_json_double(body, "altitude", &alt);   /* optional in SignalK */
    if (!pos_fix_valid(lat, lon, alt)) return;
    maybe_update_decl(ctx, wmm, lat, lon, alt,
                      last_lat, last_lon, last_fix_time);
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
    bool wmm_ok = (wmm_load(ctx->wmm_file, &wmm) == 0);
    if (!wmm_ok)
        LOG_W("[pos] WMM file '%s' unavailable — "
                "position fixes will not update declination\n",
                ctx->wmm_file);

    /* Compose the banner first: log lines must be single emissions so the
     * per-line timestamp/priority prefixes can't split them. */
    {
        /* Sized for the worst case: the literal + gpsd host[64] + signalk
         * host[64] and path[128] with their labels (~316 bytes). */
        char banner[384];
        int  bn = snprintf(banner, sizeof banner, "[pos] position thread started");
        if (ctx->cfg->pos_gpsd_enabled && bn > 0 && bn < (int)sizeof banner)
            bn += snprintf(banner + bn, sizeof banner - (size_t)bn,
                           "  gpsd=%s:%d",
                           ctx->cfg->pos_gpsd_host, ctx->cfg->pos_gpsd_port);
        if (ctx->cfg->pos_signalk_enabled && bn > 0 && bn < (int)sizeof banner)
            snprintf(banner + bn, sizeof banner - (size_t)bn,
                     "  signalk=%s:%d%s",
                     ctx->cfg->pos_signalk_host, ctx->cfg->pos_signalk_port,
                     ctx->cfg->pos_signalk_path);
        LOG_I("%s\n", banner);
    }

    /* Force first update regardless of position change. */
    double last_lat = POS_FORCE_UPDATE_DEG, last_lon = POS_FORCE_UPDATE_DEG;

    /* Timestamp of the last valid GPS fix received (any source).
     * Updated by maybe_update_decl() on every fix, regardless of threshold.
     * 0 = no fix obtained yet this session. */
    time_t last_fix_time = 0;

    /* Timestamp of the last SignalK poll — used to enforce the 30s minimum
     * interval when SignalK runs as a fallback in the gpsd-enabled path.
     * In SignalK-only mode the interruptible_sleep already provides the delay. */
    time_t last_signalk_time = 0;

    while (!ctx->stop) {
        /* Fix-TTL watchdog (also runs inside run_gpsd's read loop — see
         * check_fix_ttl for why both call sites are needed). */
        check_fix_ttl(ctx, &last_fix_time, &last_lat, &last_lon);

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
