/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_netserv.c — unit tests for the netserv TCP broadcast server
 *
 * Pure sockets (macOS-buildable, no gpiod). Exercises:
 *   - open on an ephemeral port (port 0) reports the bound port
 *   - N clients connect and every one receives each broadcast
 *   - a client beyond NETSRV_MAX_CLIENTS is accepted-then-closed (EOF)
 *   - a closed (dead) client is disconnected and swap-removed
 *   - broadcast with zero clients / on a never-opened server is a no-op
 *   - netserv_close is idempotent
 *   - a non-numeric bind address is rejected (no DNS)
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "../include/netserv.h"
#include "fdsweep.h"

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin_test(const char *name)
{
    printf("%-52s", name);
    fflush(stdout);
}

static void end_test(int fail_before)
{
    puts(g_fail == fail_before ? "OK" : "FAIL");
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/* The test's own client sockets are close-on-exec so that fdsweep_leaks()
 * has a zero noise floor — any fd it reports then belongs to netserv. */
static int connect_client(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_port   = htons((uint16_t)port);
    inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);
    if (connect(fd, (struct sockaddr *)&sa, sizeof sa) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/* Read exactly len bytes; returns 0 on success, -1 on EOF/timeout/error. */
static int read_exact(int fd, void *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        ssize_t n = read(fd, (char *)buf + got, len - got);
        if (n <= 0) return -1;
        got += (size_t)n;
    }
    return 0;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_open_ephemeral(void)
{
    begin_test("test_open_ephemeral");
    int fb = g_fail;

    netserv_t s;
    EXPECT(netserv_open(&s, "127.0.0.1", 0, "test") == 0,
           "open on port 0 succeeds");
    EXPECT(s.port > 0, "bound port reported back");
    EXPECT(netserv_nclients(&s) == 0, "no clients after open");
    netserv_close(&s);
    end_test(fb);
}

/*
 * Every fd netserv creates must be close-on-exec.
 *
 * The accepted clients are the point. POSIX does not carry FD_CLOEXEC across
 * accept(), so the listener's flag says nothing about theirs — before
 * ACCEPT_CLOEXEC() they were plain on Linux, where the fcntl fallback in
 * APPLY_CLOEXEC compiles to ((void)0).
 */
static void test_cloexec(void)
{
    begin_test("test_cloexec");
    int fb = g_fail;

    fdsweep_t sw;
    fdsweep_begin(&sw);

    netserv_t s;
    EXPECT(netserv_open(&s, "127.0.0.1", 0, "test") == 0, "server opens");
    EXPECT(fdsweep_leaks(&sw) == 0, "listen fd is close-on-exec");

    int c = connect_client(s.port);
    EXPECT(c >= 0, "client connects");
    netserv_accept(&s);
    EXPECT(netserv_nclients(&s) == 1, "client accepted");
    EXPECT(fdsweep_leaks(&sw) == 0, "accepted client fd is close-on-exec too");

    if (c >= 0) close(c);
    netserv_close(&s);
    end_test(fb);
}

static void test_bad_bind_addr(void)
{
    begin_test("test_bad_bind_addr");
    int fb = g_fail;

    netserv_t s;
    EXPECT(netserv_open(&s, "localhost", 0, "test") < 0,
           "hostname bind address rejected (numeric IPv4 only)");
    EXPECT(netserv_nclients(&s) == 0, "failed open leaves server closed");
    netserv_close(&s);   /* must be a safe no-op */
    end_test(fb);
}

static void test_broadcast_to_all(void)
{
    begin_test("test_broadcast_to_all");
    int fb = g_fail;

    netserv_t s;
    EXPECT(netserv_open(&s, "127.0.0.1", 0, "test") == 0, "server opens");

    /* Broadcast with zero clients is a no-op (must not crash or block). */
    netserv_broadcast(&s, "x", 1);

    enum { N = 3 };
    int cli[N];
    int ok = 0;
    for (int i = 0; i < N; i++) {
        cli[i] = connect_client(s.port);
        if (cli[i] >= 0) ok++;
    }
    EXPECT(ok == N, "three clients connect");

    netserv_accept(&s);
    EXPECT(netserv_nclients(&s) == N, "all three accepted");

    const char payload[] = "$HCHDT,123.4,T*1F\r\n";
    netserv_broadcast(&s, payload, sizeof payload - 1);

    int delivered = 0;
    char buf[sizeof payload];
    for (int i = 0; i < N; i++) {
        if (read_exact(cli[i], buf, sizeof payload - 1) == 0 &&
            memcmp(buf, payload, sizeof payload - 1) == 0)
            delivered++;
    }
    EXPECT(delivered == N, "every client receives the broadcast");

    for (int i = 0; i < N; i++) close(cli[i]);
    netserv_close(&s);
    end_test(fb);
}

static void test_client_limit(void)
{
    begin_test("test_client_limit");
    int fb = g_fail;

    netserv_t s;
    EXPECT(netserv_open(&s, "127.0.0.1", 0, "test") == 0, "server opens");

    int cli[NETSRV_MAX_CLIENTS];
    for (int i = 0; i < NETSRV_MAX_CLIENTS; i++) {
        cli[i] = connect_client(s.port);
        netserv_accept(&s);
    }
    EXPECT(netserv_nclients(&s) == NETSRV_MAX_CLIENTS,
           "server holds NETSRV_MAX_CLIENTS clients");

    /* One more: accepted then immediately closed — the client sees EOF. */
    int extra = connect_client(s.port);
    EXPECT(extra >= 0, "extra client connects at TCP level");
    netserv_accept(&s);
    EXPECT(netserv_nclients(&s) == NETSRV_MAX_CLIENTS,
           "client beyond the limit is not kept");
    char b;
    EXPECT(read(extra, &b, 1) == 0, "rejected client sees EOF");
    close(extra);

    for (int i = 0; i < NETSRV_MAX_CLIENTS; i++)
        if (cli[i] >= 0) close(cli[i]);
    netserv_close(&s);
    end_test(fb);
}

static void test_dead_client_pruned(void)
{
    begin_test("test_dead_client_pruned");
    int fb = g_fail;

    netserv_t s;
    EXPECT(netserv_open(&s, "127.0.0.1", 0, "test") == 0, "server opens");

    int keep = connect_client(s.port);
    int dead = connect_client(s.port);
    netserv_accept(&s);
    EXPECT(netserv_nclients(&s) == 2, "two clients accepted");

    close(dead);          /* peer goes away */
    usleep(50 * 1000);    /* let the FIN/RST land */

    /* First send may still be accepted by the kernel; the second one after
     * the RST comes back must fail and prune the client. */
    const char payload[] = "ping\n";
    netserv_broadcast(&s, payload, sizeof payload - 1);
    usleep(50 * 1000);
    netserv_broadcast(&s, payload, sizeof payload - 1);
    EXPECT(netserv_nclients(&s) == 1, "dead client swap-removed");

    /* The surviving client still gets the data (both broadcasts). */
    char buf[2 * (sizeof payload - 1)];
    EXPECT(read_exact(keep, buf, sizeof buf) == 0,
           "surviving client still receives broadcasts");

    close(keep);
    netserv_close(&s);
    end_test(fb);
}

static void test_unopened_noops(void)
{
    begin_test("test_unopened_noops");
    int fb = g_fail;

    netserv_t s;
    netserv_init(&s);
    netserv_accept(&s);                  /* all must be safe no-ops */
    netserv_broadcast(&s, "x", 1);
    EXPECT(netserv_nclients(&s) == 0, "unopened server has no clients");
    netserv_close(&s);
    netserv_close(&s);                   /* idempotent */
    EXPECT(1, "no crash on unopened/double close");
    end_test(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud netserv tests ===");
    signal(SIGPIPE, SIG_IGN);   /* dead-client sends must not kill the test */

    test_open_ephemeral();
    test_cloexec();
    test_bad_bind_addr();
    test_broadcast_to_all();
    test_client_limit();
    test_dead_client_pruned();
    test_unopened_noops();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
