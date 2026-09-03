/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_stream.c — end-to-end test of the [stream] subscription outputs
 *
 * Links the real output.c (plus nmea.c/netserv.c/packet.c/config.c) with
 * stubbed imu_get_state()/imu_ctx_is_settled(), starts a real
 * stream_out_thread, connects as a subscriber, and verifies:
 *   - the AF_UNIX listener is created and accepts connections
 *   - packets arrive as exact sizeof(imu_packet_t) frames with correct magic/version
 *   - packet content reflects the fused state (heading roundtrip)
 *   - a disconnected subscriber is pruned and a new one can join
 *   - clean thread stop and socket unlink
 *   - the AF_UNIX listener is 0660 whatever umask the daemon was started
 *     under, and opening it leaves that umask untouched
 *   - the TCP listener ([stream] tcp_*) serves the same framed packets
 *     alongside the AF_UNIX path, and its subscribers get the final
 *     FLAG_SHUTDOWN packet on stop
 *
 * It also covers the other two output threads, which share this file because
 * they share out_ctx: nmea_out_thread and hirate_out_thread sending to real
 * loopback UDP receivers (which is the only coverage open_udp_out has), the
 * [hot] rate change through out_ctx_reload, and out_ctx_send_shutdown's final
 * FLAG_SHUTDOWN packet on the hi-rate socket.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include "../include/types.h"
#include "../include/config.h"
#include "../include/fileio.h"   /* umask_for — the L5 socket-mode helper */
#include "../include/output.h"

#define TEST_SOCK "/tmp/imud_test_stream.sock"

/* ── Stubs so output.c links without imu.c ───────────────────────────────── */

bool imu_ctx_is_settled(imu_ctx_t *ctx)
{
    (void)ctx;
    return true;
}

void imu_get_state(imu_ctx_t *ctx, fused_state_t *state_out,
                   mag_sample_t *mag_out, imu_sample_t *imu_out,
                   imu_sample_t *raw_imu_out)
{
    (void)ctx;
    if (state_out) {
        memset(state_out, 0, sizeof *state_out);
        state_out->q[0]        = 1.0f;
        state_out->heading_deg = 123.4f;
        state_out->flags       = FLAG_FUSION_CONVERGED;
        state_out->ts_wall_ns  = 1751800000000000000ULL;
    }
    if (mag_out)     memset(mag_out,     0, sizeof *mag_out);
    if (imu_out)     memset(imu_out,     0, sizeof *imu_out);
    if (raw_imu_out) memset(raw_imu_out, 0, sizeof *raw_imu_out);
}

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static int connect_subscriber(void)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, TEST_SOCK, sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/* Read exactly one wire-packet frame; returns 0 on success. */
static int read_frame(int fd, unsigned char *frame)
{
    size_t got = 0;
    while (got < sizeof(imu_packet_t)) {
        ssize_t n = read(fd, frame + got, sizeof(imu_packet_t) - got);
        if (n <= 0) return -1;   /* timeout, EOF, or error */
        got += (size_t)n;
    }
    return 0;
}

/* Grab a currently free TCP port (bind :0, read it back, release it). */
static int free_tcp_port(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);
    if (bind(fd, (struct sockaddr *)&sa, sizeof sa) < 0) { close(fd); return -1; }
    socklen_t slen = sizeof sa;
    getsockname(fd, (struct sockaddr *)&sa, &slen);
    int port = (int)ntohs(sa.sin_port);
    close(fd);
    return port;
}

static int connect_tcp_subscriber(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
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

/* ── UDP outputs: nmea_out_thread + hirate_out_thread ────────────────────── */

/*
 * The stream tests above cover stream_out_thread; these cover the other two
 * output threads and open_udp_out, which had no coverage at all.  Both send
 * to a real loopback UDP socket we bind first, so the assertions are on
 * datagrams that actually crossed the stack rather than on internal state.
 */

/* Bind a UDP receiver on loopback and report its port. */
static int udp_receiver(int *port_out)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = 0;
    if (bind(fd, (struct sockaddr *)&a, sizeof a) != 0) { close(fd); return -1; }
    socklen_t alen = sizeof a;
    if (getsockname(fd, (struct sockaddr *)&a, &alen) != 0) { close(fd); return -1; }
    *port_out = ntohs(a.sin_port);
    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return fd;
}

static void test_udp_outputs(void)
{
    puts("test_udp_outputs");
    int fb = g_fail;

    int nmea_port = 0, hi_port = 0;
    int nmea_rx = udp_receiver(&nmea_port);
    int hi_rx   = udp_receiver(&hi_port);
    EXPECT(nmea_rx >= 0 && hi_rx >= 0, "UDP receivers bound on loopback");
    if (nmea_rx < 0 || hi_rx < 0) { puts(g_fail == fb ? "OK" : "FAIL"); return; }

    imud_config_t cfg;
    config_defaults(&cfg);
    cfg.stream_enabled     = false;
    cfg.nmea_enabled       = true;
    cfg.nmea_rate_hz       = 100;          /* fast so the test finishes */
    cfg.nmea_dest_port     = nmea_port;
    cfg.nmea_tcp_enabled   = false;
    snprintf(cfg.nmea_dest_addr, sizeof cfg.nmea_dest_addr, "127.0.0.1");
    cfg.highrate_enabled   = true;
    cfg.highrate_rate_hz   = 100;
    cfg.highrate_dest_port = hi_port;
    snprintf(cfg.highrate_dest_addr, sizeof cfg.highrate_dest_addr, "127.0.0.1");

    out_ctx_t *out = NULL;
    EXPECT(out_ctx_open(&out, &cfg, (imu_ctx_t *)0x1) == 0,
           "out_ctx_open opens both UDP sockets");

    pthread_t nt, ht;
    EXPECT(pthread_create(&nt, NULL, nmea_out_thread, out) == 0,
           "nmea_out_thread starts");
    EXPECT(pthread_create(&ht, NULL, hirate_out_thread, out) == 0,
           "hirate_out_thread starts");

    /* NMEA: a sentence must arrive, start with '$' and carry a *checksum. */
    char sentence[1024];
    ssize_t n = recv(nmea_rx, sentence, sizeof sentence - 1, 0);
    EXPECT(n > 0, "an NMEA datagram arrived");
    if (n > 0) {
        sentence[n] = '\0';
        EXPECT(sentence[0] == '$', "NMEA sentence starts with '$'");
        EXPECT(strchr(sentence, '*') != NULL, "NMEA sentence carries a checksum");
        EXPECT(strstr(sentence, "\r\n") != NULL, "NMEA sentence is CRLF-terminated");
    }

    /* Highrate: a full binary packet with the right magic and version. */
    unsigned char pkt[sizeof(imu_packet_t) + 8];
    ssize_t pn = recv(hi_rx, pkt, sizeof pkt, 0);
    EXPECT(pn == (ssize_t)sizeof(imu_packet_t),
           "highrate datagram is exactly one wire packet");
    if (pn == (ssize_t)sizeof(imu_packet_t)) {
        uint32_t magic;   memcpy(&magic,   pkt,     4);
        uint16_t version; memcpy(&version, pkt + 4, 2);
        EXPECT(magic == IMUD_MAGIC, "highrate packet magic");
        EXPECT(version == IMUD_VERSION, "highrate packet wire version");
    }

    out_ctx_stop(out);
    EXPECT(pthread_join(nt, NULL) == 0, "nmea_out_thread joins cleanly");
    EXPECT(pthread_join(ht, NULL) == 0, "hirate_out_thread joins cleanly");
    out_ctx_free(out);
    close(nmea_rx); close(hi_rx);

    puts(g_fail == fb ? "OK" : "FAIL");
}

/* out_ctx_reload picks up [hot] rate changes; out_ctx_send_shutdown emits the
 * final highrate packet with FLAG_SHUTDOWN set. */
static void test_reload_and_shutdown(void)
{
    puts("test_reload_and_shutdown");
    int fb = g_fail;

    int hi_port = 0;
    int hi_rx = udp_receiver(&hi_port);
    EXPECT(hi_rx >= 0, "UDP receiver bound");
    if (hi_rx < 0) { puts(g_fail == fb ? "OK" : "FAIL"); return; }

    imud_config_t cfg;
    config_defaults(&cfg);
    cfg.stream_enabled     = false;
    cfg.nmea_enabled       = false;
    cfg.highrate_enabled   = true;
    cfg.highrate_rate_hz   = 100;
    cfg.highrate_dest_port = hi_port;
    snprintf(cfg.highrate_dest_addr, sizeof cfg.highrate_dest_addr, "127.0.0.1");

    out_ctx_t *out = NULL;
    EXPECT(out_ctx_open(&out, &cfg, (imu_ctx_t *)0x1) == 0, "out_ctx_open");

    pthread_t ht;
    EXPECT(pthread_create(&ht, NULL, hirate_out_thread, out) == 0,
           "hirate_out_thread starts");

    unsigned char pkt[sizeof(imu_packet_t) + 8];
    EXPECT(recv(hi_rx, pkt, sizeof pkt, 0) == (ssize_t)sizeof(imu_packet_t),
           "streaming at the original rate");

    /* A [hot] rate change must be picked up without a restart. */
    cfg.highrate_rate_hz = 50;
    out_ctx_reload(out, &cfg);
    EXPECT(recv(hi_rx, pkt, sizeof pkt, 0) == (ssize_t)sizeof(imu_packet_t),
           "still streaming after out_ctx_reload");

    out_ctx_stop(out);
    pthread_join(ht, NULL);

    /*
     * Clear what is already queued, so the scan below stays short.  Best
     * effort, and nothing rests on it: pthread_join proves the thread issues
     * no further sendto(), not that every datagram it already sent has reached
     * the receive queue.  A streaming datagram arriving after this loop saw an
     * empty queue is what failed on the macos-latest runner.
     */
    {
        int fl = fcntl(hi_rx, F_GETFL, 0);
        fcntl(hi_rx, F_SETFL, fl | O_NONBLOCK);
        unsigned char drop[sizeof(imu_packet_t) + 8];
        int drained = 0;
        while (recv(hi_rx, drop, sizeof drop, 0) > 0) drained++;
        fcntl(hi_rx, F_SETFL, fl);
        EXPECT(drained >= 0, "receive queue drained before the shutdown check");
    }

    /*
     * Queue a stale streaming packet ahead of the shutdown one deliberately —
     * pkt still holds a real one from the recv above.  The reader has to walk
     * past such a packet whenever the drain missed one, so injecting it makes
     * every host exercise that, rather than only one that loses the race.
     */
    {
        int decoy = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in da;
        memset(&da, 0, sizeof da);
        da.sin_family      = AF_INET;
        da.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        da.sin_port        = htons((uint16_t)hi_port);
        EXPECT(decoy >= 0 &&
               sendto(decoy, pkt, sizeof(imu_packet_t), 0,
                      (struct sockaddr *)&da, sizeof da)
                   == (ssize_t)sizeof(imu_packet_t),
               "a stale streaming packet is queued ahead of the shutdown one");
        if (decoy >= 0) close(decoy);
    }

    /* The shutdown packet is sent after the thread has stopped, by the
     * daemon's exit path — consumers use it to distinguish a clean stop
     * from a crash.  Read forward to it instead of assuming it is next;
     * anything ahead of it is a streaming packet that had not landed yet.
     * The receiver's SO_RCVTIMEO ends the scan if it never comes. */
    out_ctx_send_shutdown(out);
    bool got_pkt = false, saw_shutdown = false;
    for (;;) {
        ssize_t sn = recv(hi_rx, pkt, sizeof pkt, 0);
        if (sn != (ssize_t)sizeof(imu_packet_t)) break;
        got_pkt = true;
        imu_packet_t p;
        memcpy(&p, pkt, sizeof p);
        if (p.flags & FLAG_SHUTDOWN) { saw_shutdown = true; break; }
    }
    EXPECT(got_pkt, "a final packet was sent");
    EXPECT(saw_shutdown, "final packet carries FLAG_SHUTDOWN");

    out_ctx_free(out);
    close(hi_rx);
    puts(g_fail == fb ? "OK" : "FAIL");
}

/*
 * The AF_UNIX listener's permission mode does not depend on the umask it was
 * started under.
 *
 * The window this closes — between bind() and chmod(), where the socket sat at
 * 0777 & ~umask — is not observable from outside the process, so no test here
 * can prove it gone; that rests on bind_unix_mode() in include/fileio.h being
 * the only path to a bound socket.  What these assertions do hold is the
 * contract around it: the resulting mode is 0660 whatever the umask was, the
 * helper's mask arithmetic is right, and the process umask survives the call.
 * That last one matters most — a helper that set the umask and failed to put
 * it back would silently widen every file the daemon created afterwards.
 */
static void test_socket_mode(void)
{
    int fb = g_fail;
    printf("%-52s", "test_stream_socket_mode");
    fflush(stdout);

    EXPECT(umask_for(0660) == 0117, "umask_for(0660) is 0117");
    EXPECT(umask_for(0600) == 0177, "umask_for(0600) is 0177");
    EXPECT(umask_for(0666) == 0111, "umask_for(0666) leaves the x bits");

    /* Permissive, default and restrictive: the socket must land on 0660 from
     * all three, and the umask must come back unchanged from all three. */
    const mode_t masks[] = { 0, 022, 0077 };

    for (size_t i = 0; i < sizeof masks / sizeof masks[0]; i++) {
        imud_config_t cfg;
        config_defaults(&cfg);
        cfg.nmea_enabled     = false;
        cfg.highrate_enabled = false;
        cfg.stream_enabled   = true;
        snprintf(cfg.stream_socket, sizeof cfg.stream_socket, TEST_SOCK);

        unlink(TEST_SOCK);

        mode_t entry = umask(masks[i]);
        out_ctx_t *out = NULL;
        int rc = out_ctx_open(&out, &cfg, (imu_ctx_t *)0x1);
        mode_t after = umask(entry);      /* restore, and read back what we left */

        EXPECT(rc == 0, "out_ctx_open under a non-default umask");
        EXPECT(after == masks[i], "out_ctx_open leaves the process umask alone");

        struct stat st;
        EXPECT(stat(TEST_SOCK, &st) == 0, "stream socket exists");
        EXPECT((st.st_mode & 0777) == 0660,
               "stream socket is 0660 regardless of the umask");
        EXPECT(S_ISSOCK(st.st_mode), "and it really is a socket");

        if (rc == 0) out_ctx_free(out);
        unlink(TEST_SOCK);
    }

    puts(g_fail == fb ? "OK" : "FAIL");
}

/* The [stream] TCP listener serves the same framed packets as AF_UNIX. */
static void test_stream_tcp(void)
{
    int fb = g_fail;
    printf("%-52s", "test_stream_tcp_end_to_end");
    fflush(stdout);

    int port = free_tcp_port();
    EXPECT(port > 0, "found a free TCP port for the listener");

    imud_config_t cfg;
    config_defaults(&cfg);
    cfg.nmea_enabled       = false;
    cfg.highrate_enabled   = false;
    cfg.stream_enabled     = true;    /* both paths in one thread */
    cfg.stream_tcp_enabled = true;
    cfg.stream_rate_hz     = 200;
    cfg.stream_tcp_port    = port;
    snprintf(cfg.stream_tcp_bind_addr, sizeof cfg.stream_tcp_bind_addr,
             "127.0.0.1");
    snprintf(cfg.stream_socket, sizeof cfg.stream_socket, TEST_SOCK);

    unlink(TEST_SOCK);
    out_ctx_t *out = NULL;
    EXPECT(out_ctx_open(&out, &cfg, (imu_ctx_t *)0x1) == 0,
           "out_ctx_open creates AF_UNIX + TCP listeners");

    pthread_t tid;
    EXPECT(pthread_create(&tid, NULL, stream_out_thread, out) == 0,
           "stream_out_thread starts");

    /* One subscriber on each transport; both must see valid frames. */
    int tcp_sub  = connect_tcp_subscriber(port);
    int unix_sub = connect_subscriber();
    EXPECT(tcp_sub  >= 0, "TCP subscriber connects");
    EXPECT(unix_sub >= 0, "AF_UNIX subscriber connects alongside");

    unsigned char frame[sizeof(imu_packet_t)];
    int tcp_ok = 0;
    for (int i = 0; i < 3; i++) {
        if (read_frame(tcp_sub, frame) != 0) break;
        uint32_t magic;   memcpy(&magic,   frame,     4);
        uint16_t version; memcpy(&version, frame + 4, 2);
        float heading;
        memcpy(&heading, frame + offsetof(imu_packet_t, heading_deg), 4);
        if (magic == IMUD_MAGIC && version == IMUD_VERSION &&
            fabsf(heading - 123.4f) < 1e-4f)
            tcp_ok++;
    }
    EXPECT(tcp_ok == 3, "three exact frames over TCP with correct "
                        "magic/version/heading");
    EXPECT(read_frame(unix_sub, frame) == 0,
           "AF_UNIX subscriber receives frames concurrently");

    /* Stop: the thread must send a final FLAG_SHUTDOWN frame to the TCP
     * subscriber before closing it (same contract as AF_UNIX). */
    out_ctx_stop(out);
    EXPECT(pthread_join(tid, NULL) == 0, "stream_out_thread joins cleanly");

    bool saw_shutdown = false;
    while (read_frame(tcp_sub, frame) == 0) {
        uint16_t flags;
        memcpy(&flags, frame + offsetof(imu_packet_t, flags), 2);
        if (flags & FLAG_SHUTDOWN) saw_shutdown = true;
    }
    EXPECT(saw_shutdown, "TCP subscriber got the final FLAG_SHUTDOWN frame");

    close(tcp_sub);
    close(unix_sub);
    out_ctx_free(out);
    EXPECT(connect_tcp_subscriber(port) < 0,
           "TCP listener closed after stop/free");

    puts(g_fail == fb ? "OK" : "FAIL");
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud stream socket tests ===");
    signal(SIGPIPE, SIG_IGN);   /* macOS: netserv sends lack MSG_NOSIGNAL */
    int fb = g_fail;
    printf("%-52s", "test_stream_end_to_end");
    fflush(stdout);

    imud_config_t cfg;
    config_defaults(&cfg);
    cfg.nmea_enabled     = false;
    cfg.highrate_enabled = false;
    cfg.stream_enabled   = true;
    cfg.stream_rate_hz   = 200;   /* fast so the test finishes quickly */
    snprintf(cfg.stream_socket, sizeof cfg.stream_socket, TEST_SOCK);

    unlink(TEST_SOCK);
    out_ctx_t *out = NULL;
    EXPECT(out_ctx_open(&out, &cfg, (imu_ctx_t *)0x1) == 0,
           "out_ctx_open creates the stream listener");

    pthread_t tid;
    EXPECT(pthread_create(&tid, NULL, stream_out_thread, out) == 0,
           "stream_out_thread starts");

    /* Subscribe and read three consecutive frames. */
    int sub = connect_subscriber();
    EXPECT(sub >= 0, "subscriber connects to the Unix socket");

    unsigned char frame[sizeof(imu_packet_t)];
    int frames_ok = 0;
    for (int i = 0; i < 3; i++) {
        if (read_frame(sub, frame) != 0) break;
        uint32_t magic;   memcpy(&magic,   frame,     4);
        uint16_t version; memcpy(&version, frame + 4, 2);
        float heading;
        memcpy(&heading, frame + offsetof(imu_packet_t, heading_deg), 4);
        if (magic == IMUD_MAGIC && version == IMUD_VERSION &&
            fabsf(heading - 123.4f) < 1e-4f)
            frames_ok++;
    }
    EXPECT(frames_ok == 3, "three exact wire-size frames with correct "
                           "magic/version/heading");

    /* Drop the subscriber; the thread must prune it and accept a new one. */
    close(sub);
    usleep(100 * 1000);
    int sub2 = connect_subscriber();
    EXPECT(sub2 >= 0, "second subscriber connects after first disconnects");
    EXPECT(read_frame(sub2, frame) == 0, "second subscriber receives a frame");
    close(sub2);

    /* Clean stop. */
    out_ctx_stop(out);
    EXPECT(pthread_join(tid, NULL) == 0, "stream_out_thread joins cleanly");
    out_ctx_free(out);
    EXPECT(access(TEST_SOCK, F_OK) != 0, "socket path unlinked on free");

    puts(g_fail == fb ? "OK" : "FAIL");

    test_socket_mode();
    test_stream_tcp();
    test_udp_outputs();
    test_reload_and_shutdown();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
