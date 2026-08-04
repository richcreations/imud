/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fakebroker.h — the smallest MQTT broker that imud-mqtt will talk to
 *
 * mqtt_main.c is the one bridge whose loop cannot be reached with a socket that
 * merely accepts: libmosquitto will not report a connection until it has sent
 * CONNECT and received CONNACK, so without a peer that speaks the protocol the
 * daemon never gets past its connect retry and the publish wiring stays as dark
 * as it was before the test existed.
 *
 * This is deliberately NOT an MQTT implementation. It answers CONNECT with
 * CONNACK-accepted, records the topic and payload of every PUBLISH at QoS 0,
 * and replies to PINGREQ. Anything else it skips by length. That is the whole
 * surface imud-mqtt uses (test_mqtt.c already pins the message *contents*; what
 * is untested is whether the loop ever hands them to the library).
 *
 * MQTT 3.1.1 fixed header: byte 0 is type<<4 | flags, then a remaining-length
 * varint of 1-4 bytes. For a QoS-0 PUBLISH the variable header is a 2-byte
 * big-endian topic length, the topic, and then the payload fills the rest.
 */

#ifndef IMUD_TEST_FAKEBROKER_H
#define IMUD_TEST_FAKEBROKER_H

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#define FB_MAX_MSGS   64
#define FB_TOPIC_MAX  192
#define FB_PAYLOAD_MAX 256

typedef struct {
    char topic[FB_TOPIC_MAX];
    char payload[FB_PAYLOAD_MAX];
} fb_msg_t;

typedef struct {
    int         port;
    int         listen_fd;
    pthread_t   tid;
    bool        running;

    atomic_int  stop;
    atomic_int  connects;      /* CONNECT packets answered */
    atomic_int  nmsg;          /* PUBLISHes recorded (may exceed FB_MAX_MSGS) */

    pthread_mutex_t lock;
    fb_msg_t    msg[FB_MAX_MSGS];
} fakebroker_t;

/* MQTT control packet types (high nibble of byte 0). */
#define FB_CONNECT  1
#define FB_CONNACK  2
#define FB_PUBLISH  3
#define FB_PINGREQ 12
#define FB_PINGRESP 13
#define FB_DISCONNECT 14

/* Read exactly n bytes, or fail. */
static inline bool fb_read_n(int fd, unsigned char *b, size_t n)
{
    size_t got = 0;
    while (got < n) {
        ssize_t r = recv(fd, b + got, n - got, 0);
        if (r == 0) return false;
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        got += (size_t)r;
    }
    return true;
}

/* MQTT remaining-length varint: 7 bits per byte, high bit = continue. */
static inline bool fb_read_remaining(int fd, size_t *out)
{
    size_t   value = 0;
    unsigned mult  = 1;
    for (int i = 0; i < 4; i++) {
        unsigned char b;
        if (!fb_read_n(fd, &b, 1)) return false;
        value += (size_t)(b & 0x7F) * mult;
        if (!(b & 0x80)) { *out = value; return true; }
        mult *= 128;
    }
    return false;
}

static inline void fb_record(fakebroker_t *fb, const unsigned char *body, size_t n)
{
    if (n < 2) return;
    size_t tlen = ((size_t)body[0] << 8) | body[1];
    if (tlen + 2 > n) return;

    int idx = atomic_fetch_add(&fb->nmsg, 1);
    if (idx >= FB_MAX_MSGS) return;              /* keep the first N, count all */

    pthread_mutex_lock(&fb->lock);
    size_t tcopy = tlen < FB_TOPIC_MAX - 1 ? tlen : FB_TOPIC_MAX - 1;
    memcpy(fb->msg[idx].topic, body + 2, tcopy);
    fb->msg[idx].topic[tcopy] = '\0';

    size_t plen  = n - 2 - tlen;
    size_t pcopy = plen < FB_PAYLOAD_MAX - 1 ? plen : FB_PAYLOAD_MAX - 1;
    memcpy(fb->msg[idx].payload, body + 2 + tlen, pcopy);
    fb->msg[idx].payload[pcopy] = '\0';
    pthread_mutex_unlock(&fb->lock);
}

static inline void *fb_thread(void *arg)
{
    fakebroker_t *fb = (fakebroker_t *)arg;

    while (!atomic_load(&fb->stop)) {
        int c = accept(fb->listen_fd, NULL, NULL);
        if (c < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                struct timespec t = { 0, 2 * 1000 * 1000 };
                nanosleep(&t, NULL);
                continue;
            }
            break;
        }

        /* Blocking reads on the client are fine: a timeout bounds them, so the
         * stop flag is still reachable if the peer goes quiet. */
        struct timeval tv = { 0, 200 * 1000 };
        setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

        while (!atomic_load(&fb->stop)) {
            unsigned char hdr;
            if (!fb_read_n(c, &hdr, 1)) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                break;
            }
            size_t rem = 0;
            if (!fb_read_remaining(c, &rem)) break;

            unsigned char body[2048];
            size_t want = rem < sizeof body ? rem : sizeof body;
            if (want && !fb_read_n(c, body, want)) break;
            for (size_t left = rem - want; left > 0; ) {   /* discard oversize */
                unsigned char sink[256];
                size_t chunk = left < sizeof sink ? left : sizeof sink;
                if (!fb_read_n(c, sink, chunk)) { left = 0; break; }
                left -= chunk;
            }

            switch (hdr >> 4) {
            case FB_CONNECT: {
                unsigned char ack[4] = { FB_CONNACK << 4, 2, 0, 0 };  /* accepted */
                if (send(c, ack, sizeof ack, 0) < 0) { /* peer gone */ }
                atomic_fetch_add(&fb->connects, 1);
                break;
            }
            case FB_PUBLISH:
                fb_record(fb, body, want);
                break;
            case FB_PINGREQ: {
                unsigned char pong[2] = { FB_PINGRESP << 4, 0 };
                if (send(c, pong, sizeof pong, 0) < 0) { /* peer gone */ }
                break;
            }
            default:
                break;                                   /* SUBSCRIBE etc. */
            }
        }
        close(c);
    }
    return NULL;
}

static inline int fb_start(fakebroker_t *fb)
{
    memset(fb, 0, sizeof *fb);
    atomic_init(&fb->stop, 0);
    atomic_init(&fb->connects, 0);
    atomic_init(&fb->nmsg, 0);
    pthread_mutex_init(&fb->lock, NULL);

    fb->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fb->listen_fd < 0) return -1;
    int on = 1;
    setsockopt(fb->listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);

    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = 0;                       /* kernel picks: no collisions */
    if (bind(fb->listen_fd, (struct sockaddr *)&a, sizeof a) < 0 ||
        listen(fb->listen_fd, 4) < 0) {
        close(fb->listen_fd);
        return -1;
    }
    socklen_t al = sizeof a;
    getsockname(fb->listen_fd, (struct sockaddr *)&a, &al);
    fb->port = ntohs(a.sin_port);

    int fl = fcntl(fb->listen_fd, F_GETFL, 0);
    fcntl(fb->listen_fd, F_SETFL, fl | O_NONBLOCK);

    if (pthread_create(&fb->tid, NULL, fb_thread, fb) != 0) {
        close(fb->listen_fd);
        return -1;
    }
    fb->running = true;
    return 0;
}

static inline void fb_stop(fakebroker_t *fb)
{
    if (!fb->running) return;
    atomic_store(&fb->stop, 1);
    pthread_join(fb->tid, NULL);
    close(fb->listen_fd);
    pthread_mutex_destroy(&fb->lock);
    fb->running = false;
}

/* True once a topic containing `needle` has been published. */
static inline bool fb_wait_topic(fakebroker_t *fb, const char *needle, int timeout_ms)
{
    for (int waited = 0; waited < timeout_ms; waited += 5) {
        int n = atomic_load(&fb->nmsg);
        if (n > FB_MAX_MSGS) n = FB_MAX_MSGS;
        pthread_mutex_lock(&fb->lock);
        for (int i = 0; i < n; i++) {
            if (strstr(fb->msg[i].topic, needle)) {
                pthread_mutex_unlock(&fb->lock);
                return true;
            }
        }
        pthread_mutex_unlock(&fb->lock);
        struct timespec t = { 0, 5 * 1000 * 1000 };
        nanosleep(&t, NULL);
    }
    return false;
}

#endif /* IMUD_TEST_FAKEBROKER_H */
