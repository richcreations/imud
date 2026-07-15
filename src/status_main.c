
/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * status_main.c — imud-status: connect to imud AF_UNIX status socket and print
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#define DEFAULT_STATUS_SOCK "/run/imud/imud.sock"

static void usage(const char *p)
{
    fprintf(stderr, "Usage: %s [--socket PATH]\n", p);
}

int main(int argc, char **argv)
{
    const char *sockpath = DEFAULT_STATUS_SOCK;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--socket") == 0 && i + 1 < argc) {
            sockpath = argv[++i];
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        fprintf(stderr, "socket: %s\n", strerror(errno));
        return 2;
    }

    struct sockaddr_un addr;
    size_t plen = strlen(sockpath);
    if (plen >= sizeof(addr.sun_path)) {
        fprintf(stderr, "socket path too long (%zu bytes, max %zu): %s\n",
                plen, sizeof(addr.sun_path) - 1, sockpath);
        close(fd);
        return 2;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, sockpath, plen);   /* addr is zeroed → NUL-terminated */

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "connect(%s): %s\n", sockpath, strerror(errno));
        close(fd);
        return 3;
    }

    char buf[4096];
    ssize_t n;
    while ((n = read(fd, buf, sizeof(buf))) > 0) {
        ssize_t w = write(STDOUT_FILENO, buf, (size_t)n);
        if (w < 0) break;
    }

    close(fd);
    return 0;
}
