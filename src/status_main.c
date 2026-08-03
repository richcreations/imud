
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

#include "cli.h"

int main(int argc, char **argv)
{
    cli_status_t args;
    int cli_rc = cli_parse_status(argc, argv, &args);
    if (cli_rc != 0) return cli_rc < 0 ? 1 : 0;   /* -1 bad usage, 1 --help */
    const char *sockpath = args.sockpath;

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
