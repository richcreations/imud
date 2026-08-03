/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * sdnotify.c — sd_notify(3)-style systemd notifications
 *
 * Shared by the daemon (src/main.c) and the bridge daemons; previously each
 * carried its own copy of this function.
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "sdnotify.h"
#include "cloexec.h"

#ifndef MSG_NOSIGNAL
# define MSG_NOSIGNAL 0    /* macOS: callers ignore SIGPIPE instead */
#endif

void sd_notify_msg(const char *msg)
{
    const char *sock = getenv("NOTIFY_SOCKET");
    if (!sock) return;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (sock[0] == '@') {               /* abstract socket */
        addr.sun_path[0] = '\0';
        strncpy(addr.sun_path + 1, sock + 1, sizeof(addr.sun_path) - 2);
    } else {
        strncpy(addr.sun_path, sock, sizeof(addr.sun_path) - 1);
    }

    int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return;
    sendto(fd, msg, strlen(msg), MSG_NOSIGNAL,
           (struct sockaddr *)&addr, sizeof(addr));
    close(fd);
}
