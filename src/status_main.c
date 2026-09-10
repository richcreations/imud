
/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * status_main.c — imud-status: connect to imud AF_UNIX status socket and print
 *
 * Sends a one-line request (text or JSON, see include/paths.h) and copies the
 * answer to stdout.
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "cli.h"
#include "cloexec.h"
#include "paths.h"

int main(int argc, char **argv)
{
    cli_status_t args;
    int cli_rc = cli_parse_status(argc, argv, &args);
    if (cli_rc != 0) return cli_rc < 0 ? 1 : 0;   /* -1 bad usage, 1 --help */
    const char *sockpath = args.sockpath;

    signal(SIGPIPE, SIG_IGN);

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        fprintf(stderr, "socket: %s\n", strerror(errno));
        return 2;
    }
    APPLY_CLOEXEC(fd);

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

    /* Ask for the report we want.  A daemon before 1.11 never reads this and
     * may already have answered and hung up, so the write can draw an EPIPE —
     * hence the SIG_IGN above; the answer is still in the receive buffer. */
    const char *req = args.want_json ? IMUD_STATUS_REQ_JSON
                                     : IMUD_STATUS_REQ_TEXT;
    ssize_t nreq = write(fd, req, strlen(req));
    (void)nreq;

    char    buf[4096];
    ssize_t n;
    bool    first = true;
    while ((n = read(fd, buf, sizeof(buf))) > 0) {
        /* An old daemon ignores the request and sends the text report.  Say so
         * rather than feeding prose to whatever was going to parse this. */
        if (first && args.want_json && buf[0] != '{') {
            fprintf(stderr, "%s: daemon does not support --json"
                            " (imud older than 1.11)\n", argv[0]);
            close(fd);
            return 4;
        }
        first = false;
        ssize_t w = write(STDOUT_FILENO, buf, (size_t)n);
        if (w < 0) break;
    }

    close(fd);

    if (first && args.want_json) {
        fprintf(stderr, "%s: daemon sent no report\n", argv[0]);
        return 4;
    }
    return 0;
}
