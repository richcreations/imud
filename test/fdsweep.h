/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fdsweep.h — assert that code under test leaks no non-close-on-exec fd
 *
 * The fds we care about live inside opaque contexts (pos_ctx_t's gpsd
 * socket, netserv_t's accepted clients), so a test cannot name them. Instead
 * snapshot which descriptors are open, run the code, and check every
 * descriptor that appeared carries FD_CLOEXEC:
 *
 *     fdsweep_t sw;
 *     fdsweep_begin(&sw);
 *     ... code under test ...
 *     EXPECT(fdsweep_leaks(&sw) == 0, "every new fd is close-on-exec");
 *
 * This covers the whole call path, including fds opened by a layer the test
 * never touches directly. It is why the accept() gap was found: POSIX does
 * not propagate FD_CLOEXEC across accept(), so a listener alone proves
 * nothing about its clients.
 *
 * The test's OWN helper sockets must therefore be close-on-exec too, or they
 * register as leaks. That is deliberate: it means the sweep cannot quietly
 * pass because the noise floor was raised.
 */

#ifndef IMUD_TEST_FDSWEEP_H
#define IMUD_TEST_FDSWEEP_H

#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>

#define FDSWEEP_MAX 256

typedef struct { bool open[FDSWEEP_MAX]; } fdsweep_t;

static inline void fdsweep_begin(fdsweep_t *s)
{
    for (int fd = 0; fd < FDSWEEP_MAX; fd++)
        s->open[fd] = fcntl(fd, F_GETFD) >= 0;
}

/* Number of descriptors opened since fdsweep_begin() that lack FD_CLOEXEC.
 * Each is named on stderr so a failure points at a specific fd. */
static inline int fdsweep_leaks(const fdsweep_t *s)
{
    int leaks = 0;
    for (int fd = 0; fd < FDSWEEP_MAX; fd++) {
        if (s->open[fd]) continue;          /* already open before the run */
        int flags = fcntl(fd, F_GETFD);
        if (flags < 0) continue;            /* still closed */
        if (!(flags & FD_CLOEXEC)) {
            fprintf(stderr, "  fd %d opened without FD_CLOEXEC\n", fd);
            leaks++;
        }
    }
    return leaks;
}

#endif /* IMUD_TEST_FDSWEEP_H */
