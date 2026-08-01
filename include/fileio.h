/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fileio.h — create a file with an explicit permission mode
 *
 * fopen(path, "w") creates the file 0666 and leaves the process umask to
 * narrow it.  main() sets umask(022) so the daemon's files land 0644 however
 * it was started, but the offline tools do not, and a permission mode that
 * depends on how a process was launched is not a mode anyone chose.  Every
 * file imud writes — PID file, cal.json, the .imucap black box, an imutest
 * report — is meant to be world-readable and owner-writable, so say so at the
 * point of creation instead of inferring it.
 *
 * Header-only and static inline on purpose: the callers sit in translation
 * units with four different link sets (daemon, imud-cal, imud-imutest, the
 * capture tests), and this way none of them needs a Makefile change.  Same
 * reasoning as src/drivers/i2c_io.h.
 */
#ifndef IMUD_FILEIO_H
#define IMUD_FILEIO_H

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* Owner writes, everyone reads: the mode every file imud creates wants. */
#define IMUD_FILE_MODE 0644

/*
 * Truncate-or-create `path` with mode `perm` and wrap it in a FILE*.
 * `stdio_mode` is the fopen() mode string the caller would have used ("w",
 * "wb"); the underlying flags are always write-only/create/truncate.
 * Returns NULL with errno set, exactly like fopen().
 */
static inline FILE *fcreate(const char *path, const char *stdio_mode,
                            mode_t perm)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, perm);
    if (fd < 0) return NULL;

    FILE *f = fdopen(fd, stdio_mode);
    if (!f) {
        int saved = errno;
        close(fd);
        errno = saved;
    }
    return f;
}

#endif /* IMUD_FILEIO_H */
