/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * fileio.h — create a file or an AF_UNIX socket with an explicit permission mode
 *
 * fopen(path, "w") creates the file 0666 and leaves the process umask to
 * narrow it.  main() sets umask(022) so the daemon's files land 0644 however
 * it was started, but the offline tools do not, and a permission mode that
 * depends on how a process was launched is not a mode anyone chose.  Every
 * file imud writes — PID file, cal.json, the .imucap black box, an imutest
 * report — is meant to be world-readable and owner-writable, so say so at the
 * point of creation instead of inferring it.
 *
 * bind(2) on an AF_UNIX socket is the same story with a sharper edge: the
 * inode it creates takes 0777 & ~umask, so a bind-then-chmod leaves the socket
 * briefly as wide as the umask allowed.  bind_unix_mode() closes that.
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
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
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

/*
 * Would fcreate(path, …) be permitted?  0 if so, -1 with errno set as access(2)
 * set it.  An existing file is asked about directly, a missing one through the
 * directory that would have to hold it.
 *
 * access() rather than a trial open, so asking neither creates the file nor
 * truncates one that is already there.  Advisory and racy by nature: it exists
 * so a tool can refuse before doing work it would have to throw away — an
 * hour's mag swing, a guided imutest run — and the real open stays the
 * authority.
 */
static inline int path_writable(const char *path)
{
    if (access(path, F_OK) == 0) return access(path, W_OK);

    const char *slash = strrchr(path, '/');
    if (!slash) return access(".", W_OK | X_OK);

    size_t len = (slash == path) ? 1 : (size_t)(slash - path);  /* "/x" → "/" */
    char   dir[PATH_MAX];
    if (len >= sizeof dir) { errno = ENAMETOOLONG; return -1; }
    memcpy(dir, path, len);
    dir[len] = '\0';
    return access(dir, W_OK | X_OK);
}

/* The umask that makes a newly created object land at exactly `perm`. */
static inline mode_t umask_for(mode_t perm)
{
    return (mode_t)(0777 & ~perm);
}

/*
 * bind() an AF_UNIX socket that is never wider than `perm`, at any instant.
 *
 * The bare idiom is bind() then chmod(), which is correct in the end but leaves
 * the socket at 0777 & ~umask in between — connectable by anyone if the umask
 * happens to be permissive.  Binding under umask_for(perm) means the socket is
 * born at `perm` instead.  The chmod stays because it is what still delivers
 * `perm` when the caller's umask was *narrower*: the umask can only take bits
 * away, never add them back.
 *
 * umask(2) is process-global and not thread-safe, so this must be called before
 * any thread is created.  Both callers are daemon startup, ahead of every
 * pthread_create().
 *
 * Returns 0, or -1 with errno set from whichever call failed.
 */
static inline int bind_unix_mode(int fd, const struct sockaddr *addr,
                                 socklen_t len, const char *path, mode_t perm)
{
    mode_t saved = umask(umask_for(perm));
    int    rc    = bind(fd, addr, len);
    int    err   = errno;
    umask(saved);

    if (rc < 0) { errno = err; return -1; }
    return chmod(path, perm);
}

#endif /* IMUD_FILEIO_H */
