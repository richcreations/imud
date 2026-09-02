/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * cloexec.h — one close-on-exec idiom for the whole tree
 *
 * Linux takes SOCK_CLOEXEC in socket()/accept4() atomically; macOS has
 * neither, so the fallback sets FD_CLOEXEC with fcntl() right after the
 * fd exists. Both spellings appear at every call site:
 *
 *     int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
 *     if (fd < 0) return -1;
 *     APPLY_CLOEXEC(fd);
 *
 * On Linux APPLY_CLOEXEC is ((void)0) — not (0) — so a call site is never a
 * bare no-effect statement under -Wunused-value.
 *
 * APPLY_CLOEXEC is therefore ONLY valid on an fd that was just created by a
 * socket() call carrying SOCK_CLOEXEC — it finishes a job Linux already did.
 * For an fd of any other provenance (accept, socketpair, dup, or one handed
 * in by a caller) it does nothing at all on Linux, and the code silently
 * ships without close-on-exec. Those need an unconditional
 * fcntl(fd, F_SETFD, FD_CLOEXEC). This is not hypothetical: prom_conn_adopt()
 * used APPLY_CLOEXEC on an accepted fd, passed its own assertion on macOS,
 * and failed it on every Linux job in CI.
 *
 * open() needs none of this: O_CLOEXEC is in POSIX.1-2008 and present on
 * both platforms, so those sites just pass the flag.
 *
 * accept() is the trap. POSIX says the accepted fd does NOT inherit
 * FD_CLOEXEC from the listener, so `accept(); APPLY_CLOEXEC(c);` is correct
 * on macOS and a silent no-op on Linux — backwards from what it reads like,
 * and wrong on the platform imud actually ships on. ACCEPT_CLOEXEC() uses
 * Linux's accept4() and falls back to accept()+fcntl everywhere else.
 *
 * There is no fork/exec anywhere in imud, so nothing leaks today; this is
 * consistency and defense in depth for consumers who do exec.
 *
 * lib/libimud.c and lib/imud_client.h deliberately carry their own copies:
 * the client library must compile standalone against nothing but libc.
 */

#ifndef IMUD_CLOEXEC_H
#define IMUD_CLOEXEC_H

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>

#ifndef SOCK_CLOEXEC
# define SOCK_CLOEXEC 0
# define APPLY_CLOEXEC(fd) fcntl((fd), F_SETFD, FD_CLOEXEC)
#else
# define APPLY_CLOEXEC(fd) ((void)0)
#endif

/* accept() returning a close-on-exec fd. Evaluates lfd once; returns the new
 * fd, or -1 with errno set exactly as accept() would. */
#ifdef __linux__
# define ACCEPT_CLOEXEC(lfd) accept4((lfd), NULL, NULL, SOCK_CLOEXEC)
#else
static inline int imud_accept_cloexec(int lfd)
{
    int c = accept(lfd, NULL, NULL);
    if (c >= 0 && fcntl(c, F_SETFD, FD_CLOEXEC) < 0) {
        int e = errno; close(c); errno = e; return -1;
    }
    return c;
}
# define ACCEPT_CLOEXEC(lfd) imud_accept_cloexec(lfd)
#endif

/* There is deliberately no SOCK_NONBLOCK fallback here. A `#define
 * SOCK_NONBLOCK 0` compiles everywhere and silently yields a BLOCKING socket
 * off Linux, which is worse than not building: src/main.c's status listener
 * had one, and a blocking listener hangs the health thread on any client that
 * disconnects between its select() and its accept(). Use an explicit
 * fcntl(fd, F_SETFL, O_NONBLOCK) after socket(), as status_sock_open() does. */

#endif /* IMUD_CLOEXEC_H */
