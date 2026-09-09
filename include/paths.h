/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * paths.h — the install directories, as they were compiled in
 *
 * Every default path the tree names is derived here from five directories the
 * Makefile -D's from its own PREFIX/ETCDIR/DATADIR/RUNDIR/STATEDIR.  Each is
 * guarded, so the fallbacks below are what a compile outside the Makefile
 * gets, and they are the values a Linux build resolves to anyway — the .deb
 * builds PREFIX=/usr and comes out byte for byte identical.
 *
 * Every macro expands to a string LITERAL, so a caller concatenates rather
 * than formats: a usage string stays one literal and --help does not change
 * shape.  A directory carrying a '%' would still reach a printf format, so
 * snprintf() callers pass "%s".
 *
 * A prefix that is not /usr is what this is for: Homebrew, a FreeBSD port,
 * pkgsrc, make install PREFIX=$HOME/.local.  ETCDIR does not follow PREFIX by
 * default — a package installed into /usr still reads /etc/imud.
 */

#ifndef IMUD_PATHS_H
#define IMUD_PATHS_H

#ifndef IMUD_PREFIX
# define IMUD_PREFIX   "/usr/local"
#endif
#ifndef IMUD_ETCDIR
# define IMUD_ETCDIR   "/etc/imud"
#endif
#ifndef IMUD_DATADIR
# define IMUD_DATADIR  IMUD_PREFIX "/share"
#endif
#ifndef IMUD_RUNDIR
# define IMUD_RUNDIR   "/run/imud"
#endif
#ifndef IMUD_STATEDIR
# define IMUD_STATEDIR "/var/lib/imud"
#endif

/* Files and sockets, one place each. */
#define IMUD_SYS_CONF     IMUD_ETCDIR  "/imud.conf"
#define IMUD_CAL_FILE     IMUD_ETCDIR  "/cal.json"
#define IMUD_WMM_ETC      IMUD_ETCDIR  "/WMM.COF"
#define IMUD_WMM_DATA     IMUD_DATADIR "/imud/WMM.COF"
#define IMUD_PID_FILE     IMUD_RUNDIR  "/imud.pid"
#define IMUD_STATUS_SOCK  IMUD_RUNDIR  "/imud.sock"
#define IMUD_STREAM_SOCK  IMUD_RUNDIR  "/imud-stream.sock"

/* $(ETCDIR)/imud-<name>.conf — each bridge's own file, never imud.conf. */
#define IMUD_BRIDGE_CONF(name) IMUD_ETCDIR "/imud-" name ".conf"

#endif /* IMUD_PATHS_H */
