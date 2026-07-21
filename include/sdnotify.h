/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * sdnotify.h — sd_notify(3)-style systemd readiness/watchdog datagrams
 *
 * One shared copy of the notify sender for the daemon and every bridge.
 * libc-only (no libimud dependency) so it links into any binary.
 */

#ifndef IMUD_SDNOTIFY_H
#define IMUD_SDNOTIFY_H

/* Send one sd_notify(3)-style datagram to systemd; no-op outside systemd. */
void sd_notify_msg(const char *msg);

#endif /* IMUD_SDNOTIFY_H */
