/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * version.h — the single canonical release version for imud and all bridges
 *
 * Bump ONLY here for a release (see docs/RELEASING.md for the full checklist:
 * NEWS, packaging/<pkg>/changelog, man-page .TH strings, git tag). The wire
 * protocol version is separate: IMUD_VERSION in types.h / lib/imud_client.h.
 * The Makefile extracts VERSION from this file for `make dist` naming.
 */
#ifndef IMUD_VERSION_H
#define IMUD_VERSION_H

#define IMUD_VERSION_STR "1.3.1"

#endif /* IMUD_VERSION_H */
