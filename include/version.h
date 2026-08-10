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

#define IMUD_VERSION_STR "1.9.0"

/*
 * Release date, ISO 8601, stamped into every generated man page's .TH line.
 *
 * help2man writes the current date in "August 2026" form, which is neither
 * this project's format nor reproducible — regenerating on a different day
 * would produce a diff, and the regenerate-and-diff CI gate would fail once a
 * month for no reason.  tools/man-postprocess.py overwrites the .TH using this
 * value, so the generated pages carry the same date as the hand-written ones
 * and change only when a release does.
 *
 * tools/bump-version.sh writes both macros.
 */
#define IMUD_RELEASE_DATE "2026-08-10"

#endif /* IMUD_VERSION_H */
