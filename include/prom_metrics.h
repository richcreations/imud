/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * prom_metrics.h — Prometheus text-exposition builder for imud-prometheus
 *
 * Pure function over libimud's ABI-stable imud_data_t view (NOT the wire
 * packet — no bridge pins the wire format). Emits
 * gauges in Prometheus base units: radians, metres, seconds, celsius;
 * heading is the one conventional exception (degrees, and named so).
 */
#ifndef IMUD_PROM_METRICS_H
#define IMUD_PROM_METRICS_H

#include <stddef.h>
#include <stdint.h>
#include "../lib/imud.h"

/*
 * Render the metrics page into buf. d may be NULL (no packet received yet):
 * only imud_up (0) and imud_packets_total are emitted then.
 * Returns the byte length, or -1 if buf is too small.
 */
int prom_build_metrics(char *buf, size_t sz, const imud_data_t *d,
                       uint64_t packets_total);

#endif /* IMUD_PROM_METRICS_H */
