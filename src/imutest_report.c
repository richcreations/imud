/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * imutest_report.c — formatting for imud-imutest.
 *
 * imt_print() writes the short terminal digest; imt_write_md() writes the
 * self-contained Markdown report meant to be pasted into an issue.  Both read
 * a finished imt_report_t and compute nothing, so the numbers in the report
 * are exactly the numbers the checks were graded on.
 *
 * Every threshold is printed inline: a reviewer should never need the source
 * to know what a check was asserting.
 */

#include <stdio.h>
#include <string.h>

#include "fileio.h"
#include "imutest.h"
#include "imu_math.h"

/* ── Markdown escaping ─────────────────────────────────────────────────────── */

/*
 * Driver-authored strings reach the note field, so every cell goes through
 * this: an unescaped '|' would silently break the table, and a newline would
 * end the row.
 */
static const char *md_cell(char *buf, size_t sz, const char *s)
{
    size_t j = 0;
    if (!s) s = "";
    for (size_t i = 0; s[i] && j + 2 < sz; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '|') { buf[j++] = '\\'; buf[j++] = '|'; }
        else if (c == '\n' || c == '\r' || c == '\t') buf[j++] = ' ';
        else buf[j++] = (char)c;
    }
    buf[j] = '\0';
    return buf;
}

static const char *phase_of(const char *id)
{
    if (strncmp(id, "face.", 5) == 0 || strncmp(id, "faces.", 6) == 0)
        return "B";
    if (strncmp(id, "gyro.", 5) == 0) return "C";
    if (strncmp(id, "spin.", 5) == 0) return "D";
    return "A";
}

/* ── Terminal digest ───────────────────────────────────────────────────────── */

void imt_print(const imt_report_t *r, FILE *out)
{
    fprintf(out, "\n");
    fprintf(out, "imud-imutest %s — %s", r->imud_version, r->imu_driver);
    if (r->have_mag) fprintf(out, " + %s", r->mag_driver);
    fprintf(out, "\n");
    fprintf(out, "  %d PASS · %d WARN · %d FAIL · %d SKIP · %d INFO"
                 "   (%.0f s)\n",
            r->n_pass, r->n_warn, r->n_fail, r->n_skip, r->n_info,
            r->wall_duration_s);

    if (r->is_sim)
        fprintf(out, "\n  NOTE: the `sim` driver exercises this tool, not any "
                     "hardware.\n");
    if (r->daemon_was_running)
        fprintf(out, "\n  WARNING: imud was running — both processes drain the "
                     "same FIFO,\n           so the timing figures below are "
                     "not trustworthy.\n");

    /* Only the interesting rows: everything else lives in the report file. */
    bool header = false;
    for (int i = 0; i < r->n_checks; i++) {
        const imt_check_t *c = &r->check[i];
        if (c->status != IMT_FAIL && c->status != IMT_WARN) continue;
        if (!header) { fprintf(out, "\n"); header = true; }
        fprintf(out, "  %-4s %-20s %s\n",
                imt_status_str(c->status), c->id, c->name);
        if (c->measured[0])
            fprintf(out, "         measured %s, expected %s\n",
                    c->measured, c->expected);
        if (c->note[0])
            fprintf(out, "         %s\n", c->note);
    }

    fprintf(out, "\n  %s\n", r->verdict);
}

/* ── Markdown report ───────────────────────────────────────────────────────── */

static void write_int_table(FILE *f, const int *t, int cap)
{
    int n = 0;
    while (n < cap && t[n] != 0) n++;
    if (n == 0) { fputs("(empty)", f); return; }
    for (int i = 0; i < n; i++) fprintf(f, "%s%d", i ? ", " : "", t[i]);
}

static void write_checks_for_phase(FILE *f, const imt_report_t *r,
                                   const char *phase)
{
    char id[32], name[96], meas[80], exp[80], note[320];
    /* FAIL first, then WARN, then the rest: the reader wants the defects. */
    const imt_status_t order[] = { IMT_FAIL, IMT_WARN, IMT_PASS, IMT_INFO, IMT_SKIP };
    for (size_t k = 0; k < sizeof order / sizeof order[0]; k++) {
        for (int i = 0; i < r->n_checks; i++) {
            const imt_check_t *ck = &r->check[i];
            if (ck->status != order[k]) continue;
            if (strcmp(phase_of(ck->id), phase) != 0) continue;
            fprintf(f, "| `%s` | %s | %s | %s | %s | %s |\n",
                    md_cell(id,   sizeof id,   ck->id),
                    md_cell(name, sizeof name, ck->name),
                    imt_status_str(ck->status),
                    md_cell(meas, sizeof meas, ck->measured[0] ? ck->measured : "-"),
                    md_cell(exp,  sizeof exp,  ck->expected[0] ? ck->expected : "-"),
                    md_cell(note, sizeof note, ck->note));
        }
    }
}

int imt_write_md(const imt_report_t *r, const char *path,
                 char *errbuf, size_t errbufsz)
{
    FILE *f = fcreate(path, "w", IMUD_FILE_MODE);
    if (!f) {
        snprintf(errbuf, errbufsz, "cannot write %s", path);
        return -1;
    }

    char esc[256];

    /* ── Title and headline ──────────────────────────────────────────────── */
    fprintf(f, "# imud driver validation — `%s`", r->imu_driver);
    if (r->have_mag) fprintf(f, " + `%s`", r->mag_driver);
    fprintf(f, "\n\n");
    fprintf(f, "<!-- generated by imud-imutest %s; paste this whole file into "
               "a GitHub issue -->\n\n", r->imud_version);

    fprintf(f, "**%d PASS · %d WARN · %d FAIL · %d SKIP**\n\n",
            r->n_pass, r->n_warn, r->n_fail, r->n_skip);
    fprintf(f, "> %s\n\n", md_cell(esc, sizeof esc, r->verdict));

    if (r->is_sim)
        fprintf(f, "> **This run used the `sim` driver.** It exercises "
                   "imud-imutest itself, not any hardware.\n\n");
    if (r->daemon_was_running)
        fprintf(f, "> **imud was running during this run.** Both processes "
                   "drain the same FIFO, so each saw roughly half the samples: "
                   "the ODR and `seq` figures below understate reality. Re-run "
                   "with the daemon stopped before drawing conclusions.\n\n");

    /* ── 1. Environment ──────────────────────────────────────────────────── */
    fprintf(f, "## 1. Environment\n\n");
    fprintf(f, "| | |\n|---|---|\n");
    fprintf(f, "| imud version | %s |\n", r->imud_version);
    fprintf(f, "| host / kernel | %s / %s %s %s |\n",
            r->hostname, r->sysname, r->release, r->machine);
    fprintf(f, "| libgpiod | %s |\n", r->gpiod ? r->gpiod : "v1");
    fprintf(f, "| started (UTC) | %s |\n", r->started_utc);
    fprintf(f, "| run duration | %.1f s |\n", r->wall_duration_s);
    fprintf(f, "| config file | `%s` |\n",
            r->config_path[0] ? r->config_path : "(defaults)");
    /* Per sensor: a board may run one part on SPI and the other on I2C, and
     * which transport a driver was validated on is a fact a reviewer clearing
     * `experimental` has to be able to read off this report. */
    fprintf(f, "| IMU bus | %s `%s` |\n",
            r->imu_bus_spi ? "SPI" : "I2C", r->imu_bus);
    if (r->have_mag)
        fprintf(f, "| magnetometer bus | %s `%s` |\n",
                r->mag_bus_spi ? "SPI" : "I2C", r->mag_bus);
    fprintf(f, "| GPIO chip | `%s` |\n", r->gpio_chip);
    fprintf(f, "| imud.service | %s |\n",
            r->daemon_was_running ? "**running** (see warning above)"
                                  : "not running");
    fprintf(f, "| phases run | %s%s%s%s |\n",
            (r->phases_run & IMT_PHASE_PASSIVE) ? "passive " : "",
            (r->phases_run & IMT_PHASE_FACES)   ? "faces "   : "",
            (r->phases_run & IMT_PHASE_GYRO)    ? "gyro "    : "",
            (r->phases_run & IMT_PHASE_SPIN)    ? "spin"     : "");
    fprintf(f, "\n");

    /* ── 2. Device under test ────────────────────────────────────────────── */
    fprintf(f, "## 2. Device under test\n\n");
    fprintf(f, "### IMU — `%s` @ 0x%02X (experimental: **%s**)\n\n",
            r->imu_driver, r->imu_addr, r->imu_experimental ? "yes" : "no");
    fprintf(f, "| field | value |\n|---|---|\n");
    fprintf(f, "| int_gpio | %d%s |\n", r->imu_int_gpio,
            r->imu_int_gpio > 0 ? "" : " (polling)");
    {
        char qb[16], eb2[16];
        fprintf(f, "| ODR requested -> effective | %s Hz -> %s Hz |\n",
                MHZ_STR(qb, r->req_odr_mhz), MHZ_STR(eb2, r->eff_odr_mhz));
    }
    fprintf(f, "| accel FS / gyro FS / fifo_wm | %d g / %d dps / %d sample-sets |\n",
            r->accel_g, r->gyro_dps, r->fifo_wm);
    fprintf(f, "| has_fifo / has_hw_timestamp | %s / %s",
            r->imu_has_fifo ? "true" : "false",
            r->imu_has_hw_ts ? "true" : "false");
    if (r->imu_has_hw_ts) {
        /* Print both only when the part disagreed with its datasheet typical:
         * that difference is the whole reason ts_tick_ns_actual exists, and
         * burying it would make a 4%-fast part look like a plain one. */
        if (r->imu_ts_tick_actual_ns &&
            r->imu_ts_tick_actual_ns != r->imu_ts_tick_ns)
            fprintf(f, " (ts_tick_ns = %u typical, %u from this part)",
                    r->imu_ts_tick_ns, r->imu_ts_tick_actual_ns);
        else
            fprintf(f, " (ts_tick_ns = %u)", r->imu_ts_tick_ns);
    }
    fprintf(f, " |\n");
    fprintf(f, "| supported_odr_hz | ");   write_int_table(f, r->imu_odr_tab, 16);
    fprintf(f, " |\n| supported_accel_g | "); write_int_table(f, r->imu_accel_tab, 8);
    fprintf(f, " |\n| supported_gyro_dps | "); write_int_table(f, r->imu_gyro_tab, 8);
    fprintf(f, " |\n\n");

    if (r->have_mag) {
        fprintf(f, "### Magnetometer — `%s` @ 0x%02X (experimental: **%s**)\n\n",
                r->mag_driver, r->mag_addr, r->mag_experimental ? "yes" : "no");
        fprintf(f, "| field | value |\n|---|---|\n");
        fprintf(f, "| int_gpio | %d%s |\n", r->mag_int_gpio,
                r->mag_has_interrupt ? "" : " (driver has no external INT pin)");
        {
            char qb[16], eb2[16];
            fprintf(f, "| ODR requested -> effective | %s Hz -> %s Hz |\n",
                    MHZ_STR(qb, r->mag_req_odr_mhz), MHZ_STR(eb2, r->mag_eff_odr_mhz));
        }
        fprintf(f, "| has_interrupt / has_set_reset | %s / %s |\n",
                r->mag_has_interrupt ? "true" : "false",
                r->mag_has_set_reset ? "true" : "false");
        fprintf(f, "| set_reset pointer | %s |\n",
                r->mag_set_reset_nonnull ? "non-NULL" : "NULL");
        fprintf(f, "| supported_odr_hz | "); write_int_table(f, r->mag_odr_tab, 16);
        fprintf(f, " |\n\n");
    } else {
        fprintf(f, "### Magnetometer\n\nNone configured for this run.\n\n");
    }

    /* ── 3. Results ──────────────────────────────────────────────────────── */
    fprintf(f, "## 3. Results\n\n");
    static const struct { const char *key, *title; } phases[] = {
        { "A", "Phase A — passive (no operator action)" },
        { "B", "Phase B — guided six-face orientation" },
        { "C", "Phase C — guided gyro rotation" },
        { "D", "Phase D — guided magnetometer spin" },
    };
    for (size_t p = 0; p < sizeof phases / sizeof phases[0]; p++) {
        int count = 0;
        for (int i = 0; i < r->n_checks; i++)
            if (strcmp(phase_of(r->check[i].id), phases[p].key) == 0) count++;
        if (!count) continue;
        fprintf(f, "### %s\n\n", phases[p].title);
        fprintf(f, "| Check | Name | Status | Measured | Expected | Note |\n");
        fprintf(f, "|---|---|---|---|---|---|\n");
        write_checks_for_phase(f, r, phases[p].key);
        fprintf(f, "\n");
    }

    /* ── 4. What these results do and do not prove ───────────────────────── */
    fprintf(f, "## 4. Scope of this run\n\n");
    fprintf(f, "- **The control-register diff is raw and undecoded, by "
               "design.** Drivers are opaque to this tool, so it snapshots the "
               "safe register range before and after `init()` and prints what "
               "changed. Deciding whether those values are *correct* needs the "
               "datasheet and is the reviewer's job.\n");
    fprintf(f, "- **`imu.err.no_spurious` only tests one direction here.** "
               "This run proves the driver does not return -1 on a healthy "
               "bus. That -1 is returned *only* on a genuine I2C fault is "
               "checked off-hardware by `test_imutest`, which injects ioctl "
               "failures through the mock bus.\n");
    fprintf(f, "- **Volatile registers are found by experiment, and the filter "
               "is not perfect.** Sensor output, status, FIFO level and the "
               "timestamp counter are excluded from the diff and from the "
               "idempotency compare by reading the mapped range several times "
               "with the part running and no writes in between. A register "
               "that happens to hold the same value through every pass — a "
               "stationary accelerometer's high byte is the usual case — is "
               "not caught and still appears above.\n");
    fprintf(f, "- **Absolute gyro scale is checked only in phase C**, and only "
               "at the configured full scale. `imu.fs.gyro` records the noise "
               "floor at every range but does not grade it, because sigma only "
               "tracks full scale when quantisation dominates the noise floor "
               "and on a good part it does not.\n");
    if (!(r->phases_run & (IMT_PHASE_FACES | IMT_PHASE_GYRO | IMT_PHASE_SPIN)))
        fprintf(f, "- **No guided phase ran**, so nothing here says whether "
                   "the chip-to-board axis remap is correct — the most likely "
                   "defect in a new driver.\n");
    fprintf(f, "\n");

    /* ── 5. Appendix ─────────────────────────────────────────────────────── */
    fprintf(f, "## 5. Appendix — raw measurements\n\n");

    const imt_raw_t *w = &r->raw;

    fprintf(f, "### 5.1 Timing\n\n");
    fprintf(f, "```\n");
    fprintf(f, "reset()            %.2f ms\n", w->reset_ms);
    fprintf(f, "measured ODR       %.2f Hz over %.3f s (%llu samples)\n",
            w->odr_measured_hz, w->odr_window_s, (unsigned long long)w->odr_n);
    {
        char nb[16];
        fprintf(f, "nearest table entry %s Hz\n",
                MHZ_STR(nb, w->odr_best_table_mhz));
    }
    fprintf(f, "max read-loop gap  %.1f ms\n", w->odr_max_loop_gap_ms);
    fprintf(f, "seq                first %u, last %u, %llu gaps (max %llu), "
               "%llu reversals\n",
            w->seq_first, w->seq_last, (unsigned long long)w->seq_gaps,
            (unsigned long long)w->seq_max_gap,
            (unsigned long long)w->seq_backwards);
    fprintf(f, "read() rc          %d overflow, %d error (last errno %d)\n",
            w->rc1_count, w->rcneg_count, w->last_errno);
    if (r->imu_has_hw_ts)
        /* Zero-stamped samples belong here rather than only inside a check
         * note: they are excluded from the delta accounting, so without the
         * count there is no way to tell a clean window from one where the
         * driver's timestamp read kept failing. */
        fprintf(f, "chip_ts            median %.2f ticks/sample, implied tick "
                   "%.0f ns, wall ratio %.4f, %d wraps, %d reversals, "
                   "%d repeats, %d zero-stamped\n",
                w->ts_median_delta, w->ts_implied_tick_ns, w->ts_wall_ratio,
                w->ts_wraps, w->ts_backwards, w->ts_repeats,
                w->ts_zero_count);
    /*
     * Both DRDY counts, as counts.  The pair is the measurement, not the
     * verdict: an interrupt that only fires while something drains the FIFO
     * is a level condition, and only the second number can say so.  Worth
     * spelling out in the appendix rather than leaving inside a check note,
     * since this is the number that characterises the interrupt behaviour.
     */
    if (w->gpio_why == IMT_GPIO_OK && w->gpio_edges >= 0) {
        char idle[64];
        if (w->gpio_idle_valid)
            snprintf(idle, sizeof idle, "%d undrained (%.1f Hz)",
                     w->gpio_edges_idle, w->gpio_rate_idle_hz);
        else
            snprintf(idle, sizeof idle, "undrained pass did not run");
        fprintf(f, "DRDY edges         %d draining (%.1f Hz), %s, "
                   "%.1f s window\n",
                w->gpio_edges, w->gpio_rate_hz, idle, w->gpio_window_s);
    }
    fprintf(f, "```\n\n");

    fprintf(f, "### 5.2 Noise and gravity at rest\n\n");
    fprintf(f, "```\n");
    fprintf(f, "                        X            Y            Z\n");
    fprintf(f, "accel mean   %12.5f %12.5f %12.5f  m/s^2\n",
            w->accel.mean[0], w->accel.mean[1], w->accel.mean[2]);
    fprintf(f, "accel sigma  %12.5g %12.5g %12.5g  m/s^2\n",
            w->accel.sigma[0], w->accel.sigma[1], w->accel.sigma[2]);
    fprintf(f, "gyro  mean   %12.5g %12.5g %12.5g  rad/s\n",
            w->gyro.mean[0], w->gyro.mean[1], w->gyro.mean[2]);
    fprintf(f, "gyro  sigma  %12.5g %12.5g %12.5g  rad/s\n",
            w->gyro.sigma[0], w->gyro.sigma[1], w->gyro.sigma[2]);
    fprintf(f, "|a| mean %.5f m/s^2 (sigma %.5f), N = %llu\n",
            w->grav_mean, w->grav_sigma, (unsigned long long)w->accel.n);
    fprintf(f, "temperature %.2f C (range %.2f .. %.2f, %d distinct values)\n",
            w->temp_mean, w->temp_min, w->temp_max, w->temp_distinct);
    fprintf(f, "```\n\n");

    /*
     * The two sides are in different units on purpose -- raw counts against
     * m/s^2 -- so the table prints both and grades only the angle between
     * them. Anyone reading a decode fault wants to see WHICH axis each side
     * put gravity on, which the angle alone does not say.
     */
    if (w->direct_n > 0) {
        fprintf(f, "### 5.2a Direct registers against the FIFO\n\n");
        fprintf(f, "```\n");
        fprintf(f, "                        X            Y            Z\n");
        fprintf(f, "direct accel %12.1f %12.1f %12.1f  raw counts\n",
                w->direct_accel[0], w->direct_accel[1], w->direct_accel[2]);
        fprintf(f, "FIFO   accel %12.5f %12.5f %12.5f  m/s^2\n",
                w->fifo_accel[0], w->fifo_accel[1], w->fifo_accel[2]);
        fprintf(f, "angle between them %.2f deg over %d reads\n",
                w->direct_angle_deg, w->direct_n);
        if (w->direct_temp_c != 0.0 || w->fifo_temp_c != 0.0)
            fprintf(f, "temperature  direct %.2f C, FIFO %.2f C\n",
                    w->direct_temp_c, w->fifo_temp_c);
        if (w->direct_gyro_peak[0] != 0.0 || w->direct_gyro_peak[1] != 0.0 ||
            w->direct_gyro_peak[2] != 0.0)
            fprintf(f, "direct gyro peak during phase C "
                       "%.0f / %.0f / %.0f raw counts\n",
                    w->direct_gyro_peak[0], w->direct_gyro_peak[1],
                    w->direct_gyro_peak[2]);
        fprintf(f, "```\n\n");
    }

    if (w->fifo_steps > 0) {
        fprintf(f, "### 5.3 FIFO depth against wait\n\n");
        fprintf(f, "| wait (s) | samples returned |\n|---|---|\n");
        for (int i = 0; i < w->fifo_steps; i++)
            fprintf(f, "| %.3f | %d |\n", w->fifo_wait_s[i], w->fifo_depth[i]);
        if (w->overflow_after_s > 0)
            fprintf(f, "\nOverflow appeared after %.2f s without draining.\n",
                    w->overflow_after_s);
        fprintf(f, "\n");
    }

    if (w->n_fs_accel > 0) {
        fprintf(f, "### 5.4 Full-scale sweep\n\n");
        fprintf(f, "| accel FS | mean \\|a\\| (m/s^2) | ratio to g | verdict |\n");
        fprintf(f, "|---|---|---|---|\n");
        for (int i = 0; i < w->n_fs_accel; i++)
            fprintf(f, "| +/-%d g | %.4f | %.3f | %s |\n",
                    w->fs_accel[i].fs, w->fs_accel[i].grav_mean,
                    w->fs_accel[i].ratio, imt_status_str(w->fs_accel[i].status));
        if (w->n_fs_gyro > 0) {
            fprintf(f, "\n| gyro FS | mean sigma (rad/s) | sigma ratio to previous |\n");
            fprintf(f, "|---|---|---|\n");
            for (int i = 0; i < w->n_fs_gyro; i++) {
                char rb[16];
                if (i == 0) snprintf(rb, sizeof rb, "-");
                else        snprintf(rb, sizeof rb, "%.2f", w->fs_gyro[i].ratio);
                fprintf(f, "| +/-%d dps | %.5g | %s |\n",
                        w->fs_gyro[i].fs,
                        (w->fs_gyro[i].sigma[0] + w->fs_gyro[i].sigma[1] +
                         w->fs_gyro[i].sigma[2]) / 3.0, rb);
            }
        }
        fprintf(f, "\n");
    }

    /*
     * The chip_ts reversals themselves.  imu.chipts.monotonic reported "1
     * reversal at a burst seam" run after run and nobody could act on it; how
     * far time went backwards is what separates a drain-cadence artefact from
     * a decode defect.
     */
    if (w->n_bus_bad > 0) {
        fprintf(f, "### 5.3a Corrupted register reads\n\n");
        fprintf(f, "Expected `0x%02X` every time. What came back instead:\n\n",
                w->bus_ref_imu);
        fprintf(f, "| read # | got |\n|---|---|\n");
        for (int i = 0; i < w->n_bus_bad; i++)
            fprintf(f, "| %d | 0x%02X |\n", w->bus_bad_at[i], w->bus_bad_val[i]);
        fprintf(f, "\n");
    }

    if (w->n_ts_rev > 0) {
        fprintf(f, "### 5.4a chip_ts reversals\n\n");
        fprintf(f, "| sample | previous | current | ticks back | at a seam |\n");
        fprintf(f, "|---|---|---|---|---|\n");
        for (int i = 0; i < w->n_ts_rev; i++) {
            int64_t back = (int64_t)w->ts_rev[i].prev - (int64_t)w->ts_rev[i].cur;
            fprintf(f, "| %d | %u | %u | %lld | %s |\n",
                    w->ts_rev[i].idx, w->ts_rev[i].prev, w->ts_rev[i].cur,
                    (long long)back, w->ts_rev[i].seam ? "yes" : "no");
        }
        fprintf(f, "\n");
    }

    if (w->regdiff_imu_mapped) {
        fprintf(f, "### 5.5 Control-register diff, IMU 0x%02X\n\n", r->imu_addr);
        fprintf(f, "%d register%s excluded as volatile (they changed with no "
                   "write between reads); %d compared.\n\n",
                w->n_volatile_imu, w->n_volatile_imu == 1 ? "" : "s",
                w->n_scanned_imu);
        if (w->n_regdiff_imu == 0) {
            fprintf(f, "No registers changed across `init()`.\n\n");
        } else {
            fprintf(f, "| reg | after reset | after init |\n|---|---|---|\n");
            for (int i = 0; i < w->n_regdiff_imu; i++)
                fprintf(f, "| 0x%02X | 0x%02X | 0x%02X |\n",
                        w->regdiff_imu[i].reg, w->regdiff_imu[i].before,
                        w->regdiff_imu[i].after);
            fprintf(f, "\n");
        }
        /* The init->init diff belongs with the reset->init one rather than in
         * a section of its own: same register map, same volatile filter, and
         * a reader chasing imu.init.idempotent wants both images side by side.
         * Printed only when it is non-empty — an always-present "no registers
         * differ" table is noise on the overwhelmingly common pass. */
        if (w->n_idem_imu > 0) {
            fprintf(f, "A second `init()` did not reproduce the first image. "
                       "These registers hold a different value after two "
                       "`init()` calls than after one, so `init()` depends on "
                       "the state it was called in:\n\n");
            fprintf(f, "| reg | after 1st init | after 2nd init |\n|---|---|---|\n");
            for (int i = 0; i < w->n_idem_imu; i++)
                fprintf(f, "| 0x%02X | 0x%02X | 0x%02X |\n",
                        w->idem_imu[i].reg, w->idem_imu[i].before,
                        w->idem_imu[i].after);
            fprintf(f, "\n");
        }
    }
    /* Emitted whenever a magnetometer was under test, even with nothing to
     * show: a section that vanishes leaves a hole in the numbering and reads
     * as a bug in the tool. */
    if (w->regdiff_mag_writeonly) {
        fprintf(f, "### 5.6 Control-register diff, mag 0x%02X\n\n", r->mag_addr);
        fprintf(f, "Not available: this part's control registers are "
                   "write-only, so nothing `init()` wrote can be read back. "
                   "The register writes are covered off-hardware by "
                   "`test_drivers` against the mock bus.\n\n");
    } else if (w->regdiff_mag_mapped) {
        fprintf(f, "### 5.6 Control-register diff, mag 0x%02X\n\n", r->mag_addr);
        fprintf(f, "%d register%s excluded as volatile.\n\n",
                w->n_volatile_mag, w->n_volatile_mag == 1 ? "" : "s");
        if (w->n_regdiff_mag == 0) {
            fprintf(f, "No registers changed across `init()`.\n\n");
        } else {
            fprintf(f, "| reg | after reset | after init |\n|---|---|---|\n");
            for (int i = 0; i < w->n_regdiff_mag; i++)
                fprintf(f, "| 0x%02X | 0x%02X | 0x%02X |\n",
                        w->regdiff_mag[i].reg, w->regdiff_mag[i].before,
                        w->regdiff_mag[i].after);
            fprintf(f, "\n");
        }
    }

    if (r->have_mag && w->mag_n > 0) {
        fprintf(f, "### 5.7 Magnetometer at rest\n\n");
        fprintf(f, "```\n");
        fprintf(f, "rate        %.2f Hz (%llu samples in %.2f s)\n",
                w->mag_rate_hz, (unsigned long long)w->mag_n, w->mag_window_s);
        fprintf(f, "not-ready   %d, bus errors %d\n", w->mag_rc1, w->mag_rcneg);
        /* The rate the daemon actually gets: it waits on the interrupt rather
         * than polling, and on some parts the two differ by a factor of 3. */
        if (w->mag_drdy_edges >= 0)
            fprintf(f, "drdy rate   %.2f Hz (%d samples from %d edges in %.2f s)\n",
                    w->mag_drdy_rate_hz, w->mag_drdy_samples,
                    w->mag_drdy_edges, w->mag_drdy_window_s);
        fprintf(f, "field mean  %.2f / %.2f / %.2f uT\n",
                w->magf.mean[0], w->magf.mean[1], w->magf.mean[2]);
        fprintf(f, "field sigma %.3f / %.3f / %.3f uT\n",
                w->magf.sigma[0], w->magf.sigma[1], w->magf.sigma[2]);
        fprintf(f, "|B|         %.2f uT (range %.2f .. %.2f)\n",
                w->mag_norm_mean, w->mag_norm_min, w->mag_norm_max);
        if (w->mag_dg_n > 0) {
            fprintf(f, "\n");
            fprintf(f, "SET mean    %.2f / %.2f / %.2f uT\n",
                    w->mag_dg_set[0], w->mag_dg_set[1], w->mag_dg_set[2]);
            fprintf(f, "RESET mean  %.2f / %.2f / %.2f uT\n",
                    w->mag_dg_reset[0], w->mag_dg_reset[1], w->mag_dg_reset[2]);
            fprintf(f, "field       %.2f / %.2f / %.2f uT  |%.2f|   (SET-RESET)/2\n",
                    w->mag_dg_field[0], w->mag_dg_field[1], w->mag_dg_field[2],
                    w->mag_dg_field_norm);
            fprintf(f, "offset      %.2f / %.2f / %.2f uT  |%.2f|   (SET+RESET)/2\n",
                    w->mag_dg_offset[0], w->mag_dg_offset[1], w->mag_dg_offset[2],
                    w->mag_dg_offset_norm);
        }
        if (w->mag_bf_n > 0) {
            /* Raw, because a verdict cannot distinguish an off-by-one shift
             * from a pointer that never moved, and the bytes can. */
            fprintf(f, "\nburst  ");
            for (int i = 0; i < w->mag_bf_n; i++)
                fprintf(f, "%02X ", w->mag_bf_burst[i]);
            fprintf(f, "\nsingle ");
            for (int i = 0; i < w->mag_bf_n; i++)
                fprintf(f, "%02X ", w->mag_bf_single[i]);
            fprintf(f, "\n");
        }
        fprintf(f, "```\n\n");
    }

    if (w->n_faces > 0) {
        /* The sample count carries what the mean vector cannot: a face
         * averaged from ten samples reads the same as one averaged from ten
         * thousand, and a face the phase skipped for too few samples is in
         * the table with the count as its only mark. */
        fprintf(f, "### 5.8 Six-face orientation\n\n");
        fprintf(f, "| Face | Expected | Measured (m/s^2) | \\|a\\| | Samples "
                   "| Verdict |\n");
        fprintf(f, "|---|---|---|---|---|---|\n");
        static const char an[3] = { 'X', 'Y', 'Z' };
        for (int i = 0; i < w->n_faces; i++) {
            const imt_face_row_t *fr = &w->face[i];
            fprintf(f, "| %d. %s | %c%c | [%+.2f, %+.2f, %+.2f] | %.3f | %d "
                       "| %s |\n",
                    fr->idx + 1, fr->label ? fr->label : "",
                    fr->exp_sign > 0 ? '+' : '-', an[fr->exp_axis],
                    fr->a[0], fr->a[1], fr->a[2], fr->norm, fr->n,
                    imt_status_str(fr->status));
        }
        fprintf(f, "\nDerived accel calibration: offset "
                   "[%+.3f, %+.3f, %+.3f] m/s^2, scale [%.4f, %.4f, %.4f].\n\n",
                w->face_offset[0], w->face_offset[1], w->face_offset[2],
                w->face_scale[0], w->face_scale[1], w->face_scale[2]);
    }

    if (w->n_turns > 0) {
        /* Samples and duration are the pair that separates a driver that
         * delivered nothing through the turn from one that delivered samples
         * reading zero.  Both integrate to ~0 deg, and without the count the
         * two are indistinguishable in the table. */
        fprintf(f, "### 5.9 Gyro rotation\n\n");
        fprintf(f, "| Axis | Commanded | thetaX | thetaY | thetaZ | Samples "
                   "| Duration | dt source | Verdict |\n");
        fprintf(f, "|---|---|---|---|---|---|---|---|---|\n");
        static const char an[3] = { 'X', 'Y', 'Z' };
        for (int i = 0; i < w->n_turns; i++) {
            const imt_turn_row_t *tr = &w->turn[i];
            fprintf(f, "| %c | %+.0f deg | %+.1f | %+.1f | %+.1f | %d "
                       "| %.1f s | %s | %s |\n",
                    an[tr->axis], tr->cmd_deg,
                    tr->theta[0], tr->theta[1], tr->theta[2],
                    tr->n, tr->dur_s,
                    tr->used_chip_ts ? "chip_ts" : "nominal ODR",
                    imt_status_str(tr->status));
        }
        fprintf(f, "\n");
    }

    if (w->spin_n > 0) {
        fprintf(f, "### 5.10 Magnetometer spin\n\n");
        fprintf(f, "```\n");
        fprintf(f, "samples      %llu\n", (unsigned long long)w->spin_n);
        fprintf(f, "|B| mean     %.2f uT\n", w->spin_norm_mean);
        fprintf(f, "axis ranges  %.2f / %.2f / %.2f uT\n",
                w->spin_range[0], w->spin_range[1], w->spin_range[2]);
        fprintf(f, "mean Bz      %+.2f uT\n", w->spin_bz_mean);
        fprintf(f, "coverage     %d/%d sectors  [", w->spin_covered,
                IMT_MAG_SECTORS);
        for (int i = 0; i < IMT_MAG_SECTORS; i++)
            fputc(w->spin_sectors[i] ? '#' : '.', f);
        fprintf(f, "]\n");
        fprintf(f, "mag heading  %+.1f deg   gyro Z %+.1f deg\n",
                w->spin_heading_delta_deg, w->spin_gyro_z_deg);
        fprintf(f, "```\n\n");
    }

    /* ── 6. Reproduce ────────────────────────────────────────────────────── */
    fprintf(f, "## 6. Reproduce\n\n```\n%s\n```\n\n",
            r->cmdline[0] ? r->cmdline : "imud-imutest --all");

    /* ── 7. How to read this ─────────────────────────────────────────────── */
    fprintf(f, "## 7. How to read this\n\n");
    fprintf(f, "| Status | Meaning |\n|---|---|\n");
    fprintf(f, "| PASS | The driver met the contract in `include/drivers.h` "
               "and the driver guide. |\n");
    fprintf(f, "| FAIL | The driver violated the contract on evidence the "
               "bench cannot explain away. |\n");
    fprintf(f, "| WARN | Out of band, but with a plausible physical cause — an "
               "unlevel surface, a moving board, magnetic clutter, a starved "
               "scheduler — or a check that is one-sided by nature. A WARN "
               "never blocks clearing `experimental`; it asks you to read the "
               "number. |\n");
    fprintf(f, "| SKIP | Capability absent, phase not selected, prerequisite "
               "failed, or a resource was unavailable. |\n");
    fprintf(f, "| INFO | Recorded for the record; no pass criterion. |\n");

    if (fclose(f) != 0) {
        snprintf(errbuf, errbufsz, "error writing %s", path);
        return -1;
    }
    return 0;
}
