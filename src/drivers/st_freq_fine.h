/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * st_freq_fine.h — read the ST 6-axis parts' own declared timebase error.
 *
 * WHAT THE REGISTER IS.  INTERNAL_FREQ_FINE (0x63) is an 8-bit two's-complement
 * factory trim: the difference, in 0.15% steps, between THIS die's internal
 * oscillator and the typical one the datasheet quotes (DS13012 Rev 7 §9.41,
 * Table 139).  ST gives two formulas from it:
 *
 *     TS_Res     = 1 / (40000 + 0.0015 * FREQ_FINE * 40000)          (§9.41)
 *     ODR_Actual = (6667 + 0.0015 * FREQ_FINE * 6667) / ODR_Coeff    (§9.38)
 *
 * Both are the same correction — the timestamp counter and the sample clock
 * come off one oscillator, so a part that stamps fast also samples fast.  Only
 * the first is applied here; see the note at the bottom for why.
 *
 * WHY IT IS WORTH A BUS READ.  The reference ISM330DHCX measured 4.05% fast on
 * a Pi 5, which is well inside ST's tolerance and entirely normal
 * silicon — but 25000 ns/tick is wrong for it by 1000 ns, and imu.c has no way
 * to know until ts_anchor_t had two anchors 20 s apart.  The part could have
 * been asked at init and answered +27 in one byte.
 *
 * The register is factory-programmed ROM, not configuration: it survives reset,
 * reads the same before and after init(), and has no side effects.
 *
 * WHAT CAN GO WRONG, AND WHAT CANNOT.  A wrong tick period is worse than a
 * typical one, because every per-sample dt is scaled by it — so the failure
 * modes are worth being explicit about.  A failed read returns 0, meaning
 * "keep the declared value".
 * An out-of-range result is NOT separately guarded, because the field cannot
 * produce one: int8 x 0.15% bounds the correction to [-19.2%, +19.05%] by
 * construction, and a range check outside that would be a branch no input can
 * reach.  Reading the register off the wrong part is handled where it belongs,
 * in probe()'s WHO_AM_I gate.
 *
 * Header-only and driver-private, like bus_io.h and chip_ts.h.
 */
#ifndef IMUD_DRIVERS_ST_FREQ_FINE_H
#define IMUD_DRIVERS_ST_FREQ_FINE_H

#include <stdint.h>

#include "bus_io.h"

#define ST_REG_INTERNAL_FREQ_FINE  0x63

/*
 * Return this part's real tick period in ns, or 0 to keep `nominal_tick_ns`.
 *
 * Rounds to nearest rather than truncating: one 0.15% step is ~37 ns at a
 * 25000 ns tick, so truncation would bias every part the same direction by a
 * meaningful fraction of the very error this is correcting.
 */
static inline uint32_t st_freq_fine_tick_ns(const imud_bus_t *bus,
                                            uint32_t nominal_tick_ns)
{
    uint8_t raw = 0;
    if (nominal_tick_ns == 0) return 0;
    if (bus_reg_read(bus, ST_REG_INTERNAL_FREQ_FINE, &raw) < 0) return 0;

    /* A part reading exactly typical is the common case and needs no work. */
    int steps = (int)(int8_t)raw;
    if (steps == 0) return nominal_tick_ns;

    /* Faster oscillator (positive steps) = MORE ticks per second = a SHORTER
     * period per tick, hence the divide. */
    double ratio = 1.0 + 0.0015 * (double)steps;
    return (uint32_t)((double)nominal_tick_ns / ratio + 0.5);
}

/*
 * NOT DONE HERE: correcting the effective ODR by the same factor.  It is the
 * same physical fact and §9.38 gives the formula, but the resolved ODR is also
 * the number config validation, the MEKF tuning, the sample-latency publish
 * gates and the generated documentation tables all key off, and it is resolved
 * before the bus is open.  Making it per-part is a separate decision with a
 * much wider blast radius; ts_anchor_t already measures the true sample
 * interval at runtime.
 */

#endif /* IMUD_DRIVERS_ST_FREQ_FINE_H */
