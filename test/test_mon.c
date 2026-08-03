/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_mon.c — imud-mon's stream decoding (src/mon_parse.c)
 *
 * mon_parse_nmea runs on bytes that arrived over UDP.  imud-mon is a
 * diagnostic tool and the sender is normally the local daemon, so this is not
 * the trust boundary the fuzz harnesses cover — but "normally" is doing work
 * in that sentence, and the function copies a caller-supplied length into a
 * fixed buffer, so the clamp gets an assertion rather than a reading.
 *
 * mon_flag_str is here for the same reason: its remaining-space arithmetic
 * could underflow (see src/mon_parse.c), and the sizes that trip it are
 * exactly the ones only a direct caller can produce.
 *
 * Portable — builds and runs on the macOS dev box.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mon_parse.h"

/* ── Test framework (matches the rest of the suite) ──────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabsf((float)(a) - (float)(b)) < (float)(eps), msg)

#include <math.h>

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb)             { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── CRC32 ───────────────────────────────────────────────────────────────── */

static void test_crc32(void)
{
    begin("test_crc32");
    int fb = g_fail;

    /* The published check value for CRC-32/ISO-HDLC. If this ever fails, the
     * daemon and imud-mon disagree about what a valid packet is and every
     * binary line reads "(no data)". */
    EXPECT(mon_crc32_ieee((const uint8_t *)"123456789", 9) == 0xCBF43926u,
           "check value for \"123456789\"");

    EXPECT(mon_crc32_ieee((const uint8_t *)"", 0) == 0x00000000u,
           "empty input");

    /* A single flipped bit must change the result — the whole point. */
    uint32_t a = mon_crc32_ieee((const uint8_t *)"imud", 4);
    uint32_t b = mon_crc32_ieee((const uint8_t *)"imue", 4);
    EXPECT(a != b, "one changed byte changes the CRC");

    /* Deterministic across calls. */
    EXPECT(mon_crc32_ieee((const uint8_t *)"imud", 4) == a, "stable");

    end(fb);
}

/* ── NMEA field extraction ───────────────────────────────────────────────── */

static void test_nmea_get_field(void)
{
    begin("test_nmea_get_field");
    int fb = g_fail;
    float v;

    const char *s = "$TIROT,12.5,A*3F";
    EXPECT(mon_nmea_get_field(s, 0, &v) && fabsf(v - 12.5f) < 1e-4f,
           "field 0");

    /* Field 1 is "A" — strtof yields 0 rather than failing.  Pinned as the
     * current contract: this parser is positional and does not validate. */
    EXPECT(mon_nmea_get_field(s, 1, &v), "field 1 exists");

    EXPECT(!mon_nmea_get_field(s, 5, &v), "field past the end → false");

    /* An empty field is absent, not zero — the distinction matters for a
     * heading that the daemon chose not to emit. */
    EXPECT(!mon_nmea_get_field("$PASHR,,M,1.0", 0, &v), "empty field → false");

    /* A field terminated by the checksum delimiter is also empty. */
    EXPECT(!mon_nmea_get_field("$TIROT,*3F", 0, &v), "field ending at * → false");

    /* No commas at all. */
    EXPECT(!mon_nmea_get_field("$TIROT", 0, &v), "no comma → false");
    EXPECT(!mon_nmea_get_field("", 0, &v), "empty sentence → false");

    /* Negative values and exponents survive strtof. */
    EXPECT(mon_nmea_get_field("$X,-3.25", 0, &v) && fabsf(v + 3.25f) < 1e-4f,
           "negative value");

    /* out must be untouched when the field is absent. */
    v = 99.0f;
    EXPECT(!mon_nmea_get_field("$X", 0, &v) && fabsf(v - 99.0f) < 1e-6f,
           "out untouched on failure");

    end(fb);
}

/* ── Burst parsing ───────────────────────────────────────────────────────── */

static void test_parse_nmea(void)
{
    begin("test_parse_nmea");
    int fb = g_fail;
    mon_state_t st;

    /* A realistic burst: all three sentences imud-mon looks for, in one
     * datagram, mixed in with one it ignores. */
    const char *burst =
        "$PASHR,123.4,M,-2.50,1.25,0.00,,,,,0*00\r\n"
        "$HCHDG,123.4,,,,*00\r\n"
        "$TIROT,15.0,A*00\r\n"
        "$HCHDT,138.4,T*00\r\n";

    memset(&st, 0, sizeof st);
    mon_parse_nmea(&st, burst, strlen(burst));
    EXPECT(st.have_nmea, "have_nmea set");
    EXPECT_NEAR(st.nmea_hdg,   123.4f, 1e-3, "PASHR heading");
    EXPECT_NEAR(st.nmea_roll,   -2.50f, 1e-3, "PASHR roll (field 2)");
    EXPECT_NEAR(st.nmea_pitch,   1.25f, 1e-3, "PASHR pitch (field 3)");
    EXPECT_NEAR(st.nmea_rot,    15.0f, 1e-3, "TIROT rate of turn");
    EXPECT(st.nmea_has_true_hdg, "HCHDT present → true heading valid");
    EXPECT_NEAR(st.nmea_true_hdg, 138.4f, 1e-3, "HCHDT true heading");

    /* nmea_has_true_hdg is the only field cleared per burst: when declination
     * stops being available the daemon stops sending $HCHDT, and imud-mon must
     * stop showing a true heading rather than freezing the last one. */
    const char *no_hdt = "$PASHR,200.0,M,0.00,0.00,0.00,,,,,0*00\r\n";
    mon_parse_nmea(&st, no_hdt, strlen(no_hdt));
    EXPECT(!st.nmea_has_true_hdg, "HCHDT absent → true heading goes invalid");
    EXPECT_NEAR(st.nmea_hdg, 200.0f, 1e-3, "new PASHR heading applied");
    /* …while the others deliberately persist. */
    EXPECT_NEAR(st.nmea_rot, 15.0f, 1e-3, "TIROT persists across a burst");

    /* A malformed $PASHR must leave the previous attitude alone rather than
     * writing partial values. */
    const char *bad = "$PASHR,junk\r\n";
    mon_parse_nmea(&st, bad, strlen(bad));
    EXPECT_NEAR(st.nmea_hdg, 200.0f, 1e-3, "malformed PASHR keeps last heading");

    /* Sentence order must not matter. */
    const char *reordered =
        "$HCHDT,10.0,T*00\r\n$TIROT,1.0,A*00\r\n"
        "$PASHR,20.0,M,3.00,4.00,0.00,,,,,0*00\r\n";
    memset(&st, 0, sizeof st);
    mon_parse_nmea(&st, reordered, strlen(reordered));
    EXPECT_NEAR(st.nmea_hdg,      20.0f, 1e-3, "reordered: heading");
    EXPECT_NEAR(st.nmea_rot,       1.0f, 1e-3, "reordered: rot");
    EXPECT_NEAR(st.nmea_true_hdg, 10.0f, 1e-3, "reordered: true heading");

    /* Nothing recognisable: have_nmea still set (a datagram did arrive), no
     * field written. */
    memset(&st, 0, sizeof st);
    mon_parse_nmea(&st, "garbage\r\n", 9);
    EXPECT(st.have_nmea && st.nmea_hdg == 0.0f, "unrecognised burst is inert");

    end(fb);
}

static void test_parse_nmea_bounds(void)
{
    begin("test_parse_nmea_bounds");
    int fb = g_fail;

    /* An over-length datagram must be clamped into nmea_raw, not copied past
     * it.  The state struct is bracketed by guard bytes so an overrun is
     * visible even though nmea_raw is its first member. */
    struct { unsigned char before[64]; mon_state_t st; unsigned char after[64]; } g;
    memset(&g, 0xAA, sizeof g);
    memset(&g.st, 0, sizeof g.st);

    char big[NMEA_MAX * 3];
    memset(big, 'A', sizeof big);
    memcpy(big, "$TIROT,7.5,A*00\r\n", 17);

    mon_parse_nmea(&g.st, big, sizeof big);

    EXPECT(strlen(g.st.nmea_raw) == NMEA_MAX - 1, "clamped to NMEA_MAX - 1");
    EXPECT(g.st.nmea_raw[NMEA_MAX - 1] == '\0', "and NUL-terminated");
    EXPECT_NEAR(g.st.nmea_rot, 7.5f, 1e-3, "still parses what did fit");

    int guard_ok = 1;
    for (size_t i = 0; i < sizeof g.before; i++) if (g.before[i] != 0xAA) guard_ok = 0;
    for (size_t i = 0; i < sizeof g.after;  i++) if (g.after[i]  != 0xAA) guard_ok = 0;
    EXPECT(guard_ok, "no write outside mon_state_t");

    /* Exactly at the limit, and one under. */
    mon_state_t st;
    memset(&st, 0, sizeof st);
    mon_parse_nmea(&st, big, NMEA_MAX);
    EXPECT(strlen(st.nmea_raw) == NMEA_MAX - 1, "len == NMEA_MAX clamps");

    memset(&st, 0, sizeof st);
    mon_parse_nmea(&st, big, NMEA_MAX - 1);
    EXPECT(strlen(st.nmea_raw) == NMEA_MAX - 1, "len == NMEA_MAX - 1 fits exactly");

    /* Zero-length datagram. */
    memset(&st, 0, sizeof st);
    mon_parse_nmea(&st, "", 0);
    EXPECT(st.have_nmea && st.nmea_raw[0] == '\0', "empty datagram");

    end(fb);
}

/* ── Flag summary ────────────────────────────────────────────────────────── */

static void test_flag_str(void)
{
    begin("test_flag_str");
    int fb = g_fail;
    char b[64];

    mon_flag_str(0, b, sizeof b);
    EXPECT(strcmp(b, "0x0000 []") == 0, "no flags");

    mon_flag_str(FLAG_FUSION_CONVERGED, b, sizeof b);
    EXPECT(strcmp(b, "0x0004 [C]") == 0, "one flag, hex value included");

    /* Order is fixed: C V A G M D S ! */
    uint16_t all = FLAG_FUSION_CONVERGED | FLAG_MAG_VALID | FLAG_ACCEL_CAL |
                   FLAG_GYRO_CAL | FLAG_MAG_CAL | FLAG_DECLINATION_VALID |
                   FLAG_STARTUP | FLAG_FIFO_OVERFLOW;
    mon_flag_str(all, b, sizeof b);
    EXPECT(strstr(b, "[CVAGMDS!]") != NULL, "all eight, in order");

    /* Bits 0 and 4, but printed in the table's order (V before G), not the
     * bit order — the label sequence is a display choice, not the flags word. */
    mon_flag_str(FLAG_MAG_VALID | FLAG_GYRO_CAL, b, sizeof b);
    EXPECT(strcmp(b, "0x0011 [VG]") == 0, "subset keeps the display order");

    /* The size the shipped call site uses, against the widest possible value:
     * it must not truncate. */
    char real[32];
    mon_flag_str(0xFFFF, real, sizeof real);
    EXPECT(strlen(real) == 17, "the 32-byte call-site buffer is not tight");
    EXPECT(real[strlen(real) - 1] == ']', "and the summary is closed");

    end(fb);
}

/*
 * Every buffer size, with all eight flags set — the case that walked the old
 * implementation's remaining-space counter from 1 to 0 to SIZE_MAX and then
 * wrote past the end.  Guard bytes either side catch that directly.
 */
static void test_flag_str_truncation(void)
{
    begin("test_flag_str_truncation");
    int fb = g_fail;

    uint16_t all = FLAG_FUSION_CONVERGED | FLAG_MAG_VALID | FLAG_ACCEL_CAL |
                   FLAG_GYRO_CAL | FLAG_MAG_CAL | FLAG_DECLINATION_VALID |
                   FLAG_STARTUP | FLAG_FIFO_OVERFLOW;

    char full[64];
    mon_flag_str(all, full, sizeof full);
    size_t flen = strlen(full);

    int bad_term = 0, bad_guard = 0, bad_prefix = 0;
    for (size_t sz = 0; sz <= flen + 4; sz++) {
        unsigned char arena[256];
        memset(arena, 0xAA, sizeof arena);
        char *b = (char *)arena + 64;

        mon_flag_str(all, b, sz);

        if (sz > 0) {
            if (memchr(b, '\0', sz) == NULL) bad_term++;
            /* Truncated output is a prefix of the untruncated one, except that
             * the closing ']' is dropped rather than overwriting a flag. */
            if (strncmp(b, full, strlen(b)) != 0 &&
                !(strlen(b) > 0 && b[strlen(b) - 1] == ']')) bad_prefix++;
        }
        for (size_t g = 0; g < 64; g++)
            if (arena[g] != 0xAA) { bad_guard++; break; }
        for (size_t g = 64 + sz; g < sizeof arena; g++)
            if (arena[g] != 0xAA) { bad_guard++; break; }
    }

    EXPECT(bad_term   == 0, "always NUL-terminated inside the buffer");
    EXPECT(bad_prefix == 0, "truncated output is a prefix");
    EXPECT(bad_guard  == 0, "never writes outside [buf, buf + sz)");

    /* sz == 0 must not write at all, and NULL must not be dereferenced. */
    unsigned char zero[16];
    memset(zero, 0xAA, sizeof zero);
    mon_flag_str(all, (char *)zero, 0);
    int untouched = 1;
    for (size_t i = 0; i < sizeof zero; i++) if (zero[i] != 0xAA) untouched = 0;
    EXPECT(untouched, "sz == 0 writes nothing");
    mon_flag_str(all, NULL, 32);
    EXPECT(1, "NULL buffer returns without dereferencing");

    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("test_mon — imud-mon stream decoding\n");

    test_crc32();
    test_nmea_get_field();
    test_parse_nmea();
    test_parse_nmea_bounds();
    test_flag_str();
    test_flag_str_truncation();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
