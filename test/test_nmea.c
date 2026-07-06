/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_nmea.c — unit tests for the NMEA 0183 sentence encoder (src/nmea.c)
 *
 * Key correctness properties verified:
 *   - Every emitted sentence has a valid XOR checksum
 *   - Sentences are CRLF-terminated
 *   - Exactly 5 sentences per burst (no FLAG_DECLINATION_VALID)
 *   - Exactly 6 sentences per burst when FLAG_DECLINATION_VALID is set
 *   - $HCHDT not emitted without FLAG_DECLINATION_VALID
 *   - $HCHDT emitted with correct true heading when FLAG_DECLINATION_VALID set
 *   - $HCHDG always emitted; variation fields carry declination (E/W) when
 *     FLAG_DECLINATION_VALID is set, empty otherwise
 *   - PASHR T/M flag is always 'M' (magnetic heading)
 *   - Rate-of-turn sign preserved in $TIROT
 *   - Buffer-too-small returns -1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "nmea.h"
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Test framework ──────────────────────────────────────────────────────── */

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

#define EXPECT_NEAR(a, b, eps, msg) \
    EXPECT(fabs((double)(a) - (double)(b)) < (double)(eps), msg)

static void begin(const char *name) { printf("%-48s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/*
 * Verify the XOR checksum of one "$...*HH\r\n" sentence.
 * Returns true if valid.
 */
static bool verify_checksum(const char *s)
{
    if (*s != '$') return false;
    const char *p = s + 1;
    uint8_t cs = 0;
    while (*p && *p != '*') cs ^= (uint8_t)*p++;
    if (*p != '*') return false;
    unsigned int reported = 0;
    if (sscanf(p + 1, "%2X", &reported) != 1) return false;
    return (uint8_t)reported == cs;
}

/* Count occurrences of '$' in the buffer (= number of sentences). */
static int count_sentences(const char *buf)
{
    int n = 0;
    for (const char *p = buf; *p; p++) if (*p == '$') n++;
    return n;
}

/* Return true if a sentence of the given type appears in the buffer. */
static bool has_sentence(const char *buf, const char *type)
{
    char needle[24];
    snprintf(needle, sizeof(needle), "$%s,", type);
    return strstr(buf, needle) != NULL;
}

/* Return pointer to the field at index (0-based after sentence name). */
static const char *sentence_field(const char *buf, const char *type, int field)
{
    char needle[24];
    snprintf(needle, sizeof(needle), "$%s,", type);
    const char *p = strstr(buf, needle);
    if (!p) return NULL;
    p += strlen(needle);
    for (int i = 0; i < field; i++) {
        p = strchr(p, ',');
        if (!p) return NULL;
        p++;
    }
    return p;
}

/* Build a minimal fused_state_t from degree inputs. */
static fused_state_t make_state(float pitch_deg, float roll_deg,
                                float heading_deg, float rot_deg_min)
{
    fused_state_t s;
    memset(&s, 0, sizeof(s));
    s.pitch        = pitch_deg  * (float)(M_PI / 180.0);
    s.roll         = roll_deg   * (float)(M_PI / 180.0);
    s.heading_deg  = heading_deg;
    s.rate_of_turn = rot_deg_min;
    /* cov[0] = roll error var, cov[4] = pitch error var (rad²) */
    s.cov[0] = 0.0001f;   /* sqrt → 0.01 rad → 0.57° */
    s.cov[4] = 0.0001f;
    s.flags = FLAG_FUSION_CONVERGED;
    return s;
}

/* ── Tests ───────────────────────────────────────────────────────────────── */

static void test_all_checksums_valid(void)
{
    begin("test_all_checksums_valid");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(3.0f, -9.5f, 214.7f, -6.2f);
    int n = nmea_encode(buf, sizeof(buf), &s);
    EXPECT(n > 0, "encode succeeded");

    /* Walk each sentence and verify its checksum. */
    const char *p = buf;
    int checked = 0;
    while ((p = strchr(p, '$')) != NULL) {
        EXPECT(verify_checksum(p), "sentence has valid checksum");
        checked++;
        p++;
    }
    EXPECT(checked == 5, "exactly 5 sentences present");
    end(fb);
}

static void test_sentence_count(void)
{
    begin("test_sentence_count");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    nmea_encode(buf, sizeof(buf), &s);
    EXPECT(count_sentences(buf) == 5, "exactly 5 sentences per burst");
    end(fb);
}

/* $HCHDT is not emitted without FLAG_DECLINATION_VALID. */
static void test_hchdt_without_flag(void)
{
    begin("test_hchdt_without_flag");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    nmea_encode(buf, sizeof(buf), &s);
    EXPECT(!has_sentence(buf, "HCHDT"), "$HCHDT not emitted without FLAG_DECLINATION_VALID");
    end(fb);
}

/* $HCHDT is emitted when FLAG_DECLINATION_VALID is set. */
static void test_hchdt_emitted_with_declination(void)
{
    begin("test_hchdt_emitted_with_declination");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    s.flags |= FLAG_DECLINATION_VALID;
    s.declination_deg = 14.5f;
    nmea_encode(buf, sizeof(buf), &s);
    EXPECT(has_sentence(buf, "HCHDT"), "$HCHDT emitted when FLAG_DECLINATION_VALID set");
    end(fb);
}

/* 5 sentences per burst when FLAG_DECLINATION_VALID is set. */
static void test_sentence_count_with_declination(void)
{
    begin("test_sentence_count_with_declination");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    s.flags |= FLAG_DECLINATION_VALID;
    s.declination_deg = 14.5f;
    nmea_encode(buf, sizeof(buf), &s);
    EXPECT(count_sentences(buf) == 6, "6 sentences when FLAG_DECLINATION_VALID set");
    end(fb);
}

/* $HCHDG is always emitted; variation fields are empty without declination. */
static void test_hchdg_no_declination(void)
{
    begin("test_hchdg_no_declination");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 214.7f, 0);
    nmea_encode(buf, sizeof(buf), &s);
    EXPECT(has_sentence(buf, "HCHDG"), "$HCHDG emitted without declination");
    EXPECT(strstr(buf, "$HCHDG,214.7,,,,*") != NULL,
           "variation fields empty without FLAG_DECLINATION_VALID");
    end(fb);
}

/* $HCHDG variation: value + E for positive (east) declination. */
static void test_hchdg_variation_east(void)
{
    begin("test_hchdg_variation_east");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    s.flags |= FLAG_DECLINATION_VALID;
    s.declination_deg = 13.2f;
    nmea_encode(buf, sizeof(buf), &s);
    EXPECT(strstr(buf, "$HCHDG,090.0,,,13.2,E*") != NULL,
           "positive declination → variation 13.2,E");
    end(fb);
}

/* $HCHDG variation: magnitude + W for negative (west) declination. */
static void test_hchdg_variation_west(void)
{
    begin("test_hchdg_variation_west");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    s.flags |= FLAG_DECLINATION_VALID;
    s.declination_deg = -4.5f;
    nmea_encode(buf, sizeof(buf), &s);
    EXPECT(strstr(buf, "$HCHDG,090.0,,,4.5,W*") != NULL,
           "negative declination → variation 4.5,W (magnitude + W)");
    end(fb);
}

/* $HCHDT true heading = fmodf(mag_heading + decl + 360, 360). */
static void test_hchdt_true_heading_value(void)
{
    begin("test_hchdt_true_heading_value");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    s.flags |= FLAG_DECLINATION_VALID;
    s.declination_deg = 14.5f;
    nmea_encode(buf, sizeof(buf), &s);

    const char *f = sentence_field(buf, "HCHDT", 0);
    EXPECT(f != NULL, "$HCHDT heading field present");
    float true_hdg = f ? strtof(f, NULL) : 0.0f;
    float expected = fmodf(90.0f + 14.5f + 360.0f, 360.0f);   /* 104.5 */
    EXPECT_NEAR(true_hdg, expected, 0.1f, "$HCHDT heading = mag + decl");
    end(fb);
}

/* $HCHDT checksum must be valid. */
static void test_hchdt_checksum_valid(void)
{
    begin("test_hchdt_checksum_valid");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 270.0f, 0);
    s.flags |= FLAG_DECLINATION_VALID;
    s.declination_deg = -10.0f;
    nmea_encode(buf, sizeof(buf), &s);

    const char *ps = strstr(buf, "$HCHDT");
    EXPECT(ps != NULL, "$HCHDT sentence present");
    EXPECT(ps && verify_checksum(ps), "$HCHDT checksum valid");
    end(fb);
}

static void test_crlf_termination(void)
{
    begin("test_crlf_termination");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 0, 0);
    int n = nmea_encode(buf, sizeof(buf), &s);
    EXPECT(n >= 2, "non-empty output");
    EXPECT(buf[n - 2] == '\r' && buf[n - 1] == '\n', "buffer ends with CRLF");

    /* Every sentence must end with CRLF immediately before its checksum tail */
    const char *p = buf;
    while ((p = strstr(p, "\r\n")) != NULL) {
        const char *after = p + 2;
        EXPECT(*after == '$' || *after == '\0',
               "CRLF only appears at sentence boundaries");
        p += 2;
    }
    end(fb);
}

/* PASHR always shows 'M' (magnetic) — declination is left to consumers. */
static void test_tm_flag_always_magnetic(void)
{
    begin("test_tm_flag_always_magnetic");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 90.0f, 0);
    nmea_encode(buf, sizeof(buf), &s);
    /* $PASHR field 1 (after heading) is T or M */
    const char *f = sentence_field(buf, "PASHR", 1);
    EXPECT(f && *f == 'M', "PASHR always shows M (magnetic only)");
    end(fb);
}

static void test_rot_sign_positive(void)
{
    begin("test_rot_sign_positive");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 0, 12.5f);   /* turning right */
    nmea_encode(buf, sizeof(buf), &s);

    const char *f = sentence_field(buf, "TIROT", 0);
    EXPECT(f != NULL, "$TIROT field present");
    float rot = f ? strtof(f, NULL) : 0.0f;
    EXPECT(rot > 0.0f, "positive rate-of-turn is positive in $TIROT");
    EXPECT_NEAR(rot, 12.5f, 0.2f, "$TIROT rate-of-turn value");
    end(fb);
}

static void test_rot_sign_negative(void)
{
    begin("test_rot_sign_negative");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 0, -8.3f);   /* turning left */
    nmea_encode(buf, sizeof(buf), &s);

    const char *f = sentence_field(buf, "TIROT", 0);
    float rot = f ? strtof(f, NULL) : 0.0f;
    EXPECT(rot < 0.0f, "negative rate-of-turn is negative in $TIROT");
    end(fb);
}

static void test_iixdr_signs(void)
{
    begin("test_iixdr_signs");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    /* pitch=+5° (bow up), roll=-10° (port up) */
    fused_state_t s = make_state(5.0f, -10.0f, 0, 0);
    nmea_encode(buf, sizeof(buf), &s);

    /* $IIXDR,A,PPP.P,D,PTCH,A,RRR.R,D,ROLL
     * Field 1 = pitch, field 5 = roll */
    const char *fp = sentence_field(buf, "IIXDR", 1);
    const char *fr = sentence_field(buf, "IIXDR", 5);
    EXPECT(fp && fr, "$IIXDR pitch and roll fields found");
    float pitch = fp ? strtof(fp, NULL) : 0.0f;
    float roll  = fr ? strtof(fr, NULL) : 0.0f;
    EXPECT_NEAR(pitch,  5.0f, 0.1f, "$IIXDR pitch +5°");
    EXPECT_NEAR(roll,  -10.0f, 0.1f, "$IIXDR roll -10°");
    end(fb);
}

/* heading=0.0 must format as "000.0", not "-000.0" or "  0.0" */
static void test_nmea_heading_zero(void)
{
    begin("test_nmea_heading_zero");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 0.0f, 0);
    int n = nmea_encode(buf, sizeof(buf), &s);
    EXPECT(n > 0, "encode succeeded at heading=0");

    const char *f = sentence_field(buf, "PASHR", 0);
    EXPECT(f != NULL, "$PASHR heading field present");
    EXPECT(f && strncmp(f, "000.0", 5) == 0, "heading=0.0 encodes as '000.0'");

    const char *fh = sentence_field(buf, "HCHDM", 0);
    EXPECT(fh != NULL, "$HCHDM heading field present");
    EXPECT(fh && strncmp(fh, "000.0", 5) == 0, "$HCHDM heading=0.0 encodes as '000.0'");
    end(fb);
}

/* heading=359.9 must survive formatting without wrapping to 360 or going negative */
static void test_nmea_heading_359_9(void)
{
    begin("test_nmea_heading_359_9");
    int fb = g_fail;

    char buf[NMEA_BUF_MIN];
    fused_state_t s = make_state(0, 0, 359.9f, 0);
    nmea_encode(buf, sizeof(buf), &s);

    const char *f = sentence_field(buf, "PASHR", 0);
    EXPECT(f != NULL, "$PASHR heading field present");
    float hdg = f ? strtof(f, NULL) : 0.0f;
    EXPECT_NEAR(hdg, 359.9f, 0.05f, "heading=359.9 encoded correctly in $PASHR");

    /* Must have valid checksum even at this boundary value */
    const char *ps = strstr(buf, "$PASHR");
    EXPECT(ps && verify_checksum(ps), "$PASHR checksum valid at hdg=359.9");
    end(fb);
}

static void test_buffer_too_small(void)
{
    begin("test_buffer_too_small");
    int fb = g_fail;

    char buf[16];   /* much smaller than NMEA_BUF_MIN */
    fused_state_t s = make_state(0, 0, 0, 0);
    int n = nmea_encode(buf, sizeof(buf), &s);
    EXPECT(n == -1, "returns -1 when buffer < NMEA_BUF_MIN");
    end(fb);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    puts("=== imud nmea tests ===");

    test_all_checksums_valid();
    test_sentence_count();
    test_hchdt_without_flag();
    test_hchdt_emitted_with_declination();
    test_sentence_count_with_declination();
    test_hchdt_true_heading_value();
    test_hchdt_checksum_valid();
    test_hchdg_no_declination();
    test_hchdg_variation_east();
    test_hchdg_variation_west();
    test_crlf_termination();
    test_tm_flag_always_magnetic();
    test_rot_sign_positive();
    test_rot_sign_negative();
    test_iixdr_signs();
    test_nmea_heading_zero();
    test_nmea_heading_359_9();
    test_buffer_too_small();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
