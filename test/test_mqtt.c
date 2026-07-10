/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_mqtt.c — unit tests for the MQTT builders (src/mqtt_publish.c)
 *
 * Drives mqtt_build_state() and mqtt_build_discovery() with crafted packets and
 * asserts on topic names, value formatting (deg vs rad), declination/heave
 * gating, and Home Assistant discovery payload shape. Pure functions — no broker.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mqtt_publish.h"

static int g_pass, g_fail;

#define EXPECT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  FAIL  %s:%d  %s\n", \
           __FILE__, __LINE__, (msg)); } \
} while (0)

static void begin(const char *name) { printf("%-52s", name); fflush(stdout); }
static void end(int fb) { puts(g_fail == fb ? "OK" : "FAIL"); }

static const mqtt_msg_t *find_topic(const mqtt_msg_t *m, int n, const char *topic)
{
    for (int i = 0; i < n; i++)
        if (strcmp(m[i].topic, topic) == 0) return &m[i];
    return NULL;
}

/* Known packet: heading 90°, RoT 60 dpm, roll 0.10 rad, pitch -0.05, yaw 1.23,
 * heave 0.42 m, temp 31.4 °C. */
static imud_packet_t make_pkt(void)
{
    imud_packet_t p;
    memset(&p, 0, sizeof p);
    p.magic        = IMUD_MAGIC;
    p.version      = IMUD_VERSION;
    p.heading_deg  = 90.0f;
    p.rate_of_turn = 60.0f;
    p.roll  = 0.10f;
    p.pitch = -0.05f;
    p.yaw   = 1.23f;
    p.heave_m = 0.42f;
    p.heave_rate = 0.25f;
    p.temp_c  = 31.4f;
    return p;
}

static void test_state_deg(void)
{
    begin("test_state_deg");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    mqtt_msg_t m[16];
    int n = mqtt_build_state(m, 16, &p, "imud", false, true);
    /* always-on set, no declination, no heave = 6 */
    EXPECT(n == 6, "6 state msgs (no declination, no heave)");

    const mqtt_msg_t *h = find_topic(m, n, "imud/navigation/headingMagnetic");
    EXPECT(h != NULL, "headingMagnetic topic present");
    EXPECT(h && fabs(strtod(h->payload, NULL) - 90.0) < 1e-3, "heading 90.00° in deg mode");

    const mqtt_msg_t *r = find_topic(m, n, "imud/attitude/roll");
    EXPECT(r != NULL, "attitude/roll topic present");
    EXPECT(r && fabs(strtod(r->payload, NULL) - 5.7296) < 1e-2, "roll 0.10 rad → 5.73° (native, not negated)");

    const mqtt_msg_t *rot = find_topic(m, n, "imud/navigation/rateOfTurn");
    EXPECT(rot && fabs(strtod(rot->payload, NULL) - 60.0) < 1e-2, "rateOfTurn 60 °/min in deg mode");

    const mqtt_msg_t *t = find_topic(m, n, "imud/imu/temperature");
    EXPECT(t && fabs(strtod(t->payload, NULL) - 31.4) < 1e-2, "temperature 31.4 °C");

    EXPECT(find_topic(m, n, "imud/navigation/headingTrue") == NULL, "no headingTrue without declination");
    EXPECT(find_topic(m, n, "imud/environment/heave") == NULL, "no heave when emit_heave=false");
    end(fb);
}

static void test_state_rad(void)
{
    begin("test_state_rad");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    mqtt_msg_t m[16];
    int n = mqtt_build_state(m, 16, &p, "imud", false, false);   /* deg=false → SI */

    const mqtt_msg_t *h = find_topic(m, n, "imud/navigation/headingMagnetic");
    EXPECT(h && fabs(strtod(h->payload, NULL) - M_PI/2.0) < 1e-3, "heading 90° → π/2 rad");

    const mqtt_msg_t *r = find_topic(m, n, "imud/attitude/roll");
    EXPECT(r && fabs(strtod(r->payload, NULL) - 0.10) < 1e-3, "roll 0.10 rad passthrough (native)");

    const mqtt_msg_t *rot = find_topic(m, n, "imud/navigation/rateOfTurn");
    EXPECT(rot && fabs(strtod(rot->payload, NULL) - 60.0*(M_PI/180.0)/60.0) < 1e-5, "rateOfTurn 60 dpm → rad/s");
    end(fb);
}

static void test_declination_gated(void)
{
    begin("test_declination_gated");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    mqtt_msg_t m[16];

    p.flags = 0;
    p.declination_deg = 13.2f;
    int n = mqtt_build_state(m, 16, &p, "imud", false, true);
    EXPECT(find_topic(m, n, "imud/navigation/headingTrue") == NULL, "headingTrue absent without flag");
    EXPECT(find_topic(m, n, "imud/navigation/magneticVariation") == NULL, "variation absent without flag");

    p.flags = IMUD_FLAG_DECLINATION_VALID;
    n = mqtt_build_state(m, 16, &p, "imud", false, true);
    const mqtt_msg_t *var = find_topic(m, n, "imud/navigation/magneticVariation");
    EXPECT(var && fabs(strtod(var->payload, NULL) - 13.2) < 1e-2, "variation 13.2° present with flag");
    const mqtt_msg_t *ht = find_topic(m, n, "imud/navigation/headingTrue");
    EXPECT(ht && fabs(strtod(ht->payload, NULL) - 103.2) < 1e-2, "headingTrue = 90 + 13.2 = 103.2°");
    end(fb);
}

static void test_heave_gated(void)
{
    begin("test_heave_gated");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    mqtt_msg_t m[16];

    /* Off when emit_heave=false, even if the estimator has settled. */
    p.flags = IMUD_FLAG_HEAVE_VALID;
    int n = mqtt_build_state(m, 16, &p, "imud", false, true);
    EXPECT(find_topic(m, n, "imud/environment/heave") == NULL, "heave absent when emit_heave=false");
    EXPECT(find_topic(m, n, "imud/environment/heaveRate") == NULL, "heaveRate absent when emit_heave=false");

    /* Config on but not settled (flag clear) → still suppressed. */
    p.flags = 0;
    n = mqtt_build_state(m, 16, &p, "imud", true, true);
    EXPECT(find_topic(m, n, "imud/environment/heave") == NULL, "heave suppressed until HEAVE_VALID");
    EXPECT(find_topic(m, n, "imud/environment/heaveRate") == NULL, "heaveRate suppressed until HEAVE_VALID");

    /* Config on AND settled → both published. */
    p.flags = IMUD_FLAG_HEAVE_VALID;
    n = mqtt_build_state(m, 16, &p, "imud", true, true);
    const mqtt_msg_t *hv = find_topic(m, n, "imud/environment/heave");
    EXPECT(hv && fabs(strtod(hv->payload, NULL) - 0.42) < 1e-3, "heave 0.42 m when settled");
    const mqtt_msg_t *hr = find_topic(m, n, "imud/environment/heaveRate");
    EXPECT(hr && fabs(strtod(hr->payload, NULL) - 0.25) < 1e-3, "heaveRate 0.25 m/s when settled");
    end(fb);
}

static void test_prefix_and_count_cap(void)
{
    begin("test_prefix_and_count_cap");
    int fb = g_fail;

    imud_packet_t p = make_pkt();
    p.flags = IMUD_FLAG_DECLINATION_VALID | IMUD_FLAG_HEAVE_VALID;
    mqtt_msg_t m[16];

    int n = mqtt_build_state(m, 16, &p, "boat", true, true);
    EXPECT(n == 10, "10 state msgs with declination + heave + heaveRate");
    EXPECT(find_topic(m, n, "boat/attitude/yaw") != NULL, "custom prefix applied");

    /* max caps the count, no overflow */
    int c = mqtt_build_state(m, 3, &p, "imud", true, true);
    EXPECT(c == 3, "count capped at max");
    end(fb);
}

static void test_discovery(void)
{
    begin("test_discovery");
    int fb = g_fail;

    mqtt_msg_t m[16];
    int n = mqtt_build_discovery(m, 16, "imud", "homeassistant", "imud", false, true);
    EXPECT(n == 8, "8 discovery configs (no heave/heaveRate)");

    const mqtt_msg_t *d = find_topic(m, n, "homeassistant/sensor/imud_roll/config");
    EXPECT(d != NULL, "roll discovery config topic present");
    if (d) {
        EXPECT(strstr(d->payload, "\"stat_t\":\"imud/attitude/roll\"") != NULL, "state_topic matches state msg");
        EXPECT(strstr(d->payload, "\"uniq_id\":\"imud_roll\"") != NULL, "unique_id present");
        EXPECT(strstr(d->payload, "\"unit_of_meas\":\"°\"") != NULL, "degree unit in deg mode");
        EXPECT(strstr(d->payload, "\"avty_t\":\"imud/status/online\"") != NULL, "availability topic present");
        EXPECT(strstr(d->payload, "\"ids\":[\"imud_imud\"]") != NULL, "shared HA device id present");
    }

    EXPECT(find_topic(m, n, "homeassistant/sensor/imud_heave/config") == NULL, "no heave discovery when off");
    EXPECT(find_topic(m, n, "homeassistant/sensor/imud_heave_rate/config") == NULL, "no heaveRate discovery when off");

    /* Discovery is config-gated (emit_heave), NOT gated on the runtime valid flag,
     * so HA keeps the entity and shows it 'unavailable' until heave settles. */
    n = mqtt_build_discovery(m, 16, "imud", "homeassistant", "imud", true, true);
    EXPECT(n == 10, "10 discovery configs with heave + heaveRate");
    EXPECT(find_topic(m, n, "homeassistant/sensor/imud_heave/config") != NULL, "heave discovery when on");
    const mqtt_msg_t *dhr = find_topic(m, n, "homeassistant/sensor/imud_heave_rate/config");
    EXPECT(dhr != NULL, "heaveRate discovery when on");
    EXPECT(dhr && strstr(dhr->payload, "\"unit_of_meas\":\"m/s\"") != NULL, "heaveRate unit m/s");

    /* rad mode advertises radian units */
    mqtt_build_discovery(m, 16, "imud", "homeassistant", "imud", true, false);
    const mqtt_msg_t *dy = find_topic(m, 10, "homeassistant/sensor/imud_yaw/config");
    EXPECT(dy && strstr(dy->payload, "\"unit_of_meas\":\"rad\"") != NULL, "radian unit in rad mode");
    end(fb);
}

int main(void)
{
    puts("=== imud mqtt builder tests ===");
    test_state_deg();
    test_state_rad();
    test_declination_gated();
    test_heave_gated();
    test_prefix_and_count_cap();
    test_discovery();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
