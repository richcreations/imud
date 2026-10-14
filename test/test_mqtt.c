/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_mqtt.c — unit tests for the MQTT builders (src/mqtt_publish.c)
 *
 * Drives mqtt_build_state() and mqtt_build_discovery() with crafted imud_data_t
 * samples and asserts on topic names, value formatting (deg vs rad),
 * declination/heave gating, and Home Assistant discovery payload shape. Pure
 * functions — no broker.
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

/* Known sample: heading 90°, RoT 60 dpm, roll 0.10 rad, pitch -0.05, yaw 1.23,
 * heave 0.42 m, temp 31.4 °C.  heading_true_deg is a member libimud has already
 * computed, so it starts at the "declination not known" sentinel. */
static imud_data_t make_data(void)
{
    imud_data_t d;
    memset(&d, 0, sizeof d);
    d.heading_true_deg = -1.0f;
    d.heading_deg  = 90.0f;
    d.rate_of_turn = 60.0f;
    d.roll  = 0.10f;
    d.pitch = -0.05f;
    d.yaw   = 1.23f;
    d.heave_m = 0.42f;
    d.heave_rate = 0.25f;
    /* A fused, calibrated magnetometer is the baseline: without it the
     * heading paths are withheld, which is its own set of cases below. */
    d.flags = IMUD_FLAG_MAG_VALID;
    d.temp_c  = 31.4f;
    return d;
}

static void test_state_deg(void)
{
    begin("test_state_deg");
    int fb = g_fail;

    imud_data_t d = make_data();
    mqtt_msg_t m[24];
    int n = mqtt_build_state(m, 24, &d, "imud", false, true);
    /* always-on set, no declination, no heave = 6; engine/running and
     * navigation/headingReferenced are both published unconditionally. */
    EXPECT(n == 8, "8 state msgs (no declination, no heave; 2 status topics)");

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

    imud_data_t d = make_data();
    mqtt_msg_t m[24];
    int n = mqtt_build_state(m, 24, &d, "imud", false, false);   /* deg=false → SI */

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

    imud_data_t d = make_data();
    mqtt_msg_t m[24];

    d.flags = IMUD_FLAG_MAG_VALID;
    d.declination_deg = 13.2f;
    int n = mqtt_build_state(m, 24, &d, "imud", false, true);
    EXPECT(find_topic(m, n, "imud/navigation/headingTrue") == NULL, "headingTrue absent without flag");
    EXPECT(find_topic(m, n, "imud/navigation/magneticVariation") == NULL, "variation absent without flag");

    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_DECLINATION_VALID;
    d.heading_true_deg = 103.2f;               /* 90 mag + 13.2 E, per libimud */
    n = mqtt_build_state(m, 24, &d, "imud", false, true);
    const mqtt_msg_t *var = find_topic(m, n, "imud/navigation/magneticVariation");
    EXPECT(var && fabs(strtod(var->payload, NULL) - 13.2) < 1e-2, "variation 13.2° present with flag");
    const mqtt_msg_t *ht = find_topic(m, n, "imud/navigation/headingTrue");
    EXPECT(ht && fabs(strtod(ht->payload, NULL) - 103.2) < 1e-2, "headingTrue = 90 + 13.2 = 103.2°");

    /*
     * True heading is libimud's member, not something the builder recomputes.
     * The two are indistinguishable until the sum wraps: 350 + 20 E is 10°,
     * and a builder adding the fields itself would publish 370° here.
     */
    d.heading_deg      = 350.0f;
    d.declination_deg  = 20.0f;
    d.heading_true_deg = 10.0f;
    n = mqtt_build_state(m, 24, &d, "imud", false, true);
    ht = find_topic(m, n, "imud/navigation/headingTrue");
    EXPECT(ht && fabs(strtod(ht->payload, NULL) - 10.0) < 1e-2,
           "headingTrue is the library's wrapped value, not heading + declination");
    end(fb);
}

static void test_heave_gated(void)
{
    begin("test_heave_gated");
    int fb = g_fail;

    imud_data_t d = make_data();
    mqtt_msg_t m[24];

    /* Off when emit_heave=false, even if the estimator has settled. */
    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_HEAVE_VALID;
    int n = mqtt_build_state(m, 24, &d, "imud", false, true);
    EXPECT(find_topic(m, n, "imud/environment/heave") == NULL, "heave absent when emit_heave=false");
    EXPECT(find_topic(m, n, "imud/environment/heaveRate") == NULL, "heaveRate absent when emit_heave=false");

    /* Config on but not settled (flag clear) → still suppressed. */
    d.flags = IMUD_FLAG_MAG_VALID;
    n = mqtt_build_state(m, 24, &d, "imud", true, true);
    EXPECT(find_topic(m, n, "imud/environment/heave") == NULL, "heave suppressed until HEAVE_VALID");
    EXPECT(find_topic(m, n, "imud/environment/heaveRate") == NULL, "heaveRate suppressed until HEAVE_VALID");

    /* Config on AND settled → both published. */
    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_HEAVE_VALID;
    n = mqtt_build_state(m, 24, &d, "imud", true, true);
    const mqtt_msg_t *hv = find_topic(m, n, "imud/environment/heave");
    EXPECT(hv && fabs(strtod(hv->payload, NULL) - 0.42) < 1e-3, "heave 0.42 m when settled");
    const mqtt_msg_t *hr = find_topic(m, n, "imud/environment/heaveRate");
    EXPECT(hr && fabs(strtod(hr->payload, NULL) - 0.25) < 1e-3, "heaveRate 0.25 m/s when settled");
    end(fb);
}

static void test_wave_gated(void)
{
    begin("test_wave_gated");
    int fb = g_fail;

    imud_data_t d = make_data();
    d.wave_height_m = 1.2f;
    d.wave_period_s = 5.5f;
    d.roll_period_s = 4.0f;
    d.roll_amplitude = 0.1f;
    d.pitch_period_s = 5.0f;
    d.pitch_amplitude = 0.05f;
    mqtt_msg_t m[24];

    /* Suppressed until WAVE_VALID even with heave settled and publishing. */
    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_HEAVE_VALID;
    int n = mqtt_build_state(m, 24, &d, "imud", true, true);
    EXPECT(find_topic(m, n, "imud/environment/waveHeight") == NULL,
           "waveHeight suppressed until WAVE_VALID");

    /* Off when emit_heave=false even if valid (rides publish_heave). */
    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_HEAVE_VALID | IMUD_FLAG_WAVE_VALID;
    n = mqtt_build_state(m, 24, &d, "imud", false, true);
    EXPECT(find_topic(m, n, "imud/environment/waveHeight") == NULL,
           "waveHeight absent when emit_heave=false");

    /* On when valid: values in SI, period precision 0.1 s. */
    n = mqtt_build_state(m, 24, &d, "imud", true, true);
    const mqtt_msg_t *wh = find_topic(m, n, "imud/environment/waveHeight");
    EXPECT(wh && fabs(strtod(wh->payload, NULL) - 1.2) < 1e-2, "waveHeight 1.2 m");
    const mqtt_msg_t *wp = find_topic(m, n, "imud/environment/wavePeriod");
    EXPECT(wp && fabs(strtod(wp->payload, NULL) - 5.5) < 1e-2, "wavePeriod 5.5 s");
    const mqtt_msg_t *rp = find_topic(m, n, "imud/environment/rollPeriod");
    EXPECT(rp && fabs(strtod(rp->payload, NULL) - 4.0) < 1e-2, "rollPeriod 4.0 s");
    /* amplitudes are angles: deg mode converts (0.1 rad = 5.73 deg) */
    const mqtt_msg_t *ra = find_topic(m, n, "imud/environment/rollAmplitude");
    EXPECT(ra && fabs(strtod(ra->payload, NULL) - 5.73) < 1e-2, "rollAmplitude 0.1 rad -> 5.73 deg");
    const mqtt_msg_t *pp = find_topic(m, n, "imud/environment/pitchPeriod");
    EXPECT(pp && fabs(strtod(pp->payload, NULL) - 5.0) < 1e-2, "pitchPeriod 5.0 s");
    EXPECT(n == 16, "16 state msgs with heave family (6 wave), no declination");
    end(fb);
}

static void test_engine_binary(void)
{
    begin("test_engine_binary");
    int fb = g_fail;

    imud_data_t d = make_data();
    mqtt_msg_t m[24];

    /* Always published; OFF without the flag, ON with it. */
    int n = mqtt_build_state(m, 24, &d, "imud", false, true);
    const mqtt_msg_t *e = find_topic(m, n, "imud/engine/running");
    EXPECT(e && strcmp(e->payload, "OFF") == 0, "engine OFF without flag");

    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_ENGINE_ON;
    n = mqtt_build_state(m, 24, &d, "imud", false, true);
    e = find_topic(m, n, "imud/engine/running");
    EXPECT(e && strcmp(e->payload, "ON") == 0, "engine ON with flag");

    /* Discovery: binary_sensor config with payload map + device class. */
    n = mqtt_build_discovery(m, 24, "imud", "homeassistant", "imud", false, true, 0);
    const mqtt_msg_t *cfg = find_topic(m, n, "homeassistant/binary_sensor/imud_engine/config");
    EXPECT(cfg != NULL, "engine binary_sensor discovery present");
    EXPECT(cfg && strstr(cfg->payload, "\"pl_on\":\"ON\"") != NULL, "pl_on present");
    EXPECT(cfg && strstr(cfg->payload, "\"dev_cla\":\"running\"") != NULL, "device_class running");
    EXPECT(cfg && strstr(cfg->payload, "\"stat_t\":\"imud/engine/running\"") != NULL, "state topic wired");
    end(fb);
}

static void test_prefix_and_count_cap(void)
{
    begin("test_prefix_and_count_cap");
    int fb = g_fail;

    imud_data_t d = make_data();
    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_DECLINATION_VALID | IMUD_FLAG_HEAVE_VALID;
    mqtt_msg_t m[24];

    int n = mqtt_build_state(m, 24, &d, "boat", true, true);
    EXPECT(n == 12, "12 state msgs with declination + heave family (no WAVE_VALID)");
    EXPECT(find_topic(m, n, "boat/attitude/yaw") != NULL, "custom prefix applied");

    /* max caps the count, no overflow */
    int c = mqtt_build_state(m, 3, &d, "imud", true, true);
    EXPECT(c == 3, "count capped at max");
    end(fb);
}

/*
 * The topic prefix lands inside JSON strings in a discovery config, and the
 * client id in Home Assistant's discovery topic, which takes [A-Za-z0-9_-].
 */
static void test_discovery_names_are_safe(void)
{
    begin("test_discovery_names_are_safe");
    int fb = g_fail;

    char node[64];
    mqtt_node_id(node, sizeof node, "aft nav/1.b");
    EXPECT(strcmp(node, "aft_nav_1_b") == 0, "the node id keeps only what HA takes");
    mqtt_node_id(node, sizeof node, "");
    EXPECT(strcmp(node, "imud") == 0, "an empty client id is imud");

    mqtt_msg_t m[24];
    int n = mqtt_build_discovery(m, 24, "boat \"one\"", "homeassistant", "imud",
                                 false, true, 0);
    const mqtt_msg_t *d = find_topic(m, n, "homeassistant/sensor/imud_roll/config");
    EXPECT(d && strstr(d->payload,
                       "\"stat_t\":\"boat \\\"one\\\"/attitude/roll\"") != NULL,
           "the prefix is escaped in the state topic");
    EXPECT(d && strstr(d->payload,
                       "\"avty_t\":\"boat \\\"one\\\"/status/online\"") != NULL,
           "and in the availability topic");
    end(fb);
}

static void test_discovery(void)
{
    begin("test_discovery");
    int fb = g_fail;

    mqtt_msg_t m[24];
    int n = mqtt_build_discovery(m, 24, "imud", "homeassistant", "imud", false, true, 0);
    EXPECT(n == 9, "9 discovery configs (no heave family; engine binary_sensor always)");

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
    n = mqtt_build_discovery(m, 24, "imud", "homeassistant", "imud", true, true, 0);
    EXPECT(n == 17, "17 discovery configs (16 sensors + engine binary_sensor)");
    EXPECT(find_topic(m, n, "homeassistant/sensor/imud_heave/config") != NULL, "heave discovery when on");
    const mqtt_msg_t *dhr = find_topic(m, n, "homeassistant/sensor/imud_heave_rate/config");
    EXPECT(dhr != NULL, "heaveRate discovery when on");
    EXPECT(dhr && strstr(dhr->payload, "\"unit_of_meas\":\"m/s\"") != NULL, "heaveRate unit m/s");

    /* rad mode advertises radian units */
    mqtt_build_discovery(m, 24, "imud", "homeassistant", "imud", true, false, 0);
    const mqtt_msg_t *dy = find_topic(m, 17, "homeassistant/sensor/imud_yaw/config");
    EXPECT(dy && strstr(dy->payload, "\"unit_of_meas\":\"rad\"") != NULL, "radian unit in rad mode");
    end(fb);
}

/*
 * MQTT topics are retained, so a heading published once and then withheld
 * would be handed to every later subscriber as if current.  The heading
 * topics stop, and navigation/headingReferenced — published always — is what
 * distinguishes "withheld" from "the bridge died".
 */
static void test_heading_withheld_without_mag(void)
{
    begin("test_heading_withheld_without_mag");
    int fb = g_fail;

    mqtt_msg_t m[24];
    imud_data_t d = make_data();
    d.flags = IMUD_FLAG_DECLINATION_VALID;     /* no MAG_VALID, no MAG_UNCAL */
    d.flags_ext = IMUD_FLAG_EXT_MAG_ABSENT;
    d.declination_deg = 13.2f;
    int n = mqtt_build_state(m, 24, &d, "imud", false, true);

    EXPECT(find_topic(m, n, "imud/navigation/headingMagnetic") == NULL,
           "headingMagnetic withheld");
    EXPECT(find_topic(m, n, "imud/navigation/headingTrue") == NULL,
           "headingTrue withheld even with declination");
    EXPECT(find_topic(m, n, "imud/navigation/magneticVariation") == NULL,
           "magneticVariation withheld");

    const mqtt_msg_t *r = find_topic(m, n, "imud/navigation/headingReferenced");
    EXPECT(r != NULL, "headingReferenced published");
    EXPECT(r && strcmp(r->payload, "OFF") == 0, "headingReferenced = OFF");
    EXPECT(find_topic(m, n, "imud/attitude/yaw") != NULL, "attitude/yaw still sent");
    EXPECT(find_topic(m, n, "imud/navigation/rateOfTurn") != NULL,
           "rateOfTurn still sent");
    end(fb);
}

/*
 * Withholding a topic leaves its last retained value on the broker, where it
 * is handed to every later subscriber as current.  The topics to wipe are the
 * ones that were being published and now are not — and, at the first look,
 * every one being withheld, since an earlier run may have left them.
 */
static void test_withheld_topics_are_cleared(void)
{
    begin("test_withheld_topics_are_cleared");
    int fb = g_fail;

    mqtt_msg_t m[24];
    mqtt_retained_t kept = {0};
    imud_data_t d = make_data();    /* magnetometer fused; no declination, no heave */

    /* First look: declination ×2, heave ×2 and sea state ×6 are withheld. */
    int n = mqtt_build_clears(m, 24, &kept, &d, "imud", false);
    EXPECT(n == 10, "every withheld topic is cleared at the first look");
    EXPECT(find_topic(m, n, "imud/navigation/headingTrue") != NULL,
           "headingTrue among them");
    EXPECT(find_topic(m, n, "imud/environment/heave") != NULL, "and heave");
    EXPECT(find_topic(m, n, "imud/navigation/headingMagnetic") == NULL,
           "a topic that is being published is left alone");
    EXPECT(find_topic(m, n, "imud/attitude/roll") == NULL,
           "and so is one that is never withheld");
    bool empty = true;
    for (int i = 0; i < n; i++) if (m[i].payload[0] != '\0') empty = false;
    EXPECT(empty, "each clear is an empty payload");

    EXPECT(mqtt_build_clears(m, 24, &kept, &d, "imud", false) == 0,
           "nothing changed, nothing to clear");

    /* The magnetometer stops being fused. */
    d.flags = 0;
    n = mqtt_build_clears(m, 24, &kept, &d, "imud", false);
    EXPECT(n == 1 && find_topic(m, n, "imud/navigation/headingMagnetic") != NULL,
           "headingMagnetic is cleared when it stops being published");
    EXPECT(mqtt_build_clears(m, 24, &kept, &d, "imud", false) == 0, "once");

    /* Fused again with a declination, then the magnetometer goes: the true
     * heading and the variation are gated on it too. */
    d.flags = IMUD_FLAG_MAG_VALID | IMUD_FLAG_DECLINATION_VALID;
    EXPECT(mqtt_build_clears(m, 24, &kept, &d, "imud", false) == 0,
           "nothing to clear while they publish");
    d.flags = IMUD_FLAG_DECLINATION_VALID;
    n = mqtt_build_clears(m, 24, &kept, &d, "imud", false);
    EXPECT(n == 3 &&
           find_topic(m, n, "imud/navigation/headingMagnetic")   != NULL &&
           find_topic(m, n, "imud/navigation/headingTrue")       != NULL &&
           find_topic(m, n, "imud/navigation/magneticVariation") != NULL,
           "all three heading topics go with the magnetometer");

    /* Heave settles, then the estimator restarts. */
    d.flags = IMUD_FLAG_HEAVE_VALID;
    EXPECT(mqtt_build_clears(m, 24, &kept, &d, "imud", true) == 0,
           "heave is published once it has settled");
    d.flags = 0;
    n = mqtt_build_clears(m, 24, &kept, &d, "imud", true);
    EXPECT(n == 2 && find_topic(m, n, "imud/environment/heaveRate") != NULL,
           "and both its topics are cleared when it is withdrawn");

    /* After a reconnect the broker may never have had any of it. */
    kept.primed = false;
    EXPECT(mqtt_build_clears(m, 24, &kept, &d, "imud", true) == 11,
           "a reconnect clears every withheld topic again");
    end(fb);
}

/*
 * Home Assistant ignores an empty payload, so a cleared topic does not stop
 * it showing the last value.  expire_after is what does.
 */
static void test_discovery_expires(void)
{
    begin("test_discovery_expires");
    int fb = g_fail;

    mqtt_msg_t m[24];
    int n = mqtt_build_discovery(m, 24, "imud", "homeassistant", "imud",
                                 false, true, 12);
    const mqtt_msg_t *h = find_topic(m, n,
        "homeassistant/sensor/imud_heading_magnetic/config");
    EXPECT(h && strstr(h->payload, "\"exp_aft\":12,") != NULL,
           "a sensor's config carries the expiry");
    const mqtt_msg_t *e = find_topic(m, n,
        "homeassistant/binary_sensor/imud_engine/config");
    EXPECT(e && strstr(e->payload, "\"exp_aft\":12,") != NULL,
           "and so does the engine binary sensor's");
    EXPECT(h && h->payload[strlen(h->payload) - 1] == '}',
           "the config still ends where it should");

    n = mqtt_build_discovery(m, 24, "imud", "homeassistant", "imud",
                             false, true, 0);
    h = find_topic(m, n, "homeassistant/sensor/imud_heading_magnetic/config");
    EXPECT(h && strstr(h->payload, "exp_aft") == NULL, "0 leaves the key out");
    end(fb);
}

/* An uncalibrated fuse still publishes, and says so. */
static void test_heading_published_when_uncal(void)
{
    begin("test_heading_published_when_uncal");
    int fb = g_fail;

    mqtt_msg_t m[24];
    imud_data_t d = make_data();
    d.flags = IMUD_FLAG_MAG_UNCAL;
    int n = mqtt_build_state(m, 24, &d, "imud", false, true);

    EXPECT(find_topic(m, n, "imud/navigation/headingMagnetic") != NULL,
           "headingMagnetic sent on an uncalibrated fuse");
    const mqtt_msg_t *r = find_topic(m, n, "imud/navigation/headingReferenced");
    EXPECT(r && strcmp(r->payload, "ON") == 0, "headingReferenced = ON");
    end(fb);
}

int main(void)
{
    puts("=== imud mqtt builder tests ===");
    test_state_deg();
    test_state_rad();
    test_declination_gated();
    test_heave_gated();
    test_wave_gated();
    test_engine_binary();
    test_prefix_and_count_cap();
    test_heading_withheld_without_mag();
    test_heading_published_when_uncal();
    test_withheld_topics_are_cleared();
    test_discovery_expires();
    test_discovery();
    test_discovery_names_are_safe();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
