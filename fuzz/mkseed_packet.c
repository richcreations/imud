/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * mkseed_packet.c — regenerate test/fuzz/corpus/packet/valid_v<N>.bin.
 *
 * The packet fuzz target only reaches its decode path through packet_ok(),
 * which checks size, magic and version — so a seed from an older wire
 * revision is inert and the fuzzer starts from nothing useful. That had
 * already happened once (a v14/260-byte seed survived the v17 bump
 * unnoticed), so the seed is now generated from packet_build() itself and
 * guarded by test_packet's seed-validity check, which fails loudly on the
 * next wire change.
 *
 *   make fuzz-seeds        # rebuild the corpus seed for the current wire
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "packet.h"

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: mkseed_packet <out.bin>\n"); return 2; }
    fused_state_t st;  memset(&st, 0, sizeof st);
    mag_sample_t  mag; memset(&mag, 0, sizeof mag);
    imu_sample_t  imu, raw;
    memset(&imu, 0, sizeof imu); memset(&raw, 0, sizeof raw);

    /* Plausible under-way values: 12 deg heel, 4 deg trim, heading 137. */
    float roll = 12.0f*(float)M_PI/180.0f, pitch = 4.0f*(float)M_PI/180.0f;
    float yaw  = 137.0f*(float)M_PI/180.0f;
    float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
    float cy=cosf(yaw/2),sy=sinf(yaw/2);
    st.q[0]=cy*cp*cr+sy*sp*sr; st.q[1]=cy*cp*sr-sy*sp*cr;
    st.q[2]=cy*sp*cr+sy*cp*sr; st.q[3]=sy*cp*cr-cy*sp*sr;
    st.roll=roll; st.pitch=pitch; st.yaw=yaw; st.heading_deg=137.0f;
    st.rate_of_turn=-3.2f; st.declination_deg=-1.4f;
    st.heave_m=0.42f; st.heave_rate=-0.31f;
    for (int i=0;i<9;i++) st.cov[i]= (i%4==0)? 3.1e-5f : 2.0e-7f;
    st.bias_gyro[0]=2.0e-3f; st.bias_gyro[1]=-1.0e-3f; st.bias_gyro[2]=1.5e-3f;
    for (int i=0;i<3;i++) st.bias_gyro_var[i]=8.0e-8f;
    st.quiescence=0.0031f;
    st.wave_height_m=1.35f; st.wave_period_s=5.1f;
    st.roll_period_s=4.6f;  st.roll_amplitude=0.21f;
    st.pitch_period_s=6.9f; st.pitch_amplitude=0.07f;
    st.mag_anomaly=0.014f;  st.mag_residual=0.021f;
    st.innov_weight=0.83f;  st.innov_reject=0.007f;
    st.nis_accel=19.3f;     st.nis_mag=1.6f;
    st.imu_seq=1234567u; st.ts_wall_ns=1785000000000000000ULL;
    st.ts_tai_ns=st.ts_wall_ns+37000000000ULL;
    st.ts_chip_ticks=987654321u; st.anchor_gen=3u;
    st.flags = FLAG_MAG_VALID|FLAG_FUSION_CONVERGED|FLAG_ACCEL_CAL|
               FLAG_GYRO_CAL|FLAG_MAG_CAL|FLAG_DECLINATION_VALID|
               FLAG_HEAVE_VALID|FLAG_WAVE_VALID;

    imu.accel[0]=0.68f; imu.accel[1]=-2.03f; imu.accel[2]=-9.58f;
    imu.gyro[0]=0.021f; imu.gyro[1]=-0.008f; imu.gyro[2]=-0.056f;
    imu.temp_c=28.5f;
    memcpy(raw.accel_raw, imu.accel, sizeof raw.accel_raw);
    memcpy(raw.gyro, imu.gyro, sizeof raw.gyro);
    mag.field[0]=21.4f; mag.field[1]=-4.8f; mag.field[2]=41.9f;
    memcpy(mag.field_raw, mag.field, sizeof mag.field_raw);
    mag.valid=true;

    imu_packet_t pkt;
    packet_build(&pkt, &st, &mag, &imu, &raw, "NED");
    uint8_t wire[IMUD_PACKET_BYTES];
    packet_encode(wire, &pkt);

    FILE *f = fopen(argv[1], "wb");
    if (!f) { perror("fopen"); return 1; }
    fwrite(wire, 1, sizeof wire, f);
    fclose(f);
    fprintf(stderr, "wrote %zu bytes, version %u\n", sizeof wire, pkt.version);
    return 0;
}
