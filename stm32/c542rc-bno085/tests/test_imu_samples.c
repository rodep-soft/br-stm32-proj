#include "imu_samples.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static sh2_SensorValue_t accel(uint8_t seq, float x, float y, float z)
{
    sh2_SensorValue_t v = {0};
    v.sensorId = SH2_ACCELEROMETER;
    v.sequence = seq;
    v.un.accelerometer.x = x;
    v.un.accelerometer.y = y;
    v.un.accelerometer.z = z;
    return v;
}

static sh2_SensorValue_t gyro(uint8_t seq, float x, float y, float z)
{
    sh2_SensorValue_t v = {0};
    v.sensorId = SH2_GYROSCOPE_CALIBRATED;
    v.sequence = seq;
    v.un.gyroscope.x = x;
    v.un.gyroscope.y = y;
    v.un.gyroscope.z = z;
    return v;
}

static void update(imu_samples_t *s, sh2_SensorValue_t *v, uint32_t now_ms)
{
    uint32_t now_us = now_ms * 1000U;
    v->timestamp = now_us;
    imu_samples_update(s, v, now_ms, now_us);
}

int main(void)
{
    imu_samples_t s;
    uint8_t bytes[BNO085_CAN_BNO085_DATA_LENGTH];
    imu_samples_reset(&s);
    assert(!imu_samples_pack(&s, 0, bytes));
    sh2_SensorValue_t a = accel(254, 1.5f, -2.25f, 9.8f), g = gyro(17, 0.25f, -0.125f, -0.5f);
    update(&s, &a, 10);
    assert(!imu_samples_pack(&s, 10, bytes));
    update(&s, &g, 10); /* A second callback in the same SHTP packet. */

    /* timestamp_ms=15 (0x0000000f), seq=0, status=1 (OK),
       accel=(1500, -2250, 9800), gyro=(250, -125, -500), padded to 20B. */
    const uint8_t golden[BNO085_CAN_BNO085_DATA_LENGTH] = {
        0x0f, 0x00, 0x00, 0x00, /* timestamp_ms = 15 */
        0x00,                   /* seq = 0 */
        0x01,                   /* status = 1 (OK) */
        0xdc, 0x05,             /* accel_x = 1500 */
        0x36, 0xf7,             /* accel_y = -2250 */
        0x48, 0x26,             /* accel_z = 9800 */
        0xfa, 0x00,             /* gyro_x = 250 */
        0x83, 0xff,             /* gyro_y = -125 */
        0x0c, 0xfe,             /* gyro_z = -500 */
        0x00, 0x00              /* 20-byte CAN FD padding */
    };
    memset(bytes, 0xaa, sizeof(bytes));
    assert(imu_samples_pack(&s, 15, bytes));
    assert(memcmp(bytes, golden, sizeof(bytes)) == 0);
    assert(imu_samples_pack(&s, 16, bytes)); /* Full CAN FIFO must not consume data or advance seq. */
    assert(bytes[4] == 0x00 && bytes[0] == 0x10);
    imu_samples_sent(&s);
    assert(s.data.seq == 1);
    assert(!imu_samples_pack(&s, 16, bytes));
    update(&s, &a, 17);
    update(&s, &g, 17);
    assert(!imu_samples_pack(&s, 17, bytes)); /* Duplicate sensor sequences are not new data. */

    a.sequence = 255; g.sequence = 18;
    update(&s, &a, UINT32_MAX - 2);
    update(&s, &g, UINT32_MAX - 2);
    assert(imu_samples_pack(&s, 3, bytes)); /* Host tick rollover. */
    assert(!imu_samples_pack(&s, 100, bytes)); /* Expired data cannot be sent. */
    imu_samples_sent(&s);
    a.sequence = 0; g.sequence = 19;
    update(&s, &a, 101);
    update(&s, &g, 101);
    assert(imu_samples_pack(&s, 101, bytes)); /* Sensor sequence rollover. */
    a.sequence = 1; a.un.accelerometer.x = NAN;
    update(&s, &a, 102);
    assert(!imu_samples_pack(&s, 102, bytes));
    a.un.accelerometer.x = 32.768f;
    update(&s, &a, 103);
    assert(!imu_samples_pack(&s, 103, bytes));
    a.un.accelerometer.x = 1.5f;
    update(&s, &a, 104);
    assert(imu_samples_pack(&s, 104, bytes));
    g.sequence = 20; g.un.gyroscope.z = INFINITY;
    update(&s, &g, 104);
    assert(!imu_samples_pack(&s, 104, bytes));
    imu_samples_reset(&s);
    assert(!imu_samples_pack(&s, 104, bytes));

    /* Sweep all frame sequence values and signed boundary values against explicit LE bytes. */
    const int16_t values[] = {-32768, -32767, -513, -1, 0, 1, 511, 32767};
    imu_samples_reset(&s);
    for (unsigned seq = 0; seq < 256; ++seq) {
        for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
            int16_t raw_ax = values[i], raw_ay = values[7 - i], raw_az = values[(i + 2) % 8];
            int16_t raw_gx = values[(i + 1) % 8], raw_gy = values[(i + 5) % 8], raw_gz = values[(i + 3) % 8];
            uint32_t ts = 100U + seq * 8U + i;
            uint8_t s_seq = (uint8_t)(seq * 8U + i + 1U);
            if (s.accel_seen && s_seq == s.accel_seq) s_seq++;
            a = accel(s_seq, raw_ax * 0.001f, raw_ay * 0.001f, raw_az * 0.001f);
            g = gyro(s_seq, raw_gx * 0.001f, raw_gy * 0.001f, raw_gz * 0.001f);
            update(&s, &g, ts);
            update(&s, &a, ts);
            uint8_t expected_frame_seq = s.data.seq;
            assert(imu_samples_pack(&s, ts, bytes));
            const uint8_t expected[BNO085_CAN_BNO085_DATA_LENGTH] = {
                (uint8_t)ts, (uint8_t)(ts >> 8), (uint8_t)(ts >> 16), (uint8_t)(ts >> 24),
                expected_frame_seq, BNO085_CAN_STATUS_OK,
                (uint8_t)raw_ax, (uint8_t)((uint16_t)raw_ax >> 8),
                (uint8_t)raw_ay, (uint8_t)((uint16_t)raw_ay >> 8),
                (uint8_t)raw_az, (uint8_t)((uint16_t)raw_az >> 8),
                (uint8_t)raw_gx, (uint8_t)((uint16_t)raw_gx >> 8),
                (uint8_t)raw_gy, (uint8_t)((uint16_t)raw_gy >> 8),
                (uint8_t)raw_gz, (uint8_t)((uint16_t)raw_gz >> 8),
                0x00, 0x00
            };
            assert(memcmp(bytes, expected, sizeof(expected)) == 0);
            struct bno085_can_bno085_data_t unpacked = {0};
            assert(bno085_can_bno085_data_unpack(&unpacked, bytes, BNO085_CAN_BNO085_DATA_PAYLOAD_LENGTH) == 0);
            assert(unpacked.timestamp_ms == ts && unpacked.seq == expected_frame_seq &&
                   unpacked.status == BNO085_CAN_STATUS_OK &&
                   unpacked.accel_x == raw_ax && unpacked.accel_y == raw_ay && unpacked.accel_z == raw_az &&
                   unpacked.gyro_x == raw_gx && unpacked.gyro_y == raw_gy && unpacked.gyro_z == raw_gz);
            imu_samples_sent(&s);
        }
    }
    /* A fresh delivery time must not hide an old sensor sample. */
    imu_samples_reset(&s);
    a = accel(1, 1, 2, 3); g = gyro(1, 1, 2, 3);
    a.timestamp = 900000; g.timestamp = 999000;
    uint32_t stale_before = imu_stale_reports;
    imu_samples_update(&s, &a, 1000, 1000000);
    imu_samples_update(&s, &g, 1000, 1000000);
    assert(imu_stale_reports == stale_before + 1 && !s.accel_seen);
    assert(!imu_samples_pack(&s, 1000, bytes));
    a.timestamp = 901001;
    imu_samples_update(&s, &a, 1000, 1000000);
    assert(s.accel_ms == 901 && imu_samples_pack(&s, 1000, bytes));
    assert(!imu_samples_pack(&s, 1001, bytes)); /* Deadline from measurement. */
    a.sequence++; a.timestamp = 1000001; /* Future/invalid time is rejected. */
    imu_samples_update(&s, &a, 1000, 1000000);
    assert(imu_stale_reports == stale_before + 2 && !s.accel_pending);

    /* TIM2 us rollover and HAL ms rollover are independent clock domains. */
    imu_samples_reset(&s);
    a.timestamp = (uint64_t)UINT32_MAX - 499;
    g.timestamp = (UINT64_C(1) << 32) + 250;
    imu_samples_update(&s, &a, 0, 500);
    imu_samples_update(&s, &g, 0, 500);
    assert(s.accel_ms == UINT32_MAX && s.gyro_ms == UINT32_MAX);
    assert(imu_samples_pack(&s, 0, bytes));
    assert(!imu_samples_pack(&s, 99, bytes));
    puts("PASS: CAN FD 0x500 18B/20B golden payload, 2048 boundary/sequence cases, freshness, reset, retry and invalid data");
    return 0;
}
