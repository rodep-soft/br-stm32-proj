#include "imu_samples.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static sh2_SensorValue_t accel(uint8_t seq, float x, float y)
{
    sh2_SensorValue_t v = {0};
    v.sensorId = SH2_ACCELEROMETER;
    v.sequence = seq;
    v.un.accelerometer.x = x;
    v.un.accelerometer.y = y;
    v.un.accelerometer.z = 99; /* Z must never appear in the CAN payload. */
    return v;
}

static sh2_SensorValue_t gyro(uint8_t seq, float z)
{
    sh2_SensorValue_t v = {0};
    v.sensorId = SH2_GYROSCOPE_CALIBRATED;
    v.sequence = seq;
    v.un.gyroscope.x = 11;
    v.un.gyroscope.y = 22;
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
    uint8_t bytes[8];
    imu_samples_reset(&s);
    assert(!imu_samples_pack(&s, 0, bytes));
    sh2_SensorValue_t a = accel(254, 1.5f, -2.25f), g = gyro(17, -0.5f);
    update(&s, &a, 10);
    assert(!imu_samples_pack(&s, 10, bytes));
    update(&s, &g, 10); /* A second callback in the same SHTP packet. */
    const uint8_t golden[8] = {0x80, 0x01, 0xc0, 0xfd, 0xfe, 0x00, 0xff, 0x11};
    assert(imu_samples_pack(&s, 15, bytes));
    assert(memcmp(bytes, golden, sizeof(bytes)) == 0);
    assert(imu_samples_pack(&s, 16, bytes)); /* Full CAN FIFO must not consume data. */
    imu_samples_sent(&s);
    assert(!imu_samples_pack(&s, 16, bytes));
    update(&s, &a, 17);
    update(&s, &g, 17);
    assert(!imu_samples_pack(&s, 17, bytes)); /* Duplicate sequences are not new data. */

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
    a.un.accelerometer.x = 128.0f;
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

    /* Sweep all sequence values and signed boundary values against explicit LE bytes. */
    const int16_t values[] = {-32768, -32767, -513, -1, 0, 1, 511, 32767};
    for (unsigned seq = 0; seq < 256; ++seq) {
        for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
            int16_t raw_x = values[i], raw_y = values[7 - i], raw_z = values[(i + 3) % 8];
            imu_samples_reset(&s);
            a = accel((uint8_t)seq, raw_x / 256.0f, raw_y / 256.0f);
            g = gyro((uint8_t)(255 - seq), raw_z / 512.0f);
            update(&s, &g, 100);
            update(&s, &a, 100);
            assert(imu_samples_pack(&s, 100, bytes));
            const uint8_t expected[] = {
                (uint8_t)raw_x, (uint8_t)((uint16_t)raw_x >> 8),
                (uint8_t)raw_y, (uint8_t)((uint16_t)raw_y >> 8), (uint8_t)seq,
                (uint8_t)raw_z, (uint8_t)((uint16_t)raw_z >> 8), (uint8_t)(255 - seq)
            };
            assert(memcmp(bytes, expected, 8) == 0);
        }
    }
    /* A fresh delivery time must not hide an old sensor sample. */
    imu_samples_reset(&s);
    a = accel(1, 1, 2); g = gyro(1, 3);
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
    puts("PASS: CAN golden payload, 2048 boundary/sequence cases, freshness, reset, retry and invalid data");
    return 0;
}
