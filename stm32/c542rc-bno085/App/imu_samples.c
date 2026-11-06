#include "imu_samples.h"
#include <math.h>
#include <string.h>

volatile uint32_t imu_stale_reports, imu_invalid_reports;

void imu_samples_reset(imu_samples_t *s)
{
    memset(s, 0, sizeof(*s));
}

void imu_samples_update(imu_samples_t *s, const sh2_SensorValue_t *v,
                        uint32_t now_ms, uint32_t now_us)
{
    if (v->sensorId != SH2_ACCELEROMETER && v->sensorId != SH2_GYROSCOPE_CALIBRATED) return;
    /* Compare low 32 bits in the transport clock domain, including TIM2 wrap.
       Future/invalid timestamps also fail this bounded unsigned age check. */
    uint32_t age_us = now_us - (uint32_t)v->timestamp;
    if (age_us >= IMU_MAX_SAMPLE_AGE_MS * 1000U) {
        imu_stale_reports++;
        if (v->sensorId == SH2_ACCELEROMETER) s->accel_pending = false;
        else s->gyro_pending = false;
        return;
    }
    /* Round age upward: never extend the freshness deadline through rounding. */
    uint32_t sample_ms = now_ms - (age_us + 999U) / 1000U;
    if (v->sensorId == SH2_ACCELEROMETER) {
        const float x = v->un.accelerometer.x, y = v->un.accelerometer.y;
        if (!isfinite(x) || !isfinite(y) ||
            !bno085_can_bno085_data_accel_x_is_in_phys_range(x) ||
            !bno085_can_bno085_data_accel_y_is_in_phys_range(y)) {
            s->accel_pending = false;
            imu_invalid_reports++;
            return;
        }
        if (s->accel_seen && s->data.accel_seq == v->sequence) return;
        s->data.accel_x = bno085_can_bno085_data_accel_x_encode(x);
        s->data.accel_y = bno085_can_bno085_data_accel_y_encode(y);
        s->data.accel_seq = v->sequence;
        s->accel_ms = sample_ms;
        s->accel_seen = s->accel_pending = true;
    } else if (v->sensorId == SH2_GYROSCOPE_CALIBRATED) {
        const float z = v->un.gyroscope.z;
        if (!isfinite(z) || !bno085_can_bno085_data_gyro_z_is_in_phys_range(z)) {
            s->gyro_pending = false;
            imu_invalid_reports++;
            return;
        }
        if (s->gyro_seen && s->data.gyro_seq == v->sequence) return;
        s->data.gyro_z = bno085_can_bno085_data_gyro_z_encode(z);
        s->data.gyro_seq = v->sequence;
        s->gyro_ms = sample_ms;
        s->gyro_seen = s->gyro_pending = true;
    }
}

bool imu_samples_pack(const imu_samples_t *s, uint32_t now_ms, uint8_t data[8])
{
    if (!s->accel_pending || !s->gyro_pending ||
        (uint32_t)(now_ms - s->accel_ms) >= IMU_MAX_SAMPLE_AGE_MS ||
        (uint32_t)(now_ms - s->gyro_ms) >= IMU_MAX_SAMPLE_AGE_MS) return false;
    return bno085_can_bno085_data_pack(data, &s->data, 8) == 8;
}

void imu_samples_sent(imu_samples_t *s)
{
    s->accel_pending = s->gyro_pending = false;
}
