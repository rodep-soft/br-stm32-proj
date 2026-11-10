#ifndef IMU_SAMPLES_H
#define IMU_SAMPLES_H

#include <stdbool.h>
#include <stdint.h>
#include "sh2_SensorValue.h"
#include "bno085_can.h"

#define IMU_MAX_SAMPLE_AGE_MS 100U
#define IMU_REPORT_INTERVAL_US 5000U

typedef struct {
    struct bno085_can_bno085_data_t data;
    uint32_t accel_ms, gyro_ms;
    uint8_t accel_seq, gyro_seq;
    bool accel_seen, gyro_seen;
    bool accel_pending, gyro_pending;
} imu_samples_t;

void imu_samples_reset(imu_samples_t *samples);
/* now_us must use the same free-running clock as the SH-2 transport. */
void imu_samples_update(imu_samples_t *samples, const sh2_SensorValue_t *value,
                        uint32_t now_ms, uint32_t now_us);
extern volatile uint32_t imu_stale_reports, imu_invalid_reports;
bool imu_samples_pack(const imu_samples_t *samples, uint32_t now_ms,
                      uint8_t data[BNO085_CAN_BNO085_DATA_LENGTH]);
void imu_samples_sent(imu_samples_t *samples);

#endif
