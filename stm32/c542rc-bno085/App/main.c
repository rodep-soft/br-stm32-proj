#include "mx_system.h"
#include "mx_fdcan1.h"
#include "mx_spi1.h"
#include "mx_usart2.h"
#include "bno_transport.h"
#include "dma_io.h"
#include "watchdog.h"
#include "can_tx.h"
#include "imu_samples.h"
#include "sh2.h"
#include "sh2_err.h"
#include <stdio.h>

#define SENSOR_TIMEOUT_MS 1000U
#define RETRY_INTERVAL_MS 1000U

static imu_samples_t samples;
static bool sensor_reset;
static bool gyro_accuracy_valid;

/* Debugger-visible counters; none are added to the CAN payload. */
volatile uint32_t bno_init_failures, bno_recoveries;
volatile uint32_t accel_interval_us, gyro_interval_us, shtp_events, bno_init_stage;
volatile int bno_config_status;
volatile uint32_t bno_config_retries;
volatile uint32_t accel_reports, gyro_reports;
volatile uint32_t accel_accuracy, gyro_accuracy;
volatile uint32_t calibration_before, calibration_enabled;
volatile uint32_t can_bus_off_events, can_last_error, can_protocol_status, can_error_counters;
volatile system_status_t startup_status;

static void async_event(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (!event) return;
    if (event->eventId == SH2_RESET) sensor_reset = true;
    else if (event->eventId == SH2_SHTP_EVENT) shtp_events++;
    else if (event->eventId == SH2_GET_FEATURE_RESP) {
        const sh2_SensorConfigResp_t *response = &event->sh2SensorConfigResp;
        if (response->sensorId == SH2_ACCELEROMETER)
            accel_interval_us = response->sensorConfig.reportInterval_us;
        if (response->sensorId == SH2_GYROSCOPE_CALIBRATED)
            gyro_interval_us = response->sensorConfig.reportInterval_us;
    }
}

static void sensor_event(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) == SH2_OK) {
        if (value.sensorId == SH2_ACCELEROMETER) { accel_reports++; accel_accuracy = value.status & 3U; }
        if (value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            gyro_reports++;
            uint32_t accuracy = value.status & 3U;
            if (!gyro_accuracy_valid || accuracy != gyro_accuracy)
                printf("Gyro accuracy ms=%lu state=%lu z_q9=%ld\r\n", (unsigned long)HAL_GetTick(),
                       (unsigned long)accuracy, (long)(value.un.gyroscope.z * 512.0f));
            gyro_accuracy = accuracy;
            gyro_accuracy_valid = true;
        }
        imu_samples_update(&samples, &value, HAL_GetTick(), app_time_us());
    }
}

static bool configure_sensor(sh2_SensorId_t id, const sh2_SensorConfig_t *config,
                             volatile uint32_t *actual_interval)
{
    *actual_interval = 0;
    /* Set Feature is idempotent but asynchronous. A lost command/response must
       not be reported as ready. Retry locally before resetting the whole hub. */
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        if (attempt) bno_config_retries++;
        bno_config_status = sh2_setSensorConfig(id, config);
        if (bno_config_status != SH2_OK) return false;
        uint32_t start = HAL_GetTick();
        while (!*actual_interval) {
            sh2_service();
            if (sensor_reset || bno_transport_take_error()) return false;
            if ((uint32_t)(HAL_GetTick() - start) >= 250U) break;
        }
        if (*actual_interval) return true;
    }
    return false;
}

static bool start_sensor(void)
{
    sh2_close();
    imu_samples_reset(&samples);
    sensor_reset = false;
    accel_interval_us = gyro_interval_us = 0;
    accel_accuracy = gyro_accuracy = 0;
    gyro_accuracy_valid = false;
    bno_init_stage = 1;
    if (sh2_open(bno_transport_get(), async_event, NULL) != SH2_OK) return false;
    sh2_ProductIds_t ids = {0};
    bno_init_stage = 2;
    if (sh2_getProdIds(&ids) != SH2_OK || ids.numEntries == 0) return false;
    /* Calibration controls are volatile across BNO resets. Retain all existing
       BNO085 calibration enables and add only dynamic gyro calibration. */
    uint8_t cal = 0;
    bno_init_stage = 9;
    bno_config_status = sh2_getCalConfig(&cal);
    if (bno_config_status != SH2_OK) return false;
    calibration_before = cal;
    uint8_t requested_cal = cal | SH2_CAL_GYRO;
    bno_init_stage = 10;
    bno_config_status = sh2_setCalConfig(requested_cal);
    if (bno_config_status != SH2_OK) return false;
    bno_init_stage = 11;
    bno_config_status = sh2_getCalConfig(&cal);
    if (bno_config_status != SH2_OK) return false;
    calibration_enabled = cal;
    if (cal != requested_cal) { bno_config_status = SH2_ERR_HUB; return false; }
    printf("Calibration before=%02lx enabled=%02lx gyro=ON (verified)\r\n",
           (unsigned long)calibration_before, (unsigned long)calibration_enabled);
    if (sh2_setSensorCallback(sensor_event, NULL) != SH2_OK) return false;

    /* The initial reset event belongs to this startup, not a later recovery. */
    sensor_reset = false;
    sh2_SensorConfig_t config = {0};
    config.reportInterval_us = IMU_REPORT_INTERVAL_US;
    bno_init_stage = 3;
    if (!configure_sensor(SH2_ACCELEROMETER, &config, &accel_interval_us)) return false;
    bno_init_stage = 4;
    if (!configure_sensor(SH2_GYROSCOPE_CALIBRATED, &config, &gyro_interval_us)) return false;

    sh2_SensorConfig_t actual = {0};
    bno_init_stage = 5;
    bno_config_status = sh2_getSensorConfig(SH2_ACCELEROMETER, &actual);
    if (bno_config_status != SH2_OK || !actual.reportInterval_us) return false;
    accel_interval_us = actual.reportInterval_us;
    bno_init_stage = 6;
    bno_config_status = sh2_getSensorConfig(SH2_GYROSCOPE_CALIBRATED, &actual);
    if (bno_config_status != SH2_OK || !actual.reportInterval_us) return false;
    gyro_interval_us = actual.reportInterval_us;
    bno_init_stage = 7;
    uint32_t start = HAL_GetTick();
    for (;;) {
        sh2_service();
        uint32_t now = HAL_GetTick();
        if (sensor_reset || bno_transport_take_error()) return false;
        if (samples.accel_seen && samples.gyro_seen &&
            (uint32_t)(now - samples.accel_ms) < IMU_MAX_SAMPLE_AGE_MS &&
            (uint32_t)(now - samples.gyro_ms) < IMU_MAX_SAMPLE_AGE_MS) break;
        if ((uint32_t)(now - start) >= SENSOR_TIMEOUT_MS) return false;
    }
    bno_init_stage = 8;
    bno_max_int_latency_us = 0; /* Measure steady operation separately from boot. */
    printf("BNO085 ready (DMA): requested=5000us actual accel=%lu gyro=%lu us; CANFD=0x500/18(20)/1Mbps\r\n",
           (unsigned long)accel_interval_us, (unsigned long)gyro_interval_us);
    return true;
}

#if defined(BNO_HARDWARE_SELFTEST)
/* Deliberately disrupt the bench only in an explicitly selected test build.
   After IWDG resets us, skip injections so recovery can be observed. */
static void hardware_selftest(uint32_t now)
{
    static unsigned phase;
    if (startup_reset_flags & RCC_RSR_IWDGRSTF) return;
    if (phase == 0 && now >= 5000U) {
        printf("SELFTEST CAN standby\r\n");
        HAL_GPIO_WritePin(HAL_GPIOE, HAL_GPIO_PIN_2, HAL_GPIO_PIN_SET);
        phase++;
    } else if (phase == 1 && now >= 6500U) {
        HAL_GPIO_WritePin(HAL_GPIOE, HAL_GPIO_PIN_2, HAL_GPIO_PIN_RESET);
        printf("SELFTEST CAN resume\r\n");
        phase++;
    } else if (phase == 2 && now >= 10000U) {
        printf("SELFTEST sensor reset event\r\n");
        sensor_reset = true;
        phase++;
    } else if (phase == 3 && now >= 15000U) {
        printf("SELFTEST main stalled; expect IWDG reset\r\n");
        HAL_Delay(20); /* Allow the UART DMA log to finish. */
        for (;;) {} /* SysTick/IRQs still run, but cannot feed the watchdog. */
    }
}
#endif

int main(void)
{
    watchdog_start();
    startup_status = mx_system_init();
    if (startup_status != SYSTEM_OK) for (;;) {}
    if (!dma_io_init()) for (;;) {}
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nNUCLEO-C542RC BNO085 SPI-to-CAN\r\n");
    printf("Reset flags=%08lx IWDG=%u; watchdog nominal=8s\r\n",
           (unsigned long)startup_reset_flags, (unsigned)!!(startup_reset_flags & RCC_RSR_IWDGRSTF));
    if (HAL_FDCAN_Start(mx_fdcan1_gethandle()) != HAL_OK) {
        printf("FDCAN start failed\r\n");
        for (;;) {}
    }

    bool online = false;
    bool bus_off = false;
    unsigned consecutive_init_failures = 0;
    uint32_t retry_interval = RETRY_INTERVAL_MS, last_status = HAL_GetTick();
    uint32_t last_retry = HAL_GetTick() - RETRY_INTERVAL_MS;
    uint32_t online_since = 0, last_tx = 0, last_can_check = 0;
    for (;;) {
        /* Only the main application feeds IWDG; a hung ISR or SH-2 call cannot. */
        watchdog_feed();
        uint32_t now = HAL_GetTick();
#if defined(BNO_HARDWARE_SELFTEST)
        hardware_selftest(now);
#endif
        log_service();
        can_tx_service(now, !online);
        if ((uint32_t)(now - last_can_check) >= 100U) {
            last_can_check = now;
            /* PSR reads clear LEC and ECR reads clear CEL: capture before Recover. */
            can_protocol_status = FDCAN1->PSR;
            can_error_counters = FDCAN1->ECR;
            uint32_t lec = can_protocol_status & FDCAN_PSR_LEC;
            if (lec && lec != 7U) can_last_error = lec;
            bool current_bus_off = (can_protocol_status & FDCAN_PSR_BO) != 0;
            if (current_bus_off && !bus_off) can_bus_off_events++;
            bus_off = current_bus_off;
            (void)HAL_FDCAN_Recover(mx_fdcan1_gethandle());
        }
        if ((uint32_t)(now - last_status) >= 5000U) {
            last_status = now;
            printf("Health ms=%lu online=%u a=%lu g=%lu acc=%lu/%lu sent=%lu expired=%lu queued=%lu unsent=%lu "
                   "busoff=%lu lec=%lu psr=%08lx ecr=%08lx stale=%lu invalid=%lu "
                   "initfail=%lu recover=%lu spi=%u uart=%lu drop=%lu stack_free=%lu\r\n",
                   (unsigned long)now, (unsigned)online, (unsigned long)accel_reports,
                   (unsigned long)gyro_reports, (unsigned long)accel_accuracy, (unsigned long)gyro_accuracy,
                   (unsigned long)can_sent_frames, (unsigned long)can_expired_frames,
                   (unsigned long)can_queued_frames, (unsigned long)can_unsent_frames,
                   (unsigned long)can_bus_off_events, (unsigned long)can_last_error,
                   (unsigned long)can_protocol_status, (unsigned long)can_error_counters,
                   (unsigned long)imu_stale_reports, (unsigned long)imu_invalid_reports,
                   (unsigned long)bno_init_failures, (unsigned long)bno_recoveries, bno_transport_fault,
                   (unsigned long)log_dma_errors, (unsigned long)log_dropped_bytes,
                   (unsigned long)stack_unused_bytes());
        }
        if (!online) {
            if ((uint32_t)(now - last_retry) < retry_interval) { __WFI(); continue; }
            online = start_sensor();
            last_retry = HAL_GetTick();
            if (online) {
                consecutive_init_failures = 0;
                retry_interval = RETRY_INTERVAL_MS;
                online_since = last_retry;
                last_tx = last_retry;
            } else {
                bno_init_failures++;
                if (++consecutive_init_failures >= 3) retry_interval = 5000U;
                sh2_close();
                printf("BNO085 init failed; stage=%lu status=%d fault=%u header=%08x rates=%lu/%lu; retry_ms=%lu\r\n",
                       (unsigned long)bno_init_stage, bno_config_status, bno_transport_fault, bno_last_header,
                       (unsigned long)accel_interval_us, (unsigned long)gyro_interval_us,
                       (unsigned long)retry_interval);
            }
            continue;
        }

        /* Read all reports via SH-2 callbacks, including multiple reports in one packet.
           Poll the active-low INT level so a missed edge cannot stop reception. */
        sh2_service();
        now = HAL_GetTick();
        uint32_t accel_age = now - (samples.accel_seen ? samples.accel_ms : online_since);
        uint32_t gyro_age = now - (samples.gyro_seen ? samples.gyro_ms : online_since);
        if (sensor_reset || bno_transport_take_error() ||
            accel_age >= SENSOR_TIMEOUT_MS || gyro_age >= SENSOR_TIMEOUT_MS) {
            online = false;
            bno_recoveries++;
            can_tx_service(now, true);
            imu_samples_reset(&samples);
            sh2_close();
            last_retry = now;
            printf("BNO085 reconnect: reset=%u fault=%u header=%08x accel_age=%lu gyro_age=%lu queued=%lu\r\n",
                   (unsigned)sensor_reset, bno_transport_fault, bno_last_header,
                   (unsigned long)accel_age, (unsigned long)gyro_age,
                   (unsigned long)can_queued_frames);
            continue;
        }

        if ((uint32_t)(now - last_tx) >= BNO085_CAN_BNO085_DATA_CYCLE_TIME_MS) {
            uint8_t payload[BNO085_CAN_BNO085_DATA_LENGTH];
            uint32_t oldest_ms = (uint32_t)(now - samples.accel_ms) > (uint32_t)(now - samples.gyro_ms)
                                 ? samples.accel_ms : samples.gyro_ms;
            if (imu_samples_pack(&samples, now, payload) && can_tx_send(payload, oldest_ms)) {
                last_tx = now;
                imu_samples_sent(&samples);
            }
        }
        /* Sleep only after an atomic work check. EXTI, DMA and SysTick wake us;
           an already-low INT or completed packet is serviced without sleeping. */
        __disable_irq();
        if (!bno_transport_has_work()) { __DSB(); __WFI(); }
        __enable_irq();
    }
}
