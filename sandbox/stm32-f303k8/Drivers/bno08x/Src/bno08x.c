#include "../Inc/bno08x.h"
#include "sh2_hal_stm32.h"
#include <string.h>
#include <stdio.h>

static sh2_ProductIds_t prod_ids;
static sh2_SensorValue_t *current_sensor_value = NULL;
static volatile bool reset_occurred = false;

static void async_event_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event != NULL && event->eventId == SH2_RESET) {
        reset_occurred = true;
    }
}

static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    if (current_sensor_value != NULL && event != NULL) {
        if (sh2_decodeSensorEvent(current_sensor_value, event) != SH2_OK) {
            current_sensor_value->timestamp = 0;
        }
    }
}

int bno08x_init(void)
{
    // 前回のSHTPセッションが残っていれば閉じる（リトライ時のインスタンス枯渇防止）
    sh2_close();

    sh2_Hal_t *hal = sh2_hal_stm32_get();
    if (hal == NULL) {
        return -1;
    }

    reset_occurred = false;

    // SH2インターフェースの初期化
    int status = sh2_open(hal, async_event_handler, NULL);
    if (status != SH2_OK) {
        return -2;
    }

    // 接続確認のため製品IDを取得
    memset(&prod_ids, 0, sizeof(prod_ids));
    printf("Requesting Product IDs...\r\n");

    status = sh2_getProdIds(&prod_ids);
    if (status != SH2_OK) {
        printf("[ERR] sh2_getProdIds failed (status=%d)\r\n", status);
        return -3;
    }

    printf("[OK] Found %u product ID entries:\r\n", prod_ids.numEntries);
    for (unsigned i = 0; i < prod_ids.numEntries; i++) {
        printf("  Entry %u: Part %lu, Ver %u.%u.%u (Build %lu)\r\n",
               i,
               prod_ids.entry[i].swPartNumber,
               prod_ids.entry[i].swVersionMajor,
               prod_ids.entry[i].swVersionMinor,
               prod_ids.entry[i].swVersionPatch,
               prod_ids.entry[i].swBuildNumber);
    }


    // センサデータ受信コールバックを登録
    sh2_setSensorCallback(sensor_event_handler, NULL);

    return 0;
}

bool bno08x_enable_report(sh2_SensorId_t sensor_id, uint32_t interval_us)
{
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));

    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;
    config.reportInterval_us = interval_us;

    return (sh2_setSensorConfig(sensor_id, &config) == SH2_OK);
}



void bno08x_service(void)
{
    sh2_service();
}

bool bno08x_get_sensor_event(sh2_SensorValue_t *value)
{
    if (value == NULL) {
        return false;
    }

    current_sensor_value = value;
    //value->timestamp = 0;

    sh2_service();

    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        return false;
    }

    return true;
}

bool bno08x_was_reset(void)
{
    bool x = reset_occurred;
    reset_occurred = false;
    return x;
}
