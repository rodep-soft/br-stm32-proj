#ifndef BNO08X_H
#define BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BNO08xの初期化（SPIオープン、製品ID確認、イベントリスナ登録）
 * @return 0: 成功, 負数: エラー
 */
int bno08x_init(void);

/**
 * @brief 各センサレポート（加速度、ジャイロ、クォータニオン等）を有効化する
 * @param sensor_id レポートID (例: SH2_ACCELEROMETER, SH2_GYROSCOPE_CALIBRATED, SH2_ROTATION_VECTOR)
 * @param interval_us レポート周期（マイクロ秒, 例: 10000 = 10ms = 100Hz）
 * @return true: 成功, false: 失敗
 */
bool bno08x_enable_report(sh2_SensorId_t sensor_id, uint32_t interval_us);

/**
 * @brief SHTPパケットを処理する（メインループで定期的に呼ぶ）
 */
void bno08x_service(void);

/**
 * @brief 最新のセンサ値を取得する
 * @param value 格納先ポインタ
 * @return true: 新しいイベントを取得した, false: 新しいイベントなし
 */
bool bno08x_get_sensor_event(sh2_SensorValue_t *value);

/**
 * @brief センサがリセットされたかをチェックする
 * @return true: リセットが発生した（センサレポートの再有効化が必要）
 */
bool bno08x_was_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* BNO08X_H */
