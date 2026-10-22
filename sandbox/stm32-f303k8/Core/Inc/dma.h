#ifndef __DMA_H
#define __DMA_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1段階DMAで固定長受信するバイト数（SHTPヘッダ4B + タイムスタンプ5B + センサデータ10B×2 = 29B を十分に収容） */
#define BNO08X_DMA_READ_LEN    64U

/* DMA転送のタイムアウト時間 (ms) */
#define BNO_DMA_TIMEOUT_MS     50U

/**
 * @brief  DMA受信モードの有効/無効を切り替える（初期化・設定完了後にtrueにする）
 * @param  enabled true: DMAモード有効, false: ブロッキング通信モード
 */
void bno_dma_set_mode(bool enabled);

/**
 * @brief  現在DMA受信モードが有効かどうかを返す
 * @return true: DMAモード有効, false: 無効
 */
bool bno_dma_is_enabled(void);

/**
 * @brief  EXTI割り込み等から1段階DMA受信を開始する
 * @return true: DMA開始成功, false: 開始条件を満たさないかビジー
 */
bool bno_dma_start(void);

/**
 * @brief  SPI DMA送受信完了コールバック時に呼ぶ処理（CSをHIGHに戻しデータ準備完了にする）
 */
void bno_dma_finish(void);

/**
 * @brief  SPI DMAエラー時や強制停止時にDMA状態をリセットする
 */
void bno_dma_abort(void);

/**
 * @brief  DMA通信のタイムアウトを監視し、タイムアウト時はDMAを破棄・リセットする
 * @param  timeout_ms タイムアウト時間 (ms)
 * @return true: タイムアウトが発生してリセットした, false: 正常
 */
bool bno_dma_check_timeout(uint32_t timeout_ms);

/**
 * @brief  DMAでRAMバッファに受信済みのSHTPパケットを取り出す（hal_readから呼ばれる）
 * @param  pBuffer コピー先バッファ
 * @param  len     コピー先バッファの最大サイズ
 * @param  t_us    受信タイムスタンプ(us)格納先
 * @return 読み出したパケットのバイト数（データなし・無効時は0）
 */
int bno_dma_read_packet(uint8_t *pBuffer, unsigned len, uint32_t *t_us);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H */
