#include "dma.h"
#include "sh2_hal_stm32.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define BNO_CS_LOW()     HAL_GPIO_WritePin(BNO08X_CS_GPIO_PORT, BNO08X_CS_PIN, GPIO_PIN_RESET)
#define BNO_CS_HIGH()    HAL_GPIO_WritePin(BNO08X_CS_GPIO_PORT, BNO08X_CS_PIN, GPIO_PIN_SET)
#define BNO_INT_IS_LOW() (HAL_GPIO_ReadPin(BNO08X_INT_GPIO_PORT, BNO08X_INT_PIN) == GPIO_PIN_RESET)

/* --- 1段階DMA受信用のバッファと状態管理変数 --- */
static uint8_t s_dma_tx_dummy[BNO08X_DMA_READ_LEN] = {0};
static uint8_t s_dma_rx_buf[BNO08X_DMA_READ_LEN];
static volatile bool s_dma_mode_enabled = false;
static volatile bool s_dma_busy = false;
static volatile bool s_dma_packet_ready = false;
static volatile uint32_t s_dma_start_tick = 0;
static volatile uint32_t s_dma_timestamp_us = 0;

void bno_dma_set_mode(bool enabled)
{
    if (!enabled) {
        bno_dma_abort();
        s_dma_mode_enabled = false;
    } else {
        s_dma_packet_ready = false;
        s_dma_busy = false;
        s_dma_mode_enabled = true;
        // 既にINTピンがLOWになっていれば初回のDMAをキックする
        if (BNO_INT_IS_LOW()) {
            bno_dma_start();
        }
    }
}

bool bno_dma_is_enabled(void)
{
    return s_dma_mode_enabled;
}

bool bno_dma_start(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!s_dma_mode_enabled || s_dma_busy || s_dma_packet_ready || !BNO_INT_IS_LOW()) {
        if (!primask) {
            __enable_irq();
        }
        return false;
    }

    s_dma_busy = true;
    s_dma_start_tick = HAL_GetTick();
    s_dma_timestamp_us = s_dma_start_tick * 1000U;

    if (!primask) {
        __enable_irq();
    }

    BNO_CS_LOW();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_dma_tx_dummy, s_dma_rx_buf, BNO08X_DMA_READ_LEN) != HAL_OK) {
        BNO_CS_HIGH();
        s_dma_busy = false;
        return false;
    }

    return true;
}

void bno_dma_finish(void)
{
    BNO_CS_HIGH();
    s_dma_busy = false;
    s_dma_packet_ready = true;
}

void bno_dma_abort(void)
{
    HAL_SPI_Abort(&hspi1);
    BNO_CS_HIGH();
    s_dma_busy = false;
    s_dma_packet_ready = false;
}

bool bno_dma_check_timeout(uint32_t timeout_ms)
{
    if (s_dma_busy && ((HAL_GetTick() - s_dma_start_tick) >= timeout_ms)) {
        bno_dma_abort();
        // INTがまだLOWなら再度DMAを試みる
        if (BNO_INT_IS_LOW()) {
            bno_dma_start();
        }
        return true;
    }

    // 万が一EXTIの立ち下がりを取り逃してINTがLOWのままアイドルになっている場合の復帰
    if (s_dma_mode_enabled && !s_dma_busy && !s_dma_packet_ready && BNO_INT_IS_LOW()) {
        bno_dma_start();
    }

    return false;
}

int bno_dma_read_packet(uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    if (!s_dma_packet_ready) {
        return 0;
    }

    if (t_us != NULL) {
        *t_us = s_dma_timestamp_us;
    }

    uint16_t packet_size = (uint16_t)s_dma_rx_buf[0] | ((uint16_t)s_dma_rx_buf[1] << 8);
    packet_size &= ~0x8000;

    // パケットサイズ検証 (空パケット、異常値、バッファ超過)
    if (packet_size < 4 || packet_size > len || packet_size == 0x7FFF) {
        s_dma_packet_ready = false;
        if (BNO_INT_IS_LOW()) {
            bno_dma_start();
        }
        return 0;
    }

    uint16_t copy_len = (packet_size <= BNO08X_DMA_READ_LEN) ? packet_size : BNO08X_DMA_READ_LEN;
    memcpy(pBuffer, s_dma_rx_buf, copy_len);
    s_dma_packet_ready = false;

    // 次のパケットが既にBNO085側に溜まっていてINTがLOWのままなら、続けてDMAを開始する
    if (BNO_INT_IS_LOW()) {
        bno_dma_start();
    }

    return copy_len;
}
