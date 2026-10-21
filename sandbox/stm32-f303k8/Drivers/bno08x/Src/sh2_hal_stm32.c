#include "sh2_hal_stm32.h"
#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

#define BNO_CS_LOW()     HAL_GPIO_WritePin(BNO08X_CS_GPIO_PORT, BNO08X_CS_PIN, GPIO_PIN_RESET)
#define BNO_CS_HIGH()    HAL_GPIO_WritePin(BNO08X_CS_GPIO_PORT, BNO08X_CS_PIN, GPIO_PIN_SET)
#define BNO_RST_LOW()    HAL_GPIO_WritePin(BNO08X_RST_GPIO_PORT, BNO08X_RST_PIN, GPIO_PIN_RESET)
#define BNO_RST_HIGH()   HAL_GPIO_WritePin(BNO08X_RST_GPIO_PORT, BNO08X_RST_PIN, GPIO_PIN_SET)
#define BNO_WAKE_LOW()   HAL_GPIO_WritePin(BNO08X_WAKE_GPIO_PORT, BNO08X_WAKE_PIN, GPIO_PIN_RESET)
#define BNO_WAKE_HIGH()  HAL_GPIO_WritePin(BNO08X_WAKE_GPIO_PORT, BNO08X_WAKE_PIN, GPIO_PIN_SET)
#define BNO_INT_IS_LOW() (HAL_GPIO_ReadPin(BNO08X_INT_GPIO_PORT, BNO08X_INT_PIN) == GPIO_PIN_RESET)

static bool wait_for_int_low(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!BNO_INT_IS_LOW()) {
        HAL_Delay(1);
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return false;
        }
    }
    return true;
}

void sh2_hal_stm32_reset(void)
{
    // 仕様: リセット前段階から PS0/WAKE と CS は HIGH を維持（SPIモード選択のため）
    BNO_WAKE_HIGH();
    BNO_CS_HIGH();
    BNO_RST_HIGH();
    HAL_Delay(10);

    // リセットパルス (Active LOW)
    BNO_RST_LOW();
    HAL_Delay(10);
    BNO_RST_HIGH();
    HAL_Delay(10);
}

static int hal_open(sh2_Hal_t *self)
{
    (void)self;

    sh2_hal_stm32_reset();

    // 仕様: 最初の H_INTN アサーション（LOW）後まで PS0/WAKE ピンは HIGH を維持する
    uint32_t start = HAL_GetTick();
    if (!wait_for_int_low(1000)) {
        printf("[OPEN] Reset timeout: INT never went LOW (INT=%d)\r\n", 
               HAL_GPIO_ReadPin(BNO08X_INT_GPIO_PORT, BNO08X_INT_PIN));
        return -1;
    }

    //printf("[OPEN] Reset OK: INT went LOW in %lu ms (SPI Mode Locked)\r\n", HAL_GetTick() - start);
    return 0;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
    BNO_CS_HIGH();
    BNO_RST_LOW();
}

static uint8_t dummy_tx[1024] = {0};

static uint8_t dummy_rx[1024];

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;
    uint16_t packet_size = 0;

    // 受信データがない（INTピンがHIGH）場合は即座に0を返す
    if (!BNO_INT_IS_LOW()) {
        return 0;
    }

    if (t_us != NULL) {
        *t_us = HAL_GetTick() * 1000;
    }

    // 1. SPIトランザクション開始 (CS LOW)
    BNO_CS_LOW();

    // 2. ヘッダ4バイトを受信
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, dummy_tx, pBuffer, 4, 10);
    if (status != HAL_OK) {
        BNO_CS_HIGH();
        return 0;
    }

    packet_size = (uint16_t)pBuffer[0] | ((uint16_t)pBuffer[1] << 8);
    packet_size &= ~0x8000;

    // パケットサイズ検証 (空パケット、異常値、バッファ超過)
    if (packet_size < 4 || packet_size > len || packet_size == 0x7FFF) {
        BNO_CS_HIGH();
        return 0;
    }

    // 3. 残りのペイロードを受信（CS LOW を維持したまま同一トランザクションで読み切る）
    if (packet_size > 4) {
        status = HAL_SPI_TransmitReceive(&hspi1, dummy_tx, pBuffer + 4, packet_size - 4, 50);
        if (status != HAL_OK) {
            BNO_CS_HIGH();
            return 0;
        }
    }

    // 4. トランザクション完了 (CS HIGH)
    BNO_CS_HIGH();

    // センサデータ(ch=3)以外の制御パケットのみログ出力
    if (pBuffer[2] != 3) {
        printf("[READ] len=%u ch=%d seq=%d\r\n", packet_size, pBuffer[2], pBuffer[3]);
    }
    return packet_size;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;

    if (len == 0 || pBuffer == NULL) {
        return 0;
    }

    bool wake_asserted = false;

    // 仕様: リセット後、BNO085がスリープ状態（INT=HIGH）の場合、
    // ホストから通信を開始するためにPS0/WAKE信号をLOWにして復帰させる
    if (!BNO_INT_IS_LOW()) {
        BNO_WAKE_LOW();
        wake_asserted = true;

        // BNO085が復帰してINTをLOWに落とすのを待つ（最大100ms）
        if (!wait_for_int_low(100)) {
            printf("[WRITE] Wakeup timeout: INT stayed HIGH\r\n");
            BNO_WAKE_HIGH();
            return 0;
        }
    }

    // Host-to-Device送信: CS LOWにしてSPI送信
    BNO_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, pBuffer, dummy_rx, len, 50);
    BNO_CS_HIGH();

    // 送信完了後、WAKEピンをHIGHに戻す
    if (wake_asserted) {
        BNO_WAKE_HIGH();
    }

    if (status != HAL_OK) {
        printf("[WRITE] SPI error status=%d\r\n", status);
        return 0;
    }

    printf("[WRITE] sent len=%u ch=%d seq=%d\r\n", len, pBuffer[2], pBuffer[3]);
    return len;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return HAL_GetTick() * 1000;
}

static sh2_Hal_t s_hal = {
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_getTimeUs,
};

sh2_Hal_t *sh2_hal_stm32_get(void)
{
    return &s_hal;
}
