#ifndef SH2_HAL_STM32_H
#define SH2_HAL_STM32_H

#include "sh2_hal.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pin Definitions
 * PA0 : INT  (Input, Active LOW)
 * PA1 : RST  (Output, Active LOW)
 * PA3 : WAKE (Output, Active LOW, PS0)
 * PA4 : CS   (Output, Active LOW)
 */
#define BNO08X_INT_GPIO_PORT   GPIOA
#define BNO08X_INT_PIN         GPIO_PIN_0

#define BNO08X_RST_GPIO_PORT   GPIOA
#define BNO08X_RST_PIN         GPIO_PIN_1

#define BNO08X_WAKE_GPIO_PORT  GPIOA
#define BNO08X_WAKE_PIN        GPIO_PIN_3

#define BNO08X_CS_GPIO_PORT    GPIOA
#define BNO08X_CS_PIN          GPIO_PIN_4

/**
 * @brief  STM32用の sh2_Hal_t インスタンスを取得する
 * @return sh2_Hal_t*
 */
sh2_Hal_t *sh2_hal_stm32_get(void);

/**
 * @brief  ハードウェアリセットを実行する（RSTピン操作）
 */
void sh2_hal_stm32_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SH2_HAL_STM32_H */
