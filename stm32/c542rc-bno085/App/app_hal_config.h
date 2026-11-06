#ifndef APP_HAL_CONFIG_H
#define APP_HAL_CONFIG_H
/* Forced into every translation unit so HAL handle layouts agree. CubeMX
 * owns the base configuration; application-owned DMA survives regeneration. */
#ifndef __ASSEMBLER__
#include "stm32c5xx_hal_conf.h"
#undef USE_HAL_SPI_DMA
#define USE_HAL_SPI_DMA 1U
#undef USE_HAL_UART_DMA
#define USE_HAL_UART_DMA 1U
#endif
#endif
