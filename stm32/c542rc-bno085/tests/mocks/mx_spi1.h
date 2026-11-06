#ifndef MOCK_SPI_H
#define MOCK_SPI_H
#include <stdint.h>
#include <stddef.h>
typedef struct { int unused; } hal_spi_handle_t;
enum { HAL_OK, HAL_ERROR, EXTI0_IRQn };
extern uint32_t mock_irq_mask;
#define __DMB() ((void)0)
#define __WFI() ((void)0)
#define __get_PRIMASK() mock_irq_mask
#define __disable_irq() (mock_irq_mask = 1)
#define __set_PRIMASK(x) (mock_irq_mask = (x))
#define NVIC_SetPriority(a,b) ((void)0)
#define NVIC_EnableIRQ(a) ((void)0)
hal_spi_handle_t *mx_spi1_gethandle(void);
int HAL_SPI_MASTER_EnableKeepIOState(hal_spi_handle_t *);
int HAL_SPI_Abort(hal_spi_handle_t *);
int HAL_SPI_TransmitReceive_DMA(hal_spi_handle_t *, const void *, void *, uint32_t);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
#endif
