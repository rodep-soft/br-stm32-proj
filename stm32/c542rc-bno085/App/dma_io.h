#ifndef DMA_IO_H
#define DMA_IO_H
#include <stdbool.h>
#include <stdint.h>
bool dma_io_init(void);
uint32_t app_time_us(void);
void log_service(void);
extern volatile uint32_t log_dropped_bytes, log_dma_errors;
#endif
