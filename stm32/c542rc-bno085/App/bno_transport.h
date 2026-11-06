#ifndef BNO_TRANSPORT_H
#define BNO_TRANSPORT_H
#include "sh2_hal.h"
#include <stdbool.h>

sh2_Hal_t *bno_transport_get(void);
bool bno_transport_take_error(void);
bool bno_transport_has_work(void);
extern volatile uint32_t bno_dma_reads, bno_dma_writes, bno_max_int_latency_us;
/* Debugger/UART diagnostics; never transmitted on CAN. */
extern volatile unsigned bno_transport_fault, bno_last_header;
#endif
