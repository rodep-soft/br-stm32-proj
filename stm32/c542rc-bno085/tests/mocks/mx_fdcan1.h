#ifndef MOCK_FDCAN_H
#define MOCK_FDCAN_H
#include <stdint.h>
typedef struct { int unused; } hal_fdcan_handle_t;
typedef struct { struct {
    uint32_t identifier, identifier_type, frame_type, data_length, error_state_indicator;
    uint32_t bit_rate_switch, frame_format, event_fifo_control;
} b; } hal_fdcan_tx_header_t;
typedef struct { uint32_t TXBTO; } mock_fdcan_registers_t;
extern mock_fdcan_registers_t mock_fdcan;
#define FDCAN1 (&mock_fdcan)
enum { HAL_OK = 0, HAL_ERROR = 1, HAL_FDCAN_TX_BUFFER_NOT_PENDING = 0,
       HAL_FDCAN_TX_BUFFER_PENDING = 1, HAL_FDCAN_TX_FIFO_FREE_LEVEL_0 = 0,
       HAL_FDCAN_ID_STANDARD = 0, HAL_FDCAN_FRAME_DATA = 0,
       HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE = 8,
       HAL_FDCAN_DATA_LEN_FDCAN_20_BYTE = 0xBU,
       HAL_FDCAN_ERROR_STATE_IND_ACTIVE = 0,
       HAL_FDCAN_BIT_RATE_SWITCH_OFF = 0, HAL_FDCAN_HEADER_FRAME_FORMAT_CAN = 0,
       HAL_FDCAN_HEADER_FRAME_FORMAT_FD_CAN = 1,
       HAL_FDCAN_TX_EVENTS_FIFO_DISCARD = 0 };
hal_fdcan_handle_t *mx_fdcan1_gethandle(void);
uint32_t HAL_GetTick(void);
int HAL_FDCAN_GetTxBufferMessageStatus(const hal_fdcan_handle_t *, uint32_t);
int HAL_FDCAN_ReqAbortOfTxBuffer(const hal_fdcan_handle_t *, uint32_t);
int HAL_FDCAN_GetTxFifoFreeLevel(const hal_fdcan_handle_t *);
int HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hal_fdcan_handle_t *, const hal_fdcan_tx_header_t *, const uint8_t *);
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const hal_fdcan_handle_t *);
#endif
