#include "can_tx.h"
#include "imu_samples.h"
#include "mx_fdcan1.h"

static uint32_t pending_buffer, sample_ms;
static bool abort_requested;
volatile uint32_t can_queued_frames, can_tx_failures, can_expired_frames, can_sent_frames;
volatile uint32_t can_unsent_frames;

void can_tx_service(uint32_t now_ms, bool invalidate)
{
    hal_fdcan_handle_t *can = mx_fdcan1_gethandle();
    if (!pending_buffer) return;
    if (HAL_FDCAN_GetTxBufferMessageStatus(can, pending_buffer) == HAL_FDCAN_TX_BUFFER_NOT_PENDING) {
        /* TXBTO distinguishes actual completion from cancellation, including a
           frame which finished just as its cancellation was requested. */
        if (FDCAN1->TXBTO & pending_buffer) can_sent_frames++;
        else if (abort_requested) can_expired_frames++;
        /* DAR releases unsuccessful attempts (ACK/error/arbitration loss).
           Do not hold the next fresh sample until the age deadline. */
        else can_unsent_frames++;
        pending_buffer = 0;
        abort_requested = false;
        return;
    }
    if (!abort_requested && (invalidate || (uint32_t)(now_ms - sample_ms) >= IMU_MAX_SAMPLE_AGE_MS)) {
        if (HAL_FDCAN_ReqAbortOfTxBuffer(can, pending_buffer) == HAL_OK) abort_requested = true;
        else can_tx_failures++;
    }
}

bool can_tx_send(const uint8_t payload[BNO085_CAN_BNO085_DATA_LENGTH], uint32_t oldest_sample_ms)
{
    hal_fdcan_handle_t *can = mx_fdcan1_gethandle();
    if (pending_buffer || (uint32_t)(HAL_GetTick() - oldest_sample_ms) >= IMU_MAX_SAMPLE_AGE_MS ||
        HAL_FDCAN_GetTxFifoFreeLevel(can) == HAL_FDCAN_TX_FIFO_FREE_LEVEL_0) return false;
    hal_fdcan_tx_header_t header = {0};
    header.b.identifier = BNO085_CAN_BNO085_DATA_FRAME_ID;
    header.b.identifier_type = HAL_FDCAN_ID_STANDARD;
    header.b.frame_type = HAL_FDCAN_FRAME_DATA;
    header.b.data_length = HAL_FDCAN_DATA_LEN_FDCAN_20_BYTE;
    header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
    header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
    header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_FD_CAN;
    header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
    if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(can, &header, payload) != HAL_OK) {
        can_tx_failures++;
        return false;
    }
    pending_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(can);
    sample_ms = oldest_sample_ms;
    abort_requested = false;
    can_queued_frames++;
    return true;
}
