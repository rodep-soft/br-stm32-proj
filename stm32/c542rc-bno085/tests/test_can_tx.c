#include <assert.h>
#include <stdio.h>
#include "../App/can_tx.c"
mock_fdcan_registers_t mock_fdcan;
static hal_fdcan_handle_t handle;
static uint32_t now, pending, aborts;
static int send_status, abort_status;
hal_fdcan_handle_t *mx_fdcan1_gethandle(void) { return &handle; }
uint32_t HAL_GetTick(void) { return now; }
int HAL_FDCAN_GetTxBufferMessageStatus(const hal_fdcan_handle_t *h, uint32_t mask)
{ (void)h; return (pending & mask) != 0; }
int HAL_FDCAN_ReqAbortOfTxBuffer(const hal_fdcan_handle_t *h, uint32_t mask)
{ (void)h; assert(mask == pending); aborts++; return abort_status; }
int HAL_FDCAN_GetTxFifoFreeLevel(const hal_fdcan_handle_t *h) { (void)h; return 3; }
int HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hal_fdcan_handle_t *h, const hal_fdcan_tx_header_t *header, const uint8_t *data)
{
    (void)h; (void)data;
    assert(header->b.identifier == 0x100 && header->b.data_length == 8);
    if (send_status) return send_status;
    pending = 2; mock_fdcan.TXBTO = 0; return HAL_OK;
}
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const hal_fdcan_handle_t *h) { (void)h; return 2; }
int main(void)
{
    uint8_t data[8] = {0};
    now = 1000;
    assert(!can_tx_send(data, 900)); /* Already stale. */
    send_status = HAL_ERROR;
    assert(!can_tx_send(data, 990) && can_tx_failures == 1);
    send_status = HAL_OK;
    assert(can_tx_send(data, 990));
    assert(!can_tx_send(data, 1000)); /* Never queue behind an old pending frame. */
    can_tx_service(1089, false); assert(aborts == 0);
    can_tx_service(1090, false); assert(aborts == 1);
    can_tx_service(1200, false); assert(aborts == 1);
    assert(!can_tx_send(data, 1000)); /* Wait for hardware cancellation acknowledgement. */
    pending = 0; can_tx_service(1200, false);
    assert(can_expired_frames == 1 && can_sent_frames == 0);
    now = 1200;
    assert(can_tx_send(data, now));
    can_tx_service(now, true); assert(aborts == 2); /* Sensor reset invalidates immediately. */
    pending = 0; mock_fdcan.TXBTO = 2; can_tx_service(now, false);
    assert(can_sent_frames == 1); /* Completed at the cancellation boundary. */
    now = UINT32_MAX - 20;
    assert(can_tx_send(data, now));
    can_tx_service(78, false); assert(aborts == 2);
    abort_status = HAL_ERROR;
    can_tx_service(79, false); assert(aborts == 3 && can_tx_failures == 2);
    abort_status = HAL_OK;
    can_tx_service(80, false); assert(aborts == 4);
    pending = 0; can_tx_service(81, false);
    assert(can_expired_frames == 2);

    /* DAR ends an unsuccessful attempt without a software abort. A new sample
       can be queued at the next 5 ms slot instead of waiting 100 ms. */
    now = 2000;
    assert(can_tx_send(data, now));
    pending = 0; can_tx_service(now + 1, false);
    assert(can_unsent_frames == 1 && can_expired_frames == 2 && aborts == 4);
    now += 5;
    assert(can_tx_send(data, now));
    pending = 0; mock_fdcan.TXBTO = 2; can_tx_service(now + 1, false);
    assert(can_sent_frames == 2 && can_unsent_frames == 1);
    puts("PASS: CAN one-shot failure release, expiry, invalidation, completion race and tick wrap");
}
