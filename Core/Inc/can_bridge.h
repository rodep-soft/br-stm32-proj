/**
 * @file can_bridge.h
 * @brief Ultra-low latency, robust CAN1 HAL driver & FreeRTOS queue interface
 *
 * Professional design principles:
 * - ISR does ONLY hardware FIFO pop -> FreeRTOS queue push (< 2us).
 * - Zero processing/unpacking inside interrupt context.
 * - Hardware Auto-Bus-Off recovery enabled.
 * - Pure byte-frame abstraction: any protocol/topic can run over this layer.
 */

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_BRIDGE_RX_QUEUE_SIZE 64  /* Deep enough to burst 16 multi-frame packets */
#define CAN_BRIDGE_TX_QUEUE_SIZE 32

/**
 * @brief Standard CAN 2.0B frame representation
 */
typedef struct {
    uint32_t id;      /**< 11-bit standard ID */
    uint8_t  dlc;     /**< Data length code (0..8) */
    uint8_t  data[8]; /**< Payload */
} can_frame_t;

/**
 * @brief Hardware & Queue initialization.
 *        Configures 500 kbps bit-timing (48MHz APB1), filter bank 0,
 *        FIFO0 interrupts, and FreeRTOS queues.
 */
void can_bridge_init(void);

/**
 * @brief Get the RX queue handle to receive raw CAN frames in worker tasks.
 */
QueueHandle_t can_bridge_get_rx_queue(void);

/**
 * @brief Send a CAN frame to the hardware TX mailbox (thread-safe, with mailbox backpressure).
 * @param frame   Frame to transmit
 * @param timeout Tick timeout to wait for free mailbox
 * @return pdTRUE on success, pdFALSE on timeout / error
 */
BaseType_t can_bridge_send_frame(const can_frame_t *frame, TickType_t timeout);

/**
 * @brief Diagnostic statistics
 */
void can_bridge_get_stats(uint32_t *rx_frames, uint32_t *tx_frames, uint32_t *drop_count);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BRIDGE_H */
