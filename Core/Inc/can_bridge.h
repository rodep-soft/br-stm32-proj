/**
 * @file can_bridge.h
 * @brief Ultra-low latency, robust CAN1 HAL driver & FreeRTOS queue interface
 *
 * Professional design:
 * - Deterministic, zero-wait execution via DTCM-RAM buffers.
 * - Hardware acceptance filter bank auto-generation (16-bit exact ID list mode).
 * - Real-time CAN bus health diagnostics (TEC, REC, LEC, Bus-Off, Error Passive).
 * - Ultra-minimal ISR (< 2µs) - hardware FIFO pop only.
 */

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f7xx_hal.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_BRIDGE_RX_QUEUE_SIZE CONFIG_QUEUE_CAN_RX_DEPTH
#define CAN_BRIDGE_TX_QUEUE_SIZE CONFIG_QUEUE_CAN_TX_DEPTH

/**
 * @brief Standard CAN 2.0B frame representation
 */
typedef struct {
    uint32_t id;      /**< 11-bit standard ID */
    uint8_t  dlc;     /**< Data length code (0..8) */
    uint8_t  data[8]; /**< Payload */
} can_frame_t;

/**
 * @brief CAN Bus Diagnostic Health Information
 */
typedef struct {
    uint8_t tec;            /**< Transmit Error Counter (0..255) */
    uint8_t rec;            /**< Receive Error Counter (0..255) */
    uint8_t lec;            /**< Last Error Code (0=None, 1=Stuff, 2=Form, 3=Ack, 4=BitRec, 5=BitDom, 6=CRC) */
    bool    is_bus_off;     /**< BOFF: Node in Bus-Off state */
    bool    is_passive;     /**< EPVF: Node in Error Passive state (> 127 errors) */
    bool    is_warning;     /**< EWGF: Error Warning (> 96 errors) */
} can_bus_health_t;

/**
 * @brief Initialize CAN hardware & queues.
 * @param filter_ids Array of standard CAN IDs to accept (exact match). If NULL, accepts all.
 * @param filter_id_count Number of IDs in filter_ids array (max 56).
 */
void can_bridge_init(const uint32_t *filter_ids, size_t filter_id_count);

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

/**
 * @brief Query current hardware CAN bus health registers (TEC, REC, LEC, Bus-Off)
 */
void can_bridge_get_bus_health(can_bus_health_t *health);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BRIDGE_H */
