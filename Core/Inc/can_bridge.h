/**
 * @file can_bridge.h
 * @brief CAN Bridge Module - FreeRTOS queue-based CAN↔Application interface
 *
 * Provides lock-free message passing between CAN ISR and application tasks.
 * CAN RX: ISR reassembles multi-frame messages → RX queue → bridge task
 * CAN TX: Application → TX queue → TX task → CAN hardware
 */
#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f7xx_hal.h"
#include "can_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_BRIDGE_RX_QUEUE_LEN 32
#define CAN_BRIDGE_TX_QUEUE_LEN 16

/** Complete assembled message from CAN bus (CAN → Zenoh direction) */
typedef struct {
    can_msg_type_t type;
    union {
        robot_msgs_MotorStatus motor_status;
        robot_msgs_ImuData     imu_data;
    } data;
} can_bridge_rx_msg_t;

/** Message to send on CAN bus (Zenoh → CAN direction) */
typedef struct {
    can_msg_type_t type;
    union {
        robot_msgs_MotorCommand motor_cmd;
    } data;
} can_bridge_tx_msg_t;

/**
 * @brief Initialize CAN1 peripheral, filters, interrupts, and FreeRTOS queues.
 *        Must be called from a FreeRTOS task context (after scheduler starts).
 */
void can_bridge_init(void);

/**
 * @brief Blocking read of a fully-assembled CAN message from RX queue.
 * @param msg     Output message buffer
 * @param timeout FreeRTOS tick timeout (portMAX_DELAY for infinite)
 * @return pdTRUE if a message was received, pdFALSE on timeout
 */
BaseType_t can_bridge_receive(can_bridge_rx_msg_t *msg, TickType_t timeout);

/**
 * @brief Non-blocking enqueue of a message for CAN transmission.
 * @param msg Message to send
 * @return pdTRUE if enqueued, pdFALSE if queue full
 */
BaseType_t can_bridge_send(const can_bridge_tx_msg_t *msg);

/**
 * @brief Blocking read from TX queue (for CAN TX task).
 * @param msg     Output message buffer
 * @param timeout FreeRTOS tick timeout
 * @return pdTRUE if a message was received, pdFALSE on timeout
 */
BaseType_t can_bridge_tx_receive(can_bridge_tx_msg_t *msg, TickType_t timeout);

/**
 * @brief Get the CAN HAL handle (for direct TX operations).
 * @return Pointer to the CAN_HandleTypeDef
 */
CAN_HandleTypeDef *can_bridge_get_handle(void);

/**
 * @brief Get bridge statistics counters.
 * @param rx_count  Total assembled RX messages
 * @param tx_count  Total TX messages enqueued
 * @param err_count Total errors (queue full, HAL errors)
 */
void can_bridge_get_stats(uint32_t *rx_count, uint32_t *tx_count, uint32_t *err_count);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BRIDGE_H */
