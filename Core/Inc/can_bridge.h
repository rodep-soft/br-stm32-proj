/**
 * @file can_bridge.h
 * @brief High-performance CAN Bridge Task for STM32F767ZI / Zenoh-ROS2
 *
 * Bridges ROS 2 can_msgs/msg/Frame messages received over Zenoh
 * directly to the STM32 CAN bus (CAN1, PB8=RX, PB9=TX).
 */

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "generated/Frame.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Common CAN baudrates for 48MHz APB1 peripheral clock */
#define CAN_BAUDRATE_1000K  1000000U
#define CAN_BAUDRATE_500K   500000U
#define CAN_BAUDRATE_250K   250000U
#define CAN_BAUDRATE_125K   125000U

/* Default CAN settings (configured in app_config.h) */
#define CAN_BRIDGE_DEFAULT_BAUDRATE CONFIG_CAN_BAUDRATE
#define CAN_BRIDGE_QUEUE_SIZE       CONFIG_CAN_QUEUE_SIZE

/* Statistics for monitoring */
typedef struct {
    uint32_t rx_from_zenoh;       /* Frames received from Zenoh topic */
    uint32_t tx_to_can_success;   /* Frames successfully queued into CAN mailbox */
    uint32_t tx_to_can_fail;      /* CAN transmission failures/timeouts */
    uint32_t dropped_queue_full;  /* Frames dropped because FreeRTOS queue was full */
} can_bridge_stats_t;

/**
 * @brief Initialize the CAN1 hardware peripheral.
 * @param baudrate Desired baudrate in bps (e.g. CAN_BAUDRATE_500K)
 * @return true on success, false on failure
 */
bool can_bridge_init(uint32_t baudrate);

/**
 * @brief Start the CAN Bridge FreeRTOS worker task and queue.
 * @return true on success
 */
bool can_bridge_start(void);

/**
 * @brief Post a can_msgs_Frame to the CAN bridge queue for transmission.
 * Typically called from Zenoh subscriber callback. Non-blocking.
 *
 * @param frame Pointer to the frame to transmit
 * @return true if queued, false if queue is full or uninitialized
 */
bool can_bridge_post_frame(const can_msgs_Frame *frame);

/**
 * @brief Directly send a can_msgs_Frame to CAN1 hardware (blocking until mailbox free or timeout).
 *
 * @param frame Pointer to the frame to transmit
 * @return true on success
 */
bool can_bridge_send_frame(const can_msgs_Frame *frame);

/**
 * @brief Get current CAN bridge statistics.
 */
void can_bridge_get_stats(can_bridge_stats_t *out_stats);

/**
 * @brief Get the underlying CAN_HandleTypeDef pointer.
 */
CAN_HandleTypeDef *can_bridge_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BRIDGE_H */
