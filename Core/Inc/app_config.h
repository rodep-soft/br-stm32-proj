/**
 * @file app_config.h
 * @brief Ultra-Lean Centralized Configuration & Topic Routing Table
 *
 * ALL USER CONFIGURATIONS AND TOPICS ARE HERE:
 * 1. Network & IP settings
 * 2. Zenoh connection locators
 * 3. CAN bitrate & timings
 * 4. Master Topic Routing Table (Register topics in 1 line!)
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ucdr/microcdr.h>

/* Generated message headers */
#include "generated/MotorStatus.h"
#include "generated/ImuData.h"
#include "generated/MotorCommand.h"
#include "generated/Frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================================================================
 * 1. Network (Ethernet) Settings
 * ============================================================================== */
#define CONFIG_NET_USE_DHCP           0  /**< 0: Instant static IP (< 1s boot), 1: DHCP fallback */
#define CONFIG_NET_STATIC_IP          "192.168.50.77"
#define CONFIG_NET_STATIC_NETMASK     "255.255.255.0"
#define CONFIG_NET_STATIC_GATEWAY     "192.168.50.1"
#define CONFIG_NET_DHCP_TIMEOUT_SEC   5

/* ==============================================================================
 * 2. Zenoh & ROS 2 Settings
 * ============================================================================== */
#define CONFIG_ZENOH_MODE             "client"

#define CONFIG_ZENOH_LOCATOR_LIST \
    "udp/192.168.50.30:7447", \
    "udp/192.168.50.10:7447", \
    "udp/192.168.50.50:7447", \
    "udp/192.168.50.150:7447"

#define CONFIG_ROS2_NODE_NAME         "stm32_bridge"
#define CONFIG_ROS2_NODE_NS           "/"
#define CONFIG_ROS2_DOMAIN_ID         0

/* ==============================================================================
 * 3. CAN Bus Settings
 * ============================================================================== */
#define CONFIG_CAN_BITRATE            500000U  /**< Default: 500 kbps (1M, 500k, 250k, 125k) */
#define CONFIG_CAN_STATS_PERIOD_MS    3000     /**< Diagnostics reporting interval */
#define CONFIG_CAN_WATCHDOG_MS        1500     /**< Disconnect timeout before Red LED alert */
#define CONFIG_CAN_FRAME_TIMEOUT_MS   100      /**< Incomplete multi-frame drop timeout */
#define CONFIG_CAN_TX_TIMEOUT_MS      10       /**< Mailbox wait timeout */

/* ==============================================================================
 * 4. FreeRTOS Tasks & Memory Settings
 * ============================================================================== */
#define CONFIG_QUEUE_CAN_RX_DEPTH     64       /**< Raw CAN frame queue size in DTCM-RAM */
#define CONFIG_STACK_ZENOH_TASK       2048     /**< Stack words for Zenoh manager */
#define CONFIG_STACK_BRIDGE_TASK      1024     /**< Stack words for CAN->Zenoh worker */

/* ==============================================================================
 * 5. Automatic Timing Calculation & Verification
 * ============================================================================== */
#if (CONFIG_CAN_BITRATE != 1000000U && CONFIG_CAN_BITRATE != 500000U && \
     CONFIG_CAN_BITRATE != 250000U  && CONFIG_CAN_BITRATE != 125000U)
#error "Invalid CONFIG_CAN_BITRATE! Supported: 1000000, 500000, 250000, 125000"
#endif

#define CONFIG_CAN_PRESCALER          (48000000U / (16U * (CONFIG_CAN_BITRATE)))

/* ==============================================================================
 * 6. Master Topic Routing Table (Declarative)
 * ============================================================================== */
typedef bool (*cdr_serialize_fn_t)(ucdrBuffer *ub, const void *topic);
typedef bool (*cdr_deserialize_fn_t)(ucdrBuffer *ub, void *topic);

typedef enum {
    BRIDGE_DIR_CAN_TO_ROS = 0,  /**< CAN frames assembled -> ROS 2 (Publish) */
    BRIDGE_DIR_ROS_TO_CAN,      /**< ROS 2 message (Subscribe) -> CAN frames */
} bridge_dir_t;

typedef struct {
    const char           *topic_name;
    bridge_dir_t          dir;
    uint32_t              can_base_id;
    size_t                msg_size;
    const char           *dds_type;
    const char           *type_hash;
    cdr_serialize_fn_t    serialize_fn;
    cdr_deserialize_fn_t  deserialize_fn;
} bridge_topic_t;

#define BRIDGE_CAN_TO_ROS(topic, msg, can_id) \
    { \
        .topic_name     = topic, \
        .dir            = BRIDGE_DIR_CAN_TO_ROS, \
        .can_base_id    = can_id, \
        .msg_size       = sizeof(robot_msgs_##msg), \
        .dds_type       = robot_msgs_##msg##_DDS_TYPE, \
        .type_hash      = robot_msgs_##msg##_TYPE_HASH, \
        .serialize_fn   = (cdr_serialize_fn_t)robot_msgs_##msg##_serialize, \
        .deserialize_fn = NULL, \
    }

#define BRIDGE_ROS_TO_CAN(topic, msg, can_id) \
    { \
        .topic_name     = topic, \
        .dir            = BRIDGE_DIR_ROS_TO_CAN, \
        .can_base_id    = can_id, \
        .msg_size       = sizeof(robot_msgs_##msg), \
        .dds_type       = robot_msgs_##msg##_DDS_TYPE, \
        .type_hash      = robot_msgs_##msg##_TYPE_HASH, \
        .serialize_fn   = NULL, \
        .deserialize_fn = (cdr_deserialize_fn_t)robot_msgs_##msg##_deserialize, \
    }

/**
 * 🌟 MASTER TOPIC TABLE: Add your sensors and actuators here!
 */
static const bridge_topic_t g_bridge_topics[] = {
    /* 1. CAN -> ROS 2: MotorStatus (28B = 4 CAN frames: 0x100..0x103) */
    BRIDGE_CAN_TO_ROS("motor_status",  MotorStatus,  0x100),

    /* 2. CAN -> ROS 2: ImuData (24B = 3 CAN frames: 0x200..0x202) */
    BRIDGE_CAN_TO_ROS("imu_data",      ImuData,      0x200),

    /* 3. ROS 2 -> CAN: MotorCommand (8B = 1 CAN frame: 0x300) */
    BRIDGE_ROS_TO_CAN("motor_command", MotorCommand, 0x300),
};

#define BRIDGE_TOPIC_COUNT  (sizeof(g_bridge_topics) / sizeof(g_bridge_topics[0]))
#define BRIDGE_MAX_MSG_SIZE 64

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
