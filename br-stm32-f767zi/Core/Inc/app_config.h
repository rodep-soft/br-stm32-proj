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
#include "generated/transport/Frame.h"
#include "generated/transport/TimedFrame.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================================================================
 * 1. Network (Ethernet) Settings
 * ============================================================================== */
// DHCPはなるべく使わないこと
#define CONFIG_NET_USE_DHCP           0  /**< 0: static-first, 1: DHCP-first with static fallback */
// stm32の静的(static)IP
#define CONFIG_NET_STATIC_IP          "192.168.50.77"
#define CONFIG_NET_STATIC_NETMASK     "255.255.255.0"

// これは部室用
#define CONFIG_NET_STATIC_GATEWAY     "192.168.50.1"
#define CONFIG_NET_DHCP_TIMEOUT_SEC   5
#define CONFIG_NET_LINK_TIMEOUT_SEC   10

/* ==============================================================================
 * 2. Zenoh & ROS 2 Settings
 * ============================================================================== */
#define CONFIG_ZENOH_MODE             "client"

// 接続先IPアドレス
// udpを使うこと!
#define CONFIG_ZENOH_LOCATOR_LIST \
    "udp/192.168.50.10:7447", \
    "udp/192.168.50.30:7447", \
    "udp/192.168.50.50:7447", \
    "udp/192.168.50.150:7447"

#define CONFIG_ROS2_NODE_NAME         "stm32_bridge"
#define CONFIG_ROS2_NODE_NS           "/"
#define CONFIG_ROS2_DOMAIN_ID         0

/* ==============================================================================
 * 3. CAN Bus Settings
 * ============================================================================== */
 
// baudrateは合わせる
#define CONFIG_CAN_BITRATE            1000000U  /**< Default: 500 kbps (1M, 500k, 250k, 125k) */
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
    BRIDGE_DIR_CAN_RECV,        /**< Standalone CAN Receiver: log to serial without PC/Zenoh */
    BRIDGE_DIR_CAN_ECHO = BRIDGE_DIR_CAN_RECV, /**< Alias for backward compatibility */
    BRIDGE_DIR_ROS_TO_CAN_RAW,  /**< can_transport_msgs/Frame -> Direct CAN frame TX */
    BRIDGE_DIR_CAN_TO_ROS_RAW,  /**< Direct CAN frame RX -> can_transport_msgs/Frame */
} bridge_dir_t;

typedef void (*msg_print_fn_t)(const void *topic);

typedef struct {
    const char           *topic_name;
    bridge_dir_t          dir;
    uint32_t              can_base_id;
    size_t                msg_size;
    const char           *dds_type;
    const char           *type_hash;
    cdr_serialize_fn_t    serialize_fn;
    cdr_deserialize_fn_t  deserialize_fn;
    msg_print_fn_t        print_fn;
} bridge_topic_t;

#define BRIDGE_ROS_TO_CAN_FRAME(topic) \
    { \
        .topic_name     = topic, \
        .dir            = BRIDGE_DIR_ROS_TO_CAN_RAW, \
        .can_base_id    = 0, \
        .msg_size       = sizeof(can_msgs_Frame), \
        .dds_type       = can_msgs_Frame_DDS_TYPE, \
        .type_hash      = can_msgs_Frame_TYPE_HASH, \
        .serialize_fn   = NULL, \
        .deserialize_fn = (cdr_deserialize_fn_t)can_msgs_Frame_deserialize, \
        .print_fn       = (msg_print_fn_t)can_msgs_Frame_print, \
    }

#define BRIDGE_CAN_TO_ROS_FRAME(topic) \
    { \
        .topic_name     = topic, \
        .dir            = BRIDGE_DIR_CAN_TO_ROS_RAW, \
        .can_base_id    = 0, \
        .msg_size       = sizeof(can_msgs_Frame), \
        .dds_type       = can_msgs_Frame_DDS_TYPE, \
        .type_hash      = can_msgs_Frame_TYPE_HASH, \
        .serialize_fn   = (cdr_serialize_fn_t)can_msgs_Frame_serialize, \
        .deserialize_fn = NULL, \
        .print_fn       = (msg_print_fn_t)can_msgs_Frame_print, \
    }

/**
 * MASTER TOPIC TABLE: Add your sensors, actuators, and ping-pong devices here.
 */

// ここにデータ送受信を定義
// can <--> ros2の方向に注意
static const bridge_topic_t g_bridge_topics[] = {
    /* Generic classic CAN transport */
    BRIDGE_ROS_TO_CAN_FRAME("can/tx"),
    BRIDGE_CAN_TO_ROS_FRAME("can/rx"),
};

#define BRIDGE_TOPIC_COUNT  (sizeof(g_bridge_topics) / sizeof(g_bridge_topics[0]))
#define BRIDGE_MAX_MSG_SIZE 128

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
