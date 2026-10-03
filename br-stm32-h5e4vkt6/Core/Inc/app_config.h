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
#include "generated/Ping.h"
#include "generated/transport_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================================================================
 * 1. Network (Ethernet) Settings
 * ============================================================================== */
// DHCPはなるべく使わないこと
#define CONFIG_NET_USE_DHCP           0  /**< 0: Instant static IP (< 1s boot), 1: DHCP fallback */
// stm32の静的(static)IP
#define CONFIG_NET_STATIC_IP          "192.168.50.77"
#define CONFIG_NET_STATIC_NETMASK     "255.255.255.0"

// これは部室用
#define CONFIG_NET_STATIC_GATEWAY     "192.168.50.1"
#define CONFIG_NET_DHCP_TIMEOUT_SEC   5

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
#define CONFIG_CAN1_BITRATE           1000000U
#define CONFIG_CAN2_NOMINAL_BITRATE   1000000U
#define CONFIG_CAN2_DATA_BITRATE      2000000U
#define CONFIG_CAN2_USE_FD            1
#define CONFIG_CAN_BITRATE            CONFIG_CAN1_BITRATE
#define CONFIG_CAN_CLASSIC_BUS        BRIDGE_FDCAN1
#define CONFIG_CAN_FD_BUS             BRIDGE_FDCAN2
#define CONFIG_CAN_STATS_PERIOD_MS    500      /**< Diagnostics reporting interval */
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

#define CONFIG_CAN_KERNEL_CLOCK_HZ    25000000U
#define CONFIG_CAN_PRESCALER          (CONFIG_CAN_KERNEL_CLOCK_HZ / (25U * CONFIG_CAN_BITRATE))

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
    BRIDGE_DIR_ROS_TO_CAN_RAW,  /**< can_msgs/Frame (Subscribe) -> Direct CAN frame TX */
    BRIDGE_DIR_CAN_TO_ROS_RAW,  /**< Direct CAN frame RX -> can_msgs/Frame (Publish) */
} bridge_dir_t;

typedef enum {
    BRIDGE_CAN_ANY = 0,
    BRIDGE_FDCAN1 = 1,
    BRIDGE_FDCAN2 = 2
} bridge_can_bus_t;

typedef enum {
    BRIDGE_FRAME_CLASSIC = 0,
    BRIDGE_FRAME_FD      = 1
} bridge_frame_mode_t;

typedef enum {
    BRIDGE_PAYLOAD_TYPED = 0,
    BRIDGE_PAYLOAD_CLASSIC_FRAME,
    BRIDGE_PAYLOAD_FD_FRAME
} bridge_payload_kind_t;

typedef void (*msg_print_fn_t)(const void *topic);

typedef struct {
    const char           *topic_name;
    bridge_dir_t          dir;
    bridge_can_bus_t      bus;
    bridge_frame_mode_t   frame_mode;
    bridge_payload_kind_t payload_kind;
    uint32_t              can_base_id;
    size_t                msg_size;
    const char           *dds_type;
    const char           *type_hash;
    cdr_serialize_fn_t    serialize_fn;
    cdr_deserialize_fn_t  deserialize_fn;
    msg_print_fn_t        print_fn;
} bridge_topic_t;

#define BRIDGE_TOPIC_INIT(topic, direction, can_bus, mode, can_id, payload, \
                          type_size, type_name, serializer, deserializer, printer) \
    { \
        .topic_name     = topic, \
        .dir            = direction, \
        .bus            = can_bus, \
        .frame_mode     = mode, \
        .payload_kind   = payload, \
        .can_base_id    = can_id, \
        .msg_size       = type_size, \
        .dds_type       = type_name##_DDS_TYPE, \
        .type_hash      = type_name##_TYPE_HASH, \
        .serialize_fn   = serializer, \
        .deserialize_fn = deserializer, \
        .print_fn       = printer, \
    }

#define BRIDGE_CAN_TO_ROS(topic, msg, can_id, can_bus, mode) \
    BRIDGE_TOPIC_INIT(topic, BRIDGE_DIR_CAN_TO_ROS, can_bus, mode, can_id, \
                      BRIDGE_PAYLOAD_TYPED, \
                      sizeof(robot_msgs_##msg), robot_msgs_##msg, \
                      (cdr_serialize_fn_t)robot_msgs_##msg##_serialize, NULL, \
                      (msg_print_fn_t)robot_msgs_##msg##_print)

#define BRIDGE_ROS_TO_CAN(topic, msg, can_id, can_bus, mode) \
    BRIDGE_TOPIC_INIT(topic, BRIDGE_DIR_ROS_TO_CAN, can_bus, mode, can_id, \
                      BRIDGE_PAYLOAD_TYPED, \
                      sizeof(robot_msgs_##msg), robot_msgs_##msg, NULL, \
                      (cdr_deserialize_fn_t)robot_msgs_##msg##_deserialize, \
                      (msg_print_fn_t)robot_msgs_##msg##_print)

#define BRIDGE_CAN_RECV(topic, msg, rx_id, can_bus, mode) \
    BRIDGE_TOPIC_INIT(topic, BRIDGE_DIR_CAN_RECV, can_bus, mode, rx_id, \
                      BRIDGE_PAYLOAD_TYPED, \
                      sizeof(robot_msgs_##msg), robot_msgs_##msg, \
                      (cdr_serialize_fn_t)robot_msgs_##msg##_serialize, \
                      (cdr_deserialize_fn_t)robot_msgs_##msg##_deserialize, \
                      (msg_print_fn_t)robot_msgs_##msg##_print)

#define BRIDGE_ROS_TO_CAN_FRAME(topic, can_bus) \
    BRIDGE_TOPIC_INIT(topic, BRIDGE_DIR_ROS_TO_CAN_RAW, can_bus, BRIDGE_FRAME_CLASSIC, 0, \
                      BRIDGE_PAYLOAD_CLASSIC_FRAME, \
                      sizeof(can_msgs_Frame), can_msgs_Frame, NULL, \
                      (cdr_deserialize_fn_t)can_msgs_Frame_deserialize, \
                      (msg_print_fn_t)can_msgs_Frame_print)

#define BRIDGE_CAN_TO_ROS_FRAME(topic, can_bus) \
    BRIDGE_TOPIC_INIT(topic, BRIDGE_DIR_CAN_TO_ROS_RAW, can_bus, BRIDGE_FRAME_CLASSIC, 0, \
                      BRIDGE_PAYLOAD_CLASSIC_FRAME, \
                      sizeof(can_msgs_Frame), can_msgs_Frame, \
                      (cdr_serialize_fn_t)can_msgs_Frame_serialize, NULL, \
                      (msg_print_fn_t)can_msgs_Frame_print)

#define BRIDGE_ROS_TO_CAN_FD_FRAME(topic, can_bus) \
    BRIDGE_TOPIC_INIT(topic, BRIDGE_DIR_ROS_TO_CAN_RAW, can_bus, BRIDGE_FRAME_FD, 0, \
                      BRIDGE_PAYLOAD_FD_FRAME, \
                      sizeof(can_msgs_FDFrame), can_msgs_FDFrame, NULL, \
                      (cdr_deserialize_fn_t)can_msgs_FDFrame_deserialize, \
                      (msg_print_fn_t)can_msgs_FDFrame_print)

#define BRIDGE_CAN_TO_ROS_FD_FRAME(topic, can_bus) \
    BRIDGE_TOPIC_INIT(topic, BRIDGE_DIR_CAN_TO_ROS_RAW, can_bus, BRIDGE_FRAME_FD, 0, \
                      BRIDGE_PAYLOAD_FD_FRAME, \
                      sizeof(can_msgs_FDFrame), can_msgs_FDFrame, \
                      (cdr_serialize_fn_t)can_msgs_FDFrame_serialize, NULL, \
                      (msg_print_fn_t)can_msgs_FDFrame_print)

/**
 * MASTER TOPIC TABLE: Add your sensors, actuators, and ping-pong devices here.
 */

// ここにデータ送受信を定義
// can <--> ros2の方向に注意
static const bridge_topic_t g_bridge_topics[] = {
    /* 1. CAN -> ROS 2: MotorStatus (28B = 4 CAN frames: 0x100..0x103) */
    BRIDGE_CAN_TO_ROS("motor_status", MotorStatus, 0x100,
                      BRIDGE_FDCAN1, BRIDGE_FRAME_CLASSIC),

    /* 2. CAN -> ROS 2: ImuData (24B = 3 CAN frames: 0x200..0x202) */
    BRIDGE_CAN_TO_ROS("imu_data", ImuData, 0x200,
                      BRIDGE_FDCAN1, BRIDGE_FRAME_CLASSIC),

    /* 3. ROS 2 -> CAN: MotorCommand (8B = 1 CAN frame: 0x300) */
    BRIDGE_ROS_TO_CAN("motor_command", MotorCommand, 0x300,
                      BRIDGE_FDCAN1, BRIDGE_FRAME_CLASSIC),

    /* 4. Standalone CAN Receiver: Ping (rx: 0x400) */
    BRIDGE_CAN_RECV("ping_echo", Ping, 0x400,
                    BRIDGE_FDCAN1, BRIDGE_FRAME_CLASSIC),
    // 成功！
    BRIDGE_CAN_TO_ROS("kokura_speak", Ping, 0x400,
                      BRIDGE_FDCAN1, BRIDGE_FRAME_CLASSIC),

    /* 5. Generic classic CAN transport for sensor gateways */
    BRIDGE_ROS_TO_CAN_FRAME("can/tx", CONFIG_CAN_CLASSIC_BUS),
    BRIDGE_CAN_TO_ROS_FRAME("can/rx", BRIDGE_CAN_ANY),

    /* 6. Generic CAN-FD transport */
    BRIDGE_ROS_TO_CAN_FD_FRAME("canfd/tx", CONFIG_CAN_FD_BUS),
    BRIDGE_CAN_TO_ROS_FD_FRAME("canfd/rx", BRIDGE_CAN_ANY),
};

#define BRIDGE_TOPIC_COUNT  (sizeof(g_bridge_topics) / sizeof(g_bridge_topics[0]))
#define BRIDGE_MAX_MSG_SIZE 128

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
