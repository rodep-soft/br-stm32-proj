/**
 * @file app_config.h
 * @brief Centralized Application & Network Configuration for STM32
 *
 * All user-modifiable parameters (IP addresses, Zenoh locators, CAN baudrates,
 * ROS 2 topic names) are consolidated in this single header.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================================================================
 * 1. Network (Ethernet / Static IP) Configuration
 * ============================================================================== */
#define CONFIG_STATIC_IP_ADDR0        192
#define CONFIG_STATIC_IP_ADDR1        168
#define CONFIG_STATIC_IP_ADDR2        50
#define CONFIG_STATIC_IP_ADDR3        10

#define CONFIG_STATIC_NETMASK0        255
#define CONFIG_STATIC_NETMASK1        255
#define CONFIG_STATIC_NETMASK2        255
#define CONFIG_STATIC_NETMASK3        0

#define CONFIG_STATIC_GW_ADDR0        192
#define CONFIG_STATIC_GW_ADDR1        168
#define CONFIG_STATIC_GW_ADDR2        50
#define CONFIG_STATIC_GW_ADDR3        1

/* Helper strings for logging / display */
#define CONFIG_STATIC_IP_STR          "192.168.50.10"
#define CONFIG_STATIC_NETMASK_STR     "255.255.255.0"
#define CONFIG_STATIC_GW_STR          "192.168.50.1"

/* ==============================================================================
 * 2. Zenoh Communication Configuration
 * ============================================================================== */
#define CONFIG_ZENOH_MODE             "client"

/* Candidate endpoints to connect to (PC / Zenoh routers, UDP port 7447) */
#define CONFIG_ZENOH_LOCATOR_1        "udp/192.168.50.30:7447"
#define CONFIG_ZENOH_LOCATOR_2        "udp/192.168.50.2:7447"
#define CONFIG_ZENOH_LOCATOR_3        "udp/192.168.50.50:7447"
#define CONFIG_ZENOH_LOCATOR_4        "udp/192.168.50.150:7447"

/* ==============================================================================
 * 3. ROS 2 Settings
 * ============================================================================== */
#define CONFIG_ROS2_NODE_NAME         "stm32_node"
#define CONFIG_ROS2_NODE_NS           "/"
#define CONFIG_ROS2_DOMAIN_ID         0

#define CONFIG_ROS2_TOPIC_CHATTER     "chatter"
#define CONFIG_ROS2_TOPIC_CAN_FRAME   "can_msgs/frame"
#define CONFIG_ROS2_CHATTER_PERIOD_MS 1000

/* ==============================================================================
 * 4. CAN Bus Configuration
 * ============================================================================== */
/* Supported common baudrates: 1000000U (1M), 500000U (500k), 250000U (250k), 125000U (125k) */
#define CONFIG_CAN_BAUDRATE           500000U

/* FreeRTOS queue capacity for incoming CAN frames */
#define CONFIG_CAN_QUEUE_SIZE         64

/* CAN transmission mailbox wait timeout in milliseconds */
#define CONFIG_CAN_TX_TIMEOUT_MS      5

/* Periodic statistics report interval in milliseconds */
#define CONFIG_CAN_STATS_PERIOD_MS    5000

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
