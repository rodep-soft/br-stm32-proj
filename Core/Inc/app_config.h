/**
 * @file app_config.h
 * @brief Centralized Application & Network Configuration for STM32
 *
 * All user-modifiable parameters (IP addresses, Zenoh locators, CAN baudrates,
 * ROS 2 node/topic configuration) are consolidated in this single header.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================================================================
 * 1. Network (Ethernet / Static IP Fallback) Configuration
 * ============================================================================== */
#define CONFIG_STATIC_IP              "192.168.50.77"
#define CONFIG_STATIC_NETMASK         "255.255.255.0"
#define CONFIG_STATIC_GATEWAY         "192.168.50.1"

/* ==============================================================================
 * 2. Zenoh Communication Configuration
 * ============================================================================== */
#define CONFIG_ZENOH_MODE             "client"

/* Candidate endpoints to connect to (PC / Zenoh routers, UDP port 7447) */
static const char *const CONFIG_ZENOH_LOCATORS[] = {
    "udp/192.168.50.30:7447",
    "udp/192.168.50.10:7447",
    "udp/192.168.50.50:7447",
    "udp/192.168.50.150:7447",
};
#define CONFIG_ZENOH_LOCATOR_COUNT    (sizeof(CONFIG_ZENOH_LOCATORS) / sizeof(CONFIG_ZENOH_LOCATORS[0]))

/* ==============================================================================
 * 3. ROS 2 Settings
 * ============================================================================== */
#define CONFIG_ROS2_NODE_NAME         "stm32_bridge"
#define CONFIG_ROS2_NODE_NS           "/"
#define CONFIG_ROS2_DOMAIN_ID         0

/* ==============================================================================
 * 4. CAN Bus Configuration
 * ============================================================================== */
#define CONFIG_CAN_BAUDRATE           500000U
#define CONFIG_CAN_STATS_PERIOD_MS    3000
#define CONFIG_CAN_WATCHDOG_TIMEOUT_MS 1500

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
