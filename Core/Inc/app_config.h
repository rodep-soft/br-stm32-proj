/**
 * @file app_config.h
 * @brief Production-Grade Centralized Configuration for STM32 Zenoh-CAN Bridge
 *
 * All user settings are consolidated here:
 * 1. Network / IP Configuration (Instant Static IP or DHCP Fallback)
 * 2. Zenoh Locators & ROS 2 Graph Domain
 * 3. CAN Bus Bitrate & System Timing
 * 4. FreeRTOS Task Stacks & Queue Depths
 *
 * Robust Design:
 * - Fail-fast compile-time validation (#error on invalid settings)
 * - Automatic hardware timing derivation from human-readable bitrate
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================================================================
 * 1. Network (Ethernet) Configuration
 * ============================================================================== */
/**
 * Set to 0 for instant Static IP (Recommended for direct robot PC connection, boot < 1s)
 * Set to 1 to try DHCP first, falling back to Static IP after timeout
 */
#define CONFIG_NET_USE_DHCP           0

#define CONFIG_NET_STATIC_IP          "192.168.50.77"
#define CONFIG_NET_STATIC_NETMASK     "255.255.255.0"
#define CONFIG_NET_STATIC_GATEWAY     "192.168.50.1"

#define CONFIG_NET_DHCP_TIMEOUT_SEC   5   /**< Seconds before falling back to static IP */

/* ==============================================================================
 * 2. Zenoh & ROS 2 Configuration
 * ============================================================================== */
#define CONFIG_ZENOH_MODE             "client"

/**
 * Zenoh router endpoints (UDP port 7447).
 * Connects to the first reachable locator.
 */
#define CONFIG_ZENOH_LOCATOR_LIST \
    "udp/192.168.50.30:7447", \
    "udp/192.168.50.10:7447", \
    "udp/192.168.50.50:7447", \
    "udp/192.168.50.150:7447"

#define CONFIG_ROS2_NODE_NAME         "stm32_bridge"
#define CONFIG_ROS2_NODE_NS           "/"
#define CONFIG_ROS2_DOMAIN_ID         0

/* ==============================================================================
 * 3. CAN Hardware & Bitrate Configuration
 * ============================================================================== */
/**
 * Supported Bitrates: 1000000 (1M), 500000 (500k), 250000 (250k), 125000 (125k)
 * Sample point is fixed at automotive-optimal 87.5% (16 TQ).
 */
#define CONFIG_CAN_BITRATE            500000U

/* Timing & Diagnostic Watchdogs */
#define CONFIG_CAN_STATS_PERIOD_MS    3000   /**< Diagnostics log interval */
#define CONFIG_CAN_WATCHDOG_MS        1500   /**< Heartbeat silence timeout before Red LED alert */
#define CONFIG_CAN_FRAME_TIMEOUT_MS   100    /**< Incomplete multi-frame reassembly drop timeout */
#define CONFIG_CAN_TX_TIMEOUT_MS      10     /**< Mailbox wait timeout before frame drop */

/* ==============================================================================
 * 4. System Queues & FreeRTOS Resources
 * ============================================================================== */
#define CONFIG_QUEUE_CAN_RX_DEPTH     64     /**< Raw CAN frame RX queue depth (in DTCM-RAM) */
#define CONFIG_QUEUE_CAN_TX_DEPTH     32     /**< Raw CAN frame TX queue depth (in DTCM-RAM) */

#define CONFIG_STACK_ZENOH_TASK       2048   /**< Stack words for Zenoh session manager */
#define CONFIG_STACK_BRIDGE_TASK      1024   /**< Stack words for CAN->Zenoh worker */
#define CONFIG_STACK_CAN_TX_TASK      512    /**< Stack words for CAN TX worker */
#define CONFIG_STACK_STATS_TASK       256    /**< Stack words for diagnostics task */

/* ==============================================================================
 * 5. Automatic Compile-Time Validation & Hardware Timing Derivation
 * ============================================================================== */
#if (CONFIG_CAN_BITRATE != 1000000U && \
     CONFIG_CAN_BITRATE != 500000U  && \
     CONFIG_CAN_BITRATE != 250000U  && \
     CONFIG_CAN_BITRATE != 125000U)
#error "Invalid CONFIG_CAN_BITRATE! Supported values: 1000000, 500000, 250000, 125000"
#endif

/* Auto-derive prescaler: APB1=48MHz, 16 TQ per bit -> 48000000 / (16 * bitrate) */
#define CONFIG_CAN_PRESCALER          (48000000U / (16U * (CONFIG_CAN_BITRATE)))
#define CONFIG_CAN_BS1                13     /* CAN_BS1_13TQ */
#define CONFIG_CAN_BS2                2      /* CAN_BS2_2TQ  */
#define CONFIG_CAN_SJW                1      /* CAN_SJW_1TQ  */

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
