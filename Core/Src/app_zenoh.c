/**
 * @file app_zenoh.c
 * @brief Zenoh ↔ CAN Bridge Application
 *
 * Main application that bridges CAN bus messages to/from ROS 2 via Zenoh-Pico.
 * Architecture:
 *   - zenoh_task: Manages Zenoh session, ROS2 node, publishers/subscribers
 *   - bridge_task: Reads assembled CAN messages from queue, publishes to Zenoh;
 *                  also processes Zenoh→CAN TX queue
 *   - CAN RX ISR: Assembles multi-frame CAN messages, pushes to bridge queue
 */

#include "app_zenoh.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"

#include <zenoh-pico.h>
#include "zenoh_ros2.h"
#include "can_bridge.h"
#include "can_protocol.h"

#include "generated/MotorStatus.h"
#include "generated/ImuData.h"
#include "generated/MotorCommand.h"

extern struct netif gnetif;

/* ─────────────────────────── Configuration ─────────────────────────── */

#define ZENOH_MODE "client"

static const char *const ZENOH_LOCATORS[] = {
    "udp/192.168.50.30:7447",
    "udp/192.168.50.10:7447",
    "udp/192.168.50.50:7447",
    "udp/192.168.50.150:7447",
};
#define ZENOH_LOCATOR_COUNT (sizeof(ZENOH_LOCATORS) / sizeof(ZENOH_LOCATORS[0]))

#define ROS2_NODE_NAME    "stm32_bridge"
#define ROS2_NODE_NS      "/"
#define ROS2_DOMAIN_ID    0

/* ROS 2 topic names */
#define TOPIC_MOTOR_STATUS  "motor_status"
#define TOPIC_IMU_DATA      "imu_data"
#define TOPIC_MOTOR_COMMAND "motor_command"

/* Task stack sizes (words) */
#define ZENOH_TASK_STACK_SIZE   2048
#define BRIDGE_TASK_STACK_SIZE  1024
#define CAN_TX_TASK_STACK_SIZE  512

/* ─────────────────────────── Static State ──────────────────────────── */

/** Bridge context: shared between zenoh task and bridge tasks */
typedef struct {
    /* Zenoh session & ROS 2 node (owned by zenoh_task) */
    z_owned_session_t   session;
    zenoh_ros2_node_t   node;

    /* Publishers: CAN → Zenoh */
    zenoh_ros2_pub_t    pub_motor_status;
    zenoh_ros2_pub_t    pub_imu_data;

    /* Subscribers: Zenoh → CAN */
    zenoh_ros2_sub_t    sub_motor_command;

    /* Lifecycle flags */
    volatile bool       zenoh_ready;

    /* Statistics */
    volatile uint32_t   can_to_zenoh_count;
    volatile uint32_t   zenoh_to_can_count;
} bridge_ctx_t;

static bridge_ctx_t g_ctx;

/* ──────────────────────── Network Helpers ──────────────────────────── */

static void init_zenoh_config(z_owned_config_t *config) {
    z_config_default(config);
    zp_config_insert(z_loan_mut(*config), Z_CONFIG_MODE_KEY, ZENOH_MODE);

    for (size_t i = 0; i < ZENOH_LOCATOR_COUNT; i++) {
        if (ZENOH_LOCATORS[i] != NULL && strlen(ZENOH_LOCATORS[i]) > 0) {
            zp_config_insert(z_loan_mut(*config), Z_CONFIG_CONNECT_KEY, ZENOH_LOCATORS[i]);
        }
    }
}

static void wait_for_network(void) {
    printf("[ETH] Waiting for IP address...\r\n");
    int wait_sec = 0;
    while (netif_is_up(&gnetif) == 0 || gnetif.ip_addr.addr == 0) {
        osDelay(1000);
        wait_sec++;
        struct dhcp *d = netif_dhcp_data(&gnetif);
        uint8_t dhcp_state = d ? d->state : 255;

        if (wait_sec % 2 == 0) {
            printf("[ETH] waiting (%ds)... link=%d, netif=%d, dhcp_state=%u, ip=%s\r\n",
                   wait_sec, netif_is_link_up(&gnetif), netif_is_up(&gnetif),
                   (unsigned int)dhcp_state, ip4addr_ntoa(&gnetif.ip_addr));
        }

        if (netif_is_link_up(&gnetif) && (d == NULL || dhcp_state == DHCP_STATE_OFF)) {
            printf("[ETH] Triggering dhcp_start...\r\n");
            dhcp_start(&gnetif);
        }

        if (wait_sec >= 10 && gnetif.ip_addr.addr == 0) {
            printf("[ETH] DHCP timeout! Falling back to static IP 192.168.50.77...\r\n");
            dhcp_stop(&gnetif);
            ip4_addr_t static_ip, static_mask, static_gw;
            IP4_ADDR(&static_ip, 192, 168, 50, 77);
            IP4_ADDR(&static_mask, 255, 255, 255, 0);
            IP4_ADDR(&static_gw, 192, 168, 50, 1);
            netif_set_addr(&gnetif, &static_ip, &static_mask, &static_gw);
            netif_set_up(&gnetif);
            break;
        }
    }
    printf("[ETH] IP: %s  Mask: %s  GW: %s\r\n",
           ip4addr_ntoa(&gnetif.ip_addr),
           ip4addr_ntoa(&gnetif.netmask),
           ip4addr_ntoa(&gnetif.gw));
}

/* ──────────────── Zenoh → CAN: Subscriber Callback ────────────────── */

/**
 * @brief Called by zenoh-pico when a MotorCommand message arrives from ROS 2.
 *        Deserializes CDR payload and sends to CAN TX queue.
 *        Runs in zenoh's internal receive thread context.
 */
static void on_motor_command(const uint8_t *payload, size_t len, void *ctx) {
    (void)ctx;

    /* Deserialize CDR → struct */
    robot_msgs_MotorCommand cmd;
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, (uint8_t *)payload, len);

    if (!robot_msgs_MotorCommand_deserialize(&ub, &cmd)) {
        return;  /* malformed CDR */
    }

    /* Forward to CAN TX queue */
    can_bridge_tx_msg_t tx_msg;
    tx_msg.type = CAN_MSG_MOTOR_COMMAND;
    tx_msg.data.motor_cmd = cmd;

    if (can_bridge_send(&tx_msg) == pdTRUE) {
        g_ctx.zenoh_to_can_count++;
    }
}

/* ─────────────── CAN → Zenoh: Bridge Task ─────────────────────────── */

/**
 * @brief Bridge task: reads assembled CAN messages and publishes to Zenoh.
 *        This is the hot path - optimized for minimum latency.
 */
static void bridge_can_to_zenoh_task(void const *argument) {
    (void)argument;

    /* Wait for Zenoh session to be ready */
    while (!g_ctx.zenoh_ready) {
        osDelay(100);
    }
    printf("[Bridge] CAN→Zenoh task started\r\n");

    can_bridge_rx_msg_t rx_msg;
    uint8_t cdr_buf[128];  /* Max CDR size: MotorStatus ~32B, ImuData ~28B */

    for (;;) {
        /* Block until a complete CAN message is assembled by ISR */
        if (can_bridge_receive(&rx_msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ucdrBuffer ub;
        size_t cdr_len = 0;

        switch (rx_msg.type) {
        case CAN_MSG_MOTOR_STATUS: {
            ucdr_init_buffer(&ub, cdr_buf, sizeof(cdr_buf));
            if (robot_msgs_MotorStatus_serialize(&ub, &rx_msg.data.motor_status)) {
                cdr_len = ucdr_buffer_length(&ub);
                zenoh_ros2_pub_send(&g_ctx.pub_motor_status, cdr_buf, cdr_len);
                g_ctx.can_to_zenoh_count++;
            }
            break;
        }
        case CAN_MSG_IMU_DATA: {
            ucdr_init_buffer(&ub, cdr_buf, sizeof(cdr_buf));
            if (robot_msgs_ImuData_serialize(&ub, &rx_msg.data.imu_data)) {
                cdr_len = ucdr_buffer_length(&ub);
                zenoh_ros2_pub_send(&g_ctx.pub_imu_data, cdr_buf, cdr_len);
                g_ctx.can_to_zenoh_count++;
            }
            break;
        }
        default:
            break;
        }

        /* Toggle LED on each bridged message */
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}

/* ─────────────── CAN TX Task ──────────────────────────────────────── */

/**
 * @brief CAN TX task: reads from TX queue and sends CAN frames.
 *        Runs at above-normal priority to minimize TX latency.
 */
static void bridge_can_tx_task(void const *argument) {
    (void)argument;

    /* Wait for CAN to be ready */
    while (!g_ctx.zenoh_ready) {
        osDelay(100);
    }
    printf("[Bridge] CAN TX task started\r\n");

    can_bridge_tx_msg_t tx_msg;

    for (;;) {
        if (can_bridge_tx_receive(&tx_msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (tx_msg.type) {
        case CAN_MSG_MOTOR_COMMAND: {
            uint8_t data[8];
            uint8_t dlc;
            can_pack_motor_command(&tx_msg.data.motor_cmd, 0, data, &dlc);

            CAN_TxHeaderTypeDef tx_header;
            tx_header.StdId = CAN_ID_MOTOR_CMD_BASE;
            tx_header.ExtId = 0;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = dlc;
            tx_header.TransmitGlobalTime = DISABLE;

            uint32_t tx_mailbox;
            /* Wait for a free TX mailbox (up to 10ms) */
            uint32_t start = HAL_GetTick();
            while (HAL_CAN_GetTxMailboxesFreeLevel(can_bridge_get_handle()) == 0) {
                if ((HAL_GetTick() - start) > 10) {
                    break;  /* timeout, drop frame */
                }
                osDelay(1);
            }

            HAL_CAN_AddTxMessage(can_bridge_get_handle(), &tx_header, data, &tx_mailbox);
            break;
        }
        default:
            break;
        }
    }
}

/* ─────────────── Stats Task ───────────────────────────────────────── */

static void stats_task(void const *argument) {
    (void)argument;

    for (;;) {
        osDelay(5000);
        if (g_ctx.zenoh_ready) {
            uint32_t rx_cnt, tx_cnt, err_cnt;
            can_bridge_get_stats(&rx_cnt, &tx_cnt, &err_cnt);
            printf("[Stats] CAN→Zenoh: %lu  Zenoh→CAN: %lu  CAN_RX: %lu  CAN_TX: %lu  CAN_ERR: %lu\r\n",
                   (unsigned long)g_ctx.can_to_zenoh_count,
                   (unsigned long)g_ctx.zenoh_to_can_count,
                   (unsigned long)rx_cnt,
                   (unsigned long)tx_cnt,
                   (unsigned long)err_cnt);
        }
    }
}

/* ──────────────────────── Zenoh Main Task ──────────────────────────── */

static void zenoh_task(void const *argument) {
    (void)argument;

    printf("\r\n========================================\r\n");
    printf("  STM32F767ZI Zenoh-CAN Bridge v1.0    \r\n");
    printf("========================================\r\n");

    /* 1. Wait for network */
    wait_for_network();

    /* 2. Initialize CAN bridge (HAL + queues) */
    can_bridge_init();
    printf("[CAN] Bridge initialized (500 kbps)\r\n");

    /* 3. Open Zenoh session */
    z_owned_config_t config;
    init_zenoh_config(&config);

    for (size_t i = 0; i < ZENOH_LOCATOR_COUNT; i++) {
        if (ZENOH_LOCATORS[i] != NULL && strlen(ZENOH_LOCATORS[i]) > 0) {
            printf("[Zenoh] Locator [%u]: %s\r\n", (unsigned int)i, ZENOH_LOCATORS[i]);
        }
    }
    printf("[Zenoh] Mode: %s\r\n", ZENOH_MODE);

    z_result_t res;
    while ((res = z_open(&g_ctx.session, z_move(config), NULL)) < 0) {
        printf("[Zenoh] Session open failed (err: %d). Retry in 2s...\r\n", (int)res);
        osDelay(2000);
        init_zenoh_config(&config);
    }
    printf("[Zenoh] Session opened!\r\n");

    /* 4. Initialize ROS 2 node */
    if (!zenoh_ros2_node_init(&g_ctx.node, &g_ctx.session,
                              ROS2_NODE_NAME, ROS2_NODE_NS, ROS2_DOMAIN_ID)) {
        printf("[ROS2] Node init failed!\r\n");
        z_drop(z_move(g_ctx.session));
        vTaskDelete(NULL);
        return;
    }

    /* 5. Create publishers (CAN → Zenoh) */
    if (!zenoh_ros2_pub_create(&g_ctx.pub_motor_status, &g_ctx.node,
                               TOPIC_MOTOR_STATUS,
                               robot_msgs_MotorStatus_DDS_TYPE,
                               robot_msgs_MotorStatus_TYPE_HASH)) {
        printf("[ROS2] MotorStatus publisher failed!\r\n");
    } else {
        printf("[ROS2] Publisher: /%s\r\n", TOPIC_MOTOR_STATUS);
    }

    if (!zenoh_ros2_pub_create(&g_ctx.pub_imu_data, &g_ctx.node,
                               TOPIC_IMU_DATA,
                               robot_msgs_ImuData_DDS_TYPE,
                               robot_msgs_ImuData_TYPE_HASH)) {
        printf("[ROS2] ImuData publisher failed!\r\n");
    } else {
        printf("[ROS2] Publisher: /%s\r\n", TOPIC_IMU_DATA);
    }

    /* 6. Create subscribers (Zenoh → CAN) */
    if (!zenoh_ros2_sub_create(&g_ctx.sub_motor_command, &g_ctx.node,
                               TOPIC_MOTOR_COMMAND,
                               robot_msgs_MotorCommand_DDS_TYPE,
                               robot_msgs_MotorCommand_TYPE_HASH,
                               on_motor_command, NULL)) {
        printf("[ROS2] MotorCommand subscriber failed!\r\n");
    } else {
        printf("[ROS2] Subscriber: /%s\r\n", TOPIC_MOTOR_COMMAND);
    }

    /* 7. Signal ready and start bridge tasks */
    g_ctx.zenoh_ready = true;
    printf("[Bridge] All systems GO!\r\n");

    /* 8. Start zenoh-pico internal read & lease tasks.
     *    These run as FreeRTOS tasks and handle session keep-alive
     *    and incoming message dispatching automatically. */
    if (zp_start_read_task(z_loan_mut(g_ctx.session), NULL) < 0) {
        printf("[Zenoh] WARNING: Failed to start read task\r\n");
    }
    if (zp_start_lease_task(z_loan_mut(g_ctx.session), NULL) < 0) {
        printf("[Zenoh] WARNING: Failed to start lease task\r\n");
    }
    printf("[Zenoh] Read & lease tasks started\r\n");

    /* 9. This task is now idle — just keep alive for cleanup purposes */
    for (;;) {
        osDelay(10000);
    }

    /* Unreachable cleanup */
    zenoh_ros2_sub_destroy(&g_ctx.sub_motor_command);
    zenoh_ros2_pub_destroy(&g_ctx.pub_imu_data);
    zenoh_ros2_pub_destroy(&g_ctx.pub_motor_status);
    zenoh_ros2_node_fini(&g_ctx.node);
    z_drop(z_move(g_ctx.session));
}

/* ────────────────────── Public Entry Point ─────────────────────────── */

void app_zenoh_start(void) {
    memset(&g_ctx, 0, sizeof(g_ctx));

    /* Zenoh + session management task (highest stack, normal priority) */
    osThreadDef(zenohTask, zenoh_task, osPriorityNormal, 0, ZENOH_TASK_STACK_SIZE);
    osThreadCreate(osThread(zenohTask), NULL);

    /* CAN → Zenoh bridge task (above normal priority for low latency) */
    osThreadDef(bridgeTask, bridge_can_to_zenoh_task, osPriorityAboveNormal, 0, BRIDGE_TASK_STACK_SIZE);
    osThreadCreate(osThread(bridgeTask), NULL);

    /* CAN TX task (above normal priority) */
    osThreadDef(canTxTask, bridge_can_tx_task, osPriorityAboveNormal, 0, CAN_TX_TASK_STACK_SIZE);
    osThreadCreate(osThread(canTxTask), NULL);

    /* Statistics task (low priority) */
    osThreadDef(statsTask, stats_task, osPriorityLow, 0, 256);
    osThreadCreate(osThread(statsTask), NULL);
}
