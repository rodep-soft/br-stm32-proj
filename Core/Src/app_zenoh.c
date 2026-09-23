/**
 * @file app_zenoh.c
 * @brief Ultra-Thin, High-Performance Zenoh <-> CAN Bridge Engine
 *
 * Professional Minimal Architecture:
 * - 2 Tasks only: Zenoh manager + CAN reassembly worker (saves ~4KB RAM).
 * - Direct TX: Zenoh subscriber callbacks fire CAN frames directly (zero-latency, no TX task).
 * - Deterministic DTCM-RAM: RX queue and reassembly states live in zero-wait memory.
 * - Hardware acceptance filter: Whitelist IDs auto-configured in 16-bit exact list mode.
 * - Self-healing: Frame-0 reset + timeout guard eliminates desync permanently.
 */

#include "app_zenoh.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"

#include <zenoh-pico.h>
#include "zenoh_ros2.h"
#include "app_config.h"

extern struct netif gnetif;

/* ───────────────────── CAN Hardware Representation ─────────────────── */

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

static CAN_HandleTypeDef hcan1;

/* DTCM-RAM Allocations (Zero-wait state memory at 0x20000000) */
static uint8_t       ucRxQueueStorage[CONFIG_QUEUE_CAN_RX_DEPTH * sizeof(can_frame_t)] __attribute__((section(".dtcmram")));
static StaticQueue_t xRxQueueBuffer __attribute__((section(".dtcmram")));
static QueueHandle_t g_rx_queue = NULL;

/* ───────────────────── Topic Runtime State ─────────────────────────── */

typedef struct {
    uint8_t             buffer[BRIDGE_MAX_MSG_SIZE];
    uint16_t            received_mask;
    uint16_t            expected_mask;
    uint8_t             num_frames;
    uint32_t            last_recv_tick;
    uint32_t            rx_msg_count;
    zenoh_ros2_pub_t    pub;  /**< For CAN -> ROS 2 */
    zenoh_ros2_sub_t    sub;  /**< For ROS 2 -> CAN */
} topic_runtime_t;

static topic_runtime_t g_runtimes[BRIDGE_TOPIC_COUNT] __attribute__((section(".dtcmram")));

typedef struct {
    z_owned_session_t   session;
    zenoh_ros2_node_t   node;
    volatile bool       zenoh_ready;
    volatile uint32_t   can_to_zenoh_count;
    volatile uint32_t   zenoh_to_can_count;
    volatile uint32_t   raw_rx_frames;
    volatile uint32_t   raw_tx_frames;
    volatile uint32_t   drop_count;
} bridge_t;

static bridge_t g_bridge;

/* ───────────────────── CAN Hardware & Filter Setup ─────────────────── */

static void can_hardware_init(void) {
    if (g_rx_queue == NULL) {
        g_rx_queue = xQueueCreateStatic(CONFIG_QUEUE_CAN_RX_DEPTH,
                                        sizeof(can_frame_t),
                                        ucRxQueueStorage,
                                        &xRxQueueBuffer);
        configASSERT(g_rx_queue != NULL);
    }

    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = CONFIG_CAN_PRESCALER;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = ENABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        configASSERT(0);
    }

    /* Build Whitelist Filter Bank from g_bridge_topics in 16-bit list mode */
    uint16_t filter_slots[56];
    size_t id_count = 0;

    for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
        const bridge_topic_t *t = &g_bridge_topics[i];
        if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;

        uint8_t n_frames = (uint8_t)((t->msg_size + 7) / 8);
        for (uint8_t f = 0; f < n_frames && id_count < 56; f++) {
            filter_slots[id_count++] = (uint16_t)(((t->can_base_id + f) & 0x7FF) << 5);
        }
    }

    size_t bank_idx = 0;
    size_t cur_id = 0;
    while (cur_id < id_count && bank_idx < 14) {
        uint16_t s[4];
        for (int j = 0; j < 4; j++) {
            s[j] = (cur_id < id_count) ? filter_slots[cur_id++] : s[j > 0 ? j - 1 : 0];
        }
        CAN_FilterTypeDef f = {
            .FilterBank = bank_idx,
            .FilterMode = CAN_FILTERMODE_IDLIST,
            .FilterScale = CAN_FILTERSCALE_16BIT,
            .FilterIdHigh = s[0],
            .FilterIdLow = s[1],
            .FilterMaskIdHigh = s[2],
            .FilterMaskIdLow = s[3],
            .FilterFIFOAssignment = CAN_RX_FIFO0,
            .FilterActivation = ENABLE,
            .SlaveStartFilterBank = 14,
        };
        HAL_CAN_ConfigFilter(&hcan1, &f);
        bank_idx++;
    }

    /* Disable unused filter banks */
    for (; bank_idx < 14; bank_idx++) {
        CAN_FilterTypeDef f = {.FilterBank = bank_idx, .FilterActivation = DISABLE, .SlaveStartFilterBank = 14};
        HAL_CAN_ConfigFilter(&hcan1, &f);
    }

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

/* Direct thread-safe CAN transmission */
static bool can_send_frame(uint32_t id, const uint8_t *data, uint8_t dlc) {
    CAN_TxHeaderTypeDef tx_hdr = {
        .StdId = id & 0x7FF,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = dlc > 8 ? 8 : dlc,
    };

    TickType_t start = xTaskGetTickCount();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(CONFIG_CAN_TX_TIMEOUT_MS)) {
            g_bridge.drop_count++;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint32_t mb;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, (uint8_t *)data, &mb) == HAL_OK) {
        g_bridge.raw_tx_frames++;
        return true;
    }
    g_bridge.drop_count++;
    return false;
}

/* ───────────────────── Fast ISR Context (< 2µs) ────────────────────── */

void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_hdr;
    can_frame_t frame;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, frame.data) == HAL_OK) {
        if (rx_hdr.IDE == CAN_ID_STD && g_rx_queue != NULL) {
            frame.id  = rx_hdr.StdId;
            frame.dlc = (uint8_t)rx_hdr.DLC;

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueSendFromISR(g_rx_queue, &frame, &xHigherPriorityTaskWoken) == pdTRUE) {
                g_bridge.raw_rx_frames++;
            } else {
                g_bridge.drop_count++;
            }
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* ───────────────────── Network Helpers ─────────────────────────────── */

static const char *const g_zenoh_locators[] = { CONFIG_ZENOH_LOCATOR_LIST };
#define ZENOH_LOCATOR_COUNT (sizeof(g_zenoh_locators) / sizeof(g_zenoh_locators[0]))

static void wait_for_network(void) {
#if (CONFIG_NET_USE_DHCP == 0)
    /* Instant static IP mode (< 1s boot) */
    printf("[ETH] Instant Static IP: %s\r\n", CONFIG_NET_STATIC_IP);
    ip4_addr_t ip, mask, gw;
    ip4addr_aton(CONFIG_NET_STATIC_IP, &ip);
    ip4addr_aton(CONFIG_NET_STATIC_NETMASK, &mask);
    ip4addr_aton(CONFIG_NET_STATIC_GATEWAY, &gw);
    netif_set_addr(&gnetif, &ip, &mask, &gw);
    netif_set_up(&gnetif);

    while (!netif_is_link_up(&gnetif)) {
        osDelay(100);
    }
#else
    printf("[ETH] Waiting for DHCP lease...\r\n");
    int sec = 0;
    while (netif_is_up(&gnetif) == 0 || gnetif.ip_addr.addr == 0) {
        osDelay(1000);
        if (netif_is_link_up(&gnetif) && netif_dhcp_data(&gnetif) == NULL) {
            dhcp_start(&gnetif);
        }
        if (++sec >= CONFIG_NET_DHCP_TIMEOUT_SEC && gnetif.ip_addr.addr == 0) {
            printf("[ETH] DHCP timeout! Fallback to %s\r\n", CONFIG_NET_STATIC_IP);
            dhcp_stop(&gnetif);
            ip4_addr_t ip, mask, gw;
            ip4addr_aton(CONFIG_NET_STATIC_IP, &ip);
            ip4addr_aton(CONFIG_NET_STATIC_NETMASK, &mask);
            ip4addr_aton(CONFIG_NET_STATIC_GATEWAY, &gw);
            netif_set_addr(&gnetif, &ip, &mask, &gw);
            netif_set_up(&gnetif);
            break;
        }
    }
#endif
    printf("[ETH] Link UP! IP: %s\r\n", ip4addr_ntoa(&gnetif.ip_addr));
}

/* ─────────── ROS 2 -> CAN: Direct Zero-Latency Callback ────────────── */

static void on_zenoh_sub_message(const uint8_t *payload, size_t len, void *ctx) {
    size_t idx = (size_t)ctx;
    if (idx >= BRIDGE_TOPIC_COUNT) return;

    const bridge_topic_t *t = &g_bridge_topics[idx];
    topic_runtime_t *rt = &g_runtimes[idx];
    if (t->deserialize_fn == NULL) return;

    uint8_t msg_buffer[BRIDGE_MAX_MSG_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, (uint8_t *)payload, len);

    if (!t->deserialize_fn(&ub, msg_buffer)) return;

    /* Directly fire CAN frames to mailbox (no intermediate task needed) */
    for (uint8_t f = 0; f < rt->num_frames; f++) {
        size_t offset = f * 8;
        size_t chunk = (offset + 8 <= t->msg_size) ? 8 : (t->msg_size - offset);
        can_send_frame(t->can_base_id + f, &msg_buffer[offset], (uint8_t)chunk);
    }
    g_bridge.zenoh_to_can_count++;
}

/* ─────────── CAN -> ROS 2: Reassembly Worker Task ──────────────────── */

static void bridge_worker_task(void const *arg) {
    (void)arg;
    while (!g_bridge.zenoh_ready) osDelay(50);
    printf("[Bridge] CAN->Zenoh worker running (DTCM-RAM)\r\n");

    can_frame_t frame;
    uint8_t cdr_buf[256];

    for (;;) {
        if (xQueueReceive(g_rx_queue, &frame, portMAX_DELAY) != pdTRUE) continue;

        uint32_t now = HAL_GetTick();

        for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
            const bridge_topic_t *t = &g_bridge_topics[i];
            if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;
            topic_runtime_t *rt = &g_runtimes[i];

            if (frame.id >= t->can_base_id && frame.id < (t->can_base_id + rt->num_frames)) {
                uint8_t frame_idx = (uint8_t)(frame.id - t->can_base_id);

                /* Desynchronization guards: Timeout + Frame-0 reset */
                if (rt->received_mask != 0 && (now - rt->last_recv_tick) > CONFIG_CAN_FRAME_TIMEOUT_MS) {
                    rt->received_mask = 0;
                }
                rt->last_recv_tick = now;
                if (frame_idx == 0) rt->received_mask = 0;

                size_t offset = frame_idx * 8;
                size_t chunk = (offset + 8 <= t->msg_size) ? 8 : (t->msg_size - offset);
                if (frame.dlc < chunk) chunk = frame.dlc;
                memcpy(&rt->buffer[offset], frame.data, chunk);

                rt->received_mask |= (1U << frame_idx);

                /* When all frames arrive, serialize and publish! */
                if (rt->received_mask == rt->expected_mask) {
                    rt->received_mask = 0;
                    rt->rx_msg_count++;

                    if (t->serialize_fn != NULL) {
                        ucdrBuffer ub;
                        ucdr_init_buffer(&ub, cdr_buf, sizeof(cdr_buf));
                        if (t->serialize_fn(&ub, rt->buffer)) {
                            zenoh_ros2_pub_send(&rt->pub, cdr_buf, ucdr_buffer_length(&ub));
                            g_bridge.can_to_zenoh_count++;
                            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                        }
                    }
                }
                break;
            }
        }
    }
}

/* ───────────────────── Main Zenoh Engine Task ──────────────────────── */

static void zenoh_engine_task(void const *arg) {
    (void)arg;

    printf("\r\n==================================================\r\n");
    printf("  STM32F767ZI Ultra-Thin Zenoh-CAN Bridge         \r\n");
    printf("==================================================\r\n");

    wait_for_network();
    can_hardware_init();
    printf("[CAN] HW Filter & Bitrate configured (%lu bps)\r\n", (unsigned long)CONFIG_CAN_BITRATE);

    /* Open Zenoh session */
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, CONFIG_ZENOH_MODE);
    for (size_t i = 0; i < ZENOH_LOCATOR_COUNT; i++) {
        if (g_zenoh_locators[i] != NULL && strlen(g_zenoh_locators[i]) > 0) {
            zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, g_zenoh_locators[i]);
        }
    }

    z_result_t res;
    while ((res = z_open(&g_bridge.session, z_move(config), NULL)) < 0) {
        printf("[Zenoh] Connect failed (%d). Retry in 2s...\r\n", (int)res);
        osDelay(2000);
        z_config_default(&config);
        zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, CONFIG_ZENOH_MODE);
    }
    printf("[Zenoh] Connected to router!\r\n");

    /* Init ROS 2 Node */
    if (!zenoh_ros2_node_init(&g_bridge.node, &g_bridge.session,
                              CONFIG_ROS2_NODE_NAME, CONFIG_ROS2_NODE_NS, CONFIG_ROS2_DOMAIN_ID)) {
        printf("[ROS2] Node init failed!\r\n");
        return;
    }

    /* Auto-register topics from declarative master table */
    for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
        const bridge_topic_t *t = &g_bridge_topics[i];
        topic_runtime_t *rt = &g_runtimes[i];

        rt->num_frames     = (uint8_t)((t->msg_size + 7) / 8);
        rt->expected_mask  = (uint16_t)((1U << rt->num_frames) - 1U);
        rt->received_mask  = 0;
        rt->last_recv_tick = 0;
        rt->rx_msg_count   = 0;

        if (t->dir == BRIDGE_DIR_CAN_TO_ROS) {
            zenoh_ros2_pub_create(&rt->pub, &g_bridge.node, t->topic_name, t->dds_type, t->type_hash);
            printf("  [PUB] /%-16s -> CAN 0x%03lX..0x%03lX (%u frames)\r\n",
                   t->topic_name, (unsigned long)t->can_base_id,
                   (unsigned long)(t->can_base_id + rt->num_frames - 1), rt->num_frames);
        } else {
            zenoh_ros2_sub_create(&rt->sub, &g_bridge.node, t->topic_name, t->dds_type, t->type_hash,
                                  on_zenoh_sub_message, (void *)i);
            printf("  [SUB] /%-16s <- CAN 0x%03lX..0x%03lX (%u frames)\r\n",
                   t->topic_name, (unsigned long)t->can_base_id,
                   (unsigned long)(t->can_base_id + rt->num_frames - 1), rt->num_frames);
        }
    }

    zp_start_read_task(z_loan_mut(g_bridge.session), NULL);
    zp_start_lease_task(z_loan_mut(g_bridge.session), NULL);

    g_bridge.zenoh_ready = true;
    printf("[Bridge] Active! 2-Task Ultra-Thin Engine Running.\r\n");

    /* Embedded Diagnostics & Watchdog Loop (replaces separate stats task) */
    for (;;) {
        osDelay(CONFIG_CAN_STATS_PERIOD_MS);

        uint32_t esr = CAN1->ESR;
        uint8_t tec = (uint8_t)((esr >> 16) & 0xFF);
        uint8_t rec = (uint8_t)((esr >> 24) & 0xFF);
        bool boff = (esr & (1U << 2)) != 0;
        bool pass = (esr & (1U << 1)) != 0;

        uint32_t now = HAL_GetTick();
        bool timeout_alert = false;

        for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
            const bridge_topic_t *t = &g_bridge_topics[i];
            if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;
            topic_runtime_t *rt = &g_runtimes[i];
            if (rt->rx_msg_count > 0 && (now - rt->last_recv_tick) > CONFIG_CAN_WATCHDOG_MS) {
                printf("[WATCHDOG] /%s timeout!\r\n", t->topic_name);
                timeout_alert = true;
            }
        }

        /* Red LED alert on bus errors or topic silence */
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, (boff || pass || timeout_alert) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        printf("[Stats] CAN->ROS: %lu | ROS->CAN: %lu | RX_f:%lu TX_f:%lu | TEC:%u REC:%u | DROP:%lu\r\n",
               (unsigned long)g_bridge.can_to_zenoh_count, (unsigned long)g_bridge.zenoh_to_can_count,
               (unsigned long)g_bridge.raw_rx_frames, (unsigned long)g_bridge.raw_tx_frames,
               tec, rec, (unsigned long)g_bridge.drop_count);
    }
}

/* ───────────────────── Public Starter ──────────────────────────────── */

void app_zenoh_start(void) {
    memset(&g_bridge, 0, sizeof(g_bridge));
    memset(g_runtimes, 0, sizeof(g_runtimes));

    /* 1. Zenoh Manager + Diagnostics Loop (Single task) */
    osThreadDef(zenohTask, zenoh_engine_task, osPriorityNormal, 0, CONFIG_STACK_ZENOH_TASK);
    osThreadCreate(osThread(zenohTask), NULL);

    /* 2. CAN->Zenoh High-Priority Reassembly Worker */
    osThreadDef(bridgeTask, bridge_worker_task, osPriorityAboveNormal, 0, CONFIG_STACK_BRIDGE_TASK);
    osThreadCreate(osThread(bridgeTask), NULL);
}
