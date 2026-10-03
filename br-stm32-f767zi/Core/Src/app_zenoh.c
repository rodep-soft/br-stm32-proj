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
#include "FreeRTOS.h"
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
    bool     is_extended;
    bool     is_rtr;
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
    hcan1.Init.SyncJumpWidth        = CAN_SJW_2TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_12TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = ENABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        configASSERT(0);
    }

    /* Check if raw CAN frame bridging is requested */
    bool has_raw_bridge = false;
    for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
        if (g_bridge_topics[i].dir == BRIDGE_DIR_CAN_TO_ROS_RAW ||
            g_bridge_topics[i].dir == BRIDGE_DIR_ROS_TO_CAN_RAW) {
            has_raw_bridge = true;
            break;
        }
    }

    if (has_raw_bridge) {
        /* Accept all standard and extended frames in 32-bit mask mode on Bank 0 */
        CAN_FilterTypeDef f = {
            .FilterBank = 0,
            .FilterMode = CAN_FILTERMODE_IDMASK,
            .FilterScale = CAN_FILTERSCALE_32BIT,
            .FilterIdHigh = 0x0000,
            .FilterIdLow = 0x0000,
            .FilterMaskIdHigh = 0x0000,
            .FilterMaskIdLow = 0x0000,
            .FilterFIFOAssignment = CAN_RX_FIFO0,
            .FilterActivation = ENABLE,
            .SlaveStartFilterBank = 14,
        };
        HAL_CAN_ConfigFilter(&hcan1, &f);

        for (size_t bank_idx = 1; bank_idx < 14; bank_idx++) {
            CAN_FilterTypeDef f_dis = {.FilterBank = bank_idx, .FilterActivation = DISABLE, .SlaveStartFilterBank = 14};
            HAL_CAN_ConfigFilter(&hcan1, &f_dis);
        }
        printf("[CAN] HW Filter: Raw Transparent Bridge Mode (Accept All Standard/Extended)\r\n");
    } else {
        /* Build Whitelist Filter Bank from g_bridge_topics in 16-bit list mode */
        uint16_t filter_slots[56];
        size_t id_count = 0;

        for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
            const bridge_topic_t *t = &g_bridge_topics[i];
            if (t->dir == BRIDGE_DIR_ROS_TO_CAN ||
                t->dir == BRIDGE_DIR_ROS_TO_CAN_RAW ||
                t->dir == BRIDGE_DIR_CAN_TO_ROS_RAW) continue;

            if (t->dir == BRIDGE_DIR_CAN_RECV) {
                if (id_count < 56) {
                    filter_slots[id_count++] = (uint16_t)(((t->can_base_id) & 0x7FF) << 5);
                }
                continue;
            }

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
    }

    HAL_StatusTypeDef start_res = HAL_CAN_Start(&hcan1);
    printf("[CAN] HAL_CAN_Start result: %d (state=%d, err=0x%08lX, MSR=0x%08lX)\r\n",
           (int)start_res, (int)hcan1.State, (unsigned long)hcan1.ErrorCode, (unsigned long)hcan1.Instance->MSR);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    printf("[CAN] HW Filter & Bitrate configured (%lu bps)\r\n", (unsigned long)CONFIG_CAN_BITRATE);
}

/* Direct thread-safe CAN transmission supporting Standard (11-bit) and Extended (29-bit) IDs */
static bool can_send_frame_ex(uint32_t id, const uint8_t *data, uint8_t dlc, bool is_extended, bool is_rtr) {
    if (data == NULL) {
        g_bridge.drop_count++;
        return false;
    }

    CAN_TxHeaderTypeDef tx_hdr = {
        .DLC = dlc > 8 ? 8 : dlc,
        .RTR = is_rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA,
    };

    if (is_extended || id > 0x7FFU) {
        tx_hdr.IDE = CAN_ID_EXT;
        tx_hdr.ExtId = id & 0x1FFFFFFFU;
    } else {
        tx_hdr.IDE = CAN_ID_STD;
        tx_hdr.StdId = id & 0x7FFU;
    }

    /* Ensure CAN controller is listening / ready */
    if (hcan1.State != HAL_CAN_STATE_LISTENING && hcan1.State != HAL_CAN_STATE_READY) {
        printf("[CAN-TX-WARN] hcan1.State=%d, restarting CAN...\r\n", (int)hcan1.State);
        HAL_CAN_Start(&hcan1);
    }

    TickType_t start = xTaskGetTickCount();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(CONFIG_CAN_TX_TIMEOUT_MS)) {
            printf("[CAN-TX-ERR] Tx Mailbox Full! TSR=0x%08lX free=%lu state=%d err=0x%08lX\r\n",
                   (unsigned long)hcan1.Instance->TSR,
                   (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
                   (int)hcan1.State, (unsigned long)hcan1.ErrorCode);
            HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            g_bridge.drop_count++;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint32_t mb;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, (uint8_t *)data, &mb);
    if (status == HAL_OK) {
        g_bridge.raw_tx_frames++;
        return true;
    }
    printf("[CAN-TX-ERR] HAL_CAN_AddTxMessage failed: status=%d, err=0x%08lX, state=%d TSR=0x%08lX\r\n",
           (int)status, (unsigned long)hcan1.ErrorCode, (int)hcan1.State, (unsigned long)hcan1.Instance->TSR);
    g_bridge.drop_count++;
    return false;
}

static inline bool can_send_frame(uint32_t id, const uint8_t *data, uint8_t dlc) {
    return can_send_frame_ex(id, data, dlc, (id > 0x7FFU), false);
}

/* ───────────────────── Fast ISR Context (< 2µs) ────────────────────── */

void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_hdr;
    can_frame_t frame;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, frame.data) == HAL_OK) {
        if (g_rx_queue != NULL) {
            if (rx_hdr.IDE == CAN_ID_EXT) {
                frame.id = rx_hdr.ExtId;
                frame.is_extended = true;
            } else {
                frame.id = rx_hdr.StdId;
                frame.is_extended = false;
            }
            frame.is_rtr = (rx_hdr.RTR == CAN_RTR_REMOTE);
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

static void configure_zenoh_session(z_owned_config_t *config) {
    z_config_default(config);
    zp_config_insert(z_loan_mut(*config), Z_CONFIG_MODE_KEY, CONFIG_ZENOH_MODE);
    for (size_t i = 0; i < ZENOH_LOCATOR_COUNT; i++) {
        if (g_zenoh_locators[i] != NULL && strlen(g_zenoh_locators[i]) > 0) {
            zp_config_insert(z_loan_mut(*config), Z_CONFIG_CONNECT_KEY, g_zenoh_locators[i]);
        }
    }
}

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

    uint32_t wait_count = 0;
    while (!netif_is_link_up(&gnetif)) {
        if (wait_count++ % 30 == 0) {
            printf("[ETH] Waiting for Ethernet cable link (plug in cable to connect Zenoh)...\r\n");
        }
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

    /* Raw CAN Frame -> Direct CAN TX */
    if (t->dir == BRIDGE_DIR_ROS_TO_CAN_RAW) {
        const can_msgs_Frame *cf = (const can_msgs_Frame *)msg_buffer;
        printf("[RAW-TX] id=0x%08lX ext=%d dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               (unsigned long)cf->id, cf->is_extended, cf->dlc,
               cf->data[0], cf->data[1], cf->data[2], cf->data[3],
               cf->data[4], cf->data[5], cf->data[6], cf->data[7]);
        can_send_frame_ex(cf->id, cf->data, cf->dlc, cf->is_extended, cf->is_rtr);
        g_bridge.zenoh_to_can_count++;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        return;
    }

    printf("[ROS-RX] /%s: ", t->topic_name);
    if (t->print_fn != NULL) {
        t->print_fn(msg_buffer);
    } else {
        printf("(%zu bytes)\r\n", t->msg_size);
    }

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
    printf("[Bridge] CAN worker running (DTCM-RAM)\r\n");

    can_frame_t frame;
    uint8_t cdr_buf[256];

    for (;;) {
        if (g_rx_queue == NULL) {
            osDelay(10);
            continue;
        }
        if (xQueueReceive(g_rx_queue, &frame, portMAX_DELAY) != pdTRUE) continue;

        uint32_t now = HAL_GetTick();

        if (frame.id != 0x400) {
            printf("[RAW-RX] id=0x%08lX ext=%d dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                   (unsigned long)frame.id, frame.is_extended, frame.dlc,
                   frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                   frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        }

        /* 1. Raw Transparent CAN Frame Forwarding -> ROS 2 (/from_can_bus) */
        if (g_bridge.zenoh_ready) {
            for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
                const bridge_topic_t *t = &g_bridge_topics[i];
                if (t->dir == BRIDGE_DIR_CAN_TO_ROS_RAW) {
                    topic_runtime_t *rt = &g_runtimes[i];

                    can_msgs_Frame out_frame;
                    memset(&out_frame, 0, sizeof(out_frame));
                    strncpy(out_frame.header.frame_id, "can1", sizeof(out_frame.header.frame_id) - 1);
                    out_frame.id = frame.id;
                    out_frame.is_extended = frame.is_extended;
                    out_frame.is_rtr = frame.is_rtr;
                    out_frame.is_error = false;
                    out_frame.dlc = frame.dlc;
                    memcpy(out_frame.data, frame.data, frame.dlc > 8 ? 8 : frame.dlc);

                    ucdrBuffer ub;
                    ucdr_init_buffer(&ub, cdr_buf, sizeof(cdr_buf));
                    if (can_msgs_Frame_serialize(&ub, &out_frame)) {
                        zenoh_ros2_pub_send(&rt->pub, cdr_buf, ucdr_buffer_length(&ub));
                        g_bridge.can_to_zenoh_count++;
                        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                    }
                }
            }
        }

        /* 2. Topic-specific CAN Handlers (Standalone Receiver & Multi-frame Reassembly) */
        for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
            const bridge_topic_t *t = &g_bridge_topics[i];

            /* Standalone CAN Receiver: log to serial (USART3 / ST-LINK VCP) and toggle LED */
            if (t->dir == BRIDGE_DIR_CAN_RECV) {
                if (frame.id == t->can_base_id) {
                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                    static uint32_t last_print = 0;
                    if (now - last_print >= 1000) {
                        last_print = now;
                        printf("[CAN-RX] %s (0x%03lX): ", t->topic_name, (unsigned long)t->can_base_id);
                        if (t->print_fn != NULL) {
                            t->print_fn(frame.data);
                        } else {
                            printf("dlc=%u\r\n", frame.dlc);
                        }
                    }
                    continue;
                }
                continue;
            }

            if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;
            if (!g_bridge.zenoh_ready) continue;

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

                /* When all frames arrive, serialize and publish to ROS 2 (TX)! */
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

                            static uint32_t last_tx_print = 0;
                            if (now - last_tx_print >= 1000) {
                                last_tx_print = now;
                                printf("[ROS-TX] /%s: ", t->topic_name);
                                if (t->print_fn != NULL) {
                                    t->print_fn(rt->buffer);
                                } else {
                                    printf("(%zu bytes)\r\n", t->msg_size);
                                }
                            }
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

    /* Open Zenoh session */
    z_owned_config_t config;
    configure_zenoh_session(&config);

    z_result_t res;
    while ((res = z_open(&g_bridge.session, z_move(config), NULL)) < 0) {
        printf("[Zenoh] Connect failed (%d). Retry in 2s...\r\n", (int)res);
        osDelay(2000);
        configure_zenoh_session(&config);
    }
    printf("[Zenoh] Connected to router!\r\n");

    /* Init ROS 2 Node */
    if (!zenoh_ros2_node_init(&g_bridge.node, &g_bridge.session,
                              CONFIG_ROS2_NODE_NAME, CONFIG_ROS2_NODE_NS, CONFIG_ROS2_DOMAIN_ID)) {
        printf("[ROS2] Node init failed!\r\n");
        return;
    }

    /* Auto-register topics from declarative master table */
    printf("[Bridge] Free heap before topic init: %u bytes\r\n", (unsigned)xPortGetFreeHeapSize());
    for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
        const bridge_topic_t *t = &g_bridge_topics[i];
        topic_runtime_t *rt = &g_runtimes[i];

        printf("[Bridge] Registering [%u] %s (heap: %u)\r\n",
               (unsigned)i, t->topic_name, (unsigned)xPortGetFreeHeapSize());

        if (t->dir == BRIDGE_DIR_CAN_RECV) {
            printf("  [RECV] %-16s : CAN 0x%03lX (Standalone Receiver)\r\n",
                   t->topic_name, (unsigned long)t->can_base_id);
            continue;
        }

        rt->num_frames     = (uint8_t)((t->msg_size + 7) / 8);
        rt->expected_mask  = (uint16_t)((1U << rt->num_frames) - 1U);
        rt->received_mask  = 0;
        rt->last_recv_tick = 0;
        rt->rx_msg_count   = 0;

        if (t->dir == BRIDGE_DIR_CAN_TO_ROS) {
            bool ok = zenoh_ros2_pub_create(&rt->pub, &g_bridge.node, t->topic_name, t->dds_type, t->type_hash);
            printf("  [PUB] /%-16s -> CAN 0x%03lX..0x%03lX (%u frames) [%s]\r\n",
                   t->topic_name, (unsigned long)t->can_base_id,
                   (unsigned long)(t->can_base_id + rt->num_frames - 1), rt->num_frames,
                   ok ? "OK" : "FAIL");
        } else if (t->dir == BRIDGE_DIR_CAN_TO_ROS_RAW) {
            bool ok = zenoh_ros2_pub_create(&rt->pub, &g_bridge.node, t->topic_name, t->dds_type, t->type_hash);
            printf("  [PUB] /%-16s -> Raw CAN Frame Bridge [%s]\r\n", t->topic_name, ok ? "OK" : "FAIL");
        } else if (t->dir == BRIDGE_DIR_ROS_TO_CAN_RAW) {
            bool ok = zenoh_ros2_sub_create(&rt->sub, &g_bridge.node, t->topic_name, t->dds_type, t->type_hash,
                                  on_zenoh_sub_message, (void *)i);
            printf("  [SUB] /%-16s <- Raw CAN Frame Bridge [%s]\r\n", t->topic_name, ok ? "OK" : "FAIL");
        } else {
            bool ok = zenoh_ros2_sub_create(&rt->sub, &g_bridge.node, t->topic_name, t->dds_type, t->type_hash,
                                  on_zenoh_sub_message, (void *)i);
            printf("  [SUB] /%-16s <- CAN 0x%03lX..0x%03lX (%u frames) [%s]\r\n",
                   t->topic_name, (unsigned long)t->can_base_id,
                   (unsigned long)(t->can_base_id + rt->num_frames - 1), rt->num_frames,
                   ok ? "OK" : "FAIL");
        }
    }
    printf("[Bridge] Free heap after topic init: %u bytes\r\n", (unsigned)xPortGetFreeHeapSize());

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
        uint8_t lec = (uint8_t)((esr >> 4) & 0x07);
        static const char *const lec_names[] = {
            "None", "Stuff", "Form", "Ack", "BitRec", "BitDom", "CRC", "SW"
        };
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

        printf("[Stats] CAN->ROS: %lu | ROS->CAN: %lu | RX_f:%lu TX_f:%lu | TEC:%u REC:%u LEC:%s(%u)%s%s | DROP:%lu\r\n",
               (unsigned long)g_bridge.can_to_zenoh_count, (unsigned long)g_bridge.zenoh_to_can_count,
               (unsigned long)g_bridge.raw_rx_frames, (unsigned long)g_bridge.raw_tx_frames,
               tec, rec, lec_names[lec], lec, boff ? " [BOFF]" : "", pass ? " [PASS]" : "",
               (unsigned long)g_bridge.drop_count);
    }
}

/* ───────────────────── Public Starter ──────────────────────────────── */

void app_zenoh_start(void) {
    memset(&g_bridge, 0, sizeof(g_bridge));
    memset(g_runtimes, 0, sizeof(g_runtimes));

    /* Initialize CAN hardware & queues BEFORE creating tasks to prevent NULL queue asserts */
    can_hardware_init();

    /* 1. Zenoh Manager + Diagnostics Loop (Single task) */
    osThreadDef(zenohTask, zenoh_engine_task, osPriorityNormal, 0, CONFIG_STACK_ZENOH_TASK);
    osThreadCreate(osThread(zenohTask), NULL);

    /* 2. CAN->Zenoh High-Priority Reassembly Worker */
    osThreadDef(bridgeTask, bridge_worker_task, osPriorityAboveNormal, 0, CONFIG_STACK_BRIDGE_TASK);
    osThreadCreate(osThread(bridgeTask), NULL);
}
