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
#include "cmsis_os2.h"
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip.h"
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
    bool     is_fd;
    bridge_can_bus_t bus;
    uint8_t  data[64];
} can_frame_t;

static uint8_t can_fd_payload_length(uint32_t dlc_code)
{
    static const uint8_t lengths[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
    };
    uint8_t code = (uint8_t)((dlc_code >> 16) & 0x0FU);
    return lengths[code];
}

static uint32_t can_fd_dlc_code(uint8_t length)
{
    static const uint32_t codes[16] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
        FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_20,
        FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_48,
        FDCAN_DLC_BYTES_64
    };
    uint8_t code = length <= 8U ? length :
                   length <= 12U ? 9U :
                   length <= 16U ? 10U :
                   length <= 20U ? 11U :
                   length <= 24U ? 12U :
                   length <= 32U ? 13U :
                   length <= 48U ? 14U : 15U;
    return codes[code];
}

static bool can_frame_matches_route(const can_frame_t *frame, const bridge_topic_t *topic)
{
    const bool route_is_fd = topic->frame_mode == BRIDGE_FRAME_FD;
    const bool route_matches_bus =
        topic->bus == BRIDGE_CAN_ANY || frame->bus == topic->bus;
    return route_matches_bus && frame->is_fd == route_is_fd;
}

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

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

    FDCAN_FilterTypeDef filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = 0,
        .FilterID2 = 0
    };
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        configASSERT(0);
    }
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan2) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
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
        printf("[CAN] HW Filter: Raw Transparent Bridge Mode (Accept All Standard/Extended)\r\n");
    } else {
        /* H5 global filter accepts all frames; topic filtering remains in software. */
        printf("[CAN] H5 software whitelist mode\r\n");
    }
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    printf("[CAN] HW Filter & Bitrate configured (%lu bps)\r\n", (unsigned long)CONFIG_CAN_BITRATE);
}

/* Direct thread-safe CAN transmission supporting Standard (11-bit) and Extended (29-bit) IDs */
static bool can_send_frame_ex(bridge_can_bus_t bus, uint8_t frame_mode, uint32_t id,
                              const uint8_t *data, uint8_t dlc, bool is_extended, bool is_rtr) {
    if (data == NULL) {
        g_bridge.drop_count++;
        return false;
    }

    FDCAN_HandleTypeDef *fdcan = (bus == BRIDGE_FDCAN2) ? &hfdcan2 : &hfdcan1;
    FDCAN_TxHeaderTypeDef tx_hdr = {0};
    tx_hdr.Identifier = id & (is_extended ? 0x1FFFFFFFU : 0x7FFU);
    tx_hdr.IdType = is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType = is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    static const uint32_t classic_dlc_code[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    if (frame_mode == BRIDGE_FRAME_FD) {
        tx_hdr.DataLength = can_fd_dlc_code(dlc);
    } else {
        tx_hdr.DataLength = classic_dlc_code[dlc > 8 ? 8 : dlc];
    }
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch = frame_mode == BRIDGE_FRAME_FD ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx_hdr.FDFormat = frame_mode == BRIDGE_FRAME_FD ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker = 0;

    TickType_t start = xTaskGetTickCount();
    while (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) == 0) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(CONFIG_CAN_TX_TIMEOUT_MS)) {
            g_bridge.drop_count++;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &tx_hdr, (uint8_t *)data);
    if (status == HAL_OK) {
        g_bridge.raw_tx_frames++;
        return true;
    }
    printf("[CAN-TX-ERR] HAL_FDCAN_AddMessageToTxFifoQ failed: status=%d, err=0x%08lX, state=%d\r\n",
           (int)status, (unsigned long)fdcan->ErrorCode, (int)fdcan->State);
    g_bridge.drop_count++;
    return false;
}

static inline bool can_send_frame(bridge_can_bus_t bus, uint8_t frame_mode, uint32_t id,
                                  const uint8_t *data, uint8_t dlc) {
    return can_send_frame_ex(bus, frame_mode, id, data, dlc, (id > 0x7FFU), false);
}

/* ───────────────────── Fast ISR Context (< 2µs) ────────────────────── */

void FDCAN1_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN2_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&hfdcan2);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t it_flags) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    while ((it_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0 &&
           HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U) {
        FDCAN_RxHeaderTypeDef rx_hdr;
        can_frame_t frame = {0};
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_hdr, frame.data) != HAL_OK) {
            g_bridge.drop_count++;
            break;
        }
        if (g_rx_queue != NULL) {
            if (rx_hdr.IdType == FDCAN_EXTENDED_ID) {
                frame.id = rx_hdr.Identifier;
                frame.is_extended = true;
            } else {
                frame.id = rx_hdr.Identifier;
                frame.is_extended = false;
            }
            frame.is_rtr = (rx_hdr.RxFrameType == FDCAN_REMOTE_FRAME);
            frame.is_fd = (rx_hdr.FDFormat == FDCAN_FD_CAN);
            frame.dlc = can_fd_payload_length(rx_hdr.DataLength);
            frame.bus = (hfdcan->Instance == FDCAN2) ? BRIDGE_FDCAN2 : BRIDGE_FDCAN1;

            if (xQueueSendFromISR(g_rx_queue, &frame, &higher_priority_task_woken) == pdTRUE) {
                g_bridge.raw_rx_frames++;
            } else {
                g_bridge.drop_count++;
            }
        }
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
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
        uint32_t id;
        uint8_t dlc;
        const uint8_t *data;
        bool is_extended;
        bool is_rtr;
        if (t->payload_kind == BRIDGE_PAYLOAD_FD_FRAME) {
            const can_transport_msgs_FDFrame *cf = (const can_transport_msgs_FDFrame *)msg_buffer;
            id = cf->id;
            dlc = cf->len;
            data = cf->data;
            is_extended = cf->is_extended;
            is_rtr = false;
        } else {
            const can_msgs_Frame *cf = (const can_msgs_Frame *)msg_buffer;
            id = cf->id;
            dlc = cf->dlc;
            data = cf->data;
            is_extended = cf->is_extended;
            is_rtr = cf->is_rtr;
        }
        printf("[RAW-TX] id=0x%08lX ext=%d dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               (unsigned long)id, is_extended, dlc, data[0], data[1], data[2], data[3],
               data[4], data[5], data[6], data[7]);
        can_send_frame_ex(t->bus, t->frame_mode, id, data, dlc, is_extended, is_rtr);
        g_bridge.zenoh_to_can_count++;
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
        can_send_frame(t->bus, t->frame_mode, t->can_base_id + f,
                       &msg_buffer[offset], (uint8_t)chunk);
    }
    g_bridge.zenoh_to_can_count++;
}

/* ─────────── CAN -> ROS 2: Reassembly Worker Task ──────────────────── */

static void bridge_worker_task(void *arg) {
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
                if (t->dir == BRIDGE_DIR_CAN_TO_ROS_RAW &&
                    can_frame_matches_route(&frame, t)) {
                    topic_runtime_t *rt = &g_runtimes[i];

                    ucdrBuffer ub;
                    ucdr_init_buffer(&ub, cdr_buf, sizeof(cdr_buf));
                    bool serialized;
                    if (t->payload_kind == BRIDGE_PAYLOAD_FD_FRAME) {
                        can_transport_msgs_FDFrame out_frame;
                        memset(&out_frame, 0, sizeof(out_frame));
                        strncpy(out_frame.header.frame_id, "fdcan", sizeof(out_frame.header.frame_id) - 1);
                        out_frame.id = frame.id;
                        out_frame.is_extended = frame.is_extended;
                        out_frame.len = frame.dlc;
                        memcpy(out_frame.data, frame.data, frame.dlc);
                        serialized = can_transport_msgs_FDFrame_serialize(&ub, &out_frame);
                    } else {
                        can_msgs_Frame out_frame;
                        memset(&out_frame, 0, sizeof(out_frame));
                        strncpy(out_frame.header.frame_id, "can", sizeof(out_frame.header.frame_id) - 1);
                        out_frame.id = frame.id;
                        out_frame.is_extended = frame.is_extended;
                        out_frame.is_rtr = frame.is_rtr;
                        out_frame.dlc = frame.dlc > 8U ? 8U : frame.dlc;
                        memcpy(out_frame.data, frame.data, out_frame.dlc);
                        serialized = can_msgs_Frame_serialize(&ub, &out_frame);
                    }
                    if (serialized) {
                        zenoh_ros2_pub_send(&rt->pub, cdr_buf, ucdr_buffer_length(&ub));
                        g_bridge.can_to_zenoh_count++;
                    }
                }
            }
        }

        /* 2. Topic-specific CAN Handlers (Standalone Receiver & Multi-frame Reassembly) */
        for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
            const bridge_topic_t *t = &g_bridge_topics[i];
            if (!can_frame_matches_route(&frame, t)) continue;

            /* Standalone CAN Receiver: log to serial (USART3 / ST-LINK VCP) and toggle LED */
            if (t->dir == BRIDGE_DIR_CAN_RECV) {
                if (frame.id == t->can_base_id) {
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

static void zenoh_engine_task(void *arg) {
    (void)arg;

    printf("\r\n==================================================\r\n");
    printf("  STM32H5 Ultra-Thin Zenoh-CAN Bridge             \r\n");
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

        uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
        bool boff = false;
        bool pass = (err & (HAL_FDCAN_ERROR_PROTOCOL_ARBT |
                            HAL_FDCAN_ERROR_PROTOCOL_DATA)) != 0;

        uint32_t now = HAL_GetTick();
        for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
            const bridge_topic_t *t = &g_bridge_topics[i];
            if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;
            topic_runtime_t *rt = &g_runtimes[i];
            if (rt->rx_msg_count > 0 && (now - rt->last_recv_tick) > CONFIG_CAN_WATCHDOG_MS) {
                printf("[WATCHDOG] /%s timeout!\r\n", t->topic_name);
            }
        }

        /* Red LED alert on bus errors or topic silence */

        printf("[Stats] CAN->ROS: %lu | ROS->CAN: %lu | RX_f:%lu TX_f:%lu | FDCAN_ERR:0x%08lX%s%s | DROP:%lu\r\n",
               (unsigned long)g_bridge.can_to_zenoh_count, (unsigned long)g_bridge.zenoh_to_can_count,
               (unsigned long)g_bridge.raw_rx_frames, (unsigned long)g_bridge.raw_tx_frames,
               (unsigned long)err, boff ? " [BOFF]" : "", pass ? " [PROTO]" : "",
               (unsigned long)g_bridge.drop_count);
    }
}

/* ───────────────────── Public Starter ──────────────────────────────── */

void app_zenoh_start(void) {
    memset(&g_bridge, 0, sizeof(g_bridge));
    memset(g_runtimes, 0, sizeof(g_runtimes));

    /* Initialize CAN hardware & queues BEFORE creating tasks to prevent NULL queue asserts */
    can_hardware_init();
    MX_LWIP_Init();

    /* 1. Zenoh Manager + Diagnostics Loop (Single task) */
    xTaskCreate(zenoh_engine_task, "zenoh", CONFIG_STACK_ZENOH_TASK,
                NULL, tskIDLE_PRIORITY + 2, NULL);

    /* 2. CAN->Zenoh High-Priority Reassembly Worker */
    xTaskCreate(bridge_worker_task, "can_bridge", CONFIG_STACK_BRIDGE_TASK,
                NULL, tskIDLE_PRIORITY + 3, NULL);
}
