/**
 * @file app_zenoh.c
 * @brief High-Performance, Data-Driven Zenoh <-> CAN Bridge Engine
 *
 * Professional Architecture:
 * - Table-driven design: Zero hardcoded topics. Topics are declared in bridge_topics.h.
 * - Centralized config: Network and ROS2 parameters loaded from app_config.h.
 * - Hardware acceptance filtering: Auto-generates exact match ID list from table.
 * - Deterministic DTCM-RAM buffers: Zero cache-miss jitter on Cortex-M7.
 * - Comprehensive diagnostics: CAN bus health (TEC, REC, LEC) + per-topic heartbeat watchdog.
 * - Lock-free FreeRTOS queues for all data paths.
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
#include "app_config.h"
#include "bridge_topics.h"

extern struct netif gnetif;

#define ZENOH_TASK_STACK_SIZE   CONFIG_STACK_ZENOH_TASK
#define BRIDGE_TASK_STACK_SIZE  CONFIG_STACK_BRIDGE_TASK
#define CAN_TX_TASK_STACK_SIZE  CONFIG_STACK_CAN_TX_TASK
#define STATS_TASK_STACK_SIZE   CONFIG_STACK_STATS_TASK

/* ──────────────────────── Topic Runtime State ──────────────────────── */

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

/* Place reassembly runtime state in DTCM-RAM for maximum memory throughput */
static topic_runtime_t g_runtimes[BRIDGE_TOPIC_COUNT] __attribute__((section(".dtcmram")));

typedef struct {
    z_owned_session_t   session;
    zenoh_ros2_node_t   node;
    QueueHandle_t       can_tx_queue;
    volatile bool       zenoh_ready;
    volatile uint32_t   can_to_zenoh_count;
    volatile uint32_t   zenoh_to_can_count;
} bridge_engine_t;

static bridge_engine_t g_engine;

/* Static storage for CAN TX queue in DTCM-RAM */
static uint8_t ucTxQueueStorage[CAN_BRIDGE_TX_QUEUE_SIZE * sizeof(can_frame_t)] __attribute__((section(".dtcmram")));
static StaticQueue_t xTxQueueBuffer __attribute__((section(".dtcmram")));

/* ──────────────────────── Network Helpers ──────────────────────────── */

static const char *const g_zenoh_locators[] = { CONFIG_ZENOH_LOCATOR_LIST };
#define ZENOH_LOCATOR_COUNT (sizeof(g_zenoh_locators) / sizeof(g_zenoh_locators[0]))

static void init_zenoh_config(z_owned_config_t *config) {
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
    /* Instant Static IP mode: bind IP immediately and skip DHCP entirely (< 1s boot) */
    printf("[ETH] Instant Static IP mode: %s\r\n", CONFIG_NET_STATIC_IP);
    ip4_addr_t static_ip, static_mask, static_gw;
    ip4addr_aton(CONFIG_NET_STATIC_IP, &static_ip);
    ip4addr_aton(CONFIG_NET_STATIC_NETMASK, &static_mask);
    ip4addr_aton(CONFIG_NET_STATIC_GATEWAY, &static_gw);
    netif_set_addr(&gnetif, &static_ip, &static_mask, &static_gw);
    netif_set_up(&gnetif);

    while (!netif_is_link_up(&gnetif)) {
        osDelay(100);
    }
#else
    /* DHCP mode with static IP fallback */
    printf("[ETH] Waiting for DHCP lease (timeout: %ds)...\r\n", CONFIG_NET_DHCP_TIMEOUT_SEC);
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

        if (wait_sec >= CONFIG_NET_DHCP_TIMEOUT_SEC && gnetif.ip_addr.addr == 0) {
            printf("[ETH] DHCP timeout! Falling back to static IP %s...\r\n", CONFIG_NET_STATIC_IP);
            dhcp_stop(&gnetif);
            ip4_addr_t static_ip, static_mask, static_gw;
            ip4addr_aton(CONFIG_NET_STATIC_IP, &static_ip);
            ip4addr_aton(CONFIG_NET_STATIC_NETMASK, &static_mask);
            ip4addr_aton(CONFIG_NET_STATIC_GATEWAY, &static_gw);
            netif_set_addr(&gnetif, &static_ip, &static_mask, &static_gw);
            netif_set_up(&gnetif);
            break;
        }
    }
#endif
    printf("[ETH] Link UP! IP: %s  Mask: %s  GW: %s\r\n",
           ip4addr_ntoa(&gnetif.ip_addr),
           ip4addr_ntoa(&gnetif.netmask),
           ip4addr_ntoa(&gnetif.gw));
}

/* ─────────── ROS 2 -> CAN: Universal Subscriber Callback ──────────── */

static void on_zenoh_sub_message(const uint8_t *payload, size_t len, void *ctx) {
    size_t idx = (size_t)ctx;
    if (idx >= BRIDGE_TOPIC_COUNT) return;

    const bridge_topic_t *t = &g_bridge_topics[idx];
    topic_runtime_t *rt = &g_runtimes[idx];
    if (t->deserialize_fn == NULL) return;

    /* 1. Deserialize CDR payload into stack struct */
    uint8_t msg_buffer[BRIDGE_MAX_MSG_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, (uint8_t *)payload, len);

    if (!t->deserialize_fn(&ub, msg_buffer)) {
        return; /* Malformed CDR frame dropped */
    }

    /* 2. Universal Fragmenter: slice struct into 8-byte CAN frames */
    for (uint8_t f = 0; f < rt->num_frames; f++) {
        can_frame_t frame;
        frame.id = t->can_base_id + f;
        size_t offset = f * 8;
        size_t chunk = (offset + 8 <= t->msg_size) ? 8 : (t->msg_size - offset);
        frame.dlc = (uint8_t)chunk;
        memcpy(frame.data, &msg_buffer[offset], chunk);

        xQueueSend(g_engine.can_tx_queue, &frame, 0);
    }

    g_engine.zenoh_to_can_count++;
}

/* ─────────── CAN -> ROS 2: Universal Reassembly Worker Task ────────── */

static void bridge_can_to_zenoh_task(void const *argument) {
    (void)argument;

    while (!g_engine.zenoh_ready) {
        osDelay(50);
    }
    printf("[Bridge] Universal CAN->Zenoh worker running (DTCM-RAM)\r\n");

    QueueHandle_t rx_q = can_bridge_get_rx_queue();
    can_frame_t frame;
    uint8_t cdr_buf[256];

    for (;;) {
        if (xQueueReceive(rx_q, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint32_t now = HAL_GetTick();

        /* Search matching topic in declarative table */
        for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
            const bridge_topic_t *t = &g_bridge_topics[i];
            if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;
            topic_runtime_t *rt = &g_runtimes[i];

            if (frame.id >= t->can_base_id && frame.id < (t->can_base_id + rt->num_frames)) {
                uint8_t frame_idx = (uint8_t)(frame.id - t->can_base_id);

                /* Desynchronization guard: timeout resets stale partial frames */
                if (rt->received_mask != 0 && (now - rt->last_recv_tick) > CONFIG_CAN_FRAME_TIMEOUT_MS) {
                    rt->received_mask = 0;
                }
                rt->last_recv_tick = now;

                /* Frame 0 arrival always resets mask -> self-heals immediately after drop */
                if (frame_idx == 0) {
                    rt->received_mask = 0;
                }

                /* Copy data slice */
                size_t offset = frame_idx * 8;
                size_t chunk = (offset + 8 <= t->msg_size) ? 8 : (t->msg_size - offset);
                if (frame.dlc < chunk) chunk = frame.dlc;
                memcpy(&rt->buffer[offset], frame.data, chunk);

                rt->received_mask |= (1U << frame_idx);

                /* When all frames arrive, serialize to CDR and publish! */
                if (rt->received_mask == rt->expected_mask) {
                    rt->received_mask = 0;
                    rt->rx_msg_count++;

                    if (t->serialize_fn != NULL) {
                        ucdrBuffer ub;
                        ucdr_init_buffer(&ub, cdr_buf, sizeof(cdr_buf));
                        if (t->serialize_fn(&ub, rt->buffer)) {
                            size_t cdr_len = ucdr_buffer_length(&ub);
                            zenoh_ros2_pub_send(&rt->pub, cdr_buf, cdr_len);
                            g_engine.can_to_zenoh_count++;
                            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                        }
                    }
                }
                break;
            }
        }
    }
}

/* ──────────────────────── CAN TX Worker Task ───────────────────────── */

static void bridge_can_tx_task(void const *argument) {
    (void)argument;

    while (!g_engine.zenoh_ready) {
        osDelay(50);
    }
    printf("[Bridge] CAN TX worker running\r\n");

    can_frame_t frame;
    for (;;) {
        if (xQueueReceive(g_engine.can_tx_queue, &frame, portMAX_DELAY) == pdTRUE) {
            can_bridge_send_frame(&frame, pdMS_TO_TICKS(CONFIG_CAN_TX_TIMEOUT_MS));
        }
    }
}

/* ──────────────────────── Diagnostic & Watchdog Task ───────────────── */

static const char *lec_to_str(uint8_t lec) {
    switch (lec) {
        case 0: return "None";
        case 1: return "Stuff";
        case 2: return "Form";
        case 3: return "Ack";
        case 4: return "BitRecessive";
        case 5: return "BitDominant";
        case 6: return "CRC";
        default: return "Set";
    }
}

static void stats_task(void const *argument) {
    (void)argument;

    for (;;) {
        osDelay(CONFIG_CAN_STATS_PERIOD_MS);
        if (g_engine.zenoh_ready) {
            uint32_t rx_frames, tx_frames, drop_cnt;
            can_bridge_get_stats(&rx_frames, &tx_frames, &drop_cnt);

            can_bus_health_t health;
            can_bridge_get_bus_health(&health);

            uint32_t now = HAL_GetTick();
            bool has_timeout_error = false;

            /* Check per-topic watchdog (Heartbeat check) */
            for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
                const bridge_topic_t *t = &g_bridge_topics[i];
                if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;
                topic_runtime_t *rt = &g_runtimes[i];

                /* If active topic has received nothing for watchdog timeout */
                if (rt->rx_msg_count > 0 && (now - rt->last_recv_tick) > CONFIG_CAN_WATCHDOG_MS) {
                    printf("[WATCHDOG WARN] Topic /%s timeout! (last seen %lu ms ago)\r\n",
                           t->topic_name, (unsigned long)(now - rt->last_recv_tick));
                    has_timeout_error = true;
                }
            }

            /* Red LED (LD3) alerts if bus error or topic timeout detected */
            if (health.is_bus_off || health.is_passive || has_timeout_error) {
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
            }

            printf("[CAN Health] TEC:%u REC:%u LEC:%s | BOFF:%d PASS:%d WARN:%d | DROP:%lu\r\n",
                   health.tec, health.rec, lec_to_str(health.lec),
                   health.is_bus_off, health.is_passive, health.is_warning,
                   (unsigned long)drop_cnt);
            printf("[Bridge Stats] CAN->ROS: %lu msgs | ROS->CAN: %lu msgs | Raw RX:%lu TX:%lu\r\n",
                   (unsigned long)g_engine.can_to_zenoh_count,
                   (unsigned long)g_engine.zenoh_to_can_count,
                   (unsigned long)rx_frames,
                   (unsigned long)tx_frames);
        }
    }
}

/* ──────────────────────── Main Zenoh Engine Task ───────────────────── */

static void zenoh_task(void const *argument) {
    (void)argument;

    printf("\r\n==================================================\r\n");
    printf("  STM32F767ZI Ultimate Zenoh-CAN Bridge Engine    \r\n");
    printf("==================================================\r\n");

    /* 1. Wait for Ethernet link & IP */
    wait_for_network();

    /* 2. Build Hardware Acceptance Filter List from master table */
    uint32_t filter_ids[56];
    size_t filter_id_count = 0;

    for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
        const bridge_topic_t *t = &g_bridge_topics[i];
        if (t->dir != BRIDGE_DIR_CAN_TO_ROS) continue;

        uint8_t n_frames = (uint8_t)((t->msg_size + 7) / 8);
        for (uint8_t f = 0; f < n_frames && filter_id_count < 56; f++) {
            filter_ids[filter_id_count++] = t->can_base_id + f;
        }
    }

    /* Initialize CAN hardware with exact hardware filter list */
    can_bridge_init(filter_ids, filter_id_count);
    printf("[CAN] HW Filter active: %u IDs whitelisted in 16-bit list mode\r\n", (unsigned int)filter_id_count);

    /* 3. Open Zenoh Session */
    z_owned_config_t config;
    init_zenoh_config(&config);
    printf("[Zenoh] Connecting in %s mode...\r\n", CONFIG_ZENOH_MODE);

    z_result_t res;
    while ((res = z_open(&g_engine.session, z_move(config), NULL)) < 0) {
        printf("[Zenoh] Session open failed (%d). Retry in 2s...\r\n", (int)res);
        osDelay(2000);
        init_zenoh_config(&config);
    }
    printf("[Zenoh] Session opened successfully!\r\n");

    /* 4. Initialize ROS 2 Node */
    if (!zenoh_ros2_node_init(&g_engine.node, &g_engine.session,
                              CONFIG_ROS2_NODE_NAME, CONFIG_ROS2_NODE_NS, CONFIG_ROS2_DOMAIN_ID)) {
        printf("[ROS2] Node initialization failed!\r\n");
        z_drop(z_move(g_engine.session));
        vTaskDelete(NULL);
        return;
    }

    /* 5. Auto-register all topics from declarative table! */
    printf("[Bridge] Registering topics from Master Routing Table:\r\n");
    for (size_t i = 0; i < BRIDGE_TOPIC_COUNT; i++) {
        const bridge_topic_t *t = &g_bridge_topics[i];
        topic_runtime_t *rt = &g_runtimes[i];

        rt->num_frames     = (uint8_t)((t->msg_size + 7) / 8);
        rt->expected_mask  = (uint16_t)((1U << rt->num_frames) - 1U);
        rt->received_mask  = 0;
        rt->last_recv_tick = 0;
        rt->rx_msg_count   = 0;

        if (t->dir == BRIDGE_DIR_CAN_TO_ROS) {
            if (zenoh_ros2_pub_create(&rt->pub, &g_engine.node, t->topic_name, t->dds_type, t->type_hash)) {
                printf("  [PUB] /%-16s -> CAN 0x%03lX..0x%03lX (%u frames, %u bytes)\r\n",
                       t->topic_name,
                       (unsigned long)t->can_base_id,
                       (unsigned long)(t->can_base_id + rt->num_frames - 1),
                       rt->num_frames, (unsigned int)t->msg_size);
            }
        } else {
            if (zenoh_ros2_sub_create(&rt->sub, &g_engine.node, t->topic_name, t->dds_type, t->type_hash,
                                      on_zenoh_sub_message, (void *)i)) {
                printf("  [SUB] /%-16s <- CAN 0x%03lX..0x%03lX (%u frames, %u bytes)\r\n",
                       t->topic_name,
                       (unsigned long)t->can_base_id,
                       (unsigned long)(t->can_base_id + rt->num_frames - 1),
                       rt->num_frames, (unsigned int)t->msg_size);
            }
        }
    }

    /* 6. Start Zenoh-Pico internal read & lease tasks */
    zp_start_read_task(z_loan_mut(g_engine.session), NULL);
    zp_start_lease_task(z_loan_mut(g_engine.session), NULL);

    /* 7. Signal all workers ready! */
    g_engine.zenoh_ready = true;
    printf("[Bridge] All systems GO! Deterministic DTCM-backed bridge active.\r\n");

    for (;;) {
        osDelay(10000);
    }
}

/* ──────────────────────── Public Starter ───────────────────────────── */

void app_zenoh_start(void) {
    memset(&g_engine, 0, sizeof(g_engine));
    memset(g_runtimes, 0, sizeof(g_runtimes));

    /* Static TX queue placed in DTCM-RAM */
    g_engine.can_tx_queue = xQueueCreateStatic(CAN_BRIDGE_TX_QUEUE_SIZE,
                                               sizeof(can_frame_t),
                                               ucTxQueueStorage,
                                               &xTxQueueBuffer);
    configASSERT(g_engine.can_tx_queue != NULL);

    /* Zenoh Management Task */
    osThreadDef(zenohTask, zenoh_task, osPriorityNormal, 0, ZENOH_TASK_STACK_SIZE);
    osThreadCreate(osThread(zenohTask), NULL);

    /* High-priority CAN->Zenoh Reassembly Worker */
    osThreadDef(bridgeTask, bridge_can_to_zenoh_task, osPriorityAboveNormal, 0, BRIDGE_TASK_STACK_SIZE);
    osThreadCreate(osThread(bridgeTask), NULL);

    /* High-priority CAN TX Worker */
    osThreadDef(canTxTask, bridge_can_tx_task, osPriorityAboveNormal, 0, CAN_TX_TASK_STACK_SIZE);
    osThreadCreate(osThread(canTxTask), NULL);

    /* Diagnostic & Heartbeat Watchdog Task */
    osThreadDef(statsTask, stats_task, osPriorityLow, 0, STATS_TASK_STACK_SIZE);
    osThreadCreate(osThread(statsTask), NULL);
}
