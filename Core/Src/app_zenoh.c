#include "app_zenoh.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include <zenoh-pico.h>
#include "zenoh_ros2.h"
#include "can_bridge.h"
#include "generated/Frame.h"
#include "app_config.h"

extern struct netif gnetif;

/* Zenoh 接続先エンドポイント (app_config.h で設定) */
static const char *const ZENOH_LOCATORS[] = {
    CONFIG_ZENOH_LOCATOR_1,
    CONFIG_ZENOH_LOCATOR_2,
    CONFIG_ZENOH_LOCATOR_3,
    CONFIG_ZENOH_LOCATOR_4,
};
#define ZENOH_LOCATOR_COUNT (sizeof(ZENOH_LOCATORS) / sizeof(ZENOH_LOCATORS[0]))

/**
 * @brief Zenoh 設定の初期化 (複数ロケータ対応)
 */
static void init_zenoh_config(z_owned_config_t *config) {
    z_config_default(config);
    zp_config_insert(z_loan_mut(*config), Z_CONFIG_MODE_KEY, CONFIG_ZENOH_MODE);

    for (size_t i = 0; i < ZENOH_LOCATOR_COUNT; i++) {
        if (ZENOH_LOCATORS[i] != NULL && strlen(ZENOH_LOCATORS[i]) > 0) {
            zp_config_insert(z_loan_mut(*config), Z_CONFIG_CONNECT_KEY, ZENOH_LOCATORS[i]);
        }
    }
}

/**
 * @brief 静的 IP 設定 & 物理リンクアップ待機
 */
static void wait_for_network(void) {
    /* 1. 静的 IP を設定 */
    ip4_addr_t static_ip, static_mask, static_gw;
    IP4_ADDR(&static_ip, CONFIG_STATIC_IP_ADDR0, CONFIG_STATIC_IP_ADDR1, CONFIG_STATIC_IP_ADDR2, CONFIG_STATIC_IP_ADDR3);
    IP4_ADDR(&static_mask, CONFIG_STATIC_NETMASK0, CONFIG_STATIC_NETMASK1, CONFIG_STATIC_NETMASK2, CONFIG_STATIC_NETMASK3);
    IP4_ADDR(&static_gw, CONFIG_STATIC_GW_ADDR0, CONFIG_STATIC_GW_ADDR1, CONFIG_STATIC_GW_ADDR2, CONFIG_STATIC_GW_ADDR3);

    netif_set_addr(&gnetif, &static_ip, &static_mask, &static_gw);
    netif_set_up(&gnetif);

    printf("[ETH] Static IP Configured:\r\n");
    printf("[ETH]   IP Address : %s\r\n", ip4addr_ntoa(&gnetif.ip_addr));
    printf("[ETH]   Netmask    : %s\r\n", ip4addr_ntoa(&gnetif.netmask));
    printf("[ETH]   Gateway    : %s\r\n", ip4addr_ntoa(&gnetif.gw));

    /* 2. 物理 Ethernet リンクアップを待機（ケーブル接続確認） */
    printf("[ETH] Waiting for Ethernet physical link...\r\n");
    while (!netif_is_link_up(&gnetif)) {
        osDelay(100);
    }
    printf("[ETH] Ethernet link is UP! Ready for communication.\r\n");
}

static void can_frame_sub_callback(const uint8_t *payload, size_t len, void *ctx) {
    (void)ctx;
    if (payload == NULL || len == 0) return;

    can_msgs_Frame frame;
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, (uint8_t *)payload, len);

    if (can_msgs_Frame_deserialize(&reader, &frame)) {
        if (!can_bridge_post_frame(&frame)) {
            printf("[Bridge] Warning: CAN queue full, frame ID 0x%03lX dropped!\r\n", (unsigned long)frame.id);
        }
    } else {
        printf("[Bridge] Error: Failed to deserialize can_msgs/msg/Frame (len: %zu)!\r\n", len);
    }
}

static void zenoh_task(void const *argument) {
    (void)argument;

    printf("\r\n========================================\r\n");
    printf("   STM32F767ZI Zenoh-Pico -> ROS 2     \r\n");
    printf("========================================\r\n");

    /* 1. DHCP による IP 取得を待機 */
    wait_for_network();

    /* 2. Zenoh セッション設定 & オープン */
    z_owned_config_t config;
    init_zenoh_config(&config);

    size_t active_locators = 0;
    for (size_t i = 0; i < ZENOH_LOCATOR_COUNT; i++) {
        if (ZENOH_LOCATORS[i] != NULL && strlen(ZENOH_LOCATORS[i]) > 0) {
            printf("[Zenoh] Connect locator [%u]: %s\r\n", (unsigned int)i, ZENOH_LOCATORS[i]);
            active_locators++;
        }
    }
    if (active_locators == 0) {
        printf("[Zenoh] Using multicast scouting to find peers/routers...\r\n");
    }
    printf("[Zenoh] Mode: %s\r\n", CONFIG_ZENOH_MODE);

    z_owned_session_t s;
    z_result_t res;
    while ((res = z_open(&s, z_move(config), NULL)) < 0) {
        printf("[Zenoh] Unable to open session (err: %d). Retrying in 2s...\r\n", (int)res);
        osDelay(2000);
        init_zenoh_config(&config);
    }
    printf("[Zenoh] Session opened successfully!\r\n");

    /* 3. ROS 2 Node の初期化 (Liveliness Token の自動宣言) */
    zenoh_ros2_node_t node;
    if (!zenoh_ros2_node_init(&node, &s, CONFIG_ROS2_NODE_NAME, CONFIG_ROS2_NODE_NS, CONFIG_ROS2_DOMAIN_ID)) {
        printf("[ROS2] Failed to initialize node!\r\n");
        z_drop(z_move(s));
        return;
    }

    /* 4. ROS 2 Publisher の作成 (トピック: /chatter) */
    zenoh_ros2_pub_t chatter_pub;
    if (!zenoh_ros2_pub_create(&chatter_pub, &node, CONFIG_ROS2_TOPIC_CHATTER,
                              ROS2_TYPE_STD_MSGS_STRING, ROS2_HASH_STD_MSGS_STRING)) {
        printf("[ROS2] Failed to create publisher!\r\n");
        zenoh_ros2_node_fini(&node);
        z_drop(z_move(s));
        return;
    }

    /* 5. ROS 2 Subscriber の作成 (トピック: can_msgs/frame -> CAN バス垂れ流し) */
    zenoh_ros2_sub_t can_frame_sub;
    if (!zenoh_ros2_sub_create(&can_frame_sub, &node, CONFIG_ROS2_TOPIC_CAN_FRAME,
                              can_msgs_Frame_DDS_TYPE, can_msgs_Frame_TYPE_HASH,
                              can_frame_sub_callback, NULL)) {
        printf("[ROS2] Warning: Failed to create subscriber for %s!\r\n", CONFIG_ROS2_TOPIC_CAN_FRAME);
    } else {
        printf("[ROS2] Subscribed to /%s -> CAN Bridge active\r\n", CONFIG_ROS2_TOPIC_CAN_FRAME);
    }

    /* 6. 定期パブリッシュループ */
    char text_buf[128];
    uint8_t cdr_buf[256];
    uint32_t count = 0;
    while (1) {
        snprintf(text_buf, sizeof(text_buf), "Hello from STM32F767ZI Zenoh-Pico! (count: %lu)", (unsigned long)count++);
        printf("[ROS2] Publishing on /%s: %s\r\n", CONFIG_ROS2_TOPIC_CHATTER, text_buf);

        /* ROS 2 CDR 形式にシリアライズ */
        size_t cdr_len = zenoh_ros2_serialize_string(cdr_buf, sizeof(cdr_buf), text_buf);

        /* パブリッシュ送信 */
        if (cdr_len > 0) {
            if (zenoh_ros2_pub_send(&chatter_pub, cdr_buf, cdr_len)) {
                /* 通信成功時に LED2 (緑) をトグル */
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            } else {
                printf("[ROS2] Warning: Failed to publish message #%lu\r\n", (unsigned long)count - 1);
            }
        }

        osDelay(CONFIG_ROS2_CHATTER_PERIOD_MS);
    }

    /* クリーンアップ */
    zenoh_ros2_sub_destroy(&can_frame_sub);
    zenoh_ros2_pub_destroy(&chatter_pub);
    zenoh_ros2_node_fini(&node);
    z_drop(z_move(s));
}

void app_zenoh_start(void) {
    /* CAN Bridge タスクとキューを開始 */
    can_bridge_start();

    /* Zenoh タスクを生成 (スタックサイズ 2048 words = 8KB) */
    osThreadDef(zenohTask, zenoh_task, osPriorityNormal, 0, 2048);
    osThreadCreate(osThread(zenohTask), NULL);
}
