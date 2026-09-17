#include "app_zenoh.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include <zenoh-pico.h>
#include <ucdr/microcdr.h>

extern struct netif gnetif;

/* 
 * Zenoh & ROS 2 設定:
 * - ROS 2 トピック名: /chatter
 *   Zenoh 上では ROS トピックはプレフィックス "rt/" が付くため "rt/chatter" になります。
 * - メッセージ型: std_msgs/msg/String (CDR シリアライズ)
 * - ZENOH_MODE:
 *   - "peer"   : 指定した複数のピア (192.168.50.30, 192.168.50.50) と同時に直接ユニキャスト通信します。
 *   - "client" : 指定した複数のルーターのうち利用可能な1つと接続します (フェイルオーバー)。
 * - ZENOH_LOCATORS:
 *   - 接続先を配列で複数指定可能 ("tcp/<IP>:<PORT>", デフォルトポート: 7447)
 *   - 配列が空または空文字列 "" の場合はマルチキャスト自動探索 (UDP scout)
 */
#define ZENOH_MODE "client"

/*
 * ROS 2 Lyrical (rmw_zenoh) ネイティブ通信設定 (bridgeなし):
 * - KeyExpr 形式: <domain_id>/<topic_name>/<type_name>/<type_hash>
 * - メッセージ型: std_msgs/msg/String (CDR シリアライズ)
 */
#define ROS_DOMAIN_ID "0"
#define ROS_TOPIC_NAME "chatter"
#define ROS_MSG_TYPE "std_msgs::msg::dds_::String_"
#define ROS_TYPE_HASH "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"

#define ZENOH_KEYEXPR ROS_DOMAIN_ID "/" ROS_TOPIC_NAME "/" ROS_MSG_TYPE "/" ROS_TYPE_HASH
#define ZENOH_VALUE_PREFIX "Hello from STM32F767ZI Zenoh-Pico!"

static const char *const ZENOH_LOCATORS[] = {
    "tcp/192.168.50.30:7447",
    "tcp/192.168.50.50:7447",
};
#define ZENOH_LOCATOR_COUNT (sizeof(ZENOH_LOCATORS) / sizeof(ZENOH_LOCATORS[0]))

/**
 * @brief Zenoh 設定の初期化 (複数ロケータ対応)
 */
static void init_zenoh_config(z_owned_config_t *config) {
    z_config_default(config);
    zp_config_insert(z_loan_mut(*config), Z_CONFIG_MODE_KEY, ZENOH_MODE);

    for (size_t i = 0; i < ZENOH_LOCATOR_COUNT; i++) {
        if (ZENOH_LOCATORS[i] != NULL && strlen(ZENOH_LOCATORS[i]) > 0) {
            zp_config_insert(z_loan_mut(*config), Z_CONFIG_CONNECT_KEY, ZENOH_LOCATORS[i]);
        }
    }
}

/**
 * @brief Micro-CDR を使用した ROS 2 std_msgs/msg/String 用 CDR シリアライズ関数
 * 
 * ROS 2 の CDR フォーマット:
 * - 4 バイト: CDR encapsulation header (0x00, 0x01, 0x00, 0x00 = CDR Little Endian)
 * - 4 バイト: 文字列長 (uint32_t, null終端文字を含む)
 * - N バイト: 文字列データ + '\0' (Micro-CDR がアライメントと境界チェックを自動処理)
 */
static size_t serialize_ros2_string(uint8_t *dst, size_t dst_max, const char *str) {
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, dst, dst_max);

    // 1. CDR Header (Little Endian: 0x00, 0x01, 0x00, 0x00)
    ucdr_serialize_uint8_t(&writer, 0x00);
    ucdr_serialize_uint8_t(&writer, 0x01);
    ucdr_serialize_uint16_t(&writer, 0x0000);

    // 2. 文字列 (長さ + null終端を自動シリアライズ)
    ucdr_serialize_string(&writer, str);

    if (ucdr_buffer_has_error(&writer)) {
        return 0;
    }

    return ucdr_buffer_length(&writer);
}

static void zenoh_task(void const *argument) {
    (void)argument;

    printf("\r\n========================================\r\n");
    printf("   STM32F767ZI Zenoh-Pico -> ROS 2     \r\n");
    printf("========================================\r\n");

    /* 1. DHCP による IP 取得を待機 */
    printf("[ETH] Waiting for IP address...\r\n");
    while (netif_is_up(&gnetif) == 0 || gnetif.ip_addr.addr == 0) {
        osDelay(500);
    }
    printf("[ETH] IP Address : %s\r\n", ip4addr_ntoa(&gnetif.ip_addr));
    printf("[ETH] Netmask    : %s\r\n", ip4addr_ntoa(&gnetif.netmask));
    printf("[ETH] Gateway    : %s\r\n", ip4addr_ntoa(&gnetif.gw));

    /* 2. Zenoh セッション設定 */
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
    printf("[Zenoh] Mode: %s\r\n", ZENOH_MODE);

    /* 3. Zenoh セッションオープン */
    z_owned_session_t s;
    z_result_t res;
    while ((res = z_open(&s, z_move(config), NULL)) < 0) {
        printf("[Zenoh] Unable to open session (err: %d). Retrying in 2s...\r\n", (int)res);
        osDelay(2000);
        init_zenoh_config(&config);
    }
    printf("[Zenoh] Session opened successfully!\r\n");

    /* 4. ROS 2 Publisher の宣言 (トピック: /chatter -> Key: rt/chatter) */
    printf("[Zenoh] Declaring ROS 2 publisher for '%s' (/chatter)...\r\n", ZENOH_KEYEXPR);
    z_owned_publisher_t pub;
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, ZENOH_KEYEXPR);
    if (z_declare_publisher(z_loan(s), &pub, z_loan(ke), NULL) < 0) {
        printf("[Zenoh] Error: failed to declare publisher!\r\n");
        z_drop(z_move(s));
        return;
    }

    /* 5. 定期パブリッシュループ */
    char text_buf[128];
    uint8_t cdr_buf[256];
    uint32_t count = 0;
    while (1) {
        snprintf(text_buf, sizeof(text_buf), "%s (count: %lu)", ZENOH_VALUE_PREFIX, (unsigned long)count++);
        printf("[ROS2] Publishing on /chatter: %s\r\n", text_buf);

        /* ROS 2 CDR 形式にシリアライズ */
        size_t cdr_len = serialize_ros2_string(cdr_buf, sizeof(cdr_buf), text_buf);

        /* Zenoh パブリッシュ */
        z_owned_bytes_t payload;
        z_bytes_copy_from_buf(&payload, cdr_buf, cdr_len);

        z_publisher_put_options_t options;
        z_publisher_put_options_default(&options);
        z_publisher_put(z_loan(pub), z_move(payload), &options);

        /* 通信時に LED2 (緑) をトグル */
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        osDelay(1000);
    }

    /* クリーンアップ */
    z_drop(z_move(pub));
    z_drop(z_move(s));
}

void app_zenoh_start(void) {
    /* Zenoh タスクを生成 (スタックサイズ 2048 words = 8KB) */
    osThreadDef(zenohTask, zenoh_task, osPriorityNormal, 0, 2048);
    osThreadCreate(osThread(zenohTask), NULL);
}
