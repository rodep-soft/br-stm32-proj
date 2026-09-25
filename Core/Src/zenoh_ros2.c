/**
 * @file zenoh_ros2.c
 * @brief Professional ROS 2 Native Abstraction Layer implementation for STM32 / Zenoh-Pico
 */

#include "zenoh_ros2.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include <ucdr/microcdr.h>

#define ZENOH_ROS2_SESSION_ID "stm32"

static uint32_t g_node_counter = 1;

static const char *strip_leading_slash(const char *s) {
    if (s == NULL) return "";
    while (*s == '/') s++;
    return s;
}

static void sanitize_for_token(char *dst, size_t dst_len, const char *src) {
    if (dst_len == 0) return;
    if (src == NULL || src[0] == '\0' || (src[0] == '/' && src[1] == '\0')) {
        snprintf(dst, dst_len, "%%");
        return;
    }

    size_t i = 0;
    if (src[0] != '/') {
        if (i < dst_len - 1) dst[i++] = '%';
    }
    for (size_t j = 0; src[j] != '\0' && i < dst_len - 1; j++) {
        if (src[j] == '/') {
            dst[i++] = '%';
        } else {
            dst[i++] = src[j];
        }
    }
    dst[i] = '\0';
}

bool zenoh_ros2_node_init(zenoh_ros2_node_t *node, z_owned_session_t *session,
                          const char *name, const char *ns, uint32_t domain_id) {
    if (!node || !session || !name) {
        return false;
    }

    memset(node, 0, sizeof(*node));
    node->session = session;
    node->name = name;
    node->ns = ns ? ns : "/";
    node->domain_id = domain_id;
    node->node_id = g_node_counter++;
    node->next_entity_id = 1;

    char token_ns[64];
    sanitize_for_token(token_ns, sizeof(token_ns), node->ns);

    char token_ke_str[ZENOH_ROS2_MAX_KEYEXPR_LEN];
    snprintf(token_ke_str, sizeof(token_ke_str),
             "@ros2_lv/%lu/%s/%lu/0/NN/%%/%s/%s",
             (unsigned long)node->domain_id,
             ZENOH_ROS2_SESSION_ID,
             (unsigned long)node->node_id,
             token_ns,
             node->name);

    z_view_keyexpr_t token_ke;
    z_view_keyexpr_from_str_unchecked(&token_ke, token_ke_str);

    if (z_liveliness_declare_token(z_loan(*node->session), &node->token, z_loan(token_ke), NULL) < 0) {
        printf("[ROS2] Error: failed to declare node liveliness token (%s)\r\n", token_ke_str);
        return false;
    }

    node->is_declared = true;
    printf("[ROS2] Node '%s' initialized (domain: %lu, node_id: %lu)\r\n",
           node->name, (unsigned long)node->domain_id, (unsigned long)node->node_id);
    return true;
}

void zenoh_ros2_node_fini(zenoh_ros2_node_t *node) {
    if (!node) return;
    if (node->is_declared) {
        z_drop(z_move(node->token));
        node->is_declared = false;
        printf("[ROS2] Node '%s' finalized\r\n", node->name ? node->name : "");
    }
}

bool zenoh_ros2_pub_create(zenoh_ros2_pub_t *pub, zenoh_ros2_node_t *node,
                           const char *topic_name, const char *type_name, const char *type_hash) {
    if (!pub || !node || !node->session || !topic_name || !type_name || !type_hash) {
        return false;
    }

    memset(pub, 0, sizeof(*pub));
    pub->node = node;
    pub->topic_name = topic_name;
    pub->type_name = type_name;
    pub->type_hash = type_hash;
    pub->entity_id = node->next_entity_id++;
    pub->sequence_number = 1;

    /* Initialize GID (16 bytes) */
    memset(pub->gid, 0, sizeof(pub->gid));
    pub->gid[0] = (uint8_t)(node->domain_id & 0xFF);
    pub->gid[1] = (uint8_t)(node->node_id & 0xFF);
    pub->gid[2] = (uint8_t)(pub->entity_id & 0xFF);

    const char *raw_topic = strip_leading_slash(topic_name);

    char token_ns[64];
    sanitize_for_token(token_ns, sizeof(token_ns), node->ns);

    char token_topic[96];
    sanitize_for_token(token_topic, sizeof(token_topic), topic_name);

    /* 1. Declare Publisher Liveliness Token */
    char token_ke_str[ZENOH_ROS2_MAX_KEYEXPR_LEN];
    snprintf(token_ke_str, sizeof(token_ke_str),
             "@ros2_lv/%lu/%s/%lu/%lu/MP/%%/%s/%s/%s/%s/%s/%s",
             (unsigned long)node->domain_id,
             ZENOH_ROS2_SESSION_ID,
             (unsigned long)node->node_id,
             (unsigned long)pub->entity_id,
             token_ns,
             node->name,
             token_topic,
             type_name,
             type_hash,
             ZENOH_ROS2_DEFAULT_QOS);

    z_view_keyexpr_t token_ke;
    z_view_keyexpr_from_str_unchecked(&token_ke, token_ke_str);

    if (z_liveliness_declare_token(z_loan(*node->session), &pub->token, z_loan(token_ke), NULL) < 0) {
        printf("[ROS2] Error: failed to declare publisher liveliness token (%s)\r\n", token_ke_str);
        return false;
    }

    /* 2. Declare Data Publisher */
    char data_ke_str[ZENOH_ROS2_MAX_KEYEXPR_LEN];
    snprintf(data_ke_str, sizeof(data_ke_str),
             "%lu/%s/%s/%s",
             (unsigned long)node->domain_id,
             raw_topic,
             type_name,
             type_hash);

    z_view_keyexpr_t data_ke;
    z_view_keyexpr_from_str_unchecked(&data_ke, data_ke_str);

    if (z_declare_publisher(z_loan(*node->session), &pub->pub, z_loan(data_ke), NULL) < 0) {
        printf("[ROS2] Error: failed to declare publisher for key (%s)\r\n", data_ke_str);
        z_drop(z_move(pub->token));
        return false;
    }

    pub->is_declared = true;
    printf("[ROS2] Publisher declared for '/%s' (entity_id: %lu)\r\n",
           raw_topic, (unsigned long)pub->entity_id);
    return true;
}

bool zenoh_ros2_pub_send(zenoh_ros2_pub_t *pub, const uint8_t *cdr_payload, size_t len) {
    if (!pub || !pub->is_declared || !cdr_payload || len == 0) {
        return false;
    }

    /* rmw_zenoh_cpp attachment metadata format (33 bytes total):
     * - 8 bytes: int64_t sequence_number (little endian)
     * - 8 bytes: int64_t source_timestamp in nanoseconds (little endian)
     * - 1 byte : sequence length of publisher GID (16 = 0x10)
     * - 16 bytes: publisher GID array
     */
    uint8_t att_buf[33];
    int64_t seq = pub->sequence_number++;
    int64_t ts = (int64_t)HAL_GetTick() * 1000000LL;

    memcpy(&att_buf[0], &seq, 8);
    memcpy(&att_buf[8], &ts, 8);
    att_buf[16] = 16;
    memcpy(&att_buf[17], pub->gid, 16);

    z_owned_bytes_t attachment;
    z_bytes_copy_from_buf(&attachment, att_buf, sizeof(att_buf));

    z_owned_bytes_t payload;
    z_bytes_copy_from_buf(&payload, cdr_payload, len);

    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);
    options.attachment = z_move(attachment);

    z_result_t res = z_publisher_put(z_loan(pub->pub), z_move(payload), &options);
    return (res >= 0);
}

void zenoh_ros2_pub_destroy(zenoh_ros2_pub_t *pub) {
    if (!pub) return;
    if (pub->is_declared) {
        z_drop(z_move(pub->token));
        z_drop(z_move(pub->pub));
        pub->is_declared = false;
        printf("[ROS2] Publisher '/%s' destroyed\r\n",
               pub->topic_name ? pub->topic_name : "");
    }
}

static void subscriber_trampoline(z_loaned_sample_t *sample, void *arg) {
    zenoh_ros2_sub_t *sub = (zenoh_ros2_sub_t *)arg;
    if (sub && sub->user_cb) {
        z_owned_slice_t slice;
        if (z_bytes_to_slice(z_sample_payload(sample), &slice) == 0) {
            const uint8_t *data = z_slice_data(z_loan(slice));
            size_t len = z_slice_len(z_loan(slice));
            sub->user_cb(data, len, sub->user_ctx);
            z_drop(z_move(slice));
        }
    }
}

bool zenoh_ros2_sub_create(zenoh_ros2_sub_t *sub, zenoh_ros2_node_t *node,
                           const char *topic_name, const char *type_name, const char *type_hash,
                           zenoh_ros2_sub_cb_t cb, void *ctx) {
    if (!sub || !node || !node->session || !topic_name || !type_name || !type_hash || !cb) {
        return false;
    }

    memset(sub, 0, sizeof(*sub));
    sub->node = node;
    sub->topic_name = topic_name;
    sub->type_name = type_name;
    sub->type_hash = type_hash;
    sub->user_cb = cb;
    sub->user_ctx = ctx;
    sub->entity_id = node->next_entity_id++;

    const char *raw_topic = strip_leading_slash(topic_name);

    char token_ns[64];
    sanitize_for_token(token_ns, sizeof(token_ns), node->ns);

    char token_topic[96];
    sanitize_for_token(token_topic, sizeof(token_topic), topic_name);

    /* 1. Declare Subscriber Liveliness Token */
    char token_ke_str[ZENOH_ROS2_MAX_KEYEXPR_LEN];
    snprintf(token_ke_str, sizeof(token_ke_str),
             "@ros2_lv/%lu/%s/%lu/%lu/MS/%%/%s/%s/%s/%s/%s/%s",
             (unsigned long)node->domain_id,
             ZENOH_ROS2_SESSION_ID,
             (unsigned long)node->node_id,
             (unsigned long)sub->entity_id,
             token_ns,
             node->name,
             token_topic,
             type_name,
             type_hash,
             ZENOH_ROS2_DEFAULT_QOS);

    z_view_keyexpr_t token_ke;
    z_view_keyexpr_from_str_unchecked(&token_ke, token_ke_str);

    if (z_liveliness_declare_token(z_loan(*node->session), &sub->token, z_loan(token_ke), NULL) < 0) {
        printf("[ROS2] Error: failed to declare subscriber liveliness token (%s)\r\n", token_ke_str);
        return false;
    }

    /* 2. Declare Data Subscriber */
    char data_ke_str[ZENOH_ROS2_MAX_KEYEXPR_LEN];
    snprintf(data_ke_str, sizeof(data_ke_str),
             "%lu/%s/%s/%s",
             (unsigned long)node->domain_id,
             raw_topic,
             type_name,
             type_hash);

    z_view_keyexpr_t data_ke;
    z_view_keyexpr_from_str_unchecked(&data_ke, data_ke_str);

    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, subscriber_trampoline, NULL, sub);

    if (z_declare_subscriber(z_loan(*node->session), &sub->sub, z_loan(data_ke), z_move(callback), NULL) < 0) {
        printf("[ROS2] Error: failed to declare subscriber for key (%s)\r\n", data_ke_str);
        z_drop(z_move(sub->token));
        return false;
    }

    sub->is_declared = true;
    printf("[ROS2] Subscriber declared for '/%s' (entity_id: %lu)\r\n",
           raw_topic, (unsigned long)sub->entity_id);
    return true;
}

void zenoh_ros2_sub_destroy(zenoh_ros2_sub_t *sub) {
    if (!sub) return;
    if (sub->is_declared) {
        z_drop(z_move(sub->token));
        z_drop(z_move(sub->sub));
        sub->is_declared = false;
        printf("[ROS2] Subscriber '/%s' destroyed\r\n",
               sub->topic_name ? sub->topic_name : "");
    }
}
