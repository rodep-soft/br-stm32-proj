/**
 * @file zenoh_ros2.h
 * @brief Professional ROS 2 Native Abstraction Layer over Zenoh-Pico
 *
 * Handles ROS 2 Node/Publisher/Subscriber lifecycle, key expression formatting,
 * CDR serialization, and rmw_zenoh_cpp Liveliness Token generation for
 * automatic ROS 2 graph discovery (ROS 2 Jazzy, Lyrical, Rolling compatible).
 */

#ifndef ZENOH_ROS2_H
#define ZENOH_ROS2_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <zenoh-pico.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZENOH_ROS2_DEFAULT_DOMAIN_ID 0
#define ZENOH_ROS2_MAX_KEYEXPR_LEN   384

/* Default QoS: Volatile, Best-Effort / Reliable, Transient-Local off */
#define ZENOH_ROS2_DEFAULT_QOS "::,:,,:,,:,,"

/* Common ROS 2 Message Types & REP-2016 Type Hashes (Jazzy / Lyrical / Rolling) */
#define ROS2_TYPE_STD_MSGS_STRING "std_msgs::msg::dds_::String_"
#define ROS2_HASH_STD_MSGS_STRING "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"

#define ROS2_TYPE_STD_MSGS_INT32  "std_msgs::msg::dds_::Int32_"
#define ROS2_HASH_STD_MSGS_INT32  "RIHS01_93e1507d4b4f0b2f5670caae048fa56294d1abcb52b1464c24ccb50a6ef2b1d3"

#define ROS2_TYPE_STD_MSGS_BOOL   "std_msgs::msg::dds_::Bool_"
#define ROS2_HASH_STD_MSGS_BOOL   "RIHS01_561bc699cfef9fa271f2514eb58c679a6136ff9c065f4ec38d4f4ff70ad255dc"

/**
 * @brief ROS 2 Node representation in Zenoh
 */
typedef struct {
    z_owned_session_t *session;
    const char *name;
    const char *ns;
    uint32_t domain_id;
    uint32_t node_id;
    uint32_t next_entity_id;
    z_owned_liveliness_token_t token;
    bool is_declared;
} zenoh_ros2_node_t;

/**
 * @brief ROS 2 Publisher representation in Zenoh
 */
typedef struct {
    zenoh_ros2_node_t *node;
    const char *topic_name;
    const char *type_name;
    const char *type_hash;
    uint32_t entity_id;
    int64_t sequence_number;
    uint8_t gid[16];
    z_owned_publisher_t pub;
    z_owned_liveliness_token_t token;
    bool is_declared;
} zenoh_ros2_pub_t;

/**
 * @brief ROS 2 Subscriber callback signature
 */
typedef void (*zenoh_ros2_sub_cb_t)(const uint8_t *payload, size_t len, void *ctx);

/**
 * @brief ROS 2 Subscriber representation in Zenoh
 */
typedef struct {
    zenoh_ros2_node_t *node;
    const char *topic_name;
    const char *type_name;
    const char *type_hash;
    uint32_t entity_id;
    z_owned_subscriber_t sub;
    z_owned_liveliness_token_t token;
    zenoh_ros2_sub_cb_t user_cb;
    void *user_ctx;
    bool is_declared;
} zenoh_ros2_sub_t;

/**
 * @brief Initialize a ROS 2 Node on an active Zenoh session.
 * Automatically generates Node ID and declares the Node Liveliness Token for ROS graph discovery.
 *
 * @param node      Node structure to initialize
 * @param session   Active Zenoh session pointer
 * @param name      ROS 2 node name (e.g. "stm32_node")
 * @param ns        Namespace (NULL, "" or "/" for root namespace)
 * @param domain_id ROS_DOMAIN_ID (default: 0)
 * @return true on success
 */
bool zenoh_ros2_node_init(zenoh_ros2_node_t *node, z_owned_session_t *session,
                          const char *name, const char *ns, uint32_t domain_id);

/**
 * @brief Destroy a ROS 2 Node and undeclare its liveliness token.
 */
void zenoh_ros2_node_fini(zenoh_ros2_node_t *node);

/**
 * @brief Create a ROS 2 Publisher on a node.
 * Automatically registers the Topic Data KeyExpr and Publisher Liveliness Token.
 *
 * @param pub         Publisher structure to initialize
 * @param node        Parent ROS 2 node
 * @param topic_name  Topic name (e.g. "chatter" or "/chatter")
 * @param type_name   DDS type name (e.g. ROS2_TYPE_STD_MSGS_STRING)
 * @param type_hash   REP-2016 RIHS01 hash (e.g. ROS2_HASH_STD_MSGS_STRING)
 * @return true on success
 */
bool zenoh_ros2_pub_create(zenoh_ros2_pub_t *pub, zenoh_ros2_node_t *node,
                           const char *topic_name, const char *type_name, const char *type_hash);

/**
 * @brief Publish CDR serialized payload to the ROS 2 topic.
 *
 * @param pub          Publisher handle
 * @param cdr_payload  Serialized CDR byte buffer
 * @param len          Length of CDR payload
 * @return true on success
 */
bool zenoh_ros2_pub_send(zenoh_ros2_pub_t *pub, const uint8_t *cdr_payload, size_t len);

/**
 * @brief Destroy a ROS 2 Publisher and release Zenoh tokens.
 */
void zenoh_ros2_pub_destroy(zenoh_ros2_pub_t *pub);

/**
 * @brief Create a ROS 2 Subscriber on a node.
 *
 * @param sub         Subscriber structure to initialize
 * @param node        Parent ROS 2 node
 * @param topic_name  Topic name (e.g. "cmd_vel" or "/cmd_vel")
 * @param type_name   DDS type name
 * @param type_hash   REP-2016 RIHS01 hash
 * @param cb          Callback invoked when a message is received
 * @param ctx         User context pointer passed to callback
 * @return true on success
 */
bool zenoh_ros2_sub_create(zenoh_ros2_sub_t *sub, zenoh_ros2_node_t *node,
                           const char *topic_name, const char *type_name, const char *type_hash,
                           zenoh_ros2_sub_cb_t cb, void *ctx);

/**
 * @brief Destroy a ROS 2 Subscriber and release Zenoh tokens.
 */
void zenoh_ros2_sub_destroy(zenoh_ros2_sub_t *sub);

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_ROS2_H */
