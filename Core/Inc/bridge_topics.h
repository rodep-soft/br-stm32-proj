/**
 * @file bridge_topics.h
 * @brief Declarative Bridge Routing Table (Single Source of Truth)
 *
 * HOW TO ADD A NEW TOPIC:
 * 1. Place your .msg file in test_msgs/msg/ and run `make msg`.
 * 2. #include the generated header below.
 * 3. Add a single line using BRIDGE_CAN_TO_ROS() or BRIDGE_ROS_TO_CAN()!
 */

#ifndef BRIDGE_TOPICS_H
#define BRIDGE_TOPICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ucdr/microcdr.h>

/* Generated message headers */
#include "generated/MotorStatus.h"
#include "generated/ImuData.h"
#include "generated/MotorCommand.h"
#include "generated/Frame.h"

typedef bool (*cdr_serialize_fn_t)(ucdrBuffer *ub, const void *topic);
typedef bool (*cdr_deserialize_fn_t)(ucdrBuffer *ub, void *topic);

typedef enum {
    BRIDGE_DIR_CAN_TO_ROS = 0,  /**< CAN frames assembled -> ROS 2 (Zenoh Publish) */
    BRIDGE_DIR_ROS_TO_CAN,      /**< ROS 2 message (Zenoh Subscribe) -> CAN frames */
} bridge_dir_t;

typedef struct {
    const char           *topic_name;       /**< ROS 2 topic name (e.g. "motor_status") */
    bridge_dir_t          dir;              /**< Communication direction */
    uint32_t              can_base_id;      /**< Starting CAN ID (subsequent frames use base+1, base+2...) */
    size_t                msg_size;         /**< sizeof(MessageStruct) */
    const char           *dds_type;         /**< ROS 2 DDS type string */
    const char           *type_hash;        /**< REP-2016 Type hash */
    cdr_serialize_fn_t    serialize_fn;     /**< (For CAN_TO_ROS) Micro-CDR serialize function */
    cdr_deserialize_fn_t  deserialize_fn;   /**< (For ROS_TO_CAN) Micro-CDR deserialize function */
} bridge_topic_t;

/* ──────────────────── Professional Helper Macros ──────────────────── */

/**
 * @brief Register a CAN -> ROS 2 topic (Sensor telemetry)
 * @param topic   ROS 2 topic name (e.g. "motor_status")
 * @param msg     Message name without package prefix (e.g. MotorStatus)
 * @param can_id  Base CAN ID (e.g. 0x100)
 */
#define BRIDGE_CAN_TO_ROS(topic, msg, can_id) \
    { \
        .topic_name     = topic, \
        .dir            = BRIDGE_DIR_CAN_TO_ROS, \
        .can_base_id    = can_id, \
        .msg_size       = sizeof(robot_msgs_##msg), \
        .dds_type       = robot_msgs_##msg##_DDS_TYPE, \
        .type_hash      = robot_msgs_##msg##_TYPE_HASH, \
        .serialize_fn   = (cdr_serialize_fn_t)robot_msgs_##msg##_serialize, \
        .deserialize_fn = NULL, \
    }

/**
 * @brief Register a ROS 2 -> CAN topic (Actuator / control command)
 * @param topic   ROS 2 topic name (e.g. "motor_command")
 * @param msg     Message name without package prefix (e.g. MotorCommand)
 * @param can_id  Base CAN ID (e.g. 0x300)
 */
#define BRIDGE_ROS_TO_CAN(topic, msg, can_id) \
    { \
        .topic_name     = topic, \
        .dir            = BRIDGE_DIR_ROS_TO_CAN, \
        .can_base_id    = can_id, \
        .msg_size       = sizeof(robot_msgs_##msg), \
        .dds_type       = robot_msgs_##msg##_DDS_TYPE, \
        .type_hash      = robot_msgs_##msg##_TYPE_HASH, \
        .serialize_fn   = NULL, \
        .deserialize_fn = (cdr_deserialize_fn_t)robot_msgs_##msg##_deserialize, \
    }

/**
 * @brief Custom package variant for third-party ROS 2 messages (e.g. can_msgs/Frame)
 */
#define BRIDGE_CAN_TO_ROS_PKG(pkg, topic, msg, can_id) \
    { \
        .topic_name     = topic, \
        .dir            = BRIDGE_DIR_CAN_TO_ROS, \
        .can_base_id    = can_id, \
        .msg_size       = sizeof(pkg##_##msg), \
        .dds_type       = pkg##_##msg##_DDS_TYPE, \
        .type_hash      = pkg##_##msg##_TYPE_HASH, \
        .serialize_fn   = (cdr_serialize_fn_t)pkg##_##msg##_serialize, \
        .deserialize_fn = NULL, \
    }

/* ──────────────────── Master Routing Table ─────────────────────────── */

static const bridge_topic_t g_bridge_topics[] = {
    /* 1. CAN -> ROS 2: MotorStatus (28B = 4 CAN frames: 0x100..0x103) */
    BRIDGE_CAN_TO_ROS("motor_status",  MotorStatus,  0x100),

    /* 2. CAN -> ROS 2: ImuData (24B = 3 CAN frames: 0x200..0x202) */
    BRIDGE_CAN_TO_ROS("imu_data",      ImuData,      0x200),

    /* 3. ROS 2 -> CAN: MotorCommand (8B = 1 CAN frame: 0x300) */
    BRIDGE_ROS_TO_CAN("motor_command", MotorCommand, 0x300),
};

#define BRIDGE_TOPIC_COUNT (sizeof(g_bridge_topics) / sizeof(g_bridge_topics[0]))
#define BRIDGE_MAX_MSG_SIZE 64  /* Max message payload buffer size */

#endif /* BRIDGE_TOPICS_H */
