/**
 * @file bridge_topics.h
 * @brief Declarative Bridge Routing Table (Single Source of Truth)
 *
 * HOW TO ADD A NEW TOPIC:
 * 1. Place your .msg file in test_msgs/msg/ and run `make msg`.
 * 2. #include the generated header here.
 * 3. Add an entry to g_bridge_topics[].
 * That's it! Zenoh pub/sub creation, CAN ID binding, fragmentation,
 * and CDR conversion are all handled automatically by the engine.
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

/**
 * @brief Master Routing Table
 */
static const bridge_topic_t g_bridge_topics[] = {
    /* 1. MotorStatus: CAN -> ROS 2 (/motor_status, 28B = 4 CAN frames: 0x100..0x103) */
    {
        .topic_name     = "motor_status",
        .dir            = BRIDGE_DIR_CAN_TO_ROS,
        .can_base_id    = 0x100,
        .msg_size       = sizeof(robot_msgs_MotorStatus),
        .dds_type       = robot_msgs_MotorStatus_DDS_TYPE,
        .type_hash      = robot_msgs_MotorStatus_TYPE_HASH,
        .serialize_fn   = (cdr_serialize_fn_t)robot_msgs_MotorStatus_serialize,
        .deserialize_fn = NULL,
    },

    /* 2. ImuData: CAN -> ROS 2 (/imu_data, 24B = 3 CAN frames: 0x200..0x202) */
    {
        .topic_name     = "imu_data",
        .dir            = BRIDGE_DIR_CAN_TO_ROS,
        .can_base_id    = 0x200,
        .msg_size       = sizeof(robot_msgs_ImuData),
        .dds_type       = robot_msgs_ImuData_DDS_TYPE,
        .type_hash      = robot_msgs_ImuData_TYPE_HASH,
        .serialize_fn   = (cdr_serialize_fn_t)robot_msgs_ImuData_serialize,
        .deserialize_fn = NULL,
    },

    /* 3. MotorCommand: ROS 2 -> CAN (/motor_command, 8B = 1 CAN frame: 0x300) */
    {
        .topic_name     = "motor_command",
        .dir            = BRIDGE_DIR_ROS_TO_CAN,
        .can_base_id    = 0x300,
        .msg_size       = sizeof(robot_msgs_MotorCommand),
        .dds_type       = robot_msgs_MotorCommand_DDS_TYPE,
        .type_hash      = robot_msgs_MotorCommand_TYPE_HASH,
        .serialize_fn   = NULL,
        .deserialize_fn = (cdr_deserialize_fn_t)robot_msgs_MotorCommand_deserialize,
    },
};

#define BRIDGE_TOPIC_COUNT (sizeof(g_bridge_topics) / sizeof(g_bridge_topics[0]))
#define BRIDGE_MAX_MSG_SIZE 64  /* Max message payload buffer size */

#endif /* BRIDGE_TOPICS_H */
