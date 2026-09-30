/**
 * @file app_config_test.c
 * @brief Test topic table for bridge engine tests
 */

#include "app_config_test.h"

const bridge_topic_t g_bridge_topics[] = {
    {
        .topic_name = "motor_status",
        .dir = BRIDGE_DIR_CAN_TO_ROS,
        .bus = BRIDGE_FDCAN1,
        .frame_mode = BRIDGE_FRAME_CLASSIC,
        .payload_kind = BRIDGE_PAYLOAD_TYPED,
        .can_base_id = 0x100,
        .msg_size = sizeof(robot_msgs_MotorStatus),
        .dds_type = robot_msgs_MotorStatus_DDS_TYPE,
        .type_hash = robot_msgs_MotorStatus_TYPE_HASH,
        .serialize_fn = (cdr_serialize_fn_t)robot_msgs_MotorStatus_serialize,
        .deserialize_fn = NULL,
        .print_fn = (msg_print_fn_t)robot_msgs_MotorStatus_print
    },
    {
        .topic_name = "imu_data",
        .dir = BRIDGE_DIR_CAN_TO_ROS,
        .bus = BRIDGE_FDCAN1,
        .frame_mode = BRIDGE_FRAME_CLASSIC,
        .payload_kind = BRIDGE_PAYLOAD_TYPED,
        .can_base_id = 0x200,
        .msg_size = sizeof(robot_msgs_ImuData),
        .dds_type = robot_msgs_ImuData_DDS_TYPE,
        .type_hash = robot_msgs_ImuData_TYPE_HASH,
        .serialize_fn = (cdr_serialize_fn_t)robot_msgs_ImuData_serialize,
        .deserialize_fn = NULL,
        .print_fn = (msg_print_fn_t)robot_msgs_ImuData_print
    },
    {
        .topic_name = "motor_command",
        .dir = BRIDGE_DIR_ROS_TO_CAN,
        .bus = BRIDGE_FDCAN1,
        .frame_mode = BRIDGE_FRAME_CLASSIC,
        .payload_kind = BRIDGE_PAYLOAD_TYPED,
        .can_base_id = 0x300,
        .msg_size = sizeof(robot_msgs_MotorCommand),
        .dds_type = robot_msgs_MotorCommand_DDS_TYPE,
        .type_hash = robot_msgs_MotorCommand_TYPE_HASH,
        .serialize_fn = NULL,
        .deserialize_fn = (cdr_deserialize_fn_t)robot_msgs_MotorCommand_deserialize,
        .print_fn = (msg_print_fn_t)robot_msgs_MotorCommand_print
    },
    {
        .topic_name = "ping_echo",
        .dir = BRIDGE_DIR_CAN_RECV,
        .bus = BRIDGE_FDCAN1,
        .frame_mode = BRIDGE_FRAME_CLASSIC,
        .payload_kind = BRIDGE_PAYLOAD_TYPED,
        .can_base_id = 0x400,
        .msg_size = sizeof(robot_msgs_Ping),
        .dds_type = robot_msgs_Ping_DDS_TYPE,
        .type_hash = robot_msgs_Ping_TYPE_HASH,
        .serialize_fn = (cdr_serialize_fn_t)robot_msgs_Ping_serialize,
        .deserialize_fn = (cdr_deserialize_fn_t)robot_msgs_Ping_deserialize,
        .print_fn = (msg_print_fn_t)robot_msgs_Ping_print
    },
};

const size_t BRIDGE_TOPIC_COUNT = sizeof(g_bridge_topics) / sizeof(g_bridge_topics[0]);