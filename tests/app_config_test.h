/**
 * @file app_config_test.h
 * @brief Minimal test configuration for bridge engine tests
 */

#ifndef APP_CONFIG_TEST_H
#define APP_CONFIG_TEST_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ucdr/microcdr.h>

/* Generated message headers */
#include "generated/MotorStatus.h"
#include "generated/ImuData.h"
#include "generated/MotorCommand.h"
#include "generated/Ping.h"
#include "generated/transport/Frame.h"
#include "generated/transport/FDFrame.h"

/* Bridge types from app_config.h */
typedef enum {
    BRIDGE_DIR_CAN_TO_ROS = 0,
    BRIDGE_DIR_ROS_TO_CAN,
    BRIDGE_DIR_CAN_RECV,
    BRIDGE_DIR_CAN_ECHO = BRIDGE_DIR_CAN_RECV,
    BRIDGE_DIR_ROS_TO_CAN_RAW,
    BRIDGE_DIR_CAN_TO_ROS_RAW,
} bridge_dir_t;

typedef enum {
    BRIDGE_FDCAN1 = 1,
    BRIDGE_FDCAN2 = 2
} bridge_can_bus_t;

typedef enum {
    BRIDGE_FRAME_CLASSIC = 0,
    BRIDGE_FRAME_FD      = 1
} bridge_frame_mode_t;

typedef enum {
    BRIDGE_PAYLOAD_TYPED = 0,
    BRIDGE_PAYLOAD_CLASSIC_FRAME,
    BRIDGE_PAYLOAD_FD_FRAME
} bridge_payload_kind_t;

typedef void (*msg_print_fn_t)(const void *topic);
typedef bool (*cdr_serialize_fn_t)(ucdrBuffer *ub, const void *topic);
typedef bool (*cdr_deserialize_fn_t)(ucdrBuffer *ub, void *topic);

typedef struct {
    const char           *topic_name;
    bridge_dir_t          dir;
    bridge_can_bus_t      bus;
    bridge_frame_mode_t   frame_mode;
    bridge_payload_kind_t payload_kind;
    uint32_t              can_base_id;
    size_t                msg_size;
    const char           *dds_type;
    const char           *type_hash;
    cdr_serialize_fn_t    serialize_fn;
    cdr_deserialize_fn_t  deserialize_fn;
    msg_print_fn_t        print_fn;
} bridge_topic_t;

#define BRIDGE_MAX_MSG_SIZE 128

/* Test topic table - mirrors app_config.h */
extern const bridge_topic_t g_bridge_topics[];
extern const size_t BRIDGE_TOPIC_COUNT;

#endif /* APP_CONFIG_TEST_H */