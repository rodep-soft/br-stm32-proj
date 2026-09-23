#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "generated/MotorStatus.h"
#include "generated/ImuData.h"
#include "generated/MotorCommand.h"

/* CAN IDs */
#define CAN_ID_MOTOR_STATUS_BASE 0x100
#define CAN_ID_IMU_DATA_BASE     0x200
#define CAN_ID_MOTOR_COMMAND     0x300
#define CAN_ID_MOTOR_CMD_BASE    CAN_ID_MOTOR_COMMAND
#define CAN_ID_HEARTBEAT         0x001

/* Frame Counts */
#define CAN_MSG_MOTOR_STATUS_FRAME_COUNT 4
#define CAN_MSG_IMU_DATA_FRAME_COUNT     3
#define CAN_MSG_MOTOR_COMMAND_FRAME_COUNT 1

typedef enum {
    CAN_MSG_MOTOR_STATUS,
    CAN_MSG_IMU_DATA,
    CAN_MSG_MOTOR_COMMAND,
    CAN_MSG_HEARTBEAT,
    CAN_MSG_UNKNOWN
} can_msg_type_t;

static inline void can_pack_motor_status(const robot_msgs_MotorStatus *msg, uint8_t frame_idx, uint8_t data[8], uint8_t *dlc) {
    if (!msg || !data || !dlc) return;
    
    switch (frame_idx) {
        case 0:
            memcpy(&data[0], &msg->sequence, 4);
            memcpy(&data[4], &msg->velocity, 4);
            *dlc = 8;
            break;
        case 1:
            memcpy(&data[0], &msg->current, 4);
            memcpy(&data[4], &msg->position[0], 4);
            *dlc = 8;
            break;
        case 2:
            memcpy(&data[0], &msg->position[1], 4);
            memcpy(&data[4], &msg->position[2], 4);
            *dlc = 8;
            break;
        case 3:
            data[0] = msg->is_enabled ? 1 : 0;
            *dlc = 1;
            break;
        default:
            *dlc = 0;
            break;
    }
}

static inline void can_unpack_motor_status(robot_msgs_MotorStatus *msg, uint8_t frame_idx, const uint8_t data[8], uint8_t dlc) {
    if (!msg || !data) return;
    
    switch (frame_idx) {
        case 0:
            if (dlc == 8) {
                memcpy(&msg->sequence, &data[0], 4);
                memcpy(&msg->velocity, &data[4], 4);
            }
            break;
        case 1:
            if (dlc == 8) {
                memcpy(&msg->current, &data[0], 4);
                memcpy(&msg->position[0], &data[4], 4);
            }
            break;
        case 2:
            if (dlc == 8) {
                memcpy(&msg->position[1], &data[0], 4);
                memcpy(&msg->position[2], &data[4], 4);
            }
            break;
        case 3:
            if (dlc >= 1) {
                msg->is_enabled = (data[0] != 0);
            }
            break;
        default:
            break;
    }
}

static inline void can_pack_imu_data(const robot_msgs_ImuData *msg, uint8_t frame_idx, uint8_t data[8], uint8_t *dlc) {
    if (!msg || !data || !dlc) return;
    
    switch (frame_idx) {
        case 0:
            memcpy(&data[0], &msg->accel_x, 4);
            memcpy(&data[4], &msg->accel_y, 4);
            *dlc = 8;
            break;
        case 1:
            memcpy(&data[0], &msg->accel_z, 4);
            memcpy(&data[4], &msg->gyro_x, 4);
            *dlc = 8;
            break;
        case 2:
            memcpy(&data[0], &msg->gyro_y, 4);
            memcpy(&data[4], &msg->gyro_z, 4);
            *dlc = 8;
            break;
        default:
            *dlc = 0;
            break;
    }
}

static inline void can_unpack_imu_data(robot_msgs_ImuData *msg, uint8_t frame_idx, const uint8_t data[8], uint8_t dlc) {
    if (!msg || !data) return;
    
    switch (frame_idx) {
        case 0:
            if (dlc == 8) {
                memcpy(&msg->accel_x, &data[0], 4);
                memcpy(&msg->accel_y, &data[4], 4);
            }
            break;
        case 1:
            if (dlc == 8) {
                memcpy(&msg->accel_z, &data[0], 4);
                memcpy(&msg->gyro_x, &data[4], 4);
            }
            break;
        case 2:
            if (dlc == 8) {
                memcpy(&msg->gyro_y, &data[0], 4);
                memcpy(&msg->gyro_z, &data[4], 4);
            }
            break;
        default:
            break;
    }
}

static inline void can_pack_motor_command(const robot_msgs_MotorCommand *msg, uint8_t frame_idx, uint8_t data[8], uint8_t *dlc) {
    if (!msg || !data || !dlc) return;
    
    if (frame_idx == 0) {
        memcpy(&data[0], &msg->velocity, 4);
        memcpy(&data[4], &msg->torque_limit, 4);
        *dlc = 8;
    } else {
        *dlc = 0;
    }
}

static inline void can_unpack_motor_command(robot_msgs_MotorCommand *msg, uint8_t frame_idx, const uint8_t data[8], uint8_t dlc) {
    if (!msg || !data) return;
    
    if (frame_idx == 0 && dlc == 8) {
        memcpy(&msg->velocity, &data[0], 4);
        memcpy(&msg->torque_limit, &data[4], 4);
    }
}

#endif /* CAN_PROTOCOL_H */
