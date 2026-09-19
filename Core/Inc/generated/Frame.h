#ifndef _CAN_MSGS_FRAME_H_
#define _CAN_MSGS_FRAME_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ucdr/microcdr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Type metadata (ROS 2 REP-2016 compatible) */
#define can_msgs_Frame_PACKAGE "can_msgs"
#define can_msgs_Frame_MSG_NAME "Frame"
#define can_msgs_Frame_DDS_TYPE "can_msgs::msg::dds_::Frame_"
#define can_msgs_Frame_TYPE_HASH "RIHS01_0cdc7d15657573ee6ff151e5cc1943888301e370d957e2c077a154f4755fd976"

/* Macro to generate Zenoh KeyExpr: <domain_id>/<topic_name>/<dds_type>/<type_hash> */
#define can_msgs_Frame_KEYEXPR(domain_id, topic_name) \
    domain_id "/" topic_name "/" can_msgs_Frame_DDS_TYPE "/" can_msgs_Frame_TYPE_HASH

/* Message structure */
typedef struct {
    struct {
        int32_t sec;
        uint32_t nanosec;
        char frame_id[32];
    } header;
    uint32_t id;
    bool is_rtr;
    bool is_extended;
    bool is_error;
    uint8_t dlc;
    uint8_t data[8];
} can_msgs_Frame;

/**
 * @brief Serialize can_msgs_Frame to CDR format with header
 */
static inline bool can_msgs_Frame_serialize(ucdrBuffer* ub, const can_msgs_Frame* topic) {
    if (ub == NULL || topic == NULL) return false;

    // 1. CDR encapsulation header (Little Endian)
    ucdr_serialize_uint8_t(ub, 0x00);
    ucdr_serialize_uint8_t(ub, 0x01);
    ucdr_serialize_uint16_t(ub, 0x0000);

    // 2. Header
    ucdr_serialize_int32_t(ub, topic->header.sec);
    ucdr_serialize_uint32_t(ub, topic->header.nanosec);
    const char *fid = topic->header.frame_id ? topic->header.frame_id : "";
    ucdr_serialize_string(ub, fid);

    // 3. Frame fields
    ucdr_serialize_uint32_t(ub, topic->id);
    ucdr_serialize_bool(ub, topic->is_rtr);
    ucdr_serialize_bool(ub, topic->is_extended);
    ucdr_serialize_bool(ub, topic->is_error);
    ucdr_serialize_uint8_t(ub, topic->dlc);
    ucdr_serialize_array_uint8_t(ub, topic->data, 8);

    return !ucdr_buffer_has_error(ub);
}

/**
 * @brief Deserialize can_msgs_Frame from CDR format with header
 */
static inline bool can_msgs_Frame_deserialize(ucdrBuffer* ub, can_msgs_Frame* topic) {
    if (ub == NULL || topic == NULL) return false;

    // 1. Consume CDR encapsulation header
    uint8_t dummy8;
    uint16_t dummy16;
    ucdr_deserialize_uint8_t(ub, &dummy8);
    ucdr_deserialize_uint8_t(ub, &dummy8);
    ucdr_deserialize_uint16_t(ub, &dummy16);

    // 2. Header
    ucdr_deserialize_int32_t(ub, &topic->header.sec);
    ucdr_deserialize_uint32_t(ub, &topic->header.nanosec);
    ucdr_deserialize_string(ub, topic->header.frame_id, sizeof(topic->header.frame_id));

    // 3. Frame fields
    ucdr_deserialize_uint32_t(ub, &topic->id);
    ucdr_deserialize_bool(ub, &topic->is_rtr);
    ucdr_deserialize_bool(ub, &topic->is_extended);
    ucdr_deserialize_bool(ub, &topic->is_error);
    ucdr_deserialize_uint8_t(ub, &topic->dlc);
    ucdr_deserialize_array_uint8_t(ub, topic->data, 8);

    return !ucdr_buffer_has_error(ub);
}

#ifdef __cplusplus
}
#endif

#endif // _CAN_MSGS_FRAME_H_
