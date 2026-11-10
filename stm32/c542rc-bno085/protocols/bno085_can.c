/**
 * @file bno085_can.c
 *
 * @brief BNO085 CAN-FD 18-byte IMU protocol implementation (20-byte DLC frame).
 *
 * @copyright Copyright (c) 2018-2019 Erik Moqvist
 *
 * @par License
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <math.h>
#include <string.h>

#include "bno085_can.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u32(
    uint32_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u32(
    uint32_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint32_t unpack_left_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

static inline uint32_t unpack_right_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) >> shift);
}

static uint64_t cantools_unsigned_from_double(double value)
{
    if (value < 0) {
        return 0;
    }

    return (uint64_t)value;
}

int bno085_can_bno085_data_pack(
    uint8_t *dst_p,
    const struct bno085_can_bno085_data_t *src_p,
    size_t size)
{
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;

    if (size < BNO085_CAN_BNO085_DATA_PAYLOAD_LENGTH) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, size >= BNO085_CAN_BNO085_DATA_LENGTH
                         ? BNO085_CAN_BNO085_DATA_LENGTH
                         : BNO085_CAN_BNO085_DATA_PAYLOAD_LENGTH);

    dst_p[0] |= pack_left_shift_u32(src_p->timestamp_ms, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(src_p->timestamp_ms, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->timestamp_ms, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->timestamp_ms, 24u, 0xffu);
    dst_p[4] |= pack_left_shift_u8(src_p->seq, 0u, 0xffu);
    dst_p[5] |= pack_left_shift_u8(src_p->status, 0u, 0xffu);
    accel_x = (uint16_t)src_p->accel_x;
    dst_p[6] |= pack_left_shift_u16(accel_x, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(accel_x, 8u, 0xffu);
    accel_y = (uint16_t)src_p->accel_y;
    dst_p[8] |= pack_left_shift_u16(accel_y, 0u, 0xffu);
    dst_p[9] |= pack_right_shift_u16(accel_y, 8u, 0xffu);
    accel_z = (uint16_t)src_p->accel_z;
    dst_p[10] |= pack_left_shift_u16(accel_z, 0u, 0xffu);
    dst_p[11] |= pack_right_shift_u16(accel_z, 8u, 0xffu);
    gyro_x = (uint16_t)src_p->gyro_x;
    dst_p[12] |= pack_left_shift_u16(gyro_x, 0u, 0xffu);
    dst_p[13] |= pack_right_shift_u16(gyro_x, 8u, 0xffu);
    gyro_y = (uint16_t)src_p->gyro_y;
    dst_p[14] |= pack_left_shift_u16(gyro_y, 0u, 0xffu);
    dst_p[15] |= pack_right_shift_u16(gyro_y, 8u, 0xffu);
    gyro_z = (uint16_t)src_p->gyro_z;
    dst_p[16] |= pack_left_shift_u16(gyro_z, 0u, 0xffu);
    dst_p[17] |= pack_right_shift_u16(gyro_z, 8u, 0xffu);

    return (int)BNO085_CAN_BNO085_DATA_PAYLOAD_LENGTH;
}

int bno085_can_bno085_data_unpack(
    struct bno085_can_bno085_data_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;

    if (size < BNO085_CAN_BNO085_DATA_PAYLOAD_LENGTH) {
        return (-EINVAL);
    }

    dst_p->timestamp_ms = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    dst_p->timestamp_ms |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    dst_p->timestamp_ms |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    dst_p->timestamp_ms |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);
    dst_p->seq = unpack_right_shift_u8(src_p[4], 0u, 0xffu);
    dst_p->status = unpack_right_shift_u8(src_p[5], 0u, 0xffu);
    accel_x = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    accel_x |= unpack_left_shift_u16(src_p[7], 8u, 0xffu);
    dst_p->accel_x = (int16_t)accel_x;
    accel_y = unpack_right_shift_u16(src_p[8], 0u, 0xffu);
    accel_y |= unpack_left_shift_u16(src_p[9], 8u, 0xffu);
    dst_p->accel_y = (int16_t)accel_y;
    accel_z = unpack_right_shift_u16(src_p[10], 0u, 0xffu);
    accel_z |= unpack_left_shift_u16(src_p[11], 8u, 0xffu);
    dst_p->accel_z = (int16_t)accel_z;
    gyro_x = unpack_right_shift_u16(src_p[12], 0u, 0xffu);
    gyro_x |= unpack_left_shift_u16(src_p[13], 8u, 0xffu);
    dst_p->gyro_x = (int16_t)gyro_x;
    gyro_y = unpack_right_shift_u16(src_p[14], 0u, 0xffu);
    gyro_y |= unpack_left_shift_u16(src_p[15], 8u, 0xffu);
    dst_p->gyro_y = (int16_t)gyro_y;
    gyro_z = unpack_right_shift_u16(src_p[16], 0u, 0xffu);
    gyro_z |= unpack_left_shift_u16(src_p[17], 8u, 0xffu);
    dst_p->gyro_z = (int16_t)gyro_z;

    return (0);
}

int bno085_can_bno085_data_init(struct bno085_can_bno085_data_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct bno085_can_bno085_data_t));

    return 0;
}

uint32_t bno085_can_bno085_data_timestamp_ms_encode(double value)
{
    return (uint32_t)(cantools_unsigned_from_double(value));
}

double bno085_can_bno085_data_timestamp_ms_decode(uint32_t value)
{
    return ((double)value);
}

bool bno085_can_bno085_data_timestamp_ms_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_timestamp_ms_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 4294967295.0));
}

uint8_t bno085_can_bno085_data_seq_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double bno085_can_bno085_data_seq_decode(uint8_t value)
{
    return ((double)value);
}

bool bno085_can_bno085_data_seq_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_seq_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

uint8_t bno085_can_bno085_data_status_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double bno085_can_bno085_data_status_decode(uint8_t value)
{
    return ((double)value);
}

bool bno085_can_bno085_data_status_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_status_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int16_t bno085_can_bno085_data_accel_x_encode(double value)
{
    return (int16_t)round(value * 1000.0);
}

double bno085_can_bno085_data_accel_x_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool bno085_can_bno085_data_accel_x_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_accel_x_is_in_phys_range(double value)
{
    double scaled = round(value * 1000.0);
    return ((scaled >= -32768.0) && (scaled <= 32767.0));
}

int16_t bno085_can_bno085_data_accel_y_encode(double value)
{
    return (int16_t)round(value * 1000.0);
}

double bno085_can_bno085_data_accel_y_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool bno085_can_bno085_data_accel_y_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_accel_y_is_in_phys_range(double value)
{
    double scaled = round(value * 1000.0);
    return ((scaled >= -32768.0) && (scaled <= 32767.0));
}

int16_t bno085_can_bno085_data_accel_z_encode(double value)
{
    return (int16_t)round(value * 1000.0);
}

double bno085_can_bno085_data_accel_z_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool bno085_can_bno085_data_accel_z_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_accel_z_is_in_phys_range(double value)
{
    double scaled = round(value * 1000.0);
    return ((scaled >= -32768.0) && (scaled <= 32767.0));
}

int16_t bno085_can_bno085_data_gyro_x_encode(double value)
{
    return (int16_t)round(value * 1000.0);
}

double bno085_can_bno085_data_gyro_x_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool bno085_can_bno085_data_gyro_x_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_gyro_x_is_in_phys_range(double value)
{
    double scaled = round(value * 1000.0);
    return ((scaled >= -32768.0) && (scaled <= 32767.0));
}

int16_t bno085_can_bno085_data_gyro_y_encode(double value)
{
    return (int16_t)round(value * 1000.0);
}

double bno085_can_bno085_data_gyro_y_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool bno085_can_bno085_data_gyro_y_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_gyro_y_is_in_phys_range(double value)
{
    double scaled = round(value * 1000.0);
    return ((scaled >= -32768.0) && (scaled <= 32767.0));
}

int16_t bno085_can_bno085_data_gyro_z_encode(double value)
{
    return (int16_t)round(value * 1000.0);
}

double bno085_can_bno085_data_gyro_z_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool bno085_can_bno085_data_gyro_z_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

bool bno085_can_bno085_data_gyro_z_is_in_phys_range(double value)
{
    double scaled = round(value * 1000.0);
    return ((scaled >= -32768.0) && (scaled <= 32767.0));
}
