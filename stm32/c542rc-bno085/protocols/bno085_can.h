/**
 * @file bno085_can.h
 *
 * @brief BNO085 CAN-FD 18-byte IMU protocol definitions (20-byte DLC frame).
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

#ifndef BNO085_CAN_H
#define BNO085_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef EINVAL
#    define EINVAL 22
#endif

/* Frame ids. */
#define BNO085_CAN_BNO085_DATA_FRAME_ID (0x500u)

/* Frame lengths in bytes (18-byte payload, 20-byte CAN FD DLC). */
#define BNO085_CAN_BNO085_DATA_PAYLOAD_LENGTH (18u)
#define BNO085_CAN_BNO085_DATA_LENGTH (20u)

/* Extended or standard frame types. */
#define BNO085_CAN_BNO085_DATA_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */
#define BNO085_CAN_BNO085_DATA_CYCLE_TIME_MS (5u)

/* Status choices. */
#define BNO085_CAN_STATUS_INIT (0u)
#define BNO085_CAN_STATUS_OK (1u)
#define BNO085_CAN_STATUS_WARN (2u)
#define BNO085_CAN_STATUS_ERROR (3u)

/* Frame Names. */
#define BNO085_CAN_BNO085_DATA_NAME "BNO085_Data"

/* Signal Names. */
#define BNO085_CAN_BNO085_DATA_TIMESTAMP_MS_NAME "Timestamp_Ms"
#define BNO085_CAN_BNO085_DATA_SEQ_NAME "Seq"
#define BNO085_CAN_BNO085_DATA_STATUS_NAME "Status"
#define BNO085_CAN_BNO085_DATA_ACCEL_X_NAME "Accel_X"
#define BNO085_CAN_BNO085_DATA_ACCEL_Y_NAME "Accel_Y"
#define BNO085_CAN_BNO085_DATA_ACCEL_Z_NAME "Accel_Z"
#define BNO085_CAN_BNO085_DATA_GYRO_X_NAME "Gyro_X"
#define BNO085_CAN_BNO085_DATA_GYRO_Y_NAME "Gyro_Y"
#define BNO085_CAN_BNO085_DATA_GYRO_Z_NAME "Gyro_Z"

/**
 * Signals in message BNO085_Data (CAN ID 0x500, 18-byte payload, 20-byte CAN FD DLC).
 *
 * Offset | Field                  | Type / Size
 * -------+------------------------+---------------------
 * 0..4   | timestamp_ms           | uint32_t (4B)
 * 4..5   | seq                    | uint8_t  (1B)
 * 5..6   | status                 | uint8_t  (1B)
 * 6..12  | accel_x, y, z          | int16_t x 3 (6B)
 * 12..18 | gyro_x, y, z           | int16_t x 3 (6B)
 *
 * All signal values are as on the CAN bus (little-endian).
 */
struct bno085_can_bno085_data_t {
    /**
     * Boot-relative timestamp in milliseconds.
     *
     * Range: 0..4294967295 (0..4294967295 ms)
     * Scale: 1
     * Offset: 0
     */
    uint32_t timestamp_ms;

    /**
     * Rolling frame sequence number.
     *
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t seq;

    /**
     * Firmware status (0=INIT, 1=OK, 2=WARN, 3=ERROR).
     *
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t status;

    /**
     * X-axis acceleration (0.001 m/s^2).
     *
     * Range: -32768..32767 (-32.768..32.767 m/s^2)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t accel_x;

    /**
     * Y-axis acceleration (0.001 m/s^2).
     *
     * Range: -32768..32767 (-32.768..32.767 m/s^2)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t accel_y;

    /**
     * Z-axis acceleration (0.001 m/s^2).
     *
     * Range: -32768..32767 (-32.768..32.767 m/s^2)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t accel_z;

    /**
     * X-axis angular velocity (0.001 rad/s).
     *
     * Range: -32768..32767 (-32.768..32.767 rad/s)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t gyro_x;

    /**
     * Y-axis angular velocity (0.001 rad/s).
     *
     * Range: -32768..32767 (-32.768..32.767 rad/s)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t gyro_y;

    /**
     * Z-axis angular velocity (0.001 rad/s).
     *
     * Range: -32768..32767 (-32.768..32.767 rad/s)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t gyro_z;
};

/**
 * Pack message BNO085_Data.
 *
 * Packs the 18-byte payload into dst_p. If size >= 20, bytes 18..19 are zero-padded
 * for the 20-byte CAN FD DLC slot.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p (must be >= 18).
 *
 * @return 18 on success, or negative error code (-EINVAL).
 */
int bno085_can_bno085_data_pack(
    uint8_t *dst_p,
    const struct bno085_can_bno085_data_t *src_p,
    size_t size);

/**
 * Unpack message BNO085_Data.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p (must be >= 18).
 *
 * @return zero(0) or negative error code (-EINVAL).
 */
int bno085_can_bno085_data_unpack(
    struct bno085_can_bno085_data_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from BNO085_Data.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int bno085_can_bno085_data_init(struct bno085_can_bno085_data_t *msg_p);

uint32_t bno085_can_bno085_data_timestamp_ms_encode(double value);
double bno085_can_bno085_data_timestamp_ms_decode(uint32_t value);
bool bno085_can_bno085_data_timestamp_ms_is_in_range(uint32_t value);
bool bno085_can_bno085_data_timestamp_ms_is_in_phys_range(double value);

uint8_t bno085_can_bno085_data_seq_encode(double value);
double bno085_can_bno085_data_seq_decode(uint8_t value);
bool bno085_can_bno085_data_seq_is_in_range(uint8_t value);
bool bno085_can_bno085_data_seq_is_in_phys_range(double value);

uint8_t bno085_can_bno085_data_status_encode(double value);
double bno085_can_bno085_data_status_decode(uint8_t value);
bool bno085_can_bno085_data_status_is_in_range(uint8_t value);
bool bno085_can_bno085_data_status_is_in_phys_range(double value);

int16_t bno085_can_bno085_data_accel_x_encode(double value);
double bno085_can_bno085_data_accel_x_decode(int16_t value);
bool bno085_can_bno085_data_accel_x_is_in_range(int16_t value);
bool bno085_can_bno085_data_accel_x_is_in_phys_range(double value);

int16_t bno085_can_bno085_data_accel_y_encode(double value);
double bno085_can_bno085_data_accel_y_decode(int16_t value);
bool bno085_can_bno085_data_accel_y_is_in_range(int16_t value);
bool bno085_can_bno085_data_accel_y_is_in_phys_range(double value);

int16_t bno085_can_bno085_data_accel_z_encode(double value);
double bno085_can_bno085_data_accel_z_decode(int16_t value);
bool bno085_can_bno085_data_accel_z_is_in_range(int16_t value);
bool bno085_can_bno085_data_accel_z_is_in_phys_range(double value);

int16_t bno085_can_bno085_data_gyro_x_encode(double value);
double bno085_can_bno085_data_gyro_x_decode(int16_t value);
bool bno085_can_bno085_data_gyro_x_is_in_range(int16_t value);
bool bno085_can_bno085_data_gyro_x_is_in_phys_range(double value);

int16_t bno085_can_bno085_data_gyro_y_encode(double value);
double bno085_can_bno085_data_gyro_y_decode(int16_t value);
bool bno085_can_bno085_data_gyro_y_is_in_range(int16_t value);
bool bno085_can_bno085_data_gyro_y_is_in_phys_range(double value);

int16_t bno085_can_bno085_data_gyro_z_encode(double value);
double bno085_can_bno085_data_gyro_z_decode(int16_t value);
bool bno085_can_bno085_data_gyro_z_is_in_range(int16_t value);
bool bno085_can_bno085_data_gyro_z_is_in_phys_range(double value);

#ifdef __cplusplus
}
#endif

#endif
