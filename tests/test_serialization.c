#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <ucdr/microcdr.h>
#include "generated/MotorStatus.h"

// Float comparison helper
static bool float_eq(float a, float b) {
    return fabsf(a - b) < 1e-5f;
}

static void test_motor_status_roundtrip(void) {
    printf("[TEST] Running test_motor_status_roundtrip...\n");

    // 1. Prepare original data
    robot_msgs_MotorStatus original = {
        .sequence = 123456,
        .velocity = 3.141592f,
        .current = -2.71828f,
        .position = {10.5f, -20.25f, 30.125f},
        .is_enabled = true
    };

    // 2. Serialize
    uint8_t buffer[256];
    memset(buffer, 0xFF, sizeof(buffer)); // Fill with sentinel
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, buffer, sizeof(buffer));

    bool ser_ok = robot_msgs_MotorStatus_serialize(&writer, &original);
    assert(ser_ok == true);

    size_t serialized_len = ucdr_buffer_length(&writer);
    printf("       Serialized length: %zu bytes\n", serialized_len);

    // 3. Verify CDR Header manually
    // CDR Little Endian header: 0x00, 0x01, 0x00, 0x00
    assert(buffer[0] == 0x00);
    assert(buffer[1] == 0x01);
    assert(buffer[2] == 0x00);
    assert(buffer[3] == 0x00);

    // 4. Deserialize
    robot_msgs_MotorStatus restored;
    memset(&restored, 0, sizeof(restored));
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, buffer, serialized_len);

    bool deser_ok = robot_msgs_MotorStatus_deserialize(&reader, &restored);
    assert(deser_ok == true);

    // 5. Assert equality
    assert(restored.sequence == original.sequence);
    assert(float_eq(restored.velocity, original.velocity));
    assert(float_eq(restored.current, original.current));
    assert(float_eq(restored.position[0], original.position[0]));
    assert(float_eq(restored.position[1], original.position[1]));
    assert(float_eq(restored.position[2], original.position[2]));
    assert(restored.is_enabled == original.is_enabled);

    printf("       test_motor_status_roundtrip: PASSED ✅\n");
}

static void test_buffer_overflow_protection(void) {
    printf("[TEST] Running test_buffer_overflow_protection...\n");

    robot_msgs_MotorStatus data = {
        .sequence = 1,
        .velocity = 1.0f,
        .current = 1.0f,
        .position = {0.0f, 0.0f, 0.0f},
        .is_enabled = false
    };

    // Buffer too small (only 4 bytes, cannot even fit full CDR payload)
    uint8_t tiny_buffer[4];
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, tiny_buffer, sizeof(tiny_buffer));

    bool ser_ok = robot_msgs_MotorStatus_serialize(&writer, &data);
    assert(ser_ok == false); // Must fail safely!

    printf("       test_buffer_overflow_protection: PASSED ✅\n");
}

static void test_keyexpr_macro(void) {
    printf("[TEST] Running test_keyexpr_macro...\n");

    const char *ke = robot_msgs_MotorStatus_KEYEXPR("0", "my_motor");
    assert(ke != NULL);
    assert(strstr(ke, "0/my_motor/") != NULL);
    assert(strstr(ke, "robot_msgs::msg::dds_::MotorStatus_") != NULL);
    assert(strstr(ke, "RIHS01_") != NULL);

    printf("       KeyExpr: %s\n", ke);
    printf("       test_keyexpr_macro: PASSED ✅\n");
}

#include "generated/std_msgs/UInt32.h"

static void test_std_msgs_uint32_roundtrip(void) {
    printf("[TEST] Running test_std_msgs_uint32_roundtrip...\n");

    std_msgs_UInt32 original = { .data = 4294967295U };
    uint8_t buffer[64];
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, buffer, sizeof(buffer));

    assert(std_msgs_UInt32_serialize(&writer, &original) == true);
    size_t len = ucdr_buffer_length(&writer);
    assert(len == 8); // 4 bytes CDR header + 4 bytes uint32

    std_msgs_UInt32 restored = {0};
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, buffer, len);
    assert(std_msgs_UInt32_deserialize(&reader, &restored) == true);
    assert(restored.data == original.data);

    printf("       test_std_msgs_uint32_roundtrip: PASSED ✅\n");
}

#include "bridge_core.h"
#include "generated/transport/BridgeStatus.h"

static void test_bridge_status_roundtrip(void) {
    printf("[TEST] Running test_bridge_status_roundtrip...\n");

    can_transport_msgs_BridgeStatus original = {
        .sequence        = 100,
        .uptime_ms       = 54321,
        .state           = BRIDGE_STATE_ACTIVE,
        .can_state       = BRIDGE_CAN_STATE_OK,
        .can_rx_count    = 1234,
        .can_tx_count    = 5678,
        .drop_count      = 0,
        .free_heap_bytes = 20480
    };

    uint8_t buffer[128];
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, buffer, sizeof(buffer));

    assert(can_transport_msgs_BridgeStatus_serialize(&writer, &original) == true);
    size_t len = ucdr_buffer_length(&writer);

    can_transport_msgs_BridgeStatus restored = {0};
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, buffer, len);
    assert(can_transport_msgs_BridgeStatus_deserialize(&reader, &restored) == true);

    assert(restored.sequence == original.sequence);
    assert(restored.uptime_ms == original.uptime_ms);
    assert(restored.state == original.state);
    assert(restored.can_state == original.can_state);
    assert(restored.can_rx_count == original.can_rx_count);
    assert(restored.can_tx_count == original.can_tx_count);
    assert(restored.drop_count == original.drop_count);
    assert(restored.free_heap_bytes == original.free_heap_bytes);

    printf("       test_bridge_status_roundtrip: PASSED ✅\n");
}

int main(void) {
    printf("========================================\n");
    printf("   Micro-CDR Serialization Unit Tests   \n");
    printf("========================================\n");

    test_motor_status_roundtrip();
    test_buffer_overflow_protection();
    test_keyexpr_macro();
    test_std_msgs_uint32_roundtrip();
    test_bridge_status_roundtrip();

    printf("========================================\n");
    printf("   ALL UNIT TESTS PASSED SUCCESSFULLY!  \n");
    printf("========================================\n");
    return 0;
}
