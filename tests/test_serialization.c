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

int main(void) {
    printf("========================================\n");
    printf("   Micro-CDR Serialization Unit Tests   \n");
    printf("========================================\n");

    test_motor_status_roundtrip();
    test_buffer_overflow_protection();
    test_keyexpr_macro();

    printf("========================================\n");
    printf("   ALL UNIT TESTS PASSED SUCCESSFULLY!  \n");
    printf("========================================\n");
    return 0;
}
