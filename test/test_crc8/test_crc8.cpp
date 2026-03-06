#include <unity.h>
#include "../../lib/protocol/Crc8.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

void test_crc8_computation(void) {
    const char* data = "0002:ARM_MOVE_TO:x=10:y=20:z=30:";
    uint8_t crc = crc8_compute(data, strlen(data));
    TEST_ASSERT_EQUAL_HEX8(0x70, crc);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_computation);
    return UNITY_END();
}
