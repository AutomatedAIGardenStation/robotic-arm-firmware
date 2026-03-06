#include <unity.h>
#include "../../lib/hal/boards/ArduinoLimitSwitch.h"

// Mock Arduino functions for native testing
int mock_pins[100];

void pinMode(int pin, int mode) {
    // Just a stub
}

int digitalRead(int pin) {
    if (pin >= 0 && pin < 100) {
        return mock_pins[pin];
    }
    return 0; // default return
}

void setUp(void) {
    for (int i = 0; i < 100; i++) {
        mock_pins[i] = 1; // Assuming normally closed / pulled up to HIGH
    }
}

void tearDown(void) {
}

void test_initialization() {
    ArduinoLimitSwitch limit_switch(2);
    // Pin is initialized, checking behavior instead
    TEST_ASSERT_FALSE(limit_switch.isTriggered());
}

void test_is_triggered() {
    ArduinoLimitSwitch limit_switch(2);

    // Default HIGH -> not triggered
    mock_pins[2] = 1;
    TEST_ASSERT_FALSE(limit_switch.isTriggered());

    // Pull LOW -> triggered
    mock_pins[2] = 0;
    TEST_ASSERT_TRUE(limit_switch.isTriggered());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_is_triggered);
    return UNITY_END();
}
