#include <unity.h>
#include "../../lib/hal/boards/ArduinoMotorDriver.h"

// Mock Arduino functions for native testing
int mock_pins[100];

void pinMode(int pin, int mode) {
    // Just a stub
}

void digitalWrite(int pin, int val) {
    if (pin >= 0 && pin < 100) {
        mock_pins[pin] = val;
    }
}

void delayMicroseconds(unsigned int us) {
    // Stub
}

void setUp(void) {
    for (int i = 0; i < 100; i++) {
        mock_pins[i] = 0; // Reset all mock pins
    }
}

void tearDown(void) {
}

void test_initialization() {
    // Step, Dir, Enable pins
    ArduinoMotorDriver driver(2, 3, 4);

    // Enable pin is usually active low, constructor calls disable() which sets it HIGH
    TEST_ASSERT_EQUAL(1, mock_pins[4]);
}

void test_enable_disable() {
    ArduinoMotorDriver driver(2, 3, 4);

    driver.enable();
    TEST_ASSERT_EQUAL(0, mock_pins[4]); // LOW = enable

    driver.disable();
    TEST_ASSERT_EQUAL(1, mock_pins[4]); // HIGH = disable
}

void test_step() {
    ArduinoMotorDriver driver(2, 3, 4);

    // Check direction true (forward)
    driver.step(true, 1);
    TEST_ASSERT_EQUAL(1, mock_pins[3]); // DIR pin HIGH
    // We can't easily check the step pin toggling since it happens fast in sequence,
    // but at the end of the pulse it should be LOW
    TEST_ASSERT_EQUAL(0, mock_pins[2]);

    // Check direction false (backward)
    driver.step(false, 1);
    TEST_ASSERT_EQUAL(0, mock_pins[3]); // DIR pin LOW
    TEST_ASSERT_EQUAL(0, mock_pins[2]);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_enable_disable);
    RUN_TEST(test_step);
    return UNITY_END();
}
