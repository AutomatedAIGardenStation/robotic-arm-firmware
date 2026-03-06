#include <unity.h>
#include "MotionController.h"
#include "MockMotorDriver.h"
#include "MockLimitSwitch.h"
#include "SafetyMonitor.h"
#include "MockEncoder.h"
#include "EncoderReader.h"
#include <string.h>

MockMotorDriver drivers[6];
IMotorDriver* driver_ptrs[6];
MockLimitSwitch switches[6];
ILimitSwitch* switch_ptrs[6];
MockEncoder mock_encoders[6];
IEncoder* encoder_ptrs[6];
EncoderReader* reader;

SafetyMonitor* monitor;
MotionController* controller;

char last_event[128];
void protocol_emit_event(const char* event) {
    strncpy(last_event, event, sizeof(last_event) - 1);
    last_event[sizeof(last_event) - 1] = '\0';
}

void setUp(void) {
    for (int i = 0; i < 6; i++) {
        drivers[i].reset();
        driver_ptrs[i] = &drivers[i];

        switches[i].setTriggered(false);
        switch_ptrs[i] = &switches[i];

        mock_encoders[i].reset();
        encoder_ptrs[i] = &mock_encoders[i];
    }
    reader = new EncoderReader(encoder_ptrs);
    monitor = new SafetyMonitor(switch_ptrs, driver_ptrs);
    controller = new MotionController(driver_ptrs, monitor, reader);
    last_event[0] = '\0';
}

void tearDown(void) {
    delete monitor;
    delete controller;
    delete reader;
}

void test_get_position_returns_mocked_value(void) {
    mock_encoders[0].setPosition(150);
    mock_encoders[5].setPosition(-50);

    TEST_ASSERT_EQUAL(150, reader->getPosition(1));
    TEST_ASSERT_EQUAL(-50, reader->getPosition(6));
    TEST_ASSERT_EQUAL(0, reader->getPosition(2));
}

void test_reset_all_sets_mocks_to_zero(void) {
    for (int i = 0; i < 6; i++) {
        mock_encoders[i].setPosition(100 * (i + 1));
    }

    reader->resetAll();

    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL(0, reader->getPosition(i + 1));
    }
}

void test_motion_controller_emits_arm_done_when_matched(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 90.0f;

    controller->execute(cmd);

    // In MockMotorDriver, we step by 100 for valid targets in ARM_MOVE_TO.
    mock_encoders[0].setPosition(100);

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

void test_motion_controller_emits_fault_when_deviated(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 90.0f;

    controller->execute(cmd);

    // Position tolerance is 5. We deviate by 6.
    mock_encoders[0].setPosition(106);

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::FAULT, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=POSITION_MISMATCH", last_event);
}

void test_motion_controller_emits_done_within_tolerance(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 90.0f;

    controller->execute(cmd);

    // Position tolerance is 5. We deviate by 5.
    mock_encoders[0].setPosition(105);

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_get_position_returns_mocked_value);
    RUN_TEST(test_reset_all_sets_mocks_to_zero);
    RUN_TEST(test_motion_controller_emits_arm_done_when_matched);
    RUN_TEST(test_motion_controller_emits_fault_when_deviated);
    RUN_TEST(test_motion_controller_emits_done_within_tolerance);
    return UNITY_END();
}
