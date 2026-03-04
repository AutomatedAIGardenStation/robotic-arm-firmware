#include <unity.h>
#include "MotionController.h"
#include "MockMotorDriver.h"
#include <string.h>

MockMotorDriver drivers[6];
IMotorDriver* driver_ptrs[6];
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
    }
    controller = new MotionController(driver_ptrs);
    last_event[0] = '\0';
}

void tearDown(void) {
    delete controller;
}

void test_initial_state_is_idle(void) {
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

void test_arm_home_transitions_to_moving_and_completes(void) {
    Command cmd;
    cmd.type = CommandType::ARM_HOME;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

void test_arm_move_to_valid_angles(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_angle[0] = true;
    cmd.angles[0] = 90.0f; // valid yaw

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);

    // verify mock drivers triggered
    TEST_ASSERT_EQUAL(1, drivers[0].enable_calls);
    TEST_ASSERT_EQUAL(1, drivers[0].disable_calls);
}

void test_arm_move_to_out_of_range_angle(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_angle[0] = true;
    cmd.angles[0] = 200.0f; // invalid yaw (> 180)

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE", last_event);

    // verify mock drivers not triggered
    TEST_ASSERT_EQUAL(0, drivers[0].enable_calls);
}

void test_gripper_open_completes(void) {
    Command cmd;
    cmd.type = CommandType::GRIPPER_OPEN;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

void test_gripper_close_completes(void) {
    Command cmd;
    cmd.type = CommandType::GRIPPER_CLOSE;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_idle);
    RUN_TEST(test_arm_home_transitions_to_moving_and_completes);
    RUN_TEST(test_arm_move_to_valid_angles);
    RUN_TEST(test_arm_move_to_out_of_range_angle);
    RUN_TEST(test_gripper_open_completes);
    RUN_TEST(test_gripper_close_completes);
    return UNITY_END();
}
