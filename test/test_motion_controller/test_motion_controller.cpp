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

void test_multiple_commands_queued_and_executed(void) {
    Command cmd1; cmd1.type = CommandType::ARM_HOME;
    Command cmd2; cmd2.type = CommandType::GRIPPER_OPEN;
    Command cmd3; cmd3.type = CommandType::GRIPPER_CLOSE;
    Command cmd4; cmd4.type = CommandType::H1;

    // Send first command, should transition to MOVING
    controller->execute(cmd1);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Send 3 more commands while MOVING, they should be queued
    controller->execute(cmd2);
    controller->execute(cmd3);
    controller->execute(cmd4);

    // Queue is full now (4 commands: 1 active, 3 queued, wait: CAPACITY is 4, so 3 queued is fine)
    // Send 5th command, should emit QUEUE_FULL
    Command cmd5; cmd5.type = CommandType::ARM_MOVE_TO;
    Command cmd6; cmd6.type = CommandType::ARM_HOME;

    // Execute cmd5, it gets queued (queue has 4 items now)
    controller->execute(cmd5);

    // Execute cmd6, it should be rejected because queue is full
    controller->execute(cmd6);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=QUEUE_FULL", last_event);

    // Now complete cmd1
    controller->update(); // Completes ARM_HOME, dequeues GRIPPER_OPEN, state is MOVING again
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());
    // EVT:ARM_DONE should be emitted, but wait, last_event is overwritten during update if process_command calls anything? No, process_command doesn't emit anything on success. But update() emitted EVT:ARM_DONE first, so last_event is EVT:ARM_DONE.
    // However, wait, in process_command, mock driver enable is called.

    // Complete cmd2
    controller->update(); // Completes GRIPPER_OPEN, dequeues GRIPPER_CLOSE
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd3
    controller->update(); // Completes GRIPPER_CLOSE, dequeues H1
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd4
    controller->update(); // Completes H1, dequeues ARM_MOVE_TO
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd5
    controller->update(); // Completes ARM_MOVE_TO, queue empty
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_idle);
    RUN_TEST(test_arm_home_transitions_to_moving_and_completes);
    RUN_TEST(test_arm_move_to_valid_angles);
    RUN_TEST(test_arm_move_to_out_of_range_angle);
    RUN_TEST(test_gripper_open_completes);
    RUN_TEST(test_gripper_close_completes);
    RUN_TEST(test_multiple_commands_queued_and_executed);
    return UNITY_END();
}
