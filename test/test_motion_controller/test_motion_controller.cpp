#include <unity.h>
#include "MotionController.h"
#include "MockMotorDriver.h"
#include "MockLimitSwitch.h"
#include "SafetyMonitor.h"
#include "MockEncoder.h"
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

void test_initial_state_is_idle(void) {
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

void test_arm_home_transitions_to_moving_and_completes(void) {
    Command cmd;
    cmd.type = CommandType::ARM_HOME;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Position doesn't matter for ARM_HOME during execution, but it resets at the end

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_HOMED", last_event);
}

void test_arm_move_to_valid_cartesians(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 100.0f;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    mock_encoders[0].setPosition(100);

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);

    // verify mock drivers triggered
    TEST_ASSERT_EQUAL(1, drivers[0].enable_calls);
    TEST_ASSERT_EQUAL(1, drivers[0].disable_calls);
}

void test_arm_move_to_out_of_range_cartesian(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 2000.0f; // invalid X (> 1000)

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft", last_event);

    // verify mock drivers not triggered
    TEST_ASSERT_EQUAL(0, drivers[0].enable_calls);
}

void test_tool_dock_completes(void) {
    Command cmd;
    cmd.type = CommandType::TOOL_DOCK;
    strcpy(cmd.tool_name, "CAMERA");

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    mock_encoders[5].setPosition(10); // tool seq step

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:TOOL_DOCKED:tool=CAMERA", last_event);
}

void test_tool_release_completes(void) {
    Command cmd;
    cmd.type = CommandType::TOOL_RELEASE;
    strcpy(cmd.tool_name, "CAMERA");

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    mock_encoders[5].setPosition(10); // tool seq step

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:TOOL_RELEASED:tool=CAMERA", last_event);
}

void test_tool_release_misaligned_emits_fault(void) {
    Command cmd;
    cmd.type = CommandType::TOOL_RELEASE;
    strcpy(cmd.tool_name, "MISALIGNED");

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    mock_encoders[5].setPosition(10); // tool seq step

    controller->update();
    TEST_ASSERT_EQUAL(MotionState::FAULT, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:TOOL_FAULT:code=ALIGNMENT_ERROR", last_event);
}

void test_arm_home_when_faulted_clears_fault_and_executes(void) {
    switches[0].setTriggered(true);
    monitor->poll(); // fault
    TEST_ASSERT_TRUE(monitor->isFaulted());

    switches[0].setTriggered(false); // release switch

    Command cmd;
    cmd.type = CommandType::ARM_HOME;
    controller->execute(cmd);

    TEST_ASSERT_FALSE(monitor->isFaulted());
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());
}

void test_arm_home_when_faulted_emits_limit_active_if_still_pressed(void) {
    switches[0].setTriggered(true);
    monitor->poll(); // fault
    TEST_ASSERT_TRUE(monitor->isFaulted());

    // do NOT release switch

    Command cmd;
    cmd.type = CommandType::ARM_HOME;
    controller->execute(cmd);

    TEST_ASSERT_TRUE(monitor->isFaulted());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=LIMIT_ACTIVE:tier=hard", last_event);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

void test_multiple_commands_queued_and_executed(void) {
    Command cmd1; cmd1.type = CommandType::ARM_HOME;
    Command cmd2; cmd2.type = CommandType::GRIPPER_OPEN;
    Command cmd3; cmd3.type = CommandType::GRIPPER_CLOSE;
    Command cmd4; cmd4.type = CommandType::WRIST_SET;

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
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=QUEUE_FULL:tier=soft", last_event);

    // Now complete cmd1
    controller->update(); // Completes ARM_HOME, dequeues GRIPPER_OPEN, state is MOVING again
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());
    // EVT:ARM_DONE should be emitted, but wait, last_event is overwritten during update if process_command calls anything? No, process_command doesn't emit anything on success. But update() emitted EVT:ARM_DONE first, so last_event is EVT:ARM_DONE.
    // However, wait, in process_command, mock driver enable is called.

    // Complete cmd2
    controller->update(); // Completes GRIPPER_OPEN, dequeues GRIPPER_CLOSE
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd3
    controller->update(); // Completes GRIPPER_CLOSE, dequeues WRIST_SET
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd4
    controller->update(); // Completes WRIST_SET, dequeues ARM_MOVE_TO
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd5
    controller->update(); // Completes ARM_MOVE_TO, queue empty
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_idle);
    RUN_TEST(test_arm_home_transitions_to_moving_and_completes);
    RUN_TEST(test_arm_move_to_valid_cartesians);
    RUN_TEST(test_arm_move_to_out_of_range_cartesian);
    RUN_TEST(test_tool_dock_completes);
    RUN_TEST(test_tool_release_completes);
    RUN_TEST(test_tool_release_misaligned_emits_fault);
    RUN_TEST(test_arm_home_when_faulted_clears_fault_and_executes);
    RUN_TEST(test_arm_home_when_faulted_emits_limit_active_if_still_pressed);
    RUN_TEST(test_multiple_commands_queued_and_executed);
    return UNITY_END();
}
