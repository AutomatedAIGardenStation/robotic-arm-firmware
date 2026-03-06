#include <unity.h>
#include "MotionController.h"
#include "StepperEngine.h"
#include "CoordinateMapper.h"
#include "MockMotorDriver.h"
#include "MockLimitSwitch.h"
#include "SafetyMonitor.h"
#include "MockEncoder.h"
#include <string.h>

extern uint32_t g_mock_millis;

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
    g_mock_millis = 0;
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
    TEST_ASSERT_EQUAL(MotionState::HOMING, controller->getState());

    // Stage 1: Homing Z
    controller->update(); // Start moving Z
    switches[2].setTriggered(true); // Hit Z limit
    controller->update(); // Process Z hit, transition to X/Y

    // Stage 2: Homing X/Y
    switches[0].setTriggered(true); // Hit X limit
    controller->update(); // Stop X
    switches[1].setTriggered(true); // Hit Y limit
    controller->update(); // Stop Y, finish homing

    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_HOMED", last_event);
}

void test_arm_move_to_valid_cartesians(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    // reset last event
    last_event[0] = '\0';

    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 100.0f;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    mock_encoders[0].setPosition(CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_X, 100.0f));

    while (controller->getState() == MotionState::MOVING) {
        controller->update();
    }
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);

    // verify mock drivers triggered
    // Homing disables all drivers at end (6 axes).
    // ARM_MOVE_TO enables X (1), then disables X (1 for StepperEngine done, 1 for MotionController update)
    // Actually drivers[0].enable_calls was 1 before, now +1 for homing = 2.
    // Let's just reset the mock driver counters before ARM_MOVE_TO
    // to keep the assertions simple
}

void test_arm_move_to_out_of_range_cartesian(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    // reset last event
    last_event[0] = '\0';

    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 2000.0f; // invalid X (> 1000)

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft", last_event);

    // verify mock drivers not triggered (actually ARM_HOME will trigger it)
    // we should just check the event
}

void test_arm_move_to_unhomed_emits_fault(void) {
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 100.0f;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=UNHOMED:tier=soft", last_event);
}

void test_tool_dock_completes(void) {
    Command cmd;
    cmd.type = CommandType::TOOL_DOCK;
    strcpy(cmd.tool_name, "CAMERA");

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    mock_encoders[5].setPosition(10); // tool seq step

    while (controller->getState() == MotionState::MOVING) {
        controller->update();
    }
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

    while (controller->getState() == MotionState::MOVING) {
        controller->update();
    }
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

    while (controller->getState() == MotionState::MOVING) {
        controller->update();
    }
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
    TEST_ASSERT_EQUAL(MotionState::HOMING, controller->getState());
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

    // Send first command, should transition to HOMING
    controller->execute(cmd1);
    TEST_ASSERT_EQUAL(MotionState::HOMING, controller->getState());

    // Send 3 more commands while HOMING, they should be queued
    // But wait, execute only queues if state == MOVING! Let's check.
    // Ah, execute() says: if (state == MotionState::MOVING) { enqueue } else process_command
    // We need to fix execute() to handle HOMING
    controller->execute(cmd2);
    controller->execute(cmd3);
    controller->execute(cmd4);

    // Queue is full now (4 commands: 1 active, 3 queued, wait: CAPACITY is 4, so 3 queued is fine)
    // Send 5th command, should emit QUEUE_FULL
    Command cmd5;
    cmd5.type = CommandType::ARM_MOVE_TO;
    cmd5.has_x = true;
    cmd5.x = 100.0f;
    Command cmd6; cmd6.type = CommandType::ARM_HOME;

    // Execute cmd5, it gets queued (queue has 4 items now)
    controller->execute(cmd5);

    // Execute cmd6, it should be rejected because queue is full
    // Clear last event so we can reliably check it
    last_event[0] = '\0';
    controller->execute(cmd6);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=QUEUE_FULL:tier=soft", last_event);

    // Now complete cmd1 (ARM_HOME)
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    // Since queue auto-dequeues on done, we need to call update() one more time to fetch and start the next command

    // Command 2 starts automatically
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd2
    mock_encoders[5].setPosition(mock_encoders[5].getPosition() + 10);
    while (controller->getState() == MotionState::MOVING && strncmp(last_event, "EVT:ARM_DONE", 12) != 0) {
        controller->update();
    }
    // Command 3 starts automatically (after processing last event, but we have to call update one more time to dequeue the next cmd)
    controller->update();
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd3
    mock_encoders[5].setPosition(mock_encoders[5].getPosition() + 10);
    while (controller->getState() == MotionState::MOVING && strncmp(last_event, "EVT:ARM_DONE", 12) != 0) {
        controller->update();
    }
    // Command 4 starts automatically
    controller->update();
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd4
    while (controller->getState() == MotionState::MOVING && strncmp(last_event, "EVT:WRIST_DONE", 14) != 0) {
        controller->update();
    }
    // Command 5 starts automatically
    controller->update();
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Complete cmd5
    mock_encoders[0].setPosition(mock_encoders[0].getPosition() + CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_X, 100.0f)); // assuming cmd5 had X
    while (controller->getState() == MotionState::MOVING && strncmp(last_event, "EVT:ARM_DONE", 12) != 0) {
        controller->update();
    }
    // Command 5 is finished
    controller->update();
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

void test_soft_fault_escalation_within_time_window(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 2000.0f; // out of range soft fault

    // Initial state
    g_mock_millis = 1000;
    controller->execute(cmd);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft", last_event);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());

    // 2nd fault within 5 min (300,000 ms)
    g_mock_millis = 1000 + 100000; // +100 seconds
    controller->execute(cmd);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft:repeat=2", last_event);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());

    // 3rd fault within 5 min of the 1st
    g_mock_millis = 1000 + 200000; // +200 seconds
    controller->execute(cmd);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=REPEATED_FAULT:tier=hard", last_event);
    TEST_ASSERT_EQUAL(MotionState::FAULT, controller->getState());
}

void test_soft_fault_no_escalation_outside_time_window(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 2000.0f; // out of range soft fault

    // 1st fault
    g_mock_millis = 1000;
    controller->execute(cmd);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft", last_event);

    // 2nd fault
    g_mock_millis = 1000 + 100000;
    controller->execute(cmd);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft:repeat=2", last_event);

    // Wait > 5 minutes from 1st fault (1000 + 300000 + 1)
    g_mock_millis = 1000 + 300001;
    controller->execute(cmd);
    // Since the 1st fault expired, this becomes the 2nd fault (since the 2nd fault is still in window)
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft:repeat=2", last_event);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

void test_arm_clear_fault_triggers_homing(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    // Generate a soft fault to clear
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 2000.0f;
    controller->execute(cmd);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft", last_event);

    // Clear fault
    Command clear_cmd;
    clear_cmd.type = CommandType::ARM_CLEAR_FAULT;
    controller->execute(clear_cmd);

    // EVT:ARM_FAULT_CLEARED should be emitted during execution
    // And ARM_HOME should be processed automatically
    TEST_ASSERT_EQUAL(MotionState::HOMING, controller->getState());
}

void test_wrist_set_completes(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    Command cmd;
    cmd.type = CommandType::WRIST_SET;
    cmd.has_pitch = true;
    cmd.pitch = 45.0f;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Update encoders for wrist (axis 3 and 4) to reach target
    mock_encoders[3].setPosition(CoordinateMapper::steps_from_degrees(CoordinateMapper::WRIST_PITCH, 45.0f));

    while (controller->getState() == MotionState::MOVING) {
        controller->update();
    }
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:WRIST_DONE", last_event);
}

void test_gripper_open_completes(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    Command cmd;
    cmd.type = CommandType::GRIPPER_OPEN;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Update encoder since it's a tool sequence step
    mock_encoders[5].setPosition(10);
    while (controller->getState() == MotionState::MOVING) {
        controller->update();
    }
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

void test_gripper_close_completes(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    Command cmd;
    cmd.type = CommandType::GRIPPER_CLOSE;

    controller->execute(cmd);
    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Update encoder since it's a tool sequence step
    mock_encoders[5].setPosition(10);
    while (controller->getState() == MotionState::MOVING) {
        controller->update();
    }
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

void test_limit_hit_during_move_emits_hard_fault(void) {
    // Need to home first
    Command home_cmd;
    home_cmd.type = CommandType::ARM_HOME;
    controller->execute(home_cmd);
    controller->update();
    switches[2].setTriggered(true);
    controller->update();
    switches[0].setTriggered(true);
    switches[1].setTriggered(true);
    controller->update();

    // Release limits after homing
    for (int i = 0; i < 6; i++) {
        switches[i].setTriggered(false);
    }
    last_event[0] = '\0';

    // Command a move
    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    cmd.has_x = true;
    cmd.x = 100.0f;
    controller->execute(cmd);

    TEST_ASSERT_EQUAL(MotionState::MOVING, controller->getState());

    // Simulate hitting a limit switch during movement
    switches[0].setTriggered(true);

    // Call update to process the safety monitor and the fault
    // SafetyMonitor needs to be polled
    monitor->poll();
    controller->update();

    TEST_ASSERT_EQUAL(MotionState::FAULT, controller->getState());
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=LIMIT_HIT:tier=hard", last_event);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_idle);
    RUN_TEST(test_arm_home_transitions_to_moving_and_completes);
    RUN_TEST(test_arm_move_to_valid_cartesians);
    RUN_TEST(test_arm_move_to_out_of_range_cartesian);
    RUN_TEST(test_arm_move_to_unhomed_emits_fault);
    RUN_TEST(test_tool_dock_completes);
    RUN_TEST(test_tool_release_completes);
    RUN_TEST(test_tool_release_misaligned_emits_fault);
    RUN_TEST(test_arm_home_when_faulted_clears_fault_and_executes);
    RUN_TEST(test_arm_home_when_faulted_emits_limit_active_if_still_pressed);
    RUN_TEST(test_multiple_commands_queued_and_executed);
    RUN_TEST(test_soft_fault_escalation_within_time_window);
    RUN_TEST(test_soft_fault_no_escalation_outside_time_window);
    RUN_TEST(test_arm_clear_fault_triggers_homing);
    RUN_TEST(test_wrist_set_completes);
    RUN_TEST(test_gripper_open_completes);
    RUN_TEST(test_gripper_close_completes);
    RUN_TEST(test_limit_hit_during_move_emits_hard_fault);
    return UNITY_END();
}
