#include <unity.h>
#include "SafetyMonitor.h"
#include "MockLimitSwitch.h"
#include "MockMotorDriver.h"
#include "MotionController.h"
#include <string.h>

MockLimitSwitch switches[6];
ILimitSwitch* switch_ptrs[6];
MockMotorDriver drivers[6];
IMotorDriver* driver_ptrs[6];
SafetyMonitor* monitor;
MotionController* controller;

char last_event[128];
void protocol_emit_event(const char* event) {
    strncpy(last_event, event, sizeof(last_event) - 1);
    last_event[sizeof(last_event) - 1] = '\0';
}

void setUp(void) {
    for (int i = 0; i < 6; i++) {
        switches[i].setTriggered(false);
        switch_ptrs[i] = &switches[i];

        drivers[i].reset();
        driver_ptrs[i] = &drivers[i];
    }
    monitor = new SafetyMonitor(switch_ptrs, driver_ptrs);
    controller = new MotionController(driver_ptrs, monitor);
    last_event[0] = '\0';
}

void tearDown(void) {
    delete monitor;
    delete controller;
}

void test_no_fault_when_switches_open(void) {
    monitor->poll();
    TEST_ASSERT_FALSE(monitor->isFaulted());
}

void test_fault_triggers_when_switch_closed_and_brakes_all(void) {
    switches[2].setTriggered(true);
    monitor->poll();

    TEST_ASSERT_TRUE(monitor->isFaulted());

    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL(1, drivers[i].disable_calls);
    }
}

void test_clear_fault_fails_if_still_triggered(void) {
    switches[1].setTriggered(true);
    monitor->poll();
    TEST_ASSERT_TRUE(monitor->isFaulted());

    bool cleared = monitor->clearFault();
    TEST_ASSERT_FALSE(cleared);
    TEST_ASSERT_TRUE(monitor->isFaulted());
}

void test_clear_fault_succeeds_when_released(void) {
    switches[1].setTriggered(true);
    monitor->poll();
    TEST_ASSERT_TRUE(monitor->isFaulted());

    switches[1].setTriggered(false);
    bool cleared = monitor->clearFault();
    TEST_ASSERT_TRUE(cleared);
    TEST_ASSERT_FALSE(monitor->isFaulted());
}

void test_motion_controller_rejects_moves_when_faulted_with_limit_hit(void) {
    switches[0].setTriggered(true);
    monitor->poll();
    TEST_ASSERT_TRUE(monitor->isFaulted());

    Command cmd;
    cmd.type = CommandType::ARM_MOVE_TO;
    controller->execute(cmd);

    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=LIMIT_HIT:tier=hard", last_event);
    TEST_ASSERT_EQUAL(MotionState::IDLE, controller->getState());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_no_fault_when_switches_open);
    RUN_TEST(test_fault_triggers_when_switch_closed_and_brakes_all);
    RUN_TEST(test_clear_fault_fails_if_still_triggered);
    RUN_TEST(test_clear_fault_succeeds_when_released);
    RUN_TEST(test_motion_controller_rejects_moves_when_faulted_with_limit_hit);
    return UNITY_END();
}
