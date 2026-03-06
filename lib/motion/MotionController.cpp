#include "MotionController.h"
#include "CoordinateMapper.h"
#include <string.h>

// Declaration to satisfy compiler without including protocol.h
extern void protocol_emit_event(const char* event);

#include "../../config/pins.h"
#include <stdlib.h>

MotionController::MotionController(IMotorDriver* drvs[6], SafetyMonitor* safety, EncoderReader* encoder_reader) : engine(drvs) {
    state = MotionState::IDLE;
    current_command_type = CommandType::NONE;
    homing_stage = 0;
    is_homed = false;
    safety_monitor = safety;
    this->encoder_reader = encoder_reader;
    soft_fault_count = 0;
    for (int i = 0; i < 3; i++) soft_fault_timestamps[i] = 0;

    for (int i = 0; i < 6; i++) {
        if (drvs) {
            drivers[i] = drvs[i];
        } else {
            drivers[i] = nullptr;
        }
        current_steps[i] = 0;
        expected_steps[i] = 0;
        active_joints[i] = false;
    }
}

MotionState MotionController::getState() const {
    return state;
}

bool MotionController::hasHardFault() const {
    if (state == MotionState::FAULT) {
        return true;
    }
    if (safety_monitor && safety_monitor->isFaulted()) {
        return true;
    }
    return false;
}

void MotionController::execute(const Command& cmd) {
    if (cmd.type == CommandType::ARM_CLEAR_FAULT) {
        // Soft fault cleared only if there's no hard fault
        // SafetyMonitor limits and REPEATED_FAULT are hard faults.
        if (hasHardFault()) {
            protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_ACTIVE:tier=hard");
            return;
        }

        soft_fault_count = 0;
        for (int i = 0; i < 3; i++) soft_fault_timestamps[i] = 0;

        protocol_emit_event("EVT:ARM_FAULT_CLEARED");

        Command home_cmd;
        home_cmd.type = CommandType::ARM_HOME;
        process_command(home_cmd);
        return;
    }

    if (safety_monitor && safety_monitor->isFaulted()) {
        if (cmd.type == CommandType::ARM_HOME) {
            if (!safety_monitor->clearFault()) {
                protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_ACTIVE:tier=hard");
                return;
            }
        } else {
            protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_HIT:tier=hard");
            return;
        }
    }

    if (state == MotionState::FAULT) {
        return;
    }

    if (state == MotionState::MOVING || state == MotionState::HOMING) {
        if (!queue.enqueue(cmd)) {
            emitSoftFault("QUEUE_FULL");
        }
        return;
    }

    process_command(cmd);
}

void MotionController::process_command(const Command& cmd) {
    current_cmd = cmd;

    if (cmd.type == CommandType::ARM_MOVE_TO) {
        if (!is_homed) {
            emitSoftFault("UNHOMED");
            return;
        }
        if (cmd.has_x && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, cmd.x)) {
            emitSoftFault("OUT_OF_RANGE");
            return;
        }
        if (cmd.has_y && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Y, cmd.y)) {
            emitSoftFault("OUT_OF_RANGE");
            return;
        }
        if (cmd.has_z && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Z, cmd.z)) {
            emitSoftFault("OUT_OF_RANGE");
            return;
        }
    } else if (cmd.type == CommandType::WRIST_SET) {
        if (cmd.has_pitch && !CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, cmd.pitch)) {
            emitSoftFault("OUT_OF_RANGE");
            return;
        }
        if (cmd.has_roll && !CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, cmd.roll)) {
            emitSoftFault("OUT_OF_RANGE");
            return;
        }
    }

    state = MotionState::MOVING;
    current_command_type = cmd.type;

    for (int i = 0; i < 6; i++) {
        active_joints[i] = false;
    }

    for (int i = 0; i < 6; i++) {
        expected_steps[i] = current_steps[i];
    }

    if (cmd.type == CommandType::ARM_MOVE_TO) {
        if (cmd.has_x) { expected_steps[0] = CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_X, cmd.x); active_joints[0] = true; }
        if (cmd.has_y) { expected_steps[1] = CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_Y, cmd.y); active_joints[1] = true; }
        if (cmd.has_z) { expected_steps[2] = CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_Z, cmd.z); active_joints[2] = true; }
    } else if (cmd.type == CommandType::WRIST_SET) {
        if (cmd.has_pitch) { expected_steps[3] = CoordinateMapper::steps_from_degrees(CoordinateMapper::WRIST_PITCH, cmd.pitch); active_joints[3] = true; }
        if (cmd.has_roll) { expected_steps[4] = CoordinateMapper::steps_from_degrees(CoordinateMapper::WRIST_ROLL, cmd.roll); active_joints[4] = true; }
    } else if (cmd.type == CommandType::TOOL_DOCK || cmd.type == CommandType::TOOL_RELEASE) {
        // Tool dock/release logic, just advance dummy joint
        expected_steps[5] += 10;
        active_joints[5] = true;
    } else if (cmd.type == CommandType::ARM_HOME) {
        state = MotionState::HOMING;
        homing_stage = 1; // Z priority
        if (safety_monitor) {
            safety_monitor->setHomingMode(true);
        }
        protocol_emit_event("EVT:ARM_HOMING:axis=Z");

        // Move Z negative by a large amount
        expected_steps[2] -= 100000;
        active_joints[2] = true;
        engine.moveTo(current_steps, expected_steps);
        return;
    } else if (cmd.type == CommandType::GRIPPER_OPEN || cmd.type == CommandType::GRIPPER_CLOSE) {
        // Advance dummy joint
        expected_steps[5] += 10;
        active_joints[5] = true;
    }

    engine.moveTo(current_steps, expected_steps);
}

void MotionController::update() {
    if (state == MotionState::HOMING) {
        engine.update();

        if (homing_stage == 1) {
            if (safety_monitor && safety_monitor->isLimitTriggered(2)) { // Z limit hit
                engine.stopAxis(2);
                current_steps[2] = 0;
                expected_steps[2] = 0;
                homing_stage = 2; // X/Y next

                protocol_emit_event("EVT:ARM_HOMING:axis=X");
                protocol_emit_event("EVT:ARM_HOMING:axis=Y");

                // Move X and Y negative by a large amount
                expected_steps[0] -= 100000;
                expected_steps[1] -= 100000;
                active_joints[0] = true;
                active_joints[1] = true;
                engine.moveTo(current_steps, expected_steps);
            } else if (!engine.isMoving()) {
                // Should not happen unless arm is physically disconnected or steps exhausted
                state = MotionState::FAULT;
                if (safety_monitor) safety_monitor->setHomingMode(false);
                protocol_emit_event("EVT:ARM_FAULT:code=HOME_Z_TIMEOUT:tier=hard");
            }
        } else if (homing_stage == 2) {
            bool x_hit = (safety_monitor && safety_monitor->isLimitTriggered(0));
            bool y_hit = (safety_monitor && safety_monitor->isLimitTriggered(1));

            if (x_hit) {
                engine.stopAxis(0);
                current_steps[0] = 0;
                expected_steps[0] = 0;
            }
            if (y_hit) {
                engine.stopAxis(1);
                current_steps[1] = 0;
                expected_steps[1] = 0;
            }

            if (x_hit && y_hit) {
                engine.stopAxis(0);
                engine.stopAxis(1);

                if (safety_monitor) {
                    safety_monitor->setHomingMode(false);
                }

                if (encoder_reader) {
                    encoder_reader->resetAll();
                }

                for (int i = 0; i < 6; i++) {
                    current_steps[i] = 0;
                    expected_steps[i] = 0;
                    active_joints[i] = false;
                }

                state = MotionState::IDLE;
                homing_stage = 0;
                is_homed = true;
                protocol_emit_event("EVT:ARM_HOMED");

                Command next_cmd;
                if (queue.dequeue(next_cmd)) {
                    process_command(next_cmd);
                }
            } else if (!engine.isMoving()) {
                state = MotionState::FAULT;
                if (safety_monitor) safety_monitor->setHomingMode(false);
                protocol_emit_event("EVT:ARM_FAULT:code=HOME_XY_TIMEOUT:tier=hard");
            }
        }
        return;
    }

    if (state == MotionState::MOVING) {
        engine.update();

        if (safety_monitor && safety_monitor->isFaulted()) {
            state = MotionState::FAULT;
            is_homed = false; // Lost position tracking
            protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_HIT:tier=hard");
            return;
        }

        if (engine.isMoving()) {
            return;
        }

        bool position_mismatch = false;

        if (encoder_reader) {
            for (int i = 0; i < 6; i++) {
                if (active_joints[i]) {
                    int32_t actual = encoder_reader->getPosition(i + 1);
                    int32_t diff = labs(actual - expected_steps[i]);
                    if (diff > POSITION_TOLERANCE_STEPS) {
                        position_mismatch = true;
                        break;
                    }
                }
            }
        }

        for (int i = 0; i < 6; i++) {
            if (drivers[i]) {
                drivers[i]->disable();
            }
        }

        if (position_mismatch) {
            state = MotionState::FAULT;
            is_homed = false; // Lost position tracking
            protocol_emit_event("EVT:ARM_FAULT:code=POSITION_MISMATCH:tier=hard");
            return;
        }

        for (int i = 0; i < 6; i++) {
            current_steps[i] = expected_steps[i];
        }

        if (current_command_type == CommandType::WRIST_SET) {
            protocol_emit_event("EVT:WRIST_DONE");
        } else if (current_command_type == CommandType::TOOL_DOCK) {
            // Include the tool name if available, else emit basic DOCKED
            if (current_cmd.tool_name[0] != '\0') {
                char buf[64] = "EVT:TOOL_DOCKED:tool=";
                strncat(buf, current_cmd.tool_name, 16);
                protocol_emit_event(buf);
            } else {
                protocol_emit_event("EVT:TOOL_DOCKED");
            }
        } else if (current_command_type == CommandType::TOOL_RELEASE) {
            // Check for mock alignment error
            if (strncmp(current_cmd.tool_name, "MISALIGNED", 10) == 0) {
                state = MotionState::FAULT;
                protocol_emit_event("EVT:TOOL_FAULT:code=ALIGNMENT_ERROR");
                return;
            } else {
                // To keep the buffer small, construct the string manually
                char buf[64] = "EVT:TOOL_RELEASED:tool=";
                strncat(buf, current_cmd.tool_name, 16);
                protocol_emit_event(buf);
            }
        } else {
            protocol_emit_event("EVT:ARM_DONE");
        }

        state = MotionState::IDLE;

        Command next_cmd;
        if (queue.dequeue(next_cmd)) {
            process_command(next_cmd);
        }
    }
}

#include <stdio.h> // for snprintf
#ifdef ARDUINO
#include <Arduino.h>
#endif

uint32_t MotionController::get_millis() {
#ifdef ARDUINO
    return millis();
#else
    extern uint32_t g_mock_millis;
    return g_mock_millis;
#endif
}

void MotionController::emitSoftFault(const char* code) {
    uint32_t now = get_millis();

    // Clean up old faults (> 5 minutes = 300,000 ms)
    uint8_t new_count = 0;
    uint32_t new_timestamps[3] = {0, 0, 0};
    for (int i = 0; i < soft_fault_count; i++) {
        if (now - soft_fault_timestamps[i] <= 300000) {
            new_timestamps[new_count++] = soft_fault_timestamps[i];
        }
    }

    for (int i = 0; i < 3; i++) {
        soft_fault_timestamps[i] = new_timestamps[i];
    }
    soft_fault_count = new_count;

    if (soft_fault_count < 3) {
        soft_fault_timestamps[soft_fault_count++] = now;
    }

    if (soft_fault_count == 1) {
        char buf[64];
        snprintf(buf, sizeof(buf), "EVT:ARM_FAULT:code=%s:tier=soft", code);
        protocol_emit_event(buf);
    } else if (soft_fault_count == 2) {
        char buf[64];
        snprintf(buf, sizeof(buf), "EVT:ARM_FAULT:code=%s:tier=soft:repeat=2", code);
        protocol_emit_event(buf);
    } else if (soft_fault_count >= 3) {
        soft_fault_count = 0;
        state = MotionState::FAULT;
        is_homed = false;
        protocol_emit_event("EVT:ARM_FAULT:code=REPEATED_FAULT:tier=hard");
    }
}
