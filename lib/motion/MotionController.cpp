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
    safety_monitor = safety;
    this->encoder_reader = encoder_reader;

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

void MotionController::execute(const Command& cmd) {
    if (cmd.type == CommandType::ARM_CLEAR_FAULT) {
        if (safety_monitor && safety_monitor->clearFault()) {
            state = MotionState::IDLE;
            protocol_emit_event("EVT:ARM_FAULT_CLEARED");
        } else {
            protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_ACTIVE:tier=hard");
        }
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

    if (state == MotionState::MOVING) {
        if (!queue.enqueue(cmd)) {
            protocol_emit_event("EVT:ARM_FAULT:code=QUEUE_FULL:tier=soft");
        }
        return;
    }

    process_command(cmd);
}

void MotionController::process_command(const Command& cmd) {
    current_cmd = cmd;

    if (cmd.type == CommandType::ARM_MOVE_TO) {
        if (cmd.has_x && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, cmd.x)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft");
            return;
        }
        if (cmd.has_y && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Y, cmd.y)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft");
            return;
        }
        if (cmd.has_z && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Z, cmd.z)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft");
            return;
        }
    } else if (cmd.type == CommandType::WRIST_SET) {
        if (cmd.has_pitch && !CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, cmd.pitch)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft");
            return;
        }
        if (cmd.has_roll && !CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, cmd.roll)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE:tier=soft");
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
        // Return to 0
        for (int i = 0; i < 6; i++) {
            expected_steps[i] = 0;
            active_joints[i] = true;
        }
    } else if (cmd.type == CommandType::GRIPPER_OPEN || cmd.type == CommandType::GRIPPER_CLOSE) {
        // Advance dummy joint
        expected_steps[5] += 10;
        active_joints[5] = true;
    }

    engine.moveTo(current_steps, expected_steps);
}

void MotionController::update() {
    if (state == MotionState::MOVING) {
        engine.update();

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
            protocol_emit_event("EVT:ARM_FAULT:code=POSITION_MISMATCH:tier=hard");
            return;
        }

        for (int i = 0; i < 6; i++) {
            current_steps[i] = expected_steps[i];
        }

        if (current_command_type == CommandType::ARM_HOME) {
            if (encoder_reader) {
                encoder_reader->resetAll();
            }
            for (int i = 0; i < 6; i++) {
                current_steps[i] = 0;
                expected_steps[i] = 0;
                active_joints[i] = false;
            }
            protocol_emit_event("EVT:ARM_HOMED");
        } else if (current_command_type == CommandType::WRIST_SET) {
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
