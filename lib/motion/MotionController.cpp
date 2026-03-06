#include "MotionController.h"
#include "CoordinateMapper.h"
#include <string.h>

// Declaration to satisfy compiler without including protocol.h
extern void protocol_emit_event(const char* event);

#include "../../config/pins.h"
#include <stdlib.h>

MotionController::MotionController(IMotorDriver* drvs[6], SafetyMonitor* safety, EncoderReader* encoder_reader) {
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
            protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_ACTIVE");
        }
        return;
    }

    if (safety_monitor && safety_monitor->isFaulted()) {
        if (cmd.type == CommandType::ARM_HOME) {
            if (!safety_monitor->clearFault()) {
                protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_ACTIVE");
                return;
            }
        } else {
            protocol_emit_event("EVT:ARM_FAULT:code=LIMIT_HIT");
            return;
        }
    }

    if (state == MotionState::FAULT) {
        return;
    }

    if (state == MotionState::MOVING) {
        if (!queue.enqueue(cmd)) {
            protocol_emit_event("EVT:ARM_FAULT:code=QUEUE_FULL");
        }
        return;
    }

    process_command(cmd);
}

void MotionController::process_command(const Command& cmd) {
    current_cmd = cmd;

    if (cmd.type == CommandType::ARM_MOVE_TO) {
        if (cmd.has_x && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, cmd.x)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE");
            return;
        }
        if (cmd.has_y && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Y, cmd.y)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE");
            return;
        }
        if (cmd.has_z && !CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Z, cmd.z)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE");
            return;
        }
    } else if (cmd.type == CommandType::WRIST_SET) {
        if (cmd.has_pitch && !CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, cmd.pitch)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE");
            return;
        }
        if (cmd.has_roll && !CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, cmd.roll)) {
            protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE");
            return;
        }
    }

    state = MotionState::MOVING;
    current_command_type = cmd.type;

    for (int i = 0; i < 6; i++) {
        active_joints[i] = false;
    }

    // Delegate to mock driver to record the call so we know it tried to move
    if (cmd.type == CommandType::ARM_MOVE_TO) {
        if (cmd.has_x && drivers[0]) { drivers[0]->enable(); drivers[0]->step(true, 100); active_joints[0] = true; expected_steps[0] += 100; }
        if (cmd.has_y && drivers[1]) { drivers[1]->enable(); drivers[1]->step(true, 100); active_joints[1] = true; expected_steps[1] += 100; }
        if (cmd.has_z && drivers[2]) { drivers[2]->enable(); drivers[2]->step(true, 100); active_joints[2] = true; expected_steps[2] += 100; }
    } else if (cmd.type == CommandType::WRIST_SET) {
        if (cmd.has_pitch && drivers[3]) { drivers[3]->enable(); drivers[3]->step(true, 100); active_joints[3] = true; expected_steps[3] += 100; }
        if (cmd.has_roll && drivers[4]) { drivers[4]->enable(); drivers[4]->step(true, 100); active_joints[4] = true; expected_steps[4] += 100; }
    } else if (cmd.type == CommandType::TOOL_DOCK || cmd.type == CommandType::TOOL_RELEASE) {
        // Mock sequence step
        if (drivers[5]) { drivers[5]->enable(); drivers[5]->step(true, 10); active_joints[5] = true; expected_steps[5] += 10; }
    } else {
        // generic move for other commands like HOME / GRIPPER
        for (int i = 0; i < 6; i++) {
            if (drivers[i]) { drivers[i]->enable(); }
        }
    }
}

void MotionController::update() {
    if (state == MotionState::MOVING) {
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
            protocol_emit_event("EVT:ARM_FAULT:code=POSITION_MISMATCH");
            return;
        }

        if (current_command_type == CommandType::ARM_HOME) {
            if (encoder_reader) {
                encoder_reader->resetAll();
            }
            for (int i = 0; i < 6; i++) {
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
                protocol_emit_event("EVT:TOOL_FAULT:reason=ALIGNMENT_ERROR");
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
