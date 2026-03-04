#include "MotionController.h"
#include "CoordinateMapper.h"

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
    if (cmd.type == CommandType::ARM_MOVE_TO) {
        for (int i = 0; i < 6; i++) {
            if (cmd.has_angle[i]) {
                if (!CoordinateMapper::is_in_range(i + 1, cmd.angles[i])) {
                    protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE");
                    return;
                }
            }
        }
    }

    state = MotionState::MOVING;
    current_command_type = cmd.type;

    for (int i = 0; i < 6; i++) {
        active_joints[i] = false;
    }

    // Delegate to mock driver to record the call so we know it tried to move
    for (int i = 0; i < 6; i++) {
        if (drivers[i]) {
            drivers[i]->enable();
            // Just some arbitrary step call to show we triggered the driver
            // Real implementation will calculate steps using CoordinateMapper
            if (cmd.type == CommandType::ARM_MOVE_TO && cmd.has_angle[i]) {
                drivers[i]->step(true, 100);
                active_joints[i] = true;
                expected_steps[i] += 100; // Increment expected steps by mock step count
            }
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
        }

        state = MotionState::IDLE;
        protocol_emit_event("EVT:ARM_DONE");

        Command next_cmd;
        if (queue.dequeue(next_cmd)) {
            process_command(next_cmd);
        }
    }
}
