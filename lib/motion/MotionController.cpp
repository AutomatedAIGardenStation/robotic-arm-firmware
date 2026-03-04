#include "MotionController.h"
#include "CoordinateMapper.h"

// Declaration to satisfy compiler without including protocol.h
extern void protocol_emit_event(const char* event);

MotionController::MotionController(IMotorDriver* drvs[6]) {
    state = MotionState::IDLE;
    has_buffered_cmd = false;

    for (int i = 0; i < 6; i++) {
        if (drvs) {
            drivers[i] = drvs[i];
        } else {
            drivers[i] = nullptr;
        }
    }
}

MotionState MotionController::getState() const {
    return state;
}

void MotionController::execute(const Command& cmd) {
    if (state == MotionState::FAULT) {
        return;
    }

    if (state == MotionState::MOVING) {
        // Single slot buffer
        buffered_cmd = cmd;
        has_buffered_cmd = true;
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

    // Delegate to mock driver to record the call so we know it tried to move
    for (int i = 0; i < 6; i++) {
        if (drivers[i]) {
            drivers[i]->enable();
            // Just some arbitrary step call to show we triggered the driver
            // Real implementation will calculate steps using CoordinateMapper
            if (cmd.type == CommandType::ARM_MOVE_TO && cmd.has_angle[i]) {
                drivers[i]->step(true, 100);
            }
        }
    }
}

void MotionController::update() {
    if (state == MotionState::MOVING) {
        state = MotionState::IDLE;
        protocol_emit_event("EVT:ARM_DONE");

        for (int i = 0; i < 6; i++) {
            if (drivers[i]) {
                drivers[i]->disable();
            }
        }

        if (has_buffered_cmd) {
            has_buffered_cmd = false;
            process_command(buffered_cmd);
        }
    }
}
