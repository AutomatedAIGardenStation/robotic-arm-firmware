#include "MotionController.h"
#include "CoordinateMapper.h"

// Declaration to satisfy compiler without including protocol.h
extern void protocol_emit_event(const char* event);

MotionController::MotionController(IMotorDriver* drvs[6], SafetyMonitor* safety) {
    state = MotionState::IDLE;
    safety_monitor = safety;

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

        Command next_cmd;
        if (queue.dequeue(next_cmd)) {
            process_command(next_cmd);
        }
    }
}
