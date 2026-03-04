#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "../hal/IMotorDriver.h"
#include <stdint.h>

enum class MotionState {
    IDLE,
    MOVING,
    FAULT
};

enum class CommandType {
    NONE,
    ARM_HOME,
    ARM_MOVE_TO,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    H1
};

struct Command {
    CommandType type;
    float angles[6]; // For ARM_MOVE_TO: indices 0-5 map to J1-J6
    bool has_angle[6];

    Command() {
        type = CommandType::NONE;
        for (int i = 0; i < 6; i++) {
            angles[i] = 0.0f;
            has_angle[i] = false;
        }
    }
};

class MotionController {
public:
    explicit MotionController(IMotorDriver* drivers[6]);

    void execute(const Command& cmd);
    MotionState getState() const;
    void update(); // call periodically to process moves/buffered commands

private:
    MotionState state;
    IMotorDriver* drivers[6];

    bool has_buffered_cmd;
    Command buffered_cmd;

    void process_command(const Command& cmd);
};

#endif // MOTION_CONTROLLER_H
