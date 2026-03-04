#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "../hal/IMotorDriver.h"
#include "CommandQueue.h"
#include "Command.h"
#include <stdint.h>
#include <stddef.h>

enum class MotionState {
    IDLE,
    MOVING,
    FAULT
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

    CommandQueue queue;

    void process_command(const Command& cmd);
};

#endif // MOTION_CONTROLLER_H
