#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "../hal/IMotorDriver.h"
#include "CommandQueue.h"
#include "Command.h"
#include "../safety/SafetyMonitor.h"
#include "EncoderReader.h"
#include <stdint.h>
#include <stddef.h>

enum class MotionState {
    IDLE,
    MOVING,
    FAULT
};

class MotionController {
public:
    explicit MotionController(IMotorDriver* drivers[6], SafetyMonitor* safety, EncoderReader* encoder_reader = nullptr);

    void execute(const Command& cmd);
    MotionState getState() const;
    void update(); // call periodically to process moves/buffered commands

private:
    MotionState state;
    IMotorDriver* drivers[6];
    SafetyMonitor* safety_monitor;
    EncoderReader* encoder_reader;

    int32_t expected_steps[6];
    bool active_joints[6];
    CommandType current_command_type;

    CommandQueue queue;

    void process_command(const Command& cmd);
};

#endif // MOTION_CONTROLLER_H
