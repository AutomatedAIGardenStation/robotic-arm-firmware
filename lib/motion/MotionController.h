#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "../hal/IMotorDriver.h"
#include "CommandQueue.h"
#include "Command.h"
#include "../safety/SafetyMonitor.h"
#include "EncoderReader.h"
#include "StepperEngine.h"
#include <stdint.h>
#include <stddef.h>

enum class MotionState {
    IDLE,
    MOVING,
    HOMING,
    FAULT
};

class MotionController {
public:
    explicit MotionController(IMotorDriver* drivers[6], SafetyMonitor* safety, EncoderReader* encoder_reader = nullptr);

    void execute(const Command& cmd);
    MotionState getState() const;
    void update(); // call periodically to process moves/buffered commands
    bool hasHardFault() const;

    // Exposed for testing purposes
    Command current_cmd;

private:
    MotionState state;
    IMotorDriver* drivers[6];
    SafetyMonitor* safety_monitor;
    EncoderReader* encoder_reader;
    StepperEngine engine;

    int32_t current_steps[6];
    int32_t expected_steps[6];
    bool active_joints[6];
    CommandType current_command_type;
    uint8_t homing_stage; // 0=Idle, 1=Homing Z, 2=Homing X/Y
    bool is_homed;

    CommandQueue queue;

    void process_command(const Command& cmd);

    // Soft fault tracking
    uint32_t soft_fault_timestamps[3];
    uint8_t soft_fault_count;

    void emitSoftFault(const char* code);
    uint32_t get_millis();
};

#endif // MOTION_CONTROLLER_H
