#ifndef STEPPER_ENGINE_H
#define STEPPER_ENGINE_H

#include "../hal/IMotorDriver.h"
#include <stdint.h>

class StepperEngine {
public:
    explicit StepperEngine(IMotorDriver* drivers[6]);

    // Plan a synchronous multi-axis move to the target step positions
    // start_steps is the current known position
    void moveTo(const int32_t start_steps[6], const int32_t target_steps[6]);

    // To be called periodically to advance the step sequence
    void update();

    // Check if a move is currently active
    bool isMoving() const;

    // Immediately stop a specific axis
    void stopAxis(int axis_index);

private:
    IMotorDriver* drivers[6];
    bool moving;

    // Current planned move parameters
    int32_t current_steps[6];
    int32_t target_steps[6];

    // Bresenham's line algorithm parameters for multi-axis
    int32_t delta[6];
    int32_t error[6];
    int8_t step_dir[6];
    int32_t max_delta;

    // Linear acceleration profile parameters
    int32_t step_count;
    uint32_t current_delay_us;
    uint32_t min_delay_us;     // max speed
    uint32_t max_delay_us;     // start/stop speed
    int32_t accel_steps;
    int32_t decel_steps;

    uint32_t last_step_time_us;

    uint32_t get_micros();
};

#endif // STEPPER_ENGINE_H
