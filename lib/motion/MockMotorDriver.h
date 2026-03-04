#ifndef MOCK_MOTOR_DRIVER_H
#define MOCK_MOTOR_DRIVER_H

#include "../hal/IMotorDriver.h"

class MockMotorDriver : public IMotorDriver {
public:
    int enable_calls = 0;
    int disable_calls = 0;
    int step_calls = 0;
    int set_speed_calls = 0;

    bool last_dir = false;
    uint32_t last_count = 0;
    uint32_t last_speed = 0;

    void enable() override {
        enable_calls++;
    }

    void disable() override {
        disable_calls++;
    }

    void step(bool dir, uint32_t count) override {
        step_calls++;
        last_dir = dir;
        last_count = count;
    }

    void setSpeed(uint32_t steps_per_sec) override {
        set_speed_calls++;
        last_speed = steps_per_sec;
    }

    void reset() {
        enable_calls = 0;
        disable_calls = 0;
        step_calls = 0;
        set_speed_calls = 0;
        last_dir = false;
        last_count = 0;
        last_speed = 0;
    }
};

#endif // MOCK_MOTOR_DRIVER_H
