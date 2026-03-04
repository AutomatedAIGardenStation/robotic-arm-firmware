#ifndef I_MOTOR_DRIVER_H
#define I_MOTOR_DRIVER_H

#include <stdint.h>

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    virtual void enable() = 0;
    virtual void disable() = 0;

    // Direction typically represented as a bool (e.g., true for forward, false for backward)
    virtual void step(bool dir, uint32_t count) = 0;

    // Speed in steps per second
    virtual void setSpeed(uint32_t steps_per_sec) = 0;
};

#endif // I_MOTOR_DRIVER_H
