#ifndef ARDUINO_MOTOR_DRIVER_H
#define ARDUINO_MOTOR_DRIVER_H

#include "../IMotorDriver.h"

class ArduinoMotorDriver : public IMotorDriver {
public:
    explicit ArduinoMotorDriver(int step_pin, int dir_pin, int en_pin);
    ~ArduinoMotorDriver() override = default;

    void enable() override;
    void disable() override;
    void step(bool dir, uint32_t count) override;
    void setSpeed(uint32_t steps_per_sec) override;

private:
    int _step_pin;
    int _dir_pin;
    int _en_pin;
    uint32_t _speed;
};

#endif // ARDUINO_MOTOR_DRIVER_H
