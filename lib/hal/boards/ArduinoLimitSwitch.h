#ifndef ARDUINO_LIMIT_SWITCH_H
#define ARDUINO_LIMIT_SWITCH_H

#include "../ILimitSwitch.h"

class ArduinoLimitSwitch : public ILimitSwitch {
public:
    explicit ArduinoLimitSwitch(int pin);
    ~ArduinoLimitSwitch() override = default;

    bool isTriggered() const override;

private:
    int _pin;
};

#endif // ARDUINO_LIMIT_SWITCH_H
