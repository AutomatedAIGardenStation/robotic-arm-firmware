#include "ArduinoLimitSwitch.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
// For native testing
extern void pinMode(int pin, int mode);
extern int digitalRead(int pin);
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 2
#endif
#ifndef LOW
#define LOW 0
#endif
#endif

ArduinoLimitSwitch::ArduinoLimitSwitch(int pin) : _pin(pin) {
    if (_pin >= 0) {
        pinMode(_pin, INPUT_PULLUP);
    }
}

bool ArduinoLimitSwitch::isTriggered() const {
    if (_pin < 0) return false;
    // Assuming switches are Normally Closed (NC), pulled high, LOW when triggered.
    // Wait, the README says:
    // Limit switches (NC, pulled high – LOW = triggered)
    return digitalRead(_pin) == LOW;
}
