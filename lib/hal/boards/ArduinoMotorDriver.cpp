#include "ArduinoMotorDriver.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
// For native testing
#include <stdint.h>
extern void pinMode(int pin, int mode);
extern void digitalWrite(int pin, int val);
extern void delayMicroseconds(unsigned int us);
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif
#endif

ArduinoMotorDriver::ArduinoMotorDriver(int step_pin, int dir_pin, int en_pin)
    : _step_pin(step_pin), _dir_pin(dir_pin), _en_pin(en_pin), _speed(1000) {
    if (_step_pin >= 0) pinMode(_step_pin, OUTPUT);
    if (_dir_pin >= 0) pinMode(_dir_pin, OUTPUT);
    if (_en_pin >= 0) pinMode(_en_pin, OUTPUT);
    disable(); // Start disabled
}

void ArduinoMotorDriver::enable() {
    // Enable pin is usually active low, but we'll assume LOW = enabled
    // Note: Depends on driver board, standard A4988 / DRV8825 uses LOW to enable
    if (_en_pin >= 0) digitalWrite(_en_pin, LOW);
}

void ArduinoMotorDriver::disable() {
    if (_en_pin >= 0) digitalWrite(_en_pin, HIGH);
}

void ArduinoMotorDriver::step(bool dir, uint32_t count) {
    if (_dir_pin >= 0) digitalWrite(_dir_pin, dir ? HIGH : LOW);

    // We do minimal pulse width.
    // In StepperEngine, typically it calls step(dir, 1).
    // Delay between steps is handled by StepperEngine.
    for (uint32_t i = 0; i < count; i++) {
        if (_step_pin >= 0) digitalWrite(_step_pin, HIGH);
#ifdef ARDUINO
        delayMicroseconds(2); // Short pulse
#else
        delayMicroseconds(2);
#endif
        if (_step_pin >= 0) digitalWrite(_step_pin, LOW);
#ifdef ARDUINO
        delayMicroseconds(2);
#else
        delayMicroseconds(2);
#endif
    }
}

void ArduinoMotorDriver::setSpeed(uint32_t steps_per_sec) {
    _speed = steps_per_sec;
}
