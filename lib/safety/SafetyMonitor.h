#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "../hal/ILimitSwitch.h"
#include "../hal/IMotorDriver.h"

class SafetyMonitor {
public:
    explicit SafetyMonitor(ILimitSwitch* switches[6], IMotorDriver* drivers[6]);

    void poll();
    bool isFaulted() const;
    bool clearFault();

private:
    void brakeAll();

    ILimitSwitch* switches[6];
    IMotorDriver* drivers[6];
    bool system_fault;
};

#endif // SAFETY_MONITOR_H
