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
    void triggerFault();

    void setHomingMode(bool enabled);
    bool isLimitTriggered(int axis_index) const;

private:
    void brakeAll();

    ILimitSwitch* switches[6];
    IMotorDriver* drivers[6];
    bool system_fault;
    bool homing_mode;
};

#endif // SAFETY_MONITOR_H
