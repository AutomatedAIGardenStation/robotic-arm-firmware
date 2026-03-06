#include "SafetyMonitor.h"

SafetyMonitor::SafetyMonitor(ILimitSwitch* sw[6], IMotorDriver* drvs[6]) {
    system_fault = false;
    homing_mode = false;
    for (int i = 0; i < 6; i++) {
        if (sw) {
            switches[i] = sw[i];
        } else {
            switches[i] = nullptr;
        }

        if (drvs) {
            drivers[i] = drvs[i];
        } else {
            drivers[i] = nullptr;
        }
    }
}

void SafetyMonitor::poll() {
    if (homing_mode) {
        return; // Global faults disabled during homing
    }

    bool triggered = false;
    for (int i = 0; i < 6; i++) {
        if (switches[i] && switches[i]->isTriggered()) {
            triggered = true;
            break;
        }
    }

    if (triggered) {
        system_fault = true;
        brakeAll();
    }
}

bool SafetyMonitor::isFaulted() const {
    return system_fault;
}

bool SafetyMonitor::clearFault() {
    for (int i = 0; i < 6; i++) {
        if (switches[i] && switches[i]->isTriggered()) {
            return false;
        }
    }

    system_fault = false;
    return true;
}

void SafetyMonitor::triggerFault() {
    system_fault = true;
    brakeAll();
}

void SafetyMonitor::setHomingMode(bool enabled) {
    homing_mode = enabled;
}

bool SafetyMonitor::isLimitTriggered(int axis_index) const {
    if (axis_index < 0 || axis_index >= 6) {
        return false;
    }
    if (switches[axis_index]) {
        return switches[axis_index]->isTriggered();
    }
    return false;
}

void SafetyMonitor::brakeAll() {
    for (int i = 0; i < 6; i++) {
        if (drivers[i]) {
            drivers[i]->disable();
        }
    }
}
