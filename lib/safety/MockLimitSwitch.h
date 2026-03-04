#ifndef MOCK_LIMIT_SWITCH_H
#define MOCK_LIMIT_SWITCH_H

#include "../hal/ILimitSwitch.h"

class MockLimitSwitch : public ILimitSwitch {
public:
    MockLimitSwitch() : triggered(false) {}

    bool isTriggered() const override {
        return triggered;
    }

    void setTriggered(bool state) {
        triggered = state;
    }

private:
    bool triggered;
};

#endif // MOCK_LIMIT_SWITCH_H
