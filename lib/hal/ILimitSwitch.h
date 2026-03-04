#ifndef I_LIMIT_SWITCH_H
#define I_LIMIT_SWITCH_H

class ILimitSwitch {
public:
    virtual ~ILimitSwitch() = default;

    virtual bool isTriggered() const = 0;
};

#endif // I_LIMIT_SWITCH_H
