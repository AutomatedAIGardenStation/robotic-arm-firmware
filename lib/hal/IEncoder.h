#ifndef I_ENCODER_H
#define I_ENCODER_H

#include <stdint.h>

class IEncoder {
public:
    virtual ~IEncoder() = default;

    virtual int32_t getPosition() const = 0;
    virtual void reset() = 0;
};

#endif // I_ENCODER_H
