#ifndef MOCK_ENCODER_H
#define MOCK_ENCODER_H

#include "../hal/IEncoder.h"

class MockEncoder : public IEncoder {
public:
    int32_t position = 0;

    int32_t getPosition() const override {
        return position;
    }

    void reset() override {
        position = 0;
    }

    void setPosition(int32_t pos) {
        position = pos;
    }
};

#endif // MOCK_ENCODER_H
