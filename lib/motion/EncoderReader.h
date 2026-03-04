#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <stdint.h>
#include "../hal/IEncoder.h"

class EncoderReader {
public:
    explicit EncoderReader(IEncoder* encoders[6]);

    int32_t getPosition(uint8_t joint_id) const;
    void resetAll();

private:
    IEncoder* encoders[6];
};

#endif // ENCODER_READER_H
