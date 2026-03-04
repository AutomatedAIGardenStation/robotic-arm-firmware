#include "EncoderReader.h"

EncoderReader::EncoderReader(IEncoder* encs[6]) {
    for (int i = 0; i < 6; i++) {
        if (encs) {
            encoders[i] = encs[i];
        } else {
            encoders[i] = nullptr;
        }
    }
}

int32_t EncoderReader::getPosition(uint8_t joint_id) const {
    if (joint_id >= 1 && joint_id <= 6) {
        if (encoders[joint_id - 1]) {
            return encoders[joint_id - 1]->getPosition();
        }
    }
    return 0;
}

void EncoderReader::resetAll() {
    for (int i = 0; i < 6; i++) {
        if (encoders[i]) {
            encoders[i]->reset();
        }
    }
}
