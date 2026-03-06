#include "Crc8.h"

uint8_t crc8_compute(const void* data, size_t length) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
