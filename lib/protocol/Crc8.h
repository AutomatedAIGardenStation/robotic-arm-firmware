#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * Computes CRC-8 with polynomial 0x07, initial value 0x00,
 * no input/output reflection, final XOR 0x00.
 */
uint8_t crc8_compute(const void* data, size_t length);
