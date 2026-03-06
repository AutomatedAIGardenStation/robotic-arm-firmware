#include "CoordinateMapper.h"

float CoordinateMapper::steps_from_mm(uint8_t axis_id, float mm) {
    float steps_per_mm = 0.0f;

    switch (axis_id) {
        case AXIS_X:
            steps_per_mm = STEPS_PER_MM_X;
            break;
        case AXIS_Y:
            steps_per_mm = STEPS_PER_MM_Y;
            break;
        case AXIS_Z:
            steps_per_mm = STEPS_PER_MM_Z;
            break;
        default:
            return 0.0f; // Unknown or rotary axis
    }

    return mm * steps_per_mm;
}

float CoordinateMapper::steps_from_degrees(uint8_t axis_id, float degrees) {
    float steps_per_rev = 0.0f;
    float microstepping = 0.0f;
    float gear_ratio = 0.0f;

    switch (axis_id) {
        case WRIST_PITCH:
            steps_per_rev = STEPS_PER_REV_PITCH;
            microstepping = MICROSTEPPING_PITCH;
            gear_ratio = GEAR_RATIO_PITCH;
            break;
        case WRIST_ROLL:
            steps_per_rev = STEPS_PER_REV_ROLL;
            microstepping = MICROSTEPPING_ROLL;
            gear_ratio = GEAR_RATIO_ROLL;
            break;
        default:
            return 0.0f; // Unknown or linear axis
    }

    // Full steps per output revolution
    float steps_per_out_rev = steps_per_rev * microstepping * gear_ratio;

    // Degrees to fraction of revolution
    float rev_fraction = degrees / 360.0f;

    return rev_fraction * steps_per_out_rev;
}

bool CoordinateMapper::is_in_range(uint8_t axis_id, float value) {
    switch (axis_id) {
        case AXIS_X:
            // X axis: 0 to 1000 mm
            return (value >= 0.0f && value <= 1000.0f);
        case AXIS_Y:
            // Y axis: 0 to 1000 mm
            return (value >= 0.0f && value <= 1000.0f);
        case AXIS_Z:
            // Z axis: 0 to 500 mm
            return (value >= 0.0f && value <= 500.0f);
        case WRIST_PITCH:
            // WristPitch: ±90°
            return (value >= -90.0f && value <= 90.0f);
        case WRIST_ROLL:
            // WristRoll: ±90°
            return (value >= -90.0f && value <= 90.0f);
        default:
            return false; // Unknown axis, consider out of range
    }
}
