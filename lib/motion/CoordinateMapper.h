#ifndef COORDINATE_MAPPER_H
#define COORDINATE_MAPPER_H

#include <stdint.h>

class CoordinateMapper {
public:
    // Axis IDs
    static const uint8_t AXIS_X = 1;
    static const uint8_t AXIS_Y = 2;
    static const uint8_t AXIS_Z = 3;
    static const uint8_t WRIST_PITCH = 4;
    static const uint8_t WRIST_ROLL = 5;

    // Linear axis constants: Steps per mm
    static constexpr float STEPS_PER_MM_X = 40.0f;
    static constexpr float STEPS_PER_MM_Y = 40.0f;
    static constexpr float STEPS_PER_MM_Z = 40.0f;

    // Rotary axis constants
    static constexpr float STEPS_PER_REV_PITCH = 200.0f;
    static constexpr float MICROSTEPPING_PITCH = 16.0f;
    static constexpr float GEAR_RATIO_PITCH = 1.0f;

    static constexpr float STEPS_PER_REV_ROLL = 200.0f;
    static constexpr float MICROSTEPPING_ROLL = 16.0f;
    static constexpr float GEAR_RATIO_ROLL = 1.0f;

    /**
     * @brief Converts linear distance (in mm) to signed step count.
     * @param axis_id The linear axis to calculate for (1-3 for X, Y, Z)
     * @param mm The linear distance in mm
     * @return signed step count
     */
    static float steps_from_mm(uint8_t axis_id, float mm);

    /**
     * @brief Converts joint angular delta (in degrees) to signed step count.
     * @param axis_id The rotary axis to calculate for (4-5 for Pitch, Roll)
     * @param degrees The angular delta in degrees
     * @return signed step count
     */
    static float steps_from_degrees(uint8_t axis_id, float degrees);

    /**
     * @brief Checks if the given position/angle is within the mechanical sanity limits.
     * @param axis_id The axis to check
     * @param value The target position in mm or angle in degrees
     * @return true if within range, false otherwise
     */
    static bool is_in_range(uint8_t axis_id, float value);
};

#endif // COORDINATE_MAPPER_H
