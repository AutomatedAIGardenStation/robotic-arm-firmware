#ifndef COORDINATE_MAPPER_H
#define COORDINATE_MAPPER_H

#include <stdint.h>

class CoordinateMapper {
public:
    // Joint IDs
    static const uint8_t JOINT_YAW = 1;
    static const uint8_t JOINT_SHOULDER = 2;
    static const uint8_t JOINT_ELBOW = 3;
    static const uint8_t JOINT_WRIST_ROLL = 4;
    static const uint8_t JOINT_WRIST_PITCH = 5;
    static const uint8_t JOINT_GRIPPER = 6;

    // Joint constants: Steps per revolution, microstepping, and gear ratio
    // Defaulting to typical 200 steps/rev, 16 microsteps, 1:1 gear ratio
    static constexpr float STEPS_PER_REV_J1 = 200.0f;
    static constexpr float MICROSTEPPING_J1 = 16.0f;
    static constexpr float GEAR_RATIO_J1 = 1.0f;

    static constexpr float STEPS_PER_REV_J2 = 200.0f;
    static constexpr float MICROSTEPPING_J2 = 16.0f;
    static constexpr float GEAR_RATIO_J2 = 1.0f;

    static constexpr float STEPS_PER_REV_J3 = 200.0f;
    static constexpr float MICROSTEPPING_J3 = 16.0f;
    static constexpr float GEAR_RATIO_J3 = 1.0f;

    static constexpr float STEPS_PER_REV_J4 = 200.0f;
    static constexpr float MICROSTEPPING_J4 = 16.0f;
    static constexpr float GEAR_RATIO_J4 = 1.0f;

    static constexpr float STEPS_PER_REV_J5 = 200.0f;
    static constexpr float MICROSTEPPING_J5 = 16.0f;
    static constexpr float GEAR_RATIO_J5 = 1.0f;

    static constexpr float STEPS_PER_REV_J6 = 200.0f;
    static constexpr float MICROSTEPPING_J6 = 16.0f;
    static constexpr float GEAR_RATIO_J6 = 1.0f;

    /**
     * @brief Converts joint angular delta (in degrees) to signed step count.
     * @param joint_id The joint to calculate for (1-6)
     * @param degrees The angular delta in degrees
     * @return signed step count
     */
    static float steps_from_degrees(uint8_t joint_id, float degrees);

    /**
     * @brief Checks if the given angle is within the mechanical sanity limits of the joint.
     * @param joint_id The joint to check (1-6)
     * @param degrees The target angle in degrees
     * @return true if the angle is within range, false otherwise
     */
    static bool is_in_range(uint8_t joint_id, float degrees);
};

#endif // COORDINATE_MAPPER_H
