#include "CoordinateMapper.h"

float CoordinateMapper::steps_from_degrees(uint8_t joint_id, float degrees) {
    float steps_per_rev = 0.0f;
    float microstepping = 0.0f;
    float gear_ratio = 0.0f;

    switch (joint_id) {
        case JOINT_YAW:
            steps_per_rev = STEPS_PER_REV_J1;
            microstepping = MICROSTEPPING_J1;
            gear_ratio = GEAR_RATIO_J1;
            break;
        case JOINT_SHOULDER:
            steps_per_rev = STEPS_PER_REV_J2;
            microstepping = MICROSTEPPING_J2;
            gear_ratio = GEAR_RATIO_J2;
            break;
        case JOINT_ELBOW:
            steps_per_rev = STEPS_PER_REV_J3;
            microstepping = MICROSTEPPING_J3;
            gear_ratio = GEAR_RATIO_J3;
            break;
        case JOINT_WRIST_ROLL:
            steps_per_rev = STEPS_PER_REV_J4;
            microstepping = MICROSTEPPING_J4;
            gear_ratio = GEAR_RATIO_J4;
            break;
        case JOINT_WRIST_PITCH:
            steps_per_rev = STEPS_PER_REV_J5;
            microstepping = MICROSTEPPING_J5;
            gear_ratio = GEAR_RATIO_J5;
            break;
        case JOINT_GRIPPER:
            steps_per_rev = STEPS_PER_REV_J6;
            microstepping = MICROSTEPPING_J6;
            gear_ratio = GEAR_RATIO_J6;
            break;
        default:
            return 0.0f; // Unknown joint
    }

    // Full steps per output revolution
    float steps_per_out_rev = steps_per_rev * microstepping * gear_ratio;

    // Degrees to fraction of revolution
    float rev_fraction = degrees / 360.0f;

    return rev_fraction * steps_per_out_rev;
}

bool CoordinateMapper::is_in_range(uint8_t joint_id, float degrees) {
    switch (joint_id) {
        case JOINT_YAW:
            // J1 Yaw: ±180°
            return (degrees >= -180.0f && degrees <= 180.0f);
        case JOINT_SHOULDER:
            // J2 Shoulder: 0°–180°
            return (degrees >= 0.0f && degrees <= 180.0f);
        case JOINT_ELBOW:
            // J3 Elbow: 0°–150°
            return (degrees >= 0.0f && degrees <= 150.0f);
        case JOINT_WRIST_ROLL:
            // J4 WristRoll: ±180°
            return (degrees >= -180.0f && degrees <= 180.0f);
        case JOINT_WRIST_PITCH:
            // J5 WristPitch: ±90°
            return (degrees >= -90.0f && degrees <= 90.0f);
        case JOINT_GRIPPER:
            // J6 Gripper: 0°–90°
            return (degrees >= 0.0f && degrees <= 90.0f);
        default:
            return false; // Unknown joint, consider out of range
    }
}
