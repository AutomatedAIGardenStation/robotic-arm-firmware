#ifndef COMMAND_H
#define COMMAND_H

enum class CommandType {
    NONE,
    ARM_HOME,
    ARM_MOVE_TO,
    ARM_CLEAR_FAULT,
    WRIST_SET,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    TOOL_DOCK,
    TOOL_RELEASE
};

struct Command {
    CommandType type;

    // Cartesian targets (mm)
    float x;
    float y;
    float z;
    bool has_x;
    bool has_y;
    bool has_z;

    // Wrist targets (degrees)
    float pitch;
    float roll;
    bool has_pitch;
    bool has_roll;

    // Tool target
    char tool_name[16];

    Command() {
        type = CommandType::NONE;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        has_x = false;
        has_y = false;
        has_z = false;

        pitch = 0.0f;
        roll = 0.0f;
        has_pitch = false;
        has_roll = false;

        for (int i = 0; i < 16; i++) {
            tool_name[i] = '\0';
        }
    }
};

#endif // COMMAND_H
