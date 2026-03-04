#ifndef COMMAND_H
#define COMMAND_H

enum class CommandType {
    NONE,
    ARM_HOME,
    ARM_MOVE_TO,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    H1
};

struct Command {
    CommandType type;
    float angles[6]; // For ARM_MOVE_TO: indices 0-5 map to J1-J6
    bool has_angle[6];

    Command() {
        type = CommandType::NONE;
        for (int i = 0; i < 6; i++) {
            angles[i] = 0.0f;
            has_angle[i] = false;
        }
    }
};

#endif // COMMAND_H
