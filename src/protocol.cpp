/**
 * arm_controller – Serial protocol implementation.
 * Dispatches inbound ASCII lines from the backend and emits events.
 *
 */
#include "protocol.h"
#include <Arduino.h>
#include <string.h>   // strncmp, strlen, strstr
#include <stdlib.h>
#include "CoordinateMapper.h"
#include "../lib/motion/MotionController.h"
#include "../lib/motion/MockMotorDriver.h"
#include "../lib/safety/MockLimitSwitch.h"
#include "../lib/safety/SafetyMonitor.h"
#include "../lib/motion/EncoderReader.h"
#include "../lib/motion/MockEncoder.h"
#include "../lib/motion/ZoneRegistry.h"

// Instantiate drivers and controller globally
MockMotorDriver g_drivers[6];
IMotorDriver* g_driver_ptrs[6] = { &g_drivers[0], &g_drivers[1], &g_drivers[2], &g_drivers[3], &g_drivers[4], &g_drivers[5] };

MockLimitSwitch g_limit_switches[6];
ILimitSwitch* g_limit_switch_ptrs[6] = { &g_limit_switches[0], &g_limit_switches[1], &g_limit_switches[2], &g_limit_switches[3], &g_limit_switches[4], &g_limit_switches[5] };

MockEncoder g_encoders[6];
IEncoder* g_encoder_ptrs[6] = { &g_encoders[0], &g_encoders[1], &g_encoders[2], &g_encoders[3], &g_encoders[4], &g_encoders[5] };

EncoderReader g_encoder_reader(g_encoder_ptrs);

SafetyMonitor g_safety_monitor(g_limit_switch_ptrs, g_driver_ptrs);
MotionController g_motion_controller(g_driver_ptrs, &g_safety_monitor, &g_encoder_reader);

// ── helpers ──────────────────────────────────────────────────────────────────

static inline bool cmd_match(const char* line, const char* cmd, size_t line_len) {
    size_t cmd_len = strlen(cmd);
    if (line_len < cmd_len) return false;
    return strncmp(line, cmd, cmd_len) == 0
        && (line_len == cmd_len || line[cmd_len] == ':');
}

static void arm_move_zone(const char* param_str) {
    if (!param_str) {
        Serial.println("ERR:UNKNOWN_ZONE:");
        return;
    }

    const char* zone_name = nullptr;
    char buffer[128];
    strncpy(buffer, param_str, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    const char* token = strtok(buffer, ":");
    while (token != nullptr) {
        if (strncmp(token, "zone=", 5) == 0) {
            zone_name = token + 5;
            break;
        }
        token = strtok(nullptr, ":");
    }

    if (!zone_name) {
        Serial.println("ERR:UNKNOWN_ZONE:");
        return;
    }

    float angles[6];
    if (resolve_zone(zone_name, angles)) {
        Command cmd;
        cmd.type = CommandType::ARM_MOVE_TO;
        for (int i = 0; i < 6; i++) {
            cmd.angles[i] = angles[i];
            cmd.has_angle[i] = true;
        }
        g_motion_controller.execute(cmd);
        // Motion controller will emit EVT:ARM_DONE when complete
    } else {
        Serial.print("ERR:UNKNOWN_ZONE:");
        Serial.println(zone_name);
    }
}

static void arm_pollinate_sequence() {
    // TODO: execute pollination motion pattern
}

// ── public API ───────────────────────────────────────────────────────────────

bool protocol_handle_line(const char* line) {
    if (!line) return false;
    size_t len = strlen(line);
    // Trim trailing CR/LF/space
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' '))
        len--;
    if (len == 0) return false;

    const char* params = nullptr;
    const char* sep = static_cast<const char*>(memchr(line, ':', len));
    if (sep) params = sep + 1;

    if (cmd_match(line, CMD_ARM_HOME, len)) {
        Command cmd;
        cmd.type = CommandType::ARM_HOME;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(line, CMD_ARM_MOVE, len)) {
        Command cmd;
        cmd.type = CommandType::ARM_MOVE_TO;
        if (params) {
            char buffer[128];
            strncpy(buffer, params, sizeof(buffer) - 1);
            buffer[sizeof(buffer) - 1] = '\0';

            const char* token = strtok(buffer, ":");
            while (token != nullptr) {
                if (token[0] == 'j' || token[0] == 'J') {
                    int joint_id = token[1] - '0';
                    const char* val_str = strchr(token, '=');
                    if (val_str && joint_id >= 1 && joint_id <= 6) {
                        float degrees = atof(val_str + 1);
                        cmd.angles[joint_id - 1] = degrees;
                        cmd.has_angle[joint_id - 1] = true;
                    }
                }
                token = strtok(nullptr, ":");
            }
        }
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(line, CMD_GRIPPER_OPEN, len)) {
        Command cmd;
        cmd.type = CommandType::GRIPPER_OPEN;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(line, CMD_GRIPPER_CLOSE, len)) {
        Command cmd;
        cmd.type = CommandType::GRIPPER_CLOSE;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(line, CMD_MOVE_ZONE, len)) {
        arm_move_zone(params ? params : "");
        return true;
    }
    if (cmd_match(line, CMD_POLLINATE, len)) {
        arm_pollinate_sequence();
        protocol_emit_event(EVT_ARM_DONE);
        return true;
    }
    if (cmd_match(line, CMD_HARVEST, len)) {
        Command cmd;
        cmd.type = CommandType::H1;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(line, CMD_NOP, len)) {
        return true;
    }

    // Unknown command
    Serial.print("ERR:UNKNOWN:");
    Serial.println(line);
    return false;
}

void protocol_emit_event(const char* event) {
    Serial.println(event);
}
