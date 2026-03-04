/**
 * arm_controller – Serial protocol implementation.
 * Dispatches inbound ASCII lines from the backend and emits events.
 *
 * TODO: Replace stub actions with real HAL calls for your motor driver library.
 */
#include "protocol.h"
#include <Arduino.h>
#include <string.h>   // strncmp, strlen, strstr
#include <stdlib.h>
#include "CoordinateMapper.h"

// ── helpers ──────────────────────────────────────────────────────────────────

static inline bool cmd_match(const char* line, const char* cmd, size_t line_len) {
    size_t cmd_len = strlen(cmd);
    if (line_len < cmd_len) return false;
    return strncmp(line, cmd, cmd_len) == 0
        && (line_len == cmd_len || line[cmd_len] == ':');
}

// ── stubs – replace with real HAL calls ─────────────────────────────────────

static void arm_go_home() {
    // TODO: command stepper/servo drivers to home position via limit switches
}

static void arm_move_to(const char* param_str) {
    if (!param_str) return;

    // Parse ":j1=<v>:j2=<v>..."
    char buffer[128];
    strncpy(buffer, param_str, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    const char* token = strtok(buffer, ":");
    while (token != nullptr) {
        if (token[0] == 'j' || token[0] == 'J') {
            int joint_id = token[1] - '0';
            const char* val_str = strchr(token, '=');
            if (val_str && joint_id >= 1 && joint_id <= 6) {
                float degrees = atof(val_str + 1);
                if (!CoordinateMapper::is_in_range(joint_id, degrees)) {
                    protocol_emit_event("EVT:ARM_FAULT:code=OUT_OF_RANGE");
                    return;
                }
                // (Optional) convert to steps to prepare motion command here
            }
        }
        token = strtok(nullptr, ":");
    }

    // TODO: drive servos to target angles
}

static void gripper_open() {
    // TODO: energise gripper servo to OPEN angle
}

static void gripper_close() {
    // TODO: energise gripper servo to CLOSE angle
}

static void arm_move_zone(const char* param_str) {
    // TODO: look up named zone coordinates and delegate to arm_move_to
    (void)param_str;
}

static void arm_pollinate_sequence() {
    // TODO: execute pollination motion pattern
}

static void arm_harvest_sequence() {
    // TODO: move to harvest position, close gripper, retract to deposit zone
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
        arm_go_home();
        protocol_emit_event(EVT_ARM_DONE);
        return true;
    }
    if (cmd_match(line, CMD_ARM_MOVE, len)) {
        arm_move_to(params ? params : "");
        // EVT_ARM_DONE should be emitted when motion completes (motion ISR or polling)
        return true;
    }
    if (cmd_match(line, CMD_GRIPPER_OPEN, len)) {
        gripper_open();
        protocol_emit_event(EVT_ARM_DONE);
        return true;
    }
    if (cmd_match(line, CMD_GRIPPER_CLOSE, len)) {
        gripper_close();
        protocol_emit_event(EVT_ARM_DONE);
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
        arm_harvest_sequence();
        protocol_emit_event(EVT_ARM_DONE);
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
