/**
 * arm_controller – Serial protocol implementation.
 * Dispatches inbound ASCII lines from the backend and emits events.
 *
 */
#include "protocol.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
extern void delay(unsigned long ms);
#ifndef PROTOCOL_SERIAL_DUMMY_DEFINED
class ProtocolSerialDummy {
public:
    void print(const char* s) {}
    void println(const char* s) {}
};
static ProtocolSerialDummy Serial;
#endif
#endif
#include <string.h>   // strncmp, strlen, strstr
#include <stdlib.h>
#include "../lib/protocol/Crc8.h"
#include "CoordinateMapper.h"
#include "../lib/motion/MotionController.h"
#include "../lib/motion/MockMotorDriver.h"
#include "../lib/safety/MockLimitSwitch.h"
#include "../lib/safety/SafetyMonitor.h"
#include "../lib/motion/EncoderReader.h"
#include "../lib/motion/MockEncoder.h"
#include "../../config/Config.h"

extern uint32_t g_last_ping_ms;

void protocol_ping_received() {
#ifdef ARDUINO
    g_last_ping_ms = millis();
#else
    extern uint32_t millis();
    g_last_ping_ms = millis();
#endif
}

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

// ── public API ───────────────────────────────────────────────────────────────

static uint8_t parse_hex_byte(const char* hex) {
    uint8_t val = 0;
    for (int i = 0; i < 2; i++) {
        char c = hex[i];
        val <<= 4;
        if (c >= '0' && c <= '9') val |= (c - '0');
        else if (c >= 'A' && c <= 'F') val |= (c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') val |= (c - 'a' + 10);
    }
    return val;
}

bool protocol_handle_line(const char* line) {
    if (!line) return false;
    size_t len = strlen(line);
    // Trim trailing CR/LF/space
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' '))
        len--;
    if (len == 0) return false;

    // Minimum length check: "<SEQ>:<CMD>:CRC=<XX>" => 4 + 1 + 3 + 1 + 4 + 1 + 2 = 16
    if (len < 16) {
        Serial.print("ERR:UNKNOWN:");
        Serial.println(line);
        return false;
    }

    // Parse SEQ (4 hex characters)
    char seq_str[5] = {0};
    strncpy(seq_str, line, 4);

    // Find ":CRC=" starting from the end
    const char* crc_ptr = strstr(line, ":CRC=");
    if (!crc_ptr || (crc_ptr + 7) > (line + len)) {
        Serial.print("ERR:UNKNOWN:");
        Serial.println(line);
        return false;
    }

    // Verify CRC
    uint8_t expected_crc = parse_hex_byte(crc_ptr + 5);
    size_t crc_calc_len = (crc_ptr - line) + 1; // Include the ':' before CRC
    uint8_t actual_crc = crc8_compute(line, crc_calc_len);

    if (actual_crc != expected_crc) {
        Serial.print("NACK:");
        Serial.print(seq_str);
        Serial.println(":BADCRC");
        return false;
    }

    // Get command part after SEQ:
    const char* cmd_start = line + 5; // Skip "<SEQ>:"
    size_t cmd_len = crc_ptr - cmd_start;

    const char* params = nullptr;
    const char* sep = static_cast<const char*>(memchr(cmd_start, ':', cmd_len));
    if (sep) {
        // Create a null-terminated params string for strtok
        // We will make a copy of the params string
        params = sep + 1;
    }

    // Define a helper to emit ACK
    auto emit_ack = [&]() {
        Serial.print("ACK:");
        Serial.println(seq_str);
    };

    if (cmd_match(cmd_start, CMD_ARM_HOME, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::ARM_HOME;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_ARM_MOVE, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::ARM_MOVE_TO;
        if (params) {
            char buffer[128];
            size_t param_len = crc_ptr - params;
            size_t copy_len = param_len < (sizeof(buffer) - 1) ? param_len : (sizeof(buffer) - 1);
            strncpy(buffer, params, copy_len);
            buffer[copy_len] = '\0';

            const char* token = strtok(buffer, ":");
            int param_index = 0;
            while (token != nullptr) {
                if (strncmp(token, "x=", 2) == 0) {
                    cmd.x = atof(token + 2);
                    cmd.has_x = true;
                } else if (strncmp(token, "y=", 2) == 0) {
                    cmd.y = atof(token + 2);
                    cmd.has_y = true;
                } else if (strncmp(token, "z=", 2) == 0) {
                    cmd.z = atof(token + 2);
                    cmd.has_z = true;
                } else if (strchr(token, '=') == nullptr) {
                    if (param_index == 0) {
                        cmd.x = atof(token);
                        cmd.has_x = true;
                    } else if (param_index == 1) {
                        cmd.y = atof(token);
                        cmd.has_y = true;
                    } else if (param_index == 2) {
                        cmd.z = atof(token);
                        cmd.has_z = true;
                    }
                }
                param_index++;
                token = strtok(nullptr, ":");
            }
        }
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_ARM_CLEAR_FAULT, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::ARM_CLEAR_FAULT;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_WRIST_SET, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::WRIST_SET;
        if (params) {
            char buffer[128];
            size_t param_len = crc_ptr - params;
            size_t copy_len = param_len < (sizeof(buffer) - 1) ? param_len : (sizeof(buffer) - 1);
            strncpy(buffer, params, copy_len);
            buffer[copy_len] = '\0';

            const char* token = strtok(buffer, ":");
            while (token != nullptr) {
                if (strncmp(token, "pitch=", 6) == 0) {
                    cmd.pitch = atof(token + 6);
                    cmd.has_pitch = true;
                } else if (strncmp(token, "roll=", 5) == 0) {
                    cmd.roll = atof(token + 5);
                    cmd.has_roll = true;
                }
                token = strtok(nullptr, ":");
            }
        }
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_GRIPPER_OPEN, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::GRIPPER_OPEN;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_GRIPPER_CLOSE, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::GRIPPER_CLOSE;
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_TOOL_DOCK, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::TOOL_DOCK;
        if (params) {
            char buffer[128];
            size_t param_len = crc_ptr - params;
            size_t copy_len = param_len < (sizeof(buffer) - 1) ? param_len : (sizeof(buffer) - 1);
            strncpy(buffer, params, copy_len);
            buffer[copy_len] = '\0';

            const char* token = strtok(buffer, ":");
            while (token != nullptr) {
                if (strncmp(token, "tool=", 5) == 0) {
                    strncpy(cmd.tool_name, token + 5, sizeof(cmd.tool_name) - 1);
                    cmd.tool_name[sizeof(cmd.tool_name) - 1] = '\0';
                }
                token = strtok(nullptr, ":");
            }
        }
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_TOOL_RELEASE, cmd_len)) {
        emit_ack();
        Command cmd;
        cmd.type = CommandType::TOOL_RELEASE;
        if (params) {
            char buffer[128];
            size_t param_len = crc_ptr - params;
            size_t copy_len = param_len < (sizeof(buffer) - 1) ? param_len : (sizeof(buffer) - 1);
            strncpy(buffer, params, copy_len);
            buffer[copy_len] = '\0';

            const char* token = strtok(buffer, ":");
            while (token != nullptr) {
                if (strncmp(token, "tool=", 5) == 0) {
                    strncpy(cmd.tool_name, token + 5, sizeof(cmd.tool_name) - 1);
                    cmd.tool_name[sizeof(cmd.tool_name) - 1] = '\0';
                }
                token = strtok(nullptr, ":");
            }
        }
        g_motion_controller.execute(cmd);
        return true;
    }
    if (cmd_match(cmd_start, CMD_PING, cmd_len)) {
        emit_ack();
        protocol_ping_received();
        return true;
    }
    if (cmd_match(cmd_start, CMD_NOP, cmd_len)) {
        emit_ack();
        return true;
    }

    // Unknown command
    Serial.print("NACK:");
    Serial.print(seq_str);
    Serial.println(":UNKNOWN");
    return false;
}

void protocol_emit_event(const char* event) {
    Serial.println(event);
}
