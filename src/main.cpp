/**
 * arm_controller – main entry point.
 *
 * Reads ASCII command lines from Serial (backend Serial Manager), one per line.
 * Baud: 115200.
 *
 * Architecture:
 *   Backend ──serial──► arm_controller (this firmware) ──HAL──► Motors / Servos
 *   arm_controller ──serial──► Backend  (events: EVT:ARM_DONE, EVT:ARM_FAULT, EVT:HEARTBEAT)
 *
 * See: project-info/Docs/06_Software/arm_controller/Serial Protocol & Interfaces.md
 */
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
class MainSerialDummy {
public:
    void begin(int baud) {}
    operator bool() const { return true; }
    int available() { return 0; }
    int read() { return -1; }
};
static MainSerialDummy Serial;
extern uint32_t millis();
#endif
#include "protocol.h"

#ifndef ARDUINO
// cppcheck-suppress unusedFunction
int main(int argc, char **argv) {
    setup();
    // cppcheck-suppress knownConditionTrueFalse
    while (Serial.available()) {
        loop();
        break; // just run once for compilation check
    }
    return 0;
}
uint32_t millis() { return 0; }
void delay(unsigned long ms) {}
#endif
#include "../lib/safety/SafetyMonitor.h"
#include "../lib/motion/MotionController.h"

#define SERIAL_BAUD    115200
#define LINE_BUF_SIZE  128
#define HEARTBEAT_MS   1000UL   // emit heartbeat every 1 s

static char     g_line_buf[LINE_BUF_SIZE];
static uint8_t  g_line_len   = 0;
static uint32_t g_last_hb_ms = 0;

// cppcheck-suppress unusedFunction
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { /* wait for USB-serial */ }
    g_line_buf[0] = '\0';
    g_line_len    = 0;
    g_last_hb_ms  = millis();

    // TODO: initialise motor drivers, limit-switch pins here
    protocol_emit_event("EVT:BOOT:fw=arm_controller:v=0.1.0");
}

extern SafetyMonitor g_safety_monitor; // Defined in protocol.cpp
extern MotionController g_motion_controller;

// cppcheck-suppress unusedFunction
void loop() {
    static MotionState g_last_motion_state = MotionState::IDLE;
    g_safety_monitor.poll();

    // ── Read Serial ──────────────────────────────────────────────────────────
    // cppcheck-suppress knownConditionTrueFalse
    while (Serial.available()) {
        char c = (char)Serial.read();
        // cppcheck-suppress knownConditionTrueFalse
        if (c == '\n' || c == '\r') {
            if (g_line_len > 0) {
                g_line_buf[g_line_len] = '\0';
                protocol_handle_line(g_line_buf);
                g_line_len = 0;
            }
        } else if (g_line_len < (LINE_BUF_SIZE - 1)) {
            g_line_buf[g_line_len++] = c;
        }
        // Silently drop bytes when buffer is full
    }

    // ── Heartbeat ────────────────────────────────────────────────────────────
    uint32_t now = millis();
    if ((now - g_last_hb_ms) >= HEARTBEAT_MS) {
        protocol_emit_event(EVT_HEARTBEAT ":status=OK");
        g_last_hb_ms = now;
    }

    g_motion_controller.update();
    MotionState current_state = g_motion_controller.getState();
    if (g_last_motion_state == MotionState::MOVING) {
        if (current_state == MotionState::IDLE) {
            protocol_emit_event("EVT:ARM_DONE");
        } else if (current_state == MotionState::FAULT) {
            protocol_emit_event("EVT:ARM_FAULT:code=UNKNOWN");
        }
    }
    g_last_motion_state = current_state;
}
