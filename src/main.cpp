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
#include <Arduino.h>
#include "protocol.h"
#include "../lib/safety/SafetyMonitor.h"

#define SERIAL_BAUD    115200
#define LINE_BUF_SIZE  128
#define HEARTBEAT_MS   5000UL   // emit heartbeat every 5 s

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

// cppcheck-suppress unusedFunction
void loop() {
    g_safety_monitor.poll();

    // ── Read Serial ──────────────────────────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
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

    // TODO: poll motion-complete flags and emit EVT:ARM_DONE / EVT:ARM_FAULT
}
