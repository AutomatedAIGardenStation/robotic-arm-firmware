/**
 * arm_controller – command protocol.
 * Implements the ASCII line protocol defined in:
 *   project-info/Docs/06_Software/arm_controller/Serial Protocol & Interfaces.md
 *
 * All commands are issued by the backend's Serial Manager.
 * Responses are unsolicited event lines emitted by this firmware.
 *
 * Baud: 115200  Frame: 1 start, 8 data, 1 stop, no parity.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

// ── Inbound commands (backend → arm_controller) ─────────────────────────────
#define CMD_ARM_HOME        "ARM_HOME"        // Return arm to home position
#define CMD_ARM_MOVE        "ARM_MOVE_TO"     // Move to Cartesian coords: ARM_MOVE_TO:x=...:y=...:z=...
#define CMD_ARM_CLEAR_FAULT "ARM_CLEAR_FAULT" // Clear soft-tier fault
#define CMD_WRIST_SET       "WRIST_SET"       // Set 2-DOF wrist: WRIST_SET:pitch=...:roll=...
#define CMD_GRIPPER_OPEN    "GRIPPER_OPEN"    // Open gripper
#define CMD_GRIPPER_CLOSE   "GRIPPER_CLOSE"   // Close gripper
#define CMD_TOOL_DOCK       "TOOL_DOCK"       // Dock current tool at tool station
#define CMD_TOOL_RELEASE    "TOOL_RELEASE"    // Pick up a named tool: TOOL_RELEASE:tool=<name>
#define CMD_PING            "PING"            // Watchdog ping
#define CMD_NOP             "NOP"             // No operation

// ── Outbound events  (arm_controller → backend) ──────────────────────────────
// Emitted as: EVT:<NAME>[:<key>=<value>]*\n
#define EVT_ARM_HOMED       "EVT:ARM_HOMED"
#define EVT_ARM_DONE        "EVT:ARM_DONE"
#define EVT_WRIST_DONE      "EVT:WRIST_DONE"
#define EVT_ARM_FAULT       "EVT:ARM_FAULT"   // :code=<reason>
#define EVT_TOOL_DOCKED     "EVT:TOOL_DOCKED"
#define EVT_TOOL_RELEASED   "EVT:TOOL_RELEASED"
#define EVT_TOOL_FAULT      "EVT:TOOL_FAULT"
#define EVT_HEARTBEAT       "EVT:HEARTBEAT"   // :status=OK

// Called when PING is received to reset watchdog
void protocol_ping_received();

// Parse one line from Serial and execute the corresponding action.
// Returns true if a known command was handled.
bool protocol_handle_line(const char* line);

// Emit an event string back to the backend over Serial.
void protocol_emit_event(const char* event);

#endif  // PROTOCOL_H
