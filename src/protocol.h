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
#define CMD_ARM_MOVE        "ARM_MOVE_TO"     // Move to joint coords: ARM_MOVE_TO:j1=...:j2=...
#define CMD_GRIPPER_OPEN    "GRIPPER_OPEN"    // Open gripper
#define CMD_GRIPPER_CLOSE   "GRIPPER_CLOSE"   // Close gripper
#define CMD_POLLINATE       "P1"              // Initiate pollination sequence
#define CMD_HARVEST         "H1"              // Initiate harvest sequence
#define CMD_MOVE_ZONE       "M1"              // Move arm to a named zone
#define CMD_NOP             "NOP"             // No operation

// ── Outbound events  (arm_controller → backend) ──────────────────────────────
// Emitted as: EVT:<NAME>[:<key>=<value>]*\n
#define EVT_ARM_DONE        "EVT:ARM_DONE"
#define EVT_ARM_FAULT       "EVT:ARM_FAULT"   // :code=<reason>
#define EVT_HEARTBEAT       "EVT:HEARTBEAT"   // :status=OK

// Parse one line from Serial and execute the corresponding action.
// Returns true if a known command was handled.
bool protocol_handle_line(const char* line);

// Emit an event string back to the backend over Serial.
void protocol_emit_event(const char* event);

#endif  // PROTOCOL_H
