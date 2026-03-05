# arm_controller

Firmware for the **robotic arm** subsystem of Garden Station.

Handles: Cartesian gantry motion (X, Y, Z), 2-DOF wrist (pitch, roll), magnetic/twist-lock tool head, limit switches.

> **Architecture**: The backend's Serial Manager is the only process that sends commands to this controller. The firmware never communicates with other software modules directly.
>
> Full protocol spec: `project-info/Docs/06_Software/arm_controller/Serial Protocol & Interfaces.md`

### Module Contract

| | |
|---|---|
| **Input** | Pure Cartesian coordinates (X, Y, Z in mm), wrist orientation (pitch, roll in degrees), tool commands (dock/release) |
| **Output** | `EVT:ARM_DONE`, `EVT:ARM_FAULT`, `EVT:ARM_HOMED`, `EVT:WRIST_DONE`, `EVT:TOOL_DOCKED`, `EVT:TOOL_RELEASED` |
| **Constraint** | No inverse kinematics, no zone logic, no sequence orchestration — firmware executes raw Cartesian step counts |

---

## Quick Start

```bash
# Build (requires PlatformIO CLI)
pio run -e mega2560

# Flash to target
pio run -e mega2560 -t upload

# Monitor serial output
pio device monitor -b 115200

# Run tests
pio test -e native
```

Available environments: `mega2560`, `nucleo_f446re`, `esp32dev`

---

## Project Structure

```
arm_controller/
├── platformio.ini      # PlatformIO environments for supported boards
├── config/
│   └── pins.h          # Board-specific pin assignments – edit for your wiring
├── lib/
│   ├── hal/            # Hardware abstraction layer interfaces
│   ├── motion/         # Motion controller and logic
│   └── safety/         # Safety monitor and limit switches
└── src/
    ├── main.cpp        # Setup / loop, Serial reader, heartbeat
    ├── protocol.h      # Command & event constants, public API
    └── protocol.cpp    # Command dispatcher – stub HAL calls to replace
```

---

## Serial Protocol (summary)

| Direction | Format |
|-----------|--------|
| Backend → Controller | `<COMMAND>[:<key>=<val>]*\n` |
| Controller → Backend | `EVT:<NAME>[:<key>=<val>]*\n` |

**Commands:**
| Command | Description |
| --- | --- |
| `ARM_MOVE_TO:x=<mm>:y=<mm>:z=<mm>` | Move to Cartesian coordinates |
| `ARM_HOME` | Full homing sequence (Z → X → Y) |
| `ARM_CLEAR_FAULT` | Clear soft-tier fault, trigger re-homing |
| `WRIST_SET:pitch=<deg>:roll=<deg>` | Set 2-DOF wrist (-90 to 90 degrees) |
| `GRIPPER_OPEN` | Open gripper / tool actuator |
| `GRIPPER_CLOSE` | Close gripper / tool actuator |
| `TOOL_DOCK` | Dock current tool at tool station |
| `TOOL_RELEASE:tool=<name>` | Pick up a named tool from dock (CAMERA, GRIPPER, POLLINATOR, SCISSORS) |

**Events:**
| Event | Description |
| --- | --- |
| `EVT:ARM_BOOT` | MCU powered on / reset — position unknown, homing begins |
| `EVT:ARM_HOMING:axis=<X\|Y\|Z>` | Homing in progress for specified axis |
| `EVT:ARM_HOMED` | All axes homed — arm at (0,0,0), motion commands accepted |
| `EVT:ARM_DONE` | Cartesian gantry motion complete |
| `EVT:WRIST_DONE` | Wrist servo motion complete |
| `EVT:ARM_FAULT:code=<reason>:tier=<soft\|hard>` | Fault condition (LIMIT_HIT, OVERCURRENT, E_STOP, HOME_*_TIMEOUT, REPEATED_FAULT) |
| `EVT:ARM_FAULT_CLEARED` | Soft fault cleared — firmware re-homing |
| `EVT:TOOL_DOCKED:tool=<name>` | Tool successfully docked at station |
| `EVT:TOOL_RELEASED:tool=<name>` | Tool picked up and secured |
| `EVT:TOOL_FAULT:reason=<code>` | Tool change failed (ALIGNMENT_ERROR, LOCK_FAIL, SLOT_EMPTY) |
| `EVT:HEARTBEAT:status=OK` | Periodic heartbeat |

---

## Development Notes

- Implement HAL stubs in `src/protocol.cpp` for your stepper drivers (XYZ linear rails) and servo drivers (wrist pitch/roll).
- Update `config/pins.h` to match your physical wiring.
- No inverse kinematics — firmware executes raw Cartesian step counts. Coordinate mapping is 1:1 (mm to steps via known lead screw pitch).
- Motion-complete events (`EVT:ARM_DONE`) **must** be emitted after every command so the backend's state machine can advance.
- The watchdog / heartbeat is emitted every 1 s. The backend treats a missed heartbeat as a controller fault.

See also: [07_Development/arm_controller](../project-info/Docs/07_Development/arm_controller/README.md)
