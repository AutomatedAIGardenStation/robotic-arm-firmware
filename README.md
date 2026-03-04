# arm_controller

Firmware for the **robotic arm** subsystem of Garden Station.

Handles: joint motion (servos / steppers), gripper, limit switches, encoder feedback.

> **Architecture**: The backend's Serial Manager is the only process that sends commands to this controller. The firmware never communicates with other software modules directly.
>
> Full protocol spec: `project-info/Docs/06_Software/arm_controller/Serial Protocol & Interfaces.md`

---

## Quick Start

```bash
# Build (requires PlatformIO CLI)
pio run -e mega2560

# Flash to target
pio run -e mega2560 -t upload

# Monitor serial output
pio device monitor -b 115200
```

Available environments: `mega2560`, `nucleo_f446re`, `esp32dev`

---

## Project Structure

```
arm_controller/
├── platformio.ini      # PlatformIO environments for supported boards
├── config/
│   └── pins.h          # Board-specific pin assignments – edit for your wiring
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

**Key commands:** `ARM_HOME`, `ARM_MOVE_TO:j1=<deg>:...`, `GRIPPER_OPEN`, `GRIPPER_CLOSE`, `H1` (harvest), `P1` (pollinate), `M1` (move zone)

**Key events:** `EVT:ARM_DONE`, `EVT:ARM_FAULT:code=<reason>`, `EVT:HEARTBEAT:status=OK`

---

## Development Notes

- Implement HAL stubs in `src/protocol.cpp` for your motor driver library (e.g. AccelStepper, Servo).
- Update `config/pins.h` to match your physical wiring.
- Motion-complete events (`EVT:ARM_DONE`) **must** be emitted after every command so the backend's state machine can advance.
- The watchdog / heartbeat is emitted every 5 s. The backend treats a missed heartbeat as a controller fault.

See also: [07_Development/arm_controller](../project-info/Docs/07_Development/arm_controller/README.md)
