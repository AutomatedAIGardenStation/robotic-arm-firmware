1. **Create `lib/motion/MockMotorDriver.h`**
   - Implement `IMotorDriver` interface
   - Add tracking for calls to `enable`, `disable`, `step`, and `setSpeed` to allow test verification.
2. **Create `lib/motion/MotionController.h` and `lib/motion/MotionController.cpp`**
   - Define `State` enum (`IDLE`, `MOVING`, `FAULT`)
   - Define `Command` and `CommandType` struct/enum mapping to ARM_MOVE_TO, ARM_HOME, GRIPPER_OPEN, GRIPPER_CLOSE, H1 (HARVEST), P1 (POLLINATE), M1 (MOVE_ZONE)
   - Add `execute(Command cmd)` and `getState()`
   - Add `update()` to process moving state and buffered commands.
   - Buffer logic: single-slot buffer for commands received while in `MOVING` state.
   - Use `CoordinateMapper::is_in_range` for validating commands, rejecting and emitting `EVT:ARM_FAULT:code=OUT_OF_RANGE`.
   - On completion of move, emit `EVT:ARM_DONE` and return to `IDLE` state.
3. **Update `src/protocol.cpp`**
   - Include `MotionController.h`.
   - Instantiate a global `MotionController` (using MockMotorDrivers for now or injecting them)
   - Replace empty function stubs (`arm_go_home`, `arm_move_to`, `gripper_open`, `gripper_close`, `arm_harvest_sequence`, `arm_pollinate_sequence`, `arm_move_zone`) with calls to `MotionController::execute()`.
   - Make sure no TODOs remain for those functions.
4. **Create `test/test_motion_controller/test_motion_controller.cpp`**
   - Setup Unity tests.
   - Add tests for:
     - Starts in IDLE state.
     - ARM_HOME transitions to MOVING, completes, returns to IDLE, EVT:ARM_DONE emitted.
     - ARM_MOVE_TO with valid angles: transitions to MOVING and back.
     - ARM_MOVE_TO with out-of-range angle: stays IDLE, EVT:ARM_FAULT emitted.
     - GRIPPER_OPEN / GRIPPER_CLOSE complete correctly from IDLE.
5. **Run tests & static analysis**
   - Run `pio test -e native` to ensure all tests pass.
   - Run `pio run` on `mega2560`, `nucleo_f446re`, `esp32dev` to check compilation.
   - Run `cppcheck` to ensure no errors.
6. **Pre-commit checks**
   - Ensure proper testing, verifications, reviews and reflections are done.
7. **Submit the change**
   - Commit to branch `agent/robotic-arm-firmware/issue-6`.
   - Ensure commit message follows PR requirements.
